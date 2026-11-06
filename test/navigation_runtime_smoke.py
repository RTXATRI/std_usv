#!/usr/bin/env python3
"""Bounded, isolated std_usv ROS/Gazebo runtime smoke test.

This manual acceptance harness owns only the roslaunch process tree it creates.
It never starts an X server, stops unrelated processes, or changes source
files.  A default short run validates infrastructure and observed closed-loop
motion; full navigation is reported successful only after goal_reached and
independent ground-truth/zero-command checks.  Run from a sourced workspace:

  python3 test/navigation_runtime_smoke.py
  python3 test/navigation_runtime_smoke.py --until-goal --sim-seconds 600 --max-wall 1800

Planner-stop acceptance permits 0.5 s input timeout plus one 20 Hz output
period: at most 0.55 simulation seconds to receive all four zero commands.
Full navigation requires a valid map-frame /wamv/navigation/goal_request and a final independent
truth speed <=0.1 m/s and yaw rate <=0.1 rad/s. The production planner owns the
continuous two-second low-speed arrival check.
With --dynamic-obstacles, supported buoy models must actually move along their
routes. Motion is sampled every 0.1 simulation seconds; cumulative XY speed
allows 25 percent variation and up to two seconds of initial service readiness.

ROS/Gazebo are not started until main() is invoked.  Configurable private ports
are checked for collisions before launch.  This script is deliberately excluded
from default unit tests and package installation.
"""

import argparse
import collections
import http.client
import importlib.util
import json
import math
import os
import pathlib
import queue
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import xml.etree.ElementTree as ET
import xmlrpc.client


ROS_PORT = 11481
GAZEBO_PORT = 11482
CALLER = "/std_usv_navigation_runtime_smoke"
THRUSTERS = ("left_front", "right_front", "left_rear", "right_rear")
MAX_STOP_DELAY = .5 + 1. / 20.
DEFAULT_USV_POSE = dict(usv_x=-570., usv_y=167., usv_z=.1,
                        usv_roll=0., usv_pitch=0., usv_yaw=1.)
TOPICS = {
    "clock": "/clock",
    "odometry": "/wamv/robot_localization/odometry/filtered",
    "gps_odometry": "/wamv/robot_localization/odometry/gps",
    "scan": "/scan",
    "gps": "/wamv/sensors/gps/gps/fix",
    "imu": "/wamv/sensors/imu/imu/data",
    "raw_lidar": "/wamv/sensors/lidars/lidar_wamv/points",
    "status": "/wamv/navigation/status",
    "command": "/wamv/cmd_vel",
    "goal": "/wamv/navigation/goal_request",
}
CAMERAS = ('front_left_camera', 'front_right_camera', 'middle_right_camera')


def json_payload(value):
    def safe(item):
        if isinstance(item, float) and not math.isfinite(item):
            return str(item)
        if isinstance(item, dict):
            return {key: safe(part) for key, part in item.items()}
        if isinstance(item, (list, tuple)):
            return [safe(part) for part in item]
        return item
    return json.dumps(safe(value), indent=2, sort_keys=True, allow_nan=False)


def atomic_json(path, value):
    path = pathlib.Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json_payload(value) + '\n')
    temporary.replace(path)


def valid_extra_sensor(key, message):
    """Check message payloads without copying full-resolution camera buffers."""
    if key.startswith('camera_') and key.endswith('_image'):
        return (message.width > 0 and message.height > 0 and message.step > 0 and
                bool(message.encoding) and len(message.data) == message.height * message.step)
    if key.startswith('camera_') and key.endswith('_info'):
        return (message.width > 0 and message.height > 0 and
                all(math.isfinite(value) for value in list(message.K) + list(message.P)) and
                message.K[0] > 0 and message.K[4] > 0 and message.P[0] > 0 and message.P[5] > 0)
    if key == 'gps_velocity':
        return all(math.isfinite(value) for value in (message.vector.x, message.vector.y, message.vector.z))
    if key == 'pinger':
        return (all(math.isfinite(value) for value in (message.range, message.bearing, message.elevation)) and
                message.range >= 0)
    return None


def convex_polygons_intersect(first, second):
    """Separating-axis test for convex polygons; touching counts as contact."""
    for polygon in (first, second):
        for index, start in enumerate(polygon):
            end = polygon[(index + 1) % len(polygon)]
            axis = (start[1] - end[1], end[0] - start[0])
            if axis[0] * axis[0] + axis[1] * axis[1] < 1e-20:
                continue
            a = [point[0] * axis[0] + point[1] * axis[1] for point in first]
            b = [point[0] * axis[0] + point[1] * axis[1] for point in second]
            if max(a) < min(b) - 1e-10 or max(b) < min(a) - 1e-10:
                return False
    return True


def pose_rotation_matrix(pose, np):
    q = pose.orientation
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if norm < 1e-12 or not math.isfinite(norm):
        raise SmokeFailure('Invalid quaternion in collision oracle')
    x, y, z, w = q.x / math.sqrt(norm), q.y / math.sqrt(norm), q.z / math.sqrt(norm), q.w / math.sqrt(norm)
    return np.asarray([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                       [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                       [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


class CollisionOracle:
    """Conservative 2D acceptance oracle; never publishes planner inputs.

    Known geometry is conservatively rasterized at 0.5 m. Unknown models use
    their local collision AABB projected at the current full 3D model pose.
    Contact of these projected regions is stricter than a Gazebo 3D contact.
    The physical hull has no planner clearance margin added.
    """
    def __init__(self, np, geometry, occupied, origin, resolution, footprint, unknowns):
        self.np, self.geometry = np, geometry
        self.occupied = np.asarray(occupied, dtype=bool)
        self.mask = np.zeros(self.occupied.shape, dtype=bool)
        self.origin, self.resolution = tuple(origin[:2]), float(resolution)
        self.footprint = np.column_stack((np.asarray(footprint, dtype=float), np.zeros(len(footprint))))
        self.unknowns = unknowns
        self.interval = .1
        self.last_clock = None
        self.samples = 0
        self.min_interval = None
        self.max_interval = None
        self.first_collision = None
        self.missing_models = set()
        self.errors = []

    @classmethod
    def from_world(cls, world_path):
        import numpy as np
        import rospkg
        import yaml
        package = pathlib.Path(rospkg.RosPack().get_path('std_usv'))
        def load_source(name, filename):
            spec = importlib.util.spec_from_file_location(name, str(package / 'scripts' / filename))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
        geometry = load_source('_smoke_collision_geometry', 'generate_navigation_maps.py')
        maps = load_source('_smoke_collision_maps', 'navigation_map_publisher.py')
        resolution, origin, values = maps.read_map(maps.map_for_world(world_path, package))
        if abs(origin[2]) > 1e-12:
            raise SmokeFailure('Collision oracle requires an axis-aligned map origin')
        footprint = yaml.safe_load((package / 'config/wamv_model.yaml').read_text())['navigation']['model']['footprint']
        sources = geometry.Sources()
        world = sources.xml(world_path).find('world')
        unknowns = {}
        for include in world.findall('include'):
            name = include.findtext('name', '')
            if not name.startswith(geometry.UNKNOWN):
                continue
            model = sources.model(include.findtext('uri'))
            shapes = [data.reshape(-1, 3) for data, _ in geometry.collision_shapes(model, sources)]
            if not shapes:
                raise SmokeFailure('Unknown model has no collision shapes: ' + name)
            vertices = np.concatenate(shapes)
            low, high = vertices.min(axis=0), vertices.max(axis=0)
            unknowns[name] = np.asarray([[x, y, z] for x in (low[0], high[0])
                                        for y in (low[1], high[1]) for z in (low[2], high[2])])
        return cls(np, geometry, values != 0, origin, resolution, footprint, unknowns)

    def projection(self, local_points, pose):
        position = self.np.asarray([pose.position.x, pose.position.y, pose.position.z])
        vertices = local_points @ pose_rotation_matrix(pose, self.np).T + position
        return self.geometry.convex_hull(vertices[:, :2])

    def check(self, clock, states):
        if clock is None or not math.isfinite(clock):
            return
        if self.last_clock is not None and clock - self.last_clock < self.interval - 1e-8:
            return
        poses = dict(zip(states.name, states.pose))
        if 'wamv' not in poses:
            return  # Gazebo publishes while the launch spawn node is still waiting.
        missing = set(self.unknowns).difference(poses)
        if missing:
            self.missing_models.update(missing)
            return
        if self.last_clock is not None:
            interval = clock - self.last_clock
            self.min_interval = interval if self.min_interval is None else min(self.min_interval, interval)
            self.max_interval = interval if self.max_interval is None else max(self.max_interval, interval)
        self.last_clock = clock
        self.samples += 1
        boat = self.projection(self.footprint, poses['wamv'])
        self.mask.fill(False)
        self.geometry.raster_polygon(self.mask, boat, self.origin, self.resolution)
        cells = self.np.argwhere(self.mask & self.occupied)
        collision = None
        if len(cells):
            iy, ix = map(int, cells[0])
            collision = dict(kind='known_occupancy_grid', grid_cell=[ix, iy],
                             cell_center=[self.origin[0] + (ix+.5)*self.resolution,
                                          self.origin[1] + (iy+.5)*self.resolution])
        else:
            for name, local_box in self.unknowns.items():
                obstacle = self.projection(local_box, poses[name])
                if len(obstacle) >= 3 and convex_polygons_intersect(boat, obstacle):
                    collision = dict(kind='unknown_collision_bounds', model=name)
                    break
        if collision is not None and self.first_collision is None:
            self.first_collision = dict(clock=clock, boat_pose=pose_dict(poses['wamv']), **collision)

    def export(self):
        return dict(enabled=True, requested_interval_sim_seconds=self.interval,
                    samples=self.samples, min_interval_sim_seconds=self.min_interval,
                    max_interval_sim_seconds=self.max_interval, first_collision=self.first_collision,
                    missing_models=sorted(self.missing_models), errors=list(self.errors),
                    unknown_model_names=sorted(self.unknowns),
                    method='physical hull XY projection vs conservative PGM / current projected collision AABBs')


def collision_self_test():
    """Pure geometry regression: this never imports rospy or starts ROS."""
    import numpy as np
    from types import SimpleNamespace as Message
    source = pathlib.Path(__file__).resolve().parents[1] / 'scripts/generate_navigation_maps.py'
    spec = importlib.util.spec_from_file_location('_oracle_selftest_geometry', str(source))
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    footprint = [[-.3, -.3], [.3, -.3], [.3, .3], [-.3, .3]]
    def pose(x, y, angle=0.):
        return Message(position=Message(x=x, y=y, z=0.),
                       orientation=Message(x=0., y=0., z=math.sin(angle/2.), w=math.cos(angle/2.)))
    def states(boat, obstacle=None):
        return Message(name=['wamv'] + (['dynamic_obstacle_test'] if obstacle is not None else []),
                       pose=[boat] + ([obstacle] if obstacle is not None else []))
    free = np.zeros((10, 10), dtype=bool)
    occupied = free.copy()
    occupied[3, 3] = True
    oracle = CollisionOracle(np, geometry, occupied, (0, 0), 1., footprint, {})
    oracle.check(0., states(pose(.5, .5)))
    assert oracle.first_collision is None
    oracle.check(.1, states(pose(3.5, 3.5)))
    assert oracle.first_collision['kind'] == 'known_occupancy_grid'
    assert oracle.first_collision['grid_cell'] == [3, 3]
    oracle.check(.15, states(pose(3.5, 3.5)))
    assert oracle.samples == 2, 'Oracle must not run on each physics step'
    assert abs(oracle.min_interval - .1) < 1e-12
    box = np.asarray([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.2, .2)])
    dynamic = CollisionOracle(np, geometry, free, (0, 0), 1., footprint,
                              {'dynamic_obstacle_test': box})
    dynamic.check(0., states(pose(3.9, 0.), pose(8., 0.)))
    assert dynamic.first_collision is None
    dynamic.check(.1, states(pose(3.9, 0.), pose(3., 0., math.pi/4)))
    assert dynamic.first_collision['model'] == 'dynamic_obstacle_test'
    assert convex_polygons_intersect(footprint, [[.3, -.3], [.9, -.3], [.9, .3], [.3, .3]])
    assert not convex_polygons_intersect(footprint, [[.31, -.3], [.9, -.3], [.9, .3], [.31, .3]])
    negative = CollisionOracle(np, geometry, occupied, (-5, -5), 1., footprint, {})
    negative.check(0., states(pose(-1.5, -1.5)))
    assert negative.first_collision['grid_cell'] == [3, 3]
    print(json.dumps(dict(collision_self_test='passed', checks=['known_grid', 'negative_coordinates',
        'touching_contact', 'SAT_separation', 'dynamic_rotated_bounds', 'physics_step_throttle'])))


class SmokeFailure(RuntimeError):
    pass


class Interrupted(SmokeFailure):
    pass


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout=.5):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        if self._connection and self._connection[0] == host:
            return self._connection[1]
        actual_host, self._extra_headers, _ = self.get_host_info(host)
        connection = http.client.HTTPConnection(actual_host, timeout=self.timeout)
        self._connection = (host, connection)
        return connection


def bounded_call(function, timeout, label):
    """ROS ServiceProxy has no call timeout; its worker must never block main."""
    result = queue.Queue(maxsize=1)

    def run():
        try:
            result.put((True, function()))
        except BaseException as error:
            result.put((False, error))

    worker = threading.Thread(target=run, name="bounded_" + label, daemon=True)
    worker.start()
    try:
        success, value = result.get(timeout=max(.001, timeout))
    except queue.Empty:
        raise SmokeFailure("Timed out calling " + label)
    if not success:
        raise value
    return value


def check_ports_free(ros_port, gazebo_port):
    sockets = []
    try:
        for port in (ros_port, gazebo_port):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sockets.append(sock)
            try:
                sock.bind(("0.0.0.0", port))
            except OSError as error:
                raise SmokeFailure("Refusing to launch: port %d is busy (%s)" % (port, error))
    finally:
        for sock in sockets:
            sock.close()


def process_identity(pid):
    try:
        raw = pathlib.Path("/proc/%d/stat" % pid).read_text()
        fields = raw[raw.rfind(")") + 2:].split()
        # fields start at proc(5) field 3: state, ppid, pgrp, session...
        return dict(pid=pid, ppid=int(fields[1]), pgrp=int(fields[2]),
                    session=int(fields[3]), start=int(fields[19]), state=fields[0])
    except (OSError, ValueError, IndexError):
        return None


def process_table():
    table = {}
    for path in pathlib.Path("/proc").iterdir():
        if path.name.isdigit():
            entry = process_identity(int(path.name))
            if entry is not None:
                table[entry["pid"]] = entry
    return table


class OwnedLaunch:
    """Track birth identities so cleanup can only signal our own descendants."""
    def __init__(self, command, environment, log_path):
        self.log_stream = open(log_path, "wb", buffering=0)
        try:
            self.process = subprocess.Popen(command, env=environment, stdin=subprocess.DEVNULL,
                                            stdout=self.log_stream, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        except BaseException:
            self.log_stream.close()
            raise
        self.lock = threading.Lock()
        self.known = {}
        root = process_identity(self.process.pid)
        self.root_start = root['start'] if root is not None else None
        self.finished = threading.Event()
        self.capture()
        self.monitor = threading.Thread(target=self.track, name="owned_process_tracker", daemon=True)
        self.monitor.start()

    def capture(self):
        table = process_table()
        with self.lock:
            previous = dict(self.known)
        # Birth identities prevent a reused PID from becoming an owned root.
        owned = {pid for pid, entry in previous.items()
                 if pid in table and table[pid]['start'] == entry['start']}
        root = table.get(self.process.pid)
        if root is not None and root['start'] == self.root_start:
            owned.add(self.process.pid)
        changed = True
        while changed:
            changed = False
            for pid, entry in table.items():
                if pid not in owned and entry["ppid"] in owned:
                    owned.add(pid)
                    changed = True
        with self.lock:
            for pid in owned:
                if pid in table:
                    self.known[pid] = table[pid]

    def track(self):
        while not self.finished.wait(.1):
            self.capture()

    def owns(self, pid):
        self.capture()
        current = process_identity(pid)
        with self.lock:
            identity = self.known.get(pid)
        return bool(current and identity and current["start"] == identity["start"])

    def live(self):
        with self.lock:
            identities = list(self.known.values())
        alive = []
        for entry in identities:
            current = process_identity(entry["pid"])
            if current and current["start"] == entry["start"] and current["state"] != "Z":
                alive.append(current)
        return alive

    def signal_owned(self, sig):
        self.capture()
        # The roslaunch leader owns this process group/session.  Its own
        # graceful shutdown also handles child nodes with private sessions.
        if self.owns(self.process.pid):
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                pass
        # Nodes created by roslaunch can call setsid(); signal those exact,
        # previously observed descendants too, checking PID birth identity.
        for entry in self.live():
            if entry["pgrp"] != self.process.pid:
                try:
                    os.kill(entry["pid"], sig)
                except ProcessLookupError:
                    pass

    def cleanup(self):
        self.signal_owned(signal.SIGINT)
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            self.capture()
            if not self.live():
                break
            time.sleep(.1)
        remaining = self.live()
        post_kill_wait = 0.
        if remaining:
            self.signal_owned(signal.SIGKILL)
            # Delivery of SIGKILL is asynchronous, particularly for a Gazebo
            # process tearing down rendering threads. Poll only saved birth
            # identities and permit two wall seconds before reporting residue.
            started = time.monotonic()
            self.wait_for_owned_exit(2.)
            post_kill_wait = time.monotonic() - started
        try:
            self.process.wait(timeout=1.)
        except subprocess.TimeoutExpired:
            pass
        self.finished.set()
        self.monitor.join(timeout=.5)
        self.log_stream.close()
        return dict(launch_returncode=self.process.poll(),
                    killed_pids=[entry["pid"] for entry in remaining],
                    post_kill_wait_wall_seconds=post_kill_wait,
                    remaining_owned_pids=[entry["pid"] for entry in self.live()])

    def wait_for_owned_exit(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            self.process.poll()  # Reap the direct child once it has exited.
            remaining = self.live()  # Already excludes zombies and reused PIDs.
            if not remaining or time.monotonic() >= deadline:
                return remaining
            time.sleep(min(.02, max(0., deadline-time.monotonic())))


def cleanup_wait_self_test():
    """Exercise asynchronous reap/birth filtering without signalling processes."""
    from types import SimpleNamespace as Message
    from unittest.mock import patch
    clock = [0.]
    looked_up = []
    subject = object.__new__(OwnedLaunch)
    subject.lock = threading.Lock()
    subject.known = {101: dict(pid=101, start=7), 102: dict(pid=102, start=8)}
    subject.process = Message(poll=lambda: None)
    def identity(pid):
        looked_up.append(pid)
        if pid == 101 and clock[0] < .06:
            return dict(pid=101, start=7, state='S')
        if pid == 102:
            return dict(pid=102, start=8, state='Z')
        return None
    def advance(duration):
        clock[0] += duration
    with patch(__name__ + '.process_identity', identity), patch.object(time, 'monotonic', lambda: clock[0]), \
            patch.object(time, 'sleep', advance):
        assert subject.wait_for_owned_exit(2.) == []
        assert .06 <= clock[0] < .1
        assert set(looked_up) == {101, 102}, 'Never wait for unrelated PIDs'
    # An unreaped zombie or a reused PID is already gone for ownership purposes.
    with patch(__name__ + '.process_identity', lambda pid: dict(pid=pid, start=99, state='S')):
        assert subject.live() == []
    clock[0] = 0.
    with patch(__name__ + '.process_identity', lambda pid: dict(pid=pid, start=7 if pid == 101 else 8, state='S')), \
            patch.object(time, 'monotonic', lambda: clock[0]), patch.object(time, 'sleep', advance):
        remaining = subject.wait_for_owned_exit(.1)
        assert {entry['pid'] for entry in remaining} == {101, 102}, 'True survivors must still be reported'
        assert .1 <= clock[0] < .121
    print(json.dumps(dict(cleanup_wait_self_test='passed', checks=['asynchronous_exit_wait',
        'only_saved_pids', 'zombie_ignored', 'birth_reuse_ignored', 'true_survivors_reported', 'bounded_wait'])))


def yaw(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def pose_dict(pose):
    return dict(x=pose.position.x, y=pose.position.y, z=pose.position.z,
                yaw=yaw(pose.orientation))


def finite_pose(pose):
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return all(math.isfinite(value) for value in values) and sum(value * value for value in values[3:]) > .5


def orientation_error_to_rpy(quaternion, roll, pitch, yaw_angle):
    """Full attitude error, invariant to quaternion sign and Euler wrapping."""
    cr, sr = math.cos(roll/2.), math.sin(roll/2.)
    cp, sp = math.cos(pitch/2.), math.sin(pitch/2.)
    cy, sy = math.cos(yaw_angle/2.), math.sin(yaw_angle/2.)
    expected = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    actual = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = math.sqrt(sum(value*value for value in actual))
    if not math.isfinite(norm) or norm < 1e-12:
        return math.inf
    dot = sum(first*second for first, second in zip(expected, actual)) / norm
    return 2.*math.acos(min(1., abs(dot)))


def ecef(latitude, longitude, altitude):
    a, flattening = 6378137., 1. / 298.257223563
    eccentricity2 = flattening * (2. - flattening)
    latitude, longitude = math.radians(latitude), math.radians(longitude)
    normal = a / math.sqrt(1. - eccentricity2 * math.sin(latitude) ** 2)
    return ((normal + altitude) * math.cos(latitude) * math.cos(longitude),
            (normal + altitude) * math.cos(latitude) * math.sin(longitude),
            (normal * (1. - eccentricity2) + altitude) * math.sin(latitude))


def gps_enu(message, reference):
    reference_xyz = ecef(reference["latitude"], reference["longitude"], reference["elevation"])
    measured_xyz = ecef(message.latitude, message.longitude, message.altitude)
    dx, dy, dz = (a - b for a, b in zip(measured_xyz, reference_xyz))
    latitude, longitude = math.radians(reference["latitude"]), math.radians(reference["longitude"])
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (-math.sin(latitude) * math.cos(longitude) * dx -
             math.sin(latitude) * math.sin(longitude) * dy + math.cos(latitude) * dz)
    up = (math.cos(latitude) * math.cos(longitude) * dx +
          math.cos(latitude) * math.sin(longitude) * dy + math.sin(latitude) * dz)
    heading = math.radians(reference["heading"])
    return (math.cos(heading) * east + math.sin(heading) * north,
            -math.sin(heading) * east + math.cos(heading) * north, up)


class StopObservation:
    """Latch the callback time of the first complete post-boundary zero set."""
    def __init__(self, channels):
        self.channels = tuple(channels)
        self.boundary = None
        self.arrivals = {}
        self.zero_clock = None

    def arm(self, clock):
        if clock is None or not math.isfinite(clock):
            raise SmokeFailure('A valid simulation clock is required before stopping the planner')
        self.boundary = clock
        self.arrivals = {}  # Discard every command observed before this boundary.
        self.zero_clock = None

    def record(self, channel, value, clock):
        if self.boundary is None or clock is None or not math.isfinite(clock) or clock < self.boundary:
            return
        if channel not in self.channels:
            return
        self.arrivals[channel] = (clock, value)
        if self.zero_clock is None and len(self.arrivals) == len(self.channels):
            if all(math.isfinite(value) and abs(value) <= 1e-6 for _, value in self.arrivals.values()):
                self.zero_clock = max(clock for clock, _ in self.arrivals.values())

    def export(self):
        return dict(boundary_clock=self.boundary, first_all_zero_callback_clock=self.zero_clock,
                    zero_delay_sim_seconds=(self.zero_clock - self.boundary)
                    if self.zero_clock is not None else None)


def stop_observation_self_test():
    observer = StopObservation(THRUSTERS)
    for channel in THRUSTERS:
        observer.record(channel, 0., 9.9)
    observer.arm(10.)
    assert observer.zero_clock is None, 'Old zero frames must not satisfy a new boundary'
    for index, channel in enumerate(THRUSTERS[:3]):
        observer.record(channel, 0., 10.501 + .001 * index)
    observer.record(THRUSTERS[3], .2, 10.504)
    assert observer.zero_clock is None, 'Every channel must be zero'
    observer.record(THRUSTERS[3], 0., 10.505)
    assert abs(observer.zero_clock - 10.505) < 1e-12
    observer.record(THRUSTERS[0], 0., 10.6)
    assert abs(observer.zero_clock - 10.505) < 1e-12, 'The first all-zero set must stay latched'
    observer.arm(20.)
    for channel in THRUSTERS:
        observer.record(channel, 0., 19.9)
    assert observer.zero_clock is None
    for channel in THRUSTERS:
        observer.record(channel, 0., 20.1)
    assert abs(observer.export()['zero_delay_sim_seconds'] - .1) < 1e-12
    observer.arm(30.)
    for channel in THRUSTERS:
        observer.record(channel, float('nan'), 30.1)
    assert observer.zero_clock is None
    for index, channel in enumerate(THRUSTERS):
        observer.record(channel, 0., 30.2 + .001 * index)
    assert abs(observer.zero_clock - 30.203) < 1e-12
    print(json.dumps(dict(stop_observation_self_test='passed', checks=['old_frames_ignored',
        'all_four_channels', 'first_callback_time_latched', 'rearm', 'nonfinite_rejected', 'max_callback_timestamp'])))


class DynamicMotionObservation:
    """Read-only model-truth motion acceptance, independent of navigation."""
    def __init__(self, routes, speed):
        self.routes = dict(routes)
        self.speed = float(speed)
        self.interval = .1
        self.last_sample = None
        self.min_interval = None
        self.max_interval = None
        self.records = {}
        self.last_missing = set(routes)
        self.errors = []

    @classmethod
    def from_world(cls, world, speed):
        source = pathlib.Path(__file__).resolve().parents[1] / 'scripts/navigation_dynamic_buoys.py'
        spec = importlib.util.spec_from_file_location('_smoke_dynamic_routes', str(source))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return cls(module.routes_for_world(world), speed)

    @staticmethod
    def cross_track(point, route):
        distances = []
        for start, end in zip(route[:-1], route[1:]):
            dx, dy = end[0] - start[0], end[1] - start[1]
            fraction = max(0., min(1., ((point[0]-start[0])*dx + (point[1]-start[1])*dy) /
                                   (dx*dx + dy*dy)))
            distances.append(math.hypot(point[0]-start[0]-fraction*dx, point[1]-start[1]-fraction*dy))
        return min(distances)

    def check(self, clock, states):
        if not self.routes or clock is None or not math.isfinite(clock):
            return
        if self.last_sample is not None and clock - self.last_sample < self.interval - 1e-8:
            return
        if self.last_sample is not None:
            interval = clock - self.last_sample
            self.min_interval = interval if self.min_interval is None else min(self.min_interval, interval)
            self.max_interval = interval if self.max_interval is None else max(self.max_interval, interval)
        self.last_sample = clock
        poses = dict(zip(states.name, states.pose))
        self.last_missing = set(self.routes).difference(poses)
        for name, route in self.routes.items():
            if name not in poses:
                continue
            position = poses[name].position
            point = [position.x, position.y]
            if not all(math.isfinite(value) for value in point):
                detail = 'Non-finite dynamic model position: ' + name
                if detail not in self.errors:
                    self.errors.append(detail)
                continue
            record = self.records.get(name)
            if record is None:
                record = dict(first_clock=clock, last_clock=clock, first_xy=point[:], last_xy=point[:],
                              samples=0, cumulative_xy_distance_m=0., max_displacement_m=0.,
                              max_route_cross_track_m=0.)
                self.records[name] = record
            record['cumulative_xy_distance_m'] += math.hypot(point[0]-record['last_xy'][0],
                                                            point[1]-record['last_xy'][1])
            record['max_displacement_m'] = max(record['max_displacement_m'],
                math.hypot(point[0]-record['first_xy'][0], point[1]-record['first_xy'][1]))
            record['max_route_cross_track_m'] = max(record['max_route_cross_track_m'], self.cross_track(point, route))
            record['last_xy'], record['last_clock'] = point[:], clock
            record['samples'] += 1

    def export(self, now=None):
        checks, models = list(self.errors), {}
        if self.routes:
            checks.extend('Missing dynamic model at latest sample: ' + name for name in sorted(self.last_missing))
        for name in self.routes:
            original = self.records.get(name)
            if original is None:
                checks.append('No observed motion samples for dynamic model: ' + name)
                continue
            record = dict(original)
            duration = record['last_clock'] - record['first_clock']
            expected_distance = self.speed * duration
            # Full-run average detects a buoy that moved briefly and then froze.
            # The lower bound separately tolerates finite startup readiness.
            ready_grace = min(2., duration)
            min_distance = min(.25, .25 * expected_distance)
            min_displacement = min(.5, .2 * expected_distance)
            lower_distance = max(min_distance, .75 * self.speed * max(0., duration-ready_grace))
            upper_distance = 1.25 * expected_distance + .05
            record.update(duration_sim_seconds=duration,
                          average_xy_speed_mps=record['cumulative_xy_distance_m']/duration if duration > 0 else None,
                          expected_xy_speed_mps=self.speed,
                          allowed_distance_m=[lower_distance, upper_distance],
                          minimum_motion_distance_m=min_distance, minimum_max_displacement_m=min_displacement)
            models[name] = record
            if record['samples'] < 5 or duration < 1.:
                checks.append('Insufficient continuous dynamic motion samples: ' + name)
            if now is not None and now-record['last_clock'] > .5:
                checks.append('Stale dynamic motion truth samples: ' + name)
            if record['cumulative_xy_distance_m'] <= min_distance or record['max_displacement_m'] < min_displacement:
                checks.append('Dynamic model did not move a significant distance: ' + name)
            if not lower_distance <= record['cumulative_xy_distance_m'] <= upper_distance:
                checks.append('Observed dynamic model speed differs from buoy_speed: ' + name)
            if record['max_route_cross_track_m'] > .5:
                checks.append('Dynamic model left its expected route: ' + name)
        return dict(enabled=True, applicable=bool(self.routes), expected_models=sorted(self.routes),
                    requested_speed_mps=self.speed, requested_interval_sim_seconds=self.interval,
                    speed_tolerance_fraction=.25, maximum_initial_ready_grace_sim_seconds=2.,
                    min_interval_sim_seconds=self.min_interval, max_interval_sim_seconds=self.max_interval,
                    missing_models=sorted(self.last_missing), models=models, errors=checks, pass_=not checks)


def dynamic_motion_self_test():
    from types import SimpleNamespace as Message
    routes = {'buoy': ((0., 0., .3), (10., 0., .3))}
    def states(x):
        return Message(name=['buoy'], pose=[Message(position=Message(x=x, y=0., z=.3))])
    moving, frozen = DynamicMotionObservation(routes, .7), DynamicMotionObservation(routes, .7)
    delayed = DynamicMotionObservation(routes, .7)
    for index in range(101):
        clock = .1*index
        moving.check(clock, states(.7*clock))
        frozen.check(clock, states(0.))
        delayed.check(clock, states(.7*max(0., clock-1.5)))
    assert moving.export(10.)['pass_']
    assert delayed.export(10.)['pass_'], 'Initial service readiness must be tolerated'
    assert not frozen.export(10.)['pass_'], 'A static buoy must never pass a dynamic run'
    assert abs(moving.export()['models']['buoy']['average_xy_speed_mps']-.7) < 1e-10
    assert not moving.export(11.)['pass_'], 'Stale model truth must fail'
    missing = DynamicMotionObservation(routes, .7)
    missing.check(0., Message(name=[], pose=[]))
    assert not missing.export()['pass_']
    unsupported = DynamicMotionObservation({}, .7)
    assert unsupported.export()['pass_'] and not unsupported.export()['applicable']
    moving.check(10.05, states(7.))
    assert moving.records['buoy']['samples'] == 101, 'Do not sample at every physics step'
    stopped = DynamicMotionObservation(routes, .7)
    for index in range(101):
        clock = .1*index
        stopped.check(clock, states(.7*min(clock, 2.)))
    assert not stopped.export(10.)['pass_'], 'Brief initial movement cannot hide a frozen buoy'
    print(json.dumps(dict(dynamic_motion_self_test='passed', checks=['moving_speed', 'ready_grace',
        'frozen_rejected', 'stale_rejected', 'missing_rejected', 'zero_routes', 'sample_throttle', 'late_freeze_rejected'])))


class Samples:
    def __init__(self, rospy):
        self.rospy = rospy
        self.lock = threading.RLock()
        self.stats = {}
        self.latest = {}
        self.latest_arrivals = {}
        self.statuses = collections.Counter()
        self.invalid_final_commands = []
        self.nonzero_command_seen = False
        self.goal_reached = False
        self.goal_reached_clock = None
        self.scan_totals = collections.Counter()
        self.latest_scan_counts = {}
        self.model_names = []
        self.collision_oracle = None
        self.stop_observation = StopObservation(THRUSTERS)
        self.dynamic_motion = None

    def record(self, key, message):
        with self.lock:
            record = self.stats.setdefault(key, dict(count=0, first_stamp=None, last_stamp=None,
                                                      distinct_headers=0, backward_headers=0, frame_ids=[]))
            record["count"] += 1
            valid = valid_extra_sensor(key, message)
            if valid is not None:
                metric = 'valid_payloads' if valid else 'invalid_payloads'
                record[metric] = record.get(metric, 0) + 1
            self.latest[key] = message
            self.latest_arrivals[key] = self.clock_unlocked()
            if hasattr(message, "header"):
                stamp = message.header.stamp.to_sec()
                frame = message.header.frame_id
                if frame not in record["frame_ids"]:
                    record["frame_ids"].append(frame)
                if math.isfinite(stamp) and stamp > 0.:
                    if record["first_stamp"] is None:
                        record["first_stamp"] = stamp
                        record["last_stamp"] = stamp
                        record["distinct_headers"] = 1
                    elif stamp > record["last_stamp"]:
                        record["distinct_headers"] += 1
                        record["last_stamp"] = stamp
                    elif stamp < record["last_stamp"]:
                        record["backward_headers"] += 1
            if key == "status":
                self.statuses[message.data] += 1
                if message.data == "goal_reached":
                    self.goal_reached = True
                    if self.goal_reached_clock is None:
                        self.goal_reached_clock = self.clock_unlocked()
            elif key.startswith("thruster_"):
                value = message.data
                self.stop_observation.record(key[len('thruster_'):], value, self.clock_unlocked())
                if not math.isfinite(value) or not -1. <= value <= 1.:
                    if len(self.invalid_final_commands) < 20:
                        self.invalid_final_commands.append(dict(channel=key, value=str(value)))
                elif abs(value) > 1e-6:
                    self.nonzero_command_seen = True
            elif key == "scan":
                counts = collections.Counter()
                for value in message.ranges:
                    if math.isnan(value):
                        counts["nan"] += 1
                    elif math.isinf(value):
                        counts["positive_infinity" if value > 0 else "negative_infinity"] += 1
                    elif value < 0:
                        counts["negative"] += 1
                    else:
                        counts["finite"] += 1
                counts["bins"] = len(message.ranges)
                self.latest_scan_counts = dict(counts)
                self.scan_totals.update(counts)
            elif key == "model_states":
                self.model_names = list(message.name)
                if self.dynamic_motion is not None:
                    try:
                        self.dynamic_motion.check(self.clock_unlocked(), message)
                    except Exception as error:
                        detail = '%s: %s' % (type(error).__name__, error)
                        if detail not in self.dynamic_motion.errors:
                            self.dynamic_motion.errors.append(detail)
                if self.collision_oracle is not None:
                    try:
                        self.collision_oracle.check(self.clock_unlocked(), message)
                    except Exception as error:
                        detail = '%s: %s' % (type(error).__name__, error)
                        if detail not in self.collision_oracle.errors:
                            self.collision_oracle.errors.append(detail)

    def clock_unlocked(self):
        message = self.latest.get("clock")
        return message.clock.to_sec() if message is not None else None

    def clock(self):
        with self.lock:
            return self.clock_unlocked()

    def get(self, key):
        with self.lock:
            return self.latest.get(key)

    def export(self):
        with self.lock:
            stats = {}
            for key, value in self.stats.items():
                copy = dict(value)
                span = ((copy["last_stamp"] - copy["first_stamp"])
                        if copy["first_stamp"] is not None else 0.)
                copy["header_rate_hz"] = (copy["distinct_headers"] - 1) / span if span > 0. else None
                stats[key] = copy
            finals = {name: (self.latest["thruster_" + name].data
                             if "thruster_" + name in self.latest else None) for name in THRUSTERS}
            def twist_dict(twist):
                return dict(linear=dict(x=twist.linear.x, y=twist.linear.y, z=twist.linear.z),
                            angular=dict(x=twist.angular.x, y=twist.angular.y, z=twist.angular.z))
            odom = self.latest.get('odometry')
            command = self.latest.get('command')
            return dict(sensor_stats=stats, status_counts=dict(self.statuses),
                        actual_model_names=self.model_names, final_commands=finals,
                        any_nonzero_final_command=self.nonzero_command_seen,
                        invalid_final_commands=list(self.invalid_final_commands),
                        scan_latest=self.latest_scan_counts, scan_totals=dict(self.scan_totals),
                        goal_reached=self.goal_reached, goal_reached_clock=self.goal_reached_clock,
                        latest_filtered_twist=twist_dict(odom.twist.twist) if odom is not None else None,
                        latest_command=twist_dict(command) if command is not None else None,
                        current_goal=pose_dict(self.latest['goal'].pose) if 'goal' in self.latest else None,
                        collision_check=self.collision_oracle.export() if self.collision_oracle is not None else dict(enabled=False),
                        stop_observation=self.stop_observation.export(),
                        dynamic_obstacle_check=self.dynamic_motion.export(self.clock_unlocked())
                        if self.dynamic_motion is not None else dict(enabled=False),
                        last_clock=self.clock_unlocked())

    def arm_stop_boundary(self):
        with self.lock:
            clock = self.clock_unlocked()
            self.stop_observation.arm(clock)
            return clock

    def first_all_zero_clock(self):
        with self.lock:
            return self.stop_observation.zero_clock

    def fresh_final_zero(self, after_clock):
        with self.lock:
            for name in THRUSTERS:
                key = 'thruster_' + name
                message = self.latest.get(key)
                arrival = self.latest_arrivals.get(key)
                if message is None or arrival is None or arrival < after_clock:
                    return False
                if not math.isfinite(message.data) or abs(message.data) > 1e-6:
                    return False
            return True


class Harness:
    def __init__(self, args, job_directory):
        self.args = args
        self.directory = pathlib.Path(job_directory)
        self.started = time.monotonic()
        self.deadline = self.started + args.max_wall
        self.owned = None
        self.rospy = None
        self.samples = None
        self.subscribers = []
        self.errors = []
        self.summary = dict(pass_=False, errors=self.errors, warnings=[],
                            full_navigation_pass=False, navigation_assessed=False,
                            ros_master_uri="http://127.0.0.1:%d" % args.ros_port,
                            gazebo_master_uri="http://127.0.0.1:%d" % args.gazebo_port,
                            directory=str(self.directory), requested_sim_seconds=args.sim_seconds,
                            requested_until_goal=args.until_goal,
                            runner_pid=os.getpid(), runner_ppid=os.getppid(),
                            runner_pgid=os.getpgrp(), runner_sid=os.getsid(0),
                            state='starting', received_signals=[])
        self.master = xmlrpc.client.ServerProxy(self.summary["ros_master_uri"],
                                               transport=TimeoutTransport(.4), allow_none=True)

    def checkpoint(self, state):
        self.summary['state'] = state
        payload = dict(self.summary)
        if self.samples is not None:
            payload.update(self.samples.export())
        payload['checkpoint_wall_seconds'] = time.monotonic() - self.started
        atomic_json(self.directory / 'checkpoint.json', payload)

    def remaining(self, maximum=None):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0.:
            raise SmokeFailure("Global wall-clock deadline exceeded")
        return remaining if maximum is None else min(remaining, maximum)

    def ensure_running(self):
        self.remaining()
        if self.owned and self.owned.process.poll() is not None:
            raise SmokeFailure("roslaunch exited unexpectedly with code %s" % self.owned.process.returncode)

    def call(self, function, label, timeout=1.):
        self.ensure_running()
        return bounded_call(function, self.remaining(timeout), label)

    def master_call(self, method, *arguments):
        response = self.call(lambda: getattr(self.master, method)(CALLER, *arguments), "master_" + method, .7)
        if response[0] != 1:
            raise SmokeFailure("Master %s failed: %s" % (method, response[1]))
        return response[2]

    def wait_master(self, startup_deadline):
        last_error = None
        while time.monotonic() < startup_deadline:
            self.ensure_running()
            try:
                pid = int(self.master_call("getPid"))
                if not self.owned.owns(pid):
                    raise SmokeFailure("Master PID %d is outside the owned launch tree; refusing foreign master" % pid)
                self.summary["master_pid"] = pid
                return
            except SmokeFailure as error:
                if "foreign master" in str(error):
                    raise
                last_error = error
            except (OSError, xmlrpc.client.Error) as error:
                last_error = error
            time.sleep(min(.1, self.remaining()))
        raise SmokeFailure("ROS master unavailable within startup deadline: %s" % last_error)

    def connect_ros(self):
        import rospy
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, LaserScan, NavSatFix, PointCloud2
        from std_msgs.msg import Float32, String
        self.rospy = rospy
        rospy.init_node('std_usv_navigation_runtime_smoke', anonymous=True, disable_signals=True)
        self.samples = Samples(rospy)
        self.samples.collision_oracle = self.collision_oracle
        self.samples.dynamic_motion = self.dynamic_motion
        self.goal_publisher = rospy.Publisher('/wamv/navigation/goal', PoseStamped, queue_size=1)
        types = dict(clock=Clock, odometry=Odometry, gps_odometry=Odometry,
                     scan=LaserScan, gps=NavSatFix, imu=Imu, raw_lidar=PointCloud2,
                     status=String, command=Twist, goal=PoseStamped)
        for key, topic in TOPICS.items():
            self.subscribers.append(rospy.Subscriber(topic, types[key],
                lambda message, key=key: self.samples.record(key, message), queue_size=50))
        self.subscribers.append(rospy.Subscriber('/gazebo/model_states', ModelStates,
            lambda message: self.samples.record('model_states', message), queue_size=10))
        for name in THRUSTERS:
            self.subscribers.append(rospy.Subscriber('/wamv/thrusters/' + name + '_thrust_cmd', Float32,
                lambda message, key='thruster_' + name: self.samples.record(key, message), queue_size=50))
        if self.args.all_sensors:
            from geometry_msgs.msg import Vector3Stamped
            from sensor_msgs.msg import CameraInfo, Image
            from usv_msgs.msg import RangeBearing
            extras = [('gps_velocity', '/wamv/sensors/gps/gps/fix_velocity', Vector3Stamped),
                      ('pinger', '/wamv/sensors/pingers/pinger/range_bearing', RangeBearing)]
            for camera in CAMERAS:
                prefix = '/wamv/sensors/cameras/' + camera
                extras.extend([('camera_' + camera + '_image', prefix + '/image_raw', Image),
                               ('camera_' + camera + '_info', prefix + '/camera_info', CameraInfo)])
            for key, topic, message_type in extras:
                self.subscribers.append(rospy.Subscriber(topic, message_type,
                    lambda message, key=key: self.samples.record(key, message),
                    queue_size=1, buff_size=16 * 1024 * 1024))

    def wait_service(self, name, service_type, startup_deadline):
        while time.monotonic() < startup_deadline:
            self.ensure_running()
            try:
                self.rospy.wait_for_service(name, timeout=self.remaining(.3))
                return self.rospy.ServiceProxy(name, service_type, persistent=False)
            except self.rospy.ROSException:
                pass
        raise SmokeFailure('Service not available: ' + name)

    def model(self):
        response = self.call(lambda: self.get_model('wamv', 'world'), 'get_model_state')
        if not response.success:
            raise SmokeFailure('GetModelState failed: ' + response.status_message)
        if not finite_pose(response.pose):
            raise SmokeFailure('Gazebo returned an invalid model pose')
        return response

    def pause_check(self, startup_deadline):
        from gazebo_msgs.srv import GetModelState, GetPhysicsProperties
        from std_srvs.srv import Empty
        self.get_model = self.wait_service('/gazebo/get_model_state', GetModelState, startup_deadline)
        self.get_physics = self.wait_service('/gazebo/get_physics_properties', GetPhysicsProperties, startup_deadline)
        self.unpause = self.wait_service('/gazebo/unpause_physics', Empty, startup_deadline)
        while time.monotonic() < startup_deadline:
            try:
                initial = self.model()
                break
            except SmokeFailure:
                self.ensure_running()
                time.sleep(min(.2, self.remaining()))
        else:
            raise SmokeFailure('WAM-V was not spawned within startup deadline')
        properties = self.call(self.get_physics, 'get_physics_properties')
        self.summary['paused_before_explicit_unpause'] = bool(properties.pause)
        if not properties.pause:
            raise SmokeFailure('paused:=true was unexpectedly unpaused')
        first_clock = self.samples.clock()
        first_pose = pose_dict(initial.pose)
        time.sleep(min(.5, self.remaining()))
        final_clock = self.samples.clock()
        second_model_pose = self.model().pose
        second_pose = pose_dict(second_model_pose)
        advance = (final_clock - first_clock) if first_clock is not None and final_clock is not None else None
        self.summary['paused_clock_advance'] = advance
        self.summary['paused_pose'] = second_pose
        if advance is not None and abs(advance) > .001:
            raise SmokeFailure('Simulation clock advanced while paused')
        if math.hypot(second_pose['x'] - first_pose['x'], second_pose['y'] - first_pose['y']) > .001:
            raise SmokeFailure('Model moved while paused')
        expected = {key[len('usv_'):]: (getattr(self.args, key) if getattr(self.args, key) is not None else default)
                    for key, default in DEFAULT_USV_POSE.items()}
        position_error = math.sqrt(sum((second_pose[axis]-expected[axis])**2 for axis in ('x', 'y', 'z')))
        attitude_error = orientation_error_to_rpy(second_model_pose.orientation,
            expected['roll'], expected['pitch'], expected['yaw'])
        self.summary.update(paused_expected_pose=expected, paused_position_error_m=position_error,
                            paused_orientation_error_rad=attitude_error)
        if position_error > .01 or attitude_error > .01:
            raise SmokeFailure('WAM-V spawn pose does not match launch parameters')

    def stop_owned_planner(self):
        name = '/wamv/navigation/coordinator'
        uri = self.master_call('lookupNode', name)
        node = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport(.5), allow_none=True)
        identity = self.call(lambda: node.getPid(CALLER), 'planner_getPid', .7)
        if identity[0] != 1 or not self.owned.owns(int(identity[2])):
            raise SmokeFailure('Refusing to stop a planner outside the owned launch tree')
        # Arm before shutdown: subscriber callbacks can receive all four zeros
        # while the XMLRPC call or the main polling loop is still waiting.
        before = self.samples.arm_stop_boundary()
        response = self.call(lambda: node.shutdown(CALLER, 'Owned watchdog smoke test'),
                             'planner_shutdown', .7)
        if response[0] != 1:
            raise SmokeFailure('Planner XMLRPC shutdown failed: ' + str(response))
        self.summary['planner_stop'] = dict(node=name, pid=int(identity[2]),
            request_clock=before, response=response, zero_clock=None, zero_delay_sim_seconds=None)
        return before

    def replace_goal(self):
        from geometry_msgs.msg import PoseStamped
        if self.goal_publisher.get_num_connections() < 1:
            raise SmokeFailure('No subscriber for /wamv/navigation/goal replacement')
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.rospy.Time.now()
        message.pose.position.x = self.args.replacement_x
        message.pose.position.y = self.args.replacement_y
        message.pose.orientation.z = math.sin(self.args.replacement_yaw / 2.)
        message.pose.orientation.w = math.cos(self.args.replacement_yaw / 2.)
        with self.samples.lock:
            self.samples.goal_reached = False
            self.samples.goal_reached_clock = None
        self.goal_publisher.publish(message)
        self.summary['replacement_goal'] = dict(published_once=True,
            clock=self.samples.clock(), frame='map', **pose_dict(message.pose))

    def assess(self, initial_truth):
        samples = self.samples
        truth_response = self.model()
        truth = pose_dict(truth_response.pose)
        self.summary['last_truth_pose'] = truth
        self.summary['truth_distance_moved'] = math.hypot(truth['x'] - initial_truth['x'], truth['y'] - initial_truth['y'])
        snapshot = samples.export()
        self.summary.update(snapshot)
        if snapshot['actual_model_names'].count('wamv') != 1:
            self.errors.append('Expected exactly one wamv model')
        for key in ('odometry', 'gps_odometry', 'scan', 'gps', 'imu', 'raw_lidar'):
            stat = snapshot['sensor_stats'].get(key, {})
            if stat.get('distinct_headers', 0) < 3:
                self.errors.append('Insufficient continuous samples for ' + key)
        if self.args.all_sensors:
            expected = dict(gps_velocity=15., pinger=1.)
            for camera in CAMERAS:
                expected['camera_' + camera + '_image'] = 30.
                expected['camera_' + camera + '_info'] = 30.
            self.summary['expected_extra_sensor_rates_hz'] = expected
            for key, rate in expected.items():
                stat = snapshot['sensor_stats'].get(key, {})
                if stat.get('distinct_headers', 0) < 3 or stat.get('valid_payloads', 0) < 3:
                    self.errors.append('Insufficient valid continuous extra sensor data for ' + key)
                if stat.get('invalid_payloads', 0):
                    self.errors.append('Invalid sensor payload observed for ' + key)
                measured = stat.get('header_rate_hz')
                if measured is None or abs(measured - rate) > .2 * rate:
                    self.errors.append('Observed sensor header rate is outside 20 percent tolerance for ' + key)
        for key in THRUSTERS:
            if snapshot['final_commands'][key] is None:
                self.errors.append('Missing final thruster channel ' + key)
        if snapshot['invalid_final_commands']:
            self.errors.append('Non-finite or out-of-range final thruster commands')
        scan = snapshot['scan_totals']
        if scan.get('nan', 0) or scan.get('negative', 0) or scan.get('negative_infinity', 0):
            self.errors.append('Invalid LaserScan values')
        if not scan.get('finite', 0):
            self.errors.append('No finite LaserScan returns observed')
        odometry = samples.get('odometry')
        if odometry is None or not finite_pose(odometry.pose.pose):
            self.errors.append('Missing or invalid filtered pose')
        else:
            estimate = pose_dict(odometry.pose.pose)
            error = math.hypot(estimate['x'] - truth['x'], estimate['y'] - truth['y'])
            yaw_error = abs(angle_difference(estimate['yaw'], truth['yaw']))
            self.summary.update(last_estimated_pose=estimate, position_error_m=error, yaw_error_rad=yaw_error)
            if error > .5 or yaw_error > .1:
                self.errors.append('Localization error exceeds 0.5 m / 0.1 rad')
        active = sum(snapshot['status_counts'].get(key, 0) for key in ('navigating', 'aligning', 'goal_reached'))
        if not active:
            self.errors.append('Planner never entered an active navigation status')
        if not snapshot['any_nonzero_final_command']:
            self.errors.append('No nonzero final thrust observed during navigation')
        if self.args.check_collision:
            collision = snapshot['collision_check']
            if collision['first_collision'] is not None:
                self.errors.append('Collision oracle detected contact: ' + str(collision['first_collision']))
            if collision['samples'] < 3 or collision['missing_models'] or collision['errors']:
                self.errors.append('Collision oracle had insufficient or incomplete samples')
        if self.args.dynamic_obstacles:
            dynamic = snapshot['dynamic_obstacle_check']
            if not dynamic['pass_']:
                self.errors.extend(dynamic['errors'])
            missing = set(dynamic['expected_models']).difference(snapshot['actual_model_names'])
            self.errors.extend('Dynamic model absent at final observation: ' + name for name in sorted(missing))
        if self.args.replace_goal_at_sim is not None:
            goal = snapshot['current_goal']
            if 'replacement_goal' not in self.summary or goal is None:
                self.errors.append('Goal replacement was not exercised')
            elif (math.hypot(goal['x'] - self.args.replacement_x, goal['y'] - self.args.replacement_y) > 1e-6 or
                  abs(angle_difference(goal['yaw'], self.args.replacement_yaw)) > 1e-6):
                self.errors.append('Current goal does not match the published replacement')
        if self.args.stop_planner_at_sim is not None:
            stop = self.summary.get('planner_stop')
            if stop is None:
                self.errors.append('Planner stop was not exercised')
            elif stop['zero_delay_sim_seconds'] is None or stop['zero_delay_sim_seconds'] > MAX_STOP_DELAY + 1e-9:
                self.errors.append('Four thrusters did not stop within 0.55 simulation seconds')
            if any(value is None or abs(value) > 1e-6 for value in snapshot['final_commands'].values()):
                self.errors.append('Thruster commands did not remain zero after planner shutdown')
        if self.args.until_goal:
            self.summary['navigation_assessed'] = True
            if not snapshot['goal_reached']:
                self.errors.append('Goal was not reached within requested simulation/wall bounds')
            goal = samples.get('goal')
            if goal is None or goal.header.frame_id.lstrip('/') != 'map' or not finite_pose(goal.pose):
                self.errors.append('Full navigation requires a valid map-frame /wamv/navigation/goal_request')
            else:
                target = pose_dict(goal.pose)
                distance = math.hypot(truth['x'] - target['x'], truth['y'] - target['y'])
                angular_error = abs(angle_difference(truth['yaw'], target['yaw']))
                self.summary.update(truth_goal_distance_m=distance, truth_goal_yaw_error_rad=angular_error)
                if distance > 1. or angular_error > .15:
                    self.errors.append('Independent final truth pose is outside arrival bounds')
            speed = math.hypot(truth_response.twist.linear.x, truth_response.twist.linear.y)
            yawrate = abs(truth_response.twist.angular.z)
            self.summary.update(truth_final_planar_speed_mps=speed, truth_final_yawrate_rps=yawrate)
            if not math.isfinite(speed) or not math.isfinite(yawrate) or speed > .1 or yawrate > .1:
                self.errors.append('Independent final truth velocity exceeds 0.1 m/s or 0.1 rad/s')
            if any(value is None or abs(value) > 1e-6 for value in snapshot['final_commands'].values()):
                self.errors.append('Final thrust did not return to zero after arrival')
            self.summary['full_navigation_pass'] = not self.errors
        self.summary['pass_'] = not self.errors

    def run(self):
        self.collision_oracle = None
        self.dynamic_motion = None
        if self.args.dynamic_obstacles:
            self.dynamic_motion = DynamicMotionObservation.from_world(self.args.world or 'map1-3.world', self.args.buoy_speed)
        if self.args.check_collision:
            import rospkg
            world = self.args.world or str(pathlib.Path(rospkg.RosPack().get_path('ocean_simulation_gazebo')) /
                                           'worlds/usv_path_planning/map1-3.world')
            self.collision_oracle = CollisionOracle.from_world(world)
        check_ports_free(self.args.ros_port, self.args.gazebo_port)
        environment = dict(os.environ)
        environment.update(ROS_MASTER_URI=self.summary['ros_master_uri'],
                           GAZEBO_MASTER_URI=self.summary['gazebo_master_uri'],
                           ROS_HOSTNAME='localhost', ROS_IP='127.0.0.1',
                           ROS_HOME=str(self.directory / 'ros_home'),
                           ROS_LOG_DIR=str(self.directory / 'ros_log'))
        for key in ('ROS_HOME', 'ROS_LOG_DIR'):
            pathlib.Path(environment[key]).mkdir(parents=True, exist_ok=True)
        # rospy and rosservice read their connection settings from our environment.
        os.environ.update({key: environment[key] for key in
                           ('ROS_MASTER_URI', 'GAZEBO_MASTER_URI', 'ROS_HOSTNAME', 'ROS_IP', 'ROS_HOME', 'ROS_LOG_DIR')})
        command = ['roslaunch', 'std_usv', 'demo_astar_dwa.launch', 'gui:=false',
                   'rviz:=false', 'paused:=true', 'verbose:=true']
        if self.args.world:
            command.append('world:=' + self.args.world)
        for key in ('goal_x', 'goal_y', 'goal_yaw') + tuple(DEFAULT_USV_POSE):
            value = getattr(self.args, key)
            if value is not None:
                command.append(key + ':=' + str(value))
        command.extend(['dynamic_obstacles:=' + str(self.args.dynamic_obstacles).lower(),
                        'buoy_speed:=' + str(self.args.buoy_speed)])
        self.summary['launch_command'] = command
        self.summary['launch_log'] = str(self.directory / 'roslaunch.log')
        self.owned = OwnedLaunch(command, environment, self.summary['launch_log'])
        self.summary['launch_pid'] = self.owned.process.pid
        self.checkpoint('roslaunch_started')
        startup_deadline = min(self.deadline, time.monotonic() + 45.)
        self.wait_master(startup_deadline)
        self.connect_ros()
        self.pause_check(startup_deadline)
        initial_truth = pose_dict(self.model().pose)
        self.call(self.unpause, 'explicit_unpause')
        initial_clock = self.samples.clock()
        if initial_clock is None:
            initial_clock = 0.
        self.summary['unpause_clock'] = initial_clock
        planner_stop_clock = None
        last_progress_clock, last_progress_wall = initial_clock, time.monotonic()
        while True:
            self.ensure_running()
            current_clock = self.samples.clock()
            if current_clock is not None:
                if (self.args.replace_goal_at_sim is not None and 'replacement_goal' not in self.summary and
                        current_clock - initial_clock >= self.args.replace_goal_at_sim):
                    self.replace_goal()
                if current_clock - last_progress_clock >= 20. or time.monotonic() - last_progress_wall >= 30.:
                    odom, command, status = (self.samples.get(key) for key in ('odometry', 'command', 'status'))
                    report = dict(clock=current_clock,
                                  status=status.data if status is not None else None)
                    if odom is not None:
                        report.update(x=odom.pose.pose.position.x, y=odom.pose.pose.position.y,
                                      v=odom.twist.twist.linear.x, w=odom.twist.twist.angular.z)
                    if command is not None:
                        report.update(cmd_v=command.linear.x, cmd_w=command.angular.z)
                    print('PROGRESS ' + json.dumps(report, sort_keys=True), file=sys.stderr, flush=True)
                    self.checkpoint('measuring')
                    last_progress_clock, last_progress_wall = current_clock, time.monotonic()
                if self.args.stop_planner_at_sim is not None:
                    if planner_stop_clock is None and current_clock - initial_clock >= self.args.stop_planner_at_sim:
                        planner_stop_clock = self.stop_owned_planner()
                    if planner_stop_clock is not None:
                        stop = self.summary['planner_stop']
                        callback_clock = self.samples.first_all_zero_clock()
                        if stop['zero_clock'] is None and callback_clock is not None:
                            stop['zero_clock'] = callback_clock
                            stop['zero_delay_sim_seconds'] = max(0., callback_clock - planner_stop_clock)
                        if current_clock - planner_stop_clock >= 2.:
                            break
                replacement_ready = self.args.replace_goal_at_sim is None or 'replacement_goal' in self.summary
                if self.args.until_goal and replacement_ready and self.samples.goal_reached:
                    settled = self.samples.goal_reached_clock
                    if settled is not None and current_clock - settled >= 2.:
                        break
                if self.args.stop_planner_at_sim is None and current_clock - initial_clock >= self.args.sim_seconds:
                    break
            time.sleep(min(.1, self.remaining()))
        self.summary['elapsed_sim_seconds'] = self.samples.clock() - initial_clock
        self.assess(initial_truth)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-wall', type=float, default=90.)
    parser.add_argument('--sim-seconds', type=float, default=10.)
    parser.add_argument('--until-goal', action='store_true')
    parser.add_argument('--world', default='')
    parser.add_argument('--ros-port', type=int, default=ROS_PORT)
    parser.add_argument('--gazebo-port', type=int, default=GAZEBO_PORT)
    parser.add_argument('--dynamic-obstacles', action='store_true')
    parser.add_argument('--all-sensors', action='store_true')
    parser.add_argument('--check-collision', action='store_true')
    parser.add_argument('--self-test-collision', action='store_true')
    parser.add_argument('--self-test-stop-observation', action='store_true')
    parser.add_argument('--self-test-dynamic-motion', action='store_true')
    parser.add_argument('--self-test-cleanup-wait', action='store_true')
    parser.add_argument('--buoy-speed', type=float, default=.7)
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    parser.add_argument('--goal-yaw', type=float)
    for key in DEFAULT_USV_POSE:
        parser.add_argument('--' + key.replace('_', '-'), type=float)
    parser.add_argument('--stop-planner-at-sim', type=float)
    parser.add_argument('--replace-goal-at-sim', type=float)
    parser.add_argument('--replacement-x', type=float, default=-505.)
    parser.add_argument('--replacement-y', type=float, default=232.)
    parser.add_argument('--replacement-yaw', type=float, default=.3)
    parser.add_argument('--output-directory', default='')
    args = parser.parse_args()
    if args.self_test_collision:
        collision_self_test()
        return 0
    if args.self_test_stop_observation:
        stop_observation_self_test()
        return 0
    if args.self_test_dynamic_motion:
        dynamic_motion_self_test()
        return 0
    if args.self_test_cleanup_wait:
        cleanup_wait_self_test()
        return 0
    if not math.isfinite(args.max_wall) or not math.isfinite(args.sim_seconds) or args.max_wall <= 0 or args.sim_seconds <= 0:
        parser.error('Time bounds must be finite and positive')
    if any(not 1 <= port <= 65535 for port in (args.ros_port, args.gazebo_port)) or args.ros_port == args.gazebo_port:
        parser.error('ROS and Gazebo ports must be distinct integers in 1..65535')
    if not math.isfinite(args.buoy_speed) or args.buoy_speed <= 0:
        parser.error('Buoy speed must be finite and positive')
    if any(value is not None and not math.isfinite(value) for value in
           (args.goal_x, args.goal_y, args.goal_yaw, args.stop_planner_at_sim,
            args.replace_goal_at_sim, args.replacement_x, args.replacement_y, args.replacement_yaw,
            *(getattr(args, key) for key in DEFAULT_USV_POSE))):
        parser.error('Goal, placement and stop time values must be finite')
    if args.stop_planner_at_sim is not None and (args.stop_planner_at_sim <= 0 or args.until_goal):
        parser.error('--stop-planner-at-sim must be positive and cannot be combined with --until-goal')
    if args.replace_goal_at_sim is not None and (args.replace_goal_at_sim <= 0 or
            args.replace_goal_at_sim >= args.sim_seconds or args.stop_planner_at_sim is not None):
        parser.error('--replace-goal-at-sim must be inside the simulation interval and cannot combine with planner stop')
    directory = args.output_directory or tempfile.mkdtemp(prefix='std-usv-runtime-')
    pathlib.Path(directory).mkdir(parents=True, exist_ok=True)
    harness = Harness(args, directory)
    old_handlers = {}

    def interrupt(signum, frame):
        harness.summary['received_signals'].append(signum)
        raise Interrupted('Interrupted by signal %d' % signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, interrupt)
    try:
        harness.checkpoint('starting')
        harness.run()
    except BaseException as error:
        harness.errors.append('%s: %s' % (type(error).__name__, error))
        harness.summary['traceback'] = traceback.format_exc()
        if harness.samples is not None:
            harness.summary.update(harness.samples.export())
    finally:
        # ROS shutdown and process cleanup have separate bounded time budgets.
        for sig in old_handlers:
            signal.signal(sig, signal.SIG_IGN)
        if harness.rospy is not None:
            try:
                bounded_call(lambda: harness.rospy.signal_shutdown('Owned smoke test complete'),
                             1.5, 'rospy_shutdown')
            except BaseException as error:
                harness.errors.append('ROS shutdown error: ' + str(error))
        if harness.owned is not None:
            try:
                harness.summary['cleanup'] = harness.owned.cleanup()
            except BaseException as error:
                harness.errors.append('Cleanup error: ' + str(error))
        harness.summary['wall_seconds_including_cleanup'] = time.monotonic() - harness.started
        harness.summary['pass_'] = bool(harness.summary['pass_'] and not harness.errors)
        harness.summary['state'] = 'complete'
        result_path = pathlib.Path(directory) / 'result.json'
        harness.summary['result_path'] = str(result_path)
        atomic_json(result_path, harness.summary)
        try:
            print(json_payload(harness.summary), flush=True)
        except BrokenPipeError:
            pass  # A disconnected terminal must not discard the on-disk result.
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 0 if harness.summary['pass_'] else 1


if __name__ == '__main__':
    sys.exit(main())
