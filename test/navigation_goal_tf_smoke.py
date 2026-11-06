#!/usr/bin/env python3
"""Manual CPU-only goal/TF ordering check; no Gazebo or actuator is started.

Source-only, excluded from ordinary unit tests and installation. Start after
building/sourcing the workspace. The script owns a private ROS master and the
two production goal/planner nodes, supplies synthetic map/odom/scan/base TF,
and checks delayed goal TF, fixed map goals and replacement cancellation.
The work deadline is at most 50 wall seconds, followed by at most eight seconds
of cleanup restricted to the process identities created by this script.
"""

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import tempfile
import threading
import time
import xmlrpc.client

import roslib.packages


def helpers():
    # Reuse the source-only harness's birth-identity-aware process ownership;
    # importing it never starts ROS, Gazebo, or any of its acceptance cases.
    path = Path(__file__).with_name("navigation_runtime_smoke.py")
    spec = importlib.util.spec_from_file_location("_goal_tf_smoke_helpers", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GoalTfCheck:
    def __init__(self, args):
        self.args = args
        self.deadline = time.monotonic() + args.max_wall
        self.folder = Path(tempfile.mkdtemp(prefix="std-usv-goal-tf-"))
        self.environment = dict(os.environ)
        self.environment.update(ROS_MASTER_URI="http://127.0.0.1:{}".format(args.ros_port),
                                ROS_HOSTNAME="localhost", ROS_IP="127.0.0.1",
                                ROS_HOME=str(self.folder / "ros_home"),
                                ROS_LOG_DIR=str(self.folder / "ros_log"))
        for name in ("ROS_HOME", "ROS_LOG_DIR"):
            Path(self.environment[name]).mkdir()
        self.helper = helpers()
        self.owned = []
        self.timer = None
        self.lock = threading.RLock()
        self.events = {"status": [], "command": [], "path": [], "goal_request": []}
        self.sent_requests = []
        self.frames = {}
        self.boat = (0., 0., 0.)
        self.map_version = 0
        self.result = dict(pass_=False, errors=[], output=str(self.folder),
                           ros_master_uri=self.environment["ROS_MASTER_URI"],
                           gazebo_started=False, checks=[])

    def wait_until(self, condition, seconds, failure):
        end = min(self.deadline, time.monotonic() + seconds)
        while time.monotonic() < end:
            if any(item.process.poll() is not None for item in self.owned):
                raise RuntimeError("Owned ROS process exited before check completed")
            if condition():
                return
            time.sleep(.02)
        raise RuntimeError(failure)

    def hold(self, seconds):
        end = time.monotonic() + seconds
        self.wait_until(lambda: time.monotonic() >= end, seconds + .1, "Wall deadline expired")

    def launch(self, command, log):
        child = self.helper.OwnedLaunch(command, self.environment, self.folder / log)
        self.owned.append(child)
        return child

    def record(self, name, message):
        with self.lock:
            self.events[name].append((time.monotonic(), message))

    def latest_status(self):
        with self.lock:
            return self.events["status"][-1][1].data if self.events["status"] else None

    def wait_status(self, status):
        self.wait_until(lambda: self.latest_status() == status, 5,
                        "Planner never entered " + status)

    def assert_zero(self, status, duration=.4):
        self.wait_status(status)
        self.hold(.15)  # Drain independently delivered status/command messages.
        start = time.monotonic()
        self.hold(duration)
        with self.lock:
            commands = [m for t, m in self.events["command"] if t >= start]
            statuses = [m.data for t, m in self.events["status"] if t >= start]
        if len(commands) < 2 or not statuses or any(s != status for s in statuses):
            raise RuntimeError("Insufficient continuous stopped " + status + " observations")
        if any(abs(v) > 1e-9 for m in commands for v in
               (m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z)):
            raise RuntimeError("Nonzero command while " + status)

    @staticmethod
    def endpoint(x, y):
        return (math.floor((x + 10.) / .5) * .5 - 9.75,
                math.floor((y + 10.) / .5) * .5 - 9.75)

    def wait_path(self, target, since=0.):
        expected = self.endpoint(*target)

        def found():
            with self.lock:
                paths = [m for t, m in self.events["path"] if t > since and m.poses]
            return bool(paths and paths[-1].header.frame_id == "map" and
                        math.hypot(paths[-1].poses[-1].pose.position.x - expected[0],
                                   paths[-1].poses[-1].pose.position.y - expected[1]) < 1e-6)

        self.wait_until(found, 5, "No map path ending at expected cell " + str(expected))

    def publish_map(self):
        message = self.OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.rospy.Time.now()
        message.info.width = message.info.height = 60
        message.info.resolution = .5
        message.info.origin.position.x = message.info.origin.position.y = -10.
        message.info.origin.orientation.w = 1.
        message.data = [0] * 3600
        message.data[0] = 100 if self.map_version % 2 else 0
        self.map_publisher.publish(message)

    def publish_inputs(self, _event):
        now = self.rospy.Time.now()
        with self.lock:
            boat, frames = self.boat, dict(self.frames)
        transforms = []
        for name, (x, y, yaw) in {"wamv/base_link": boat, **frames}.items():
            t = self.TransformStamped()
            t.header.frame_id, t.child_frame_id, t.header.stamp = "map", name, now
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
            transforms.append(t)
        self.broadcaster.sendTransform(transforms)
        odom = self.Odometry()
        odom.header.frame_id, odom.child_frame_id, odom.header.stamp = "map", "wamv/base_link", now
        odom.pose.pose.position.x, odom.pose.pose.position.y = boat[:2]
        odom.pose.pose.orientation.z = math.sin(boat[2] / 2.)
        odom.pose.pose.orientation.w = math.cos(boat[2] / 2.)
        self.odom_publisher.publish(odom)
        scan = self.LaserScan()
        scan.header.frame_id, scan.header.stamp = "wamv/base_link", now
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = math.pi / 180.
        scan.range_min, scan.range_max = .2, 30.
        scan.ranges = [math.inf] * 360
        self.scan_publisher.publish(scan)

    def goal(self, frame, x=0., y=0., yaw=0., bad_quaternion=False):
        message = self.PoseStamped()
        message.header.frame_id = frame
        message.pose.position.x, message.pose.position.y = x, y
        if not bad_quaternion:
            # Exercise normalization too; valid goals need not already be unit.
            message.pose.orientation.z = 2. * math.sin(yaw / 2.)
            message.pose.orientation.w = 2. * math.cos(yaw / 2.)
        since = time.monotonic()
        self.sent_requests.append(message)
        self.goal_publisher.publish(message)  # Intentionally once only.

        def forwarded():
            with self.lock:
                return any(t >= since and self.same_request(message, received)
                           for t, received in self.events["goal_request"])

        self.wait_until(forwarded, 5, "Goal request was not relayed unchanged")

    @staticmethod
    def same_request(sent, received):
        # ROS transports may assign a new header sequence number on publication.
        return (sent.header.frame_id == received.header.frame_id and
                sent.header.stamp == received.header.stamp and sent.pose == received.pose)

    def check(self):
        with socket.socket() as probe:
            probe.bind(("0.0.0.0", self.args.ros_port))  # Refuse an existing master.
        self.launch(["roscore", "-p", str(self.args.ros_port)], "roscore.log")
        master = xmlrpc.client.ServerProxy(self.environment["ROS_MASTER_URI"],
                                          transport=self.helper.TimeoutTransport(.3))

        def ready():
            try:
                code, _, pid = master.getPid("/navigation_goal_tf_smoke")
                return code == 1 and self.owned[0].owns(pid)
            except (OSError, xmlrpc.client.Error):
                return False

        self.wait_until(ready, 12, "Private ROS master did not start")
        os.environ.update({k: self.environment[k] for k in
                           ("ROS_MASTER_URI", "ROS_HOSTNAME", "ROS_IP", "ROS_HOME", "ROS_LOG_DIR")})
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.rospy, self.PoseStamped = rospy, PoseStamped
        self.TransformStamped, self.OccupancyGrid = TransformStamped, OccupancyGrid
        self.Odometry, self.LaserScan = Odometry, LaserScan
        rospy.init_node("navigation_goal_tf_smoke", anonymous=True, disable_signals=True)
        rospy.set_param("/use_sim_time", False)
        rospy.set_param("/wamv/navigation/model/footprint", [[-.6, -.4], [.6, -.4], [.6, .4], [-.6, .4]])
        self.map_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.scan_publisher = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.goal_publisher = rospy.Publisher("/wamv/navigation/goal", PoseStamped, queue_size=1)
        self.subscribers = [rospy.Subscriber(topic, message_type,
            lambda m, name=name: self.record(name, m), queue_size=100)
            for topic, message_type, name in [
                ("/wamv/navigation/status", String, "status"), ("/cmd_vel", Twist, "command"),
                ("/wamv/navigation/global_path", NavPath, "path"),
                ("/wamv/navigation/goal_request", PoseStamped, "goal_request")]]
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(.05), self.publish_inputs)
        self.publish_map()
        binaries = {name: roslib.packages.find_node("std_usv", name) for name in
                    ("navigation_goal_relay", "navigation_coordinator_node")}
        if not all(binaries.values()):
            raise RuntimeError("Build/source the production goal and planner nodes first")
        # The real initial-goal publisher sends once before the planner starts.
        self.launch([binaries["navigation_goal_relay"][0], "__ns:=/wamv/navigation", "__name:=goal_relay",
                     "_goal_frame:=late_goal", "_goal_x:=2",
                     "_goal_y:=1", "_goal_yaw:=-0.2"], "goal_gateway.log")
        self.wait_until(lambda: self.goal_publisher.get_num_connections() > 0, 5,
                        "Goal gateway did not subscribe")
        self.wait_until(lambda: bool(self.events["goal_request"]), 5, "Initial goal request was not published")
        late_requests = []
        self.subscribers.append(rospy.Subscriber("/wamv/navigation/goal_request", PoseStamped,
                                                late_requests.append, queue_size=1))
        self.wait_until(lambda: bool(late_requests), 5, "Late subscriber did not receive latched request")
        if (late_requests[0]._connection_header.get("latching") != "1" or
                not self.same_request(self.events["goal_request"][0][1], late_requests[0])):
            raise RuntimeError("Goal request latch or payload was not preserved")
        self.launch([binaries["navigation_coordinator_node"][0], "__ns:=/wamv/navigation",
                     "__name:=coordinator", "_GOAL_YAW_TOLERANCE:=0.1"], "planner.log")
        self.assert_zero("waiting_for_inputs")
        self.result["checks"].append("one_initial_goal_waits_and_stops_without_tf")
        code, _, state = master.getSystemState("/navigation_goal_tf_smoke")
        goal_inputs = [topic for topic, nodes in state[1]
                       if "/wamv/navigation/coordinator" in nodes and "goal" in topic]
        if code != 1 or goal_inputs != ["/wamv/navigation/goal_request"]:
            raise RuntimeError("Coordinator subscribed to unexpected goal inputs: " + str(goal_inputs))
        self.result["checks"].append("coordinator_subscribes_only_to_goal_request")

        target = (3. + 2. * math.cos(.4) - math.sin(.4),
                  2. + 2. * math.sin(.4) + math.cos(.4))
        with self.lock:
            self.frames["late_goal"] = (3., 2., .4)
        self.wait_path(target)
        self.wait_status("navigating")
        self.result["initial_map_goal"] = dict(x=target[0], y=target[1], yaw=.2)
        self.result["checks"].append("late_nonzero_translation_and_yaw_tf_resolves_without_resend")

        # At the transformed position, goal yaw +.2 must request positive turn;
        # retaining the source yaw -.2 would instead request a negative turn.
        with self.lock:
            self.boat = (target[0], target[1], 0.)
        self.wait_status("aligning")
        self.hold(.15)
        yaw_start = time.monotonic()
        self.hold(.25)
        with self.lock:
            yaw_commands = [m.angular.z for t, m in self.events["command"] if t >= yaw_start]
        if len(yaw_commands) < 2 or any(value <= .01 for value in yaw_commands):
            raise RuntimeError("Transformed goal yaw did not produce continuous positive alignment")
        self.result["alignment_yaw_commands"] = yaw_commands
        self.result["checks"].append("goal_orientation_transformed")
        # Error .125 exceeds the configured .1 but is below the old default .15.
        # This proves the renamed parameter is read and applies to final yaw.
        with self.lock:
            self.boat = (target[0], target[1], .075)
        self.hold(.25)
        yaw_start = time.monotonic()
        self.hold(.25)
        with self.lock:
            yaw_commands = [m.angular.z for t, m in self.events["command"] if t >= yaw_start]
        if len(yaw_commands) < 2 or any(value <= .01 for value in yaw_commands):
            raise RuntimeError("GOAL_YAW_TOLERANCE did not control final orientation")
        with self.lock:
            self.boat = (target[0], target[1], .125)  # Error .075 is within .1.
        self.assert_zero("aligning", duration=.25)
        self.result["checks"].append("goal_yaw_tolerance_controls_final_orientation")
        with self.lock:
            self.boat = (0., 0., 0.)
            self.frames["late_goal"] = (-3., 5., -.5)
        self.hold(.25)
        start = time.monotonic()
        self.map_version += 1
        self.publish_map()  # Force a new route, proving the map goal is frozen.
        self.wait_path(target, since=start)
        self.result["checks"].append("accepted_goal_does_not_follow_moving_source_frame")
        with self.lock:
            if len(self.events["goal_request"]) != 1:
                raise RuntimeError("Initial goal request was published more than once")
        self.result["checks"].append("initial_goal_request_is_latched_and_sent_once")

        self.goal("cancelled_goal", 3., 2., .6)
        self.assert_zero("waiting_for_inputs")
        self.goal("map", bad_quaternion=True)
        self.wait_status("invalid_goal")
        with self.lock:
            self.frames["cancelled_goal"] = (5., 5., -.3)
        self.assert_zero("invalid_goal", duration=.6)
        with self.lock:
            if self.events["path"][-1][1].poses:
                raise RuntimeError("Invalid replacement allowed an old pending route to revive")
        self.result["checks"].append("invalid_quaternion_replacement_cancels_pending_goal")
        self.goal("")
        self.assert_zero("invalid_goal", duration=.25)
        self.result["checks"].append("empty_frame_remains_invalid")

        self.goal("superseded_goal", 1., 0., .4)
        self.assert_zero("waiting_for_inputs", duration=.25)
        self.goal("map", 7., 2., 0.)
        self.wait_path((7., 2.))
        with self.lock:
            self.frames["superseded_goal"] = (9., 9., 1.)
        self.hold(.2)
        start = time.monotonic()
        self.map_version += 1
        self.publish_map()
        self.wait_path((7., 2.), since=start)
        self.wait_status("navigating")
        self.result["checks"].append("new_map_goal_cancels_old_pending_request")
        with self.lock:
            forwarded = [message for _, message in self.events["goal_request"]][1:]
        if len(forwarded) != len(self.sent_requests) or any(
                not self.same_request(sent, received) for sent, received in zip(self.sent_requests, forwarded)):
            raise RuntimeError("Replacement goal requests were duplicated or modified")
        self.result["checks"].append("replacement_requests_forwarded_once_without_modification")

    def cleanup(self):
        if self.timer is not None:
            self.timer.shutdown()
        for child in reversed(self.owned):
            child.signal_owned(signal.SIGINT)
        end = time.monotonic() + 4.
        while time.monotonic() < end:
            if not any(child.live() for child in self.owned):
                break
            time.sleep(.05)
        for child in reversed(self.owned):
            if child.live():
                child.signal_owned(signal.SIGKILL)
        end = time.monotonic() + 2.
        while time.monotonic() < end:
            for child in self.owned:
                child.process.poll()
            if not any(child.live() for child in self.owned):
                break
            time.sleep(.05)
        remaining = []
        for child in self.owned:
            remaining.extend(entry["pid"] for entry in child.live())
            child.finished.set()
            child.monitor.join(timeout=.1)
            child.log_stream.close()
            child.process.poll()
        self.result["remaining_owned_pids"] = remaining
        if remaining:
            self.result["errors"].append("Owned processes remained after bounded cleanup")

    def run(self):
        started = time.monotonic()
        try:
            self.check()
        except (Exception, KeyboardInterrupt) as error:
            self.result["errors"].append(str(error) or "Interrupted")
        finally:
            self.cleanup()
        self.result["wall_seconds_including_cleanup"] = time.monotonic() - started
        self.result["pass_"] = not self.result["errors"]
        (self.folder / "result.json").write_text(json.dumps(self.result, indent=2) + "\n")
        print(json.dumps(self.result, indent=2), flush=True)
        return 0 if self.result["pass_"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-port", type=int, default=12021)
    parser.add_argument("--max-wall", type=float, default=40.)
    args = parser.parse_args()
    if not 1024 <= args.ros_port <= 65535 or not 10. <= args.max_wall <= 50.:
        parser.error("Choose a private port 1024..65535 and a 10..50 second work deadline")
    return GoalTfCheck(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
