#!/usr/bin/env python3
"""Convert allocator forces to the unmodified WAM-V GLF plugin commands.

The archived allocator publishes force / 800.  This adapter restores newtons,
scales all four forces by one factor, and numerically inverts the actual
UsvThrust::GlfThrustCmd branches (including their unusual ``cmd < .01`` test).
The plugin's float coefficients and Float32 command transport are represented
explicitly.  Its gap between the two branches makes a maximum ~0.58 N inverse
error unavoidable for small positive forces.  A requested stop is exactly the
normalized command 0; the unchanged plugin maps that to ~0.0414 N, not zero N.

Only this node publishes final thrust commands.  Invalid/stale inputs and
inactive navigation always publish normalized zero.  A same-configuration
dynamic-reconfigure call resets the existing velocity PID before each enable;
the service runs on a daemon thread and the gate stays closed until its reply
and a complete new allocator frame have arrived.

A fresh zero velocity target during navigating/aligning keeps feedback control
enabled, allowing the existing PID to brake and settle.  Navigation status,
freshness and validity determine whether thrust is allowed; inactive states
including goal_reached always bypass control and publish normalized zero.
"""

import math
import struct
import threading


THRUSTERS = ("left_front", "right_front", "left_rear", "right_rear")
PID_KEYS = ("linear_p", "linear_i", "linear_d", "linear_sat",
            "angular_p", "angular_i", "angular_d", "angular_sat")


def float32(value):
    return struct.unpack("f", struct.pack("f", value))[0]


FORWARD = tuple(float32(x) for x in (.01, 59.82, 5., .38, .56, .28))
REVERSE = tuple(float32(x) for x in (-199.13, -.09, 8.84, 5.34, .99, -.57))
# The closest Float32 values below and above the double branch threshold.
BRANCH_BELOW = float32(.01)
BRANCH_ABOVE = struct.unpack("f", struct.pack("I", struct.unpack("I", struct.pack("f", .01))[0] + 1))[0]


def glf(command, coefficients):
    a, k, b, v, c, m = coefficients
    return a + (k - a) / (c + math.exp(-b * (command - m))) ** (1. / v)


def glf_force(command, max_forward=250., max_reverse=-100.):
    """Match the source's parameter order, branch condition and force clamps."""
    if not math.isfinite(command) or not -1. <= command <= 1.:
        raise ValueError("normalized command must be finite and within [-1, 1]")
    if command > .01:
        return min(glf(command, FORWARD), max_forward)
    if command < .01:
        return max(glf(command, REVERSE), max_reverse)
    return 0.


def force_limits(max_forward=250., max_reverse=-100.):
    if not (math.isfinite(max_forward) and max_forward > 0. and
            math.isfinite(max_reverse) and max_reverse < 0.):
        raise ValueError("force limits must have positive forward and negative reverse values")
    return glf_force(-1., max_forward, max_reverse), glf_force(1., max_forward, max_reverse)


def scale_forces(forces, max_forward=250., max_reverse=-100.):
    """Keep force signs and ratios with a common saturation factor."""
    if len(forces) != 4 or not all(math.isfinite(x) for x in forces):
        raise ValueError("a complete finite four-thruster frame is required")
    reverse, forward = force_limits(max_forward, max_reverse)
    scale = 1.
    for force in forces:
        if force > forward:
            scale = min(scale, forward / force)
        elif force < reverse:
            scale = min(scale, reverse / force)
    return tuple(force * scale for force in forces)


def inverse_glf(force, max_forward=250., max_reverse=-100.):
    """Return the nearest realizable Float32 command, with explicit zero stop.

    Both monotonic branches are searched.  Around the positive discontinuity
    the closer endpoint wins; small negative commands can produce small
    positive force because this is what the installed plugin implements.
    """
    if not math.isfinite(force):
        raise ValueError("force must be finite")
    if force == 0.:
        return 0.
    lower, upper = force_limits(max_forward, max_reverse)
    target = max(lower, min(upper, force))
    candidates = [0., -1., 1., BRANCH_BELOW, BRANCH_ABOVE]
    for low, high in ((-1., BRANCH_BELOW), (BRANCH_ABOVE, 1.)):
        for _ in range(64):
            mid = (low + high) / 2.
            if glf_force(mid, max_forward, max_reverse) < target:
                low = mid
            else:
                high = mid
        candidates.extend((float32(low), float32(high)))
    return min(candidates, key=lambda command: (
        abs(glf_force(command, max_forward, max_reverse) - target), abs(command)))


def convert_raw_frame(raw, allocator_scale=800., max_forward=250., max_reverse=-100.):
    if not math.isfinite(allocator_scale) or allocator_scale <= 0.:
        raise ValueError("allocator_scale must be positive and finite")
    forces = scale_forces(tuple(value * allocator_scale for value in raw), max_forward, max_reverse)
    return tuple(inverse_glf(force, max_forward, max_reverse) for force in forces)


class OutputGuard:
    """ROS-independent freshness, coherent-frame and PID-reset state machine."""

    DEFAULT_TIMEOUTS = dict(command=.5, odometry=.5, scan=.5, gps=1., imu=1., status=.5, raw=.5)

    def __init__(self, timeouts=None, sync_window=.1):
        self.timeouts = dict(self.DEFAULT_TIMEOUTS)
        self.timeouts.update(timeouts or {})
        if not all(math.isfinite(v) and v > 0. for v in self.timeouts.values()):
            raise ValueError("timeouts must be positive and finite")
        if not math.isfinite(sync_window) or sync_window < 0.:
            raise ValueError("sync_window must be nonnegative and finite")
        self.sync_window = sync_window
        self.samples = {}
        self.command = (0.,) * 6
        self.status = "waiting_for_inputs"
        self.pending_raw = {}
        self.frame = None
        self.frame_stamps = None
        self.reset_required = True
        self.reset_pending = None
        self.generation = 0
        self.reset_ack = None
        self.active = False
        self.last_now = None

    def close(self):
        if not self.reset_required or self.reset_pending is not None:
            self.generation += 1
        self.active = False
        self.reset_required = True
        self.reset_pending = None
        self.reset_ack = None

    def observe(self, name, stamp, valid=True):
        self.samples[name] = stamp if valid and math.isfinite(stamp) else None
        if self.samples[name] is None:
            self.close()

    def set_command(self, values, stamp):
        valid = len(values) == 6 and all(math.isfinite(v) for v in values)
        self.command = tuple(values) if valid else (0.,) * 6
        self.observe("command", stamp, valid)

    def set_status(self, status, stamp):
        self.status = status
        self.observe("status", stamp, isinstance(status, str))
        if status not in ("navigating", "aligning"):
            self.close()

    def set_raw(self, name, value, stamp):
        if name not in THRUSTERS:
            raise ValueError("unknown thruster")
        if not math.isfinite(value) or not math.isfinite(stamp):
            self.frame = self.frame_stamps = None
            self.pending_raw.clear()
            self.close()
            return
        self.pending_raw[name] = (value, stamp)
        if len(self.pending_raw) == 4:
            stamps = tuple(self.pending_raw[key][1] for key in THRUSTERS)
            if max(stamps) - min(stamps) <= self.sync_window:
                self.frame = tuple(self.pending_raw[key][0] for key in THRUSTERS)
                self.frame_stamps = stamps
                self.pending_raw.clear()
            else:
                oldest = min(stamps)
                self.pending_raw = {key: item for key, item in self.pending_raw.items() if item[1] > oldest}

    @staticmethod
    def fresh(stamp, now, timeout):
        return stamp is not None and math.isfinite(now) and 0. <= now - stamp <= timeout

    def base_ready(self, now):
        if not math.isfinite(now) or now <= 0.:
            return False
        if self.status not in ("navigating", "aligning"):
            return False
        # A fresh zero target is valid feedback control: the existing PID
        # must be able to brake while navigation is active and settling.
        return all(self.fresh(self.samples.get(key), now, self.timeouts[key])
                   for key in ("command", "odometry", "scan", "gps", "imu", "status"))

    def tick(self, now):
        if self.last_now is not None and now < self.last_now:
            self.samples.clear()
            self.pending_raw.clear()
            self.frame = self.frame_stamps = None
            self.close()
        self.last_now = now
        if not self.base_ready(now):
            self.close()
            return False
        if self.reset_required:
            return False
        raw_ready = (self.frame is not None and self.frame_stamps is not None and
                     all(self.fresh(stamp, now, self.timeouts["raw"]) and stamp > self.reset_ack
                         for stamp in self.frame_stamps))
        if not raw_ready:
            if self.active:
                self.close()
            self.active = False
            return False
        self.active = True
        return True

    def begin_reset(self, now):
        if not self.reset_required or self.reset_pending is not None or not self.base_ready(now):
            return None
        self.generation += 1
        self.reset_pending = self.generation
        return self.reset_pending

    def finish_reset(self, token, success, now):
        if token != self.reset_pending:
            return False
        self.reset_pending = None
        if not success or not self.base_ready(now):
            self.close()
            return False
        self.reset_required = False
        self.reset_ack = now
        self.frame = self.frame_stamps = None
        self.pending_raw.clear()
        return True


class ThrusterOutputAdapter:
    def __init__(self):
        import rospy
        from dynamic_reconfigure.msg import Config, DoubleParameter
        from dynamic_reconfigure.srv import Reconfigure
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan, NavSatFix
        from std_msgs.msg import Float32, String

        self.rospy = rospy
        self.Float32 = Float32
        self.Config, self.DoubleParameter, self.Reconfigure = Config, DoubleParameter, Reconfigure
        self.lock = threading.RLock()
        timeouts = {key: float(rospy.get_param("~" + key + "_timeout", value))
                    for key, value in OutputGuard.DEFAULT_TIMEOUTS.items()}
        self.guard = OutputGuard(timeouts, float(rospy.get_param("~raw_sync_window", .1)))
        self.allocator_scale = float(rospy.get_param("~allocator_scale", 800.))
        self.max_forward = float(rospy.get_param("~max_forward_force", 250.))
        self.max_reverse = float(rospy.get_param("~max_reverse_force", -100.))
        # /clock and sensor messages use separate connections, so callbacks
        # can observe a sensor one physics step before its clock update.
        self.future_tolerance = float(rospy.get_param("~future_tolerance", .05))
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.:
            raise ValueError("future_tolerance must be nonnegative and finite")
        convert_raw_frame((0.,) * 4, self.allocator_scale, self.max_forward, self.max_reverse)
        rate = float(rospy.get_param("~rate", 20.))
        self.reset_retry = float(rospy.get_param("~reset_retry", 1.))
        if not math.isfinite(rate) or rate <= 0. or not math.isfinite(self.reset_retry) or self.reset_retry <= 0.:
            raise ValueError("rate and reset_retry must be positive and finite")
        self.reset_node = rospy.get_param("~velocity_controller", "/wamv/velocity_control").rstrip("/")
        self.reset_worker = None
        self.last_reset_attempt = -math.inf
        raw_prefix = rospy.get_param("~raw_prefix", "/wamv/navigation/raw_thrusters").rstrip("/")
        output_prefix = rospy.get_param("~output_prefix", "/wamv/thrusters").rstrip("/")
        self.publishers = [rospy.Publisher(output_prefix + "/" + name + "_thrust_cmd", Float32, queue_size=1)
                           for name in THRUSTERS]
        self.angle_publishers = []
        if rospy.get_param("~publish_zero_angles", True):
            self.angle_publishers = [rospy.Publisher(output_prefix + "/" + name + "_thrust_angle", Float32, queue_size=1)
                                     for name in THRUSTERS]
        self.subscribers = [rospy.Subscriber(raw_prefix + "/" + name + "_thrust_cmd", Float32,
                                             self.raw_callback, callback_args=name, queue_size=1)
                            for name in THRUSTERS]
        for parameter, default, message_type, callback in (
                ("command_topic", "/wamv/cmd_vel", Twist, self.command_callback),
                ("status_topic", "/wamv/navigation/status", String, self.status_callback),
                ("odometry_topic", "/wamv/robot_localization/odometry/filtered", Odometry, self.odometry_callback),
                ("scan_topic", "/scan", LaserScan, self.scan_callback),
                ("gps_topic", "/wamv/sensors/gps/gps/fix", NavSatFix, self.gps_callback),
                ("imu_topic", "/wamv/sensors/imu/imu/data", Imu, self.imu_callback)):
            self.subscribers.append(rospy.Subscriber(rospy.get_param("~" + parameter, default), message_type,
                                                     callback, queue_size=1))
        rospy.on_shutdown(self.stop)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.timer_callback, reset=True)

    def now(self):
        return self.rospy.Time.now().to_sec()

    def sensor_stamp(self, message, now):
        stamp = message.header.stamp.to_sec()
        if (math.isfinite(now) and math.isfinite(stamp) and
                0. < stamp <= now + self.future_tolerance):
            return min(stamp, now)
        return math.nan

    @staticmethod
    def vector(vector):
        return (vector.x, vector.y, vector.z)

    @staticmethod
    def quaternion_valid(quaternion):
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        return all(math.isfinite(v) for v in values) and .5 <= sum(v * v for v in values) <= 1.5

    def command_callback(self, message):
        with self.lock:
            self.guard.set_command(self.vector(message.linear) + self.vector(message.angular), self.now())

    def status_callback(self, message):
        with self.lock:
            self.guard.set_status(message.data, self.now())

    def raw_callback(self, message, name):
        with self.lock:
            self.guard.set_raw(name, message.data, self.now())

    def odometry_callback(self, message):
        values = self.vector(message.pose.pose.position) + self.vector(message.twist.twist.linear) + self.vector(message.twist.twist.angular)
        valid = all(math.isfinite(v) for v in values) and self.quaternion_valid(message.pose.pose.orientation)
        with self.lock:
            self.guard.observe("odometry", self.sensor_stamp(message, self.now()), valid)

    def scan_callback(self, message):
        metadata = (message.angle_min, message.angle_max, message.angle_increment, message.range_min, message.range_max)
        valid = (all(math.isfinite(v) for v in metadata) and message.angle_increment > 0. and
                 message.range_min >= 0. and message.range_max > message.range_min and
                 bool(message.ranges) and all(not math.isnan(v) and v >= 0. for v in message.ranges))
        with self.lock:
            self.guard.observe("scan", self.sensor_stamp(message, self.now()), valid)

    def gps_callback(self, message):
        valid = (message.status.status >= 0 and all(math.isfinite(v) for v in (message.latitude, message.longitude, message.altitude))
                 and -90. <= message.latitude <= 90. and -180. <= message.longitude <= 180.)
        with self.lock:
            self.guard.observe("gps", self.sensor_stamp(message, self.now()), valid)

    def imu_callback(self, message):
        valid = (self.quaternion_valid(message.orientation) and message.orientation_covariance[0] != -1. and
                 all(math.isfinite(v) for v in self.vector(message.angular_velocity) + self.vector(message.linear_acceleration)))
        with self.lock:
            self.guard.observe("imu", self.sensor_stamp(message, self.now()), valid)

    def reset_pid(self, token):
        success = False
        try:
            # Rewriting the current eight PID parameters invokes the existing
            # config_callback, constructing new PIDRegulators with zero history.
            values = {key: float(self.rospy.get_param(self.reset_node + "/" + key)) for key in PID_KEYS}
            if not all(math.isfinite(v) and v >= 0. for v in values.values()):
                raise ValueError("invalid velocity controller configuration")
            config = self.Config()
            config.doubles = [self.DoubleParameter(name=key, value=value) for key, value in values.items()]
            service_name = self.reset_node + "/set_parameters"
            self.rospy.wait_for_service(service_name, timeout=1.)
            response = self.rospy.ServiceProxy(service_name, self.Reconfigure)(config=config)
            returned = {parameter.name: parameter.value for parameter in response.config.doubles}
            success = all(key in returned and math.isfinite(returned[key]) and abs(returned[key] - value) <= 1e-9
                          for key, value in values.items())
            if not success:
                raise ValueError("PID reset response did not acknowledge the current configuration")
        except Exception as error:
            self.rospy.logwarn_throttle(5., "Thruster gate waiting for velocity PID reset: %s", str(error))
        finally:
            with self.lock:
                self.guard.finish_reset(token, success, self.now())

    def publish(self, commands):
        for publisher, command in zip(self.publishers, commands):
            publisher.publish(self.Float32(data=command))
        for publisher in self.angle_publishers:
            publisher.publish(self.Float32(data=0.))

    def timer_callback(self, _event):
        with self.lock:
            now = self.now()
            enabled = self.guard.tick(now)
            commands = (0.,) * 4
            if enabled:
                try:
                    commands = convert_raw_frame(self.guard.frame, self.allocator_scale, self.max_forward, self.max_reverse)
                except (ValueError, OverflowError):
                    self.guard.close()
            if (self.guard.reset_required and self.guard.base_ready(now) and
                    (self.reset_worker is None or not self.reset_worker.is_alive()) and
                    (now < self.last_reset_attempt or now - self.last_reset_attempt >= self.reset_retry)):
                token = self.guard.begin_reset(now)
                if token is not None:
                    self.last_reset_attempt = now
                    self.reset_worker = threading.Thread(target=self.reset_pid, args=(token,), daemon=True)
                    self.reset_worker.start()
            self.publish(commands)

    def stop(self):
        with self.lock:
            self.guard.close()
            self.publish((0.,) * 4)


def main():
    import rospy
    rospy.init_node("thruster_output_adapter")
    try:
        ThrusterOutputAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
