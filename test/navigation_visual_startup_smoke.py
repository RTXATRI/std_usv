#!/usr/bin/env python3
"""Manual Gazebo transport startup regression; requires working OpenGL.

Source-only: not installed or included in the ordinary unit-test target.
After building/sourcing the workspace, run this file with Python 3. It owns
private ROS/Gazebo process groups, first checks shutdown without Gazebo, then
starts a paused Gazebo after a deliberate test delay and checks that both debug
visuals actually receive the server-side plugin. It never unpauses the world.
The wall-time limit applies to test work; bounded cleanup can take another 15 s.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

import roslib.packages


ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
WORLD = """<sdf version="1.6"><world name="robotx_example_course">
  <model name="robotx_dock_probe"><static>true</static><link name="bay_visualization">
    <visual name="bay_box"><geometry><box><size>1.5 4 2</size></box></geometry></visual>
    <visual name="bay_box_exterior"><pose>0 6 0 0 0 0</pose>
      <geometry><box><size>8 1.5 2</size></box></geometry></visual>
  </link></model>
  <model name="observer"><static>true</static><link name="camera">
    <pose>0 0 10 0 1.5708 0</pose><sensor name="test_camera" type="camera">
      <always_on>true</always_on><update_rate>1</update_rate><camera>
        <horizontal_fov>1</horizontal_fov><image><width>8</width><height>8</height>
          <format>R8G8B8</format></image><clip><near>.1</near><far>30</far></clip>
      </camera></sensor></link></model>
</world></sdf>
"""


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.
        return connection


class StartupCheck:
    def __init__(self, args):
        self.args = args
        self.deadline = time.monotonic() + args.max_wall
        if args.output:
            self.folder = Path(args.output).expanduser().resolve()
            if self.folder.exists() and any(self.folder.iterdir()):
                raise ValueError("Use an empty output directory; existing files will not be overwritten")
            self.folder.mkdir(parents=True, exist_ok=True)
        else:
            self.folder = Path(tempfile.mkdtemp(prefix="std-usv-visual-startup-"))
        binaries = roslib.packages.find_node("std_usv", "navigation_visual_filter_node")
        if not binaries:
            raise ValueError("Build/source std_usv before running this check")
        self.binary = Path(binaries[0]).resolve()
        plugin_directory = self.binary.parent.parent
        self.plugin = plugin_directory / "libstd_usv_navigation_sensor_visibility.so"
        if not self.plugin.is_file():
            raise ValueError("Visibility plugin is missing: " + str(self.plugin))
        self.roscore = shutil.which("roscore")
        self.gzserver = shutil.which("gzserver")
        if not self.roscore or not self.gzserver:
            raise ValueError("Source the ROS/Gazebo environment first")
        self.environment = dict(os.environ)
        self.environment.update(ROS_MASTER_URI="http://127.0.0.1:{}".format(args.ros_port),
                                GAZEBO_MASTER_URI="http://127.0.0.1:{}".format(args.gazebo_port),
                                ROS_HOSTNAME="localhost", ROS_IP="127.0.0.1",
                                ROS_HOME=str(self.folder / "ros_home"),
                                ROS_LOG_DIR=str(self.folder / "ros_log"))
        for key in ("ROS_HOME", "ROS_LOG_DIR"):
            Path(self.environment[key]).mkdir()
        self.environment["GAZEBO_PLUGIN_PATH"] = str(plugin_directory) + ":" + self.environment.get("GAZEBO_PLUGIN_PATH", "")
        self.world = self.folder / "probe.world"
        self.world.write_text(WORLD)
        self.processes = []
        self.handles = []
        self.result = {"output": str(self.folder), "binary": str(self.binary),
                       "ros_master_uri": self.environment["ROS_MASTER_URI"],
                       "gazebo_master_uri": self.environment["GAZEBO_MASTER_URI"],
                       "max_wall_seconds": args.max_wall, "errors": []}

    def read_log(self, name):
        path = self.folder / name
        return ANSI.sub("", path.read_text(errors="replace")) if path.exists() else ""

    def wait_until(self, predicate, seconds, message):
        deadline = min(self.deadline, time.monotonic() + seconds)
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.1)
        raise RuntimeError(message)

    def launch(self, command, log):
        handle = (self.folder / log).open("w")
        self.handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   env=self.environment, start_new_session=True)
        self.processes.append(process)
        return process

    @staticmethod
    def stop(process, timeout=5):
        # Popen.poll reaps our exact child; a recycled PID is never signalled.
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
            raise RuntimeError("Owned process required SIGKILL: " + str(process.args))

    def check_ports(self):
        for port in (self.args.ros_port, self.args.gazebo_port):
            with socket.socket() as probe:
                probe.settimeout(.25)
                if probe.connect_ex(("127.0.0.1", port)) == 0:
                    raise RuntimeError("Refusing an occupied private port: {}".format(port))

    def check(self):
        self.check_ports()
        self.launch([self.roscore, "-p", str(self.args.ros_port)], "roscore.log")
        master = xmlrpc.client.ServerProxy(self.environment["ROS_MASTER_URI"],
                                          transport=TimeoutTransport())

        def master_ready():
            try:
                return master.getPid("/visual_startup_check")[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False

        self.wait_until(master_ready, 15, "Private ROS master did not start")
        for name in ("visual_filter_no_gazebo", "visual_filter_delayed_gazebo"):
            # Explicit fixture allow-list; no production map metadata is changed.
            master.setParam("/visual_startup_check", "/" + name + "/known_docks", ["robotx_dock_probe"])
        no_gazebo = self.launch([str(self.binary), "_world:=" + str(self.world),
                                "__name:=visual_filter_no_gazebo"], "no_gazebo.log")
        self.wait_until(lambda: "waiting for Gazebo master" in self.read_log("no_gazebo.log"),
                        8, "Filter never reported the missing master")
        before = time.monotonic()
        self.stop(no_gazebo, timeout=2)
        self.result.update(no_gazebo_shutdown_seconds=time.monotonic() - before,
                           no_gazebo_exit_code=no_gazebo.returncode)
        if no_gazebo.returncode != 0:
            raise RuntimeError("No-master filter did not exit normally")

        delayed = self.launch([str(self.binary), "_world:=" + str(self.world),
                               "__name:=visual_filter_delayed_gazebo"], "delayed_gazebo.log")
        self.wait_until(lambda: "waiting for Gazebo master" in self.read_log("delayed_gazebo.log"),
                        8, "Delayed filter never entered master wait")
        start_gazebo_after = time.monotonic() + self.args.delay_seconds
        self.wait_until(lambda: time.monotonic() >= start_gazebo_after,
                        self.args.delay_seconds + 1, "Wall limit expired before the delayed startup")
        if delayed.poll() is not None:
            raise RuntimeError("Filter exited before delayed Gazebo startup")
        self.launch([self.gzserver, "-u", "--verbose", "-s", "libgazebo_ros_api_plugin.so",
                     str(self.world)], "gazebo.log")
        self.wait_until(lambda: "world response publisher is ready" in self.read_log("delayed_gazebo.log"),
                        30, "Filter did not discover the delayed Gazebo world")
        self.wait_until(lambda: self.read_log("delayed_gazebo.log").count("Requested GUI-only") == 2,
                        20, "Filter did not request both debug visual attachments")
        self.result["requested_visual_count"] = 2
        self.result["late_master_connected"] = True

        os.environ.update({key: self.environment[key] for key in
                           ("ROS_MASTER_URI", "ROS_HOSTNAME", "ROS_IP", "ROS_HOME", "ROS_LOG_DIR")})
        import rospy
        from gazebo_msgs.srv import GetPhysicsProperties
        rospy.init_node("visual_startup_check", anonymous=True, disable_signals=True)
        rospy.wait_for_service("/gazebo/get_physics_properties", timeout=min(10., max(.1, self.deadline - time.monotonic())))
        physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)()
        self.result["paused_after_delayed_startup"] = physics.pause
        if not physics.pause:
            raise RuntimeError("Filter unexpectedly unpaused Gazebo")
        before = time.monotonic()
        self.stop(delayed, timeout=3)
        self.result.update(connected_shutdown_seconds=time.monotonic() - before,
                           connected_exit_code=delayed.returncode)
        if delayed.returncode != 0:
            raise RuntimeError("Connected filter did not exit normally")

    def run(self):
        try:
            self.check()
        except (Exception, KeyboardInterrupt) as error:
            self.result["errors"].append(str(error) or "Interrupted")
        finally:
            for process in reversed(self.processes):
                try:
                    self.stop(process)
                except Exception as error:
                    self.result["errors"].append(str(error))
            for handle in self.handles:
                handle.close()
        # Gazebo's colourized C++ stream may flush only during shutdown. Verify
        # actual server-side application after cleanup, not just ROS requests.
        server_log = self.read_log("gazebo.log")
        applied = [line for line in server_log.splitlines()
                   if "NavigationSensorVisibility applied" in line and "(server=true)" in line]
        self.result["server_applied_visual_count"] = len(applied)
        for leaf in ("bay_box", "bay_box_exterior"):
            expected = "robotx_dock_probe::bay_visualization::" + leaf + " (server=true)"
            if not any(expected in line for line in applied):
                self.result["errors"].append("No server-side application evidence for " + leaf)
        self.result["pass"] = not self.result["errors"]
        (self.folder / "result.json").write_text(json.dumps(self.result, indent=2))
        print(json.dumps(self.result, indent=2), flush=True)
        return 0 if self.result["pass"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-port", type=int, default=11611)
    parser.add_argument("--gazebo-port", type=int, default=11612)
    parser.add_argument("--max-wall", type=float, default=90.)
    parser.add_argument("--delay-seconds", type=float, default=10.)
    parser.add_argument("--output", help="Empty output directory; default: a unique directory under /tmp")
    args = parser.parse_args()
    if not all(1024 <= port <= 65535 for port in (args.ros_port, args.gazebo_port)) or args.ros_port == args.gazebo_port:
        parser.error("Choose two distinct private ports between 1024 and 65535")
    if args.delay_seconds < 0 or args.max_wall < args.delay_seconds + 20:
        parser.error("max-wall must allow at least delay-seconds + 20 seconds")
    try:
        return StartupCheck(args).run()
    except Exception as error:
        parser.exit(1, str(error) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
