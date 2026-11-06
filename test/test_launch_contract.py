#!/usr/bin/env python3
"""Resolve the real launch graph without starting a ROS master or simulator."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import rospkg
import roslaunch
import yaml


class LaunchContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = Path(rospkg.RosPack().get_path("std_usv"))
        cls.simulation = Path(rospkg.RosPack().get_path("ocean_simulation_gazebo"))

    def load(self, name, arguments=None):
        # XML loading resolves includes/xacro/params without machine assignment,
        # which otherwise enumerates network interfaces even for a static test.
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(self.package / "launch" / name),
                                           config, argv=arguments or [], verbose=False)
        return config

    def test_default_demo_has_one_ship_no_coverage_or_aircraft(self):
        graph = self.load("demo_astar_dwa.launch", ["gui:=false", "rviz:=false"])
        ships = [n for n in graph.nodes if n.package == "gazebo_ros" and n.type == "spawn_model"]
        self.assertEqual(len(ships), 1)
        self.assertIn("-model wamv", ships[0].args)
        forbidden = {"binn_ccpp_node", "px4", "mavros_node", "twist2thrust.py"}
        self.assertFalse(forbidden.intersection(n.type for n in graph.nodes))
        self.assertEqual(graph.params["/wamv/navigation/goal_relay/goal_x"].value, -500)
        self.assertEqual(graph.params["/wamv/navigation/goal_relay/goal_y"].value, 232)
        self.assertEqual(graph.params["/wamv/navigation/coordinator/HZ"].value, 10)
        relay = next(n for n in graph.nodes if n.type == "navigation_goal_relay")
        coordinator = next(n for n in graph.nodes if n.type == "navigation_coordinator_node")
        self.assertEqual((relay.namespace, relay.name), ("/wamv/navigation/", "goal_relay"))
        self.assertEqual((coordinator.namespace, coordinator.name), ("/wamv/navigation/", "coordinator"))
        for key, value in {"PREFERRED_VELOCITY": .8, "GOAL_BEARING_COST_GAIN": 0.,
                           "GOAL_YAW_TOLERANCE": .15}.items():
            self.assertAlmostEqual(graph.params["/wamv/navigation/coordinator/" + key].value, value)

    def test_navigation_only_does_not_spawn_or_launch_gazebo(self):
        graph = self.load("astar_dwa_mix.launch", ["rviz:=false"])
        self.assertFalse(any(n.package == "gazebo_ros" for n in graph.nodes))
        self.assertEqual(sum(n.type == "VelocityControl.py" for n in graph.nodes), 1)
        self.assertEqual(sum(n.type == "thrust_allocator.py" for n in graph.nodes), 1)
        self.assertFalse(any(n.type == "navigation_dynamic_buoys.py" for n in graph.nodes))

    def test_all_maps_custom_goal_paused_and_dynamic_parameters(self):
        for world in ["map1-1", "map1-2", "map1-3", "map2-1", "map2-2"]:
            world_path = self.simulation / "worlds/usv_path_planning" / (world + ".world")
            graph = self.load("demo_astar_dwa.launch", ["world:=" + str(world_path),
                "gui:=false", "rviz:=false", "paused:=true", "usv_x:=-575",
                "goal_x:=-505", "goal_yaw:=0.4", "dynamic_obstacles:=true"])
            server = next(n for n in graph.nodes if n.package == "gazebo_ros" and n.type == "gzserver")
            spawn = next(n for n in graph.nodes if n.type == "spawn_model")
            self.assertIn("-u", server.args.split())
            self.assertNotIn("-unpause", spawn.args)
            self.assertIn("-x -575", spawn.args)
            self.assertEqual(graph.params["/wamv/navigation/goal_relay/goal_x"].value, -505)
            self.assertAlmostEqual(graph.params["/wamv/navigation/goal_relay/goal_yaw"].value, .4)
            self.assertEqual(graph.params["/navigation_dynamic_buoys/world"].value, str(world_path))
            self.assertEqual(graph.params["/wamv/robot_localization/ekf/initial_state"].value[0], -575)
            robot_names = [m.get("name") for m in ET.parse(str(world_path)).iter("model")]
            self.assertNotIn("wamv", robot_names)

    def test_control_is_routed_through_adapter_and_goal_tool(self):
        graph = self.load("astar_dwa_mix.launch", ["rviz:=false", "use_initial_goal:=false"])
        allocator = next(n for n in graph.nodes if n.type == "thrust_allocator.py")
        self.assertEqual(len(allocator.remap_args), 4)
        self.assertTrue(all("/navigation/raw_thrusters/" in target for _, target in allocator.remap_args))
        self.assertFalse(graph.params["/wamv/navigation/goal_relay/use_initial_goal"].value)
        rviz = yaml.safe_load((self.package / "config/wamv_rviz.rviz").read_text())
        tool = next(t for t in rviz["Visualization Manager"]["Tools"] if t["Class"] == "rviz/SetGoal")
        self.assertEqual(tool["Topic"], "/wamv/navigation/goal")
        displays = {display["Name"]: display for display in rviz["Visualization Manager"]["Displays"]}
        self.assertEqual(displays["Current goal"]["Topic"], "/wamv/navigation/goal_request")
        self.assertEqual(displays["DWA candidates"]["Marker Topic"],
                         "/wamv/navigation/coordinator/candidate_trajectories")
        self.assertEqual(displays["DWA selected"]["Marker Topic"],
                         "/wamv/navigation/coordinator/selected_trajectory")

    def test_custom_semantic_parameters_reach_coordinator(self):
        parameters = yaml.safe_load((self.package / "config/dwa_param.yaml").read_text())
        overrides = {"PREFERRED_VELOCITY": .65, "GOAL_BEARING_COST_GAIN": 2.5,
                     "GOAL_YAW_TOLERANCE": .12, "MAX_VELOCITY": .9,
                     "MAX_ACCELERATION": .4, "FOOTPRINT_MARGIN": .35, "INPUT_TIMEOUT": .6}
        parameters.update(overrides)
        with tempfile.TemporaryDirectory(prefix="std-usv-rename-launch-") as directory:
            path = Path(directory) / "dwa.yaml"
            path.write_text(yaml.safe_dump(parameters))
            graph = self.load("astar_dwa_mix.launch", ["rviz:=false", "dwa_param:=" + str(path)])
        for key, value in overrides.items():
            self.assertAlmostEqual(graph.params["/wamv/navigation/coordinator/" + key].value, value)

    def test_combined_model_preserves_navigation_and_controller_namespaces(self):
        settings = yaml.safe_load((self.package / "config/wamv_model.yaml").read_text())
        model, pid = settings["navigation"]["model"], settings["pid"]
        self.assertEqual(pid["mass"], model["mass"])
        self.assertEqual(pid["inertial"], model["inertial"])
        graph = self.load("astar_dwa_mix.launch", ["rviz:=false"])
        for key in ("frame_id", "footprint", "mass", "inertia_reference", "center_of_mass"):
            self.assertEqual(graph.params["/wamv/navigation/model/" + key].value, model[key])
        self.assertEqual(graph.params["/wamv/pid/mass"].value, model["mass"])
        for key, value in model["inertial"].items():
            self.assertEqual(graph.params["/wamv/pid/inertial/" + key].value, value)
        root = ET.parse(str(self.package / "launch/astar_dwa_mix.launch")).getroot()
        loads = [element for element in root.iter("rosparam")
                 if element.get("file", "").endswith("/config/wamv_model.yaml")]
        self.assertEqual(len(loads), 1)
        self.assertEqual(loads[0].get("ns"), "/wamv")

    def test_retired_sources_configs_and_launch_fragments_are_absent(self):
        retired = ("src/binn_ccpp.cpp", "include/path_planning/binn_ccpp.h",
                   "include/path_planning/utmll.h", "scripts/dynamic_buoy_1_controller.py",
                   "scripts/dynamic_buoy_2_controller.py", "scripts/dynamic_buoy_3_controller.py",
                   "scripts/dynamic_obstacle_controller.py", "scripts/map.py", "config/los_param.yaml",
                   "config/robot_param.yaml", "config/wamv_inertial.yaml",
                   "launch/core/navigation_sensors.launch", "launch/core/navigation_control.launch")
        for relative in retired:
            self.assertFalse((self.package / relative).exists(), relative)
        root = ET.parse(str(self.package / "launch/astar_dwa_mix.launch")).getroot()
        self.assertNotIn("robot_param", [element.get("name") for element in root.findall("arg")])
        self.assertFalse(root.findall("include"))


if __name__ == "__main__":
    unittest.main()
