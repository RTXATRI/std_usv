#!/usr/bin/env python3
"""Original buoy path geometry and deterministic simulated-time movement."""

import importlib.util
import math
import pathlib
import types
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "navigation_dynamic_buoys.py"
SPEC = importlib.util.spec_from_file_location("navigation_dynamic_buoys", str(SCRIPT))
buoys = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(buoys)


class DynamicBuoyTests(unittest.TestCase):
    def test_world_selection(self):
        self.assertEqual(set(buoys.routes_for_world("/worlds/map1-3.world")),
                         {"dynamic_obstacle_1", "dynamic_obstacle_2"})
        self.assertEqual(set(buoys.routes_for_world("map2-1")), {"dynamic_obstacle_3"})
        for world in ("", "map1-1.world", "map1-2.world", "map2-2.world", "map1.world"):
            self.assertEqual(buoys.routes_for_world(world), {})

    def test_original_route_endpoints(self):
        self.assertEqual(buoys.ROUTES["dynamic_obstacle_1"], ((-529., 215.5, .3), (-562., 215.5, .3)))
        self.assertEqual(buoys.ROUTES["dynamic_obstacle_2"],
                         ((-547.5, 234.5, .3), (-547.5, 246.5, .3), (-516., 246.5, .3)))
        self.assertEqual(buoys.ROUTES["dynamic_obstacle_3"], ((-498.5, 196., .3), (-498.5, 234.5, .3)))

    def test_reflected_motion_passes_corners_in_both_directions(self):
        route = buoys.ROUTES["dynamic_obstacle_2"]
        length = buoys.route_length(route)
        self.assertAlmostEqual(length, 43.5)
        self.assertEqual(buoys.route_position(route, 0.), route[0])
        self.assertEqual(buoys.route_position(route, 12.), route[1])
        self.assertEqual(buoys.route_position(route, length), route[2])
        self.assertEqual(buoys.route_position(route, 2. * length - 12.), route[1])
        self.assertEqual(buoys.route_position(route, 2. * length), route[0])
        self.assertEqual(buoys.route_position(route, 20.), (-539.5, 246.5, .3))
        self.assertEqual(buoys.route_position(route, 2. * length - 20.), (-539.5, 246.5, .3))

    def test_pause_speed_and_large_time_step(self):
        route = buoys.ROUTES["dynamic_obstacle_1"]
        self.assertEqual(buoys.route_position(route, .7 * 10.), (-536., 215.5, .3))
        self.assertEqual(buoys.route_position(route, 66. * 1000. + 5.), (-534., 215.5, .3))
        for distance in range(200):
            x, y, z = buoys.route_position(route, distance)
            self.assertTrue(-562. <= x <= -529.)
            self.assertEqual((y, z), (215.5, .3))

    def test_invalid_geometry_and_distance(self):
        for route in ((), ((0., 0., 0.),), ((0., 0., 0.), (0., 0., 0.)), ((math.nan, 0., 0.), (1., 0., 0.))):
            with self.assertRaises(ValueError):
                buoys.route_length(route)
        for distance in (-1., math.nan, math.inf):
            with self.assertRaises(ValueError):
                buoys.route_position(buoys.ROUTES["dynamic_obstacle_1"], distance)

    def node(self):
        node = buoys.NavigationDynamicBuoys.__new__(buoys.NavigationDynamicBuoys)
        node.clock = 10.
        node.calls = []
        node.rospy = types.SimpleNamespace(
            Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: node.clock)),
            ServiceException=RuntimeError, logwarn_throttle=lambda *args: None)
        node.routes = buoys.routes_for_world("map1-3")
        node.speed = .7
        node.present_models = set(node.routes)
        node.ready = False
        node.last_time = None
        node.distance = 0.
        node.get_state = lambda *args: types.SimpleNamespace(success=True, status_message="")
        node.set_state = lambda state: node.calls.append(state) or types.SimpleNamespace(success=True, status_message="")
        node.ModelState = lambda: types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(), orientation=types.SimpleNamespace()))
        return node

    def test_model_wait_pause_and_simulation_clock_motion(self):
        node = self.node()
        node.present_models = set()
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.calls, [])
        node.present_models = set(node.routes)
        node.update(None)
        self.assertTrue(node.ready)
        node.update(None)  # Paused ROS clock produces no service update.
        self.assertEqual(node.calls, [])
        node.clock = 11.
        node.update(None)
        self.assertEqual(len(node.calls), 2)
        self.assertAlmostEqual(node.calls[0].pose.position.x, -529.7)
        self.assertAlmostEqual(node.distance, .7)

    def test_get_and_set_service_response_failures_preserve_wait_state(self):
        node = self.node()
        node.get_state = lambda *args: types.SimpleNamespace(success=False, status_message="not spawned")
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.calls, [])
        node.get_state = lambda *args: types.SimpleNamespace(success=True, status_message="")
        node.update(None)
        node.clock = 11.
        node.set_state = lambda state: types.SimpleNamespace(success=False, status_message="missing model")
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.distance, 0.)


if __name__ == "__main__":
    unittest.main()
