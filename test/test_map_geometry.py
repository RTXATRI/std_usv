#!/usr/bin/env python3
"""Source-based geometry and map asset regression tests; no ROS master needed."""

from collections import deque
import hashlib
import math
from pathlib import Path
import runpy
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import yaml

PACKAGE = Path(rospkg.RosPack().get_path("std_usv"))
sys.path.insert(0, str(PACKAGE / "scripts"))
from generate_navigation_maps import (Sources, ModelFrames, ORIGIN, RESOLUTION,
                                      boat_calibration, clip_above, generate,
                                      known_includes, pose_matrix, raster_polygon,
                                      world_grid)
from navigation_map_publisher import map_for_world, read_map, validate_world_assets


def index_xy(point):
    return tuple(np.floor((np.asarray(point) - ORIGIN) / RESOLUTION).astype(int))


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sources = Sources()

    def test_sdf_joint_frames_and_rotations(self):
        model = ET.fromstring('''<model name="test">
          <link name="base"/>
          <joint name="mount"><parent>base</parent><child>sensor</child>
            <pose relative_to="base">1 2 3 0 0 1.5707963267948966</pose></joint>
          <link name="sensor"><pose relative_to="mount">1 0 0 0 0 0</pose></link>
        </model>''')
        frames = ModelFrames(model)
        np.testing.assert_allclose(frames.resolve("sensor")[:3, 3], [1., 3., 3.], atol=1e-12)
        np.testing.assert_allclose(frames.resolve("sensor")[:3, 0], [0., 1., 0.], atol=1e-12)
        with self.assertRaisesRegex(ValueError, "Unresolved"):
            frames.resolve("not_a_frame")

    def test_current_sensor_offsets_are_not_double_applied(self):
        model = self.sources.xml(self.sources.packages["ocean_simulation_gazebo"] /
                                 "models/vehicle/wamv/model.sdf").find("model")
        frames = ModelFrames(model)
        np.testing.assert_allclose(frames.resolve("wamv/gps_wamv_link")[:3, 3], [-.85, 0, 1.3])
        np.testing.assert_allclose(frames.resolve("wamv/imu_wamv_link")[:3, 3], [.3, -.2, 1.3])
        np.testing.assert_allclose(frames.resolve("wamv/left_rear_engine_link")[:3, 3],
                                   [-2.37378, 1.02713, .318237])

    def test_thin_collision_marks_intersected_cells(self):
        # A post entirely to one side of the cell centre still blocks that cell.
        grid = np.zeros((3, 3), dtype=bool)
        raster_polygon(grid, [[.01, .01], [.02, .01], [.02, .02], [.01, .02]],
                       origin=(0, 0), resolution=1.)
        self.assertTrue(grid[0, 0])
        self.assertEqual(int(grid.sum()), 1)
        rotated = np.zeros((10, 10), dtype=bool)
        raster_polygon(rotated, [[2, 4], [4, 2], [6, 4], [4, 6]], origin=(0, 0), resolution=1.)
        self.assertTrue(rotated[4, 4])
        self.assertFalse(rotated[0, 0])

    def test_shore_below_water_is_clipped_before_projection(self):
        triangle = np.array([[0., 0., -1.], [2., 0., -1.], [0., 2., 1.]])
        clipped = clip_above(triangle, 0.)
        self.assertGreaterEqual(len(clipped), 3)
        self.assertTrue((clipped[:, 2] >= 0).all())
        self.assertTrue((clipped[:, 1] >= 1.).all())
        self.assertEqual(len(clip_above(triangle - [0, 0, 5], 0.)), 0)

    def test_aggregate_inertia_includes_engines_and_parallel_axes(self):
        calibration, controller = boat_calibration(self.sources)
        self.assertAlmostEqual(calibration["mass"], 314.285, places=6)
        self.assertGreater(calibration["inertial"]["izz"], 800.)
        self.assertLess(calibration["inertial"]["izz"], 810.)
        self.assertGreater(calibration["inertial"]["izz"],
                           calibration["inertial_at_center_of_mass"]["izz"])
        self.assertEqual(controller["pid"]["inertial"], calibration["inertial"])
        footprint = np.asarray(calibration["footprint"])
        self.assertLess(footprint[:, 0].min(), -2.85)
        self.assertGreater(footprint[:, 0].max(), 2.54)
        self.assertAlmostEqual(float(abs(footprint[:, 1]).max()), 1.23, places=5)
        # Hull orientation must suit the planner's oriented-footprint checks.
        area = np.sum(footprint[:, 0] * np.roll(footprint[:, 1], -1) -
                      footprint[:, 1] * np.roll(footprint[:, 0], -1)) / 2.
        self.assertGreater(area, 0.)


class MapAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sources = Sources()
        cls.worlds = cls.sources.packages["ocean_simulation_gazebo"] / "worlds/usv_path_planning"

    def test_unknown_obstacles_are_excluded_and_known_families_match(self):
        expected = {"map1-1": 0, "map1-2": 6, "map1-3": 5, "map2-1": 20, "map2-2": 0}
        signatures = {}
        for stem, count in expected.items():
            world = self.sources.xml(self.worlds / (stem + ".world")).find("world")
            known, excluded = known_includes(world)
            self.assertEqual(len(excluded), count, stem)
            self.assertTrue(all(not row["name"].startswith(("custom_dock_block_", "dynamic_obstacle_"))
                                for row in known))
            family = stem[:4]
            if family in signatures:
                self.assertEqual(known, signatures[family], stem)
            signatures[family] = known
        self.assertEqual(len(signatures["map2"]) - len(signatures["map1"]), 3)
        # Existing course docks are never removed while classifying unknowns.
        self.assertEqual(sum(row["name"].startswith("robotx_dock_")
                             for row in signatures["map1"]), 12)

    def test_unknown_platform_not_written_to_static_map(self):
        grid, _, _, _ = world_grid(self.worlds / "map1-2.world", self.sources)
        x, y = index_xy((-556.5, 185.))
        self.assertFalse(grid[y, x])
        x, y = index_xy((-568.305, 196.321))
        self.assertTrue(grid[y, x])
        x, y = index_xy((-570, 160))
        self.assertTrue(grid[y, x])  # shoreline, not a dock or unknown platform

    def test_generated_assets_match_sources_and_have_portable_provenance(self):
        outputs = generate()
        self.assertEqual(len(outputs), 6)
        for path, content in outputs.items():
            self.assertTrue(path.is_file(), str(path))
            self.assertEqual(path.read_bytes(), content, str(path))
        settings = yaml.safe_load(outputs[PACKAGE / "config/wamv_model.yaml"])
        calibration, controller = boat_calibration(self.sources)
        self.assertEqual(settings, {"navigation": {"model": calibration}, "pid": controller["pid"]})
        self.assertNotIn(PACKAGE / "config/wamv_inertial.yaml", outputs)
        provenance = yaml.safe_load((PACKAGE / "maps/provenance.yaml").read_text())
        self.assertEqual(len(provenance["families"]), 2)
        for uri, digest in provenance["sources_sha256"].items():
            self.assertTrue(uri.startswith("package://"))
            self.assertNotIn("/home/", uri)
            self.assertEqual(hashlib.sha256(self.sources.resource(uri).read_bytes()).hexdigest(), digest)

    def test_spawn_and_goal_connected_after_two_metre_inflation(self):
        for family in ("map1", "map2"):
            res, origin, values = read_map(PACKAGE / "maps" / (family + ".yaml"))
            self.assertEqual(res, .5)
            self.assertEqual(values.shape, (240, 240))
            raw = values != 0
            padded = np.pad(raw, 4, constant_values=True)
            occupied = raw.copy()
            for dy in range(-4, 5):
                for dx in range(-4, 5):
                    if dx * dx + dy * dy <= 16:
                        occupied |= padded[4 + dy:244 + dy, 4 + dx:244 + dx]
            start, goal = index_xy((-570, 167)), index_xy((-500, 232))
            self.assertFalse(occupied[start[1], start[0]], family)
            self.assertFalse(occupied[goal[1], goal[0]], family)
            queue, seen = deque([start]), {start}
            while queue and goal not in seen:
                x, y = queue.popleft()
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    p = x + dx, y + dy
                    if (0 <= p[0] < 240 and 0 <= p[1] < 240 and p not in seen
                            and not occupied[p[1], p[0]]):
                        queue.append(p)
                        seen.add(p)
            self.assertIn(goal, seen, family)

    def test_pgm_row_direction_thresholds_and_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / "tiny.pgm").write_bytes(b"P5\n# comment\n2 2\n255\n" + bytes([0, 254, 205, 254]))
            settings = {"image": "tiny.pgm", "resolution": .5, "origin": [-4, -3, 0],
                        "negate": 0, "free_thresh": .196, "occupied_thresh": .65}
            path = folder / "tiny.yaml"
            path.write_text(yaml.safe_dump(settings))
            _, origin, data = read_map(path)
            self.assertEqual(origin, [-4., -3., 0.])
            self.assertEqual(data.tolist(), [[-1, 0], [100, 0]])
            settings["resolution"] = 0
            path.write_text(yaml.safe_dump(settings))
            with self.assertRaises(ValueError):
                read_map(path)
        for stem in ("map1-1", "map1-2", "map1-3", "map2-1", "map2-2"):
            self.assertTrue(map_for_world(stem + ".world", PACKAGE).is_file())
        with self.assertRaisesRegex(ValueError, "custom map"):
            map_for_world("other.world", PACKAGE)

    def test_runtime_layout_and_datum_checks_allow_unknown_obstacle_changes(self):
        map_yaml = PACKAGE / "maps/map1.yaml"
        validate_world_assets(self.worlds / "map1-2.world", map_yaml)
        with tempfile.TemporaryDirectory() as folder:
            world_path = Path(folder) / "map1-2.world"
            root = ET.parse(str(self.worlds / "map1-2.world")).getroot()
            world = root.find("world")
            unknown = next(i for i in world.findall("include")
                           if i.findtext("name") == "custom_dock_block_1")
            unknown.find("pose").text = "-550 185 .5 0 0 0"
            ET.ElementTree(root).write(str(world_path))
            validate_world_assets(world_path, map_yaml)
            latitude = world.find("spherical_coordinates/latitude_deg")
            latitude.text = "31.3143"
            ET.ElementTree(root).write(str(world_path))
            with self.assertRaisesRegex(ValueError, "geographic reference"):
                validate_world_assets(world_path, map_yaml)
            latitude.text = "30.3143"
            known = next(i for i in world.findall("include")
                         if i.findtext("name") == "robotx_dock_2018")
            known.find("pose").text = "-568 196 0 0 0 0"
            ET.ElementTree(root).write(str(world_path))
            with self.assertRaisesRegex(ValueError, "Known world layout"):
                validate_world_assets(world_path, map_yaml)

    def test_catkin_relays_do_not_shadow_geometry_helper_import(self):
        """The relay's exec uses separate globals, so importing it exports no API."""
        relay_template = '''python_script = {source!r}
with open(python_script, "r") as fh:
    context = {{"__builtins__": __builtins__, "__file__": python_script,
                "__name__": __name__, "__package__": None}}
    exec(compile(fh.read(), python_script, "exec"), context)
'''
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            for name in ("navigation_map_publisher.py", "generate_navigation_maps.py"):
                (folder / name).write_text(relay_template.format(source=str(PACKAGE / "scripts" / name)))
            # Do not let the module imported above hide the relay-shadowing bug.
            previous = sys.modules.pop("generate_navigation_maps", None)
            helper = sys.modules.pop("_std_usv_map_geometry", None)
            sys.path.insert(0, str(folder))
            try:
                namespace = runpy.run_path(str(folder / "navigation_map_publisher.py"),
                                          run_name="navigation_map_relay_test")
                context = namespace["context"]
                context["validate_world_assets"](self.worlds / "map1-3.world", PACKAGE / "maps/map1.yaml")
                self.assertEqual(Path(context["geometry_module"]().__file__).resolve(),
                                 (PACKAGE / "scripts/generate_navigation_maps.py").resolve())
            finally:
                sys.path.pop(0)
                sys.modules.pop("generate_navigation_maps", None)
                sys.modules.pop("_std_usv_map_geometry", None)
                if previous is not None:
                    sys.modules["generate_navigation_maps"] = previous
                if helper is not None:
                    sys.modules["_std_usv_map_geometry"] = helper


if __name__ == "__main__":
    unittest.main()
