#!/usr/bin/env python3
"""Five real course layouts through the pure A*/DWA core, without a ROS master.

This deterministic regression uses ideal planar lidar and first-order velocity
lag. It checks positional arrival and footprint collisions only; it does not
validate GPS/IMU localization, real propulsion, heading settling or Gazebo.
Unknown obstacles are supplied only to the sensor/oracle, never the static map.
"""

import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
import roslib.packages
import rospkg
import yaml


PACKAGE = Path(rospkg.RosPack().get_path("std_usv"))


def source_module(name, script):
    spec = importlib.util.spec_from_file_location(name, str(PACKAGE / "scripts" / script))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


geometry = source_module("course_geometry", "generate_navigation_maps.py")
maps = source_module("course_maps", "navigation_map_publisher.py")


def unknown_boxes(world_path, sources):
    """Conservative XY collision AABBs from current SDF instances/resources.

    Current custom blocks and buoy models have no instance yaw. Using projected
    bounds also preserves conservative collision geometry if their pose changes.
    Duplicate stacked block footprints collapse to one planar obstacle.
    """
    boxes = set()
    world = sources.xml(world_path).find("world")
    for include in world.findall("include"):
        if not include.findtext("name", "").startswith(geometry.UNKNOWN):
            continue
        model = sources.model(include.findtext("uri"))
        transform = geometry.pose_matrix(include.findtext("pose"))
        shapes = [vertices.reshape(-1, 3) for vertices, _ in
                  geometry.collision_shapes(model, sources, transform)]
        if not shapes:
            raise ValueError("Unknown obstacle has no collision geometry")
        vertices = np.concatenate(shapes)
        minimum, maximum = vertices[:, :2].min(axis=0), vertices[:, :2].max(axis=0)
        boxes.add(tuple(round(float(v), 9) for v in (*minimum, *maximum)))
    return sorted(boxes)


class CourseNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        nodes = roslib.packages.find_node("std_usv", "course_navigation_smoke")
        if not nodes:
            raise RuntimeError("Build the course_navigation_smoke test target first")
        cls.binary = nodes[0]
        cls.temporary = tempfile.TemporaryDirectory(prefix="std-usv-courses-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.folder = Path(cls.temporary.name)
        cls.sources = geometry.Sources()
        cls.worlds = cls.sources.packages["ocean_simulation_gazebo"] / "worlds/usv_path_planning"
        footprint = yaml.safe_load((PACKAGE / "config/wamv_model.yaml").read_text())["navigation"]["model"]["footprint"]
        for family in ("map1", "map2"):
            resolution, origin, values = maps.read_map(PACKAGE / "maps" / (family + ".yaml"))
            height, width = values.shape
            path = cls.folder / (family + ".txt")
            with path.open("w") as stream:
                stream.write("{} {} {} {} {}\n".format(width, height, resolution, *origin[:2]))
                np.savetxt(stream, (values != 0).astype(int), fmt="%d")
                stream.write(str(len(footprint)) + "\n")
                np.savetxt(stream, footprint, fmt="%.12g")

    def check_course(self, stem):
        boxes = unknown_boxes(self.worlds / (stem + ".world"), self.sources)
        obstacles = self.folder / (stem + "-unknown.txt")
        with obstacles.open("w") as stream:
            stream.write(str(len(boxes)) + "\n")
            for box in boxes:
                stream.write("{} {} {} {}\n".format(*box))
        result = subprocess.run([self.binary, str(self.folder / (stem[:4] + ".txt")),
                                 str(obstacles)], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, universal_newlines=True, timeout=45)
        self.assertEqual(result.returncode, 0, stem + ": " + result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertLessEqual(report["position_error"], 1., stem)
        self.assertLessEqual(report["sim_seconds"], 600., stem)
        self.assertTrue(report["collision_free"], stem)
        print("{} algorithm-only: {}".format(stem, json.dumps(report, sort_keys=True)))

    def test_map1_1(self):
        self.check_course("map1-1")

    def test_map1_2(self):
        self.check_course("map1-2")

    def test_map1_3(self):
        self.check_course("map1-3")

    def test_map2_1(self):
        self.check_course("map2-1")

    def test_map2_2(self):
        self.check_course("map2-2")


if __name__ == "__main__":
    unittest.main()
