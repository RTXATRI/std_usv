#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

SPEC = importlib.util.spec_from_file_location(
    "cloud_filter", str(Path(__file__).resolve().parents[1] / "scripts/navigation_cloud_filter.py"))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CloudFilterTest(unittest.TestCase):
    def setUp(self):
        self.hull = [[-1, -1], [1, -1], [1, 1], [-1, 1]]

    def test_mount_translation_height_self_and_invalid_values(self):
        transform = np.eye(4)
        transform[:3, 3] = [.7, 0, 1.8]
        points = [[0, 0, 0], [4, 0, -1.5], [4, 0, -2],
                  [4, 0, 1], [float("nan"), 0, 0], [float("inf"), 0, 0]]
        actual = MODULE.filter_points(points, transform, self.hull)
        np.testing.assert_allclose(actual, [[4.7, 0, .3]])

    def test_rotation_and_clockwise_footprint(self):
        transform = np.eye(4)
        transform[:2, :2] = [[0, -1], [1, 0]]
        actual = MODULE.filter_points([[3, 0, .5]], transform, self.hull[::-1])
        np.testing.assert_allclose(actual, [[0, 3, .5]])

    def test_empty_scan_and_invalid_transform(self):
        self.assertEqual(MODULE.filter_points([], np.eye(4), self.hull).shape, (0, 3))
        with self.assertRaises(ValueError):
            MODULE.filter_points([[3, 0, .5]], np.zeros((2, 2)), self.hull)

if __name__ == "__main__":
    unittest.main()
