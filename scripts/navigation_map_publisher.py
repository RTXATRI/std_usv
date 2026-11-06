#!/usr/bin/env python3
"""Publish an offline known-environment map; never queries Gazebo ground truth."""

import math
import hashlib
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import rospkg
import yaml


def geometry_module():
    """Load our sibling source, bypassing catkin's non-exporting relay scripts."""
    name = "_std_usv_map_geometry"
    path = Path(__file__).with_name("generate_navigation_maps.py")
    cached = sys.modules.get(name)
    if cached is not None and Path(cached.__file__).resolve() == path.resolve():
        return cached
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise ValueError("Cannot load navigation geometry helper: " + str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(name, None)
        raise
    return module


def map_for_world(world, package_path):
    stem = Path(world).stem
    if stem in ("map1-1", "map1-2", "map1-3"):
        return Path(package_path) / "maps/map1.yaml"
    if stem in ("map2-1", "map2-2"):
        return Path(package_path) / "maps/map2.yaml"
    raise ValueError("Unsupported navigation world {!r}; set ~map_yaml for a custom map".format(stem))


def read_map(path):
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict):
        raise ValueError("Map YAML must contain a mapping")
    resolution = float(config["resolution"])
    origin = [float(v) for v in config["origin"]]
    free, occupied = float(config["free_thresh"]), float(config["occupied_thresh"])
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or not np.isfinite(origin).all():
        raise ValueError("Map resolution/origin must be finite and resolution must be positive")
    if not 0 <= free < occupied <= 1:
        raise ValueError("Map occupancy thresholds must satisfy 0 <= free < occupied <= 1")
    image_path = path.parent / config["image"]
    with Image.open(str(image_path)) as image:
        pixels = np.asarray(image.convert("L"), dtype=np.float64)
    probability = pixels / 255. if int(config.get("negate", 0)) else (255. - pixels) / 255.
    data = np.full(probability.shape, -1, dtype=np.int8)
    data[probability > occupied] = 100
    data[probability < free] = 0
    return resolution, origin, np.flipud(data)


def validate_world_assets(world_path, map_yaml):
    """Reject stale known geometry or an incompatible localization datum.

    Unknown-obstacle placements are deliberately not fingerprinted at runtime:
    changing those must not turn them into known obstacles. Package URIs make
    collision-source checks work in both devel and installed workspaces.
    """
    geometry = geometry_module()
    manifest = yaml.safe_load((Path(map_yaml).parent / "provenance.yaml").read_text())
    family = Path(map_yaml).stem
    expected = manifest["families"][family]
    world = ET.parse(str(world_path)).getroot().find("world")
    if world is None:
        raise ValueError("World file has no world element")
    if not geometry.references_match(geometry.world_reference(world), manifest["geographic_reference"]):
        raise ValueError("World geographic reference does not match the map/localization datum")
    known, _ = geometry.known_includes(world)
    if known != expected["known_instances"]:
        raise ValueError("Known world layout has changed; regenerate the navigation maps")
    sources = geometry.Sources()
    for uri, expected_hash in manifest["sources_sha256"].items():
        if uri.endswith(".world"):
            continue
        actual = hashlib.sha256(sources.resource(uri).read_bytes()).hexdigest()
        if actual != expected_hash:
            raise ValueError("Collision/model source has changed; regenerate maps/calibration: " + uri)


def main():
    import rospy
    from nav_msgs.msg import OccupancyGrid
    rospy.init_node("navigation_map_publisher")
    try:
        package = rospkg.RosPack().get_path("std_usv")
        configured = rospy.get_param("~map_yaml", "")
        path = Path(configured) if configured else map_for_world(rospy.get_param("~world"), package)
        if not configured:
            validate_world_assets(rospy.get_param("~world"), path)
        resolution, origin, data = read_map(path)
        frame = rospy.get_param("~frame_id", "map").lstrip("/")
        if not frame:
            raise ValueError("Map frame_id cannot be empty")
        message = OccupancyGrid()
        message.header.frame_id = frame
        message.info.resolution = resolution
        message.info.height, message.info.width = data.shape
        message.info.origin.position.x, message.info.origin.position.y = origin[:2]
        message.info.origin.orientation.z = math.sin(origin[2] / 2.)
        message.info.origin.orientation.w = math.cos(origin[2] / 2.)
        message.data = data.reshape(-1).tolist()
        publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        # A zero timestamp is valid while a paused simulation has not advanced.
        message.header.stamp = rospy.Time.now()
        message.info.map_load_time = message.header.stamp
        publisher.publish(message)
        rospy.loginfo("Known navigation map: %s (%dx%d, %.3fm)", path, data.shape[1], data.shape[0], resolution)
        rospy.spin()
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError, yaml.YAMLError) as error:
        rospy.logfatal("Cannot load navigation map: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
