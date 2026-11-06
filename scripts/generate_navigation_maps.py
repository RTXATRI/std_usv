#!/usr/bin/env python3
"""Rebuild known-environment navigation maps and WAM-V calibration from SDF.

The generated maps deliberately exclude the scenario's custom_dock_block_* and
dynamic_obstacle_* instances: these are unknown obstacles sensed by the lidar.
No Gazebo server, saved state, or robot ground-truth pose is used.
"""

import argparse
import hashlib
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import yaml


ORIGIN = (-580.0, 157.0)
RESOLUTION = 0.5
WIDTH = HEIGHT = 240
SHORE_MIN_Z = -0.5
OBSTACLE_MAX_Z = 2.2
FAMILIES = {"map1": ("map1-1", "map1-2", "map1-3"),
            "map2": ("map2-1", "map2-2")}
UNKNOWN = ("custom_dock_block_", "dynamic_obstacle_")
GEOGRAPHIC_REFERENCE = {"surface_model": "EARTH_WGS84", "world_frame_orientation": "ENU",
                        "latitude_deg": 30.3143, "longitude_deg": 120.338,
                        "elevation": 0.0, "heading_deg": 0.0}
KNOWN_NAME = re.compile(r"^(sydneyregatta|post_\d+|antenna|ground_station_\d+|"
                        r"blue_projectile|ocean_waves|robotx_dock_.+)$")
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def pose_matrix(text=None):
    """SDF pose: translation followed by extrinsic XYZ roll/pitch/yaw."""
    values = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError("SDF poses must contain six finite values")
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                   [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                   [-sp, cp * sr, cp * cr]]
    out[:3, 3] = [x, y, z]
    return out


def transformed(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def convex_hull(points):
    """Counterclockwise monotone-chain hull, independent of input order."""
    pts = sorted(set(tuple(map(float, p)) for p in points))
    if len(pts) < 3:
        return np.asarray(pts)
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-12:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-12:
            upper.pop()
        upper.append(p)
    return np.asarray(lower[:-1] + upper[:-1])


def clip_above(vertices, height):
    """Clip a triangle/polygon against z >= height before XY projection."""
    result = []
    previous = vertices[-1]
    for current in vertices:
        a, b = previous[2] >= height, current[2] >= height
        if a != b:
            fraction = (height - previous[2]) / (current[2] - previous[2])
            result.append(previous + fraction * (current - previous))
        if b:
            result.append(current)
        previous = current
    return np.asarray(result)


def raster_polygon(grid, vertices, origin=ORIGIN, resolution=RESOLUTION):
    """Mark cells intersecting a convex polygon, using exact separating axes.

    Cell-centre-only sampling loses thin posts and shore edges. This instead
    tests intersection with each entire cell square, including touched edges.
    """
    poly = convex_hull(np.asarray(vertices)[:, :2])
    if len(poly) < 3:
        return
    origin = np.asarray(origin)
    lo = np.maximum(np.floor((poly.min(axis=0) - origin) / resolution).astype(int), 0)
    hi = np.minimum(np.floor((poly.max(axis=0) - origin) / resolution).astype(int),
                    [grid.shape[1] - 1, grid.shape[0] - 1])
    if (lo > hi).any():
        return
    xx, yy = np.meshgrid(origin[0] + (np.arange(lo[0], hi[0] + 1) + .5) * resolution,
                         origin[1] + (np.arange(lo[1], hi[1] + 1) + .5) * resolution)
    intersects = np.ones(xx.shape, dtype=bool)
    axes = [np.array([1., 0.]), np.array([0., 1.])]
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge = b - a
        axes.append(np.array([-edge[1], edge[0]]))
    for axis in axes:
        projection = poly @ axis
        centre = xx * axis[0] + yy * axis[1]
        radius = .5 * resolution * np.abs(axis).sum()
        intersects &= (centre + radius >= projection.min() - 1e-10)
        intersects &= (centre - radius <= projection.max() + 1e-10)
    grid[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1] |= intersects


class ModelFrames:
    """Resolve the explicit SDF link/joint frames used by the current models."""
    def __init__(self, model):
        self.frames = {"__model__": (None, np.eye(4))}
        self.resolved = {"__model__": np.eye(4)}
        for elem in list(model):
            if elem.tag not in ("link", "joint", "frame", "model"):
                continue
            pose = elem.find("pose")
            relative = pose.get("relative_to", pose.get("frame", "")) if pose is not None else ""
            if not relative:
                # SDF joint poses default to the child-link frame.
                relative = elem.findtext("child") if elem.tag == "joint" else "__model__"
            self.frames[elem.get("name")] = (relative, pose_matrix(pose.text if pose is not None else None))

    def resolve(self, name, visiting=None):
        if name in self.resolved:
            return self.resolved[name]
        if name not in self.frames:
            raise ValueError("Unresolved SDF frame: " + str(name))
        visiting = set() if visiting is None else visiting
        if name in visiting:
            raise ValueError("Cyclic SDF frames: " + str(name))
        parent, local = self.frames[name]
        result = self.resolve(parent, visiting | {name}) @ local
        self.resolved[name] = result
        return result

    def element_pose(self, elem, default):
        pose = elem.find("pose")
        if pose is None:
            return self.resolve(default)
        relative = pose.get("relative_to", pose.get("frame", default))
        return self.resolve(relative) @ pose_matrix(pose.text)


class Sources:
    def __init__(self):
        ros = rospkg.RosPack()
        self.packages = {name: Path(ros.get_path(name)).resolve() for name in
                         ("ocean_simulation_gazebo", "vrx_gazebo", "wamv_description", "wamv_gazebo", "wave_gazebo")}
        sim = self.packages["ocean_simulation_gazebo"]
        self.model_roots = [sim / "models/vehicle", sim / "models/other",
                            self.packages["wamv_description"] / "models",
                            self.packages["wamv_gazebo"] / "models",
                            self.packages["vrx_gazebo"] / "models",
                            self.packages["wave_gazebo"] / "world_models",
                            self.packages["wave_gazebo"] / "models"]
        self.files = {}
        self.xml_cache = {}
        self.mesh_cache = {}

    def portable(self, path):
        path = Path(path).resolve()
        for name, root in self.packages.items():
            try:
                return "package://" + name + "/" + str(path.relative_to(root))
            except ValueError:
                continue
        raise ValueError("Source is outside registered ROS packages: " + str(path))

    def record(self, path):
        path = Path(path).resolve()
        self.files[self.portable(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

    def xml(self, path):
        path = Path(path).resolve()
        self.record(path)
        if path not in self.xml_cache:
            self.xml_cache[path] = ET.parse(str(path)).getroot()
        return self.xml_cache[path]

    def resource(self, uri):
        if uri.startswith("package://"):
            package, relative = uri[10:].split("/", 1)
            path = self.packages[package] / relative
            if path.exists():
                return path
        if uri.startswith(("model://", "file://")):
            relative = uri.split("://", 1)[1]
            for root in self.model_roots:
                path = root / relative
                if path.exists():
                    return path
        raise ValueError("Cannot resolve ROS/Gazebo resource: " + uri)

    def model(self, uri):
        path = self.resource(uri)
        if path.is_dir():
            config = path / "model.config"
            if config.exists():
                root = self.xml(config)
                candidates = root.findall("sdf")
                name = candidates[-1].text if candidates else "model.sdf"
            else:
                name = "model.sdf"
            path = path / name
        root = self.xml(path)
        model = root.find("model")
        if model is None:
            raise ValueError("SDF model not found: " + str(path))
        return model

    def mesh_triangles(self, uri):
        path = self.resource(uri)
        self.record(path)
        if path in self.mesh_cache:
            return self.mesh_cache[path]
        if path.suffix.lower() != ".dae":
            raise ValueError("Unsupported collision mesh format: " + str(path))
        root = self.xml(path)
        geometries = {}
        for geometry in root.findall("c:library_geometries/c:geometry", NS):
            mesh = geometry.find("c:mesh", NS)
            sources = {}
            for source in mesh.findall("c:source", NS):
                floats = source.find("c:float_array", NS)
                if floats is None:
                    continue
                accessor = source.find("c:technique_common/c:accessor", NS)
                stride = int(accessor.get("stride", "1"))
                sources[source.get("id")] = np.fromstring(floats.text, sep=" ").reshape(-1, stride)[:, :3]
            vertices = {v.get("id"): v.find("c:input[@semantic='POSITION']", NS).get("source")[1:]
                        for v in mesh.findall("c:vertices", NS)}
            triangles = []
            for primitive in mesh.findall("c:triangles", NS):
                inputs = primitive.findall("c:input", NS)
                stride = 1 + max(int(i.get("offset", "0")) for i in inputs)
                inp = next(i for i in inputs if i.get("semantic") == "VERTEX")
                indices = np.fromstring(primitive.find("c:p", NS).text, sep=" ", dtype=int)
                indices = indices.reshape(-1, stride)[:, int(inp.get("offset", "0"))].reshape(-1, 3)
                triangles.extend(sources[vertices[inp.get("source")[1:]]][indices])
            if not triangles:
                raise ValueError("Collision DAE has no triangles: " + str(path))
            geometries[geometry.get("id")] = np.asarray(triangles)
        unit = root.find("c:asset/c:unit", NS)
        scale = float(unit.get("meter", "1")) if unit is not None else 1.
        axis = root.findtext("c:asset/c:up_axis", "Y_UP", NS)
        asset = np.diag([scale, scale, scale, 1.])
        if axis == "Y_UP":
            asset = pose_matrix("0 0 0 1.5707963267948966 0 0") @ asset
        elif axis != "Z_UP":
            raise ValueError("Unsupported collision DAE up-axis: " + axis)
        scene_url = root.find("c:scene/c:instance_visual_scene", NS)
        scene_id = scene_url.get("url")[1:] if scene_url is not None else None
        scenes = root.findall("c:library_visual_scenes/c:visual_scene", NS)
        scene = next((s for s in scenes if s.get("id") == scene_id), scenes[0] if scenes else None)
        result = []
        def visit(node, parent):
            local = np.eye(4)
            for child in node:
                tag = child.tag.split("}")[-1]
                values = np.fromstring(child.text or "", sep=" ") if tag in ("matrix", "translate", "rotate", "scale") else []
                change = np.eye(4)
                if tag == "matrix":
                    change = values.reshape(4, 4).T
                elif tag == "translate":
                    change[:3, 3] = values
                elif tag == "scale":
                    change = np.diag(list(values) + [1.])
                elif tag == "rotate":
                    axis_v, angle = values[:3], math.radians(values[3])
                    axis_v = axis_v / np.linalg.norm(axis_v)
                    a, b, c = axis_v
                    skew = np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])
                    change[:3, :3] = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)
                else:
                    continue
                local = local @ change
            world = parent @ local
            for instance in node.findall("c:instance_geometry", NS):
                data = geometries[instance.get("url")[1:]]
                result.extend(transformed(data.reshape(-1, 3), world).reshape(-1, 3, 3))
            for child in node.findall("c:node", NS):
                visit(child, world)
        if scene is not None:
            for node in scene.findall("c:node", NS):
                visit(node, asset)
        else:
            for data in geometries.values():
                result.extend(transformed(data.reshape(-1, 3), asset).reshape(-1, 3, 3))
        self.mesh_cache[path] = np.asarray(result)
        return self.mesh_cache[path]


def primitive_vertices(geometry):
    box = geometry.find("box")
    if box is not None:
        half = np.fromstring(box.findtext("size"), sep=" ") / 2.
        return np.asarray([[x, y, z] for x in (-half[0], half[0])
                           for y in (-half[1], half[1]) for z in (-half[2], half[2])])
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if cylinder is not None or sphere is not None:
        shape = cylinder if cylinder is not None else sphere
        radius = float(shape.findtext("radius"))
        height = float(shape.findtext("length")) / 2. if cylinder is not None else radius
        # Circumscription avoids underestimating the continuous cylinder surface.
        radius /= math.cos(math.pi / 64.)
        theta = (np.arange(64) + .5) * (2 * math.pi / 64.)
        return np.asarray([[radius * math.cos(t), radius * math.sin(t), z]
                           for t in theta for z in (-height, height)])
    raise ValueError("Unsupported primitive collision geometry: " + geometry[0].tag)


def collision_shapes(model, sources, matrix=None):
    """Yield (mesh triangles or primitive points, is_mesh) in the root frame."""
    matrix = np.eye(4) if matrix is None else matrix
    frames = ModelFrames(model)
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            transform = matrix @ frames.element_pose(collision, link.get("name"))
            geometry = collision.find("geometry")
            mesh = geometry.find("mesh")
            if mesh is not None:
                data = sources.mesh_triangles(mesh.findtext("uri"))
                scale = np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ")
                yield transformed(data.reshape(-1, 3) * scale, transform).reshape(-1, 3, 3), True
            else:
                yield transformed(primitive_vertices(geometry), transform), False
    for nested in model.findall("model"):
        yield from collision_shapes(nested, sources, matrix @ frames.resolve(nested.get("name")))
    for include in model.findall("include"):
        child = sources.model(include.findtext("uri"))
        pose = include.find("pose")
        if pose is not None:
            transform = frames.element_pose(include, "__model__")
        else:
            transform = pose_matrix(child.findtext("pose"))
        yield from collision_shapes(child, sources, matrix @ transform)


def known_includes(world):
    known, excluded = [], []
    for include in world.findall("include"):
        name = include.findtext("name") or include.findtext("uri").split("/")[-1]
        if name.startswith(UNKNOWN):
            excluded.append(name)
            continue
        if not KNOWN_NAME.fullmatch(name):
            raise ValueError("Unknown environment model must be classified explicitly: " + name)
        known.append({"name": name, "uri": include.findtext("uri"),
                      "pose": [float(v) for v in include.findtext("pose", "0 0 0 0 0 0").split()]})
    if world.findall("model"):
        raise ValueError("Navigation-map inputs must use the current pure-environment include-based worlds")
    return sorted(known, key=lambda row: row["name"]), sorted(excluded)


def world_reference(world):
    element = world.find("spherical_coordinates")
    if element is None:
        raise ValueError("World has no explicit geographic reference")
    reference = {}
    for key, expected in GEOGRAPHIC_REFERENCE.items():
        text = element.findtext(key)
        if text is None:
            raise ValueError("World geographic reference is missing " + key)
        reference[key] = float(text) if isinstance(expected, float) else text.strip()
    return reference


def references_match(actual, expected):
    for key, value in expected.items():
        other = actual.get(key)
        if isinstance(value, (int, float)):
            if not isinstance(other, (int, float)) or not math.isclose(other, value, rel_tol=0., abs_tol=1e-9):
                return False
        elif other != value:
            return False
    return True


def world_grid(world_path, sources):
    world = sources.xml(world_path).find("world")
    known, excluded = known_includes(world)
    grid = np.zeros((HEIGHT, WIDTH), dtype=bool)
    counts = {"primitive_collisions": 0, "mesh_triangles": 0}
    for instance in known:
        model = sources.model(instance["uri"])
        transform = pose_matrix(" ".join(map(str, instance["pose"])))
        for data, is_mesh in collision_shapes(model, sources, transform):
            if is_mesh:
                for triangle in data:
                    if triangle[:, 2].max() < SHORE_MIN_Z:
                        continue
                    polygon = clip_above(triangle, SHORE_MIN_Z)
                    if len(polygon) >= 3:
                        raster_polygon(grid, polygon)
                        counts["mesh_triangles"] += 1
            elif data[:, 2].max() >= SHORE_MIN_Z and data[:, 2].min() <= OBSTACLE_MAX_Z:
                raster_polygon(grid, data)
                counts["primitive_collisions"] += 1
    return grid, known, excluded, counts


def rounded(value):
    if isinstance(value, np.ndarray):
        return rounded(value.tolist())
    if isinstance(value, list):
        return [rounded(v) for v in value]
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    if isinstance(value, (float, np.floating)):
        return round(float(value), 9)
    return value


def inertial_dict(tensor):
    return {"ixx": tensor[0, 0], "ixy": tensor[0, 1], "ixz": tensor[0, 2],
            "iyy": tensor[1, 1], "iyz": tensor[1, 2], "izz": tensor[2, 2]}


def boat_calibration(sources):
    path = sources.packages["ocean_simulation_gazebo"] / "models/vehicle/wamv/model.sdf"
    model = sources.xml(path).find("model")
    frames = ModelFrames(model)
    mass, weighted, inertia = 0., np.zeros(3), np.zeros((3, 3))
    for link in model.findall("link"):
        element = link.find("inertial")
        if element is None:
            continue
        value = float(element.findtext("mass"))
        transform = frames.element_pose(element, link.get("name"))
        centre = transform[:3, 3]
        raw = element.find("inertia")
        vals = {k: float(raw.findtext(k, "0")) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
        tensor = np.array([[vals["ixx"], vals["ixy"], vals["ixz"]],
                           [vals["ixy"], vals["iyy"], vals["iyz"]],
                           [vals["ixz"], vals["iyz"], vals["izz"]]])
        rotation = transform[:3, :3]
        inertia += rotation @ tensor @ rotation.T + value * (np.dot(centre, centre) * np.eye(3) - np.outer(centre, centre))
        mass += value
        weighted += value * centre
    centre = weighted / mass
    com_inertia = inertia - mass * (np.dot(centre, centre) * np.eye(3) - np.outer(centre, centre))
    points = []
    for shape, mesh in collision_shapes(model, sources):
        points.extend(shape.reshape(-1, 3)[:, :2])
    footprint = convex_hull(points)
    source = {"uri": sources.portable(path), "sha256": sources.files[sources.portable(path)]}
    calibration = rounded({"frame_id": "wamv/base_link", "joint_configuration": "zero joint positions, existing X thruster mounting",
                           "footprint": footprint, "mass": mass, "inertial": inertial_dict(inertia),
                           "inertia_reference": "wamv/base_link origin; parallel-axis sum of all link inertias",
                           "center_of_mass": centre, "inertial_at_center_of_mass": inertial_dict(com_inertia),
                           "source": source})
    controller = rounded({"pid": {"mass": mass, "inertial": inertial_dict(inertia)}})
    return calibration, controller


def yaml_bytes(data):
    return yaml.safe_dump(data, allow_unicode=True, sort_keys=False, width=110).encode("utf-8")


def pgm_bytes(grid):
    # OccupancyGrid rows start at the bottom; PGM starts with the top row.
    pixels = np.where(np.flipud(grid), 0, 254).astype(np.uint8)
    return ("P5\n# Generated from known environment collisions; unknown scenario obstacles excluded.\n"
            "{} {}\n255\n".format(grid.shape[1], grid.shape[0])).encode("ascii") + pixels.tobytes()


def generate(output_dir=None):
    sources = Sources()
    # rospack also locates the installed share/std_usv directory correctly.
    package = Path(rospkg.RosPack().get_path("std_usv"))
    output_dir = Path(output_dir) if output_dir is not None else package / "maps"
    outputs = {}
    provenance = {"schema_version": 1, "frame_id": "map", "resolution": RESOLUTION,
                  "origin": list(ORIGIN), "size_cells": [WIDTH, HEIGHT],
                  "geographic_reference": GEOGRAPHIC_REFERENCE,
                  "shore_min_z": SHORE_MIN_Z, "primitive_max_z": OBSTACLE_MAX_Z,
                  "unknown_instance_prefixes": list(UNKNOWN), "families": {}}
    world_dir = sources.packages["ocean_simulation_gazebo"] / "worlds/usv_path_planning"
    for family, variants in FAMILIES.items():
        grid = None
        signatures = None
        records = {}
        for stem in variants:
            path = world_dir / (stem + ".world")
            world = sources.xml(path).find("world")
            if not references_match(world_reference(world), GEOGRAPHIC_REFERENCE):
                raise ValueError("World reference differs from the navigation datum: " + stem)
            known, excluded = known_includes(world)
            if signatures is None:
                signatures = known
                grid, _, _, counts = world_grid(path, sources)
            elif known != signatures:
                raise ValueError("Known model layout differs within map family: " + stem)
            records[stem + ".world"] = {"source": sources.portable(path), "excluded_instances": excluded}
        config = {"image": family + ".pgm", "resolution": RESOLUTION,
                  "origin": [ORIGIN[0], ORIGIN[1], 0.0], "negate": 0,
                  "occupied_thresh": 0.65, "free_thresh": 0.196}
        outputs[output_dir / (family + ".yaml")] = yaml_bytes(config)
        outputs[output_dir / (family + ".pgm")] = pgm_bytes(grid)
        provenance["families"][family] = {"variants": records, "known_instances": signatures,
                                           "occupied_cells": int(grid.sum()), "geometry_counts": counts}
    calibration, controller = boat_calibration(sources)
    # One file is loaded once under /wamv, preserving both parameter namespaces.
    outputs[package / "config/wamv_model.yaml"] = yaml_bytes(
        {"navigation": {"model": calibration}, "pid": controller["pid"]})
    provenance["sources_sha256"] = dict(sorted(sources.files.items()))
    outputs[output_dir / "provenance.yaml"] = yaml_bytes(provenance)
    return outputs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, help="Map output directory (default: std_usv/maps)")
    parser.add_argument("--check", action="store_true", help="Check maps and calibration are current without writing")
    args = parser.parse_args(argv)
    outputs = generate(args.output_dir)
    outdated = []
    for path, content in outputs.items():
        if args.check:
            if not path.exists() or path.read_bytes() != content:
                outdated.append(str(path))
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
            print("Generated", path)
    if outdated:
        print("Generated assets are missing or stale:\n" + "\n".join(outdated), file=sys.stderr)
        return 1
    if args.check:
        print("Navigation maps and WAM-V calibration match the current SDF sources.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
