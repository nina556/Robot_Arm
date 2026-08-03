#!/usr/bin/env python3
"""Generate a MuJoCo-friendly URDF from the Web visual URDF."""

from __future__ import annotations

import argparse
import copy
import math
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

MAX_MUJOCO_STL_FACES = 190_000


def is_binary_stl(data: bytes) -> bool:
    if len(data) < 84:
        return False
    face_count = struct.unpack_from("<I", data, 80)[0]
    return 84 + face_count * 50 == len(data)


def binary_stl_face_count(data: bytes) -> int:
    if len(data) < 84:
        return 0
    face_count = struct.unpack_from("<I", data, 80)[0]
    if 84 + face_count * 50 != len(data):
        return 0
    return face_count


def binary_stl_vertices(data: bytes):
    face_count = binary_stl_face_count(data)
    if face_count <= 0:
        return []
    vertices = []
    for face_index in range(face_count):
        offset = 84 + face_index * 50 + 12
        vertices.extend(
            (
                struct.unpack_from("<3f", data, offset),
                struct.unpack_from("<3f", data, offset + 12),
                struct.unpack_from("<3f", data, offset + 24),
            )
        )
    return vertices


def box_vertices_from_bounds(minimum, maximum):
    min_x, min_y, min_z = minimum
    max_x, max_y, max_z = maximum
    corners = [
        (min_x, min_y, min_z),
        (max_x, min_y, min_z),
        (max_x, max_y, min_z),
        (min_x, max_y, min_z),
        (min_x, min_y, max_z),
        (max_x, min_y, max_z),
        (max_x, max_y, max_z),
        (min_x, max_y, max_z),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6),
        (3, 0, 4), (3, 4, 7),
    ]
    vertices = []
    for face in faces:
        vertices.extend(corners[index] for index in face)
    return vertices


def bounding_box_vertices(vertices):
    if not vertices:
        return []
    minimum = tuple(min(vertex[axis] for vertex in vertices) for axis in range(3))
    maximum = tuple(max(vertex[axis] for vertex in vertices) for axis in range(3))
    return box_vertices_from_bounds(minimum, maximum)


def normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length <= 1e-12:
        return 0.0, 0.0, 0.0
    return nx / length, ny / length, nz / length


def ascii_stl_vertices(text: str):
    vertices = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append(tuple(float(value) for value in parts[1:]))
    if len(vertices) % 3 != 0:
        raise ValueError("ASCII STL vertex count is not divisible by 3")
    return vertices


def write_binary_stl(vertices, target: Path) -> None:
    face_count = len(vertices) // 3
    with target.open("wb") as fh:
        fh.write(b"unoarm mujoco generated binary stl".ljust(80, b"\0"))
        fh.write(struct.pack("<I", face_count))
        for index in range(face_count):
            a, b, c = vertices[index * 3 : index * 3 + 3]
            fh.write(struct.pack("<3f", *normal(a, b, c)))
            fh.write(struct.pack("<3f", *a))
            fh.write(struct.pack("<3f", *b))
            fh.write(struct.pack("<3f", *c))
            fh.write(struct.pack("<H", 0))


def copy_or_convert_stl(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    data = source.read_bytes()
    if source.suffix.lower() != ".stl":
        shutil.copy2(source, target)
        return
    if is_binary_stl(data):
        face_count = binary_stl_face_count(data)
        if 0 < face_count <= MAX_MUJOCO_STL_FACES:
            shutil.copy2(source, target)
            return
        vertices = bounding_box_vertices(binary_stl_vertices(data))
    else:
        vertices = ascii_stl_vertices(data.decode("utf-8", errors="ignore"))
    write_binary_stl(vertices, target)


def ensure_inertial(link: ET.Element) -> None:
    if link.find("inertial") is not None:
        return
    inertial = ET.Element("inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "0.001"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "1e-8",
            "ixy": "0",
            "ixz": "0",
            "iyy": "1e-8",
            "iyz": "0",
            "izz": "1e-8",
        },
    )
    link.insert(0, inertial)


def root_link_name(root: ET.Element) -> str:
    links = [link.get("name") for link in root.findall("link") if link.get("name")]
    children = {
        child.get("link")
        for child in root.findall("joint/child")
        if child.get("link")
    }
    roots = [name for name in links if name not in children]
    if not roots:
        raise ValueError("URDF has no root link")
    return roots[0]


def add_mujoco_world_rotation(root: ET.Element) -> None:
    if root.find("./joint[@name='world_to_base_link']") is not None:
        return
    base = root_link_name(root)
    if base == "world":
        return
    world = ET.Element("link", {"name": "world"})
    joint = ET.Element("joint", {"name": "world_to_base_link", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "1.5707963268 0 0"})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": base})
    root.insert(0, joint)
    root.insert(0, world)


def mesh_source_path(filename: str, source_dir: Path) -> Path:
    if filename.startswith("package://"):
        raise ValueError(f"package:// mesh paths are not supported here: {filename}")
    path = Path(filename)
    if path.is_absolute():
        return path
    return (source_dir / path).resolve()


def mujoco_mesh_filename(relative: Path) -> Path:
    parts = [part for part in relative.parts if part not in ("", ".")]
    stem = "__".join(parts)
    if not stem:
        stem = relative.name
    return Path(stem)


def convert_mesh_references(root: ET.Element, source_dir: Path, mesh_dir: Path) -> None:
    converted = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        source = mesh_source_path(filename, source_dir)
        relative = Path(filename)
        target_relative = mujoco_mesh_filename(relative)
        target = mesh_dir / target_relative
        key = str(source)
        if key not in converted:
            copy_or_convert_stl(source, target)
            converted[key] = target_relative
        mesh.set("filename", str(Path("meshes") / converted[key]).replace("\\", "/"))


def visual_to_collision(link: ET.Element) -> None:
    if link.get("name") in {"Left_Wrist_RGBD", "Right_Wrist_RGBD"}:
        # Eye-in-hand housings are visualization-only accessories.  Turning
        # them into collision geoms changes grasp planning/contact behavior.
        for collision in list(link.findall("collision")):
            link.remove(collision)
        return
    visuals = list(link.findall("visual"))
    if not visuals:
        return
    for collision in list(link.findall("collision")):
        link.remove(collision)
    for visual in visuals:
        collision = ET.Element("collision")
        origin = visual.find("origin")
        geometry = visual.find("geometry")
        if origin is not None:
            collision.append(copy.deepcopy(origin))
        if geometry is not None:
            collision.append(copy.deepcopy(geometry))
        link.append(collision)


def synchronize_arm_joint_limits(root: ET.Element, limits_source: Path) -> None:
    """Use the same arm joint types and limits as MoveIt's canonical URDF."""
    canonical_root = ET.parse(limits_source).getroot()
    canonical = {
        joint.get("name"): joint
        for joint in canonical_root.findall("joint")
        if joint.get("name") in {
            *(f"Right_Joint{index}" for index in range(1, 8)),
            *(f"Left_Joint{index}" for index in range(1, 8)),
        }
    }
    for joint in root.findall("joint"):
        reference = canonical.get(joint.get("name"))
        if reference is None:
            continue
        joint.set("type", reference.get("type", joint.get("type", "revolute")))
        old_limit = joint.find("limit")
        new_limit = reference.find("limit")
        if old_limit is not None:
            joint.remove(old_limit)
        if new_limit is not None:
            joint.append(copy.deepcopy(new_limit))


def generate(
    source: Path,
    output: Path,
    mesh_dir: Path,
    joint_limits_source: Path | None = None,
) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    source_dir = source.parent.resolve()
    mesh_dir = mesh_dir.resolve()
    add_mujoco_world_rotation(root)
    if joint_limits_source is not None:
        synchronize_arm_joint_limits(root, joint_limits_source)
    convert_mesh_references(root, source_dir, mesh_dir)
    for link in root.findall("link"):
        visual_to_collision(link)
        ensure_inertial(link)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mesh-dir", required=True, type=Path)
    parser.add_argument("--joint-limits-source", type=Path)
    args = parser.parse_args()
    generate(args.source, args.output, args.mesh_dir, args.joint_limits_source)


if __name__ == "__main__":
    main()
