#!/usr/bin/env python3
"""Build the UnoArm MuJoCo scene with fixed cameras and simple table objects."""

import argparse
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_GRASP_OBJECT_MESH = (
    ROOT_DIR / "web_control/frontend/public/models/part.stl"
)
SLOT_MESH = ROOT_DIR / "asm0003/meshes/槽.STL"
SLOT_MESH_SCALE = 0.001
# 槽.STL 使用 Y 轴作为高度轴，原始尺寸单位为 mm。
SLOT_MESH_CENTER_MM = (64.35, 70.0, 64.35)
SCENE_CONFIG_FILE = Path(os.environ.get(
    "MUJOCO_SCENE_CONFIG",
    str(Path(__file__).resolve().parents[1] / "mujoco" / "scene_config.json"),
))


def load_scene_config():
    try:
        data = json.loads(SCENE_CONFIG_FILE.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


SCENE_CONFIG = load_scene_config()


# Fixed transforms copied from web_control/frontend/public/urdf/unoarm.urdf.
HEAD_MOUNT_XYZ = (0.0, 1.403, 0.181)
HEAD_MOUNT_RPY = (0.0, math.pi, 0.0)
HEAD_D455_XYZ = (0.005706037095, 0.222006569836, -0.077006573900)
HEAD_D455_RPY = (math.pi / 4.0, 0.0, 0.0)
WORLD_TO_BASE_ROLL = math.pi / 2.0
# The URDF link origin lies on the D455 front housing plane.  Move the virtual
# pinhole just beyond that plane so the camera does not render its own shell.
D455_OPTICAL_CENTER_FORWARD = 0.03
# Mirror the complete eye-in-hand transform about the robot center plane:
# right uses -90 degrees about Link7 X, left uses +90 degrees. Both optical
# centers are also shifted toward the fingertips.
WRIST_CAMERA_PITCH = math.radians(-78.0)
WRIST_HOUSING_FORWARD = 0.120
WRIST_OPTICAL_FORWARD = 0.137


def rpy_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def wrist_housing_transform(side):
    roll = math.radians(-90.0 if side == "Right" else 90.0)
    yaw = math.radians(-5.0 if side == "Right" else 5.0)
    lateral = 0.067 if side == "Right" else -0.067
    return (WRIST_HOUSING_FORWARD, lateral, 0.0), rpy_quat(roll, 0.0, yaw)


def wrist_camera_transform(side):
    roll = math.radians(-90.0 if side == "Right" else 90.0)
    lateral = 0.068 if side == "Right" else -0.068
    half_roll = roll / 2.0
    half_pitch = WRIST_CAMERA_PITCH / 2.0
    quat = (
        math.cos(half_roll) * math.cos(half_pitch),
        math.sin(half_roll) * math.cos(half_pitch),
        math.cos(half_roll) * math.sin(half_pitch),
        math.sin(half_roll) * math.sin(half_pitch),
    )
    return (WRIST_OPTICAL_FORWARD, lateral, 0.0), quat


def parse_quat_wxyz(value, default):
    raw = value if value is not None else " ".join(str(part) for part in default)
    parts = [float(part) for part in str(raw).replace(",", " ").split()]
    if len(parts) != 4 or not all(math.isfinite(part) for part in parts):
        raise ValueError(f"invalid quaternion: {raw!r}")
    length = math.sqrt(sum(part * part for part in parts))
    if length <= 1e-9:
        raise ValueError(f"zero-length quaternion: {raw!r}")
    return tuple(part / length for part in parts)


def quat_rotate(quat, vector):
    w, x, y, z = quat
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )
TABLE_CENTER = (
    float(os.environ.get("MUJOCO_TABLE_CENTER_X", "0.0")),
    float(os.environ.get("MUJOCO_TABLE_CENTER_Y", "-0.72")),
    float(os.environ.get("MUJOCO_TABLE_CENTER_Z", "0.33")),
)
TABLE_SIZE = (
    float(os.environ.get("MUJOCO_TABLE_SIZE_X", SCENE_CONFIG.get("table_x", "0.40"))),
    float(os.environ.get("MUJOCO_TABLE_SIZE_Y", SCENE_CONFIG.get("table_y", "0.30"))),
    float(os.environ.get("MUJOCO_TABLE_SIZE_Z", "1.00")),
)
GRASP_OBJECT_SIZE = (
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_X", "0.04")),
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_Y", "0.04")),
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_Z", "0.027")),
)
GRASP_OBJECT_MESH_QUAT = parse_quat_wxyz(
    os.environ.get("MUJOCO_GRASP_OBJECT_MESH_QUAT"),
    # Flip the rotor upside down: the current visible top face becomes the
    # tabletop-facing bottom, and the previous bottom face becomes the top.
    (0.0, 1.0, 0.0, 0.0),
)
GRASP_OBJECT_MESH = Path(
    os.environ.get("MUJOCO_GRASP_OBJECT_MESH", str(DEFAULT_GRASP_OBJECT_MESH))
).expanduser().resolve()
GRASP_OBJECT_MESH_SCALE = float(
    os.environ.get("MUJOCO_GRASP_OBJECT_MESH_SCALE", "0.001")
)
GRASP_OBJECT_MESH_CENTER = (
    float(os.environ.get("MUJOCO_GRASP_OBJECT_MESH_CENTER_X", "0.0")),
    float(os.environ.get("MUJOCO_GRASP_OBJECT_MESH_CENTER_Y", "0.0")),
    float(os.environ.get("MUJOCO_GRASP_OBJECT_MESH_CENTER_Z", "0.0113")),
)
GRASP_OBJECT_CLEARANCE = float(
    os.environ.get("MUJOCO_GRASP_OBJECT_CLEARANCE", "0.002")
)
DEFAULT_GRASP_OBJECT_Z = (
    TABLE_CENTER[2]
    + TABLE_SIZE[2] * 0.5
    + GRASP_OBJECT_SIZE[2] * 0.5
    + GRASP_OBJECT_CLEARANCE
)
GRASP_OBJECT_CENTER = (
    float(os.environ.get("MUJOCO_GRASP_TARGET_X", SCENE_CONFIG.get("object_x", "0.0"))),
    float(os.environ.get("MUJOCO_GRASP_TARGET_Y", SCENE_CONFIG.get("object_y", "-0.72"))),
    float(os.environ.get("MUJOCO_GRASP_TARGET_Z", str(DEFAULT_GRASP_OBJECT_Z))),
)
CALIBRATION_BLOCK_CENTER_XY = (
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_X", SCENE_CONFIG.get("calibration_block_x", "0.13"))),
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_Y", SCENE_CONFIG.get("calibration_block_y", "-0.62"))),
)
CALIBRATION_BLOCK_SIZE = (
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_SIZE_X", "0.10")),
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_SIZE_Y", "0.08")),
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_SIZE_Z", "0.05")),
)


def add(a, b):
    return tuple(a[index] + b[index] for index in range(3))


def rotate_x(vector, angle):
    x, y, z = vector
    cosine, sine = math.cos(angle), math.sin(angle)
    return (x, cosine * y - sine * z, sine * y + cosine * z)


def quat_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quat_rotate(quaternion, vector):
    pure = (0.0, *vector)
    conjugate = (
        quaternion[0],
        -quaternion[1],
        -quaternion[2],
        -quaternion[3],
    )
    return quat_multiply(quat_multiply(quaternion, pure), conjugate)[1:]


def head_d455_pose():
    """Return MuJoCo world pose and optical axis from the fixed URDF chain."""
    world_quat = rpy_quat(WORLD_TO_BASE_ROLL, 0.0, 0.0)
    mount_quat = rpy_quat(*HEAD_MOUNT_RPY)
    world_mount_quat = quat_multiply(world_quat, mount_quat)
    position = add(
        quat_rotate(world_quat, HEAD_MOUNT_XYZ),
        quat_rotate(world_mount_quat, HEAD_D455_XYZ),
    )
    # The D455 mesh frame is reversed by the mount's 180-degree Y rotation;
    # keep the virtual optical frame aimed toward the robot workspace.
    camera_roll = WORLD_TO_BASE_ROLL - HEAD_D455_RPY[0]
    quaternion = (
        0.0,
        0.0,
        math.sin(camera_roll / 2.0),
        math.cos(camera_roll / 2.0),
    )
    x, y, z = rotate_x((0.0, 0.0, -1.0), camera_roll)
    optical_axis = (-x, -y, z)
    position = add(
        position,
        tuple(value * D455_OPTICAL_CENTER_FORWARD for value in optical_axis),
    )
    return position, quaternion, optical_axis


def vec(values):
    return " ".join(f"{value:.8g}" for value in values)


def add_scene(root, worldbody):
    if not GRASP_OBJECT_MESH.is_file():
        raise FileNotFoundError(
            f"MuJoCo grasp object mesh not found: {GRASP_OBJECT_MESH}"
        )
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", {
        "name": "grasp_object_part_mesh",
        "file": str(GRASP_OBJECT_MESH),
        "scale": vec((GRASP_OBJECT_MESH_SCALE,) * 3),
    })
    if SLOT_MESH.is_file():
        ET.SubElement(asset, "mesh", {
            "name": "stator_slot_mesh",
            "file": str(SLOT_MESH),
            "scale": vec((SLOT_MESH_SCALE,) * 3),
        })

    head_position, head_quaternion, optical_axis = head_d455_pose()
    ET.SubElement(worldbody, "camera", {
        "name": "head_d455_rgb",
        "mode": "fixed",
        "pos": vec(head_position),
        "quat": vec(head_quaternion),
        "fovy": "60",
    })
    ET.SubElement(worldbody, "camera", {
        "name": "overview_camera",
        "mode": "fixed",
        "pos": "3.2 0.4 2.5",
        "xyaxes": "-0.325 0.946 0 -0.446 -0.153 0.881",
        "fovy": "52",
    })
    worldbody.append(ET.Comment(
        f"head_d455_rgb optical axis in world: {vec(optical_axis)}"
    ))

    # Imported URDF mesh geoms use generic contact defaults.  Give both gripper
    # mechanisms a higher-friction, torsional contact model so a kinematically
    # driven finger pair can retain the free object during lift.
    for body in worldbody.iter("body"):
        if "_Gripper_" not in body.get("name", ""):
            continue
        for geom in body.findall("geom"):
            geom.set("friction", "2.0 0.03 0.003")
            geom.set("condim", "4")
            geom.set("solref", "0.01 1")
            geom.set("solimp", "0.95 0.99 0.001")

    table = ET.SubElement(worldbody, "body", {
        "name": "work_table",
        "pos": vec(TABLE_CENTER),
    })
    ET.SubElement(table, "geom", {
        "name": "work_table_collision",
        "type": "box",
        "size": vec(tuple(value * 0.5 for value in TABLE_SIZE)),
        "rgba": "0.82 0.84 0.86 1",
        "friction": "1.0 0.01 0.001",
        # Keep dynamic parts visually on the tabletop instead of allowing the
        # default soft contact to settle a fraction of a millimetre inside it.
        "solref": "0.002 1",
        "solimp": "0.99 0.999 0.0001",
        "contype": "1",
        "conaffinity": "1",
    })
    # Invisible robot-only proximity layer.  Its collision bit does not match
    # the free object's bit, so the stator still rests on the physical tabletop
    # while robot links trigger the hard interlock 3 mm before penetration.
    ET.SubElement(table, "geom", {
        "name": "work_table_robot_guard",
        "type": "box",
        "size": vec(tuple(value * 0.5 for value in TABLE_SIZE)),
        "rgba": "0 0 0 0",
        "margin": "0.003",
        "contype": "4",
        "conaffinity": "1",
    })

    item = ET.SubElement(worldbody, "body", {
        "name": "grasp_object",
        "pos": vec(GRASP_OBJECT_CENTER),
    })
    ET.SubElement(item, "freejoint", {"name": "grasp_object_free"})
    ET.SubElement(item, "site", {
        "name": "grasp_object_site",
        "pos": "0 0 0",
        "quat": "1 0 0 0",
        "size": "0.002",
        "rgba": "0 0 0 0",
    })
    ET.SubElement(item, "geom", {
        "name": "grasp_object_geom",
        "type": "mesh",
        "mesh": "grasp_object_part_mesh",
        "pos": vec(tuple(-value for value in quat_rotate(
            GRASP_OBJECT_MESH_QUAT,
            GRASP_OBJECT_MESH_CENTER,
        ))),
        "quat": vec(GRASP_OBJECT_MESH_QUAT),
        "mass": "0.08",
        "rgba": "0.78 0.06 0.05 1",
        "friction": "1.8 0.03 0.003",
        "condim": "4",
        "solref": "0.01 1",
        "solimp": "0.95 0.99 0.001",
        "contype": "2",
        "conaffinity": "1",
    })

    # 可选场景物体 (固定在桌面, 不可抓取)
    configured_objects = SCENE_CONFIG.get("objects", [])
    default_objects = ",".join(configured_objects) if isinstance(configured_objects, list) else str(configured_objects)
    scene_objects = os.environ.get("MUJOCO_SCENE_OBJECTS", default_objects).strip()
    scene_object_set = {s.strip() for s in scene_objects.split(",") if s.strip()}
    table_z = TABLE_CENTER[2]
    table_half_z = TABLE_SIZE[2] * 0.5

    plate_visible = "plate" in scene_object_set
    plate_x = float(os.environ.get("MUJOCO_PLATE_X", SCENE_CONFIG.get("plate_x", -0.25)))
    plate_y = float(os.environ.get("MUJOCO_PLATE_Y", SCENE_CONFIG.get("plate_y", -0.68)))
    plate = ET.SubElement(worldbody, "body", {
        "name": "stator_slot_object",
        "pos": vec((plate_x, plate_y, table_z + table_half_z)),
    })
    # Fixtures always exist in the model so the Web UI can toggle them without
    # rebuilding. Hidden fixtures are both transparent and non-colliding.
    ET.SubElement(plate, "geom", {
        "name": "stator_slot_geom",
        "type": "mesh",
        "mesh": "stator_slot_mesh",
        # 先将 STL 的 Y 轴立起，再绕世界 Z 轴转 180°，使槽口朝向机械臂。
        "quat": vec((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))),
        "pos": vec((
            -SLOT_MESH_CENTER_MM[0] * SLOT_MESH_SCALE,
            SLOT_MESH_CENTER_MM[2] * SLOT_MESH_SCALE,
            -SLOT_MESH_CENTER_MM[1] * SLOT_MESH_SCALE,
        )),
        "mass": "0.3",
        "rgba": f"0.72 0.74 0.78 {1 if plate_visible else 0}",
        "friction": "1.0 0.01 0.001",
        "condim": "4",
        "solref": "0.004 1",
        "solimp": "0.98 0.995 0.0005",
        "contype": "1" if plate_visible else "0",
        "conaffinity": "1" if plate_visible else "0",
    })

    if "cylinder" in scene_object_set:
        cyl = ET.SubElement(worldbody, "body", {
            "name": "cylinder_object",
            "pos": vec((TABLE_CENTER[0] + 0.22, TABLE_CENTER[1] + 0.08, table_z + table_half_z + 0.04)),
        })
        ET.SubElement(cyl, "geom", {
            "name": "cylinder_geom",
            "type": "cylinder",
            "size": "0.025 0.04",
            "mass": "0.15",
            "rgba": "0.2 0.75 0.3 1",
            "friction": "1.2 0.01 0.001",
            "contype": "1",
            "conaffinity": "1",
        })

    block_visible = "calibration_block" in scene_object_set
    block = ET.SubElement(worldbody, "body", {
        "name": "calibration_block",
        "pos": vec((
            CALIBRATION_BLOCK_CENTER_XY[0],
            CALIBRATION_BLOCK_CENTER_XY[1],
            table_z + table_half_z + CALIBRATION_BLOCK_SIZE[2] * 0.5,
        )),
    })
    ET.SubElement(block, "geom", {
        "name": "calibration_block_geom",
        "type": "box",
        "size": vec(tuple(value * 0.5 for value in CALIBRATION_BLOCK_SIZE)),
        "mass": "0.5",
        "rgba": f"0.16 0.43 0.52 {1 if block_visible else 0}",
        "friction": "1.2 0.01 0.001",
        "condim": "4",
        "solref": "0.004 1",
        "solimp": "0.98 0.995 0.0005",
        "contype": "1" if block_visible else "0",
        "conaffinity": "1" if block_visible else "0",
    })

    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    for side in ("Right", "Left"):
        link7 = next(
            (body for body in worldbody.iter("body")
             if body.get("name") == f"{side}_Link7"),
            None,
        )
        if link7 is None:
            raise RuntimeError(f"saved MuJoCo model has no {side}_Link7 body")
        ET.SubElement(link7, "site", {
            "name": f"{side.lower()}_grasp_site",
            # *_Gripper_TCP is a fixed link and is fused into Link7 by the
            # MuJoCo URDF compiler.  Its URDF origin is +0.34 m on Link7 X.
            "pos": "0.34 0 0",
            "quat": "1 0 0 0",
            "size": "0.002",
            "rgba": "0 0 0 0",
        })
        housing_position, housing_quat = wrist_housing_transform(side)
        housing = ET.SubElement(link7, "body", {
            "name": f"{side.lower()}_wrist_rgbd_housing",
            "pos": vec(housing_position),
            "quat": vec(housing_quat),
        })
        ET.SubElement(housing, "geom", {
            "name": f"{side.lower()}_wrist_rgbd_body",
            "type": "box",
            "size": "0.009 0.028 0.022",
            "rgba": "0.10 0.13 0.15 1",
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
        })
        for lens_name, lens_y in (("depth", -0.014), ("rgb", 0.014)):
            ET.SubElement(housing, "geom", {
                "name": f"{side.lower()}_wrist_rgbd_{lens_name}_lens",
                "type": "cylinder",
                "pos": vec((0.011, lens_y, 0.0)),
                "quat": "0.7071067812 0 0.7071067812 0",
                "size": "0.0065 0.002",
                "rgba": "0.01 0.08 0.14 1",
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
            })
        wrist_camera_position, wrist_camera_quat = wrist_camera_transform(side)
        ET.SubElement(link7, "camera", {
            "name": f"{side.lower()}_wrist_rgb",
            "mode": "fixed",
            "pos": vec(wrist_camera_position),
            "quat": vec(wrist_camera_quat),
            "fovy": "70",
        })
        ET.SubElement(equality, "weld", {
            "name": f"{side.lower()}_grasp_weld",
            "site1": f"{side.lower()}_grasp_site",
            "site2": "grasp_object_site",
            "active": "false",
            "solref": "0.02 1",
            "solimp": "0.9 0.95 0.001",
            "torquescale": "0.03",
        })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    # Load from URDF/XML, not MJB: mj_saveLastXML needs the compiler's XML
    # representation in order to emit editable native MJCF.
    model = mujoco.MjModel.from_xml_path(str(args.source.resolve()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Keep the intermediate XML beside the source model so relative mesh paths
    # emitted by mj_saveLastXML continue to resolve against asm0003/mujoco.
    with tempfile.NamedTemporaryFile(
        prefix=".unoarm_camera_scene_", suffix=".xml", dir=args.source.parent,
        delete=False,
    ) as tmp:
        xml_path = Path(tmp.name)
    try:
        mujoco.mj_saveLastXML(str(xml_path), model)
        tree = ET.parse(xml_path)
        root = tree.getroot()
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise RuntimeError("saved MuJoCo model has no worldbody")
        add_scene(root, worldbody)
        tree.write(xml_path, encoding="utf-8", xml_declaration=True)
        scene_model = mujoco.MjModel.from_xml_path(str(xml_path))
        mujoco.mj_saveModel(scene_model, str(args.output))
    finally:
        xml_path.unlink(missing_ok=True)

    print(f"MuJoCo camera scene built: {args.output}")


if __name__ == "__main__":
    main()
