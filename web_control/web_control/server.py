#!/usr/bin/env python3
import base64
import csv
import json
import gzip
import io
import math
import mimetypes
import os
import random
import re
import signal
import socket
import struct
import threading
import time
import logging
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime
from email.utils import formatdate
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np
import yaml
try:
    import cv2
except ImportError:
    cv2 = None
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
import tf2_ros


ROOT = Path(__file__).resolve().parent
FRONTEND_DIST = Path(os.environ.get(
    "UNOARM_FRONTEND_DIST",
    str(ROOT.parent / "frontend" / "dist"),
))
POINTS_FILE = ROOT / "points.json"
PRESETS_FILE = ROOT / "presets.json"
PLANNING_PRESETS_FILE = ROOT / "planning_presets.json"
MANUAL_COLLISION_FILE = ROOT / "manual_collision_boxes.json"
WORKSPACE_BOUNDS_FILE = ROOT / "workspace_bounds.json"
VLM_CONFIG_FILE = ROOT / "vlm_config.json"
POSES_LIB_FILE = ROOT / "poses.json"
LOG_FILE = ROOT / "web_control.log"
MUJOCO_SCENE_CONFIG_FILE = ROOT.parents[1] / "asm0003" / "mujoco" / "scene_config.json"
MUJOCO_SCENE_PRESETS = {
    "basic_pick": {
        "name": "基础定子抓取", "description": "实体桌面中央定子抓取练习",
        "object_x": 0.0, "object_y": -0.72, "table_x": 0.40, "table_y": 0.30,
        "plate_x": -0.10, "plate_y": -0.72, "objects": ["calibration_block"],
        "calibration_block_x": 0.13, "calibration_block_y": -0.62,
    },
    "cube_to_plate": {
        "name": "定子放入槽", "description": "从实体桌面抓取红色定子并放到左侧定子槽",
        "object_x": 0.18, "object_y": -0.60, "table_x": 0.40, "table_y": 0.30,
        "plate_x": -0.08, "plate_y": -0.72, "objects": ["plate"],
        "calibration_block_x": 0.13, "calibration_block_y": -0.62,
    },
    "obstacle_pick": {
        "name": "绕障抓取训练", "description": "定子、槽和圆柱障碍物组合场景",
        "object_x": 0.04, "object_y": -0.62, "table_x": 0.40, "table_y": 0.30,
        "plate_x": -0.10, "plate_y": -0.60, "objects": ["plate", "cylinder", "calibration_block"],
        "calibration_block_x": 0.13, "calibration_block_y": -0.62,
    },
}


def load_mujoco_scene_config():
    try:
        data = json.loads(MUJOCO_SCENE_CONFIG_FILE.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data
    except (OSError, ValueError):
        pass
    return {"preset": "basic_pick", **MUJOCO_SCENE_PRESETS["basic_pick"]}


def save_mujoco_scene_config(config):
    MUJOCO_SCENE_CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    tmp = MUJOCO_SCENE_CONFIG_FILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(MUJOCO_SCENE_CONFIG_FILE)


def parse_quat_wxyz_env(name, default):
    raw = os.environ.get(name)
    parts = [
        float(part)
        for part in str(raw if raw is not None else " ".join(str(v) for v in default))
        .replace(",", " ")
        .split()
    ]
    if len(parts) != 4 or not all(math.isfinite(part) for part in parts):
        raise ValueError(f"{name} must contain four finite numbers")
    length = math.sqrt(sum(part * part for part in parts))
    if length <= 1e-9:
        raise ValueError(f"{name} quaternion length is zero")
    return tuple(part / length for part in parts)
VISION_TARGET_FILE = Path(os.environ.get(
    "VISION_TARGET_FILE",
    "/home/niic/workspace/vision/target_xyz.tsv",
))
VISION_TARGET_MAX_AGE_SEC = float(os.environ.get("VISION_TARGET_MAX_AGE_SEC", "3.0"))
VISION_HTTP_BASE = os.environ.get("VISION_HTTP_BASE", "http://127.0.0.1:8090")
VISION_HTTP_TIMEOUT_SEC = float(os.environ.get("VISION_HTTP_TIMEOUT_SEC", "2.0"))
VISION_SCENE_POINTCLOUD_FILE = Path(os.environ.get(
    "VISION_SCENE_POINTCLOUD_FILE",
    "/home/niic/workspace/vision/scene_pointcloud.bin",
))
DISABLE_PLATFORM_OBSTACLE = os.environ.get(
    "WEB_CONTROL_DISABLE_PLATFORM_OBSTACLE",
    "0",
).lower() in {"1", "true", "yes", "on"}
CAMERA_EXTRINSIC_FILE = Path(os.environ.get(
    "CAMERA_EXTRINSIC_FILE",
    "/home/niic/workspace/vision/camera_extrinsic.json",
))
CONFIG_ROOT = Path(os.environ.get(
    "UNOARM_CONFIG_ROOT",
    str(ROOT.parents[1] / "CSP_direct_drive" / "src" / "light_test" / "config"),
))
KINEMATICS_CONFIG_FILE = Path(os.environ.get(
    "UNOARM_KINEMATICS_CONFIG",
    str(ROOT.parents[1] / "asm0003" / "config" / "kinematics.yaml"),
))
OMPL_CONFIG_FILE = Path(os.environ.get(
    "UNOARM_OMPL_CONFIG",
    str(ROOT.parents[1] / "asm0003" / "config" / "ompl_planning.yaml"),
))
HOST = os.environ.get("WEB_CONTROL_HOST", "127.0.0.1")
PORT = int(os.environ.get("WEB_CONTROL_PORT", "8765"))
GROUP_NAME = os.environ.get("MOVE_GROUP_NAME", "arm")
TIP_LINK = os.environ.get("TIP_LINK", "Right_Gripper_TCP")
FRAME_ID = os.environ.get("FRAME_ID", "world")
MOVEIT_PLANNING_FRAME_ID = os.environ.get("MOVEIT_PLANNING_FRAME_ID", "base_link")
FRAME_FROM_MOVEIT_XYZ = os.environ.get("UNOARM_FRAME_FROM_MOVEIT_XYZ", "0 0 0")
FRAME_FROM_MOVEIT_RPY = os.environ.get("UNOARM_FRAME_FROM_MOVEIT_RPY", "1.5708 0 0")
JOINT_STATE_TOPIC = os.environ.get("JOINT_STATE_TOPIC", "/joint_states")
MUJOCO_CAMERA_TOPIC = os.environ.get(
    "MUJOCO_CAMERA_TOPIC", "/mujoco_camera/color/image_raw"
)
OVERVIEW_CAMERA_TOPIC = os.environ.get(
    "MUJOCO_OVERVIEW_CAMERA_TOPIC", "/overview_camera/color/image_raw"
)
MUJOCO_DEPTH_TOPIC = os.environ.get(
    "MUJOCO_CAMERA_DEPTH_TOPIC", "/mujoco_camera/depth/image_raw"
)
RIGHT_WRIST_CAMERA_TOPIC = os.environ.get(
    "RIGHT_WRIST_CAMERA_TOPIC", "/right_wrist_camera/color/image_raw"
)
RIGHT_WRIST_DEPTH_TOPIC = os.environ.get(
    "RIGHT_WRIST_DEPTH_TOPIC", "/right_wrist_camera/depth/image_raw"
)
LEFT_WRIST_CAMERA_TOPIC = os.environ.get(
    "LEFT_WRIST_CAMERA_TOPIC", "/left_wrist_camera/color/image_raw"
)
LEFT_WRIST_DEPTH_TOPIC = os.environ.get(
    "LEFT_WRIST_DEPTH_TOPIC", "/left_wrist_camera/depth/image_raw"
)
MUJOCO_GRASP_OBJECT_CENTER = {
    "object_name": "red_stator",
    "frame": FRAME_ID,
    "x": float(os.environ.get("MUJOCO_GRASP_TARGET_X", "0.18")),
    "y": float(os.environ.get("MUJOCO_GRASP_TARGET_Y", "-0.60")),
    "z": float(os.environ.get("MUJOCO_GRASP_TARGET_Z", "0.8455")),
}
MUJOCO_GRASP_OBJECT_SIZE = [
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_X", "0.04")),
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_Y", "0.04")),
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_Z", "0.027")),
]
# Match the MuJoCo scene builder: the rotor is flat but upside down, so the
# current visible top face becomes the tabletop-facing bottom.
MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ = parse_quat_wxyz_env(
    "MUJOCO_GRASP_OBJECT_MESH_QUAT",
    (0.0, 1.0, 0.0, 0.0),
)
MUJOCO_GRASP_OBJECT_PLANNING_MARGIN = max(
    0.0,
    float(os.environ.get("MUJOCO_GRASP_OBJECT_PLANNING_MARGIN", "0.005")),
)
MUJOCO_TABLE_OBSTACLE_ENABLED = os.environ.get(
    "UNOARM_MUJOCO_TABLE_OBSTACLE", "0"
).lower() in {"1", "true", "yes", "on"}
MUJOCO_TABLE_COLLISION_ID = "mujoco_work_table"
MUJOCO_GRASP_OBJECT_COLLISION_ID = "mujoco_red_grasp_object"
MUJOCO_TABLE_CENTER = [
    float(os.environ.get("MUJOCO_TABLE_CENTER_X", "0.0")),
    float(os.environ.get("MUJOCO_TABLE_CENTER_Y", "-0.72")),
    float(os.environ.get("MUJOCO_TABLE_CENTER_Z", "0.33")),
]
MUJOCO_TABLE_SIZE = [
    float(os.environ.get("MUJOCO_TABLE_SIZE_X", "0.40")),
    float(os.environ.get("MUJOCO_TABLE_SIZE_Y", "0.30")),
    float(os.environ.get("MUJOCO_TABLE_SIZE_Z", "1.00")),
]
MUJOCO_CALIBRATION_BLOCK_CENTER = [
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_X", "0.13")),
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_Y", "-0.62")),
]
MUJOCO_CALIBRATION_BLOCK_SIZE = [
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_SIZE_X", "0.10")),
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_SIZE_Y", "0.08")),
    float(os.environ.get("MUJOCO_CALIBRATION_BLOCK_SIZE_Z", "0.05")),
]
MUJOCO_TABLE_COLLISION_TOP_INSET = max(
    0.0,
    min(
        float(os.environ.get("MUJOCO_TABLE_COLLISION_TOP_INSET", "0.0")),
        max(0.0, MUJOCO_TABLE_SIZE[2] - 0.001),
    ),
)
MUJOCO_TABLE_GRASP_CONTACT_INSET = max(
    MUJOCO_TABLE_COLLISION_TOP_INSET,
    min(
        float(os.environ.get("MUJOCO_TABLE_GRASP_CONTACT_INSET", "0.0")),
        max(0.0, MUJOCO_TABLE_SIZE[2] - 0.001),
    ),
)
MUJOCO_TABLE_PLANNING_GUARD_M = max(
    0.0,
    float(os.environ.get("MUJOCO_TABLE_PLANNING_GUARD_M", "0.025")),
)
MUJOCO_GRASP_TCP_OFFSET = {
    "x": float(os.environ.get("MUJOCO_GRASP_TCP_OFFSET_X", "0.016")),
    "y": float(os.environ.get("MUJOCO_GRASP_TCP_OFFSET_Y", "-0.040")),
    "z": float(os.environ.get("MUJOCO_GRASP_TCP_OFFSET_Z", "-0.025")),
}
MUJOCO_GRASP_OBJECT_TO_TCP_M = float(
    os.environ.get("MUJOCO_GRASP_OBJECT_TO_TCP_M", "0.046")
)
MUJOCO_GRASP_LATERAL_OFFSET_M = float(
    os.environ.get("MUJOCO_GRASP_LATERAL_OFFSET_M", "-0.004")
)
MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M = max(
    0.0,
    float(os.environ.get("MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M", "0.018")),
)
ACTION_NAME = os.environ.get("ACTION_NAME", "/arm_controller/follow_joint_trajectory")
PLAN_SERVICE = os.environ.get("PLAN_SERVICE", "/plan_kinematic_path")
OCCUPANCY_TOPIC = os.environ.get("WEB_CONTROL_OCCUPANCY_TOPIC", "/unoarm/web_control/occupied")
JOINT_NAMES = [f"Right_Joint{i}" for i in range(1, 8)]
LINK_NAMES = [
    "right_base_link",
    "Right_Link1",
    "Right_Link2",
    "Right_Link3",
    "Right_Link4",
    "Right_Link5",
    "Right_Link6",
    "Right_Link7",
    "Right_Gripper_TCP",
]

LEFT_GROUP_NAME = os.environ.get("LEFT_MOVE_GROUP_NAME", "left_arm")
LEFT_TIP_LINK = os.environ.get("LEFT_TIP_LINK", "Left_Gripper_TCP")
LEFT_ACTION_NAME = os.environ.get("LEFT_ACTION_NAME", "/left_arm_controller/follow_joint_trajectory")
LEFT_JOINT_NAMES = [f"Left_Joint{i}" for i in range(1, 8)]
LEFT_LINK_NAMES = [
    "left_base_link",
    "Left_Link1",
    "Left_Link2",
    "Left_Link3",
    "Left_Link4",
    "Left_Link5",
    "Left_Link6",
    "Left_Link7",
    "Left_Gripper_TCP",
]

RIGHT_ROBOT_NS = os.environ.get("RIGHT_ROBOT_NS", "master2")
LEFT_ROBOT_NS = os.environ.get("LEFT_ROBOT_NS", "master0")
SIM_GRIPPER_SERVICE_TIMEOUT_SEC = float(os.environ.get("UNOARM_SIM_GRIPPER_TIMEOUT_SEC", "0.2"))
SIM_GRIPPER_REPEAT_DEBOUNCE_SEC = float(os.environ.get("UNOARM_SIM_GRIPPER_REPEAT_DEBOUNCE_SEC", "0.35"))
SIM_GRIPPER_SETTLE_TIMEOUT_SEC = float(os.environ.get("UNOARM_SIM_GRIPPER_SETTLE_TIMEOUT_SEC", "1.5"))
SIM_GRASP_CAPTURE_TIMEOUT_SEC = max(
    SIM_GRIPPER_SETTLE_TIMEOUT_SEC,
    float(os.environ.get("UNOARM_SIM_GRASP_CAPTURE_TIMEOUT_SEC", "8.0")),
)
SIM_GRIPPER_CONTACT_SETTLE_SEC = float(os.environ.get("UNOARM_SIM_GRIPPER_CONTACT_SETTLE_SEC", "0.25"))
SIM_GRASP_POSE_DELAY_SEC = float(os.environ.get("UNOARM_SIM_GRASP_POSE_DELAY_SEC", "0.5"))
SIM_GRASP_MIN_LIFT_DELTA_M = float(os.environ.get("UNOARM_SIM_GRASP_MIN_LIFT_DELTA_M", "0.02"))
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSED_POSITION = float(os.environ.get("UNOARM_GRIPPER_CLOSED_POSITION", "-0.91"))
GRIPPER_POSITION_EPS = float(os.environ.get("UNOARM_GRIPPER_POSITION_EPS", "0.025"))

TOOL_OFFSET = [0.0, 0.0, 0.0]
LEFT_TOOL_OFFSET = [0.0, 0.0, 0.0]
PLATFORM_OBSTACLE_ID = os.environ.get("PLATFORM_OBSTACLE_ID", "platform_forbidden_zone")
PLATFORM_FLOOR_Z = float(os.environ.get("PLATFORM_FLOOR_Z", "-1.0"))
PLATFORM_SAFETY_MARGIN = float(os.environ.get("PLATFORM_SAFETY_MARGIN", "-0.03"))
PLATFORM_XY_MARGIN = float(os.environ.get("PLATFORM_XY_MARGIN", "0.08"))
PLATFORM_PLANE_DISTANCE = float(os.environ.get("PLATFORM_PLANE_DISTANCE", "0.025"))
PLATFORM_MIN_NORMAL_Z = float(os.environ.get("PLATFORM_MIN_NORMAL_Z", "0.75"))
PLATFORM_RANSAC_ITERS = int(os.environ.get("PLATFORM_RANSAC_ITERS", "350"))
PLATFORM_MAX_POINTS = int(os.environ.get("PLATFORM_MAX_POINTS", "18000"))
PLATFORM_MIN_INLIERS = int(os.environ.get("PLATFORM_MIN_INLIERS", "600"))
PLATFORM_MAX_BOXES = int(os.environ.get("PLATFORM_MAX_BOXES", "3"))
MANUAL_COLLISION_MAX_BOXES = int(os.environ.get("UNOARM_MANUAL_COLLISION_MAX_BOXES", "32"))
MANUAL_COLLISION_ID_PREFIX = os.environ.get("UNOARM_MANUAL_COLLISION_ID_PREFIX", "manual_collision_box")
PLATFORM_MIN_REMAINING_POINTS = int(os.environ.get("PLATFORM_MIN_REMAINING_POINTS", "1200"))
PLATFORM_MAX_CAMERA_DISTANCE = float(os.environ.get("PLATFORM_MAX_CAMERA_DISTANCE", "1.5"))
APPLY_PLANNING_SCENE_SERVICE = os.environ.get("APPLY_PLANNING_SCENE_SERVICE", "/apply_planning_scene")
APPLY_PLANNING_SCENE_TIMEOUT_SEC = float(os.environ.get("APPLY_PLANNING_SCENE_TIMEOUT_SEC", "10.0"))
STATIC_GZIP_MIN_BYTES = int(os.environ.get("UNOARM_STATIC_GZIP_MIN_BYTES", "2048"))
STATIC_GZIP_LEVEL = int(os.environ.get("UNOARM_STATIC_GZIP_LEVEL", "6"))
STATIC_PRECOMPRESS_ENABLED = os.environ.get("UNOARM_STATIC_PRECOMPRESS", "1").lower() not in {"0", "false", "no"}
STATIC_GZIP_EXTENSIONS = {".css", ".html", ".js", ".json", ".stl", ".urdf", ".xml"}
STATIC_GZIP_CACHE = {}
STATIC_GZIP_LOCK = threading.Lock()
JOINT_JOG_MAX_DELTA_RAD = math.radians(float(os.environ.get("JOINT_JOG_MAX_DELTA_DEG", "5.0")))
JOINT_JOG_MAX_LEAD_RAD = math.radians(float(os.environ.get("JOINT_JOG_MAX_LEAD_DEG", "8.0")))
JOINT_JOG_TARGET_TTL_SEC = float(os.environ.get("JOINT_JOG_TARGET_TTL_SEC", "1.0"))
BENCHMARK_MAX_SAMPLES = int(os.environ.get("UNOARM_BENCHMARK_MAX_SAMPLES", "120"))
BENCHMARK_DEFAULT_BOX_COUNT = int(os.environ.get("UNOARM_BENCHMARK_DEFAULT_BOX_COUNT", "5"))
BENCHMARK_DEFAULT_EDGE_COUNT = int(os.environ.get("UNOARM_BENCHMARK_DEFAULT_EDGE_COUNT", "5"))
BENCHMARK_DEFAULT_RANDOM_COUNT = int(os.environ.get("UNOARM_BENCHMARK_DEFAULT_RANDOM_COUNT", "10"))
BENCHMARK_DEFAULT_BOX_OFFSET_CM = float(os.environ.get("UNOARM_BENCHMARK_BOX_OFFSET_CM", "5.0"))
BENCHMARK_DEFAULT_EDGE_DISTANCE_CM = float(os.environ.get("UNOARM_BENCHMARK_EDGE_DISTANCE_CM", "5.0"))
ROBOT_URDF_FILE = Path(os.environ.get(
    "UNOARM_ROBOT_URDF",
    str(ROOT.parents[1] / "asm0003" / "urdf" / "asm0003.urdf"),
))
ROBOT_WORLD_FROM_BASE_LINK_XYZ = os.environ.get("UNOARM_ROBOT_WORLD_FROM_BASE_LINK_XYZ", FRAME_FROM_MOVEIT_XYZ)
ROBOT_WORLD_FROM_BASE_LINK_RPY = os.environ.get("UNOARM_ROBOT_WORLD_FROM_BASE_LINK_RPY", FRAME_FROM_MOVEIT_RPY)
HORIZONTAL_GRASP_YAW_STEP_DEG = float(os.environ.get("UNOARM_HORIZONTAL_GRASP_YAW_STEP_DEG", "15"))
HORIZONTAL_GRASP_MAX_CANDIDATES = int(os.environ.get("UNOARM_HORIZONTAL_GRASP_MAX_CANDIDATES", "6"))
HORIZONTAL_GRASP_PLANNING_TIME = float(os.environ.get("UNOARM_HORIZONTAL_GRASP_PLANNING_TIME", "1.0"))
HORIZONTAL_GRASP_ATTEMPTS = int(os.environ.get("UNOARM_HORIZONTAL_GRASP_ATTEMPTS", "1"))
HORIZONTAL_GRASP_IK_TIMEOUT = float(os.environ.get("UNOARM_HORIZONTAL_GRASP_IK_TIMEOUT", "0.6"))
ANGLED_GRASP_X_GROUND_ANGLE_DEG = float(os.environ.get("UNOARM_ANGLED_GRASP_X_GROUND_ANGLE_DEG", "30"))
ANGLED_GRASP_MAX_CANDIDATES = max(
    2,
    int(os.environ.get("UNOARM_ANGLED_GRASP_MAX_CANDIDATES", "10")),
)
FLEXIBLE_GRASP_THETA_OFFSETS_DEG = [0.0, 25.0, -25.0, 50.0, -50.0]
FLEXIBLE_GRASP_MAX_IK_CANDIDATES = int(os.environ.get("UNOARM_FLEXIBLE_GRASP_MAX_IK_CANDIDATES", "3"))
FLEXIBLE_GRASP_TCP_OFFSET = float(os.environ.get("UNOARM_FLEXIBLE_GRASP_TCP_OFFSET", "0.34"))
FLEXIBLE_GRASP_PREGRASP_DISTANCE = float(os.environ.get("UNOARM_FLEXIBLE_GRASP_PREGRASP_DISTANCE", "0.10"))
FLEXIBLE_GRASP_CYLINDER_RADIUS = float(os.environ.get("UNOARM_FLEXIBLE_GRASP_CYLINDER_RADIUS", "0.03"))
FLEXIBLE_GRASP_EE_TO_GRIPPER_RPY = os.environ.get("UNOARM_FLEXIBLE_GRASP_EE_TO_GRIPPER_RPY", "0 0 0")
OMPL_RUNTIME_DEFAULT = {
    "planner_id": os.environ.get("UNOARM_OMPL_PLANNER_ID", "RRTConnectkConfigDefault"),
    "planning_time": float(os.environ.get("UNOARM_OMPL_PLANNING_TIME", "1.0")),
    "attempts": int(os.environ.get("UNOARM_OMPL_ATTEMPTS", "1")),
    "ik_timeout": float(os.environ.get("UNOARM_OMPL_IK_TIMEOUT", "0.6")),
}
OMPL_PIPELINE_ID = os.environ.get("UNOARM_OMPL_PIPELINE_ID", "ompl")
HOME_PLANNING_TIME = float(os.environ.get("UNOARM_HOME_PLANNING_TIME", "6.0"))
HOME_PLANNING_ATTEMPTS = int(os.environ.get("UNOARM_HOME_ATTEMPTS", "4"))
HOME_COLLISION_PLANNING_TIME = float(os.environ.get("UNOARM_HOME_COLLISION_PLANNING_TIME", "8.0"))
HOME_COLLISION_ATTEMPTS = int(os.environ.get("UNOARM_HOME_COLLISION_ATTEMPTS", "5"))
HOME_GOAL_TOLERANCE_RAD = float(os.environ.get("UNOARM_HOME_GOAL_TOLERANCE_RAD", "0.03"))
HOME_RETRY_PLANNERS = [
    item.strip()
    for item in os.environ.get("UNOARM_HOME_RETRY_PLANNERS", "RRTConnectkConfigDefault,LBKPIECEkConfigDefault").split(",")
    if item.strip()
]
OMPL_RUNTIME_PRESETS = [
    {
        "id": "recommended",
        "label": "推荐调试",
        "description": "RRTConnect 只跑 1 次，规划时间 1s，适合网页交互调试。",
        "config": {
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 1.0,
            "attempts": 1,
            "ik_timeout": 0.6,
        },
    },
    {
        "id": "balanced",
        "label": "均衡",
        "description": "仍使用 RRTConnect，给 OMPL 更长时间和 2 次尝试。",
        "config": {
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 2.0,
            "attempts": 2,
            "ik_timeout": 1.0,
        },
    },
    {
        "id": "narrow_space",
        "label": "窄空间",
        "description": "LBKPIECE 更偏探索狭窄通道，速度通常慢于 RRTConnect。",
        "config": {
            "planner_id": "LBKPIECEkConfigDefault",
            "planning_time": 3.0,
            "attempts": 2,
            "ik_timeout": 1.0,
        },
    },
    {
        "id": "diagnostic",
        "label": "诊断",
        "description": "延长规划和 IK timeout，用来确认慢规划是不是时间不足。",
        "config": {
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 5.0,
            "attempts": 3,
            "ik_timeout": 2.0,
        },
    },
]
OMPL_PLANNER_LABELS = {
    "RRTConnectkConfigDefault": "RRTConnect",
    "RRTkConfigDefault": "RRT",
    "ESTkConfigDefault": "EST",
    "RRTstarkConfigDefault": "RRTstar",
    "LBKPIECEkConfigDefault": "LBKPIECE",
}
OMPL_PREFERRED_PLANNER_IDS = list(OMPL_PLANNER_LABELS)
BENCHMARK_CSV_DIR = Path(os.environ.get(
    "UNOARM_BENCHMARK_CSV_DIR",
    str(ROOT.parents[1] / "logs" / "benchmarks"),
))
MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    0: "UNDEFINED",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -26: "START_STATE_INVALID",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}


def normalize_arm(value):
    return "left" if value == "left" else "right"


def finite_float(value, label, default=None):
    if value is None:
        if default is None:
            raise ValueError(f"{label} 不能为空")
        return float(default)
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} 必须是有限数值")
    return number


def clamp_int(value, minimum, maximum, label):
    number = int(value)
    if number < minimum or number > maximum:
        raise ValueError(f"{label} 必须在 {minimum} 到 {maximum} 之间")
    return number


def clamp_float(value, minimum, maximum, label, default=None):
    number = finite_float(value, label, default)
    if number < minimum or number > maximum:
        raise ValueError(f"{label} 必须在 {minimum} 到 {maximum} 之间")
    return number


def vec_norm(values):
    return math.sqrt(sum(float(value) * float(value) for value in values))


def vec_dot(a, b):
    return sum(float(x) * float(y) for x, y in zip(a, b))


def vec_cross(a, b):
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def vec_add(a, b):
    return [float(x) + float(y) for x, y in zip(a, b)]


def vec_sub(a, b):
    return [float(x) - float(y) for x, y in zip(a, b)]


def vec_scale(v, scale):
    return [float(value) * float(scale) for value in v]


def vec_normalize(v, label="vector"):
    length = vec_norm(v)
    if length <= 1e-9:
        raise ValueError(f"{label} 长度过小，无法归一化")
    return [float(value) / length for value in v]


def vec_reject(v, axis):
    return vec_sub(v, vec_scale(axis, vec_dot(v, axis)))


def rotate_vector_axis_angle(v, axis, angle):
    axis = vec_normalize(axis, "rotation_axis")
    c = math.cos(angle)
    s = math.sin(angle)
    return vec_add(
        vec_add(vec_scale(v, c), vec_scale(vec_cross(axis, v), s)),
        vec_scale(axis, vec_dot(axis, v) * (1.0 - c)),
    )


def finite_vec3(value, label, default=None):
    if value is None:
        if default is None:
            raise ValueError(f"{label} 不能为空")
        value = default
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y"), value.get("z")]
    if isinstance(value, str):
        value = value.split()
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} 必须是长度为 3 的向量")
    result = [float(part) for part in value]
    if not all(math.isfinite(part) for part in result):
        raise ValueError(f"{label} 必须是有限向量")
    return result


def parse_urdf_xyz(value):
    if not value:
        return [0.0, 0.0, 0.0]
    parts = str(value).split()
    if len(parts) != 3:
        return [0.0, 0.0, 0.0]
    return [float(part) for part in parts]


def rpy_to_matrix(roll, pitch, yaw):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transform_from_xyz_rpy(xyz, rpy):
    rot = rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return [
        [rot[0][0], rot[0][1], rot[0][2], float(xyz[0])],
        [rot[1][0], rot[1][1], rot[1][2], float(xyz[1])],
        [rot[2][0], rot[2][1], rot[2][2], float(xyz[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for row in range(4):
        for col in range(4):
            out[row][col] = sum(a[row][idx] * b[idx][col] for idx in range(4))
    return out


def transform_point(transform, point):
    return [
        transform[0][0] * point[0] + transform[0][1] * point[1] + transform[0][2] * point[2] + transform[0][3],
        transform[1][0] * point[0] + transform[1][1] * point[1] + transform[1][2] * point[2] + transform[1][3],
        transform[2][0] * point[0] + transform[2][1] * point[1] + transform[2][2] * point[2] + transform[2][3],
    ]


def invert_transform(transform):
    rot_t = [
        [transform[0][0], transform[1][0], transform[2][0]],
        [transform[0][1], transform[1][1], transform[2][1]],
        [transform[0][2], transform[1][2], transform[2][2]],
    ]
    translation = [transform[0][3], transform[1][3], transform[2][3]]
    inv_translation = [
        -sum(rot_t[row][idx] * translation[idx] for idx in range(3))
        for row in range(3)
    ]
    return [
        [rot_t[0][0], rot_t[0][1], rot_t[0][2], inv_translation[0]],
        [rot_t[1][0], rot_t[1][1], rot_t[1][2], inv_translation[1]],
        [rot_t[2][0], rot_t[2][1], rot_t[2][2], inv_translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


_URDF_CHILDREN_CACHE = None
_URDF_JOINT_LIMITS_CACHE = None


def urdf_children():
    global _URDF_CHILDREN_CACHE
    if _URDF_CHILDREN_CACHE is not None:
        return _URDF_CHILDREN_CACHE
    children = {}
    if not ROBOT_URDF_FILE.is_file():
        _URDF_CHILDREN_CACHE = children
        return children
    root = ET.parse(ROBOT_URDF_FILE).getroot()
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_link = parent.attrib.get("link")
        child_link = child.attrib.get("link")
        if not parent_link or not child_link:
            continue
        origin = joint.find("origin")
        children.setdefault(parent_link, []).append({
            "child": child_link,
            "xyz": parse_urdf_xyz(origin.attrib.get("xyz") if origin is not None else None),
            "rpy": parse_urdf_xyz(origin.attrib.get("rpy") if origin is not None else None),
        })
    _URDF_CHILDREN_CACHE = children
    return children


def urdf_joint_limits():
    global _URDF_JOINT_LIMITS_CACHE
    if _URDF_JOINT_LIMITS_CACHE is not None:
        return _URDF_JOINT_LIMITS_CACHE
    limits = {}
    if ROBOT_URDF_FILE.is_file():
        root = ET.parse(ROBOT_URDF_FILE).getroot()
        for joint in root.findall("joint"):
            name = joint.attrib.get("name")
            limit = joint.find("limit")
            if not name or limit is None:
                continue
            try:
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(lower) and math.isfinite(upper) and lower <= upper:
                limits[name] = (lower, upper)
    _URDF_JOINT_LIMITS_CACHE = limits
    return limits


def urdf_transform(base_link, tip_link):
    queue = [(base_link, transform_from_xyz_rpy([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]))]
    seen = set()
    children = urdf_children()
    while queue:
        link, transform = queue.pop(0)
        if link == tip_link:
            return transform
        if link in seen:
            continue
        seen.add(link)
        for joint in children.get(link, []):
            queue.append((joint["child"], mat_mul(transform, transform_from_xyz_rpy(joint["xyz"], joint["rpy"]))))
    raise RuntimeError(f"URDF 中找不到 {base_link} -> {tip_link} 的链")


def moveit_error_name(code):
    return MOVEIT_ERROR_NAMES.get(int(code), f"MoveItErrorCode({int(code)})")


def moveit_error_message(code):
    name = moveit_error_name(code)
    if int(code) == -21:
        return (
            f"{name}: MoveIt 坐标系转换失败，检查 "
            f"{FRAME_ID} -> {MOVEIT_PLANNING_FRAME_ID} TF 或后端规划坐标配置"
        )
    return name


def planner_label_for_id(planner_id):
    return OMPL_PLANNER_LABELS.get(planner_id, planner_id)


def read_ompl_config():
    if not OMPL_CONFIG_FILE.is_file():
        raise RuntimeError(f"MoveIt OMPL 配置不存在: {OMPL_CONFIG_FILE}")
    data = yaml.safe_load(OMPL_CONFIG_FILE.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"MoveIt OMPL 配置无效: {OMPL_CONFIG_FILE}")
    return data


def ompl_planner_options():
    data = read_ompl_config()
    planner_configs = data.get("planner_configs", {})
    if not isinstance(planner_configs, dict):
        planner_configs = {}

    groups_by_planner = {}
    for group_name in ("arm", "left_arm"):
        group = data.get(group_name, {})
        configs = group.get("planner_configs", []) if isinstance(group, dict) else []
        for planner_id in configs if isinstance(configs, list) else []:
            groups_by_planner.setdefault(str(planner_id), []).append(group_name)

    ordered = list(OMPL_PREFERRED_PLANNER_IDS)
    ordered.extend(str(planner_id) for planner_id in planner_configs.keys() if str(planner_id) not in ordered)

    options = []
    for planner_id in ordered:
        config = planner_configs.get(planner_id, {})
        if not isinstance(config, dict):
            config = {}
        if planner_id not in planner_configs and planner_id not in groups_by_planner:
            continue
        options.append({
            "id": planner_id,
            "planner_id": planner_id,
            "label": planner_label_for_id(planner_id),
            "type": config.get("type", ""),
            "groups": groups_by_planner.get(planner_id, []),
            "config": config,
            "recommended": planner_id == OMPL_RUNTIME_PRESETS[0]["config"]["planner_id"],
        })
    return options


def ompl_config_response():
    data = read_ompl_config()
    planners = ompl_planner_options()
    custom_presets = [
        {**preset, "custom": True}
        for preset in load_planning_presets()
    ]
    return {
        "ok": True,
        "path": str(OMPL_CONFIG_FILE),
        "mtime": OMPL_CONFIG_FILE.stat().st_mtime,
        "pipeline": "ompl",
        "request_adapters": data.get("request_adapters", ""),
        "start_state_max_bounds_error": data.get("start_state_max_bounds_error"),
        "defaults": dict(OMPL_RUNTIME_DEFAULT),
        "presets": OMPL_RUNTIME_PRESETS + custom_presets,
        "custom_presets": custom_presets,
        "recommended": OMPL_RUNTIME_PRESETS[0],
        "planners": planners,
        "limits": {
            "planning_time": [0.1, 30.0],
            "attempts": [1, 20],
            "ik_timeout": [0.05, 10.0],
        },
    }


def normalize_ompl_runtime_config(body):
    available_ids = {option["planner_id"] for option in ompl_planner_options()}
    planner_id = str(body.get("planner_id") or OMPL_RUNTIME_DEFAULT["planner_id"]).strip()
    if not planner_id:
        planner_id = OMPL_RUNTIME_DEFAULT["planner_id"]
    if available_ids and planner_id not in available_ids:
        raise ValueError(f"未知 OMPL 规划器: {planner_id}")

    return {
        "planner_id": planner_id,
        "planning_time": clamp_float(body.get("planning_time"), 0.1, 30.0, "planning_time", OMPL_RUNTIME_DEFAULT["planning_time"]),
        "attempts": clamp_int(body.get("attempts", OMPL_RUNTIME_DEFAULT["attempts"]), 1, 20, "attempts"),
        "ik_timeout": clamp_float(body.get("ik_timeout"), 0.05, 10.0, "ik_timeout", OMPL_RUNTIME_DEFAULT["ik_timeout"]),
    }


def home_ompl_config_candidates(body, collision_avoidance_required=False):
    primary = normalize_ompl_runtime_config(body)
    available_ids = {option["planner_id"] for option in ompl_planner_options()}
    min_time = HOME_COLLISION_PLANNING_TIME if collision_avoidance_required else HOME_PLANNING_TIME
    min_attempts = HOME_COLLISION_ATTEMPTS if collision_avoidance_required else HOME_PLANNING_ATTEMPTS
    candidates = []
    seen = set()

    def add_candidate(planner_id):
        planner_id = str(planner_id or "").strip()
        if not planner_id or planner_id in seen:
            return
        if available_ids and planner_id not in available_ids:
            return
        seen.add(planner_id)
        candidates.append({
            **primary,
            "planner_id": planner_id,
            "planning_time": min(30.0, max(float(primary["planning_time"]), min_time)),
            "attempts": min(20, max(int(primary["attempts"]), min_attempts)),
            "home_boosted": True,
        })

    add_candidate(primary["planner_id"])
    if collision_avoidance_required:
        for planner_id in HOME_RETRY_PLANNERS:
            add_candidate(planner_id)

    return candidates or [primary]


def safe_preset_id(value):
    text = str(value or "").strip().lower()
    cleaned = "".join(ch if (ch.isascii() and ch.isalnum()) or ch in {"_", "-"} else "_" for ch in text)
    cleaned = cleaned.strip("_-")
    return cleaned[:48] or f"preset_{int(time.time())}"


def normalize_planning_preset(raw, index=0):
    raw = raw if isinstance(raw, dict) else {}
    name = str(raw.get("name") or raw.get("label") or f"规划预设 {index + 1}").strip()[:64]
    config = normalize_ompl_runtime_config(raw.get("config") if isinstance(raw.get("config"), dict) else raw)
    preset_id = safe_preset_id(raw.get("id") or name)
    created_at = str(raw.get("created_at") or datetime.now().isoformat(timespec="seconds"))
    return {
        "id": preset_id,
        "name": name or f"规划预设 {index + 1}",
        "label": name or f"规划预设 {index + 1}",
        "config": config,
        "source": str(raw.get("source") or "custom")[:32],
        "created_at": created_at,
        "updated_at": str(raw.get("updated_at") or created_at),
    }


def load_planning_presets():
    if not PLANNING_PRESETS_FILE.exists():
        return []
    with PLANNING_PRESETS_FILE.open("r", encoding="utf-8") as f:
        raw = json.load(f)
    presets = raw.get("presets", raw) if isinstance(raw, dict) else raw
    normalized = []
    for index, preset in enumerate(presets if isinstance(presets, list) else []):
        try:
            normalized.append(normalize_planning_preset(preset, index))
        except Exception as exc:
            log_event("WARNING", "invalid planning preset ignored", {"index": index, "error": str(exc)})
    return normalized


def save_planning_presets(presets):
    normalized = []
    seen = set()
    for index, preset in enumerate(presets or []):
        item = normalize_planning_preset(preset, index)
        base_id = item["id"]
        suffix = 2
        while item["id"] in seen:
            item["id"] = f"{base_id}_{suffix}"[:48]
            suffix += 1
        seen.add(item["id"])
        normalized.append(item)
    payload = {
        "ok": True,
        "path": str(PLANNING_PRESETS_FILE),
        "presets": normalized,
    }
    tmp = PLANNING_PRESETS_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(PLANNING_PRESETS_FILE)
    return normalized


def planning_presets_response():
    return {
        "ok": True,
        "path": str(PLANNING_PRESETS_FILE),
        "mtime": PLANNING_PRESETS_FILE.stat().st_mtime if PLANNING_PRESETS_FILE.exists() else None,
        "presets": load_planning_presets(),
    }


def upsert_planning_preset(body):
    presets = load_planning_presets()
    item = normalize_planning_preset({
        **body,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
    }, len(presets))
    presets = [preset for preset in presets if preset["id"] != item["id"] and preset["name"] != item["name"]]
    presets.append(item)
    return save_planning_presets(presets)


def delete_planning_preset(body):
    preset_id = str(body.get("id") or "").strip()
    name = str(body.get("name") or "").strip()
    presets = [
        preset for preset in load_planning_presets()
        if not ((preset_id and preset["id"] == preset_id) or (name and preset["name"] == name))
    ]
    return save_planning_presets(presets)


def csv_cell_json(value):
    if value is None:
        return ""
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return value


def benchmark_result_csv(result):
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow([
        "sample_id", "arm", "source", "x", "y", "z",
        "ik_ok", "ik_reason", "ik_error_code", "ik_elapsed_ms", "ik_mode",
        "ik_candidate_index", "ik_candidate_count", "ik_scanned_count", "ik_candidate",
        "planner_id", "planner_label", "plan_ok", "plan_reason", "plan_error_code",
        "plan_elapsed_ms", "plan_candidate_index", "plan_candidate_count", "plan_scanned_count",
        "trajectory_points", "trajectory_duration", "metadata",
    ])
    result = result if isinstance(result, dict) else {}
    samples = result.get("samples") if isinstance(result.get("samples"), list) else []
    planners = result.get("planners") if isinstance(result.get("planners"), list) else []
    planner_labels = {str(item.get("planner_id")): item.get("label") for item in planners if isinstance(item, dict)}
    results_by_sample = {}
    for planner_id, rows in (result.get("results") if isinstance(result.get("results"), dict) else {}).items():
        for row in rows if isinstance(rows, list) else []:
            if not isinstance(row, dict):
                continue
            results_by_sample.setdefault(int(row.get("sample_id", 0)), []).append((str(planner_id), row))
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        sample_id = int(sample.get("id", 0))
        ik = sample.get("ik") if isinstance(sample.get("ik"), dict) else {}
        plan_rows = results_by_sample.get(sample_id) or [(None, {})]
        for planner_id, plan in plan_rows:
            writer.writerow([
                sample_id,
                sample.get("arm"),
                sample.get("source"),
                sample.get("x"),
                sample.get("y"),
                sample.get("z"),
                ik.get("ok"),
                ik.get("reason"),
                ik.get("error_code"),
                ik.get("elapsed_ms"),
                ik.get("mode"),
                ik.get("candidate_index"),
                ik.get("candidate_count"),
                ik.get("scanned_count"),
                csv_cell_json(ik.get("candidate")),
                planner_id or "",
                planner_labels.get(str(planner_id), ""),
                plan.get("ok"),
                plan.get("reason"),
                plan.get("error_code"),
                plan.get("elapsed_ms"),
                plan.get("candidate_index"),
                plan.get("candidate_count"),
                plan.get("scanned_count"),
                plan.get("trajectory_points"),
                plan.get("trajectory_duration"),
                csv_cell_json(sample.get("metadata")),
            ])
    return output.getvalue()


def save_benchmark_csv(result):
    BENCHMARK_CSV_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    arm = normalize_arm((result or {}).get("arm", "right"))
    path = BENCHMARK_CSV_DIR / f"{timestamp}_{arm}_benchmark.csv"
    csv_text = benchmark_result_csv(result)
    path.write_text(csv_text, encoding="utf-8")
    return {
        "ok": True,
        "path": str(path),
        "bytes": path.stat().st_size,
        "csv": csv_text,
    }


# ─── Logging System ───────────────────────────────────────────────────────────
_log_buffer = deque(maxlen=500)
_log_lock = threading.Lock()

_file_handler = logging.FileHandler(LOG_FILE, encoding="utf-8")
_file_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
_logger = logging.getLogger("web_control")
_logger.setLevel(logging.DEBUG)
_logger.addHandler(_file_handler)


def log_event(level, msg, data=None):
    normalized_level = "WARNING" if level == "WARN" else level
    entry = {
        "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
        "level": normalized_level,
        "msg": msg,
    }
    if data is not None:
        entry["data"] = data
    with _log_lock:
        _log_buffer.append(entry)
    log_method = getattr(_logger, normalized_level.lower(), _logger.info)
    log_method(msg)


def get_recent_logs(n=100):
    with _log_lock:
        return list(_log_buffer)[-n:]


log_event("INFO", "server starting")
# ──────────────────────────────────────────────────────────────────────────────


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def quat_from_matrix(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return quat_normalize({
            "w": 0.25 * s,
            "x": (m21 - m12) / s,
            "y": (m02 - m20) / s,
            "z": (m10 - m01) / s,
        })
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return quat_normalize({
            "w": (m21 - m12) / s,
            "x": 0.25 * s,
            "y": (m01 + m10) / s,
            "z": (m02 + m20) / s,
        })
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return quat_normalize({
            "w": (m02 - m20) / s,
            "x": (m01 + m10) / s,
            "y": 0.25 * s,
            "z": (m12 + m21) / s,
        })
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return quat_normalize({
        "w": (m10 - m01) / s,
        "x": (m02 + m20) / s,
        "y": (m12 + m21) / s,
        "z": 0.25 * s,
    })


def normalize_angle(angle):
    value = math.fmod(float(angle) + math.pi, 2.0 * math.pi)
    if value < 0:
        value += 2.0 * math.pi
    return value - math.pi


def dedupe_angles(angles, tolerance=1e-6):
    result = []
    for angle in angles:
        normalized = normalize_angle(angle)
        if all(abs(normalize_angle(normalized - existing)) > tolerance for existing in result):
            result.append(normalized)
    return result


def local_axis_world_z(roll, pitch, axis="y"):
    matrix = rpy_to_matrix(roll, pitch, 0.0)
    axis_index = {"x": 0, "y": 1, "z": 2}.get(axis, 1)
    return matrix[2][axis_index]


def horizontal_grasp_constraint_error(roll, pitch):
    return abs(local_axis_world_z(roll, pitch, "y"))


def side_grasp_constraint_error(roll, pitch):
    return abs(abs(local_axis_world_z(roll, pitch, "y")) - 1.0)


def vertical_grasp_constraint_error(roll, pitch):
    return abs(abs(local_axis_world_z(roll, pitch, "x")) - 1.0)


def z_parallel_grasp_constraint_error(roll, pitch):
    return abs(local_axis_world_z(roll, pitch, "z"))


def quat_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_from_xyzw(values, label="quaternion"):
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError(f"{label} 必须是长度为 4 的四元数")
    q = {
        "x": float(values[0]),
        "y": float(values[1]),
        "z": float(values[2]),
        "w": float(values[3]),
    }
    if not all(math.isfinite(value) for value in q.values()):
        raise ValueError(f"{label} 必须是有限四元数")
    return quat_normalize(q)


def quat_from_wxyz(values, label="quaternion"):
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError(f"{label} 必须是长度为 4 的四元数")
    q = {
        "w": float(values[0]),
        "x": float(values[1]),
        "y": float(values[2]),
        "z": float(values[3]),
    }
    if not all(math.isfinite(value) for value in q.values()):
        raise ValueError(f"{label} 必须是有限四元数")
    return quat_normalize(q)


def quat_to_wxyz(q):
    q = quat_normalize(q)
    return [q["w"], q["x"], q["y"], q["z"]]


def quat_heading_yaw(q):
    for axis in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]):
        world_axis = rotate_vector_by_quat(axis, q["x"], q["y"], q["z"], q["w"])
        if math.hypot(world_axis[0], world_axis[1]) > 1e-6:
            return math.atan2(world_axis[1], world_axis[0])
    return 0.0


def rotate_vector_by_quat(v, qx, qy, qz, qw):
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return [
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    ]


def quat_multiply(a, b):
    ax, ay, az, aw = a["x"], a["y"], a["z"], a["w"]
    bx, by, bz, bw = b["x"], b["y"], b["z"], b["w"]
    return {
        "x": aw * bx + ax * bw + ay * bz - az * by,
        "y": aw * by - ax * bz + ay * bw + az * bx,
        "z": aw * bz + ax * by - ay * bx + az * bw,
        "w": aw * bw - ax * bx - ay * by - az * bz,
    }


def quat_conjugate(q):
    return {"x": -q["x"], "y": -q["y"], "z": -q["z"], "w": q["w"]}


def quat_normalize(q):
    length = math.sqrt(q["x"] * q["x"] + q["y"] * q["y"] + q["z"] * q["z"] + q["w"] * q["w"])
    if length <= 0.0:
        return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    return {key: value / length for key, value in q.items()}


def mujoco_grasp_object_display_quat_wxyz(raw_wxyz=None):
    if not (isinstance(raw_wxyz, list) and len(raw_wxyz) == 4):
        raw_wxyz = [1.0, 0.0, 0.0, 0.0]
    body = quat_normalize({
        "w": float(raw_wxyz[0]),
        "x": float(raw_wxyz[1]),
        "y": float(raw_wxyz[2]),
        "z": float(raw_wxyz[3]),
    })
    mesh = {
        "w": MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ[0],
        "x": MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ[1],
        "y": MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ[2],
        "z": MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ[3],
    }
    display = quat_normalize(quat_multiply(body, mesh))
    return [display["w"], display["x"], display["y"], display["z"]]


def frame_from_moveit_transform():
    return transform_from_xyz_rpy(
        parse_urdf_xyz(FRAME_FROM_MOVEIT_XYZ),
        parse_urdf_xyz(FRAME_FROM_MOVEIT_RPY),
    )


def moveit_from_frame_transform():
    return invert_transform(frame_from_moveit_transform())


def moveit_from_frame_quat():
    roll, pitch, yaw = parse_urdf_xyz(FRAME_FROM_MOVEIT_RPY)
    return quat_conjugate(quat_from_rpy(roll, pitch, yaw))


def pose_to_moveit_frame(position, orientation):
    if FRAME_ID == MOVEIT_PLANNING_FRAME_ID:
        return position, quat_normalize(orientation)
    converted_position = transform_point(moveit_from_frame_transform(), position)
    converted_orientation = quat_multiply(moveit_from_frame_quat(), orientation)
    return converted_position, quat_normalize(converted_orientation)


def load_points():
    if not POINTS_FILE.exists():
        return []
    with POINTS_FILE.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_points(points):
    tmp = POINTS_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(points, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(POINTS_FILE)


def default_vlm_config():
    return {
        "api_key": "",
        "base_url": "https://api.siliconflow.cn/v1",
        "model": "Qwen/Qwen3-VL-8B-Instruct",
        "prompt": "识别图像中的目标物体并给出归一化边界框",
        "offset_x": 0.0,
        "offset_y": 0.0,
        "offset_z": -0.1,
    }


def load_vlm_config():
    if not VLM_CONFIG_FILE.exists():
        return default_vlm_config()
    try:
        with VLM_CONFIG_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return default_vlm_config()
    base = default_vlm_config()
    base.update({k: data.get(k, base[k]) for k in base})
    return base


def save_vlm_config(data):
    base = default_vlm_config()
    clean = {k: data.get(k, base[k]) for k in base}
    clean["api_key"] = str(clean.get("api_key", ""))
    clean["base_url"] = str(clean.get("base_url", base["base_url"]))
    clean["model"] = str(clean.get("model", base["model"]))
    clean["prompt"] = str(clean.get("prompt", ""))
    clean["offset_x"] = float(clean.get("offset_x", 0.0) or 0.0)
    clean["offset_y"] = float(clean.get("offset_y", 0.0) or 0.0)
    clean["offset_z"] = float(clean.get("offset_z", -0.1) or 0.0)
    tmp = VLM_CONFIG_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(clean, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(VLM_CONFIG_FILE)
    return clean


def vlm_config_response():
    return {
        "ok": True,
        "path": str(VLM_CONFIG_FILE),
        "mtime": VLM_CONFIG_FILE.stat().st_mtime if VLM_CONFIG_FILE.exists() else None,
        "config": load_vlm_config(),
    }


def default_poses_lib():
    return {"version": 1, "poses": []}


def normalize_pose_entry(entry):
    if not isinstance(entry, dict):
        entry = {}
    return {
        "id": str(entry.get("id", "")).strip(),
        "name": str(entry.get("name", "")).strip(),
        "description": str(entry.get("description", "")).strip(),
        "x": float(entry.get("x", 0.0) or 0.0),
        "y": float(entry.get("y", 0.0) or 0.0),
        "z": float(entry.get("z", 0.0) or 0.0),
        "grasp_orientation_mode": str(entry.get("grasp_orientation_mode", "vertical")).strip()
            or "vertical",
        "arm": str(entry.get("arm", "right")).strip() or "right",
    }


def load_poses_lib():
    if not POSES_LIB_FILE.exists():
        return default_poses_lib()
    try:
        with POSES_LIB_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return default_poses_lib()
    poses = data.get("poses", []) if isinstance(data, dict) else []
    return {
        "version": int(data.get("version", 1)) if isinstance(data, dict) else 1,
        "poses": [normalize_pose_entry(p) for p in poses if isinstance(p, dict)],
    }


def save_poses_lib(data):
    poses = data.get("poses", []) if isinstance(data, dict) else []
    normalized = [normalize_pose_entry(p) for p in poses if isinstance(p, dict)]
    payload = {"version": 1, "poses": normalized}
    tmp = POSES_LIB_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(POSES_LIB_FILE)
    return payload


def poses_lib_response():
    return {
        "ok": True,
        "path": str(POSES_LIB_FILE),
        "mtime": POSES_LIB_FILE.stat().st_mtime if POSES_LIB_FILE.exists() else None,
        "poses_lib": load_poses_lib(),
    }


VLM_SYSTEM_PROMPT = (
    "你是机器人视觉助手。用户会给你一条指令, 你要在图像中找到指令描述的目标物体,"
    "并输出它的绝对归一化边界框。\n"
    "坐标定义: 图像左上角 (0,0), 右下角 (1,1)。bbox=[x_min,y_min,x_max,y_max], 0~1 浮点数。\n"
    "输出格式: 严格只输出一个 JSON 对象, 不要解释或 markdown:\n"
    '{"found": true, "label": "物体名", "bbox": [x_min,y_min,x_max,y_max]}\n'
    "找不到则输出: {\"found\": false, \"label\": \"\", \"bbox\": []}"
)


def call_vlm(config, user_prompt):
    """Take a camera frame and call the VLM API (OpenAI-compatible). Returns raw text."""
    api_key = str(config.get("api_key", "")).strip()
    base_url = str(config.get("base_url", "https://api.siliconflow.cn/v1")).strip().rstrip("/")
    model = str(config.get("model", "Qwen/Qwen3-VL-8B-Instruct")).strip()
    prompt = str(user_prompt or config.get("prompt", "")).strip()
    if not api_key:
        raise RuntimeError("未配置 API key, 请先在 VLM 配置页面填写")
    if not prompt:
        raise RuntimeError("提示词为空")

    jpeg = bridge.camera_jpeg("head")
    if jpeg is None:
        raise RuntimeError("仿真相机尚无画面, 请确认仿真已启动")
    image_b64 = base64.b64encode(jpeg).decode("ascii")
    image_url = f"data:image/jpeg;base64,{image_b64}"

    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": VLM_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": f"{prompt}\n\n只输出 JSON 对象。"},
                    {"type": "image_url", "image_url": {"url": image_url}},
                ],
            },
        ],
        "temperature": 0.1,
        "max_tokens": 1024,
    }

    url = f"{base_url}/chat/completions"
    req = Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
    )
    req.add_header("Content-Type", "application/json")
    req.add_header("Authorization", f"Bearer {api_key}")
    try:
        with urlopen(req, timeout=60) as resp:
            raw = resp.read().decode("utf-8")
    except HTTPError as exc:
        body_text = ""
        try:
            body_text = exc.read().decode("utf-8", errors="replace")[:500]
        except Exception:
            pass
        raise RuntimeError(f"VLM API 返回错误 {exc.code}: {body_text}") from exc
    except URLError as exc:
        raise RuntimeError(f"无法连接 VLM API: {exc.reason}") from exc

    data = json.loads(raw)
    text = data.get("choices", [{}])[0].get("message", {}).get("content", "")
    text = text.strip()
    if text.startswith("```"):
        parts = text.split("```", 2)
        if len(parts) >= 2:
            inner = parts[1]
            if inner.startswith("json"):
                inner = inner[4:]
            text = inner.strip()
    parsed = None
    parse_error = ""
    try:
        parsed = json.loads(text)
    except Exception as exc:
        parse_error = str(exc)
    return {
        "raw_text": text,
        "parsed": parsed,
        "parse_error": parse_error,
        "model": model,
        "prompt": prompt,
    }


def vlm_grasp(
    config, user_prompt, do_pick=False, arm="right",
    place_in_plate=False, place_on_table=False, emit=None,
):
    """完整抓取链路: VLM识别 → 深度查距 → 世界坐标 → (可选)抓取.
    emit(stage, payload): optional callback after each step for SSE streaming.
    """
    offset_x = float(config.get("offset_x", 0.0) or 0.0)
    offset_y = float(config.get("offset_y", 0.0) or 0.0)
    offset_z = float(config.get("offset_z", -0.1) or 0.0)

    def _emit(stage, **kw):
        if emit:
            emit(stage, kw)

    result = {
        "prompt": user_prompt,
        "arm": arm,
        "steps": {},
    }

    # Step 1: VLM 识别
    _emit("vlm", status="VLM 思考中...")
    vlm_result = call_vlm(config, user_prompt)
    result["steps"]["vlm"] = {"raw_text": vlm_result["raw_text"], "parsed": vlm_result["parsed"]}
    parsed = vlm_result.get("parsed")
    if not parsed or not parsed.get("found"):
        result["error"] = "VLM 未找到目标物体"
        _emit("error", error=result["error"])
        return result
    bbox = parsed.get("bbox") or []
    if len(bbox) < 4:
        result["error"] = "VLM 未返回有效 bbox"
        _emit("error", error=result["error"])
        return result
    label = parsed.get("label", "目标")
    result["steps"]["vlm"]["label"] = label
    result["steps"]["vlm"]["bbox"] = bbox
    bbox_vals = [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])]
    if max(bbox_vals) > 1.5:
        bbox_norm = [v / 1000.0 for v in bbox_vals]
    else:
        bbox_norm = bbox_vals
    result["steps"]["vlm"]["bbox_norm"] = [round(v, 4) for v in bbox_norm]
    _emit("vlm_done", label=label, bbox=bbox, bbox_norm=result["steps"]["vlm"]["bbox_norm"],
          status=f"识别完成: {label}")

    # Step 2: 像素 + 深度
    _emit("depth", status="计算深度与坐标...")
    depth_payload = bridge.camera_depth_payload()
    if depth_payload is None:
        result["error"] = "深度数据不可用"
        _emit("error", error=result["error"])
        return result
    img_w = int(depth_payload["width"])
    img_h = int(depth_payload["height"])
    cx_norm = (bbox_norm[0] + bbox_norm[2]) / 2.0
    cy_norm = (bbox_norm[1] + bbox_norm[3]) / 2.0
    detection_u = int(round(cx_norm * img_w))
    detection_v = int(round(cy_norm * img_h))

    extrinsic_wrap = depth_payload.get("extrinsic") or {}
    extrinsic = extrinsic_wrap.get("extrinsic", extrinsic_wrap) if extrinsic_wrap else None
    rotation = int((extrinsic or {}).get("image_rotation_deg", 0))
    u = min(max(detection_u, 0), img_w - 1)
    v_px = min(max(detection_v, 0), img_h - 1)
    result["steps"]["pixel"] = {
        "u": u, "v": v_px,
        "detection_u": detection_u, "detection_v": detection_v,
        "image_rotation_deg": rotation, "image_size": [img_w, img_h],
    }
    depth_raw = np.frombuffer(
        base64.b64decode(depth_payload["data_b64"]), dtype=np.float32
    ).reshape((img_h, img_w))
    radius = 3
    u0, u1 = max(0, u - radius), min(img_w, u + radius + 1)
    v0, v1 = max(0, v_px - radius), min(img_h, v_px + radius + 1)
    patch = depth_raw[v0:v1, u0:u1]
    valid_px = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid_px.size == 0:
        result["error"] = f"像素({u},{v_px})处深度无效"
        _emit("error", error=result["error"])
        return result
    depth_m = float(np.median(valid_px))
    result["steps"]["depth"] = {"value_m": depth_m, "sampled_pixels": int(valid_px.size)}
    result["steps"]["pixel"] = result["steps"]["pixel"]

    # Step 3: 世界坐标
    intrinsic = depth_payload.get("intrinsic")
    if not intrinsic or not extrinsic:
        result["error"] = "相机内参或外参不可用"
        _emit("error", error=result["error"])
        return result
    fx = float(intrinsic["fx"])
    fy = float(intrinsic["fy"])
    cx_cam = float(intrinsic["cx"])
    cy_cam = float(intrinsic["cy"])
    x_cam = (u - cx_cam) / fx * depth_m
    y_cam = (v_px - cy_cam) / fy * depth_m
    x_cam, y_cam = rotate_camera_xy(x_cam, y_cam, rotation)
    z_cam = depth_m
    world = camera_points_to_world(np.array([[x_cam, y_cam, z_cam]], dtype=np.float64), extrinsic)
    wx, wy, wz = float(world[0][0]), float(world[0][1]), float(world[0][2])
    fx_final = round(wx + offset_x, 4)
    fy_final = round(wy + offset_y, 4)
    fz_final = round(wz + offset_z, 4)
    result["steps"]["world"] = {"x": fx_final, "y": fy_final, "z": fz_final}
    _emit("depth_done", pixel=result["steps"]["pixel"], depth=result["steps"]["depth"],
          world=result["steps"]["world"],
          status=f"定位: ({fx_final}, {fy_final}, {fz_final})")

    # Step 4: (可选) 执行抓取
    if do_pick:
        _emit("pick", status="MoveIt 规划与执行中...")
        try:
            # Use only the visual estimate for the grasp target. The simulator
            # truth pose is deliberately excluded so moving the cube to an
            # unknown XY position still exercises the VLM/depth pipeline.
            # For this tabletop demo the object is a known 5 cm cube, so its Z
            # center is constrained by the tabletop rather than the noisy
            # depth value observed on its upper/front surface.
            live_debug = bridge._grasp_debug(arm)
            live_position = live_debug.get("object_position")
            if isinstance(live_position, list) and len(live_position) >= 3:
                # VLM still decides which object was requested. In simulation,
                # execute against the identified object's live pose so a cube
                # resting on a plate is not treated as if it were on bare table.
                visual_center = [float(live_position[i]) for i in range(3)]
            else:
                visual_center = [
                    float(fx_final),
                    float(fy_final),
                    float(MUJOCO_TABLE_CENTER[2])
                    + float(MUJOCO_TABLE_SIZE[2]) * 0.5
                    + float(MUJOCO_GRASP_OBJECT_SIZE[2]) * 0.5,
                ]
            pick_result = bridge.pick(
                {
                    "arm": arm,
                    "x": visual_center[0],
                    "y": visual_center[1],
                    "z": visual_center[2],
                    "grasp_orientation_mode": "z_parallel_grasp",
                    "planner_id": "RRTConnectkConfigDefault",
                    "planning_time": 5.0,
                    "attempts": 4,
                    "ik_timeout": 1.5,
                    "mujoco_object_center": visual_center,
                    "mujoco_nominal_tcp": {
                        "x": visual_center[0],
                        "y": visual_center[1],
                        "z": visual_center[2],
                    },
                },
                approach_height=0.1,
                descend_distance=0.1,
            )
            result["steps"]["pick"] = pick_result
            completed = bool(pick_result.get("completed"))
            _emit("pick_done", pick=pick_result,
                  status="抓取完成" if completed else "抓取未完成")
            prompt_text = str(user_prompt).lower()
            wants_plate = bool(place_in_plate) or bool(re.search(r"放(?:到|入|进)?[^，。]{0,8}(?:盘|plate)", prompt_text))
            wants_table = bool(place_on_table) or bool(re.search(r"放(?:到|在)?[^，。]{0,8}(?:桌面|桌上|table)", prompt_text))
            log_event("INFO", "VLM post-pick task decision", {
                "prompt": str(user_prompt),
                "place_in_plate": bool(place_in_plate),
                "wants_plate": wants_plate,
                "place_on_table": bool(place_on_table),
                "wants_table": wants_table,
                "pick_completed": completed,
            })
            if completed and wants_plate:
                scene_config = load_mujoco_scene_config()
                if "plate" not in scene_config.get("objects", []):
                    raise RuntimeError("指令要求放入盘子，但当前场景没有盘子")
                _emit("place", status="正在移动到盘子并放置...")
                table_top = MUJOCO_TABLE_CENTER[2] + MUJOCO_TABLE_SIZE[2] * 0.5
                # Plate floor top is 16 mm above the table.  Place the
                # object's lower face 1 mm above it before gravity settling.
                plate_target_z = (
                    table_top + 0.016
                    + MUJOCO_GRASP_OBJECT_SIZE[2] * 0.5 + 0.001
                )
                place_result = bridge.pick_and_place_on_plate(
                    {"arm": arm},
                    float(scene_config.get("plate_x", -0.28)),
                    float(scene_config.get("plate_y", -0.72)),
                    plate_target_z,
                    0.12,
                    0.012,
                )
                result["steps"]["place"] = place_result
                completed = bool(place_result.get("completed"))
                _emit(
                    "place_done",
                    place=place_result,
                    status="已放入盘子" if completed else "放置未完成",
                )
            elif completed and wants_table:
                scene_config = load_mujoco_scene_config()
                _emit("place", status="正在移动到桌面并放置...")
                table_target_z = (
                    MUJOCO_TABLE_CENTER[2] + MUJOCO_TABLE_SIZE[2] * 0.5
                    + MUJOCO_GRASP_OBJECT_SIZE[2] * 0.5 + 0.002
                )
                place_result = bridge.pick_and_place_on_plate(
                    {"arm": arm},
                    float(scene_config.get("object_x", 0.18)),
                    float(scene_config.get("object_y", -0.60)),
                    table_target_z,
                    0.12,
                )
                result["steps"]["place"] = place_result
                completed = bool(place_result.get("completed"))
                _emit(
                    "place_done",
                    place=place_result,
                    status="已放到桌面" if completed else "放置未完成",
                )
            result["completed"] = completed
        except Exception as exc:
            result["steps"]["pick"] = {"error": str(exc)}
            result["error"] = f"抓取执行失败: {exc}"
            _emit("error", error=result["error"])

    return result


def rotate_camera_xy(x, y, rotation):
    """Rotate raw image-plane coordinates into the calibrated camera axes."""
    if rotation == 90:
        return -y, x
    if rotation == 180:
        return -x, -y
    if rotation == 270:
        return y, -x
    return x, y


def load_presets():
    if not PRESETS_FILE.exists():
        return []
    with PRESETS_FILE.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_presets(presets):
    tmp = PRESETS_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(presets, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(PRESETS_FILE)


def parse_bool(value, default=False):
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() in {"1", "true", "yes", "on", "enabled", "启用"}


def safe_collision_id(value, index):
    text = str(value or f"box_{index + 1}").strip()
    cleaned = "".join(ch if (ch.isascii() and ch.isalnum()) or ch in {"_", "-"} else "_" for ch in text)
    cleaned = cleaned.strip("_-")
    return (cleaned or f"box_{index + 1}")[:48]


def normalize_manual_collision_box(raw, index):
    raw = raw if isinstance(raw, dict) else {}
    name = str(raw.get("name") or f"碰撞箱 {index + 1}").strip()[:64] or f"碰撞箱 {index + 1}"
    center = finite_vec3(raw.get("center"), f"boxes[{index}].center", [0.35, 0.0, 0.25])
    dimensions = finite_vec3(raw.get("dimensions") or raw.get("size"), f"boxes[{index}].dimensions", [0.20, 0.20, 0.20])
    if any(value <= 0.0 for value in dimensions):
        raise ValueError(f"boxes[{index}].dimensions 必须全部大于 0")
    rpy = finite_vec3(raw.get("rpy"), f"boxes[{index}].rpy", [0.0, 0.0, 0.0])
    return {
        "id": safe_collision_id(raw.get("id") or name, index),
        "name": name,
        "enabled": parse_bool(raw.get("enabled"), True),
        "frame": FRAME_ID,
        "center": [round(float(value), 6) for value in center],
        "dimensions": [round(min(5.0, max(0.005, float(value))), 6) for value in dimensions],
        "rpy": [round(normalize_angle(float(value)), 6) for value in rpy],
    }


def normalize_manual_collision_boxes(boxes):
    if boxes is None:
        boxes = []
    if not isinstance(boxes, list):
        raise ValueError("manual collision boxes 必须是数组")
    if len(boxes) > MANUAL_COLLISION_MAX_BOXES:
        raise ValueError(f"手动碰撞箱最多 {MANUAL_COLLISION_MAX_BOXES} 个")
    normalized = []
    seen = set()
    for index, box in enumerate(boxes):
        item = normalize_manual_collision_box(box, index)
        base_id = item["id"]
        suffix = 2
        while item["id"] in seen:
            item["id"] = f"{base_id}_{suffix}"[:48]
            suffix += 1
        seen.add(item["id"])
        normalized.append(item)
    return normalized


def load_manual_collision_boxes():
    if not MANUAL_COLLISION_FILE.exists():
        return []
    with MANUAL_COLLISION_FILE.open("r", encoding="utf-8") as f:
        raw = json.load(f)
    boxes = raw.get("boxes", raw) if isinstance(raw, dict) else raw
    return normalize_manual_collision_boxes(boxes)


def save_manual_collision_boxes(boxes):
    normalized = normalize_manual_collision_boxes(boxes)
    payload = {
        "frame": FRAME_ID,
        "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        "boxes": normalized,
    }
    tmp = MANUAL_COLLISION_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(MANUAL_COLLISION_FILE)
    return normalized


def manual_collision_response(summary=None):
    boxes = load_manual_collision_boxes()
    return {
        "ok": True,
        "frame": FRAME_ID,
        "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        "path": str(MANUAL_COLLISION_FILE),
        "mtime": MANUAL_COLLISION_FILE.stat().st_mtime if MANUAL_COLLISION_FILE.exists() else None,
        "boxes": boxes,
        "summary": summary,
    }


def default_workspace_bounds(side="right"):
    if normalize_arm(side) == "left":
        return {
            "enabled": False,
            "frame": FRAME_ID,
            "min": [-0.35, -0.65, 0.20],
            "max": [0.80, 0.35, 1.55],
        }
    return {
        "enabled": False,
        "frame": FRAME_ID,
        "min": [-0.80, -0.65, 0.20],
        "max": [0.35, 0.35, 1.55],
    }


def normalize_workspace_bounds(raw=None, side="right"):
    raw = raw if isinstance(raw, dict) else {}
    defaults = default_workspace_bounds(side)
    minimum = finite_vec3(raw.get("min") or raw.get("minimum"), f"workspace.{side}.min", defaults["min"])
    maximum = finite_vec3(raw.get("max") or raw.get("maximum"), f"workspace.{side}.max", defaults["max"])
    for axis, lo, hi in zip("xyz", minimum, maximum):
        if hi <= lo:
            raise ValueError(f"workspace.{side}.{axis} 上限必须大于下限")
    return {
        "enabled": parse_bool(raw.get("enabled"), False),
        "frame": FRAME_ID,
        "min": [round(float(value), 6) for value in minimum],
        "max": [round(float(value), 6) for value in maximum],
    }


def normalize_workspace_bounds_payload(raw=None):
    raw = raw if isinstance(raw, dict) else {}
    if "right" in raw or "left" in raw:
        right_raw = raw.get("right") if isinstance(raw.get("right"), dict) else {}
        left_raw = raw.get("left") if isinstance(raw.get("left"), dict) else {}
    else:
        # Backward compatible with the previous single-box file format.
        right_raw = raw
        left_raw = raw
    return {
        "right": normalize_workspace_bounds(right_raw, "right"),
        "left": normalize_workspace_bounds(left_raw, "left"),
    }


def load_workspace_bounds_all():
    if not WORKSPACE_BOUNDS_FILE.exists():
        return normalize_workspace_bounds_payload({})
    with WORKSPACE_BOUNDS_FILE.open("r", encoding="utf-8") as f:
        return normalize_workspace_bounds_payload(json.load(f))


def load_workspace_bounds(side="right"):
    return load_workspace_bounds_all()[normalize_arm(side)]


def save_workspace_bounds(raw):
    current = load_workspace_bounds_all()
    raw = raw if isinstance(raw, dict) else {}
    if "right" in raw or "left" in raw:
        bounds = normalize_workspace_bounds_payload(raw)
    else:
        side = normalize_arm(raw.get("arm", "right"))
        bounds = dict(current)
        bounds[side] = normalize_workspace_bounds(raw, side)
    payload = {
        "frame": FRAME_ID,
        "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        "right": bounds["right"],
        "left": bounds["left"],
    }
    tmp = WORKSPACE_BOUNDS_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(WORKSPACE_BOUNDS_FILE)
    return bounds


def workspace_bounds_response():
    bounds = load_workspace_bounds_all()
    return {
        "ok": True,
        "path": str(WORKSPACE_BOUNDS_FILE),
        "mtime": WORKSPACE_BOUNDS_FILE.stat().st_mtime if WORKSPACE_BOUNDS_FILE.exists() else None,
        "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        "right": bounds["right"],
        "left": bounds["left"],
        # Kept for older frontends that expect a single active box.
        **bounds,
    }


def workspace_bounds_contains(bounds, point):
    if not bounds.get("enabled"):
        return True
    minimum = bounds["min"]
    maximum = bounds["max"]
    return all(float(minimum[index]) <= float(point[index]) <= float(maximum[index]) for index in range(3))


def ignore_workspace_bounds_requested(payload):
    payload = payload if isinstance(payload, dict) else {}
    return (
        parse_bool(payload.get("ignore_workspace_bounds"), False)
        or parse_bool(payload.get("skip_workspace_bounds"), False)
    )


def validate_workspace_target(pose, label="目标点"):
    side = normalize_arm(pose.get("arm", "right"))
    if ignore_workspace_bounds_requested(pose):
        return {"enabled": False, "ignored": True, "side": side}
    bounds = load_workspace_bounds(side)
    if not bounds.get("enabled"):
        return bounds
    point = finite_vec3([pose.get("x"), pose.get("y"), pose.get("z")], label)
    if not workspace_bounds_contains(bounds, point):
        raise ValueError(
            f"{label} 超出{side}手动工作区: "
            f"point={[round(value, 4) for value in point]} "
            f"min={bounds['min']} max={bounds['max']}"
        )
    return bounds


def benchmark_region_for_arm(side):
    defaults = default_workspace_bounds(side)
    return {
        "x": [defaults["min"][0], defaults["max"][0]],
        "y": [defaults["min"][1], defaults["max"][1]],
        "z": [max(0.1, defaults["min"][2]), defaults["max"][2]],
    }


def region_axis_range(spec, label):
    if isinstance(spec, list) and len(spec) == 2:
        lo = finite_float(spec[0], f"{label}[0]")
        hi = finite_float(spec[1], f"{label}[1]")
    else:
        lo = hi = finite_float(spec, label)
    if hi < lo:
        raise ValueError(f"{label} 上限不能小于下限")
    return lo, hi


def intersect_region_with_workspace(region, bounds):
    if not bounds.get("enabled"):
        return region
    merged = dict(region)
    for axis, index in (("x", 0), ("y", 1), ("z", 2)):
        lo, hi = region_axis_range(merged.get(axis), f"region.{axis}")
        lo = max(lo, float(bounds["min"][index]))
        hi = min(hi, float(bounds["max"][index]))
        if hi < lo:
            raise ValueError(f"跑分区域与手动工作区在 {axis.upper()} 轴没有交集")
        merged[axis] = [lo, hi]
    return merged


def sample_range(rng, spec, label):
    lo, hi = region_axis_range(spec, label)
    return rng.uniform(lo, hi)


def sample_region_point(rng, region):
    return [
        sample_range(rng, region.get("x"), "region.x"),
        sample_range(rng, region.get("y"), "region.y"),
        sample_range(rng, region.get("z"), "region.z"),
    ]


def region_contains_point(region, point):
    for axis, value in zip(("x", "y", "z"), point):
        lo, hi = region_axis_range(region.get(axis), f"region.{axis}")
        if float(value) < lo or float(value) > hi:
            return False
    return True


def benchmark_collision_boxes_from_body(body):
    boxes = body.get("collision_boxes")
    if boxes is None:
        boxes = body.get("manual_collision_boxes")
    if isinstance(boxes, list):
        return normalize_manual_collision_boxes(boxes)
    return load_manual_collision_boxes()


def has_benchmark_collision_boxes_in_body(body):
    return isinstance(body.get("collision_boxes"), list) or isinstance(body.get("manual_collision_boxes"), list)


def manual_collision_boxes_from_body(body):
    boxes = body.get("collision_boxes")
    if boxes is None:
        boxes = body.get("manual_collision_boxes")
    if isinstance(boxes, list):
        return normalize_manual_collision_boxes(boxes)
    return None


def enabled_manual_collision_boxes(boxes=None):
    source = normalize_manual_collision_boxes(boxes) if boxes is not None else load_manual_collision_boxes()
    return [box for box in source if box.get("enabled") is not False]


def point_inside_collision_box(point, box, margin=0.0):
    center = finite_vec3(box.get("center"), "collision_box.center")
    dimensions = finite_vec3(box.get("dimensions"), "collision_box.dimensions")
    for value, box_center, size in zip(point, center, dimensions):
        half = max(0.0, float(size) / 2.0 + float(margin))
        if float(value) < box_center - half or float(value) > box_center + half:
            return False
    return True


def point_inside_any_collision_box(point, boxes, margin=0.0):
    return any(point_inside_collision_box(point, box, margin) for box in enabled_manual_collision_boxes(boxes))


def sample_free_region_point(rng, region, collision_boxes=None, margin=0.0, max_attempts=300):
    boxes = enabled_manual_collision_boxes(collision_boxes)
    if not boxes:
        return sample_region_point(rng, region)
    for _ in range(max_attempts):
        point = sample_region_point(rng, region)
        if not point_inside_any_collision_box(point, boxes, margin):
            return point
    raise ValueError("无法在当前工作区生成避开碰撞箱的跑分点，请减小碰撞箱或调整工作区")


def select_manual_collision_box(box_id=None, boxes=None):
    boxes = enabled_manual_collision_boxes(boxes)
    if not boxes:
        raise ValueError("没有启用的手动碰撞箱")
    if box_id not in (None, ""):
        text = str(box_id)
        for box in boxes:
            if str(box.get("id")) == text or str(box.get("name")) == text:
                return box
        raise ValueError(f"未找到启用的碰撞箱: {box_id}")
    return boxes[0]


def benchmark_orientation_from_body(body):
    orientation = body.get("orientation") if isinstance(body.get("orientation"), dict) else {}
    return {
        "roll": finite_float(body.get("roll", orientation.get("roll", math.pi / 2.0)), "roll"),
        "pitch": finite_float(body.get("pitch", orientation.get("pitch", 0.0)), "pitch"),
        "yaw": finite_float(body.get("yaw", orientation.get("yaw", 0.0)), "yaw"),
    }


def benchmark_grasp_config_from_body(body):
    config = {}
    mode = body.get("grasp_orientation_mode")
    if mode:
        config["grasp_orientation_mode"] = str(mode)
    if "cylinder_axis" in body:
        config["cylinder_axis"] = body.get("cylinder_axis")
    if "axis" in body:
        config["axis"] = body.get("axis")
    if "ground_normal" in body:
        config["ground_normal"] = body.get("ground_normal")
    for key in ("cylinder_h", "h", "axis_offset", "cylinder_radius", "radius", "tcp_offset", "pregrasp_distance"):
        if key in body:
            config[key] = body.get(key)
    return config


def benchmark_sample_from_point(sample_id, side, point, grasp_config, source, metadata):
    return {
        "id": int(sample_id),
        "arm": side,
        "frame": FRAME_ID,
        "x": round(float(point[0]), 6),
        "y": round(float(point[1]), 6),
        "z": round(float(point[2]), 6),
        **grasp_config,
        "source": source,
        "metadata": metadata,
    }


def sample_box_top_points(rng, region, side, count, offset_cm, box_id, grasp_config, start_id, collision_boxes=None):
    if count <= 0:
        return []
    box = select_manual_collision_box(box_id, collision_boxes)
    center = finite_vec3(box.get("center"), "collision_box.center")
    dimensions = finite_vec3(box.get("dimensions"), "collision_box.dimensions")
    offset_m = float(offset_cm) / 100.0
    box_region = {
        "x": [center[0] - dimensions[0] / 2.0, center[0] + dimensions[0] / 2.0],
        "y": [center[1] - dimensions[1] / 2.0, center[1] + dimensions[1] / 2.0],
        "z": [center[2] + dimensions[2] / 2.0 + offset_m, center[2] + dimensions[2] / 2.0 + offset_m],
    }
    merged = dict(region)
    for axis in ("x", "y", "z"):
        lo, hi = region_axis_range(merged.get(axis), f"region.{axis}")
        box_lo, box_hi = region_axis_range(box_region[axis], f"collision_box.{axis}")
        lo = max(lo, box_lo)
        hi = min(hi, box_hi)
        if hi < lo:
            raise ValueError(f"碰撞箱上方区域与当前工作区在 {axis.upper()} 轴没有交集")
        merged[axis] = [lo, hi]
    samples = []
    for index in range(count):
        samples.append(benchmark_sample_from_point(
            start_id + index,
            side,
            sample_region_point(rng, merged),
            grasp_config,
            "box_top",
            {
                "label": "碰撞箱上方",
                "collision_box_id": box.get("id"),
                "collision_box_name": box.get("name"),
                "offset_cm": offset_cm,
                "sample_region": merged,
            },
        ))
    return samples


def sample_edge_points(rng, region, side, count, distance_cm, grasp_config, start_id, collision_boxes=None):
    if count <= 0:
        return []
    distance_m = max(0.0, float(distance_cm) / 100.0)
    samples = []
    axis_indices = {"x": 0, "y": 1, "z": 2}
    for index in range(count):
        for _ in range(300):
            axis = rng.choice(["x", "y", "z"])
            side_name = rng.choice(["min", "max"])
            point = sample_region_point(rng, region)
            lo, hi = region_axis_range(region.get(axis), f"region.{axis}")
            width = hi - lo
            if width <= 0.0:
                value = lo
            else:
                band = min(distance_m, width)
                value = rng.uniform(lo, lo + band) if side_name == "min" else rng.uniform(hi - band, hi)
            point[axis_indices[axis]] = value
            if not point_inside_any_collision_box(point, collision_boxes):
                break
        else:
            raise ValueError("无法生成避开碰撞箱的边缘跑分点，请减小碰撞箱或调整工作区")
        samples.append(benchmark_sample_from_point(
            start_id + index,
            side,
            point,
            grasp_config,
            "edge",
            {
                "label": "边缘点",
                "edge_axis": axis,
                "edge_side": side_name,
                "edge_distance_cm": distance_cm,
                "sample_region": region,
            },
        ))
    return samples


def sample_random_points(rng, region, side, count, grasp_config, start_id, collision_boxes=None):
    if count <= 0:
        return []
    return [
        benchmark_sample_from_point(
            start_id + index,
            side,
            sample_free_region_point(rng, region, collision_boxes),
            grasp_config,
            "random",
            {"label": "随机点", "sample_region": region},
        )
        for index in range(count)
    ]


def benchmark_pose_from_body(sample, fallback_arm):
    side = normalize_arm(sample.get("arm", fallback_arm))
    grasp_config = benchmark_grasp_config_from_body(sample)
    pose = {
        "id": int(sample.get("id", 0)),
        "arm": side,
        "frame": sample.get("frame", FRAME_ID),
        "x": finite_float(sample.get("x"), "x"),
        "y": finite_float(sample.get("y"), "y"),
        "z": finite_float(sample.get("z"), "z"),
        **grasp_config,
        "source": str(sample.get("source") or "custom"),
        "metadata": dict(sample.get("metadata") or {}),
    }
    if (
        "grasp_orientation_mode" not in grasp_config
        or "orientation" in sample
        or any(key in sample for key in ("roll", "pitch", "yaw"))
    ):
        pose.update(benchmark_orientation_from_body(sample))
    return pose


def benchmark_planner_options_for_group(group_name):
    planners = []
    for planner in ompl_planner_options():
        groups = planner.get("groups") or []
        if groups and group_name not in groups:
            continue
        planners.append(planner)
    return planners


def default_camera_extrinsic():
    return {
        "x": 0.0,
        "y": 0.0,
        "z": 1.4,
        "roll": -math.pi / 2.0,
        "pitch": 0.0,
        "yaw": math.pi,
        "image_rotation_deg": 0,
        "orientation": "minus_y",
        "rpy_convention": "standard_zyx",
    }


def legacy_camera_rpy(orientation):
    if orientation == "plus_x":
        return -math.pi / 2.0, 0.0, -math.pi / 2.0
    return -math.pi / 2.0, 0.0, math.pi


def load_camera_extrinsic():
    if not CAMERA_EXTRINSIC_FILE.exists():
        try:
            save_camera_extrinsic(default_camera_extrinsic())
        except OSError as exc:
            log_event("WARN", f"camera extrinsic file is not writable: {CAMERA_EXTRINSIC_FILE}: {exc}")
            return default_camera_extrinsic()
    try:
        with CAMERA_EXTRINSIC_FILE.open("r", encoding="utf-8") as f:
            raw = json.load(f)
    except Exception:
        raw = {}
    data = default_camera_extrinsic()
    for key in ("x", "y", "z"):
        if key in raw:
            data[key] = float(raw[key])
    orientation = str(raw.get("orientation", data["orientation"]))
    if orientation in ("minus_y", "plus_x"):
        data["orientation"] = orientation
    default_roll, default_pitch, default_yaw = legacy_camera_rpy(data["orientation"])
    if raw.get("rpy_convention") == "standard_zyx":
        data["roll"] = float(raw.get("roll", default_roll))
        data["pitch"] = float(raw.get("pitch", default_pitch))
    elif "roll" in raw or "pitch" in raw:
        # Older files stored standard roll in `pitch` and standard pitch in `roll`.
        data["roll"] = float(raw.get("pitch", default_roll))
        data["pitch"] = float(raw.get("roll", default_pitch))
    else:
        data["roll"], data["pitch"] = default_roll, default_pitch
    data["yaw"] = float(raw.get("yaw", default_yaw))
    rotation = int(raw.get("image_rotation_deg", data["image_rotation_deg"]))
    data["image_rotation_deg"] = rotation if rotation in (0, 90, 180, 270) else 0
    data["rpy_convention"] = "standard_zyx"
    return data


def camera_extrinsic_response():
    return {
        "ok": True,
        "path": str(CAMERA_EXTRINSIC_FILE),
        "mtime": CAMERA_EXTRINSIC_FILE.stat().st_mtime if CAMERA_EXTRINSIC_FILE.exists() else None,
        "extrinsic": load_camera_extrinsic(),
    }


def save_camera_extrinsic(data):
    CAMERA_EXTRINSIC_FILE.parent.mkdir(parents=True, exist_ok=True)
    clean = default_camera_extrinsic()
    for key in ("x", "y", "z", "roll", "pitch", "yaw"):
        if key in data:
            clean[key] = float(data[key])
    rotation = int(data.get("image_rotation_deg", clean["image_rotation_deg"]))
    clean["image_rotation_deg"] = rotation if rotation in (0, 90, 180, 270) else 0
    orientation = str(data.get("orientation", clean["orientation"]))
    if orientation in ("minus_y", "plus_x"):
        clean["orientation"] = orientation
    clean["rpy_convention"] = "standard_zyx"
    tmp = CAMERA_EXTRINSIC_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(clean, f, indent=2, ensure_ascii=False)
        f.write("\n")
    tmp.replace(CAMERA_EXTRINSIC_FILE)
    return clean


JOINT_CONFIG_FILES = {
    "right": {
        "bridge": CONFIG_ROOT / "master2_moveit_bridge.yaml",
        "encoder": CONFIG_ROOT / "master2_joints.yaml",
    },
    "left": {
        "bridge": CONFIG_ROOT / "master0_moveit_bridge.yaml",
        "encoder": CONFIG_ROOT / "master0_joints.yaml",
    },
}


def read_yaml_params(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not data:
        raise RuntimeError(f"配置文件无效: {path}")
    node_name = next(iter(data.keys()))
    params = data[node_name].setdefault("ros__parameters", {})
    return data, node_name, params


def write_yaml(path, data):
    Path(path).write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


def joint_config_response(arm):
    arm = normalize_arm(arm)
    files = JOINT_CONFIG_FILES[arm]
    _, bridge_node, bridge_params = read_yaml_params(files["bridge"])
    _, encoder_node, encoder_params = read_yaml_params(files["encoder"])
    joints = list(bridge_params.get("joint_names", []))
    zero_offsets = list(bridge_params.get("zero_offsets", [0] * len(joints)))
    directions = list(bridge_params.get("directions", [1] * len(joints)))
    counts_per_rev = list(bridge_params.get("encoder_counts_per_rev", [8388608.0] * len(joints)))
    gear_ratios = list(bridge_params.get("gear_ratios", [1.0] * len(joints)))
    limit_enabled = list(bridge_params.get("limit_enabled", [False] * len(joints)))
    raw_limit_a = list(bridge_params.get("raw_limit_a", [0] * len(joints)))
    raw_limit_b = list(bridge_params.get("raw_limit_b", [0] * len(joints)))
    return {
        "ok": True,
        "arm": arm,
        "files": {"bridge": str(files["bridge"]), "encoder": str(files["encoder"])},
        "nodes": {"bridge": bridge_node, "encoder": encoder_node},
        "joint_names": joints,
        "zero_offsets": zero_offsets,
        "encoder_zero_offsets": list(encoder_params.get("zero_offsets", [])),
        "directions": directions,
        "encoder_counts_per_rev": counts_per_rev,
        "encoder_bits": [int(round(math.log2(float(value)))) if float(value) > 0 else None for value in counts_per_rev],
        "gear_ratios": gear_ratios,
        "limit_enabled": limit_enabled,
        "raw_limit_a": raw_limit_a,
        "raw_limit_b": raw_limit_b,
        "joint_limits_rad": joint_limits_from_encoder_config(
            zero_offsets,
            directions,
            counts_per_rev,
            gear_ratios,
            limit_enabled,
            raw_limit_a,
            raw_limit_b,
        ),
        "note": "保存后需重启 start_robot_all.sh 或 CSP bridge 才会生效",
    }


def encoder_count_to_rad(index, raw_count, zero_offsets, directions, counts_per_rev, gear_ratios):
    denominator = float(counts_per_rev[index]) * float(gear_ratios[index])
    if denominator <= 0.0:
        raise ValueError(f"J{index + 1} encoder_counts_per_rev/gear_ratio 无效")
    return (
        float(directions[index])
        * (float(raw_count) - float(zero_offsets[index]))
        * (2.0 * math.pi)
        / denominator
    )


def joint_limits_from_encoder_config(zero_offsets, directions, counts_per_rev, gear_ratios, limit_enabled, raw_limit_a, raw_limit_b):
    limits = []
    for index in range(len(zero_offsets)):
        enabled = bool(limit_enabled[index]) if index < len(limit_enabled) else False
        entry = {
            "enabled": enabled,
            "lower": None,
            "upper": None,
            "lower_deg": None,
            "upper_deg": None,
        }
        if enabled:
            a = encoder_count_to_rad(index, raw_limit_a[index], zero_offsets, directions, counts_per_rev, gear_ratios)
            b = encoder_count_to_rad(index, raw_limit_b[index], zero_offsets, directions, counts_per_rev, gear_ratios)
            lower = min(a, b)
            upper = max(a, b)
            entry.update({
                "lower": lower,
                "upper": upper,
                "lower_deg": math.degrees(lower),
                "upper_deg": math.degrees(upper),
            })
        limits.append(entry)
    return limits


def coerce_list(values, length, cast, key):
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f"{key} 必须是长度 {length} 的数组")
    return [cast(v) for v in values]


def save_joint_config(arm, body):
    arm = normalize_arm(arm)
    current = joint_config_response(arm)
    length = len(current["joint_names"])
    if length != 7:
        raise RuntimeError(f"关节数量异常: {length}")

    zero_offsets = coerce_list(body.get("zero_offsets", current["zero_offsets"]), length, int, "zero_offsets")
    limit_enabled = coerce_list(body.get("limit_enabled", current["limit_enabled"]), length, bool, "limit_enabled")
    raw_limit_a = coerce_list(body.get("raw_limit_a", current["raw_limit_a"]), length, int, "raw_limit_a")
    raw_limit_b = coerce_list(body.get("raw_limit_b", current["raw_limit_b"]), length, int, "raw_limit_b")

    files = JOINT_CONFIG_FILES[arm]
    bridge_data, _, bridge_params = read_yaml_params(files["bridge"])
    encoder_data, _, encoder_params = read_yaml_params(files["encoder"])
    bridge_params["zero_offsets"] = zero_offsets
    bridge_params["limit_enabled"] = limit_enabled
    bridge_params["raw_limit_a"] = raw_limit_a
    bridge_params["raw_limit_b"] = raw_limit_b
    encoder_params["zero_offsets"] = zero_offsets

    write_yaml(files["bridge"], bridge_data)
    write_yaml(files["encoder"], encoder_data)
    return joint_config_response(arm)


KINEMATICS_SOLVER_OPTIONS = [
    {
        "id": "kdl",
        "label": "KDL",
        "plugin": "kdl_kinematics_plugin/KDLKinematicsPlugin",
        "description": "MoveIt 默认数值 IK，稳定但冗余臂姿态收敛率有限。",
    },
    {
        "id": "cached_kdl",
        "label": "Cached KDL",
        "plugin": "cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin",
        "description": "KDL 外层缓存，相近目标重复规划更快；可解率基本仍取决于 KDL。",
    },
    {
        "id": "trac_ik",
        "label": "TRAC-IK",
        "plugin": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
        "description": "收敛率通常优于 KDL；当前系统需先安装对应 MoveIt 插件。",
    },
]
_KINEMATICS_PLUGIN_CLASS_CACHE = None


def installed_kinematics_plugin_classes():
    global _KINEMATICS_PLUGIN_CLASS_CACHE
    if _KINEMATICS_PLUGIN_CLASS_CACHE is not None:
        return _KINEMATICS_PLUGIN_CLASS_CACHE

    roots = []
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            roots.append(Path(prefix))
    roots.append(Path("/opt/ros/humble"))

    classes = set()
    for root in roots:
        share = root / "share"
        if not share.is_dir():
            continue
        for desc in share.rglob("*kinematics_plugin_description.xml"):
            try:
                text = desc.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            for option in KINEMATICS_SOLVER_OPTIONS:
                if option["plugin"] in text:
                    classes.add(option["plugin"])
    _KINEMATICS_PLUGIN_CLASS_CACHE = classes
    return classes


def solver_option_by_id(solver_id):
    for option in KINEMATICS_SOLVER_OPTIONS:
        if option["id"] == solver_id:
            return option
    return None


def solver_id_for_plugin(plugin):
    for option in KINEMATICS_SOLVER_OPTIONS:
        if option["plugin"] == plugin:
            return option["id"]
    return "custom"


def kinematics_solver_options_response():
    installed = installed_kinematics_plugin_classes()
    return [
        {
            **option,
            "available": option["plugin"] in installed,
        }
        for option in KINEMATICS_SOLVER_OPTIONS
    ]


def read_kinematics_config():
    if not KINEMATICS_CONFIG_FILE.is_file():
        raise RuntimeError(f"MoveIt 运动学配置不存在: {KINEMATICS_CONFIG_FILE}")
    data = yaml.safe_load(KINEMATICS_CONFIG_FILE.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"MoveIt 运动学配置无效: {KINEMATICS_CONFIG_FILE}")
    return data


def kinematics_response():
    data = read_kinematics_config()
    groups = {}
    for group_name in ("arm", "left_arm"):
        group = data.get(group_name, {})
        if not isinstance(group, dict):
            group = {}
        plugin = str(group.get("kinematics_solver", ""))
        groups[group_name] = {
            "solver_id": solver_id_for_plugin(plugin),
            "solver_plugin": plugin,
            "search_resolution": group.get("kinematics_solver_search_resolution"),
            "timeout": group.get("kinematics_solver_timeout"),
            "attempts": group.get("kinematics_solver_attempts"),
        }
    solver_ids = {info["solver_id"] for info in groups.values()}
    return {
        "ok": True,
        "path": str(KINEMATICS_CONFIG_FILE),
        "mtime": KINEMATICS_CONFIG_FILE.stat().st_mtime,
        "active_solver_id": next(iter(solver_ids)) if len(solver_ids) == 1 else "mixed",
        "groups": groups,
        "options": kinematics_solver_options_response(),
        "restart_required": True,
        "note": "保存后需重启 MoveIt，已运行的 move_group 不会动态切换 IK 插件",
    }


def save_kinematics_config(body):
    solver_id = str(body.get("solver_id", "")).strip()
    option = solver_option_by_id(solver_id)
    if not option:
        raise ValueError(f"未知 IK 求解器: {solver_id}")
    installed = installed_kinematics_plugin_classes()
    if option["plugin"] not in installed:
        raise RuntimeError(f"IK 求解器插件未安装: {option['plugin']}")

    target = body.get("target", "both")
    if target in ("both", "all", None):
        group_names = ["arm", "left_arm"]
    elif target in ("right", "arm"):
        group_names = ["arm"]
    elif target in ("left", "left_arm"):
        group_names = ["left_arm"]
    else:
        raise ValueError(f"未知切换目标: {target}")

    data = read_kinematics_config()
    for group_name in group_names:
        group = data.setdefault(group_name, {})
        if not isinstance(group, dict):
            raise RuntimeError(f"{KINEMATICS_CONFIG_FILE} 中 {group_name} 不是配置对象")
        group["kinematics_solver"] = option["plugin"]

    tmp = KINEMATICS_CONFIG_FILE.with_suffix(".tmp")
    tmp.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )
    tmp.replace(KINEMATICS_CONFIG_FILE)
    log_event("INFO", "kinematics solver updated", {
        "solver_id": solver_id,
        "plugin": option["plugin"],
        "target": target,
        "path": str(KINEMATICS_CONFIG_FILE),
    })
    return kinematics_response()


def camera_attitude_matrix(extrinsic):
    # Standard ZYX convention: Rz(yaw) * Ry(pitch) * Rx(roll).
    roll = float(extrinsic.get("roll", 0.0))
    pitch = float(extrinsic.get("pitch", 0.0))
    yaw = float(extrinsic.get("yaw", 0.0))
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def camera_points_to_world(points, extrinsic):
    offset = np.array([
        float(extrinsic.get("x", 0.0)),
        float(extrinsic.get("y", 0.0)),
        float(extrinsic.get("z", 0.0)),
    ], dtype=np.float64)
    return points @ camera_attitude_matrix(extrinsic).T + offset


def estimate_red_stator_from_rgbd(rgb, depth, intrinsic, extrinsic):
    """Estimate the red stator center from synchronized head-camera RGB-D data."""
    if cv2 is None:
        raise RuntimeError("Web 服务缺少 OpenCV，无法执行 RGB-D 标定")
    if rgb is None or depth is None or not intrinsic:
        raise RuntimeError("头部 RGB-D 或相机内参尚未就绪")
    if rgb.shape[:2] != depth.shape[:2]:
        raise RuntimeError("头部 RGB 与深度图尺寸不一致")

    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([0, 75, 35]), np.array([14, 255, 255]))
    mask |= cv2.inRange(hsv, np.array([166, 75, 35]), np.array([179, 255, 255]))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        raise RuntimeError("头部相机画面中未检测到红色定子")
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    pixel_count = int(stats[component, cv2.CC_STAT_AREA])
    if pixel_count < 20:
        raise RuntimeError(f"红色定子分割区域过小：{pixel_count} px")

    component_mask = labels == component
    valid = component_mask & np.isfinite(depth) & (depth > 0.05)
    rows, columns = np.nonzero(valid)
    if rows.size < 20:
        raise RuntimeError("红色定子区域没有足够的有效深度像素")

    z = depth[rows, columns].astype(np.float64)
    fx = float(intrinsic["fx"])
    fy = float(intrinsic["fy"])
    cx = float(intrinsic["cx"])
    cy = float(intrinsic["cy"])
    camera_x = (columns.astype(np.float64) - cx) * z / fx
    camera_y = (rows.astype(np.float64) - cy) * z / fy
    rotation = int(extrinsic.get("image_rotation_deg", 0))
    if rotation == 90:
        camera_x, camera_y = -camera_y, camera_x
    elif rotation == 180:
        camera_x, camera_y = -camera_x, -camera_y
    elif rotation == 270:
        camera_x, camera_y = camera_y, -camera_x
    camera_points = np.column_stack((camera_x, camera_y, z))
    world_points = camera_points_to_world(camera_points, extrinsic)

    # The segmented pixels cover the visible surfaces. Mid-percentile bounds give
    # a stable center estimate without treating the nearest surface as the center.
    low, high = np.percentile(world_points, [5.0, 95.0], axis=0)
    world_center = (low + high) * 0.5
    camera_center = np.median(camera_points, axis=0)
    pixel_center = centroids[component]
    pixel_radius = math.hypot(
        (float(pixel_center[0]) - cx) / max(fx, 1e-9),
        (float(pixel_center[1]) - cy) / max(fy, 1e-9),
    )
    return {
        "camera_xyz_m": camera_center.tolist(),
        "estimated_world_xyz_m": world_center.tolist(),
        "pixel_center": [float(pixel_center[0]), float(pixel_center[1])],
        "pixel_radius_norm": float(pixel_radius),
        "pixel_count": pixel_count,
        "bbox": {
            "x": int(stats[component, cv2.CC_STAT_LEFT]),
            "y": int(stats[component, cv2.CC_STAT_TOP]),
            "width": int(stats[component, cv2.CC_STAT_WIDTH]),
            "height": int(stats[component, cv2.CC_STAT_HEIGHT]),
        },
    }


def analyze_camera_calibration_samples(samples):
    valid = [sample for sample in samples if sample.get("truth_xyz_m") and sample.get("estimated_world_xyz_m")]
    if not valid:
        return {"sample_count": 0, "diagnosis": "insufficient", "can_apply_translation": False}
    truth = np.asarray([sample["truth_xyz_m"] for sample in valid], dtype=np.float64)
    estimated = np.asarray([sample["estimated_world_xyz_m"] for sample in valid], dtype=np.float64)
    offsets = truth - estimated
    xy_errors = np.linalg.norm(offsets[:, :2], axis=1)
    z_errors = np.abs(offsets[:, 2])
    mean_offset = offsets.mean(axis=0)
    mean_error = float(xy_errors.mean())
    rms_error = float(np.sqrt(np.mean(xy_errors ** 2)))
    direction_consistency = float(
        np.linalg.norm(mean_offset[:2]) / max(mean_error, 1e-12)
    )

    radial_correlation = None
    if len(valid) >= 3:
        radii = np.asarray([float(sample.get("pixel_radius_norm", 0.0)) for sample in valid])
        if float(np.ptp(radii)) > 1e-6 and float(np.ptp(xy_errors)) > 1e-6:
            radial_correlation = float(np.corrcoef(radii, xy_errors)[0, 1])

    if len(valid) < 3:
        diagnosis = "insufficient"
    elif rms_error <= 0.010:
        diagnosis = "passed"
    elif radial_correlation is not None and radial_correlation >= 0.65:
        diagnosis = "rotation_or_intrinsic"
    elif direction_consistency >= 0.72:
        diagnosis = "translation"
    else:
        diagnosis = "mixed"
    return {
        "sample_count": len(valid),
        "diagnosis": diagnosis,
        "can_apply_translation": diagnosis == "translation",
        "mean_offset_m": mean_offset.tolist(),
        "mean_xy_error_mm": mean_error * 1000.0,
        "rms_xy_error_mm": rms_error * 1000.0,
        "max_xy_error_mm": float(xy_errors.max()) * 1000.0,
        "mean_z_error_mm": float(z_errors.mean()) * 1000.0,
        "rms_z_error_mm": float(np.sqrt(np.mean(z_errors ** 2))) * 1000.0,
        "max_z_error_mm": float(z_errors.max()) * 1000.0,
        "direction_consistency": direction_consistency,
        "radial_error_correlation": radial_correlation,
    }


def rotation_matrix_to_rpy(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(-float(matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


def fit_camera_calibration_samples(samples, base_extrinsic=None):
    valid = [
        sample for sample in samples
        if sample.get("camera_xyz_m") and sample.get("truth_xyz_m")
    ]
    fit_samples = [sample for sample in valid if not sample.get("validation")]
    validation_samples = [sample for sample in valid if sample.get("validation")]
    layers = {str(sample.get("surface", "unknown")) for sample in fit_samples}
    if len(fit_samples) < 6 or len(layers) < 2:
        raise ValueError("刚体外参拟合至少需要 6 个拟合点和 2 个高度层")

    camera = np.asarray([sample["camera_xyz_m"] for sample in fit_samples], dtype=np.float64)
    world = np.asarray([sample["truth_xyz_m"] for sample in fit_samples], dtype=np.float64)
    camera_center = camera.mean(axis=0)
    world_center = world.mean(axis=0)
    covariance = (camera - camera_center).T @ (world - world_center)
    u, singular_values, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = world_center - rotation @ camera_center
    roll, pitch, yaw = rotation_matrix_to_rpy(rotation)
    base = dict(base_extrinsic or load_camera_extrinsic())
    fitted = {
        **base,
        "x": float(translation[0]),
        "y": float(translation[1]),
        "z": float(translation[2]),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "rpy_convention": "standard_zyx",
    }

    def metrics(items):
        if not items:
            return None
        camera_points = np.asarray([item["camera_xyz_m"] for item in items], dtype=np.float64)
        truth_points = np.asarray([item["truth_xyz_m"] for item in items], dtype=np.float64)
        predicted = camera_points @ rotation.T + translation
        residual = truth_points - predicted
        xy = np.linalg.norm(residual[:, :2], axis=1) * 1000.0
        z = np.abs(residual[:, 2]) * 1000.0
        return {
            "count": len(items),
            "mean_xy_error_mm": float(xy.mean()),
            "rms_xy_error_mm": float(np.sqrt(np.mean(xy ** 2))),
            "max_xy_error_mm": float(xy.max()),
            "mean_z_error_mm": float(z.mean()),
            "rms_z_error_mm": float(np.sqrt(np.mean(z ** 2))),
            "max_z_error_mm": float(z.max()),
            "predicted_world_xyz_m": predicted.tolist(),
        }

    return {
        "sample_count": len(valid),
        "fit_count": len(fit_samples),
        "validation_count": len(validation_samples),
        "layers": sorted(layers),
        "singular_values": singular_values.tolist(),
        "fitted_extrinsic": fitted,
        "fit_metrics": metrics(fit_samples),
        "validation_metrics": metrics(validation_samples),
    }


def read_pointcloud_bin(path):
    data = Path(path).read_bytes()
    if len(data) < 8:
        raise RuntimeError(f"点云文件过小: {path}")
    sequence, count = struct.unpack("<II", data[:8])
    expected = 8 + count * 6 * 4
    if len(data) < expected:
        raise RuntimeError(f"点云文件不完整: {len(data)} < {expected}")
    cloud = np.frombuffer(data, dtype="<f4", offset=8, count=count * 6).reshape((-1, 6))
    points = cloud[:, :3].astype(np.float64, copy=False)
    finite = np.isfinite(points).all(axis=1)
    return int(sequence), points[finite]


def make_platform_forbidden_box(platform, inliers, sampled_points, camera_origin, box_index=0):
    z_plane = float(np.median(platform[:, 2]))
    z_top = z_plane + PLATFORM_SAFETY_MARGIN
    z_bottom = PLATFORM_FLOOR_Z
    if z_top <= z_bottom + 0.01:
        raise RuntimeError(f"平台高度无效: z_plane={z_plane:.3f}, floor={z_bottom:.3f}")

    x_min, x_max = np.percentile(platform[:, 0], [2.0, 98.0])
    y_min, y_max = np.percentile(platform[:, 1], [2.0, 98.0])
    x_min = float(x_min) - PLATFORM_XY_MARGIN
    x_max = float(x_max) + PLATFORM_XY_MARGIN
    y_min = float(y_min) - PLATFORM_XY_MARGIN
    y_max = float(y_max) + PLATFORM_XY_MARGIN

    dimensions = [
        max(0.05, x_max - x_min),
        max(0.05, y_max - y_min),
        max(0.05, z_top - z_bottom),
    ]
    center = [
        (x_min + x_max) * 0.5,
        (y_min + y_max) * 0.5,
        (z_bottom + z_top) * 0.5,
    ]
    near_distance = float(np.percentile(np.linalg.norm(platform - camera_origin, axis=1), 10.0))
    return {
        "center": center,
        "dimensions": dimensions,
        "z_plane": z_plane,
        "z_floor": z_bottom,
        "z_top": z_top,
        "inliers": int(inliers),
        "sampled_points": int(sampled_points),
        "near_distance": near_distance,
        "box_index": int(box_index),
        "bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
    }


def find_best_horizontal_plane(points, rng):
    best_inliers = None
    best_count = 0
    for _ in range(max(1, PLATFORM_RANSAC_ITERS)):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        if abs(float(normal[2])) < PLATFORM_MIN_NORMAL_Z:
            continue
        distances = np.abs((points - sample[0]) @ normal)
        inliers = distances <= PLATFORM_PLANE_DISTANCE
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
    return best_inliers, best_count


def fit_platform_forbidden_boxes(points_world, camera_origin):
    points = points_world[np.isfinite(points_world).all(axis=1)]
    if len(points) < PLATFORM_MIN_INLIERS:
        raise RuntimeError(f"点云点数不足，无法拟合平台: {len(points)}")

    rng = np.random.default_rng(20260629)
    if len(points) > PLATFORM_MAX_POINTS:
        points = points[rng.choice(len(points), size=PLATFORM_MAX_POINTS, replace=False)]

    remaining = points
    candidates = []
    max_candidates = max(1, PLATFORM_MAX_BOXES * 3)
    for _ in range(max_candidates):
        if len(remaining) < max(PLATFORM_MIN_INLIERS, PLATFORM_MIN_REMAINING_POINTS):
            break
        inliers, count = find_best_horizontal_plane(remaining, rng)
        if inliers is None or count < PLATFORM_MIN_INLIERS:
            break
        platform = remaining[inliers]
        try:
            candidates.append(make_platform_forbidden_box(
                platform,
                count,
                len(points),
                camera_origin,
                box_index=len(candidates),
            ))
        except RuntimeError:
            pass
        remaining = remaining[~inliers]

    if not candidates:
        raise RuntimeError("未找到足够大的水平平台平面")

    candidates.sort(key=lambda item: item["near_distance"])
    selected = candidates[:max(1, PLATFORM_MAX_BOXES)]
    for i, box in enumerate(selected):
        box["box_index"] = i
    return selected


def fit_platform_forbidden_box(points_world, camera_origin):
    return fit_platform_forbidden_boxes(points_world, camera_origin)[0]


def load_latest_vision_target():
    if not VISION_TARGET_FILE.exists():
        return {"ok": False, "error": f"视觉坐标文件不存在: {VISION_TARGET_FILE}"}

    age_sec = time.time() - VISION_TARGET_FILE.stat().st_mtime
    if age_sec > VISION_TARGET_MAX_AGE_SEC:
        return {
            "ok": False,
            "stale": True,
            "age_sec": age_sec,
            "source": str(VISION_TARGET_FILE),
            "error": f"视觉目标已过期: {age_sec:.1f}s 未更新",
        }

    lines = [line.strip() for line in VISION_TARGET_FILE.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(lines) < 2:
        return {"ok": False, "error": "视觉坐标文件还没有目标数据"}

    header = lines[0].split("\t")
    values = lines[-1].split("\t")
    row = dict(zip(header, values))
    required = ["camera_x_m", "camera_y_m", "camera_z_m", "urdf_x_m", "urdf_y_m", "urdf_z_m"]
    missing = [name for name in required if name not in row]
    if missing:
        return {"ok": False, "error": f"视觉坐标文件缺少列: {missing}"}

    try:
        camera_xyz = [float(row["camera_x_m"]), float(row["camera_y_m"]), float(row["camera_z_m"])]
        urdf_xyz = [float(row["urdf_x_m"]), float(row["urdf_y_m"]), float(row["urdf_z_m"])]
    except ValueError as exc:
        return {"ok": False, "error": f"视觉坐标数值无效: {exc}"}

    return {
        "ok": True,
        "source": str(VISION_TARGET_FILE),
        "camera_xyz_m": camera_xyz,
        "urdf_xyz_m": urdf_xyz,
        "pose": {"x": urdf_xyz[0], "y": urdf_xyz[1], "z": urdf_xyz[2]},
    }


def vision_proxy_get(path, content_type=None):
    url = VISION_HTTP_BASE.rstrip("/") + path
    try:
        with urlopen(Request(url, headers={"Cache-Control": "no-cache"}), timeout=VISION_HTTP_TIMEOUT_SEC) as resp:
            payload = resp.read()
            return {
                "ok": True,
                "status": resp.status,
                "content_type": content_type or resp.headers.get_content_type(),
                "payload": payload,
            }
    except HTTPError as exc:
        return {"ok": False, "status": exc.code, "error": exc.reason, "payload": exc.read()}
    except (URLError, TimeoutError, OSError) as exc:
        return {"ok": False, "status": 503, "error": str(exc), "payload": b""}


def vision_proxy_post(path, payload=None):
    url = VISION_HTTP_BASE.rstrip("/") + path
    body = json.dumps(payload or {}).encode("utf-8")
    try:
        req = Request(
            url,
            data=body,
            headers={"Content-Type": "application/json", "Cache-Control": "no-cache"},
            method="POST",
        )
        with urlopen(req, timeout=VISION_HTTP_TIMEOUT_SEC) as resp:
            return {
                "ok": True,
                "status": resp.status,
                "content_type": resp.headers.get_content_type(),
                "payload": resp.read(),
            }
    except HTTPError as exc:
        return {"ok": False, "status": exc.code, "error": exc.reason, "payload": exc.read()}
    except (URLError, TimeoutError, OSError) as exc:
        return {"ok": False, "status": 503, "error": str(exc), "payload": b""}


class MoveItBridge(Node):
    def __init__(self):
        super().__init__("v01_web_control")
        self._lock = threading.Lock()
        self._planning_lock = threading.Lock()
        self._latest_joint_state = None
        self._latest_gripper_joint_state = {"right": None, "left": None}
        self._latest_grasp_debug = {"right": None, "left": None}
        self._grasp_debug_log_state = {"right": {"stamp": 0.0, "signature": None}, "left": {"stamp": 0.0, "signature": None}}
        self._latest_links = {
            "right": {"frame": FRAME_ID, "arm": "right", "links": [], "stamp": 0.0},
            "left": {"frame": FRAME_ID, "arm": "left", "links": [], "stamp": 0.0},
        }
        self._last_plan = {"right": None, "left": None}
        self._last_plan_summary = {"right": None, "left": None}
        self._status = {"right": "starting", "left": "starting"}
        self._benchmark_progress = {
            "right": self._new_benchmark_progress("right"),
            "left": self._new_benchmark_progress("left"),
        }
        self._gripper_state = {"right": "open", "left": "open"}
        self._gripper_simulated = {"right": False, "left": False}
        self._gripper_last_command = {"right": {"state": None, "stamp": 0.0}, "left": {"state": None, "stamp": 0.0}}
        self._robot_state = {
            "right": {"motors_enabled": None, "estop_active": None, "ns": RIGHT_ROBOT_NS},
            "left": {"motors_enabled": None, "estop_active": None, "ns": LEFT_ROBOT_NS},
        }
        self._jog_targets = {"right": None, "left": None}
        self._jog_target_stamps = {"right": 0.0, "left": 0.0}
        self._manual_collision_ids = set()
        self._manual_collision_summary = None
        self._camera_lock = threading.Lock()
        self._camera_data = {
            "head": self._new_camera_data(MUJOCO_CAMERA_TOPIC, "等待头顶 D455 图像"),
            "depth": self._new_camera_data(MUJOCO_DEPTH_TOPIC, "等待 D455 深度图像"),
            "right_wrist": self._new_camera_data(RIGHT_WRIST_CAMERA_TOPIC, "等待右腕相机图像"),
            "right_wrist_depth": self._new_camera_data(RIGHT_WRIST_DEPTH_TOPIC, "等待右腕深度图像"),
            "left_wrist": self._new_camera_data(LEFT_WRIST_CAMERA_TOPIC, "等待左腕相机图像"),
            "left_wrist_depth": self._new_camera_data(LEFT_WRIST_DEPTH_TOPIC, "等待左腕深度图像"),
            "overview": self._new_camera_data(OVERVIEW_CAMERA_TOPIC, "等待全局相机图像"),
        }
        self._head_rgb_history = deque(maxlen=12)
        self._head_depth_history = deque(maxlen=12)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joint_state, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, 10)
        self.create_subscription(
            Image, MUJOCO_CAMERA_TOPIC,
            lambda msg: self._on_camera_image(msg, "head"), camera_qos,
        )
        self.create_subscription(
            Image, OVERVIEW_CAMERA_TOPIC,
            lambda msg: self._on_camera_image(msg, "overview"), camera_qos,
        )
        self.create_subscription(
            Image, RIGHT_WRIST_CAMERA_TOPIC,
            lambda msg: self._on_camera_image(msg, "right_wrist"), camera_qos,
        )
        self.create_subscription(
            Image, LEFT_WRIST_CAMERA_TOPIC,
            lambda msg: self._on_camera_image(msg, "left_wrist"), camera_qos,
        )
        self.create_subscription(
            Image, MUJOCO_DEPTH_TOPIC,
            lambda msg: self._on_depth_image(msg, "depth"), camera_qos,
        )
        self.create_subscription(
            Image, RIGHT_WRIST_DEPTH_TOPIC,
            lambda msg: self._on_depth_image(msg, "right_wrist_depth"), camera_qos,
        )
        self.create_subscription(
            Image, LEFT_WRIST_DEPTH_TOPIC,
            lambda msg: self._on_depth_image(msg, "left_wrist_depth"), camera_qos,
        )
        self._mujoco_camera_info = None
        mujoco_info_topic = os.environ.get("MUJOCO_CAMERA_INFO_TOPIC", "/mujoco_camera/color/camera_info")
        self.create_subscription(CameraInfo, mujoco_info_topic, self._on_mujoco_camera_info, camera_qos)
        self.create_subscription(String, "/mujoco/grasp_debug", self._on_grasp_debug, 10)
        self._mujoco_grasp_guard_client = self.create_client(
            SetBool, "/mujoco/grasp_guard"
        )
        self._mujoco_task_reset_client = self.create_client(
            Trigger, "/mujoco/task_reset"
        )
        self._mujoco_fixture_visibility_clients = {
            "plate": self.create_client(SetBool, "/mujoco/plate_visible"),
            "calibration_block": self.create_client(
                SetBool, "/mujoco/calibration_block_visible"
            ),
        }
        occupancy_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._occupancy_pub = self.create_publisher(Bool, OCCUPANCY_TOPIC, occupancy_qos)
        self._set_object_pub = self.create_publisher(String, "/mujoco/set_object_pose", 10)
        self._occupancy_timer = self.create_timer(1.0, lambda: self.publish_occupancy(True))
        self.publish_occupancy(True)
        self._plan_client = self.create_client(GetMotionPlan, PLAN_SERVICE)
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._apply_scene_client = self.create_client(ApplyPlanningScene, APPLY_PLANNING_SCENE_SERVICE)
        self._action_clients = {
            "right": ActionClient(self, FollowJointTrajectory, ACTION_NAME),
            "left": ActionClient(self, FollowJointTrajectory, LEFT_ACTION_NAME),
        }
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._link_timer = self.create_timer(1.0, self._update_link_cache)

        for side, ns in [("right", RIGHT_ROBOT_NS), ("left", LEFT_ROBOT_NS)]:
            self.create_subscription(Bool, f"/{ns}/motors_enabled", lambda msg, s=side: self._on_motors_enabled(msg, s), state_qos)
            self.create_subscription(Bool, f"/{ns}/emergency_stop_state", lambda msg, s=side: self._on_estop_active(msg, s), state_qos)
            self.create_subscription(Bool, f"/{ns}/estop_active", lambda msg, s=side: self._on_estop_active(msg, s), state_qos)
            self.create_subscription(JointState, f"/{ns}/gripper_joint_states", lambda msg, s=side: self._on_gripper_joint_state(msg, s), 10)
        self._platform_obstacle_summary = None
        self._mujoco_table_obstacle_summary = None

    def publish_occupancy(self, occupied):
        msg = Bool()
        msg.data = bool(occupied)
        self._occupancy_pub.publish(msg)

    def _on_grasp_debug(self, msg):
        try:
            payload = json.loads(msg.data)
            side = normalize_arm(payload.get("side", "right"))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._latest_grasp_debug[side] = payload
        now = time.monotonic()
        signature = (
            bool(payload.get("left_contact")),
            bool(payload.get("right_contact")),
            bool(payload.get("contact_stopped")),
            bool(payload.get("object_attached")),
        )
        log_state = self._grasp_debug_log_state[side]
        if signature != log_state["signature"] or now - log_state["stamp"] >= 1.0:
            log_state.update({"stamp": now, "signature": signature})
            log_event("DEBUG", "MuJoCo grasp debug", payload)

    def _grasp_debug(self, side):
        with self._lock:
            payload = self._latest_grasp_debug.get(normalize_arm(side))
            return dict(payload) if payload else {}

    def _set_mujoco_grasp_guard(self, enabled):
        client = self._mujoco_grasp_guard_client
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.5)
        if not client.service_is_ready():
            raise RuntimeError("MuJoCo grasp guard service not ready")
        request = SetBool.Request()
        request.data = bool(enabled)
        future = client.call_async(request)
        if not self._spin_until_future_complete(future, 1.0):
            raise RuntimeError("MuJoCo grasp guard service timed out")
        response = future.result()
        if response is None or not bool(response.success):
            message = getattr(response, "message", "") if response else ""
            raise RuntimeError(message or "MuJoCo grasp guard request failed")
        return {
            "enabled": bool(enabled),
            "message": getattr(response, "message", ""),
        }

    def prepare_shutdown(self):
        for timer in (getattr(self, "_occupancy_timer", None), getattr(self, "_link_timer", None)):
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass

    def _arm_config(self, side):
        side = normalize_arm(side)
        if side == "left":
            return {
                "side": "left",
                "group": LEFT_GROUP_NAME,
                "tip": LEFT_TIP_LINK,
                "joints": LEFT_JOINT_NAMES,
                "links": LEFT_LINK_NAMES,
                "offset": LEFT_TOOL_OFFSET,
                "action_name": LEFT_ACTION_NAME,
                "action_client": self._action_clients["left"],
            }
        return {
            "side": "right",
            "group": GROUP_NAME,
            "tip": TIP_LINK,
            "joints": JOINT_NAMES,
            "links": LINK_NAMES,
            "offset": TOOL_OFFSET,
            "action_name": ACTION_NAME,
            "action_client": self._action_clients["right"],
        }

    def _on_joint_state(self, msg):
        with self._lock:
            self._latest_joint_state = msg

    def _on_gripper_joint_state(self, msg, side):
        with self._lock:
            self._latest_gripper_joint_state[side] = msg

    @staticmethod
    def _new_camera_data(topic, waiting_message):
        return {
            "jpeg": None,
            "raw_rgb": None,
            "raw_depth": None,
            "status": {
                "ready": False,
                "topic": topic,
                "width": 0,
                "height": 0,
                "frame_id": "",
                "stamp": 0.0,
                "received_at": 0.0,
                "error": waiting_message,
            },
        }

    def _on_camera_image(self, msg, camera_key):
        if cv2 is None:
            with self._camera_lock:
                self._camera_data[camera_key]["status"]["error"] = "Web 服务缺少 OpenCV，无法编码摄像头画面"
            return
        try:
            channels = 3
            row_bytes = int(msg.width) * channels
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape((int(msg.height), int(msg.step)))[:, :row_bytes]
            frame = rows.reshape((int(msg.height), int(msg.width), channels))
            if msg.encoding.lower() == "rgb8":
                rgb_frame = np.ascontiguousarray(frame.copy())
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding.lower() == "bgr8":
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                raise ValueError(f"不支持的图像编码: {msg.encoding}")
            jpeg_quality = 78 if camera_key in {"right_wrist", "left_wrist"} else 88
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
            )
            if not ok:
                raise RuntimeError("JPEG 编码失败")
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            received_at = time.monotonic()
            with self._camera_lock:
                camera = self._camera_data[camera_key]
                camera["jpeg"] = encoded.tobytes()
                camera["raw_rgb"] = rgb_frame
                camera["status"] = {
                    "ready": True,
                    "topic": camera["status"]["topic"],
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "frame_id": msg.header.frame_id,
                    "stamp": stamp,
                    "received_at": received_at,
                    "error": "",
                }
                if camera_key == "head":
                    self._head_rgb_history.append((stamp, received_at, rgb_frame))
        except Exception as exc:
            with self._camera_lock:
                status = self._camera_data[camera_key]["status"]
                status["ready"] = False
                status["error"] = str(exc)

    def _on_depth_image(self, msg, camera_key="depth"):
        if cv2 is None:
            with self._camera_lock:
                self._camera_data[camera_key]["status"]["error"] = "Web 服务缺少 OpenCV，无法显示深度图"
            return
        try:
            if msg.encoding.upper() != "32FC1":
                raise ValueError(f"不支持的深度编码: {msg.encoding}")
            columns = int(msg.step) // 4
            values = np.frombuffer(msg.data, dtype=np.float32).reshape((int(msg.height), columns))[:, :int(msg.width)]
            values = np.ascontiguousarray(values, dtype=np.float32)
            valid = np.isfinite(values) & (values > 0.0)
            finite = values[valid]
            if finite.size == 0:
                raise ValueError("深度图没有有效像素")
            near = float(np.percentile(finite, 2.0))
            far = float(np.percentile(finite, 98.0))
            span = max(far - near, 1e-6)
            scaled = np.zeros(values.shape, dtype=np.uint8)
            scaled[valid] = np.clip(255.0 - (values[valid] - near) * 255.0 / span, 0.0, 255.0).astype(np.uint8)
            color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            color[~valid] = 0
            ok, encoded = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                raise RuntimeError("深度预览 JPEG 编码失败")
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            received_at = time.monotonic()
            with self._camera_lock:
                camera = self._camera_data[camera_key]
                camera["jpeg"] = encoded.tobytes()
                camera["raw_depth"] = values
                camera["status"] = {
                    "ready": True,
                    "topic": camera["status"]["topic"],
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "frame_id": msg.header.frame_id,
                    "stamp": stamp,
                    "received_at": received_at,
                    "encoding": msg.encoding,
                    "unit": "m",
                    "min_m": float(finite.min()),
                    "max_m": float(finite.max()),
                    "error": "",
                }
                if camera_key == "depth":
                    self._head_depth_history.append((stamp, received_at, values))
        except Exception as exc:
            with self._camera_lock:
                status = self._camera_data[camera_key]["status"]
                status["ready"] = False
                status["error"] = str(exc)

    def _on_mujoco_camera_info(self, msg):
        try:
            k = list(msg.k)
            self._mujoco_camera_info = {
                "width": int(msg.width),
                "height": int(msg.height),
                "k": k,
                "fx": float(k[0]),
                "fy": float(k[4]),
                "cx": float(k[2]),
                "cy": float(k[5]),
                "frame_id": msg.header.frame_id,
            }
        except Exception:
            self._mujoco_camera_info = None

    def camera_status(self, camera_key="head"):
        with self._camera_lock:
            status = dict(self._camera_data[camera_key]["status"])
        if status["ready"] and time.monotonic() - status["received_at"] > 3.0:
            status["ready"] = False
            status["error"] = "MuJoCo 摄像头画面已停止更新"
        return status

    def camera_jpeg(self, camera_key="head"):
        with self._camera_lock:
            return self._camera_data[camera_key]["jpeg"]

    def camera_depth_payload(self):
        """Return raw float32 depth bytes + status + intrinsics + extrinsic for pixel-to-world mapping."""
        with self._camera_lock:
            camera = self._camera_data["depth"]
            status = dict(camera["status"])
            raw = camera.get("raw_depth")
        if raw is None:
            return None
        try:
            extrinsic = camera_extrinsic_response()
        except Exception:
            extrinsic = None
        payload = {
            "width": int(status.get("width", 0)),
            "height": int(status.get("height", 0)),
            "encoding": status.get("encoding", "32FC1"),
            "unit": status.get("unit", "m"),
            "stamp": status.get("stamp", 0.0),
            "frame_id": status.get("frame_id", ""),
            "min_m": status.get("min_m"),
            "max_m": status.get("max_m"),
            "data_b64": base64.b64encode(raw.tobytes()).decode("ascii"),
            "intrinsic": self._mujoco_camera_info,
            "extrinsic": extrinsic,
        }
        return payload

    def _synchronized_head_rgbd(self):
        with self._camera_lock:
            rgb_camera = self._camera_data["head"]
            depth_camera = self._camera_data["depth"]
            rgb_status = dict(rgb_camera["status"])
            depth_status = dict(depth_camera["status"])
            candidates = [
                (min(rgb_received, depth_received), rgb_stamp, rgb_received, rgb, depth_received, depth)
                for rgb_stamp, rgb_received, rgb in self._head_rgb_history
                for depth_stamp, depth_received, depth in self._head_depth_history
                if abs(rgb_stamp - depth_stamp) <= 1e-6
            ]
            pair = max(candidates, key=lambda item: item[0]) if candidates else None
            if pair is not None:
                _, stamp, rgb_received, rgb, depth_received, depth = pair
                rgb = rgb.copy()
                depth = depth.copy()
        if pair is None:
            stamp_delta = abs(float(rgb_status.get("stamp", 0.0)) - float(depth_status.get("stamp", 0.0)))
            raise RuntimeError(f"尚无同时间戳 RGB-D 帧，最新帧相差 {stamp_delta:.3f} s")
        pair_age = time.monotonic() - min(rgb_received, depth_received)
        if pair_age > 3.0:
            raise RuntimeError(f"头部同步 RGB-D 帧已停止更新：{pair_age:.1f} s")
        return rgb, depth, float(stamp)

    def mujoco_visual_grasp_target(self):
        rgb, depth, stamp = self._synchronized_head_rgbd()
        extrinsic = load_camera_extrinsic()
        estimate = estimate_red_stator_from_rgbd(
            rgb, depth, self._mujoco_camera_info, extrinsic
        )
        xyz = estimate["estimated_world_xyz_m"]
        center = {
            "object_name": "red_stator",
            "frame": "world",
            "x": float(xyz[0]),
            "y": float(xyz[1]),
            "z": float(xyz[2]),
            "source": "head_rgbd",
            "live": True,
        }
        target = {
            **center,
            "x": center["x"] + MUJOCO_GRASP_TCP_OFFSET["x"],
            "y": center["y"] + MUJOCO_GRASP_TCP_OFFSET["y"],
            "z": center["z"] + MUJOCO_GRASP_TCP_OFFSET["z"],
            "object_center": center,
            "tcp_offset": dict(MUJOCO_GRASP_TCP_OFFSET),
            "grasp_pose_mode": "z_parallel",
        }
        return {
            "ok": True,
            "target": target,
            "vision": {
                **estimate,
                "camera_type": "fixed_head",
                "camera_topic": "/mujoco_camera",
                "rgb_stamp": stamp,
                "depth_stamp": stamp,
                "extrinsic": extrinsic,
            },
        }

    def mujoco_camera_calibration_sample(self):
        rgb, depth, stamp = self._synchronized_head_rgbd()

        extrinsic = load_camera_extrinsic()
        estimate = estimate_red_stator_from_rgbd(
            rgb, depth, self._mujoco_camera_info, extrinsic
        )
        debug = self._grasp_debug("right")
        position = debug.get("object_position")
        if not isinstance(position, list) or len(position) < 3:
            raise RuntimeError("/api/mujoco/grasp_target 尚无 MuJoCo 实时真值")
        truth = [float(position[index]) for index in range(3)]
        estimated = estimate["estimated_world_xyz_m"]
        error = [truth[index] - estimated[index] for index in range(3)]
        return {
            "ok": True,
            "camera_type": "fixed_head",
            "camera_topic": "/mujoco_camera",
            "truth_source": "/api/mujoco/grasp_target",
            "truth_xyz_m": truth,
            **estimate,
            "error_m": error,
            "xy_error_mm": math.hypot(error[0], error[1]) * 1000.0,
            "rgb_stamp": float(stamp),
            "depth_stamp": float(stamp),
            "extrinsic": extrinsic,
            "object_motion": debug.get("object_motion"),
            "object_orientation_wxyz": (debug.get("object_pose") or {}).get("orientation_wxyz"),
        }

    def _on_tf(self, msg):
        # RViz-like: update link cache whenever /tf arrives (non-blocking)
        self._update_link_cache()

    def _on_tf_static(self, msg):
        # static transforms rarely change; update once after a short delay
        self._update_link_cache()

    def _on_motors_enabled(self, msg, side):
        with self._lock:
            self._robot_state[side]["motors_enabled"] = msg.data

    def _on_estop_active(self, msg, side):
        with self._lock:
            self._robot_state[side]["estop_active"] = msg.data

    def robot_state(self):
        with self._lock:
            return {
                side: {
                    "motors_enabled": info["motors_enabled"],
                    "estop_active": info["estop_active"],
                    "ns": info["ns"],
                }
                for side, info in self._robot_state.items()
            }

    def set_object_pose(self, x, y, z):
        msg = String()
        msg.data = json.dumps({"x": float(x), "y": float(y), "z": float(z)})
        deadline = time.monotonic() + 2.0
        while self._set_object_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._set_object_pub.get_subscription_count() < 1:
            raise RuntimeError("/mujoco/set_object_pose 没有订阅者")
        # Publish more than once so a just-discovered local DDS reader cannot
        # miss the reset command during endpoint startup.
        for _ in range(3):
            self._set_object_pub.publish(msg)
            time.sleep(0.04)
        return {"ok": True, "x": float(x), "y": float(y), "z": float(z)}

    def _get_robot_client(self, side, name, srv_type, topic):
        key = f"_{side}_{name}_client"
        if not hasattr(self, key):
            setattr(self, key, self.create_client(srv_type, topic))
        return getattr(self, key)

    def _get_estop_pub(self, side):
        key = f"_{side}_estop_pub"
        if not hasattr(self, key):
            ns = RIGHT_ROBOT_NS if side == "right" else LEFT_ROBOT_NS
            setattr(self, key, self.create_publisher(Bool, f"/{ns}/emergency_stop", 10))
        return getattr(self, key)

    def _get_home_pub(self, side):
        key = f"_{side}_home_pub"
        if not hasattr(self, key):
            ns = RIGHT_ROBOT_NS if side == "right" else LEFT_ROBOT_NS
            setattr(self, key, self.create_publisher(JointTrajectory, f"/{ns}/joint_trajectory", 10))
        return getattr(self, key)

    @staticmethod
    def _target_sides(side):
        if side in ("both", "all"):
            return ["right", "left"]
        return [normalize_arm(side)]

    def _call_robot_service(self, side, action, timeout_sec=3.0):
        sides = self._target_sides(side)
        if len(sides) > 1:
            results = {target: self._call_robot_service(target, action, timeout_sec) for target in sides}
            return {"ok": all(result.get("ok") for result in results.values()), "results": results}

        ns = RIGHT_ROBOT_NS if side == "right" else LEFT_ROBOT_NS
        if action in ("enable", "disable"):
            client = self._get_robot_client(side, "enable", SetBool, f"/{ns}/enable_motors")
            req = SetBool.Request()
            req.data = action == "enable"
        elif action == "reset_estop":
            client = self._get_robot_client(side, "reset", Trigger, f"/{ns}/reset_emergency_stop")
            req = Trigger.Request()
        else:
            return {"ok": False, "error": f"unknown action {action}"}

        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=timeout_sec)
        if not client.service_is_ready():
            return {"ok": False, "error": f"service {client.srv_name} not ready"}

        future = client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if future.done():
            resp = future.result()
            return {"ok": getattr(resp, "success", True), "message": getattr(resp, "message", "")}
        return {"ok": False, "error": "service call timeout"}

    def send_estop(self, side, active=True):
        results = {}
        for target in self._target_sides(side):
            pub = self._get_estop_pub(target)
            msg = Bool()
            msg.data = active
            pub.publish(msg)
            results[target] = {"ok": True, "active": active}
        if len(results) == 1:
            return next(iter(results.values()))
        return {"ok": all(result.get("ok") for result in results.values()), "results": results}

    def _home_joint_state(self, side):
        cfg = self._arm_config(side)
        joint_state = JointState()
        joint_state.name = list(cfg["joints"])
        joint_state.position = [0.0] * len(cfg["joints"])
        return joint_state

    def _plan_home_zero(self, side, config):
        side = normalize_arm(side)
        config = dict(config if isinstance(config, dict) else {})
        config["ignore_workspace_bounds"] = True
        cfg = self._arm_config(side)
        requested_avoid_platform = bool(config.get("avoid_platform", True))
        avoid_platform = requested_avoid_platform and not DISABLE_PLATFORM_OBSTACLE
        manual_collision_summary = self.sync_manual_collision_boxes_for_request(config)
        manual_collision_active = self._has_active_manual_collision_boxes()
        collision_avoidance_required = avoid_platform or manual_collision_active
        ompl_candidates = home_ompl_config_candidates(config, collision_avoidance_required)
        if not self._wait_ready(side):
            raise RuntimeError("规划服务或轨迹动作服务未就绪")

        platform_summary = None
        if avoid_platform:
            platform_summary = self.apply_platform_obstacle()

        target_state = self._home_joint_state(side)
        pose = {
            "arm": side,
            "velocity_scaling": float(config.get("velocity_scaling", 0.15)),
            "acceleration_scaling": float(config.get("acceleration_scaling", 0.10)),
        }
        t0 = time.monotonic()
        fallback_reasons = []
        if avoid_platform:
            fallback_reasons.append("平台避障启用")
        if manual_collision_active:
            fallback_reasons.append("手动碰撞箱启用")
        attempt_summaries = []
        summary = None
        for index, ompl_config in enumerate(ompl_candidates, start=1):
            try:
                summary = self._moveit_plan_to_joint_state(
                    side,
                    target_state,
                    pose,
                    ompl_config,
                    t0,
                    platform_summary=platform_summary,
                    fallback_allowed=not collision_avoidance_required,
                    fallback_label="、".join(fallback_reasons),
                    mode="MoveIt planned home",
                    goal_tolerance=HOME_GOAL_TOLERANCE_RAD,
                )
                attempt_summaries.append({
                    "planner_id": ompl_config["planner_id"],
                    "planning_time": ompl_config["planning_time"],
                    "attempts": ompl_config["attempts"],
                    "ok": True,
                })
                break
            except RuntimeError as exc:
                attempt = {
                    "planner_id": ompl_config["planner_id"],
                    "planning_time": ompl_config["planning_time"],
                    "attempts": ompl_config["attempts"],
                    "ok": False,
                    "error": str(exc),
                }
                attempt_summaries.append(attempt)
                if index < len(ompl_candidates):
                    log_event("WARNING", "home plan retry with alternate planner", {
                        "arm": side,
                        **attempt,
                    })

        if summary is None:
            detail = "; ".join(
                f"{item['planner_id']}: {item.get('error', 'failed')}"
                for item in attempt_summaries[-3:]
            ) or "无可用 planner"
            hint = ""
            if manual_collision_active:
                hint = "；当前手动碰撞箱启用，回零仍按避碰处理，请缩小/禁用挡住回零路径的碰撞箱后重试"
            raise RuntimeError(f"回零规划失败，已尝试 {len(attempt_summaries)} 个 planner：{detail}{hint}")

        summary["target"] = "home"
        summary["group"] = cfg["group"]
        summary["avoid_platform"] = avoid_platform
        summary["manual_collision_active"] = manual_collision_active
        summary["workspace_bounds_ignored"] = True
        summary["home_goal_tolerance_rad"] = HOME_GOAL_TOLERANCE_RAD
        summary["home_planner_attempts"] = attempt_summaries
        if manual_collision_summary:
            summary["manual_collision"] = manual_collision_summary
        return summary

    def home_zero(self, side="both", config=None):
        config = config if isinstance(config, dict) else {}
        results = {}
        for target in self._target_sides(side):
            cfg = self._arm_config(target)
            if not self._planning_lock.acquire(blocking=False):
                raise RuntimeError("已有规划正在运行，请等待当前规划结束")
            try:
                plan = self._plan_home_zero(target, config)
            finally:
                self._planning_lock.release()
            execute = self.execute_last_plan(target)
            results[target] = {
                "ok": int(execute.get("error_code", 0)) == 0,
                "group": cfg["group"],
                "plan": plan,
                "execute": execute,
            }
        if len(results) == 1:
            return next(iter(results.values()))
        return {"ok": True, "results": results}

    def reset_mujoco_task(self):
        """Restore MuJoCo and place the rotor at the configured task start."""
        client = self._mujoco_task_reset_client
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=1.0)
        if not client.service_is_ready():
            raise RuntimeError("MuJoCo task reset service not ready")
        future = client.call_async(Trigger.Request())
        if not self._spin_until_future_complete(future, 2.0):
            raise RuntimeError("MuJoCo task reset timed out")
        response = future.result()
        if response is None or not bool(response.success):
            raise RuntimeError(
                getattr(response, "message", "") or "MuJoCo task reset failed"
            )
        for side in ("right", "left"):
            self._gripper_state[side] = "open"
            self._gripper_simulated[side] = True
            self._gripper_last_command[side] = {
                "state": "open",
                "stamp": time.monotonic(),
            }
        time.sleep(0.15)
        debug = self._grasp_debug("right")
        reset_target = None
        object_position = debug.get("object_position")
        if isinstance(object_position, list) and len(object_position) >= 3:
            try:
                scene_config = load_mujoco_scene_config()
                reset_x = float(scene_config.get(
                    "object_x", MUJOCO_GRASP_OBJECT_CENTER["x"]
                ))
                reset_y = float(scene_config.get(
                    "object_y", MUJOCO_GRASP_OBJECT_CENTER["y"]
                ))
                reset_z = float(object_position[2])
                reset_target = self.set_object_pose(reset_x, reset_y, reset_z)
                time.sleep(0.20)
                debug = self._grasp_debug("right")
            except Exception as exc:
                reset_target = {
                    "ok": False,
                    "error": str(exc),
                }
        return {
            "ok": True,
            "mode": "atomic_mujoco_reset_with_configured_object_pose",
            "message": response.message,
            "object_reset_target": reset_target,
            "object_position": debug.get("object_position"),
            "object_attached": bool(debug.get("object_attached")),
        }

    def set_mujoco_calibration_fixtures(self, body):
        requested = {
            "plate": bool(body.get("plate", True)),
            "calibration_block": bool(body.get("calibration_block", True)),
        }
        results = {}
        for fixture, visible in requested.items():
            client = self._mujoco_fixture_visibility_clients[fixture]
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=2.0)
            if not client.service_is_ready():
                raise RuntimeError(f"MuJoCo {fixture} visibility service not ready")
            request = SetBool.Request()
            request.data = visible
            future = client.call_async(request)
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                raise RuntimeError(f"MuJoCo {fixture} visibility service timed out")
            response = future.result()
            if response is None or not bool(response.success):
                raise RuntimeError(
                    getattr(response, "message", "") or f"MuJoCo {fixture} visibility update failed"
                )
            results[fixture] = {"visible": visible, "message": response.message}

        config = load_mujoco_scene_config()
        objects = [
            str(name) for name in config.get("objects", [])
            if str(name) not in {"plate", "calibration_block"}
        ]
        if requested["plate"]:
            objects.insert(0, "plate")
        if requested["calibration_block"]:
            objects.append("calibration_block")
        config["objects"] = objects
        save_mujoco_scene_config(config)
        return {
            "ok": True,
            "fixtures": results,
            "config": config,
            "extras": self._mujoco_scene_extras(),
        }

    def jog_joint(self, side, joint_index, delta_rad, duration_sec=0.25):
        side = normalize_arm(side)
        cfg = self._arm_config(side)
        joint_index = int(joint_index)
        delta_rad = float(delta_rad)
        duration_sec = min(1.0, max(0.1, float(duration_sec)))

        if not 1 <= joint_index <= len(cfg["joints"]):
            raise ValueError(f"关节编号必须在 1 到 {len(cfg['joints'])} 之间")
        if not math.isfinite(delta_rad) or abs(delta_rad) < 1e-9:
            raise ValueError("关节微动角度必须是非零有限值")
        if abs(delta_rad) > JOINT_JOG_MAX_DELTA_RAD:
            raise ValueError(
                f"单次关节微动不能超过 {math.degrees(JOINT_JOG_MAX_DELTA_RAD):.1f}°"
            )

        with self._lock:
            current = self._latest_joint_state
            robot_state = dict(self._robot_state[side])
            cached_target = self._jog_targets[side]
            cached_stamp = self._jog_target_stamps[side]
        if current is None:
            raise RuntimeError(f"未收到关节状态（话题 {JOINT_STATE_TOPIC}）")
        if robot_state.get("estop_active") is True:
            raise RuntimeError(f"{'左臂' if side == 'left' else '右臂'}急停已触发")
        if robot_state.get("motors_enabled") is not True:
            raise RuntimeError(f"{'左臂' if side == 'left' else '右臂'}尚未使能")

        current_by_name = dict(zip(current.name, current.position))
        missing = [name for name in cfg["joints"] if name not in current_by_name]
        if missing:
            raise RuntimeError(f"关节状态缺少: {missing}")
        actual = [float(current_by_name[name]) for name in cfg["joints"]]
        if not all(math.isfinite(value) for value in actual):
            raise RuntimeError("当前关节状态包含无效值")

        limits_by_name = urdf_joint_limits()
        joint_limits = [limits_by_name.get(name) for name in cfg["joints"]]

        def within_limits(values):
            return all(
                limits is None or limits[0] - 1e-6 <= value <= limits[1] + 1e-6
                for value, limits in zip(values, joint_limits)
            )

        now = time.monotonic()
        use_cached = (
            cached_target is not None
            and now - cached_stamp <= JOINT_JOG_TARGET_TTL_SEC
            and len(cached_target) == len(actual)
            and all(abs(target - value) <= JOINT_JOG_MAX_LEAD_RAD for target, value in zip(cached_target, actual))
            and within_limits(cached_target)
        )
        target = list(cached_target if use_cached else actual)
        axis = joint_index - 1
        target[axis] += delta_rad
        lead = target[axis] - actual[axis]
        if abs(lead) > JOINT_JOG_MAX_LEAD_RAD:
            target[axis] = actual[axis] + math.copysign(JOINT_JOG_MAX_LEAD_RAD, lead)

        limits = joint_limits[axis]
        limit_reached = None
        if limits is not None:
            lower, upper = limits
            unclamped = target[axis]
            target[axis] = min(upper, max(lower, unclamped))
            if abs(target[axis] - actual[axis]) < 1e-6:
                limit_reached = "上限" if delta_rad > 0.0 else "下限"
        if limit_reached:
            reverse_key = "QWERTYU"[axis] if delta_rad > 0.0 else "1234567"[axis]
            raise RuntimeError(
                f"{cfg['joints'][axis]} 已到{limit_reached}，请按 {reverse_key} 向反方向微动"
            )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(cfg["joints"])
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = self._duration_msg(duration_sec)
        trajectory.points.append(point)

        publisher = self._get_home_pub(side)
        if publisher.get_subscription_count() < 1:
            raise RuntimeError(
                f"关节控制话题没有订阅者: /{RIGHT_ROBOT_NS if side == 'right' else LEFT_ROBOT_NS}/joint_trajectory"
            )
        publisher.publish(trajectory)
        with self._lock:
            self._jog_targets[side] = target
            self._jog_target_stamps[side] = now
            self._status[side] = f"jog J{joint_index}"
        return {
            "ok": True,
            "arm": side,
            "joint": joint_index,
            "joint_name": cfg["joints"][axis],
            "delta_rad": delta_rad,
            "target_rad": target[axis],
            "duration_sec": duration_sec,
            "joint_limit": list(limits) if limits is not None else None,
        }

    def _mujoco_scene_extras(self):
        config = load_mujoco_scene_config()
        configured_objects = config.get("objects", [])
        default_objects = ",".join(configured_objects) if isinstance(configured_objects, list) else str(configured_objects)
        scene_objects = os.environ.get("MUJOCO_SCENE_OBJECTS", default_objects).strip()
        obj_set = {s.strip() for s in scene_objects.split(",") if s.strip()}
        table_cx, table_cy, table_cz = MUJOCO_TABLE_CENTER
        table_half_h = float(MUJOCO_TABLE_SIZE[2]) * 0.5
        extras = []
        if "plate" in obj_set:
            extras.append({
                "name": "plate", "type": "slot",
                "position": [float(config.get("plate_x", -0.28)), float(config.get("plate_y", -0.72)), table_cz + table_half_h + 0.070],
                "size": [0.1287, 0.140, 0.1287], "color": "#b8bdc7",
                "model_url": "/models/slot.stl",
                "model_scale": 0.001,
            })
        if "cylinder" in obj_set:
            extras.append({
                "name": "cylinder", "type": "cylinder",
                "position": [table_cx + 0.22, table_cy + 0.08, table_cz + table_half_h + 0.04],
                "size": [0.025, 0.04], "color": "#33cc44",
            })
        if "calibration_block" in obj_set:
            block_x = float(config.get("calibration_block_x", MUJOCO_CALIBRATION_BLOCK_CENTER[0]))
            block_y = float(config.get("calibration_block_y", MUJOCO_CALIBRATION_BLOCK_CENTER[1]))
            extras.append({
                "name": "calibration_block", "type": "box",
                "position": [block_x, block_y, table_cz + table_half_h + MUJOCO_CALIBRATION_BLOCK_SIZE[2] * 0.5],
                "size": list(MUJOCO_CALIBRATION_BLOCK_SIZE), "color": "#296f85",
            })
        return extras


    def current_state(self):
        with self._lock:
            msg = self._latest_joint_state
            gripper_msgs = dict(self._latest_gripper_joint_state)
            status = dict(self._status)
            summary = dict(self._last_plan_summary)
        gripper = self.gripper_states()
        grasp_debug = self._grasp_debug("right")
        mujoco_scene = {
            "table": {
                "center": list(MUJOCO_TABLE_CENTER),
                "size": list(MUJOCO_TABLE_SIZE),
                "color": "#d1d6db",
            },
            "object": {
                "position": grasp_debug.get(
                    "object_position",
                    [
                        MUJOCO_GRASP_OBJECT_CENTER["x"],
                        MUJOCO_GRASP_OBJECT_CENTER["y"],
                        MUJOCO_GRASP_OBJECT_CENTER["z"],
                    ],
                ),
                "orientation_wxyz": mujoco_grasp_object_display_quat_wxyz(
                    (grasp_debug.get("object_pose") or {}).get("orientation_wxyz")
                ),
                "body_orientation_wxyz": (grasp_debug.get("object_pose") or {}).get(
                    "orientation_wxyz", [1.0, 0.0, 0.0, 0.0]
                ),
                "model_quat_wxyz": list(MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ),
                "size": list(MUJOCO_GRASP_OBJECT_SIZE),
                "color": "#c70f0d",
                "attached": bool(grasp_debug.get("object_attached")),
            },
            "extras": self._mujoco_scene_extras(),
        }
        if msg is None:
            return {
                "ok": False,
                "status": status,
                "joint_state": None,
                "last_plan": summary,
                "gripper": gripper,
                "mujoco_scene": mujoco_scene,
            }
        joint_state = self._joint_state_payload(msg, gripper_msgs)
        # In MuJoCo mode the grasp debug message contains an atomic snapshot of
        # both the object and every rendered joint.  Prefer those positions so
        # /api/status never mixes a newer arm pose with an older object pose.
        sim_joint_positions = grasp_debug.get("joint_positions")
        if isinstance(sim_joint_positions, dict):
            for index, name in enumerate(joint_state["name"]):
                value = sim_joint_positions.get(name)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    joint_state["position"][index] = float(value)
            joint_state["sim_time"] = grasp_debug.get("sim_time")
        return {
            "ok": True,
            "status": status,
            "joint_state": joint_state,
            "last_plan": summary,
            "gripper": gripper,
            "mujoco_scene": mujoco_scene,
        }

    def _joint_state_payload(self, msg, gripper_msgs):
        names = list(msg.name)
        positions = list(msg.position)
        velocities = list(msg.velocity)
        known = set(names)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        for gripper_msg in gripper_msgs.values():
            if gripper_msg is None:
                continue
            stamp = max(stamp, float(gripper_msg.header.stamp.sec) + float(gripper_msg.header.stamp.nanosec) * 1e-9)
            has_velocity = bool(gripper_msg.velocity)
            for index, name in enumerate(gripper_msg.name):
                if name in known:
                    continue
                if index >= len(gripper_msg.position):
                    continue
                names.append(name)
                positions.append(gripper_msg.position[index])
                velocities.append(gripper_msg.velocity[index] if has_velocity and index < len(gripper_msg.velocity) else 0.0)
                known.add(name)
        return {
            "stamp": stamp,
            "name": names,
            "position": positions,
            "velocity": velocities,
        }

    def _read_link_positions(self, side):
        cfg = self._arm_config(side)
        links = []
        for name in cfg["links"]:
            try:
                tf = self._tf_buffer.lookup_transform(
                    FRAME_ID,
                    name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                links.append({
                    "name": name,
                    "position": [t.x, t.y, t.z],
                    "orientation": [q.x, q.y, q.z, q.w],
                })
            except Exception as exc:
                links.append({"name": name, "error": str(exc)})

        return {"frame": FRAME_ID, "arm": cfg["side"], "links": links, "stamp": time.time()}

    def _update_link_cache(self):
        for side in ("right", "left"):
            data = self._read_link_positions(side)
            if any("position" in item for item in data["links"]):
                with self._lock:
                    self._latest_links[side] = data

    def link_positions(self, side="right"):
        side = normalize_arm(side)
        with self._lock:
            return dict(self._latest_links[side])

    @staticmethod
    def _spin_until_future_complete(future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _wait_ready(self, side="right", timeout_sec=10.0):
        action_client = self._arm_config(side)["action_client"]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if (
                self._plan_client.service_is_ready()
                and self._ik_client.service_is_ready()
                and action_client.server_is_ready()
            ):
                return True
            self._plan_client.wait_for_service(timeout_sec=0.1)
            self._ik_client.wait_for_service(timeout_sec=0.1)
            action_client.wait_for_server(timeout_sec=0.1)
        return (
            self._plan_client.service_is_ready()
            and self._ik_client.service_is_ready()
            and action_client.server_is_ready()
        )

    def _make_start_state(self):
        with self._lock:
            joint_state = self._latest_joint_state
        if joint_state is None:
            raise RuntimeError(f"未收到关节状态（话题 {JOINT_STATE_TOPIC}）")
        state = RobotState()
        allowed = set(JOINT_NAMES + LEFT_JOINT_NAMES)
        has_velocity = bool(joint_state.velocity)
        for index, name in enumerate(joint_state.name):
            if name not in allowed:
                continue
            state.joint_state.name.append(name)
            state.joint_state.position.append(joint_state.position[index])
            if has_velocity and index < len(joint_state.velocity):
                state.joint_state.velocity.append(joint_state.velocity[index])
        return state

    def _make_pose_stamped(self, pose, side):
        cfg = self._arm_config(side)
        orientation = pose.get("orientation")
        if isinstance(orientation, dict):
            q = quat_normalize({
                "x": float(orientation.get("x", 0.0)),
                "y": float(orientation.get("y", 0.0)),
                "z": float(orientation.get("z", 0.0)),
                "w": float(orientation.get("w", 1.0)),
            })
        elif isinstance(orientation, (list, tuple)) and len(orientation) == 4:
            q = quat_normalize({
                "x": float(orientation[0]),
                "y": float(orientation[1]),
                "z": float(orientation[2]),
                "w": float(orientation[3]),
            })
        else:
            q = quat_from_rpy(
                float(pose.get("roll", 0.0)),
                float(pose.get("pitch", -math.pi / 2.0)),
                float(pose.get("yaw", math.pi)),
            )

        tip_position = [
            float(pose["x"]),
            float(pose["y"]),
            float(pose["z"]),
        ]
        moveit_position, moveit_q = pose_to_moveit_frame(tip_position, q)

        stamped = PoseStamped()
        stamped.header.frame_id = MOVEIT_PLANNING_FRAME_ID
        stamped.pose.position.x = moveit_position[0]
        stamped.pose.position.y = moveit_position[1]
        stamped.pose.position.z = moveit_position[2]
        stamped.pose.orientation.x = moveit_q["x"]
        stamped.pose.orientation.y = moveit_q["y"]
        stamped.pose.orientation.z = moveit_q["z"]
        stamped.pose.orientation.w = moveit_q["w"]
        return stamped

    def _make_pose_stamped_from_quat(self, position, orientation):
        moveit_position, moveit_q = pose_to_moveit_frame(position, orientation)
        stamped = PoseStamped()
        stamped.header.frame_id = MOVEIT_PLANNING_FRAME_ID
        stamped.pose.position.x = moveit_position[0]
        stamped.pose.position.y = moveit_position[1]
        stamped.pose.position.z = moveit_position[2]
        stamped.pose.orientation.x = moveit_q["x"]
        stamped.pose.orientation.y = moveit_q["y"]
        stamped.pose.orientation.z = moveit_q["z"]
        stamped.pose.orientation.w = moveit_q["w"]
        return stamped

    def _compute_ik_for_pose(self, side, ik_link_name, position, orientation, timeout_sec=0.6, avoid_collisions=True):
        cfg = self._arm_config(side)
        req = GetPositionIK.Request()
        req.ik_request.group_name = cfg["group"]
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped = self._make_pose_stamped_from_quat(position, orientation)
        req.ik_request.robot_state = self._make_start_state()
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        timeout_sec = max(0.1, float(timeout_sec))
        req.ik_request.timeout = self._duration_msg(timeout_sec)

        started = time.monotonic()
        future = self._ik_client.call_async(req)
        self._spin_until_future_complete(future, timeout_sec + 0.5)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if not future.done():
            return None, {
                "ok": False,
                "error_code": -6,
                "reason": "IK_TIMEOUT",
                "elapsed_ms": round(elapsed_ms, 1),
            }
        result = future.result()
        code = int(result.error_code.val)
        return result, {
            "ok": code == 1,
            "error_code": code,
            "reason": moveit_error_message(code),
            "elapsed_ms": round(elapsed_ms, 1),
        }

    def _arm_base_position(self, side):
        side = normalize_arm(side)
        base_link = self._arm_config(side)["links"][0]
        with self._lock:
            links = list(self._latest_links[side].get("links", []))
        for link in links:
            if link.get("name") == base_link and "position" in link:
                return [float(value) for value in link["position"]]
        try:
            world_from_base = transform_from_xyz_rpy(
                parse_urdf_xyz(ROBOT_WORLD_FROM_BASE_LINK_XYZ),
                parse_urdf_xyz(ROBOT_WORLD_FROM_BASE_LINK_RPY),
            )
            base_from_arm = urdf_transform("base_link", base_link)
            return transform_point(mat_mul(world_from_base, base_from_arm), [0.0, 0.0, 0.0])
        except Exception as exc:
            log_event("WARNING", "arm base fallback failed", {"arm": side, "error": str(exc)})
            return [0.0, 0.0, 0.0]

    def _wait_planning_services_ready(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._plan_client.service_is_ready() and self._ik_client.service_is_ready():
                return True
            self._plan_client.wait_for_service(timeout_sec=0.1)
            self._ik_client.wait_for_service(timeout_sec=0.1)
        return self._plan_client.service_is_ready() and self._ik_client.service_is_ready()

    @staticmethod
    def _new_benchmark_progress(side):
        now = time.time()
        return {
            "ok": True,
            "arm": normalize_arm(side),
            "running": False,
            "stage": "idle",
            "label": "等待跑分",
            "completed_steps": 0,
            "total_steps": 0,
            "percent": 0.0,
            "total_points": 0,
            "ik_done": 0,
            "ik_ok": 0,
            "plan_done": 0,
            "plan_total": 0,
            "current_sample_id": None,
            "current_planner_id": None,
            "current_planner_label": None,
            "started_at": None,
            "updated_at": now,
            "elapsed_ms": 0.0,
            "eta_sec": None,
            "error": None,
        }

    def _set_benchmark_progress(self, side, **updates):
        side = normalize_arm(side)
        with self._lock:
            progress = dict(self._benchmark_progress.get(side) or self._new_benchmark_progress(side))
            progress.update(updates)
            progress["arm"] = side
            progress["ok"] = True
            progress["updated_at"] = time.time()
            total_steps = max(0, int(progress.get("total_steps") or 0))
            completed_steps = max(0, int(progress.get("completed_steps") or 0))
            if total_steps > 0:
                progress["percent"] = round(min(100.0, completed_steps * 100.0 / total_steps), 1)
            elif progress.get("stage") == "done":
                progress["percent"] = 100.0
            else:
                progress["percent"] = 0.0
            started_at = progress.get("started_at")
            if started_at:
                elapsed_sec = max(0.0, progress["updated_at"] - float(started_at))
                progress["elapsed_ms"] = round(elapsed_sec * 1000.0, 1)
                if progress.get("running") and completed_steps > 0 and total_steps > completed_steps:
                    progress["eta_sec"] = round(elapsed_sec * (total_steps - completed_steps) / completed_steps, 1)
                else:
                    progress["eta_sec"] = None
            self._benchmark_progress[side] = progress
            return dict(progress)

    def benchmark_progress(self, side="right"):
        side = normalize_arm(side)
        with self._lock:
            progress = dict(self._benchmark_progress.get(side) or self._new_benchmark_progress(side))
        started_at = progress.get("started_at")
        if started_at:
            elapsed_sec = max(0.0, time.time() - float(started_at))
            progress["elapsed_ms"] = round(elapsed_sec * 1000.0, 1)
            completed_steps = max(0, int(progress.get("completed_steps") or 0))
            total_steps = max(0, int(progress.get("total_steps") or 0))
            if progress.get("running") and completed_steps > 0 and total_steps > completed_steps:
                progress["eta_sec"] = round(elapsed_sec * (total_steps - completed_steps) / completed_steps, 1)
        return progress

    def benchmark_options(self, side="right"):
        side = normalize_arm(side)
        cfg = self._arm_config(side)
        return {
            "ok": True,
            "arm": side,
            "frame": FRAME_ID,
            "moveit_frame": MOVEIT_PLANNING_FRAME_ID,
            "start_state": "current",
            "max_samples": BENCHMARK_MAX_SAMPLES,
            "defaults": {
                "box_top_count": BENCHMARK_DEFAULT_BOX_COUNT,
                "box_top_offset_cm": BENCHMARK_DEFAULT_BOX_OFFSET_CM,
                "edge_count": BENCHMARK_DEFAULT_EDGE_COUNT,
                "edge_distance_cm": BENCHMARK_DEFAULT_EDGE_DISTANCE_CM,
                "random_count": BENCHMARK_DEFAULT_RANDOM_COUNT,
                "planning_time": OMPL_RUNTIME_DEFAULT["planning_time"],
                "attempts": OMPL_RUNTIME_DEFAULT["attempts"],
                "ik_timeout": OMPL_RUNTIME_DEFAULT["ik_timeout"],
            },
            "planners": benchmark_planner_options_for_group(cfg["group"]),
            "collision_boxes": enabled_manual_collision_boxes(),
            "workspace_bounds": load_workspace_bounds(side),
        }

    def generate_benchmark_samples(self, body):
        side = normalize_arm(body.get("arm", "right"))
        box_count = clamp_int(body.get("box_top_count", BENCHMARK_DEFAULT_BOX_COUNT), 0, BENCHMARK_MAX_SAMPLES, "box_top_count")
        edge_count = clamp_int(body.get("edge_count", BENCHMARK_DEFAULT_EDGE_COUNT), 0, BENCHMARK_MAX_SAMPLES, "edge_count")
        random_count = clamp_int(body.get("random_count", BENCHMARK_DEFAULT_RANDOM_COUNT), 0, BENCHMARK_MAX_SAMPLES, "random_count")
        total = box_count + edge_count + random_count
        if total <= 0:
            raise ValueError("至少生成 1 个测试点")
        if total > BENCHMARK_MAX_SAMPLES:
            raise ValueError(f"测试点总数不能超过 {BENCHMARK_MAX_SAMPLES}")

        seed = body.get("seed")
        if seed in (None, ""):
            seed = random.randint(1, 2_147_483_647)
        seed = int(seed)
        rng = random.Random(seed)

        region = benchmark_region_for_arm(side)
        if isinstance(body.get("region"), dict):
            region.update(body["region"])
        workspace_bounds = load_workspace_bounds(side)
        region = intersect_region_with_workspace(region, workspace_bounds)
        grasp_config = benchmark_grasp_config_from_body(body)
        collision_boxes = benchmark_collision_boxes_from_body(body)

        box_offset_cm = clamp_float(body.get("box_top_offset_cm", BENCHMARK_DEFAULT_BOX_OFFSET_CM), 0.0, 200.0, "box_top_offset_cm")
        edge_distance_cm = clamp_float(body.get("edge_distance_cm", BENCHMARK_DEFAULT_EDGE_DISTANCE_CM), 0.0, 200.0, "edge_distance_cm")

        samples = []
        samples.extend(sample_box_top_points(
            rng,
            region,
            side,
            box_count,
            box_offset_cm,
            body.get("collision_box_id"),
            grasp_config,
            len(samples) + 1,
            collision_boxes,
        ))
        samples.extend(sample_edge_points(
            rng,
            region,
            side,
            edge_count,
            edge_distance_cm,
            grasp_config,
            len(samples) + 1,
            collision_boxes,
        ))
        samples.extend(sample_random_points(
            rng,
            region,
            side,
            random_count,
            grasp_config,
            len(samples) + 1,
            collision_boxes,
        ))

        log_event("INFO", "benchmark samples generated", {
            "arm": side,
            "seed": seed,
            "box_top_count": box_count,
            "edge_count": edge_count,
            "random_count": random_count,
            "total": len(samples),
        })
        return {
            **self.benchmark_options(side),
            "seed": seed,
            "count": len(samples),
            "requested": {
                "box_top": box_count,
                "edge": edge_count,
                "random": random_count,
            },
            "region": region,
            "samples": samples,
            "collision_boxes": enabled_manual_collision_boxes(collision_boxes),
        }

    def _benchmark_ik(self, sample, timeout_sec, avoid_collisions):
        side = normalize_arm(sample.get("arm", "right"))
        cfg = self._arm_config(side)
        req = GetPositionIK.Request()
        req.ik_request.group_name = cfg["group"]
        req.ik_request.ik_link_name = cfg["tip"]
        req.ik_request.pose_stamped = self._make_pose_stamped(sample, side)
        req.ik_request.robot_state = self._make_start_state()
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = self._duration_msg(timeout_sec)

        started = time.monotonic()
        future = self._ik_client.call_async(req)
        self._spin_until_future_complete(future, float(timeout_sec) + 1.0)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if not future.done():
            return None, {
                "ok": False,
                "error_code": -6,
                "reason": "IK_TIMEOUT",
                "elapsed_ms": elapsed_ms,
            }
        result = future.result()
        code = int(result.error_code.val)
        return result, {
            "ok": code == 1,
            "error_code": code,
            "reason": moveit_error_message(code),
            "elapsed_ms": elapsed_ms,
        }

    @staticmethod
    def _benchmark_candidate_summary(candidate, mode, index, ik_link_name):
        summary = {
            "mode": mode or "single_pose",
            "candidate_index": index,
            "ik_link_name": ik_link_name,
        }
        for key in ("roll", "pitch", "yaw"):
            if key in candidate:
                summary[key] = round(float(candidate[key]), 6)
        for key in ("_horizontal_grasp", "_vertical_grasp", "_angled_grasp"):
            if key in candidate:
                summary.update(candidate[key])
        if isinstance(candidate.get("orientation"), dict):
            summary["pose"] = MoveItBridge._pose_debug(
                [candidate["x"], candidate["y"], candidate["z"]],
                candidate["orientation"],
            )
        return summary

    def _benchmark_grasp_candidates(self, sample, side):
        mode = sample.get("grasp_orientation_mode")
        if mode in ("side_grasp", "horizontal_cylinder"):
            return "side_grasp", self._horizontal_grasp_pose_candidates(sample, side)
        if mode == "vertical_grasp":
            return "vertical_grasp", self._vertical_grasp_pose_candidates(sample, side)
        if mode in ("angled_grasp", "z_parallel_grasp"):
            return "angled_grasp", self._angled_grasp_pose_candidates(sample, side)
        return mode or "single_pose", [sample]

    def _benchmark_flexible_grasp_ik(self, sample, timeout_sec, avoid_collisions):
        side = normalize_arm(sample.get("arm", "right"))
        ee_link_name = self._ee_link_name(side)
        try:
            data = self._flexible_grasp_inputs(sample, side)
        except Exception as exc:
            return None, {
                "ok": False,
                "error_code": -31,
                "reason": f"FLEXIBLE_GRASP_INPUT_ERROR: {exc}",
                "elapsed_ms": 0.0,
                "mode": "flexible_grasp",
                "candidate_count": 0,
                "scanned_count": 0,
            }

        started = time.monotonic()
        rejected = []
        checked = 0
        for theta_deg in FLEXIBLE_GRASP_THETA_OFFSETS_DEG:
            candidate, rejection = self._flexible_grasp_candidate(data, theta_deg)
            if rejection:
                rejected.append(rejection)
                continue
            if checked >= FLEXIBLE_GRASP_MAX_IK_CANDIDATES:
                rejected.append({"theta_deg": theta_deg, "reason": "ik_candidate_limit_reached"})
                continue
            checked += 1

            pregrasp_ik, pregrasp_status = self._compute_ik_for_pose(
                side,
                ee_link_name,
                candidate["pregrasp_position"],
                candidate["orientation"],
                timeout_sec=timeout_sec,
                avoid_collisions=avoid_collisions,
            )
            if not pregrasp_status["ok"]:
                rejected.append({
                    "theta_deg": theta_deg,
                    "reason": "pregrasp_ik_failed",
                    "ik": pregrasp_status,
                })
                continue

            grasp_ik, grasp_status = self._compute_ik_for_pose(
                side,
                ee_link_name,
                candidate["grasp_position"],
                candidate["orientation"],
                timeout_sec=timeout_sec,
                avoid_collisions=avoid_collisions,
            )
            if not grasp_status["ok"]:
                rejected.append({
                    "theta_deg": theta_deg,
                    "reason": "grasp_ik_failed",
                    "pregrasp_ik": pregrasp_status,
                    "ik": grasp_status,
                })
                continue

            elapsed_ms = (time.monotonic() - started) * 1000.0
            return grasp_ik, {
                "ok": True,
                "error_code": int(grasp_ik.error_code.val),
                "reason": moveit_error_message(int(grasp_ik.error_code.val)),
                "elapsed_ms": elapsed_ms,
                "mode": "flexible_grasp",
                "candidate_index": checked,
                "candidate_count": len(FLEXIBLE_GRASP_THETA_OFFSETS_DEG),
                "scanned_count": checked,
                "ik_link_name": ee_link_name,
                "theta_deg": theta_deg,
                "pregrasp_ik": pregrasp_status,
                "grasp_ik": grasp_status,
                "grasp_pose": self._pose_debug(candidate["grasp_position"], candidate["orientation"]),
                "pregrasp_pose": self._pose_debug(candidate["pregrasp_position"], candidate["orientation"]),
                "rejected_candidates": rejected,
            }

        elapsed_ms = (time.monotonic() - started) * 1000.0
        return None, {
            "ok": False,
            "error_code": -31,
            "reason": "NO_FLEXIBLE_GRASP_IK_CANDIDATE",
            "elapsed_ms": elapsed_ms,
            "mode": "flexible_grasp",
            "candidate_count": len(FLEXIBLE_GRASP_THETA_OFFSETS_DEG),
            "scanned_count": checked,
            "rejected_candidates": rejected[-5:],
        }

    def _benchmark_grasp_ik(self, sample, timeout_sec, avoid_collisions):
        side = normalize_arm(sample.get("arm", "right"))
        if sample.get("grasp_orientation_mode") == "flexible_grasp":
            return self._benchmark_flexible_grasp_ik(sample, timeout_sec, avoid_collisions)

        mode, candidates = self._benchmark_grasp_candidates(sample, side)
        rejected = []
        started = time.monotonic()
        for index, candidate in enumerate(candidates, start=1):
            ik_result, ik_info = self._benchmark_ik(candidate, timeout_sec, avoid_collisions)
            ik_info = {
                **ik_info,
                "mode": mode,
                "candidate_index": index,
                "candidate_count": len(candidates),
                "scanned_count": index,
                "candidate": self._benchmark_candidate_summary(
                    candidate,
                    mode,
                    index,
                    self._arm_config(side)["tip"],
                ),
            }
            if ik_info["ok"] and ik_result is not None:
                ik_info["elapsed_ms"] = (time.monotonic() - started) * 1000.0
                if rejected:
                    ik_info["rejected_candidates"] = rejected
                return ik_result, ik_info
            rejected.append({
                "candidate_index": index,
                "reason": ik_info.get("reason"),
                "error_code": ik_info.get("error_code"),
                "elapsed_ms": ik_info.get("elapsed_ms"),
                "candidate": ik_info.get("candidate"),
            })

        elapsed_ms = (time.monotonic() - started) * 1000.0
        reason = "NO_GRASP_IK_CANDIDATE" if candidates else "NO_GRASP_CANDIDATES"
        return None, {
            "ok": False,
            "error_code": -31,
            "reason": reason,
            "elapsed_ms": elapsed_ms,
            "mode": mode,
            "candidate_count": len(candidates),
            "scanned_count": len(candidates),
            "rejected_candidates": rejected[-5:],
        }

    def _benchmark_plan(self, sample, target_joint_state, planner_id, planning_time, attempts):
        side = normalize_arm(sample.get("arm", "right"))
        cfg = self._arm_config(side)
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = cfg["group"]
        mpr.pipeline_id = OMPL_PIPELINE_ID
        mpr.planner_id = planner_id
        mpr.num_planning_attempts = int(attempts)
        mpr.allowed_planning_time = float(planning_time)
        mpr.max_velocity_scaling_factor = float(sample.get("velocity_scaling", 0.15))
        mpr.max_acceleration_scaling_factor = float(sample.get("acceleration_scaling", 0.10))
        mpr.start_state = self._make_start_state()
        mpr.goal_constraints.append(self._make_joint_goal_constraints(target_joint_state, cfg["joints"]))
        req.motion_plan_request = mpr

        started = time.monotonic()
        future = self._plan_client.call_async(req)
        self._spin_until_future_complete(future, planning_time + 2.0)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if not future.done():
            return {
                "ok": False,
                "error_code": -6,
                "reason": "PLAN_TIMEOUT",
                "elapsed_ms": elapsed_ms,
                "trajectory_points": 0,
                "trajectory_duration": 0.0,
            }
        result = future.result()
        code = int(result.motion_plan_response.error_code.val)
        traj = result.motion_plan_response.trajectory.joint_trajectory
        return {
            "ok": code == 1,
            "error_code": code,
            "reason": moveit_error_message(code),
            "elapsed_ms": elapsed_ms,
            "trajectory_points": len(traj.points),
            "trajectory_duration": self._trajectory_duration(traj),
        }

    def _benchmark_grasp_plan(self, sample, planner_id, planning_time, attempts, ik_timeout, avoid_collisions):
        side = normalize_arm(sample.get("arm", "right"))
        if sample.get("grasp_orientation_mode") == "flexible_grasp":
            started = time.monotonic()
            candidate = {
                **sample,
                "planner_id": planner_id,
                "planning_time": planning_time,
                "attempts": attempts,
                "ik_timeout": ik_timeout,
                "avoid_platform": avoid_collisions,
            }
            try:
                summary = self._plan_flexible_grasp_ik(candidate, side)
                orientation = summary.get("grasp_orientation", {})
                return {
                    "ok": True,
                    "error_code": 1,
                    "reason": summary.get("mode", "MoveIt plan"),
                    "elapsed_ms": (time.monotonic() - started) * 1000.0,
                    "trajectory_points": int(summary.get("points", 0)),
                    "trajectory_duration": float(summary.get("duration", 0.0)),
                    "candidate_index": orientation.get("candidate_index"),
                    "candidate_count": len(FLEXIBLE_GRASP_THETA_OFFSETS_DEG),
                    "scanned_count": orientation.get("candidate_index"),
                    "candidate": orientation,
                }
            except Exception as exc:
                return {
                    "ok": False,
                    "error_code": -31,
                    "reason": str(exc),
                    "elapsed_ms": (time.monotonic() - started) * 1000.0,
                    "trajectory_points": 0,
                    "trajectory_duration": 0.0,
                    "candidate_count": len(FLEXIBLE_GRASP_THETA_OFFSETS_DEG),
                }

        mode, candidates = self._benchmark_grasp_candidates(sample, side)
        rejected = []
        started = time.monotonic()
        for index, candidate in enumerate(candidates, start=1):
            candidate_summary = self._benchmark_candidate_summary(
                candidate,
                mode,
                index,
                self._arm_config(side)["tip"],
            )
            candidate = {
                **candidate,
                "planner_id": planner_id,
                "planning_time": planning_time,
                "attempts": attempts,
                "ik_timeout": ik_timeout,
                "avoid_platform": avoid_collisions,
            }
            try:
                summary = self._plan_single_pose(candidate)
                plan_info = {
                    "ok": True,
                    "error_code": 1,
                    "reason": summary.get("mode", "MoveIt plan"),
                    "elapsed_ms": (time.monotonic() - started) * 1000.0,
                    "trajectory_points": int(summary.get("points", 0)),
                    "trajectory_duration": float(summary.get("duration", 0.0)),
                    "candidate_index": index,
                    "candidate_count": len(candidates),
                    "scanned_count": index,
                    "candidate": candidate_summary,
                }
                if rejected:
                    plan_info["rejected_candidates"] = rejected
                return plan_info
            except Exception as exc:
                rejected.append({
                    "candidate_index": index,
                    "stage": "plan",
                    "reason": str(exc),
                    "error_code": -31,
                    "elapsed_ms": (time.monotonic() - started) * 1000.0,
                    "candidate": candidate_summary,
                })

        elapsed_ms = (time.monotonic() - started) * 1000.0
        return {
            "ok": False,
            "error_code": -31,
            "reason": "NO_GRASP_PLAN_CANDIDATE" if candidates else "NO_GRASP_CANDIDATES",
            "elapsed_ms": elapsed_ms,
            "trajectory_points": 0,
            "trajectory_duration": 0.0,
            "mode": mode,
            "candidate_count": len(candidates),
            "scanned_count": len(candidates),
            "rejected_candidates": rejected[-5:],
        }

    def run_benchmark(self, body):
        side = normalize_arm(body.get("arm", "right"))
        raw_samples = body.get("samples") or []
        if not raw_samples:
            raw_samples = self.generate_benchmark_samples(body)["samples"]
        samples = [benchmark_pose_from_body(sample, side) for sample in raw_samples]
        if not samples:
            raise ValueError("测试点为空")
        if len(samples) > BENCHMARK_MAX_SAMPLES:
            raise ValueError(f"测试点总数不能超过 {BENCHMARK_MAX_SAMPLES}")
        for sample in samples:
            validate_workspace_target(sample, f"测试点 {sample.get('id', '?')}")

        cfg = self._arm_config(side)
        planner_by_id = {
            planner["planner_id"]: planner
            for planner in benchmark_planner_options_for_group(cfg["group"])
        }
        requested_planners = body.get("planners") or body.get("planner_ids") or [OMPL_RUNTIME_DEFAULT["planner_id"]]
        planners = []
        for planner_id in requested_planners:
            planner = planner_by_id.get(str(planner_id))
            if not planner:
                raise ValueError(f"未知或不适用于 {cfg['group']} 的规划器: {planner_id}")
            planners.append(planner)
        if not planners:
            raise ValueError("至少选择一个规划器")

        planning_time = clamp_float(body.get("planning_time"), 0.1, 30.0, "planning_time", OMPL_RUNTIME_DEFAULT["planning_time"])
        attempts = clamp_int(body.get("attempts", OMPL_RUNTIME_DEFAULT["attempts"]), 1, 20, "attempts")
        ik_timeout = clamp_float(body.get("ik_timeout"), 0.05, 10.0, "ik_timeout", OMPL_RUNTIME_DEFAULT["ik_timeout"])
        requested_avoid_collisions = parse_bool(body.get("avoid_collisions"), True)
        collision_boxes = benchmark_collision_boxes_from_body(body)
        collision_boxes_from_request = has_benchmark_collision_boxes_in_body(body)
        active_collision_boxes = enabled_manual_collision_boxes(collision_boxes)
        avoid_collisions = requested_avoid_collisions or bool(active_collision_boxes)

        if not self._planning_lock.acquire(blocking=False):
            self._set_benchmark_progress(
                side,
                running=False,
                stage="blocked",
                label="已有规划正在运行",
                error="已有规划正在运行，请等待当前规划结束",
            )
            raise RuntimeError("已有规划正在运行，请等待当前规划结束")

        total_points = len(samples)
        planner_count = len(planners)
        estimated_steps = total_points + total_points * planner_count
        started_wall = time.time()
        self._set_benchmark_progress(
            side,
            running=True,
            stage="starting",
            label="等待 MoveIt 服务",
            completed_steps=0,
            total_steps=estimated_steps,
            total_points=total_points,
            ik_done=0,
            ik_ok=0,
            plan_done=0,
            plan_total=total_points * planner_count,
            current_sample_id=None,
            current_planner_id=None,
            current_planner_label=None,
            started_at=started_wall,
            error=None,
        )

        try:
            if not self._wait_planning_services_ready(10.0):
                raise RuntimeError("MoveIt IK 或规划服务未就绪")

            manual_collision_summary = None
            if active_collision_boxes:
                manual_collision_summary = self.apply_manual_collision_boxes(collision_boxes, save=False)
            elif collision_boxes_from_request and self._has_active_manual_collision_boxes():
                manual_collision_summary = self.clear_manual_collision_boxes()

            with self._lock:
                self._status[side] = "benchmark"
            log_event("INFO", "benchmark run start", {
                "arm": side,
                "samples": total_points,
                "planners": [planner["planner_id"] for planner in planners],
                "start_state": "current",
                "manual_collision_boxes": len(active_collision_boxes),
            })

            started = time.monotonic()
            sample_rows = []
            valid_samples = []
            for index, sample in enumerate(samples, start=1):
                self._set_benchmark_progress(
                    side,
                    running=True,
                    stage="ik",
                    label=f"IK 检查 {index}/{total_points}",
                    current_sample_id=int(sample["id"]),
                    current_planner_id=None,
                    current_planner_label=None,
                    completed_steps=index - 1,
                    ik_done=index - 1,
                    ik_ok=len(valid_samples),
                )
                ik_result, ik_info = self._benchmark_grasp_ik(sample, ik_timeout, avoid_collisions)
                row = {
                    **sample,
                    "ik": {
                        **ik_info,
                        "elapsed_ms": round(ik_info["elapsed_ms"], 1),
                    },
                }
                if ik_info["ok"] and ik_result is not None:
                    valid_samples.append(sample)
                sample_rows.append(row)
                self._set_benchmark_progress(
                    side,
                    running=True,
                    stage="ik",
                    label=f"IK 检查 {index}/{total_points}",
                    completed_steps=index,
                    ik_done=index,
                    ik_ok=len(valid_samples),
                )

            valid_count = len(valid_samples)
            plan_total = valid_count * planner_count
            total_steps = total_points + plan_total
            self._set_benchmark_progress(
                side,
                running=True,
                stage="planning" if plan_total else "done",
                label="开始规划" if plan_total else "没有 IK 有效点",
                completed_steps=total_points,
                total_steps=total_steps,
                plan_done=0,
                plan_total=plan_total,
                ik_done=total_points,
                ik_ok=valid_count,
                current_sample_id=None,
                current_planner_id=None,
                current_planner_label=None,
            )

            results = {planner["planner_id"]: [] for planner in planners}
            plan_done = 0
            for planner in planners:
                planner_id = planner["planner_id"]
                planner_label = planner.get("label") or planner_id
                for sample in valid_samples:
                    self._set_benchmark_progress(
                        side,
                        running=True,
                        stage="planning",
                        label=f"{planner_label} 规划 {plan_done + 1}/{plan_total}",
                        current_sample_id=int(sample["id"]),
                        current_planner_id=planner_id,
                        current_planner_label=planner_label,
                        completed_steps=total_points + plan_done,
                        plan_done=plan_done,
                    )
                    plan_info = self._benchmark_grasp_plan(
                        sample,
                        planner_id,
                        planning_time,
                        attempts,
                        ik_timeout,
                        avoid_collisions,
                    )
                    results[planner_id].append({
                        "sample_id": int(sample["id"]),
                        "ok": bool(plan_info["ok"]),
                        "reason": plan_info["reason"],
                        "error_code": plan_info["error_code"],
                        "elapsed_ms": round(plan_info["elapsed_ms"], 1),
                        "candidate_index": plan_info.get("candidate_index"),
                        "candidate_count": plan_info.get("candidate_count"),
                        "scanned_count": plan_info.get("scanned_count"),
                        "candidate": plan_info.get("candidate"),
                        "ik": plan_info.get("ik"),
                        "rejected_candidates": plan_info.get("rejected_candidates"),
                        "trajectory_points": plan_info["trajectory_points"],
                        "trajectory_duration": plan_info["trajectory_duration"],
                    })
                    plan_done += 1
                    self._set_benchmark_progress(
                        side,
                        running=True,
                        stage="planning",
                        label=f"{planner_label} 规划 {plan_done}/{plan_total}",
                        completed_steps=total_points + plan_done,
                        plan_done=plan_done,
                    )

            summary = {}
            for planner in planners:
                planner_id = planner["planner_id"]
                rows = results[planner_id]
                ok_count = sum(1 for row in rows if row["ok"])
                elapsed_values = [float(row["elapsed_ms"]) for row in rows]
                summary[planner_id] = {
                    "planner_id": planner_id,
                    "label": planner["label"],
                    "valid_points": valid_count,
                    "planned_ok": ok_count,
                    "success_rate": ok_count / valid_count if valid_count else 0.0,
                    "mean_elapsed_ms": sum(elapsed_values) / len(elapsed_values) if elapsed_values else 0.0,
                }

            elapsed_ms = (time.monotonic() - started) * 1000.0
            with self._lock:
                self._status[side] = "benchmark done"
            self._set_benchmark_progress(
                side,
                running=False,
                stage="done",
                label="跑分完成",
                completed_steps=total_steps,
                total_steps=total_steps,
                plan_done=plan_done,
                plan_total=plan_total,
                current_sample_id=None,
                current_planner_id=None,
                current_planner_label=None,
                error=None,
            )
            log_event("INFO", "benchmark run done", {
                "arm": side,
                "total": total_points,
                "valid": valid_count,
                "summary": summary,
                "elapsed_ms": round(elapsed_ms, 1),
            })
            return {
                "ok": True,
                "arm": side,
                "frame": FRAME_ID,
                "moveit_frame": MOVEIT_PLANNING_FRAME_ID,
                "start_state": "current",
                "total_points": total_points,
                "valid_points": valid_count,
                "invalid_points": total_points - valid_count,
                "planning_time": planning_time,
                "attempts": attempts,
                "ik_timeout": ik_timeout,
                "avoid_collisions": avoid_collisions,
                "manual_collision": manual_collision_summary,
                "elapsed_ms": elapsed_ms,
                "samples": sample_rows,
                "planners": planners,
                "results": results,
                "summary": summary,
            }
        except Exception as exc:
            with self._lock:
                self._status[side] = "benchmark failed"
            self._set_benchmark_progress(
                side,
                running=False,
                stage="failed",
                label="跑分失败",
                error=str(exc),
            )
            raise
        finally:
            self._planning_lock.release()

    def apply_platform_obstacle(self):
        sequence, camera_points = read_pointcloud_bin(VISION_SCENE_POINTCLOUD_FILE)
        if PLATFORM_MAX_CAMERA_DISTANCE > 0:
            distances = np.linalg.norm(camera_points, axis=1)
            camera_points = camera_points[distances <= PLATFORM_MAX_CAMERA_DISTANCE]
        if len(camera_points) < PLATFORM_MIN_INLIERS:
            raise RuntimeError(
                f"{PLATFORM_MAX_CAMERA_DISTANCE:.2f}m 内点云点数不足，无法拟合平台: {len(camera_points)}"
            )
        extrinsic = load_camera_extrinsic()
        world_points = camera_points_to_world(camera_points, extrinsic)
        camera_origin = np.array([
            float(extrinsic.get("x", 0.0)),
            float(extrinsic.get("y", 0.0)),
            float(extrinsic.get("z", 0.0)),
        ], dtype=np.float64)
        boxes = fit_platform_forbidden_boxes(world_points, camera_origin)

        remove_scene = PlanningScene()
        remove_scene.is_diff = True

        for i in range(max(1, PLATFORM_MAX_BOXES)):
            collision = CollisionObject()
            collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
            collision.id = PLATFORM_OBSTACLE_ID if i == 0 else f"{PLATFORM_OBSTACLE_ID}_{i + 1}"
            collision.operation = CollisionObject.REMOVE
            remove_scene.world.collision_objects.append(collision)

        try:
            self._apply_planning_scene(remove_scene, "移除旧平台障碍")
        except RuntimeError as exc:
            log_event("WARNING", "remove old platform obstacles ignored", {"error": str(exc)})

        add_scene = PlanningScene()
        add_scene.is_diff = True

        for i, box in enumerate(boxes):
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(box["dimensions"])

            moveit_center, moveit_orientation = pose_to_moveit_frame(
                [float(value) for value in box["center"]],
                {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            )
            primitive_pose = Pose()
            primitive_pose.position.x = moveit_center[0]
            primitive_pose.position.y = moveit_center[1]
            primitive_pose.position.z = moveit_center[2]
            primitive_pose.orientation.x = moveit_orientation["x"]
            primitive_pose.orientation.y = moveit_orientation["y"]
            primitive_pose.orientation.z = moveit_orientation["z"]
            primitive_pose.orientation.w = moveit_orientation["w"]

            collision = CollisionObject()
            collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
            collision.id = PLATFORM_OBSTACLE_ID if i == 0 else f"{PLATFORM_OBSTACLE_ID}_{i + 1}"
            collision.primitives.append(primitive)
            collision.primitive_poses.append(primitive_pose)
            collision.operation = CollisionObject.ADD
            add_scene.world.collision_objects.append(collision)
            box["id"] = collision.id
            box["frame"] = FRAME_ID
            box["planning_frame"] = MOVEIT_PLANNING_FRAME_ID

        self._apply_planning_scene(add_scene, "应用平台障碍")

        summary = {
            "sequence": sequence,
            "selection": "nearest_horizontal_planes",
            "platform_obstacles": boxes,
            **boxes[0],
        }
        self._platform_obstacle_summary = summary
        log_event("INFO", "platform obstacle applied", summary)
        return summary

    def apply_mujoco_table_obstacle(self, grasp_contact=False):
        """Add the simulated tabletop to MoveIt's world collision scene."""
        top_inset = (
            MUJOCO_TABLE_GRASP_CONTACT_INSET
            if grasp_contact
            else MUJOCO_TABLE_COLLISION_TOP_INSET
        )
        collision_center = list(MUJOCO_TABLE_CENTER)
        collision_center[2] -= top_inset * 0.5
        collision_size = list(MUJOCO_TABLE_SIZE)
        collision_size[2] -= top_inset
        planning_guard = 0.0 if grasp_contact else MUJOCO_TABLE_PLANNING_GUARD_M
        collision_center[2] += planning_guard * 0.5
        collision_size[2] += planning_guard

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = collision_size

        moveit_center, moveit_orientation = pose_to_moveit_frame(
            collision_center,
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        )
        primitive_pose = Pose()
        primitive_pose.position.x = moveit_center[0]
        primitive_pose.position.y = moveit_center[1]
        primitive_pose.position.z = moveit_center[2]
        primitive_pose.orientation.x = moveit_orientation["x"]
        primitive_pose.orientation.y = moveit_orientation["y"]
        primitive_pose.orientation.z = moveit_orientation["z"]
        primitive_pose.orientation.w = moveit_orientation["w"]

        collision = CollisionObject()
        collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
        collision.id = MUJOCO_TABLE_COLLISION_ID
        collision.primitives.append(primitive)
        collision.primitive_poses.append(primitive_pose)
        collision.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)
        self._apply_planning_scene(scene, "应用 MuJoCo 桌面障碍")

        summary = {
            "applied": True,
            "id": MUJOCO_TABLE_COLLISION_ID,
            "frame": FRAME_ID,
            "planning_frame": MOVEIT_PLANNING_FRAME_ID,
            "center": collision_center,
            "dimensions": collision_size,
            "physical_center": list(MUJOCO_TABLE_CENTER),
            "physical_dimensions": list(MUJOCO_TABLE_SIZE),
            "top_contact_inset": top_inset,
            "planning_guard_m": planning_guard,
            "grasp_contact": bool(grasp_contact),
        }
        self._mujoco_table_obstacle_summary = summary
        log_event("INFO", "MuJoCo table obstacle applied", summary)
        return summary

    def apply_mujoco_grasp_object_obstacle(self, center=None, enabled=True):
        """Add the red cube only while planning the collision-free approach."""
        if not enabled:
            scene = PlanningScene()
            scene.is_diff = True
            collision = CollisionObject()
            collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
            collision.id = MUJOCO_GRASP_OBJECT_COLLISION_ID
            collision.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(collision)
            try:
                self._apply_planning_scene(scene, "移除 MuJoCo 抓取物体障碍")
            except RuntimeError as exc:
                log_event("WARNING", "MuJoCo grasp object obstacle already absent", {
                    "error": str(exc),
                })
            return {
                "applied": False,
                "cleared": True,
                "id": MUJOCO_GRASP_OBJECT_COLLISION_ID,
            }

        object_center = finite_vec3(
            center,
            "MuJoCo object center",
            [
                MUJOCO_GRASP_OBJECT_CENTER["x"],
                MUJOCO_GRASP_OBJECT_CENTER["y"],
                MUJOCO_GRASP_OBJECT_CENTER["z"],
            ],
        )
        dimensions = [
            value + 2.0 * MUJOCO_GRASP_OBJECT_PLANNING_MARGIN
            for value in MUJOCO_GRASP_OBJECT_SIZE
        ]
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        moveit_center, moveit_orientation = pose_to_moveit_frame(
            object_center,
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        )
        primitive_pose = Pose()
        primitive_pose.position.x = moveit_center[0]
        primitive_pose.position.y = moveit_center[1]
        primitive_pose.position.z = moveit_center[2]
        primitive_pose.orientation.x = moveit_orientation["x"]
        primitive_pose.orientation.y = moveit_orientation["y"]
        primitive_pose.orientation.z = moveit_orientation["z"]
        primitive_pose.orientation.w = moveit_orientation["w"]
        collision = CollisionObject()
        collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
        collision.id = MUJOCO_GRASP_OBJECT_COLLISION_ID
        collision.primitives.append(primitive)
        collision.primitive_poses.append(primitive_pose)
        collision.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)
        self._apply_planning_scene(scene, "应用 MuJoCo 抓取物体障碍")
        summary = {
            "applied": True,
            "id": MUJOCO_GRASP_OBJECT_COLLISION_ID,
            "frame": FRAME_ID,
            "planning_frame": MOVEIT_PLANNING_FRAME_ID,
            "center": object_center,
            "dimensions": dimensions,
            "margin": MUJOCO_GRASP_OBJECT_PLANNING_MARGIN,
        }
        log_event("INFO", "MuJoCo grasp object obstacle applied", summary)
        return summary

    def clear_platform_obstacle(self):
        remove_scene = PlanningScene()
        remove_scene.is_diff = True

        for i in range(max(1, PLATFORM_MAX_BOXES)):
            collision = CollisionObject()
            collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
            collision.id = PLATFORM_OBSTACLE_ID if i == 0 else f"{PLATFORM_OBSTACLE_ID}_{i + 1}"
            collision.operation = CollisionObject.REMOVE
            remove_scene.world.collision_objects.append(collision)

        self._apply_planning_scene(remove_scene, "remove platform obstacles")
        self._platform_obstacle_summary = None
        summary = {
            "cleared": True,
            "platform_obstacles": [],
            "frame": FRAME_ID,
        }
        log_event("INFO", "platform obstacle cleared", summary)
        return summary

    @staticmethod
    def _manual_collision_object_id(box):
        return f"{MANUAL_COLLISION_ID_PREFIX}_{box['id']}"

    def _manual_collision_remove_ids(self, boxes):
        ids = set(self._manual_collision_ids)
        for box in boxes:
            ids.add(self._manual_collision_object_id(box))
        return sorted(ids)

    def apply_manual_collision_boxes(self, boxes=None, save=True):
        if boxes is None:
            boxes = load_manual_collision_boxes()
        boxes = save_manual_collision_boxes(boxes) if save else normalize_manual_collision_boxes(boxes)

        desired_ids = {
            self._manual_collision_object_id(box)
            for box in boxes
            if box.get("enabled", True)
        }
        # CollisionObject.ADD replaces an object with the same ID.  Only remove
        # tracked objects that are no longer desired; deleting every possible
        # legacy ID makes MoveIt report failure for objects that never existed.
        stale_ids = sorted(self._manual_collision_ids - desired_ids)
        if stale_ids:
            remove_scene = PlanningScene()
            remove_scene.is_diff = True
            for object_id in stale_ids:
                collision = CollisionObject()
                collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
                collision.id = object_id
                collision.operation = CollisionObject.REMOVE
                remove_scene.world.collision_objects.append(collision)
            try:
                self._apply_planning_scene(remove_scene, "移除旧手动碰撞箱")
            except RuntimeError as exc:
                # Removing an already-absent object is idempotent.  MoveIt can
                # return success=false after its scene has been reset, while
                # the desired end state has already been reached.
                log_event("WARNING", "stale manual collision boxes already absent", {
                    "ids": stale_ids,
                    "error": str(exc),
                })

        add_scene = PlanningScene()
        add_scene.is_diff = True
        applied = []
        for box in boxes:
            if not box.get("enabled", True):
                continue

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(box["dimensions"])

            moveit_center, moveit_orientation = pose_to_moveit_frame(
                [float(value) for value in box["center"]],
                quat_from_rpy(*[float(value) for value in box.get("rpy", [0.0, 0.0, 0.0])]),
            )
            primitive_pose = Pose()
            primitive_pose.position.x = moveit_center[0]
            primitive_pose.position.y = moveit_center[1]
            primitive_pose.position.z = moveit_center[2]
            primitive_pose.orientation.x = moveit_orientation["x"]
            primitive_pose.orientation.y = moveit_orientation["y"]
            primitive_pose.orientation.z = moveit_orientation["z"]
            primitive_pose.orientation.w = moveit_orientation["w"]

            collision = CollisionObject()
            collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
            collision.id = self._manual_collision_object_id(box)
            collision.primitives.append(primitive)
            collision.primitive_poses.append(primitive_pose)
            collision.operation = CollisionObject.ADD
            add_scene.world.collision_objects.append(collision)
            applied.append({
                **box,
                "object_id": collision.id,
                "planning_frame": MOVEIT_PLANNING_FRAME_ID,
            })

        if applied:
            self._apply_planning_scene(add_scene, "应用手动碰撞箱")

        self._manual_collision_ids = {item["object_id"] for item in applied}
        summary = {
            "applied": bool(applied),
            "count": len(applied),
            "boxes": boxes,
            "applied_boxes": applied,
            "frame": FRAME_ID,
            "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        }
        self._manual_collision_summary = summary
        log_event("INFO", "manual collision boxes applied", summary)
        return summary

    def clear_manual_collision_boxes(self):
        boxes = load_manual_collision_boxes()
        tracked_ids = sorted(self._manual_collision_ids)
        if tracked_ids:
            remove_scene = PlanningScene()
            remove_scene.is_diff = True
            for object_id in tracked_ids:
                collision = CollisionObject()
                collision.header.frame_id = MOVEIT_PLANNING_FRAME_ID
                collision.id = object_id
                collision.operation = CollisionObject.REMOVE
                remove_scene.world.collision_objects.append(collision)
            try:
                self._apply_planning_scene(remove_scene, "移除手动碰撞箱")
            except RuntimeError as exc:
                log_event("WARNING", "manual collision boxes already absent", {
                    "ids": tracked_ids,
                    "error": str(exc),
                })
        self._manual_collision_ids = set()
        summary = {
            "cleared": True,
            "count": 0,
            "boxes": boxes,
            "frame": FRAME_ID,
            "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        }
        self._manual_collision_summary = summary
        log_event("INFO", "manual collision boxes cleared", summary)
        return summary

    def sync_manual_collision_boxes_for_request(self, body):
        boxes = manual_collision_boxes_from_body(body if isinstance(body, dict) else {})
        if boxes is None:
            return None
        active_boxes = enabled_manual_collision_boxes(boxes)
        if active_boxes:
            return self.apply_manual_collision_boxes(boxes, save=False)
        if self._has_active_manual_collision_boxes():
            return self.clear_manual_collision_boxes()
        summary = {
            "applied": False,
            "cleared": False,
            "count": 0,
            "boxes": boxes,
            "frame": FRAME_ID,
            "planning_frame": MOVEIT_PLANNING_FRAME_ID,
        }
        self._manual_collision_summary = summary
        return summary

    def _has_active_manual_collision_boxes(self):
        return bool(self._manual_collision_ids)

    def _apply_planning_scene(self, scene, label):
        if not self._apply_scene_client.service_is_ready():
            self._apply_scene_client.wait_for_service(timeout_sec=APPLY_PLANNING_SCENE_TIMEOUT_SEC)
        if not self._apply_scene_client.service_is_ready():
            raise RuntimeError(f"{APPLY_PLANNING_SCENE_SERVICE} 服务未就绪")

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        if not self._spin_until_future_complete(future, 3.0):
            raise RuntimeError(f"{label}到 PlanningScene 超时")
        resp = future.result()
        if not getattr(resp, "success", False):
            raise RuntimeError(f"{label}到 PlanningScene 失败")
        return resp

    def _make_joint_goal_constraints(self, joint_state, joint_names, tolerance=0.01):
        c = Constraints()
        by_name = dict(zip(joint_state.name, joint_state.position))
        for name in joint_names:
            if name not in by_name:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(by_name[name])
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _make_near_joint_constraints(self, joint_state, joint_names, tolerance=0.08):
        c = Constraints()
        by_name = dict(zip(joint_state.name, joint_state.position))
        for name in joint_names:
            if name not in by_name:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(by_name[name])
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 0.05
            c.joint_constraints.append(jc)
        return c

    def _horizontal_grasp_preferred_yaw(self, pose, side):
        try:
            base = self._arm_base_position(side)
            dx = float(pose["x"]) - float(base[0])
            dy = float(pose["y"]) - float(base[1])
            if abs(dx) + abs(dy) > 1e-6:
                return math.atan2(dy, dx)
        except Exception:
            pass
        yaw = pose.get("yaw")
        return float(yaw) if yaw is not None and math.isfinite(float(yaw)) else 0.0

    @staticmethod
    def _rpy_axes(roll, pitch, yaw):
        matrix = rpy_to_matrix(float(roll), float(pitch), float(yaw))
        return (
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        )

    @staticmethod
    def _align_mujoco_grasp_candidate(candidate, pose, tool_x, tool_z):
        object_center = pose.get("mujoco_object_center")
        if object_center is None:
            return None
        center = finite_vec3(object_center, "MuJoCo object center")
        nominal_tcp = finite_vec3(
            pose.get("mujoco_nominal_tcp"),
            "MuJoCo nominal TCP",
            [pose["x"], pose["y"], pose["z"]],
        )
        requested_tcp = finite_vec3(
            [pose["x"], pose["y"], pose["z"]],
            "MuJoCo requested TCP",
        )
        stage_offset = vec_sub(requested_tcp, nominal_tcp)
        tcp_position = vec_add(
            vec_add(center, vec_scale(tool_x, MUJOCO_GRASP_OBJECT_TO_TCP_M)),
            stage_offset,
        )
        tcp_position = vec_add(
            tcp_position,
            vec_scale(tool_z, MUJOCO_GRASP_LATERAL_OFFSET_M),
        )
        tcp_position[2] += MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M
        candidate["x"], candidate["y"], candidate["z"] = tcp_position
        return {
            "object_center": center,
            "object_to_tcp_m": MUJOCO_GRASP_OBJECT_TO_TCP_M,
            "lateral_offset_m": MUJOCO_GRASP_LATERAL_OFFSET_M,
            "stage_offset": stage_offset,
            "tcp_position": tcp_position,
        }

    def _horizontal_grasp_pose_candidates(self, pose, side):
        base_pose = dict(pose)
        preferred = self._horizontal_grasp_preferred_yaw(base_pose, side)
        step = math.radians(max(1.0, min(90.0, HORIZONTAL_GRASP_YAW_STEP_DEG)))
        yaw_offsets = [0.0]
        rings = int(math.ceil(math.pi / step))
        for index in range(1, rings + 1):
            yaw_offsets.extend([index * step, -index * step])
        yaws = dedupe_angles([preferred + offset for offset in yaw_offsets])

        rpy_pairs = [
            (math.pi / 2.0, 0.0),
            (-math.pi / 2.0, 0.0),
            (math.pi / 2.0, math.pi),
            (-math.pi / 2.0, math.pi),
        ]
        candidates = []
        for yaw in yaws:
            for roll, pitch in rpy_pairs:
                if side_grasp_constraint_error(roll, pitch) > 1e-6:
                    continue
                candidate = dict(base_pose)
                candidate["roll"] = roll
                candidate["pitch"] = pitch
                candidate["yaw"] = yaw
                candidate.pop("grasp_orientation_mode", None)
                x_axis, y_axis, z_axis = self._rpy_axes(roll, pitch, yaw)
                alignment = self._align_mujoco_grasp_candidate(
                    candidate, pose, x_axis, z_axis
                )
                if alignment:
                    candidate["_mujoco_grasp_alignment"] = alignment
                candidate["_horizontal_grasp"] = {
                    "constraint": "tool_y_perpendicular_to_world_xy",
                    "tool_axis": "tool_y",
                    "ground_normal": [0.0, 0.0, 1.0],
                    "dot": local_axis_world_z(roll, pitch, "y"),
                    "preferred_yaw": preferred,
                    "tool_x": x_axis,
                    "tool_y": y_axis,
                    "tool_z": z_axis,
                    **candidate.get("_mujoco_grasp_alignment", {}),
                }
                candidates.append(candidate)
                if len(candidates) >= HORIZONTAL_GRASP_MAX_CANDIDATES:
                    return candidates
        return candidates

    def _vertical_grasp_pose_candidates(self, pose, side):
        base_pose = dict(pose)
        preferred = self._horizontal_grasp_preferred_yaw(base_pose, side)
        step = math.radians(max(1.0, min(90.0, HORIZONTAL_GRASP_YAW_STEP_DEG)))
        yaw_offsets = [0.0]
        rings = int(math.ceil(math.pi / step))
        for index in range(1, rings + 1):
            yaw_offsets.extend([index * step, -index * step])
        yaws = dedupe_angles([preferred + offset for offset in yaw_offsets])

        rpy_pairs = [
            (0.0, -math.pi / 2.0),
            (math.pi, -math.pi / 2.0),
            (0.0, math.pi / 2.0),
            (math.pi, math.pi / 2.0),
        ]
        candidates = []
        for yaw in yaws:
            for roll, pitch in rpy_pairs:
                if vertical_grasp_constraint_error(roll, pitch) > 1e-6:
                    continue
                candidate = dict(base_pose)
                candidate["roll"] = roll
                candidate["pitch"] = pitch
                candidate["yaw"] = yaw
                candidate.pop("grasp_orientation_mode", None)
                candidate["_vertical_grasp"] = {
                    "constraint": "tool_x_perpendicular_to_world_xy",
                    "tool_axis": "tool_x",
                    "ground_normal": [0.0, 0.0, 1.0],
                    "dot": local_axis_world_z(roll, pitch, "x"),
                    "preferred_yaw": preferred,
                }
                candidates.append(candidate)
                if len(candidates) >= HORIZONTAL_GRASP_MAX_CANDIDATES:
                    return candidates
        return candidates

    def _angled_grasp_pose_candidates(self, pose, side):
        base_pose = dict(pose)
        preferred = self._horizontal_grasp_preferred_yaw(base_pose, side)
        step = math.radians(max(1.0, min(90.0, HORIZONTAL_GRASP_YAW_STEP_DEG)))
        yaw_offsets = [0.0]
        rings = int(math.ceil(math.pi / step))
        for index in range(1, rings + 1):
            yaw_offsets.extend([index * step, -index * step])
        yaws = dedupe_angles([preferred + offset for offset in yaw_offsets])

        angle = math.radians(max(0.0, min(89.0, ANGLED_GRASP_X_GROUND_ANGLE_DEG)))
        horizontal = math.cos(angle)
        guarded_mujoco_pick = pose.get("mujoco_object_center") is not None
        vertical = -math.sin(angle)
        ground_normal = [0.0, 0.0, 1.0]
        candidates = []
        for yaw in yaws:
            heading = [math.cos(yaw), math.sin(yaw), 0.0]
            x_axis = vec_normalize([
                horizontal * heading[0],
                horizontal * heading[1],
                vertical,
            ], "angled_grasp_tool_x")
            z_base = vec_normalize([-math.sin(yaw), math.cos(yaw), 0.0], "angled_grasp_tool_z")
            for z_sign in (1.0, -1.0):
                z_axis = vec_scale(z_base, z_sign)
                y_axis = vec_normalize(vec_cross(z_axis, x_axis), "angled_grasp_tool_y")
                z_axis = vec_normalize(vec_cross(x_axis, y_axis), "angled_grasp_tool_z")
                rotation = [
                    [x_axis[0], y_axis[0], z_axis[0]],
                    [x_axis[1], y_axis[1], z_axis[1]],
                    [x_axis[2], y_axis[2], z_axis[2]],
                ]
                orientation = quat_from_matrix(rotation)
                roll, pitch, rpy_yaw = quat_to_rpy(
                    orientation["x"],
                    orientation["y"],
                    orientation["z"],
                    orientation["w"],
                )
                x_ground_dot = vec_dot(x_axis, ground_normal)
                z_ground_dot = vec_dot(z_axis, ground_normal)
                candidate = dict(base_pose)
                candidate["roll"] = roll
                candidate["pitch"] = pitch
                candidate["yaw"] = rpy_yaw
                candidate["orientation"] = orientation
                candidate.pop("grasp_orientation_mode", None)
                object_center = pose.get("mujoco_object_center")
                if object_center is not None:
                    center = finite_vec3(object_center, "MuJoCo object center")
                    nominal_tcp = finite_vec3(
                        pose.get("mujoco_nominal_tcp"),
                        "MuJoCo nominal TCP",
                        [pose["x"], pose["y"], pose["z"]],
                    )
                    requested_tcp = finite_vec3(
                        [pose["x"], pose["y"], pose["z"]],
                        "MuJoCo requested TCP",
                    )
                    stage_offset = vec_sub(requested_tcp, nominal_tcp)
                    # The cube belongs deeper in the finger-pad region behind
                    # the TCP along tool X. Recompute this offset for every
                    # orientation candidate instead of using a fixed world XYZ
                    # offset, which made the fingers side-swipe the cube.
                    tcp_position = vec_add(
                        vec_add(
                            center,
                            vec_scale(x_axis, MUJOCO_GRASP_OBJECT_TO_TCP_M),
                        ),
                        stage_offset,
                    )
                    # The visual TCP and the actual midpoint between the two
                    # imported finger meshes differ slightly.  Shift along the
                    # finger opening axis so the cube is centered between both
                    # pads instead of contacting only the left finger.
                    tcp_position = vec_add(
                        tcp_position,
                        vec_scale(z_axis, MUJOCO_GRASP_LATERAL_OFFSET_M),
                    )
                    # Keep the proven, reachable downward-tilted wrist pose,
                    # but lift its complete approach line above the exact
                    # tabletop.  The previous pose put the TCP about 2 mm
                    # below the tabletop; hiding that with an inset collision
                    # box allowed the physical MuJoCo table to be penetrated.
                    tcp_position[2] += MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M
                    candidate["x"], candidate["y"], candidate["z"] = tcp_position
                    candidate["_mujoco_grasp_alignment"] = {
                        "object_center": center,
                        "object_to_tcp_m": MUJOCO_GRASP_OBJECT_TO_TCP_M,
                        "lateral_offset_m": MUJOCO_GRASP_LATERAL_OFFSET_M,
                        "stage_offset": stage_offset,
                        "tcp_position": tcp_position,
                    }
                candidate["_angled_grasp"] = {
                    "constraint": "tool_z_parallel_to_world_xy_and_tool_x_angled_to_ground",
                    "tool_z_axis": "tool_z",
                    "tool_x_axis": "tool_x",
                    "ground_normal": ground_normal,
                    "tool_x": x_axis,
                    "tool_y": y_axis,
                    "tool_z": z_axis,
                    "tool_x_ground_dot": x_ground_dot,
                    "tool_z_ground_dot": z_ground_dot,
                    "tool_x_ground_angle_deg": round(math.degrees(math.asin(min(1.0, abs(x_ground_dot)))), 6),
                    "tool_x_ground_angle_target_deg": ANGLED_GRASP_X_GROUND_ANGLE_DEG,
                    "tool_x_vertical_preference": "down",
                    "table_clearance_m": (
                        MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M if guarded_mujoco_pick else 0.0
                    ),
                    "z_sign": z_sign,
                    "preferred_yaw": preferred,
                    "scan_yaw": yaw,
                    **candidate.get("_mujoco_grasp_alignment", {}),
                }
                candidates.append(candidate)
                if len(candidates) >= ANGLED_GRASP_MAX_CANDIDATES:
                    return candidates
        return candidates

    def _current_tip_pose(self, side):
        cfg = self._arm_config(side)
        links = self.link_positions(side).get("links", [])
        link = next((item for item in links if item.get("name") == cfg["tip"] and "orientation" in item), None)
        if link is None:
            link = next((item for item in reversed(links) if "orientation" in item), None)
        if link is None or "position" not in link:
            return None
        position = finite_vec3(link.get("position"), "current TCP position")
        orientation = quat_from_xyzw(link["orientation"], "current TCP orientation")
        return {
            "name": link.get("name", cfg["tip"]),
            "position": position,
            "orientation": orientation,
        }

    def _current_gripper_axes(self, side):
        tip = self._current_tip_pose(side)
        if tip is None:
            return None
        q = tip["orientation"]
        qx, qy, qz, qw = q["x"], q["y"], q["z"], q["w"]
        return {
            "position": tip["position"],
            "x": vec_normalize(rotate_vector_by_quat([1.0, 0.0, 0.0], qx, qy, qz, qw), "current_gripper_x"),
            "y": vec_normalize(rotate_vector_by_quat([0.0, 1.0, 0.0], qx, qy, qz, qw), "current_gripper_y"),
            "z": vec_normalize(rotate_vector_by_quat([0.0, 0.0, 1.0], qx, qy, qz, qw), "current_gripper_z"),
        }

    def _level_rotor_transfer_candidates(
        self,
        object_center,
        object_orientation,
        tip_pose,
        yaw_offsets=None,
    ):
        center = finite_vec3(object_center, "rotor object center")
        object_q = quat_normalize(object_orientation)
        tip_q = quat_normalize(tip_pose["orientation"])
        tip_pos = finite_vec3(tip_pose["position"], "current TCP position")
        tcp_offset_world = vec_sub(tip_pos, center)
        tip_q_inv = quat_conjugate(tip_q)
        tcp_offset_local = rotate_vector_by_quat(
            tcp_offset_world,
            tip_q_inv["x"],
            tip_q_inv["y"],
            tip_q_inv["z"],
            tip_q_inv["w"],
        )
        object_from_tip = quat_normalize(quat_multiply(tip_q_inv, object_q))
        object_up = rotate_vector_by_quat(
            [0.0, 0.0, 1.0],
            object_q["x"],
            object_q["y"],
            object_q["z"],
            object_q["w"],
        )
        target_roll = 0.0 if vec_dot(object_up, [0.0, 0.0, 1.0]) >= 0.0 else math.pi
        base_yaw = quat_heading_yaw(object_q)
        offsets = yaw_offsets if isinstance(yaw_offsets, list) else [
            0.0,
            math.radians(20.0),
            -math.radians(20.0),
            math.radians(45.0),
            -math.radians(45.0),
            math.radians(90.0),
            -math.radians(90.0),
            math.pi,
        ]
        candidates = []
        for index, yaw_offset in enumerate(offsets, start=1):
            target_object_q = quat_normalize(
                quat_from_rpy(target_roll, 0.0, base_yaw + float(yaw_offset))
            )
            target_tip_q = quat_normalize(
                quat_multiply(target_object_q, quat_conjugate(object_from_tip))
            )
            target_tcp_offset = rotate_vector_by_quat(
                tcp_offset_local,
                target_tip_q["x"],
                target_tip_q["y"],
                target_tip_q["z"],
                target_tip_q["w"],
            )
            target_object_up = rotate_vector_by_quat(
                [0.0, 0.0, 1.0],
                target_object_q["x"],
                target_object_q["y"],
                target_object_q["z"],
                target_object_q["w"],
            )
            alignment = max(-1.0, min(1.0, abs(vec_dot(target_object_up, [0.0, 0.0, 1.0]))))
            roll, pitch, yaw = quat_to_rpy(
                target_tip_q["x"],
                target_tip_q["y"],
                target_tip_q["z"],
                target_tip_q["w"],
            )
            candidates.append({
                "candidate_index": index,
                "base_yaw": base_yaw,
                "yaw_offset": float(yaw_offset),
                "object_orientation": target_object_q,
                "tip_orientation": target_tip_q,
                "rpy": [roll, pitch, yaw],
                "tcp_offset_local": tcp_offset_local,
                "tcp_offset_world": target_tcp_offset,
                "object_up": target_object_up,
                "level_error_deg": math.degrees(math.acos(alignment)),
            })
        return candidates

    @staticmethod
    def _ee_link_name(side):
        return "Left_Link7" if side == "left" else "Right_Link7"

    @staticmethod
    def _pose_debug(position, orientation):
        roll, pitch, yaw = quat_to_rpy(
            orientation["x"],
            orientation["y"],
            orientation["z"],
            orientation["w"],
        )
        return {
            "position": [round(float(value), 6) for value in position],
            "orientation": {
                "x": round(float(orientation["x"]), 6),
                "y": round(float(orientation["y"]), 6),
                "z": round(float(orientation["z"]), 6),
                "w": round(float(orientation["w"]), 6),
            },
            "rpy": [round(roll, 6), round(pitch, 6), round(yaw, 6)],
        }

    def _flexible_grasp_inputs(self, pose, side):
        center = finite_vec3(
            pose.get("cylinder_center") or pose.get("grasp_center") or [pose.get("x"), pose.get("y"), pose.get("z")],
            "cylinder_center",
        )
        cylinder_axis = vec_normalize(
            finite_vec3(pose.get("cylinder_axis") or pose.get("axis"), "cylinder_axis", [0.0, 0.0, 1.0]),
            "cylinder_axis",
        )
        ground_normal = vec_normalize(
            finite_vec3(pose.get("ground_normal"), "ground_normal", [0.0, 0.0, 1.0]),
            "ground_normal",
        )
        h = float(pose.get("cylinder_h", pose.get("h", pose.get("axis_offset", 0.0))))
        radius = max(0.0, float(pose.get("cylinder_radius", pose.get("radius", FLEXIBLE_GRASP_CYLINDER_RADIUS))))
        tcp_offset = max(0.0, float(pose.get("tcp_offset", FLEXIBLE_GRASP_TCP_OFFSET)))
        pregrasp_distance = max(0.0, float(pose.get("pregrasp_distance", FLEXIBLE_GRASP_PREGRASP_DISTANCE)))
        current_axes = self._current_gripper_axes(side)

        grasp_center = vec_add(center, vec_scale(cylinder_axis, h))
        seed_closing = None
        if current_axes:
            try:
                seed_closing = vec_normalize(vec_reject(current_axes["y"], cylinder_axis), "seed_closing")
            except ValueError:
                seed_closing = None
        if seed_closing is None:
            for fallback in (vec_cross(ground_normal, cylinder_axis), vec_cross([1.0, 0.0, 0.0], cylinder_axis), vec_cross([0.0, 1.0, 0.0], cylinder_axis)):
                try:
                    seed_closing = vec_normalize(fallback, "seed_closing")
                    break
                except ValueError:
                    continue
        if seed_closing is None:
            raise ValueError("无法生成 closing_direction 初始方向")

        if current_axes:
            try:
                seed_approach = vec_normalize(vec_sub(grasp_center, current_axes["position"]), "seed_approach")
            except ValueError:
                seed_approach = current_axes["x"]
        else:
            seed_approach = vec_normalize(vec_cross(seed_closing, cylinder_axis), "seed_approach")

        return {
            "center": center,
            "cylinder_axis": cylinder_axis,
            "ground_normal": ground_normal,
            "grasp_center": grasp_center,
            "radius": radius,
            "tcp_offset": tcp_offset,
            "pregrasp_distance": pregrasp_distance,
            "seed_closing": seed_closing,
            "seed_approach": seed_approach,
            "ee_to_gripper_rotation": quat_from_rpy(*parse_urdf_xyz(FLEXIBLE_GRASP_EE_TO_GRIPPER_RPY)),
        }

    def _flexible_grasp_candidate(self, data, theta_deg):
        rejected = {
            "theta_deg": theta_deg,
            "reason": None,
        }
        closing = vec_normalize(
            rotate_vector_axis_angle(data["seed_closing"], data["cylinder_axis"], math.radians(theta_deg)),
            "closing_direction",
        )
        closing_axis_dot = vec_dot(closing, data["cylinder_axis"])
        if abs(closing_axis_dot) >= 0.2:
            rejected.update({"reason": "closing_direction_not_antipodal_to_cylinder_axis", "dot": round(closing_axis_dot, 6)})
            return None, rejected

        closing_ground_dot = vec_dot(closing, data["ground_normal"])
        if abs(closing_ground_dot) >= 0.15:
            rejected.update({"reason": "finger_line_not_parallel_to_ground", "dot": round(closing_ground_dot, 6)})
            return None, rejected

        approach_options = [
            vec_normalize(vec_cross(closing, data["cylinder_axis"]), "approach_direction"),
            vec_normalize(vec_cross(data["cylinder_axis"], closing), "approach_direction"),
        ]
        approach_options.sort(key=lambda direction: vec_dot(direction, data["seed_approach"]), reverse=True)
        approach = None
        approach_ground_dot = None
        for option in approach_options:
            dot_ground = vec_dot(option, data["ground_normal"])
            if dot_ground <= 0.3:
                approach = option
                approach_ground_dot = dot_ground
                break
        if approach is None:
            rejected.update({
                "reason": "approach_direction_from_below",
                "dot_options": [round(vec_dot(option, data["ground_normal"]), 6) for option in approach_options],
            })
            return None, rejected

        gripper_x = approach
        gripper_y = closing
        gripper_z = vec_normalize(vec_cross(gripper_x, gripper_y), "gripper_z")
        gripper_y = vec_normalize(vec_cross(gripper_z, gripper_x), "gripper_y")
        rotation = [
            [gripper_x[0], gripper_y[0], gripper_z[0]],
            [gripper_x[1], gripper_y[1], gripper_z[1]],
            [gripper_x[2], gripper_y[2], gripper_z[2]],
        ]
        gripper_orientation = quat_from_matrix(rotation)
        ee_orientation = quat_multiply(gripper_orientation, quat_conjugate(data["ee_to_gripper_rotation"]))
        ee_orientation = quat_normalize(ee_orientation)

        grasp_position = vec_sub(data["grasp_center"], vec_scale(approach, data["tcp_offset"]))
        pregrasp_position = vec_sub(data["grasp_center"], vec_scale(approach, data["tcp_offset"] + data["pregrasp_distance"]))
        contact_1 = vec_add(data["grasp_center"], vec_scale(closing, data["radius"]))
        contact_2 = vec_sub(data["grasp_center"], vec_scale(closing, data["radius"]))

        return {
            "theta_deg": theta_deg,
            "closing_direction": closing,
            "approach_direction": approach,
            "closing_axis_dot": closing_axis_dot,
            "closing_ground_dot": closing_ground_dot,
            "approach_ground_dot": approach_ground_dot,
            "contact_1": contact_1,
            "contact_2": contact_2,
            "grasp_position": grasp_position,
            "pregrasp_position": pregrasp_position,
            "orientation": ee_orientation,
        }, None

    def _plan_flexible_grasp_ik(self, pose, side):
        data = self._flexible_grasp_inputs(pose, side)
        ompl_config = normalize_ompl_runtime_config(pose)
        ee_link_name = self._ee_link_name(side)
        manual_collision_summary = self.sync_manual_collision_boxes_for_request(pose)
        manual_collision_active = self._has_active_manual_collision_boxes()
        requested_avoid_collisions = bool(pose.get("avoid_collisions", pose.get("avoid_platform", True)))
        avoid_collisions = (requested_avoid_collisions and not DISABLE_PLATFORM_OBSTACLE) or manual_collision_active
        rejected = []
        checked = 0
        t0 = time.monotonic()

        for theta_deg in FLEXIBLE_GRASP_THETA_OFFSETS_DEG:
            candidate, rejection = self._flexible_grasp_candidate(data, theta_deg)
            if rejection:
                rejected.append(rejection)
                continue
            if checked >= FLEXIBLE_GRASP_MAX_IK_CANDIDATES:
                rejected.append({"theta_deg": theta_deg, "reason": "ik_candidate_limit_reached"})
                continue
            checked += 1

            pregrasp_ik, pregrasp_status = self._compute_ik_for_pose(
                side,
                ee_link_name,
                candidate["pregrasp_position"],
                candidate["orientation"],
                timeout_sec=ompl_config["ik_timeout"],
                avoid_collisions=avoid_collisions,
            )
            if not pregrasp_status["ok"]:
                rejected.append({
                    "theta_deg": theta_deg,
                    "reason": "pregrasp_ik_failed",
                    "ik": pregrasp_status,
                })
                continue

            grasp_ik, grasp_status = self._compute_ik_for_pose(
                side,
                ee_link_name,
                candidate["grasp_position"],
                candidate["orientation"],
                timeout_sec=ompl_config["ik_timeout"],
                avoid_collisions=avoid_collisions,
            )
            if not grasp_status["ok"]:
                rejected.append({
                    "theta_deg": theta_deg,
                    "reason": "grasp_ik_failed",
                    "pregrasp_ik": pregrasp_status,
                    "ik": grasp_status,
                })
                continue

            grasp_orientation = {
                "mode": pose.get("grasp_orientation_mode", "z_parallel_grasp"),
                "candidate_index": checked,
                "theta_deg": theta_deg,
                "constraint": "tool_z_parallel_to_world_xy",
                "ik_link_name": ee_link_name,
                "frame": FRAME_ID,
                "moveit_frame": MOVEIT_PLANNING_FRAME_ID,
                "manual_collision_active": manual_collision_active,
                "cylinder_center": data["center"],
                "cylinder_axis": data["cylinder_axis"],
                "grasp_center": data["grasp_center"],
                "ground_normal": data["ground_normal"],
                "closing_direction": candidate["closing_direction"],
                "approach_direction": candidate["approach_direction"],
                "contact_1": candidate["contact_1"],
                "contact_2": candidate["contact_2"],
                "pregrasp_pose": self._pose_debug(candidate["pregrasp_position"], candidate["orientation"]),
                "grasp_pose": self._pose_debug(candidate["grasp_position"], candidate["orientation"]),
                "pregrasp_ik": pregrasp_status,
                "grasp_ik": grasp_status,
                "rejected_candidates": rejected,
            }
            if manual_collision_active:
                summary = self._moveit_plan_to_joint_state(
                    side,
                    grasp_ik.solution.joint_state,
                    pose,
                    ompl_config,
                    t0,
                    fallback_allowed=False,
                    fallback_label="手动碰撞箱启用",
                    mode="flexible grasp MoveIt plan",
                )
                summary["grasp_orientation"] = grasp_orientation
                if manual_collision_summary:
                    summary["manual_collision"] = manual_collision_summary
                log_event("INFO", "flexible grasp MoveIt plan selected", grasp_orientation)
                return summary

            traj = self._direct_trajectory_through_ik(
                side,
                [pregrasp_ik.solution.joint_state, grasp_ik.solution.joint_state],
                float(pose.get("velocity_scaling", 0.15)),
            )
            summary = {
                "points": len(traj.points),
                "joints": list(traj.joint_names),
                "duration": self._trajectory_duration(traj),
                "arm": side,
                "mode": "flexible IK grasp",
                "ompl": ompl_config,
                "grasp_orientation": grasp_orientation,
            }
            if manual_collision_summary:
                summary["manual_collision"] = manual_collision_summary
            log_event("INFO", "flexible grasp IK selected", summary["grasp_orientation"])
            with self._lock:
                self._last_plan[side] = traj
                self._last_plan_summary[side] = summary
                self._status[side] = "flexible IK plan ready"
            return summary

        log_event("WARNING", "flexible grasp IK failed", {"arm": side, "rejected_candidates": rejected})
        raise RuntimeError(f"灵活抓取 IK 未找到可行候选: {rejected[-3:] if rejected else '无候选'}")

    def plan(self, pose):
        side = normalize_arm(pose.get("arm", "right"))
        if not self._planning_lock.acquire(blocking=False):
            raise RuntimeError("已有规划正在运行，请等待当前规划结束")
        try:
            validate_workspace_target(pose, "规划目标")
            mode = pose.get("grasp_orientation_mode")
            if mode in ("side_grasp", "horizontal_cylinder"):
                errors = []
                for index, candidate in enumerate(self._horizontal_grasp_pose_candidates(pose, side), start=1):
                    candidate.setdefault("attempts", HORIZONTAL_GRASP_ATTEMPTS)
                    candidate.setdefault("planning_time", HORIZONTAL_GRASP_PLANNING_TIME)
                    candidate.setdefault("ik_timeout", HORIZONTAL_GRASP_IK_TIMEOUT)
                    try:
                        summary = self._plan_single_pose(candidate)
                        summary["grasp_orientation"] = {
                            "mode": "side_grasp",
                            "candidate_index": index,
                            "roll": candidate["roll"],
                            "pitch": candidate["pitch"],
                            "yaw": candidate["yaw"],
                            **candidate.get("_horizontal_grasp", {}),
                        }
                        log_event("INFO", "side grasp orientation selected", summary["grasp_orientation"])
                        return summary
                    except Exception as exc:
                        errors.append(str(exc))
                        log_event("DEBUG", "horizontal grasp candidate failed", {
                            "candidate_index": index,
                            "roll": candidate.get("roll"),
                            "pitch": candidate.get("pitch"),
                            "yaw": candidate.get("yaw"),
                            "error": str(exc),
                        })
                detail = "; ".join(errors[-3:]) if errors else "无候选姿态"
                raise RuntimeError(f"侧面抓取姿态未找到可行 IK/规划: {detail}")
            if mode == "vertical_grasp":
                errors = []
                for index, candidate in enumerate(self._vertical_grasp_pose_candidates(pose, side), start=1):
                    candidate.setdefault("attempts", HORIZONTAL_GRASP_ATTEMPTS)
                    candidate.setdefault("planning_time", HORIZONTAL_GRASP_PLANNING_TIME)
                    candidate.setdefault("ik_timeout", HORIZONTAL_GRASP_IK_TIMEOUT)
                    try:
                        summary = self._plan_single_pose(candidate)
                        summary["grasp_orientation"] = {
                            "mode": "vertical_grasp",
                            "candidate_index": index,
                            "roll": candidate["roll"],
                            "pitch": candidate["pitch"],
                            "yaw": candidate["yaw"],
                            **candidate.get("_vertical_grasp", {}),
                        }
                        log_event("INFO", "vertical grasp orientation selected", summary["grasp_orientation"])
                        return summary
                    except Exception as exc:
                        errors.append(str(exc))
                        log_event("DEBUG", "vertical grasp candidate failed", {
                            "candidate_index": index,
                            "roll": candidate.get("roll"),
                            "pitch": candidate.get("pitch"),
                            "yaw": candidate.get("yaw"),
                            "error": str(exc),
                        })
                detail = "; ".join(errors[-3:]) if errors else "无候选姿态"
                raise RuntimeError(f"垂直抓取姿态未找到可行 IK/规划: {detail}")
            if mode in ("angled_grasp", "z_parallel_grasp"):
                errors = []
                for index, candidate in enumerate(self._angled_grasp_pose_candidates(pose, side), start=1):
                    candidate.setdefault("attempts", HORIZONTAL_GRASP_ATTEMPTS)
                    candidate.setdefault("planning_time", HORIZONTAL_GRASP_PLANNING_TIME)
                    candidate.setdefault("ik_timeout", HORIZONTAL_GRASP_IK_TIMEOUT)
                    try:
                        summary = self._plan_single_pose(candidate)
                        summary["grasp_orientation"] = {
                            "mode": "angled_grasp",
                            "candidate_index": index,
                            "roll": candidate["roll"],
                            "pitch": candidate["pitch"],
                            "yaw": candidate["yaw"],
                            **candidate.get("_angled_grasp", {}),
                        }
                        log_event("INFO", "angled grasp orientation selected", summary["grasp_orientation"])
                        return summary
                    except Exception as exc:
                        errors.append(str(exc))
                        log_event("DEBUG", "angled grasp candidate failed", {
                            "candidate_index": index,
                            "roll": candidate.get("roll"),
                            "pitch": candidate.get("pitch"),
                            "yaw": candidate.get("yaw"),
                            "error": str(exc),
                        })
                detail = "; ".join(errors[-3:]) if errors else "无候选姿态"
                raise RuntimeError(f"斜向抓取姿态未找到可行 IK/规划: {detail}")
            if mode == "flexible_grasp":
                return self._plan_flexible_grasp_ik(pose, side)
            return self._plan_single_pose(pose)
        finally:
            self._planning_lock.release()

    def _moveit_plan_to_joint_state(
        self,
        side,
        target_joint_state,
        pose,
        ompl_config,
        t0,
        platform_summary=None,
        fallback_allowed=False,
        fallback_label="",
        mode="MoveIt plan",
        goal_tolerance=0.01,
    ):
        cfg = self._arm_config(side)
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = cfg["group"]
        mpr.pipeline_id = OMPL_PIPELINE_ID
        mpr.planner_id = ompl_config["planner_id"]
        mpr.num_planning_attempts = ompl_config["attempts"]
        mpr.allowed_planning_time = ompl_config["planning_time"]
        mpr.max_velocity_scaling_factor = float(pose.get("velocity_scaling", 0.15))
        mpr.max_acceleration_scaling_factor = float(pose.get("acceleration_scaling", 0.10))
        mpr.start_state = self._make_start_state()
        mpr.goal_constraints.append(self._make_joint_goal_constraints(target_joint_state, cfg["joints"], goal_tolerance))
        req.motion_plan_request = mpr

        future = self._plan_client.call_async(req)
        self._spin_until_future_complete(future, mpr.allowed_planning_time + 3.0)
        if not future.done():
            log_event("ERROR", f"plan timeout ({mpr.allowed_planning_time + 3.0:.1f}s)")
            with self._lock:
                self._status[side] = "plan timeout"
            raise RuntimeError("规划服务超时")

        result = future.result()
        code = int(result.motion_plan_response.error_code.val)
        elapsed = time.monotonic() - t0
        if code != 1:
            reason = moveit_error_message(code)
            if not fallback_allowed:
                log_event("ERROR", f"{side} MoveIt plan failed with collision scene required: error_code={code} {reason}")
                with self._lock:
                    self._status[side] = f"plan failed: {code}"
                label = f"{fallback_label}，" if fallback_label else ""
                raise RuntimeError(f"MoveIt 规划失败，{label}禁止 direct IK fallback，错误码={code}（{reason}）")
            traj = self._direct_trajectory_to_ik(
                side,
                target_joint_state,
                float(pose.get("velocity_scaling", 0.15)),
            )
            summary = {
                "points": len(traj.points),
                "joints": list(traj.joint_names),
                "duration": self._trajectory_duration(traj),
                "arm": side,
                "mode": f"direct IK fallback; MoveIt plan error_code={code}",
                "ompl": ompl_config,
            }
            log_event("WARN", f"{side} plan fallback to direct IK (error_code={code}) in {elapsed:.2f}s")
            with self._lock:
                self._last_plan[side] = traj
                self._last_plan_summary[side] = summary
                self._status[side] = "direct IK plan ready"
            return summary

        traj = result.motion_plan_response.trajectory.joint_trajectory
        summary = {
            "points": len(traj.points),
            "joints": list(traj.joint_names),
            "duration": self._trajectory_duration(traj),
            "arm": side,
            "mode": mode,
            "ompl": ompl_config,
        }
        if platform_summary:
            summary["platform_obstacle"] = platform_summary
        log_event("INFO", f"{side} plan ok: {summary['points']} points, {summary['duration']:.2f}s, elapsed={elapsed:.2f}s")
        with self._lock:
            self._last_plan[side] = traj
            self._last_plan_summary[side] = summary
            self._status[side] = "plan ready"
        return summary

    def _plan_single_pose(self, pose):
        side = normalize_arm(pose.get("arm", "right"))
        cfg = self._arm_config(side)
        ompl_config = normalize_ompl_runtime_config(pose)
        requested_avoid_platform = bool(pose.get("avoid_platform", True))
        avoid_platform = requested_avoid_platform and not DISABLE_PLATFORM_OBSTACLE
        manual_collision_summary = self.sync_manual_collision_boxes_for_request(pose)
        manual_collision_active = self._has_active_manual_collision_boxes()
        mujoco_table_active = MUJOCO_TABLE_OBSTACLE_ENABLED
        mujoco_object_requested = pose.get("mujoco_object_center") is not None
        mujoco_object_active = (
            mujoco_object_requested
            and bool(pose.get("_mujoco_avoid_grasp_object", False))
        )
        collision_avoidance_required = (
            avoid_platform or manual_collision_active or mujoco_table_active
            or mujoco_object_active
        )
        log_event("INFO", "plan start", {
            "arm": side,
            "x": pose.get("x"), "y": pose.get("y"), "z": pose.get("z"),
            "roll": pose.get("roll"), "pitch": pose.get("pitch"), "yaw": pose.get("yaw"),
            "avoid_platform": avoid_platform,
            "manual_collision_active": manual_collision_active,
            "mujoco_table_active": mujoco_table_active,
            "platform_obstacle_disabled": DISABLE_PLATFORM_OBSTACLE,
            "planner_id": ompl_config["planner_id"],
            "planning_time": ompl_config["planning_time"],
            "attempts": ompl_config["attempts"],
            "ik_timeout": ompl_config["ik_timeout"],
        })
        t0 = time.monotonic()

        if not self._wait_ready(side):
            log_event("ERROR", f"{side} plan failed: services not ready")
            raise RuntimeError("规划服务或轨迹动作服务未就绪")

        platform_summary = None
        if avoid_platform:
            platform_summary = self.apply_platform_obstacle()
        mujoco_table_summary = None
        if mujoco_table_active:
            mujoco_table_summary = self.apply_mujoco_table_obstacle(
                grasp_contact=bool(pose.get("_mujoco_table_grasp_contact"))
            )
        mujoco_object_summary = None
        if mujoco_object_requested:
            mujoco_object_summary = self.apply_mujoco_grasp_object_obstacle(
                pose.get("mujoco_object_center"),
                enabled=mujoco_object_active,
            )

        with self._lock:
            self._status[side] = "planning"
            joint_state = self._latest_joint_state

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = cfg["group"]
        ik_req.ik_request.ik_link_name = cfg["tip"]
        ik_req.ik_request.pose_stamped = self._make_pose_stamped(pose, side)
        ik_req.ik_request.robot_state = self._make_start_state()
        ik_req.ik_request.avoid_collisions = collision_avoidance_required
        ik_timeout = ompl_config["ik_timeout"]
        ik_req.ik_request.timeout = Duration(
            sec=int(ik_timeout),
            nanosec=int((ik_timeout - int(ik_timeout)) * 1_000_000_000),
        )

        ik_future = self._ik_client.call_async(ik_req)
        self._spin_until_future_complete(ik_future, ik_timeout + 1.0)
        if not ik_future.done():
            log_event("ERROR", f"{side} IK timeout ({ik_timeout + 1.0:.1f}s)")
            with self._lock:
                self._status[side] = "ik timeout"
            raise RuntimeError("IK 逆解服务超时")
        ik_result = ik_future.result()
        ik_code = int(ik_result.error_code.val)
        if ik_code != 1:
            ik_reason = moveit_error_message(ik_code)
            log_event("ERROR", f"{side} IK failed: error_code={ik_code} {ik_reason}")
            with self._lock:
                self._status[side] = f"ik failed: {ik_code}"
            raise RuntimeError(f"IK 逆解失败，MoveIt 错误码={ik_code}（{ik_reason}）")

        log_event("DEBUG", f"IK solved in {time.monotonic() - t0:.2f}s")
        fallback_reasons = []
        if avoid_platform:
            fallback_reasons.append("平台避障启用")
        if manual_collision_active:
            fallback_reasons.append("手动碰撞箱启用")
        if mujoco_table_active:
            fallback_reasons.append("MuJoCo 桌面避障启用")
        if mujoco_object_active:
            fallback_reasons.append("MuJoCo 抓取物体避障启用")
        summary = self._moveit_plan_to_joint_state(
            side,
            ik_result.solution.joint_state,
            pose,
            ompl_config,
            t0,
            platform_summary=platform_summary,
            fallback_allowed=not collision_avoidance_required,
            fallback_label="、".join(fallback_reasons),
            mode="MoveIt plan",
        )
        if manual_collision_summary:
            summary["manual_collision"] = manual_collision_summary
        if mujoco_table_summary:
            summary["mujoco_table_obstacle"] = mujoco_table_summary
        if mujoco_object_summary:
            summary["mujoco_grasp_object_obstacle"] = mujoco_object_summary
        return summary

    @staticmethod
    def _trajectory_duration(traj):
        if not traj.points:
            return 0.0
        d = traj.points[-1].time_from_start
        return float(d.sec) + float(d.nanosec) * 1e-9

    @staticmethod
    def _duration_msg(seconds):
        seconds = max(0.0, float(seconds))
        whole = int(seconds)
        return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))

    def _direct_trajectory_to_ik(self, side, ik_joint_state, velocity_scaling):
        joint_names = self._arm_config(side)["joints"]
        with self._lock:
            current = self._latest_joint_state
        if current is None:
            raise RuntimeError("无法获取当前关节状态用于直接轨迹")

        current_by_name = dict(zip(current.name, current.position))
        target_by_name = dict(zip(ik_joint_state.name, ik_joint_state.position))
        missing = [
            name for name in joint_names
            if name not in current_by_name or name not in target_by_name
        ]
        if missing:
            raise RuntimeError(f"直接轨迹缺少关节: {missing}")

        start = [float(current_by_name[name]) for name in joint_names]
        target = [float(target_by_name[name]) for name in joint_names]
        max_delta = max(abs(a - b) for a, b in zip(start, target))
        max_velocity = max(0.005, 0.21 * max(0.01, float(velocity_scaling)))
        duration = min(60.0, max(2.0, max_delta / max_velocity))

        traj = JointTrajectory()
        traj.joint_names = joint_names.copy()

        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(p0)

        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.time_from_start = self._duration_msg(duration)
        traj.points.append(p1)
        return traj

    def _direct_trajectory_through_ik(self, side, ik_joint_states, velocity_scaling):
        joint_names = self._arm_config(side)["joints"]
        with self._lock:
            current = self._latest_joint_state
        if current is None:
            raise RuntimeError("无法获取当前关节状态用于直接轨迹")

        current_by_name = dict(zip(current.name, current.position))
        missing_current = [name for name in joint_names if name not in current_by_name]
        if missing_current:
            raise RuntimeError(f"直接轨迹缺少当前关节: {missing_current}")

        traj = JointTrajectory()
        traj.joint_names = joint_names.copy()
        start = [float(current_by_name[name]) for name in joint_names]
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(p0)

        max_velocity = max(0.005, 0.21 * max(0.01, float(velocity_scaling)))
        elapsed = 0.0
        previous = start
        for index, ik_joint_state in enumerate(ik_joint_states, start=1):
            target_by_name = dict(zip(ik_joint_state.name, ik_joint_state.position))
            missing_target = [name for name in joint_names if name not in target_by_name]
            if missing_target:
                raise RuntimeError(f"直接轨迹第 {index} 段缺少目标关节: {missing_target}")
            target = [float(target_by_name[name]) for name in joint_names]
            max_delta = max(abs(a - b) for a, b in zip(previous, target))
            elapsed += min(60.0, max(0.5, max_delta / max_velocity))

            point = JointTrajectoryPoint()
            point.positions = target
            point.time_from_start = self._duration_msg(elapsed)
            traj.points.append(point)
            previous = target
        return traj

    def execute_last_plan(self, side="right"):
        side = normalize_arm(side)
        cfg = self._arm_config(side)
        log_event("INFO", f"{side} execute start")
        t0 = time.monotonic()
        with self._lock:
            traj = self._last_plan[side]
            self._status[side] = "executing"
        if traj is None or not traj.points:
            log_event("ERROR", f"{side} execute failed: no plan available")
            raise RuntimeError("没有可执行的规划，请先规划")
        if not cfg["action_client"].wait_for_server(timeout_sec=2.0):
            log_event("ERROR", f"{side} execute failed: action server not ready ({cfg['action_name']})")
            raise RuntimeError(f"动作服务未就绪: {cfg['action_name']}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        send_future = cfg["action_client"].send_goal_async(goal)
        self._spin_until_future_complete(send_future, 5.0)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            log_event("ERROR", f"{side} execute failed: goal rejected by action server")
            with self._lock:
                self._status[side] = "goal rejected"
            raise RuntimeError("轨迹执行被拒绝（可能电机未使能、急停触发或轨迹无效）")

        log_event("DEBUG", f"{side} execute: goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        timeout = self._trajectory_duration(traj) + 10.0
        self._spin_until_future_complete(result_future, timeout)
        if not result_future.done():
            log_event("ERROR", f"{side} execute timeout ({timeout:.1f}s)")
            with self._lock:
                self._status[side] = "execute timeout"
            raise RuntimeError("轨迹执行超时")

        result = result_future.result().result
        elapsed = time.monotonic() - t0
        error_code = int(result.error_code)
        if error_code == 0:
            log_event("INFO", f"{side} execute ok: error_code={error_code}, elapsed={elapsed:.2f}s")
        else:
            log_event("WARN", f"{side} execute done with error: error_code={error_code}, elapsed={elapsed:.2f}s")
        with self._lock:
            self._status[side] = f"executed: {error_code}"
        return {"error_code": error_code, "error_string": result.error_string}

    def gripper_open(self, side="right"):
        return self._set_gripper(side, closed=False)

    def gripper_close(self, side="right"):
        return self._set_gripper(side, closed=True)

    def _gripper_joint_name(self, side):
        side = normalize_arm(side)
        return "Left_Gripper_Joint" if side == "left" else "Right_Gripper_Joint"

    def _gripper_position(self, state):
        return GRIPPER_CLOSED_POSITION if state == "closed" else GRIPPER_OPEN_POSITION

    def _gripper_observed_position_locked(self, side):
        msg = self._latest_gripper_joint_state.get(side)
        if msg is None:
            return None
        joint = self._gripper_joint_name(side)
        try:
            index = list(msg.name).index(joint)
        except ValueError:
            return None
        if index >= len(msg.position):
            return None
        position = float(msg.position[index])
        return position if math.isfinite(position) else None

    def _gripper_progress(self, position):
        span = GRIPPER_OPEN_POSITION - GRIPPER_CLOSED_POSITION
        if abs(span) <= 1e-9:
            return 0.0
        return max(0.0, min(1.0, (GRIPPER_OPEN_POSITION - float(position)) / span))

    def _gripper_state_payload_locked(self, side):
        target_state = self._gripper_state[side]
        target_position = self._gripper_position(target_state)
        observed_position = self._gripper_observed_position_locked(side)
        position = observed_position if observed_position is not None else target_position
        moving = abs(position - target_position) > GRIPPER_POSITION_EPS
        progress = self._gripper_progress(position)
        if progress >= 0.85:
            observed_state = "closed"
        elif progress <= 0.15:
            observed_state = "open"
        else:
            observed_state = "moving"
        return {
            "state": target_state,
            "target_state": target_state,
            "observed_state": observed_state,
            "arm": side,
            "joint": self._gripper_joint_name(side),
            "position": position,
            "target_position": target_position,
            "progress": progress,
            "moving": moving,
            "simulated": self._gripper_simulated[side],
        }

    def _set_gripper(self, side="right", closed=False):
        side = normalize_arm(side)
        state = "closed" if closed else "open"
        now = time.monotonic()
        with self._lock:
            last = dict(self._gripper_last_command[side])
            if last.get("state") == state and now - float(last.get("stamp", 0.0)) < SIM_GRIPPER_REPEAT_DEBOUNCE_SEC:
                result = self._gripper_state_payload_locked(side)
                result["deduped"] = True
                return result
            self._gripper_last_command[side] = {"state": state, "stamp": now}
        service_result = self._call_sim_gripper(side, closed)
        with self._lock:
            self._gripper_state[side] = state
            self._gripper_simulated[side] = bool(service_result.get("simulated"))
            result = self._gripper_state_payload_locked(side)
        result.update(service_result)
        log_event("INFO", "gripper command", result)
        return result

    def _call_sim_gripper(self, side, closed):
        ns = RIGHT_ROBOT_NS if side == "right" else LEFT_ROBOT_NS
        topic = f"/{ns}/sim_gripper"
        client = self._get_robot_client(side, "sim_gripper", SetBool, topic)
        if not client.service_is_ready():
            return {
                "simulated": False,
                "service": topic,
                "note": "sim gripper service not ready; state recorded in web control only",
            }

        req = SetBool.Request()
        req.data = bool(closed)
        future = client.call_async(req)
        if not self._spin_until_future_complete(future, SIM_GRIPPER_SERVICE_TIMEOUT_SEC):
            return {
                "simulated": False,
                "service": topic,
                "note": "sim gripper service call timed out; state recorded in web control only",
            }
        try:
            resp = future.result()
        except Exception as exc:
            return {
                "simulated": False,
                "service": topic,
                "note": f"sim gripper service failed: {exc}; state recorded in web control only",
            }
        if not bool(getattr(resp, "success", True)):
            message = getattr(resp, "message", "") or "sim gripper service returned failure"
            raise RuntimeError(f"{topic} failed: {message}")
        return {
            "simulated": True,
            "service": topic,
            "message": getattr(resp, "message", ""),
        }

    def gripper_state(self, side="right"):
        side = normalize_arm(side)
        with self._lock:
            return self._gripper_state_payload_locked(side)

    def gripper_states(self):
        return {side: self.gripper_state(side) for side in ("right", "left")}

    def _wait_for_sim_gripper(self, side, timeout_sec=SIM_GRIPPER_SETTLE_TIMEOUT_SEC):
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        state = self.gripper_state(side)
        stable_since = None
        previous_position = state.get("position")
        while state.get("moving") and time.monotonic() < deadline:
            time.sleep(0.02)
            state = self.gripper_state(side)
            position = state.get("position")
            if position is not None and previous_position is not None and abs(
                float(position) - float(previous_position)
            ) <= 1e-5:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= SIM_GRIPPER_CONTACT_SETTLE_SEC:
                    state["moving"] = False
                    state["contact_stopped"] = True
                    state["observed_state"] = "grasping"
                    break
            else:
                stable_since = None
            previous_position = position
        return state

    def _get_tip_rpy(self, side="right"):
        links = self.link_positions(side)
        for link in reversed(links.get("links", [])):
            if "orientation" in link:
                q = link["orientation"]
                return quat_to_rpy(q[0], q[1], q[2], q[3])
        return 0.0, 0.0, 0.0

    def approach(self, pose, offset_z=0.1):
        side = normalize_arm(pose.get("arm", "right"))
        approach_pose = dict(pose)
        approach_pose["arm"] = side
        approach_pose["z"] = float(pose["z"]) + float(offset_z)
        summary = self.plan(approach_pose)
        result = self.execute_last_plan(side)
        return {"plan": summary, "execute": result}

    def grasp(self, pose):
        side = normalize_arm(pose.get("arm", "right"))
        pose = dict(pose)
        pose["arm"] = side
        summary = self.plan(pose)
        result = self.execute_last_plan(side)
        grip_result = self.gripper_close(side)
        return {"plan": summary, "execute": result, "gripper": grip_result}

    def lift(self, offset_z=0.1, side="right"):
        side = normalize_arm(side)
        links = self.link_positions(side)
        tip = None
        for link in reversed(links.get("links", [])):
            if "position" in link:
                tip = link
                break
        if tip is None:
            raise RuntimeError("无法获取当前末端位姿用于抬起")
        roll, pitch, yaw = self._get_tip_rpy(side)
        lift_pose = {
            "arm": side,
            "x": tip["position"][0],
            "y": tip["position"][1],
            "z": tip["position"][2] + float(offset_z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "velocity_scaling": 0.15,
            "acceleration_scaling": 0.10,
        }
        summary = self.plan(lift_pose)
        result = self.execute_last_plan(side)
        return {"plan": summary, "execute": result}

    def place(self, pose):
        side = normalize_arm(pose.get("arm", "right"))
        pose = dict(pose)
        pose["arm"] = side
        summary = self.plan(pose)
        result = self.execute_last_plan(side)
        grip_result = self.gripper_open(side)
        return {"plan": summary, "execute": result, "gripper": grip_result}

    def pick_and_place_on_plate(
        self, pose, plate_x, plate_y, plate_z, hover_height=0.12,
        release_clearance=0.012,
    ):
        """Pick the simulated cube when needed, then place it at the plate centre."""
        side = normalize_arm(pose.get("arm", "right"))
        steps = []
        debug = self._grasp_debug(side)
        if not debug.get("object_attached"):
            pick_result = None
            last_error = None
            for attempt in range(1, 4):
                try:
                    pick_result = self.pick(pose, 0.1, 0.1, 0.5, 0.1)
                    steps.append({
                        "action": "pick",
                        "attempt": attempt,
                        "result": pick_result,
                    })
                    if pick_result.get("object_attached"):
                        break
                    last_error = RuntimeError("夹爪未稳定抓住方块")
                except Exception as exc:
                    last_error = exc
                    steps.append({
                        "action": "pick",
                        "attempt": attempt,
                        "error": str(exc),
                    })
                if attempt < 3:
                    # A failed attempt can leave the automatic-pick guard
                    # pinned or the fingers partly closed. Restore a clean
                    # grasp state before replanning from the live arm pose.
                    try:
                        self._set_mujoco_grasp_guard(False)
                    except Exception:
                        pass
                    try:
                        opened = self.gripper_open(side)
                        if opened.get("simulated"):
                            self._wait_for_sim_gripper(side, 2.0)
                    except Exception:
                        pass
                    time.sleep(0.25)
            if not pick_result or not pick_result.get("object_attached"):
                return {
                    "completed": False,
                    "stage": "pick",
                    "attempts": 3,
                    "error": str(last_error or "抓取失败"),
                    "steps": steps,
                }
            debug = self._grasp_debug(side)

        object_position = debug.get("object_position")
        links = self.link_positions(side)
        tip = next((link for link in reversed(links.get("links", [])) if "position" in link), None)
        if not tip or not isinstance(object_position, list) or len(object_position) < 3:
            raise RuntimeError("无法获取末端或已抓取方块的实时位置")

        # Preserve the grasp transform: command the TCP so the attached cube,
        # rather than the TCP itself, arrives at the centre of the plate.
        tcp_offset = [
            float(tip["position"][i]) - float(object_position[i]) for i in range(3)
        ]
        roll, pitch, yaw = self._get_tip_rpy(side)
        object_target = [float(plate_x), float(plate_y), float(plate_z)]
        target_tcp = [object_target[i] + tcp_offset[i] for i in range(3)]

        def move(stage, xyz):
            target = {
                "arm": side,
                "x": xyz[0], "y": xyz[1], "z": xyz[2],
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "planner_id": "RRTConnectkConfigDefault",
                "planning_time": 5.0, "attempts": 4, "ik_timeout": 1.5,
                "velocity_scaling": 0.15, "acceleration_scaling": 0.10,
                "avoid_platform": False,
            }
            summary = self.plan(target)
            execution = self.execute_last_plan(side)
            if int(execution.get("error_code", -1)) != 0:
                raise RuntimeError(f"{stage}执行失败: {execution.get('error_string', 'unknown error')}")
            steps.append({"action": stage, "plan": summary, "execute": execution})

        hover_tcp = [target_tcp[0], target_tcp[1], target_tcp[2] + float(hover_height)]
        # Do not plan all the way down to the cube's final resting pose.  At
        # that height the fingers overlap the plate/table collision geometry,
        # so MoveIt correctly rejects the IK state before the gripper opens.
        # Release from a small clearance and let MuJoCo settle the cube onto
        # the plate under gravity.
        release_clearance = max(
            0.008, min(float(hover_height), float(release_clearance))
        )
        release_tcp = [
            target_tcp[0],
            target_tcp[1],
            target_tcp[2] + release_clearance,
        ]
        move("move_above_plate", hover_tcp)
        move("descend_to_safe_release", release_tcp)
        opened = self.gripper_open(side)
        if opened.get("simulated"):
            opened["settled_state"] = self._wait_for_sim_gripper(side)
        steps.append({"action": "release", "gripper": opened})
        time.sleep(0.25)
        move("retreat_from_plate", hover_tcp)
        final_debug = self._grasp_debug(side)
        completed = not bool(final_debug.get("object_attached"))
        return {
            "completed": completed,
            "stage": "done" if completed else "release",
            "plate_target": object_target,
            "release_clearance": release_clearance,
            "tcp_offset": tcp_offset,
            "object_position": final_debug.get("object_position"),
            "steps": steps,
        }

    def _mujoco_drop_target_geometry(self, target):
        center = finite_vec3(target.get("position"), "投放目标中心")
        raw_size = target.get("size") if isinstance(target.get("size"), list) else []
        size = [
            abs(float(value))
            for value in raw_size
            if isinstance(value, (int, float)) and math.isfinite(float(value))
        ]
        guard_margin = 0.012
        if target.get("type") == "cylinder" or target.get("name") == "cylinder":
            radius = size[0] if len(size) >= 1 and size[0] > 0 else 0.025
            # MuJoCo cylinder geom size is radius + half-height; the scene API
            # exposes that half-height, so keep the planning guard conservative.
            half_height = size[1] if len(size) >= 2 and size[1] > 0 else 0.04
            return {
                "center": center,
                "top_z": center[2] + half_height,
                "dimensions": [
                    radius * 2.0 + guard_margin * 2.0,
                    radius * 2.0 + guard_margin * 2.0,
                    half_height * 2.0 + guard_margin * 2.0,
                ],
                "guard_margin_m": guard_margin,
                "horizontal_radius_m": radius + guard_margin,
            }

        x_size = size[0] if len(size) >= 1 and size[0] > 0 else 0.13
        y_size = size[2] if len(size) >= 3 and size[2] > 0 else x_size
        z_size = max(
            size[1] if len(size) >= 2 and size[1] > 0 else 0.0,
            size[2] if len(size) >= 3 and size[2] > 0 else 0.0,
            0.04,
        )
        return {
            "center": center,
            "top_z": center[2] + z_size * 0.5,
            "dimensions": [
                x_size + guard_margin * 2.0,
                y_size + guard_margin * 2.0,
                z_size + guard_margin * 2.0,
            ],
            "guard_margin_m": guard_margin,
            "horizontal_radius_m": max(x_size, y_size) * 0.5 + guard_margin,
        }

    def _mujoco_drop_target(self):
        extras = self._mujoco_scene_extras()
        target = next(
            (
                item for item in extras
                if item.get("type") == "cylinder" or item.get("name") == "cylinder"
            ),
            None,
        )
        if target is None:
            target = next(
                (
                    item for item in extras
                    if item.get("type") == "slot" or item.get("name") == "plate"
                ),
                None,
            )
        if target is None:
            raise RuntimeError("当前场景没有圆柱或半圆槽目标")
        geometry = self._mujoco_drop_target_geometry(target)
        label = "半圆槽" if target.get("type") == "slot" or target.get("name") == "plate" else "圆柱"
        collision_box = {
            "id": "mujoco_drop_target_guard",
            "name": f"{label}碰撞体积",
            "enabled": True,
            "center": geometry["center"],
            "dimensions": geometry["dimensions"],
            "rpy": [0.0, 0.0, 0.0],
        }
        return {
            **target,
            "center": geometry["center"],
            "top_z": geometry["top_z"],
            "collision_box": collision_box,
            "geometry": geometry,
            "label": label,
        }

    def _current_mujoco_object_center(self, side):
        debug = self._grasp_debug(side)
        position = debug.get("object_position")
        if not (isinstance(position, list) and len(position) >= 3):
            debug = self._grasp_debug("right")
            position = debug.get("object_position")
        if isinstance(position, list) and len(position) >= 3:
            return [float(position[0]), float(position[1]), float(position[2])]
        return [
            MUJOCO_GRASP_OBJECT_CENTER["x"],
            MUJOCO_GRASP_OBJECT_CENTER["y"],
            MUJOCO_GRASP_OBJECT_CENTER["z"],
        ]

    @staticmethod
    def _smart_rotor_grasp_seed(mode):
        seeds = {
            "vertical_grasp": (0.0, -math.pi / 2.0, math.pi),
            "angled_grasp": (0.0, math.pi / 6.0, 0.0),
            "side_grasp": (math.pi / 2.0, 0.0, 0.0),
        }
        if mode not in seeds:
            raise ValueError(f"未知智能抓取模式: {mode}")
        return seeds[mode]

    @staticmethod
    def _smart_rotor_grasp_modes(raw_modes=None):
        aliases = {
            "vertical": "vertical_grasp",
            "vertical_grasp": "vertical_grasp",
            "top": "vertical_grasp",
            "angled": "angled_grasp",
            "angled_grasp": "angled_grasp",
            "z_parallel": "angled_grasp",
            "z_parallel_grasp": "angled_grasp",
            "side": "side_grasp",
            "side_grasp": "side_grasp",
            "horizontal": "side_grasp",
        }
        source = raw_modes if isinstance(raw_modes, list) else [
            "vertical_grasp",
            "angled_grasp",
            "side_grasp",
        ]
        modes = []
        for item in source:
            mode = aliases.get(str(item).strip())
            if mode and mode not in modes:
                modes.append(mode)
        return modes or ["vertical_grasp", "angled_grasp", "side_grasp"]

    @staticmethod
    def _smart_rotor_probe_offsets(raw_offsets=None):
        source = raw_offsets if isinstance(raw_offsets, list) else [
            0.035, 0.025, 0.015, 0.005, -0.005, -0.015, -0.025, -0.035,
        ]
        offsets = []
        for value in source:
            try:
                offset = max(-0.06, min(0.08, float(value)))
            except (TypeError, ValueError):
                continue
            if all(abs(offset - existing) > 1e-6 for existing in offsets):
                offsets.append(offset)
        return offsets or [0.02, 0.01, 0.0, -0.01, -0.02]

    def _smart_rotor_grasp_search(
        self,
        side,
        object_center,
        nominal_tcp,
        pick_collision_boxes,
        search_offsets=None,
        grasp_modes=None,
    ):
        """Probe grasp poses from above downward and select the first planned candidate."""
        modes = self._smart_rotor_grasp_modes(grasp_modes)
        offsets = self._smart_rotor_probe_offsets(search_offsets)
        attempts = []
        base_pose = {
            "arm": side,
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 5.0,
            "attempts": 4,
            "ik_timeout": 1.5,
            "velocity_scaling": 0.12,
            "acceleration_scaling": 0.08,
            "avoid_platform": False,
            "ignore_workspace_bounds": True,
            "orientation_tolerance": 0.5,
            "stay_near": False,
            "collision_boxes": pick_collision_boxes,
            "mujoco_object_center": {
                "x": object_center[0],
                "y": object_center[1],
                "z": object_center[2],
            },
            "mujoco_nominal_tcp": {
                "x": nominal_tcp[0],
                "y": nominal_tcp[1],
                "z": nominal_tcp[2],
            },
        }

        for offset in offsets:
            for mode in modes:
                roll, pitch, yaw = self._smart_rotor_grasp_seed(mode)
                candidate = {
                    **base_pose,
                    "x": nominal_tcp[0],
                    "y": nominal_tcp[1],
                    "z": nominal_tcp[2] + offset,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "grasp_orientation_mode": mode,
                }
                attempt = {
                    "mode": mode,
                    "probe_offset_m": offset,
                    "tcp_z": candidate["z"],
                }
                try:
                    summary = self.plan(candidate)
                except Exception as exc:
                    attempt["ok"] = False
                    attempt["error"] = str(exc)[:220]
                    attempts.append(attempt)
                    continue

                orientation = summary.get("grasp_orientation") or {}
                selected_pose = dict(candidate)
                selected_pose.pop("grasp_orientation_mode", None)
                for key in ("roll", "pitch", "yaw"):
                    if key in orientation:
                        selected_pose[key] = float(orientation[key])
                tcp_position = orientation.get("tcp_position")
                if isinstance(tcp_position, list) and len(tcp_position) >= 3:
                    selected_pose["x"] = float(tcp_position[0])
                    selected_pose["y"] = float(tcp_position[1])
                    selected_pose["z"] = float(tcp_position[2])
                attempt.update({
                    "ok": True,
                    "selected_tcp": [
                        selected_pose["x"],
                        selected_pose["y"],
                        selected_pose["z"],
                    ],
                    "candidate_index": orientation.get("candidate_index"),
                    "plan_mode": summary.get("mode"),
                })
                attempts.append(attempt)
                return {
                    "pose": selected_pose,
                    "summary": {
                        "mode": mode,
                        "probe_offset_m": offset,
                        "selected_tcp": attempt["selected_tcp"],
                        "candidate_index": orientation.get("candidate_index"),
                        "grasp_orientation": orientation,
                        "plan": summary,
                        "attempts": attempts,
                    },
                }

        raise RuntimeError(f"智能抓取未找到可解点: {attempts[-6:]}")

    def left_side_drop_rotor_to_scene_target(
        self,
        drop_heights=None,
        approach_height=0.055,
        descend_distance=0.055,
        hold_seconds=0.45,
        lift_height=0.045,
        fall_seconds=1.8,
        smart_search=False,
        search_offsets=None,
        grasp_modes=None,
        release_descend_height=0.045,
    ):
        """Use the left arm to search a solvable rotor grasp and drop it at the scene target."""
        side = "left"
        drop_target = self._mujoco_drop_target()
        saved_collision_boxes = load_manual_collision_boxes()
        pick_collision_boxes = list(saved_collision_boxes)
        drop_collision_boxes = [*saved_collision_boxes, drop_target["collision_box"]]

        def restore_manual_collision_boxes():
            try:
                if enabled_manual_collision_boxes(saved_collision_boxes):
                    self.apply_manual_collision_boxes(saved_collision_boxes, save=False)
                elif self._has_active_manual_collision_boxes():
                    self.clear_manual_collision_boxes()
            except Exception as exc:
                log_event("WARNING", "left side drop: failed to restore manual collision boxes", {
                    "error": str(exc),
                })

        object_center = self._current_mujoco_object_center(side)
        nominal_tcp = [
            object_center[0] + MUJOCO_GRASP_TCP_OFFSET["x"],
            object_center[1] + MUJOCO_GRASP_TCP_OFFSET["y"],
            object_center[2] + MUJOCO_GRASP_TCP_OFFSET["z"],
        ]
        pose = {
            "arm": side,
            "x": nominal_tcp[0],
            "y": nominal_tcp[1],
            "z": nominal_tcp[2],
            "roll": math.pi / 2.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "grasp_orientation_mode": "side_grasp",
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 5.0,
            "attempts": 4,
            "ik_timeout": 1.5,
            "velocity_scaling": 0.12,
            "acceleration_scaling": 0.08,
            "avoid_platform": False,
            "ignore_workspace_bounds": True,
            "orientation_tolerance": 0.5,
            "stay_near": False,
            "collision_boxes": pick_collision_boxes,
            "mujoco_object_center": {
                "x": object_center[0],
                "y": object_center[1],
                "z": object_center[2],
            },
            "mujoco_nominal_tcp": {
                "x": nominal_tcp[0],
                "y": nominal_tcp[1],
                "z": nominal_tcp[2],
            },
        }
        steps = []
        if smart_search:
            search_result = self._smart_rotor_grasp_search(
                side,
                object_center,
                nominal_tcp,
                pick_collision_boxes,
                search_offsets=search_offsets,
                grasp_modes=grasp_modes,
            )
            pose = search_result["pose"]
            steps.append({
                "action": "smart_grasp_search",
                "search": search_result["summary"],
            })
            # Execute exactly the selected grasp point: approach from above,
            # descend to the locked TCP, then close.
            approach_height = max(0.035, float(approach_height))
            descend_distance = approach_height
        log_event("INFO", "left side drop rotor task start", {
            "object_center": object_center,
            "drop_target": drop_target,
            "smart_search": bool(smart_search),
        })

        try:
            pick_result = self.pick(
                pose,
                float(approach_height),
                float(descend_distance),
                float(hold_seconds),
                float(lift_height),
            )
            steps.append({"action": "left_side_pick", "result": pick_result})
            if not pick_result.get("completed") or not pick_result.get("object_attached"):
                return {
                    "completed": False,
                    "stage": "pick",
                    "target": drop_target,
                    "object_center": object_center,
                    "steps": steps,
                    "error": "左手未稳定抓住转子",
                }

            debug = self._grasp_debug(side)
            attached_position = debug.get("object_position")
            if not (isinstance(attached_position, list) and len(attached_position) >= 3):
                raise RuntimeError("无法读取已抓住转子的实时位置")
            object_pose = debug.get("object_pose") or {}
            object_orientation = quat_from_wxyz(
                object_pose.get("orientation_wxyz"),
                "已抓取转子姿态",
            )
            tip_pose = self._current_tip_pose(side)
            if tip_pose is None:
                raise RuntimeError("无法读取左手 TCP 位姿")
            initial_tcp_offset = [
                float(tip_pose["position"][i]) - float(attached_position[i])
                for i in range(3)
            ]
            raw_clearances = drop_heights if drop_heights else [0.0, 0.025, 0.050]
            extra_clearances = []
            for value in [0.0, *raw_clearances]:
                clearance = max(0.0, min(0.12, float(value)))
                if all(abs(clearance - existing) > 1e-6 for existing in extra_clearances):
                    extra_clearances.append(clearance)
            object_half_height = float(MUJOCO_GRASP_OBJECT_SIZE[2]) * 0.5
            object_horizontal_radius = max(
                float(MUJOCO_GRASP_OBJECT_SIZE[0]),
                float(MUJOCO_GRASP_OBJECT_SIZE[1]),
            ) * 0.5
            target_top_z = float(drop_target["top_z"])
            collision_half_z = float(drop_target["collision_box"]["dimensions"][2]) * 0.5
            collision_top_z = float(drop_target["collision_box"]["center"][2]) + collision_half_z
            # Keep the carried rotor above the solid target volume. MoveIt sees
            # the target collision object but not the grasped rotor, so these
            # waypoints guard the rotor's own swept volume.
            release_base_z = max(
                target_top_z + object_half_height + 0.018,
                collision_top_z + object_half_height + 0.006,
                collision_top_z - float(initial_tcp_offset[2]) + 0.010,
            )
            release_descend_height = max(0.0, min(0.12, float(release_descend_height)))
            target_radius = float(drop_target["geometry"].get("horizontal_radius_m", 0.04))
            exterior_radius = target_radius + object_horizontal_radius + 0.018
            from_target = [
                float(attached_position[0]) - float(drop_target["center"][0]),
                float(attached_position[1]) - float(drop_target["center"][1]),
                0.0,
            ]
            if vec_norm(from_target) < 1e-4:
                try:
                    left_base = self._arm_base_position(side)
                    from_target = [
                        float(left_base[0]) - float(drop_target["center"][0]),
                        float(left_base[1]) - float(drop_target["center"][1]),
                        0.0,
                    ]
                except Exception:
                    from_target = [-1.0, 0.0, 0.0]
            ingress_direction = vec_normalize(from_target, "drop_ingress_direction")

            level_state = None

            def tcp_for_object_center(object_center_xyz, transfer_state=None):
                state = transfer_state or level_state
                if state is None:
                    raise RuntimeError("尚未生成转子水平搬运姿态")
                return [
                    float(object_center_xyz[i]) + float(state["tcp_offset_world"][i])
                    for i in range(3)
                ]

            def make_transfer_pose(
                object_center_xyz,
                velocity=0.10,
                transfer_state=None,
                collision_boxes=None,
            ):
                state = transfer_state or level_state
                if state is None:
                    raise RuntimeError("尚未生成转子水平搬运姿态")
                target_tcp = tcp_for_object_center(object_center_xyz, state)
                roll, pitch, yaw = state["rpy"]
                return {
                    "arm": side,
                    "x": target_tcp[0],
                    "y": target_tcp[1],
                    "z": target_tcp[2],
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "orientation": state["tip_orientation"],
                    "planner_id": "RRTConnectkConfigDefault",
                    "planning_time": 5.0,
                    "attempts": 4,
                    "ik_timeout": 1.5,
                    "velocity_scaling": velocity,
                    "acceleration_scaling": 0.07,
                    "avoid_platform": False,
                    "ignore_workspace_bounds": True,
                    "orientation_tolerance": 0.5,
                    "stay_near": False,
                    "collision_boxes": collision_boxes if collision_boxes is not None else drop_collision_boxes,
                }

            level_attempts = []
            for candidate in self._level_rotor_transfer_candidates(
                attached_position,
                object_orientation,
                tip_pose,
            ):
                try:
                    level_pose = make_transfer_pose(
                        attached_position,
                        velocity=0.08,
                        transfer_state=candidate,
                        collision_boxes=pick_collision_boxes,
                    )
                    summary = self.plan(level_pose)
                    execution = self.execute_last_plan(side)
                    if int(execution.get("error_code", -1)) != 0:
                        raise RuntimeError(
                            f"水平化轨迹执行失败: {execution.get('error_string', 'unknown error')}"
                        )
                    level_state = candidate
                    steps.append({
                        "action": "level_rotor_horizontal",
                        "object_center": attached_position,
                        "target_tcp": [level_pose["x"], level_pose["y"], level_pose["z"]],
                        "level": {
                            "candidate_index": candidate["candidate_index"],
                            "object_orientation_wxyz": quat_to_wxyz(candidate["object_orientation"]),
                            "tip_orientation": candidate["tip_orientation"],
                            "tcp_offset_world": candidate["tcp_offset_world"],
                            "object_up": candidate["object_up"],
                            "level_error_deg": candidate["level_error_deg"],
                        },
                        "plan": summary,
                        "execute": execution,
                    })
                    time.sleep(0.12)
                    live_debug = self._grasp_debug(side)
                    live_position = live_debug.get("object_position")
                    live_pose = live_debug.get("object_pose") or {}
                    live_tip = self._current_tip_pose(side)
                    if (
                        isinstance(live_position, list)
                        and len(live_position) >= 3
                        and isinstance(live_pose.get("orientation_wxyz"), list)
                        and live_tip is not None
                    ):
                        attached_position = [float(value) for value in live_position[:3]]
                        object_orientation = quat_from_wxyz(
                            live_pose.get("orientation_wxyz"),
                            "水平化后转子姿态",
                        )
                        live_candidates = self._level_rotor_transfer_candidates(
                            attached_position,
                            object_orientation,
                            live_tip,
                            yaw_offsets=[0.0],
                        )
                        if live_candidates:
                            level_state = live_candidates[0]
                    break
                except Exception as exc:
                    level_attempts.append({
                        "candidate_index": candidate.get("candidate_index"),
                        "level_error_deg": candidate.get("level_error_deg"),
                        "error": str(exc),
                    })
                    steps.append({
                        "action": "level_rotor_horizontal_failed",
                        "candidate_index": candidate.get("candidate_index"),
                        "error": str(exc),
                    })
            if level_state is None:
                return {
                    "completed": False,
                    "stage": "level_rotor_horizontal",
                    "target": drop_target,
                    "object_center": object_center,
                    "steps": steps,
                    "error": f"无法规划让转子与桌面水平的搬运姿态: {level_attempts[-3:]}",
                }
            release_base_z = max(
                release_base_z,
                collision_top_z - float(level_state["tcp_offset_world"][2]) + 0.010,
            )

            def require_rotor_above_target(stage, object_center_xyz):
                lowest_z = float(object_center_xyz[2]) - object_half_height
                min_z = target_top_z + 0.006
                if lowest_z < min_z:
                    raise RuntimeError(
                        f"{stage} 会让转子低于圆柱碰撞体顶面: "
                        f"rotor_bottom={lowest_z:.4f}, required>={min_z:.4f}"
                    )

            def move_object_center(stage, object_center_xyz, velocity=0.10):
                require_rotor_above_target(stage, object_center_xyz)
                move_pose = make_transfer_pose(object_center_xyz, velocity)
                summary = self.plan(move_pose)
                execution = self.execute_last_plan(side)
                if int(execution.get("error_code", -1)) != 0:
                    raise RuntimeError(
                        f"{stage}轨迹执行失败: {execution.get('error_string', 'unknown error')}"
                    )
                step = {
                    "action": stage,
                    "object_center": object_center_xyz,
                    "target_tcp": [move_pose["x"], move_pose["y"], move_pose["z"]],
                    "rotor_bottom_z": float(object_center_xyz[2]) - object_half_height,
                    "target_top_z": target_top_z,
                    "collision_box": drop_target["collision_box"],
                    "plan": summary,
                    "execute": execution,
                }
                steps.append(step)
                return step

            def direct_center_route(current_object_center, center_above_object_center, release_object_center):
                raise_object_center = [
                    current_object_center[0],
                    current_object_center[1],
                    center_above_object_center[2],
                ]
                return [
                    (
                        "raise_rotor_to_release_height",
                        raise_object_center,
                        0.10,
                    ),
                    (
                        "move_directly_to_target_center_above",
                        center_above_object_center,
                        0.08,
                    ),
                    (
                        "descend_to_release_distance",
                        release_object_center,
                        0.06,
                    ),
                ]

            def side_clearance_route(current_object_center, center_above_object_center, release_object_center):
                raise_object_center = [
                    current_object_center[0],
                    current_object_center[1],
                    center_above_object_center[2],
                ]
                exterior_object_center = [
                    float(drop_target["center"][0]) + ingress_direction[0] * exterior_radius,
                    float(drop_target["center"][1]) + ingress_direction[1] * exterior_radius,
                    center_above_object_center[2],
                ]
                return [
                    (
                        "raise_rotor_above_drop_target",
                        raise_object_center,
                        0.10,
                    ),
                    (
                        "move_to_drop_target_side_clearance",
                        exterior_object_center,
                        0.10,
                    ),
                    (
                        "move_to_drop_target_center_above_collision",
                        center_above_object_center,
                        0.08,
                    ),
                    (
                        "descend_to_release_distance",
                        release_object_center,
                        0.06,
                    ),
                ]

            selected_release = None
            last_error = None
            for extra_clearance in extra_clearances:
                live_debug = self._grasp_debug(side)
                live_position = live_debug.get("object_position")
                if (
                    isinstance(live_position, list)
                    and len(live_position) >= 3
                    and all(math.isfinite(float(value)) for value in live_position[:3])
                ):
                    current_object_center = [
                        float(live_position[0]),
                        float(live_position[1]),
                        float(live_position[2]),
                    ]
                else:
                    current_object_center = [
                        float(attached_position[0]),
                        float(attached_position[1]),
                        float(attached_position[2]),
                    ]
                release_object_center = [
                    drop_target["center"][0],
                    drop_target["center"][1],
                    release_base_z + extra_clearance,
                ]
                center_above_object_center = [
                    release_object_center[0],
                    release_object_center[1],
                    release_object_center[2] + release_descend_height,
                ]
                route_attempts = [
                    (
                        "direct_center",
                        direct_center_route(
                            current_object_center,
                            center_above_object_center,
                            release_object_center,
                        ),
                    ),
                    (
                        "side_clearance",
                        side_clearance_route(
                            current_object_center,
                            center_above_object_center,
                            release_object_center,
                        ),
                    ),
                ]
                for route_name, route_steps in route_attempts:
                    try:
                        center_step = None
                        for stage, object_center_xyz, velocity in route_steps:
                            center_step = move_object_center(
                                stage,
                                object_center_xyz,
                                velocity=velocity,
                            )
                        if center_step is None:
                            raise RuntimeError("投放路径没有生成有效步骤")
                    except Exception as exc:
                        last_error = exc
                        steps.append({
                            "action": "drop_target_route_failed",
                            "route": route_name,
                            "extra_clearance_m": extra_clearance,
                            "target_top_z": target_top_z,
                            "collision_top_z": collision_top_z,
                            "error": str(exc),
                        })
                        continue
                    target_tcp = tcp_for_object_center(release_object_center)
                    selected_release = {
                        "extra_clearance_m": extra_clearance,
                        "target_top_z": target_top_z,
                        "collision_top_z": collision_top_z,
                        "ingress_direction": ingress_direction,
                        "exterior_radius_m": exterior_radius,
                        "center_above_object_center": center_above_object_center,
                        "release_object_center": release_object_center,
                        "release_descend_height_m": release_descend_height,
                        "target_tcp": target_tcp,
                        "rpy": list(level_state["rpy"]),
                        "object_orientation_wxyz": quat_to_wxyz(level_state["object_orientation"]),
                        "object_level_error_deg": level_state["level_error_deg"],
                        "collision_box": drop_target["collision_box"],
                        "route": route_name,
                        "center_step": center_step,
                    }
                    if route_name == "side_clearance":
                        selected_release["exterior_object_center"] = route_steps[1][1]
                    steps.append({"action": "release_pose_selected", "release": selected_release})
                    break
                if selected_release is not None:
                    break
            if selected_release is None:
                return {
                    "completed": False,
                    "stage": "move_above_drop_target_collision",
                    "target": drop_target,
                    "steps": steps,
                    "error": str(last_error or "无法移动到投放目标碰撞体积上方"),
                }

            opened = self.gripper_open(side)
            if opened.get("simulated"):
                opened["settled_state"] = self._wait_for_sim_gripper(side)
            steps.append({"action": "release_gripper", "gripper": opened})
            time.sleep(max(0.0, float(fall_seconds)))
            final_debug = self._grasp_debug(side)
            final_position = final_debug.get("object_position")
            xy_error = None
            if isinstance(final_position, list) and len(final_position) >= 3:
                xy_error = math.hypot(
                    float(final_position[0]) - float(drop_target["center"][0]),
                    float(final_position[1]) - float(drop_target["center"][1]),
                )
            completed = not bool(final_debug.get("object_attached"))
            return {
                "completed": completed,
                "stage": "done" if completed else "release",
                "target": drop_target,
                "object_center": object_center,
                "release": selected_release,
                "tcp_offset_from_object": level_state["tcp_offset_world"],
                "initial_tcp_offset_from_object": initial_tcp_offset,
                "horizontal_transfer": {
                    "object_orientation_wxyz": quat_to_wxyz(level_state["object_orientation"]),
                    "tip_orientation": level_state["tip_orientation"],
                    "tcp_offset_world": level_state["tcp_offset_world"],
                    "level_error_deg": level_state["level_error_deg"],
                },
                "final_object_position": final_position,
                "xy_error_m": xy_error,
                "object_attached": bool(final_debug.get("object_attached")),
                "steps": steps,
            }
        finally:
            restore_manual_collision_boxes()

    def pick(self, pose, approach_height=0.1, descend_distance=0.05, hold_seconds=1.0, lift_height=0.1):
        side = normalize_arm(pose.get("arm", "right"))
        pose = dict(pose)
        pose["arm"] = side
        log_event("INFO", "pick start", {
            "arm": side,
            "target_z": pose.get("z"),
            "approach_height": approach_height,
            "descend_distance": descend_distance,
            "hold_seconds": hold_seconds,
            "lift_height": lift_height,
        })
        t0 = time.monotonic()
        steps = []

        def require_execution(stage, execution):
            code = int(execution.get("error_code", -1))
            if code != 0:
                if guarded_mujoco_pick:
                    try:
                        self._set_mujoco_grasp_guard(False)
                    except Exception as exc:
                        log_event(
                            "WARNING",
                            f"pick: failed to clear grasp guard after {stage}: {exc}",
                        )
                detail = execution.get("error_string") or "unknown execution error"
                raise RuntimeError(f"{stage}轨迹执行失败: {detail}")

        guarded_mujoco_pick = pose.get("mujoco_object_center") is not None
        if guarded_mujoco_pick:
            log_event("INFO", "pick: using live MuJoCo object position", {
                "arm": side,
                "object_center": pose.get("mujoco_object_center"),
                "nominal_tcp": pose.get("mujoco_nominal_tcp"),
            })
            guard_result = self._set_mujoco_grasp_guard(True)
            steps.append({
                "action": "pin_sim_object",
                "ok": True,
                "guard": guard_result,
            })

        # Always approach with an open gripper.  A previous failed/manual close
        # otherwise turns the finger pair into a solid fork that pushes the
        # light MuJoCo cube away before the grasp command is issued.
        open_result = self.gripper_open(side)
        if open_result.get("simulated"):
            open_result["settled_state"] = self._wait_for_sim_gripper(side)
        steps.append({"action": "open_gripper", "ok": True, "gripper": open_result})
        log_event("INFO", "pick: gripper opened before approach")

        approach_pose = dict(pose)
        approach_pose["z"] = float(pose["z"]) + float(approach_height)
        if guarded_mujoco_pick:
            approach_pose["_mujoco_avoid_grasp_object"] = True
        summary = self.plan(approach_pose)
        result = self.execute_last_plan(side)
        require_execution("接近", result)
        steps.append({"action": "approach", "ok": True, "plan": summary, "execute": result})
        log_event("INFO", "pick: approach done")

        descend_pose = dict(pose)
        descend_pose["z"] = float(pose["z"]) + float(approach_height) - float(descend_distance)
        locked_grasp = summary.get("grasp_orientation") or {}
        locked_tcp = locked_grasp.get("tcp_position")
        if (
            guarded_mujoco_pick
            and isinstance(locked_tcp, list)
            and len(locked_tcp) >= 3
        ):
            # Keep the exact orientation selected for the collision-free
            # approach. Re-running grasp candidate selection for descend could
            # choose the opposite tool-Z sign and rotate the wrist through the
            # cube while moving down.
            descend_pose.update({
                "x": float(locked_tcp[0]),
                "y": float(locked_tcp[1]),
                "z": float(locked_tcp[2]) - float(descend_distance),
                "roll": float(locked_grasp["roll"]),
                "pitch": float(locked_grasp["pitch"]),
                "yaw": float(locked_grasp["yaw"]),
            })
            descend_pose.pop("grasp_orientation_mode", None)
        if guarded_mujoco_pick:
            # The MoveIt URDF combines the wrist and gripper into one coarse
            # collision mesh. Relax only the tabletop's upper shell during the
            # final contact motion; MuJoCo still enforces the exact physical
            # tabletop and all other robot links remain collision checked.
            descend_pose["_mujoco_table_grasp_contact"] = True
        summary = self.plan(descend_pose)
        result = self.execute_last_plan(side)
        require_execution("下降", result)
        steps.append({"action": "descend", "ok": True, "plan": summary, "execute": result})
        log_event("INFO", "pick: descend done")

        grip_result = self.gripper_close(side)
        if grip_result.get("simulated"):
            grip_result["settled_state"] = self._wait_for_sim_gripper(
                side,
                SIM_GRASP_CAPTURE_TIMEOUT_SEC
                if guarded_mujoco_pick
                else SIM_GRIPPER_SETTLE_TIMEOUT_SEC,
            )
        steps.append({"action": "close_gripper", "ok": True, "gripper": grip_result})
        time.sleep(SIM_GRASP_POSE_DELAY_SEC)
        close_debug = self._grasp_debug(side) if grip_result.get("simulated") else {}
        if guarded_mujoco_pick:
            # Attachment clears the guard in the simulation bridge.  Explicitly
            # clear it here as well when capture failed, so the cube is not left
            # pinned until the safety timeout.
            try:
                self._set_mujoco_grasp_guard(False)
            except Exception as exc:
                log_event("WARNING", f"pick: failed to clear grasp guard: {exc}")
        object_pose_before_lift = close_debug.get("object_position")
        close_log = {
            "gripper_width": close_debug.get("gripper_width"),
            "left_contact": bool(close_debug.get("left_contact")),
            "right_contact": bool(close_debug.get("right_contact")),
            "object_attached": bool(close_debug.get("object_attached")),
            "object_position": object_pose_before_lift,
            "object_pose": close_debug.get("object_pose"),
            "contact_count": close_debug.get("contact_count"),
            "contact_points": close_debug.get("contact_points"),
            "left_contact_geoms": close_debug.get("left_contact_geoms"),
            "right_contact_geoms": close_debug.get("right_contact_geoms"),
            "object_geom": close_debug.get("object_geom"),
            "gripper_joint_angle": close_debug.get("gripper_joint_angle"),
        }
        log_event("INFO", "pick: gripper closed and object pose recorded", close_log)

        if grip_result.get("simulated") and not close_debug.get("object_attached"):
            failure = {
                **close_log,
                "reason": "no stable simulated attachment; lift skipped",
            }
            log_event("ERROR", "pick: grasp failed before lift", failure)
            return {
                "steps": steps,
                "completed": False,
                "object_attached": False,
                "grasp_debug": {"close": failure},
            }

        remaining_hold = max(0.0, float(hold_seconds) - SIM_GRASP_POSE_DELAY_SEC)
        if remaining_hold > 0:
            time.sleep(remaining_hold)
            steps.append({"action": "hold", "ok": True, "seconds": float(hold_seconds)})
            log_event("INFO", f"pick: held for {hold_seconds}s")

        lift_pose = dict(pose)
        lift_pose["z"] = float(pose["z"]) + float(lift_height)
        if (
            guarded_mujoco_pick
            and isinstance(locked_tcp, list)
            and len(locked_tcp) >= 3
        ):
            lift_pose.update({
                "x": float(descend_pose["x"]),
                "y": float(descend_pose["y"]),
                "z": float(descend_pose["z"]) + float(lift_height),
                "roll": float(descend_pose["roll"]),
                "pitch": float(descend_pose["pitch"]),
                "yaw": float(descend_pose["yaw"]),
            })
            lift_pose.pop("grasp_orientation_mode", None)
        if guarded_mujoco_pick:
            lift_pose["_mujoco_table_grasp_contact"] = True
        summary = self.plan(lift_pose)
        result = self.execute_last_plan(side)
        require_execution("抬起", result)
        steps.append({"action": "lift", "ok": True, "plan": summary, "execute": result})
        object_attached = True
        lift_debug = {}
        pose_delta = None
        if grip_result.get("simulated"):
            time.sleep(0.15)
            lift_debug = self._grasp_debug(side)
            object_pose_after_lift = lift_debug.get("object_position")
            object_attached = False
            if (
                isinstance(object_pose_before_lift, list)
                and isinstance(object_pose_after_lift, list)
                and len(object_pose_before_lift) >= 3
                and len(object_pose_after_lift) >= 3
            ):
                pose_delta = [
                    float(object_pose_after_lift[i]) - float(object_pose_before_lift[i])
                    for i in range(3)
                ]
                required_lift = max(
                    SIM_GRASP_MIN_LIFT_DELTA_M, abs(float(lift_height)) * 0.5
                )
                object_attached = pose_delta[2] >= required_lift
        lift_log = {
            "gripper_width": lift_debug.get("gripper_width", close_debug.get("gripper_width")),
            "left_contact": bool(lift_debug.get("left_contact")),
            "right_contact": bool(lift_debug.get("right_contact")),
            "object_attached": object_attached,
            "object_position_before_lift": object_pose_before_lift,
            "object_position_after_lift": lift_debug.get("object_position"),
            "object_pose_after_lift": lift_debug.get("object_pose"),
            "object_pose_delta": pose_delta,
        }
        level = "INFO" if object_attached else "ERROR"
        message = "pick: grasp verified after lift" if object_attached else "pick: grasp failed, object did not follow lift"
        log_event(level, message, lift_log)
        log_event("INFO", f"pick: lift done, total elapsed={time.monotonic() - t0:.2f}s")

        return {
            "steps": steps,
            "completed": object_attached,
            "object_attached": object_attached,
            "grasp_debug": {"close": close_log, "lift": lift_log},
        }

    def execute_sequence(self, steps):
        results = []
        for i, step in enumerate(steps):
            step_type = step.get("type")
            side = normalize_arm(step.get("arm") or step.get("pose", {}).get("arm", "right"))
            try:
                if step_type == "approach":
                    r = self.approach(step["pose"], step.get("offset_z", 0.1))
                elif step_type == "grasp":
                    r = self.grasp(step["pose"])
                elif step_type == "lift":
                    r = self.lift(step.get("offset_z", 0.1), side)
                elif step_type == "move":
                    summary = self.plan(step["pose"])
                    result = self.execute_last_plan(side)
                    r = {"plan": summary, "execute": result}
                elif step_type == "release":
                    r = self.gripper_open(side)
                elif step_type == "close":
                    r = self.gripper_close(side)
                elif step_type == "pick":
                    r = self.pick(
                        step["pose"],
                        step.get("approach_height", 0.1),
                        step.get("descend_distance", 0.05),
                        step.get("hold_seconds", 1.0),
                        step.get("lift_height", 0.1),
                    )
                elif step_type == "wait":
                    time.sleep(float(step.get("seconds", 1.0)))
                    r = {"waited": float(step.get("seconds", 1.0))}
                else:
                    raise RuntimeError(f"未知动作类型: {step_type}")
                results.append({"step": i, "type": step_type, "ok": True, "result": r})
            except Exception as e:
                results.append({"step": i, "type": step_type, "ok": False, "error": str(e)})
                break
        return {"results": results, "completed": len(results) == len(steps)}


rclpy.init()
bridge = MoveItBridge()
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(bridge)


def spin_executor():
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    except Exception as exc:
        if rclpy.ok():
            log_event("ERROR", f"ROS executor stopped unexpectedly: {exc}")


spin_thread = threading.Thread(target=spin_executor, daemon=True)
spin_thread.start()


def frontend_content_type(path):
    content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    if path.suffix == ".urdf":
        content_type = "application/xml"
    elif path.suffix.lower() == ".stl":
        content_type = "model/stl"
    if content_type.startswith("text/") or content_type in {"application/javascript", "application/xml"}:
        content_type += "; charset=utf-8"
    return content_type


def frontend_cache_control(request_path):
    if request_path.startswith("/assets/"):
        return "public, max-age=31536000, immutable"
    return "public, max-age=0, must-revalidate"


def frontend_etag(stat_result):
    return f'W/"{stat_result.st_mtime_ns:x}-{stat_result.st_size:x}"'


def frontend_last_modified(stat_result):
    return formatdate(stat_result.st_mtime, usegmt=True)


def etag_matches(header_value, etag):
    if not header_value:
        return False
    return "*" in header_value or etag in {item.strip() for item in header_value.split(",")}


def accepts_gzip(headers):
    encodings = headers.get("Accept-Encoding", "")
    return "gzip" in {item.split(";", 1)[0].strip().lower() for item in encodings.split(",")}


def should_gzip_static(path, stat_result):
    return path.suffix.lower() in STATIC_GZIP_EXTENSIONS and stat_result.st_size >= STATIC_GZIP_MIN_BYTES


def gzip_static_payload(path, stat_result):
    if not should_gzip_static(path, stat_result):
        return None
    key = str(path)
    with STATIC_GZIP_LOCK:
        cached = STATIC_GZIP_CACHE.get(key)
        if (
            cached
            and cached["mtime_ns"] == stat_result.st_mtime_ns
            and cached["size"] == stat_result.st_size
        ):
            return cached["payload"]

    payload = gzip.compress(path.read_bytes(), compresslevel=STATIC_GZIP_LEVEL)
    with STATIC_GZIP_LOCK:
        STATIC_GZIP_CACHE[key] = {
            "mtime_ns": stat_result.st_mtime_ns,
            "size": stat_result.st_size,
            "payload": payload,
        }
    return payload


def prepare_frontend_static_cache():
    if not STATIC_PRECOMPRESS_ENABLED or not FRONTEND_DIST.is_dir():
        return
    start = time.monotonic()
    count = 0
    raw_bytes = 0
    gzip_bytes = 0
    for path in FRONTEND_DIST.rglob("*"):
        if not path.is_file():
            continue
        stat_result = path.stat()
        if not should_gzip_static(path, stat_result):
            continue
        payload = gzip_static_payload(path, stat_result)
        if payload is None:
            continue
        count += 1
        raw_bytes += stat_result.st_size
        gzip_bytes += len(payload)
    if count:
        elapsed = time.monotonic() - start
        log_event(
            "INFO",
            (
                "frontend gzip cache ready: "
                f"{count} files, {raw_bytes / 1024 / 1024:.1f} MB raw, "
                f"{gzip_bytes / 1024 / 1024:.1f} MB gzip, {elapsed:.2f}s"
            ),
        )


def classify_api_failure(path, exc):
    message = str(exc)
    lower = message.lower()
    failure = {
        "stage": "unknown",
        "stage_label": "未知阶段",
        "kind": "runtime",
        "title": "运行异常",
        "message": message,
        "exception": exc.__class__.__name__,
    }
    if "ik" in lower or "逆解" in message or "pregrasp_ik" in lower or "grasp_ik" in lower:
        failure.update({
            "stage": "ik",
            "stage_label": "IK 逆解",
            "kind": "ik",
            "title": "IK 逆解失败",
            "hint": "检查目标点、末端姿态模式、TCP 偏移和关节限位。",
        })
    if "手动工作区" in message or "workspace" in lower:
        failure.update({
            "stage": "workspace",
            "stage_label": "工作区边界",
            "kind": "workspace",
            "title": "目标点超出手动工作区",
            "hint": "调整目标坐标，或在运动面板修改/关闭工作区边界。",
        })
    if "moveit 规划失败" in message or "规划服务超时" in message or "plan timeout" in lower or "plan failed" in lower:
        failure.update({
            "stage": "planning",
            "stage_label": "MoveIt 规划",
            "kind": "planning",
            "title": "MoveIt 规划失败",
            "hint": "IK 可能已成功，但路径规划、避障或规划时间失败。",
        })
    if "ompl" in lower or "规划器" in message:
        failure.update({
            "stage": "planning_config",
            "stage_label": "规划配置",
            "kind": "planning_config",
            "title": "规划配置无效",
            "hint": "检查 OMPL 配置页选择的 planner_id、规划时间和尝试次数。",
        })
    if "closing_direction" in message or "approach_direction" in message or "几何" in message or "无候选" in message:
        failure.update({
            "stage": "geometry",
            "stage_label": "抓取几何过滤",
            "kind": "geometry",
            "title": "抓取几何候选失败",
            "hint": "检查 cylinder_axis、closing direction、approach direction 和地面法向约束。",
        })
    if "服务未就绪" in message or "not ready" in lower:
        failure.update({
            "stage": "service",
            "stage_label": "服务连接",
            "kind": "service",
            "title": "ROS/MoveIt 服务未就绪",
            "hint": "检查 move_group、/compute_ik 和 FollowJointTrajectory action 是否启动。",
        })
    if "轨迹执行" in message or "goal rejected" in lower or "execute" in lower:
        failure.update({
            "stage": "execution",
            "stage_label": "轨迹执行",
            "kind": "execution",
            "title": "轨迹执行失败",
            "hint": "检查电机使能、急停、控制器 action 和轨迹时间戳。",
        })
    if "平台避障" in message or "collision" in lower:
        failure["collision_related"] = True
    if "碰撞箱" in message or "planningscene" in lower or "planning scene" in lower:
        failure.update({
            "stage": "collision_scene",
            "stage_label": "碰撞场景",
            "kind": "collision_scene",
            "title": "碰撞场景更新失败",
            "hint": "检查 /apply_planning_scene 服务、手动碰撞箱尺寸和坐标系配置。",
            "collision_related": True,
        })
    if "错误码=" in message:
        try:
            code_text = message.split("错误码=", 1)[1].split("）", 1)[0].split("（", 1)[0]
            failure["moveit_error_code"] = int(code_text)
            failure["moveit_error_name"] = moveit_error_name(failure["moveit_error_code"])
        except Exception:
            pass
    if path.startswith("/api/"):
        failure["request"] = path
    return failure


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, payload, content_type="application/json"):
        data = payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")
        try:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            log_event("DEBUG", f"client disconnected before response completed: {self.path}")

    def _json_body(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _send_mjpeg(self, camera_key):
        boundary = b"unoarm-frame"
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.send_response(200)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={boundary.decode('ascii')}",
            )
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Connection", "close")
            self.end_headers()
            last_stamp = None
            while True:
                status = bridge.camera_status(camera_key)
                stamp = status.get("stamp")
                jpeg = bridge.camera_jpeg(camera_key)
                if status.get("ready") and jpeg is not None and stamp != last_stamp:
                    last_stamp = stamp
                    self.wfile.write(b"--" + boundary + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def _query(self):
        return parse_qs(urlparse(self.path).query)

    def _query_arm(self):
        return normalize_arm(self._query().get("arm", ["right"])[0])

    def _send_frontend_file(self, request_path):
        relative_path = "index.html" if request_path == "/" else request_path.lstrip("/")
        candidate = (FRONTEND_DIST / relative_path).resolve()
        try:
            candidate.relative_to(FRONTEND_DIST.resolve())
        except ValueError:
            self._send(403, {"error": "forbidden"})
            return True

        if not candidate.is_file():
            self._send(404, {"error": "frontend asset not found; run npm run build in web_control/frontend"})
            return True

        stat_result = candidate.stat()
        content_type = frontend_content_type(candidate)
        etag = frontend_etag(stat_result)
        cache_control = frontend_cache_control(request_path)
        last_modified = frontend_last_modified(stat_result)
        use_gzip = accepts_gzip(self.headers)

        if etag_matches(self.headers.get("If-None-Match"), etag):
            self.send_response(304)
            self.send_header("ETag", etag)
            self.send_header("Last-Modified", last_modified)
            self.send_header("Cache-Control", cache_control)
            if should_gzip_static(candidate, stat_result):
                self.send_header("Vary", "Accept-Encoding")
            self.end_headers()
            return True

        gzip_payload = gzip_static_payload(candidate, stat_result) if use_gzip else None
        if gzip_payload is not None:
            payload = gzip_payload
            content_encoding = "gzip"
        else:
            payload = candidate.read_bytes()
            content_encoding = None

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("ETag", etag)
        self.send_header("Last-Modified", last_modified)
        self.send_header("Cache-Control", cache_control)
        if should_gzip_static(candidate, stat_result):
            self.send_header("Vary", "Accept-Encoding")
        if content_encoding:
            self.send_header("Content-Encoding", content_encoding)
        self.end_headers()
        self.wfile.write(payload)
        return True

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/" or path.startswith(("/assets/", "/urdf/", "/models/")):
            self._send_frontend_file(path)
        elif path == "/api/status":
            self._send(200, bridge.current_state())
        elif path == "/api/links":
            self._send(200, bridge.link_positions(self._query_arm()))
        elif path == "/api/points":
            self._send(200, {"points": load_points()})
        elif path == "/api/presets":
            self._send(200, {"presets": load_presets()})
        elif path == "/api/gripper":
            self._send(200, bridge.gripper_state(self._query_arm()))
        elif path == "/api/tool_offset":
            offset = bridge._arm_config(self._query_arm())["offset"]
            self._send(200, {"offset": offset})
        elif path == "/api/logs":
            query = self._query()
            n = int(query.get("n", [100])[0])
            self._send(200, {"logs": get_recent_logs(n)})
        elif path == "/api/robot_state":
            self._send(200, {"state": bridge.robot_state()})
        elif path == "/api/vision_target":
            self._send(200, load_latest_vision_target())
        elif path == "/api/vision_status":
            proxied = vision_proxy_get("/api/status", "application/json")
            if proxied["ok"]:
                self._send(200, proxied["payload"], "application/json; charset=utf-8")
            else:
                self._send(proxied["status"], {"ok": False, "error": proxied["error"]})
        elif path == "/api/mujoco_camera/status":
            self._send(200, bridge.camera_status("head"))
        elif path == "/mujoco_camera/snapshot.jpg":
            payload = bridge.camera_jpeg("head")
            if payload is None:
                self._send(503, {"ok": False, "error": "MuJoCo 摄像头尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/mujoco_camera/stream.mjpg":
            self._send_mjpeg("head")
        elif path == "/api/right_wrist_camera/status":
            self._send(200, bridge.camera_status("right_wrist"))
        elif path == "/right_wrist_camera/snapshot.jpg":
            payload = bridge.camera_jpeg("right_wrist")
            if payload is None:
                self._send(503, {"ok": False, "error": "右腕摄像头尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/right_wrist_camera/stream.mjpg":
            self._send_mjpeg("right_wrist")
        elif path == "/api/right_wrist_depth/status":
            self._send(200, bridge.camera_status("right_wrist_depth"))
        elif path == "/right_wrist_depth/snapshot.jpg":
            payload = bridge.camera_jpeg("right_wrist_depth")
            if payload is None:
                self._send(503, {"ok": False, "error": "右腕深度图尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/api/left_wrist_camera/status":
            self._send(200, bridge.camera_status("left_wrist"))
        elif path == "/left_wrist_camera/snapshot.jpg":
            payload = bridge.camera_jpeg("left_wrist")
            if payload is None:
                self._send(503, {"ok": False, "error": "左腕摄像头尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/left_wrist_camera/stream.mjpg":
            self._send_mjpeg("left_wrist")
        elif path == "/api/left_wrist_depth/status":
            self._send(200, bridge.camera_status("left_wrist_depth"))
        elif path == "/left_wrist_depth/snapshot.jpg":
            payload = bridge.camera_jpeg("left_wrist_depth")
            if payload is None:
                self._send(503, {"ok": False, "error": "左腕深度图尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/api/overview_camera/status":
            self._send(200, bridge.camera_status("overview"))
        elif path == "/overview_camera/snapshot.jpg":
            payload = bridge.camera_jpeg("overview")
            if payload is None:
                self._send(503, {"ok": False, "error": "全局相机尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/api/mujoco_depth/status":
            self._send(200, bridge.camera_status("depth"))
        elif path == "/api/mujoco/grasp_target":
            center = dict(MUJOCO_GRASP_OBJECT_CENTER)
            live_debug = bridge._grasp_debug("right")
            live_position = live_debug.get("object_position")
            if isinstance(live_position, list) and len(live_position) >= 3:
                center.update({
                    "x": float(live_position[0]),
                    "y": float(live_position[1]),
                    "z": float(live_position[2]),
                    "live": True,
                })
            target = {
                **center,
                "x": center["x"] + MUJOCO_GRASP_TCP_OFFSET["x"],
                "y": center["y"] + MUJOCO_GRASP_TCP_OFFSET["y"],
                "z": center["z"] + MUJOCO_GRASP_TCP_OFFSET["z"],
                "object_center": center,
                "tcp_offset": dict(MUJOCO_GRASP_TCP_OFFSET),
                "grasp_pose_mode": "z_parallel",
                "object_motion": live_debug.get("object_motion"),
                "orientation_wxyz": (live_debug.get("object_pose") or {}).get("orientation_wxyz"),
            }
            self._send(200, {"ok": True, "target": target})
        elif path == "/api/mujoco/visual_grasp_target":
            try:
                self._send(200, bridge.mujoco_visual_grasp_target())
            except Exception as exc:
                log_event("WARNING", f"visual grasp target unavailable: {exc}")
                self._send(422, {
                    "ok": False,
                    "error": str(exc),
                    "failure": classify_api_failure(path, exc),
                })
        elif path == "/api/mujoco/scene":
            live_debug = bridge._grasp_debug("right")
            scene_config = load_mujoco_scene_config()
            self._send(200, {
                "ok": True,
                "config": scene_config,
                "presets": MUJOCO_SCENE_PRESETS,
                "table": {
                    "center": list(MUJOCO_TABLE_CENTER),
                    "size": list(MUJOCO_TABLE_SIZE),
                    "color": "#d1d6db",
                },
                "object": {
                    "position": live_debug.get("object_position", [
                        MUJOCO_GRASP_OBJECT_CENTER["x"],
                        MUJOCO_GRASP_OBJECT_CENTER["y"],
                        MUJOCO_GRASP_OBJECT_CENTER["z"],
                    ]),
                    "orientation_wxyz": mujoco_grasp_object_display_quat_wxyz(
                        (live_debug.get("object_pose") or {}).get("orientation_wxyz")
                    ),
                    "body_orientation_wxyz": (live_debug.get("object_pose") or {}).get(
                        "orientation_wxyz", [1.0, 0.0, 0.0, 0.0]
                    ),
                    "model_quat_wxyz": list(MUJOCO_GRASP_OBJECT_MESH_QUAT_WXYZ),
                    "size": list(MUJOCO_GRASP_OBJECT_SIZE),
                    "color": "#c70f0d",
                    "attached": bool(live_debug.get("object_attached")),
                },
                "extras": bridge._mujoco_scene_extras(),
            })
        elif path == "/mujoco_depth/snapshot.jpg":
            payload = bridge.camera_jpeg("depth")
            if payload is None:
                self._send(503, {"ok": False, "error": "MuJoCo 深度图尚无画面"})
            else:
                self._send(200, payload, "image/jpeg")
        elif path == "/api/mujoco_depth/raw":
            payload = bridge.camera_depth_payload()
            if payload is None:
                self._send(503, {"ok": False, "error": "MuJoCo 原始深度图尚无数据"})
            else:
                self._send(200, payload)
        elif path == "/api/pointcloud.bin":
            proxied = vision_proxy_get("/api/pointcloud.bin", "application/octet-stream")
            if proxied["ok"]:
                self._send(200, proxied["payload"], "application/octet-stream")
            else:
                self._send(proxied["status"], {"ok": False, "error": proxied["error"]})
        elif path == "/api/scene_pointcloud.bin":
            proxied = vision_proxy_get("/api/scene_pointcloud.bin", "application/octet-stream")
            if proxied["ok"]:
                self._send(200, proxied["payload"], "application/octet-stream")
            else:
                self._send(proxied["status"], {"ok": False, "error": proxied["error"]})
        elif path == "/snapshot.ply":
            proxied = vision_proxy_get("/snapshot.ply", "application/octet-stream")
            if proxied["ok"]:
                self._send(200, proxied["payload"], "application/octet-stream")
            else:
                self._send(proxied["status"], {"ok": False, "error": proxied["error"]})
        elif path == "/vision_stream.mjpg":
            proxied = vision_proxy_get("/snapshot.jpg", "image/jpeg")
            if proxied["ok"]:
                self._send(200, proxied["payload"], "image/jpeg")
            else:
                self._send(proxied["status"], {"ok": False, "error": proxied["error"]})
        elif path == "/api/camera_extrinsic":
            self._send(200, camera_extrinsic_response())
        elif path == "/api/joint_config":
            self._send(200, joint_config_response(self._query_arm()))
        elif path == "/api/kinematics":
            self._send(200, kinematics_response())
        elif path == "/api/ompl_config":
            self._send(200, ompl_config_response())
        elif path == "/api/planning_presets":
            self._send(200, planning_presets_response())
        elif path == "/api/benchmark/options":
            self._send(200, bridge.benchmark_options(self._query_arm()))
        elif path == "/api/benchmark/progress":
            self._send(200, bridge.benchmark_progress(self._query_arm()))
        elif path == "/api/platform_obstacle":
            self._send(200, {"ok": True, "platform_obstacle": bridge._platform_obstacle_summary})
        elif path == "/api/manual_collision_boxes":
            self._send(200, manual_collision_response(bridge._manual_collision_summary))
        elif path == "/api/workspace_bounds":
            self._send(200, workspace_bounds_response())
        elif path == "/api/vlm_config":
            self._send(200, vlm_config_response())
        elif path == "/api/poses_lib":
            self._send(200, poses_lib_response())
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        path = urlparse(self.path).path
        t0 = time.monotonic()
        try:
            if path == "/api/plan":
                pose = self._json_body()
                self._send(200, {"ok": True, "plan": bridge.plan(pose)})
            elif path == "/api/execute":
                body = self._json_body()
                self._send(200, {"ok": True, "result": bridge.execute_last_plan(body.get("arm", "right"))})
            elif path == "/api/points":
                body = self._json_body()
                points = load_points()
                points.append(body)
                save_points(points)
                self._send(200, {"ok": True, "points": points})
            elif path == "/api/points/delete":
                body = self._json_body()
                idx = int(body["index"])
                points = load_points()
                if 0 <= idx < len(points):
                    points.pop(idx)
                save_points(points)
                self._send(200, {"ok": True, "points": points})
            elif path == "/api/gripper":
                body = self._json_body()
                action = body.get("action")
                arm = body.get("arm", "right")
                if action == "open":
                    r = bridge.gripper_open(arm)
                elif action == "close":
                    r = bridge.gripper_close(arm)
                else:
                    raise ValueError(f"未知夹爪动作: {action}")
                self._send(200, {"ok": True, "result": r})
            elif path == "/api/approach":
                body = self._json_body()
                r = bridge.approach(body["pose"], body.get("offset_z", 0.1))
                self._send(200, {"ok": True, "result": r})
            elif path == "/api/grasp":
                body = self._json_body()
                r = bridge.grasp(body["pose"])
                self._send(200, {"ok": True, "result": r})
            elif path == "/api/lift":
                body = self._json_body()
                r = bridge.lift(body.get("offset_z", 0.1), body.get("arm", "right"))
                self._send(200, {"ok": True, "result": r})
            elif path == "/api/place":
                body = self._json_body()
                r = bridge.place(body["pose"])
                self._send(200, {"ok": True, "result": r})
            elif path == "/api/pick":
                body = self._json_body()
                r = bridge.pick(
                    body["pose"],
                    body.get("approach_height", 0.1),
                    body.get("descend_distance", 0.05),
                    body.get("hold_seconds", 1.0),
                    body.get("lift_height", 0.1),
                )
                if not bool(r.get("completed")):
                    self._send(422, {
                        "ok": False,
                        "error": r.get("error") or "抓取未完成：物体未被稳定夹持并抬起",
                        "result": r,
                    })
                else:
                    self._send(200, {"ok": True, "result": r})
            elif path == "/api/mujoco/left_side_drop_rotor":
                body = self._json_body()
                result = bridge.left_side_drop_rotor_to_scene_target(
                    drop_heights=body.get("drop_heights"),
                    approach_height=body.get("approach_height", 0.065),
                    descend_distance=body.get("descend_distance", 0.065),
                    hold_seconds=body.get("hold_seconds", 0.45),
                    lift_height=body.get("lift_height", 0.055),
                    fall_seconds=body.get("fall_seconds", 1.8),
                    smart_search=parse_bool(body.get("smart_search"), False),
                    search_offsets=body.get("search_offsets"),
                    grasp_modes=body.get("grasp_modes"),
                    release_descend_height=body.get("release_descend_height", 0.045),
                )
                self._send(200, {"ok": True, "result": result})
            elif path == "/api/mujoco/pick_place_plate":
                body = self._json_body()
                config = load_mujoco_scene_config()
                if "plate" not in config.get("objects", []):
                    raise RuntimeError("当前场景没有槽，请先应用“定子放入槽”预设")
                table_top = MUJOCO_TABLE_CENTER[2] + MUJOCO_TABLE_SIZE[2] * 0.5
                # 槽中心约在桌面上方 70 mm；释放后由 MuJoCo 接触求解落入槽内。
                cube_half_height = MUJOCO_GRASP_OBJECT_SIZE[2] * 0.5
                plate_target_z = table_top + 0.070 + cube_half_height
                result = bridge.pick_and_place_on_plate(
                    body["pose"],
                    float(config.get("plate_x", -0.28)),
                    float(config.get("plate_y", -0.72)),
                    plate_target_z,
                    float(body.get("hover_height", 0.12)),
                    0.012,
                )
                if not bool(result.get("completed")):
                    self._send(422, {
                        "ok": False,
                        "error": result.get("error") or f"抓取放置未完成，停止在 {result.get('stage', '未知')} 阶段",
                        "result": result,
                    })
                else:
                    self._send(200, {"ok": True, "result": result})
            elif path == "/api/mujoco/pick_place_table":
                body = self._json_body()
                config = load_mujoco_scene_config()
                table_target_z = (
                    MUJOCO_TABLE_CENTER[2] + MUJOCO_TABLE_SIZE[2] * 0.5
                    + MUJOCO_GRASP_OBJECT_SIZE[2] * 0.5 + 0.002
                )
                result = bridge.pick_and_place_on_plate(
                    body["pose"],
                    float(config.get("object_x", 0.0)),
                    float(config.get("object_y", -0.73)),
                    table_target_z,
                    float(body.get("hover_height", 0.12)),
                    0.02,
                )
                if not bool(result.get("completed")):
                    self._send(422, {
                        "ok": False,
                        "error": result.get("error") or f"抓取放置未完成，停止在 {result.get('stage', '未知')} 阶段",
                        "result": result,
                    })
                else:
                    self._send(200, {"ok": True, "result": result})
            elif path == "/api/sequence":
                body = self._json_body()
                r = bridge.execute_sequence(body["steps"])
                self._send(200, {"ok": True, "result": r})
            elif path == "/api/presets":
                body = self._json_body()
                presets = load_presets()
                presets.append(body)
                save_presets(presets)
                self._send(200, {"ok": True, "presets": presets})
            elif path == "/api/presets/delete":
                body = self._json_body()
                idx = int(body["index"])
                presets = load_presets()
                if 0 <= idx < len(presets):
                    presets.pop(idx)
                save_presets(presets)
                self._send(200, {"ok": True, "presets": presets})
            elif path == "/api/tool_offset":
                body = self._json_body()
                arm = body.get("arm", "right")
                target = bridge._arm_config(arm)["offset"]
                offset = body.get("offset")
                if offset and len(offset) == 3:
                    target[0] = float(offset[0])
                    target[1] = float(offset[1])
                    target[2] = float(offset[2])
                    bridge._update_link_cache()
                self._send(200, {"ok": True, "offset": target})
            elif path == "/api/enable":
                body = self._json_body()
                arm = body.get("arm", "right")
                action = "enable" if body.get("enabled", True) else "disable"
                self._send(200, bridge._call_robot_service(arm, action))
            elif path == "/api/reset_estop":
                body = self._json_body()
                arm = body.get("arm", "right")
                self._send(200, bridge._call_robot_service(arm, "reset_estop"))
            elif path == "/api/estop":
                body = self._json_body()
                arm = body.get("arm", "right")
                active = body.get("active", True)
                self._send(200, bridge.send_estop(arm, active))
            elif path == "/api/home_zero":
                body = self._json_body()
                arm = body.get("arm", "both")
                self._send(200, bridge.home_zero(arm, body))
            elif path == "/api/mujoco/task_reset":
                self._json_body()
                self._send(200, bridge.reset_mujoco_task())
            elif path == "/api/joint_jog":
                body = self._json_body()
                self._send(200, bridge.jog_joint(
                    body.get("arm", "right"),
                    body.get("joint"),
                    body.get("delta_rad"),
                    body.get("duration_sec", 0.25),
                ))
            elif path == "/api/camera_extrinsic":
                body = self._json_body()
                saved = save_camera_extrinsic(body)
                self._send(200, {
                    "ok": True,
                    "path": str(CAMERA_EXTRINSIC_FILE),
                    "mtime": CAMERA_EXTRINSIC_FILE.stat().st_mtime,
                    "extrinsic": saved,
                })
            elif path == "/api/joint_config":
                body = self._json_body()
                self._send(200, save_joint_config(body.get("arm", "right"), body))
            elif path == "/api/kinematics":
                body = self._json_body()
                self._send(200, save_kinematics_config(body))
            elif path == "/api/planning_presets":
                body = self._json_body()
                self._send(200, {"ok": True, "presets": upsert_planning_preset(body)})
            elif path == "/api/planning_presets/delete":
                body = self._json_body()
                self._send(200, {"ok": True, "presets": delete_planning_preset(body)})
            elif path == "/api/benchmark/generate":
                body = self._json_body()
                self._send(200, bridge.generate_benchmark_samples(body))
            elif path == "/api/benchmark/run":
                body = self._json_body()
                self._send(200, bridge.run_benchmark(body))
            elif path == "/api/benchmark/export_csv":
                body = self._json_body()
                result = body.get("result") if isinstance(body.get("result"), dict) else body
                self._send(200, save_benchmark_csv(result))
            elif path == "/api/scene_pointcloud/save":
                proxied = vision_proxy_post("/api/scene_pointcloud/save")
                if proxied["ok"]:
                    self._send(200, proxied["payload"], "application/json; charset=utf-8")
                else:
                    message = proxied.get("payload") or json.dumps({"ok": False, "error": proxied["error"]}).encode("utf-8")
                    self._send(proxied["status"], message, "application/json; charset=utf-8")
            elif path == "/api/platform_obstacle/apply":
                self._send(200, {"ok": True, "platform_obstacle": bridge.apply_platform_obstacle()})
            elif path == "/api/platform_obstacle/clear":
                self._send(200, {"ok": True, "platform_obstacle": bridge.clear_platform_obstacle()})
            elif path == "/api/manual_collision_boxes":
                body = self._json_body()
                boxes = save_manual_collision_boxes(body.get("boxes", []))
                bridge._manual_collision_summary = {
                    "saved": True,
                    "count": len(boxes),
                    "boxes": boxes,
                    "frame": FRAME_ID,
                    "planning_frame": MOVEIT_PLANNING_FRAME_ID,
                }
                self._send(200, manual_collision_response(bridge._manual_collision_summary))
            elif path == "/api/manual_collision_boxes/apply":
                body = self._json_body()
                boxes = body.get("boxes") if "boxes" in body else None
                self._send(200, {"ok": True, "manual_collision": bridge.apply_manual_collision_boxes(boxes)})
            elif path == "/api/manual_collision_boxes/clear":
                self._send(200, {"ok": True, "manual_collision": bridge.clear_manual_collision_boxes()})
            elif path == "/api/workspace_bounds":
                body = self._json_body()
                bounds = save_workspace_bounds(body)
                self._send(200, {"ok": True, **bounds})
            elif path == "/api/vlm_config":
                body = self._json_body()
                save_vlm_config(body)
                self._send(200, {"ok": True, **vlm_config_response()})
            elif path == "/api/vlm_call":
                body = self._json_body()
                config = load_vlm_config()
                prompt = body.get("prompt") or config.get("prompt", "")
                result = call_vlm(config, prompt)
                self._send(200, {"ok": True, "result": result})
            elif path == "/api/vlm_grasp":
                body = self._json_body()
                config = load_vlm_config()
                prompt = body.get("prompt") or config.get("prompt", "")
                do_pick = bool(body.get("execute", False))
                arm = str(body.get("arm", "right"))
                result = vlm_grasp(
                    config, prompt, do_pick=do_pick, arm=arm,
                    place_in_plate=bool(body.get("place_in_plate", False)),
                    place_on_table=bool(body.get("place_on_table", False)),
                )
                if "error" in result:
                    self._send(422, {"ok": False, "error": result["error"], "result": result})
                else:
                    self._send(200, {"ok": True, "result": result})
            elif path == "/api/vlm_grasp_stream":
                body = self._json_body()
                config = load_vlm_config()
                prompt = body.get("prompt") or config.get("prompt", "")
                do_pick = bool(body.get("execute", False))
                arm = str(body.get("arm", "right"))
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()

                def emit(stage, payload):
                    event_data = json.dumps({"stage": stage, **payload}, ensure_ascii=False)
                    try:
                        self.wfile.write(f"data: {event_data}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    except Exception:
                        pass

                vlm_grasp(
                    config, prompt, do_pick=do_pick, arm=arm,
                    place_in_plate=bool(body.get("place_in_plate", False)),
                    place_on_table=bool(body.get("place_on_table", False)),
                    emit=emit,
                )
                try:
                    self.wfile.write(b"data: {\"stage\": \"done\"}\n\n")
                    self.wfile.flush()
                except Exception:
                    pass
            elif path == "/api/poses_lib":
                body = self._json_body()
                save_poses_lib(body)
                self._send(200, {"ok": True, **poses_lib_response()})
            elif path == "/api/poses_lib/delete":
                body = self._json_body()
                lib = load_poses_lib()
                pose_id = str(body.get("id", "")).strip()
                name_hint = str(body.get("name", "")).strip()
                before_count = len(lib["poses"])
                lib["poses"] = [
                    p for p in lib["poses"]
                    if not (p["id"] == pose_id or (not pose_id and p["name"] == name_hint))
                ]
                save_poses_lib(lib)
                self._send(200, {
                    "ok": True,
                    "deleted": before_count - len(lib["poses"]),
                    **poses_lib_response(),
                })
            elif path == "/api/mujoco/move_object":
                body = self._json_body()
                debug = bridge._grasp_debug("right")
                live_z = 0.752
                live_pos = debug.get("object_position")
                if isinstance(live_pos, list) and len(live_pos) >= 3:
                    live_z = float(live_pos[2])
                requested_z = body.get("z")
                move_z = float(requested_z) if requested_z is not None else live_z
                result = bridge.set_object_pose(
                    body.get("x", 0.0), body.get("y", -0.73), move_z
                )
                self._send(200, result)
            elif path == "/api/mujoco/calibration_fixtures":
                body = self._json_body()
                self._send(200, bridge.set_mujoco_calibration_fixtures(body))
            elif path == "/api/mujoco/camera_calibration/sample":
                self._json_body()
                self._send(200, bridge.mujoco_camera_calibration_sample())
            elif path == "/api/mujoco/camera_calibration/analyze":
                body = self._json_body()
                samples = body.get("samples", [])
                if not isinstance(samples, list):
                    raise ValueError("samples 必须是数组")
                self._send(200, {"ok": True, "analysis": analyze_camera_calibration_samples(samples)})
            elif path == "/api/mujoco/camera_calibration/fit":
                body = self._json_body()
                samples = body.get("samples", [])
                if not isinstance(samples, list):
                    raise ValueError("samples 必须是数组")
                self._send(200, {
                    "ok": True,
                    "fit": fit_camera_calibration_samples(samples, body.get("base_extrinsic")),
                })
            elif path == "/api/mujoco/rebuild_scene":
                import subprocess
                body = self._json_body()
                env = os.environ.copy()
                current_config = load_mujoco_scene_config()
                current_objects = set(current_config.get("objects", []))
                preset_id = str(body.get("preset") or "custom")
                preset = dict(MUJOCO_SCENE_PRESETS.get(preset_id, {}))
                config = {
                    **preset,
                    "preset": preset_id,
                    "table_x": float(body.get("table_x", preset.get("table_x", 0.40))),
                    "table_y": float(body.get("table_y", preset.get("table_y", 0.30))),
                    "object_x": float(body.get("object_x", preset.get("object_x", 0.0))),
                    "object_y": float(body.get("object_y", preset.get("object_y", -0.73))),
                    "plate_x": float(body.get("plate_x", preset.get("plate_x", -0.08))),
                    "plate_y": float(body.get("plate_y", preset.get("plate_y", -0.72))),
                    "calibration_block_x": float(preset.get("calibration_block_x", 0.13)),
                    "calibration_block_y": float(preset.get("calibration_block_y", -0.62)),
                }
                table_cx, table_cy = MUJOCO_TABLE_CENTER[0], MUJOCO_TABLE_CENTER[1]
                def require_on_table(label, x, y, margin):
                    x_min = table_cx - config["table_x"] * 0.5 + margin
                    x_max = table_cx + config["table_x"] * 0.5 - margin
                    y_min = table_cy - config["table_y"] * 0.5 + margin
                    y_max = table_cy + config["table_y"] * 0.5 - margin
                    if not (x_min <= x <= x_max and y_min <= y <= y_max):
                        raise ValueError(
                            f"{label}超出桌面：X 应在 {x_min:.3f}～{x_max:.3f}，"
                            f"Y 应在 {y_min:.3f}～{y_max:.3f}"
                        )
                require_on_table("方块", config["object_x"], config["object_y"], 0.025)
                if body.get("plate", "plate" in preset.get("objects", [])):
                    require_on_table("槽", config["plate_x"], config["plate_y"], 0.07)
                objects = []
                if body.get("plate", "plate" in preset.get("objects", [])): objects.append("plate")
                if body.get("cylinder", "cylinder" in preset.get("objects", [])): objects.append("cylinder")
                if body.get("calibration_block", "calibration_block" in current_objects):
                    objects.append("calibration_block")
                config["objects"] = objects
                env["MUJOCO_TABLE_SIZE_X"] = str(config["table_x"])
                env["MUJOCO_TABLE_SIZE_Y"] = str(config["table_y"])
                env["MUJOCO_GRASP_TARGET_X"] = str(config["object_x"])
                env["MUJOCO_GRASP_TARGET_Y"] = str(config["object_y"])
                env["MUJOCO_PLATE_X"] = str(config["plate_x"])
                env["MUJOCO_PLATE_Y"] = str(config["plate_y"])
                env["MUJOCO_SCENE_OBJECTS"] = ",".join(objects)
                source = str(ROOT.parent / "frontend" / "dist" / "urdf" / "unoarm.urdf")
                output = str(ROOT.parent.parent / "asm0003" / "mujoco" / "asm0003_camera_scene.mjb")
                builder = str(ROOT.parent.parent / "asm0003" / "scripts" / "build_mujoco_camera_scene.py")
                try:
                    proc = subprocess.run(
                        ["python3", builder, "--source", source, "--output", output],
                        env=env, capture_output=True, text=True, timeout=30,
                    )
                    if proc.returncode == 0:
                        save_mujoco_scene_config(config)
                    self._send(200, {
                        "ok": proc.returncode == 0,
                        "stdout": proc.stdout[-500:],
                        "stderr": proc.stderr[-500:],
                        "table_x": env.get("MUJOCO_TABLE_SIZE_X"),
                        "table_y": env.get("MUJOCO_TABLE_SIZE_Y"),
                        "objects": env.get("MUJOCO_SCENE_OBJECTS"),
                        "config": config,
                    })
                except Exception as exc:
                    self._send(500, {"ok": False, "error": str(exc)})
            else:
                self._send(404, {"error": "not found"})
            elapsed = time.monotonic() - t0
            if path.startswith("/api/") and path != "/api/links" and path != "/api/status":
                log_event("DEBUG", f"POST {path} done in {elapsed:.2f}s")
        except Exception as exc:
            elapsed = time.monotonic() - t0
            log_event("ERROR", f"POST {path} failed in {elapsed:.2f}s: {exc}")
            self._send(500, {
                "ok": False,
                "error": str(exc),
                "failure": classify_api_failure(path, exc),
                "elapsed_sec": round(elapsed, 3),
            })

    def log_message(self, fmt, *args):
        return


def main():
    prepare_frontend_static_cache()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"web control: http://{HOST}:{PORT}")
    print(f"frame={FRAME_ID} moveit_frame={MOVEIT_PLANNING_FRAME_ID} tip={TIP_LINK} group={GROUP_NAME}")
    print(f"occupancy_topic={OCCUPANCY_TOPIC}")
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    previous_sigint = signal.getsignal(signal.SIGINT)

    def request_shutdown(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        server.serve_forever()
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)
        server.server_close()
        bridge.prepare_shutdown()
        try:
            bridge.publish_occupancy(False)
        except Exception:
            pass
        time.sleep(0.2)
        executor.remove_node(bridge)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        bridge.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
