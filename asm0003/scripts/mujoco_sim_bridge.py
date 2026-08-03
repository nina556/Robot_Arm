#!/usr/bin/env python3
"""MuJoCo-backed ROS 2 simulation bridge for the UnoArm interfaces."""

import argparse
import importlib
import json
import math
import os
import threading
import time
from dataclasses import dataclass

import mujoco
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory


RIGHT_JOINTS = [f"Right_Joint{i}" for i in range(1, 8)]
LEFT_JOINTS = [f"Left_Joint{i}" for i in range(1, 8)]
RIGHT_GRIPPER_MIMIC = {
    "Right_Gripper_Joint": 1.0,
    "Right_Gripper_Left_Support_Joint": -1.0,
    "Right_Gripper_Left_2_Joint": 1.0,
    "Right_Gripper_Right_2_Joint": -1.0,
    "Right_Gripper_Right_1_Joint": -1.0,
    "Right_Gripper_Right_Support_Joint": -1.0,
}
LEFT_GRIPPER_MIMIC = {
    "Left_Gripper_Joint": 1.0,
    "Left_Gripper_Left_Support_Joint": -1.0,
    "Left_Gripper_Left_2_Joint": 1.0,
    "Left_Gripper_Right_2_Joint": -1.0,
    "Left_Gripper_Right_1_Joint": -1.0,
    "Left_Gripper_Right_Support_Joint": -1.0,
}
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSED_POSITION = float(
    os.environ.get("UNOARM_GRIPPER_CLOSED_POSITION", "-0.91")
)
GRIPPER_SPEED_RAD_PER_SEC = float(os.environ.get("UNOARM_SIM_GRIPPER_SPEED_RAD_PER_SEC", "0.5"))
GRIPPER_UNILATERAL_SPEED_RAD_PER_SEC = max(
    0.01,
    float(os.environ.get("UNOARM_SIM_GRIPPER_UNILATERAL_SPEED_RAD_PER_SEC", "0.10")),
)
GRASP_GUARD_TIMEOUT_SEC = max(
    15.0,
    float(os.environ.get("UNOARM_SIM_GRASP_GUARD_TIMEOUT_SEC", "180.0")),
)
GRASP_OBJECT_HALF_SIZE = np.array([
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_X", "0.04")) * 0.5,
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_Y", "0.04")) * 0.5,
    float(os.environ.get("MUJOCO_GRASP_OBJECT_SIZE_Z", "0.027")) * 0.5,
], dtype=np.float64)


def duration_sec(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def point_positions(point, joint_names, current):
    positions = dict(current)
    for i, name in enumerate(joint_names):
        if i < len(point.positions):
            positions[name] = float(point.positions[i])
    return positions


@dataclass
class ActiveTrajectory:
    joint_names: list
    points: list
    start_positions: dict
    start_time: int
    done: threading.Event
    error: str = ""
    success: bool = False


class MujocoCameraPublisher:
    """Publish synchronized RGB-D frames from one named MuJoCo camera."""

    def __init__(self, node, model, camera_name, width, height, fps, topic,
                 frame_id, depth_enabled=False, depth_topic=None,
                 camera_info_topic=None, max_range=10.0, depth_fps=None):
        self._node = node
        self._model = model
        self._width = max(16, int(width))
        self._height = max(16, int(height))
        self._period_ns = int(1e9 / max(0.1, float(fps)))
        self._last_render_ns = 0
        self._frame_id = frame_id
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = node.create_publisher(Image, topic, camera_qos)
        self._depth_enabled = bool(depth_enabled)
        self._depth_period_ns = int(
            1e9 / max(0.1, float(depth_fps if depth_fps is not None else fps))
        )
        self._last_depth_render_ns = 0
        self._depth_publisher = (
            node.create_publisher(Image, depth_topic, camera_qos)
            if self._depth_enabled and depth_topic else None
        )
        self._camera_info_publisher = (
            node.create_publisher(CameraInfo, camera_info_topic, camera_qos)
            if self._depth_enabled and camera_info_topic else None
        )
        self._camera_info_period_ns = int(1e9)
        self._last_camera_info_ns = 0
        self._max_range = max(0.01, float(max_range))
        self._renderer = None
        self._initialization_attempted = False
        self._camera_name = camera_name
        self._topic = topic
        self._fps = float(fps)
        camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        if camera_id < 0:
            raise RuntimeError(f"MuJoCo model has no camera named {camera_name}")
        self._fovy_deg = float(model.cam_fovy[camera_id])
        self._camera_info = self._make_camera_info()

    def _make_camera_info(self):
        fovy_rad = math.radians(self._fovy_deg)
        focal = self._height / (2.0 * math.tan(fovy_rad / 2.0))
        cx = self._width / 2.0
        cy = self._height / 2.0
        msg = CameraInfo()
        msg.width = self._width
        msg.height = self._height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            focal, 0.0, cx, 0.0,
            0.0, focal, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    @property
    def enabled(self):
        return not self._initialization_attempted or self._renderer is not None

    def _ensure_renderer(self):
        if self._renderer is not None:
            return True
        if self._initialization_attempted:
            return False
        self._initialization_attempted = True
        try:
            # Create the GL context in the timer callback thread that will use it.
            self._renderer = mujoco.Renderer(
                self._model, height=self._height, width=self._width
            )
            self._node.get_logger().info(
                f"MuJoCo named camera ready: {self._camera_name} -> {self._topic} "
                f"({self._width}x{self._height}, {self._fps:g} Hz, "
                f"fovy={self._fovy_deg:g}, depth={self._depth_enabled})"
            )
            return True
        except Exception as exc:
            self._node.get_logger().error(
                f"MuJoCo offscreen camera disabled: {exc}. "
                "Robot simulation will continue without camera images."
            )
            return False

    def maybe_publish(self, data, now_ns):
        if not self.enabled or now_ns - self._last_render_ns < self._period_ns:
            return
        self._last_render_ns = now_ns
        if not self._ensure_renderer():
            return
        try:
            self._renderer.update_scene(data, camera=self._camera_name)
            rgb = self._renderer.render()
            stamp = self._node.get_clock().now().to_msg()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = self._height
            msg.width = self._width
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = self._width * 3
            msg.data = rgb.tobytes()
            self._publisher.publish(msg)
        except Exception as exc:
            self._node.get_logger().error(
                f"MuJoCo camera render failed; disabling camera: {exc}"
            )
            self.close()
            return

        if (
            self._depth_enabled
            and self._depth_publisher is not None
            and now_ns - self._last_depth_render_ns >= self._depth_period_ns
        ):
            self._last_depth_render_ns = now_ns
            try:
                self._renderer.enable_depth_rendering()
                depth = np.asarray(self._renderer.render(), dtype=np.float32)
                valid = np.isfinite(depth) & (depth > 0.0) & (depth < self._max_range)
                depth = np.ascontiguousarray(
                    np.where(valid, depth, np.nan), dtype=np.float32
                )
                depth_msg = Image()
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = self._frame_id
                depth_msg.height = self._height
                depth_msg.width = self._width
                depth_msg.encoding = "32FC1"
                depth_msg.is_bigendian = False
                depth_msg.step = self._width * np.dtype(np.float32).itemsize
                depth_msg.data = depth.tobytes()
                self._depth_publisher.publish(depth_msg)
            except Exception as exc:
                self._depth_enabled = False
                self._node.get_logger().error(
                    f"MuJoCo depth rendering failed; RGB will continue: {exc}"
                )
            finally:
                try:
                    self._renderer.disable_depth_rendering()
                except Exception:
                    pass

        if (
            self._camera_info_publisher is not None
            and now_ns - self._last_camera_info_ns >= self._camera_info_period_ns
        ):
            self._last_camera_info_ns = now_ns
            self._camera_info.header.stamp = stamp
            self._camera_info.header.frame_id = self._frame_id
            self._camera_info_publisher.publish(self._camera_info)

    def close(self):
        renderer, self._renderer = self._renderer, None
        if renderer is not None:
            try:
                renderer.close()
            except Exception:
                pass


class MujocoSimBridge(Node):
    def __init__(self, model_path, publish_rate_hz=100.0, viewer=False,
                 camera_config=None):
        super().__init__("mujoco_sim_bridge")
        self._model = mujoco.MjModel.from_binary_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._lock = threading.RLock()
        self._period = 1.0 / max(1.0, publish_rate_hz)
        self._viewer = None
        self._viewer_requested = bool(viewer)
        self._cameras = []
        self._simulation_stop = threading.Event()
        self._simulation_thread = None
        self._camera_thread = None
        self._camera_data = None
        self._viewer_initialization_done = threading.Event()

        self._joint_qpos = {}
        self._joint_dof = {}
        # Mirror the compiled MuJoCo limits at the ROS command boundary so
        # manual/direct trajectories cannot poison MoveIt's next start state.
        self._joint_limits = {}
        self._positions = {}
        self._velocities = {}
        for i in range(self._model.njnt):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self._joint_qpos[name] = int(self._model.jnt_qposadr[i])
                self._joint_dof[name] = int(self._model.jnt_dofadr[i])
                if name in RIGHT_JOINTS or name in LEFT_JOINTS:
                    self._positions[name] = 0.0
                    self._velocities[name] = 0.0
                    if bool(self._model.jnt_limited[i]):
                        lower, upper = self._model.jnt_range[i]
                        self._joint_limits[name] = (float(lower), float(upper))

        missing = sorted(set(RIGHT_JOINTS + LEFT_JOINTS) - set(self._joint_qpos))
        if missing:
            raise RuntimeError(f"MuJoCo model is missing joints: {missing}")
        self._scene_physics_enabled = "grasp_object_free" in self._joint_qpos
        self._physics_substeps = 1
        if self._scene_physics_enabled:
            # The imported model uses a 1 ms step, which forced ten expensive
            # mj_step calls in every 100 Hz Web-control tick.  Two 5 ms steps
            # are sufficient for the simple table/object scene and preserve
            # real-time physics without starving robot command processing.
            self._physics_substeps = max(
                1, int(os.environ.get("UNOARM_SIM_PHYSICS_SUBSTEPS", "2"))
            )
            self._model.opt.timestep = self._period / self._physics_substeps

        # The scene builder reserves one TCP site per hand.  Fixed
        # URDF links (including *_Gripper_TCP) are fused by MuJoCo, so looking
        # them up as bodies never works.  Keep the compiled site/link ids and
        # save a rigid relative pose when a nearby gripper closes.
        self._grasp_weld_active = False
        self._grasp_weld_side = None
        self._grasp_body_id = -1
        self._grasp_geom_id = -1
        self._grasp_mesh_vertices = None
        self._grasp_free_qpos_adr = -1
        self._grasp_free_dof_adr = -1
        self._grasp_relative_pos = None
        self._grasp_relative_quat = None
        self._grasp_guard_active = False
        self._grasp_guard_qpos = None
        self._grasp_guard_deadline = 0.0
        self._grasp_capture_active = False
        self._grasp_contact_mode = {"right": "none", "left": "none"}
        self._grasp_reset_qpos = None
        self._grasp_initial_qpos = None
        self._grasp_last_safe_qpos = None
        self._last_object_guard_log = 0.0
        self._grasp_eq = {}
        self._grasp_site = {}
        self._grasp_link_body = {}
        self._grasp_finger_bodies = {}
        self._table_geom_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_GEOM, "work_table_robot_guard"
        )
        self._physical_table_geom_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_GEOM, "work_table_collision"
        )
        fixture_names = {
            "plate": ["stator_slot_geom"],
            "calibration_block": ["calibration_block_geom"],
        }
        self._fixture_geoms = {}
        self._fixture_geom_defaults = {}
        for fixture, geom_names in fixture_names.items():
            geom_ids = [
                mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, name)
                for name in geom_names
            ]
            geom_ids = [geom_id for geom_id in geom_ids if geom_id >= 0]
            self._fixture_geoms[fixture] = geom_ids
            for geom_id in geom_ids:
                rgba = self._model.geom_rgba[geom_id].copy()
                rgba[3] = 1.0
                self._fixture_geom_defaults[geom_id] = {
                    "rgba": rgba,
                    "contype": 1,
                    "conaffinity": 1,
                }
        if self._scene_physics_enabled:
            self._grasp_body_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_BODY, "grasp_object"
            )
            self._grasp_geom_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_GEOM, "grasp_object_geom"
            )
            if self._grasp_geom_id >= 0:
                mesh_id = int(self._model.geom_dataid[self._grasp_geom_id])
                if mesh_id >= 0:
                    vertex_adr = int(self._model.mesh_vertadr[mesh_id])
                    vertex_count = int(self._model.mesh_vertnum[mesh_id])
                    self._grasp_mesh_vertices = self._model.mesh_vert[
                        vertex_adr:vertex_adr + vertex_count
                    ].copy()
            free_joint_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, "grasp_object_free"
            )
            if free_joint_id >= 0:
                self._grasp_free_qpos_adr = int(self._model.jnt_qposadr[free_joint_id])
                self._grasp_free_dof_adr = int(self._model.jnt_dofadr[free_joint_id])
                qpos = self._grasp_free_qpos_adr
                self._grasp_reset_qpos = self._data.qpos[qpos:qpos + 7].copy()
                self._grasp_initial_qpos = self._grasp_reset_qpos.copy()
                self._grasp_last_safe_qpos = self._grasp_reset_qpos.copy()
            for side, prefix in (("right", "right"), ("left", "left")):
                self._grasp_eq[side] = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_EQUALITY,
                    f"{prefix}_grasp_weld",
                )
                self._grasp_site[side] = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_SITE,
                    f"{prefix}_grasp_site",
                )
                self._grasp_link_body[side] = mujoco.mj_name2id(
                    self._model, mujoco.mjtObj.mjOBJ_BODY,
                    "Right_Link7" if side == "right" else "Left_Link7",
                )
                label = "Right" if side == "right" else "Left"
                self._grasp_finger_bodies[side] = (
                    mujoco.mj_name2id(
                        self._model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        f"{label}_Gripper_Left_Support_Link",
                    ),
                    mujoco.mj_name2id(
                        self._model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        f"{label}_Gripper_Right_Support_Link",
                    ),
                )

        self._side = {
            "right": {
                "ns": "master2",
                "joints": RIGHT_JOINTS,
                "gripper_mimic": RIGHT_GRIPPER_MIMIC,
                "gripper_joints": [name for name in RIGHT_GRIPPER_MIMIC if name in self._joint_qpos],
                "gripper_state": "open",
                "gripper_position": GRIPPER_OPEN_POSITION,
                "gripper_target": GRIPPER_OPEN_POSITION,
                "gripper_last_command": GRIPPER_OPEN_POSITION,
                "action": "/arm_controller/follow_joint_trajectory",
                "active": None,
                "enabled": True,
                "estop": False,
            },
            "left": {
                "ns": "master0",
                "joints": LEFT_JOINTS,
                "gripper_mimic": LEFT_GRIPPER_MIMIC,
                "gripper_joints": [name for name in LEFT_GRIPPER_MIMIC if name in self._joint_qpos],
                "gripper_state": "open",
                "gripper_position": GRIPPER_OPEN_POSITION,
                "gripper_target": GRIPPER_OPEN_POSITION,
                "gripper_last_command": GRIPPER_OPEN_POSITION,
                "action": "/left_arm_controller/follow_joint_trajectory",
                "active": None,
                "enabled": True,
                "estop": False,
            },
        }
        for cfg in self._side.values():
            cfg["state_joints"] = list(cfg["joints"]) + list(cfg["gripper_joints"])
            if not cfg["gripper_joints"]:
                raise RuntimeError("MuJoCo model is missing gripper joints for the real simulation chain")
            self._apply_gripper_position(cfg, GRIPPER_OPEN_POSITION, 0.0)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._grasp_debug_pub = self.create_publisher(
            String, "/mujoco/grasp_debug", 10
        )
        self._pending_object_pose = None
        self.create_subscription(
            String, "/mujoco/set_object_pose", self._on_set_object_pose, 10
        )
        self._grasp_guard_srv = self.create_service(
            SetBool, "/mujoco/grasp_guard", self._on_grasp_guard_service
        )
        self._task_reset_srv = self.create_service(
            Trigger, "/mujoco/task_reset", self._on_task_reset
        )
        self._fixture_visibility_srvs = {
            fixture: self.create_service(
                SetBool,
                f"/mujoco/{fixture}_visible",
                lambda request, response, name=fixture: self._on_fixture_visibility(
                    name, request, response
                ),
            )
            for fixture in self._fixture_geoms
        }
        for side, cfg in self._side.items():
            ns = cfg["ns"]
            cfg["state_pub"] = self.create_publisher(JointState, f"/{ns}/joint_states", 10)
            cfg["gripper_state_pub"] = self.create_publisher(JointState, f"/{ns}/gripper_joint_states", 10)
            cfg["enabled_pub"] = self.create_publisher(Bool, f"/{ns}/motors_enabled", state_qos)
            cfg["estop_pub"] = self.create_publisher(Bool, f"/{ns}/emergency_stop_state", state_qos)
            cfg["estop_pub_compat"] = self.create_publisher(Bool, f"/{ns}/estop_active", state_qos)
            cfg["trajectory_sub"] = self.create_subscription(
                JointTrajectory,
                f"/{ns}/joint_trajectory",
                lambda msg, s=side: self._on_topic_trajectory(s, msg),
                10,
            )
            cfg["estop_sub"] = self.create_subscription(
                Bool,
                f"/{ns}/emergency_stop",
                lambda msg, s=side: self._on_estop(s, msg),
                10,
            )
            cfg["enable_srv"] = self.create_service(
                SetBool,
                f"/{ns}/enable_motors",
                lambda req, res, s=side: self._on_enable(s, req, res),
            )
            cfg["reset_srv"] = self.create_service(
                Trigger,
                f"/{ns}/reset_emergency_stop",
                lambda req, res, s=side: self._on_reset_estop(s, req, res),
            )
            cfg["gripper_srv"] = self.create_service(
                SetBool,
                f"/{ns}/sim_gripper",
                lambda req, res, s=side: self._on_sim_gripper(s, req, res),
            )
            cfg["action_server"] = ActionServer(
                self,
                FollowJointTrajectory,
                cfg["action"],
                lambda goal_handle, s=side: self._execute_action(s, goal_handle),
                goal_callback=lambda goal_request, s=side: self._goal_callback(s, goal_request),
            )

        for config in camera_config or []:
            if config.get("enabled", True):
                self._cameras.append(MujocoCameraPublisher(
                    self,
                    self._model,
                    config["name"],
                    config["width"],
                    config["height"],
                    config["fps"],
                    config["topic"],
                    config["frame_id"],
                    config.get("depth_enabled", False),
                    config.get("depth_topic"),
                    config.get("camera_info_topic"),
                    config.get("max_range", 10.0),
                    config.get("depth_fps"),
                ))

        self._simulation_thread = threading.Thread(
            target=self._simulation_loop,
            name="mujoco_simulation_and_viewer",
            daemon=True,
        )
        self._simulation_thread.start()

        if self._cameras:
            # Rendering is deliberately kept out of the 100 Hz control loop.
            # A private MjData snapshot lets slow GPU readback never hold the
            # robot-state lock or delay trajectory interpolation.
            self._camera_data = mujoco.MjData(self._model)
            self._camera_thread = threading.Thread(
                target=self._camera_loop,
                name="mujoco_camera_render",
                daemon=True,
            )
            self._camera_thread.start()
        self.get_logger().info(f"Loaded MuJoCo model: {model_path}")
        self.get_logger().info("Simulation bridge is exposing the real robot ROS interfaces")
        self.get_logger().info("Simulation gripper services: /master2/sim_gripper, /master0/sim_gripper")
        self.get_logger().info(
            "Simulation gripper joints: "
            f"right={self._side['right']['gripper_joints']} "
            f"left={self._side['left']['gripper_joints']}"
        )

    def _camera_loop(self):
        """Own every offscreen renderer without blocking robot control."""
        # GLFW installs a process-global X11 error handler while creating the
        # passive viewer.  Initializing an offscreen GLX renderer concurrently
        # can abort inside GLFW, so the two initializations must be serialized.
        while not self._viewer_initialization_done.wait(timeout=0.1):
            if self._simulation_stop.is_set() or not rclpy.ok():
                return
        render_period = min(camera._period_ns for camera in self._cameras) * 1e-9
        render_period = max(0.001, render_period)
        next_render = time.monotonic()
        try:
            while rclpy.ok() and not self._simulation_stop.is_set():
                with self._lock:
                    self._camera_data.time = self._data.time
                    self._camera_data.qpos[:] = self._data.qpos
                    self._camera_data.qvel[:] = self._data.qvel
                    if self._model.na:
                        self._camera_data.act[:] = self._data.act
                mujoco.mj_forward(self._model, self._camera_data)
                # Rendering cadence must not depend on /clock. MuJoCo time can
                # move backwards during resets, while monotonic time cannot.
                now = time.monotonic_ns()
                for camera in self._cameras:
                    camera.maybe_publish(self._camera_data, now)
                next_render += render_period
                delay = next_render - time.monotonic()
                if delay > 0.0:
                    self._simulation_stop.wait(delay)
                else:
                    # Do not build an ever-growing backlog when rendering is
                    # slower than the requested frame rate.
                    next_render = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"MuJoCo camera loop stopped: {exc}")
        finally:
            for camera in self._cameras:
                camera.close()

    def _simulation_loop(self):
        """Own MuJoCo stepping and the optional passive viewer."""
        try:
            if self._viewer_requested:
                mujoco_viewer = importlib.import_module("mujoco.viewer")
                self._viewer = mujoco_viewer.launch_passive(self._model, self._data)
                self.get_logger().info("MuJoCo passive viewer started")
        except Exception as exc:
            self.get_logger().error(
                f"Could not start MuJoCo viewer; simulation will continue: {exc}"
            )
        finally:
            self._viewer_initialization_done.set()

        next_tick = time.monotonic()
        try:
            while rclpy.ok() and not self._simulation_stop.is_set():
                self._tick()
                next_tick += self._period
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    self._simulation_stop.wait(delay)
                elif delay < -self._period:
                    next_tick = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"MuJoCo simulation loop stopped: {exc}")
        finally:
            if self._viewer is not None:
                try:
                    self._viewer.close()
                except Exception:
                    pass
                self._viewer = None

    def stop_simulation(self):
        self._simulation_stop.set()
        for thread in (self._simulation_thread, self._camera_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=5.0)

    def _goal_callback(self, side, goal_request):
        with self._lock:
            cfg = self._side[side]
            if cfg["estop"] or not cfg["enabled"]:
                return GoalResponse.REJECT
            ok, error = self._validate_trajectory(side, goal_request.trajectory)
            if not ok:
                self.get_logger().error(f"{side} action rejected: {error}")
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def _execute_action(self, side, goal_handle):
        trajectory = goal_handle.request.trajectory
        active = self._set_active_trajectory(side, trajectory)
        while rclpy.ok() and not active.done.wait(timeout=0.05):
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(trajectory.joint_names)
            for name in feedback.joint_names:
                feedback.actual.positions.append(self._positions.get(name, 0.0))
            goal_handle.publish_feedback(feedback)

        result = FollowJointTrajectory.Result()
        if active.success:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "sim trajectory completed"
        else:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = active.error or "sim trajectory aborted"
        return result

    def _on_topic_trajectory(self, side, msg):
        with self._lock:
            cfg = self._side[side]
            if cfg["estop"] or not cfg["enabled"]:
                self.get_logger().error(f"{side} topic trajectory ignored: disabled or estop")
                return
            ok, error = self._validate_trajectory(side, msg)
            if not ok:
                self.get_logger().error(f"{side} topic trajectory ignored: {error}")
                return
            self._set_active_trajectory(side, msg)

    def _on_estop(self, side, msg):
        with self._lock:
            cfg = self._side[side]
            cfg["estop"] = bool(msg.data)
            if cfg["estop"]:
                cfg["enabled"] = False
                self._abort_active(side, "emergency stop")

    def _on_enable(self, side, request, response):
        with self._lock:
            cfg = self._side[side]
            if request.data and cfg["estop"]:
                response.success = False
                response.message = "sim estop is active; reset first"
            else:
                cfg["enabled"] = bool(request.data)
                response.success = True
                response.message = f"sim motors {'enabled' if cfg['enabled'] else 'disabled'}"
                if not cfg["enabled"]:
                    self._abort_active(side, "motors disabled")
            return response

    def _on_reset_estop(self, side, _request, response):
        with self._lock:
            self._side[side]["estop"] = False
            response.success = True
            response.message = "sim emergency stop reset; motors remain disabled"
            return response

    def _on_task_reset(self, _request, response):
        """Atomically restore the simulation state without planning motion."""
        with self._lock:
            for side, cfg in self._side.items():
                self._abort_active(side, "MuJoCo task reset")
                for name in cfg["joints"]:
                    self._positions[name] = 0.0
                    self._velocities[name] = 0.0
                cfg["gripper_position"] = GRIPPER_OPEN_POSITION
                cfg["gripper_target"] = GRIPPER_OPEN_POSITION
                cfg["gripper_last_command"] = GRIPPER_OPEN_POSITION
                cfg["gripper_state"] = "open"
                self._apply_gripper_position(
                    cfg, GRIPPER_OPEN_POSITION, 0.0
                )

            if self._grasp_weld_active:
                slot = self._grasp_eq.get(self._grasp_weld_side, -1)
                if slot >= 0:
                    self._data.eq_active[slot] = 0
            self._grasp_weld_active = False
            self._grasp_weld_side = None
            self._grasp_relative_pos = None
            self._grasp_relative_quat = None
            self._grasp_capture_active = False
            self._grasp_guard_active = False
            self._grasp_guard_qpos = None
            self._pending_object_pose = None
            self._grasp_contact_mode = {"right": "none", "left": "none"}

            if (
                self._grasp_free_qpos_adr >= 0
                and self._grasp_initial_qpos is not None
            ):
                qpos = self._grasp_free_qpos_adr
                dof = self._grasp_free_dof_adr
                self._data.qpos[qpos:qpos + 7] = self._grasp_initial_qpos
                self._data.qvel[dof:dof + 6] = 0.0
                self._grasp_reset_qpos = self._grasp_initial_qpos.copy()
                self._grasp_last_safe_qpos = self._grasp_initial_qpos.copy()

            self._write_mujoco_state()
            mujoco.mj_forward(self._model, self._data)
            response.success = True
            response.message = "MuJoCo arms, grippers and object restored"
            self.get_logger().info(response.message)
            return response

    def _on_fixture_visibility(self, fixture, request, response):
        with self._lock:
            geom_ids = self._fixture_geoms.get(fixture, [])
            if not geom_ids:
                response.success = False
                response.message = f"fixture geometry not found: {fixture}"
                return response
            visible = bool(request.data)
            for geom_id in geom_ids:
                defaults = self._fixture_geom_defaults[geom_id]
                self._model.geom_rgba[geom_id] = defaults["rgba"]
                self._model.geom_rgba[geom_id, 3] = 1.0 if visible else 0.0
                self._model.geom_contype[geom_id] = defaults["contype"] if visible else 0
                self._model.geom_conaffinity[geom_id] = defaults["conaffinity"] if visible else 0
            mujoco.mj_forward(self._model, self._data)
            response.success = True
            response.message = f"{fixture} {'visible' if visible else 'hidden'}"
            return response

    def _on_sim_gripper(self, side, request, response):
        closed = bool(request.data)
        state = "closed" if closed else "open"
        target = GRIPPER_CLOSED_POSITION if closed else GRIPPER_OPEN_POSITION
        with self._lock:
            if (
                not closed
                and self._grasp_weld_active
                and self._grasp_weld_side == side
            ):
                self._detach_grasp_object()
            if not closed and not self._grasp_guard_active:
                self._grasp_capture_active = False
                self._grasp_contact_mode[side] = "none"
            cfg = self._side[side]
            if not cfg["gripper_joints"]:
                response.success = False
                response.message = f"sim {side} gripper has no MuJoCo joints in this model"
                return response
            manual_capture = False
            if (
                closed
                and not self._grasp_weld_active
                and not self._grasp_capture_active
            ):
                # Manual Web/ROS close commands do not arm the automatic-pick
                # guard. Reuse the compliant close only when the free object is
                # already near the gripper; never attract a distant object.
                mujoco.mj_forward(self._model, self._data)
                manual_capture = self._object_near_gripper(side)
                if manual_capture:
                    self._grasp_capture_active = True
                    self._grasp_contact_mode[side] = "none"
                    self.get_logger().info(
                        f"sim {side} manual close armed compliant object capture"
                    )
            if abs(float(cfg["gripper_last_command"]) - target) <= 1e-6:
                response.success = True
                response.message = f"sim {side} gripper already targeting {state} at {target:.3f} rad"
                return response
            cfg["gripper_state"] = state
            cfg["gripper_target"] = target
            cfg["gripper_last_command"] = target
            if closed and self._grasp_capture_active:
                # The object is pinned only while the arm approaches. Release
                # it before the fingers move so one pad can gently slide the
                # cube across the table until the opposite pad also contacts.
                self._grasp_guard_active = False
                self._grasp_guard_qpos = None
                self._grasp_contact_mode[side] = "none"
                if not manual_capture:
                    self.get_logger().info(
                        f"sim {side} grasp capture released object for compliant close"
                    )
        response.success = True
        response.message = f"sim {side} gripper target {state} at {target:.3f} rad"
        self.get_logger().info(response.message)
        return response

    def _on_grasp_guard_service(self, request, response):
        """Synchronously pin the free cube before automatic-pick motion."""
        with self._lock:
            if (
                bool(request.data)
                and not self._grasp_weld_active
                and self._grasp_free_qpos_adr >= 0
            ):
                qpos = self._grasp_free_qpos_adr
                self._grasp_guard_qpos = self._data.qpos[qpos:qpos + 7].copy()
                self._grasp_guard_active = True
                self._grasp_capture_active = True
                self._grasp_guard_deadline = time.monotonic() + GRASP_GUARD_TIMEOUT_SEC
                self.get_logger().info(
                    "Grasp guard pinned the free object for automatic pick"
                )
                response.success = True
                response.message = "free grasp object pinned"
            elif not bool(request.data):
                self._grasp_guard_active = False
                self._grasp_guard_qpos = None
                self._grasp_capture_active = False
                response.success = True
                response.message = "grasp guard released"
            else:
                response.success = False
                response.message = "grasp object cannot be pinned in current state"
        return response

    def _validate_trajectory(self, side, trajectory):
        if not trajectory.joint_names:
            return False, "trajectory has no joint names"
        if not trajectory.points:
            return False, "trajectory has no points"
        allowed = set(self._side[side]["joints"])
        unknown = sorted(set(trajectory.joint_names) - allowed)
        if unknown:
            return False, f"unknown joints for {side}: {unknown}"
        last_time = -1.0
        for index, point in enumerate(trajectory.points):
            if len(point.positions) < len(trajectory.joint_names):
                return False, f"point {index} has fewer positions than joint names"
            t = duration_sec(point.time_from_start)
            if t < last_time:
                return False, "trajectory time_from_start is not monotonic"
            last_time = t
            for value in point.positions:
                if not math.isfinite(float(value)):
                    return False, "trajectory contains non-finite position"
            for joint_index, name in enumerate(trajectory.joint_names):
                limits = self._joint_limits.get(name)
                if limits is None:
                    continue
                value = float(point.positions[joint_index])
                lower, upper = limits
                if value < lower - 1e-6 or value > upper + 1e-6:
                    return False, (
                        f"point {index} joint {name}={value:.6f} is outside "
                        f"[{lower:.6f}, {upper:.6f}]"
                    )
        return True, ""

    def _set_active_trajectory(self, side, trajectory):
        active = ActiveTrajectory(
            joint_names=list(trajectory.joint_names),
            points=list(trajectory.points),
            start_positions=dict(self._positions),
            start_time=self.get_clock().now().nanoseconds,
            done=threading.Event(),
        )
        with self._lock:
            self._abort_active(side, "replaced by newer trajectory")
            self._side[side]["active"] = active
        return active

    def _abort_active(self, side, reason):
        active = self._side[side].get("active")
        if active and not active.done.is_set():
            active.success = False
            active.error = reason
            active.done.set()
        self._side[side]["active"] = None

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        with self._lock:
            safe_positions = dict(self._positions)
            safe_velocities = dict(self._velocities)
            safe_grippers = {
                side: {
                    "position": float(cfg["gripper_position"]),
                    "target": float(cfg["gripper_target"]),
                    "state": cfg["gripper_state"],
                }
                for side, cfg in self._side.items()
            }
            for side in ("right", "left"):
                self._advance_side(side, now)
                self._advance_gripper(side)

            # Arm joints are kinematic inputs, so MuJoCo contact forces cannot
            # stop a commanded link from entering the table.  Evaluate the new
            # command pose first and roll each colliding arm back to its last
            # safe state before advancing physics.
            self._write_mujoco_state()
            mujoco.mj_forward(self._model, self._data)
            table_collisions = self._robot_table_collision_sides()
            for side, bodies in table_collisions.items():
                cfg = self._side[side]
                for name in cfg["state_joints"]:
                    if name in safe_positions:
                        self._positions[name] = safe_positions[name]
                        self._velocities[name] = safe_velocities.get(name, 0.0)
                gripper = safe_grippers[side]
                cfg["gripper_position"] = gripper["position"]
                cfg["gripper_target"] = gripper["position"]
                cfg["gripper_state"] = gripper["state"]
                reason = (
                    "MuJoCo table collision guard blocked motion: "
                    + ", ".join(sorted(bodies))
                )
                self._abort_active(side, reason)
                self.get_logger().error(f"{side} {reason}")
            if table_collisions:
                self._write_mujoco_state()
                mujoco.mj_forward(self._model, self._data)

            # DDS callbacks queue object teleports because MuJoCo state must be
            # mutated by the simulation thread.  Apply the queued pose before
            # stepping physics so task reset takes effect immediately.
            self._apply_pending_object_pose()

            if self._scene_physics_enabled:
                # Keep the arms kinematic while allowing the free grasp object
                # to fall onto and collide with the table/robot geometry.
                for _ in range(self._physics_substeps):
                    self._write_mujoco_state()
                    mujoco.mj_step(self._model, self._data)
                    # Re-apply a pinned/attached object immediately after each
                    # physics substep.  Otherwise a kinematic finger can impart
                    # a very large transient velocity before the next Web tick.
                    self._write_mujoco_state()
                    mujoco.mj_forward(self._model, self._data)
                    if not self._grasp_weld_active:
                        invalid_reason = self._grasp_object_invalid_reason()
                        if invalid_reason:
                            self._restore_grasp_object(
                                self._grasp_reset_qpos,
                                invalid_reason,
                            )
                            break
                        if self._grasp_object_penetrates_table():
                            self._restore_grasp_object(
                                self._grasp_last_safe_qpos,
                                "table penetration",
                            )
                            break
                        if not self._grasp_guard_active:
                            self._remember_safe_grasp_object_pose()
            self._write_mujoco_state()
            mujoco.mj_forward(self._model, self._data)
            self._publish_state(now)
            self._publish_grasp_debug()
            if self._viewer is not None and self._viewer.is_running():
                self._viewer.sync()

    def _advance_side(self, side, now):
        active = self._side[side].get("active")
        if active is None:
            for name in self._side[side]["joints"]:
                self._velocities[name] = 0.0
            return

        elapsed = (now - active.start_time) * 1e-9
        points = active.points
        end_time = duration_sec(points[-1].time_from_start)
        if elapsed >= end_time:
            target = point_positions(points[-1], active.joint_names, self._positions)
            for name in active.joint_names:
                self._positions[name] = target[name]
                self._velocities[name] = 0.0
            active.success = True
            active.done.set()
            self._side[side]["active"] = None
            return

        previous_time = 0.0
        previous_positions = dict(active.start_positions)
        for point in points:
            point_time = duration_sec(point.time_from_start)
            current_positions = point_positions(point, active.joint_names, previous_positions)
            if elapsed <= point_time:
                span = max(1e-6, point_time - previous_time)
                alpha = min(1.0, max(0.0, (elapsed - previous_time) / span))
                for name in active.joint_names:
                    start = previous_positions.get(name, 0.0)
                    end = current_positions.get(name, start)
                    value = start + (end - start) * alpha
                    self._positions[name] = value
                    self._velocities[name] = (end - start) / span
                return
            previous_time = point_time
            previous_positions = current_positions

    def _advance_gripper(self, side):
        cfg = self._side[side]
        current = float(cfg["gripper_position"])
        target = float(cfg["gripper_target"])
        error = target - current
        close_speed = GRIPPER_SPEED_RAD_PER_SEC
        if target < current and self._grasp_capture_active:
            contact = self._grasp_debug_payload(side)
            if contact["left_contact"] and contact["right_contact"]:
                cfg["gripper_target"] = current
                self._apply_gripper_position(cfg, current, 0.0)
                self._attach_grasp_object(side, require_bilateral_contact=True)
                self._grasp_contact_mode[side] = "bilateral"
                self.get_logger().info(
                    f"sim {side} gripper stopped on bilateral object contact "
                    f"at {current:.3f} rad"
                )
                return
            if contact["left_contact"] != contact["right_contact"]:
                touched = "left" if contact["left_contact"] else "right"
                close_speed = GRIPPER_UNILATERAL_SPEED_RAD_PER_SEC
                mode = f"unilateral_{touched}"
                if self._grasp_contact_mode[side] != mode:
                    self.get_logger().info(
                        f"sim {side} gripper entered compliant close after "
                        f"{touched} contact at {current:.3f} rad"
                    )
                self._grasp_contact_mode[side] = mode
                self._compliant_center_grasp_object(side)
            else:
                self._grasp_contact_mode[side] = "none"
        if abs(error) <= 1e-6:
            self._apply_gripper_position(cfg, target, 0.0)
            if (
                target >= GRIPPER_OPEN_POSITION - 0.05
                and self._grasp_weld_active
                and self._grasp_weld_side == side
            ):
                self._detach_grasp_object()
            return
        # While moving towards open, detach early so the object drops.
        if (
            target > current
            and self._grasp_weld_active
            and self._grasp_weld_side == side
        ):
            self._detach_grasp_object()
        step = close_speed * self._period
        next_value = target if abs(error) <= step else current + math.copysign(step, error)
        velocity = (next_value - current) / self._period
        self._apply_gripper_position(cfg, next_value, velocity)

    def _apply_gripper_position(self, cfg, main_position, main_velocity):
        cfg["gripper_position"] = float(main_position)
        for name in cfg["gripper_joints"]:
            multiplier = cfg["gripper_mimic"].get(name, 1.0)
            self._positions[name] = float(main_position) * multiplier
            self._velocities[name] = float(main_velocity) * multiplier

    def _object_near_gripper(self, side, threshold=0.10):
        """Check if the free grasp object is within threshold meters of TCP."""
        if self._grasp_body_id < 0 or self._grasp_free_qpos_adr < 0:
            return False
        site_id = self._grasp_site.get(side, -1)
        if site_id < 0:
            return False
        obj_pos = self._data.xpos[self._grasp_body_id]
        tcp_pos = self._data.site_xpos[site_id]
        return float(np.linalg.norm(obj_pos - tcp_pos)) < threshold

    def _compliant_center_grasp_object(self, side, max_step=0.0002):
        """Slide a contacted free object toward the midpoint of both fingers."""
        if (
            self._grasp_free_qpos_adr < 0
            or self._grasp_free_dof_adr < 0
            or self._grasp_weld_active
        ):
            return
        left_id, right_id = self._grasp_finger_bodies.get(side, (-1, -1))
        if left_id < 0 or right_id < 0:
            return
        left_pos = np.asarray(self._data.xpos[left_id], dtype=np.float64)
        right_pos = np.asarray(self._data.xpos[right_id], dtype=np.float64)
        axis = right_pos - left_pos
        # The object remains supported by the table. Only correct the lateral
        # error in its XY plane; never pull it vertically or along the approach
        # direction.
        axis[2] = 0.0
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            return
        axis /= norm
        qpos = self._grasp_free_qpos_adr
        dof = self._grasp_free_dof_adr
        object_pos = self._data.qpos[qpos:qpos + 3]
        midpoint = 0.5 * (left_pos + right_pos)
        lateral_error = float(np.dot(midpoint - object_pos, axis))
        step = max(-float(max_step), min(float(max_step), lateral_error))
        if abs(step) <= 1e-7:
            return
        self._data.qpos[qpos:qpos + 3] = object_pos + axis * step
        self._data.qvel[dof:dof + 6] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def _align_grasp_site_to_object(self, side):
        """Store the current object pose in the gripper Link7 frame."""
        site_id = self._grasp_site.get(side, -1)
        link_id = self._grasp_link_body.get(side, -1)
        if site_id < 0 or link_id < 0 or self._grasp_body_id < 0:
            return False
        link_pos = self._data.xpos[link_id]
        link_mat = self._data.xmat[link_id].reshape(3, 3)
        object_pos = self._data.xpos[self._grasp_body_id]
        object_mat = self._data.xmat[self._grasp_body_id].reshape(3, 3)
        self._model.site_pos[site_id] = link_mat.T @ (object_pos - link_pos)
        relative_mat = np.ascontiguousarray(link_mat.T @ object_mat).reshape(9)
        relative_quat = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(relative_quat, relative_mat)
        self._model.site_quat[site_id] = relative_quat
        mujoco.mj_forward(self._model, self._data)
        return True

    def _attach_grasp_object(self, side, require_bilateral_contact=False):
        """Attach grasp_object to a nearby gripper without a pose jump."""
        if self._grasp_weld_active or self._grasp_body_id < 0:
            return
        if self._grasp_site.get(side, -1) < 0:
            return
        if not self._object_near_gripper(side):
            return
        if require_bilateral_contact:
            contact = self._grasp_debug_payload(side)
            if not (contact["left_contact"] and contact["right_contact"]):
                return
        if not self._align_grasp_site_to_object(side):
            return
        site_id = self._grasp_site[side]
        self._grasp_relative_pos = self._model.site_pos[site_id].copy()
        self._grasp_relative_quat = self._model.site_quat[site_id].copy()
        self._grasp_weld_active = True
        self._grasp_weld_side = side
        self._grasp_guard_active = False
        self._grasp_guard_qpos = None
        self._grasp_capture_active = False
        self._sync_attached_object_pose()
        mujoco.mj_forward(self._model, self._data)
        self.get_logger().info(f"Grasp object attached to {side} gripper TCP")

    def _detach_grasp_object(self):
        """Release the rigid grasp so the free object can fall."""
        if not self._grasp_weld_active:
            return
        slot = self._grasp_eq.get(self._grasp_weld_side, -1)
        if slot >= 0:
            self._data.eq_active[slot] = 0
        site_id = self._grasp_site.get(self._grasp_weld_side, -1)
        if site_id >= 0:
            self._model.site_pos[site_id] = (0.34, 0.0, 0.0)
            self._model.site_quat[site_id] = (1.0, 0.0, 0.0, 0.0)
        self._grasp_weld_active = False
        self._grasp_weld_side = None
        self._grasp_capture_active = False
        self._grasp_relative_pos = None
        self._grasp_relative_quat = None
        self._place_released_object_above_table()
        self.get_logger().info("Grasp object detached and returned to free physics")

    def _place_released_object_above_table(self, clearance=0.001):
        """Keep a table release at its current XY instead of restoring an old pose."""
        if (
            self._grasp_free_qpos_adr < 0
            or self._grasp_free_dof_adr < 0
            or self._grasp_geom_id < 0
            or self._physical_table_geom_id < 0
        ):
            return
        mujoco.mj_forward(self._model, self._data)
        table_pos = self._data.geom_xpos[self._physical_table_geom_id]
        table_half = self._model.geom_size[self._physical_table_geom_id]
        object_pos, object_half = self._grasp_object_world_bounds()
        inside_xy = (
            abs(float(object_pos[0] - table_pos[0]))
            <= float(table_half[0] + object_half[0])
            and abs(float(object_pos[1] - table_pos[1]))
            <= float(table_half[1] + object_half[1])
        )
        if not inside_xy:
            return
        lowest_z = self._grasp_object_lowest_z()
        table_top = float(table_pos[2] + table_half[2])
        correction = table_top + float(clearance) - lowest_z
        if correction > 0.0:
            qpos = self._grasp_free_qpos_adr
            self._data.qpos[qpos + 2] += correction
        dof = self._grasp_free_dof_adr
        self._data.qvel[dof:dof + 6] = 0.0
        mujoco.mj_forward(self._model, self._data)
        self._remember_safe_grasp_object_pose()

    def _sync_attached_object_pose(self):
        """Rigidly carry the free object with the kinematically driven hand."""
        if (
            not self._grasp_weld_active
            or self._grasp_free_qpos_adr < 0
            or self._grasp_free_dof_adr < 0
            or self._grasp_relative_pos is None
            or self._grasp_relative_quat is None
        ):
            return
        link_id = self._grasp_link_body.get(self._grasp_weld_side, -1)
        if link_id < 0:
            return
        link_mat = self._data.xmat[link_id].reshape(3, 3)
        world_pos = (
            self._data.xpos[link_id]
            + link_mat @ self._grasp_relative_pos
        )
        relative_mat = np.empty(9, dtype=np.float64)
        mujoco.mju_quat2Mat(relative_mat, self._grasp_relative_quat)
        world_mat = np.ascontiguousarray(
            link_mat @ relative_mat.reshape(3, 3)
        ).reshape(9)
        world_quat = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(world_quat, world_mat)
        qpos = self._grasp_free_qpos_adr
        self._data.qpos[qpos:qpos + 3] = world_pos
        self._data.qpos[qpos + 3:qpos + 7] = world_quat
        dof = self._grasp_free_dof_adr
        self._data.qvel[dof:dof + 6] = 0.0

    def _geom_body_name(self, geom_id):
        if geom_id < 0:
            return ""
        body_id = int(self._model.geom_bodyid[geom_id])
        return mujoco.mj_id2name(
            self._model, mujoco.mjtObj.mjOBJ_BODY, body_id
        ) or ""

    @staticmethod
    def _robot_side_for_body(body_name):
        if body_name == "right_base_link" or body_name.startswith("Right_"):
            return "right"
        if body_name == "left_base_link" or body_name.startswith("Left_"):
            return "left"
        return None

    def _robot_table_collision_sides(self):
        collisions = {}
        if self._table_geom_id < 0:
            return collisions
        for index in range(self._data.ncon):
            contact = self._data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if self._table_geom_id not in (geom1, geom2):
                continue
            other = geom2 if geom1 == self._table_geom_id else geom1
            body_name = self._geom_body_name(other)
            side = self._robot_side_for_body(body_name)
            if side is not None:
                collisions.setdefault(side, set()).add(body_name)
        return collisions

    def _grasp_object_invalid_reason(self):
        if self._grasp_free_qpos_adr < 0:
            return None
        qpos = self._grasp_free_qpos_adr
        pose = self._data.qpos[qpos:qpos + 7]
        if not np.all(np.isfinite(pose)):
            return "non-finite pose"
        if float(np.linalg.norm(pose[3:7])) < 0.5:
            return "invalid orientation"
        position = pose[:3]
        if abs(float(position[0])) > 5.0 or abs(float(position[1])) > 5.0:
            return "escaped scene bounds"
        if float(position[2]) < 0.4 or float(position[2]) > 5.0:
            return "escaped vertical bounds"
        return None

    def _grasp_object_penetrates_table(self, tolerance=0.003):
        if (
            self._grasp_body_id < 0
            or self._grasp_geom_id < 0
            or self._physical_table_geom_id < 0
        ):
            return False
        table_pos = self._data.geom_xpos[self._physical_table_geom_id]
        table_half = self._model.geom_size[self._physical_table_geom_id]
        object_pos, object_half = self._grasp_object_world_bounds()
        lowest_z = self._grasp_object_lowest_z()
        table_top = float(table_pos[2] + table_half[2])
        inside_xy = (
            abs(float(object_pos[0] - table_pos[0]))
            <= float(table_half[0] + object_half[0])
            and abs(float(object_pos[1] - table_pos[1]))
            <= float(table_half[1] + object_half[1])
        )
        return inside_xy and lowest_z < table_top - float(tolerance)

    def _grasp_object_world_bounds(self):
        """Return conservative stator bounds in world axes."""
        object_pos = self._data.xpos[self._grasp_body_id]
        object_mat = self._data.xmat[self._grasp_body_id].reshape(3, 3)
        object_half = np.abs(object_mat) @ GRASP_OBJECT_HALF_SIZE
        return object_pos, object_half

    def _grasp_object_lowest_z(self):
        """Return the compiled mesh bottom instead of treating it as a box."""
        if self._grasp_mesh_vertices is not None and self._grasp_geom_id >= 0:
            geom_pos = self._data.geom_xpos[self._grasp_geom_id]
            geom_mat = self._data.geom_xmat[self._grasp_geom_id].reshape(3, 3)
            return float(
                geom_pos[2]
                + np.min(self._grasp_mesh_vertices @ geom_mat[2])
            )
        object_pos, object_half = self._grasp_object_world_bounds()
        return float(object_pos[2] - object_half[2])

    def _remember_safe_grasp_object_pose(self):
        if self._grasp_free_qpos_adr < 0:
            return
        qpos = self._grasp_free_qpos_adr
        pose = self._data.qpos[qpos:qpos + 7]
        if np.all(np.isfinite(pose)):
            self._grasp_last_safe_qpos = pose.copy()

    def _restore_grasp_object(self, pose, reason):
        if self._grasp_free_qpos_adr < 0 or pose is None:
            return
        qpos = self._grasp_free_qpos_adr
        dof = self._grasp_free_dof_adr
        self._data.qpos[qpos:qpos + 7] = pose
        self._data.qvel[dof:dof + 6] = 0.0
        self._grasp_guard_active = False
        self._grasp_guard_qpos = None
        mujoco.mj_forward(self._model, self._data)
        now = time.monotonic()
        if now - self._last_object_guard_log >= 1.0:
            self.get_logger().error(
                f"MuJoCo grasp object guard restored safe pose: {reason}"
            )
            self._last_object_guard_log = now

    def _grasp_debug_payload(self, side):
        object_contacts = []
        left_geoms = []
        right_geoms = []
        max_penetration = 0.0
        if self._grasp_geom_id >= 0:
            for index in range(self._data.ncon):
                contact = self._data.contact[index]
                geom1, geom2 = int(contact.geom1), int(contact.geom2)
                if self._grasp_geom_id not in (geom1, geom2):
                    continue
                other = geom2 if geom1 == self._grasp_geom_id else geom1
                body_name = self._geom_body_name(other)
                point = [float(value) for value in contact.pos]
                penetration = max(0.0, -float(contact.dist))
                max_penetration = max(max_penetration, penetration)
                object_contacts.append({
                    "body": body_name,
                    "point": point,
                    "penetration_m": penetration,
                })
                prefix = "Right_" if side == "right" else "Left_"
                if body_name.startswith(prefix + "Gripper_Left_"):
                    left_geoms.append(body_name)
                elif body_name.startswith(prefix + "Gripper_Right_"):
                    right_geoms.append(body_name)
        cfg = self._side[side]
        object_position = None
        object_pose = None
        object_motion = None
        if self._grasp_body_id >= 0:
            object_position = [
                float(value) for value in self._data.xpos[self._grasp_body_id]
            ]
            object_pose = {
                "position": object_position,
                "orientation_wxyz": [
                    float(value) for value in self._data.xquat[self._grasp_body_id]
                ],
            }
            if self._grasp_free_dof_adr >= 0:
                dof = self._grasp_free_dof_adr
                linear = np.asarray(self._data.qvel[dof:dof + 3], dtype=np.float64)
                angular = np.asarray(self._data.qvel[dof + 3:dof + 6], dtype=np.float64)
                object_motion = {
                    "linear_velocity_mps": linear.tolist(),
                    "angular_velocity_radps": angular.tolist(),
                    "linear_speed_mps": float(np.linalg.norm(linear)),
                    "angular_speed_radps": float(np.linalg.norm(angular)),
                }
        return {
            "side": side,
            # Keep a same-tick joint snapshot next to the free-object pose.
            # The Web backend otherwise combines independently delivered
            # /joint_states and /mujoco/grasp_debug samples, which can make the
            # rendered hand lead the attached object by one or more frames.
            "sim_time": float(self._data.time),
            "joint_positions": {
                name: float(value) for name, value in self._positions.items()
            },
            "gripper_joint_angle": float(cfg["gripper_position"]),
            "gripper_width": None,
            "left_contact": bool(left_geoms),
            "right_contact": bool(right_geoms),
            "contact_stopped": bool(left_geoms and right_geoms),
            "contact_count": len(object_contacts),
            "max_penetration_m": max_penetration,
            "contact_points": object_contacts,
            "left_contact_geoms": sorted(set(left_geoms)),
            "right_contact_geoms": sorted(set(right_geoms)),
            "object_geom": "grasp_object_geom",
            "object_position": object_position,
            "object_pose": object_pose,
            "object_motion": object_motion,
            "object_attached": bool(
                self._grasp_weld_active and self._grasp_weld_side == side
            ),
        }

    def _on_set_object_pose(self, msg):
        try:
            data = json.loads(msg.data)
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            z = float(data.get("z", 0.0))
            self._pending_object_pose = (x, y, z)
        except Exception as exc:
            self.get_logger().error(f"Invalid set_object_pose: {exc}")

    def _apply_pending_object_pose(self):
        if self._pending_object_pose is None:
            return
        if self._grasp_free_qpos_adr < 0:
            self._pending_object_pose = None
            return
        x, y, z = self._pending_object_pose
        self._pending_object_pose = None
        if self._grasp_weld_active:
            self._detach_grasp_object()
        adr = self._grasp_free_qpos_adr
        self._data.qpos[adr] = x
        self._data.qpos[adr + 1] = y
        self._data.qpos[adr + 2] = z
        self._data.qpos[adr + 3:adr + 7] = (1.0, 0.0, 0.0, 0.0)
        dof = self._grasp_free_dof_adr
        self._data.qvel[dof:dof + 6] = 0.0
        self._grasp_guard_active = False
        self._grasp_guard_qpos = None
        mujoco.mj_forward(self._model, self._data)
        self._remember_safe_grasp_object_pose()
        self._grasp_reset_qpos = self._grasp_last_safe_qpos.copy()
        self.get_logger().info(
            f"Applied requested object pose: ({x:.3f}, {y:.3f}, {z:.3f})"
        )

    def _publish_grasp_debug(self):
        for side in ("right", "left"):
            msg = String()
            msg.data = json.dumps(
                self._grasp_debug_payload(side), separators=(",", ":")
            )
            self._grasp_debug_pub.publish(msg)

    def _write_mujoco_state(self):
        for name, value in self._positions.items():
            qpos = self._joint_qpos[name]
            dof = self._joint_dof[name]
            self._data.qpos[qpos] = value
            self._data.qvel[dof] = self._velocities.get(name, 0.0)
        # Arm joints are kinematic inputs.  Refresh their body transforms, then
        # carry an attached free object with the saved TCP-relative transform.
        mujoco.mj_kinematics(self._model, self._data)
        if self._grasp_weld_active:
            self._sync_attached_object_pose()
        elif self._grasp_guard_active:
            if time.monotonic() >= self._grasp_guard_deadline:
                self._grasp_guard_active = False
                self._grasp_guard_qpos = None
            elif self._grasp_guard_qpos is not None:
                qpos = self._grasp_free_qpos_adr
                dof = self._grasp_free_dof_adr
                self._data.qpos[qpos:qpos + 7] = self._grasp_guard_qpos
                self._data.qvel[dof:dof + 6] = 0.0

    def _publish_state(self, _now):
        stamp = self.get_clock().now().to_msg()
        for cfg in self._side.values():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(cfg["joints"])
            msg.position = [self._positions[name] for name in cfg["joints"]]
            msg.velocity = [self._velocities[name] for name in cfg["joints"]]
            msg.effort = [0.0 for _ in cfg["joints"]]
            cfg["state_pub"].publish(msg)

            gripper_names = list(cfg["gripper_joints"])
            if gripper_names:
                gripper = JointState()
                gripper.header.stamp = stamp
                gripper.name = gripper_names
                gripper.position = [self._positions[name] for name in gripper_names]
                gripper.velocity = [self._velocities[name] for name in gripper_names]
                gripper.effort = [0.0 for _ in gripper_names]
                cfg["gripper_state_pub"].publish(gripper)

            estop = Bool()
            estop.data = bool(cfg["estop"])
            cfg["estop_pub"].publish(estop)
            cfg["estop_pub_compat"].publish(estop)

            enabled = Bool()
            enabled.data = bool(cfg["enabled"])
            cfg["enabled_pub"].publish(enabled)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Path to compiled MuJoCo .mjb model")
    parser.add_argument("--publish-rate", type=float, default=100.0)
    parser.add_argument("--viewer", action="store_true", help="Open a MuJoCo passive viewer")
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=5.0)
    parser.add_argument("--camera-topic", default="/mujoco_camera/color/image_raw")
    parser.add_argument("--camera-frame-id", default="vlm_camera_link")
    parser.add_argument("--camera-name", default="head_d455_rgb")
    parser.add_argument("--camera-depth", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--camera-depth-topic", default="/mujoco_camera/depth/image_raw")
    parser.add_argument("--camera-depth-fps", type=float, default=2.0)
    parser.add_argument("--camera-info-topic", default="/mujoco_camera/color/camera_info")
    parser.add_argument("--camera-max-range", type=float, default=10.0)
    parser.add_argument("--wrist-camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--wrist-camera-width", type=int, default=480)
    parser.add_argument("--wrist-camera-height", type=int, default=360)
    parser.add_argument("--wrist-camera-fps", type=float, default=15.0)
    parser.add_argument("--wrist-depth-fps", type=float, default=0.2)
    parser.add_argument("--wrist-camera-max-range", type=float, default=3.0)
    parser.add_argument("--right-wrist-camera-topic", default="/right_wrist_camera/color/image_raw")
    parser.add_argument("--right-wrist-depth-topic", default="/right_wrist_camera/depth/image_raw")
    parser.add_argument("--right-wrist-camera-info-topic", default="/right_wrist_camera/color/camera_info")
    parser.add_argument("--left-wrist-camera-topic", default="/left_wrist_camera/color/image_raw")
    parser.add_argument("--left-wrist-depth-topic", default="/left_wrist_camera/depth/image_raw")
    parser.add_argument("--left-wrist-camera-info-topic", default="/left_wrist_camera/color/camera_info")
    parser.add_argument("--overview-camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--overview-camera-fps", type=float, default=2.0)
    parser.add_argument("--overview-camera-topic", default="/overview_camera/color/image_raw")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    camera_config = [{
        "enabled": args.camera,
        "name": args.camera_name,
        "width": args.camera_width,
        "height": args.camera_height,
        "fps": args.camera_fps,
        "topic": args.camera_topic,
        "frame_id": args.camera_frame_id,
        "depth_enabled": args.camera_depth,
        "depth_topic": args.camera_depth_topic,
        "camera_info_topic": args.camera_info_topic,
        "max_range": args.camera_max_range,
        "depth_fps": args.camera_depth_fps,
    }, {
        "enabled": args.wrist_camera,
        "name": "right_wrist_rgb",
        "width": args.wrist_camera_width,
        "height": args.wrist_camera_height,
        "fps": args.wrist_camera_fps,
        "topic": args.right_wrist_camera_topic,
        "frame_id": "right_wrist_camera_optical_frame",
        "depth_enabled": True,
        "depth_topic": args.right_wrist_depth_topic,
        "camera_info_topic": args.right_wrist_camera_info_topic,
        "max_range": args.wrist_camera_max_range,
        "depth_fps": args.wrist_depth_fps,
    }, {
        "enabled": args.wrist_camera,
        "name": "left_wrist_rgb",
        "width": args.wrist_camera_width,
        "height": args.wrist_camera_height,
        "fps": args.wrist_camera_fps,
        "topic": args.left_wrist_camera_topic,
        "frame_id": "left_wrist_camera_optical_frame",
        "depth_enabled": True,
        "depth_topic": args.left_wrist_depth_topic,
        "camera_info_topic": args.left_wrist_camera_info_topic,
        "max_range": args.wrist_camera_max_range,
        "depth_fps": args.wrist_depth_fps,
    }, {
        "enabled": args.overview_camera,
        "name": "overview_camera",
        "width": args.camera_width,
        "height": args.camera_height,
        "fps": args.overview_camera_fps,
        "topic": args.overview_camera_topic,
        "frame_id": "overview_camera",
    }]
    node = MujocoSimBridge(
        args.model, args.publish_rate, args.viewer, camera_config
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if "context is not valid" not in str(exc):
            raise
    finally:
        node.stop_simulation()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
