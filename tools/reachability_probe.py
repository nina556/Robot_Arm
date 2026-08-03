#!/usr/bin/env python3
"""Random grasp-target sampler with pluggable reachability checkers.

The tool intentionally keeps ROS dependencies lazy.  The geometry checker can
run on a development machine; MoveIt checkers require a sourced ROS 2
environment and a running MoveIt stack that exposes /compute_ik and
/plan_kinematic_path.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
import os
import random
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "asm0003" / "urdf" / "asm0003.urdf"
DEFAULT_FRAME_ID = os.environ.get("FRAME_ID", "world")
DEFAULT_JOINT_STATE_TOPIC = os.environ.get("JOINT_STATE_TOPIC", "/joint_states")
DEFAULT_PLAN_SERVICE = os.environ.get("PLAN_SERVICE", "/plan_kinematic_path")
DEFAULT_IK_SERVICE = os.environ.get("IK_SERVICE", "/compute_ik")
DEFAULT_TOOL_OFFSETS = {
    "right": [0.0, 0.15, 0.0],
    "left": [0.0, 0.15, 0.0],
}
DEFAULT_ARMS = {
    "right": {
        "group": os.environ.get("MOVE_GROUP_NAME", "arm"),
        "base_link": "right_base_link",
        "tip_link": os.environ.get("TIP_LINK", "Right_Link7"),
        "joints": [f"Right_Joint{i}" for i in range(1, 8)],
        "tool_offset": DEFAULT_TOOL_OFFSETS["right"],
    },
    "left": {
        "group": os.environ.get("LEFT_MOVE_GROUP_NAME", "left_arm"),
        "base_link": "left_base_link",
        "tip_link": os.environ.get("LEFT_TIP_LINK", "Left_Link7"),
        "joints": [f"Left_Joint{i}" for i in range(1, 8)],
        "tool_offset": DEFAULT_TOOL_OFFSETS["left"],
    },
}
OMPL_CHECKER_SET = [
    "moveit_plan:RRTConnectkConfigDefault",
    "moveit_plan:RRTkConfigDefault",
    "moveit_plan:RRTstarkConfigDefault",
    "moveit_plan:ESTkConfigDefault",
    "moveit_plan:LBKPIECEkConfigDefault",
]
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


@dataclass(frozen=True)
class PoseSample:
    sample_id: int
    region: str
    arm: str
    frame_id: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.sample_id,
            "region": self.region,
            "arm": self.arm,
            "frame_id": self.frame_id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "metadata": dict(self.metadata),
        }


@dataclass
class CheckResult:
    ok: bool
    reason: str = ""
    score: float | None = None
    elapsed_ms: float = 0.0
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        data = {
            "ok": bool(self.ok),
            "reason": self.reason,
            "elapsed_ms": round(float(self.elapsed_ms), 3),
            "details": self.details,
        }
        if self.score is not None:
            data["score"] = self.score
        return data


class Checker:
    name = "checker"

    def check(self, sample: PoseSample) -> CheckResult:
        raise NotImplementedError


def deep_update(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_update(result[key], value)
        else:
            result[key] = value
    return result


def default_config() -> dict[str, Any]:
    return {
        "frame_id": DEFAULT_FRAME_ID,
        "moveit_request_frame": "base_link",
        "seed": 7,
        "count": 100,
        "urdf": str(DEFAULT_URDF),
        "joint_state_topic": DEFAULT_JOINT_STATE_TOPIC,
        "sampling": {
            "enforce_arm_reach": True,
            "max_resample_attempts": 1000,
        },
        "world_from_base_link": {
            "xyz": [0.0, 0.0, 0.0],
            "rpy": [1.5708, 0.0, 0.0],
        },
        "arms": DEFAULT_ARMS,
        "regions": {
            "right_front_box": {
                "sampler": "box",
                "arm": "right",
                "x": [-0.80, 0.35],
                "y": [-0.65, 0.25],
                "z": [0.75, 1.55],
                "roll": math.pi,
                "pitch": 0.0,
                "yaw": [-math.pi, math.pi],
            },
            "left_front_box": {
                "sampler": "box",
                "arm": "left",
                "x": [-0.35, 0.80],
                "y": [-0.65, 0.25],
                "z": [0.75, 1.55],
                "roll": math.pi,
                "pitch": 0.0,
                "yaw": [-math.pi, math.pi],
            },
        },
        "checkers": [{"name": "geometry"}],
        "geometry": {
            "min_reach": 0.04,
            "reach_margin": 0.0,
        },
        "moveit_ik": {
            "service": DEFAULT_IK_SERVICE,
            "timeout_sec": 2.0,
            "wait_service_sec": 8.0,
            "avoid_collisions": True,
            "use_current_state": True,
        },
        "moveit_plan": {
            "ik_service": DEFAULT_IK_SERVICE,
            "plan_service": DEFAULT_PLAN_SERVICE,
            "ik_timeout_sec": 2.0,
            "planning_time_sec": 3.0,
            "wait_service_sec": 8.0,
            "attempts": 3,
            "avoid_collisions": True,
            "velocity_scaling": 0.05,
            "acceleration_scaling": 0.05,
        },
    }


def load_config(path: Path | None) -> dict[str, Any]:
    cfg = default_config()
    if path is None:
        return cfg
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        loaded = json.loads(text)
    else:
        try:
            import yaml  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                f"{path} looks like YAML but PyYAML is not available. "
                "Use a .json config or install python3-yaml."
            ) from exc
        loaded = yaml.safe_load(text) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a mapping/object at top level")
    merged = deep_update(cfg, loaded)
    # Region/checker lists are experiment definitions; when a config provides
    # them, replacing the defaults is less surprising than merging in samples
    # from unrelated built-in regions.
    for replace_key in ("regions", "checkers"):
        if replace_key in loaded:
            merged[replace_key] = loaded[replace_key]
    return merged


def require_number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number, got {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite, got {value!r}")
    return number


def sample_scalar(spec: Any, rng: random.Random, label: str, default: float | None = None) -> float:
    if spec is None:
        if default is None:
            raise ValueError(f"{label} is required")
        return default
    if isinstance(spec, dict):
        if "fixed" in spec:
            return require_number(spec["fixed"], label)
        if "range" in spec:
            return sample_scalar(spec["range"], rng, label)
        if "choices" in spec:
            choices = spec["choices"]
            if not isinstance(choices, list) or not choices:
                raise ValueError(f"{label}.choices must be a non-empty list")
            return require_number(rng.choice(choices), label)
        raise ValueError(f"{label} dict must contain fixed, range, or choices")
    if isinstance(spec, list):
        if len(spec) == 2 and all(isinstance(v, (int, float)) for v in spec):
            lo = require_number(spec[0], f"{label}[0]")
            hi = require_number(spec[1], f"{label}[1]")
            if hi < lo:
                raise ValueError(f"{label} range max must be >= min")
            return rng.uniform(lo, hi)
        if spec:
            return require_number(rng.choice(spec), label)
        raise ValueError(f"{label} choices must not be empty")
    return require_number(spec, label)


def normalize_arm(value: Any) -> str:
    return "left" if str(value).lower() == "left" else "right"


class RegionSampler:
    def __init__(self, name: str, cfg: dict[str, Any], frame_id: str):
        self.name = name
        self.cfg = cfg
        self.frame_id = str(cfg.get("frame_id", frame_id))
        self.arm = normalize_arm(cfg.get("arm", "right"))
        self.kind = str(cfg.get("sampler", cfg.get("type", "box"))).lower()
        self.weight = max(0.0, float(cfg.get("weight", 1.0)))

    def sample(self, sample_id: int, rng: random.Random) -> PoseSample:
        if self.kind == "box":
            x = sample_scalar(self.cfg.get("x"), rng, f"{self.name}.x")
            y = sample_scalar(self.cfg.get("y"), rng, f"{self.name}.y")
            z = sample_scalar(self.cfg.get("z"), rng, f"{self.name}.z")
            metadata = {"sampler": "box"}
        elif self.kind == "cylinder":
            center = self.cfg.get("center", [0.0, 0.0])
            if not isinstance(center, list) or len(center) != 2:
                raise ValueError(f"{self.name}.center must be [x, y]")
            radius_spec = self.cfg.get("radius", [0.0, 0.5])
            if isinstance(radius_spec, list) and len(radius_spec) == 2:
                r0 = require_number(radius_spec[0], f"{self.name}.radius[0]")
                r1 = require_number(radius_spec[1], f"{self.name}.radius[1]")
                radius = math.sqrt(rng.uniform(r0 * r0, r1 * r1))
            else:
                radius = sample_scalar(radius_spec, rng, f"{self.name}.radius")
            theta = sample_scalar(self.cfg.get("theta", [-math.pi, math.pi]), rng, f"{self.name}.theta")
            x = require_number(center[0], f"{self.name}.center[0]") + radius * math.cos(theta)
            y = require_number(center[1], f"{self.name}.center[1]") + radius * math.sin(theta)
            z = sample_scalar(self.cfg.get("z"), rng, f"{self.name}.z")
            metadata = {"sampler": "cylinder", "radius": radius, "theta": theta}
        else:
            raise ValueError(f"unsupported sampler for {self.name}: {self.kind}")

        orientation = self.cfg.get("orientation", {})
        if orientation is None:
            orientation = {}
        if not isinstance(orientation, dict):
            raise ValueError(f"{self.name}.orientation must be a mapping")
        roll = sample_scalar(self.cfg.get("roll", orientation.get("roll")), rng, f"{self.name}.roll", math.pi)
        pitch = sample_scalar(self.cfg.get("pitch", orientation.get("pitch")), rng, f"{self.name}.pitch", 0.0)
        yaw = sample_scalar(
            self.cfg.get("yaw", orientation.get("yaw")),
            rng,
            f"{self.name}.yaw",
            rng.uniform(-math.pi, math.pi),
        )
        return PoseSample(
            sample_id=sample_id,
            region=self.name,
            arm=self.arm,
            frame_id=self.frame_id,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            metadata=metadata,
        )


def vec_norm(v: list[float]) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transform_from_xyz_rpy(xyz: list[float], rpy: list[float]) -> list[list[float]]:
    rot = rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return [
        [rot[0][0], rot[0][1], rot[0][2], float(xyz[0])],
        [rot[1][0], rot[1][1], rot[1][2], float(xyz[1])],
        [rot[2][0], rot[2][1], rot[2][2], float(xyz[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_from_rotation_translation(rot: list[list[float]], xyz: list[float]) -> list[list[float]]:
    return [
        [rot[0][0], rot[0][1], rot[0][2], float(xyz[0])],
        [rot[1][0], rot[1][1], rot[1][2], float(xyz[1])],
        [rot[2][0], rot[2][1], rot[2][2], float(xyz[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for r in range(4):
        for c in range(4):
            out[r][c] = sum(a[r][k] * b[k][c] for k in range(4))
    return out


def inverse_transform(t: list[list[float]]) -> list[list[float]]:
    rot_t = [[t[c][r] for c in range(3)] for r in range(3)]
    trans = [t[0][3], t[1][3], t[2][3]]
    inv_trans = [-sum(rot_t[r][k] * trans[k] for k in range(3)) for r in range(3)]
    return matrix_from_rotation_translation(rot_t, inv_trans)


def transform_point(t: list[list[float]], p: list[float]) -> list[float]:
    return [
        t[0][0] * p[0] + t[0][1] * p[1] + t[0][2] * p[2] + t[0][3],
        t[1][0] * p[0] + t[1][1] * p[1] + t[1][2] * p[2] + t[1][3],
        t[2][0] * p[0] + t[2][1] * p[1] + t[2][2] * p[2] + t[2][3],
    ]


def rotation_from_transform(t: list[list[float]]) -> list[list[float]]:
    return [row[:3] for row in t[:3]]


def translation_from_transform(t: list[list[float]]) -> list[float]:
    return [t[0][3], t[1][3], t[2][3]]


def quat_from_matrix(rot: list[list[float]]) -> dict[str, float]:
    trace = rot[0][0] + rot[1][1] + rot[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return {
            "w": 0.25 * s,
            "x": (rot[2][1] - rot[1][2]) / s,
            "y": (rot[0][2] - rot[2][0]) / s,
            "z": (rot[1][0] - rot[0][1]) / s,
        }
    if rot[0][0] > rot[1][1] and rot[0][0] > rot[2][2]:
        s = math.sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) * 2.0
        return {
            "w": (rot[2][1] - rot[1][2]) / s,
            "x": 0.25 * s,
            "y": (rot[0][1] + rot[1][0]) / s,
            "z": (rot[0][2] + rot[2][0]) / s,
        }
    if rot[1][1] > rot[2][2]:
        s = math.sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]) * 2.0
        return {
            "w": (rot[0][2] - rot[2][0]) / s,
            "x": (rot[0][1] + rot[1][0]) / s,
            "y": 0.25 * s,
            "z": (rot[1][2] + rot[2][1]) / s,
        }
    s = math.sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]) * 2.0
    return {
        "w": (rot[1][0] - rot[0][1]) / s,
        "x": (rot[0][2] + rot[2][0]) / s,
        "y": (rot[1][2] + rot[2][1]) / s,
        "z": 0.25 * s,
    }


def parse_xyz(value: str | None) -> list[float]:
    if not value:
        return [0.0, 0.0, 0.0]
    parts = value.split()
    if len(parts) != 3:
        raise ValueError(f"xyz/rpy must contain 3 numbers, got {value!r}")
    return [float(v) for v in parts]


class RobotGeometry:
    def __init__(self, urdf_path: Path):
        self.urdf_path = urdf_path
        root = ET.parse(urdf_path).getroot()
        self.children: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            origin = joint.find("origin")
            xyz = parse_xyz(origin.attrib.get("xyz") if origin is not None else None)
            rpy = parse_xyz(origin.attrib.get("rpy") if origin is not None else None)
            item = {
                "name": joint.attrib.get("name", ""),
                "type": joint.attrib.get("type", ""),
                "parent": parent.attrib.get("link"),
                "child": child.attrib.get("link"),
                "xyz": xyz,
                "rpy": rpy,
            }
            self.children[item["parent"]].append(item)

    def chain(self, base: str, tip: str) -> list[dict[str, Any]]:
        queue: list[tuple[str, list[dict[str, Any]]]] = [(base, [])]
        seen = set()
        while queue:
            link, path = queue.pop(0)
            if link == tip:
                return path
            if link in seen:
                continue
            seen.add(link)
            for joint in self.children.get(link, []):
                queue.append((joint["child"], path + [joint]))
        raise ValueError(f"no URDF chain from {base} to {tip}")

    def transform(self, base: str, tip: str) -> list[list[float]]:
        t = transform_from_xyz_rpy([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        for joint in self.chain(base, tip):
            t = mat_mul(t, transform_from_xyz_rpy(joint["xyz"], joint["rpy"]))
        return t

    def chain_length(self, base: str, tip: str) -> float:
        return sum(vec_norm(joint["xyz"]) for joint in self.chain(base, tip))


class GeometryReachChecker(Checker):
    name = "geometry"

    def __init__(self, cfg: dict[str, Any]):
        self.cfg = cfg
        self.urdf_path = Path(str(cfg.get("urdf", DEFAULT_URDF))).expanduser()
        self.geometry = RobotGeometry(self.urdf_path)
        self.arms = deep_update(DEFAULT_ARMS, cfg.get("arms", {}))
        world_from_base = cfg.get("world_from_base_link", {})
        self.world_from_base = transform_from_xyz_rpy(
            [float(v) for v in world_from_base.get("xyz", [0.0, 0.0, 0.0])],
            [float(v) for v in world_from_base.get("rpy", [0.0, 0.0, 0.0])],
        )
        gcfg = cfg.get("geometry", {})
        self.min_reach = float(gcfg.get("min_reach", 0.04))
        self.reach_margin = float(gcfg.get("reach_margin", 0.05))
        self._arm_cache: dict[str, dict[str, Any]] = {}

    def _arm_geometry(self, arm: str) -> dict[str, Any]:
        arm = normalize_arm(arm)
        if arm in self._arm_cache:
            return self._arm_cache[arm]
        cfg = self.arms[arm]
        base_link = cfg["base_link"]
        tip_link = cfg["tip_link"]
        base_from_robot = self.geometry.transform("base_link", base_link)
        world_from_arm_base = mat_mul(self.world_from_base, base_from_robot)
        base_position = transform_point(world_from_arm_base, [0.0, 0.0, 0.0])
        tool_offset = [float(v) for v in cfg.get("tool_offset", DEFAULT_TOOL_OFFSETS[arm])]
        max_reach = cfg.get("max_reach")
        if max_reach is None:
            max_reach = self.geometry.chain_length(base_link, tip_link) + vec_norm(tool_offset) + self.reach_margin
        out = {
            "base_link": base_link,
            "tip_link": tip_link,
            "base_position": base_position,
            "max_reach": float(max_reach),
            "tool_offset": tool_offset,
        }
        self._arm_cache[arm] = out
        return out

    def check(self, sample: PoseSample) -> CheckResult:
        start = time.monotonic()
        arm = self._arm_geometry(sample.arm)
        target = [sample.x, sample.y, sample.z]
        distance = vec_norm([target[i] - arm["base_position"][i] for i in range(3)])
        ok = self.min_reach <= distance <= arm["max_reach"]
        if ok:
            reason = "within_radius_envelope"
            score = 1.0 - min(1.0, distance / max(arm["max_reach"], 1e-9))
        elif distance < self.min_reach:
            reason = "too_close_to_arm_base"
            score = 0.0
        else:
            reason = "outside_radius_envelope"
            score = 0.0
        return CheckResult(
            ok=ok,
            reason=reason,
            score=score,
            elapsed_ms=(time.monotonic() - start) * 1000.0,
            details={
                "distance": distance,
                "min_reach": self.min_reach,
                "max_reach": arm["max_reach"],
                "base_position": arm["base_position"],
                "base_link": arm["base_link"],
                "tip_link": arm["tip_link"],
            },
        )


def config_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class ArmReachSamplingConstraint:
    def __init__(self, cfg: dict[str, Any]):
        self.geometry = GeometryReachChecker(cfg)
        self.frame_id = str(cfg.get("frame_id", DEFAULT_FRAME_ID))

    def accept(self, sample: PoseSample, attempt: int) -> tuple[bool, PoseSample, CheckResult]:
        if sample.frame_id != self.frame_id:
            raise ValueError(
                f"arm reach sampling constraint expects frame {self.frame_id!r}, got {sample.frame_id!r}"
            )
        result = self.geometry.check(sample)
        details = dict(result.details)
        metadata = dict(sample.metadata)
        metadata.update({
            "arm_reach_enforced": True,
            "arm_reach_attempt": attempt,
            "arm_reach_distance": details.get("distance"),
            "arm_max_reach": details.get("max_reach"),
            "arm_base_position": details.get("base_position"),
        })
        return result.ok, replace(sample, metadata=metadata), result


def sample_with_constraints(
    sampler: RegionSampler,
    sample_id: int,
    rng: random.Random,
    reach_constraint: ArmReachSamplingConstraint | None,
    max_attempts: int,
) -> PoseSample:
    if reach_constraint is None:
        return sampler.sample(sample_id, rng)
    last_result = None
    last_sample = None
    for attempt in range(1, max_attempts + 1):
        candidate = sampler.sample(sample_id, rng)
        ok, constrained, result = reach_constraint.accept(candidate, attempt)
        if ok:
            return constrained
        last_sample = candidate
        last_result = result
    details = last_result.details if last_result is not None else {}
    raise RuntimeError(
        f"could not sample {sampler.name!r} inside {sampler.arm} arm reach "
        f"after {max_attempts} attempts; last_reason={getattr(last_result, 'reason', 'none')} "
        f"last_xyz={None if last_sample is None else [last_sample.x, last_sample.y, last_sample.z]} "
        f"distance={details.get('distance')} max_reach={details.get('max_reach')}"
    )


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> dict[str, float]:
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


def rotate_vector_by_quat(v: list[float], qx: float, qy: float, qz: float, qw: float) -> list[float]:
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return [
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    ]


_RCLPY_INITIALIZED = False


def import_moveit_runtime() -> dict[str, Any]:
    global _RCLPY_INITIALIZED
    try:
        import rclpy
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
        from moveit_msgs.srv import GetMotionPlan, GetPositionIK
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError(
            "MoveIt checker requires a sourced ROS 2/MoveIt environment. "
            "Run `source /opt/ros/humble/setup.bash` and source this workspace install first."
        ) from exc
    if not _RCLPY_INITIALIZED and not rclpy.ok():
        rclpy.init(args=None)
        _RCLPY_INITIALIZED = True
    return {
        "rclpy": rclpy,
        "Duration": Duration,
        "PoseStamped": PoseStamped,
        "Constraints": Constraints,
        "JointConstraint": JointConstraint,
        "MotionPlanRequest": MotionPlanRequest,
        "RobotState": RobotState,
        "GetMotionPlan": GetMotionPlan,
        "GetPositionIK": GetPositionIK,
        "JointState": JointState,
    }


class MoveItBaseChecker(Checker):
    def __init__(self, cfg: dict[str, Any], section: str, node_name: str):
        self.cfg = cfg
        self.section = cfg.get(section, {})
        self.arms = deep_update(DEFAULT_ARMS, cfg.get("arms", {}))
        self.frame_id = str(cfg.get("frame_id", DEFAULT_FRAME_ID))
        self.moveit_request_frame = str(cfg.get("moveit_request_frame", self.frame_id) or self.frame_id)
        world_from_base = cfg.get("world_from_base_link", {})
        self.world_from_base = transform_from_xyz_rpy(
            [float(v) for v in world_from_base.get("xyz", [0.0, 0.0, 0.0])],
            [float(v) for v in world_from_base.get("rpy", [0.0, 0.0, 0.0])],
        )
        self.joint_state_topic = str(cfg.get("joint_state_topic", DEFAULT_JOINT_STATE_TOPIC))
        self.rt = import_moveit_runtime()
        self.rclpy = self.rt["rclpy"]
        self.node = self.rclpy.create_node(node_name)
        self.latest_joint_state = None
        self._service_clients: dict[tuple[Any, str], Any] = {}
        self.node.create_subscription(
            self.rt["JointState"],
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )

    def _on_joint_state(self, msg: Any) -> None:
        self.latest_joint_state = msg

    def _arm_config(self, arm: str) -> dict[str, Any]:
        return self.arms[normalize_arm(arm)]

    def _wait_for_client(self, client: Any, timeout_sec: float) -> None:
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=timeout_sec)
        if not client.service_is_ready():
            raise RuntimeError(f"service not ready: {client.srv_name}")

    def _client(self, srv_type: Any, service: str) -> Any:
        key = (srv_type, service)
        if key not in self._service_clients:
            self._service_clients[key] = self.node.create_client(srv_type, service)
        return self._service_clients[key]

    def _spin_some(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.latest_joint_state is not None:
                return

    def _duration_msg(self, seconds: float) -> Any:
        duration = self.rt["Duration"]()
        seconds = max(0.0, float(seconds))
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1e9)
        return duration

    def _robot_state(self) -> Any:
        state = self.rt["RobotState"]()
        if self.latest_joint_state is not None:
            allowed = {
                joint
                for arm_cfg in self.arms.values()
                for joint in arm_cfg.get("joints", [])
            }
            filtered_names = []
            filtered_positions = []
            filtered_velocities = []
            has_velocity = bool(self.latest_joint_state.velocity)
            for index, name in enumerate(self.latest_joint_state.name):
                if name not in allowed:
                    continue
                filtered_names.append(name)
                filtered_positions.append(self.latest_joint_state.position[index])
                if has_velocity and index < len(self.latest_joint_state.velocity):
                    filtered_velocities.append(self.latest_joint_state.velocity[index])
            state.joint_state.name = filtered_names
            state.joint_state.position = filtered_positions
            if filtered_velocities:
                state.joint_state.velocity = filtered_velocities
        return state

    def _sample_pose_in_request_frame(self, sample: PoseSample) -> tuple[str, list[float], dict[str, float]]:
        source_frame = sample.frame_id or self.frame_id
        target_frame = self.moveit_request_frame or source_frame
        source_position = [sample.x, sample.y, sample.z]
        source_rot = rpy_to_matrix(sample.roll, sample.pitch, sample.yaw)
        source_pose = matrix_from_rotation_translation(source_rot, source_position)
        if source_frame == target_frame:
            return target_frame, source_position, quat_from_matrix(source_rot)
        if source_frame == self.frame_id and target_frame == "base_link":
            target_pose = mat_mul(inverse_transform(self.world_from_base), source_pose)
            return target_frame, translation_from_transform(target_pose), quat_from_matrix(rotation_from_transform(target_pose))
        raise ValueError(f"cannot transform sample frame {source_frame!r} to MoveIt request frame {target_frame!r}")

    def _pose_stamped(self, sample: PoseSample) -> Any:
        cfg = self._arm_config(sample.arm)
        tool_offset = [float(v) for v in cfg.get("tool_offset", DEFAULT_TOOL_OFFSETS[sample.arm])]
        request_frame, position, q = self._sample_pose_in_request_frame(sample)
        offset_world = rotate_vector_by_quat(tool_offset, q["x"], q["y"], q["z"], q["w"])
        stamped = self.rt["PoseStamped"]()
        stamped.header.frame_id = request_frame
        # Keep the timestamp at zero, matching web_control/server.py.  MoveIt
        # then uses the latest available TF instead of requiring a transform at
        # the probe node's wall-clock timestamp.
        stamped.pose.position.x = position[0] - offset_world[0]
        stamped.pose.position.y = position[1] - offset_world[1]
        stamped.pose.position.z = position[2] - offset_world[2]
        stamped.pose.orientation.x = q["x"]
        stamped.pose.orientation.y = q["y"]
        stamped.pose.orientation.z = q["z"]
        stamped.pose.orientation.w = q["w"]
        return stamped

    def _call_ik(self, sample: PoseSample, service: str, timeout_sec: float, avoid_collisions: bool) -> tuple[Any, float]:
        cfg = self._arm_config(sample.arm)
        client = self._client(self.rt["GetPositionIK"], service)
        self._wait_for_client(client, float(self.section.get("wait_service_sec", 8.0)))
        if bool(self.section.get("use_current_state", True)):
            self._spin_some(0.25)
        req = self.rt["GetPositionIK"].Request()
        req.ik_request.group_name = cfg["group"]
        req.ik_request.ik_link_name = cfg["tip_link"]
        req.ik_request.pose_stamped = self._pose_stamped(sample)
        req.ik_request.robot_state = self._robot_state()
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = self._duration_msg(timeout_sec)
        start = time.monotonic()
        future = client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec + 1.0)
        elapsed_ms = (time.monotonic() - start) * 1000.0
        if not future.done():
            raise TimeoutError(f"{service} timed out after {timeout_sec + 1.0:.2f}s")
        return future.result(), elapsed_ms


class MoveItIKChecker(MoveItBaseChecker):
    name = "moveit_ik"

    def __init__(self, cfg: dict[str, Any]):
        super().__init__(cfg, "moveit_ik", "reachability_moveit_ik_probe")
        self.service = str(self.section.get("service", DEFAULT_IK_SERVICE))
        self.timeout_sec = float(self.section.get("timeout_sec", 2.0))
        self.avoid_collisions = bool(self.section.get("avoid_collisions", True))

    def check(self, sample: PoseSample) -> CheckResult:
        try:
            result, elapsed_ms = self._call_ik(sample, self.service, self.timeout_sec, self.avoid_collisions)
            code = int(result.error_code.val)
            ok = code == 1
            return CheckResult(
                ok=ok,
                reason=MOVEIT_ERROR_NAMES.get(code, f"MoveItErrorCode({code})"),
                elapsed_ms=elapsed_ms,
                details={"error_code": code, "service": self.service},
            )
        except Exception as exc:
            return CheckResult(ok=False, reason=type(exc).__name__, details={"error": str(exc)})


class MoveItPlanChecker(MoveItBaseChecker):
    def __init__(self, cfg: dict[str, Any], planner_id: str | None = None):
        suffix = planner_id or "default"
        self.name = f"moveit_plan:{suffix}"
        super().__init__(cfg, "moveit_plan", f"reachability_moveit_plan_probe_{suffix.lower()}")
        self.ik_service = str(self.section.get("ik_service", DEFAULT_IK_SERVICE))
        self.plan_service = str(self.section.get("plan_service", DEFAULT_PLAN_SERVICE))
        self.ik_timeout_sec = float(self.section.get("ik_timeout_sec", 2.0))
        self.planning_time_sec = float(self.section.get("planning_time_sec", 3.0))
        self.attempts = int(self.section.get("attempts", 3))
        self.avoid_collisions = bool(self.section.get("avoid_collisions", True))
        self.velocity_scaling = float(self.section.get("velocity_scaling", 0.05))
        self.acceleration_scaling = float(self.section.get("acceleration_scaling", 0.05))
        self.planner_id = planner_id or str(self.section.get("planner_id", ""))
        self.plan_client = self._client(self.rt["GetMotionPlan"], self.plan_service)
        self._wait_for_client(self.plan_client, float(self.section.get("wait_service_sec", 8.0)))

    def _make_joint_goal_constraints(self, joint_state: Any, joint_names: list[str]) -> Any:
        constraints = self.rt["Constraints"]()
        by_name = dict(zip(joint_state.name, joint_state.position))
        for name in joint_names:
            if name not in by_name:
                continue
            jc = self.rt["JointConstraint"]()
            jc.joint_name = name
            jc.position = float(by_name[name])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def check(self, sample: PoseSample) -> CheckResult:
        start = time.monotonic()
        try:
            ik_result, ik_elapsed_ms = self._call_ik(
                sample,
                self.ik_service,
                self.ik_timeout_sec,
                self.avoid_collisions,
            )
            ik_code = int(ik_result.error_code.val)
            if ik_code != 1:
                return CheckResult(
                    ok=False,
                    reason=f"ik:{MOVEIT_ERROR_NAMES.get(ik_code, ik_code)}",
                    elapsed_ms=(time.monotonic() - start) * 1000.0,
                    details={"ik_error_code": ik_code, "ik_elapsed_ms": ik_elapsed_ms},
                )

            cfg = self._arm_config(sample.arm)
            req = self.rt["GetMotionPlan"].Request()
            mpr = self.rt["MotionPlanRequest"]()
            mpr.group_name = cfg["group"]
            if self.planner_id:
                mpr.planner_id = self.planner_id
            mpr.num_planning_attempts = self.attempts
            mpr.allowed_planning_time = self.planning_time_sec
            mpr.max_velocity_scaling_factor = self.velocity_scaling
            mpr.max_acceleration_scaling_factor = self.acceleration_scaling
            mpr.start_state = self._robot_state()
            mpr.goal_constraints.append(
                self._make_joint_goal_constraints(ik_result.solution.joint_state, list(cfg["joints"]))
            )
            req.motion_plan_request = mpr
            future = self.plan_client.call_async(req)
            self.rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=self.planning_time_sec + 2.0,
            )
            elapsed_ms = (time.monotonic() - start) * 1000.0
            if not future.done():
                return CheckResult(
                    ok=False,
                    reason="plan_timeout",
                    elapsed_ms=elapsed_ms,
                    details={"planner_id": self.planner_id, "ik_elapsed_ms": ik_elapsed_ms},
                )
            result = future.result()
            code = int(result.motion_plan_response.error_code.val)
            ok = code == 1
            traj = result.motion_plan_response.trajectory.joint_trajectory
            return CheckResult(
                ok=ok,
                reason=MOVEIT_ERROR_NAMES.get(code, f"MoveItErrorCode({code})"),
                elapsed_ms=elapsed_ms,
                details={
                    "planner_id": self.planner_id,
                    "error_code": code,
                    "ik_elapsed_ms": ik_elapsed_ms,
                    "trajectory_points": len(traj.points),
                },
            )
        except Exception as exc:
            return CheckResult(
                ok=False,
                reason=type(exc).__name__,
                elapsed_ms=(time.monotonic() - start) * 1000.0,
                details={"error": str(exc), "planner_id": self.planner_id},
            )


def load_external_checker(spec: str, cfg: dict[str, Any]) -> Checker:
    if spec.startswith("python:"):
        payload = spec[len("python:") :]
        path_text, _, class_name = payload.rpartition(":")
        if not path_text or not class_name:
            raise ValueError("python checker spec must be python:/path/to/file.py:ClassName")
        path = Path(path_text).expanduser().resolve()
        module_name = f"reachability_plugin_{path.stem}_{abs(hash(str(path)))}"
        module_spec = importlib.util.spec_from_file_location(module_name, path)
        if module_spec is None or module_spec.loader is None:
            raise ValueError(f"cannot load plugin file: {path}")
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    else:
        module_name, _, class_name = spec.rpartition(":")
        if not module_name or not class_name:
            raise ValueError(f"external checker spec must be module:ClassName, got {spec}")
        module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    checker = cls(cfg)
    if not hasattr(checker, "check"):
        raise TypeError(f"{spec} did not create an object with check(sample)")
    return checker


def checker_from_spec(spec: Any, cfg: dict[str, Any]) -> Checker:
    if isinstance(spec, dict):
        name = str(spec.get("name", "")).strip()
        merged = deep_update(cfg, {name: deep_update(cfg.get(name, {}), {k: v for k, v in spec.items() if k != "name"})})
        if name == "moveit_plan" and spec.get("planner_id"):
            return MoveItPlanChecker(merged, str(spec["planner_id"]))
        return checker_from_spec(name, merged)
    spec_text = str(spec).strip()
    if spec_text == "geometry":
        return GeometryReachChecker(cfg)
    if spec_text == "moveit_ik":
        return MoveItIKChecker(cfg)
    if spec_text == "moveit_plan":
        return MoveItPlanChecker(cfg)
    if spec_text.startswith("moveit_plan:"):
        return MoveItPlanChecker(cfg, spec_text.split(":", 1)[1])
    if spec_text.startswith("python:") or ":" in spec_text:
        return load_external_checker(spec_text, cfg)
    raise ValueError(f"unknown checker spec: {spec_text}")


def result_as_dict(result: Any) -> dict[str, Any]:
    if hasattr(result, "as_dict"):
        data = result.as_dict()
    elif isinstance(result, dict):
        data = dict(result)
    else:
        data = {
            "ok": bool(getattr(result, "ok")),
            "reason": str(getattr(result, "reason", "")),
            "details": dict(getattr(result, "details", {})),
        }
        if hasattr(result, "elapsed_ms"):
            data["elapsed_ms"] = float(getattr(result, "elapsed_ms"))
        if hasattr(result, "score") and getattr(result, "score") is not None:
            data["score"] = getattr(result, "score")
    if "ok" not in data:
        raise ValueError(f"checker result must contain ok: {result!r}")
    data["ok"] = bool(data["ok"])
    data.setdefault("reason", "")
    data.setdefault("details", {})
    data.setdefault("elapsed_ms", 0.0)
    return data


def checker_specs_from_args(args: argparse.Namespace, cfg: dict[str, Any]) -> list[Any]:
    specs: list[Any] = []
    for item in args.checker or []:
        specs.append(item)
    if args.checker_set:
        for name in args.checker_set:
            if name == "ompl":
                specs.extend(OMPL_CHECKER_SET)
            elif name == "moveit":
                specs.extend(["moveit_ik", "moveit_plan:RRTConnectkConfigDefault"])
            elif name == "geometry":
                specs.append("geometry")
            else:
                raise ValueError(f"unknown checker set: {name}")
    if not specs:
        specs = list(cfg.get("checkers") or [{"name": "geometry"}])
    return specs


def build_region_samplers(cfg: dict[str, Any], names: list[str] | None = None) -> list[RegionSampler]:
    regions = cfg.get("regions")
    if not isinstance(regions, dict) or not regions:
        raise ValueError("config must define at least one region")
    selected = names or list(regions.keys())
    missing = [name for name in selected if name not in regions]
    if missing:
        raise ValueError(f"unknown region(s): {', '.join(missing)}")
    samplers = [RegionSampler(name, regions[name], str(cfg.get("frame_id", DEFAULT_FRAME_ID))) for name in selected]
    if not any(s.weight > 0 for s in samplers):
        raise ValueError("at least one selected region must have positive weight")
    return samplers


def choose_sampler(samplers: list[RegionSampler], rng: random.Random) -> RegionSampler:
    total = sum(s.weight for s in samplers)
    mark = rng.uniform(0.0, total)
    acc = 0.0
    for sampler in samplers:
        acc += sampler.weight
        if mark <= acc:
            return sampler
    return samplers[-1]


def update_summary(summary: dict[str, Any], record: dict[str, Any]) -> None:
    sample = record["sample"]
    all_ok = all(result["ok"] for result in record["results"].values())
    summary["total"] += 1
    summary["all_ok"] += int(all_ok)
    region = sample["region"]
    summary["regions"][region]["total"] += 1
    summary["regions"][region]["all_ok"] += int(all_ok)
    for name, result in record["results"].items():
        checker_summary = summary["checkers"][name]
        checker_summary["total"] += 1
        checker_summary["ok"] += int(result["ok"])
        checker_summary["elapsed_ms_total"] += float(result.get("elapsed_ms", 0.0))
        checker_summary["reasons"][result.get("reason", "")] += 1
        region_checker = summary["regions"][region]["checkers"][name]
        region_checker["total"] += 1
        region_checker["ok"] += int(result["ok"])


def finalize_summary(summary: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {
        "total": summary["total"],
        "all_ok": summary["all_ok"],
        "all_ok_rate": summary["all_ok"] / summary["total"] if summary["total"] else 0.0,
        "checkers": {},
        "regions": {},
    }
    for name, data in summary["checkers"].items():
        total = data["total"]
        reasons = dict(data["reasons"].most_common(8))
        out["checkers"][name] = {
            "total": total,
            "ok": data["ok"],
            "ok_rate": data["ok"] / total if total else 0.0,
            "mean_elapsed_ms": data["elapsed_ms_total"] / total if total else 0.0,
            "top_reasons": reasons,
        }
    for name, data in summary["regions"].items():
        total = data["total"]
        out["regions"][name] = {
            "total": total,
            "all_ok": data["all_ok"],
            "all_ok_rate": data["all_ok"] / total if total else 0.0,
            "checkers": {
                checker: {
                    "total": cdata["total"],
                    "ok": cdata["ok"],
                    "ok_rate": cdata["ok"] / cdata["total"] if cdata["total"] else 0.0,
                }
                for checker, cdata in data["checkers"].items()
            },
        }
    return out


def print_summary(summary: dict[str, Any]) -> None:
    print(f"samples={summary['total']} all_ok={summary['all_ok']} all_ok_rate={summary['all_ok_rate']:.3f}")
    for name, data in summary["checkers"].items():
        print(
            f"checker={name} ok={data['ok']}/{data['total']} "
            f"rate={data['ok_rate']:.3f} mean_ms={data['mean_elapsed_ms']:.1f}"
        )
        if data["top_reasons"]:
            reasons = ", ".join(f"{reason}:{count}" for reason, count in data["top_reasons"].items())
            print(f"  reasons: {reasons}")
    for name, data in summary["regions"].items():
        print(f"region={name} all_ok={data['all_ok']}/{data['total']} rate={data['all_ok_rate']:.3f}")


def run_probe(args: argparse.Namespace) -> dict[str, Any]:
    cfg = load_config(Path(args.config).expanduser() if args.config else None)
    if args.seed is not None:
        cfg["seed"] = args.seed
    if args.urdf:
        cfg["urdf"] = args.urdf
    count = int(args.count if args.count is not None else cfg.get("count", 100))
    if count <= 0:
        raise ValueError("--count must be > 0")
    rng = random.Random(int(cfg.get("seed", 7)))
    samplers = build_region_samplers(cfg, args.region)
    sampling_cfg = cfg.get("sampling", {})
    if not isinstance(sampling_cfg, dict):
        raise ValueError("sampling config must be a mapping/object")
    reach_constraint = None
    if config_bool(sampling_cfg.get("enforce_arm_reach"), True):
        reach_constraint = ArmReachSamplingConstraint(cfg)
    max_resample_attempts = int(sampling_cfg.get("max_resample_attempts", 1000))
    if max_resample_attempts <= 0:
        raise ValueError("sampling.max_resample_attempts must be > 0")
    checker_specs = checker_specs_from_args(args, cfg)
    checkers = [checker_from_spec(spec, cfg) for spec in checker_specs]
    if not checkers:
        raise ValueError("at least one checker is required")

    summary = {
        "total": 0,
        "all_ok": 0,
        "checkers": defaultdict(lambda: {"total": 0, "ok": 0, "elapsed_ms_total": 0.0, "reasons": Counter()}),
        "regions": defaultdict(
            lambda: {
                "total": 0,
                "all_ok": 0,
                "checkers": defaultdict(lambda: {"total": 0, "ok": 0}),
            }
        ),
    }
    output_handle = None
    if args.output:
        output_path = Path(args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_handle = output_path.open("w", encoding="utf-8")
    try:
        sample_id = 0
        if args.per_region:
            sample_plan = []
            for sampler in samplers:
                sample_plan.extend([sampler] * count)
        else:
            sample_plan = [choose_sampler(samplers, rng) for _ in range(count)]
        for sampler in sample_plan:
            sample = sample_with_constraints(
                sampler,
                sample_id,
                rng,
                reach_constraint,
                max_resample_attempts,
            )
            sample_id += 1
            results = {}
            for checker in checkers:
                result = checker.check(sample)
                checker_name = str(getattr(checker, "name", checker.__class__.__name__))
                results[checker_name] = result_as_dict(result)
            record = {"sample": sample.as_dict(), "results": results}
            update_summary(summary, record)
            if output_handle:
                output_handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            if args.progress_every and sample_id % args.progress_every == 0:
                done = finalize_summary(summary)
                print(f"progress {sample_id}/{len(sample_plan)} all_ok_rate={done['all_ok_rate']:.3f}", file=sys.stderr)
    finally:
        if output_handle:
            output_handle.close()
    final = finalize_summary(summary)
    if args.summary:
        summary_path = Path(args.summary).expanduser()
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(final, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    return final


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate random target poses in configured regions and test reachability with pluggable checkers.",
    )
    parser.add_argument("--config", help="YAML/JSON region and checker config. Defaults to built-in UnoArm regions.")
    parser.add_argument("--count", type=int, help="Number of samples. With --per-region this is samples per region.")
    parser.add_argument("--seed", type=int, help="Random seed.")
    parser.add_argument("--region", action="append", help="Region name to sample. Can be repeated.")
    parser.add_argument("--per-region", action="store_true", help="Generate --count samples for each selected region.")
    parser.add_argument("--checker", action="append", help="Checker spec: geometry, moveit_ik, moveit_plan[:planner], module:Class, or python:/path.py:Class.")
    parser.add_argument("--checker-set", action="append", choices=["geometry", "moveit", "ompl"], help="Convenience checker set.")
    parser.add_argument("--urdf", help=f"URDF path for geometry checker. Default: {DEFAULT_URDF}")
    parser.add_argument("--output", help="Write per-sample JSONL results.")
    parser.add_argument("--summary", help="Write summary JSON.")
    parser.add_argument("--progress-every", type=int, default=0, help="Print progress every N samples to stderr.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        summary = run_probe(args)
    except Exception as exc:
        print(f"reachability_probe: error: {exc}", file=sys.stderr)
        return 2
    print_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
