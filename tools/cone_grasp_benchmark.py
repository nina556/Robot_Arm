#!/usr/bin/env python3
"""Batch benchmark cone/stator grasp IK and optional OMPL planning poses.

The script talks to the running web_control API, so it uses the same MoveIt,
workspace bounds, manual collision boxes, and MuJoCo table constraints as the
current simulation stack.  It intentionally does not call execute endpoints.
For IK-only runs it still uses the server-side grasp candidate chains for each
grasp_orientation_mode.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = ROOT / "logs" / "cone_grasp_benchmark"

MODES = {
    "angled_grasp": "斜向抓取",
    "side_grasp": "侧面抓取",
    "vertical_grasp": "垂直抓取",
}

AXIS_SWEEP_MODES = {
    "axis_x_down": "TCP X 轴向下",
    "axis_x_up": "TCP X 轴向上",
    "axis_y_down": "TCP Y 轴向下",
    "axis_y_up": "TCP Y 轴向上",
    "axis_z_down": "TCP Z 轴向下",
    "axis_z_up": "TCP Z 轴向上",
}

MODE_Z_OFFSET_M = {
    "side_grasp": 0.035,
    "angled_grasp": 0.065,
    "vertical_grasp": 0.105,
}


def mode_label(mode: str) -> str:
    return MODES.get(mode) or AXIS_SWEEP_MODES.get(mode) or mode

CSV_COLUMNS = [
    "run_id",
    "mode",
    "mode_label",
    "seed",
    "sample_id",
    "arm",
    "x",
    "y",
    "z",
    "ik_ok",
    "ik_reason",
    "ik_error_code",
    "ik_elapsed_ms",
    "ik_mode",
    "ik_candidate_index",
    "ik_candidate_count",
    "ik_scanned_count",
    "ik_candidate",
    "joint_names",
    "joint_positions",
    "joint_by_name",
    "planner_id",
    "plan_ok",
    "plan_reason",
    "plan_error_code",
    "ompl_elapsed_ms",
    "trajectory_points",
    "trajectory_duration",
    "sample_metadata",
]


def api_json(base_url: str, method: str, path: str, body: dict[str, Any] | None = None, timeout: float = 30.0) -> dict[str, Any]:
    url = base_url.rstrip("/") + path
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        headers["Content-Type"] = "application/json; charset=utf-8"
    req = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(req, timeout=timeout) as resp:
            payload = resp.read()
    except HTTPError as exc:
        payload = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{method} {path} failed: HTTP {exc.code}: {payload}") from exc
    except URLError as exc:
        raise RuntimeError(f"{method} {path} failed: {exc}") from exc
    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"{method} {path} returned non-JSON payload: {payload[:200]!r}") from exc


def finite_float(value: Any, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def load_scene_config() -> dict[str, Any]:
    path = ROOT / "asm0003" / "mujoco" / "scene_config.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def default_table_info(scene: dict[str, Any]) -> dict[str, Any]:
    table_center = [
        finite_float(scene.get("table_center_x"), 0.0),
        finite_float(scene.get("table_center_y"), -0.72),
        finite_float(scene.get("table_center_z"), 0.33),
    ]
    table_size = [
        finite_float(scene.get("table_x"), 0.40),
        finite_float(scene.get("table_y"), 0.30),
        finite_float(scene.get("table_z"), 1.00),
    ]
    object_size = [
        finite_float(scene.get("object_size_x"), 0.04),
        finite_float(scene.get("object_size_y"), 0.04),
        finite_float(scene.get("object_size_z"), 0.027),
    ]
    table_top = table_center[2] + table_size[2] * 0.5
    object_center_z = table_top + object_size[2] * 0.5 + 0.002
    return {
        "table_center": table_center,
        "table_size": table_size,
        "table_top": table_top,
        "object_size": object_size,
        "object_center_z": object_center_z,
    }


def clamp_range(lo: float, hi: float, min_value: float, max_value: float, label: str) -> tuple[float, float]:
    lo = max(lo, min_value)
    hi = min(hi, max_value)
    if hi < lo:
        raise ValueError(f"{label} has no intersection: [{lo:.3f}, {hi:.3f}]")
    return lo, hi


def sample_region_from_environment(options: dict[str, Any], table: dict[str, Any], arm: str, margin_m: float) -> dict[str, list[float]]:
    bounds = options.get("workspace_bounds") or {}
    if not bounds.get("enabled"):
        bounds = {
            "min": [-0.5, -0.7, 0.6] if arm == "right" else [-0.02, -0.7, 0.6],
            "max": [0.02, -0.2, 1.1] if arm == "right" else [0.5, -0.2, 1.1],
        }
    bmin = [float(v) for v in bounds["min"]]
    bmax = [float(v) for v in bounds["max"]]
    tc = table["table_center"]
    ts = table["table_size"]
    x_lo, x_hi = clamp_range(tc[0] - ts[0] * 0.5 + margin_m, tc[0] + ts[0] * 0.5 - margin_m, bmin[0], bmax[0], "x")
    y_lo, y_hi = clamp_range(tc[1] - ts[1] * 0.5 + margin_m, tc[1] + ts[1] * 0.5 - margin_m, bmin[1], bmax[1], "y")
    z_lo, z_hi = clamp_range(table["object_center_z"] + 0.02, table["object_center_z"] + 0.14, bmin[2], bmax[2], "z")
    return {"x": [x_lo, x_hi], "y": [y_lo, y_hi], "z": [z_lo, z_hi]}


def vec_dot(a: list[float], b: list[float]) -> float:
    return sum(float(x) * float(y) for x, y in zip(a, b))


def vec_cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def vec_norm(v: list[float]) -> float:
    return math.sqrt(max(0.0, vec_dot(v, v)))


def vec_normalize(v: list[float]) -> list[float]:
    norm = vec_norm(v)
    if norm <= 1e-9:
        raise ValueError(f"cannot normalize near-zero vector {v!r}")
    return [float(x) / norm for x in v]


def quat_from_matrix(matrix: list[list[float]]) -> dict[str, float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return {"x": qx / norm, "y": qy / norm, "z": qz / norm, "w": qw / norm}


def axis_sweep_orientation(mode: str, spin_rad: float) -> dict[str, float]:
    _, axis, direction = mode.split("_", 2)
    target = [0.0, 0.0, -1.0 if direction == "down" else 1.0]
    helper = [1.0, 0.0, 0.0]
    if abs(vec_dot(target, helper)) > 0.9:
        helper = [0.0, 1.0, 0.0]
    u = vec_normalize(vec_cross(target, helper))
    v = vec_normalize(vec_cross(target, u))
    spin_axis = vec_normalize([
        math.cos(spin_rad) * u[i] + math.sin(spin_rad) * v[i]
        for i in range(3)
    ])
    if axis == "x":
        x_axis = target
        y_axis = spin_axis
        z_axis = vec_normalize(vec_cross(x_axis, y_axis))
    elif axis == "y":
        y_axis = target
        z_axis = spin_axis
        x_axis = vec_normalize(vec_cross(y_axis, z_axis))
    elif axis == "z":
        z_axis = target
        x_axis = spin_axis
        y_axis = vec_normalize(vec_cross(z_axis, x_axis))
    else:
        raise ValueError(f"unknown axis sweep mode: {mode}")
    rotation = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return quat_from_matrix(rotation)


def generate_axis_sweep_samples(mode: str, arm: str, seed: int, count: int, region: dict[str, list[float]], table: dict[str, Any], z_jitter_m: float, full_region_z: bool = False) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    samples = []
    base_z = table["object_center_z"] + 0.075
    if full_region_z:
        z_min, z_max = region["z"]
    else:
        z_min = max(region["z"][0], base_z - abs(z_jitter_m))
        z_max = min(region["z"][1], base_z + abs(z_jitter_m))
        if z_max < z_min:
            z_min, z_max = region["z"]
    for index in range(1, count + 1):
        spin = rng.uniform(-math.pi, math.pi)
        x = rng.uniform(*region["x"])
        y = rng.uniform(*region["y"])
        z = rng.uniform(z_min, z_max)
        samples.append({
            "id": index,
            "arm": arm,
            "x": round(x, 6),
            "y": round(y, 6),
            "z": round(z, 6),
            "orientation": axis_sweep_orientation(mode, spin),
            "source": "axis_sweep",
            "metadata": {
                "seed": seed,
                "mode": mode,
                "mode_label": mode_label(mode),
                "spin_rad": round(spin, 6),
                "tcp_axis_test": mode,
                "tcp_link": "Right_Gripper_TCP" if arm == "right" else "Left_Gripper_TCP",
                "tcp_origin_in_parent": [0.34, 0.0, 0.0],
                "sample_region": region,
            },
        })
    return samples


def generate_samples(mode: str, arm: str, seed: int, count: int, region: dict[str, list[float]], table: dict[str, Any], z_jitter_m: float, full_region_z: bool = False) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    samples = []
    base_z = table["object_center_z"] + MODE_Z_OFFSET_M.get(mode, 0.06)
    if full_region_z:
        z_min, z_max = region["z"]
    else:
        z_min = max(region["z"][0], base_z - abs(z_jitter_m))
        z_max = min(region["z"][1], base_z + abs(z_jitter_m))
        if z_max < z_min:
            z_min, z_max = region["z"]
    for index in range(1, count + 1):
        x = rng.uniform(*region["x"])
        y = rng.uniform(*region["y"])
        z = rng.uniform(z_min, z_max)
        samples.append({
            "id": index,
            "arm": arm,
            "x": round(x, 6),
            "y": round(y, 6),
            "z": round(z, 6),
            "grasp_orientation_mode": mode,
            "source": "cone_table_seed",
            "metadata": {
                "seed": seed,
                "mode": mode,
                "mode_label": mode_label(mode),
                "table_top": round(table["table_top"], 6),
                "object_center_z": round(table["object_center_z"], 6),
                "sample_region": region,
                "full_region_z": bool(full_region_z),
            },
        })
    return samples


def rows_from_result(run_id: str, mode: str, seed: int, result: dict[str, Any]) -> list[dict[str, Any]]:
    plans_by_sample: dict[int, list[tuple[str, dict[str, Any]]]] = {}
    for planner_id, rows in (result.get("results") or {}).items():
        if not isinstance(rows, list):
            continue
        for plan in rows:
            plans_by_sample.setdefault(int(plan.get("sample_id", 0)), []).append((str(planner_id), plan))

    output = []
    for sample in result.get("samples") or []:
        if not isinstance(sample, dict):
            continue
        sample_id = int(sample.get("id", 0))
        ik = sample.get("ik") if isinstance(sample.get("ik"), dict) else {}
        plan_rows = plans_by_sample.get(sample_id) or [("", {})]
        for planner_id, plan in plan_rows:
            output.append({
                "run_id": run_id,
                "mode": mode,
                "mode_label": mode_label(mode),
                "seed": seed,
                "sample_id": sample_id,
                "arm": sample.get("arm"),
                "x": sample.get("x"),
                "y": sample.get("y"),
                "z": sample.get("z"),
                "ik_ok": bool(ik.get("ok")),
                "ik_reason": ik.get("reason"),
                "ik_error_code": ik.get("error_code"),
                "ik_elapsed_ms": ik.get("elapsed_ms"),
                "ik_mode": ik.get("mode"),
                "ik_candidate_index": ik.get("candidate_index"),
                "ik_candidate_count": ik.get("candidate_count"),
                "ik_scanned_count": ik.get("scanned_count"),
                "ik_candidate": json.dumps(ik.get("candidate"), ensure_ascii=False, separators=(",", ":")) if ik.get("candidate") is not None else "",
                "joint_names": json.dumps(ik.get("joint_names") or [], ensure_ascii=False, separators=(",", ":")),
                "joint_positions": json.dumps(ik.get("joint_positions") or [], ensure_ascii=False, separators=(",", ":")),
                "joint_by_name": json.dumps(ik.get("joint_by_name") or {}, ensure_ascii=False, separators=(",", ":")),
                "planner_id": planner_id,
                "plan_ok": bool(plan.get("ok")) if plan else False,
                "plan_reason": plan.get("reason") if plan else "",
                "plan_error_code": plan.get("error_code") if plan else "",
                "ompl_elapsed_ms": plan.get("elapsed_ms") if plan else "",
                "trajectory_points": plan.get("trajectory_points") if plan else "",
                "trajectory_duration": plan.get("trajectory_duration") if plan else "",
                "sample_metadata": json.dumps(sample.get("metadata") or {}, ensure_ascii=False, separators=(",", ":")),
            })
    return output


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in CSV_COLUMNS})


def write_point_csv(path: Path, rows: list[dict[str, Any]], mode: str, ik_ok: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "run_id",
        "mode",
        "mode_label",
        "seed",
        "sample_id",
        "arm",
        "x",
        "y",
        "z",
        "ik_ok",
        "ik_reason",
        "ik_elapsed_ms",
        "ik_candidate",
        "joint_names",
        "joint_positions",
        "joint_by_name",
    ]
    selected = [row for row in rows if row["mode"] == mode and bool(row.get("ik_ok")) is ik_ok]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in selected:
            writer.writerow({key: row.get(key, "") for key in columns})


def project_point(point: tuple[float, float, float], mins: tuple[float, float, float], spans: tuple[float, float, float]) -> tuple[float, float]:
    x = (point[0] - mins[0]) / spans[0] - 0.5
    y = (point[1] - mins[1]) / spans[1] - 0.5
    z = (point[2] - mins[2]) / spans[2]
    u = (x - y) * 0.866
    v = (x + y) * 0.35 - z * 0.95
    return u, v


def svg_map(projected: list[tuple[float, float]], width: int, height: int) -> tuple[float, float, float]:
    if not projected:
        return 1.0, width / 2.0, height / 2.0
    min_u = min(p[0] for p in projected)
    max_u = max(p[0] for p in projected)
    min_v = min(p[1] for p in projected)
    max_v = max(p[1] for p in projected)
    span_u = max(1e-9, max_u - min_u)
    span_v = max(1e-9, max_v - min_v)
    scale = min((width - 90) / span_u, (height - 110) / span_v)
    ox = 45 - min_u * scale
    oy = 70 - min_v * scale
    return scale, ox, oy


def write_svg(path: Path, mode: str, rows: list[dict[str, Any]], region: dict[str, list[float]], ik_only: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 980, 760
    point_rows = [row for row in rows if row["mode"] == mode]
    coords = [
        (float(row["x"]), float(row["y"]), float(row["z"]))
        for row in point_rows
        if row.get("x") not in (None, "") and row.get("y") not in (None, "") and row.get("z") not in (None, "")
    ]
    corners = [
        (x, y, z)
        for x in region["x"]
        for y in region["y"]
        for z in region["z"]
    ]
    all_coords = coords + corners
    if not all_coords:
        all_coords = [(0.0, 0.0, 0.0)]
    mins = tuple(min(p[i] for p in all_coords) for i in range(3))
    maxs = tuple(max(p[i] for p in all_coords) for i in range(3))
    spans = tuple(max(1e-6, maxs[i] - mins[i]) for i in range(3))
    projected_corners = [project_point(corner, mins, spans) for corner in corners]
    projected_points = [project_point(coord, mins, spans) for coord in coords]
    scale, ox, oy = svg_map(projected_corners + projected_points, width, height)

    def xy(point: tuple[float, float]) -> tuple[float, float]:
        return point[0] * scale + ox, point[1] * scale + oy

    edges = [(0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7), (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7)]
    ik_ok = sum(1 for row in point_rows if row.get("ik_ok"))
    plan_ok = sum(1 for row in point_rows if row.get("plan_ok"))
    title_suffix = "IK 3D 点云" if ik_only else "IK/OMPL 3D 点云"
    subtitle = (
        f"IK 成功 {ik_ok}/{len(point_rows)}；绿色=IK 成功，红色=IK 失败"
        if ik_only
        else f"IK 有效 {ik_ok}/{len(point_rows)}，OMPL 成功 {plan_ok}/{len(point_rows)}；绿色=IK+OMPL 成功，橙色=IK 可解但规划失败，红色=IK 不可解"
    )
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fbfcfe"/>',
        f'<text x="40" y="38" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#14213d">{mode_label(mode)} {title_suffix}</text>',
        f'<text x="40" y="64" font-family="Arial, sans-serif" font-size="14" fill="#53616f">{subtitle}</text>',
    ]
    corner_xy = [xy(point) for point in projected_corners]
    for a, b in edges:
        x1, y1 = corner_xy[a]
        x2, y2 = corner_xy[b]
        svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#9aa7b5" stroke-width="1.2" stroke-dasharray="5 4"/>')
    for row in sorted(point_rows, key=lambda item: (bool(item.get("ik_ok")), bool(item.get("plan_ok")), float(item.get("z") or 0))):
        coord = (float(row["x"]), float(row["y"]), float(row["z"]))
        x, y = xy(project_point(coord, mins, spans))
        if ik_only and row.get("ik_ok"):
            color = "#16a34a"
            radius = 3.8
            opacity = 0.82
        elif row.get("plan_ok"):
            color = "#16a34a"
            radius = 4.2
            opacity = 0.88
        elif row.get("ik_ok"):
            color = "#f97316"
            radius = 3.8
            opacity = 0.82
        else:
            color = "#dc2626"
            radius = 2.6
            opacity = 0.32
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius:.1f}" fill="{color}" fill-opacity="{opacity}"/>')
    legend_y = height - 80
    legend_items = (
        (("#16a34a", "IK 成功"), ("#dc2626", "IK 失败"))
        if ik_only
        else (("#16a34a", "IK+OMPL 成功"), ("#f97316", "IK 可解"), ("#dc2626", "IK 失败"))
    )
    for i, (color, text) in enumerate(legend_items):
        x = 44 + i * 180
        svg.append(f'<circle cx="{x}" cy="{legend_y}" r="6" fill="{color}"/>')
        svg.append(f'<text x="{x + 14}" y="{legend_y + 5}" font-family="Arial, sans-serif" font-size="14" fill="#334155">{text}</text>')
    svg.append(f'<text x="40" y="{height - 34}" font-family="Arial, sans-serif" font-size="12" fill="#64748b">工作区 x[{region["x"][0]:.3f},{region["x"][1]:.3f}] y[{region["y"][0]:.3f},{region["y"][1]:.3f}] z[{region["z"][0]:.3f},{region["z"][1]:.3f}]，坐标单位: m</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n", encoding="utf-8")


def summarize(rows: list[dict[str, Any]], modes: list[str]) -> dict[str, dict[str, Any]]:
    summary = {}
    for mode in modes:
        selected = [row for row in rows if row["mode"] == mode]
        ik_times = [float(row["ik_elapsed_ms"]) for row in selected if row.get("ik_ok") and row.get("ik_elapsed_ms") not in ("", None)]
        plan_times = [float(row["ompl_elapsed_ms"]) for row in selected if row.get("plan_ok") and row.get("ompl_elapsed_ms") not in ("", None)]
        summary[mode] = {
            "label": mode_label(mode),
            "total": len(selected),
            "ik_ok": sum(1 for row in selected if row.get("ik_ok")),
            "plan_ok": sum(1 for row in selected if row.get("plan_ok")),
            "ik_success_rate": round(sum(1 for row in selected if row.get("ik_ok")) / len(selected), 4) if selected else 0.0,
            "mean_ik_ms": round(sum(ik_times) / len(ik_times), 1) if ik_times else None,
            "mean_ompl_ms": round(sum(plan_times) / len(plan_times), 1) if plan_times else None,
        }
    return summary


def write_summary(path: Path, rows: list[dict[str, Any]], modes: list[str], region: dict[str, list[float]], files: dict[str, str], ik_only: bool = False) -> None:
    data = summarize(rows, modes)
    lines = [
        "# Cone grasp benchmark summary",
        "",
        f"- Generated at: {datetime.now().isoformat(timespec='seconds')}",
        f"- Mode: {'IK-only' if ik_only else 'IK + OMPL'}",
        f"- Sample region: x={region['x']}, y={region['y']}, z={region['z']}",
        f"- CSV: {files['csv']}",
        "",
    ]
    if ik_only:
        lines.extend([
            "| Mode | Samples | IK ok | IK success rate | Mean IK ms | SVG | Success CSV | Failure CSV |",
            "|---|---:|---:|---:|---:|---|---|---|",
        ])
    else:
        lines.extend([
            "| Mode | Samples | IK ok | OMPL ok | Mean IK ms | Mean OMPL ms | SVG |",
            "|---|---:|---:|---:|---:|---:|---|",
        ])
    for mode in modes:
        item = data[mode]
        if ik_only:
            lines.append(
                f"| {item['label']} | {item['total']} | {item['ik_ok']} | {item['ik_success_rate']:.1%} | "
                f"{item['mean_ik_ms'] if item['mean_ik_ms'] is not None else ''} | {files.get(mode, '')} | "
                f"{files.get(mode + '_success_csv', '')} | {files.get(mode + '_failure_csv', '')} |"
            )
        else:
            lines.append(
                f"| {item['label']} | {item['total']} | {item['ik_ok']} | {item['plan_ok']} | "
                f"{item['mean_ik_ms'] if item['mean_ik_ms'] is not None else ''} | "
                f"{item['mean_ompl_ms'] if item['mean_ompl_ms'] is not None else ''} | {files.get(mode, '')} |"
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_modes(raw_modes: list[str] | None) -> list[str]:
    if not raw_modes:
        return list(MODES)
    modes = []
    aliases = {
        "angled": "angled_grasp",
        "side": "side_grasp",
        "vertical": "vertical_grasp",
        "斜向": "angled_grasp",
        "侧面": "side_grasp",
        "垂直": "vertical_grasp",
    }
    for item in raw_modes:
        mode = aliases.get(item, item)
        if mode not in MODES and mode not in AXIS_SWEEP_MODES:
            raise ValueError(f"unknown mode {item!r}; choose from {', '.join(list(MODES) + list(AXIS_SWEEP_MODES))}")
        modes.append(mode)
    return list(dict.fromkeys(modes))


def parse_xyz_triplet(raw: str | None, label: str) -> list[float] | None:
    if raw is None or str(raw).strip() == "":
        return None
    parts = [part.strip() for part in str(raw).replace(";", ",").split(",")]
    if len(parts) != 3:
        raise ValueError(f"{label} must contain exactly three comma-separated numbers, got {raw!r}")
    values = [float(part) for part in parts]
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains non-finite values: {raw!r}")
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run cone grasp IK/OMPL benchmark and write 3D point-cloud SVGs.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8765", help="web_control base URL")
    parser.add_argument("--arm", default="right", choices=["right", "left"], help="arm to benchmark")
    parser.add_argument("--mode", action="append", help="grasp mode: angled_grasp/side_grasp/vertical_grasp")
    parser.add_argument("--axis-sweep", action="store_true", help="test six explicit TCP axis vertical definitions instead of built-in grasp modes")
    parser.add_argument("--seed-start", type=int, default=1001, help="first task seed")
    parser.add_argument("--samples-per-mode", type=int, default=None, help="fixed sample count per mode; disables early stopping")
    parser.add_argument("--runs-per-mode", type=int, default=4, help="seeded runs per mode")
    parser.add_argument("--samples-per-run", type=int, default=30, help="samples per seeded run; web server max is usually 120")
    parser.add_argument("--target-ik-per-mode", type=int, default=60, help="stop each mode once this many IK-valid samples are collected")
    parser.add_argument("--ik-only", action="store_true", help="run only grasp IK checks and skip all OMPL planning")
    parser.add_argument("--planner", default="RRTConnectkConfigDefault", help="OMPL planner id")
    parser.add_argument("--planning-time", type=float, default=1.0, help="allowed OMPL planning time per target")
    parser.add_argument("--attempts", type=int, default=3, help="OMPL attempts")
    parser.add_argument("--ik-timeout", type=float, default=0.6, help="IK timeout per candidate")
    parser.add_argument("--max-seconds", type=float, default=1800.0, help="overall wall-clock stop limit")
    parser.add_argument("--table-margin-m", type=float, default=0.02, help="margin inside the table footprint")
    parser.add_argument("--region-min", default=None, help="explicit sample region min as x,y,z; overrides environment/table intersection when paired with --region-max")
    parser.add_argument("--region-max", default=None, help="explicit sample region max as x,y,z; overrides environment/table intersection when paired with --region-min")
    parser.add_argument("--full-region-z", action="store_true", help="sample z uniformly across the whole region instead of the mode-specific grasp height window")
    parser.add_argument("--z-jitter-m", type=float, default=0.025, help="random z jitter around each mode's nominal grasp height")
    parser.add_argument("--output-dir", type=Path, default=None, help="output directory; defaults to logs/cone_grasp_benchmark/<timestamp>")
    parser.add_argument("--api-timeout", type=float, default=600.0, help="HTTP timeout for each benchmark run")
    args = parser.parse_args(argv)

    modes = list(AXIS_SWEEP_MODES) if args.axis_sweep else parse_modes(args.mode)
    output_dir = args.output_dir or DEFAULT_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)

    options = api_json(args.base_url, "GET", f"/api/benchmark/options?{urlencode({'arm': args.arm})}", timeout=10.0)
    scene = load_scene_config()
    table = default_table_info(scene)
    region_min = parse_xyz_triplet(args.region_min, "--region-min")
    region_max = parse_xyz_triplet(args.region_max, "--region-max")
    if (region_min is None) != (region_max is None):
        raise ValueError("--region-min and --region-max must be provided together")
    if region_min is not None and region_max is not None:
        if any(hi < lo for lo, hi in zip(region_min, region_max)):
            raise ValueError(f"invalid explicit region: min={region_min}, max={region_max}")
        region = {"x": [region_min[0], region_max[0]], "y": [region_min[1], region_max[1]], "z": [region_min[2], region_max[2]]}
    else:
        region = sample_region_from_environment(options, table, args.arm, args.table_margin_m)

    all_rows: list[dict[str, Any]] = []
    full_results: list[dict[str, Any]] = []
    started = time.monotonic()

    for mode_index, mode in enumerate(modes):
        mode_ik_ok = 0
        mode_total = 0
        runs_for_mode = (
            int(math.ceil(args.samples_per_mode / args.samples_per_run))
            if args.samples_per_mode is not None
            else args.runs_per_mode
        )
        for run_index in range(runs_for_mode):
            if time.monotonic() - started > args.max_seconds:
                print(f"stop: reached --max-seconds={args.max_seconds}", file=sys.stderr)
                break
            seed = args.seed_start + mode_index * 10_000 + run_index
            run_id = f"{mode}_{seed}"
            sample_count = args.samples_per_run
            if args.samples_per_mode is not None:
                remaining = args.samples_per_mode - mode_total
                if remaining <= 0:
                    break
                sample_count = min(args.samples_per_run, remaining)
            if mode in AXIS_SWEEP_MODES:
                samples = generate_axis_sweep_samples(mode, args.arm, seed, sample_count, region, table, args.z_jitter_m, args.full_region_z)
            else:
                samples = generate_samples(mode, args.arm, seed, sample_count, region, table, args.z_jitter_m, args.full_region_z)
            body = {
                "arm": args.arm,
                "samples": samples,
                "ik_only": bool(args.ik_only),
                "planners": [] if args.ik_only else [args.planner],
                "planning_time": args.planning_time,
                "attempts": args.attempts,
                "ik_timeout": args.ik_timeout,
                "avoid_collisions": True,
            }
            print(f"running {mode_label(mode)} seed={seed} samples={len(samples)}", file=sys.stderr, flush=True)
            result = api_json(args.base_url, "POST", "/api/benchmark/run", body, timeout=args.api_timeout)
            result_path = output_dir / f"{run_id}.json"
            result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            full_results.append({"run_id": run_id, "mode": mode, "seed": seed, "path": str(result_path), "result": result})
            rows = rows_from_result(run_id, mode, seed, result)
            all_rows.extend(rows)
            mode_total = sum(1 for row in all_rows if row["mode"] == mode)
            mode_ik_ok = sum(1 for row in all_rows if row["mode"] == mode and row.get("ik_ok"))
            write_csv(output_dir / "cone_grasp_records.csv", all_rows)
            print(f"done {mode_label(mode)} seed={seed}: samples={mode_total} ik_ok={mode_ik_ok}", file=sys.stderr, flush=True)
            if args.samples_per_mode is None and args.target_ik_per_mode > 0 and mode_ik_ok >= args.target_ik_per_mode:
                break

    csv_path = output_dir / "cone_grasp_records.csv"
    write_csv(csv_path, all_rows)
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps({
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "base_url": args.base_url,
        "arm": args.arm,
        "modes": modes,
        "ik_only": bool(args.ik_only),
        "region": region,
        "table": table,
        "args": vars(args) | {"output_dir": str(output_dir)},
        "runs": [{"run_id": item["run_id"], "mode": item["mode"], "seed": item["seed"], "path": item["path"]} for item in full_results],
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    files = {"csv": str(csv_path)}
    for mode in modes:
        svg_path = output_dir / f"{mode}_3d_pointcloud.svg"
        write_svg(svg_path, mode, all_rows, region, ik_only=args.ik_only)
        files[mode] = str(svg_path)
        success_path = output_dir / f"{mode}_ik_success_points.csv"
        failure_path = output_dir / f"{mode}_ik_failure_points.csv"
        write_point_csv(success_path, all_rows, mode, True)
        write_point_csv(failure_path, all_rows, mode, False)
        files[mode + "_success_csv"] = str(success_path)
        files[mode + "_failure_csv"] = str(failure_path)
    summary_path = output_dir / "summary.md"
    write_summary(summary_path, all_rows, modes, region, files, ik_only=args.ik_only)

    print(json.dumps({
        "ok": True,
        "output_dir": str(output_dir),
        "csv": str(csv_path),
        "summary": str(summary_path),
        "svg": {mode: files[mode] for mode in modes},
        "stats": summarize(all_rows, modes),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
