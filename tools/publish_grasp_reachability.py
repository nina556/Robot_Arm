#!/usr/bin/env python3
"""Publish grasp benchmark/fitted-solid outputs for the web scene overlay."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PUBLIC = ROOT / "web_control" / "frontend" / "public" / "models"
DEFAULT_DIST = ROOT / "web_control" / "frontend" / "dist" / "models"


LABELS = {
    "angled_grasp": "斜向抓取",
    "side_grasp": "侧面抓取",
    "vertical_grasp": "垂直抓取",
}


def parse_xyz(raw: str, label: str) -> list[float]:
    values = [float(item.strip()) for item in raw.split(",")]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must be three finite comma-separated numbers")
    return values


def truthy(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok"}


def finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def read_mode_rows(records: Path, mode: str) -> list[dict[str, str]]:
    with records.open(encoding="utf-8", newline="") as handle:
        rows = [row for row in csv.DictReader(handle) if row.get("mode") == mode]
    if not rows:
        raise ValueError(f"no {mode} rows found in {records}")
    return rows


def write_mode_csv(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def points(rows: list[dict[str, str]], ok: bool) -> list[list[float]]:
    out: list[list[float]] = []
    for row in rows:
        if truthy(row.get("ik_ok")) is not ok:
            continue
        xyz = [finite_float(row.get(axis)) for axis in ("x", "y", "z")]
        if all(value is not None for value in xyz):
            out.append([round(float(value), 6) for value in xyz])
    return out


def average(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def stats(rows: list[dict[str, str]], fit_summary: dict[str, Any]) -> dict[str, Any]:
    total = len(rows)
    ik_ok = sum(1 for row in rows if truthy(row.get("ik_ok")))
    plan_ok = sum(1 for row in rows if truthy(row.get("plan_ok")))
    ik_times = [value for row in rows if (value := finite_float(row.get("ik_elapsed_ms"))) is not None]
    ompl_times = [
        value
        for row in rows
        if truthy(row.get("plan_ok")) and (value := finite_float(row.get("ompl_elapsed_ms"))) is not None
    ]
    return {
        "total": total,
        "ik_ok": ik_ok,
        "ik_failed": total - ik_ok,
        "plan_ok": plan_ok,
        "ik_rate": ik_ok / total if total else 0.0,
        "plan_rate": plan_ok / total if total else 0.0,
        "mean_ik_ms": average(ik_times),
        "mean_ompl_ms": average(ompl_times),
        "fitted_volume_m3": fit_summary.get("fitted_volume_m3"),
        "fitted_volume_liters": fit_summary.get("fitted_volume_liters"),
        "workspace_share": fit_summary.get("workspace_share"),
        "fit": fit_summary,
    }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")


def publish_to_dir(target_dir: Path, prefix: str, payload: dict[str, Any], obj: Path, html: Path | None) -> None:
    target_dir.mkdir(parents=True, exist_ok=True)
    write_json(target_dir / f"{prefix}_workspace_reachability.json", payload)
    shutil.copy2(obj, target_dir / f"{prefix}_workspace_reachability.obj")
    if html and html.exists():
        shutil.copy2(html, target_dir / f"{prefix}_solid_smooth.html")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--mode", required=True)
    parser.add_argument("--arm", choices=["right", "left"], required=True)
    parser.add_argument("--region-min", required=True)
    parser.add_argument("--region-max", required=True)
    parser.add_argument("--fit-summary", type=Path, required=True)
    parser.add_argument("--obj", type=Path, required=True)
    parser.add_argument("--html", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None, help="also write a mode-filtered CSV here")
    parser.add_argument("--public-dir", type=Path, default=DEFAULT_PUBLIC)
    parser.add_argument("--dist-dir", type=Path, default=DEFAULT_DIST)
    args = parser.parse_args()

    rows = read_mode_rows(args.records, args.mode)
    if args.output_dir:
        write_mode_csv(args.output_dir / f"{args.mode}_records.csv", rows)

    fit_summary = json.loads(args.fit_summary.read_text(encoding="utf-8"))
    prefix = f"{args.arm}_{args.mode}" if args.arm == "left" else args.mode
    payload = {
        "mode": args.mode,
        "arm": args.arm,
        "label": LABELS.get(args.mode, args.mode),
        "region": {
            "min": parse_xyz(args.region_min, "--region-min"),
            "max": parse_xyz(args.region_max, "--region-max"),
        },
        "points": {
            "ik_success": points(rows, True),
            "ik_failure": points(rows, False),
        },
        "stats": stats(rows, fit_summary),
    }
    publish_to_dir(args.public_dir, prefix, payload, args.obj, args.html)
    publish_to_dir(args.dist_dir, prefix, payload, args.obj, args.html)
    print(json.dumps({
        "ok": True,
        "mode": args.mode,
        "arm": args.arm,
        "prefix": prefix,
        "total": payload["stats"]["total"],
        "ik_rate": payload["stats"]["ik_rate"],
        "plan_rate": payload["stats"]["plan_rate"],
        "fitted_volume_m3": payload["stats"]["fitted_volume_m3"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
