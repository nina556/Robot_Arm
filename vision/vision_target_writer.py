#!/usr/bin/env python3
"""Standalone RealSense + YOLO target coordinate writer.

This script is intentionally independent from minipilot's daemon/bus system.
It reads a RealSense color+depth stream, runs an Ultralytics detection model,
computes target camera XYZ from depth, converts that point into the rough URDF
base_link frame, and appends rows to target_xyz.tsv.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import struct
import threading
import time
from collections import Counter, deque
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np


BASE_DIR = Path(__file__).resolve().parent
BOUNDARY = b"frame"
HEADER = "camera_x_m\tcamera_y_m\tcamera_z_m\turdf_x_m\turdf_y_m\turdf_z_m\n"
COLORS = (
    (52, 211, 153),
    (56, 189, 248),
    (251, 191, 36),
    (248, 113, 113),
    (192, 132, 252),
)


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int


@dataclass
class Object3D:
    object_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: list[int]
    center_pixel: list[int]
    center_camera_xyz_m: list[float]
    distance_m: float
    extent_xyz_m: list[float]
    point_count: int
    depth_band_m: float


@dataclass
class PointCloudSettings:
    min_depth_m: float
    max_depth_m: float
    object_depth_band_m: float
    point_stride: int


class VisionWebState:
    def __init__(
        self,
        args: argparse.Namespace,
        model_path: Path,
        output_path: Path,
        scene_pointcloud_path: Path,
    ) -> None:
        self.args = args
        self.started_at = time.monotonic()
        self.scene_pointcloud_path = scene_pointcloud_path
        self.stop_event = threading.Event()
        self.frame_ready = threading.Condition()
        self.state_lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.latest_cloud = np.empty((0, 6), dtype=np.float32)
        self.latest_cloud_payload: bytes | None = None
        self.scene_cloud_payload: bytes | None = None
        self.frame_sequence = 0
        self.frame_times: deque[float] = deque()
        self.settings_lock = threading.Lock()
        self.status = {
            "ready": False,
            "error": "",
            "model": model_path.name,
            "camera": f"RealSense {args.serial or 'default'}",
            "resolution": f"{args.width}x{args.height}",
            "stream_resolution": "--",
            "target_fps": args.fps,
            "stream_fps": 0.0,
            "inference_ms": 0.0,
            "frame_ms": 0.0,
            "jpeg_kb": 0.0,
            "detections": 0,
            "localized_objects": 0,
            "class_counts": {},
            "point_count": 0,
            "cloud_center_xyz_m": [0.0, 0.0, 1.0],
            "objects": [],
            "preview_detections": [],
            "pointcloud": {},
            "scene_pointcloud": self.scene_pointcloud_status(),
            "output": str(output_path),
            "latest_target": None,
            "frame_sequence": 0,
        }
        self.load_scene_pointcloud()

    def update_status(self, **values: Any) -> None:
        with self.state_lock:
            self.status.update(values)

    def update_settings(self, payload: dict) -> dict:
        with self.settings_lock:
            if "confidence" in payload:
                value = float(payload["confidence"])
                self.args.confidence = min(0.95, max(0.05, value))
            if "show_labels" in payload:
                self.args.no_labels = not bool(payload["show_labels"])
            if "min_depth_m" in payload:
                self.args.min_depth = min(3.0, max(0.05, float(payload["min_depth_m"])))
            if "max_depth_m" in payload:
                self.args.max_depth = min(10.0, max(0.2, float(payload["max_depth_m"])))
            if self.args.max_depth <= self.args.min_depth + 0.05:
                self.args.max_depth = self.args.min_depth + 0.05
            if "object_depth_band_m" in payload:
                self.args.object_depth_band = min(0.40, max(0.015, float(payload["object_depth_band_m"])))
            if "point_stride" in payload:
                self.args.point_stride = min(12, max(1, int(payload["point_stride"])))
            return {
                "confidence": self.args.confidence,
                "show_labels": not self.args.no_labels,
                "min_depth_m": self.args.min_depth,
                "max_depth_m": self.args.max_depth,
                "object_depth_band_m": self.args.object_depth_band,
                "point_stride": self.args.point_stride,
            }

    def get_status(self) -> dict:
        with self.state_lock:
            snapshot = dict(self.status)
        with self.settings_lock:
            snapshot["confidence"] = self.args.confidence
            snapshot["show_labels"] = not self.args.no_labels
            snapshot["pointcloud_settings"] = asdict(self.pointcloud_settings())
        snapshot["uptime_seconds"] = int(time.monotonic() - self.started_at)
        return snapshot

    def scene_pointcloud_status(self) -> dict:
        exists = self.scene_pointcloud_path.exists()
        return {
            "available": exists,
            "path": str(self.scene_pointcloud_path),
            "mtime": self.scene_pointcloud_path.stat().st_mtime if exists else None,
            "bytes": self.scene_pointcloud_path.stat().st_size if exists else 0,
        }

    def load_scene_pointcloud(self) -> None:
        if self.scene_pointcloud_path.exists():
            with self.frame_ready:
                self.scene_cloud_payload = self.scene_pointcloud_path.read_bytes()
            self.update_status(scene_pointcloud=self.scene_pointcloud_status())

    def pointcloud_settings(self) -> PointCloudSettings:
        return PointCloudSettings(
            min_depth_m=float(self.args.min_depth),
            max_depth_m=float(self.args.max_depth),
            object_depth_band_m=float(self.args.object_depth_band),
            point_stride=int(self.args.point_stride),
        )

    def publish_frame(
        self,
        jpeg: bytes,
        detections: list[Detection],
        names: dict[int, str],
        stream_size: tuple[int, int],
        inference_ms: float,
        frame_ms: float,
        latest_target: dict[str, list[float]] | None,
        pointcloud_payload: bytes | None = None,
        cloud: np.ndarray | None = None,
        objects: list[Object3D] | None = None,
        preview_detections: list[Detection] | None = None,
        cloud_center: list[float] | None = None,
        geometry_ms: float = 0.0,
        error: str = "",
    ) -> None:
        now = time.monotonic()
        self.frame_times.append(now)
        while self.frame_times and self.frame_times[0] < now - 2.0:
            self.frame_times.popleft()
        stream_fps = (
            (len(self.frame_times) - 1) / (self.frame_times[-1] - self.frame_times[0])
            if len(self.frame_times) > 1
            else 0.0
        )
        counts = Counter(names.get(item.cls, str(item.cls)) for item in detections)
        object_dicts = [asdict(item) for item in (objects or [])]
        preview_detection_dicts = [
            {
                "bbox_xyxy": [round(item.x1), round(item.y1), round(item.x2), round(item.y2)],
                "center_pixel": [
                    round((item.x1 + item.x2) / 2.0),
                    round((item.y1 + item.y2) / 2.0),
                ],
                "class_id": int(item.cls),
                "class_name": names.get(int(item.cls), str(item.cls)),
                "confidence": round(float(item.score), 4),
            }
            for item in (preview_detections or [])
        ]
        with self.frame_ready:
            self.latest_jpeg = jpeg
            if cloud is not None:
                self.latest_cloud = cloud
            self.latest_cloud_payload = pointcloud_payload
            self.frame_sequence += 1
            sequence = self.frame_sequence
            self.frame_ready.notify_all()
        with self.state_lock:
            self.status.update(
                {
                    "ready": not error,
                    "error": error,
                    "stream_fps": round(stream_fps, 1),
                    "inference_ms": round(inference_ms, 2),
                    "frame_ms": round(frame_ms, 2),
                    "jpeg_kb": round(len(jpeg) / 1024.0, 1),
                    "detections": len(detections),
                    "localized_objects": len(object_dicts),
                    "class_counts": dict(counts),
                    "point_count": int(len(cloud)) if cloud is not None else 0,
                    "cloud_center_xyz_m": cloud_center or [0.0, 0.0, 1.0],
                    "objects": object_dicts,
                    "preview_detections": preview_detection_dicts,
                    "geometry_ms": round(geometry_ms, 2),
                    "stream_resolution": f"{stream_size[0]}x{stream_size[1]}",
                    "latest_target": latest_target,
                    "frame_sequence": sequence,
                    "pointcloud": {
                        "available": pointcloud_payload is not None,
                        "sequence": sequence,
                    },
                }
            )

    def wait_for_frame(
        self, previous_sequence: int, timeout: float = 10.0
    ) -> tuple[int, bytes | None]:
        with self.frame_ready:
            self.frame_ready.wait_for(
                lambda: self.frame_sequence != previous_sequence
                or self.stop_event.is_set(),
                timeout=timeout,
            )
            return self.frame_sequence, self.latest_jpeg

    def snapshot(self) -> bytes | None:
        with self.frame_ready:
            return self.latest_jpeg

    def pointcloud_binary(self) -> bytes | None:
        with self.frame_ready:
            return self.latest_cloud_payload

    def scene_pointcloud_binary(self) -> bytes | None:
        with self.frame_ready:
            if self.scene_cloud_payload is not None:
                return self.scene_cloud_payload
        self.load_scene_pointcloud()
        with self.frame_ready:
            return self.scene_cloud_payload

    def save_scene_pointcloud(self) -> dict:
        with self.frame_ready:
            payload = self.latest_cloud_payload
        if payload is None:
            raise RuntimeError("No live point cloud available to save")
        self.scene_pointcloud_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.scene_pointcloud_path.with_suffix(self.scene_pointcloud_path.suffix + ".tmp")
        tmp.write_bytes(payload)
        tmp.replace(self.scene_pointcloud_path)
        with self.frame_ready:
            self.scene_cloud_payload = payload
        status = self.scene_pointcloud_status()
        self.update_status(scene_pointcloud=status)
        return status

    def pointcloud_ply(self) -> bytes:
        with self.frame_ready:
            cloud = self.latest_cloud.copy()
        return point_cloud_ply(cloud)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read RealSense + YOLO detections and write camera/URDF XYZ TSV."
    )
    parser.add_argument("--model", default="/home/niic/workspace/vision/exp.pt")
    parser.add_argument("--output", default="/home/niic/workspace/vision/target_xyz.tsv")
    parser.add_argument("--scene-pointcloud", default="/home/niic/workspace/vision/scene_pointcloud.bin")
    parser.add_argument("--extrinsic-file", default="/home/niic/workspace/vision/camera_extrinsic.json")
    parser.add_argument("--serial", default="241122306062")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=15)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--confidence", type=float, default=0.50)
    parser.add_argument("--min-depth", type=float, default=0.20)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument("--object-depth-band", type=float, default=0.08)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--output-hz", type=float, default=5.0)
    parser.add_argument(
        "--max-rows",
        type=int,
        default=100,
        help="Keep at most this many target rows in the TSV file. Use 0 to disable trimming.",
    )
    parser.add_argument(
        "--camera-offset",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 1.4],
        metavar=("X", "Y", "Z"),
        help="Camera optical origin in URDF base_link, meters: x forward, y left, z up.",
    )
    parser.add_argument(
        "--orientation",
        choices=("minus_y", "plus_x"),
        default="minus_y",
        help="Rough camera optical orientation relative to URDF base_link.",
    )
    parser.add_argument(
        "--all-targets",
        action="store_true",
        help="Write every valid detection instead of only the highest-score target.",
    )
    parser.add_argument(
        "--print-targets",
        action="store_true",
        help="Print target coordinates to the terminal. By default coordinates are only written to TSV.",
    )
    parser.add_argument(
        "--log-misses",
        action="store_true",
        help="Print periodic messages when no target row is written.",
    )
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8090)
    parser.add_argument(
        "--no-web",
        action="store_true",
        help="Disable the built-in MJPEG web preview.",
    )
    parser.add_argument(
        "--stream-width",
        type=int,
        default=0,
        help="Web preview width. Use 0 with --stream-height to derive width from the frame aspect.",
    )
    parser.add_argument(
        "--stream-height",
        type=int,
        default=0,
        help="Web preview height. Use 0 to derive height from --stream-width.",
    )
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument(
        "--no-labels",
        action="store_true",
        help="Hide class names and confidence labels in the web preview.",
    )
    return parser.parse_args()


def default_extrinsic(offset: list[float], orientation: str) -> dict[str, Any]:
    roll, pitch, yaw = legacy_rpy(orientation)
    return {
        "x": float(offset[0]),
        "y": float(offset[1]),
        "z": float(offset[2]),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "image_rotation_deg": 0,
        "orientation": orientation,
        "rpy_convention": "standard_zyx",
    }


def ensure_extrinsic(path: Path, offset: list[float], orientation: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return
    path.write_text(
        json.dumps(default_extrinsic(offset, orientation), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def legacy_rpy(orientation: str) -> tuple[float, float, float]:
    if orientation == "plus_x":
        return -math.pi / 2.0, 0.0, -math.pi / 2.0
    return -math.pi / 2.0, 0.0, math.pi


def load_extrinsic(path: Path, fallback_offset: list[float], fallback_orientation: str) -> dict[str, Any]:
    fallback_roll, fallback_pitch, fallback_yaw = legacy_rpy(fallback_orientation)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        offset = [
            float(raw.get("x", fallback_offset[0])),
            float(raw.get("y", fallback_offset[1])),
            float(raw.get("z", fallback_offset[2])),
        ]
        orientation = str(raw.get("orientation", fallback_orientation))
        if orientation not in ("minus_y", "plus_x"):
            orientation = fallback_orientation
        default_roll, default_pitch, default_yaw = legacy_rpy(orientation)
        rotation = int(raw.get("image_rotation_deg", 0))
        if rotation not in (0, 90, 180, 270):
            rotation = 0
        fallback_rpy = (
            (default_roll, default_pitch, default_yaw)
            if "orientation" in raw
            else (fallback_roll, fallback_pitch, fallback_yaw)
        )
        if raw.get("rpy_convention") == "standard_zyx":
            roll = float(raw.get("roll", fallback_rpy[0]))
            pitch = float(raw.get("pitch", fallback_rpy[1]))
        elif "roll" in raw or "pitch" in raw:
            roll = float(raw.get("pitch", fallback_rpy[0]))
            pitch = float(raw.get("roll", fallback_rpy[1]))
        else:
            roll, pitch = fallback_rpy[:2]
        return {
            "offset": offset,
            "orientation": orientation,
            "roll": roll,
            "pitch": pitch,
            "yaw": float(raw.get("yaw", fallback_rpy[2])),
            "image_rotation_deg": rotation,
            "rpy_convention": "standard_zyx",
        }
    except Exception:
        return {
            "offset": list(fallback_offset),
            "orientation": fallback_orientation,
            "roll": fallback_roll,
            "pitch": fallback_pitch,
            "yaw": fallback_yaw,
            "image_rotation_deg": 0,
            "rpy_convention": "standard_zyx",
        }


def ensure_header(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        with path.open("r", encoding="utf-8") as f:
            if f.readline() == HEADER:
                return
        backup = path.with_suffix(path.suffix + ".old")
        path.replace(backup)
    path.write_text(HEADER, encoding="utf-8")


def start_realsense(args: argparse.Namespace) -> tuple[Any, Any, float, object]:
    import pyrealsense2 as rs

    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.depth_width, args.depth_height, rs.format.z16, args.depth_fps)

    pipeline = rs.pipeline()
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    return pipeline, align, depth_scale, intrinsics


def model_names(model: Any) -> dict[int, str]:
    names = getattr(model, "names", None)
    if isinstance(names, dict):
        return {int(key): str(value) for key, value in names.items()}
    if isinstance(names, list):
        return {idx: str(value) for idx, value in enumerate(names)}
    return {}


def infer(model: Any, frame_rgb: np.ndarray, args: argparse.Namespace) -> list[Detection]:
    import cv2

    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    kwargs = {
        "source": bgr,
        "conf": args.confidence,
        "imgsz": args.imgsz,
        "verbose": False,
    }
    if args.device:
        kwargs["device"] = args.device
    result = model.predict(**kwargs)[0]
    if result.boxes is None:
        return []

    detections = []
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy()
        detections.append(
            Detection(
                float(x1),
                float(y1),
                float(x2),
                float(y2),
                float(box.conf[0].item()),
                int(box.cls[0].item()),
            )
        )
    detections.sort(key=lambda det: det.score, reverse=True)
    return detections


def resolve_stream_size(args: argparse.Namespace, frame_width: int, frame_height: int) -> tuple[int, int]:
    if args.stream_width <= 0 and args.stream_height <= 0:
        return frame_width, frame_height
    if args.stream_width <= 0:
        width = round(args.stream_height * frame_width / frame_height)
        return max(1, width), max(1, args.stream_height)
    if args.stream_height <= 0:
        height = round(args.stream_width * frame_height / frame_width)
        return max(1, args.stream_width), max(1, height)
    return max(1, args.stream_width), max(1, args.stream_height)


def resize_for_stream(
    frame: np.ndarray, width: int, height: int
) -> tuple[np.ndarray, float, float]:
    import cv2

    source_height, source_width = frame.shape[:2]
    if width == source_width and height == source_height:
        return frame, 1.0, 1.0
    interpolation = cv2.INTER_AREA if width < source_width else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (width, height), interpolation=interpolation)
    return resized, width / source_width, height / source_height


def scale_detections(
    detections: list[Detection],
    scale_x: float,
    scale_y: float,
    width: int,
    height: int,
) -> list[Detection]:
    if scale_x == 1.0 and scale_y == 1.0:
        return detections
    return [
        Detection(
            x1=max(0.0, min(round(item.x1 * scale_x), width - 1)),
            y1=max(0.0, min(round(item.y1 * scale_y), height - 1)),
            x2=max(0.0, min(round(item.x2 * scale_x), width - 1)),
            y2=max(0.0, min(round(item.y2 * scale_y), height - 1)),
            score=item.score,
            cls=item.cls,
        )
        for item in detections
    ]


def draw_detections(
    frame: np.ndarray,
    detections: list[Detection],
    names: dict[int, str],
    show_labels: bool,
) -> np.ndarray:
    import cv2

    output = frame.copy()
    line_width = 2 if output.shape[1] >= 900 else 1
    font_scale = 0.58 if output.shape[1] >= 900 else 0.48
    for item in detections:
        color = COLORS[item.cls % len(COLORS)]
        x1, y1, x2, y2 = (int(round(value)) for value in (item.x1, item.y1, item.x2, item.y2))
        cv2.rectangle(output, (x1, y1), (x2, y2), color, line_width)
        if not show_labels:
            continue

        label = f"{names.get(item.cls, item.cls)} {item.score:.0%}"
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, line_width
        )
        top = max(0, y1 - text_height - baseline - 10)
        right = min(output.shape[1] - 1, x1 + text_width + 14)
        cv2.rectangle(output, (x1, top), (right, y1), color, -1)
        cv2.putText(
            output,
            label,
            (x1 + 7, y1 - baseline - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (8, 15, 22),
            line_width,
            cv2.LINE_AA,
        )
    return output


def draw_error(frame: np.ndarray, message: str) -> np.ndarray:
    import cv2

    output = frame.copy()
    cv2.rectangle(output, (0, 0), (output.shape[1], 56), (30, 30, 150), -1)
    cv2.putText(
        output,
        f"Vision error: {message[:100]}",
        (16, 37),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def encode_preview_frame(
    frame_bgr: np.ndarray,
    detections: list[Detection],
    names: dict[int, str],
    args: argparse.Namespace,
) -> tuple[bytes, tuple[int, int], list[Detection]]:
    import cv2

    stream_width, stream_height = resolve_stream_size(args, frame_bgr.shape[1], frame_bgr.shape[0])
    stream_frame, scale_x, scale_y = resize_for_stream(frame_bgr, stream_width, stream_height)
    stream_detections = scale_detections(detections, scale_x, scale_y, stream_width, stream_height)
    annotated = draw_detections(stream_frame, stream_detections, names, not args.no_labels)
    ok, encoded = cv2.imencode(
        ".jpg",
        annotated,
        [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, args.jpeg_quality))],
    )
    if not ok:
        raise RuntimeError("failed to encode preview JPEG")
    return encoded.tobytes(), (stream_width, stream_height), stream_detections


class VisionWebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], state: VisionWebState) -> None:
        super().__init__(address, VisionRequestHandler)
        self.state = state


class VisionRequestHandler(BaseHTTPRequestHandler):
    server: VisionWebServer

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self._send_file(BASE_DIR / "index.html", "text/html; charset=utf-8")
        elif path == "/api/status":
            self._send_json(self.server.state.get_status())
        elif path == "/snapshot.jpg":
            jpeg = self.server.state.snapshot()
            if jpeg is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame available")
            else:
                self._send_bytes(jpeg, "image/jpeg", cache=False)
        elif path == "/api/pointcloud.bin":
            payload = self.server.state.pointcloud_binary()
            if payload is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No point cloud available")
            else:
                self._send_bytes(payload, "application/octet-stream", cache=False)
        elif path == "/api/scene_pointcloud.bin":
            payload = self.server.state.scene_pointcloud_binary()
            if payload is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No saved scene point cloud available")
            else:
                self._send_bytes(payload, "application/octet-stream", cache=False)
        elif path == "/snapshot.ply":
            payload = self.server.state.pointcloud_ply()
            self._send_bytes(
                payload,
                "application/octet-stream",
                cache=False,
                headers={"Content-Disposition": 'attachment; filename="unoarm_d455_cloud.ply"'},
            )
        elif path == "/healthz":
            status = self.server.state.get_status()
            code = HTTPStatus.OK if status.get("ready") else HTTPStatus.SERVICE_UNAVAILABLE
            self._send_json(status, status=code)
        elif path == "/stream.mjpg":
            self._stream_video()
        elif path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/scene_pointcloud/save":
            try:
                status = self.server.state.save_scene_pointcloud()
                self._send_json({"ok": True, "scene_pointcloud": status})
            except RuntimeError as exc:
                self._send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.SERVICE_UNAVAILABLE)
            return
        if path != "/api/config":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = min(int(self.headers.get("Content-Length", "0")), 4096)
            payload = json.loads(self.rfile.read(length) or b"{}")
            settings = self.server.state.update_settings(payload)
            self._send_json(settings)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def _stream_video(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        sequence = -1
        try:
            while not self.server.state.stop_event.is_set():
                sequence, jpeg = self.server.state.wait_for_frame(sequence)
                if jpeg is None:
                    continue
                self.wfile.write(b"--" + BOUNDARY + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self._send_bytes(path.read_bytes(), content_type)

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        self._send_bytes(
            json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            "application/json; charset=utf-8",
            status=status,
        )

    def _send_bytes(
        self,
        payload: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
        cache: bool = False,
        headers: dict[str, str] | None = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        if not cache:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt: str, *args: object) -> None:
        return


def start_web_server(args: argparse.Namespace, state: VisionWebState) -> VisionWebServer | None:
    if args.no_web:
        return None
    server = VisionWebServer((args.web_host, args.web_port), state)
    thread = threading.Thread(target=server.serve_forever, name="vision-web-server", daemon=True)
    thread.start()
    print(f"vision web preview: http://127.0.0.1:{args.web_port}", flush=True)
    return server


def stop_web_server(server: VisionWebServer | None, state: VisionWebState | None) -> None:
    if state is not None:
        state.stop_event.set()
        with state.frame_ready:
            state.frame_ready.notify_all()
    if server is not None:
        server.shutdown()
        server.server_close()


def robust_depth_m(depth: np.ndarray, cx: int, cy: int, scale: float) -> float:
    radius = 4
    h, w = depth.shape[:2]
    x0 = max(0, cx - radius)
    x1 = min(w, cx + radius + 1)
    y0 = max(0, cy - radius)
    y1 = min(h, cy + radius + 1)
    patch = depth[y0:y1, x0:x1].astype(np.float32) * scale
    valid = patch[(patch > 0.05) & (patch < 5.0)]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))


def rotate_frame_for_detection(frame_bgr: np.ndarray, extrinsic: dict[str, Any]) -> np.ndarray:
    rotation = int(extrinsic.get("image_rotation_deg", 0))
    if rotation == 90:
        return np.ascontiguousarray(np.rot90(frame_bgr, k=-1))
    if rotation == 180:
        return np.ascontiguousarray(np.rot90(frame_bgr, k=2))
    if rotation == 270:
        return np.ascontiguousarray(np.rot90(frame_bgr, k=1))
    return frame_bgr


def detection_pixel_to_original(
    x: float,
    y: float,
    original_width: int,
    original_height: int,
    extrinsic: dict[str, Any],
) -> tuple[float, float]:
    rotation = int(extrinsic.get("image_rotation_deg", 0))
    if rotation == 90:
        return y, original_height - 1.0 - x
    if rotation == 180:
        return original_width - 1.0 - x, original_height - 1.0 - y
    if rotation == 270:
        return original_width - 1.0 - y, x
    return x, y


def camera_xyz(
    det: Detection,
    depth: np.ndarray,
    depth_scale: float,
    intrinsics: object,
    extrinsic: dict[str, Any],
) -> list[float] | None:
    h, w = depth.shape[:2]
    tx = (det.x1 + det.x2) * 0.5
    ty = (det.y1 + det.y2) * 0.5
    ox, oy = detection_pixel_to_original(tx, ty, w, h, extrinsic)
    cx = int(round(ox))
    cy = int(round(oy))
    if not (0 <= cx < w and 0 <= cy < h):
        return None
    z = robust_depth_m(depth, cx, cy, depth_scale)
    if z <= 0.0:
        return None
    x = (cx - float(intrinsics.ppx)) / float(intrinsics.fx) * z
    y = (cy - float(intrinsics.ppy)) / float(intrinsics.fy) * z
    return [float(x), float(y), float(z)]


def robust_object_mask(
    det: Detection,
    depth_m: np.ndarray,
    settings: PointCloudSettings,
) -> tuple[np.ndarray, float] | None:
    import cv2

    height, width = depth_m.shape
    x1 = max(0, min(int(det.x1), width - 1))
    y1 = max(0, min(int(det.y1), height - 1))
    x2 = max(x1 + 1, min(int(det.x2), width))
    y2 = max(y1 + 1, min(int(det.y2), height))
    roi = depth_m[y1:y2, x1:x2]
    if roi.size < 16:
        return None

    valid = (
        np.isfinite(roi)
        & (roi >= settings.min_depth_m)
        & (roi <= settings.max_depth_m)
    )
    valid_values = roi[valid]
    if valid_values.size < 20:
        return None

    roi_h, roi_w = roi.shape
    margin_x = max(1, int(round(roi_w * 0.28)))
    margin_y = max(1, int(round(roi_h * 0.28)))
    inner = roi[
        margin_y : max(margin_y + 1, roi_h - margin_y),
        margin_x : max(margin_x + 1, roi_w - margin_x),
    ]
    inner_valid = inner[
        np.isfinite(inner)
        & (inner >= settings.min_depth_m)
        & (inner <= settings.max_depth_m)
    ]
    if inner_valid.size >= 12:
        seed_depth = float(np.median(inner_valid))
    else:
        sorted_depth = np.sort(valid_values)
        foreground_count = max(12, int(round(sorted_depth.size * 0.60)))
        seed_depth = float(np.median(sorted_depth[:foreground_count]))

    depth_band = max(settings.object_depth_band_m, min(0.18, seed_depth * 0.035))
    candidate = valid & (np.abs(roi - seed_depth) <= depth_band)
    if int(candidate.sum()) < 12:
        return None

    kernel = np.ones((3, 3), np.uint8)
    candidate_u8 = cv2.morphologyEx(candidate.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    candidate_u8 = cv2.morphologyEx(candidate_u8, cv2.MORPH_OPEN, kernel)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(candidate_u8, connectivity=8)
    if count <= 1:
        selected = candidate_u8.astype(bool)
    else:
        cx = (roi_w - 1) * 0.5
        cy = (roi_h - 1) * 0.5
        best_label = 0
        best_score = -math.inf
        diagonal = max(1.0, math.hypot(roi_w, roi_h))
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 9:
                continue
            distance = math.hypot(float(centroids[label, 0]) - cx, float(centroids[label, 1]) - cy)
            score = math.log1p(area) - 2.0 * distance / diagonal
            if score > best_score:
                best_label = label
                best_score = score
        if best_label == 0:
            return None
        selected = labels == best_label

    local_y, local_x = np.where(selected)
    if local_x.size < 12:
        return None
    z = roi[local_y, local_x]
    z_median = float(np.median(z))
    mad = float(np.median(np.abs(z - z_median)))
    tolerance = max(0.018, 3.5 * 1.4826 * mad)
    keep = np.abs(z - z_median) <= tolerance
    local_x = local_x[keep]
    local_y = local_y[keep]
    if local_x.size < 12:
        return None

    mask = np.zeros_like(depth_m, dtype=bool)
    mask[y1 + local_y, x1 + local_x] = True
    return mask, depth_band


def locate_objects(
    detections: list[Detection],
    depth_m: np.ndarray,
    intrinsics: object,
    names: dict[int, str],
    settings: PointCloudSettings,
) -> tuple[list[Object3D], np.ndarray]:
    labels = np.full(depth_m.shape, -1, dtype=np.int16)
    objects: list[Object3D] = []

    for det in detections:
        result = robust_object_mask(det, depth_m, settings)
        if result is None:
            continue
        mask, depth_band = result
        ys, xs = np.where(mask)
        z = depth_m[ys, xs].astype(np.float32)
        x = (xs.astype(np.float32) - float(intrinsics.ppx)) * z / float(intrinsics.fx)
        y = (ys.astype(np.float32) - float(intrinsics.ppy)) * z / float(intrinsics.fy)
        points = np.column_stack((x, y, z))
        if len(points) < 12:
            continue

        lower = np.percentile(points, 5.0, axis=0)
        upper = np.percentile(points, 95.0, axis=0)
        center = np.median(points, axis=0)
        extent = np.maximum(0.0, upper - lower)
        object_id = len(objects) + 1
        labels[mask] = object_id - 1
        objects.append(
            Object3D(
                object_id=object_id,
                class_id=int(det.cls),
                class_name=names.get(int(det.cls), str(det.cls)),
                confidence=round(float(det.score), 4),
                bbox_xyxy=[int(det.x1), int(det.y1), int(det.x2), int(det.y2)],
                center_pixel=[int(round(float(np.median(xs)))), int(round(float(np.median(ys))))],
                center_camera_xyz_m=[round(float(value), 4) for value in center],
                distance_m=round(float(np.linalg.norm(center)), 4),
                extent_xyz_m=[round(float(value), 4) for value in extent],
                point_count=int(len(points)),
                depth_band_m=round(float(depth_band), 4),
            )
        )
    return objects, labels


def build_point_cloud(
    color_image: np.ndarray,
    depth_m: np.ndarray,
    labels: np.ndarray,
    intrinsics: object,
    settings: PointCloudSettings,
) -> tuple[np.ndarray, list[float]]:
    stride = max(1, int(settings.point_stride))
    height, width = depth_m.shape
    ys = np.arange(0, height, stride, dtype=np.int32)
    xs = np.arange(0, width, stride, dtype=np.int32)
    grid_x, grid_y = np.meshgrid(xs, ys)
    z = depth_m[grid_y, grid_x]
    valid = (
        np.isfinite(z)
        & (z >= settings.min_depth_m)
        & (z <= settings.max_depth_m)
    )
    if not np.any(valid):
        return np.empty((0, 6), dtype=np.float32), [0.0, 0.0, 1.0]

    u = grid_x[valid].astype(np.float32)
    v = grid_y[valid].astype(np.float32)
    z_valid = z[valid].astype(np.float32)
    x = (u - float(intrinsics.ppx)) * z_valid / float(intrinsics.fx)
    y = (v - float(intrinsics.ppy)) * z_valid / float(intrinsics.fy)
    positions = np.column_stack((x, y, z_valid)).astype(np.float32)

    bgr = color_image[grid_y[valid], grid_x[valid]].astype(np.float32) / 255.0
    colors = bgr[:, ::-1].copy()
    sampled_labels = labels[grid_y[valid], grid_x[valid]]
    for index in np.unique(sampled_labels[sampled_labels >= 0]):
        b, g, r = COLORS[int(index) % len(COLORS)]
        colors[sampled_labels == index] = (r / 255.0, g / 255.0, b / 255.0)

    cloud = np.column_stack((positions, colors)).astype("<f4", copy=False)
    center = np.median(positions, axis=0)
    return cloud, [round(float(value), 4) for value in center]


def point_cloud_payload(sequence: int, cloud: np.ndarray) -> bytes:
    header = struct.pack("<II", int(sequence), int(len(cloud)))
    return header + cloud.astype("<f4", copy=False).tobytes(order="C")


def point_cloud_ply(cloud: np.ndarray) -> bytes:
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment UnoArm RealSense aligned color point cloud\n"
        f"element vertex {len(cloud)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    vertices = np.empty(
        len(cloud),
        dtype=np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("red", "u1"),
                ("green", "u1"),
                ("blue", "u1"),
            ]
        ),
    )
    if len(cloud):
        vertices["x"] = cloud[:, 0]
        vertices["y"] = cloud[:, 1]
        vertices["z"] = cloud[:, 2]
        rgb = np.clip(cloud[:, 3:6] * 255.0, 0, 255).astype(np.uint8)
        vertices["red"] = rgb[:, 0]
        vertices["green"] = rgb[:, 1]
        vertices["blue"] = rgb[:, 2]
    return header + vertices.tobytes(order="C")


def rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def camera_attitude_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    return rpy_matrix(roll, pitch, yaw)


def urdf_xyz(camera: list[float], extrinsic: dict[str, Any]) -> list[float]:
    cx, cy, cz = camera
    ox, oy, oz = extrinsic["offset"]
    matrix = camera_attitude_matrix(
        float(extrinsic.get("roll", 0.0)),
        float(extrinsic.get("pitch", 0.0)),
        float(extrinsic.get("yaw", 0.0)),
    )
    return [
        ox + matrix[0][0] * cx + matrix[0][1] * cy + matrix[0][2] * cz,
        oy + matrix[1][0] * cx + matrix[1][1] * cy + matrix[1][2] * cz,
        oz + matrix[2][0] * cx + matrix[2][1] * cy + matrix[2][2] * cz,
    ]


def trim_rows(path: Path, max_rows: int) -> None:
    if max_rows <= 0 or not path.exists():
        return
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        path.write_text(HEADER, encoding="utf-8")
        return
    data = lines[1:]
    if len(data) <= max_rows:
        return
    kept = data[-max_rows:]
    path.write_text(HEADER + "\n".join(kept) + "\n", encoding="utf-8")


def append_rows(path: Path, rows: list[tuple[list[float], list[float]]], max_rows: int) -> None:
    if not rows:
        return
    with path.open("a", encoding="utf-8") as f:
        for camera, urdf in rows:
            f.write(
                f"{camera[0]:.6f}\t{camera[1]:.6f}\t{camera[2]:.6f}\t"
                f"{urdf[0]:.6f}\t{urdf[1]:.6f}\t{urdf[2]:.6f}\n"
            )
    trim_rows(path, max_rows)


def main() -> int:
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    out_path = Path(args.output)
    scene_pointcloud_path = Path(args.scene_pointcloud)
    extrinsic_path = Path(args.extrinsic_file)
    ensure_header(out_path)
    ensure_extrinsic(extrinsic_path, args.camera_offset, args.orientation)
    web_state = VisionWebState(args, model_path, out_path, scene_pointcloud_path)
    web_server: VisionWebServer | None = None

    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True
        web_state.stop_event.set()
        with web_state.frame_ready:
            web_state.frame_ready.notify_all()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    from ultralytics import YOLO

    model = YOLO(str(model_path))
    pipeline, align, depth_scale, intrinsics = start_realsense(args)
    web_server = start_web_server(args, web_state)
    interval = 1.0 / max(args.output_hz, 0.1)
    next_time = 0.0
    next_diag_time = 0.0
    print(f"vision target writer started: model={model_path} output={out_path}", flush=True)
    print("press Ctrl+C to stop", flush=True)

    try:
        while not stop:
            frames = pipeline.wait_for_frames(timeout_ms=args.timeout_ms)
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            now = time.monotonic()
            if now < next_time:
                continue
            next_time = now + interval

            frame_start = time.perf_counter()
            frame_bgr = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data()).copy()
            extrinsic = load_extrinsic(
                extrinsic_path,
                args.camera_offset,
                args.orientation,
            )
            detection_frame = rotate_frame_for_detection(frame_bgr, extrinsic)
            frame_rgb = detection_frame[:, :, ::-1].copy()
            infer_start = time.perf_counter()
            detections = infer(model, frame_rgb, args)
            inference_ms = (time.perf_counter() - infer_start) * 1000.0
            names = model_names(model)
            selected = detections if args.all_targets else detections[:1]

            rows = []
            depth_invalid_count = 0
            for det in selected:
                cxyz = camera_xyz(det, depth, depth_scale, intrinsics, extrinsic)
                if cxyz is None:
                    depth_invalid_count += 1
                    continue
                uxyz = urdf_xyz(cxyz, extrinsic)
                rows.append((cxyz, uxyz))

            geometry_start = time.perf_counter()
            depth_m = depth.astype(np.float32) * depth_scale
            pointcloud_settings = web_state.pointcloud_settings()
            if int(extrinsic.get("image_rotation_deg", 0)) == 0:
                objects, labels = locate_objects(
                    detections,
                    depth_m,
                    intrinsics,
                    names,
                    pointcloud_settings,
                )
            else:
                # Detection boxes are in the rotated preview frame. Keep the
                # full color cloud available, but avoid incorrect object masks.
                objects = []
                labels = np.full(depth_m.shape, -1, dtype=np.int16)
            cloud, cloud_center = build_point_cloud(
                frame_bgr,
                depth_m,
                labels,
                intrinsics,
                pointcloud_settings,
            )
            geometry_ms = (time.perf_counter() - geometry_start) * 1000.0

            append_rows(out_path, rows, args.max_rows)
            latest_target = None
            if rows:
                cxyz, uxyz = rows[0]
                latest_target = {
                    "camera_xyz_m": cxyz,
                    "urdf_xyz_m": uxyz,
                }
            if web_server is not None:
                try:
                    jpeg, stream_size, preview_detections = encode_preview_frame(
                        detection_frame, detections, names, args
                    )
                    frame_ms = (time.perf_counter() - frame_start) * 1000.0
                    web_state.publish_frame(
                        jpeg=jpeg,
                        detections=detections,
                        names=names,
                        stream_size=stream_size,
                        inference_ms=inference_ms,
                        frame_ms=frame_ms,
                        latest_target=latest_target,
                        pointcloud_payload=point_cloud_payload(web_state.frame_sequence + 1, cloud),
                        cloud=cloud,
                        objects=objects,
                        preview_detections=preview_detections,
                        cloud_center=cloud_center,
                        geometry_ms=geometry_ms,
                    )
                except Exception as exc:
                    web_state.update_status(ready=False, error=str(exc))
            if args.log_misses and not rows and now >= next_diag_time:
                next_diag_time = now + 5.0
                if not detections:
                    print("warning: no YOLO detections; target_xyz.tsv not updated", flush=True)
                else:
                    print(
                        f"warning: {len(detections)} detection(s), "
                        f"but {depth_invalid_count} had invalid depth; target_xyz.tsv not updated",
                        flush=True,
                    )
            if args.print_targets and rows:
                cxyz, uxyz = rows[0]
                print(
                    "target "
                    f"camera=({cxyz[0]:+.3f},{cxyz[1]:+.3f},{cxyz[2]:+.3f}) "
                    f"urdf=({uxyz[0]:+.3f},{uxyz[1]:+.3f},{uxyz[2]:+.3f})",
                    flush=True,
                )
    finally:
        pipeline.stop()
        stop_web_server(web_server, web_state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
