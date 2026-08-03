#!/usr/bin/env python3
"""Verify synchronized UnoArm MuJoCo RGB-D topics."""

import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def stamp_key(msg):
    return int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_mujoco_rgbd")
        self.rgb = {}
        self.depth = {}
        self.info = None
        self.create_subscription(Image, "/mujoco_camera/color/image_raw", self.on_rgb, 2)
        self.create_subscription(Image, "/mujoco_camera/depth/image_raw", self.on_depth, 2)
        self.create_subscription(CameraInfo, "/mujoco_camera/color/camera_info", self.on_info, 2)

    def on_rgb(self, msg):
        self.rgb[stamp_key(msg)] = msg

    def on_depth(self, msg):
        self.depth[stamp_key(msg)] = msg

    def on_info(self, msg):
        self.info = msg

    def result(self):
        common = sorted(set(self.rgb) & set(self.depth))
        if not common or self.info is None:
            return None
        key = common[-1]
        rgb, depth = self.rgb[key], self.depth[key]
        values = np.frombuffer(depth.data, dtype=np.float32)
        finite = values[np.isfinite(values)]
        return {
            "synchronized_stamp": {"sec": key[0], "nanosec": key[1]},
            "frame_id": depth.header.frame_id,
            "rgb": {
                "encoding": rgb.encoding,
                "width": rgb.width,
                "height": rgb.height,
                "data_bytes": len(rgb.data),
            },
            "depth": {
                "encoding": depth.encoding,
                "width": depth.width,
                "height": depth.height,
                "step": depth.step,
                "data_bytes": len(depth.data),
                "finite_pixels": int(finite.size),
                "nan_pixels": int(np.isnan(values).sum()),
                "min_m": float(finite.min()) if finite.size else math.nan,
                "max_m": float(finite.max()) if finite.size else math.nan,
            },
            "camera_info": {
                "width": self.info.width,
                "height": self.info.height,
                "k": list(self.info.k),
                "p": list(self.info.p),
                "d": list(self.info.d),
            },
        }


def main():
    rclpy.init()
    node = Verifier()
    deadline = time.monotonic() + 10.0
    result = None
    while time.monotonic() < deadline and result is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        result = node.result()
    node.destroy_node()
    rclpy.shutdown()
    if result is None:
        print("No synchronized RGB-D frame and CameraInfo received", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    depth = result["depth"]
    valid = (
        result["rgb"]["encoding"] == "rgb8"
        and depth["encoding"] == "32FC1"
        and depth["data_bytes"] == depth["width"] * depth["height"] * 4
        and result["frame_id"] == "vlm_camera_link"
        and depth["finite_pixels"] > 0
    )
    return 0 if valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
