#!/usr/bin/env python3
"""Smoke test for the MuJoCo VLA joint chunk bridge.

Run while ./start_mujoco_sim.sh is active:
  source /opt/ros/humble/setup.bash
  python3 tests/mujoco_vla_bridge_smoke.py
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
import uuid

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ModuleNotFoundError as exc:
    rclpy = None
    Node = object
    JointState = None
    String = None
    RCLPY_IMPORT_ERROR = exc
else:
    RCLPY_IMPORT_ERROR = None


ACTION_NAMES = [
    "Left_Joint1",
    "Left_Joint2",
    "Left_Joint3",
    "Left_Joint4",
    "Left_Joint5",
    "Left_Joint6",
    "Left_Joint7",
    "Left_Gripper",
    "Right_Joint1",
    "Right_Joint2",
    "Right_Joint3",
    "Right_Joint4",
    "Right_Joint5",
    "Right_Joint6",
    "Right_Joint7",
    "Right_Gripper",
]
SCRIPT_IN_DOCKER = "tests/mujoco_vla_bridge_smoke.py"


def find_mujoco_container():
    try:
        output = subprocess.check_output(
            [
                "docker",
                "ps",
                "--filter",
                "label=unoarm.mujoco=1",
                "--format",
                "{{.ID}}",
            ],
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"无法查询 MuJoCo Docker 容器: {exc}") from exc
    containers = [line.strip() for line in output.splitlines() if line.strip()]
    if not containers:
        raise RuntimeError("没有运行中的 MuJoCo Docker 容器，请先执行 ./start_mujoco_sim.sh --docker")
    return containers[0]


def exec_in_mujoco_container(script_path, argv):
    container = find_mujoco_container()
    passthrough = [arg for arg in argv if arg != "--docker"]
    command = (
        "cd /workspace/unoarm && "
        "source /opt/ros/humble/setup.bash && "
        "export ROS_DOMAIN_ID=21 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1 && "
        f"exec python3 {shlex.quote(script_path)}"
    )
    if passthrough:
        command += " " + " ".join(shlex.quote(arg) for arg in passthrough)
    docker_args = ["docker", "exec"]
    docker_args.append("-it" if sys.stdin.isatty() and sys.stdout.isatty() else "-i")
    docker_args.extend([container, "bash", "-lc", command])
    os.execvp("docker", docker_args)


class SmokeNode(Node):
    def __init__(self):
        super().__init__("mujoco_vla_bridge_smoke")
        self.left = None
        self.right = None
        self.results = {}
        self.command_pub = self.create_publisher(String, "/mujoco/vla_joint_chunk", 10)
        self.create_subscription(JointState, "/master0/joint_states", self._on_left, 10)
        self.create_subscription(JointState, "/master2/joint_states", self._on_right, 10)
        self.create_subscription(String, "/mujoco/vla_joint_chunk_result", self._on_result, 10)

    def _on_left(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        names = ACTION_NAMES[:7]
        if all(name in by_name for name in names):
            self.left = [float(by_name[name]) for name in names]

    def _on_right(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        names = ACTION_NAMES[8:15]
        if all(name in by_name for name in names):
            self.right = [float(by_name[name]) for name in names]

    def _on_result(self, msg):
        result = json.loads(msg.data)
        request_id = result.get("request_id")
        if request_id:
            self.results[str(request_id)] = result

    def current_action_row(self):
        if self.left is None or self.right is None:
            raise RuntimeError("joint_states not ready")
        return self.left + [0.0] + self.right + [0.0]

    def wait_ready(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.left is not None and self.right is not None:
                return
        raise TimeoutError("MuJoCo joint_states not ready")

    def send_and_wait(self, payload, timeout_sec):
        request_id = payload["request_id"]
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        end_publish = time.monotonic() + 1.0
        while time.monotonic() < end_publish and self.command_pub.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.command_pub.publish(msg)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            result = self.results.pop(request_id, None)
            if result is not None:
                return result
        raise TimeoutError("VLA result timeout")


def linear_chunk(start, target, steps):
    return [
        [a + (b - a) * step / steps for a, b in zip(start, target)]
        for step in range(1, steps + 1)
    ]


def main():
    parser = argparse.ArgumentParser(description="Smoke test MuJoCo VLA joint chunk topic bridge.")
    parser.add_argument("--docker", action="store_true", help="Run this script inside the active MuJoCo Docker container.")
    parser.add_argument("--arms", choices=("both", "left", "right"), default="right")
    parser.add_argument("--joint-index", type=int, default=8)
    parser.add_argument("--delta-rad", type=float, default=0.02)
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if args.docker:
        try:
            exec_in_mujoco_container(SCRIPT_IN_DOCKER, sys.argv[1:])
        except Exception as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    if rclpy is None:
        print(
            "ERROR: ROS Python modules are unavailable. Run: source /opt/ros/humble/setup.bash",
            file=sys.stderr,
        )
        print(f"import error: {RCLPY_IMPORT_ERROR}", file=sys.stderr)
        return 2

    os.environ.setdefault("ROS_DOMAIN_ID", "21")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    node = SmokeNode()
    try:
        node.wait_ready(args.timeout)
        start = node.current_action_row()
        target = list(start)
        if not 0 <= args.joint_index < len(ACTION_NAMES):
            raise ValueError("joint-index must be 0..15")
        target[args.joint_index] += float(args.delta_rad)
        payload = {
            "request_id": uuid.uuid4().hex,
            "command_type": "joint_chunk",
            "unit": "rad",
            "actions": linear_chunk(start, target, args.steps),
            "fps": args.fps,
            "arms": args.arms,
            "execute": bool(args.execute),
        }
        result = node.send_and_wait(payload, args.timeout)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("ok") is True else 1
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
