#!/usr/bin/env python3
"""Interactive normalized VLA chunk sender for the MuJoCo simulation driver."""

import argparse
import json
import math
import os
import shlex
import subprocess
import sys
import threading
import time
import uuid

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
except ModuleNotFoundError as exc:
    rclpy = None
    SingleThreadedExecutor = None
    Node = object
    JointState = None
    Bool = None
    String = None
    DurabilityPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
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
SCRIPT_IN_DOCKER = "asm0003/scripts/mujoco_vla_bridge_menu.py"


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


class ArmState:
    def __init__(self, node, prefix, joint_names):
        self.prefix = prefix
        self.joint_names = list(joint_names)
        self.positions = None
        self.motors_enabled = None
        self.estop_active = None
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            JointState, f"/{prefix}/joint_states", self._on_joint_state, 10
        )
        node.create_subscription(
            Bool, f"/{prefix}/motors_enabled", self._on_enabled, state_qos
        )
        node.create_subscription(
            Bool, f"/{prefix}/emergency_stop_state", self._on_estop, state_qos
        )

    def _on_joint_state(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in self.joint_names):
            self.positions = [float(by_name[name]) for name in self.joint_names]

    def _on_enabled(self, msg):
        self.motors_enabled = bool(msg.data)

    def _on_estop(self, msg):
        self.estop_active = bool(msg.data)

    def current_positions(self):
        return list(self.positions or [0.0] * len(self.joint_names))


class MujocoVlaBridgeMenu(Node):
    def __init__(self):
        super().__init__("mujoco_vla_bridge_menu")
        self.left = ArmState(self, "master0", ACTION_NAMES[:7])
        self.right = ArmState(self, "master2", ACTION_NAMES[8:15])
        self._last_results = {}
        self._command_pub = self.create_publisher(String, "/mujoco/vla_joint_chunk", 10)
        self.create_subscription(
            String, "/mujoco/vla_joint_chunk_result", self._on_result, 10
        )

    def _on_result(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        request_id = result.get("request_id")
        if request_id:
            self._last_results[str(request_id)] = result

    def current_action_row(self):
        return self.left.current_positions() + [0.0] + self.right.current_positions() + [0.0]

    @staticmethod
    def linear_chunk(start, target, steps):
        return [
            [a + (b - a) * step / steps for a, b in zip(start, target)]
            for step in range(1, steps + 1)
        ]

    def publish_vla_chunk(self, actions, fps, arms, execute):
        request_id = uuid.uuid4().hex
        payload = {
            "request_id": request_id,
            "command_type": "joint_chunk",
            "unit": "rad",
            "actions": actions,
            "fps": fps,
            "arms": arms,
            "execute": execute,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._command_pub.publish(msg)
        return request_id, payload

    def wait_result(self, request_id, timeout_sec):
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline and rclpy.ok():
            result = self._last_results.pop(str(request_id), None)
            if result is not None:
                return result
            time.sleep(0.05)
        return {"ok": False, "request_id": request_id, "error": "result timeout"}


def start_background_spin(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def run():
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    thread = threading.Thread(target=run, name="mujoco-vla-menu-spin", daemon=True)
    thread.start()
    return executor, stop_event, thread


def print_status(node):
    row = node.current_action_row()
    print("\n当前 MuJoCo 状态:")
    print(
        f"  right enabled={node.right.motors_enabled} estop={node.right.estop_active} "
        f"joint_state={'ok' if node.right.positions else '等待'}"
    )
    print(
        f"  left  enabled={node.left.motors_enabled} estop={node.left.estop_active} "
        f"joint_state={'ok' if node.left.positions else '等待'}"
    )
    for index, name in enumerate(ACTION_NAMES):
        print(f"  {index:2d} {name:16s} {row[index]: .4f}")


def ask_float(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"{prompt}{suffix}: ").strip()
    if not raw and default is not None:
        return float(default)
    return float(raw)


def ask_int(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"{prompt}{suffix}: ").strip()
    if not raw and default is not None:
        return int(default)
    return int(raw)


def ask_arm(default="right"):
    raw = input(f"手臂 both/left/right [{default}]: ").strip().lower()
    value = raw or default
    if value not in {"both", "left", "right"}:
        raise ValueError("手臂必须是 both、left 或 right")
    return value


def validate_actions(actions):
    if not actions:
        raise ValueError("actions 不能为空")
    for row_index, row in enumerate(actions):
        if len(row) != len(ACTION_NAMES):
            raise ValueError(f"actions[{row_index}] 必须是 16 维")
        for col_index, value in enumerate(row):
            number = float(value)
            if not math.isfinite(number):
                raise ValueError(f"actions[{row_index}][{col_index}] 不是有限数")


def confirm_execute():
    print("即将发送到 MuJoCo 模拟 driver。数值单位为 rad，driver 会按模型关节上下限校验。")
    return input("输入 MOVE 确认执行，其它任意输入取消: ").strip() == "MOVE"


def run_menu(node, args):
    print("MuJoCo VLA -> simulation driver bridge")
    print("规则: 输入数值就是 rad；不做 mean/std，不做 [-1,1] 到上下限映射。")
    print("接口: /mujoco/vla_joint_chunk -> /mujoco/vla_joint_chunk_result")
    time.sleep(args.startup_wait)
    while rclpy.ok():
        print_status(node)
        print("\n1. dry-run 当前姿态零位移 chunk")
        print("2. 输入单关节 rad 增量 dry-run")
        print("3. 输入单关节 rad 增量并执行")
        print("4. 输入完整 16 维 rad 目标 dry-run")
        print("5. 输入完整 16 维 rad 目标并执行")
        print("q. 退出")
        choice = input("选择: ").strip().lower()
        if choice == "q":
            return 0
        if choice not in {"1", "2", "3", "4", "5"}:
            print("未知选项")
            continue
        try:
            start = node.current_action_row()
            target = list(start)
            steps = ask_int("轨迹步数", args.steps)
            fps = ask_float("频率 Hz", args.fps)
            arms = ask_arm(args.arms)
            execute = choice in {"3", "5"}
            if choice == "1":
                actions = node.linear_chunk(start, target, steps)
                execute = False
            elif choice in {"2", "3"}:
                index = ask_int("动作维度下标 0-15")
                if not 0 <= index < len(ACTION_NAMES):
                    raise ValueError("下标必须在 0 到 15")
                delta = ask_float("rad 增量")
                target[index] += delta
                actions = node.linear_chunk(start, target, steps)
            elif choice in {"4", "5"}:
                raw = input("输入 16 个 rad 目标，用空格或逗号分隔: ").replace(",", " ").split()
                if len(raw) != len(ACTION_NAMES):
                    raise ValueError("必须输入 16 个数")
                target = [float(item) for item in raw]
                actions = node.linear_chunk(start, target, steps)
            validate_actions(actions)
            if execute and not confirm_execute():
                print("已取消执行")
                continue
            request_id, payload = node.publish_vla_chunk(actions, fps, arms, execute)
            print("已发送:")
            print(json.dumps(payload, ensure_ascii=False, indent=2))
            result = node.wait_result(request_id, args.result_timeout)
            print("driver 返回:")
            print(json.dumps(result, ensure_ascii=False, indent=2))
            time.sleep(0.5)
        except Exception as exc:
            print(f"失败: {exc}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Interactive rad VLA sender for MuJoCo simulation.")
    parser.add_argument("--docker", action="store_true", help="Run this script inside the active MuJoCo Docker container.")
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--arms", choices=("both", "left", "right"), default="right")
    parser.add_argument("--startup-wait", type=float, default=2.0)
    parser.add_argument("--result-timeout", type=float, default=3.0)
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
    node = MujocoVlaBridgeMenu()
    executor, stop_event, thread = start_background_spin(node)
    try:
        return run_menu(node, args)
    except KeyboardInterrupt:
        return 130
    finally:
        stop_event.set()
        executor.shutdown()
        thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
