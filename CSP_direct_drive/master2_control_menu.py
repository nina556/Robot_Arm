#!/usr/bin/env python3
import select
import os
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmContext:
    """Per-arm state, publishers, clients."""
    def __init__(self, node, prefix, joint_names):
        self.prefix = prefix
        self.joint_names = joint_names
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.motors_enabled = None
        self.estop_active = None
        self.speed_step = None

        self.enable_client = node.create_client(
            SetBool, f"/{prefix}/enable_motors"
        )
        self.reset_client = node.create_client(
            Trigger, f"/{prefix}/reset_emergency_stop"
        )
        self.estop_publisher = node.create_publisher(
            Bool, f"/{prefix}/emergency_stop", 10
        )
        self.trajectory_publisher = node.create_publisher(
            JointTrajectory, f"/{prefix}/joint_trajectory", 10
        )

        node.create_subscription(
            Bool, f"/{prefix}/motors_enabled",
            lambda msg, ctx=self: ctx._enabled_callback(msg), state_qos
        )
        node.create_subscription(
            Bool, f"/{prefix}/emergency_stop_state",
            lambda msg, ctx=self: ctx._estop_callback(msg), state_qos
        )

        self.parameter_client = AsyncParameterClient(
            node, f"{prefix}_moveit_bridge"
        )
        self.speed_get_future = None
        self.speed_set_future = None
        self.pending_speed_step = None

    def _enabled_callback(self, message):
        self.motors_enabled = message.data

    def _estop_callback(self, message):
        self.estop_active = message.data

    @staticmethod
    def _value(value):
        if value is None:
            return "等待数据"
        return "是" if value else "否"


class DualArmControlMenu(Node):
    def __init__(self):
        super().__init__("dual_arm_control_menu")
        self.right = ArmContext(
            self, "master2",
            [f"Right_Joint{i}" for i in range(1, 8)],
        )
        self.left = ArmContext(
            self, "master0",
            [f"Left_Joint{i}" for i in range(1, 8)],
        )
        self.pending = []
        self.confirm_enable_until = 0.0
        self.confirm_home_until = 0.0
        self.connection_announced = False

    def announce_connection(self):
        right_ready = self.right.motors_enabled is not None and self.right.estop_active is not None
        left_ready = self.left.motors_enabled is not None and self.left.estop_active is not None
        if not self.connection_announced and right_ready and left_ready:
            self.connection_announced = True
            print("\n左右手控制器均已连接。", flush=True)
            self.refresh_speed()
            self.print_status()

    def print_status(self):
        print(
            f"\n右手 电机使能: {ArmContext._value(self.right.motors_enabled)}"
            f"    急停锁存: {ArmContext._value(self.right.estop_active)}",
            flush=True,
        )
        print(
            f"左手 电机使能: {ArmContext._value(self.left.motors_enabled)}"
            f"    急停锁存: {ArmContext._value(self.left.estop_active)}",
            flush=True,
        )
        right_speed = (
            "等待数据"
            if self.right.speed_step is None
            else f"{self.right.speed_step / 100.0:.1f} 倍"
        )
        print(f"右手速度: {right_speed}", flush=True)

    def refresh_speed(self):
        if (
            self.right.speed_get_future is None
            and self.right.parameter_client.services_are_ready()
        ):
            self.right.speed_get_future = self.right.parameter_client.get_parameters(
                ["max_step_per_cycle"]
            )

    def set_speed_step(self, step):
        step = max(100, min(600, int(step)))
        if not self.right.parameter_client.services_are_ready():
            print("\n错误：右手动态调速服务不可用。", flush=True)
            return
        if self.right.speed_set_future is not None:
            print("\n上一次调速仍在处理中。", flush=True)
            return
        parameter = Parameter(
            "max_step_per_cycle", Parameter.Type.INTEGER, step
        )
        self.right.speed_set_future = self.right.parameter_client.set_parameters(
            [parameter]
        )
        self.right.pending_speed_step = step
        print(
            f"\n正在设置右手速度为 {step / 100.0:.1f} 倍"
            f"（{step} counts/cycle）...",
            flush=True,
        )

    def adjust_speed(self, delta):
        if self.right.speed_step is None:
            self.refresh_speed()
            print("\n正在读取右手当前速度，请稍后再按一次。", flush=True)
            return
        self.set_speed_step(self.right.speed_step + delta)

    def request_enable(self, enabled):
        label = "使能" if enabled else "正常停止"
        for ctx, name in ((self.right, "右手"), (self.left, "左手")):
            if not ctx.enable_client.service_is_ready():
                print(f"\n错误：{name}使能服务不可用。", flush=True)
                continue
            request = SetBool.Request()
            request.data = enabled
            self.pending.append((f"{name}{label}", ctx.enable_client.call_async(request)))
        print(f"\n已发送双手{label}请求。", flush=True)

    def request_reset(self):
        for ctx, name in ((self.right, "右手"), (self.left, "左手")):
            if not ctx.reset_client.service_is_ready():
                print(f"\n错误：{name}急停复位服务不可用。", flush=True)
                continue
            self.pending.append(
                (f"{name}急停复位", ctx.reset_client.call_async(Trigger.Request()))
            )
        print("\n已发送双手急停复位请求。", flush=True)

    def emergency_stop(self):
        message = Bool()
        message.data = True
        self.right.estop_publisher.publish(message)
        self.left.estop_publisher.publish(message)
        self.confirm_enable_until = 0.0
        self.confirm_home_until = 0.0
        print("\n*** 已发送双手紧急快速停止 ***", flush=True)

    def request_home_zero(self):
        for ctx, name in ((self.right, "右手"), (self.left, "左手")):
            trajectory = JointTrajectory()
            trajectory.joint_names = list(ctx.joint_names)
            point = JointTrajectoryPoint()
            point.positions = [0.0] * 7
            point.time_from_start = Duration(sec=15, nanosec=0)
            trajectory.points.append(point)
            ctx.trajectory_publisher.publish(trajectory)
        print(
            "\n已发送双手回 0 点轨迹：15 秒到达，仍走 bridge 限位/步进/跟随误差保护。",
            flush=True,
        )

    def poll_results(self):
        for ctx in (self.right, self.left):
            if ctx.speed_get_future is not None and ctx.speed_get_future.done():
                try:
                    result = ctx.speed_get_future.result()
                    if result.values:
                        ctx.speed_step = int(result.values[0].integer_value)
                except Exception as error:
                    print(f"\n读取速度失败: {error}", flush=True)
                ctx.speed_get_future = None

            if ctx.speed_set_future is not None and ctx.speed_set_future.done():
                try:
                    result = ctx.speed_set_future.result()
                    if result.results and result.results[0].successful:
                        ctx.speed_step = ctx.pending_speed_step
                        print(
                            f"\n{ctx.prefix}速度已更新为 {ctx.speed_step / 100.0:.1f} 倍"
                            f"（{ctx.speed_step} counts/cycle）。",
                            flush=True,
                        )
                    else:
                        reason = (
                            result.results[0].reason
                            if result.results else "unknown error"
                        )
                        print(f"\n调速失败: {reason}", flush=True)
                except Exception as error:
                    print(f"\n调速请求异常: {error}", flush=True)
                ctx.speed_set_future = None
                ctx.pending_speed_step = None

        remaining = []
        for label, future in self.pending:
            if not future.done():
                remaining.append((label, future))
                continue
            try:
                response = future.result()
                result = "成功" if response.success else "失败"
                print(f"\n{label}{result}: {response.message}", flush=True)
            except Exception as error:
                print(f"\n{label}请求异常: {error}", flush=True)
        self.pending = remaining


def draw_menu():
    print(
        "\033[3J\033[2J\033[H"
        "========================================\n"
        "       0&2\n"
        "========================================\n\n"
        "  [e] 双手使能电机（再按一次 e 确认）\n"
        "  [d] 双手正常停止并取消使能\n"
        "  [空格] 双手紧急快速停止\n"
        "  [r] 双手复位急停（复位后仍未使能）\n"
        "  [s] 查看当前状态\n"
        "  [h] 双手回 0 点（再按一次 h 确认）\n"
        "  [1~6] 直接设置右手 1~6 倍速度\n"
        "  [键 / ]键：右手每次降低 / 提高 0.5 倍\n"
        "  [q] 退出菜单\n\n"
        "按键无需回车，空格急停始终优先。\n"
        "========================================",
        flush=True,
    )


def main():
    if not sys.stdin.isatty():
        raise SystemExit("错误：菜单必须在终端中运行。")

    rclpy.init()
    menu = DualArmControlMenu()
    old_terminal = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        draw_menu()
        print("\n正在连接左右手控制器...", flush=True)

        running = True
        connect_started = time.monotonic()
        connection_warning_printed = False
        while running and rclpy.ok():
            rclpy.spin_once(menu, timeout_sec=0.03)
            menu.announce_connection()
            menu.poll_results()
            if (
                not menu.connection_announced
                and not connection_warning_printed
                and time.monotonic() - connect_started >= 5.0
            ):
                connection_warning_printed = True
                print(
                    "\n警告：5 秒内未收到控制器状态。"
                    "请检查 start_moveit_bridge.sh 和 ROS_DOMAIN_ID=25。",
                    flush=True,
                )

            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if not readable:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()

            if key == " ":
                menu.emergency_stop()
            elif key == "e":
                if menu.right.estop_active or menu.left.estop_active:
                    menu.confirm_enable_until = 0.0
                    print(
                        "\n急停已锁存，禁止使能。请先按 r 复位急停。",
                        flush=True,
                    )
                    continue
                if menu.right.motors_enabled is None or menu.left.motors_enabled is None:
                    print("\n控制器状态未知，禁止使能。", flush=True)
                    continue
                now = time.monotonic()
                if now <= menu.confirm_enable_until:
                    menu.confirm_enable_until = 0.0
                    menu.request_enable(True)
                else:
                    menu.confirm_enable_until = now + 3.0
                    print("\n再次按 e 确认双手使能（3 秒内），空格可急停。", flush=True)
            elif key == "d":
                menu.confirm_enable_until = 0.0
                menu.confirm_home_until = 0.0
                menu.request_enable(False)
            elif key == "r":
                menu.confirm_enable_until = 0.0
                menu.confirm_home_until = 0.0
                menu.request_reset()
            elif key == "s":
                menu.refresh_speed()
                menu.print_status()
            elif key == "h":
                if menu.right.estop_active or menu.left.estop_active:
                    print("\n急停已锁存，禁止回 0。请先按 r 复位急停。", flush=True)
                    continue
                if not menu.right.motors_enabled or not menu.left.motors_enabled:
                    print("\n有手臂未使能，禁止回 0。请先使能。", flush=True)
                    continue
                now = time.monotonic()
                if now <= menu.confirm_home_until:
                    menu.confirm_home_until = 0.0
                    menu.request_home_zero()
                else:
                    menu.confirm_home_until = now + 3.0
                    print(
                        "\n再次按 h 确认双手回 0 点（3 秒内），空格可随时急停。",
                        flush=True,
                    )
            elif key in "123456":
                menu.set_speed_step(int(key) * 100)
            elif key == "[":
                menu.adjust_speed(-50)
            elif key == "]":
                menu.adjust_speed(50)
            elif key == "q":
                running = False
            elif key == "\x03":
                running = False
            else:
                menu.confirm_enable_until = 0.0

        print("\n退出菜单。", flush=True)
    except KeyboardInterrupt:
        print("\n退出菜单。", flush=True)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_terminal)
        menu.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
