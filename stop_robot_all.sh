#!/usr/bin/env bash
set -euo pipefail

patterns=(
    "vision/vision_target_writer.py"
    "web_control/web_control/server.py"
    "ros2 launch asm0003 web_control_moveit.launch.py"
    "moveit_ros_move_group/move_group"
    "joint_state_merger.py"
    "mujoco_sim_bridge.py"
    "robot_state_publisher"
    "static_transform_publisher"
    "light_test master2_moveit_bridge"
    "master2_moveit_bridge"
)

for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" 2>/dev/null || true
done

if command -v fuser >/dev/null 2>&1; then
    fuser -k "${WEB_CONTROL_PORT:-8765}/tcp" >/dev/null 2>&1 || true
fi

sleep 1
echo "=== Stopped robot stack ==="
