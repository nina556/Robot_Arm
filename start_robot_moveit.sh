#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

set +u
source /opt/ros/humble/setup.bash
if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
    source "${ROOT_DIR}/install/setup.bash"
else
    set -u
    echo "ERROR: ${ROOT_DIR}/install/setup.bash not found." >&2
    echo "Run: colcon build --symlink-install --packages-select asm0003" >&2
    exit 1
fi
set -u

input_topics="${JOINT_STATE_INPUT_TOPICS:-/master2/joint_states,/master0/joint_states}"
joint_state_topic="${JOINT_STATE_TOPIC:-/joint_states}"

echo "=== Robot MoveIt ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}"
echo "input_topics=${input_topics}"
echo "joint_state_topic=${joint_state_topic}"

exec ros2 launch asm0003 web_control_moveit.launch.py \
    input_topics:="${input_topics}" \
    joint_state_topic:="${joint_state_topic}"
