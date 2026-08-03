#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${ROOT_DIR}/.." && pwd)"

usage() {
    cat <<EOF
Usage: $0 [OPTION]

Options:
  (none)         Start robot Web server
  --server-only  Same as default; kept for compatibility
  --check        Check robot ROS topics/actions, then exit
  --stop         Stop robot Web server, then exit
  --help         Show this help message

Environment variables:
  ROS_DOMAIN_ID                    default: 25
  ROS_AUTOMATIC_DISCOVERY_RANGE    default: SUBNET
  ROS_STATIC_PEERS                 default: 192.168.10.200
  JOINT_STATE_TOPIC                default: /joint_states
  WEB_CONTROL_HOST                 default: 0.0.0.0
  WEB_CONTROL_PORT                 default: 8765
  WEB_CONTROL_OCCUPANCY_TOPIC      default: /unoarm/web_control/occupied
  VISION_TARGET_FILE               default: <project>/vision/target_xyz.tsv when available
  CAMERA_EXTRINSIC_FILE            default: <project>/vision/camera_extrinsic.json when available
  WEB_CONTROL_FASTDDS_NO_SHM        default: 0
EOF
}

stop_nodes() {
    echo "=== Stopping robot web control server ==="
    pkill -f "${SCRIPT_DIR}/server.py" 2>/dev/null || true
    pkill -f "python3 .*web_control/web_control/server.py" 2>/dev/null || true
    python3 -c "
import os, socket, subprocess, time
port = int(os.environ.get('WEB_CONTROL_PORT', '8765'))
for _ in range(30):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('127.0.0.1', port))
        s.close()
        break
    except OSError:
        s.close()
        time.sleep(0.1)
else:
    subprocess.run(['fuser', '-k', f'{port}/tcp'], capture_output=True)
" 2>/dev/null || true
    sleep 1
}

setup_env() {
    set +u
    source /opt/ros/humble/setup.bash
    set -u

    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
    export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-192.168.10.200}"
    export JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/joint_states}"
    export WEB_CONTROL_HOST="${WEB_CONTROL_HOST:-0.0.0.0}"
    export WEB_CONTROL_PORT="${WEB_CONTROL_PORT:-8765}"
    export WEB_CONTROL_OCCUPANCY_TOPIC="${WEB_CONTROL_OCCUPANCY_TOPIC:-/unoarm/web_control/occupied}"
    if [ -f "${WORKSPACE_DIR}/vision/camera_extrinsic.json" ]; then
        default_vision_dir="${WORKSPACE_DIR}/vision"
    else
        default_vision_dir="/home/niic/workspace/vision"
    fi
    export VISION_TARGET_FILE="${VISION_TARGET_FILE:-${default_vision_dir}/target_xyz.tsv}"
    export VISION_SCENE_POINTCLOUD_FILE="${VISION_SCENE_POINTCLOUD_FILE:-${default_vision_dir}/scene_pointcloud.bin}"
    export CAMERA_EXTRINSIC_FILE="${CAMERA_EXTRINSIC_FILE:-${default_vision_dir}/camera_extrinsic.json}"

    # The FastDDS no-SHM profile is opt-in. On Humble it can shrink reader
    # history buffers enough to make rclpy node creation fail.
    if [[ "${WEB_CONTROL_FASTDDS_NO_SHM:-0}" == "1" ]]; then
        export RMW_FASTRTPS_USE_QOS_FROM_XML=1
        export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/fastdds_no_shm.xml"
    else
        unset RMW_FASTRTPS_USE_QOS_FROM_XML
        unset FASTRTPS_DEFAULT_PROFILES_FILE
    fi
}

check_not_occupied() {
    if timeout 2 ros2 topic echo --once "${WEB_CONTROL_OCCUPANCY_TOPIC}" std_msgs/msg/Bool 2>/dev/null | grep -q 'data: true'; then
        echo "WARNING: stale or active occupancy on ${WEB_CONTROL_OCCUPANCY_TOPIC}; starting after local stop." >&2
    fi
}

print_env_summary() {
    echo "=== Environment ==="
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}"
    echo "ROS_STATIC_PEERS=${ROS_STATIC_PEERS}"
    echo "JOINT_STATE_TOPIC=${JOINT_STATE_TOPIC:-/joint_states}"
    if [ "${WEB_CONTROL_HOST}" = "0.0.0.0" ]; then
        echo "WEB_CONTROL=http://<robot-ip>:${WEB_CONTROL_PORT}"
    else
        echo "WEB_CONTROL=http://${WEB_CONTROL_HOST}:${WEB_CONTROL_PORT}"
    fi
    echo "WEB_CONTROL_OCCUPANCY_TOPIC=${WEB_CONTROL_OCCUPANCY_TOPIC}"
    echo "VISION_TARGET_FILE=${VISION_TARGET_FILE}"
    echo "VISION_SCENE_POINTCLOUD_FILE=${VISION_SCENE_POINTCLOUD_FILE}"
    echo "CAMERA_EXTRINSIC_FILE=${CAMERA_EXTRINSIC_FILE}"
}

check_remote() {
    setup_env
    print_env_summary
    echo
    echo "=== Topics ==="
    ros2 topic list | grep -E 'joint_states|tf|motors_enabled|estop' || true
    echo
    echo "=== Actions ==="
    ros2 action list -t | grep -E 'follow_joint_trajectory|arm_controller' || true
    echo
    echo "=== Services ==="
    ros2 service list | grep -E 'compute_ik|plan_kinematic_path|enable_motors|reset_emergency_stop' || true
}

wait_remote_ready() {
    wait_timeout="${WEB_CONTROL_WAIT_TIMEOUT:-30}"
    deadline=$(($(date +%s) + wait_timeout))
    echo "=== Waiting for robot MoveIt graph ==="
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if ros2 service list 2>/dev/null | grep -q '/compute_ik' && \
           ros2 service list 2>/dev/null | grep -q '/plan_kinematic_path' && \
           ros2 service list 2>/dev/null | grep -q '/apply_planning_scene' && \
           ros2 action list 2>/dev/null | grep -q '/arm_controller/follow_joint_trajectory' && \
           ros2 topic list 2>/dev/null | grep -qx "${JOINT_STATE_TOPIC}"; then
            echo "=== Robot MoveIt graph ready ==="
            return 0
        fi
        sleep 1
    done

    echo "WARNING: Robot MoveIt graph not fully ready after ${wait_timeout}s." >&2
    echo "Expected: /compute_ik, /plan_kinematic_path, /apply_planning_scene, /arm_controller/follow_joint_trajectory, ${JOINT_STATE_TOPIC}" >&2
    echo "Current services:" >&2
    ros2 service list 2>/dev/null | grep -E 'compute_ik|plan_kinematic_path|apply_planning_scene' >&2 || true
    echo "Current actions:" >&2
    ros2 action list 2>/dev/null | grep -E 'follow_joint_trajectory|arm_controller' >&2 || true
    return 0
}

start_server() {
    setup_env
    stop_nodes
    check_not_occupied
    print_env_summary
    wait_remote_ready
    cd "${SCRIPT_DIR}"
    echo "=== Starting robot web control server ==="
    if [ "${WEB_CONTROL_HOST}" = "0.0.0.0" ]; then
        echo "Open: http://192.168.10.200:${WEB_CONTROL_PORT}"
    else
        echo "Open: http://${WEB_CONTROL_HOST}:${WEB_CONTROL_PORT}"
    fi
    exec ./server.py
}

case "${1:-}" in
    --help|-h)
        usage
        exit 0
        ;;
    --check)
        check_remote
        exit 0
        ;;
    --stop)
        setup_env
        stop_nodes
        echo "=== Stopped ==="
        exit 0
        ;;
    --server-only)
        start_server
        ;;
    "")
        start_server
        ;;
    *)
        echo "Unknown option: $1"
        usage
        exit 1
        ;;
esac
