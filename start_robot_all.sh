#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"

usage() {
    cat <<'EOF'
Usage: ./start_robot_all.sh [--docker]

Options:
  --docker    Run this launcher inside the UnoArm Docker development container.
  -h, --help  Show this help.
EOF
}

RUN_IN_DOCKER=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --docker)
            RUN_IN_DOCKER=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "${RUN_IN_DOCKER}" -eq 1 && "${UNOARM_IN_DOCKER:-0}" != "1" ]]; then
    exec "${ROOT_DIR}/docker/run-dev-x11.sh" ./start_robot_all.sh
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
# 如果当前终端需要 sudo 才能执行 ethercatctl/setcap，则交互式输入一次密码并导出给子脚本复用
prompt_sudo_pass_if_needed() {
    if sudo -n ethercatctl status >/dev/null 2>&1; then
        return 0
    fi
    if [[ ! -t 0 ]]; then
        echo "警告：当前无终端，无法交互输入 sudo 密码。如启动失败，请配置免密 sudo 或先设置 SUDO_PASS 环境变量。" >&2
        return 0
    fi
    echo "检测到启动 CSP 驱动需要 sudo 权限。" >&2
    local pass
    read -r -s -p "请输入 sudo 密码（仅本次启动复用）: " pass
    echo "" >&2
    if [[ -n "$pass" ]] && printf '%s\n' "$pass" | sudo -S -v >/dev/null 2>&1; then
        export SUDO_PASS="$pass"
        echo "sudo 认证成功。" >&2
    else
        echo "sudo 认证失败，CSP 驱动可能无法启动。" >&2
    fi
}

prompt_sudo_pass_if_needed

# 启动前先重启 EtherCAT masters，确保 Master0 / Master2 处于 running 状态
if command -v ethercatctl >/dev/null 2>&1; then
    echo "Restarting EtherCAT masters..."
    sudo ethercatctl restart || true
    sleep 2
fi

export SUDO_PASS="${SUDO_PASS:-1}"

CSP_SETUP="${ROOT_DIR}/CSP_direct_drive/install/setup.bash"
[[ -f "${CSP_SETUP}" ]] || CSP_SETUP="${ROOT_DIR}/install/setup.bash"

source_ros() {
    set +u
    source /opt/ros/humble/setup.bash
    [ -f "${ROOT_DIR}/install/setup.bash" ] && source "${ROOT_DIR}/install/setup.bash"
    [ -f "${ROOT_DIR}/CSP_direct_drive/install/setup.bash" ] && source "${ROOT_DIR}/CSP_direct_drive/install/setup.bash"
    set -u
}

wait_for() {
    local label="$1"
    local command="$2"
    local timeout_sec="${3:-30}"
    local deadline=$((SECONDS + timeout_sec))
    while [ "${SECONDS}" -lt "${deadline}" ]; do
        if bash -lc "${command}" >/dev/null 2>&1; then
            echo "ready: ${label}"
            return 0
        fi
        sleep 1
    done
    echo "warning: ${label} not ready after ${timeout_sec}s" >&2
    return 0
}

start_bg() {
    local name="$1"
    shift
    local log_file="${LOG_DIR}/${name}.log"
    nohup "$@" >"${log_file}" 2>&1 < /dev/null &
    echo "${name}: pid=$! log=${log_file}"
}

cleanup_and_exit() {
    echo ""
    echo "=== Stopping robot stack ==="
    "${ROOT_DIR}/stop_robot_all.sh" >/dev/null 2>&1 || true
    exit 0
}

trap cleanup_and_exit INT TERM

source_ros
"${ROOT_DIR}/stop_robot_all.sh" >/dev/null 2>&1 || true

export VISION_MODEL="${VISION_MODEL:-${ROOT_DIR}/vision/exp.pt}"
export VISION_TARGET_FILE="${VISION_TARGET_FILE:-${ROOT_DIR}/vision/target_xyz.tsv}"
export VISION_SCENE_POINTCLOUD_FILE="${VISION_SCENE_POINTCLOUD_FILE:-${ROOT_DIR}/vision/scene_pointcloud.bin}"
export CAMERA_EXTRINSIC_FILE="${CAMERA_EXTRINSIC_FILE:-${ROOT_DIR}/vision/camera_extrinsic.json}"

VISION_READY_MARKER="/tmp/unoarm_vision_start_marker"
touch "${VISION_READY_MARKER}"
start_bg vision_target \
    "${ROOT_DIR}/vision/start_vision_target_writer.sh" \
    --model "${VISION_MODEL}" \
    --output "${VISION_TARGET_FILE}" \
    --scene-pointcloud "${VISION_SCENE_POINTCLOUD_FILE}" \
    --extrinsic-file "${CAMERA_EXTRINSIC_FILE}" \
    --web-host "${VISION_WEB_HOST:-0.0.0.0}" \
    --web-port "${VISION_WEB_PORT:-8090}" \
    --stream-width "${VISION_STREAM_WIDTH:-0}" \
    --stream-height "${VISION_STREAM_HEIGHT:-0}" \
    --jpeg-quality "${VISION_JPEG_QUALITY:-92}" \
    --min-depth "${VISION_MIN_DEPTH:-0.20}" \
    --max-depth "${VISION_MAX_DEPTH:-4.0}" \
    --object-depth-band "${VISION_OBJECT_DEPTH_BAND:-0.08}" \
    --point-stride "${VISION_POINT_STRIDE:-4}"
wait_for "Vision target writer" \
    "test '${VISION_TARGET_FILE}' -nt '${VISION_READY_MARKER}' && [ \"\$(wc -l < '${VISION_TARGET_FILE}')\" -gt 1 ]" \
    45

start_bg csp_driver "${ROOT_DIR}/start_csp_driver.sh"
wait_for "CSP action server" \
    "source /opt/ros/humble/setup.bash; [ -f '${CSP_SETUP}' ] && source '${CSP_SETUP}'; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'; ros2 action list | grep -q '/arm_controller/follow_joint_trajectory'" \
    45

start_bg moveit "${ROOT_DIR}/start_robot_moveit.sh"
wait_for "MoveIt services" \
    "source /opt/ros/humble/setup.bash; [ -f '${ROOT_DIR}/install/setup.bash' ] && source '${ROOT_DIR}/install/setup.bash'; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'; ros2 service list | grep -q '/compute_ik' && ros2 service list | grep -q '/plan_kinematic_path'" \
    45

start_bg web_control "${ROOT_DIR}/run_web_control.sh"
wait_for "Web Control HTTP" \
    "curl -fsS --max-time 2 http://127.0.0.1:${WEB_CONTROL_PORT:-8765}/api/status" \
    45

echo "=== Robot stack started ==="
echo "Robot Web Control: http://192.168.10.200:${WEB_CONTROL_PORT:-8765}"
echo "Vision Preview: http://192.168.10.200:${VISION_WEB_PORT:-8090}"

# 如果是直接终端运行，前台持续显示日志；否则（如菜单后台调用）直接退出
if [[ -t 1 ]]; then
    echo "按 Ctrl+C 停止并退出日志跟踪"
    echo ""
    tail -f "${LOG_DIR}/vision_target.log" "${LOG_DIR}/csp_driver.log" "${LOG_DIR}/moveit.log" "${LOG_DIR}/web_control.log"
fi
