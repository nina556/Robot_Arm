#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROOT_DIR}/logs"
PID_FILE="${LOG_DIR}/mujoco_sim_stack.pids"
mkdir -p "${LOG_DIR}"

usage() {
    cat <<'EOF'
Usage: ./start_mujoco_sim.sh [--docker]

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

SIM_ROS_DOMAIN_ID="${MUJOCO_ROS_DOMAIN_ID:-21}"
if [[ "${SIM_ROS_DOMAIN_ID}" == "25" && "${UNOARM_ALLOW_MUJOCO_DOMAIN_25:-0}" != "1" ]]; then
    echo "ERROR: MuJoCo 仿真拒绝使用 ROS_DOMAIN_ID=25，避免和真机控制域混用。" >&2
    echo "       如确实要覆盖，请显式设置 UNOARM_ALLOW_MUJOCO_DOMAIN_25=1 MUJOCO_ROS_DOMAIN_ID=25。" >&2
    exit 2
fi

export ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${MUJOCO_ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOCALHOST_ONLY="${MUJOCO_ROS_LOCALHOST_ONLY:-1}"
export ROS_STATIC_PEERS="${MUJOCO_ROS_STATIC_PEERS:-127.0.0.1}"
# The ros2 CLI daemon may have been started on the real-robot domain.  Direct
# discovery keeps readiness checks inside this simulation's isolated domain.
export ROS2CLI_NO_DAEMON="${ROS2CLI_NO_DAEMON:-1}"

if [[ "${RUN_IN_DOCKER}" -eq 1 && "${UNOARM_IN_DOCKER:-0}" != "1" ]]; then
    # Clean up a previously orphaned Docker MuJoCo container before publishing
    # the fixed Web port. Docker Desktop can leave the container alive when its
    # foreground `docker run` wrapper is terminated.
    "${ROOT_DIR}/stop_mujoco_sim.sh" >/dev/null 2>&1 || true
    exec "${ROOT_DIR}/docker/run-dev-x11.sh" ./start_mujoco_sim.sh
fi

export VISION_TARGET_FILE="${VISION_TARGET_FILE:-${ROOT_DIR}/vision/target_xyz.tsv}"
export CAMERA_EXTRINSIC_FILE="${CAMERA_EXTRINSIC_FILE:-${ROOT_DIR}/vision/camera_extrinsic.json}"
export VISION_SCENE_POINTCLOUD_FILE="${VISION_SCENE_POINTCLOUD_FILE:-${ROOT_DIR}/vision/scene_pointcloud.bin}"
export WEB_CONTROL_DISABLE_PLATFORM_OBSTACLE="${WEB_CONTROL_DISABLE_PLATFORM_OBSTACLE:-1}"
export UNOARM_MUJOCO_TABLE_OBSTACLE="${UNOARM_MUJOCO_TABLE_OBSTACLE:-1}"
export MUJOCO_TABLE_CENTER_X="${MUJOCO_TABLE_CENTER_X:-0.0}"
export MUJOCO_TABLE_CENTER_Y="${MUJOCO_TABLE_CENTER_Y:--0.72}"
export MUJOCO_TABLE_CENTER_Z="${MUJOCO_TABLE_CENTER_Z:-0.33}"
export MUJOCO_TABLE_SIZE_X="${MUJOCO_TABLE_SIZE_X:-0.40}"
export MUJOCO_TABLE_SIZE_Y="${MUJOCO_TABLE_SIZE_Y:-0.30}"
export MUJOCO_TABLE_SIZE_Z="${MUJOCO_TABLE_SIZE_Z:-1.00}"
export MUJOCO_TABLE_COLLISION_TOP_INSET="${MUJOCO_TABLE_COLLISION_TOP_INSET:-0.0}"
export MUJOCO_TABLE_GRASP_CONTACT_INSET="${MUJOCO_TABLE_GRASP_CONTACT_INSET:-0.0}"
export MUJOCO_TABLE_PLANNING_GUARD_M="${MUJOCO_TABLE_PLANNING_GUARD_M:-0.025}"
# Align the rotor with the middle of the finger pads. Larger values move the
# tilted gripper forward past the part, so keep the TCP closer to the object.
export MUJOCO_GRASP_OBJECT_TO_TCP_M="${MUJOCO_GRASP_OBJECT_TO_TCP_M:-0.046}"
export MUJOCO_GRASP_LATERAL_OFFSET_M="${MUJOCO_GRASP_LATERAL_OFFSET_M:--0.004}"
# The 30-degree tool axis already contributes vertical offset.  This smaller
# clearance places the finger-pad center about 5 mm below the rotor origin,
# while keeping the tilted support links clear of the physical tabletop.
export MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M="${MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M:-0.018}"
export UNOARM_ANGLED_GRASP_X_GROUND_ANGLE_DEG="${UNOARM_ANGLED_GRASP_X_GROUND_ANGLE_DEG:-30}"
export UNOARM_ANGLED_GRASP_MAX_CANDIDATES="${UNOARM_ANGLED_GRASP_MAX_CANDIDATES:-10}"
export MUJOCO_GRASP_OBJECT_MESH="${MUJOCO_GRASP_OBJECT_MESH:-${ROOT_DIR}/web_control/frontend/public/models/part.stl}"
export MUJOCO_GRASP_OBJECT_MESH_SCALE="${MUJOCO_GRASP_OBJECT_MESH_SCALE:-0.001}"
export MUJOCO_GRASP_OBJECT_SIZE_X="${MUJOCO_GRASP_OBJECT_SIZE_X:-0.04}"
# The rotor is flat on the tabletop, upside down from the CAD import: the
# current visible top face becomes the tabletop-facing bottom.
# Keep the planning/guard box aligned with that displayed and simulated mesh.
export MUJOCO_GRASP_OBJECT_MESH_QUAT="${MUJOCO_GRASP_OBJECT_MESH_QUAT:-0 1 0 0}"
export MUJOCO_GRASP_OBJECT_SIZE_Y="${MUJOCO_GRASP_OBJECT_SIZE_Y:-0.04}"
export MUJOCO_GRASP_OBJECT_SIZE_Z="${MUJOCO_GRASP_OBJECT_SIZE_Z:-0.027}"
export MUJOCO_GRASP_OBJECT_PLANNING_MARGIN="${MUJOCO_GRASP_OBJECT_PLANNING_MARGIN:-0.005}"
export MUJOCO_GRASP_OBJECT_CLEARANCE="${MUJOCO_GRASP_OBJECT_CLEARANCE:-0.002}"
export MUJOCO_GRASP_TARGET_X="${MUJOCO_GRASP_TARGET_X:-0.18}"
export MUJOCO_GRASP_TARGET_Y="${MUJOCO_GRASP_TARGET_Y:--0.60}"
export MUJOCO_GRASP_TARGET_Z="${MUJOCO_GRASP_TARGET_Z:-0.8455}"
export MUJOCO_RGBD_OFFSET_X="${MUJOCO_RGBD_OFFSET_X:--0.003}"
export MUJOCO_RGBD_OFFSET_Y="${MUJOCO_RGBD_OFFSET_Y:-0.010}"
# Mesh measurements put the 5 cm cube's two-sided finger contact near
# -0.50 rad is only the maximum close command. The MuJoCo bridge now stops
# earlier on bilateral contact, so finger meshes do not close through the cube.
# Real robot launchers keep their existing -0.91 rad closed position.
export UNOARM_GRIPPER_CLOSED_POSITION="${UNOARM_GRIPPER_CLOSED_POSITION:--0.75}"
export UNOARM_SIM_GRIPPER_SPEED_RAD_PER_SEC="${UNOARM_SIM_GRIPPER_SPEED_RAD_PER_SEC:-0.5}"
export UNOARM_SIM_GRIPPER_UNILATERAL_SPEED_RAD_PER_SEC="${UNOARM_SIM_GRIPPER_UNILATERAL_SPEED_RAD_PER_SEC:-0.10}"
export UNOARM_SIM_GRASP_CAPTURE_TIMEOUT_SEC="${UNOARM_SIM_GRASP_CAPTURE_TIMEOUT_SEC:-8.0}"
export UNOARM_SIM_GRASP_GUARD_TIMEOUT_SEC="${UNOARM_SIM_GRASP_GUARD_TIMEOUT_SEC:-180}"

MODEL_PATH="${MUJOCO_MODEL_PATH:-${ROOT_DIR}/asm0003/mujoco/asm0003_camera_scene.mjb}"
SOURCE_URDF="${MUJOCO_SOURCE_URDF:-${ROOT_DIR}/web_control/frontend/dist/urdf/unoarm.urdf}"
MODEL_URDF="${MUJOCO_MODEL_URDF:-${ROOT_DIR}/asm0003/mujoco/unoarm_mujoco.urdf}"
MUJOCO_MESH_DIR="${MUJOCO_MESH_DIR:-${ROOT_DIR}/asm0003/mujoco/meshes}"
MUJOCO_URDF_GENERATOR="${ROOT_DIR}/asm0003/scripts/generate_mujoco_urdf.py"
MOVEIT_URDF="${ROOT_DIR}/asm0003/urdf/asm0003.urdf"
BRIDGE_SCRIPT="${ROOT_DIR}/asm0003/scripts/mujoco_sim_bridge.py"
SCENE_BUILDER="${ROOT_DIR}/asm0003/scripts/build_mujoco_camera_scene.py"
SCENE_CONFIG="${ROOT_DIR}/asm0003/mujoco/scene_config.json"

if [ -z "${MUJOCO_VIEWER+x}" ]; then
    if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
        MUJOCO_VIEWER=1
    else
        MUJOCO_VIEWER=0
    fi
fi
export MUJOCO_VIEWER
if [ "${MUJOCO_CAMERA_ENABLED:-1}" = "1" ] && [ "${MUJOCO_VIEWER}" = "0" ]; then
    export MUJOCO_GL="${MUJOCO_GL:-egl}"
fi

source_ros() {
    set +u
    source /opt/ros/humble/setup.bash
    [ -f "${ROOT_DIR}/install/setup.bash" ] && source "${ROOT_DIR}/install/setup.bash"
    set -u
}

wait_for() {
    local label="$1"
    local command="$2"
    local timeout_sec="${3:-30}"
    local deadline=$((SECONDS + timeout_sec))
    while [ "${SECONDS}" -lt "${deadline}" ]; do
        if timeout 3s bash -lc "${command}" >/dev/null 2>&1; then
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
    LAST_BG_PID="$!"
    printf '%s %s\n' "${name}" "${LAST_BG_PID}" >>"${PID_FILE}"
    echo "${name}: pid=${LAST_BG_PID} log=${log_file}"
}

fail_if_exited() {
    local name="$1"
    local pid="$2"
    local log_file="$3"
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
        echo "ERROR: ${name} exited before becoming ready. Last log lines:" >&2
        tail -n 80 "${log_file}" >&2 || true
        exit 1
    fi
}

stop_tracked_sim_stack() {
    "${ROOT_DIR}/stop_mujoco_sim.sh" >/dev/null 2>&1 || true
}

cleanup_and_exit() {
    echo ""
    echo "=== Stopping MuJoCo simulation stack ==="
    stop_tracked_sim_stack >/dev/null 2>&1 || true
    if [ "${UNOARM_IN_DOCKER:-0}" = "1" ] && command -v chown >/dev/null 2>&1; then
        chown -R "${UNOARM_HOST_UID:-1000}:${UNOARM_HOST_GID:-1000}" "${LOG_DIR}" >/dev/null 2>&1 || true
    fi
    exit 0
}

trap cleanup_and_exit INT TERM

generate_mujoco_urdf() {
    if [ ! -f "${SOURCE_URDF}" ]; then
        echo "ERROR: Web source URDF not found: ${SOURCE_URDF}" >&2
        echo "Run npm run build in web_control/frontend or set MUJOCO_SOURCE_URDF." >&2
        exit 1
    fi
    if [ ! -f "${MUJOCO_URDF_GENERATOR}" ]; then
        echo "ERROR: MuJoCo URDF generator not found: ${MUJOCO_URDF_GENERATOR}" >&2
        exit 1
    fi
    mkdir -p "$(dirname "${MODEL_URDF}")" "${MUJOCO_MESH_DIR}"
    echo "Generating MuJoCo URDF: ${SOURCE_URDF} -> ${MODEL_URDF}"
    python3 "${MUJOCO_URDF_GENERATOR}" \
        --source "${SOURCE_URDF}" \
        --output "${MODEL_URDF}" \
        --mesh-dir "${MUJOCO_MESH_DIR}" \
        --joint-limits-source "${MOVEIT_URDF}"
}

if [ ! -f "${MODEL_URDF}" ] || [ "${SOURCE_URDF}" -nt "${MODEL_URDF}" ] || [ "${MUJOCO_URDF_GENERATOR}" -nt "${MODEL_URDF}" ] || [ "${MUJOCO_FORCE_COMPILE:-0}" = "1" ]; then
    generate_mujoco_urdf
fi

if [ -z "${MUJOCO_MODEL_PATH:-}" ] && { [ ! -f "${MODEL_PATH}" ] || [ "${MODEL_URDF}" -nt "${MODEL_PATH}" ] || [ "${SCENE_BUILDER}" -nt "${MODEL_PATH}" ] || [ "${SCENE_CONFIG}" -nt "${MODEL_PATH}" ] || [ "${MUJOCO_GRASP_OBJECT_MESH}" -nt "${MODEL_PATH}" ] || [ "${MUJOCO_FORCE_COMPILE:-0}" = "1" ]; }; then
    if [ ! -f "${MODEL_URDF}" ]; then
        echo "ERROR: MuJoCo source URDF not found: ${MODEL_URDF}" >&2
        exit 1
    fi
    echo "Building MuJoCo camera/table scene: ${MODEL_PATH}"
    python3 "${SCENE_BUILDER}" --source "${MODEL_URDF}" --output "${MODEL_PATH}"
fi

if [ ! -f "${MODEL_PATH}" ]; then
    echo "ERROR: MuJoCo scene model not found: ${MODEL_PATH}" >&2
    exit 1
fi

if [ ! -x "${BRIDGE_SCRIPT}" ]; then
    chmod +x "${BRIDGE_SCRIPT}"
fi

source_ros
stop_tracked_sim_stack >/dev/null 2>&1 || true
: >"${PID_FILE}"

bridge_args=(python3 "${BRIDGE_SCRIPT}" --model "${MODEL_PATH}" --publish-rate "${MUJOCO_PUBLISH_RATE:-100}")
if [ "${MUJOCO_CAMERA_ENABLED:-1}" = "1" ]; then
    bridge_args+=(
        --camera
        --camera-width "${MUJOCO_CAMERA_WIDTH:-640}"
        --camera-height "${MUJOCO_CAMERA_HEIGHT:-480}"
        --camera-fps "${MUJOCO_CAMERA_FPS:-5}"
        --camera-topic "${MUJOCO_CAMERA_TOPIC:-/mujoco_camera/color/image_raw}"
        --camera-frame-id "${MUJOCO_CAMERA_FRAME_ID:-vlm_camera_link}"
        --camera-name "${MUJOCO_CAMERA_NAME:-head_d455_rgb}"
        --camera-depth
        --camera-depth-topic "${MUJOCO_CAMERA_DEPTH_TOPIC:-/mujoco_camera/depth/image_raw}"
        --camera-depth-fps "${MUJOCO_CAMERA_DEPTH_FPS:-2}"
        --camera-info-topic "${MUJOCO_CAMERA_INFO_TOPIC:-/mujoco_camera/color/camera_info}"
        --camera-max-range "${MUJOCO_CAMERA_MAX_RANGE:-10.0}"
    )
else
    bridge_args+=(--no-camera)
fi
if [ "${MUJOCO_WRIST_CAMERA_ENABLED:-1}" = "1" ]; then
    bridge_args+=(
        --wrist-camera
        --wrist-camera-width "${MUJOCO_WRIST_CAMERA_WIDTH:-480}"
        --wrist-camera-height "${MUJOCO_WRIST_CAMERA_HEIGHT:-360}"
        --wrist-camera-fps "${MUJOCO_WRIST_CAMERA_FPS:-15}"
        --wrist-depth-fps "${MUJOCO_WRIST_DEPTH_FPS:-0.2}"
        --wrist-camera-max-range "${MUJOCO_WRIST_CAMERA_MAX_RANGE:-3.0}"
        --right-wrist-camera-topic "${RIGHT_WRIST_CAMERA_TOPIC:-/right_wrist_camera/color/image_raw}"
        --right-wrist-depth-topic "${RIGHT_WRIST_DEPTH_TOPIC:-/right_wrist_camera/depth/image_raw}"
        --right-wrist-camera-info-topic "${RIGHT_WRIST_CAMERA_INFO_TOPIC:-/right_wrist_camera/color/camera_info}"
        --left-wrist-camera-topic "${LEFT_WRIST_CAMERA_TOPIC:-/left_wrist_camera/color/image_raw}"
        --left-wrist-depth-topic "${LEFT_WRIST_DEPTH_TOPIC:-/left_wrist_camera/depth/image_raw}"
        --left-wrist-camera-info-topic "${LEFT_WRIST_CAMERA_INFO_TOPIC:-/left_wrist_camera/color/camera_info}"
    )
else
    bridge_args+=(--no-wrist-camera)
fi
if [ "${MUJOCO_OVERVIEW_CAMERA_ENABLED:-0}" = "1" ]; then
    bridge_args+=(
        --overview-camera
        --overview-camera-fps "${MUJOCO_OVERVIEW_CAMERA_FPS:-2}"
        --overview-camera-topic "${MUJOCO_OVERVIEW_CAMERA_TOPIC:-/overview_camera/color/image_raw}"
    )
else
    bridge_args+=(--no-overview-camera)
fi
if [ "${MUJOCO_VIEWER:-0}" = "1" ]; then
    bridge_args+=(--viewer)
fi

start_bg mujoco_sim_bridge "${bridge_args[@]}"
bridge_pid="${LAST_BG_PID}"
sleep 1
fail_if_exited "mujoco_sim_bridge" "${bridge_pid}" "${LOG_DIR}/mujoco_sim_bridge.log"
wait_for "MuJoCo right action server" \
    "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'; export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'; ros2 action list | grep -q '/arm_controller/follow_joint_trajectory'" \
    20
fail_if_exited "mujoco_sim_bridge" "${bridge_pid}" "${LOG_DIR}/mujoco_sim_bridge.log"
wait_for "MuJoCo left action server" \
    "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'; export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'; ros2 action list | grep -q '/left_arm_controller/follow_joint_trajectory'" \
    20
fail_if_exited "mujoco_sim_bridge" "${bridge_pid}" "${LOG_DIR}/mujoco_sim_bridge.log"
wait_for "MuJoCo gripper services" \
    "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'; export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'; ros2 service list | grep -q '/master2/sim_gripper' && ros2 service list | grep -q '/master0/sim_gripper'" \
    20
fail_if_exited "mujoco_sim_bridge" "${bridge_pid}" "${LOG_DIR}/mujoco_sim_bridge.log"

start_bg moveit "${ROOT_DIR}/start_robot_moveit.sh"
moveit_pid="${LAST_BG_PID}"
wait_for "MoveIt services" \
    "source /opt/ros/humble/setup.bash; [ -f '${ROOT_DIR}/install/setup.bash' ] && source '${ROOT_DIR}/install/setup.bash'; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'; export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'; ros2 service list | grep -q '/compute_ik' && ros2 service list | grep -q '/plan_kinematic_path'" \
    45
fail_if_exited "moveit" "${moveit_pid}" "${LOG_DIR}/moveit.log"

start_bg web_control "${ROOT_DIR}/run_web_control.sh"
web_pid="${LAST_BG_PID}"
sleep 1
fail_if_exited "web_control" "${web_pid}" "${LOG_DIR}/web_control.log"
wait_for "Web Control HTTP" \
    "curl -fsS --max-time 2 http://127.0.0.1:${WEB_CONTROL_PORT:-8765}/api/status" \
    45
fail_if_exited "web_control" "${web_pid}" "${LOG_DIR}/web_control.log"

echo "=== MuJoCo simulation stack started ==="
echo "Robot Web Control: http://127.0.0.1:${WEB_CONTROL_PORT:-8765}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE: ${ROS_AUTOMATIC_DISCOVERY_RANGE}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
echo "Web source URDF: ${SOURCE_URDF}"
echo "MuJoCo generated URDF: ${MODEL_URDF}"
echo "MuJoCo model: ${MODEL_PATH}"
echo "Camera extrinsic: ${CAMERA_EXTRINSIC_FILE}"
echo "Platform obstacle disabled: ${WEB_CONTROL_DISABLE_PLATFORM_OBSTACLE}"
echo "MuJoCo viewer: ${MUJOCO_VIEWER}"
echo "MuJoCo RGB camera: ${MUJOCO_CAMERA_ENABLED:-1} (${MUJOCO_CAMERA_WIDTH:-640}x${MUJOCO_CAMERA_HEIGHT:-480} @ ${MUJOCO_CAMERA_FPS:-5} Hz)"
echo "MuJoCo depth camera: /mujoco_camera/depth/image_raw (32FC1 meters, max=${MUJOCO_CAMERA_MAX_RANGE:-10.0} m)"
echo "MuJoCo wrist RGB-D cameras: ${MUJOCO_WRIST_CAMERA_ENABLED:-1} (${MUJOCO_WRIST_CAMERA_WIDTH:-480}x${MUJOCO_WRIST_CAMERA_HEIGHT:-360} @ ${MUJOCO_WRIST_CAMERA_FPS:-15} Hz)"
echo "MuJoCo overview camera: ${MUJOCO_OVERVIEW_CAMERA_ENABLED:-0} (@ ${MUJOCO_OVERVIEW_CAMERA_FPS:-2} Hz)"
echo "MuJoCo GL backend: ${MUJOCO_GL:-default}"
echo "Set MUJOCO_VIEWER=0 before launch to disable the Python MuJoCo viewer."

if [[ -t 1 ]]; then
    echo "按 Ctrl+C 停止并退出日志跟踪"
    echo ""
    tail -f "${LOG_DIR}/mujoco_sim_bridge.log" "${LOG_DIR}/moveit.log" "${LOG_DIR}/web_control.log"
fi
