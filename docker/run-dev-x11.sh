#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
display="${DISPLAY:-:0}"
image_name="unoarm-dev:humble"

cleanup_xauth=0
if [ -n "${UNOARM_DOCKER_XAUTH:-}" ]; then
    xauth_file="${UNOARM_DOCKER_XAUTH}"
    touch "${xauth_file}"
    chmod 0600 "${xauth_file}"
else
    xauth_file="$(mktemp "/tmp/unoarm-docker-$(id -u).XXXXXX.xauth")"
    cleanup_xauth=1
    trap 'if [ "${cleanup_xauth}" -eq 1 ]; then rm -f "${xauth_file}"; fi' EXIT
fi

if command -v xauth >/dev/null 2>&1; then
    if xauth nlist "${display}" >/tmp/unoarm-xauth-list 2>/dev/null && [ -s /tmp/unoarm-xauth-list ]; then
        sed -e 's/^..../ffff/' /tmp/unoarm-xauth-list | xauth -f "${xauth_file}" nmerge - >/dev/null 2>&1 || true
        rm -f /tmp/unoarm-xauth-list
    else
        rm -f /tmp/unoarm-xauth-list
        echo "warning: no xauth entry found for DISPLAY=${display}; you may need: xhost +local:" >&2
    fi
else
    echo "warning: host xauth command not found; you may need: xhost +local:" >&2
fi

export DISPLAY="${display}"
export UNOARM_DOCKER_XAUTH="${xauth_file}"

container_cli="${UNOARM_CONTAINER_CLI:-}"
if [ -z "${container_cli}" ]; then
    docker_version=""
    if command -v docker >/dev/null 2>&1; then
        docker_version="$(docker --version 2>&1 || true)"
    fi
    if command -v podman >/dev/null 2>&1 && echo "${docker_version}" | grep -qi podman; then
        container_cli="podman"
    elif command -v docker >/dev/null 2>&1; then
        container_cli="docker"
    elif command -v podman >/dev/null 2>&1; then
        container_cli="podman"
    else
        echo "error: neither docker nor podman was found" >&2
        exit 1
    fi
fi

if ! "${container_cli}" image inspect "${image_name}" >/dev/null 2>&1; then
    echo "Container image ${image_name} not found; build it first with:" >&2
    if [ "${container_cli}" = "podman" ]; then
        echo "  ./docker/build-dev.sh" >&2
    else
        echo "  ./docker/build-dev.sh" >&2
    fi
    exit 1
fi

run_args=()
if [ "${container_cli}" = "podman" ]; then
    if [ "${UNOARM_PODMAN_KEEP_ID:-0}" = "1" ]; then
        run_args+=(--userns=keep-id)
    else
        # Rootless Podman maps container root to the host user. Without keep-id,
        # bind mounts appear owned by container root, so the image's uid 1000
        # user cannot write workspace logs.
        run_args+=(--user 0:0)
    fi
elif [ "$(id -u)" = "0" ]; then
    # The checkout is under /root; keep generated logs and build files writable.
    run_args+=(--user 0:0)
fi
env_args=()
for name in \
    MUJOCO_VIEWER MUJOCO_GL MUJOCO_CAMERA_ENABLED MUJOCO_CAMERA_WIDTH \
    MUJOCO_CAMERA_HEIGHT MUJOCO_CAMERA_FPS MUJOCO_CAMERA_TOPIC \
    MUJOCO_CAMERA_FRAME_ID MUJOCO_CAMERA_NAME MUJOCO_CAMERA_DEPTH_TOPIC MUJOCO_CAMERA_DEPTH_FPS \
    MUJOCO_CAMERA_INFO_TOPIC MUJOCO_CAMERA_MAX_RANGE \
    MUJOCO_WRIST_CAMERA_ENABLED MUJOCO_WRIST_CAMERA_WIDTH \
    MUJOCO_WRIST_CAMERA_HEIGHT MUJOCO_WRIST_CAMERA_FPS MUJOCO_WRIST_DEPTH_FPS \
    MUJOCO_WRIST_CAMERA_MAX_RANGE RIGHT_WRIST_CAMERA_TOPIC \
    RIGHT_WRIST_DEPTH_TOPIC RIGHT_WRIST_CAMERA_INFO_TOPIC \
    LEFT_WRIST_CAMERA_TOPIC LEFT_WRIST_DEPTH_TOPIC LEFT_WRIST_CAMERA_INFO_TOPIC \
    MUJOCO_OVERVIEW_CAMERA_ENABLED MUJOCO_OVERVIEW_CAMERA_FPS \
    MUJOCO_OVERVIEW_CAMERA_TOPIC MUJOCO_FORCE_COMPILE MUJOCO_GRASP_TARGET_X \
    MUJOCO_GRASP_TARGET_Y MUJOCO_GRASP_TARGET_Z MUJOCO_GRASP_TCP_OFFSET_X \
    MUJOCO_GRASP_TCP_OFFSET_Y MUJOCO_GRASP_TCP_OFFSET_Z \
    MUJOCO_GRASP_OBJECT_TO_TCP_M MUJOCO_GRASP_LATERAL_OFFSET_M \
    MUJOCO_GRASP_OBJECT_MESH MUJOCO_GRASP_OBJECT_MESH_SCALE \
    MUJOCO_GRASP_OBJECT_MESH_QUAT \
    MUJOCO_GRASP_OBJECT_SIZE_X MUJOCO_GRASP_OBJECT_SIZE_Y \
    MUJOCO_GRASP_OBJECT_SIZE_Z UNOARM_GRIPPER_CLOSED_POSITION \
    MUJOCO_GRASP_TCP_TABLE_CLEARANCE_M \
    MUJOCO_GRASP_OBJECT_PLANNING_MARGIN \
    MUJOCO_SCENE_OBJECTS \
    MUJOCO_TABLE_CENTER_X MUJOCO_TABLE_CENTER_Y MUJOCO_TABLE_CENTER_Z \
    MUJOCO_TABLE_SIZE_X MUJOCO_TABLE_SIZE_Y MUJOCO_TABLE_SIZE_Z \
    MUJOCO_TABLE_PLANNING_GUARD_M \
    UNOARM_ANGLED_GRASP_X_GROUND_ANGLE_DEG UNOARM_ANGLED_GRASP_MAX_CANDIDATES \
    UNOARM_SIM_GRIPPER_SPEED_RAD_PER_SEC \
    UNOARM_SIM_GRIPPER_UNILATERAL_SPEED_RAD_PER_SEC \
    UNOARM_SIM_GRIPPER_SETTLE_TIMEOUT_SEC \
    UNOARM_SIM_GRASP_CAPTURE_TIMEOUT_SEC \
    UNOARM_SIM_GRIPPER_CONTACT_SETTLE_SEC \
    UNOARM_SIM_GRASP_POSE_DELAY_SEC UNOARM_SIM_GRASP_MIN_LIFT_DELTA_M \
    UNOARM_SIM_GRASP_CONTACT_STABLE_SEC \
    UNOARM_SIM_GRASP_GUARD_TIMEOUT_SEC \
    UNOARM_SIM_GRASP_WELD_ENABLED \
    UNOARM_GRIPPER_CLOSED_POSITION; do
    if [ -n "${!name+x}" ]; then
        env_args+=(--env "${name}=${!name}")
    fi
done

ros_domain_id="${ROS_DOMAIN_ID:-25}"
ros_discovery_range="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
ros_localhost_only="${ROS_LOCALHOST_ONLY:-0}"
ros_static_peers="${ROS_STATIC_PEERS:-192.168.10.200}"
network_args=(--network host)
container_identity_args=()
case "${1:-}" in
    ./start_mujoco_sim.sh|start_mujoco_sim.sh)
        ros_domain_id="${MUJOCO_ROS_DOMAIN_ID:-21}"
        if [[ "${ros_domain_id}" == "25" && "${UNOARM_ALLOW_MUJOCO_DOMAIN_25:-0}" != "1" ]]; then
            echo "ERROR: MuJoCo 仿真拒绝使用 ROS_DOMAIN_ID=25，避免和真机控制域混用。" >&2
            echo "       如确实要覆盖，请显式设置 UNOARM_ALLOW_MUJOCO_DOMAIN_25=1 MUJOCO_ROS_DOMAIN_ID=25。" >&2
            exit 2
        fi
        ros_discovery_range="${MUJOCO_ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
        ros_localhost_only="${MUJOCO_ROS_LOCALHOST_ONLY:-1}"
        ros_static_peers="${MUJOCO_ROS_STATIC_PEERS:-127.0.0.1}"
        # Docker Desktop does not expose Linux host-network ports back to WSL.
        # The complete simulation stack runs in this one container, so bridge
        # networking keeps ROS isolated while publishing the Web UI explicitly.
        network_args=(--publish "${WEB_CONTROL_PORT:-8765}:${WEB_CONTROL_PORT:-8765}")
        container_identity_args=(
            --label "unoarm.mujoco=1"
            --label "unoarm.workspace=${ROOT_DIR}"
        )
        ;;
esac

"${container_cli}" run --rm -it \
    "${run_args[@]}" \
    "${network_args[@]}" \
    "${container_identity_args[@]}" \
    --ipc host \
    --workdir /workspace/unoarm \
    --env DISPLAY="${DISPLAY}" \
    --env XAUTHORITY=/tmp/unoarm-docker.xauth \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID="${ros_domain_id}" \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE="${ros_discovery_range}" \
    --env ROS_LOCALHOST_ONLY="${ros_localhost_only}" \
    --env ROS_STATIC_PEERS="${ros_static_peers}" \
    --env RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    --env WEB_CONTROL_HOST="${WEB_CONTROL_HOST:-0.0.0.0}" \
    --env WEB_CONTROL_PORT="${WEB_CONTROL_PORT:-8765}" \
    --env UNOARM_HOST_UID="$(id -u)" \
    --env UNOARM_HOST_GID="$(id -g)" \
    --env UNOARM_IN_DOCKER=1 \
    --env TERM="${TERM:-xterm-256color}" \
    "${env_args[@]}" \
    --volume "${ROOT_DIR}:/workspace/unoarm" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${UNOARM_DOCKER_XAUTH:-/tmp/unoarm-docker.xauth}:/tmp/unoarm-docker.xauth:ro" \
    --volume unoarm_colcon_home:/home/unoarm/.colcon \
    "${image_name}" \
    "$@"
