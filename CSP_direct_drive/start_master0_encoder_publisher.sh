#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}"
SDK_LIB="${WORKSPACE}/third_party/niic_ecat/lib"
PARAMS="${WORKSPACE}/src/light_test/config/master0_joints.yaml"
BINARY="${WORKSPACE}/install/light_test/lib/light_test/master2_encoder_publisher"

echo "启动 Master 0 七轴编码器发布器（左手）。"
echo "发布话题：/master0/joint_states、/master0/encoder_counts"
echo "注意：不要同时运行 master0_axis0。"

cd "${WORKSPACE}"

# Check/restart EtherCAT Master0
if command -v ethercatctl >/dev/null 2>&1; then
  echo "Checking EtherCAT masters..."
  if [[ -t 0 ]]; then
    SUDO=(sudo)
  elif sudo -n true 2>/dev/null; then
    SUDO=(sudo -n)
  else
    SUDO=()
    echo "Warning: no terminal/sudo credential; skipping automatic EtherCAT check."
    echo "Ensure sudo ethercatctl status shows Master0 running."
  fi
fi

if [[ ${#SUDO[@]} -gt 0 ]]; then
  ethercat_status="$("${SUDO[@]}" ethercatctl status)"
  if ! grep -Eq '^Master0[[:space:]]+running$' <<<"${ethercat_status}"; then
    echo "Master0 is not running; restarting EtherCAT masters..."
    "${SUDO[@]}" ethercatctl restart
    sleep 2
    ethercat_status="$("${SUDO[@]}" ethercatctl status)"
  fi
  if ! grep -Eq '^Master0[[:space:]]+running$' <<<"${ethercat_status}"; then
    echo "Error: EtherCAT Master0 did not start." >&2
    printf '%s\n' "${ethercat_status}" >&2
    exit 1
  fi
  echo "EtherCAT Master0 is running."
fi

# Set capabilities on the real ELF binary (follow symlinks from install space)
BINARY_REAL="$(readlink -f "${BINARY}")"
if file "${BINARY_REAL}" 2>/dev/null | grep -q 'ELF'; then
  if ! getcap "${BINARY_REAL}" 2>/dev/null | grep -q cap_net_raw; then
    echo "Setting EtherCAT capabilities on ${BINARY_REAL}"
    sudo setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep "${BINARY_REAL}"
  fi
else
  echo "Skipping setcap: ${BINARY_REAL} is not an ELF binary (colcon wrapper script)"
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
export LD_LIBRARY_PATH="${SDK_LIB:?}:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=25
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

exec ros2 run light_test master2_encoder_publisher --ros-args \
  --params-file "${PARAMS}"
