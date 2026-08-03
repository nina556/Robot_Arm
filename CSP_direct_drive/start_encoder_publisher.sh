#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}"
SDK_LIB="${WORKSPACE}/third_party/niic_ecat/lib"
PARAMS="${WORKSPACE}/src/light_test/config/master2_joints.yaml"
BINARY="${WORKSPACE}/install/light_test/lib/light_test/master2_encoder_publisher"

echo "启动 Master 2 七轴编码器发布器。"
echo "发布话题：/master2/joint_states、/master2/encoder_counts"
echo "注意：不要同时运行 master2_axis0。"

cd "${WORKSPACE}"

# Set capabilities on the real ELF binary (follow symlinks from install space)
BINARY_REAL="$(readlink -f "${BINARY}")"
if file "${BINARY_REAL}" 2>/dev/null | grep -q 'ELF'; then
  if ! getcap "${BINARY_REAL}" 2>/dev/null | grep -q cap_net_raw; then
    echo "正在设置 EtherCAT 网卡权限..."
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
