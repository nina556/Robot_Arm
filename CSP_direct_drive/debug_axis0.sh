#!/usr/bin/env bash
# Debug tool: single-axis EtherCAT test (no ROS 2 dependency).
#
# This is a MINIMAL hardware test — controls only Axis 0 of EtherCAT Master 2
# directly via the CLI.  Useful for:
#   - Verifying EtherCAT communication before running the full bridge
#   - Testing a single CiA402 servo drive in isolation
#   - Recording encoder zero positions
#
# For normal operation, use:
#   ./start_encoder_publisher.sh   (publish joint states)
#   ./start_moveit_bridge.sh       (full MoveIt trajectory execution)
#
# Commands after startup:
#   status   — read drive status word
#   enable   — run CiA402 enable sequence
#   zero     — save current position as software zero
#   rad 0.5  — move to 0.5 rad
#   count N  — move to raw encoder count N
#   disable  — disable drive
#   quit

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}"
ROS_SETUP="/opt/ros/humble/setup.bash"
PROJECT_SETUP="${WORKSPACE}/install/setup.bash"
SDK_LIB="${WORKSPACE}/third_party/niic_ecat/lib"
PROGRAM="${WORKSPACE}/install/light_test/lib/light_test/master2_axis0"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS 2 Humble not found at ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${PROJECT_SETUP}" || ! -x "${PROGRAM}" ]]; then
  echo "ERROR: program not built. Run:" >&2
  echo "  cd ${WORKSPACE}" >&2
  echo "  source ${ROS_SETUP}" >&2
  echo "  colcon build --packages-select light_test" >&2
  exit 1
fi

if [[ ! -d "${SDK_LIB}" ]]; then
  echo "ERROR: NIIC EtherCAT SDK not found at ${SDK_LIB}" >&2
  exit 1
fi

echo "=== EtherCAT Master 2 — Axis 0 Debug Tool ==="
echo "Type 'help' for available commands."
echo "Drive is DISABLED on startup."
echo ""

cd "${WORKSPACE}"
exec sudo bash -c "
  source '${ROS_SETUP}'
  source '${PROJECT_SETUP}'
  export LD_LIBRARY_PATH='${SDK_LIB}':"\${LD_LIBRARY_PATH:-}"
  exec ros2 run light_test master2_axis0
"

