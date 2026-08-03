#!/usr/bin/env bash
set -eo pipefail

if [[ ${EUID} -eq 0 ]]; then
  echo "Error: do not run this script with sudo; it manages sudo only for EtherCAT checks/capabilities." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}"
# 兼容本地包内 install 和 workspace 根 install
for candidate in "${WORKSPACE}/install" "${WORKSPACE}/../install"; do
  if [[ -f "${candidate}/setup.bash" && -e "${candidate}/light_test/lib/light_test/master2_moveit_bridge" ]]; then
    INSTALL_DIR="${candidate}"
    break
  fi
done
if [[ -z "${INSTALL_DIR:-}" ]]; then
  for candidate in "${WORKSPACE}/install" "${WORKSPACE}/../install"; do
    if [[ -f "${candidate}/setup.bash" ]]; then
      INSTALL_DIR="${candidate}"
      break
    fi
  done
fi
SDK_LIB="${WORKSPACE}/third_party/niic_ecat/lib"
BINARY="${INSTALL_DIR:-${WORKSPACE}/install}/light_test/lib/light_test/master2_moveit_bridge"

CALIBRATION_RIGHT="${WORKSPACE}/src/light_test/config/master2_joints.yaml"
CONTROLLER_RIGHT="${WORKSPACE}/src/light_test/config/master2_moveit_bridge.yaml"
CALIBRATION_LEFT="${WORKSPACE}/src/light_test/config/master0_joints.yaml"
CONTROLLER_LEFT="${WORKSPACE}/src/light_test/config/master0_moveit_bridge.yaml"
SUDO_PASS="${SUDO_PASS:-}"

sudo_run() {
  if [[ -n "${SUDO_PASS}" ]]; then
    printf '%s\n' "${SUDO_PASS}" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

if pgrep -f "light_test/lib/light_test/(master2_axis0|master2_encoder_publisher|master2_moveit_bridge)( |$)" >/dev/null; then
  echo "Error: a Master 2 EtherCAT process is already running." >&2
  echo "Stop it explicitly before starting the MoveIt bridge." >&2
  exit 1
fi

if command -v ethercatctl >/dev/null 2>&1; then
  echo "Checking EtherCAT masters..."
  if [[ -t 0 ]]; then
    CAN_SUDO=1
  elif [[ -n "${SUDO_PASS}" ]] && printf '%s\n' "${SUDO_PASS}" | sudo -S -v >/dev/null 2>&1; then
    CAN_SUDO=1
  elif sudo -n true 2>/dev/null; then
    CAN_SUDO=1
  else
    CAN_SUDO=0
    echo "Warning: no terminal/sudo credential; skipping automatic EtherCAT check."
    echo "Ensure sudo ethercatctl status shows Master0 and Master2 running."
  fi
fi

if [[ ${CAN_SUDO:-0} -gt 0 ]]; then
  ethercat_status="$(sudo_run ethercatctl status)"
  if ! grep -Eq '^Master0[[:space:]]+running$' <<<"${ethercat_status}" || \
     ! grep -Eq '^Master2[[:space:]]+running$' <<<"${ethercat_status}"; then
    echo "One or more masters not running; restarting EtherCAT masters..."
    sudo_run ethercatctl restart
    sleep 2
    ethercat_status="$(sudo_run ethercatctl status)"
  fi
  if ! grep -Eq '^Master0[[:space:]]+running$' <<<"${ethercat_status}"; then
    echo "Error: EtherCAT Master0 did not start." >&2
    printf '%s\n' "${ethercat_status}" >&2
    exit 1
  fi
  if ! grep -Eq '^Master2[[:space:]]+running$' <<<"${ethercat_status}"; then
    echo "Error: EtherCAT Master2 did not start." >&2
    printf '%s\n' "${ethercat_status}" >&2
    exit 1
  fi
  echo "EtherCAT Master0 and Master2 are running."
fi

# Sync calibration parameters from *_joints.yaml into *_moveit_bridge.yaml
python3 - "${CALIBRATION_RIGHT}" "${CONTROLLER_RIGHT}" "${CALIBRATION_LEFT}" "${CONTROLLER_LEFT}" <<'PY'
import os
import sys
import tempfile
import yaml

cal_right, ctrl_right, cal_left, ctrl_left = sys.argv[1:]

def sync(cal_path, ctrl_path):
    with open(cal_path) as f:
        calibration = yaml.safe_load(f)
    # Find the parameter namespace (e.g. master2_encoder_publisher)
    cal_ns = list(calibration.keys())[0]
    cal_params = calibration[cal_ns]["ros__parameters"]
    with open(ctrl_path) as f:
        controller = yaml.safe_load(f)
    ctrl_ns = list(controller.keys())[0]
    params = controller[ctrl_ns]["ros__parameters"]
    for key in ("zero_offsets", "directions", "encoder_counts_per_rev", "gear_ratios"):
        params[key] = cal_params[key]
    directory = os.path.dirname(ctrl_path)
    fd, temporary = tempfile.mkstemp(prefix=".moveit-bridge-", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(controller, f, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, ctrl_path)
        os.chmod(ctrl_path, 0o664)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    print(f"Synced {cal_path} -> {ctrl_path}")

sync(cal_right, ctrl_right)
sync(cal_left, ctrl_left)
PY

if [[ ! -e "${BINARY}" ]]; then
  echo "Error: ${BINARY} not found. Build light_test before starting the MoveIt bridge." >&2
  exit 1
fi
BINARY_REAL="$(readlink -f "${BINARY}")"
if file "${BINARY_REAL}" 2>/dev/null | grep -q 'ELF'; then
  if ! getcap "${BINARY_REAL}" 2>/dev/null | grep -q cap_net_raw; then
    echo "Setting EtherCAT capabilities on ${BINARY_REAL}"
    sudo_run setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep "${BINARY_REAL}"
  fi
else
  echo "Skipping setcap: ${BINARY_REAL} is not an ELF binary (colcon wrapper script)"
fi

source /opt/ros/humble/setup.bash
if [[ -n "${INSTALL_DIR:-}" ]]; then
  source "${INSTALL_DIR}/setup.bash"
else
  source "${WORKSPACE}/install/setup.bash"
fi
export LD_LIBRARY_PATH="${SDK_LIB:?}:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=25
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

start_bridge() {
  local config="$1"
  local node_name="$2"
  echo "Starting ${node_name}..."
  ros2 run light_test master2_moveit_bridge --ros-args \
    --params-file "${config}" \
    --remap __node:="${node_name}" &
}

echo "Starting both arm bridges. Enable only after checking E-stop and raw limits:"
echo "  Right: ros2 service call /master2/enable_motors std_srvs/srv/SetBool '{data: true}'"
echo "  Left:  ros2 service call /master0/enable_motors std_srvs/srv/SetBool '{data: true}'"

start_bridge "${CONTROLLER_RIGHT}" "master2_moveit_bridge"
start_bridge "${CONTROLLER_LEFT}" "master0_moveit_bridge"

wait
