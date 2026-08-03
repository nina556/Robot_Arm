#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
# 兼容本地包内 install 和 workspace 根 install
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/install/setup.bash"
elif [[ -f "${SCRIPT_DIR}/CSP_direct_drive/install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/CSP_direct_drive/install/setup.bash"
fi
export ROS_DOMAIN_ID=25
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 交互式菜单复用 CSP_direct_drive 中的 Python 实现
MENU_SCRIPT="${SCRIPT_DIR}/CSP_direct_drive/master2_control_menu.py"

run_command() {
  case "$1" in
    status)
      echo "Right arm:"
      echo -n "  motors_enabled: "
      timeout 5 ros2 topic echo /master2/motors_enabled --once --field data || true
      echo -n "  emergency_stop: "
      timeout 5 ros2 topic echo /master2/emergency_stop_state --once --field data || true
      echo "Left arm:"
      echo -n "  motors_enabled: "
      timeout 5 ros2 topic echo /master0/motors_enabled --once --field data || true
      echo -n "  emergency_stop: "
      timeout 5 ros2 topic echo /master0/emergency_stop_state --once --field data || true
      ;;
    enable)
      ros2 service call /master2/enable_motors std_srvs/srv/SetBool "{data: true}"
      ros2 service call /master0/enable_motors std_srvs/srv/SetBool "{data: true}"
      ;;
    disable)
      ros2 service call /master2/enable_motors std_srvs/srv/SetBool "{data: false}"
      ros2 service call /master0/enable_motors std_srvs/srv/SetBool "{data: false}"
      ;;
    estop)
      ros2 topic pub --once /master2/emergency_stop std_msgs/msg/Bool "{data: true}"
      ros2 topic pub --once /master0/emergency_stop std_msgs/msg/Bool "{data: true}"
      ;;
    reset)
      ros2 service call /master2/reset_emergency_stop std_srvs/srv/Trigger "{}"
      ros2 service call /master0/reset_emergency_stop std_srvs/srv/Trigger "{}"
      ;;
    *)
      echo "Usage: $0 {status|enable|disable|estop|reset}" >&2
      return 2
      ;;
  esac
}

draw_menu() {
  clear
  cat <<'MENUEOF'
========================================
       0&2
========================================

  [e] 双手使能电机（再按一次 e 确认）
  [d] 双手正常停止并取消使能
  [空格] 双手紧急快速停止
  [r] 双手复位急停（复位后仍未使能）
  [s] 查看当前状态
  [h] 双手回 0 点（再按一次 h 确认）
  [1~6] 直接设置右手 1~6 倍速度
  [键 / ]键：右手每次降低 / 提高 0.5 倍
  [q] 退出菜单

按键无需回车，输入后立即执行。
========================================
MENUEOF
}

if [[ $# -eq 0 ]]; then
  if [[ ! -f "$MENU_SCRIPT" ]]; then
    echo "错误：未找到 ${MENU_SCRIPT}" >&2
    exit 1
  fi
  exec python3 "$MENU_SCRIPT"
else
  run_command "$1"
fi
