#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_SCRIPT="${ROOT_DIR}/web_control/web_control/start_web_control.sh"

usage() {
    cat <<EOF
Usage: ./run_web_control.sh [OPTION]

Robot-side launcher for UNO Arm Web Control.

Options:
  (none)         Start robot Web Control
  --server-only  Same as default; kept for compatibility
  --check        Check robot ROS topics/actions, then exit
  --stop         Stop robot Web Control server
  --help         Show this help message

Useful environment variables:
  ROS_DOMAIN_ID                    default: 25
  ROS_AUTOMATIC_DISCOVERY_RANGE    default: SUBNET
  ROS_STATIC_PEERS                 default: 192.168.10.200
  WEB_CONTROL_HOST                 default: 0.0.0.0
  WEB_CONTROL_PORT                 default: 8765
  WEB_CONTROL_OCCUPANCY_TOPIC      default: /unoarm/web_control/occupied
EOF
}

ensure_scripts() {
    if [ ! -x "${WEB_SCRIPT}" ]; then
        chmod +x "${WEB_SCRIPT}"
    fi
}

ensure_scripts

case "${1:-}" in
    --help|-h)
        usage
        ;;
    --server-only)
        exec "${WEB_SCRIPT}" --server-only
        ;;
    --check)
        exec "${WEB_SCRIPT}" --check
        ;;
    --stop)
        exec "${WEB_SCRIPT}" --stop
        ;;
    "")
        exec "${WEB_SCRIPT}"
        ;;
    *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 1
        ;;
esac
