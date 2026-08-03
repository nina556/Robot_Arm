#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSP_DIR="${ROOT_DIR}/CSP_direct_drive"

if [ ! -x "${CSP_DIR}/start_moveit_bridge.sh" ]; then
    echo "ERROR: ${CSP_DIR}/start_moveit_bridge.sh not found or not executable." >&2
    echo "Restore CSP_direct_drive into ${ROOT_DIR}, then build light_test." >&2
    exit 1
fi

exec "${CSP_DIR}/start_moveit_bridge.sh"
