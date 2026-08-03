#!/usr/bin/env bash
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

if [ -f /workspace/unoarm/install/setup.bash ]; then
    set +u
    source /workspace/unoarm/install/setup.bash
    set -u
fi

if [ -f /workspace/unoarm/CSP_direct_drive/install/setup.bash ]; then
    set +u
    source /workspace/unoarm/CSP_direct_drive/install/setup.bash
    set -u
fi

exec "$@"
