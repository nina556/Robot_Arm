#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${ROOT_DIR}/logs/mujoco_sim_stack.pids"

restore_log_owner() {
    if command -v podman >/dev/null 2>&1; then
        podman unshare chown -R 0:0 "${ROOT_DIR}/logs" >/dev/null 2>&1 || true
    fi
}

stop_docker_wrappers() {
    local pids=()
    while IFS= read -r pid; do
        pids+=("${pid}")
    done < <(
        ps -eo pid=,args= |
            awk -v root="${ROOT_DIR}" '
                $0 ~ "podman run|docker run" &&
                $0 ~ root &&
                $0 ~ "unoarm-dev:humble" &&
                $0 ~ "./start_mujoco_sim.sh" {
                    print $1
                }
            '
    )
    for pid in "${pids[@]}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            echo "stopping docker mujoco wrapper: pid=${pid}"
            kill "${pid}" >/dev/null 2>&1 || true
        fi
    done
}

stop_docker_containers() {
    local cli ids=() id
    for cli in docker podman; do
        command -v "${cli}" >/dev/null 2>&1 || continue
        ids=()
        while IFS= read -r id; do
            [ -n "${id}" ] && ids+=("${id}")
        done < <(
            "${cli}" ps -q \
                --filter "label=unoarm.mujoco=1" \
                --filter "label=unoarm.workspace=${ROOT_DIR}" 2>/dev/null || true
        )
        for id in "${ids[@]}"; do
            echo "stopping docker mujoco container: id=${id}"
            "${cli}" stop "${id}" >/dev/null 2>&1 || true
        done
    done
}

restore_log_owner
if [ ! -f "${PID_FILE}" ]; then
    stop_docker_wrappers
    stop_docker_containers
    echo "=== No tracked MuJoCo simulation stack ==="
    exit 0
fi

while read -r name pid; do
    if [ -n "${pid:-}" ] && kill -0 "${pid}" >/dev/null 2>&1; then
        echo "stopping ${name}: pid=${pid}"
        kill "${pid}" >/dev/null 2>&1 || true
    fi
done <"${PID_FILE}"

sleep 1

while read -r name pid; do
    if [ -n "${pid:-}" ] && kill -0 "${pid}" >/dev/null 2>&1; then
        echo "force stopping ${name}: pid=${pid}"
        kill -9 "${pid}" >/dev/null 2>&1 || true
    fi
done <"${PID_FILE}"

stop_docker_wrappers
stop_docker_containers
restore_log_owner
rm -f "${PID_FILE}" || true
echo "=== Stopped tracked MuJoCo simulation stack ==="
