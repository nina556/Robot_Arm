#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${UNOARM_DOCKER_IMAGE:-unoarm-dev:humble}"
DOCKERFILE="${ROOT_DIR}/docker/unoarm-dev.Dockerfile"
USER_UID="${USER_UID:-$(id -u)}"
USER_GID="${USER_GID:-$(id -g)}"

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

cd "${ROOT_DIR}"

if [ "${container_cli}" = "podman" ]; then
    exec podman build \
        --network=host \
        --format docker \
        --build-arg "USER_UID=${USER_UID}" \
        --build-arg "USER_GID=${USER_GID}" \
        -t "${IMAGE_NAME}" \
        -f "${DOCKERFILE}" \
        .
fi

exec docker build \
    --build-arg "USER_UID=${USER_UID}" \
    --build-arg "USER_GID=${USER_GID}" \
    -t "${IMAGE_NAME}" \
    -f "${DOCKERFILE}" \
    .
