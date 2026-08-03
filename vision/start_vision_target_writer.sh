#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -n "${VISION_PYTHON:-}" ]]; then
  PYTHON_BIN="${VISION_PYTHON}"
elif [[ -x "${ROOT_DIR}/.venv/bin/python3" ]]; then
  PYTHON_BIN="${ROOT_DIR}/.venv/bin/python3"
else
  PYTHON_BIN="$(command -v python3 || true)"
fi

if [[ -z "${PYTHON_BIN}" || ! -x "${PYTHON_BIN}" ]]; then
  echo "ERROR: python3 not found. Set VISION_PYTHON or install python3." >&2
  exit 1
fi

exec "${PYTHON_BIN}" "${SCRIPT_DIR}/vision_target_writer.py" "$@"
