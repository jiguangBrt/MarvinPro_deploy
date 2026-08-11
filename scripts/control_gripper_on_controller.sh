#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${MARVINPRO_REMOTE_HOST:-nvidia@6.6.7.100}"
REMOTE_DIR="${MARVINPRO_REMOTE_DIR:-/tmp/MarvinPro_deploy}"

if (($# == 0)); then
  cat >&2 <<'EOF'
Usage: ./scripts/control_gripper_on_controller.sh {0|1} [--side left|right|both]

  0  fully open (default: both grippers)
  1  fully closed (default: both grippers)
EOF
  exit 2
fi

case "$1" in
  0|1) ;;
  *)
    echo "The first argument must be 0 (open) or 1 (closed)." >&2
    exit 2
    ;;
esac

echo "[1/2] Syncing gripper controller to ${REMOTE_HOST}:${REMOTE_DIR}"
rsync -az -e ssh \
  --include='/src/***' \
  --include='/pyproject.toml' \
  --exclude='*' \
  "${DEPLOY_ROOT}/" "${REMOTE_HOST}:${REMOTE_DIR}/"

remote_args=""
for arg in "$@"; do
  printf -v quoted_arg '%q' "${arg}"
  remote_args+=" ${quoted_arg}"
done

echo "[2/2] Sending direct gripper command"
remote_dir_quoted="$(printf '%q' "${REMOTE_DIR}")"
ssh "${REMOTE_HOST}" \
  "source /etc/apex/apex_ros_env.sh && cd ${remote_dir_quoted} && exec env PYTHONPATH=${remote_dir_quoted}/src:\${PYTHONPATH:-} python3 -m marvinpro_deploy.gripper_control${remote_args}"
