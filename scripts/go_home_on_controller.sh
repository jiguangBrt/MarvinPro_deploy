#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${MARVINPRO_REMOTE_HOST:-nvidia@6.6.7.100}"
REMOTE_DIR="${MARVINPRO_REMOTE_DIR:-/tmp/MarvinPro_deploy}"

if [[ " $* " != *" --check "* ]]; then
  cat >&2 <<'EOF'
This moves both arms to the recorded stack_two_cones home pose through the
official movej planner (0.2 rad/s, 0.5 rad/s^2 trapezoidal profile), then
opens both grippers (--keep-grippers skips that; anything held is released
at the home pose). Requires: robot already Robot Ready in Apex, rollout
bridge stopped, emergency stop within reach. Use --check to only report
the current pose and its distance to home without moving.
EOF
fi

echo "[1/2] Syncing deploy code to ${REMOTE_HOST}:${REMOTE_DIR}"
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

echo "[2/2] Running go_home on the controller"
remote_dir_quoted="$(printf '%q' "${REMOTE_DIR}")"
ssh "${REMOTE_HOST}" \
  "source /etc/apex/apex_ros_env.sh && cd ${remote_dir_quoted} && exec env PYTHONPATH=${remote_dir_quoted}/src:\${PYTHONPATH:-} python3 -m marvinpro_deploy.go_home${remote_args}"
