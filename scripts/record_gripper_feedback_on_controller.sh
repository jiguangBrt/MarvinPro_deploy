#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${MARVINPRO_REMOTE_HOST:-nvidia@6.6.7.100}"
REMOTE_DIR="${MARVINPRO_REMOTE_DIR:-/tmp/MarvinPro_deploy}"
DURATION="0"
OUTPUT=""

usage() {
  cat >&2 <<'EOF'
Usage: ./scripts/record_gripper_feedback_on_controller.sh [--duration SECONDS] [--output PATH]

Read-only recorder for /tj/info/gripper_feedback_L/R.
Duration 0 (the default) means keep recording until Ctrl+C.
The CSV is copied back to the local machine when the recorder exits.
EOF
}

while (($#)); do
  case "$1" in
    --duration)
      (($# >= 2)) || { echo "--duration requires a value" >&2; exit 2; }
      DURATION="$2"
      shift 2
      ;;
    --duration=*)
      DURATION="${1#*=}"
      shift
      ;;
    --output)
      (($# >= 2)) || { echo "--output requires a path" >&2; exit 2; }
      OUTPUT="$2"
      shift 2
      ;;
    --output=*)
      OUTPUT="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -z "${OUTPUT}" ]]; then
  OUTPUT="${DEPLOY_ROOT}/logs/gripper_feedback_${timestamp}.csv"
fi
mkdir -p "$(dirname "${OUTPUT}")"

remote_output="/tmp/marvinpro_gripper_feedback_${timestamp}_$$.csv"
remote_dir_quoted="$(printf '%q' "${REMOTE_DIR}")"
remote_output_quoted="$(printf '%q' "${remote_output}")"
remote_args="--duration $(printf '%q' "${DURATION}") --output ${remote_output_quoted}"

echo "[1/3] Syncing recorder to ${REMOTE_HOST}:${REMOTE_DIR}"
rsync -az -e ssh \
  --include='/src/***' \
  --exclude='*' \
  "${DEPLOY_ROOT}/" "${REMOTE_HOST}:${REMOTE_DIR}/"

if [[ "${DURATION}" == "0" || "${DURATION}" == "0.0" ]]; then
  echo "[2/3] Recording read-only feedback until Ctrl+C"
else
  echo "[2/3] Recording read-only feedback for ${DURATION}s"
fi
set +e
ssh -tt "${REMOTE_HOST}" \
  "source /etc/apex/apex_ros_env.sh && cd ${remote_dir_quoted} && exec env PYTHONPATH=${remote_dir_quoted}/src:\${PYTHONPATH:-} python3 -m marvinpro_deploy.gripper_feedback_recorder ${remote_args}"
ssh_status=$?
set -e

echo "[3/3] Copying CSV to ${OUTPUT}"
if scp -q "${REMOTE_HOST}:${remote_output}" "${OUTPUT}"; then
  echo "Saved: ${OUTPUT}"
  ssh "${REMOTE_HOST}" "rm -f ${remote_output_quoted}" >/dev/null 2>&1 || true
else
  echo "Could not copy ${remote_output}; the remote recorder may not have produced a file." >&2
  exit "${ssh_status:-1}"
fi

exit "${ssh_status}"
