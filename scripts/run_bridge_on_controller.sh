#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${MARVINPRO_REMOTE_HOST:-nvidia@6.6.7.100}"
REMOTE_DIR="${MARVINPRO_REMOTE_DIR:-/tmp/MarvinPro_deploy}"

LOCAL_LOG=""
bridge_args=()
while (($#)); do
  case "$1" in
    --local-log)
      if (($# < 2)); then
        echo "--local-log requires a path" >&2
        exit 2
      fi
      LOCAL_LOG="$2"
      shift 2
      ;;
    --local-log=*)
      LOCAL_LOG="${1#*=}"
      shift
      ;;
    *)
      bridge_args+=("$1")
      shift
      ;;
  esac
done
set -- "${bridge_args[@]}"

if [[ -n "${LOCAL_LOG}" ]]; then
  mkdir -p "$(dirname "${LOCAL_LOG}")"
  exec > >(tee -a "${LOCAL_LOG}") 2>&1
fi

printf '[log] started_at=%s deploy_commit=%s remote=%s remote_dir=%s bridge_args=' \
  "$(date --iso-8601=seconds)" \
  "$(git -C "${DEPLOY_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown)" \
  "${REMOTE_HOST}" \
  "${REMOTE_DIR}"
printf '%q ' "$@"
printf '\n'
if [[ -n "${LOCAL_LOG}" ]]; then
  echo "[log] local_log=$(realpath -m "${LOCAL_LOG}")"
fi

echo "[1/2] Syncing bridge code to ${REMOTE_HOST}:${REMOTE_DIR}"
rsync -az --delete -e ssh \
  --include='/src/***' \
  --include='/pyproject.toml' \
  --exclude='*' \
  "${DEPLOY_ROOT}/" "${REMOTE_HOST}:${REMOTE_DIR}/"

remote_args=""
serve_mode=true
for arg in "$@"; do
  printf -v quoted_arg '%q' "${arg}"
  remote_args+=" ${quoted_arg}"
  if [[ "${arg}" == "--doctor" ]]; then
    serve_mode=false
  fi
done

echo "[2/2] Starting bridge in the controller Apex environment"
remote_dir_quoted="$(printf '%q' "${REMOTE_DIR}")"
remote_cleanup=""
if [[ "${serve_mode}" == true ]]; then
  remote_cleanup="old_pids=\$(pgrep -f '^python3 -m marvinpro_deploy\\.robot_bridge( |$)' || true); if [[ -n \"\$old_pids\" ]]; then echo \"[bridge] stopping previous instance: \$old_pids\"; kill -INT \$old_pids 2>/dev/null || true; for _ in \$(seq 1 20); do pgrep -f '^python3 -m marvinpro_deploy\\.robot_bridge( |$)' >/dev/null || break; sleep 0.1; done; if pgrep -f '^python3 -m marvinpro_deploy\\.robot_bridge( |$)' >/dev/null; then pkill -TERM -f '^python3 -m marvinpro_deploy\\.robot_bridge( |$)' || true; sleep 0.5; fi; fi; "
fi

# A remote PTY forwards Ctrl+C to the bridge. exec makes the Python process the
# direct remote command, so the port is released when the operator stops it.
ssh -tt "${REMOTE_HOST}" \
  "source /etc/apex/apex_ros_env.sh && cd ${remote_dir_quoted} && ${remote_cleanup}exec env PYTHONPATH=${remote_dir_quoted}/src:\${PYTHONPATH:-} python3 -m marvinpro_deploy.robot_bridge${remote_args}"
