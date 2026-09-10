#!/usr/bin/env bash
# RECAP 采集 · 终端 4：rollout client（每条 episode 运行一次，每次自动新建 RUN_DIR）
# 流程：启动 -> 在 Apex 把 Input Mode 切到 Custom -> 确认没有 "WILL NOT BE RECORDED"
# 警告 -> 输入单个大写 E 开始动作。结束：跑满 EPISODE_SECONDS 或 Ctrl+C（发
# operator_stopped）；随后在 Apex 切回 None，client 自行断开；最后到终端 2 裁决 s/f/d，
# 裁决完再跑下一条。
set -euo pipefail

# ---- 可改参数 ----
PROMPT="Stack two red cones from right to left inside the white square to form a stable stack"
# RECAP 单条上限默认 120s/1800 帧（222 recap.py：num_frames>max_episode_frames 报错），
# 已通过 222 侧两个 build 脚本的 --max-episode-seconds 240 flag 放宽到 240s（无需改代码，
# 奖励归一化 -1/max_episode_frames 自动跟随；build_recap_value_targets.py 和
# build_recap_sidecar.py 两条命令都必须加该 flag，忘了加则 >120s 的 episode 会被显式拒收）。
# 230 = 240 上限留 10s 给 episode_end 的 drain 尾巴。
EPISODE_SECONDS=230
# 2026-09-05 起用 1.5（10Hz knot rate）：WiFi 链路下 15Hz 原速延迟超预算（实测 p95
# 270-320ms > 216ms 上限）导致 50-70% 时间回退成同步+保持，停顿伪影会毒害 RECAP 的
# value/advantage 标签；10Hz 预算 350ms 能容纳实测 p95，动作均匀变慢但平滑。
# 注意：改了它就相当于换了数据分布——不同 time-scale 的数据不要写进同一个数据集目录
# （终端 2 的 OUT_DIR，默认按天分目录；同一天内改 time-scale 要手动换目录）。
PLAYBACK_TIME_SCALE=1.5
# ------------------

RUN_DIR="/home/jh/Openpi_deploy/logs/recap_redcones_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
echo "[recap] client 日志 -> $RUN_DIR/client.log"

# 用 deploy 自己的 uv 环境（marvinpro_deploy 与 openpi-client 均已 editable 装入
# deploy/.venv，openpi-client 通过 pyproject 的 path 源指向 ../OpenPI_UR 的 checkout，
# 只需该目录存在，不需要 OpenPI_UR 的环境）。
cd /home/jh/Openpi_deploy
exec uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 --policy-host 192.168.50.73 \
  --prompt "$PROMPT" \
  --execute --episode-seconds "$EPISODE_SECONDS" \
  --rollout-schedule rtc --rtc-continuous \
  --rtc-late-result-policy discard \
  --playback-mode interpolated \
  --control-hz 100 --model-hz 15 --playback-time-scale "$PLAYBACK_TIME_SCALE" --execute-steps 20 \
  --max-stuck-replans 2 \
  --policy-connect-timeout 5 --policy-request-timeout 5 \
  --exit-mode-timeout 60 \
  --log-level DEBUG --console-log-level WARNING \
  --log-file "$RUN_DIR/client.log" \
  --record-notify-host 127.0.0.1
