#!/usr/bin/env bash
# RECAP 采集 · 终端 3：deploy 运动 bridge（控制器侧常驻，端口 7332）
# 看到 "listening on 0.0.0.0:7332; motion_allowed=True" 即就绪。
# 脚本会自动 rsync deploy 代码到控制器并替换旧 bridge；Ctrl+C 结束。
set -euo pipefail

cd /home/jh/Openpi_deploy
RUN_DIR="logs/recap_bridge_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
echo "[recap] bridge 日志 -> $PWD/$RUN_DIR/bridge.log"
# blend 包络自 2026-09-08 起对所有 knot rate 使用宽松默认
# （trajectory_timeline.DEFAULT_BLEND_CAPS 3.2/100/5000，速度另受 URDF 逐关节上限
# 约束），不再需要 --rtc-blend-max-* 覆盖；如需重新收紧可显式传这三个参数。
exec ./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion --publish-hz 100
