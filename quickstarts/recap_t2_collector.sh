#!/usr/bin/env bash
# RECAP 采集 · 终端 2：collector 远程模式（本机常驻，监听 127.0.0.1:7931）
# 必须在真实终端里运行（每条 episode 结束要用键盘裁决 s/f/d；无 TTY 会卡在待裁决）。
# 同一 OUT_DIR 自动断点续采；改参数直接改下面的变量。
set -euo pipefail

# ---- 可改参数 ----
OUT_DIR="/home/jh/tianji_tools/data/recap_redcones_$(date +%Y%m%d)_ts1.5"  # 数据集根目录（默认一天一个；续采旧批次就改成那个目录）
REPO_ID="marvinpro/rollout"
SOURCE="autonomous"      # autonomous / demonstration / evaluation
POLICY_ITERATION=0
# ------------------

cd /home/jh/tianji_tools
exec uv run marvin-collector record \
  --bridge-host 6.6.7.100 \
  --repo-id "$REPO_ID" \
  --out "$OUT_DIR" \
  --remote-control-port 7931 \
  --source "$SOURCE" --policy-iteration "$POLICY_ITERATION"
