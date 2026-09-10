#!/usr/bin/env bash
# RECAP 采集 · 终端 1：collector 数据 bridge（控制器侧常驻，端口 7331）
# 启动顺序：1 -> 2 -> 3 -> 4。看到 "TCP bridge listening on 0.0.0.0:7331" 即就绪。
# 脚本会自动 rsync collector 代码到控制器；Ctrl+C 结束。
set -euo pipefail

cd /home/jh/tianji_tools/marvinpro_collector
exec ./scripts/run_bridge_on_controller.sh
