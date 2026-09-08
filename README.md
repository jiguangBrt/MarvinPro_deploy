# Marvin Pro OpenPI 真机 Rollout

本目录实现三进程部署链路：

```text
Marvin Pro 6.6.7.100                 本机                         GPU 192.168.50.73
ROS topics <-> 双向 bridge:7332 <-> rollout client <-> WebSocket policy:8000
```

rollout 客户端运行在本机；机器人控制器只运行轻量 ROS bridge；GPU 服务器只运行 OpenPI policy server。

## 文档

- [`_HANDOFF.md`](_HANDOFF.md)：**唯一入口**。当前生产配置（policy server 启动命令、checkpoint、
  prompt）、真机快速开始（dry-run -> synchronized 回归 -> RTC shadow -> RTC 实际 merge）、
  日志约定、状态快照、待办与已知问题。
- [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md)：RTC/轨迹测试计划、调度与轨迹诊断工具、RTC 决策记录、
  异步 action chunk 错位分析，以及全部带日期的真机测试记录。
- [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md)：现场排障速查——按终端输出对照机器人为什么停了、
  怎么办（通俗版，面向操作员）。
- [`BASELINE_RUN.md`](BASELINE_RUN.md)：基线记录——2026-08-07 历史 sync 基线（旧 checkpoint/prompt）
  和 2026-08-27/28 真机 A/B 结论。

## 当前生产摘要（2026-08-28）

- policy server：`192.168.50.73:8000`，checkpoint
  `pi05_marvinpro_red_cones_slow/pi05_marvinpro_red_cones_slow_260826_full/79999`，完整启动命令见
  [`_HANDOFF.md`](_HANDOFF.md)。
- 任务 prompt：`Stack the three red cones from right to left inside the white square to form a
  stable stack.`（客户端必须显式传 `--prompt`）。
- 真机执行只用 synchronized / RTC 轨迹路径；默认的 prefetch+discrete 路径已确认严重顿挫，
  仅保留为 A/B 对照基线。
- 每次真机运行必须使用全新 `RUN_DIR` 记录 bridge/client 日志和 telemetry CSV。

## 本地测试

```bash
cd /home/jh/Openpi_deploy
uv sync
PYTHONPATH=src .venv/bin/python -m pytest tests/ -q
```
