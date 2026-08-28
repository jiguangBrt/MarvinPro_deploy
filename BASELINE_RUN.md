# Marvin Pro 真机 Rollout 基线记录

本文汇总基线结论：最新为 2026-08-27/28 真机 A/B；2026-08-07 的旧 sync 基线保留在下方作历史记录。
当前推荐的生产配置和真机执行命令见 [`_HANDOFF.md`](_HANDOFF.md)；RTC 测试计划与全部带日期的测试记录见
[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md)。

## 2026-08-27/28 真机 A/B：legacy discrete 默认路径顿挫，sync 轨迹路径正确完成抓取

两次运行使用同一台 policy server（`192.168.50.73:8000`，checkpoint
`pi05_marvinpro_red_cones_slow/pi05_marvinpro_red_cones_slow_260826_full/79999`，prompt
"Stack the three red cones from right to left inside the white square to form a stable stack."）：

| 方案 | 配置 | 真机结果 |
| --- | --- | --- |
| A：本仓库旧“首次真机执行”默认命令 | `--rollout-schedule prefetch` + `--playback-mode discrete`（15 Hz 阶梯位置目标、约每 2 个 knot 硬替换队列、`control_hz=15`、`execute_steps=5`、`prefetch_steps=3`） | 动作非常差、明显顿挫，无法完成任务；该路径从未经过 RTC |
| B：legacy sync 仓库已验证配置 | synchronized + 100 Hz 插值 + `--playback-time-scale 2` + 每次执行 10 个 knot | 运动平滑，抓取动作正确完成 |

结论与后续：

- 方案 A 的顿挫根因是 legacy discrete 路径本身（15 Hz 阶梯目标 + 硬队列替换），`_HANDOFF.md`
  历史档案中早有抖动记录；**不能据此评价 RTC**，该路径只保留为 A/B 对照基线，禁止用于任务执行。
- 方案 B 遗留问题：堆叠放置位置仍偏数厘米；目前怀疑是 sync 模式 chunk 间停顿造成的伪影，
  待本仓库 RTC 实际 merge 真机运行后复查。
- 注意参数差异：legacy sync 仓库的基线是 `--playback-time-scale 2 --execute-steps 10`；本仓库的
  argparse 强制 trajectory schedule 使用 `--playback-time-scale 3 --execute-steps 20`（固定 5 Hz
  knot rate、H=20），旧参数会被本仓库客户端直接拒绝。
- `MarvinPro_deploy_legacy_sync` 仓库需要 `/tj` topic 命名空间和 H264 解码补丁才能工作：机器人现在以
  `apex_ros_namespace:=tj` 运行，相机发布 h264 四宫格。

## 2026-08-07 历史基线（已过期，仅供查阅）

当时的基线配置：

```text
严格同步调度 + 100 Hz线性插值 + 15 Hz模型节点 + 2.0倍时间尺度 + 每次执行10个节点
```

- 2.0x 时间尺度消除了异步重规划造成的周期回弹，是当时验证过的最慢安全档；同步模式在每个 chunk
  之间等待到位、保持、重新观测和远程推理，可见的短暂停顿属于该基线的预期行为。
- 使用旧 checkpoint `pi05_marvinpro_red_cones/marvinpro_red_cones_40k_gpu67/39999` 和旧 prompt
  "Stack all three red cones into one stable stack."，均已过期。
- 客户端参数：`--episode-seconds 180 --rollout-schedule synchronized --playback-mode interpolated
  --control-hz 100 --model-hz 15 --playback-time-scale 2 --execute-steps 10`；其中
  `--playback-time-scale 2 --execute-steps 10` 已被本仓库 argparse 拒绝，不可直接复用。
- 当时预计完整任务约 55 个 chunk；客户端按墙钟停止，不保证严格执行固定推理次数。

当时的完整启动命令（policy server / bridge / rollout 三段）已由 [`_HANDOFF.md`](_HANDOFF.md) 的
“当前生产配置”和“真机快速开始”取代，此处不再保留。
