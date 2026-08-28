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

> 以下为 2026-08-07 的基线记录：2.0x 时间尺度 / 10 knot 的 synchronized 参数、旧 checkpoint
> `marvinpro_red_cones_40k_gpu67/39999` 和旧 prompt 均已过期，内容保留原样，不再逐条刷新。

```text
严格同步调度 + 100 Hz线性插值 + 15 Hz模型节点 + 2.0倍时间尺度 + 每次执行10个节点
```

不要切换到 `1.5x`。`2.0x` 已消除异步重规划造成的周期回弹，并降低机器人跟踪压力。同步模式会在
每个chunk之间等待到位、保持、重新观测和远程推理，因此可见的短暂停顿属于当前基线行为。

部署链路：

```text
Marvin Pro 6.6.7.100                 本机                         GPU 192.168.50.73
ROS topics <-> 双向 bridge:7332 <-> rollout client <-> WebSocket policy:8000
```

## 0. Apex准备

开始前完成：

- Robot Ready、Camera启动、双臂进入Joint Impedance。
- 机器人和三个红色圆锥恢复到训练数据对应的安全初始状态。
- 急停保持可触及，工作区内没有人员或无关物体。
- Apex Input Mode先保持 **None**；客户端提示后才能切换到 **Custom**。

## 1. GPU服务器启动policy

在 `192.168.50.73` 的终端运行：

```bash
cd /mnt/reacher-fast/openpi_ur_pp_202607/repo

CUDA_VISIBLE_DEVICES=5 \
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/serve_policy.py \
  --port 8000 \
  --default-prompt "Stack all three red cones into one stable stack." \
  policy:checkpoint \
  --policy.config=pi05_marvinpro_red_cones \
  --policy.dir=/mnt/reacher-fast/openpi_ur_pp_202607/repo/checkpoints/pi05_marvinpro_red_cones/marvinpro_red_cones_40k_gpu67/39999
```

看到下面的日志后保持该终端运行：

```text
server listening on 0.0.0.0:8000
```

## 2. 本机启动100 Hz bridge

在本机第二个终端运行：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion --publish-hz 100
```

脚本会停止控制器上的旧bridge并启动新实例。看到下面的日志后保持该终端运行：

```text
bridge initialized at 100.0Hz, MOTION ENABLED
[bridge] listening on 0.0.0.0:7332
canonical /joint_states mapping established
```

## 3. 本机启动完整任务rollout

在本机第三个终端运行：

```bash
cd /home/jh/OpenPI_UR/openpi

PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 180 \
  --rollout-schedule synchronized \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 2 \
  --execute-steps 10
```

`180s` 是允许启动新chunk的墙钟时限。已经启动的chunk仍会完整执行、跟踪到位、保持并重新观测，
所以总运行时间可能略超过180秒。当前预计完整任务需要约55个chunk；客户端目前按时间停止，不保证
严格执行55次推理。

客户端完成连接和warmup后会显示：

```text
Now change Apex Input Mode to Custom.
```

此时才在Apex中将Input Mode从 **None** 切到 **Custom**。确认终端显示以下参数：

```text
playback time scale: 2.00x
effective knot rate: 7.50Hz
command rate: 100.0Hz
selected chunk: 10 knots over 1.333s
rollout schedule: execute -> track -> hold -> observe -> infer
```

确认现场安全后，在客户端终端输入：

```text
EXECUTE
```