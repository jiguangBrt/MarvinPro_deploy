# Marvin Pro 当前真机 Rollout 基线

更新时间：2026-08-07

当前基线固定为：

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
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
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

PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
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