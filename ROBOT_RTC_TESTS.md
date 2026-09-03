# MarvinPro Tracking-Aware RTC Robot Test Checklist

> **2026-08-28 注**：本文汇总全部 RTC/轨迹测试材料——验收清单、诊断工具、RTC 决策记录和带日期的
> 真机测试记录（测试记录已精简，保留结论与关键数据）。命令块中的 checkout 路径已更新，但
> checkpoint、prompt 等参数保留当时记录，不再逐条刷新。当前推荐的真机执行命令（新 checkpoint
> `pi05_marvinpro_red_cones_slow`、新任务 prompt、全新 `RUN_DIR` 日志约定和 dry-run -> synchronized
> -> RTC shadow -> `--max-rtc-merges 1` 的执行顺序）以 [`_HANDOFF.md`](_HANDOFF.md) 为准；其中
> 测试3 的 RTC 配置仍是当前推荐的 RTC 验收参数。

本清单记录当前因真机未连接而不能执行的测试。不要让自动化脚本切换 Apex Input Mode、Robot Ready、
Impedance Mode、Home 或清故障；这些步骤必须由现场人员确认。远程推理端还必须先通过
`/home/jh/OpenPI_UR/openpi/REMOTE_RTC_TESTS.md`。

## 已离线验证

- protocol v10 序列化、旧版本拒绝、高频 state/image/event 分流、乱序 event 丢弃和发送公平性；
- feedback source timestamp 不会被 100 Hz timer 伪装成新反馈；
- 慢速一阶机器人会令 phase 减速/冻结，不按控制 tick 消费动作；
- H20/s10 下 A9 feedback 仍在 A8 时不产生 checkpoint，stale feedback 不能累计 `0.20 s` settle；
- fake bridge 完整执行 `Load -> A9 checkpoint -> Resume -> Stage -> integer-boundary merge`；
- merge 使用真实 `d_actual`，并将当前旧 reference 放到 `C[k-1]` 作为 anchor；
- merge 保留硬位置锚点，并以 quintic C2 blend 衔接速度和加速度（blend 窗口按秒恒定：5 Hz 下
  2～3 knot，15 Hz 下最长 9 knot）；不安全的 blend 原子拒绝；
- timed H20 chunk 在 5 秒 controller deadline 内 clean 完成；健康超时原子锁存实测臂位置且保留夹爪命令；
- RTC 失败使用结构化 reason code；可恢复故障经过一个 clean synchronized chunk 后重建 epoch，安全故障只 hold。
- fake bridge 已跑通 `RTC C2 reject -> measured hold -> sync clean -> bootstrap -> RTC merge`，并拒绝旧
  request/timeline；4.9 秒 clean checkpoint 不会被 5 秒 deadline 误报为 timeout。

离线测试不能证明控制器 ROS topic 频率、网络背压、真实关节响应或远程推理延迟满足要求。

### 2026-08-19 protocol v10 无真机现态结果

本轮没有连接控制器、没有启动 bridge、没有发送真机 action。要点：

- deploy 全套 `100 passed`；OpenPI WebSocket client 定向测试 `7 passed`；两端 Ruff 通过。
- 远程 smoke 通过：metadata 为 `rtc_v1`、H20/s10、`d_max=4`、`schedule=exp`，普通/RTC 输出均为
  有限 `(20,16)`；同一持久连接 20 次 RTC 请求 0 timeout，wall min/p50/p95/max
  `302.8/342.9/389.9/393.2 ms`，server infer p95 `184.6 ms`。5 Hz 加 50 ms guard 后 p95 对应
  `d_pred=3`，未超过 `d_max=4`。强制 recv timeout 后 reconnect 正常。
- 夹爪反馈已确认：`/tj/info/gripper_feedback_L/R` 五维信息可用；protocol v10 的 policy state、
  action state 和 RTC handoff anchor 使用实测 `q` 按 `0.0..1.25` 归一化；任一侧 feedback 缺失或
  过期关闭运动门；RTC tracking governor 只基于 14 个机械臂关节。
- 未验证项（只按文末“最小三组测试”执行）：protocol v10 controller 握手、真实 H264 observation、
  topic freshness、实机 5 秒 deadline、measured hold 无跳变、fallback 后的真实 RTC merge。

## 1. 网络与代理预检（只读）

先记录本机代理环境和路由。不要在路由不明确时启动 motion-enabled bridge。

```bash
env | rg -i '^(http|https|all|no)_proxy='
ip route get 6.6.7.100
ip route get 192.168.50.73
```

若 WebSocket 受代理影响，在当前终端显式加入私网地址；不要全局修改系统代理：

```bash
export NO_PROXY="${NO_PROXY:+${NO_PROXY},}6.6.7.100,192.168.50.73,127.0.0.1,localhost"
export no_proxy="$NO_PROXY"
```

确认 SSH 和 policy 端口可达：

```bash
ssh -o BatchMode=yes -o ConnectTimeout=5 nvidia@6.6.7.100 true
nc -vz -w 3 192.168.50.73 8000
```

通过条件：连接走机器人私网/指定 GPU 路由，没有未知 `ProxyCommand`，且不会把 pickle bridge 端口暴露到公网。

## 2. 控制器只读 doctor

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --doctor --duration 8
```

记录所有 topic 频率和最新值。`/tj/joint_states` 应足以支持 50 ms stale 门限；相机、左右夹爪、input mode、
robot state 和 arm state 均必须有消息。doctor 不通过时停止，不得通过放宽 timeout 继续。

## 3. protocol v10 dry-run

控制器启动不允许动作的 bridge：

```bash
./scripts/run_bridge_on_controller.sh --publish-hz 100
```

本机连接远程 policy，但不带 `--execute`。legacy dry-run 先确认协议、图像和 policy 输出；trajectory 模式按
设计要求必须带 `--execute`，所以不能在本阶段启用。

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --episode-seconds 10 \
  --log-level DEBUG
```

通过条件：没有版本错误、state/image stale、H264 饥饿或 policy shape/finite 错误。结束后保留完整日志。

## 4. 单 chunk tracking governor

为每轮运动测试建立独立目录，并在两个本机终端中设置为同一个绝对路径：

```bash
export RUN_DIR=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/logs/tracking_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_tracking_run_dir
```

`--local-log` 保存控制器 bridge 的 ROS/trajectory event 输出，`--log-file` 保存客户端配置、policy metadata、
100 Hz trajectory 的 10 Hz 诊断采样，以及 checkpoint/RTC 的 ID、phase、tracking/servo error、raw/sent
reference、settle 时长、state/image skew、clipping、freeze 和 delay。hold 阶段的周期状态降为 1 Hz，状态
变化和事件仍立即记录。设置 `--log-file` 后终端默认只显示 WARNING 和关键交互，完整 DEBUG 只写文件；
需要临时在终端查看详细诊断时再加 `--console-log-level DEBUG`。两端日志必须同时保留。

设置 `--log-file "$RUN_DIR/rollout.log"` 时，客户端还会自动创建
`$RUN_DIR/rollout.telemetry.csv`；也可以用 `--telemetry-file` 指定路径。该 CSV 逐条
记录 bridge 收到的实测 14 关节、bridge 100 Hz 插帧后的 `sent_target`，以及 legacy/prefetch 客户端的
插帧请求和 safety-filter 后发送值；后两者分别在 `client_reference_*` 与 `client_command_*` 列中，
并用 `record_type=bridge_state`、`client_command` 或 `client_hold` 区分来源。绘图命令为：

```bash
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
/home/jh/OpenPI_UR/openpi/.venv/bin/python \
scripts/plot_rollout_joints.py \
  "$RUN_DIR/rollout.telemetry.csv" \
  -o "$RUN_DIR/joint_diagnostics.png"
```

清空工作区、急停可触及，Apex Input Mode 初始保持 None。重新启动 motion-enabled 100 Hz bridge：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion \
  --publish-hz 100
```

执行保守的单初始 chunk 测试：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_tracking_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 5 \
  --rollout-schedule tracking \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 3 \
  --execute-steps 20 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rollout.log"
```

客户端提示后先把 Apex 切到 Custom；确认页显示运动门已打开后，再输入单个大写 `E`。运动结束后 bridge
会固定保持末端目标，客户端会明确等待人工把 Apex 切回 None，检测到 None 后才退出；这不是程序卡死。
通过条件：

- handoff 不计入 A0-A19 phase，phase 单调且跟踪变慢时会降速；
- arm clipping 为 0，raw/sent target 不持续分离；
- A9 checkpoint 有 `<=0.01 rad`、连续 `0.20 s` 的 source-timestamp 证据；
- 无 heartbeat timeout、timer overrun 或 stale feedback。

## 5. synchronized 回归

使用 [`_HANDOFF.md`](_HANDOFF.md) 快速开始中的 timed synchronized 参数运行至少两个 chunk。protocol v10 更新后，边界误差、跟踪时间、
clipping 和固定 hold 行为不得劣于旧基线。回归失败时不进入 RTC shadow。

Terminal A 在 Apex Input Mode 为 None 时启动 bridge，并记录本轮目录：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
export RUN_DIR="$PWD/logs/synchronized_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_synchronized_run_dir

./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion \
  --publish-hz 100
```

Terminal B 运行10秒同步调度：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_synchronized_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 10 \
  --rollout-schedule synchronized \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 3 \
  --execute-steps 20 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rollout.log"
```

出现确认提示后检查工作区和急停，再输入单个大写 `E`。程序结束动作后必须人工把 Input Mode 切回 None，
随后客户端才退出。保留 Terminal A/B 输出；两端完整日志位于本轮 `RUN_DIR`。

## 6. RTC shadow

远程清单、单 chunk governor 和 synchronized 回归全部通过后，运行：

```bash
export RUN_DIR="$(cat /tmp/marvinpro_tracking_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 5 \
  --rollout-schedule rtc \
  --rtc-shadow \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 3 \
  --execute-steps 20 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rtc-shadow.log"
```

RTC 输出不会 merge。确认日志证明：物理 A9 checkpoint 后才采图；推理期间继续旧 A10/A11；bridge 在整数
knot 边界记录 `d_actual`；shadow 丢弃后，本 episode 只运行 timed synchronized，不自动重试 RTC。

## 7. 两 chunk RTC

删除 `--rtc-shadow`，并加 `--max-rtc-merges 2`。达到两个成功 RTC merge 后，客户端会等待当前 RTC 段到达下一个稳定 checkpoint，再锁存当前目标；
现场人员随后把 Input Mode 切回 None。每次 merge 必须满足：

- observation 晚于 checkpoint 完成，state/image skew `<=50 ms`；
- request/plan/timeline/checkpoint ID 全匹配；
- `1 <= d_actual <= d_pred <= 4`，接管 phase 是精确整数；
- 推理期间无 clipping、hard freeze、stale state、timer overrun 或 old-prefix underrun；
- replacement anchor 等于边界时正在执行的旧 reference，边界误差显著低于 `0.16913 rad`；
- 不出现 chunk 周期回弹或可感知的 merge 抽动。

任一项失败：切回 None/必要时急停，保存 bridge 和 rollout 完整日志。本 episode 的 RTC fallback 必须保持
latched；不得现场放宽 `0.16 rad` 包络、URDF margin 或 tracking/stale 阈值。

## 8. 连续 RTC checkpoint

settled RTC 通过后，才允许用 `--rtc-continuous` 验证无停车观测。该模式在 A9 边界发出 checkpoint 后继续
执行旧轨迹，不等待误差进入 `0.01 rad` 并稳定 `0.20 s`；client 等待边界后的新图像，bridge 会把拿图期间
已经跨过的整数 knot 计入 `d_actual`。tracking governor、clipping、stale、heartbeat、ID/version、
`d_actual <= d_pred` 和整数边界 merge 保护保持不变。

第一轮必须同时使用 `--rtc-continuous --rtc-shadow`，不得执行 replacement。通过条件：

- checkpoint 事件记录 `continuous_checkpoint=True`、`settle_s=None`、`frozen=None`；
- checkpoint 前后 phase 单调继续，不出现 `frozen=checkpoint`；
- checkpoint 后图像满足 source age 和 `<=50 ms` state/image skew；
- `rtc_resumed` 中的初始 `d_actual` 正确包含拿图期间已经跨过的 knot；
- shadow 在预测 delay 边界停止并进入既有 synchronized fallback，无 clipping、hard freeze 或 stale。

连续 shadow 通过后，下一轮只允许 `--max-rtc-merges 1`；确认实际 merge 无抽动、无 checkpoint 停顿后，
再增加 merge 数。连续 shadow 的末尾 deadline freeze 和 synchronized fallback 是故意丢弃 RTC 输出的结果，
不能用于评价实际 merge 的连续性。

## 真机空闲后：最小三组测试

以下三组是当前 H20/s10、5 Hz、`d_max=4`、指数 soft mask 基线的最短真机验收路径。每组使用独立日志，
上一组完整通过后才能进入下一组。测试前物理打开两侧夹爪、清空工作区并让急停可触及；Apex Input Mode
初始为 None。不要使用 `--yes`，不要放宽 tracking、clipping、URDF、stale、C2 blend 或 delay 限制。
每次启动 motion-enabled 客户端后，都要等客户端出现确认提示，再由现场人员把 Apex 切到 Custom；确认页
显示运动门已打开后输入单个大写 `E`。动作结束后把 Apex 切回 None，等待客户端正常退出，再停止 bridge。

### 测试 1：protocol v10 motion-disabled dry-run

先完成只读 doctor、远程 H20 smoke 和 motion-disabled protocol v10 dry-run。这三步不会发送机器人动作：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --doctor --duration 8

cd /home/jh/OpenPI_UR/openpi
uv run python scripts/marvinpro_rtc_smoke.py \
  --host 192.168.50.73 --port 8000 \
  --horizon 20 --execution-horizon 10 --max-predicted-delay 4
```

doctor 和 smoke 通过后，Terminal A 启动不允许运动的 bridge：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --publish-hz 100
```

Terminal B 运行 dry-run；确认 protocol v10、H20 policy 输出、相机、joint state 和夹爪 feedback 均正常后停止两端：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 --policy-host 192.168.50.73 \
  --episode-seconds 10 --log-level DEBUG
```

通过条件：hello/event/command 均为 v10，旧版本明确拒绝；policy 普通 H20 推理成功；没有发送 action。通过后再做测试2。

### 测试 2：安全空间内两个 timed synchronized chunk

建立本组日志并重新启动 motion-enabled bridge：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
export RUN_DIR="$PWD/logs/h20_baseline_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_h20_baseline_run_dir
./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" --allow-motion --publish-hz 100
```

Terminal B 执行两个完整 timed H20 synchronized chunk，并按上述顺序人工切 Custom、确认运动门、输入 `E`：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_h20_baseline_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 --policy-host 192.168.50.73 --execute \
  --episode-seconds 10 --rollout-schedule synchronized \
  --playback-mode interpolated --control-hz 100 --model-hz 15 \
  --playback-time-scale 3 --execute-steps 20 \
  --log-level DEBUG --console-log-level WARNING \
  --sync-chunk-timeout-grace 1 --max-stuck-replans 2 \
  --log-file "$RUN_DIR/synchronized.log"
```

通过条件：至少两个 `checkpoint_ready reason_code=chunk_clean`；每段 deadline 为 load 后 5 秒且没有误报
`chunk_timed_out`；全程无 clipping、hard freeze、stale、timer/heartbeat/motion-gate drop 或 command rejection；
现场无异常冲击。测试时禁止碰桌或人为阻挡制造 timeout。

### 测试 3：短 RTC fallback/recovery/merge

重启 bridge 并建立新日志目录，然后运行短 normal RTC episode。不要使用 `--rtc-shadow`，因为 shadow 按设计
不会自动重进 RTC：

```bash
# Terminal A
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
export RUN_DIR="$PWD/logs/h20_rtc_recovery_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_h20_recovery_run_dir
./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" --allow-motion --publish-hz 100

# Terminal B
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_h20_recovery_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 --policy-host 192.168.50.73 --execute \
  --episode-seconds 20 --rollout-schedule rtc \
  --rtc-continuous --rtc-late-result-policy discard --max-rtc-merges 2 \
  --playback-mode interpolated --control-hz 100 --model-hz 15 \
  --playback-time-scale 3 --execute-steps 20 \
  --log-level DEBUG --console-log-level WARNING \
  --max-rtc-recoveries 3 --max-stuck-replans 2 \
  --policy-connect-timeout 5 --policy-request-timeout 2 \
  --log-file "$RUN_DIR/rtc-recovery.log"
```

通过条件：至少一次真实 `rtc_merged`。若本轮自然发生可恢复 late/C2/transport/observation-lag，还必须看到
`measured_holding -> 一个 chunk_clean -> recovery bootstrap -> rtc_merged`，且旧 request/timeline 无法 merge；
若全程没有自然故障，不要碰桌或阻挡来强造 timeout，保留成功日志并把 recovery 真机项标为“尚未触发”。
全程不得有 clipping、stale、hard freeze、事务错位或可感知冲击。

## 9. H20 下提升有效 knot rate 与扩大 `d_max`（待实施）

目标是逐步把执行时间语义从当前 `5 Hz` 拉回训练数据的 `15 Hz`，同时只按实测端到端延迟扩大
`d_max`。这不是取消 100 Hz bridge 插值；bridge 仍以 100 Hz 上采样，变化的是 policy knot 的物理推进速度。
当前基线和约束为：

```text
H = 20
s = 10
RTC prefix/old tail = H - s = 10 knots
effective knot rate = model_hz / playback_time_scale
current effective knot rate = 15 / 3 = 5 Hz
current d_max = 4
guard = 50 ms
```

2026-08-18 决策：保留 OpenPI 官方指数 soft mask 和 `schedule=exp`，暂不实施线性 mask。soft-mask 的非零
范围由 `H-s=10` 决定；`d_max` 只是 `d_pred` 的协议/延迟上限，不用于增加平滑点。同一个 `d_pred` 下，
提高 `d_max` 不会改变权重；实际提高 `d_pred` 反而会减少 `H-s-d_pred` 个 soft transition。因此当前
`d_max` 保持 4，只有稳定延迟预算或 knot rate 确实需要时才按本节分级方案调整，禁止设置为 10。

`d_max` 必须满足以下延迟预算，不能因为偶发链路尖峰无限增加：

```text
d_pred = ceil((stable_p95_wall_latency + 0.05 s) * effective_knot_hz)
d_pred <= d_max < H - s
```

扩大 `d_max` 会消耗 H20 的 RTC prefix，并缩短 soft overlap。`d_max=10` 会完全耗尽 10-knot prefix，
因此禁止使用；本轮预案上限为 8，且 `d_max=8` 只剩 2 knot soft overlap，必须作为最后一级而不是默认值。
历史 `808-1230 ms` 链路尖峰继续按 link fault 丢弃，不得为了覆盖尖峰扩大 deadline。

以下数值是基于历史稳定 wall latency `225-413 ms` 的起始候选，正式值必须由每轮至少 50 次持久连接请求
重新计算：

| 阶段 | playback time scale | 有效 knot rate | 起始 `d_max` | 物理 deadline | 扣除 50 ms guard 后允许的 wall latency | 剩余 soft overlap |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A（当前基线） | 3.0 | 5 Hz | 4 | 800 ms | 750 ms | 6 knots |
| B | 2.0 | 7.5 Hz | 4 | 533 ms | 483 ms | 6 knots |
| C | 1.5 | 10 Hz | 5 | 500 ms | 450 ms | 5 knots |
| D | 1.0 | 15 Hz | 7 | 467 ms | 417 ms | 3 knots |
| D 预留上限 | 1.0 | 15 Hz | 8 | 533 ms | 483 ms | 2 knots |

### 9.1 协议和离线前置条件

每次改变 `d_max` 都是跨端协议变化，不允许只改客户端参数。必须同步更新 deploy `rtc.py`、protocol version、
bridge/client 校验、OpenPI policy metadata 和 RTC 请求校验，并覆盖 `d_pred=1..d_max` 的单元测试、fake bridge、
WebSocket structured rejection 和 persistent-connection smoke test。H20 checkpoint shape 不因 `d_max` 改变，
但 policy server 公布的 metadata 必须和 deploy 完全一致，否则禁止连接真机。

每个候选频率先在远程 GPU 上完成两次 discarded warmup 和至少 50 次连续请求，分别记录 server inference、
queue、transport 和 wall latency 的 p50/p95/max。只有 `ceil((p95+50 ms)*rate) <= d_max` 才能进入对应真机阶段；
正常样本已经需要 `d_max=8` 以上时，停止提升频率，优先降低网络/推理延迟或改为本地推理。

### 9.2 先单独验证频率，再启用 RTC merge

频率按 `5 -> 7.5 -> 10 -> 15 Hz` 单向晋级，不允许跳级。每一级先用同一 checkpoint、初始姿态、prompt 和
固定 H20 chunk 执行单 chunk tracking，不发 RTC replacement，从而把“机器人是否能跟踪该速度”与
“RTC 是否能覆盖推理延迟”分开。每轮 bridge 都保持 100 Hz，C2 blend 的速度、加速度、jerk 和 URDF
限制不放宽。

进入下一级频率前必须同时满足：

- arm clipping、hard freeze、stale、timer overrun 和 motion-gate drop 均为 0；
- `phase_rate=1` 的活动时间占比至少 95%，否则名义提频实际仍被 governor 降速；
- tracking error p95 不高于 `0.02 rad`，最大值显著低于 `0.16 rad` hard-stop 包络；
- 关节速度、加速度、jerk 和位置均通过 bridge 100 Hz 检查，现场无新增抽动、回弹或冲击；
- 同一固定 chunk 至少重复 3 次，完成时间和最大 tracking error 没有明显漂移。

提高 knot rate 会缩短 2～3 knot C2 blend 的物理时间，并按 rate、rate²、rate³ 分别放大速度、加速度和
jerk。任何 `unsafe RTC blend` 拒绝都视为该频率/轨迹不通过，不得现场提高动态上限绕过。

### 9.3 每一级 RTC 验收顺序

单 chunk tracking 通过后，使用该频率对应的最小可行 `d_max`，严格按以下顺序执行：

1. motion-disabled protocol/metadata dry-run；
2. continuous RTC shadow，确认所有稳定请求满足 `d_actual <= d_pred <= d_max`；
3. `--max-rtc-merges 1`，检查 C2 blend 指标和现场连续性；
4. `--max-rtc-merges 2`，检查连续 replacement 和 estimator epoch；
5. 10 次 merge soak，最后才进行完整任务成功率测试。

发生以下任一情况立即停止该频率晋级并退回上一个已通过配置：稳定 p95 超出 delay budget、
`d_actual > d_pred`、old-prefix underrun、C2 blend 拒绝、arm clipping、hard freeze、phase rate 长时间低于1、
新增可感知抽动/回弹，或任务成功率下降。单次已分类的 link fault 只触发本 episode discard/fallback，
不允许据此继续扩大 `d_max`。

每轮除通用测试记录外，还必须记录：名义/实际 knot rate、playback time scale、`d_max`、prefix 剩余 soft
overlap、phase-rate 分布、delay budget 利用率、late/discard 比例，以及 blend 最大速度/加速度/jerk。

## 调度与轨迹诊断工具

本节吸收原 README 的诊断章节，命令保留 2026-08-07 现场形态。这些客户端用于隔离单一变量；除特别
说明外，不应带 `--execute` 重复已失败的组合。version 2 冻结计划固化在
`artifacts/marvinpro_red_cones_chunk_ab_v2.json`。

### 持续 rollout 的慢速插值诊断（失败实验，禁止真机复现）

> **失败实验，禁止继续真机复现。** 2026-08-07 真机结果出现约1.33秒周期的明显回弹和77个手臂
> 裁剪tick。以下命令仅保留用于复现实验参数，不应再次带 `--execute` 运行。后续测试必须先改为按
> 实际已发送目标衔接新计划，并缩短open-loop段。

该模式保持模型节点的 15 Hz 时间语义，把时间拉长 2 倍，并在节点之间以 100 Hz 线性插值；每次完整
消费 policy 输出的全部 10 个节点，每段持续 `1.333 s`，队列还剩 `0.30 s` 时开始推理下一段，返回后
追加到队尾，不在段中覆盖旧计划。

已执行过的5秒真机失败参数（仅供记录）：`--playback-mode interpolated --control-hz 100 --model-hz 15
--playback-time-scale 2 --execute-steps 10 --chunk-prefetch-seconds 0.30`。

### 同步执行、到位、保持、重观测诊断

该模式严格按 `完整发送当前chunk -> 锁存末目标 -> 等待全部14个臂关节到位 -> 稳定保持 -> 等待一帧
新的图像/关节观测 -> 远程推理 -> 执行下一chunk` 运行，不提前采集下一段观测，也不在当前chunk运动
期间推理。每个 H20 synchronized chunk 从 controller 的 `trajectory_loaded` 起有 `4 s + 1 s`
deadline；5 秒内到位并基于真实 source timestamp 稳定 `0.20 s` 才算 clean；健康超时时 bridge 原子
锁存当前实测臂位置，再从新图像推理；连续两次卡住后结束运动；episode 剩余不足 5 秒时不再启动新
chunk。默认到位条件是所有臂关节误差不超过 `0.01 rad` 并连续保持 `0.20 s`，夹爪不参与到位判断。

结束等待阶段只锁存一次当前测量姿态，固定目标一直保持到 Input Mode 离开 Custom，避免结束阶段的
单向漂移（该修复已通过短时真机回归确认）。若出现 `tracking timeout`，不要直接放宽误差阈值或关节
步长，先记录超时关节、误差和 `arm-clipped ticks`。

真机命令与上文第 5 节“synchronized 回归”相同，此处不再重复。

### 锁存姿态保持诊断

不连接 policy server；客户端只在开始时读取一次当前姿态，之后持续发送完全相同的绝对关节目标，
用于区分 Custom/bridge 控制链抖动和模型轨迹抖动。bridge 以 `--allow-motion` 启动（先用默认
15 Hz，停止后可加 `--publish-hz 100` 重复），Apex 完成关节阻抗模式但保持 Input Mode 为 None：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion
```

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.hold_test_client \
  --robot-host 6.6.7.100 \
  --duration 10
```

客户端显示等待后把 Apex 切到 Custom，核对模式和锁存关节值，现场安全后输入 `HOLD`；10 秒结束时
客户端继续发送同一目标，此时先在 Apex 切回 None。客户端退出时打印各关节峰峰值、标准差和最大
跟踪误差。异常时优先切回 None 或急停，不要依赖 `Ctrl+C`。

判读：15 Hz 抖动而 100 Hz 明显改善，则 bridge 低频目标发布是主要因素；两次都稳定而 rollout 抖，
问题在模型动作或重规划衔接；两次恒定目标都抖，继续检查 Custom 控制链和阻抗参数。

### 确定性慢速轨迹诊断

不连接 policy server。bridge 保持 100 Hz，默认只让 `Joint7_L` 在当前姿态附近按最小 jerk 曲线完成
`0 -> +0.04 -> -0.04 -> 0 rad`，总时长 8 秒；理论峰值速度 `0.0375 rad/s`、峰值加速度小于
`0.06 rad/s^2`，均低于官方 Home 限制。分别用 `--command-hz 15` 和 `100` 各跑一次：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.trajectory_test_client \
  --robot-host 6.6.7.100 \
  --command-hz 15
```

客户端等待时将 Apex 切到 Custom，核对后输入 `MOVE`；轨迹返回起点后先切回 None。判读：15 Hz 有
阶梯顿挫而 100 Hz 平滑，则客户端目标更新离散度是主要因素；两次都平滑但 rollout 抖，则模型动作或
异步重规划衔接是主要因素；100 Hz 仍抖，继续检查 bridge/ROS 动态指令路径和阻抗响应。不要使用
`--yes` 跳过首次真机确认。

### 单个冻结 Policy Chunk 诊断

连接真实 policy，但只保存一次 10 步输出，执行期间不再推理、不替换 chunk，并保持夹爪目标不变。
JSON 同时保存原始 policy 节点和限制在推理姿态 `±0.03 rad` 的实际回放节点。先按 15 Hz 直接发送
有界回放节点（自动用最小 jerk 返回推理姿态），再加载同一 JSON 以相同 15 Hz 时间轴做 100 Hz 线性
插值回放；两次模型目标和总时长相同，唯一变量是目标更新方式。

bridge 保持 100 Hz，Apex Input Mode 先 None：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --capture-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --playback-mode discrete
```

切到 Custom 核对后输入 `DISCRETE`；15 Hz 回放后自动返回推理姿态，切回 None。若计划超过诊断安全
上限 `0.45 rad/s` 或 `2.0 rad/s^2`，程序只保存 JSON 并拒绝运动。回到 None 后从同一姿态加载相同
计划做插值回放（输入 `INTERPOLATE`）：

```bash
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --playback-mode interpolated
```

判读：15 Hz 离散抖、100 Hz 插值平滑，则低频阶梯目标是主要因素；两次都抖，则模型 chunk 节点变化
或简单线性插值不合适；两次都平滑而持续 rollout 抖，则异步重规划延迟和 chunk 边界替换是主要因素。

需要隔离 `±0.03 rad` 诊断包络时，可加载同一 version 2 JSON 并加 `--target-source raw`（客户端先
验证原始节点的硬限位、bridge 步长包络、离散速度和加速度，任一超限不进入执行）；原始节点同样先做
15 Hz 离散、自动回锚切回 None 后再做 100 Hz 插值。保持 raw 节点和 100 Hz 插值不变、只把播放时长
拉长 2 倍可加 `--playback-time-scale 2.0`（总时长从 `0.667 s` 到 `1.333 s`，最大速度减半、加速度
降为四分之一；参数不允许小于 `1.0`）：

```bash
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --target-source raw \
  --playback-mode interpolated \
  --playback-time-scale 2.0
```

## RTC 决策与实现记录（2026-08-18/19）

### protocol v10 实现要点（2026-08-19，全部完成并真机验收）

- H20 synchronized/tracking/fallback 统一为 bridge-owned timed chunk：名义 4 秒，默认 1 秒 grace；
  健康 timeout 由 controller 原子锁存最新实测双臂位置，保留最后夹爪命令，连续两次 timeout 后固定
  hold 并结束。
- 新计划优先使用 3-knot quintic C2 handoff，失败再试 2-knot；动力学或安全限制失败时原子拒绝。
- RTC failure 使用结构化 reason code：late/discard、transport timeout、observation lag 和单次 C2
  merge infeasible 可恢复；clipping、freeze、stale、heartbeat/timer、状态门、
  事务/协议/shape/finite/URDF 错误不可恢复。
- 可恢复故障废弃旧连接/request/epoch，实测 hold，执行至少一个 clean synchronized chunk，再用新
  H20 普通推理初始化新 DelayEstimator epoch 和 `s=10` RTC bootstrap；每 episode 最多 3 次。
- 本地 OpenPI WebSocket client 支持 connect/request timeout、`close()` 解阻塞和 metadata 校验
  重连；MarvinPro 默认连接 5 秒、请求 2 秒。

### 指数 soft mask 基线冻结（2026-08-18 已决）

保留 OpenPI `schedule=exp`，不修改 soft-mask 公式、`d_max` 或远程 policy metadata：

```text
P = H - s
start = min(d_pred, P)
base[i] = clip(1 + (start - 1 - i) / (P - start + 1), 0, 1)
weight[i] = base[i] * (exp(base[i]) - 1) / (e - 1), i < P
weight[i] = 0,                                         i >= P
```

理由：soft-mask 非零范围由 `P=H-s` 决定，不由 `d_max` 决定。H20/s10 在常见 `d_pred=2` 时已有 8 个
soft transition knot；增大 `d_max` 不改变同一 `d_pred` 的权重表，实际使用更大的 `d_pred` 反而减少
`P-d_pred`，缩短 soft transition。保持 `d_max=4`；不得为了获得更多平滑点提高 `d_max`，也不得设置
`d_max=H-s=10`。客户端发送 `schedule=exp`、`beta=5.0`；远程 metadata 公布
`prefix_attention_schedule=exp` 并结构化拒绝不支持的 schedule。非真机验证：20 次持久 RTC 请求
wall p50/p95/max `375.7/447.5/568.9 ms`，错误 prefix、`d_pred=5`、错误 `s` 和不支持的 schedule
均被结构化拒绝。

### synchronized fallback 稳定一次后重新进入 RTC（已实现）

目标状态机：

```text
RTC 可恢复异常
-> invalidate 当前 RTC request/epoch
-> fixed hold
-> tracking 到位并连续稳定 0.20 s
-> 获取 hold 之后的新 state/image
-> 执行 1 个完整 synchronized H20 chunk
-> 在 chunk 末端再次 tracking 到位并连续稳定 0.20 s
-> 获取末端稳定之后的新 state/image
-> 建立新的 RTC delay/request/timeline epoch
-> 从新观测推理并加载新的 RTC 初始 chunk
-> 回到 continuous RTC
```

“稳定一次”定义为：完整 synchronized chunk 成功执行到末端 checkpoint，满足 tracking tolerance、
source timestamp 连续前进、settle 0.20 s、无 clipping/hard freeze/stale/timer overrun，再取得满足
时间屏障的新观测；不能仅靠等待固定秒数。

可自动恢复的故障：RTC late-result discard / delay budget miss；明确的瞬时 transport timeout 或
policy server 暂时不可用；可通过重新观测消除的 observation freshness/lag；单次 RTC C2 merge 不可行
（原 merge 仍原子拒绝且不放宽 jerk，经过 clean chunk 换边界后才重试）。继续锁存、不自动重进 RTC
的故障：arm clipping、tracking hard freeze、joint stale、timer overrun、heartbeat、Input
Mode/Robot Ready/arm state 异常；session/plan/timeline/checkpoint/request ID 不匹配；非 finite、
shape、URDF、安全包络、协议版本或本地 invariant 错误。bootstrap inference/安全 C2 handoff 失败消耗
一次 recovery attempt；到达 3 次上限后固定 hold。

实现约束（均已实现）：fallback 拆出“只执行一个恢复 chunk”的入口；进入 fallback 时所有旧 RTC
worker/result 失效，恢复后只接受新 request ID、新 timeline version、新 checkpoint ID；
`DelayEstimator.reset_epoch()` 后用恢复后新观测的初始推理建立新 epoch，禁止复用旧 latency；恢复后
初始 plan 使用 RTC checkpoint horizon `s=10`；每 episode 最多自动恢复 3 次，第 4 次只执行 timed
synchronized fallback。fake bridge 已覆盖端到端注入，旧 request/timeline 被拒绝。

### 夹爪实测反馈（2026-08-19）

- `/tj/info/gripper_feedback_L/R` 五维信息已确认；protocol v10 的 policy state、action state、RTC
  handoff anchor 和 measured hold 使用 feedback `q`；bridge 不再用最后发布命令覆盖夹爪实测状态。
- 尚未形成左右夹爪各自完整、同工况的开合端点标定；旧的 `0.0~1.25 -> 0~1` 不能视为已验证映射。

### 2026-08-19 20-merge 长跑暴露的问题

`logs/h20_rtc_soak20_20260819_142740`：两个 episode 均未成功 RTC merge（后续动作由 synchronized
fallback 执行），不能据此评价 H20 连续 RTC。要点：

- 首次 RTC 结果 `d_pred=2`、`d_actual=2`，hard anchor 偏差很小，但 3-knot C2 最大 jerk 分别为
  `22.06` 和 `24.57 rad/s^3`，超过当时的 `20 rad/s^3` 上限；2-knot 候选更差（`73.9`/`63.9`）。
  bridge 正确原子拒绝。根因指向“旧轨迹在 merge 点的速度/加速度与 H20 新轨迹开头不够相容”，不是
  网络延迟尖峰。当晚已将 bridge 默认 `--rtc-blend-max-jerk-rad-s3` 从 `20` 提高到 `40 rad/s^3`；
  更长的受约束 C2 blend 是否可行仍需离线回放评估，不得再直接放宽上限。
- 一段 fallback 的最后一个 chunk 始终未满足到位+稳定（`Joint4_L` 最大误差约 `0.01155 rad`），现场
  机械臂已触达桌面，高度疑似接触约束使目标物理不可达。不要通过取消 timeout 或放宽 tolerance 掩盖。
- **telemetry 覆盖事故**：两个 episode 复用同一个日志/telemetry 文件，CSV 被第二次运行覆盖。
  此后每个 episode 必须使用新的 `RUN_DIR`/文件名；该约定现为强制要求，见
  [`_HANDOFF.md`](_HANDOFF.md)。
- H10/H20 对比必须固定机械臂初始姿态、物体布局和两侧夹爪状态，并把 gripper clipping 单独统计。
- `--episode-seconds` 是外层安全时限，`--max-rtc-merges` 只是成功 replacement 上限；两者都不应取消。

## 异步 action chunk 错位分析（2026-08-07）

### 模型时间语义

训练配置 `pi05_marvinpro_red_cones`：数据集 15 Hz、`action_horizon=10`；每次输入三路图像、16 维
关节/夹爪状态和 prompt，输出 10 组 16 维绝对关节/夹爪目标。数据加载器为 action 构造的时间偏移是
`[0/15, 1/15, ..., 9/15]` 秒，即 `policy(O(t)) -> [A(t), A(t+0.0667), ..., A(t+0.6000)]`；
`action[0]` 是当前数据帧时刻的 action。训练阶段关节 action 以相对当前 state 的 delta 进入模型，
输出变换还原为绝对关节目标；夹爪维度不做关节 delta 变换。

### 错位机制与真机证据

异步 prefetch 中，第二次观测 `O1` 生成的新 chunk B 的 `B0` 基于 `O1` 所见的真实机器人状态，而旧
计划 A9 已位于更远的未来目标。把 B 追加到 A 的 raw 尾部形成
`追赶未实现的A9 -> chunk切换 -> 反向拉回O1附近 -> 再向前追赶` 的周期回弹。

失败测试使用 10 节点完整消费、100 Hz 线性插值、2 倍时间尺度和 0.30 秒预取，操作员观察到约 1.33 秒
周期的明显回弹。决定性重规划样本：

| 指标 | 数值 |
| --- | ---: |
| 第二次推理耗时 | `289.2 ms` |
| 推理返回时旧队列剩余 | `2` 个100 Hz点 |
| 旧raw尾部到最后实际发送目标 | `0.07926 rad` |
| 新 `B0` 到旧raw尾部 | `0.16913 rad` |
| 新 `B0` 到真实反馈 | `0.01041 rad` |

运动阶段 531 个发布 tick 中有 77 个手臂裁剪 tick，主要涉及 `Joint1_R` 和 `Joint4_R`；第三次推理
`386.6 ms` 超过 0.30 秒预取窗口，造成 8 个 measured-pose hold tick。教训：机器人没有实现旧 raw
轨迹尾部，不能把它当作下一 chunk 的真实起点；异步推理消除了大部分等待，但没有自动解决新旧 chunk
的时间和状态一致性；简单线性插值不能把 `B0` 变成语义正确的 `A10`；安全裁剪不能充当轨迹规划器；
增大预取窗口只会让生成 B 所用的观测更陈旧。

### 可选方案调研（结论）

1. **同步 chunk 执行**：因果最清楚、不改模型/服务器，作为安全基线；代价是 chunk 间停顿约 150 至
   400 ms 另加稳定时间。PI 说明 RTC 发布前的 π0 系列就是这种方案。
2. **Receding horizon / 短前缀执行**：只执行前 3 至 5 个节点并用新观测重规划；新 chunk 必须从最后
   实际发送目标或反馈连续衔接；固定跳到 `new[k]` 不是充分的延迟补偿。
3. **ACT temporal ensemble**：只能聚合时间对齐的预测；当前远程延迟无法支撑，PI 报告对 flow-based
   VLA 不保证有效或安全。
4. **通用异步动作队列**（LeRobot/SmolVLA）：主要解决推理期间无动作可执行；只移植队列仍可能复现
   回弹（LeRobot 文档明确区分异步队列解决 idle、RTC 解决 chunk 间不连续）。
5. **Real-Time Chunking（RTC）**：冻结必然执行的旧前缀，对剩余新 chunk 做 inpainting/guidance；
   不要求重新训练。本项目已实施并真机验收。
6. **训练时 action prefix conditioning**：对大延迟稳健但需要重新训练 checkpoint；在客户端基线与
   推理时 RTC 验证后再考虑。

### 与 2 倍时间尺度的关系

训练时 A0 至 A9 覆盖约 0.6 秒；2 倍慢放后整个动作段约 1.333 秒。时间拉伸不改变模型输出的关节目标
值，但把目标速度约降为一半、加速度约降为四分之一。冻结 chunk 中 2 倍慢放改善跟踪，说明降低目标
速度有价值；持续 rollout 的失败来自过长 open-loop 执行和错误 chunk 锚点，两件事必须分开评估。

### 不应采用的简化方案

- 不要把新 `B0` 直接追加到未实现的旧raw尾部。
- 不要固定跳到 `B2` 或 `B3` 并称为延迟补偿；真机数据已经显示更高索引未必更接近反馈。
- 不要用逐点硬裁剪制造“平滑轨迹”；裁剪会产生新的速度/加速度折角。
- 不要仅增大prefetch窗口；这会增加观测到执行之间的陈旧时间。
- 不要对未按绝对时间对齐的action做普通平均。
- 不要放宽客户端 `0.08 rad` 或bridge `0.12 rad` 包络来掩盖跟踪失败。
- 不要再次运行10节点、2倍时间、提前0.30秒并追加raw尾部的失败真机参数。

### 一手资料

- Physical Intelligence, Real-Time Action Chunking with Large Models:
  <https://www.pi.website/research/real_time_chunking>
- Black, Galliker, Levine, Real-Time Execution of Action Chunking Flow Policies, NeurIPS 2025:
  <https://arxiv.org/abs/2506.07339>
- LeRobot, Real-Time Chunking documentation:
  <https://huggingface.co/docs/lerobot/main/rtc>
- LeRobot, Asynchronous Inference documentation:
  <https://huggingface.co/docs/lerobot/main/async>
- Zhao et al., Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ACT), RSS 2023:
  <https://roboticsproceedings.org/rss19/p016.pdf>
- Chi et al., Diffusion Policy: Visuomotor Policy Learning via Action Diffusion, RSS 2023:
  <https://arxiv.org/abs/2303.04137>
- Black et al., Training-Time Action Conditioning for Efficient Real-Time Chunking:
  <https://arxiv.org/abs/2512.05964>

## 测试记录

每轮记录：日期、两仓库 commit、checkpoint、完整 CLI、GPU、网络/代理环境、state/image 频率、RTC 各阶段
延迟、`d_pred/d_actual`、phase、tracking/reference/servo error、clipping、freeze、checkpoint/merge IDs、
fallback 原因和操作员结论。以下为精简后的记录，保留结论与关键数据。

### 2026-08-10 本机到控制器只读测试

通过。路由 `6.6.7.100 dev enp49s0`，无代理/ProxyCommand；SSH 22 和 bridge 7332 可达。8 秒 doctor：
`/joint_states` 63.4 Hz、左右夹爪各 472.4 Hz、相机 11.6 Hz、robot/arm state 各 126.7 Hz。motion-disabled
安全门验证：`execute=True` 的 trajectory load 被结构化拒绝，timeline 保持 0。修复了 rclpy SIGINT 重复
shutdown（改为单一 shutdown owner）。

### 2026-08-10 单 chunk tracking governor 真机测试

通过（`logs/tracking_retry_20260810_183404`）。100 Hz command、15 Hz knot、2.0x、10 knot；phase 0->9
单调，跟踪误差大时 phase rate 自动降低；A9 checkpoint tracking error `0.005103 rad`、source timestamp
证明连续 settle `0.202 s`；checkpoint 后 state/image skew `6.1 ms`；全程无 clipping、freeze、stale、
heartbeat/timer 异常。操作员切回 None 后正常退出。

### 2026-08-11 synchronized 卡顿调查

- `logs/synchronized_20260811_095502`：2.0x、3 chunk；chunk 1 `track_peak_error=0.27883 rad`、
  `arm_clipped=228`，不通过，不能进 RTC。
- `logs/synchronized_scale3_20260811_101136`：3.0x、2 chunk；clipping 为 0，但 chunk 间仍有 `2.701 s`
  间隔。卡顿根因是调度语义（发送后等待跟踪、稳定、保持、重观测、推理），增大 time scale 只能消除
  clipping，不能消除 chunk 间停顿；synchronized 适合隔离边界问题，不适合评价连续平滑性。
- tracking governor 观测：误差 `0.0171 -> 0.0356 rad` 时 phase rate `0.7627 -> 0.1479`，随后恢复
  约 `0.91`——主动降速，会被感知为迟滞。
- bridge 定时抖动约 `20-27 ms`（平均 `99.6 Hz`），量级不足以解释 `0.7-1.0 s` 的 chunk 停顿。
- 结论：两种“卡顿”不是同一个故障；连续平滑运动修复前不进入 RTC shadow/merge。

### 2026-08-11 连续 prefetch 边界对照

`logs/prefetch_scale3_20260811_103829`：3.0x、5 秒、3 次推理、无 underrun；第二次推理的
`new action[0] -> old queue tail` 最大臂关节差 `0.15034 rad`（队尾相对已发送目标还差 `0.07810 rad`），
支持“异步观测产生的新 chunk 与旧队尾时间错位”是抽动来源。86 个 arm-clipped tick。结论：允许进入
RTC shadow（shadow 丢弃 RTC 输出），shadow 通过前不允许真实 merge。

### 2026-08-11 settled RTC 五 chunk 卡顿调查

`logs/rtc_five_chunk_20260811_115028`：初始 chunk + 4 次 replacement；`d_pred=2`，`d_actual=2,1,1,1`；
无 clipping/freeze/stale/fallback；边界速度跳变 `0.00495-0.02011 rad/knot`，无可感知回弹。但每次
checkpoint 明显停顿：首次 `1.291 s`、后续 `0.368/0.471/0.432 s`；bridge `99.53 Hz` 不能解释——根因是
settled checkpoint 状态机的固定等待（每次约 `0.20 s` settle + 首次等待跟踪误差收敛）。已新增
`--rtc-continuous` 模式。

### 2026-08-11 continuous RTC shadow

通过（`logs/rtc_continuous_shadow_20260811_135240`）。checkpoint 事件
`continuous_checkpoint=True`、`settle_s=None`、`frozen=None`，phase 事件后继续；`rtc_resumed` 初始
`d_actual=0`；无 clipping/stale/freeze。末尾的 deadline pause 和 synchronized fallback 是 shadow
丢弃输出的预期行为。允许下一轮单次实际 continuous RTC merge。

### 2026-08-11 execution horizon 6 远程验证（历史，H10/s6 阶段）

协议改为 `H=10`、`s=6`、`d_pred<=4`，模型权重未修改。稳定 `d_pred=1..4` 推理 `109-116 ms`。第一组
20 次 wall latency 出现 `p95=808.6 ms/max=1230.5 ms` 链路尖峰，不通过 delay 门槛；立即重复的稳定序列
为 `225.3-413.4 ms`，带 50 ms guard 的预测上限为 4。教训：链路尖峰按 link fault 处理，不得扩大
deadline 覆盖。

### 2026-08-14 governor 与 5 Hz 配置变更

- protocol 提升到 v6；`tracking/rtc` 固定 15 Hz 节点、3.0x 时间尺度（名义 5 Hz knot rate），旧
  2.0x/7.5 Hz 配置被拒绝；
- governor 使用 `run=0.02`、`resume=0.12`、`stop=0.16 rad`；trajectory arm clipping 和 bridge 目标
  校验包络统一为 `0.16 rad`。

### 2026-08-14 d_pred 与迟到结果治理

- protocol v7：`ResumeTrajectoryCommand` 携带 `discard|wait`，默认 `discard`；bridge 在实际
  `d_actual == d_pred` 时使迟到 epoch 无效；
- estimator 使用 epoch 内可行样本的保守 p95 + `50 ms` guard；`1.56 s` 类 horizon fault 及错过物理
  deadline 的样本不进入稳定分布；fallback 清空旧 epoch；
- 失败基线：历史 `1.56 s` wait 样本曾冻结约 `1.02 s`、速度跳变 `0.10757 rad/knot`——后续方案必须
  优于该基线；
- 当时的“不得把 `d_max` 提高到 4 以上”只适用于 4-knot old tail，已由 H20 基线和第 9 节分级方案取代。

### 2026-08-18 H20 远程推理非真机验证

只连接 policy service，未碰机器人。metadata 与本地 H20 基线一致（`rtc_v1`、H=20、s=10、`d_max=4`、
`schedule=exp`）。同一持久连接 2 次 discarded warmup + 20 次有效 RTC 请求（`d_pred=1..4` 各 5 次）：
wall min/p50/p95/max `332.6/375.7/447.5/568.9 ms`；server infer p95 `221.1 ms`；server queue 最大
`3.8 ms`；切换 `d_pred` 无 compile 量级尖峰。5 Hz 下保守 `d_pred=3`，仍在 `d_max=4` 内。错误 prefix
shape、`d_pred=5`、`s=9`、`schedule=linear` 均返回 `invalid_rtc_request`，拒绝后同连接可继续。
本轮 20 次样本只作短时链路证据，不代表长任务无延迟尖峰。

### 2026-08-19 protocol v10/H20 三组真机测试与长任务问题

- 第一组 motion-disabled dry-run：通过。topic 映射、H264 图像、夹爪反馈、v10 连接均正常，未发动作。
- 第二组 timed synchronized：通过（`logs/h20_baseline_20260819_183256`）。3 个 chunk 均在 deadline
  内到位并稳定，`arm-clipped ticks=0`，无抽动、回弹、撞击。
- 第三组短 RTC fallback/recovery/merge：通过（`logs/h20_rtc_recovery_20260819_184521`）。完成 2 次
  RTC merge，`rtc_final_status=clean_completion`，`recoveries=0`，无 clipping、stale、hard freeze 或
  可感知冲击。
- 结论：protocol v10 联通、timed synchronized、continuous RTC merge 和基础 recovery 路径均已跑通。

### 2026-08-19 长任务暴露的问题与离线修复

- `logs/h20_rtc_task5m_20260819_193006` 约 12 秒时先发生一次 `rtc_late (d_pred=2, d_actual=2)`，随后
  连续 3 次 synchronized recovery 的 C2 handoff 因 jerk 超限被原子拒绝（最大 `24.86/31.05/27.49
  rad/s^3`，2-knot 更高）。bridge 默认 `--rtc-blend-max-jerk-rad-s3` 从 `20` 提高到 `40 rad/s^3`，
  速度 `0.45 rad/s`、加速度 `2.0 rad/s^2` 和其他安全门不变。
- 已修复 recovery 状态机：第三次可恢复 recovery 失败后不再直接固定 hold 结束，而是重新锁存实测姿态、
  稳定取新观测后切换到不再重建 RTC 的 timed synchronized fallback（最多两次 C2 replan）。
- 复测顺序：短 synchronized -> 短 RTC recovery/merge -> 5 分钟任务；重点确认
  `rtc_recovery_exhausted; switching to timed synchronized fallback` 后仍继续运行。

### 2026-08-28 新任务 RTC 真机首跑（`logs/rtc_20260828_114357`）

- 环境：checkpoint `pi05_marvinpro_red_cones_slow_260826_full/79999`，prompt
  "Stack the three red cones from right to left inside the white square to form a stable stack."，
  服务器 10:23 重启后首次接受请求。参数：continuous RTC、`discard`、`--max-rtc-merges 1`、
  `--policy-request-timeout 5`、20s episode、确认键 `E`。
- 前两次尝试均在 warmup 阶段 abort：定位为 **JAX 按代码路径分别 JIT 编译**——服务器重启后普通推理
  路径和 RTC(prefix attention) 路径各自需要一次性编译，client 侧 request timeout 等不及。用
  `/tmp/policy_warmup.py`（双路径 dummy 请求）预热后两条路径均 ~110-170ms。
  **教训：服务器每次重启后先跑预热脚本再上真机。**
- 正式跑结果：`rtc_final_status=clean_completion merges=1 recoveries=0`。
  初始推理 128ms（server infer 74ms）；连续 checkpoint 于 knot 10 到达，RTC 请求 d_pred=1、
  d_actual=1、wall 137ms；merge 边界速度跳变 0.019 rad、加速度跳变 0.021 rad/s2、
  blend jerk 5.14 rad/s3（远低于 40 门限），merge 点参考突变仅 7.8e-05 rad，无感知冲击。
- 局限：`--max-rtc-merges 1` 只执行了约 4.3s 运动（2 次推理）即 hold，不足以评估叠放精度。

同日续跑与配置变更：

- 60s 完整跑（`logs/rtc_20260828_114752`，去掉 merge 上限）：`merges=30 recoveries=0`，
  clean_completion。但因 `--playback-time-scale 3`（5Hz knot rate、3 倍慢放）只完成了约半个任务。
  数据采集时已放慢示教速度，无需再慢放。
- 据此放开 argparse 门控：`synchronized/tracking/rtc` 的 `--playback-time-scale` 白名单从仅 3
  放宽为 1 或 3（`rollout_client.py` 的 `_TRACKING_ALLOWED_TIME_SCALES`，2026-09-02 起含 1.5），
  其余非法值仍拒绝；
  新增 `test_rtc_schedule_allows_native_fifteen_hz_playback_scale`，pytest 107 passed。
  下游全部走 `effective_knot_hz = model_hz / time_scale`，15Hz 下 d_pred≈3（上限 4）仍有余量。
  该改动只在 client 侧，bridge 不需要重传。下一步：time-scale 1 原速 60s 跑，评估叠放精度。

同日 15Hz 首跑暴露 blend 包络问题（`logs/rtc_20260828_133550`）与修复：

- time-scale 1（15Hz knot rate）首跑在第一次 merge 即被 bridge 原子拒绝：3-knot blend jerk
  `355.7 rad/s^3`、2-knot `1107.7`，门限 40。随后 3 次 synchronized recovery 的 C2 handoff
  （从静止交接进 15Hz 原生速度的 chunk，jerk `137-170`）也全部超限，最终 `stuck_exhausted`
  （merges=0, recoveries=4）。安全链按设计工作：机械臂锁存实测姿态 hold，无异常运动。
- 根因：blend 窗口固定 2~3 knot 且三条上限（vel `0.45` / accel `2.0` / jerk `40`）是 5Hz 下整定的
  物理值。15Hz 下窗口缩短 3 倍、边界速度大 3 倍，jerk 放大 ~27 倍；且 5Hz 遥测显示 policy 相位
  峰值速度 `0.081 rad/knot`，15Hz 物理速度 `~1.22 rad/s` 已超 blend 速度上限本身——
  固定物理上限与 15Hz 运动自相矛盾，单纯延长窗口或提高门限都不是正解。
- 修复（bridge 侧，`robot_bridge.py` + `trajectory_timeline.py`）：blend 窗口改为按秒恒定
  （目标 0.6s，knot 数随 knot rate 缩放，15Hz 候选 `(9,6,3,2)`、按 checkpoint 距离裁剪）；
  blend 上限改为按 knot rate 查表的显式包络 `BLEND_CAPS_BY_KNOT_HZ`：5Hz 保持运行验证过的
  `0.45/2.0/40`，15Hz 用 `stack_cones_slow_260826` 遥操作数据标定（104 集原生 15Hz，
  p99.9 = `0.556/2.34/49.8`，示教 max = `1.167/14.2/289`，取 `max(3x p99.9, 1.3x max)`）
  = **`1.7/18.0/380`**；CLI `--rtc-blend-max-*` 可整体覆盖，未验证 knot rate 无包络直接拒绝。
  曾考虑过按 `(knot_hz/5)^(1/2/3)` 立方缩放（15Hz jerk 上限 1080），但那是示教最大 jerk 的
  3.7 倍，过于宽松——窗口按秒恒定后接缝 jerk 只随速度差线性增长，不需要立方放大。
  按此包络复盘 13:35 的失败：首次 merge 的 355 jerk 在 9-knot 窗口下约 39，recovery 交接
  137-170 约 15-19，速度 1.22 < 1.7，均可通过。pytest 113 passed。
  **bridge 代码有改动，下次跑必须重启 bridge（重跑 run_bridge_on_controller.sh 即重传）。**
- 待验证：15Hz 完整 60s 跑的 merge 成功率、recoveries 数与叠放精度。
