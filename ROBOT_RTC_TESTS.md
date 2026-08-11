# MarvinPro Tracking-Aware RTC Robot Test Checklist

本清单记录当前机器因网络/代理条件未执行的测试。不要让自动化脚本切换 Apex Input Mode、Robot Ready、
Impedance Mode、Home 或清故障；这些步骤必须由现场人员确认。远程推理端还必须先通过
`/home/jh/OpenPI_UR/openpi/REMOTE_RTC_TESTS.md`。

## 已离线验证

- protocol v2 序列化、版本拒绝、高频 state/image/event 分流、乱序 event 丢弃和发送公平性；
- feedback source timestamp 不会被 100 Hz timer 伪装成新反馈；
- 慢速一阶机器人会令 phase 减速/冻结，不按控制 tick 消费动作；
- A3 feedback 仍在 A2 时不产生 checkpoint，stale feedback 不能累计 `0.20 s` settle；
- fake bridge 完整执行 `Load -> A3 checkpoint -> Resume -> Stage -> phase 4.0 merge`；
- merge 使用真实 `d_actual`，并将当前旧 reference 放到 `C[k-1]` 作为 anchor；
- RTC 失败后客户端进入固定 hold，再使用 bridge-owned synchronized。

离线测试不能证明控制器 ROS topic 频率、网络背压、真实关节响应或远程推理延迟满足要求。

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
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --doctor --duration 8
```

记录所有 topic 频率和最新值。`/joint_states` 应足以支持 50 ms stale 门限；相机、左右夹爪、input mode、
robot state 和 arm state 均必须有消息。doctor 不通过时停止，不得通过放宽 timeout 继续。

## 3. protocol v2 dry-run

控制器启动不允许动作的 bridge：

```bash
./scripts/run_bridge_on_controller.sh --publish-hz 100
```

本机连接远程 policy，但不带 `--execute`。legacy dry-run 先确认协议、图像和 policy 输出；trajectory 模式按
设计要求必须带 `--execute`，所以不能在本阶段启用。

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --episode-seconds 10 \
  --log-level DEBUG
```

通过条件：没有版本错误、state/image stale、JPEG 饥饿或 policy shape/finite 错误。结束后保留完整日志。

## 4. 单 chunk tracking governor

为每轮运动测试建立独立目录，并在两个本机终端中设置为同一个绝对路径：

```bash
export RUN_DIR=/home/jh/TianJi_data_collector/MarvinPro_deploy/logs/tracking_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_tracking_run_dir
```

`--local-log` 保存控制器 bridge 的 ROS/trajectory event 输出，`--log-file` 保存客户端配置、policy metadata、
100 Hz trajectory 的 10 Hz 诊断采样，以及 checkpoint/RTC 的 ID、phase、tracking/servo error、raw/sent
reference、settle 时长、state/image skew、clipping、freeze 和 delay。hold 阶段的周期状态降为 1 Hz，状态
变化和事件仍立即记录。设置 `--log-file` 后终端默认只显示 WARNING 和关键交互，完整 DEBUG 只写文件；
需要临时在终端查看详细诊断时再加 `--console-log-level DEBUG`。两端日志必须同时保留。

设置 `--log-file "$RUN_DIR/rollout.log"` 时，客户端还会自动创建
`$RUN_DIR/rollout.telemetry.csv`；也可以用 `--telemetry-file` 指定路径。该 CSV 不写终端日志，而是逐条
记录 bridge 收到的实测 14 关节、bridge 100 Hz 插帧后的 `sent_target`，以及 legacy/prefetch 客户端的
插帧请求和 safety-filter 后发送值；后两者分别在 `client_reference_*` 与 `client_command_*` 列中，
并用 `record_type=bridge_state` 或 `client_command` 区分来源。绘图命令为：

```bash
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
/home/jh/OpenPI_UR/openpi/.venv/bin/python \
scripts/plot_rollout_joints.py \
  "$RUN_DIR/rollout.telemetry.csv" \
  -o "$RUN_DIR/joint_diagnostics.png"
```

清空工作区、急停可触及，Apex Input Mode 初始保持 None。重新启动 motion-enabled 100 Hz bridge：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion \
  --publish-hz 100
```

执行保守的单初始 chunk 测试：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_tracking_run_dir)"
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 1 \
  --rollout-schedule tracking \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 2 \
  --execute-steps 10 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rollout.log"
```

客户端提示后先把 Apex 切到 Custom；确认页显示运动门已打开后，再输入单个大写 `E`。运动结束后 bridge
会固定保持末端目标，客户端会明确等待人工把 Apex 切回 None，检测到 None 后才退出；这不是程序卡死。
通过条件：

- handoff 不计入 A0-A9 phase，phase 单调且跟踪变慢时会降速；
- arm clipping 为 0，raw/sent target 不持续分离；
- A9 checkpoint 有 `<=0.01 rad`、连续 `0.20 s` 的 source-timestamp 证据；
- 无 heartbeat timeout、timer overrun 或 stale feedback。

## 5. synchronized 回归

使用 README 中已验证的 synchronized 参数运行至少两个 chunk。protocol v2 更新后，边界误差、跟踪时间、
clipping 和固定 hold 行为不得劣于旧基线。回归失败时不进入 RTC shadow。

Terminal A 在 Apex Input Mode 为 None 时启动 bridge，并记录本轮目录：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
export RUN_DIR="$PWD/logs/synchronized_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_synchronized_run_dir

./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion \
  --publish-hz 100
```

Terminal B 运行6秒同步调度：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_synchronized_run_dir)"
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 6 \
  --rollout-schedule synchronized \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 2 \
  --execute-steps 10 \
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
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
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
  --playback-time-scale 2 \
  --execute-steps 10 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rtc-shadow.log"
```

RTC 输出不会 merge。确认日志证明：物理 A3 checkpoint 后才采图；推理期间继续旧 A4/A5；bridge 在整数
knot 边界记录 `d_actual`；shadow 丢弃后，本 episode 只运行 synchronized，不重试 RTC。

## 7. 两 chunk RTC

删除 `--rtc-shadow`，并加 `--max-rtc-merges 2`。达到两个成功 RTC merge 后，客户端会等待当前 RTC 段到达下一个稳定 checkpoint，再锁存当前目标；
现场人员随后把 Input Mode 切回 None。每次 merge 必须满足：

- observation 晚于 checkpoint 完成，state/image skew `<=50 ms`；
- request/plan/timeline/checkpoint ID 全匹配；
- `1 <= d_actual <= d_pred <= 4`，接管 phase 是精确整数；
- 推理期间无 clipping、hard freeze、stale state、timer overrun 或 old-prefix underrun；
- replacement anchor 等于边界时正在执行的旧 reference，边界误差显著低于 `0.16913 rad`；
- 不出现约 `1.33 s` 周期回弹。

任一项失败：切回 None/必要时急停，保存 bridge 和 rollout 完整日志。本 episode 的 RTC fallback 必须保持
latched；不得现场放宽 `0.08/0.12 rad` 包络、URDF margin 或 tracking/stale 阈值。

## 8. 连续 RTC checkpoint

settled RTC 通过后，才允许用 `--rtc-continuous` 验证无停车观测。该模式在 A3 边界发出 checkpoint 后继续
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

## 测试记录

每轮记录：日期、两仓库 commit、checkpoint、完整 CLI、GPU、网络/代理环境、state/image 频率、RTC 各阶段
延迟、`d_pred/d_actual`、phase、tracking/reference/servo error、clipping、freeze、checkpoint/merge IDs、
fallback 原因和操作员结论。

### 2026-08-10 本机到控制器只读测试

- 代理环境：当前 shell 未设置 `HTTP_PROXY`、`HTTPS_PROXY`、`ALL_PROXY` 或 `NO_PROXY`；
- 路由：`6.6.7.100 dev enp49s0 src 6.6.7.10`，SSH 配置无 `ProxyCommand`/`ProxyJump`；
- 连通性：SSH 22 和 bridge 7332 可达；测试前后控制器均无残留 bridge 进程或 7332 listener；
- 8 秒 doctor：`/joint_states` 63.4 Hz，左右夹爪各 472.4 Hz，相机 11.6 Hz，robot/arm state
  各 126.7 Hz，joint mapping 正常；当时 Apex 为 None，状态为 `input_mode=0`、`robot_state=(0,0)`、
  `arm_state=(0,0)`；
- protocol v2 dry-run bridge：hello 为 `version=2`、`motion_allowed=False`、`publish_hz=100`，关节限位
  14/14；6 秒内收到 1091 个递增 state 和 88 张 JPEG，state 最大接收间隔 34.35 ms，joint source
  最大间隔 28.77 ms，image 最大间隔 89.95 ms，最后一帧反馈 age 为 2.9/1.6/0.5 ms；
- 未连接正在使用的 policy 服务器，未启动 `--allow-motion`，未改变 Apex 模式，未发送任何 action；
- motion-disabled 安全门：发送 `execute=True` 的 trajectory load 后收到结构化
  `trajectory_command_rejected`，原因是未启用 `--allow-motion`；timeline 保持 0，session/phase 均为空；
- 首次退出发现 rclpy SIGINT 重复 shutdown；改为单一 shutdown owner 后，短 bridge 启停退出码为 0，
  无线程异常或重复 shutdown traceback，修改后的 doctor 也可正常退出。

### 2026-08-10 单 chunk tracking governor 真机测试

- 结论：通过；日志目录为 `logs/tracking_retry_20260810_183404`，MarvinPro_deploy 基线 commit
  `780247f`，OpenPI 基线 commit `6fc6a90`，两边均包含未提交的本轮 RTC/诊断修改；
- 配置：远程 policy `192.168.50.73:8000`、controller bridge `6.6.7.100:7332`、100 Hz command、
  15 Hz policy knot、2.0x 时间尺度、10 knot、tracking schedule、1 秒 episode；
- policy metadata 为 `rtc_v1`、horizon 10、execution horizon 4、最大预测延迟 4；warmup 为
  `248.6 ms`，实际初始推理 wall time 为 `381.9 ms`，服务端 infer 为 `69.3 ms`；
- trajectory phase 从 0 单调到 9；较大跟踪误差时 phase rate 自动降低，未发生 hard freeze、heartbeat
  timeout、timer overrun、stale feedback 或 command rejection；
- A9 checkpoint tracking/servo error 为 `0.005103 rad`，低于 `0.01 rad`，source timestamp 证明连续
  settle `0.202462 s`；checkpoint 后新观测 state/image skew 为 `6.144 ms`；
- 全程 `arm_clipped=False` 且 raw/sent arm reference 最大差值为 0；policy 的小幅负夹爪输出在客户端边界
  投影到 `[0, 1]`，左夹爪 7 个点、右夹爪 10 个点被投影，未改变 14 个手臂关节；
- hold 阶段 DEBUG 周期采样约 1 Hz，终端仅保留安全 WARNING 和交互提示；操作员切回 None 后记录
  `trajectory_stopped` 和 `trajectory_exit_mode_confirmed input_mode=0`，客户端正常退出；
- 下一阶段 synchronized 多 chunk 回归尚未执行，需要操作员重新确认真机环境安全。

### 2026-08-11 synchronized 卡顿调查

- `logs/synchronized_20260811_095502`：2.0x、100 Hz、3 个 chunk；每段新增 134 个 command tick。chunk 1
  `track_peak_error=0.27883 rad`、到位耗时 `1.954 s`、`arm_clipped=228`；chunk 2/3 的 arm clipping 为 0。
  本轮不通过，且不能进入 RTC。
- `logs/synchronized_scale3_20260811_101136`：3.0x、2 个 chunk；arm clipping 为 0，但 chunk 1/2 之间仍有
  `2.701 s` 的诊断间隔，而每个 chunk 的发送时长为 `2.000 s`；`episode_underruns=222` 表示计划队列为空时
  客户端重复发送锁存末目标，属于 synchronized 的预期 hold，不是网络丢包。
- synchronized 的卡顿根因已确认是调度语义：完整 chunk 发送后必须等待目标跟踪、连续 `0.20 s` 稳定、
  再保持 `0.20 s`、采集新观测并完成远程推理，下一 chunk 才会开始。因此它适合隔离边界问题，不适合评价
  连续运动平滑性。增大 playback time scale 只能消除 arm clipping，不能消除 chunk 间停顿。
- `logs/tracking_retry_20260810_183404`：tracking governor 观测到误差从 `0.0171` 上升至约 `0.0356 rad`，
  phase rate 从 `0.7627` 降至 `0.1479`，随后随误差下降恢复至约 `0.91`。这是当前 `run=0.01`、
  `resume=0.03`、`stop=0.04 rad` governor 的主动降速，不是 heartbeat、stale feedback 或 policy 请求中断；
  但会被操作者感知为运动迟滞。
- 控制器只读 doctor（2026-08-11）：`/joint_states=80.5 Hz`、图像 `12.9 Hz`、robot/arm state `160.8 Hz`。
  新增无运动 tick 诊断显示：客户端 `late_ticks=0/skipped_ticks=0/max_gap=10.146 ms`；bridge 空载平均
  `99.73 Hz`、`65` 次间隔超过 `15 ms`、最大间隔 `22.691 ms`；连接 dry-run 后平均 `99.64 Hz`、`78` 次
  超过 `15 ms`、最大间隔 `27.034 ms`。这属于需要继续观测的控制器定时抖动，但量级不足以解释 synchronized
  已确认的 `0.7-1.0 s` chunk 停顿。
- 当前结论：两种“卡顿”不是同一个故障。synchronized 的长停顿是设计行为；tracking 的迟滞来自 governor
  对跟踪误差的降速，另有约 20-27 ms 的 bridge timer jitter 候选。未完成连续平滑运动修复前，不进入 RTC
  shadow 或 RTC merge。

### 2026-08-11 连续 prefetch 边界对照

- `logs/prefetch_scale3_20260811_103829`：3.0x、5 秒、3 次推理；除首次推理前预期的 15 个 hold tick 外，
  后续两次推理均在队列剩余 35 tick 时完成，`underruns_since_last=0`。操作员确认几乎没有 synchronized
  的 chunk 间长停顿。
- 客户端 publisher 为 `late_ticks=0`、`skipped_ticks=0`、`max_gap=10.816 ms`；bridge 平均 `99.61 Hz`、
  最大 gap `31.069 ms`。因此本轮抽动不是 action queue 空或客户端发送线程停顿。
- 第二次推理的 `new action[0] -> old queue tail` 最大臂关节差为 `0.15034 rad`，同时队尾相对当时已发送
  目标仍相差 `0.07810 rad`；第三次对应差值降为 `0.02512 rad`。这支持“异步观测产生的新 chunk 与旧队尾
  时间错位”是抽动来源，属于 RTC prefix conditioning/未来 knot merge 需要解决的问题。
- 本轮有 86 个 arm-clipped tick，记录到 action dimension 8 的一次样本为 `-1.20570 -> -1.20672`，即原始
  目标略微超出当前反馈 `0.08 rad` 包络；真正 RTC merge 时仍要求 arm clipping 为 0，否则 bridge governor
  必须冻结并 fallback。
- episode 动作结束后的人工 hold 阶段记录到两次极短门控变化：`robot_state=(2,3)` 和 `(1,3)`，随后恢复
  `(3,3)`。它们不在三次活动 chunk 的推理/追加时间点，但 RTC shadow 日志仍需确认没有活动轨迹门控丢失。
- 结论：允许进入 RTC shadow，因为 shadow 丢弃 RTC 输出，不执行 replacement merge；shadow 通过前不允许
  进入真实 RTC merge。

### 2026-08-11 settled RTC 五 chunk 卡顿调查

- `logs/rtc_five_chunk_20260811_115028` 完成初始 chunk 加 4 次 replacement，共 5 次推理；操作员确认没有
  前后抽动，但存在明显周期卡顿。
- 4 次 merge 的 `d_pred` 均为 2，`d_actual` 为 `2,1,1,1`；无 arm clipping、hard freeze、stale、拒绝或
  fallback。边界速度跳变为 `0.01913,0.01070,0.00495,0.02011 rad/knot`，未产生可感知回弹。
- 首次 A3 到 `rtc_resumed` 冻结约 `1.291 s`；随后三次约 `0.368,0.471,0.432 s`。其中每个 checkpoint
  固定要求约 `0.20 s` settle，首次还需等待跟踪误差从 `0.03286 rad` 降到容差内。
- bridge 平均 `99.53 Hz`、最大 gap `25.028 ms`，不足以解释上述 0.37-1.29 秒停顿。根因是 settled
  checkpoint 状态机，不是网络、policy 延迟或 action chunk 错位。
- 已新增显式 `--rtc-continuous` 模式；本地 fake bridge 验证 checkpoint 不暂停、拿图期间 elapsed knot
  计入 `d_actual`，以及后续整数边界 replacement 仍不暂停。下一真机阶段必须从 continuous shadow 开始。

### 2026-08-11 continuous RTC shadow

- `logs/rtc_continuous_shadow_20260811_135240`：`--rtc-continuous --rtc-shadow`，100 Hz、15 Hz、2.0x；
  checkpoint 事件为 `continuous_checkpoint=True`、`settle_s=None`、`frozen=None`，tracking error 为
  `0.02375 rad`，phase 在事件后继续运行到 `3.36`。
- `rtc_resumed` 在 `75.9 ms` 后到达，初始 `d_actual=0`；随后到 `d_pred=d_actual=2` 的 deadline 才暂停。
  该 deadline pause 和后续 synchronized fallback 是 shadow 丢弃 RTC 输出的预期行为，不是 continuous
  checkpoint 停顿。
- 本轮没有 clipping、stale、hard freeze 或 command rejection；bridge 平均 `98.94 Hz`、最大 gap `27.540 ms`。
- continuous shadow 通过，允许下一轮单次实际 continuous RTC merge；仍需操作员重新确认真机安全环境。
