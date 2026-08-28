# Marvin Pro OpenPI 真机部署交接

更新时间：2026-08-28

本文件是 Marvin Pro 真机部署的唯一入口文档。文档结构：`README.md` 为仓库索引；本文为当前生产
配置、真机快速开始、状态快照和待办与已知问题；[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 为
RTC/轨迹测试计划、诊断工具和带日期的测试记录；[`BASELINE_RUN.md`](BASELINE_RUN.md) 为基线与
真机 A/B 记录。原 `To_Be_Optimized.md`、`plan.md`、`RTC_TODO_20260818.md`、
`ASYNC_ACTION_CHUNKING.md` 的内容已分别并入本文和 `ROBOT_RTC_TESTS.md`。

部署链路：

```text
Marvin Pro 6.6.7.100                 本机                         GPU 192.168.50.73
ROS topics <-> 双向 bridge:7332 <-> rollout client <-> WebSocket policy:8000
```

rollout 客户端运行在本机，不运行在机器人控制器。机器人端只运行轻量 ROS bridge；GPU 服务器只运行
OpenPI policy server。

## 当前生产配置（2026-08-27/28 真机验证）

policy server 在 `192.168.50.73` 的 `/mnt/reacher-fast/openpi_ur_pp_202607/repo` 下启动：

```bash
cd /mnt/reacher-fast/openpi_ur_pp_202607/repo

CUDA_VISIBLE_DEVICES=2 uv run scripts/serve_policy.py --port=8000 --default-prompt "Stack the three red cones from right to left inside the white square to form a stable stack." policy:checkpoint --policy.config=pi05_marvinpro_red_cones_slow --policy.dir=checkpoints/pi05_marvinpro_red_cones_slow/pi05_marvinpro_red_cones_slow_260826_full/79999
```

- 任务 prompt：`Stack the three red cones from right to left inside the white square to form a
  stable stack.`。客户端 `DEFAULT_PROMPT` 仍是旧表述，所有客户端命令必须显式传 `--prompt`。
- 旧 checkpoint `pi05_marvinpro_red_cones` / `marvinpro_red_cones_40k_gpu67/39999` 和旧 prompt 不再
  使用，仅保留在 [`BASELINE_RUN.md`](BASELINE_RUN.md) 的历史记录中。
- 看到 `server listening on 0.0.0.0:8000` 才进入下一步。首次推理 JIT 可能很慢；rollout 默认做一次
  warmup 并丢弃其输出，绝不执行 warmup 动作。

### 已按当前设备固定的接口

- 现场接口：机器人控制与状态 topic 使用 `/tj` 命名空间；四宫格相机使用根路径
  `/quad_tile/compressed`，`CompressedImage.format` 为 `h264`，客户端用有状态 PyAV 解码连续 H264
  包再按训练布局切分。
- 观测/动作顺序：`[左臂7, 左夹爪, 右臂7, 右夹爪]`。关节单位 rad；policy server 输出已经过
  `AbsoluteActions`，是绝对关节目标。
- 夹爪：模型使用 `0=open, 1=closed`；命令直接向 `/tj/control/gripperValueL/R` 发布 `0..1`。policy
  state 使用 `/tj/info/gripper_feedback_L/R` 的实测 `q`，按 `open_raw=0.0`、`closed_raw=1.25`
  归一化并裁剪到 `0..1`。feedback 字段布局为
  `[q_position_rad, dq_velocity_rad_s, tau_torque, temp_mos, temp_motor]`；2026-08-19 实测右夹爪
  “打开→夹持→打开”时 `q` 约为 `-0.0185→1.0234→-0.0185`，稳定 `tau` 约为 `0.047→1.683→0.046`。
  任一侧 feedback 缺失或过期会关闭运动门。
- 相机：四宫格左上=`cam_high`，左下=`cam_left_wrist`，右下=`cam_right_wrist`，右上忽略；底部
  时间戳区域不进入模型。实测相机约 `11.4 Hz`；交给 policy 的是图像时刻锁存的最新关节状态。
- 双臂目标：`/tj/control/user/joint_cmd_A/B`，消息类型 `marvin_msgs/msg/JointcmdArm`，A=左、B=右。
- 硬限位：来自控制器当前 `APEX_ROBOT_MODEL=new_m6_696` 的左右臂 URDF。
- bridge 不会调用 `/control/set_input`、`set_ready`、`go_home`、`clear_fault` 等 Service，也不会
  自动改变机器人模式。

## 真机快速开始

推荐顺序：只读预检 -> dry-run -> synchronized 回归 -> RTC shadow -> RTC 实际 merge。每步的完整
验收标准见 [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md)。

**每次真机运行必须记录日志。** 客户端 telemetry CSV 以 `"w"` 覆盖模式打开，复用同名文件会丢失
上一段数据（2026-08-19 覆盖事故见 [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 的 RTC 决策记录），
因此每个 episode 必须使用全新 `RUN_DIR`（`logs/` 已被 `.gitignore` 忽略，按
`logs/<用途>_$(date +%Y%m%d_%H%M%S)` 建立），bridge 的 `--local-log` 与客户端 `--log-file` 都
指向该目录。

### 0. 机器人端只读预检

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --doctor --duration 8
```

预检不创建动作 publisher。以下输入必须都有消息：`/tj/joint_states`、`/quad_tile/compressed`、
`/tj/control/input_mode`、`/tj/info/robot_state`、`/tj/info/arm_state`、
`/tj/info/gripper_feedback_L/R`。相机没有消息时先在 Apex 启动 Camera。执行前还必须看到
`input_mode=3`、两个状态数组均为 `(3, 3)`（关节阻抗模式）；dry-run 阶段可以仍为 None/`0`。

### 1. 全链路 dry-run

终端 A 启动不允许运动的 bridge（`./scripts/run_bridge_on_controller.sh`），终端 B：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --prompt "Stack the three red cones from right to left inside the white square to form a stable stack." \
  --episode-seconds 10
```

默认没有 `--execute`，客户端不会向 bridge 发送任何 action。检查日志：policy 输出恒为 `(20, 16)`
且 finite；单次请求必须小于默认 2 秒 timeout（远程 H20 服务 2026-08-19 旧 checkpoint 实测 20 次
持久请求 wall p50/p95 约 `343/390 ms`，仅作量级参考）；相机、关节和左右夹爪 age 没有超限，日志
显示 `gripper_state_source=measured_feedback`。

### 2. synchronized 回归基线（真机第一步）

清空工作区并保持急停可触及，在 Apex 完成 Robot Ready、Impedance Mode、安全的任务起始姿态和
Camera 启动；不要在出厂打包姿态直接 Home；Input Mode 先保持 None。建立本轮日志目录并启动
motion bridge：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
RUN_DIR=logs/synchronized_$(date +%Y%m%d_%H%M%S) && mkdir -p "$RUN_DIR"
printf '%s\n' "$PWD/$RUN_DIR" | tee /tmp/marvinpro_run_dir
./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion \
  --publish-hz 100
```

终端 B。注意本仓库 argparse 强制 trajectory schedule 使用 `--control-hz 100 --model-hz 15
--playback-time-scale 3 --execute-steps 20`（固定 5 Hz knot rate、H=20）；legacy sync 仓库当年
验证的 `--playback-time-scale 2 --execute-steps 10` 会被本仓库直接拒绝：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --prompt "Stack the three red cones from right to left inside the white square to form a stable stack." \
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
  --log-file "$RUN_DIR/client.log"
```

指定 `--log-file` 后，客户端自动把逐条 telemetry 写入同名 `<stem>.telemetry.csv`（每行对应一次
bridge state update 或 client command；`record_type` 字段区分 `bridge_state` 和 `client_command`）。
测试后生成关节角与夹爪命令/实测 feedback 对照图：

```bash
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
/home/jh/OpenPI_UR/openpi/.venv/bin/python \
scripts/plot_rollout_joints.py \
  "$RUN_DIR/client.telemetry.csv" \
  -o "$RUN_DIR/joint_diagnostics.png"
```

### 3. RTC shadow（真机第二步）

synchronized 回归通过后，按同样的 RUN_DIR 约定运行 continuous RTC shadow（RTC 输出只记录、不
合并，shadow 随后固定降级 synchronized，不自动重进 RTC）：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --prompt "Stack the three red cones from right to left inside the white square to form a stable stack." \
  --execute \
  --episode-seconds 20 \
  --rollout-schedule rtc \
  --rtc-continuous \
  --rtc-shadow \
  --rtc-late-result-policy discard \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 3 \
  --execute-steps 20 \
  --max-rtc-recoveries 3 \
  --max-stuck-replans 2 \
  --policy-connect-timeout 5 \
  --policy-request-timeout 2 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rtc-shadow.log"
```

### 4. RTC 实际 merge（真机第三步）

shadow 通过后删除 `--rtc-shadow`；**首轮实际 merge 必须使用 `--max-rtc-merges 1`**，确认一次成功
replacement merge 无抽动后，再按 [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 逐步放宽到
`--max-rtc-merges 2`，merge 数按 1 -> 2 -> 10 逐级放大，不直接做 20-merge soak。每次实际 merge
尝试都要新建 `RUN_DIR`：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_run_dir)"
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --prompt "Stack the three red cones from right to left inside the white square to form a stable stack." \
  --execute \
  --episode-seconds 20 \
  --rollout-schedule rtc \
  --rtc-continuous \
  --rtc-late-result-policy discard \
  --max-rtc-merges 1 \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 3 \
  --execute-steps 20 \
  --max-rtc-recoveries 3 \
  --max-stuck-replans 2 \
  --policy-connect-timeout 5 \
  --policy-request-timeout 2 \
  --log-level DEBUG \
  --console-log-level WARNING \
  --log-file "$RUN_DIR/rtc-merge1.log"
```

### 现场操作约定与安全门控

- 程序打印实机状态后，必须手动输入单个大写 `E` 才开始动作；不建议首次执行使用 `--yes`。
- 动作结束后客户端只采样一次当前反馈位姿，并持续发送这个固定目标作为 hold（不跟随后续反馈）。
  此时在 Apex 把 Input Mode 切回 **None**；客户端检测到模式不再是 Custom 后才断开，最后再用
  `Ctrl+C` 停止 bridge。异常时优先切 None 或急停，不把 `Ctrl+C` 当作正常停止步骤。
- 切到 Custom 前后可用只读命令确认：
  `ssh nvidia@6.6.7.100 'source /etc/apex/apex_ros_env.sh; ros2 topic echo /tj/control/input_mode --once; ros2 topic echo /tj/info/robot_state --once; ros2 topic echo /tj/info/arm_state --once'`，
  预期分别是 `3`、`[3,3]`、`[3,3]`。
- 确认真机正常后再逐步增加 `--episode-seconds`。
- 真机动作要同时满足以下门控，任一失效 bridge 清空目标并停止发布：
  1. bridge 使用 `--allow-motion` 启动；
  2. rollout 使用 `--execute` 并人工输入单个大写 `E`；
  3. `input_mode == 3`；
  4. `/tj/info/robot_state == [3,3]` 且 `/tj/info/arm_state == [3,3]`（关节阻抗模式）；
  5. 关节和左右夹爪 feedback 新鲜，归一化夹爪实测值在 `[0,1]`，policy action 为 finite `(16,)`；
  6. 客户端和 bridge 的臂关节目标相对最新反馈最多 `0.16 rad`；
  7. 目标位于当前 M6-696 URDF 硬限位内并保留 `0.02 rad` 边界；
  8. bridge 收到的 action 不超过 `0.25 s`，且对应观测不超过 8 帧。
- action chunk 暂时耗尽时，客户端发送当前测量位姿作为 hold，不会重复执行过时的预测动作。
- bridge 使用 pickle 传输 JPEG 和数据结构，只能暴露在可信的机器人私有网络，不应映射到公网。

### 轨迹执行语义要点（protocol v10）

- `synchronized`、`tracking` 和 `rtc` 使用 bridge 本地 100 Hz trajectory owner；控制 timer 只对
  连续 phase 求值，不会按 100 Hz 自动消费模型动作。三种 schedule 固定 `--model-hz 15
  --playback-time-scale 3`（名义 5 Hz knot rate），其他倍率会被客户端拒绝。
- tracking governor：`error <= 0.02 rad` 时 phase rate 为 1；`0.02..0.16 rad` 按
  `(0.16-error)/0.14` 线性降低；`>= 0.16 rad` 硬冻结，降到 `<= 0.12 rad` 才解除锁存；joint state
  stale、timer overrun 和 arm clipping 直接硬冻结。臂关节 safety clipping 包络 `0.16 rad`。
- RTC A9 checkpoint 只有在全部 14 个臂关节误差不超过 `0.01 rad`，并且由持续更新的 joint source
  timestamp 证明连续稳定 `0.20 s` 后才成立；“客户端已经发出 A9，但反馈仍在 A8”不会触发新观测。
- `d_pred` 使用当前 estimator epoch 内可行 latency 样本的保守 p95 加 `50 ms` guard；会超过四个
  old-tail knot 的样本作为 link fault 单独记录。默认 `--rtc-late-result-policy discard`；`d_actual`
  始终按 bridge 实际 phase 跨过的 knot 计数，不使用 `wall_time * nominal_rate`。
- trajectory session 每 `100 ms` 发送 heartbeat；bridge 超过 `250 ms` 未收到会清空 trajectory 并
  停止发布。旧的 discrete/prefetch 仍使用 `ActionCommand`。
- protocol v10 必须同时更新控制器上的 `MarvinPro_deploy` 和本机客户端。夹爪状态使用归一化实测
  feedback；RTC 的 tracking governor 只使用 14 个机械臂关节误差，夹爪不参与机械臂到位判定。
- 每次 RTC 请求日志分段记录 observation preparation、request build/serialization、transport round
  trip、估算的 network round trip、server deserialize/queue/infer、RTC
  preprocess/denoise/postprocess、response decode 和 bridge stage/merge。

## 常用运维工具

夹爪直接控制（Apex Home 无法让夹爪完全张开时使用；先停止 rollout 和 bridge，确认 Apex 没有运行
Teleop/Replay，执行闭合前让手和物体离开夹爪；若检测到 rollout bridge 仍在运行，脚本会拒绝发布）：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy

# 同时完全打开 / 闭合左右夹爪
./scripts/control_gripper_on_controller.sh 0
./scripts/control_gripper_on_controller.sh 1

# 只控制一侧
./scripts/control_gripper_on_controller.sh 0 --side left
./scripts/control_gripper_on_controller.sh 1 --side right
```

夹爪 feedback 只读记录器（验证“打开→闭合夹住物体→打开”过程中 feedback 是否变化；CSV 保存到本机
`logs/`，`Ctrl+C` 结束后看 `changed`、`distinct`、`span`、`max_step` 摘要）：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/record_gripper_feedback_on_controller.sh
```

## 2026-08-28 状态快照

- **真机 A/B（2026-08-27/28）**：之前表现很差的真机运行来自 legacy discrete 默认路径
  （prefetch + discrete，15 Hz 阶梯目标、硬队列替换），不是 RTC；sync 轨迹路径运动平滑并正确完成
  抓取；堆叠放置仍偏数厘米，疑似 sync 模式 chunk 间停顿的伪影，待 RTC 实际 merge 真机运行后复查。
  完整记录见 [`BASELINE_RUN.md`](BASELINE_RUN.md)。
- **生产 server/checkpoint/prompt 已更换**：见上文“当前生产配置”。
- **本仓库 checkout 已迁移**到 `/home/jh/TianJi_Marvinpro/MarvinPro_deploy`，文档中的路径已全部
  更新。
- **legacy_sync 仓库**（`MarvinPro_deploy_legacy_sync`）需要 `/tj` topic 命名空间和 H264 解码补丁
  才能工作：机器人现在以 `apex_ros_namespace:=tj` 运行，相机发布 h264 四宫格。
- **文档已合并**为 `README.md`（索引）、本文、[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md)、
  [`BASELINE_RUN.md`](BASELINE_RUN.md) 四份。

## 待办与已知问题

### 已知性能问题与优化方向（2026-08-17 调研，原 To_Be_Optimized.md）

针对当前 H20/s10、5 Hz、`d_max=4`、指数 soft mask 基线。评估分安全性、运动连续性、策略响应性、
任务能力四个维度，不能合并成单一“好坏”。

1. **默认 settled checkpoint（P0：continuous 作为正式运行语义）**：settled checkpoint 要求全部 14
   关节 `<=0.01 rad` 且稳定 `0.20 s`。2026-08-11 settled 五 chunk 真机：无 clipping/freeze/stale/
   拒绝，4 次边界速度跳变无可感知回弹，但每次 checkpoint 明显停顿——首次 `1.291 s`，后续
   `0.368/0.471/0.432 s`（bridge 平均 `99.53 Hz`、最大 gap `25.028 ms`，不足以解释；根因是
   checkpoint 状态机的固定等待）。方向：continuous RTC 作为性能评估和最终运行语义，settled 仅用于
   诊断；观测等待期间跨过的 knot 计入 `d_actual`；不要靠把 `0.20 s` 改成 `0.05 s` 掩盖模式问题。
2. **Tracking governor 改变模型时间语义（P1：找固定可跟踪 knot rate）**：governor 阈值
   `run=0.02 / resume=0.12 / stop=0.16 rad`，叠加在已慢放到 5 Hz 的 knot rate 上（time scale 3 且
   `phase_rate=0.2` 时瞬时有效执行速度只有 `1 Hz`）。真机记录：误差从 `0.0171` 升到约
   `0.0356 rad` 时 phase rate 从 `0.7627` 降到 `0.1479`，随后恢复约 `0.91`，操作者感知为迟滞。
   历史 synchronized 失败运行出现 `0.27883 rad` peak tracking error 和 228 个 arm clipped tick，
   说明 governor 不是冗余功能。方向：解耦“正常运行速度”和“异常安全冻结”；用 frozen chunk/固定
   计划测试 5、7.5、10、15 Hz 并记录 tracking p50/p95/max、clipping、关节速度和操作者感知；把
   phase-rate 分布作为任务指标；governor 只按 14 个臂关节误差，不看夹爪。
3. **`d_pred`、deadline freeze 与短 horizon（P0：治理延迟尖峰）**：
   `d_pred = ceil((p95(最近20次稳定wall) + 0.05 s) * effective_knot_hz)`，限 `1..4`。
   2026-08-12 protocol v5 事故：前 9 次 RTC wall `218.7-477.5 ms`，第 10 次突增到 `1560.1 ms`；
   该结果以 `d_actual=3` merge 后，下一次 `predicted_steps()` 得到 9 超界，RTC 对该 episode 永久
   关闭并进入 synchronized fallback。当前 H20/v10 已把超 horizon 样本标为 link fault、不写入稳定
   分布，并在恢复前重建 estimator epoch；仍需真机验证不产生恢复振荡。方向：分解并治理
   transport/wall-latency 尖峰；区分稳定预算与异常尖峰；保持 `d_actual` 物理计数。
4. **Hard anchor 与执行端 C2 blend（P1：导数连续准入）**：protocol v10 从旧轨迹真实 `q/v/a` 优先
   生成 quintic C2 blend。2026-08-28 起 blend 窗口按**秒**恒定（目标 `0.6 s`，knot 数随 knot rate
   缩放：5 Hz 候选 `(3, 2)` 与历史一致；15 Hz 候选 `(9, 6, 3, 2)`，并按剩余 checkpoint 距离裁剪），
   blend 上限为按 knot rate 查表的显式包络 `BLEND_CAPS_BY_KNOT_HZ`（CLI `--rtc-blend-max-*` 可整体
   覆盖）：5 Hz 为运行验证过的 `0.45 / 2.0 / 40`；15 Hz 由 `stack_cones_slow_260826` 遥操作数据
   （104 集原生 15 Hz：p99.9 = `0.556/2.34/49.8`，示教最大值 = `1.167/14.2/289`）标定为
   `max(3x p99.9, 1.3x max)` = **`1.7 / 18.0 / 380`**，URDF 速度上限仍逐关节取 min。不要用立方
   缩放外推 jerk 上限——窗口按秒恒定后接缝 jerk 只随速度差线性增长，立方外推（1080）会是示教
   最大值的 3.7 倍。全部候选不可行时原子拒绝；未验证的 knot rate 没有包络，直接拒绝。
   历史损失：2026-08-12 `1560.1 ms` 尖峰对应的 merge 曾接受
   `boundary_velocity_jump=0.10757 rad/knot`（5 Hz 下约 `0.538 rad/s`）；2026-08-17 10-merge soak
   前 8 次成功、最大 `0.09203 rad/knot`（约 `0.460 rad/s`），操作员报告明显抽动，对照无异常
   2-merge 短测试约 `0.0143/0.0135 rad/knot`（该轮 bridge 平均 `99.49 Hz`、最大 gap `30.1 ms`、
   最大 tracking error `0.0413 rad`，抽动不能归因于 publisher 或 clipping）。H10 时代 `d=2` 的
   soft 权重仅约 `1.000/1.000/0.368/0.077`；H20 在 5 Hz 下约 4 秒预测 horizon、`d=2` 后有约 8 个
   soft transition knot，但真机仍出现过 C2 jerk 超限——更长 soft overlap 缓解生成边界，不能替代
   执行端 C2 准入和 fallback。`H=50` 属于需要重新确定时间尺度的训练实验（原论文 50 点运行在
   50 Hz、物理 horizon 约 1 秒；本项目 5 Hz 下 50 点会是 10 秒），不是部署参数修复。
5. **RTC 失败后的同步回退（P2）**：恢复链为 失效 -> 实测 fixed hold -> 稳定 `0.20 s` -> 新观测 ->
   一个 clean sync chunk -> 重建 estimator epoch -> `s=10` bootstrap -> 回 RTC，每 episode 最多 3
   次，第 4 次留在 timed synchronized。2026-08-12 长运行 fallback 后 sync 推理 wall 为
   `228.6/350.1/352.4/1194.7 ms`，最后一次推理完成时源观测已落后 17 帧（限制 8），旧版本因此
   abort；当前工作树会对该 observation-lag 情况重新观测和推理。回退结果必须单独计分
   （RTC-only 完成 / RTC 后恢复完成 / synchronized fallback 完成 / 安全 abort / 任务失败），不能
   把 fallback 完成的动作计入 RTC 成功率。

不应作为性能优化目标删除的机制：finite/shape/URDF 校验；input mode、robot state、arm state
readiness gate；joint state freshness、heartbeat 和 timer overrun 检查；
session/plan/timeline/checkpoint/request ID；`d_actual` 的物理 knot crossing 计数；state/image
时间屏障；hard stop 和 arm clipping 后的 RTC invalidation。

最终 A/B 验证要求：相同机器人、checkpoint、初始姿态、物体布局和任务条件下比较
`A: 当前 continuous RTC`、`B: A + 优化后的稳定延迟链路`、`C: B + derivative-aware merge`、
`D: C + 经过重训和离线验证的长 overlap RTC`、`E: D + 优化后的 fixed-rate/soft governor`。每个
配置至少记录：任务成功率和完成时间；RTC merge 数、fallback rate、安全 abort rate；
inference/transport latency p50/p95/p99/max；`d_pred/d_actual` 分布和 deadline freeze 时间；
phase rate 分布、tracking error 和 clipping；merge 位置/速度/加速度跳变；100 Hz reference、
sent target 与真实反馈；操作者盲评。

参考资料（2026-08-17 核对）：PI RTC 论文 <https://arxiv.org/abs/2506.07339>（真实机器人
`H=50`、`s_min=25`、`beta=5`、`b=10`、50 Hz）；
[real-time-chunking-kinetix](https://github.com/Physical-Intelligence/real-time-chunking-kinetix)；
[LeRobot RTC 文档](https://huggingface.co/docs/lerobot/en/rtc)及 ActionQueue/RTCProcessor；
[PickNik Ruckig stitch](https://docs.picknik.ai/how_to/robotics_applications/stitch_trajectories/)；
[PACE](https://arxiv.org/abs/2606.00537)；
[Training-Time Action Conditioning](https://arxiv.org/abs/2512.05964)；
[REMAC](https://arxiv.org/abs/2601.20130)。

### 实施计划状态（原 plan.md，2026-08-10 起）

阶段 0-6 已完成：协议升级到 v10、bridge 本地连续 trajectory、tracking governor、物理 checkpoint、
RTC merge、synchronized episode fallback、OpenPI 原生 JAX RTC sampler（absolute prefix 变换、VJP
guidance、`rtc_v1` envelope、结构化错误和计时）均已实现并通过真机验收。阶段 7（RTC 与 tracking
timeline 集成）的关键项已随 protocol v10 落地（稳定 p95 + 50 ms guard 的 `d_pred`、link fault
隔离、迟到结果默认 discard、fallback 重置 epoch、分段延迟日志、merge 使用 bridge 实际 knot
crossing 的 `d_actual`）。原清单中未勾选、仍需真机证据核对的项目：

- [ ] 推理开始时冻结一份 old remaining reference 和 timeline version 的完整真机证据；
- [ ] 仅当 request/plan/timeline/checkpoint ID、反馈新鲜度和 RTC 约束全部有效时允许 merge 的注入
  测试（延迟突增、响应乱序、重复响应、旧 timeline 响应全部被拒绝）；
- [ ] `d_actual > d_pred`、旧 prefix 耗尽或越过可替换边界时丢弃并 fallback 的真机证据；
- [ ] 推理期间任意 arm clipping、tracking hard freeze 或状态过期使结果失效的真机注入测试；
- [ ] merge 与 100 Hz publisher 并发无半更新状态；plan underrun 时固定 hold；
- [ ] merge 前后 raw reference、sent target、feedback 的位置/速度/加速度边界差的系统记录。

原计划的完成判据（全部仍开放）：稳态无 plan underrun；正常阶段无 arm clipping；每次 policy 观测
都有可证明的 tracking checkpoint 和新图像时间屏障；每次 merge 满足 `d_actual <= d_pred <= 4` 和
timeline version 一致；chunk 边界最大关节位置差显著低于失败样本的 `0.16913 rad` 并接近或优于
synchronized 基线；不再出现约 1.33 秒周期回弹；RTC p95 稳态延迟不破坏可行性；synchronized
fallback 和退出固定 hold 通过回归；真机任务成功率和完成时间不低于 synchronized 基线。

仍然有效的安全约束：客户端和 bridge 使用 `0.16 rad` 反馈包络；URDF 限位和 `0.02 rad` margin 不
放宽；反馈过期、motion gate 关闭、命令拒绝或计划版本不一致时立即停止推进；空计划只锁存一次固定
目标；clipping 不能被统计后忽略；未通过离线测试和 dry-run 前不得运行 RTC 真机动作；不修改官方
低层控制器参数，不把提高发布频率当作跟踪修复。

### 当前待办汇总

- [ ] **2026-08-28 真机中止待厂家确认（robot_state=(1,12)）**：15 Hz 原速 RTC 运行
  （`logs/rtc_20260828_143021`，60 s 任务）在运动后约 12.5 s 被中止：`/tj/info/robot_state`
  从 `(3,3)` 掉到 `(1,12)` 且不自愈，bridge 按设计阻断发布，client 报 `fatal_safety_hold` 退出。
  数据层面已排除指令侧原因：17 次 merge 全部通过、0 recovery，blend jerk 实测最大 ~105（包络
  上限 380），tracking error 全程 max `0.0346 rad`、跳变前 1 s 均值 `0.0072`，跳变瞬间右臂各
  关节速度 <=`0.12 rad/s`（正在搬运第一个锥筒），跳变后 6 s 关节漂移 <`0.003 rad`、夹爪保持
  夹持。状态码 `(1,12)` 的定义在控制器固件侧，需向机器人厂家确认含义与触发条件后再决定对策
  （历史上 2026-08-11 曾记录到 `(2,3)`/`(1,3)` 瞬时抖动并自愈，本次未恢复）。重连机器人后先确认
  状态恢复 `(3,3)`、Apex 无报警再重跑，复现时记录是否在抓取/搬运阶段。
- [ ] 按 [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 执行新 checkpoint 下的 dry-run ->
  synchronized -> RTC shadow -> `--max-rtc-merges 1` 真机验收；merge 数按 1 -> 2 -> 10 逐级放大，
  不直接做 20-merge soak；每次使用独立 RUN_DIR，merge 与 fallback episode 分开统计。
- [ ] RTC 实际 merge 真机运行后，复查堆叠放置数厘米偏差是否为 sync 模式 chunk 间停顿伪影
  （见 [`BASELINE_RUN.md`](BASELINE_RUN.md) A/B 记录）。
- [ ] 离线提取 2026-08-19 成功 merge 与两次失败 merge 的旧/新边界，按关节对比位置、速度、加速度
  和 jerk；评估更长但仍受约束的 C2 blend；不得再直接放宽 jerk 上限。
- [ ] H10/H20 对比必须固定机械臂初始姿态、物体布局和两侧夹爪状态，gripper clipping 单独统计。
- [ ] 夹爪左右各自完整、同工况的开合端点标定未完成；旧的 `0.0~1.25 -> 0~1` 映射不能视为已验证。
- [ ] 验证新 estimator epoch 的 `d_pred` 只来自新样本，所有成功 merge 满足
  `1 <= d_actual <= d_pred <= d_max`。
- [ ] knot rate 晋级（`5 -> 7.5 -> 10 -> 15 Hz`、分级 `d_max`，方案见
  [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 第 9 节）尚未实施；任何 `d_max`
  变更都是跨端协议变化，禁止设置 `d_max=10`。

## 历史档案（2026-08-05 至 2026-08-07）

以下为 2026-08-05/07 的真机诊断记录，保留原始数据和结论；其中使用的 checkpoint、prompt 和
2.0x/10 knot 参数均已过期，真机执行以本文上文为准。

## 2026-08-05 收工快照

当前排查目标是解释“持续 rollout 几乎每步颤抖”。截至收工，证据不支持把官方低层控制器列为首要
嫌疑，已经确认或高度怀疑的是以下四个项目侧因素：

1. 15 Hz 离散绝对位置目标形成阶梯输入；改成100 Hz插值后，多项真机指标改善。
2. 原始policy chunk本身有较高的速度和二阶变化，简单插值只改变发送连续性，不改变轨迹总时长。
3. 客户端按墙钟推进动作，不等待机器人达到上一目标；原始chunk回放中右臂 `Joint4_R` 出现明显表观
   跟踪滞后。
4. 持续rollout的推理约占2至3个15 Hz节点，但新结果到达后仍从 `new[0]` 硬替换旧计划，没有延迟
   索引补偿、边界连续化或plan underrun消除。

当前尚不能下的结论：

- 不能把报告的 `apparent tracking error` 直接解释为低层伺服误差；约11 Hz相机采样和传输相位包含
  在该数值内。
- 没有证据表明官方Home的 `0.5 rad/s^2` 规划上限也是 `marvin_robot_node` 的硬截断上限；Custom
  路径绕过了Home轨迹规划器。
- `24/140` 是冻结诊断客户端 `±0.03 rad` 包络产生的裁剪，不是bridge裁剪。bridge对超出实时反馈
  `±0.12 rad` 的命令会拒绝而非修改；本次raw计划没有触发该拒绝条件。
- 单个冻结chunk的结果不足以证明所有模型输出都过快；应先完成同一chunk的时间尺度对照。

用于明日严格复现的计划已经从 `/tmp` 固化到：

```text
/home/jh/TianJi_Marvinpro/MarvinPro_deploy/artifacts/marvinpro_red_cones_chunk_ab_v2.json
SHA-256: e72510cbba1a4a18517bbb7b76de81d9847128fc0907558f56b5ac662e11c6d0
```

该文件包含同一次推理的原始节点和有界节点，明日不要重新捕获计划，否则无法与今晚数据做严格对照。
保存的锚点按 `Joint1_L..Joint7_L, Joint1_R..Joint7_R` 排列为：

```text
[1.697043, -1.101982, -1.091558, -1.995128, -0.374360, 0.135180, 0.554539,
 -1.706437, -1.094713, 1.097878, -1.990542, 0.378431, 0.137912, -0.551586]
```

若机器人不在该姿态的 `0.01 rad` 内，客户端会在发动作前退出；不要通过放宽门控强行执行，应先用
Apex的安全方式回到测试姿态，或明确放弃严格同计划对照并重新建立一套基线。

## 代码与运行状态

- 本目录是独立 Git 仓库；此前诊断提交为 `11a1aac Add motion smoothness diagnostics`，更早基线为
  `8fb4232 Initial Marvin Pro rollout deployment`。
- 本次收工快照包含：
  - `frozen_chunk_test_client.py`：增加 `--target-source raw` 和只允许放慢的
    `--playback-time-scale`。
  - `README.md`：增加raw回放和2倍慢速回放命令。
  - `_HANDOFF.md`：本次实验数据和明日计划。
  - `artifacts/marvinpro_red_cones_chunk_ab_v2.json`：固定的version 2 A/B计划。
- 已提交的诊断代码包括：
  - `src/marvinpro_deploy/hold_test_client.py`
  - `src/marvinpro_deploy/motion_profile.py`
  - `src/marvinpro_deploy/trajectory_test_client.py`
  - `src/marvinpro_deploy/frozen_chunk_test_client.py`
  - `tests/test_motion_profile.py`
  - `pyproject.toml` 中的 `marvinpro-hold-test` 入口
  - `pyproject.toml` 中的 `marvinpro-trajectory-test` 入口
  - `pyproject.toml` 中的 `marvinpro-frozen-chunk-test` 入口
  - `README.md` 中的三项诊断说明
- 机器人控制器：`nvidia@6.6.7.100`。
- OpenPI policy server：`192.168.50.73:8000`。
- 2026-08-05 17:13 后只读确认机器人端进程为：

  ```text
  python3 -m marvinpro_deploy.robot_bridge --allow-motion --publish-hz 100
  ```

- 真机执行门控已经按当前固件修正并实测：
  - Apex Input Mode Custom：`input_mode=3`
  - 关节阻抗模式：`robot_state=(3, 3)`、`arm_state=(3, 3)`
  - 旧文档中的 `(2, 2)` 不适用于当前控制器。
- 训练数据集 `stack_red_cones/meta/info.json` 明确记录 `fps=15`，三路视频也均为15 Hz。因此将模型
  节点解释为15 Hz时间语义是有数据依据的；这不代表机器人必须用15 Hz阶梯位置命令执行节点。
- bridge的完整policy观测在 `/quad_tile/compressed` 回调中形成，实测相机约 `11.4 Hz`；关节状态
  虽约96 Hz，但交给policy和客户端统计的是图像时刻锁存的最新关节状态。

## 已确认的控制链

当前部署使用 Marvin Pro 官方低层控制链，但没有使用官方 Home 的轨迹生成器：

```text
rollout client
  -> TCP bridge
  -> /control/user/joint_cmd_A、/control/user/joint_cmd_B
  -> 官方 joint_mux_node（Custom 输入）
  -> /control/joint_cmd_A、/control/joint_cmd_B
  -> 官方 marvin_robot_node
```

官方 Home 运动由 `planner_joint_node` 生成平滑轨迹，已查到的参数为：

- 规划控制率：`500 Hz`
- 各关节 Home 速度限制：`0.2 rad/s`
- 各关节 Home 加速度限制：`0.5 rad/s^2`
- `joint_mux_node` 模式切换 ramp：`3.0 s`
- `marvin_robot_node` 控制率：`200 Hz`

原 rollout 客户端则以 `15 Hz` 给出离散绝对关节目标，默认客户端单步限幅为 `0.08 rad`，没有速度、
加速度或 jerk 轨迹整形。因此“官方 Home 很平滑”不能证明当前 rollout 的目标生成方式也平滑。

## 模型与重规划诊断

在机器人静止、完全不发送 `ActionCommand` 的条件下，对真实 policy 做过 20 次推理：

- 机器人关节观测最大跨度：`0.000007 rad`
- 端到端推理延迟：median `144.9 ms`，p95 `166.1 ms`，max `190 ms`
- action chunk 内相邻步变化：median `0.00067 rad`，p95 `0.01086 rad`，p99 `0.01809 rad`，
  max `0.02620 rad`
- chunk 内隐含速度：p99 `0.27133 rad/s`，max `0.39296 rad/s`
- chunk 内隐含加速度：median `0.12362 rad/s^2`，p95 `0.60238 rad/s^2`，
  p99 `0.94266 rad/s^2`，max `1.56389 rad/s^2`
- 相邻重规划的首目标变化：median `0.00111 rad`，p95 `0.00858 rad`，p99 `0.01447 rad`，
  max `0.01637 rad`
- 模型夹爪输出范围约为 `[-0.01428, 0.01040]`，约 `54.5%` 落在 `[0, 1]` 外，因此 rollout
  中夹爪 clamp 警告并不意外。

当前推理延迟相当于约 `2.1` 个 15 Hz 控制步。rollout 在推理期间继续执行旧 chunk，推理完成后却从新
chunk 的 index 0 开始替换，未做延迟补偿。实测新旧计划边界差异为：

- `new[0] - old[1]`：最大 `0.02002 rad`，等效 `0.300 rad/s`
- `new[0] - old[2]`：最大 `0.03284 rad`，等效 `0.493 rad/s`
- `new[0] - old[3]`：最大 `0.04652 rad`，等效 `0.698 rad/s`
- `new[0] - old[4]`：最大 `0.06386 rad`，等效 `0.958 rad/s`

因此已有较强证据表明，新旧 action chunk 的时序错位和替换跳变是 rollout 抖动的重要来源。

## 恒定姿态保持对照

`hold_test_client.py` 不连接 policy server。它只锁存一次当前 14 关节位置，随后持续发送完全相同的
16 维绝对目标，用来隔离模型和重规划。

两次测试姿态相近，样本数均为 222；统计包含 10 秒正式测试和等待操作员切回 None 的约 4.7 秒。

| 指标 | bridge 15 Hz | bridge 100 Hz | 变化 |
| --- | ---: | ---: | ---: |
| 最大关节峰峰值 | `0.003019 rad` | `0.000891 rad` | 降低约 `70.5%` |
| 最大关节标准差 | `0.000281 rad` | `0.000191 rad` | 降低约 `32.0%` |
| 最大跟踪误差 | `0.003019 rad` | `0.000892 rad` | 降低约 `70.5%` |

100 Hz 时最大峰峰值约为 `0.051 deg`。14 个关节的峰峰值都比 15 Hz 测试低。15 Hz 最大值出现在
`Joint2_R`，100 Hz 最大值出现在 `Joint6_R`。

当前结论：

1. 恒定目标下，Custom、bridge 和官方阻抗控制链总体稳定，未复现 rollout 的明显抖动。
2. 将 bridge 的 ROS 重复发布频率从 15 Hz 提高到 100 Hz 后，恒定保持数据明显改善；原 15 Hz 发布
   是一个实际影响因素。
3. 这不等于 rollout 抖动已经解决。客户端仍只以 15 Hz 改变目标值，动态目标仍是阶梯信号，且新旧
   chunk 的替换仍未做延迟补偿。
4. 当前证据不支持优先怀疑官方低层运控。更可疑的是本项目的目标更新频率、缺少轨迹整形，以及异步
   重规划衔接。

## 确定性慢速轨迹对照

在 bridge 固定为 100 Hz、姿态相近且不连接 policy server 的条件下，对 `Joint7_L` 执行相同的最小
jerk 轨迹：`0 -> +0.04 -> -0.04 -> 0 rad`。第一组客户端命令更新为 15 Hz，第二组为 100 Hz。

| 指标 | 客户端 15 Hz | 客户端 100 Hz | 100 Hz 变化 |
| --- | ---: | ---: | ---: |
| 唯一观测样本 | `113` | `114` | 基本相同 |
| 观测单步 p50 | `0.001480 rad` | `0.001334 rad` | 降低约 `9.9%` |
| 观测单步 p95 | `0.003396 rad` | `0.002989 rad` | 降低约 `12.0%` |
| 观测单步最大值 | `0.004822 rad` | `0.003814 rad` | 降低约 `20.9%` |
| 速度 p95 | `0.047453 rad/s` | `0.040497 rad/s` | 降低约 `14.7%` |
| 速度最大值 | `0.065403 rad/s` | `0.051861 rad/s` | 降低约 `20.7%` |
| 加速度 p95 | `0.316291 rad/s^2` | `0.186343 rad/s^2` | 降低约 `41.1%` |
| 加速度最大值 | `0.540110 rad/s^2` | `0.349729 rad/s^2` | 降低约 `35.2%` |
| 表观跟踪误差 RMS | `0.006767 rad` | `0.004301 rad` | 降低约 `36.4%` |
| 表观跟踪误差最大值 | `0.017042 rad` | `0.009138 rad` | 降低约 `46.4%` |
| 最终回起点误差 | `+0.002368 rad` | `-0.001661 rad` | 绝对值降低约 `29.9%` |

15 Hz 的实际观测范围为 `[-0.042227, +0.041420] rad`，总跨度 `0.083647 rad`；100 Hz 为
`[-0.038926, +0.041978] rad`，总跨度 `0.080904 rad`，更接近理论 `0.08 rad`。加速度由约 11 Hz
相机观测差分得到，绝对值包含编码器和采样噪声，但两组采样数和轨迹一致，因此相对比较有意义。

这项动态对照确认：即使数学目标本身连续且严格限制速度/加速度，15 Hz 客户端目标更新仍会造成更大的
阶梯、加速度峰值和跟踪误差。将客户端目标更新提高到 100 Hz 有明确改善。因此低频离散目标不仅影响
静态重复发布，也是动态不平滑的一个已证实因素；但它仍不能解释全部 rollout 抖动，因为本测试没有
模型输出和 action chunk 替换。

## 真实冻结 Policy Chunk A/B

使用 version 2 JSON 保存同一次真实 policy 推理的原始节点和 `±0.03 rad` 有界回放节点。两次执行
使用相同锚点、相同10步节点和相同15 Hz模型时间轴，均禁止重规划、固定夹爪；唯一变量是直接15 Hz
节点发送或100 Hz线性插值。

该chunk的离线节点统计：

| 计划 | 最大速度 | 最大加速度 |
| --- | ---: | ---: |
| 原始 policy | `0.197561 rad/s` | `1.017444 rad/s^2` |
| `±0.03 rad` 有界回放 | `0.128638 rad/s` | `1.873880 rad/s^2` |

在140个手臂节点值中有24个被裁剪。原始policy最大加速度约为官方Home限制 `0.5 rad/s^2` 的2倍；
逐点硬裁剪虽然降低了最大速度，却把离散最大加速度提高到 `1.873880 rad/s^2`，说明硬裁剪本身制造了
额外轨迹折角。

真机A/B结果：

| 指标 | 15 Hz离散 | 100 Hz插值 | 100 Hz变化 |
| --- | ---: | ---: | ---: |
| 唯一观测样本 | `10` | `12` | 接近 |
| 观测单步p95 | `0.003532 rad` | `0.002625 rad` | 降低约 `25.7%` |
| 观测单步最大值 | `0.006339 rad` | `0.004633 rad` | 降低约 `26.9%` |
| 速度p95 | `0.046199 rad/s` | `0.041133 rad/s` | 降低约 `11.0%` |
| 速度最大值 | `0.090155 rad/s` | `0.073467 rad/s` | 降低约 `18.5%` |
| 加速度p95 | `0.426862 rad/s^2` | `0.302277 rad/s^2` | 降低约 `29.2%` |
| 加速度最大值 | `0.769023 rad/s^2` | `0.662466 rad/s^2` | 降低约 `13.9%` |
| 表观跟踪误差RMS | `0.011789 rad` | `0.008208 rad` | 降低约 `30.4%` |
| 表观跟踪误差最大值 | `0.028670 rad` | `0.023925 rad` | 降低约 `16.6%` |
| 自动回锚最大误差 | `0.002485 rad` | `0.002180 rad` | 降低约 `12.3%` |

两次最大跟踪误差都位于 `Joint6_R`。100 Hz插值对所有指标均有改善，严格确认15 Hz阶梯发送是一个
因果因素；但插值后最大观测加速度仍高于官方Home限制，且跟踪误差仍明显，说明只提高频率不能完整
解决问题。

随后绕过 `±0.03 rad` 诊断裁剪，直接回放相同JSON中的原始policy节点。原始计划相对锚点最大行程
`0.095436 rad`、最大相邻差 `0.013171 rad`，位于bridge的 `0.12 rad` 实时拒绝包络和URDF硬限位内。
24个裁剪值实际影响后7/10个节点，集中在右臂五个关节。

| 指标 | raw 15 Hz离散 | raw 100 Hz插值 | 变化 |
| --- | ---: | ---: | ---: |
| 观测单步p95 | `0.004223 rad` | `0.004060 rad` | 降低约 `3.9%` |
| 观测单步最大值 | `0.007870 rad` | `0.007912 rad` | 基本不变 |
| 速度p95 | `0.063115 rad/s` | `0.067246 rad/s` | 增加约 `6.5%` |
| 速度最大值 | `0.128952 rad/s` | `0.155151 rad/s` | 增加约 `20.3%` |
| 加速度p95 | `0.430085 rad/s^2` | `0.400496 rad/s^2` | 降低约 `6.9%` |
| 加速度最大值 | `0.749478 rad/s^2` | `0.758374 rad/s^2` | 基本不变 |
| 表观跟踪误差RMS | `0.018937 rad` | `0.012923 rad` | 降低约 `31.8%` |
| 表观跟踪误差最大值 | `0.069384 rad` | `0.050226 rad` | 降低约 `27.6%` |
| 自动回锚最大误差 | `0.003479 rad` | `0.002678 rad` | 降低约 `23.0%` |

raw两组最大表观误差都转移到 `Joint4_R`。插值显著改善跟踪误差，却没有降低速度/加速度峰值，说明
原始计划的时间尺度和右臂跟踪滞后比 `±0.03 rad` 裁剪更值得怀疑。报告中的表观误差把相机采样延迟
包含在内，不能视为低层伺服器的精确误差。

2026-08-07 又保持相同raw节点与100 Hz插值不变，只用 `--playback-time-scale 2.0` 将总时长从
`0.667 s` 拉长至 `1.333 s`：

| 指标 | raw 100 Hz 1倍速 | raw 100 Hz 2倍速 | 2倍速变化 |
| --- | ---: | ---: | ---: |
| 唯一观测样本 | `13` | `22` | 时长增加后的预期变化 |
| 观测单步p95 | `0.004060 rad` | `0.003677 rad` | 降低约 `9.4%` |
| 观测单步最大值 | `0.007912 rad` | `0.005869 rad` | 降低约 `25.8%` |
| 速度p95 | `0.067246 rad/s` | `0.060283 rad/s` | 降低约 `10.4%` |
| 速度最大值 | `0.155151 rad/s` | `0.114097 rad/s` | 降低约 `26.5%` |
| 加速度p95 | `0.400496 rad/s^2` | `0.197141 rad/s^2` | 降低约 `50.8%` |
| 加速度最大值 | `0.758374 rad/s^2` | `0.557468 rad/s^2` | 降低约 `26.5%` |
| 表观跟踪误差RMS | `0.012923 rad` | `0.009011 rad` | 降低约 `30.3%` |
| 表观跟踪误差最大值 | `0.050226 rad (Joint4_R)` | `0.027845 rad (Joint4_R)` | 降低约 `44.6%` |
| 自动回锚最大误差 | `0.002678 rad` | `0.002506 rad` | 降低约 `6.4%` |

操作员主观反馈为颤抖“减轻较明显”。在相同路径、节点和发送频率下，仅放慢时间便使所有主要指标
同方向改善，因此“原始时间尺度下机器人持续追赶目标”已有真机证据，应列为主要因素；但2倍速仍有
`0.027845 rad` 的 `Joint4_R` 最大表观误差，不能把时间尺度视为唯一原因。

结合此前重规划边界离线诊断，当前已确认三个相互叠加的项目侧原因：

1. 15 Hz离散绝对目标形成阶梯输入。
2. 模型原始chunk的离散加速度偏高。
3. 逐点硬裁剪会引入更大的二阶不连续；持续rollout还额外存在约2至3步推理延迟造成的chunk边界错位。

官方低层控制不是当前首要嫌疑：恒定保持、确定性轨迹和同chunk的100 Hz版本都能在同一官方链路上
取得一致改善。

## 恒定姿态测试命令

先在 Apex 保持 Input Mode None，并启动 bridge：

```bash
cd /home/jh/TianJi_Marvinpro/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion --publish-hz 100
```

另一个终端运行：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.hold_test_client \
  --robot-host 6.6.7.100 \
  --duration 10
```

客户端等待时将 Apex 切到 Custom，确认屏幕显示 `input_mode=3`、两个状态数组均为 `(3, 3)`，再输入
`HOLD`。测试结束后必须先在 Apex 切回 None。异常时优先切 None 或急停，不把 `Ctrl+C` 作为正常
停机步骤。

## 2026-08-07 测试顺序

### 1. 已完成：原始chunk的2倍慢速对照

测试问题：在节点、路径、100 Hz插值、bridge和官方控制链全部不变时，只把轨迹时间拉长2倍，右臂
跟踪和主观平滑度是否明显改善？这是当前用于验证“机器人执行跟不上计划时间轴”的最小变量实验。

前置条件：bridge继续以 `--allow-motion --publish-hz 100` 运行；Apex先保持Input Mode None；机器人
回到与保存计划相符的锚点附近，客户端会检查最大姿态漂移 `0.01 rad`。

```bash
cd /home/jh/OpenPI_UR/openpi

PYTHONPATH=/home/jh/TianJi_Marvinpro/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /home/jh/TianJi_Marvinpro/MarvinPro_deploy/artifacts/marvinpro_red_cones_chunk_ab_v2.json \
  --target-source raw \
  --playback-mode interpolated \
  --playback-time-scale 2.0
```

确认页应显示：

- `target source: raw`
- `playback time scale: 2.00x`
- `effective knot rate: 7.50Hz`
- `playback duration: 1.333s`
- `selected effective max velocity: 0.09878rad/s`
- `selected effective max acceleration: 0.25436rad/s^2`

输入 `INTERPOLATE` 后观察，等待自动回锚完成再切回None。需要保存完整报告，同时人工记录：是否仍有
逐点颤抖、主要发生在哪条手臂/哪个阶段、相对今晚1倍速是否明显改善。

对照基线是今晚的raw 100 Hz、1倍速结果：

| 指标 | 1倍速基线 | 2倍速结果 |
| --- | ---: | ---: |
| 唯一观测样本 | `13` | `22` |
| 观测单步p95 | `0.004060 rad` | `0.003677 rad` |
| 观测单步最大值 | `0.007912 rad` | `0.005869 rad` |
| 速度p95 | `0.067246 rad/s` | `0.060283 rad/s` |
| 速度最大值 | `0.155151 rad/s` | `0.114097 rad/s` |
| 加速度p95 | `0.400496 rad/s^2` | `0.197141 rad/s^2` |
| 加速度最大值 | `0.758374 rad/s^2` | `0.557468 rad/s^2` |
| 表观跟踪误差RMS | `0.012923 rad` | `0.009011 rad` |
| 表观跟踪误差最大值 | `0.050226 rad (Joint4_R)` | `0.027845 rad (Joint4_R)` |
| 自动回锚最大误差 | `0.002678 rad` | `0.002506 rad` |

结论：主观颤抖、RMS和 `Joint4_R` 最大误差均明显下降，原始时间尺度下的跟踪滞后得到支持。

### 2. 第二优先级：持续rollout重规划边界

第一项完成前不要同时修改轨迹形状和重规划逻辑。之后先增加只读日志，不立即改变真机动作：

1. 每次推理记录观测年龄、端到端推理耗时及其折算的15 Hz节点数。
2. 记录被替换时旧计划剩余节点、上一已发送目标，以及 `new[0..4]` 分别与当前目标的边界差。
3. 记录每次 `action plan empty`、measured-pose hold持续时间和新计划恢复时的跳变量。
4. 记录机器人反馈与上一实际发送目标的误差，避免再把“即将发送的目标”当作同步目标。

2026-08-07 已实现上述只读日志且未改变动作行为。4秒静止dry-run结果：冷启动warmup耗时
`16.27 s` 并被正确丢弃；随后15次推理为 `136.9-168.0 ms`，对应 `2.05-2.52` 个15 Hz动作周期，
观测在推理返回时前进2至3帧。除第一次外，每次替换都丢弃旧计划1个剩余节点；稳态未发生underrun。
大量裁剪警告仅涉及夹爪维度 `7/15`，未发现手臂维度裁剪。dry-run中的 `new_to_last` 是未执行动作的
虚拟队列差，不能用于判断真机边界跳变。

随后完成3秒短时真机rollout，共12次推理。运动阶段统计：

| 指标 | 结果 |
| --- | ---: |
| 推理延迟 median / p95 / max | `159.5 / 181.8 / 191.4 ms` |
| 等效15 Hz周期 median / p95 / max | `2.39 / 2.73 / 2.87` |
| 推理返回观测帧差 | 7次为2帧，5次为3帧 |
| 稳态每次替换丢弃的旧节点 | 全部为1个 |
| 稳态underrun | `0/11` 次重规划 |
| 手臂裁剪 | `0/48` action ticks |
| 夹爪裁剪 | `45/48` action ticks |
| 旧计划下一点到上一目标 median / p95 / max | `0.00406 / 0.00834 / 0.01116 rad` |
| `new[0]` 到上一目标 median / p95 / max | `0.01719 / 0.04569 / 0.05242 rad` |
| `new[0]` 到机器人反馈 median / p95 / max | `0.00772 / 0.01431 / 0.01487 rad` |

关键样本为inference 8：旧计划下一点距离上一目标仅 `0.01116 rad`；新 `new[0]` 距上一目标
`0.05242 rad`，但距机器人反馈仅 `0.00420 rad`。11次稳态重规划中，`new[0]` 有9次是 `new[0..4]`
里最接近反馈的节点。这说明客户端按墙钟执行的旧目标已经跑在真机前面；新policy看到较落后的真实
机器人后，又从反馈附近生成新计划，硬替换使目标反向拉回，形成持续“追赶 -> 回拉”的振荡。

不能简单用推理延迟跳到 `new[2]`：真机统计中 `new[2]` 到反馈的median和max分别为 `0.01235`、
`0.04243 rad`，均大于 `new[0]` 的 `0.00772`、`0.01487 rad`。仅做固定延迟索引补偿可能让跟踪更差；
修复需要同时处理执行时间尺度、反馈滞后和新旧边界连续性。

OpenPI仓库的 `docs/remote_inference.md` 建议每N步调用一次policy，并在其余步骤open-loop消费预测chunk；
`ActionChunkBroker` 和LIBERO示例也都是当前选定chunk耗尽后才重新推理。当前 `prefetch_steps=3` 的异步
执行中替换是本项目自定义行为，不是这些示例的默认消费方式。

`--prefetch-steps 0` 单变量A/B也已完成：共7次推理，每次先完整执行5个节点，再在下一次推理的
`146.0-247.6 ms` 内发送测量姿态hold；所有重规划的 `discarded=0`，每段有2至3个预期的hold tick，
手臂裁剪为 `0/50`，夹爪裁剪为 `34/50`。操作员主观反馈仍是“连续颤抖”，没有因消除旧/new硬替换
而消失。因此异步硬替换可以造成个别较大边界跳变，但不是持续颤抖的必要条件；同步等待推理还会引入
周期性的“运动 -> hold -> 运动”，不适合作为最终调度方案。

当前下一项应把已在冻结chunk中验证有效的两项同时带入持续rollout的受控诊断模式：模型节点间以
100 Hz插值，并用2倍时间执行（有效节点频率7.5 Hz）。先保持每个chunk完整消费，明确验证连续颤抖
是否下降；之后再单独处理新旧边界连续性。不要继续把固定 `new[k]` 索引当作第一修复。

2026-08-07 已实现并完成该诊断模式的现场真机对照。启用参数为
`--playback-mode interpolated --control-hz 100 --model-hz 15 --playback-time-scale 2`。本项测试应显式
使用 `--execute-steps 10` 完整消费模型返回的10个节点，每段含“上一段末值到新 `action[0]`”共10个
节点间隔，持续 `1.333 s`；在剩余
`0.30 s` 时推理下一段并追加到队尾，不在段中覆盖旧计划。默认离散模式未改变。

最终10节点配置连接真实 bridge 与远程 policy 的只发推理、不发 `ActionCommand` 4秒dry-run已通过：

- 4次持续推理，端到端耗时 `153.4-172.4 ms`，每次生成134个100 Hz目标。
- 首段之后的 `queued_before` 依次为 `13/14/15`，所有稳态 `underruns_since_last=0`。
- 387个虚拟动作tick中手臂裁剪为0；夹爪输出387次均被限制到 `[0,1]`，与此前模型夹爪诊断一致。
- 初始15个空tick只发生在第一次推理返回前；真机执行时发布线程在这个阶段发送实时测量姿态hold。
- 推理返回时相机观测均前进2帧，符合此前网络延迟数据。

此前5节点调度也做过2秒和4秒dry-run，但不是本项最终真机参数。默认离散路径另做1秒真实全链路回归，
正常完成4次推理和14个虚拟动作tick，手臂裁剪为0。DEBUG模式现在只显示本客户端诊断，
OpenPI/WebSocket逐帧日志已被抑制。

新模式的长轨迹节点不能持续携带policy推理开始时的旧相机序号，否则bridge的8帧时效门控会在后半段
拒绝命令。发布线程仍保留policy源序号用于诊断，但发送时改用“当前目标刚完成本地反馈限幅检查”所对应
的最新观测序号；bridge仍会再次按最新关节反馈执行 `0.12 rad` 包络检查，没有放宽安全门控。

#### 10节点持续rollout真机结果：失败，禁止重复

5秒真机测试的主观反馈是“振动明显更剧烈，有周期回弹；移动一步后会弹到约一秒前的位置”。该现象与
1.333秒的完整chunk周期吻合，日志给出了直接证据：

| 推理 | wall | 到达时旧队列 | 空窗 | 旧raw尾部到实际上一目标 | 新`action[0]`到旧raw尾部 | 新`action[0]`到反馈 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | `166.0 ms` | 0 | 初始hold 16 tick | `0` | `0.02097 rad` | `0.02097 rad` |
| 2 | `289.2 ms` | 2 | 0 | `0.07926 rad` | `0.16913 rad` | `0.01041 rad` |
| 3 | `386.6 ms` | 0 | 8 tick | `0`（已切反馈hold） | `0.09789 rad` | `0.09789 rad` |
| 4 | `167.7 ms` | 14 | 0 | `0.01983 rad` | `0.04582 rad` | `0.01360 rad` |

第2段是周期回弹的决定性样本：机器人没有达到旧计划末端，旧raw队列尾部已比最后实际发送目标超前
`0.07926 rad`；此时新policy根据真实反馈生成的 `action[0]` 距反馈仅 `0.01041 rad`，但它相对旧raw
尾部反向相差 `0.16913 rad`。客户端先坚持消费旧raw尾部，再用一个7.5 Hz节点间隔插值回新目标，因而
形成“向前追赶 -> 拉回约一秒前反馈位置”的周期回弹。完整消费10节点的open-loop假设在当前真机跟踪
速度下不成立。

运动阶段共有 `531` 个发布tick，其中 `77` 个发生手臂裁剪；主要动作维度为8和11，分别对应
`Joint1_R`、`Joint4_R`。第一次手臂裁剪约在首段开始后0.73秒出现，之后连续约0.65秒；这说明计划目标
已经超出机器人反馈 `0.08 rad`，安全滤波器在持续饱和追赶，而不是偶发裁剪。夹爪裁剪为473 tick。

运动时网络/处理延迟也比静止dry-run更差：第2、3次推理达到 `289.2/386.6 ms`，后者超过0.30秒预取
窗口并导致8个 measured-pose hold tick。简单增大预取窗口不是解决办法，因为更早观测会让新policy
结果在chunk边界时更加陈旧。

结论：100 Hz插值和2倍慢放本身没有导致该回弹；错误在于把下一段锚定到“未实际实现的旧raw计划
尾部”，并强制完整执行过长open-loop chunk。下一项应缩短选定动作段，并在新推理返回时丢弃旧的未来
目标，从最后实际发送目标或当前反馈开始连续衔接；在实现前不要再运行本节10节点真机参数。还应增加
跟踪误差/连续手臂裁剪保护，防止计划时间轴再次持续跑在机器人前面。

最终汇总中的 `shutdown_underruns=401` 发生在3秒episode结束后、操作员切回None之前的测量姿态保持
阶段，不属于运动过程，不能作为颤抖原因。该测试排除了稳态plan underrun和手臂安全裁剪是主要原因。

每项实现都必须能通过命令行开关恢复旧行为，并先做dry-run边界统计，再做短时真机测试。

### 3. 后续修复顺序

若前两项证实时间尺度和重规划均有贡献，实施顺序为：

1. 保持模型15 Hz语义，在节点之间输出100 Hz连续目标。
2. 用实际已发送目标/反馈作为重规划锚点，禁止用未实现的raw队列尾部衔接。
3. 缩短open-loop段，并在跟踪误差过大时冻结计划时间轴，而不是持续裁剪后继续消费未来节点。
4. 消除plan underrun造成的“测量姿态hold -> 新计划”切换。
5. 最后才引入速度、加速度和jerk受限整形；不要再使用逐点硬裁剪作为轨迹整形器。

暂时不要修改官方控制器参数，也不要把提高bridge频率单独解释成完整修复。恒定保持已经证明官方链路
能够稳定保持，冻结chunk则证明100 Hz插值有效但不足。

### 4. 严格同步chunk基线：周期回弹已消失

按操作员决定，已新增 `--rollout-schedule synchronized`，用于验证周期回弹是否由提前观测和异步chunk
衔接造成。该模式与上面禁止重复的 `prefetch` 10节点实验不同，其状态顺序固定为：

```text
执行完整chunk -> 锁存末目标 -> 跟踪到位 -> 稳定保持 -> 新观测 -> 推理 -> 下一chunk
```

实现要点：

- 当前chunk的全部100 Hz插值目标实际发送完毕后，才进入跟踪检查。
- 空队列期间锁存模型末目标，不再切换成随反馈变化的measured-pose hold；否则机器人可能在到位前被停住。
- 所有14个臂关节默认需进入 `0.01 rad` 并持续 `0.20 s`，随后额外稳定保持 `0.20 s`。
- 保持完成后强制等待一帧更新的相机/关节观测，再调用远程policy；推理期间仍锁存末目标。
- 跟踪或保持任一阶段超过 `5 s` 会停止rollout，不自动放宽阈值。夹爪不参与到位判据。
- episode截止只禁止启动下一chunk；已启动chunk会执行、跟踪、保持和重观测完毕，因此墙钟时间可超出配置。
- 原 `prefetch` 路径作为默认行为保留，便于后续A/B；同步模式只允许与 `interpolated` 和 `--execute`
  配合使用。

首轮现场参数保持模型节点15 Hz语义、100 Hz线性插值、2倍时间和完整10节点，配置episode 6秒。操作员
主观确认：“回弹消失，确实变平滑了”。三段结果如下：

| chunk | 推理时间 | 首节点边界差 | 发送后到位时间 | 到位最终误差 |
| --- | ---: | ---: | ---: | ---: |
| 1 | `195.0 ms` | `0.02098 rad` | `1.66 s` | `0.00388 rad (Joint4_R)` |
| 2 | `202.5 ms` | `0.03062 rad` | `0.42 s` | `0.00448 rad (Joint1_R)` |
| 3 | `201.8 ms` | `0.01452 rad` | `0.22 s` | `0.00284 rad (Joint1_R)` |

总计781个action tick，其中165个发生手臂包络裁剪。该数量与第一段发送完成后1.66秒的追踪期接近，且
现场未再观察到周期回弹；它说明第一段末目标明显超前于反馈，需要后续单独处理，但不是本次结束漂移的
直接原因。

本次同时发现一个独立的结束保持bug：客户端提示切换None期间，机械臂沿固定方向持续漂移。根因是结束
路径原本每个100 Hz tick都读取最新反馈并将其作为新hold目标；残余运动会随新观测被逐帧锁存，形成
棘轮式漂移。现已改为结束瞬间只采样一次16维测量姿态，之后持续发布同一个固定目标，并禁止已经从队列
弹出但尚未完成记账的旧动作覆盖该shutdown latch。发布线程假连接测试已覆盖“反馈从0变化到0.01 rad，
固定hold目标仍保持0”这一回归。

结束修复随后完成3秒2.0x真机回归，操作员确认等待切换None期间“没有漂移”。该轮只执行一个chunk：
推理 `178.6 ms`、首节点边界差 `0.02627 rad`、发送结束后 `1.74 s` 到位、最终误差
`0.00268 rad (Joint4_R)`，该chunk手臂裁剪164 tick。此前同配置第一段为165 tick，说明首段末目标
长期超前反馈是可重复现象，不是偶发网络抖动。结束固定hold bug至此关闭；下一步只运行一个chunk的
1.5x对照，并比较到位时间、主观平滑度及每chunk裁剪tick。若运动明显变急、回弹恢复或tracking timeout，
立即切None，不继续1.0x。

### 旧计划文件说明

`/tmp/marvinpro_red_cones_chunk_ab.json` 是version 1旧文件，其有界计划裁剪23/140个手臂节点值，
最大速度 `0.181587 rad/s`、最大加速度 `2.423854 rad/s^2`，超过诊断安全上限，客户端会拒绝加载。
明日只使用仓库 `artifacts/` 下SHA-256已记录的version 2文件。

## 已完成验证

- `ruff check src tests`：通过。
- `python3 -m unittest discover -s tests -v`：23 项通过。
- `python3 -m py_compile src/marvinpro_deploy/frozen_chunk_test_client.py`：通过。
- `--playback-time-scale 2.0` 参数解析通过；小于 `1.0` 的加速请求会在连接机器人前拒绝。
- 固化后的version 2计划通过JSON解析，大小 `9169 bytes`，且SHA-256与今晚实际执行的 `/tmp` 文件一致。
- 本地假 bridge 集成测试：收到 13 条恒定目标命令，16 维目标逐值完全一致；Input Mode 离开 Custom
  后客户端自动停止并输出统计。
- 本地假 bridge 轨迹测试：收到 41 条命令，只有指定关节发生变化，覆盖预期正负幅度并严格返回起点。
- 本地冻结 chunk A/B 集成测试：离散阶段只发送保存的模型节点，插值阶段发送节点间目标；两次使用
  同一个 JSON 计划并自动返回相同锚点。
- raw源假bridge集成测试确认客户端选择未裁剪节点，同时仍执行硬限位、bridge包络和动态上限检查，
  并在结束后返回锚点。
- rollout交互假连接测试确认默认输出包含明确的Custom/None切换提示；进入结束阶段会先锁存一次固定
  测量姿态、关闭空计划告警并清空计划，检测到Input Mode为None后才允许客户端断开。
- 同步调度实现通过Python编译和ruff检查；参数测试覆盖同步模式必须启用真机执行与插值回放，发布线程
  测试确认空队列时锁存末目标、结束时固定hold不跟随变化的反馈。完整测试套件当前为26项通过。同步
  模式2.0x真机测试已消除周期回弹；结束固定hold修复的真机复测确认无漂移。

`--playback-time-scale 2.0`、带新只读日志的3秒短时真机rollout和 `--prefetch-steps 0` 对照均已于
2026-08-07完成。rollout默认交互已改为明确提示切换Custom/None；逐次推理诊断需用
`--log-level DEBUG` 查看。持续rollout的100 Hz插值/2倍时间/完整10节点模式已经完成真机测试，并因
周期回弹、77个手臂裁剪tick和一次8 tick空窗判定失败，禁止重复。上述日志、交互代码与本次记录仍未提交。
