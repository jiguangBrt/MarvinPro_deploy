# Marvin Pro OpenPI 真机部署交接

更新时间：2026-08-28

本文件是 Marvin Pro 真机部署的唯一入口文档。文档结构：`README.md` 为仓库索引；本文为当前生产
配置、真机快速开始、状态快照和待办与已知问题；[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 为
RTC/轨迹测试计划、诊断工具和带日期的测试记录；[`BASELINE_RUN.md`](BASELINE_RUN.md) 为基线与
真机 A/B 记录。

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
  --playback-time-scale 3`（名义 5 Hz knot rate）或 2026-08-28 起放开的 `--playback-time-scale 1`
  （15 Hz 原速），其他倍率会被客户端拒绝。
- tracking governor：`error <= 0.02 rad` 时 phase rate 为 1；`0.02..0.16 rad` 按
  `(0.16-error)/0.14` 线性降低；`>= 0.16 rad` 硬冻结，降到 `<= 0.12 rad` 才解除锁存；joint state
  stale、timer overrun 和 arm clipping 直接硬冻结。臂关节 safety clipping 包络 `0.16 rad`。
- RTC A9 checkpoint 只有在全部 14 个臂关节误差不超过 `0.01 rad`，并且由持续更新的 joint source
  timestamp 证明连续稳定 `0.20 s` 后才成立；“客户端已经发出 A9，但反馈仍在 A8”不会触发新观测。
- `d_pred` 使用当前 estimator epoch 内可行 latency 样本的保守 p95 加 `50 ms` guard。2026-08-28
  18:30 起样本口径改为 client 观测的完整请求延迟（checkpoint 事件接收到 merge/丢弃结果，含相机
  等新图像、观测准备、推理传输、merge staging）；warmup/初始推理/recovery bootstrap 等无
  checkpoint 上下文的位置记录 wall + 开销 EMA（种子 `100 ms`，每次成功 merge 按
  `full-wall` 更新）。物理超限判定不再含 guard：单样本超过 4 个 old-tail knot 才记 link fault；
  预测超协议上限 4 时钳位到 4 并按 epoch 告警（`rtc_delay_prediction_clamped`），不再抛错——
  bridge 仍自行强制物理边界并丢弃迟到 merge。默认 `--rtc-late-result-policy discard`；`d_actual`
  始终按 bridge 实际 phase 跨过的 knot 计数，不使用 `wall_time * nominal_rate`。旧口径只统计推理
  wall，15 Hz 下系统性低估约 1 个 knot，recovery 重建 epoch 后 d_pred 掉到 3 会造成每次 merge
  都在边界外（2026-08-28 18:07 运行三次 rtc_late 中止的根因）。
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
- **15 Hz 原速 RTC 已放开并完成三轮真机运行**：`--playback-time-scale` 白名单放开 `1.0`；
  blend 窗口按秒恒定（0.6 s，15 Hz 候选 9/6/3/2 knot）；blend 包络由
  `stack_cones_slow_260826` 遥操作数据标定为 `1.7 / 18.0 / 380`。运行记录：11:44
  `--max-rtc-merges 1` clean completion（1 merge）；11:47 3.0x 慢放 60 s clean completion
  （30 merges / 0 recoveries）；13:35 15 Hz 首跑因旧 3/2-knot 窗口 blend jerk 355 超限
  `stuck_exhausted`（促成窗口与包络改造）；14:30 15 Hz 重跑 17 merges / 0 recoveries、
  blend jerk 实测最大 ~105，但 12.5 s 后控制器 `robot_state` 掉到 `(1,12)` 中止（见下方
  待办第一条，与本仓库代码无关）。
- **18:01/18:07 真机运行与估算口径修复**：18:01 60 s 15 Hz 运行 clean completion
  （`logs/rtc_20260828_180126`，87 merges / 1 recovery，全程 `(3,3)`，`（1,12)` 未复现）。
  18:07 600 s 运行（`logs/rtc_20260828_180703`）47 s 后 `fatal_safety_hold`：三次 `rtc_late`
  全部来自 d_pred 估算口径缺陷（只统计推理 wall，漏算相机等待/观测准备/merge staging 约
  1 个 knot；recovery 重建 epoch 后 d_pred=3 低于物理延迟，merge 系统性越界），非机器人侧
  故障；第三次 recovery 时 hold 锁存命令与 bridge invalidation 撞车被拒，client 升级为
  fatal（bridge 实际已自行安全 hold；该竞态已于 2026-08-31 修复，见下方待办区
  recovery 竞态条目，commit bc18107）。已修复估算口径（见上文 d_pred 条目），
  117 tests passed；用两次运行日志回放到新估算器验证：15 Hz 全程 d_pred=4，5 Hz 为
  2~3（旧口径 1~2，更保守但仍在协议范围内）。改动仅在 client 侧（`rtc.py` +
  `rollout_client.py`），bridge 不 import `rtc.py`，不强制重启。
- **policy server 运维**：服务器进程每次重启后必须先在本地跑
  `cd /home/jh/OpenPI_UR/openpi && uv run python /tmp/policy_warmup.py`（双路径 JIT 预热，
  稳态 plain ~140 ms / RTC ~170 ms）；服务器侧用 tmux 会话 `policy_redcones` 运行
  （`ssh 192.168.50.73 -t 'tmux attach -t policy_redcones'` 接管），日志在服务器
  `/tmp/serve_policy_8000.log`。**不要用 `policy` 这个会话名**：2026-08-28 下午同事在同一
  服务器用 `policy`（端口 8002，pour_bowl `pi05_sft_r3`）和 `policy_cfg`（端口 8003，
  `serve_cfg_shim.py` CFG 实验）跑倒碗任务，15:14 重建 `policy` 会话时把 8000 红锥服务
  进程一并挤掉，导致客户端 `Timed out connecting to policy server`；17:57 已用独立会话名
  `policy_redcones` 重启并预热（plain 稳态 ~119 ms / RTC ~142 ms）。
- **18:35 运行暴露的两个 recovery 问题已修（2026-08-31）**：`logs/rtc_20260828_183559`
  中 recovery 机制本身工作正常（4 次成功回到 RTC），但暴露：① recovery 后 bridge 补发
  属于旧 request 的 deadline 事件，client 误判 `rtc_fatal`（`bridge deadline event belongs
  to another request`）——已改为只告警并忽略（旧 request 已被 recovery 握手取代，事件不
  影响在飞的 request）；② `--max-rtc-recoveries` 默认 3，耗尽后永久切到 synchronized
  fallback（非 RTC 精度不可用）——长跑命令改用 `--max-rtc-recoveries 20`。仅 client 侧
  `rollout_client.py`，bridge 不用重启；117 tests passed。注意：单次 recovery 中间仍会
  执行一段 synchronized 过渡 chunk（无延迟补偿，精度差属固有），跑完一个 clean chunk 即
  自动回 RTC。
- **代码已同步 GitHub**：`tmp` 与 `main` 均在 `1099ccd`
  （`git@github.com:jiguangBrt/MarvinPro_deploy`）。
- **下一步**：按 `cmd_tmp.md` 2026-08-31 09:47 段命令重跑 15 Hz 原速 600 s RTC
  （日志目录名带任务描述 `rtc_redcones_600s_rec20_<时间>`；client 代码已变，bridge 可
  沿用）；观察长时间运行下 recovery 频率、`ignoring stale bridge event` 次数、叠放精度，
  以及 `(1,12)` 是否在抓取/搬运阶段复现。

## 待办与已知问题

### 已知性能问题与优化方向（2026-08-17 调研）

针对当前 H20/s10、5 Hz、`d_max=4`、指数 soft mask 基线。评估分安全性、运动连续性、策略响应性、
任务能力四个维度，不能合并成单一“好坏”。

1. **默认 settled checkpoint（P0：continuous 作为正式运行语义）**：settled checkpoint 要求全部 14
   关节 `<=0.01 rad` 且稳定 `0.20 s`。真机五 chunk 记录：每次 checkpoint 明显停顿（首次
   `1.291 s`，后续 `0.368/0.471/0.432 s`；bridge 平均 `99.53 Hz` 不足以解释，根因是 checkpoint
   状态机的固定等待）。方向：continuous RTC 作为正式运行语义，settled 仅用于诊断；观测等待期间
   跨过的 knot 计入 `d_actual`；不要靠把 `0.20 s` 改成 `0.05 s` 掩盖模式问题。
2. **Tracking governor 改变模型时间语义（P1：找固定可跟踪 knot rate）**：governor 阈值
   `run=0.02 / resume=0.12 / stop=0.16 rad`，叠加在已慢放到 5 Hz 的 knot rate 上（time scale 3 且
   `phase_rate=0.2` 时瞬时有效执行速度只有 `1 Hz`），操作者感知为迟滞。历史 synchronized 失败
   运行出现 `0.27883 rad` peak tracking error 和 228 个 arm clipped tick，说明 governor 不是冗余
   功能。方向：解耦“正常运行速度”和“异常安全冻结”；用 frozen chunk/固定计划测试 5、7.5、10、
   15 Hz 并记录 tracking p50/p95/max、clipping 和操作者感知；governor 只按 14 个臂关节误差。
3. **`d_pred`、deadline freeze 与短 horizon（P0：治理延迟尖峰）**：
   `d_pred = ceil((p95(最近20次稳定wall) + 0.05 s) * effective_knot_hz)`，限 `1..4`。
   历史事故（2026-08-12）：单次 wall 尖峰 `1560.1 ms` 的结果以 `d_actual=3` merge 后，下一次
   `predicted_steps()` 得到 9 超界，RTC 对该 episode 永久关闭。当前 v10 已把超 horizon 样本标为
   link fault、不写入稳定分布，并在恢复前重建 estimator epoch；仍需真机验证不产生恢复振荡。
   方向：分解并治理 transport/wall-latency 尖峰；区分稳定预算与异常尖峰；保持 `d_actual` 物理计数。
4. **Hard anchor 与执行端 C2 blend（P1：导数连续准入）**：protocol v10 从旧轨迹真实 `q/v/a` 优先
   生成 quintic C2 blend。2026-08-28 起 blend 窗口按**秒**恒定（目标 `0.6 s`，knot 数随 knot rate
   缩放：5 Hz 候选 `(3, 2)`；15 Hz 候选 `(9, 6, 3, 2)`，并按剩余 checkpoint 距离裁剪），blend
   上限为按 knot rate 查表的显式包络 `BLEND_CAPS_BY_KNOT_HZ`（CLI `--rtc-blend-max-*` 可整体
   覆盖）：5 Hz 为运行验证过的 `0.45 / 2.0 / 40`；15 Hz 由 `stack_cones_slow_260826` 遥操作数据
   （104 集原生 15 Hz：p99.9 = `0.556/2.34/49.8`，示教最大值 = `1.167/14.2/289`）标定为
   `max(3x p99.9, 1.3x max)` = **`1.7 / 18.0 / 380`**，URDF 速度上限仍逐关节取 min。不要用立方
   缩放外推 jerk 上限——窗口按秒恒定后接缝 jerk 只随速度差线性增长，立方外推（1080）会是示教
   最大值的 3.7 倍。全部候选不可行时原子拒绝；未验证的 knot rate 没有包络，直接拒绝。
   历史教训：边界速度跳变 `0.09 rad/knot` 量级时操作员即可感知抽动（对照正常约 `0.014
   rad/knot`）；更长 soft overlap（H20 在 5 Hz 下 `d=2` 后约 8 个 soft transition knot）只能缓解
   生成边界，不能替代执行端 C2 准入和 fallback。`H=50` 属于需要重新确定时间尺度的训练实验
   （原论文 50 点运行在 50 Hz、物理 horizon 约 1 秒），不是部署参数修复。
5. **RTC 失败后的同步回退（P2）**：恢复链为 失效 -> 实测 fixed hold -> 稳定 `0.20 s` -> 新观测 ->
   一个 clean sync chunk -> 重建 estimator epoch -> `s=10` bootstrap -> 回 RTC，每 episode 最多 3
   次，第 4 次留在 timed synchronized。历史上 fallback 后 sync 推理曾达 `1194.7 ms`、源观测落后
   17 帧（限制 8）；当前工作树会对该 observation-lag 情况重新观测和推理。回退结果必须单独计分
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

参考资料：PI RTC 论文 <https://arxiv.org/abs/2506.07339>（真实机器人 `H=50`、`s_min=25`、
`beta=5`、`b=10`、50 Hz）；
[real-time-chunking-kinetix](https://github.com/Physical-Intelligence/real-time-chunking-kinetix)；
[LeRobot RTC 文档](https://huggingface.co/docs/lerobot/en/rtc)；
[PickNik Ruckig stitch](https://docs.picknik.ai/how_to/robotics_applications/stitch_trajectories/)；
[Training-Time Action Conditioning](https://arxiv.org/abs/2512.05964)。

### 实施计划状态（2026-08-10 起）

阶段 0-6 已完成：协议升级到 v10、bridge 本地连续 trajectory、tracking governor、物理 checkpoint、
RTC merge、synchronized episode fallback、OpenPI 原生 JAX RTC sampler（absolute prefix 变换、VJP
guidance、`rtc_v1` envelope、结构化错误和计时）均已实现并通过真机验收。阶段 7（RTC 与 tracking
timeline 集成）的关键项已随 protocol v10 落地（稳定 p95 + 50 ms guard 的 `d_pred`、link fault
隔离、迟到结果默认 discard、fallback 重置 epoch、分段延迟日志、merge 使用 bridge 实际 knot
crossing 的 `d_actual`）。仍需真机证据核对的项目：

- [ ] 推理开始时冻结一份 old remaining reference 和 timeline version 的完整真机证据；
- [ ] 仅当 request/plan/timeline/checkpoint ID、反馈新鲜度和 RTC 约束全部有效时允许 merge 的注入
  测试（延迟突增、响应乱序、重复响应、旧 timeline 响应全部被拒绝）；
- [ ] `d_actual > d_pred`、旧 prefix 耗尽或越过可替换边界时丢弃并 fallback 的真机证据；
- [ ] 推理期间任意 arm clipping、tracking hard freeze 或状态过期使结果失效的真机注入测试；
- [ ] merge 与 100 Hz publisher 并发无半更新状态；plan underrun 时固定 hold；
- [ ] merge 前后 raw reference、sent target、feedback 的位置/速度/加速度边界差的系统记录。

完成判据（全部仍开放）：稳态无 plan underrun；正常阶段无 arm clipping；每次 policy 观测都有
可证明的 tracking checkpoint 和新图像时间屏障；每次 merge 满足 `d_actual <= d_pred <= 4` 和
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
  （历史上曾记录到 `(2,3)`/`(1,3)` 瞬时抖动并自愈，本次未恢复）。重连机器人后先确认状态恢复
  `(3,3)`、Apex 无报警再重跑，复现时记录是否在抓取/搬运阶段。进展：18:01 重跑 60 s 全程
  `(3,3)` 未复现（`logs/rtc_20260828_180126`）；厂家确认状态码含义前保持观察。
- [x] **recovery 竞态（18:07 运行第三次 recovery，2026-08-31 已修复，commit bc18107）**：
  hold 锁存命令被 bridge `trajectory_command_rejected` 拒绝时 client 直接升级
  `fatal_safety_hold`，但 bridge 实际已自行安全 hold（`measured_holding` event）。
  修复采用双保险：bridge 侧锁存命令版本不匹配时若已处于 hold 则幂等确认并补发
  `measured_holding` 事件；client 侧新增 `_latch_measured_hold_with_retry` 重采
  timeline 版本重试一次，`_run_trajectory_schedule` 全部 5 处锁存调用点已切换。
  同一 commit 还修复了 observation-lag 拒绝缺少结构化 reason_code 的问题（bridge
  现在抛出 `ObservationLagError` 并在拒绝事件中携带 `observation_lag`，RTC
  recovery 判为可恢复并重新观测重试）。pytest 121 passed；竞态的端到端有效性
  仍需真机 RTC 长跑确认。
- [ ] 按 [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 执行新 checkpoint 下的 dry-run ->
  synchronized -> RTC shadow -> `--max-rtc-merges 1` 真机验收；merge 数按 1 -> 2 -> 10 逐级放大，
  不直接做 20-merge soak；每次使用独立 RUN_DIR，merge 与 fallback episode 分开统计。
- [ ] RTC 实际 merge 真机运行后，复查堆叠放置数厘米偏差是否为 sync 模式 chunk 间停顿伪影
  （见 [`BASELINE_RUN.md`](BASELINE_RUN.md) A/B 记录）。
- [ ] 离线提取成功 merge 与失败 merge 的旧/新边界，按关节对比位置、速度、加速度和 jerk；评估
  更长但仍受约束的 C2 blend；不得再直接放宽 jerk 上限。
- [ ] H10/H20 对比必须固定机械臂初始姿态、物体布局和两侧夹爪状态，gripper clipping 单独统计。
- [ ] 夹爪左右各自完整、同工况的开合端点标定未完成；旧的 `0.0~1.25 -> 0~1` 映射不能视为已验证。
- [ ] 验证新 estimator epoch 的 `d_pred` 只来自新样本，所有成功 merge 满足
  `1 <= d_actual <= d_pred <= d_max`。
- [ ] knot rate 晋级（`5 -> 7.5 -> 10 -> 15 Hz`、分级 `d_max`，方案见
  [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 第 9 节）尚未实施；任何 `d_max`
  变更都是跨端协议变化，禁止设置 `d_max=10`。

## 历史档案（2026-08-05 至 2026-08-07 抖动诊断摘要）

以下为 2026-08-05/07 的真机诊断结论与关键数据；当时使用的 checkpoint、prompt 和 2.0x/10 knot
参数均已过期，真机执行以本文上文为准。原始逐条记录已精简，保留结论和重要数据。

### 结论：三个相互叠加的项目侧抖动原因

1. 15 Hz 离散绝对位置目标形成阶梯输入；改 100 Hz 插值后多项真机指标改善。
2. 模型原始 chunk 离散加速度偏高（离线统计最大约 `1.02 rad/s^2`，约为官方 Home 限制
   `0.5 rad/s^2` 的 2 倍）；逐点硬裁剪会把离散最大加速度抬到 `1.87 rad/s^2`，制造额外折角，
   不能用作轨迹整形器。
3. 异步重规划的新旧 chunk 边界错位：推理约占 2-3 个 15 Hz 节点，新结果到达后从 `new[0]` 硬替换
   旧计划，无延迟补偿、无边界连续化。

官方低层控制链不是首要嫌疑：恒定保持、确定性轨迹和冻结 chunk 的 100 Hz 版本都能在同一官方链路
上取得一致改善。官方 Home 规划器参数（供参考）：规划率 `500 Hz`、Home 速度 `0.2 rad/s`、加速度
`0.5 rad/s^2`、模式切换 ramp `3.0 s`、`marvin_robot_node` 控制率 `200 Hz`；rollout 的 Custom
路径绕过 Home 轨迹规划器。

### 关键实验数据

- 静止 20 次推理：端到端延迟 median `144.9 ms`、p95 `166.1 ms`；新旧计划边界差
  `new[0]-old[4]` 最大 `0.06386 rad`（等效 `0.958 rad/s`）——时序错位是抖动的重要来源。
- 恒定姿态保持（hold test，不连 policy）：bridge 发布 15 Hz -> 100 Hz，最大关节峰峰值
  `0.003019 -> 0.000891 rad`（降 70.5%）。
- 确定性最小 jerk 轨迹（`Joint7_L` ±0.04 rad）：客户端目标 15 Hz -> 100 Hz，表观跟踪误差 RMS
  `0.006767 -> 0.004301 rad`，加速度 p95 降 41%——低频离散目标对动态不平滑是已证实因素。
- 冻结真实 chunk A/B（`artifacts/marvinpro_red_cones_chunk_ab_v2.json`，SHA-256
  `e72510cb...11c6d0`）：100 Hz 插值相对 15 Hz 离散跟踪误差 RMS 降约 30%；在相同节点与
  100 Hz 插值下仅做 2 倍慢放（`0.667 s -> 1.333 s`），加速度 p95 再降 51%、`Joint4_R` 最大表观
  误差 `0.050226 -> 0.027845 rad`，操作员主观“减轻较明显”——“原始时间尺度下机器人持续追赶
  目标”有真机证据，但 2 倍速仍有残余误差，不是唯一原因。version 1 旧计划文件超过诊断安全上限，
  不可使用。
- 短时真机 rollout 重规划统计（12 次推理）：推理延迟 median `159.5 ms`（约 2.4 个 15 Hz 周期）；
  `new[0]` 距上一发送目标 median `0.01719 rad`、距真实反馈 median `0.00772 rad`，且 11 次中 9
  次 `new[0]` 是 `new[0..4]` 中最接近反馈的节点——客户端按墙钟执行的旧目标跑在真机前面，新
  计划从反馈附近重新生成，硬替换形成“追赶 -> 回拉”振荡。固定跳 `new[k]` 不是充分的延迟补偿
  （`new[2]` 距反馈反而更远）。

### 10 节点持续 rollout 失败实验（2026-08-07，禁止重复）

10 节点完整消费 + 100 Hz 插值 + 2 倍时间 + 0.30 s 预取、新 chunk 追加到旧 raw 尾部：真机出现
约 1.33 s 周期的明显回弹（与完整 chunk 周期吻合）。决定性样本：旧 raw 尾部超前最后实际发送目标
`0.07926 rad`，新 `action[0]` 距反馈仅 `0.01041 rad`、距旧 raw 尾部反向 `0.16913 rad`；运动
阶段 531 个发布 tick 中 77 个手臂裁剪 tick（`Joint1_R`/`Joint4_R`），安全滤波器持续饱和追赶。

教训：完整消费 10 节点的 open-loop 假设在当前跟踪速度下不成立；不能把新 chunk 锚定到未实际实现
的旧 raw 尾部；增大预取窗口只会让观测更陈旧。`--prefetch-steps 0` 单变量对照（消除硬替换）仍有
连续颤抖，说明异步硬替换不是持续颤抖的必要条件，但同步等待推理引入的“运动 -> hold -> 运动”也
不适合作为最终调度。

### synchronized 基线建立（2026-08-07）

严格同步调度（执行完整 chunk -> 锁存末目标 -> 跟踪到位 `0.01 rad` -> 稳定保持 `0.20 s` -> 等待
新观测 -> 推理 -> 下一 chunk）消除了周期回弹，操作员确认“回弹消失，确实变平滑”；三段 chunk 到位
最终误差 `0.0028-0.0045 rad`。首段末目标长期超前反馈（约 165 个手臂裁剪 tick）是可重复现象，
需单独处理。

同期修复结束保持 bug：结束路径原本逐帧把最新反馈锁存为新 hold 目标，残余运动形成棘轮式漂移；
改为结束瞬间只采样一次固定目标，真机复测确认无漂移。
