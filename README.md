# Marvin Pro OpenPI 真机 Rollout

本目录实现三进程部署链路：

```text
Marvin Pro 6.6.7.100                 本机                         GPU 192.168.50.73
ROS topics <-> 双向 bridge:7332 <-> rollout client <-> WebSocket policy:8000
```

rollout 客户端应运行在当前电脑，不运行在机器人控制器。机器人端只运行轻量 ROS bridge；GPU
服务器只运行 OpenPI policy server。

当前 `tmp` 分支现场接口：机器人控制与状态 topic 使用 `/tj` 命名空间；四宫格相机仍使用根路径
`/quad_tile/compressed`，其 `CompressedImage.format` 为 `h264`。rollout 客户端用有状态 PyAV
解码连续 H264 包，再按训练布局切分图像。

## 已按当前设备固定的接口

- 观测/动作顺序：`[左臂7, 左夹爪, 右臂7, 右夹爪]`。
- 关节单位：rad；policy server 输出已经过 `AbsoluteActions`，是绝对关节目标。
- 夹爪：模型使用 `0=open, 1=closed`；命令直接向 `/tj/control/gripperValueL/R` 发布 `0..1`。policy state
  使用 `/tj/info/gripper_feedback_L/R` 的实测 `q`，按训练数据相同的 `open_raw=0.0`、`closed_raw=1.25`
  归一化并裁剪到 `0..1`。
- `/tj/info/gripper_feedback_L/R` 的字段布局虽然是
  `[q_position_rad, dq_velocity_rad_s, tau_torque, temp_mos, temp_motor]`。2026-08-19 已重新确认左右 topic 有动态
  信息；右夹爪实测“打开→夹持→打开”时 `q` 约为 `-0.0185→1.0234→-0.0185`，稳定 `tau` 约为
  `0.047→1.683→0.046`。feedback 现在参与 policy state、RTC anchor、运动门和实测 telemetry。
- 相机：四宫格左上=`cam_high`，左下=`cam_left_wrist`，右下=`cam_right_wrist`，右上忽略；底部
  时间戳区域不进入模型。
- 双臂目标：`/tj/control/user/joint_cmd_A/B`，消息类型
  `marvin_msgs/msg/JointcmdArm`，A=左、B=右。
- 硬限位：来自控制器当前 `APEX_ROBOT_MODEL=new_m6_696` 的左右臂 URDF。

bridge 不会调用 `/control/set_input`、`set_ready`、`go_home`、`clear_fault` 等 Service，也不会自动
改变机器人模式。

## 夹爪直接控制

当 Apex Home 无法让夹爪完全张开时，可以从本机直接发送夹爪目标。先停止 rollout 和
`run_bridge_on_controller.sh`，并确认 Apex 没有在运行 Teleop 或 Replay；执行闭合命令前让手和物体
离开夹爪。

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy

# 同时完全打开左右夹爪
./scripts/control_gripper_on_controller.sh 0

# 同时完全闭合左右夹爪
./scripts/control_gripper_on_controller.sh 1

# 只控制一侧
./scripts/control_gripper_on_controller.sh 0 --side left
./scripts/control_gripper_on_controller.sh 1 --side right
```

脚本将 `0` 或 `1` 短时连续发布到 `/tj/control/gripperValueL/R`，不会调用 Home、切换 Apex Input Mode
或发送机械臂关节命令。脚本会同时打印最新 `/tj/info/gripper_feedback_L/R` 的 `q/dq/tau/温度`；现场仍需
肉眼确认操作安全和物体是否夹稳。
若检测到 rollout bridge 仍在运行，脚本会拒绝发布，避免两个外部命令源互相覆盖。

## 夹爪 feedback 实测记录

要验证 feedback 是否会随“打开→闭合夹住物体→打开”变化，先开一个终端运行只读记录器，再用另一个终端
执行上面的控制命令。记录器不会发布任何夹爪或机械臂命令，默认一直运行到 `Ctrl+C`，并把 CSV 保存到本机
`logs/`：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/record_gripper_feedback_on_controller.sh
```

记录器完成 ROS discovery 并显示 `READY` 后，你可以在另一个终端按下面顺序操作，不需要等待其他提示：先确认两侧打开；
发布 `0` 保持打开 3 秒；放入物体并发布 `1` 保持闭合 3～5 秒；取出物体后再发布 `0` 保持打开 3 秒；
最后回到记录器终端按 `Ctrl+C`。记录器结束后重点看最终摘要：`changed`、`distinct`、`span` 和 `max_step`，
CSV 还包含命令时间标记，可用于对齐打开、闭合和夹持阶段。

## 1. GPU 服务器启动 policy

在 `192.168.50.73`：

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

看到 `server listening on 0.0.0.0:8000` 才进入下一步。首次推理 JIT 可能很慢；rollout 会默认做一次
warmup 并丢弃其输出，绝不执行 warmup 动作。

## 2. 机器人端只读预检

在本机另开终端：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --doctor --duration 8
```

预检不会创建动作 publisher，也不会发指令。以下输入必须都有消息：

- `/tj/joint_states`
- `/quad_tile/compressed`
- `/tj/control/input_mode`
- `/tj/info/robot_state`、`/tj/info/arm_state`
- `/tj/info/gripper_feedback_L/R`

doctor 会打印 `/tj/info/gripper_feedback_L/R` 的五维字段；任一侧缺失或在 rollout 中超过 state stale
阈值都会关闭运动门。
相机没有消息时，先在 Apex 启动 Camera。执行前还必须看到 `input_mode=3`、两个状态数组均为
`(3, 3)`；这台控制器用状态 `3` 表示关节阻抗模式。dry-run 阶段可以仍为 None/`0`。

## 3. 全链路 dry-run

终端 A 启动不允许运动的 bridge：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh
```

终端 B 在本机 OpenPI 环境运行 10 秒 dry-run：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --episode-seconds 10
```

默认没有 `--execute`，客户端不会向 bridge 发送任何 action。检查日志应满足：

- policy 输出恒为 `(20, 16)` 且 finite；
- 当前远程 H20 服务的 20 次持久请求 wall p50/p95 约为 `343/390 ms`，单次请求必须小于默认 2 秒 timeout；
- 相机、关节和左右夹爪 age 没有超限，日志显示 `gripper_state_source=measured_feedback`；
- 能持续完成 inference，没有图像裁剪或连接错误。

## 4. 首次真机执行

先停止 dry-run bridge。清空工作区并保持急停可触及，在 Apex 完成 Robot Ready、Impedance Mode、
安全的任务起始姿态和 Camera 启动。不要在出厂打包姿态直接 Home。

终端 A 显式允许 bridge 发布：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion
```

在 Apex 将 Input Mode 切到 **Custom**。确认：

```bash
ssh nvidia@6.6.7.100 'source /etc/apex/apex_ros_env.sh; ros2 topic echo /tj/control/input_mode --once; ros2 topic echo /tj/info/robot_state --once; ros2 topic echo /tj/info/arm_state --once'
```

预期分别是 `3`、`[3,3]`、`[3,3]`。然后终端 B 先做一个很短的 rollout：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 5
```

程序打印实机状态后，必须手动输入单个大写 `E` 才开始动作。5 秒动作结束后客户端只采样一次当前反馈
位姿，并持续发送这个固定目标作为 hold；它不会继续跟随后续反馈更新目标。此时在 Apex 把 Input Mode
切回 **None**；客户端检测到模式不再是 Custom 后才断开。最后再用 `Ctrl+C` 停止 bridge。

确认短 rollout 正常后再逐步增加 `--episode-seconds`。不建议首次执行使用 `--yes` 跳过确认。

## 安全门控

真机动作要同时满足：

1. bridge 使用 `--allow-motion` 启动；
2. rollout 使用 `--execute` 并人工输入单个大写 `E`；
3. `input_mode == 3`；
4. `/tj/info/robot_state == [3,3]` 且 `/tj/info/arm_state == [3,3]`（关节阻抗模式）；
5. 关节和左右夹爪 feedback 新鲜，归一化夹爪实测值在 `[0,1]`，policy action 为 finite `(16,)`；
6. 客户端和 bridge 的臂关节目标相对最新反馈最多 `0.16 rad`；
7. 目标位于当前 M6-696 URDF 硬限位内并保留 `0.02 rad` 边界；
8. bridge 收到的 action 不超过 `0.25 s`，且对应观测不超过 8 帧。

任一条件失效，bridge 清空目标并停止发布。action chunk 暂时耗尽时，客户端发送当前测量位姿作为
hold，不会重复执行过时的预测动作。

## 常用调节

网络偶发超过 130 ms 时，可增加缓冲但不要超过模型 horizon 10：

```bash
... -m marvinpro_deploy.rollout_client --execute --execute-steps 6 --prefetch-steps 4
```

若日志频繁出现 `safety filter clipped`，先检查起始姿态是否落在训练分布内，不要直接放宽限幅。默认
`0.16 rad` 是每个臂关节目标相对最新反馈的位移包络，不应换算成固定速度上限。

bridge 使用 pickle 传输 JPEG 和数据结构，只能暴露在可信的机器人私有网络，不应映射到公网。

## 跟踪感知时间轴与 RTC（protocol v10）

`synchronized`、`tracking` 和 `rtc` 使用 bridge 本地 100 Hz trajectory owner。控制 timer 只对连续 phase 求值，不会按
100 Hz 自动消费模型动作；机器人跟踪误差增大时，phase 会减速或冻结。RTC A9 checkpoint 只有在全部14个
臂关节误差不超过 `0.01 rad`，并且由持续更新的 joint source timestamp 证明连续稳定 `0.20 s` 后才成立。
因此“客户端已经发出 A9，但反馈仍在 A8”不会触发新观测。

三种 trajectory schedule 固定使用 `--model-hz 15 --playback-time-scale 3`，名义 knot rate 为 `5 Hz`；其他倍率
会被客户端拒绝。tracking error 在 `<=0.02 rad` 时 phase rate 为 1，在 `0.02..0.16 rad` 内按
`(0.16-error)/0.14` 线性降低，在 `>=0.16 rad` 时硬冻结。由 tracking error 触发硬冻结后，误差降到
`<=0.12 rad` 才解除锁存；joint state stale、timer overrun 和 arm clipping 仍直接硬冻结。臂关节 safety
clipping 包络为 `0.16 rad`。

`d_pred` 使用当前 estimator epoch 内可行 latency 样本的保守 p95 和 `50 ms` guard。会超过四个 old-tail
knot 的样本作为 link fault 单独记录，不进入稳定分布。RTC 默认使用 `--rtc-late-result-policy discard`：
bridge 在物理 `d_actual == d_pred` 边界使迟到 epoch 无效，结果不得 merge；`wait` 仅保留用于比较 deadline
停顿、边界跳变和恢复率。fallback 会重置 estimator epoch，但 bridge 的 `d_actual` 始终按实际 phase 跨过的
knot 计数，不使用 `wall_time * nominal_rate`。

protocol v10 必须同时更新控制器上的 `MarvinPro_deploy` 和本机客户端。夹爪状态使用归一化实测 feedback，
原始 `q/dq/tau/温度` 写入 telemetry；RTC 的 tracking governor 仍只使用 14 个机械臂关节误差，夹爪不参与
机械臂到位判定。
trajectory session 每 `100 ms`
发送 heartbeat；bridge 超过 `250 ms` 未收到会清空 trajectory 并停止发布。旧的 discrete、prefetch、
legacy discrete/prefetch 仍使用 `ActionCommand`；synchronized 已统一到 bridge-owned timed chunk。

先只验证 bridge governor，不启用 RTC merge：

```bash
# 控制器 bridge，Apex Input Mode 先保持 None
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
export RUN_DIR="$PWD/logs/tracking_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/marvinpro_tracking_run_dir
./scripts/run_bridge_on_controller.sh \
  --local-log "$RUN_DIR/bridge.log" \
  --allow-motion \
  --publish-hz 100

# 本机客户端，按提示完成确认后再切 Custom
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_tracking_run_dir)"
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
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

指定 `--log-file` 时，完整 DEBUG 诊断写入文件，终端默认仅显示 WARNING 和交互提示；可用
`--console-log-level DEBUG` 临时恢复终端详细输出。运动完成后客户端会等待操作员把 Apex Input Mode 切回
None，再安全退出。

指定 `--log-file "$RUN_DIR/rollout.log"` 还会自动生成
`$RUN_DIR/rollout.telemetry.csv`。它逐条保存 bridge 实测 14 个关节、夹爪原始/归一化 feedback、夹爪命令、
bridge 插帧后的最终命令，以及
prefetch/legacy 客户端插帧生成的请求目标和实际发送命令；`client_reference_*` 是插帧值，
`client_command_*` 是 safety-filter 后的发送值，`record_type` 字段区分 `bridge_state` 和
`client_command`。
测试后可生成关节角以及夹爪命令/实测 feedback 对照图：

```bash
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
/home/jh/OpenPI_UR/openpi/.venv/bin/python \
scripts/plot_rollout_joints.py \
  "$RUN_DIR/rollout.telemetry.csv" \
  -o "$RUN_DIR/joint_diagnostics.png"
```

远程 OpenPI 必须先完成
`/home/jh/OpenPI_UR/openpi/REMOTE_RTC_TESTS.md`。之后第一轮只运行 shadow：新观测在
物理 A9 checkpoint 后采集，推理期间 bridge 继续执行 A10/A11 等旧节点，客户端等待实际 `d_pred` 边界并
记录 `d_actual`，但不合并 RTC 输出；shadow 随后固定降级 synchronized，不自动重进 RTC。

```bash
export RUN_DIR="$(cat /tmp/marvinpro_tracking_run_dir)"
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 5 \
  --rollout-schedule rtc \
  --rtc-continuous \
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

shadow、单 chunk governor 和 synchronized 回归全部通过后，删除 `--rtc-shadow` 才允许实际 merge；首轮实际
测试必须加 `--max-rtc-merges 1`，达到一次成功 replacement merge 后，客户端会等待当前 RTC 段到达下一个 checkpoint，再锁存 hold。RTC
只在下一个整数 knot 边界替换 future，并由 bridge 重新计算 `d_actual`。clipping、hard freeze、反馈过期、
heartbeat 超时、ID/version 不匹配、迟到结果或 `d_actual > d_pred` 都会拒绝结果。late/discard、瞬时 transport、
可重采样 observation lag 和 C2 merge 不可行属于可恢复故障：bridge 原子锁存实测臂位置并保留夹爪命令，
至少完成一个 clean timed synchronized chunk 后，用新 H20 推理重建 estimator epoch 和 RTC 初始计划。每轮最多
恢复 3 次；安全、事务和协议故障只进入 fixed hold，不自动重进 RTC。

每次 RTC 请求日志分别记录 observation preparation、request build/serialization、transport round trip、
估算的 network round trip、server deserialize/queue/infer、RTC preprocess/denoise/postprocess、response decode
和 bridge stage/merge。network 字段是 transport 减去已测 server path 的残差估计，不是单向网络测量。

默认 RTC 使用 settled checkpoint，适合验证观测和 merge 证据，但会在每个 A5 等待跟踪到位并稳定
`0.20 s`。settled 多 chunk 验收通过后，可显式增加 `--rtc-continuous`：bridge 在 A5 记录观测边界后继续
旧轨迹，client 拿图期间跨过的 knot 会计入 `d_actual`，其余安全门不变。连续模式必须重新从
`--rtc-continuous --rtc-shadow` 开始验收，shadow 通过后才允许单次实际 merge；详细顺序见
[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md)。

若本机受代理或路由影响，禁止把连接失败误判成 RTC 算法失败，也不要自动尝试真机。按
[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 完成网络预检和现场分阶段验收。

## 持续 rollout 的慢速插值诊断

> **失败实验，禁止继续真机复现。** 2026-08-07 真机结果出现约1.33秒周期的明显回弹和77个手臂
> 裁剪tick。以下命令仅保留用于复现实验参数，不应再次带 `--execute` 运行。后续测试必须先改为按
> 实际已发送目标衔接新计划，并缩短open-loop段。

该模式用于验证冻结 chunk 中已经改善明显的两项设置能否降低持续重规划时的颤抖：保持模型节点的
15 Hz 时间语义，把时间拉长 2 倍，并在节点之间以 100 Hz 线性插值。它是显式诊断开关，默认
`discrete` rollout 行为不变。

新模式不会在一个已选动作段的中间覆盖旧计划。本项诊断每次完整消费 policy 输出的全部 10 个节点；
从上一段末目标到下一段 `action[0]` 也作为一个节点间隔插值。每段持续 `10 / 7.5 = 1.333 s`，当队列还剩
`0.30 s` 时开始推理下一段，推理返回后追加到队尾。当前最终配置实测约 `153-173 ms` 的推理延迟下
仍有 `13-15` 个 100 Hz 点留在队列中。

先确保 bridge 明确以 100 Hz 运行，Apex Input Mode 保持 None：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion --publish-hz 100
```

本次已经执行过的5秒真机失败参数如下，仅供记录：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.rollout_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --execute \
  --episode-seconds 5 \
  --playback-mode interpolated \
  --control-hz 100 \
  --model-hz 15 \
  --playback-time-scale 2 \
  --execute-steps 10 \
  --chunk-prefetch-seconds 0.30 \
  --log-level DEBUG
```

客户端完成 warmup 后会明确提示切到 Custom。确认页必须显示 `effective knot rate: 7.50Hz`、
`command rate: 100.0Hz` 和 `selected chunk: 10 knots over 1.333s`，再输入单个大写 `E`。结束时按提示先
切回 None。`chunk_append_diag` 中稳态 `underruns_since_last` 应为 `0`，`queued_before` 应大于 `0`；
若前者非零，先增大 `--chunk-prefetch-seconds`，不要放宽关节限幅。

## 同步执行、到位、保持、重观测诊断

该模式用于隔离周期回弹是否来自异步chunk错位。它严格按以下顺序运行，不提前采集下一段观测，也不在
当前chunk运动期间推理：

```text
完整发送当前chunk -> 锁存末目标 -> 等待全部14个臂关节到位 -> 稳定保持
-> 等待一帧新的图像/关节观测 -> 远程推理 -> 执行下一chunk
```

每个 H20 synchronized chunk 从 controller 的 `trajectory_loaded` 起有 `4 s + 1 s` deadline。5 秒内到位并基于
真实 source timestamp 稳定 `0.20 s` 才算 clean；健康超时时 bridge 会原子锁存当前实测臂位置，再从新图像
推理。连续两次卡住后结束运动。episode 剩余不足 5 秒时不再启动新 chunk。

先在 Apex 保持 Input Mode 为 None，启动100 Hz bridge：

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

另一个终端运行首轮10秒、3倍时间尺度测试：

```bash
cd /home/jh/OpenPI_UR/openpi
export RUN_DIR="$(cat /tmp/marvinpro_synchronized_run_dir)"
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
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

默认到位条件是所有臂关节误差不超过 `0.01 rad` 并连续保持 `0.20 s`。夹爪不参与到位判断。结构化日志会记录
chunk deadline、phase、最差关节、最终误差和 stuck replan 计数；episode结束后按提示将 Input Mode 切回 None。

结束等待阶段同样只锁存一次当前测量姿态。固定目标会一直保持到 Input Mode 离开 Custom，不会把残余
运动中的后续反馈再次变成新目标，从而避免结束阶段的单向漂移。该修复已通过短时2.0x真机回归确认。

若出现 `tracking timeout`，不要直接放宽误差阈值或关节步长。先记录超时关节、误差和
`arm-clipped ticks`，因为这表示10节点open-loop末目标在当前设置下未能可靠实现。只有确认2.0x时周期
回弹消失后，才保持其他参数不变依次改成 `--playback-time-scale 1.5` 和 `1.0` 做对照。

## 锁存姿态保持诊断

该测试不连接 policy server。客户端只在开始时读取一次当前姿态，之后持续发送完全相同的绝对关节
目标，用于区分 Custom/bridge 控制链抖动和模型轨迹抖动。

先启动允许控制的 bridge。在 Apex 完成关节阻抗模式，但暂时保持 Input Mode 为 None：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion
```

另一个终端运行：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.hold_test_client \
  --robot-host 6.6.7.100 \
  --duration 10
```

客户端显示正在等待后，把 Apex Input Mode 切到 Custom。核对屏幕上的模式和锁存关节值，现场安全后
输入 `HOLD`。10 秒结束时客户端会继续发送同一个目标；此时先在 Apex 把 Input Mode 切回 None。
客户端检测到后退出并打印各关节峰峰值、标准差和最大跟踪误差。异常时优先切回 None 或使用急停，
不要依赖 `Ctrl+C` 作为正常停止步骤。

第一次使用默认的 15 Hz bridge。切回 None、停止 bridge 后，可以用 100 Hz 重复相同测试：

```bash
./scripts/run_bridge_on_controller.sh --allow-motion --publish-hz 100
```

如果 15 Hz 抖动而 100 Hz 明显改善，bridge 的低频目标发布是主要因素；如果两次都稳定，而 rollout
会抖，问题位于模型动作或重规划衔接；如果两次恒定目标都抖，则继续检查 Custom 控制链和阻抗参数。

## 确定性慢速轨迹诊断

该测试同样不连接 policy server。保持 bridge 以 100 Hz 运行，默认只让 `Joint7_L` 在当前姿态附近按
最小 jerk 曲线完成 `0 -> +0.04 -> -0.04 -> 0 rad`，总时长 8 秒。理论峰值速度
`0.0375 rad/s`、峰值加速度小于 `0.06 rad/s^2`，均低于官方 Home 限制。

先用 15 Hz 客户端目标更新运行：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.trajectory_test_client \
  --robot-host 6.6.7.100 \
  --command-hz 15
```

客户端等待时将 Apex 切到 Custom，核对目标关节、幅度、bridge 频率和状态后输入 `MOVE`。轨迹返回
起点后，先将 Apex 切回 None。随后使用完全相同的姿态和参数，把 `--command-hz` 改为 `100` 再做一次。

- 15 Hz 有阶梯顿挫而 100 Hz 平滑：客户端目标更新离散度是主要因素。
- 两次确定性轨迹都平滑，但 rollout 抖：模型动作或异步重规划衔接是主要因素。
- 100 Hz 确定性轨迹仍抖：继续检查 bridge/ROS 动态指令路径和阻抗响应。

不要使用 `--yes` 跳过首次真机确认。异常时优先切回 None 或急停。

## 单个冻结 Policy Chunk 诊断

该测试连接真实 policy，但只保存一次 10 步输出，执行期间不再推理、不替换 chunk，并保持夹爪目标不变。
JSON 同时保存原始 policy 节点和限制在推理姿态 `±0.03 rad` 的实际回放节点。先按 15 Hz 直接发送
有界回放节点，自动用最小 jerk 返回推理姿态；再加载同一个 JSON，以相同 15 Hz 时间轴做 100 Hz
线性插值回放。
两次的模型目标和总时长相同，唯一变量是目标更新方式。

保持 bridge 为 100 Hz，Apex Input Mode 先设为 None，然后运行：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --capture-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --playback-mode discrete
```

客户端完成 warmup 和冻结推理后会提示切换 Custom。切换后核对原始 policy 与有界回放各自的离散
速度、加速度、裁剪数量和姿态漂移，再输入 `DISCRETE`。程序按 15 Hz 回放，然后用 2 秒最小 jerk
轨迹自动返回推理姿态；看到提示后切回 None。若计划超过诊断安全上限 `0.45 rad/s` 或
`2.0 rad/s^2`，程序只保存 JSON 并拒绝运动。

回到 None 后，从同一姿态加载完全相同的计划：

```bash
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --playback-mode interpolated
```

切到 Custom 并核对确认页后输入 `INTERPOLATE`。第二次保持同样的模型 15 Hz 时间语义，只把相邻节点
之间细分为 100 Hz 目标，结束后同样自动返回锚点，再切回 None。

- 15 Hz 离散回放抖、100 Hz 插值平滑：低频阶梯目标是该 chunk 不平滑的主要因素。
- 两次都抖：模型 chunk 自身的节点变化或简单线性插值仍不合适，需要速度/加速度/jerk 轨迹整形。
- 两次都平滑而持续 rollout 抖：异步重规划延迟和 chunk 边界替换是主要因素。

此测试会执行模型产生的动作。必须清空工作区、保持急停可触及，并核对确认页；不要首次使用 `--yes`。

需要隔离 `±0.03 rad` 诊断包络时，可以加载同一个 version 2 JSON 并增加
`--target-source raw`。客户端会先验证原始节点的硬限位、bridge步长包络、离散速度和加速度，再显示
最大锚点行程；任一检查超限都不会进入执行。原始节点也应先做15 Hz离散回放，自动回锚并切回None后，
再做100 Hz插值回放。

```bash
# Apex Input Mode 先保持 None；准备完成后再按提示切到 Custom
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --target-source raw \
  --playback-mode discrete

# 第一次自动回锚并切回 None 后，再执行第二次
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --target-source raw \
  --playback-mode interpolated
```

若要验证机器人是否因为跟不上原始时间轴而产生错位，保持 raw 节点和100 Hz插值不变，只把播放时长
拉长2倍：

```bash
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --target-source raw \
  --playback-mode interpolated \
  --playback-time-scale 2.0
```

这会把同一计划从 `0.667 s` 拉长到 `1.333 s`，最大速度减半、最大加速度降为四分之一；参数不允许
小于 `1.0`，因此不能通过该入口加速计划。

## 本地测试

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
PYTHONPATH=src python3 -m unittest discover -s tests -v
```
