# Marvin Pro OpenPI 真机 Rollout

本目录实现三进程部署链路：

```text
Marvin Pro 6.6.7.100                 本机                         GPU 192.168.50.73
ROS topics <-> 双向 bridge:7332 <-> rollout client <-> WebSocket policy:8000
```

rollout 客户端应运行在当前电脑，不运行在机器人控制器。机器人端只运行轻量 ROS bridge；GPU
服务器只运行 OpenPI policy server。

## 已按当前设备固定的接口

- 观测/动作顺序：`[左臂7, 左夹爪, 右臂7, 右夹爪]`。
- 关节单位：rad；policy server 输出已经过 `AbsoluteActions`，是绝对关节目标。
- 夹爪：模型使用 `0=open, 1=closed`；状态由 `/info/gripper_feedback_L/R.data[0] / 1.25`
  归一化，命令直接向 `/control/gripperValueL/R` 发布 `0..1`。
- 相机：四宫格左上=`cam_high`，左下=`cam_left_wrist`，右下=`cam_right_wrist`，右上忽略；底部
  时间戳区域不进入模型。
- 双臂目标：`/control/user/joint_cmd_A/B`，消息类型
  `marvin_msgs/msg/JointcmdArm`，A=左、B=右。
- 硬限位：来自控制器当前 `APEX_ROBOT_MODEL=new_m6_696` 的左右臂 URDF。

bridge 不会调用 `/control/set_input`、`set_ready`、`go_home`、`clear_fault` 等 Service，也不会自动
改变机器人模式。

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

- `/joint_states`
- `/info/gripper_feedback_L`、`/info/gripper_feedback_R`
- `/quad_tile/compressed`
- `/control/input_mode`
- `/info/robot_state`、`/info/arm_state`

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

- policy 输出恒为 `(10, 16)` 且 finite；
- `wall` 通常约为此前实测的 90-120 ms；
- 相机、关节和夹爪 age 没有超限；
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
ssh nvidia@6.6.7.100 'source /etc/apex/apex_ros_env.sh; ros2 topic echo /control/input_mode --once; ros2 topic echo /info/robot_state --once; ros2 topic echo /info/arm_state --once'
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

程序打印实机状态后，必须手动输入 `EXECUTE` 才开始动作。5 秒动作结束后客户端会持续发送当前反馈
位姿作为 hold，并提示切换模式。此时在 Apex 把 Input Mode 切回 **None**；客户端检测到模式不再是
Custom 后才断开。最后再用 `Ctrl+C` 停止 bridge。

确认短 rollout 正常后再逐步增加 `--episode-seconds`。不建议首次执行使用 `--yes` 跳过确认。

## 安全门控

真机动作要同时满足：

1. bridge 使用 `--allow-motion` 启动；
2. rollout 使用 `--execute` 并人工输入 `EXECUTE`；
3. `input_mode == 3`；
4. `/info/robot_state == [3,3]` 且 `/info/arm_state == [3,3]`（关节阻抗模式）；
5. 关节和夹爪反馈新鲜，policy action 为 finite `(16,)`；
6. 客户端单次相对当前反馈最多 `0.08 rad`，bridge 二次检查最多 `0.12 rad`；
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
`0.08 rad/15 Hz` 已允许约 `1.2 rad/s` 的最坏目标变化。

bridge 使用 pickle 传输 JPEG 和数据结构，只能暴露在可信的机器人私有网络，不应映射到公网。

## 本地测试

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
PYTHONPATH=src python3 -m unittest discover -s tests -v
```
