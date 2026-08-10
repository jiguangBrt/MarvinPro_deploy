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

清空工作区、急停可触及，Apex Input Mode 初始保持 None。重新启动 motion-enabled 100 Hz bridge：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh \
  --allow-motion \
  --publish-hz 100
```

执行保守的单初始 chunk 测试：

```bash
cd /home/jh/OpenPI_UR/openpi
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
  --log-level DEBUG
```

现场核对确认页后输入 `EXECUTE`，再按提示切 Custom。结束时先切回 None。通过条件：

- handoff 不计入 A0-A9 phase，phase 单调且跟踪变慢时会降速；
- arm clipping 为 0，raw/sent target 不持续分离；
- A9 checkpoint 有 `<=0.01 rad`、连续 `0.20 s` 的 source-timestamp 证据；
- 无 heartbeat timeout、timer overrun 或 stale feedback。

## 5. synchronized 回归

使用 README 中已验证的 synchronized 参数运行至少两个 chunk。protocol v2 更新后，边界误差、跟踪时间、
clipping 和固定 hold 行为不得劣于旧基线。回归失败时不进入 RTC shadow。

## 6. RTC shadow

远程清单、单 chunk governor 和 synchronized 回归全部通过后，运行：

```bash
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
  --log-level DEBUG
```

RTC 输出不会 merge。确认日志证明：物理 A3 checkpoint 后才采图；推理期间继续旧 A4/A5；bridge 在整数
knot 边界记录 `d_actual`；shadow 丢弃后，本 episode 只运行 synchronized，不重试 RTC。

## 7. 两 chunk RTC

删除 `--rtc-shadow`，其余参数不变。首轮只允许两个 RTC merge，现场人员在第二次 merge 后结束并切回
None。每次 merge 必须满足：

- observation 晚于 checkpoint 完成，state/image skew `<=50 ms`；
- request/plan/timeline/checkpoint ID 全匹配；
- `1 <= d_actual <= d_pred <= 4`，接管 phase 是精确整数；
- 推理期间无 clipping、hard freeze、stale state、timer overrun 或 old-prefix underrun；
- replacement anchor 等于边界时正在执行的旧 reference，边界误差显著低于 `0.16913 rad`；
- 不出现约 `1.33 s` 周期回弹。

任一项失败：切回 None/必要时急停，保存 bridge 和 rollout 完整日志。本 episode 的 RTC fallback 必须保持
latched；不得现场放宽 `0.08/0.12 rad` 包络、URDF margin 或 tracking/stale 阈值。

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
