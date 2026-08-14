# Marvin Pro 跟踪感知执行与 RTC 实施计划

更新时间：2026-08-10

本文是后续修改代码时使用的执行清单。目标不是单独把 RTC sampler 接到现有队列上，而是先让客户端能够
区分“命令已发送”“计划已推进”和“机器人实际已跟踪”，再在这个基础上实现可安全回退的 RTC。

## 当前实施状态（2026-08-10）

- protocol v2、bridge 本地连续 trajectory、tracking governor、物理 checkpoint、RTC merge 和
  synchronized episode fallback 已实现。
- OpenPI MarvinPro absolute prefix 变换、JAX VJP sampler、`rtc_v1` WebSocket envelope、结构化错误和计时
  已实现。
- 本机仅执行离线 CPU/fake bridge 测试，不连接机器人或加载本机 GPU checkpoint；当前离线测试结果和命令
  以本轮最终报告为准。
- 远程 GPU/checkpoint/JIT/延迟测试见 `/home/jh/OpenPI_UR/openpi/REMOTE_RTC_TESTS.md`。
- 机器人网络、doctor、governor、shadow 和两 chunk 真机准入见本目录 `ROBOT_RTC_TESTS.md`。
- 两份待测清单通过前，RTC 实际 merge 仍视为未获真机准入；不得跳过 shadow 或放宽安全阈值。

相关背景和现场证据：

- `ASYNC_ACTION_CHUNKING.md`
- `_HANDOFF.md`
- Physical Intelligence RTC 论文：<https://arxiv.org/abs/2506.07339>
- 官方 JAX 参考实现：<https://github.com/Physical-Intelligence/real-time-chunking-kinetix>
- LeRobot RTC 实现：<https://github.com/huggingface/lerobot/tree/main/src/lerobot/policies/rtc>

## 1. 最终目标

完成后的系统应同时满足：

1. 100 Hz 只是控制发布频率，不再等同于策略轨迹的物理进度。
2. 机器人追不上参考轨迹时，策略时间轴会减速或冻结，不再继续消费未来动作。
3. 策略观测只在指定轨迹 checkpoint 已被真实跟踪且稳定后采集。
4. RTC 推理期间继续执行旧轨迹，返回后按照真实轨迹进度而非发送包数量合并新轨迹。
5. RTC 只约束确实会被执行的旧参考轨迹。推理期间发生裁剪、反馈过期或计划版本变化时，结果必须作废。
6. 任意 RTC 异常都能退回已验证的 synchronized 模式，不能放宽现有安全包络。

第一版不追求完全无停顿。允许在中途 tracking checkpoint 短暂停住，以换取正确的物理状态对齐。得到足够
真机数据后，再评估是否放宽 checkpoint 的稳定等待。

## 2. 当前系统存在的问题

### 2.1 三种进度被混为一种

系统实际上有三种不同的时钟：

| 时钟 | 含义 | 当前实现 |
| --- | --- | --- |
| 控制时钟 | 100 Hz 命令发布 tick | `ActionPublisher` 固定按墙钟运行 |
| 计划时钟 | A0、A1 等模型轨迹节点的进度 | 被展开成 100 Hz deque，并随每次 `pop()` 无条件前进 |
| 物理进度 | 机器人关节实际沿轨迹走到哪里 | 只用于逐 tick 安全裁剪，不参与队列推进 |

因此，客户端可能已经发送 A3，队列也认为 A3 已经过期，但机器人还停留在 A2 附近。

### 2.2 安全裁剪掩盖了计划超前

`safety.filter_action()` 每次把 raw 目标限制在最新反馈的 `+/-0.16 rad` 内。这个逻辑是安全包络，不是轨迹
跟踪器。当前发布器在发生裁剪后仍然弹出下一个计划点，结果是 raw 计划持续向前，而实际发送目标长期贴着
反馈包络追赶。

RTC 不能把 raw A4、A5 当成已经承诺的命令，因为发生裁剪时机器人实际收到的并不是 raw A4、A5，并且
未来裁剪值依赖尚未发生的反馈。

### 2.3 prefetch 只依据队列剩余时间

当前 `prefetch` 调度在队列剩余指定秒数时采集观测并推理，不检查机器人是否到达相应轨迹位置。因此新
chunk 虽然基于真实反馈生成，却被追加到一个机器人没有实现的旧 raw 尾部，产生周期性回弹。

### 2.4 synchronized 只在整个 chunk 之后检查跟踪

已实现的 synchronized 模式能够消除周期回弹，但流程是“完整发送 chunk 后等待末目标到位”。它证明了
重新观测前必须等待真实跟踪，却不能直接提供连续异步执行。

### 2.5 客户端没有高频反馈

桥接进程会持续接收 `/joint_states`，但 `RobotObservation` 只在图像 callback 中创建并发送。客户端看到的
关节反馈受约 11 Hz 图像频率和传输相位限制，不能可靠驱动 100 Hz tracking gate。

### 2.6 当前 action 索引存在额外 anchor 间隔

训练语义中 A0 对应观测时刻，A0 到 A9 只有 9 个模型时间间隔。当前插值路径额外构造
`(feedback_anchor, A0, ..., A9)`，把 anchor 到 A0 也算作一个完整 knot 间隔。RTC 开始前必须明确区分：

- 模型 action phase：A0 的索引为 0。
- 初始 handoff：从真实反馈到 A0 的独立过渡，不计入 RTC 的 action 索引。

否则 `s`、`d` 和实际执行时间会持续差一拍。

## 3. 必须保持的安全不变量

所有阶段都必须保持以下约束：

- 客户端和 bridge 使用 `0.16 rad` 反馈包络。
- URDF 关节限位和 `0.02 rad` margin 不放宽。
- 反馈过期、motion gate 关闭、命令拒绝或计划版本不一致时立即停止推进。
- 空计划时只能锁存一次固定目标，不能逐帧把 measured pose 重新锁存为新目标。
- clipping 不能被统计后忽略；arm clipping 必须影响计划推进和 RTC 结果有效性。
- 当前 RTC 只允许插值回放、3.0x 时间尺度（5 Hz knot rate）和完整 H=10 配置。
- 未通过离线测试和 dry-run 前不得运行 RTC 真机动作。
- 不修改官方低层控制器参数，不把提高发布频率当作跟踪修复。

## 4. 目标架构

```text
OpenPI RTC policy server
        ^ observation + committed prefix
        | RTC action chunk
rollout scheduler / timeline versioning
        ^ plan progress + tracking state
        | trajectory plan
tracking-aware phase governor
        ^ high-rate joint feedback
        | 100 Hz position reference
existing bridge safety + Marvin low-level controller
```

核心状态变量：

- `control_tick`：100 Hz 发布计数，只用于诊断。
- `phase`：连续的模型轨迹位置，例如 `3.25` 表示位于 A3 到 A4 之间。
- `plan_id`、`timeline_version`：标识当前轨迹及其每次替换。
- `raw_reference`：当前 phase 对应的未裁剪参考目标。
- `sent_target`：经过客户端安全包络后的实际发送目标。
- `measured_joints`：按源时间戳对齐的高频反馈。
- `tracking_error`：参考目标与反馈的手臂最大关节误差。
- `phase_rate`：当前计划时钟相对 nominal knot rate 的推进比例。
- `clipped`、`state_stale`、`rtc_in_flight`：安全和调度状态。

第一版状态机：

```text
INITIAL_SYNC
  -> TRACK_TO_CHECKPOINT
  -> RTC_IN_FLIGHT
  -> MERGE_PENDING
  -> TRACK_TO_CHECKPOINT

任意状态 --异常/裁剪/超时--> FALLBACK_SYNC
任意状态 --motion gate关闭--> STOPPED
```

## 5. 按顺序执行的修改

后续阶段依赖前一阶段的验收条件。不要并行实现 RTC sampler 和执行器；否则无法判断问题来自采样 guidance
还是错误的物理进度。

### 阶段 0：冻结基线和新增功能开关

修改范围：

- `src/marvinpro_deploy/rollout_client.py`
- `tests/test_rollout_client.py`
- `README.md`

任务：

- [ ] 保留现有 `prefetch` 和 `synchronized` 行为，新增调度模式时不得改变旧模式结果。
- [ ] 新功能先使用独立选项，例如 `tracking` 和 `rtc`，默认继续使用已知模式。
- [ ] 为所有实验记录完整 CLI、git commit、policy checkpoint 和协议版本。
- [ ] 固化当前 26 项测试结果以及 synchronized 真机指标作为回归基线。

验收条件：旧测试全部通过；未启用新选项时日志和命令序列不发生行为变化。

### 阶段 1：拆分高频状态与策略图像观测

修改范围：

- `src/marvinpro_deploy/protocol.py`
- `src/marvinpro_deploy/config.py`
- `src/marvinpro_deploy/robot_bridge.py`
- `src/marvinpro_deploy/rollout_client.py`
- `tests/test_protocol.py`
- `tests/test_rollout_client.py`

任务：

- [ ] 升级 `PROTOCOL_VERSION`，新增 `RobotStateUpdate` 消息。
- [ ] 状态消息至少包含 `state_seq`、精确 joint source monotonic timestamp、14 维关节、夹爪反馈、
      motion gate、最后命令 ID/状态和当前 bridge 目标。
- [ ] 图像触发的 `RobotObservation` 继续用于 policy inference；不要把 JPEG 重复塞进高频状态消息。
- [ ] bridge 在 joint callback 或独立固定频率路径发送状态，不能依赖图像 callback。
- [ ] `RobotConnection` 分发不同消息类型，并分别提供 `latest_state()`、`wait_for_state()` 和
      `latest_observation()`。
- [ ] 记录 source timestamp、client receive timestamp 和 state age；状态超过上限时禁止计划推进。
- [ ] 处理图像和高频状态共用 socket 的背压。若 JPEG 导致状态延迟不可控，拆分状态 socket，而不是放宽
      stale 阈值。

测试：

- [ ] 协议序列化、版本不匹配和未知消息测试。
- [ ] 图像 11 Hz、joint state 100 Hz 的假 bridge 测试，确认客户端状态缓存按 state_seq 更新。
- [ ] socket 延迟/乱序测试，旧 state 不得覆盖新 state。
- [ ] 状态停止更新时，执行器必须在超时内冻结并进入安全回退。

验收条件：客户端能够持续观察高频关节状态；反馈时效不再由相机频率决定。

### 阶段 2：建立规范化轨迹时间轴

修改范围：

- `src/marvinpro_deploy/motion_profile.py`
- 新增 `src/marvinpro_deploy/trajectory_timeline.py`
- `src/marvinpro_deploy/rollout_client.py`
- `tests/test_motion_profile.py`
- 新增 `tests/test_trajectory_timeline.py`

任务：

- [ ] 用连续 `phase` 轨迹代替预先展开并无条件消费的 100 Hz deque。
- [ ] 定义 A0 的 phase 为 0，A9 的 phase 为 9；100 Hz 只对当前 phase 求值。
- [ ] 将首次 `feedback -> A0` 过渡建模为独立 handoff，不计入 RTC 的 H=10 索引。
- [ ] 所有时间计算使用 monotonic timestamp，避免 `ceil(duration * 100)` 在每个 chunk 累积相位漂移。
- [ ] timeline 支持 `peek/reference_at_phase`、冻结、恢复和在 knot 边界原子替换 future。
- [ ] 每次替换递增 `timeline_version`；过期操作必须失败，而不是静默覆盖。
- [ ] 原始策略 knots、raw reference、filtered target 和反馈分别保存，不能混成一条 action 日志。

测试：

- [ ] phase 0、整数 phase、分数 phase 和末节点求值。
- [ ] 冻结期间任意多次 100 Hz tick 都不能改变 phase。
- [ ] 5 Hz knot rate 在 100 Hz 控制时钟下长期运行不漂移。
- [ ] handoff 不改变 A0...A9 的 RTC 索引。
- [ ] 错误 timeline version 的替换被拒绝。

验收条件：计划进度与控制 tick 完全解耦；不运行 tracking gate 时可复现原插值轨迹形状。

### 阶段 3：实现 tracking-aware phase governor

修改范围：

- 新增 `src/marvinpro_deploy/tracking.py`
- `src/marvinpro_deploy/rollout_client.py`
- `src/marvinpro_deploy/safety.py`
- 新增 `tests/test_tracking.py`
- `tests/test_rollout_client.py`

执行算法：

```text
q_ref = timeline.value(phase)
e_ref = max_arm_abs(q_ref - q_measured)

e_ref <= e_run:                 phase_rate = 1
e_run < e_ref < e_stop:         phase_rate 在 1 到 0 之间平滑下降
e_ref >= e_stop 或 state stale: phase_rate = 0

phase += nominal_knot_hz * phase_rate * dt
```

任务：

- [ ] tracking error 使用高频 joint source timestamp，不使用图像 callback 时刻。
- [ ] 保存命令历史，使反馈能够与其 source timestamp 对应的参考/发送目标比较。
- [ ] 分别计算 `reference_error = raw_reference - measured` 和
      `servo_error = time_aligned_sent_target - measured`。
- [ ] arm clipping 发生时立即冻结 phase；禁止“裁剪当前点后继续消费下一点”。
- [ ] 使用带滞回的 `e_run/e_stop`，避免阈值附近反复启停。
- [ ] 当前阈值为 `e_run=0.02`、`e_resume=0.12`、`e_stop=0.16 rad`，必须通过真机遥测复核。
- [ ] tracking freeze 时保持同一个 raw reference，允许机器人追上；硬故障停止时只锁存一次固定安全目标。
- [ ] 连续 clipping、状态过期或 tracking timeout 进入 synchronized fallback 或停止，不自动放宽阈值。
- [ ] 增加 `--rollout-schedule tracking`，暂时不调用 RTC，只验证执行器。

测试：

- [ ] 一阶慢速机器人仿真：命令速度高于机器人时 phase 必须减速，不能持续跑在反馈前面。
- [ ] 发生 clipping 时 phase 不变；反馈追上后才能继续。
- [ ] 滞回区不会在相邻 tick 抖动。
- [ ] 反馈停更、时间戳回退和大延迟时进入安全状态。
- [ ] 固定 hold 不随变化中的反馈产生棘轮漂移。

验收条件：用冻结 chunk 回放时无持续 arm clipping，计划 phase 曲线能真实反映减速/冻结，末目标跟踪等待
显著短于当前“发送结束后再追赶”的时间。

说明：第一版可以把 governor 放在客户端以减少改动，但只有在高频状态延迟满足控制要求时才允许真机。如果
socket/JPEG 抢占导致状态抖动，则在本阶段把纯 `tracking.py` governor 移到 bridge 定时器中执行，由客户端
上传带版本的 trajectory；不要通过增大 stale timeout 绕过问题。

### 阶段 4：增加物理进度 checkpoint 和正确观测屏障

修改范围：

- `src/marvinpro_deploy/rollout_client.py`
- `src/marvinpro_deploy/tracking.py`
- `tests/test_rollout_client.py`

第一版 checkpoint 流程：

```text
phase 到达指定 s 边界
-> 暂停在该 reference
-> 所有臂关节进入 observation tolerance
-> 连续稳定 observation_settle_seconds
-> 等待 captured_monotonic 晚于稳定时刻的新图像
-> 使用该图像和与其时间对齐的关节状态启动推理
```

任务：

- [ ] 不允许用 `plan_steps_sent` 或队列长度声明 A3 已完成。
- [ ] checkpoint 到位判据使用全部 14 个臂关节；夹爪单独记录，不复用臂关节 rad 阈值。
- [ ] 确认图像和关节状态属于同一可接受时间窗口，超出窗口重新取帧。
- [ ] checkpoint 等待期间保持固定 reference，不使用追随 measured pose 的 hold。
- [ ] 第一版使用现有 `0.01 rad / 0.20 s` synchronized 参数作为保守起点，之后根据高频反馈重新标定。

测试：

- [ ] A3 已发送但反馈仍在 A2 时，不得触发观测。
- [ ] 到位使用旧图像时不得触发推理，必须等待新图像。
- [ ] 某一关节退出 tolerance 后 settle 计时清零。
- [ ] checkpoint 超时进入安全 fallback。

验收条件：日志能够证明每次推理观测都发生在指定物理 checkpoint 之后。

### 阶段 5：先实现 phase-aware 异步调度，不接 RTC guidance

修改范围：

- `src/marvinpro_deploy/rollout_client.py`
- `tests/test_rollout_client.py`

任务：

- [ ] 在 checkpoint 观测后异步调用现有 vanilla policy，同时旧 timeline 在 governor 控制下继续运行。
- [ ] 记录推理开始/结束 phase，而不是只记录 wall time。
- [ ] 定义 `d_actual` 为推理期间跨过的模型 knot 数；100 Hz 包数量不参与 RTC delay。
- [ ] 第一版只允许在整数 knot 边界替换，分数 phase 等待下一个边界并显式记录。
- [ ] vanilla 结果在本阶段不得直接并入真机计划，仅做 shadow mode，计算潜在边界差。
- [ ] 统计 `d_actual` 的 median、p95、max，以及推理期间发生的 freeze/clipping。

验收条件：不改变机器人动作的情况下，能够可靠得到“真实 phase 延迟”和新旧轨迹边界诊断，为 RTC 的
`d_pred` 提供数据。

### 阶段 6：在 OpenPI 中实现原生 JAX RTC sampler

OpenPI 修改范围：

- `/home/jh/OpenPI_UR/openpi/src/openpi/models/pi0.py`
- `/home/jh/OpenPI_UR/openpi/src/openpi/policies/policy.py`
- `/home/jh/OpenPI_UR/openpi/src/openpi/serving/websocket_policy_server.py`
- 对应 OpenPI tests

MarvinPro 修改范围：

- `src/marvinpro_deploy/rollout_client.py`
- 新增 `src/marvinpro_deploy/rtc.py`
- 对应 tests

RTC 请求至少包含：

```text
request_id
plan_id
timeline_version
observation
old_remaining_actions_absolute
predicted_delay_steps
execution_horizon
weight_schedule
guidance_scale
```

任务：

- [ ] 保持 vanilla 请求兼容；RTC 使用带明确版本的 envelope。
- [ ] 旧 prefix 由服务端通过现有 MarvinPro input transforms 转为模型空间：手臂绝对目标先相对本次
      observation 重锚定再 normalize，夹爪保持 absolute 语义。
- [ ] 不在客户端复制 norm stats。
- [ ] 在 flow sampling 每一步加入 VJP guidance。OpenPI 使用 `t=1` 噪声到 `t=0` action 的时间方向：

```text
x0_hat = x_t - t * v_t
error = (old_prefix - x0_hat) * weights
correction = VJP_x(x0_hat, error)
v_guided = v_t - guidance_scale * correction
```

- [ ] guidance scale 必须裁剪；先离线扫描 `{3, 5, 10}`，不直接采用 LeRobot 默认值。
- [ ] 权重支持 hard prefix、指数软过渡和 fresh tail。
- [ ] 强制检查 `d_pred <= s <= H - d_pred`。当前 H=10 第一版使用 `s=4`，只有 `d_pred<=4` 才进入 RTC。
- [ ] `d_pred>=5`、prefix 不足或 shape/transform 不匹配时返回结构化失败，客户端走同步 fallback。
- [ ] 返回 server preprocess、denoise、postprocess 和总延迟，以及 request/timeline IDs。

测试：

- [ ] 相同 observation/noise 下，weights 全零的 RTC 输出与 vanilla sampler 数值一致。
- [ ] 简单线性 denoiser 单元测试验证 VJP 符号，防止照抄相反时间方向的官方实现。
- [ ] absolute -> delta -> normalize 的 prefix round-trip；覆盖左右臂和两个 absolute gripper 维度。
- [ ] H=10、s=4、d=2/3/4 的权重位置测试。
- [ ] guidance 极端值不产生 NaN/Inf。
- [ ] JIT 首次编译和稳态延迟分别记录。

验收条件：离线固定 observation、固定 noise 下，RTC 能降低 prefix/边界误差且不违反关节限位；稳态推理
延迟仍满足 H=10 的可行约束。

### 阶段 7：RTC 与 tracking timeline 集成

修改范围：

- `src/marvinpro_deploy/rtc.py`
- `src/marvinpro_deploy/rollout_client.py`
- `src/marvinpro_deploy/trajectory_timeline.py`
- 集成测试

正确时序示例：

```text
真实跟踪到 A3，采集观测并开始推理
旧计划在推理期间继续 A4、A5
RTC 输出 C0≈A4、C1≈A5、C2... 为过渡和新动作
返回时 d_actual=2，丢弃 C0、C1
在下一个合法 knot 边界从 C2 接管
```

任务：

- [x] `d_pred` 使用当前 estimator epoch 内稳定 latency 的保守 p95 和 `50 ms` guard；不能用
      `wall_ms * 100 Hz`。
- [x] 超过四个 old-tail knot 可行预算或错过物理 deadline 的样本记录为 link fault，不污染稳定分布。
- [x] 默认丢弃迟到结果；保留显式 `wait` 模式，只用于比较停顿、边界跳变和任务恢复率。
- [x] fallback 后重置 estimator epoch；旧异常不会进入新的 RTC epoch。
- [x] 分段记录 observation preparation、client transport、server queue/denoise/decode 和 bridge stage/merge。
- [ ] 推理开始时冻结一份 old remaining reference 和 timeline version。
- [x] 返回后从 bridge 读取按实际 phase knot crossing 累积的 `d_actual`；不要假设它等于 `d_pred`。
- [ ] 仅当 request ID、plan ID、timeline version、反馈新鲜度和 RTC 约束都有效时允许 merge。
- [ ] `d_actual > d_pred`、旧 prefix 耗尽或到达结果时已经越过可替换边界，丢弃结果并 fallback。
- [ ] 第一版中，推理期间任意 arm clipping、tracking hard freeze 或状态过期都使结果失效。
- [ ] 合并采用原子 future replacement，不在旧 raw 尾部 append。
- [ ] 合并后继续受 phase governor 控制；RTC 不能绕过 tracking gate 或 safety filter。
- [ ] 记录 merge 前后的 raw reference、sent target、feedback、位置/速度/加速度边界差。

测试：

- [ ] 慢速假机器人在 A3 命令已发送但只到 A2 时，不允许启动 RTC。
- [ ] 推理期间只跨 1 个 knot 时使用 `d_actual=1`，即使 `d_pred=4`。
- [ ] 推理期间 phase 冻结时结果作废并回到同步路径。
- [ ] 延迟突增、响应乱序、重复响应和旧 timeline 响应全部被拒绝。
- [ ] merge 与 100 Hz publisher 并发时无半更新状态。
- [ ] plan underrun 时固定 hold，不从反馈连续生成新 hold 目标。

验收条件：仿真慢速机器人和保存的真机轨迹回放中，不出现计划持续超前、错误 skip 或 chunk 边界反向跳变。

### 阶段 8：分级验证和真机准入

严格按以下顺序执行，不跳级：

1. 纯单元测试和静态检查。
2. 假 bridge + 可配置一阶慢速机器人仿真。
3. 保存观测、动作和反馈的离线 replay。
4. 真 policy shadow mode：运行 RTC 推理但不采用结果。
5. frozen chunk + tracking governor 单 chunk 真机测试。
6. synchronized 模式回归，确认旧安全路径未退化。
7. RTC 两个 chunk/一个边界的短时真机测试，2.0x playback，操作员随时切 None。
8. 逐步增加边界数量；只有稳定后才比较 1.5x，暂不运行 1.0x。

每一级必须记录：

- state source/receive age 的 median、p95、max。
- inference wall latency 和 phase delay 的 median、p95、max。
- phase rate、freeze 时间和 checkpoint 等待时间。
- raw/sent/measured 三条轨迹。
- arm clipping tick、bridge rejection、plan underrun。
- merge 位置差、速度差、加速度和 jerk。
- 操作员观察到的回弹、停顿及任务结果。

真机立即停止条件：

- 任一 motion gate 关闭或 bridge command rejection。
- 连续 arm clipping，或 tracking error 达到 hard stop 阈值。
- 状态流过期、timeline version 不一致或 RTC 响应无法对齐。
- `d_actual > d_pred`，或 RTC 可行条件被打破。
- 出现已知周期回弹、固定方向漂移或异常加速。

## 6. 第一版推荐参数边界

这些是启动配置，不是最终调优结果：

```text
action_horizon H = 10
playback_time_scale = 3.0
nominal effective knot rate = 5 Hz
checkpoint/execution horizon s = 6
RTC predicted delay d_pred <= 4
tracking observation tolerance = 0.01 rad（先沿用同步基线）
tracking settle = 0.20 s（先沿用同步基线）
```

当前 governor 配置为 `e_run=0.02`、`e_resume=0.12`、`e_stop=0.16 rad`；其中 resume 是 hard freeze 的
解除阈值。这组参数仍必须通过阶段 1 至阶段 6 的真机数据复核。

H=10、s=6 时物理 old tail 只有 4 个 knot，`d_max` 不能继续放大。如果稳定 p95 已经长期消耗 3-4 个 knot，
应降低并稳定链路延迟、增加模型 action horizon 或本地化推理，而不是放宽安全门或伪造更大的 delay budget。

当 `d_pred=4, s=6, H=10` 时，RTC 权重布局是：

```text
[hard, hard, hard, hard, soft, soft, fresh, fresh, fresh, fresh]
```

这里的 4 个 hard 不是观测前已经完成的 A0...A3，而是推理期间最多可能继续执行的旧 future，例如
A4...A7。实际返回后只跳过真实跨越的 `d_actual`。

## 7. 完成判据

只有同时满足以下条件，才能把 RTC 视为完成：

- [ ] 稳态执行无 plan underrun。
- [ ] 正常阶段无 arm clipping；出现异常时 phase 在消费下一节点前冻结。
- [ ] 每次 policy observation 都有可证明的 tracking checkpoint 和新图像时间屏障。
- [ ] 每次 merge 都满足 `d_actual <= d_pred <= 4` 和 timeline version 一致。
- [ ] chunk 边界最大关节位置差显著低于失败样本的 `0.16913 rad`，并接近或优于 synchronized 基线。
- [ ] 不再出现约 1.33 秒周期回弹。
- [ ] RTC p95 稳态延迟不破坏 H=10 的可行性。
- [ ] synchronized fallback 和退出固定 hold 均通过回归测试。
- [ ] 真机任务成功率和完成时间不低于 synchronized 基线；不能只以视觉平滑作为验收。

## 8. 明确不做的事情

- 不在现有按墙钟 `pop()` 的队列上直接加入 RTC。
- 不把 `last_command_id` 的 accepted/published 状态解释为机器人到位。
- 不把 100 Hz 发布 tick 当作 RTC action step。
- 不把 raw old tail 当作 committed prefix，除非期间没有裁剪且 tracking gate 保持有效。
- 不固定跳过 B2/B3 来假装延迟补偿。
- 不靠增大 prefetch 窗口、动作包络或 stale timeout 掩盖问题。
- 不在第一轮同时调整低层控制器、playback 速度、tracking 阈值和 RTC guidance。
- 不为了复用 RTC 而整体迁移到 LeRobot；优先移植必要算法到现有 OpenPI JAX 路径。

## 9. 实际开工时的第一个改动集

第一个改动集只完成阶段 1，不改变动作调度：

1. 在协议中加入高频 `RobotStateUpdate` 并升级版本。
2. bridge 发送不带图像的高频关节状态。
3. `RobotConnection` 同时维护最新 state 和最新 policy observation。
4. 增加状态时延、频率和乱序测试。
5. 运行完整测试，并用只读 doctor/dry-run 记录真实 joint state 频率和 socket 延迟。

得到这组数据后，才能决定 governor 留在客户端还是必须移到 bridge。阶段 1 期间不发送新的真机运动命令。
