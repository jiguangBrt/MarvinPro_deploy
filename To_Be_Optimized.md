# MarvinPro RTC 待优化项分析

## 1. 文档目的与比较边界

本文详细展开当前 `MarvinPro_deploy` 中五类可能降低真机表现的机制：

1. 默认 settled checkpoint；
2. tracking governor 对模型时间语义的改变；
3. `d_pred`、deadline freeze 与短 horizon 的组合；
4. hard anchor 只保证位置连续，不保证速度和加速度连续；
5. RTC 失败后的同步回退。

这里的“表现”至少包含四个彼此不同的维度：

- **安全性**：是否越过关节限位、是否在控制状态异常或反馈失效时继续运动；
- **运动连续性**：是否发生可感知停顿、速度突变、回弹或抽动；
- **策略响应性**：新观测多久能够影响机器人，策略的执行时序是否接近训练数据；
- **任务能力**：堆叠成功率、任务完成时间、抓取或接触状态能否持续保持。

这些维度不能合并成一个“好或坏”的判断。某个机制可能提高安全性，却降低运动连续性；也可能在
异常 episode 中降低任务完成率，但阻止机器人执行已经失效的计划。

同时需要明确：Physical Intelligence 公开的
[real-time-chunking-kinetix](https://github.com/Physical-Intelligence/real-time-chunking-kinetix)
是 RTC 算法参考实现，不是其完整真机低层控制和安全系统。PI 原始 RTC 假设执行器能够按稳定步频执行
旧 action tail，并且在新 chunk 返回前仍有足够 horizon。删除本项目中的真机保护并不能自动恢复这些
假设，因此本文的优化目标是减少保护机制对正常路径的干扰，而不是关闭安全门。

当前固定参数为：

```text
H = 10
s = 6
old tail = H - s = 4 knots
d_pred <= 4
model/data rate = 15 Hz
effective knot rate = 15 / playback_time_scale = 7.5 Hz 或 5 Hz
bridge interpolation/publish rate = 100 Hz
```

相关常量见 [`src/marvinpro_deploy/rtc.py`](src/marvinpro_deploy/rtc.py)，正式真机证据主要来自
[`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md) 和 `logs/rtc_*`。

## 2. 默认 settled checkpoint

### 2.1 当前行为

RTC 轨迹在 phase 到达 execution horizon `s=6` 的 checkpoint 后有两种模式：

- 默认 settled 模式：固定在 checkpoint reference，等待真实关节跟踪到位；
- `--rtc-continuous` 模式：发出 checkpoint 事件后继续执行旧轨迹 tail，不等待 settle。

settled 模式的 checkpoint 只有在全部 14 个手臂关节满足以下条件后才成立：

```text
max_joint_tracking_error <= 0.01 rad
连续稳定时间 >= 0.20 s
joint source timestamp 持续前进
```

如果任一关节重新越过 `0.01 rad`，稳定计时会重新开始。checkpoint 成立后，客户端还要等待一张时间戳
晚于稳定时刻的新图像，并要求图像与关节状态的采样时间差不超过 `50 ms`，之后才构造 RTC 请求。

### 2.2 为什么它有用

settled checkpoint 解决的是“名义 phase 已经到 A5，但机器人实际还在 A4 附近”的物理错位。若直接使用
名义 checkpoint 采图，可能发生：

- RTC old prefix 以 A5 为起点，但真实机器人尚未到 A5；
- 图像中的物体状态、关节 state 和旧 action tail 不对应同一物理时刻；
- 新 chunk 即使数学上与 old prefix 连续，部署到真机时仍可能产生跳变；
- 把跟踪落后误判成策略需要修正，形成错误闭环。

因此 settled 模式很适合首轮真机验收、校验 joint mapping、确认 observation barrier 和定位 merge 错位。
它应被视为验证工具，而不是实时 RTC 的最终调度语义。

### 2.3 已确认的性能损失

2026-08-11 的 settled RTC 五 chunk 真机运行完成了初始 chunk 和 4 次 replacement merge：

- 没有 arm clipping、hard freeze、stale、merge 拒绝或 fallback；
- 4 次边界速度跳变均未产生可感知回弹；
- 但每个 checkpoint 都产生明显周期停顿；
- 第一次从 checkpoint 到恢复约 `1.291 s`；
- 后续三次约 `0.368 s`、`0.471 s`、`0.432 s`。

详见 [`ROBOT_RTC_TESTS.md`](ROBOT_RTC_TESTS.md#2026-08-11-settled-rtc-五-chunk-卡顿调查)。该轮 bridge
平均约 `99.53 Hz`，最大 timer gap 约 `25 ms`，不足以解释 `0.37-1.29 s` 的停顿。根因是 checkpoint
状态机的固定等待，而不是网络推理、100 Hz publisher 或 RTC action 对齐错误。

对任务的潜在影响包括：

- 周期停止破坏 PI RTC 原本“旧 chunk 持续执行、新 chunk 异步生成”的核心收益；
- 新观测频率从策略需要的闭环频率退化为“到位时间 + 0.20 s settle + 图像等待 + 推理时间”；
- 接触任务中，暂停可能改变物体受力、夹爪接触和摩擦状态；
- 对堆叠任务而言，机器人可能在尚未完成一次连续操作时反复停顿；
- 运动看起来更稳定，但任务完成时间和策略纠错速度都会变差。

### 2.4 优化方向

1. **将 continuous RTC 作为性能评估和最终运行语义。** settled 模式保留为显式验收模式，不应作为与
   PI 原始 RTC 比较运动连续性的默认配置。
2. **保留 continuous checkpoint 的实测状态采样。** continuous 不等于使用名义 state；仍应在 phase
   越过 checkpoint 时记录最新关节状态、图像边界和 old tail。
3. **观测等待必须计入 `d_actual`。** 等待下一张新图期间旧轨迹继续前进，跨过的整数 knot 必须计入
   真实 delay，当前实现已经具备这一行为，不应退回 wall-time 猜测。
4. **settled 只用于诊断。** 例如校验某个新 checkpoint、joint mapping、相机时间戳或新控制器参数时，
   可短期启用；验证通过后回到 continuous。
5. **不要通过降低 settle 时间掩盖模式问题。** 将 `0.20 s` 改成 `0.05 s` 只能缩短停顿，不能恢复
   无停 RTC 语义，也可能让噪声或偶然到位被误判为稳定。

### 2.5 验收指标

- continuous checkpoint 前后 phase 单调，不出现 `frozen=checkpoint`；
- checkpoint 到 `rtc_resumed` 期间跨过的 knot 与 `d_actual` 一致；
- 每次使用的图像晚于 checkpoint，state/image skew `<=50 ms`；
- 正常延迟下 checkpoint freeze time 为 0；
- 在相同初始状态下，连续模式的任务完成时间和成功率不低于 settled 模式；
- settled 模式仅在显式诊断命令中出现，不作为正式性能数据。

## 3. Tracking governor 改变模型时间语义

### 3.1 当前行为

tracking governor 根据当前 raw trajectory reference 与 14 个真实关节反馈之间的最大误差控制 phase rate：

```text
error <= 0.01 rad:
    phase_rate = 1

0.01 < error < 0.04 rad:
    phase_rate = (0.04 - error) / (0.04 - 0.01)

error >= 0.04 rad:
    hard freeze

hard freeze 后，error <= 0.03 rad 才允许恢复
```

此外，joint state stale、trajectory timer overrun 和 arm safety clipping 也会触发 hard freeze。代码见
[`src/marvinpro_deploy/tracking.py`](src/marvinpro_deploy/tracking.py) 和
[`src/marvinpro_deploy/robot_bridge.py`](src/marvinpro_deploy/robot_bridge.py)。

当前 RTC 又强制使用：

```text
model_hz = 15
playback_time_scale = 2 或 3
```

因此在 governor 介入之前，模型 knot 已经只按 `7.5 Hz` 或 `5 Hz` 执行。governor 的 phase rate 再乘到
这个有效 knot rate 上。例如 time scale 3 且 `phase_rate=0.2` 时，瞬时有效执行速度只有 `1 Hz`。

### 3.2 为什么它有用

RTC 和 action chunking 默认旧轨迹能够被执行器大致按时跟踪。真机若明显落后而时间轴仍固定前进，会发生：

- 控制 reference 越来越远离真实关节；
- 后续 knot 相对真实反馈超过安全包络，触发连续 clipping；
- `d_actual` 只反映软件消费了几个 knot，而不反映机器人完成了多少运动；
- checkpoint observation 与 RTC prefix 的物理状态严重错位；
- 在最坏情况下，控制器追赶快速移动的 reference，产生大速度、冲击或振荡。

governor 通过“机器人追不上就减慢模型时间”维持 reference 与真机之间的局部一致性。对于当前 MarvinPro
控制链，它不是冗余功能。已有 synchronized 失败运行出现 `0.27883 rad` peak tracking error 和 228 个 arm
clipped tick，说明直接按固定速度消费动作在当前配置下确实可能失控。

### 3.3 已确认的性能损失

真机 tracking 记录中，误差从 `0.0171 rad` 上升到约 `0.0356 rad` 时，phase rate 从 `0.7627` 降到
`0.1479`，随后随误差降低恢复到约 `0.91`。操作者会把这种连续降速感知为迟滞或“动作发软”。

主要问题不是减速本身，而是它改变了策略的时间分布：

- 训练数据为 15 Hz，action knot 表示固定时间间隔下的动作序列；
- 当前先整体慢放到 7.5/5 Hz，再根据跟踪误差动态改变间隔；
- 相同的 10 个 knot 在不同 episode、不同关节误差下可能用完全不同的物理时间执行；
- 视觉观测、夹爪闭合和手臂接触之间的相对时序被改变；
- 策略看到的是慢放后产生的环境状态，而训练时可能没有这种时间伸缩；
- 动态减速虽然减小 tracking error，却可能降低闭环纠错频率和任务完成速度。

这类影响在静态到位任务中通常可接受，但在抓取、接触、堆叠、双臂协同等依赖时间关系的任务中可能更
明显。尤其当前夹爪实际反馈不可用、policy state 使用 command proxy 时，动态慢放并不能保证物理夹爪已经
按相同节奏完成动作。

### 3.4 优化方向

1. **将“正常运行速度”和“异常安全冻结”解耦。** hard stop、stale、timer overrun、clipping 保护继续
   保留；正常 tracking 区间尽量使用一个经过真机验证的固定 knot rate。
2. **先找到真机可稳定跟踪的固定速率。** 用 frozen chunk 或固定 action 计划测试 5、7.5、10、15 Hz，
   分别记录 tracking p50/p95/max、clipping、关节速度和操作者感知，不能直接假设 15 Hz 一定可行。
3. **减少持续时间扭曲。** 可以研究更窄的 soft slowdown 区间、带低通和滞回的 governor，或者只在误差
   接近硬阈值时显著降速，避免 phase rate 在每个 100 Hz tick 高频变化。
4. **不要简单放宽 `0.04 rad` hard stop。** 若误差经常接近 stop 阈值，应优先检查轨迹速度、控制器带宽、
   joint impedance 参数和网络/定时，而不是让 reference 离真机更远。
5. **将 phase-rate 分布作为任务指标。** 如果一个成功 episode 大部分时间都在 `phase_rate<0.5`，它不应
   被视为与固定周期 PI RTC 等价。
6. **策略层和执行层需要统一时间定义。** 长期方案应考虑让模型或 action representation 显式知道实际
   执行步长，或者使用与机器人可实现速率一致的数据训练，而不是永久依赖部署端时间拉伸。

### 3.5 验收指标

- 无 hard freeze、state stale、timer overrun 和 arm clipping；
- `phase_rate=1` 的运行时间占比、p5/p50/p95 明确记录；
- tracking error p95 稳定低于 soft slowdown 区域上端；
- 相同固定轨迹多次运行的物理完成时间方差受控；
- 抓取和堆叠任务在固定可行 knot rate 下的成功率不低于动态 governor；
- 任何 governor 参数变化先 shadow/固定轨迹测试，不直接用复杂任务探索安全边界。

## 4. `d_pred`、deadline freeze 与短 horizon

### 4.1 当前行为

客户端的延迟估计器保存最近 20 次 RTC wall latency，并按下式预测：

```text
d_pred = ceil((max(last_20_wall_latency) + 0.05 s) * effective_knot_hz)
```

结果必须满足 `1 <= d_pred <= 4`。没有稳定延迟样本，或者预测超过 4 步时，RTC 直接失败并进入回退。

当前每次 checkpoint 后只剩 4 个 old-tail knot。推理开始后 bridge 继续推进旧轨迹，并按真实跨过的整数
knot 计算 `d_actual`。如果新结果尚未到达，而 `d_actual` 达到 `d_pred`，bridge 会：

```text
phase_rate = 0
pause_kind = rtc_deadline
固定当前 reference
等待 RTC 结果
```

如果结果提前返回，则先 stage，在下一个整数 knot 边界原子 merge；如果结果晚于预测边界，则真机会在
deadline 上保持，结果返回后仍可 merge。

### 4.2 为什么它有用

这组机制保护 RTC guidance 的前提。服务端生成新 chunk 时，`d_pred` 决定前多少个 old-prefix action 应被
强约束。若实际执行远远越过该区域还继续接管，新 chunk 与旧轨迹的连续性保证会减弱。

取最近历史最大延迟而不是均值，可以降低低估概率；`50 ms` guard 可以覆盖图像、序列化、调度和网络的
小抖动；deadline freeze 则阻止时间轴消费掉所有 old tail。其安全目标是合理的。

### 4.3 已确认的性能损失

2026-08-12 的 protocol v5 真机运行中：

- 前 9 次 RTC wall latency 约为 `218.7-477.5 ms`；
- 第 10 次突增到 `1560.1 ms`，但请求发出时仍使用此前的 `d_pred=3`；
- bridge 到达三步边界后进入 `rtc_deadline`，等待迟到结果；
- 该结果最终以 `d_actual=3` merge；
- `1560.1 ms` 被写入历史最大值后，下一次 `predicted_steps()` 得到 9；
- 9 超出允许范围 1..4，RTC 对本 episode 永久关闭并进入 synchronized fallback。

原始证据见
[`logs/rtc_s6_proxy_v5_merge20_20260812_101928/rollout.log`](logs/rtc_s6_proxy_v5_merge20_20260812_101928/rollout.log)。

这个行为带来四类损失：

1. **可感知 deadline 停顿。** late result 到达前，机器人固定在预测边界；一次网络尖峰可直接变成约一秒
   量级的停顿。
2. **历史样本污染。** 一个极端值会在最多后续 20 次预测中一直作为最大值；但当前实现第一次发现
   `d_pred>4` 就退出 RTC，实际上没有机会通过新的稳定样本自然淘汰它。
3. **短 horizon 没有恢复空间。** `H-s=4` 使 5 Hz 下最多约 `0.8 s`、7.5 Hz 下最多约 `0.53 s` 的
   nominal tail 可用。稳定 wall latency 已达数百毫秒，余量很小。
4. **wall latency 混合多种来源。** server denoise、远程排队、图像/网络传输、客户端线程调度被合并成一个
   数值。即使模型计算稳定，链路尖峰也会让下一次 RTC 不可用。

PI RTC 同样使用历史延迟的保守估计；当前问题不是“使用最大值”这一点本身，而是最大值、额外 guard、
固定四步 tail、远程 WebSocket 和 episode-latched fallback 叠加后形成了较脆弱的系统。

### 4.4 优化方向

1. **先治理延迟源，不先放宽安全条件。** 分别记录 observation preparation、request serialization、网络
   往返、server queue、denoise、response decode、bridge stage/merge 时间，确定 `1.56 s` 的来源。
2. **区分稳定预算和异常尖峰。** 正常 `d_pred` 可基于保守稳定分布；超过 RTC horizon 的尖峰应作为链路
   fault 处理，而不是把一个严重 outlier 当成接下来所有请求的正常预算。
3. **迟到结果需要明确策略。** 可比较两种方案：在 deadline 等待并接受结果；或到 deadline 后丢弃该结果，
   继续安全 old tail/进入受控重同步。不能只比较是否报错，还要比较停顿、边界跳变和任务恢复率。
4. **fallback 后重置或隔离 estimator epoch。** 如果系统已经 hold 并重新观测，旧异常 latency 是否仍应
   影响新的 RTC epoch，需要明确设计。不能静默忽略异常，也不能让一次异常永久污染整个 episode。
5. **评估 horizon 设计。** 如果稳定 p95 wall latency 已经消耗 3-4 个 knot，`H=10,s=6` 的工程余量不足。
   长期方案可能需要更长 action horizon、更低且稳定的链路延迟，或本地化推理。不能仅把 `d_max` 从 4
   改大，因为当前 old tail 物理上只有 4 个 knot。
6. **保持 `d_actual` 物理计数。** 优化 estimator 时不应退回 `wall_time * nominal_rate`；governor 会改变
   phase rate，只有 bridge 的实际 knot crossing 才代表执行进度。

### 4.5 验收指标

- 分别报告 client wall、server total、server infer 和 transport residual 的 p50/p95/p99/max；
- 正常运行 `d_pred<=4` 的 episode 比例和 fallback rate；
- deadline freeze 总时长、单次最大时长和每 100 次 merge 的触发次数；
- `d_actual<=d_pred` 始终成立；
- 对注入的 0.5/1.0/1.5 s 延迟尖峰，系统行为确定、可重复且不会采用过期事务；
- 链路恢复后是否允许建立新 estimator epoch，由明确测试验证；
- 不以删除 guard 或忽略 outlier 作为通过标准。

## 5. Hard anchor 只保证位置连续

### 5.1 当前行为

RTC 结果返回后，bridge 不直接从新 chunk 的预测 action 接管。它先取得当前旧轨迹的 raw reference，并在
replacement chunk 中强制覆盖：

```text
merge_phase = d_actual - 1
C[merge_phase] = current_old_reference
```

之后新的 timeline 从 `merge_phase` 开始，100 Hz bridge 在新 knot 之间继续线性插值。实现见
[`src/marvinpro_deploy/trajectory_timeline.py`](src/marvinpro_deploy/trajectory_timeline.py) 和
[`src/marvinpro_deploy/robot_bridge.py`](src/marvinpro_deploy/robot_bridge.py)。

bridge 会计算 merge 前后的离散速度与加速度跳变：

```text
v_old = old_anchor - old_previous
v_new = new_next - old_anchor
velocity_jump = max(abs(v_new - v_old))
```

但这些值目前只写入 telemetry，不参与 merge 接受或拒绝。

### 5.2 为什么它有用

即使 RTC guidance 已对 prefix 做软约束，数值采样、beta clipping、动作变换和实际 delay 都可能使新 chunk
接管点与旧 reference 存在小偏差。hard anchor 能保证接管瞬间的位置参考严格连续，即离散 C0 连续：

```text
position_before_merge == position_at_new_timeline_start
```

这直接消除了最容易被真机感知的目标位置瞬跳。settled 五 chunk 测试中没有可感知边界回弹，说明该机制
在位置层面有实际价值。

### 5.3 已确认的性能损失

只覆盖一个 anchor 会改变新 chunk 原本的局部形状，却不调整下一点和下下点。因此可能出现：

- C0 连续，但 C1 速度不连续；
- 下一 knot 为追赶模型原预测而产生较大步长；
- 100 Hz 线性插值只能平滑位置采样，不能消除 knot 边界的速度折点；
- deadline hold 越久，旧轨迹速度降为零，而迟到 chunk 的下一点仍可能带有明显运动趋势；
- 位置看似不跳，但机器人在接管后一个 knot 内突然加速，表现为抽动或冲击。

在 `1560.1 ms` 延迟尖峰对应的 merge 中，系统接受了：

```text
boundary_velocity_jump_rad = 0.1075706529 rad/knot
boundary_acceleration_jump_rad = 0.0625047561 rad/knot^2
```

该次 merge 前 bridge 处于 `rtc_deadline`，因此旧参考已经固定。虽然接管位置完全连续，但新 chunk 的下一
knot 与静止状态之间存在明显速度变化。当前没有阈值阻止该 replacement。

还需要注意：`rad/knot` 会随 effective knot rate 转换成不同的物理速度。在 5 Hz 下，
`0.10757 rad/knot` 对应约 `0.538 rad/s` 的离散速度变化量；在 7.5 Hz 下会更大。因此只比较
`rad/knot` 还不足以评价真机冲击。

2026-08-17 的 10-merge soak 真机测试进一步提供了支持证据。该轮前 8 次 RTC replacement 成功，第 9 次
因链路长尾迟到而进入 fallback；操作员报告整轮长任务存在明显抽动和卡顿。8 次成功 merge 的最大边界指标为：

```text
boundary_velocity_jump_rad = 0.09202511599 rad/knot
boundary_acceleration_jump_rad = 0.05314748920 rad/knot^2
effective knot rate = 5 Hz
等效速度变化量约为 0.460 rad/s
```

同一轮另外两次较大的速度跳变约为 `0.0440` 和 `0.0429 rad/knot`；作为对照，现场无异常的 2-merge
短测试约为 `0.0143` 和 `0.0135 rad/knot`。soak 中没有 arm clipping，bridge 平均仍约 `99.49 Hz`，最大
timer gap 约 `30.1 ms`，最大 tracking error 约 `0.0413 rad`。因此抽动与 merge 边界导数突变一致，不能
归因于 100 Hz publisher 停顿或关节目标被 clipping。证据见
[`logs/rtc_v7_soak10_20260817_114935/rollout-soak10.log`](logs/rtc_v7_soak10_20260817_114935/rollout-soak10.log)。

该轮后半段的周期卡顿是另一条故障链：第 9 个请求 wall latency 达到 `372.7 ms`，其中 transport residual
约 `272.3 ms`，而 server denoise 仍约 `89.3 ms`。结果错过 `d_pred=2` 后被正确丢弃，随后 synchronized
fallback 的 settle/observe/infer 周期产生停顿。C1 blend 只能治理成功 merge 的抽动，不能替代第 4、6 节的
延迟和 fallback 治理。

### 5.4 C0、C1 与 C2 的含义

本项目下发的是绝对关节位置。对某一关节，设旧轨迹在接管边界前后为：

```text
旧轨迹：... q[k-1] ---- q[k] |
新轨迹：                    | q'[k] ---- q'[k+1] ...
```

离散 C0 位置连续只要求：

```text
q'[k] = q[k]
```

离散 C1 还要求边界两侧的一阶差分一致：

```text
q'[k+1] - q'[k] = q[k] - q[k-1]
```

C2 则进一步要求二阶差分，也就是离散加速度，在边界两侧一致。对于当前 5 Hz knot，`rad/knot` 乘以
`5` 得到对应的 `rad/s`；二阶差分需要再乘以 `5^2` 才能转换为 `rad/s^2`。

这三者不是互斥选项：C1 按定义包含 C0，C2 又包含 C1。不能通过允许位置不连续来换取速度连续。如果
`q'[k] != q[k]`，绝对位置参考在同一时刻发生阶跃，连续时间导数在该点不存在；100 Hz 位置控制、governor
或驱动器只会把这一步骤变成一次受限但仍然很快的追赶。即使新 chunk 内部的
`q'[k+1] - q'[k]` 恰好等于旧速度，也不能消除接管瞬间的位置冲击。

当前 piecewise-linear playback 在每对 5 Hz knot 之间生成 100 Hz 线性位置命令。它可以让位置采样变密，
但每个 knot 处的线段斜率仍可能突变。要在接管边界获得真正的 C1，必须让进入 blend 的旧速度和离开 blend
的新速度相等；若还要限制冲击，应进一步约束加速度和 jerk。

### 5.5 RTC 原论文与公开实现的处理方式

Physical Intelligence 的
[RTC 原论文](https://arxiv.org/abs/2506.07339)把异步 chunk 接管建模为 flow/diffusion inpainting。
设预测 horizon 为 `H`、本轮执行 horizon 为 `s`、推理延迟为 `d`，其 prefix guidance 分成三段：

```text
index < d       : weight = 1，推理期间确定会执行的动作被强约束
d <= index < H-s: weight 从 1 向 0 逐渐衰减，形成 soft overlap
index >= H-s    : weight = 0，允许新观测自由决定未来动作
```

新结果返回后，执行器跳过推理期间已经过去的 `new[:d]`，从 `new[d:]` 接管。论文的真实机器人配置是
`H=50`、`s_min=25`、`beta=5`、delay buffer `b=10`，目标控制频率为 `50 Hz`；因此预测 horizon 约为
`1 s`，并有约半个 chunk 可用于跨 chunk 过渡。参数和控制频率见
[论文 PDF 的 Appendix A.5](https://www.pi.website/download/real_time_chunking.pdf)，模拟参考代码见
[real-time-chunking-kinetix](https://github.com/Physical-Intelligence/real-time-chunking-kinetix)。

Hugging Face 的 [LeRobot RTC 文档](https://huggingface.co/docs/lerobot/en/rtc)采用同一思路。其
[`ActionQueue.merge`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/rtc/action_queue.py)
直接用 `new_actions[real_delay:]` 替换未执行队列；
[`RTCProcessor`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/rtc/modeling_rtc.py)
在 denoise 阶段对旧 prefix 施加 soft weights。公开代码没有额外的 cubic、quintic 或 Ruckig C1 拼接器。

由论文和代码可以得出一个重要边界：标准 RTC 通过多个相邻位置 action 的一致性，间接降低速度和加速度
突变，但没有给出“接管处一阶导数严格相等”的数学保证。它是生成阶段的经验平滑，而不是执行端的硬 C1
约束。当前项目增加 hard anchor 能把数值 C0 误差压到零，但无法弥补 soft overlap 太短或 guidance 不足。

工业轨迹系统通常把这一问题作为独立的 trajectory blending 层处理。例如
[MoveIt Pro 的轨迹拼接](https://docs.picknik.ai/how_to/robotics_applications/stitch_trajectories/)使用 Ruckig
在连接点附近生成满足速度、加速度和 jerk 限制的过渡段，并要求输入 waypoint 具有 position、velocity 和
acceleration。这类方法可作为 RTC 之后的执行安全层，但会修改策略轨迹，因此仍需重新做限位、碰撞和任务
准确性验证。

### 5.6 当前短 horizon 的定量诊断

当前 OpenPI server 使用官方指数 soft-mask 公式，见 OpenPI
[`src/openpi/models/pi0.py`](../../OpenPI_UR/openpi/src/openpi/models/pi0.py)。对本次 soak 的常见参数：

```text
H = 10
s = 6
prefix attention horizon = H - s = 4
d_pred = d_actual = 2
```

对应的 action 维度权重约为：

```text
action index   0       1      |    2       3     | 4 ... 9
RTC weight     1.000   1.000  |  0.368   0.077   | 0
                               ^
                         merge 后第一个新动作
```

index 0、1 在推理期间已经由旧轨迹执行；`d_actual=2` 时，bridge 把当前旧 reference 写入 replacement 的
index 1，然后向新 chunk 的 index 2 插值。index 2 恰好只有约 `0.368` 的旧 prefix 权重，index 3 又迅速
降到约 `0.077`。因此只剩两个很弱的过渡点，无法像论文 `H=50` 的长 overlap 那样逐步改变方向。此次
`0.092 rad/knot` 的速度跳变与这一结构相符。

LeRobot 不覆盖 index 1，但它在接管前最后执行的同样是旧 index 1，随后下发的也是新 index 2。因此删除
本项目 hard anchor 不会自动恢复 C1；它只会重新引入新旧 index 1 的数值位置误差。真正需要处理的是
`old[1] -> new[2]` 的边界斜率，以及后续若干点如何平滑回到模型轨迹。

把 action chunk 改成 50 可能显著增加 soft overlap，因此可能缓解该问题，但不能只修改客户端参数：

- 当前 `pi05_marvinpro_red_cones` checkpoint 明确按 `action_horizon=10` 训练；50 点输出需要重新训练或至少
  针对新 horizon 微调，并同步修改 policy metadata、prefix shape、`s`、`d_max`、bridge 和测试协议；
- 原论文 50 点运行在 50 Hz，物理 horizon 约 1 秒；本项目当前 effective knot rate 是 5 Hz，直接使用
  50 点会得到 10 秒物理 horizon；
- 若照搬 `s=25`，两次新观测影响执行之间会相隔数秒，可能破坏接触和抓取阶段的响应性；
- 若保持 `s=6`，虽然 overlap 很长，但每轮实际执行的前几个新点仍被旧 prefix 强约束，新观测可能很难在
  下一 checkpoint 前产生足够影响；
- 更长 horizon 不能自动修复错误的 `d_pred`。本次 `372.7 ms` 延迟应使用 `d_pred=3`，现有 H=10 已可容纳；
  若 estimator 仍预测 2，即使 H=50 也会在边界 2 按 discard 策略拒绝结果。

因此 `H=50` 应视为需要重新确定时间尺度的训练实验，而不是部署参数修复。一个较保守的首轮 A/B 候选是
`H=20、s≈10`：在 5 Hz 下对应约 4 秒预测 horizon，并将 `d=2` 后的 soft transition 从当前 2 点扩展到
约 8 点。这个参数只是待验证假设，必须用数据集 episode 长度、任务响应性、推理延迟和真机跟踪共同定标。

### 5.7 优化方向

1. **禁止用位置不连续换速度连续。** C0 是绝对位置控制的最低接管条件；任何 post-processing 都必须保留
   `q_new_start == q_old_current`。
2. **增加执行端 C1/C2 blend。** 以当前旧轨迹的 `q/v`（必要时包括 `a`）为起点，以新 chunk 中未来
   2-3 个 knot 的 `q/v` 为终点，在 100 Hz 上生成 cubic Hermite、quintic 或 jerk-limited 过渡段。窗口长度
   应根据剩余 horizon 和动态限制自适应，不能固定假设总有 3 个可用 knot。
3. **把边界指标升级为 merge 准入和 blend 可行性条件。** 原始 replacement 或 blend 后轨迹超过 position、
   velocity、acceleration、jerk、URDF 或 tracking envelope 限制时，不应直接 merge；应继续安全 old tail、
   固定 hold 或进入明确的 fallback。
4. **对 blend 后的每个 100 Hz sample 重新验证。** 不能只检查 5 Hz knot；finite、URDF limit、速度、加速度、
   jerk、相对当前反馈包络、timeline version 和事务 ID 均不能被平滑器绕过。
5. **deadline merge 使用独立规则。** 如果机器人已经在 deadline 静止，起点速度应按零或可靠反馈估计，使用
   从静止重新启动的轨迹；不能把正常运动中的旧斜率套到 hold 后的 merge。
6. **先对 H=10 的 guidance 做离线消融。** 比较 exponential/linear schedule、`beta` 和更早 checkpoint，
   检查它们能否降低 `new[d]` 的位置及速度偏差。增强 guidance 可以缓解，但不能替代执行端 C1 保证。
7. **研究 phase-aware checkpoint。** [PACE](https://arxiv.org/abs/2606.00537)根据预测 chunk 的低速阶段动态
   选择执行 horizon。优先在低速、非接触边界 replan/merge 可降低冲击，但它是 C1 blend 的补充，不是替代。
8. **将长 overlap 作为独立训练路线。** 从 `H=20、s≈10` 开始，与 H=10 在相同任务和物理时间尺度下 A/B；
   只有确认响应性、推理成本和数据覆盖后再评估 H=50。
9. **若已决定重训，评估 learned continuation。** 原 RTC 作者后续的
   [Training-Time Action Conditioning](https://arxiv.org/abs/2512.05964)在训练时模拟 delay 并直接条件化 action
   prefix；[REMAC](https://arxiv.org/abs/2601.20130)则用 masked action chunk 和 prefix-preserved sampling
   同时处理 inter-chunk discontinuity 与 observation/action mismatch。这些方案比部署端不断加 heuristic 更
   接近模型原生 continuation，但训练和验证成本更高。
10. **在物理单位下评价。** 同时记录 `rad/knot`、`rad/s`、`rad/s^2`、jerk 以及真实反馈导数，不以
    `0.08/0.12 rad` 位置包络代替平滑性指标。

### 5.8 验收指标

- merge position jump 恒为 0 或在数值容差内；
- velocity/acceleration jump 的 p95/max 均有明确上限；
- blend 起点和终点分别满足 C0/C1；若宣称 C2，则加速度也必须在数值容差内连续；
- 100 Hz blend sample 的速度、加速度和 jerk 均不超过已确认的真机限制；
- deadline merge 单独统计，不与正常 staged merge 混合；
- blend 后所有 knot 仍通过 URDF、finite 和安全包络检查；
- 对 H=10、候选长 horizon 和不同 soft-mask 分别报告 `new[d]` 的位置、速度及加速度偏差；
- 在相同物理时间尺度下比较任务响应延迟，不能只用 knot 数判断 H=20/H=50 更平滑；
- 100 Hz command、真实 joint feedback 和 raw reference 三者在 merge 前后同时画图检查；
- 操作者盲测和任务成功率验证不能被“位置不跳”单一指标替代。

## 6. RTC 失败后的同步回退

### 6.1 当前行为

当前 RTC 主循环中，只要请求、响应、事务、delay、tracking 或 merge 任一步抛出异常，就会对本 episode
锁存关闭 RTC：

```text
RTC exception
-> bridge 固定 hold
-> 等待全部手臂关节跟踪到 hold reference
-> 连续稳定 0.20 s
-> 等待稳定时刻之后的新图像
-> synchronized policy inference
-> 执行完整同步 chunk
-> 到 chunk 末端再次 settle、采图、推理
-> 本 episode 不再恢复 RTC
```

代码见 [`src/marvinpro_deploy/rollout_client.py`](src/marvinpro_deploy/rollout_client.py)。当前工作树还对同步
fallback 中的 observation-lag rejection 增加了重新观测并重推理逻辑，避免直接采用已经过期的 action。

### 6.2 为什么它有用

RTC 异常后，客户端可能无法证明以下条件仍成立：

- 返回 result 属于当前 plan/timeline/checkpoint；
- `d_actual` 没有超出 guidance 使用的 `d_pred`；
- 推理期间没有 clipping、hard freeze、stale 或控制状态变化；
- old tail 仍有足够 future 可以安全 replacement；
- 当前图像与真机状态仍是可用于新推理的同步观测。

固定 hold 后重新确认真实状态，是从“不确定的异步事务”回到“已知物理状态”的最直接办法。episode-latched
fallback 还能防止 RTC 在同一故障条件下反复重试、反复 merge 和反复失败。

### 6.3 已确认的性能损失

synchronized fallback 本质上放弃了实时 chunking：每个 chunk 执行完后都等待跟踪、settle、新图像和推理。
已有 synchronized 真机调查确认 chunk 间会产生明显空档，增大 playback time scale 只能减少 clipping，不能
消除同步等待。

2026-08-12 的长运行在第 10 次 RTC merge 后，因为历史 latency 推导出 `d_pred=9` 而进入 fallback：

- bridge 立即固定 hold；
- 后续 synchronized inference wall time 包括 `228.6 ms`、`350.1 ms`、`352.4 ms` 和 `1194.7 ms`；
- 最后一次推理完成时，源 observation 已落后 17 帧，超过限制 8，旧版本运行因此 abort；
- 当前工作树会针对这一特定 observation-lag 情况重新观测和推理，但仍会产生额外 hold 和任务延迟。

对任务的影响可能比单次 merge 失败更严重：

- 抓取或接触动作在 hold 时改变物体受力；
- 双臂协作的连续动作被切断；
- 任务闭环频率下降到完整 chunk 周期；
- 即使网络恢复，本 episode 仍不能回到 RTC；
- 若把 fallback 完成的动作计入 RTC 成功，会掩盖 RTC 的真实可用率。

### 6.4 优化方向

1. **继续保留固定 hold 作为安全起点。** 不应在事务不确定时直接采用迟到 result，也不应自动放宽 ID、
   version、stale 或 delay 条件。
2. **细分失败类型。** 将故障至少分为 policy/server、transport、delay budget、tracking/safety、transaction、
   observation freshness 和本地软件错误。不同故障不一定需要相同的 episode-latched 策略。
3. **定义可恢复 RTC epoch。** 对纯 transport 尖峰，在 hold、重新同步观测、清空旧请求并建立新
   session/timeline epoch 后，研究是否可以安全恢复 RTC。tracking hard stop、clipping 或控制模式异常则应
   保持更严格的 latched fallback。
4. **回退过程避免连续同步停顿。** 可以研究重新初始化一个安全的异步 timeline，而不是永久使用完整
   synchronized chunk 循环；前提是新 epoch 的观测、ID 和延迟预算全部重新建立。
5. **回退结果单独计分。** 实验中至少区分：RTC-only 完成、RTC 后恢复完成、synchronized fallback 完成、
   安全 abort 和任务失败。
6. **设置恢复次数上限。** 即使实现自动恢复，也不应无限循环。连续链路 fault 或 tracking fault 应转入
   hold 并要求操作员处理。
7. **先解决根因再提高恢复复杂度。** 如果 transport 仍出现 1 秒以上尖峰，增加复杂恢复状态机可能只是
   隐藏基础设施问题。

### 6.5 验收指标

- 每类 fallback reason 有独立计数和完整事务上下文；
- fault 到固定 hold 的时间有上限，且 hold action 与当前 reference 一致；
- fallback 后使用的第一张图像和 action 都来自新同步 epoch；
- 不采用旧 request、旧 timeline version 或过期 observation 的 result；
- 分别报告 RTC-only success rate、recovered success rate 和 fallback rate；
- 注入 transport timeout、stale image、ID mismatch、hard freeze、clipping 时行为可重复；
- 自动恢复次数受限，安全 fault 不被错误归类为可恢复网络 fault。

## 7. 优化优先级

建议按以下顺序推进，避免同时改变过多控制变量：

| 优先级 | 项目 | 原因 |
|---|---|---|
| P0 | continuous RTC 作为正式性能基线 | 默认 settled 已确认产生 0.37-1.29 s 周期停顿 |
| P0 | 分解并治理 transport/wall-latency 尖峰 | 1.56 s 尖峰会触发 deadline hold 和 episode fallback |
| P1 | 对 hard anchor 增加 C1/C2 连续性与准入阈值 | 2026-08-17 soak 在 `0.09203 rad/knot` 时出现明显抽动，历史上还接受过 `0.10757` |
| P1 | 恢复足够的 RTC soft overlap，并 A/B 新 horizon | 当前 H=10/s=6/d=2 在接管后的权重仅约 `0.368, 0.077` |
| P1 | 找到固定可跟踪 knot rate，减少持续 phase time-warp | 当前 5/7.5 Hz 之上仍会动态降速 |
| P2 | 将 fallback 故障分类并研究安全的新 RTC epoch | 当前任一异常都会永久退化为 synchronized |

以下机制不应作为性能优化目标直接删除：

- finite/shape/URDF limit 校验；
- input mode、robot state、arm state readiness gate；
- joint state freshness、heartbeat 和 timer overrun 检查；
- session/plan/timeline/checkpoint/request ID；
- `d_actual` 的物理 knot crossing 计数；
- state/image 时间屏障；
- hard stop 和 arm clipping 后的 RTC invalidation。

## 8. 最终 A/B 验证要求

只有在相同机器人、策略 checkpoint、初始姿态、物体布局和任务条件下，才能判断优化是否改善 PI 风格 RTC
表现。建议至少比较：

```text
A: 当前 continuous RTC
B: continuous RTC + 优化后的稳定延迟链路
C: B + derivative-aware merge
D: C + 经过重训和离线验证的长 overlap RTC
E: D + 优化后的 fixed-rate/soft governor
```

每个配置至少记录：

- 任务成功率和完成时间；
- RTC merge 数、fallback rate 和安全 abort rate；
- inference/transport latency p50/p95/p99/max；
- `d_pred/d_actual` 分布和 deadline freeze 时间；
- phase rate 分布、tracking error 和 clipping；
- merge position/velocity/acceleration jump；
- 100 Hz reference、sent target 与真实关节反馈；
- 操作者对停顿、抽动和回弹的盲评。

在这些数据完成前，可以确认 settled、动态时间伸缩、deadline freeze、单点 anchor 和同步 fallback 会降低某些
性能指标，但不能客观宣称当前系统的堆叠成功率整体低于或高于 PI 原始 RTC。

## 9. 本次 RTC 连续性调研资料

以下资料于 2026-08-17 核对。优先列出论文作者、官方代码和机器人轨迹工具的原始资料：

- Physical Intelligence, [Real-Time Execution of Action Chunking Flow Policies](https://arxiv.org/abs/2506.07339)：
  RTC 算法、delay-aware freezing、soft prefix inpainting 和异步执行定义；
- Physical Intelligence, [论文 PDF](https://www.pi.website/download/real_time_chunking.pdf)：真实机器人
  `H=50`、`s_min=25`、`beta=5`、`b=10` 以及 50 Hz 控制背景；
- Physical Intelligence,
  [real-time-chunking-kinetix](https://github.com/Physical-Intelligence/real-time-chunking-kinetix)：公开模拟参考实现；
- Hugging Face, [LeRobot RTC 文档](https://huggingface.co/docs/lerobot/en/rtc)、
  [ActionQueue](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/rtc/action_queue.py) 和
  [RTCProcessor](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/rtc/modeling_rtc.py)：公开的
  RTC action queue、real-delay skip 和 denoise guidance 实现；
- PickNik, [Stitch Trajectories with Smooth Blending](https://docs.picknik.ai/how_to/robotics_applications/stitch_trajectories/)：
  使用 Ruckig 生成局部 jerk-limited 轨迹过渡的工业实现参考；
- Nie et al., [PACE: Phase-Aware Chunk Execution](https://arxiv.org/abs/2606.00537)：利用预测速度结构选择低速
  replan 边界的训练外方法；
- Black et al., [Training-Time Action Conditioning for Efficient Real-Time Chunking](https://arxiv.org/abs/2512.05964)：
  在训练阶段模拟 inference delay 并学习 prefix continuation；
- Wang et al., [Real-Time Robot Execution with Masked Action Chunking](https://arxiv.org/abs/2601.20130)：通过
  masked action chunking 和 prefix-preserved sampling 同时处理 chunk 间及 chunk 内不一致。
