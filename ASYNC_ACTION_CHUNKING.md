# 异步推理下的 Action Chunk 错位

更新时间：2026-08-07

本文记录 Marvin Pro 远程 OpenPI rollout 中已经复现的 action chunk 时间错位，以及可以后续实现的
同步、重规划、重叠聚合和 Real-Time Chunking 方案。当前目的仅是保留设计依据和实施顺序，不表示这些
复杂方案已经在本项目中实现。

## 1. 当前模型的时间语义

训练配置 `pi05_marvinpro_red_cones` 使用：

- 数据集频率：15 Hz。
- `action_horizon=10`。
- 每次模型输入：当前三路图像、16维关节/夹爪状态和prompt。
- 每次模型输出：10组16维绝对关节/夹爪目标。

OpenPI数据加载器为action构造的时间偏移是：

```text
[0/15, 1/15, 2/15, ..., 9/15] seconds
```

因此对时刻 `t` 的观测 `O(t)`，模型学习的是：

```text
policy(O(t)) -> [A(t), A(t+0.0667), ..., A(t+0.6000)]
```

`action[0]` 是当前数据帧时刻的action，不是下一帧才开始。训练阶段关节action以相对当前state的delta
进入模型，policy输出变换再把它还原为绝对关节目标；夹爪维度不做关节delta变换。

相关本地实现：

- `/home/jh/OpenPI_UR/openpi/src/openpi/training/data_loader.py`
- `/home/jh/OpenPI_UR/openpi/src/openpi/training/config.py`
- `/home/jh/OpenPI_UR/openpi/src/openpi/transforms.py`
- `/home/jh/OpenPI_UR/openpi/src/openpi/policies/marvinpro_policy.py`

## 2. 错位是怎样产生的

设第一次观测生成chunk A：

```text
O0 -> policy -> A0 ... A9
```

为了隐藏远程推理延迟，客户端在A尚未执行完时采集第二次观测：

```text
执行 A0 ... A6
采集 O1，并开始远程推理
推理期间继续执行 A7 ... A9
O1 -> policy -> B0 ... B9
```

这里 `B0` 是基于 `O1` 所见真实机器人状态的当前动作，不是A轨迹无条件延长后的 `A10`。若机器人
落后于A的目标时间轴，`B0` 通常靠近真实反馈，而旧计划A9已经位于更远的未来目标。把B直接追加到A的
raw尾部会形成：

```text
追赶未实现的A9 -> chunk切换 -> 反向拉回O1附近 -> 再向前追赶
```

### 2026-08-07 真机证据

失败测试使用10节点完整消费、100 Hz线性插值、2倍时间尺度和0.30秒预取。操作员观察到约1.33秒
周期的明显回弹。决定性重规划样本为：

| 指标 | 数值 |
| --- | ---: |
| 第二次推理耗时 | `289.2 ms` |
| 推理返回时旧队列剩余 | `2` 个100 Hz点 |
| 旧raw尾部到最后实际发送目标 | `0.07926 rad` |
| 新 `B0` 到旧raw尾部 | `0.16913 rad` |
| 新 `B0` 到真实反馈 | `0.01041 rad` |

运动阶段531个发布tick中有77个手臂裁剪tick，主要涉及 `Joint1_R` 和 `Joint4_R`。第三次推理耗时
`386.6 ms`，超过0.30秒预取窗口，造成8个 measured-pose hold tick。

该结果说明：

1. 机器人没有实现旧raw轨迹尾部，不能把它当作下一chunk的真实起点。
2. 异步推理消除了大部分等待，但没有自动解决新旧chunk的时间和状态一致性。
3. 简单线性插值只能平滑数值跳变，不能把 `B0` 变成语义正确的 `A10`。
4. 安全裁剪不能充当轨迹规划器；若裁剪后仍继续消费未来节点，计划会持续跑在反馈前面。
5. 增大预取窗口可能避免空队列，但会让生成B所用的观测更早，不能单独解决错位。

## 3. 可选方案

### 3.1 同步chunk执行

流程：

```text
执行当前chunk
等待机器人实际跟踪并稳定
保持当前姿态
采集新图像和关节状态
等待远程推理
从保持姿态执行下一chunk
```

优点：

- 因果关系最清楚，不会把提前观测的 `B0` 当作 `A10`。
- 不需要修改模型或推理服务器。
- 适合作为当前项目的下一项安全基线。

缺点：

- 每个chunk之间会停顿约150至400 ms，另加机器人跟踪稳定时间。
- 训练示范中没有周期性停顿，可能造成分布偏移和任务效率下降。
- 若只按墙钟认为A9已经完成，而不检查反馈，仍可能在机器人落后时重新规划。

Physical Intelligence说明，RTC发布前的π0、π0-FAST和π0.5采用的就是这种同步方案：执行完chunk，
等待推理，然后从静止开始下一chunk。

### 3.2 Receding Horizon / 短前缀执行

模型仍预测完整chunk，但客户端只执行较短前缀，例如10个预测节点中的前3至5个，然后丢弃未执行尾部，
使用新观测重新规划。

优点：

- 更快吸收视觉和关节反馈，减少长open-loop轨迹的累积误差。
- 不会强迫机器人执行已经过时的远期节点。

限制：

- 推理耗时必须小于可用动作前缀/缓冲区的时间，否则仍会停顿。
- 新chunk必须从最后实际发送目标或反馈连续衔接，不能再次从旧raw尾部开始。
- 固定跳到 `new[k]` 不是充分的延迟补偿；`k` 应同时考虑推理延迟和实际跟踪进度。

Diffusion Policy采用action-sequence prediction与receding-horizon control结合，并指出过长执行horizon会
降低反应速度。

### 3.3 ACT Temporal Ensemble

ACT在每个控制时刻重新预测chunk。来自不同观测的chunk会对同一个绝对未来时刻给出多个候选动作，
客户端按指数权重聚合这些“同一时刻”的预测。

关键约束：

- 只能聚合时间对齐的预测，不能直接平均相邻时刻的A9与B0。
- 需要足够高的推理吞吐；当前150至400 ms远程延迟无法在15 Hz每帧完成一次独立推理。
- Physical Intelligence报告，针对flow-based VLA的简单temporal ensembling并不能保证有效或安全的
  chunk切换，因此本项目不应直接把加权平均当作最终方案。

### 3.4 通用异步动作队列与重叠聚合

LeRobot/SmolVLA异步推理将预测与执行解耦：动作队列下降到阈值时发送新观测，收到新chunk后对重叠
部分做可配置聚合。

主要参数：

- `actions_per_chunk`：每次使用多少预测动作。
- `chunk_size_threshold`：队列剩余到什么比例时开始新推理。
- `aggregate_fn`：怎样合并重叠动作。

该方法主要解决推理期间没有动作可执行的问题。LeRobot文档明确区分：异步队列解决idle，RTC解决
chunk之间的不连续。对于本项目的π0.5 flow policy，只移植队列而不处理语义对齐仍可能复现回弹。

### 3.5 Real-Time Chunking（RTC）

RTC专门处理本项目已经复现的问题。它在旧chunk执行期间异步生成新chunk，但不会事后把独立生成的B
直接拼到A尾部，而是在flow/diffusion采样过程中：

1. 根据实际推理延迟，确定推理期间必然执行的旧动作前缀。
2. 冻结这些已经承诺的动作。
3. 对剩余新chunk执行inpainting/guidance，使重叠区与旧计划连续。
4. 通过软过渡权重在连续性与新观测反应性之间折中。

优点：

- 保持异步连续运动，同时显式处理新旧chunk不一致。
- 原论文面向diffusion/flow action policy，理论上适用于π0.5且不要求重新训练。
- LeRobot已经提供Pi0、Pi0.5和SmolVLA的RTC实现及离线可视化工具。

实施成本：

- 必须修改远程policy采样过程，让服务器接收旧动作剩余部分和动态 `inference_delay`。
- 当前OpenPI WebSocket接口只接收观测并返回独立chunk，客户端后处理不能完整实现RTC。
- 需要先离线可视化inpainting后的节点、速度和加速度，再进入真机。

这是当前项目长期最值得实现的连续执行方案。

### 3.6 训练时Action Prefix Conditioning

RTC后续工作提出在训练阶段模拟推理延迟，并让模型直接条件化于已经承诺执行的action prefix。这样可以
避免推理时inpainting的额外计算。

优点：推理运行时更简单，对较大延迟更稳健。

限制：需要修改数据采样/训练目标并重新训练checkpoint，当前39999 checkpoint不能直接获得该能力。
应在客户端基线与推理时RTC验证后再考虑。

## 4. 与2倍时间尺度的关系

训练时A0至A9覆盖约0.6秒。若严格2倍慢放，A0至A9应覆盖约1.2秒。当前诊断客户端还增加了
“实时锚点到A0”的一个节点间隔，因此整个动作段是1.333秒。

时间拉伸不会改变模型输出的关节目标值，但会：

- 把同一路径的目标速度约降为一半。
- 在理想连续缩放下把加速度约降为四分之一。
- 把原本对应训练轨迹较早物理时刻的后续节点延迟执行。
- 改变接触、夹爪闭合和物体运动的时间关系。

冻结chunk中2倍慢放改善了跟踪，说明降低目标速度确实有价值；持续rollout中的失败则来自过长open-loop
执行和错误chunk锚点。两件事必须分开评估。时间拉伸可以作为临时安全参数，但不是chunk错位修复。

## 5. 本项目建议实施顺序

### 阶段A：安全与可观测性

1. 记录每条raw目标、实际过滤后发送目标、机器人反馈和对应时间戳。
2. 记录计划进度与真实跟踪进度，不能只记录队列剩余节点。
3. 对连续手臂裁剪设置停止/冻结门槛，不允许饱和后继续消费未来节点。
4. 运动阶段推理延迟采用median、p95和max动态统计，不使用静止dry-run固定值。

### 阶段B：同步反馈门控基线

1. 100 Hz插值发布。
2. 先保留2倍时间尺度，降低首次测试速度。
3. chunk结束后保持最后安全目标，并等待实际跟踪误差进入阈值。
4. 此时才锁存新图像/关节观测并开始推理。
5. 推理期间保持姿态，结果到达后从真实保持目标连续起步。

该阶段会停顿，但应首先验证周期回弹是否消失。

### 阶段C：客户端Receding Horizon

1. 每次只执行3至5个节点。
2. 新结果到达后丢弃旧未来节点。
3. 从最后实际发送目标重建100 Hz连续轨迹。
4. 跟踪误差过大时冻结轨迹时间轴。
5. 对比同步基线的平滑度、任务反应性和空队列比例。

### 阶段D：服务端RTC

1. 调研LeRobot RTC对Pi0.5 flow采样的具体改动。
2. 扩展OpenPI远程协议，上传旧chunk剩余动作、已提交prefix和动态推理延迟。
3. 在服务器采样过程中执行freeze/inpainting，而不是客户端事后线性拼接。
4. 使用保存数据和冻结观测做离线轨迹对比。
5. 通过速度、加速度、边界连续性和硬限位检查后再做短时真机测试。

### 阶段E：必要时重新训练

若推理时RTC仍无法应对远程延迟尖峰，再考虑训练时action prefix conditioning或延迟增强训练。该阶段
需要新checkpoint，不应与当前部署调度修复混在同一轮实验中。

## 6. 不应采用的简化方案

- 不要把新 `B0` 直接追加到未实现的旧raw尾部。
- 不要固定跳到 `B2` 或 `B3` 并称为延迟补偿；真机数据已经显示更高索引未必更接近反馈。
- 不要用逐点硬裁剪制造“平滑轨迹”；裁剪会产生新的速度/加速度折角。
- 不要仅增大prefetch窗口；这会增加观测到执行之间的陈旧时间。
- 不要对未按绝对时间对齐的action做普通平均。
- 不要放宽客户端 `0.08 rad` 或bridge `0.12 rad` 包络来掩盖跟踪失败。
- 不要再次运行10节点、2倍时间、提前0.30秒并追加raw尾部的失败真机参数。

## 7. 一手资料

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
