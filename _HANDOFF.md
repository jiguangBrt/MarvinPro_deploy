# Marvin Pro OpenPI 真机部署交接

更新时间：2026-08-05

## 当前状态

- 本目录是独立 Git 仓库，基线提交为 `8fb4232 Initial Marvin Pro rollout deployment`。
- 基线之后新增了以下诊断代码：
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

结合此前重规划边界离线诊断，当前已确认三个相互叠加的项目侧原因：

1. 15 Hz离散绝对目标形成阶梯输入。
2. 模型原始chunk的离散加速度偏高。
3. 逐点硬裁剪会引入更大的二阶不连续；持续rollout还额外存在约2至3步推理延迟造成的chunk边界错位。

官方低层控制不是当前首要嫌疑：恒定保持、确定性轨迹和同chunk的100 Hz版本都能在同一官方链路上
取得一致改善。

## 恒定姿态测试命令

先在 Apex 保持 Input Mode None，并启动 bridge：

```bash
cd /home/jh/TianJi_data_collector/MarvinPro_deploy
./scripts/run_bridge_on_controller.sh --allow-motion --publish-hz 100
```

另一个终端运行：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.hold_test_client \
  --robot-host 6.6.7.100 \
  --duration 10
```

客户端等待时将 Apex 切到 Custom，确认屏幕显示 `input_mode=3`、两个状态数组均为 `(3, 3)`，再输入
`HOLD`。测试结束后必须先在 Apex 切回 None。异常时优先切 None 或急停，不把 `Ctrl+C` 作为正常
停机步骤。

## 下一步实施

暂时不要先修改官方控制器，也不要直接给模型输出加低通滤波。确定性慢速轨迹测试已经完成，结果确认
15 Hz 离散目标更新是一个实际因素。

确定性轨迹的复现命令如下；先用 `--command-hz 15`，再在相同姿态改为 `100`：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.trajectory_test_client \
  --robot-host 6.6.7.100 \
  --command-hz 15
```

真实冻结policy chunk对照已经完成。下一阶段不再需要扩大真机诊断幅度，应转为实现并分项验证：

1. 保持模型节点的15 Hz时间语义，在节点间生成100 Hz目标。
2. 用速度、加速度和jerk受限的轨迹整形代替逐点硬裁剪。
3. 按推理耗时选择新chunk的时间索引，并在旧计划与新计划之间做连续衔接，而不是从 `new[0]` 硬替换。
4. 每项均应有独立开关和日志，先短时真机测试，再组合启用。

冻结 chunk 客户端现已改为严格 A/B。它在 Input Mode None 时 warmup 并冻结一次真实 policy 输出；
夹爪保持不变；JSON 同时保存原始 policy 节点和限制在推理姿态 `±0.03 rad` 的实际回放节点，并分别
报告两者的离散速度/加速度。第一次按 15 Hz 离散发送有界节点，自动返回锚点；第二次加载同一个 JSON，
保持相同模型时间轴，以 100 Hz 线性插值发送同一组有界节点。

2026-08-05 只读检查了已有的 `/tmp/marvinpro_red_cones_chunk_ab.json`：该文件创建于 18:00:37，
早于随后 18:02 的捕获尝试，是旧的 version 1 格式。其有界计划在 140 个手臂节点值中裁剪了 23 个，
离散最大速度 `0.181587 rad/s`，最大加速度 `2.423854 rad/s^2`；后者超过诊断安全上限
`2.0 rad/s^2`，不得直接执行。计划格式已升级到 version 2，同时保存原始 policy 节点和有界回放
节点并分别报告动态统计。旧 version 1 文件会被明确拒绝加载，新实验使用 `_v2.json` 文件名。

第一次捕获并离散回放：

```bash
cd /home/jh/OpenPI_UR/openpi
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --policy-host 192.168.50.73 \
  --capture-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --playback-mode discrete
```

回到 None 后加载同一计划做插值回放：

```bash
PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /tmp/marvinpro_red_cones_chunk_ab_v2.json \
  --playback-mode interpolated
```

不要把提高频率单独解释成完整修复方案；冻结chunk A/B已经证明插值有效但不足。

## 已完成验证

- `ruff check src tests`：通过。
- `python3 -m unittest discover -s tests -v`：19 项通过。
- 本地假 bridge 集成测试：收到 13 条恒定目标命令，16 维目标逐值完全一致；Input Mode 离开 Custom
  后客户端自动停止并输出统计。
- 本地假 bridge 轨迹测试：收到 41 条命令，只有指定关节发生变化，覆盖预期正负幅度并严格返回起点。
- 本地冻结 chunk A/B 集成测试：离散阶段只发送保存的模型节点，插值阶段发送节点间目标；两次使用
  同一个 JSON 计划并自动返回相同锚点。
