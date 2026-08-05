# Marvin Pro OpenPI 真机部署交接

更新时间：2026-08-05

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
/home/jh/TianJi_data_collector/MarvinPro_deploy/artifacts/marvinpro_red_cones_chunk_ab_v2.json
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
包含在内，不能视为低层伺服器的精确误差；下一项隔离测试是保持raw节点与100 Hz插值不变，仅用
`--playback-time-scale 2.0` 将总时长从 `0.667 s` 拉长至 `1.333 s`。

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

## 明日测试顺序

### 1. 第一优先级：原始chunk的2倍慢速对照

测试问题：在节点、路径、100 Hz插值、bridge和官方控制链全部不变时，只把轨迹时间拉长2倍，右臂
跟踪和主观平滑度是否明显改善？这是当前用于验证“机器人执行跟不上计划时间轴”的最小变量实验。

前置条件：bridge继续以 `--allow-motion --publish-hz 100` 运行；Apex先保持Input Mode None；机器人
回到与保存计划相符的锚点附近，客户端会检查最大姿态漂移 `0.01 rad`。

```bash
cd /home/jh/OpenPI_UR/openpi

PYTHONPATH=/home/jh/TianJi_data_collector/MarvinPro_deploy/src \
uv run python -m marvinpro_deploy.frozen_chunk_test_client \
  --robot-host 6.6.7.100 \
  --load-plan /home/jh/TianJi_data_collector/MarvinPro_deploy/artifacts/marvinpro_red_cones_chunk_ab_v2.json \
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

| 指标 | 1倍速基线 | 2倍速待测 |
| --- | ---: | ---: |
| 唯一观测样本 | `13` | 待填 |
| 观测单步p95 | `0.004060 rad` | 待填 |
| 观测单步最大值 | `0.007912 rad` | 待填 |
| 速度p95 | `0.067246 rad/s` | 待填 |
| 速度最大值 | `0.155151 rad/s` | 待填 |
| 加速度p95 | `0.400496 rad/s^2` | 待填 |
| 加速度最大值 | `0.758374 rad/s^2` | 待填 |
| 表观跟踪误差RMS | `0.012923 rad` | 待填 |
| 表观跟踪误差最大值 | `0.050226 rad (Joint4_R)` | 待填 |
| 自动回锚最大误差 | `0.002678 rad` | 待填 |

判据：若主观颤抖、RMS和 `Joint4_R` 最大误差均明显下降，则原始时间尺度下的跟踪滞后得到支持；
若仅表观误差变化而主观运动无改善，需要先改进高频目标/反馈配对测量；若完全不改善，则转向对
`Joint4_R` 做无policy的确定性轨迹对照，检查关节负载或阻抗响应。

### 2. 第二优先级：持续rollout重规划边界

第一项完成前不要同时修改轨迹形状和重规划逻辑。之后先增加只读日志，不立即改变真机动作：

1. 每次推理记录观测年龄、端到端推理耗时及其折算的15 Hz节点数。
2. 记录被替换时旧计划剩余节点、上一已发送目标，以及 `new[0..4]` 分别与当前目标的边界差。
3. 记录每次 `action plan empty`、measured-pose hold持续时间和新计划恢复时的跳变量。
4. 记录机器人反馈与上一实际发送目标的误差，避免再把“即将发送的目标”当作同步目标。

只读数据确认后再做单变量A/B：先仅按推理延迟选择 `new[k]`，不做滤波；然后再单独增加旧/新计划
边界连续化。每项必须能通过命令行开关恢复旧行为，并先做dry-run边界统计，再做短时真机测试。

### 3. 后续修复顺序

若前两项证实时间尺度和重规划均有贡献，实施顺序为：

1. 保持模型15 Hz语义，在节点之间输出100 Hz连续目标。
2. 用延迟对应的新chunk索引替代固定 `new[0]`。
3. 消除plan underrun造成的“测量姿态hold -> 新计划”切换。
4. 最后才引入速度、加速度和jerk受限整形；不要再使用逐点硬裁剪作为轨迹整形器。

暂时不要修改官方控制器参数，也不要把提高bridge频率单独解释成完整修复。恒定保持已经证明官方链路
能够稳定保持，冻结chunk则证明100 Hz插值有效但不足。

### 旧计划文件说明

`/tmp/marvinpro_red_cones_chunk_ab.json` 是version 1旧文件，其有界计划裁剪23/140个手臂节点值，
最大速度 `0.181587 rad/s`、最大加速度 `2.423854 rad/s^2`，超过诊断安全上限，客户端会拒绝加载。
明日只使用仓库 `artifacts/` 下SHA-256已记录的version 2文件。

## 已完成验证

- `ruff check src tests`：通过。
- `python3 -m unittest discover -s tests -v`：19 项通过。
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

截至收工尚未执行 `--playback-time-scale 2.0` 真机测试；这是明日第一项，不要把它误记为已完成。
