# 真机 Rollout 排障速查（Troubleshooting）

给现场操作用的速查表。每条分三栏：**终端里看到什么**（认故障就靠这些原文）、
**通俗解释**（机器人为什么停了）、**怎么办**。

先记住三件事：

1. **看到 `ROLLOUT COMPLETE` 不等于任务成功。** 它只是说"动作停了，机械臂已定在原地"，
   程序在等你去 Apex 把 Input Mode 切回 None。真正的原因写在其上几行或日志里。
2. 每段运行都有日志目录 `logs/<名字>_<时间>/`，事后查原因一条命令：
   `grep -E "rtc_final_status|rollout aborted" logs/<目录>/client.log`
3. 程序主动停下来之后，机械臂几乎都是**定在原地的安全状态**，不会乱动。先别慌，
   按流程切 None、再 `Ctrl+C` 停 bridge。异常时优先切 None 或按急停，不把 `Ctrl+C`
   当正常停止步骤。

终端分工：采集时是四终端（`quickstarts/` 里 T1-T4 脚本），跑 rollout client 的那个终端
（两轮验证计划里叫"终端 B"，采集流程里是 T4）会打出大部分报错；bridge 跑在控制器侧。
本文的"终端里看到"默认指 client 终端；bridge 侧要看对应 `logs/` 目录里的 `bridge.log`。

---

## 1. 机器人控制器自己掉线了（最常见，厂家还没给说法）

- **终端里看到**：`robot_state=(1, 12), expected (3, 3)` 或 `(2, 12)`，
  随后 `rtc_final_status=fatal_safety_hold`、`rollout aborted: timed out waiting for trajectory event ...`。
- **通俗解释**：机器人控制器自己从"可以动"的模式跳出去了，bridge 一发现就停止发动作，
  机械臂定住（有时手臂会失去支撑力微微下垂）。这不是我们的程序、也不是网络的问题，
  是控制器固件自己的行为，状态码含义还在等厂家答复。
- **怎么办**：看 Apex 有没有报警；确认状态恢复 `(3, 3)` 再重跑。2026-09 起出现变密
  （9/4、9/7 各一次），再遇到记录当时是在空跑还是抓取/搬运阶段，攒证据找厂家。

## 2. 模型这拍动作太猛，被平顺检查拦下

- **终端里看到**：`RTC C1/C2 blend is infeasible`、`velocity ... exceeds ... rad/s`、
  `c2_blend_infeasible`，反复刷屏，最后 `rtc_final_status=stuck_exhausted`
  或 `fatal_safety_hold`。
- **通俗解释**：模型想让关节转得太快太急，超过了"动作拼接处要平顺"的上限，bridge
  拒绝接上这段动作。拦几次之后程序就放弃。9/5–9/8 的多次提前停止都是这个原因
  （当时的上限是按红锥任务调的太紧）。
- **怎么办**：2026-09-08 起上限已放宽到"几乎只拦危险动作"，正常不会再见到。
  如果还频繁出现，说明模型输出真的超了机器人关节的速度上限（3.14 rad/s），
  该回头查模型或训练数据，不要再放宽程序。

## 3. 出故障后等机器人停稳，等不到

- **终端里看到**：`rtc_final_status=fatal_safety_hold code=... settle_error=...`，
  或老日志里的 `rollout aborted: timed out waiting for robot state`
  （2026-09-08 修复后报文改为 `timed out waiting for bridge hold tracking`）。
  前面几行通常刚有一次第 2 类或第 4 类故障。
- **通俗解释**：出了小故障后，程序让机器人定住，并等它"真的停稳"（各关节误差小于
  0.01 rad 且保持 0.2 秒）。但阻抗模式下机器人夹着东西会微微下垂，误差一直停在
  0.011 左右——差一点点，永远不算"停稳"，等 5 秒就放弃了。**这不是断线。**
- **怎么办**：重跑一次即可。如果同一个任务频繁这样停，说明该给"停稳"的判定留点
  余量，或检查夹持负载。

## 4. GPU 服务器算得太慢或网络卡了

- **终端里看到**：`Policy request timed out after 5.000s`、
  `RTC link fault: request wall latency ... exceeds ... budget`、
  `rtc_failure code=transport_timeout`。偶发时程序会自己恢复，只看到一条 WARNING。
- **通俗解释**：这一拍问 GPU 服务器要动作，回答来得太晚（本机到服务器走 WiFi），
  赶不上节拍，这段动作被丢掉。偶尔一次没关系；频繁就是链路或服务器问题。
- **怎么办**：确认 policy server 进程活着（服务器上 tmux 会话 `policy_redcones`）；
  确认没被别人重启/挤占（2026-08-28 同事重建会话曾把 8000 端口服务挤掉）；
  服务器重启后先跑预热脚本再 rollout。

## 5. 机器人的数据不来了（真正接近"断线"的一类）

- **终端里看到**：`timed out waiting for robot observation`、`latest robot state is stale`、
  或启动阶段就卡住超时。
- **通俗解释**：相机、关节或夹爪的数据话题不发新消息了，程序等不到数据就停。
  启动就遇到多半是相机没开。
- **怎么办**：先在 Apex 启动 Camera；再用只读命令确认四路输入（关节/左夹爪/右夹爪/相机）
  哪一路没消息（命令见 `_HANDOFF.md` 快速开始第 0 节）。2026-09-04 出现过一次左夹爪
  反馈间歇断流，后来自己恢复。

## 6. 小故障太密，恢复次数用完

- **终端里看到**：`rtc_final_status=recovery_exhausted recoveries=N max=M`。
- **通俗解释**：每出一次小故障，程序都按固定流程自救一次（定住→停稳→重新问模型→
  先试一小段→恢复）。只有给 `--max-rtc-recoveries` 设了上限时，自救满 M 次才会放弃；
  2026-09-08 起默认不限次数，会一直自救到跑满时长或出现致命故障。
- **怎么办**：往上翻日志找反复出现的那个故障码，按本文对应条目处理根源。

## 7. 过渡动作跑不完，连续两次卡死

- **终端里看到**：`rtc_final_status=stuck_exhausted stuck_replans=2`。
- **通俗解释**：自救流程里有一小段"慢速确定"的过渡动作，这段动作在规定时间内没执行完
  （机器人没跟上，或动作又被拒绝），连续两次就放弃。多数时候根源还是第 2 类。
- **怎么办**：同上，先解决根源；单独这一条不用处理。

## 8. 正常结束和"忘了切模式"——不是故障

- 跑满时长：`rtc_final_status` 不出现，直接 `ROLLOUT COMPLETE`，collector 提示
  输入 s/f/d 裁定。这是成功。
- 你按了 `Ctrl+C`：`operator interrupted rollout`，采集时最常见，正常。
- `ROLLOUT COMPLETE` 后 60 秒内没去 Apex 切 None：
  `rollout aborted: Input Mode stayed Custom after trajectory rollout`。
  这不是机器人出问题，是忘了切，重跑就行。

## 9. bridge 进程被停了

- **终端里看到**：`rollout aborted: robot bridge receive failed: robot bridge closed the connection`。
- **通俗解释**：控制器侧的 bridge 程序停了——有人 `Ctrl+C` 了它，或启动脚本自动替换
  旧进程时把它杀了。
- **怎么办**：确认是不是自己人停的；不是的话查 bridge 终端/日志。

## 10. 启动阶段的拦阻（机器人还没动）

- `bridge motion is disabled; restart robot bridge with --allow-motion`
  → bridge 没带运动权限启动，按文档命令重启 bridge。
- `execution confirmation was not given` → 该输入大写 `E` 时输了别的，重跑并输 `E`。
- 门控检查不通过（`input_mode` 不是 3、状态不是 `(3,3)`）→ Apex 上完成 Robot Ready、
  阻抗模式、切 Custom 后再跑；可用 `_HANDOFF.md` 里的只读 echo 命令确认。

---

## 还是搞不定的时候

把这两样发给懂代码的同事：

1. `logs/<本次目录>/client.log` 里 `rtc_final_status=` 和 `rollout aborted` 那几行；
2. 同目录的 `bridge.log`（如果 client 日志指向 bridge 侧问题）。

机器人此时是定住的安全状态，不需要急着做任何补救动作。
