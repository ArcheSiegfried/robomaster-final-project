# 控制权统一仲裁（Control Arbiter）设计文档

| 项目 | 内容 |
|---|---|
| 日期 | 2026-09-18 |
| 目标分支 | `feat/control-arbiter` → PR 到 `integration` |
| 基线 | `integration` @ `a1a0d7cbd672b8a33a55b6bffcb024a5d287e15a` |
| 需求来源 | 用户指令文档《机器人多模块控制权统一仲裁修改指令》（1049 行） |
| 接口版本 | 协调器契约 **v0.2 → v0.3** |
| 状态 | **待集成负责人 review**。6 个决策已按推荐值落定，可逐条否决 |
| 文档位置说明 | 暂放在仓库外（工作区无可用 git 仓库、且同步会删除仓库内未推送文件）；实现时随分支提交进仓库 `docs/` |

---

## 一、先说结论：你要的架构，现状已经存在 80%，缺的是 4 件事

指令假设"多个模块同时调 `drive_speed`/底盘"。**实测不成立**：7 个功能模块的顶层 import 里没有 `motion_output` / `gimbal_output` / `robomaster` / `coordinator`，它们只 `return TaskUpdate(motion=MotionCommand(...))`；速度唯一落地点是 `coordinator.py:214`（外部任务）/ `:354`（巡线）/ `:407`（红灯）。而且这件事有**强制门禁**：`tests/task_harness.py:57-70` 的 `FORBIDDEN_PATTERNS` 禁止模块源码出现 `drive_speed`/`drive_wheels`/`motion_output`，`scripts/check_module.py:78-81` 对每个模块执行。

| 指令里的担忧 | 真实状态 | 证据 |
|---|---|---|
| 多模块同时控制底盘 | **不可能**（静态契约 + 单一出口） | `tests/task_harness.py:57-70`、`motion_output.py`（全文 62 行） |
| 模块结束后仍发指令 | **不存在**（模块只返回请求） | `coordinator.py:214` |
| 模块卡死后车继续跑 | **已被 SDK 兜住**：`command_timeout=0.15`，旧速度 0.15s 后自动清零 | `config.py:109-115`、`motion_output.py:43` |
| 高优先级无法抢占 | **存在**，且机制是隐式的：优先级 = 注册表顺序，第一个 RUNNING 赢 | `task_registry.py:6-7`、`coordinator.py:183` |
| 红灯被巡线覆盖 | **曾真实发生**，靠硬编码补丁修（只认 `traffic_light` 一个名字） | `coordinator.py:59,124-131,455-471` |
| 看不出谁在抢控制权 | **确实没有**：零 owner 变更日志 | `config.py:137-144` |
| 同优先级抖动 | 顺序固定 → tie-break 天然确定；但**无最小持有时间** | `coordinator.py:173-197` |
| 多线程并发写底盘 | **不存在**：控制循环单线程轮询；线程只在数据源里 | `main.py:642`、`camera_source.py:22`、`marker_source.py:111`、`robot_source.py:72` |

**所以真正缺的 4 件事**（本次要做的）：

1. **显式优先级**：今天"注册表顺序 = 优先级"是隐式的，且**注释与代码矛盾**（见第五节）。
2. **TTL / 租约**：模块没有心跳概念，"卡死"只靠 SDK 的 0.15s 兜。
3. **控制权切换的可观测性**：没有 `[ARB] owner changed` 日志 —— 实车调试时看不出谁在开车。
4. **抢占与恢复的统一语义**：红灯否决是一个绕开通用机制的补丁（`coordinator.py:455-471`），任何新模块想要"紧急抢占"都得再打一个补丁。

---

## 二、实测数据（决定方案能不能落地）

离线合成帧 + 假底盘（`tests/task_harness.py` 的真实 `LineFollower` + `TaskCoordinator` + `MotionOutput`），60 帧统计、前 10 帧热身；脚本 `tools/_work/bench_step_cost.py`。

| 场景 | 单帧耗时（mean / p95） |
|---|---|
| 基线：只有 LineFollower（0 个任务） | 2.38 / 2.84 ms（只有线）；2.69 / 2.90 ms（线+分支） |
| **全部 6 个模块每帧都被问** | **16.84 / 18.89 ms**（只有线）· **23.96 / 25.34 ms**（线+分支） |
| 今天 owner 期间实际成本（obstacle + 红灯否决） | ≈ **5.8 ms** |
| 实车 `green_junction` 单模块 | **31 ms**（超 20ms 限值，`captures/run_20260918_132330/report.md:178`） |

各模块净增（mean，ms）：`green_junction` 5.55 / **12.72** · `free_junction` 1.43 / 3.73 · `route` 1.76 / 2.86 · `obstacle` 1.85 / 1.62 · `traffic_light` 1.52 / 1.53 · `number_marker` ≈ 0。

**两条硬结论**：

1. **"每帧 step 所有模块"在实车上不可行**：30fps 预算 33.33ms，合成帧就要 24ms，实车 `green_junction` 一个模块就 31ms → 一定会掉帧。而掉帧本身就是安全问题（`config.py:106-107` 记录过实车"75% 的循环拿不到新帧"的教训）。
2. **"只问可能赢的模块"成本与今天持平**：优先级顺序扫描 + 一旦确定赢家即停，普通帧（无人接管）仍要问到第一个 RUNNING，与今天 `coordinator.py:181-189` 完全同价；有人接管时只问"优先级更高的 + 当前 owner"。

---

## 三、三个方案

### 方案 A（**推荐**）· 中央仲裁器 + 优先级顺序扫描

新增 `control_arbiter.py`（新文件，不动任何成员模块），协调器改为按**优先级**扫描并收集请求，交给仲裁器选唯一赢家，每帧只调一次出口。

- 语义满足指令："所有模块都可以提出请求，但只有中央仲裁器决定最终运动指令"。
- 成本 ≈ 今天（见第二节），不牺牲控制频率。
- 红灯否决**收编**为正式的最高优先级请求，`LIGHT_TASK_NAME` 特判删除。
- 默认**不改 5 个成员模块一行**。

### 方案 B · 只加显式优先级数字

registry 加 `priority` 字段 + `_find_takeover` 加 TTL 判据 + 加日志。改动最小，但骨架仍是"第一个 RUNNING 赢"，红灯仍是补丁，**不满足指令第二十三节验收标准里"由 priority + state + TTL 决定 Winner"的形态**。

### 方案 C · 严格照字面每帧全量收集

**否决**。实测会掉帧（第二节），且需要模块降频/异步化的大规模重构，与指令第二节"不要推倒重写"直接冲突。

---

## 四、设计

### 4.1 新增文件 `control_arbiter.py`

```python
@dataclass(frozen=True)
class ControlRequest:
    module: str            # 模块名，与 task_registry 的 name 一致
    priority: int          # 显式优先级（见 4.3）
    command: Optional[MotionCommand]   # None = 只要停车/不提供运动
    ttl: float             # 租约时长（秒）
    timestamp: float       # 产生时刻（monotonic）
    state: str             # 模块自报状态（取自 TaskUpdate.message/status，用于日志）

class ControlArbiter:
    def select(self, requests, now, current_motion_owner=None) -> Optional[ControlRequest]
    @property
    def motion_owner(self) -> str          # "line" 或 模块名
    def note_change(self, previous, winner, reason, now) -> Optional[str]   # 生成日志文本
```

`select()` 判定顺序（这是全部语义所在）：

1. **丢弃过期请求**：`now - timestamp > ttl` → 该请求失效。
2. **按 priority 降序**；priority 相同按 **registry 序号升序**（确定性 tie-break，天然消除 A/B/A/B 抖动）。
3. **抢占裕度**：挑战者必须 `challenger.priority > owner.priority + preempt_margin`（默认 `preempt_margin = 0`，即严格更高才抢）。**同优先级永远不抢当前 owner** → 抖动在判定层面就不可能发生。
4. **最小持有时间**：`now - owner_since < min_hold_seconds`（默认 0.2s）时，除了**安全级**（priority ≥ `safety_priority`，默认 80）之外不允许切换。
5. 没有有效请求 → **返回 None → 协调器 `hard_stop()`**（对应指令第七节 "如果没有 → STOP"）。

### 4.2 协调器改为"按优先级问、问到赢家即停"

```
ranked   = 全部任务按 (priority 降序, registry 序号升序)
owner_p  = 当前运动拥有者的优先级（巡线 = 10）

# 这一帧有资格说话的模块：
ask = [t for t in ranked if t.priority >= safety_priority]     # 安全级：永远被问（默认 90 = 红绿灯）
    + [当前任务 owner]                                          # 持有者必须能续租
    + ([t for t in ranked if t.priority > owner_p] if preempt_all else [])

# 无人接管时：按 ranked 顺序问，遇到第一个 RUNNING 即停（早退，与今天同价）
```

**为什么抢占集要分两档**（`preempt_all` 默认 **False**）：

| 情形 | 今天 | 本设计（`preempt_all=False`） |
|---|---|---|
| 无人接管（普通巡线） | 按注册表顺序问到第一个 RUNNING 为止 | **同价**（改成按优先级顺序，顺序本来一致） |
| 有任务在开车 | 只问 owner（+ 红灯否决特判） | 问 owner + 安全级模块 → **同价**（安全级只有红绿灯，今天本来就要问，1.52ms） |
| 非安全级模块能否中途抢？ | **不能**（要等 owner 释放） | **不能**（`preempt_all=False` 时保持） |

**关键取舍**：如果开成"任意更高优先级都能中途抢占"（`preempt_all=True`），在**你确认过的顺序**下会出现这样的组合：`obstacle(70)` 开车时必须每帧问 `green_junction(80)` 能不能抢 —— 而 `green_junction` 是**最贵**的模块（合成帧实测 5.5~12.7ms，实车单帧 31ms）→ owner 期间成本从 5.8ms 涨到 11~18ms，实车有掉帧风险；而且**行为会变**（绿灯岔路能在避障绕行中途夺走控制权，今天不会）。

所以本设计：**安全级（红绿灯）永远可抢占**（= 今天红灯否决的通用化，行为等价）；**其余模块的"中途抢占"做成一个开关**，默认关，等实车有结论再开。这是"不改变现有功能"与"支持真正抢占"之间的最小风险切分。

### 4.3 优先级表（**决策 Q1：保持今天已确认的顺序，行为零变化**）

**先更正一个我自己搞错的判断**（写在文档里留痕）：我一度认为 `task_registry.py:35`「`obstacle` 优先于岔路/巡回/标识接管」与实际顺序（`green_junction` 第 2、`obstacle` 第 3）**矛盾**。查证后**不成立**：

- `tests/test_task_registry_order.py:3-5` 明确写「与**集成负责人确认过**的优先顺序一致：红绿灯 → **红绿灯岔路** → 障碍物绕行 → 短线巡回 → **障碍物岔路** → 数字识别」→ 顺序是**有意为之且经确认**的。
- `tests/test_task_registry_order.py:45-50` 那条"障碍物在叉路之前"的断言，**只断言** `obstacle` 在 `free_junction` / `route` / `number_marker` 之前，**没有把 `green_junction` 包含进去** → 「叉路」指的是**障碍物岔路**（`free_junction`），不含红绿灯岔路。
- `task_registry.py:35` 的「岔路」在上下文里同样是**障碍物岔路**；措辞有歧义，但意图有测试钉死。

**结论：优先级数字 = 今天的确切顺序，零行为变化。**

| 优先级 | 模块 | 注册名 | 顺序依据 |
|---|---|---|---|
| **100** | 系统安全停止（人工急停 / 视频丢失 / 协调器故障） | —— | 协调器内部，不走请求，永远最高 |
| **90** | 红绿灯（红灯停） | `traffic_light` | `tests/test_task_registry_order.py:35-38`「停车是安全项，不能被任何正在开车的模块挡住」 |
| **80** | 绿灯路口（红绿灯岔路） | `green_junction` | `:3-5` 确认顺序第 2 位 |
| **70** | 紧急避障（障碍物绕行） | `obstacle` | 确认顺序第 3 位；`:45-50` 只要求它压过 free_junction/route/marker |
| **60** | 断线巡回 / 短线恢复 | `route` | 确认顺序第 4 位 |
| **50** | 障碍物岔路 | `free_junction` | 确认顺序第 5 位 |
| **40** | 数字识别触发的动作 | `number_marker` | `:40-43`「排最后：先让会动作的模块先上」 |
| **10** | 普通巡线 | —— | `LineFollower`，不在 registry |
| **≥90（安全级）** | 可随时抢占的模块 | `safety_priority = 90` | 只有红绿灯 |

优先级集中放在 `task_registry.py` 的一张显式表里（`TASK_PRIORITIES`）+ 每个类一个只读 `priority` 属性，**不改成员模块源码**。

**为什么仍要把它显式化**（虽然顺序不变，收益依然存在）：今天的"顺序即优先级"是**隐式**的（`task_registry.py:6-7`），后果是：① 任何人往元组里插一行都会**静默改掉优先级**（`number_marker` 就是这么从第 2 位掉到最末的，`:6-7` 有记录）；② 终端只能显示"顺序里第一个想接管的"（`main.py:404-413`），看不出**数值**优先级；③ 没有 TTL，无法表达"这个模块的优先级只在此刻有效"。显式化后，`tests/test_task_registry_order.py` 的语义从"顺序必须等于这个元组"升级为"优先级必须等于这张表"，插入新模块时**不会**悄悄改变既有模块的相对优先级。

### 4.4 TTL / 租约 / 心跳（**决策 Q4**）

本项目是"协调器**拉**模块"（`coordinator.py:157` 固定 `function(frame, now)`），模块**没有主动发心跳的能力**。因此 TTL 落地为**租约**：

- 模块每次返回 `RUNNING` = 续租：协调器把 `timestamp` 刷成当前时刻。
- 租约默认 `lease_seconds = 0.5`。**赢家每帧都被调用**，所以正常情况下租约永远新鲜；租约真正兜的是"协调器因为任何原因没能调用它"（bug、异常路径）→ 请求失效 → 重新仲裁 → 无人接管就 STOP。
- 与既有机制的分工（不重复、不冲突）：
  - `NOT_TRIGGERED` 而仍持有控制权 → 视为失效并走释放握手（**保持今天的行为**，`coordinator.py:440-450`）；
  - `max_task_seconds = 20.0` 硬上限（**保留**，`config.py:83`）；
  - 视频丢失 `video_gap_stop_seconds = 0.60`（**保留**，`config.py:114`）；
  - **新增**：单步耗时连续 `K = 5` 帧超过 `max_step_seconds`（0.02s）→ 强制释放（把"模块卡死"从"靠 SDK 0.15s 兜"升级为"协调器主动夺权 + STOP"）。今天只记一条 error 不处理（`coordinator.py:162-166`）。

### 4.5 任务时钟（**决策 Q3：修两个真实缺陷，零改成员模块**）

**缺陷 1 —— 红灯暂停吃掉模块内部超时**：3 个模块的超时全是墙钟（`traffic_light.py:446`、`green_junction.py:2391/2465/2531`、`number_marker.py:711/963`），而红灯暂停只补偿**协调器自己**的 20s 预算（`coordinator.py:394,406`）→ 红灯停 8 秒，`green_junction` 下次被调用就 `turn did not finish inside turn_timeout` 失败。
**缺陷 2 —— 被跳过的帧仍消耗墙钟**：非赢家模块在"有人开车"期间不被 step，但 `now` 照走 → 下帧可能直接 FAILED（"恢复时无故失败"）。

**修法**：协调器为每个任务维护一个**任务时钟** `task_now = real_now - 该任务未被调用的累计时长`，并把 `task_now` 作为 `now` 传给 `task.step()`。语义变为"**这个模块的时间线只在它被调用的时段里推进**"。

- 冻结（红灯暂停 / 被抢占停车）期间不被调用 → 它的时钟不走 → 内部超时不会被误杀。
- 零改成员模块（超时公式全在模块内部，入口只有 `now` 一个）。
- **额外修掉两个已存在的"被跳过就误判"缺陷**（勘察确认，本设计顺带治好）：
  1. `obstacle.py` 有断档自检 `STALE_STEP_GAP = 1.0`（`obstacle.py:222`，判定在 `:935-960`）：被红灯否决超过 1 秒后，它下一次被调用会以为"自己被外部踢掉了"，进入 `RECONFIRM` 并先停 0.10s（`obstacle.py:1081-1103`）。红灯最长可停 15s（`traffic_light.max_hold_seconds=15`）→ 必然触发。任务时钟让这个间隔消失。
  2. `route.py` 的接管判据是 `now - _last_clear_at > lost_grace + 0.05`（`route.py:1623`）：长时间没被调用后，恢复那一帧会"凭空"认为线丢了很久并接管。任务时钟下 `_last_clear_at` 与 `now` 同源，判据不再被跳过的时间污染。
- 注意调用节拍：主循环**只在有新帧时**才调 `coordinator.step()`（`main.py:645-662`），没新帧走 `coordinator.video_gap()`（`:651`）。所以"任务时钟"的不推进时段 = 视频间隙 + 红灯暂停 + 被抢占停车的时段，三者都会被正确排除。
- **已知副作用（我在测试里钉住）**：`number_marker` 用 `now - frame.captured_at > 0.15` 判帧过旧（`number_marker.py:1043`）。任务时钟落后于真实时间时该判据会**偏松**。兜底两条：① 任务时钟钳制为不小于本帧 `captured_at`、不大于真实 `now`；② 真正的新鲜度由 `frame.sequence` 严格匹配保证（`number_marker.py:246-247` 要求 `source_sequence == frame.sequence`），且视频丢失由协调器 `video_gap()` 兜底（`coordinator.py:612-641`）。测试：`test_task_clock_does_not_mask_stale_frame`。

### 4.6 状态语义：RUNNING ≠ OWNER（指令第十二/十三节）

不引入新枚举（**不改 `models.py`**）。区分两个概念，全部由协调器持有：

| 概念 | 含义 | 存放处 |
|---|---|---|
| **task_owner** | 逻辑上"正在执行的任务"（持租约、可恢复） | `coordinator.active_task`（沿用） |
| **motion_owner** | 本帧真正能下发运动指令的那一个 | 仲裁器 `motion_owner` |
| **paused** | task_owner 存在但本帧不是 motion_owner（红灯 / 被高优先级抢占） | 仲裁器状态表 |

**被暂停的任务不 reset、不丢状态、时钟冻结**；它恢复后从原状态继续（对应指令第十四节"不要依赖模块之间互相调用 route.start()/obstacle.stop()"）。

### 4.7 控制权日志（指令第十五节）

- `coordinator.CoordinatorDecision` 新增两个字段：`owner_change: Optional[str]`、`claims: tuple`。
- **协调器不 print**（保持 `coordinator.py:28` "never touches a camera, the SDK or the network" 的分层），由 `main.py` 的 `ConsoleStatus` 打印 —— 落点是状态变化行 `_say`（`main.py:298-304`，格式 `[%6.1fs] text`），判定与拼装放在 `_update`（`main.py:308`）与新的仲裁行之间。
- **今天是没有任何 owner 变化日志的**：`ConsoleStatus` 打的是协调器 **state** 变化（接管 `main.py:319-327`、优先级截断 `:328-336`、结束 `:337-350`、巡线恢复 `:351-352`），`owner` 只出现在 cv2 画面文字里（`main.py:487`）。所以 `[ARB]` 行是**新增能力**（指令第十五节要求），不是改文案。
- 顺带要改的一行文案：竞争探测现在打「判给 X（**顺序里第一个想接管的**）」（`main.py:404-413`），改成「判给 X（**优先级最高**）」。
- 格式（指令要求含 module / priority / reason / state）：

```
[ARB] owner changed: ROUTE -> OBSTACLE   reason=obstacle_detected  priority=80 > 50
[ARB] owner changed: OBSTACLE -> TRAFFIC_LIGHT  reason=red_light  priority=90 > 80
[ARB] owner changed: TRAFFIC_LIGHT -> ROUTE  reason=lease_released  priority=50
```

### 4.8 每帧流程（伪码）

```
step(frame, now):
    poll_gimbal()                      # 保留 v0.3 每帧 poll
    observe(frame, now)
    delta = now - previous

    if not takeover_allowed (巡线不在 TRACKING/COASTING/LINE_LOST):
        return _step_line(...)         # 模块不得自行启动机器（现有规则保留）

    requests = []
    for task in ask_list(owner_priority):        # 见 4.2
        if task is paused: continue               # 冻结：不调用、时钟不走
        update = call(task.step, frame, task_clock(task, now))
        if update.status is RUNNING:
            requests.append(request_from(task, update, now))
            if task.priority > owner_priority: break   # 更高优先级已定，无需再问

    winner = arbiter.select(requests, now, current_motion_owner)

    if winner is None or winner.module == LINE:
        if decision.force_stop: output.hard_stop()
        elif follower.motion_enabled: output.send(OWNER_LINE, decision.command)
    else:
        if motion_owner changed: output.claim(OWNER_EXTERNAL); log(...)
        output.send(OWNER_EXTERNAL, clamp(winner.command))   # 每帧只发一次
    return CoordinatorDecision(..., owner_change=..., claims=...)
```

### 4.9 保留不动的东西（指令第十七/十八节：不许破坏现有功能）

- 巡线/避障/红绿灯/岔路/巡回/数字识别 **行为逻辑一行不改**（成员模块零改动）。
- 接管前提"巡线必须 armed"（`coordinator.py:62,345`）、释放握手（`hard_stop → claim(line) → reset_fault → 等新线`，`coordinator.py:473-507`）、人工覆盖（`human_stop/reset/resume`）、视频间隔处理 —— **全部保留**。
- 云台出口 v0.3 接口（每帧 `poll()`、`at_line_view`、`restore_line_view()`）**不动**。
- `motion_output.py`（62 行、无锁）**不动**：出口仍只有 `line`/`external` 两态，"是哪个模块在开车"由仲裁器维护。
- 现有的"竞争探测帧"（`claim_probe_seconds=3.0`，`coordinator.py:320-337`）**保留并升级**：它变成"全量请求扫描帧"，配合任务时钟后**不再会误杀被跳过的模块**（修复第二节缺陷 2 的副作用）。

---

## 五、明确不做（Not Doing）

- **不做"每帧 step 所有模块"**（实测掉帧，第二节）。
- **不改 `models.py`**（`TaskStatus` 不加新值，`TaskUpdate` 不加字段）→ 不动高冲突公共文件、不破坏现有接口。
- **不改 5 个成员模块**（`traffic_light` / `green_junction` / `obstacle` / `route` / `free_junction` / `number_marker`）。优先级集中在 registry 表；红灯收编、任务时钟都在协调器内完成。**零代改**。
- **不改 `motion_output.py`** → 连带把"`hard_stop()` 的 `drive_wheels` 走 ACK 路径、最坏阻塞 3.0s（SDK `client.py:151`）"这个问题**留在本次范围之外**（决策 Q6）。
- **不改任何实车调参值**（速度、增益、阈值、超时默认值一律保留）。
- **不引入线程/锁/事件总线/插件框架**（`AGENTS.md` 明确禁止；实测单线程轮询无并发写底盘）。
- **不做"中途安全边界抢占"**（例如"绿灯岔路转到一半时避障不抢"）—— 作为观察项记录，等实车有结论再说。

---

## 六、与既有红灯否决的关系（重要）

今天的红灯否决（`coordinator.py:455-471`）之所以存在，是因为"第一个 RUNNING 赢 + 只问赢家"意味着**别的模块开车时红绿灯根本不被问**（`coordinator.py:124-131` 注释记录了实车复现："障碍接管期间红灯亮着，车仍以 forward=0.2 在走"）。

在新设计里它**不再需要特判**：`traffic_light` 优先级 90 是最高，`ask_list` 永远包含它 → 它每帧都被问 → 红灯天然能抢占任何模块。`LIGHT_TASK_NAME` 特判与 `_light_veto()` 随之删除，**行为等价但机制通用化**（以后任何模块要"紧急抢占"只需声明更高优先级）。

---

## 七、验收标准（对应指令第二十三节 Test 1~6）

新增 `tests/test_control_arbiter.py`（合成帧 + 假底盘 + **假时钟**，不跑 `main.py`、不接硬件）：

| # | 场景 | 断言 |
|---|---|---|
| Test 1 | 只有巡线 | `motion_owner == "line"`，速度正常下发 |
| Test 2 | 巡线 + 避障（80） | 避障赢；巡线 PAUSED；避障 COMPLETED 后巡线自动恢复。**不得**出现巡线覆盖避障 |
| Test 3 | 巡线 + 红灯（90） | 红绿灯赢；命令为 STOP；巡线即使每帧申请也无法覆盖 |
| Test 4 | 三模块同时申请（50/80/90） | 唯一赢家 = 优先级最高者；连续 N 帧**不交替** |
| Test 5 | 赢家停止续租（模拟卡死） | 租约到期 → 释放 → 无其他安全请求 → STOP |
| Test 6 | 两个模块同优先级 | tie-break 确定性，**不逐帧 A/B/A/B**；最小持有时间内不切换 |
| 附加 | 日志 | owner 变更恰好打印一次，含 module / priority / reason |
| 附加 | 成本 | 断言"有人接管时被 step 的模块数 ≤ 优先级更高的 + owner"，防回归成"每帧全问" |
| 附加 | 任务时钟 | 被跳过 / 被红灯暂停的模块，其墙钟超时**不**被触发；同时陈旧帧仍被识别 |

其它门禁：`python -m unittest discover -s tests` 全量回归（基线 645 tests / 7 个既有 route 红测试）、`python scripts/check_module.py --all`、`python -m py_compile`、`git diff --check` 等价检查。

---

## 八、风险与回滚

| 风险 | 缓解 |
|---|---|
| 显式化优先级时数字写错 → 静默改掉接管顺序 | 新测试断言"优先级降序展开 == `tests/test_task_registry_order.py` 的 `EXPECTED_ORDER`"，与现有顺序断言双保险；默认顺序**零变化** |
| 开启 `preempt_all` 会改变实车行为（绿灯岔路可中途夺走避障的控制权） | 默认**关闭**；开之前必须先实车复验 |
| 任务时钟让 `number_marker` 帧龄判据偏松 | 钳制 + `frame.sequence` 严格匹配 + 专项测试（4.5 节） |
| 协调器改动面大（712 行） | 只重写"选谁开车"的判定路径；释放握手/人工覆盖/视频间隔/云台逻辑不动；全量回归守住 |
| 抢占导致任务中途被打断 | 被暂停任务不 reset、状态保留、时钟冻结；恢复即续跑（4.6 节） |

**回滚**：单分支单 PR，`git revert` 即回到 `a1a0d7cb`；优先级表是纯数据，改一行即可退回今天的顺序。

---

## 九、6 个决策点（已按推荐值落定，可逐条否决）

| # | 决策 | 落定值 | 理由 / 改变条件 |
|---|---|---|---|
| Q1 | 优先级排序 | **保持今天已确认的顺序**（traffic_light 90 / green_junction 80 / obstacle 70 / route 60 / free_junction 50 / number_marker 40 / 巡线 10）——**行为零变化** | `tests/test_task_registry_order.py:3-5` 记录了"与集成负责人确认过"的顺序，且 `:45-50` 的断言不含 `green_junction`。我先前"注释与代码矛盾"的读法已被推翻（4.3 节留痕） |
| Q2 | 每帧问谁 | **优先级顺序扫描 + 定赢家即停**；安全级（红绿灯）永远被问；其余模块的中途抢占由 `preempt_all` 开关控制，**默认关**；保留 3 秒一次的"全量扫描帧"（配合任务时钟后安全） | 实测每帧全问在实车会掉帧（31ms > 33ms 预算）；且"任意高优先级中途抢占"在此顺序下会让 owner 期间成本翻倍并改变实车行为 |
| Q3 | 红灯否决 | **收编为 P90 请求 + 引入任务时钟** | 通用化，且顺带修掉"红灯暂停吃掉模块内部超时"这个真实缺陷；零改成员模块 |
| Q4 | TTL | **租约**：每帧返回 RUNNING 即续租，`lease_seconds=0.5`；另加"连续 5 帧超单步预算 → 强制释放" | 项目是拉模型，模块无法主动发心跳 |
| Q5 | 改动边界 | `coordinator.py` + 新增 `control_arbiter.py` + `config.py` + `task_registry.py` + 新测试 + 文档；**不改 `models.py` / `motion_output.py` / 成员模块**；契约 v0.2→v0.3；分支 `feat/control-arbiter` + PR 到 `integration` | 零代改，冲突面最小 |
| Q6 | `hard_stop()` 最坏阻塞 3s（`drive_wheels` 走 ACK 路径） | **本次不修**，单独开单 | 属于 `motion_output.py`（高冲突公共文件），与仲裁是两件事；修它会把接口版本决策从"协调器 v0.3"扩成"出口 v0.2" |
