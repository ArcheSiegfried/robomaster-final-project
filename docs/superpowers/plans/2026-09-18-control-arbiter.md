# 控制权统一仲裁（Control Arbiter）实现计划

> **给执行者：** 必需子技能：用 subagent-driven-development（推荐）或 executing-plans 逐任务实现本计划。步骤用 `- [ ]` 复选框跟踪。

**Goal:** 把"注册表顺序第一个 RUNNING 赢"的隐式接管裁判，换成显式的中央仲裁器（优先级 + 租约 TTL + 抢占 + 滞后 + 控制权日志），且**不改变任何现有实车行为**。

**Architecture:** 新增纯逻辑文件 `control_arbiter.py`（`ControlRequest` + `ControlArbiter.select()`）；`task_registry.py` 增加显式优先级表；`coordinator.py` 改为"按优先级问、交给仲裁器选唯一赢家、每帧只调一次出口"，并引入**任务时钟**（模块只在自己被调用的时段里消耗时间）；红灯否决特判被安全级抢占取代；`main.py` 增加 `[ARB]` 控制权变更日志。

**Tech Stack:** Python 3.8 · unittest（**不是 pytest**）· numpy/OpenCV · 合成帧 + 假底盘（`tests/task_harness.py`）

**Spec:** `docs/superpowers/specs/2026-09-18-control-arbiter-design.md`（本计划实现它；执行者两份都要读）

## Global Constraints

- 仓库根：`E:\dsh\机器人期末\robomaster-final-project-integration`（下文路径均相对它）。解释器：`F:\robomaster\.venv\Scripts\python.exe`（下称 `PYEXE`）。
- **绝不运行** `main.py` / `route_only_main.py`、不连机器人/相机/SDK、不发实车命令。只用合成帧、假时钟、假底盘。
- **不许碰**：`models.py`、`motion_output.py`、`config.py`，以及 6 个成员模块（`traffic_light.py` / `green_junction.py` / `obstacle.py` / `route.py` / `free_junction.py` / `number_marker.py`）。新参数默认值放 `control_arbiter.py`，协调器用 `getattr(settings.tasks, "名字", 默认值)` 读。
- 允许改：`control_arbiter.py`(新) · `coordinator.py` · `task_registry.py` · `main.py`(只加日志行) · `tests/`(新测试) · 文档。
- 所有文本文件 **LF 换行**；推送工具遇到 CRLF 会拒收。
- 本工作区**没有可用的 git 仓库**：分支/提交/PR 一律通过 `python tools/push_files.py <branch> --message-file <msg> <files> --work <scratch>` 走 GitHub REST 创建。计划里的"提交"步骤指这个动作。
- 基线：全量 `Ran 645 tests / FAILED (failures=7)`，7 条红全部是 `tests/test_route.py::RouteRecoveryTests` 里的**既有**失败。**本次不得新增任何失败**。
- 契约版本：协调器 docstring 由 **v0.2 → v0.3**；不得新增 `TaskStatus` 成员（`coordinator.py:451-453` 会把未知状态静默当终态）。
- 顺序铁律：优先级降序展开必须**逐字等于** `tests/test_task_registry_order.py::EXPECTED_ORDER`（`traffic_light, green_junction, obstacle, route, free_junction, number_marker`）。
- 常用命令：`& $PYEXE -m unittest tests.test_xxx -v` · 全量 `& $PYEXE -m unittest discover -s tests` · 静态 `& $PYEXE scripts\check_module.py --all --quiet` · 总闸 `powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1`（成功末行 `OFFLINE_CHECK_OK`）。

## File Structure

| 文件 | 职责 | 动作 |
|---|---|---|
| `control_arbiter.py` | 纯逻辑：请求、仲裁判定、优先级常量、日志文本拼装。不导入 coordinator/config | 新建 |
| `task_registry.py` | 显式优先级表 + 给每个任务类挂只读 `priority` | 改 |
| `coordinator.py` | 收集请求 → 仲裁 → 唯一出口；任务时钟；租约与释放 | 改（step 路径重写） |
| `main.py` | `ConsoleStatus` 打印 `[ARB]` 行（只加行，不改控制流） | 改 |
| `tests/test_control_arbiter.py` | 仲裁器纯逻辑单测（TTL/抢占/滞后/tie-break） | 新建 |
| `tests/test_coordinator_arbitration.py` | 集成：Test1~Test6 + 成本回归 + 日志 | 新建 |
| `tests/test_task_clock.py` | 任务时钟：被跳过的模块不被墙钟误杀 | 新建 |
| `MODULE_GUIDE.md` / `README.md` | 契约 v0.3 说明 | 改 |

---

### Task 1: 仲裁器纯逻辑 `control_arbiter.py`

**Files:**
- Create: `control_arbiter.py`
- Test: `tests/test_control_arbiter.py`

**Interfaces:**
- Produces: `ControlRequest(module, priority, command, ttl, timestamp, state="", order=0)`（frozen dataclass；`ttl=None` 表示永不过期；`order` 仅用于同优先级 tie-break）· `ArbitrationResult(request, owner, reason, changed)` · `ControlArbiter(settings=None, safety_priority=90, preempt_margin=0, min_hold_seconds=0.2, preempt_all=False)` · `ControlArbiter.select(requests, now, current_owner="line", current_owner_since=None) -> ArbitrationResult` · `ControlArbiter.LINE_PRIORITY = 10` · `change_text(previous, result) -> Optional[str]`

- [ ] **Step 1: 写失败测试** `tests/test_control_arbiter.py`

```python
import pathlib, sys, unittest
ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control_arbiter import ControlArbiter, ControlRequest
from models import MotionCommand


def req(module, priority, now, ttl=0.5, order=0, state=""):
    return ControlRequest(module=module, priority=priority,
                          command=MotionCommand(forward=0.1), ttl=ttl,
                          timestamp=now, state=state, order=order)


class ArbiterTests(unittest.TestCase):
    def test_highest_priority_wins_and_is_reported(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([req("route", 60, now), req("obstacle", 70, now)], now)
        self.assertEqual(result.owner, "obstacle")
        self.assertTrue(result.changed)

    def test_expired_request_is_dropped(self):
        now = 10.0
        arbiter = ControlArbiter()
        stale = req("obstacle", 70, now - 5.0, ttl=0.5)
        result = arbiter.select([stale, req("route", 60, now)], now)
        self.assertEqual(result.owner, "route")

    def test_ttl_none_never_expires(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([req("line", 10, 0.0, ttl=None)], now)
        self.assertEqual(result.owner, "line")

    def test_equal_priority_never_steals_from_current_owner(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select(
            [req("route", 60, now, order=0), req("free_junction", 60, now, order=1)],
            now, current_owner="free_junction", current_owner_since=now - 1.0)
        self.assertEqual(result.owner, "free_junction")
        self.assertFalse(result.changed)

    def test_same_priority_tie_break_is_deterministic_by_order(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([req("b", 60, now, order=5), req("a", 60, now, order=2)], now)
        self.assertEqual(result.owner, "a")

    def test_min_hold_blocks_a_non_safety_challenger(self):
        now = 10.0
        arbiter = ControlArbiter(min_hold_seconds=0.5)
        result = arbiter.select([req("obstacle", 70, now)], now,
                                current_owner="green_junction", current_owner_since=now - 0.1)
        self.assertEqual(result.owner, "green_junction")

    def test_safety_priority_ignores_min_hold(self):
        now = 10.0
        arbiter = ControlArbiter(min_hold_seconds=0.5, safety_priority=90)
        result = arbiter.select([req("traffic_light", 90, now)], now,
                                current_owner="obstacle", current_owner_since=now - 0.1)
        self.assertEqual(result.owner, "traffic_light")

    def test_no_valid_request_releases_control(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([], now, current_owner="obstacle", current_owner_since=now - 1.0)
        self.assertIsNone(result.request)
        self.assertEqual(result.owner, "line")

    def test_change_text_mentions_priority_and_reason(self):
        now = 10.0
        arbiter = ControlArbiter()
        result = arbiter.select([req("obstacle", 70, now, state="dodging")], now,
                                current_owner="line", current_owner_since=now - 1.0)
        text = arbiter.change_text("line", result)
        self.assertIn("[ARB] owner changed: line -> obstacle", text)
        self.assertIn("priority=70 > 10", text)
        self.assertIn("dodging", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

`& $PYEXE -m unittest tests.test_control_arbiter -v` → `ModuleNotFoundError: No module named 'control_arbiter'`

- [ ] **Step 3: 实现 `control_arbiter.py`**

```python
"""Central control arbiter: decides which single module may command motion.

纯逻辑，不导入 coordinator / config / runtime。契约见
`docs/.../2026-09-18-control-arbiter-design.md`。

判定顺序：丢弃过期请求 → 优先级降序（同优先级按 order 升序）→
安全级（priority >= safety_priority）可无视最小持有时间抢占 →
其余挑战者必须 priority > owner.priority + preempt_margin 且已过最小持有时间。
"""

from dataclasses import dataclass
from typing import Optional, Sequence

from models import MotionCommand

LINE_OWNER = "line"


@dataclass(frozen=True)
class ControlRequest:
    module: str
    priority: int
    command: Optional[MotionCommand]
    ttl: Optional[float]
    timestamp: float
    state: str = ""
    order: int = 0

    def expired(self, now: float) -> bool:
        return self.ttl is not None and (now - self.timestamp) > self.ttl


@dataclass(frozen=True)
class ArbitrationResult:
    request: Optional[ControlRequest]
    owner: str
    reason: str
    changed: bool


class ControlArbiter:
    LINE_PRIORITY = 10

    def __init__(self, settings=None, safety_priority: int = 90,
                 preempt_margin: int = 0, min_hold_seconds: float = 0.2,
                 preempt_all: bool = False) -> None:
        self.safety_priority = int(safety_priority)
        self.preempt_margin = int(preempt_margin)
        self.min_hold_seconds = float(min_hold_seconds)
        self.preempt_all = bool(preempt_all)

    @staticmethod
    def _ranked(requests: Sequence[ControlRequest]) -> list:
        return sorted(requests, key=lambda item: (-item.priority, item.order))

    def select(self, requests, now: float, current_owner: str = LINE_OWNER,
               current_owner_since: Optional[float] = None) -> ArbitrationResult:
        valid = [item for item in requests if not item.expired(now)]
        ranked = self._ranked(valid)
        if not ranked:
            return ArbitrationResult(None, LINE_OWNER, "no_valid_request", current_owner != LINE_OWNER)

        winner = ranked[0]
        if current_owner != LINE_OWNER:
            held = [item for item in valid if item.module == current_owner]
            if held:
                holder = held[0]
                challenger_beats = winner.priority > holder.priority + self.preempt_margin
                if winner.module != current_owner:
                    if not self.preempt_all and winner.priority < self.safety_priority:
                        return ArbitrationResult(holder, current_owner, "owner_keeps_control", False)
                    if not challenger_beats:
                        return ArbitrationResult(holder, current_owner, "owner_keeps_control", False)
                    if (current_owner_since is not None
                            and winner.priority < self.safety_priority
                            and (now - current_owner_since) < self.min_hold_seconds):
                        return ArbitrationResult(holder, current_owner, "min_hold", False)
        reason = winner.state or "claimed"
        return ArbitrationResult(
            winner, winner.module, reason, winner.module != current_owner)

    def change_text(self, previous: str, result: ArbitrationResult,
                    previous_priority: int = LINE_PRIORITY) -> Optional[str]:
        if not result.changed or result.request is None:
            return None
        return ("[ARB] owner changed: %s -> %s  reason=%s  priority=%d > %d"
                % (previous, result.owner, result.reason,
                   result.request.priority, previous_priority))
```

- [ ] **Step 4: 跑测试确认通过**

`& $PYEXE -m unittest tests.test_control_arbiter -v` → `OK`（11 tests）

- [ ] **Step 5: 静态契约 + 提交**

`& $PYEXE scripts\check_module.py --all --quiet` 必须仍为 `MODULE_CHECK_OK`（本文件不是任务模块，但确认没影响）。
推送：`python tools\push_files.py feat/control-arbiter --message-file tools\_work\arb_msg1.txt control_arbiter.py tests/test_control_arbiter.py --work <scratch>`

---

### Task 2: 显式优先级表 `task_registry.py`

**Files:**
- Modify: `task_registry.py`（在 `MOTION_TASK_CLASSES` 之后追加）
- Test: `tests/test_task_registry_order.py`（追加两条测试）

**Interfaces:**
- Consumes: 无
- Produces: `TASK_PRIORITIES: dict[str, int]`（键 = 注册名）· `priority_of(cls_or_name) -> int` · `ranked_task_classes() -> tuple`（按优先级降序、同优先级保持注册顺序）

- [ ] **Step 1: 追加失败测试**（`tests/test_task_registry_order.py` 末尾、`unittest.main()` 之前）

```python
class PriorityTableTests(unittest.TestCase):
    def test_priority_table_matches_the_confirmed_order(self):
        ordered = [name for name, _ in sorted(
            task_registry.TASK_PRIORITIES.items(), key=lambda kv: -kv[1])]
        self.assertEqual(tuple(ordered), EXPECTED_ORDER)

    def test_safety_priority_is_the_top_module(self):
        top = max(task_registry.TASK_PRIORITIES, key=task_registry.TASK_PRIORITIES.get)
        self.assertEqual(top, "traffic_light")
        self.assertEqual(task_registry.TASK_PRIORITIES[top], 90)

    def test_every_registered_class_exposes_its_priority(self):
        for cls in task_registry.MOTION_TASK_CLASSES:
            self.assertEqual(cls.priority, task_registry.TASK_PRIORITIES[cls.name])
```

- [ ] **Step 2: 跑测试确认失败**

`& $PYEXE -m unittest tests.test_task_registry_order -v` → `AttributeError: module 'task_registry' has no attribute 'TASK_PRIORITIES'`

- [ ] **Step 3: 实现**

```python
# 显式优先级：数值 = 与集成负责人确认过的接管顺序（tests/test_task_registry_order.py:3-5）。
# 顺序仍是裁判依据，但从此**不许靠元组位置表达优先级**：插入新模块必须同时给出优先级，
# 否则 tests/test_task_registry_order.py 会红。90 = 安全级（可随时抢占）。
TASK_PRIORITIES = {
    "traffic_light": 90,
    "green_junction": 80,
    "obstacle": 70,
    "route": 60,
    "free_junction": 50,
    "number_marker": 40,
}

SAFETY_PRIORITY = 90
LINE_PRIORITY = 10


def priority_of(task_or_name) -> int:
    name = task_or_name if isinstance(task_or_name, str) else task_or_name.name
    return TASK_PRIORITIES.get(name, LINE_PRIORITY)


def ranked_task_classes():
    return tuple(sorted(MOTION_TASK_CLASSES, key=lambda cls: (-priority_of(cls), MOTION_TASK_CLASSES.index(cls))))


for _cls in MOTION_TASK_CLASSES:
    if not hasattr(_cls, "priority"):
        _cls.priority = priority_of(_cls)
```

- [ ] **Step 4: 跑测试确认通过**：`& $PYEXE -m unittest tests.test_task_registry_order tests.test_task_contract -v` → `OK`
- [ ] **Step 5: 提交**（推送 `task_registry.py` + 该测试）

---

### Task 3: 协调器走仲裁（无人接管 → 有人接管）

**Files:**
- Modify: `coordinator.py:173-197`（`_find_takeover` 替换为 `_collect_requests`）、`:339-387`（`_step_line`/`_begin_takeover`）
- Test: `tests/test_coordinator_arbitration.py`

**Interfaces:**
- Consumes: `control_arbiter.ControlRequest/ControlArbiter`、`task_registry.priority_of/SAFETY_PRIORITY/LINE_PRIORITY`
- Produces: `TaskCoordinator.arbiter`（`ControlArbiter` 实例）· `CoordinatorDecision.owner_change: Optional[str]` · `CoordinatorDecision.claims: tuple` · `TaskCoordinator.motion_owner: str`（`"line"` 或模块名）

- [ ] **Step 1: 写失败测试** `tests/test_coordinator_arbitration.py`（用 `tests/task_harness.py` 的 `TaskHarness` + 假任务）

```python
import pathlib, sys, unittest
ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import MotionCommand, TaskStatus, TaskUpdate
from tests.task_harness import TaskHarness


class StubTask:
    """按脚本返回状态的假模块：每一个元素对应一帧。"""
    priority = 70
    order = 1

    def __init__(self, name, script, priority=70, order=1):
        self.name = name
        self.priority = priority
        self.order = order
        self.script = list(script)
        self.calls = 0
        self.resets = 0

    def reset(self):
        self.resets += 1

    def step(self, frame, now):
        index = min(self.calls, len(self.script) - 1)
        self.calls += 1
        status, motion = self.script[index]
        return TaskUpdate(status=status, motion=motion,
                          message="%s frame %d" % (self.name, self.calls))


RUN = (TaskStatus.RUNNING, MotionCommand(forward=0.2))
IDLE = (TaskStatus.NOT_TRIGGERED, None)
DONE = (TaskStatus.COMPLETED, MotionCommand())


class ArbitrationIntegrationTests(unittest.TestCase):
    def test_only_line_keeps_control_when_nothing_claims(self):
        task = StubTask("obstacle", [IDLE] * 5)
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        for index in range(5):
            decision = harness.feed_line(1.05 + index * 0.05)
        self.assertEqual(harness.owner, "line")
        self.assertEqual(harness.coordinator.motion_owner, "line")

    def test_higher_priority_task_takes_over_and_route_is_paused(self):
        obstacle = StubTask("obstacle", [RUN] * 3 + [DONE], priority=70)
        route = StubTask("route", [RUN] * 10, priority=60)
        harness = TaskHarness(tasks=(route, obstacle))
        harness.start_line(now=1.0)
        first = harness.feed_line(1.05)
        self.assertEqual(first.task_name, "route")
        second = harness.feed_line(1.10)
        self.assertEqual(second.task_name, "obstacle", "更高优先级必须抢占")
        self.assertIn("[ARB] owner changed: route -> obstacle", second.owner_change or "")
        self.assertEqual(route.resets, 0, "被抢占的模块不许被 reset")

    def test_completed_task_releases_and_previous_owner_resumes(self):
        obstacle = StubTask("obstacle", [RUN, DONE], priority=70)
        route = StubTask("route", [RUN] * 10, priority=60)
        harness = TaskHarness(tasks=(route, obstacle))
        harness.start_line(now=1.0)
        harness.feed_line(1.05)
        harness.feed_line(1.10)
        decision = harness.feed_line(1.15)
        self.assertEqual(decision.task_name, "route", "高优先级结束后必须回到原任务")

    def test_three_modules_do_not_alternate(self):
        traffic = StubTask("traffic_light", [RUN] * 6, priority=90, order=0)
        obstacle = StubTask("obstacle", [RUN] * 6, priority=70, order=2)
        route = StubTask("route", [RUN] * 6, priority=60, order=3)
        harness = TaskHarness(tasks=(route, obstacle, traffic))
        harness.start_line(now=1.0)
        winners = [harness.feed_line(1.05 + index * 0.05).task_name for index in range(6)]
        self.assertEqual(set(winners), {"traffic_light"}, "同一帧只能有一个赢家，且不许交替")


if __name__ == "__main__":
    unittest.main()
```

同时在 `tests/task_harness.py` 给 `TaskHarness.__init__` 增加 `tasks=None` 参数（允许一次挂多个任务：`motion_tasks=tuple(tasks) if tasks else (() if task is None else (task,))`），这是**测试基础设施**，允许改。

- [ ] **Step 2: 跑测试确认失败**：`& $PYEXE -m unittest tests.test_coordinator_arbitration -v` → `TypeError: __init__() got an unexpected keyword argument 'tasks'` / `AttributeError: 'TaskCoordinator' object has no attribute 'motion_owner'`

- [ ] **Step 3: 实现协调器改动**

在 `coordinator.py` 顶部加 `from control_arbiter import ControlArbiter, ControlRequest` 与 `from task_registry import LINE_PRIORITY, SAFETY_PRIORITY, priority_of`（**注意循环导入风险**：`task_registry` 会 import 各任务模块，而任务模块不 import coordinator，因此安全；若真出现循环，改为在 `__init__` 内延迟 import）。

`CoordinatorDecision` 增加两个字段（带默认值，保持既有构造调用可用）：

```python
    owner_change: Optional[str] = None
    claims: tuple = ()
```

`__init__` 增加：

```python
        self.arbiter = ControlArbiter(
            safety_priority=SAFETY_PRIORITY,
            preempt_margin=int(getattr(settings.tasks, "preempt_margin", 0)),
            min_hold_seconds=float(getattr(settings.tasks, "min_hold_seconds", 0.2)),
            preempt_all=bool(getattr(settings.tasks, "preempt_all", False)),
        )
        self.motion_owner = OWNER_LINE
        self._motion_owner_since: Optional[float] = None
        self._owner_priority = LINE_PRIORITY
```

用 `_collect_requests` 取代 `_find_takeover`：

```python
    def _ask_list(self) -> tuple:
        """这一帧有资格说话的模块：安全级永远问；有人开车时再问 priority 更高的。"""
        ranked = sorted(self.motion_tasks,
                        key=lambda task: (-priority_of(task), self.motion_tasks.index(task)))
        owners_priority = LINE_PRIORITY if self.motion_owner == OWNER_LINE else self._owner_priority
        asked = [task for task in ranked
                 if priority_of(task) >= self.arbiter.safety_priority]
        if self.active_task is not None and self.active_task not in asked:
            asked.append(self.active_task)
        if self.arbiter.preempt_all:
            asked.extend(task for task in ranked
                         if priority_of(task) > owners_priority and task not in asked)
        return tuple(asked)

    def _collect_requests(self, frame, now, errors):
        requests = []
        for task in self._ask_list():
            if self._is_frozen(task):
                continue
            update = self._call(task.step, task.name, frame, now, errors)
            self._note_stepped(task, now)
            if update is not None and update.status is TaskStatus.RUNNING:
                requests.append(ControlRequest(
                    module=task.name,
                    priority=priority_of(task),
                    command=update.motion,
                    ttl=float(getattr(self.settings.tasks, "lease_seconds", 0.5)),
                    timestamp=now,
                    state=str(update.message or ""),
                    order=self.motion_tasks.index(task),
                ))
        return requests
```

`_step_line` 的接管段改为：收集请求（含巡线自身的 `ControlRequest("line", LINE_PRIORITY, decision.command, ttl=None, timestamp=now)`）→ `self.arbiter.select(...)` → 赢家是巡线就照旧下发，是任务就走 `_begin_takeover`，并把 `owner_change=arbiter.change_text(previous, result, previous_priority)`、`claims=tuple(...)` 填进 `CoordinatorDecision`。

- [ ] **Step 4: 跑测试确认通过**：`& $PYEXE -m unittest tests.test_coordinator_arbitration tests.test_coordinator -v` → `OK`
- [ ] **Step 5: 全量回归 + 提交**

`& $PYEXE -m unittest discover -s tests` 期望 `Ran <n> tests / FAILED (failures=7)`（**只允许**这 7 条既有红）。

---

### Task 4: 租约续期 / 过期释放 + 超预算强制释放

**Files:**
- Modify: `coordinator.py`（`_step_active` → 并入仲裁路径；新增 `_check_step_budget`）
- Test: `tests/test_coordinator_arbitration.py`（追加）

**Interfaces:**
- Consumes: Task 3 的 `motion_owner`、`arbiter`
- Produces: `TaskCoordinator._slow_frames: dict[str, int]` · 常量 `SLOW_STEP_FRAMES = 5`（连续超 `max_step_seconds` 的帧数上限）

- [ ] **Step 1: 写失败测试**

```python
    def test_module_that_stops_refreshing_loses_control_and_robot_stops(self):
        task = StubTask("obstacle", [RUN, TaskStatus.NOT_TRIGGERED], priority=70)
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.05)                      # 接管
        decision = harness.feed_line(1.10)           # 不续租
        self.assertEqual(decision.owner, "external", "释放握手期间仍由 external 持有")
        self.assertTrue(decision.force_stop)
        self.assertEqual(harness.chassis.last_speed()["x"], 0.0, "必须先硬停")

    def test_repeated_slow_steps_force_a_release(self):
        task = StubTask("obstacle", [RUN] * 30, priority=70)

        def slow(frame, now):
            time.sleep(0.05)                          # 超 max_step_seconds=0.02
            return RUN_UPDATE

        task.step = slow
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        for index in range(6):
            decision = harness.feed_line(1.05 + index * 0.05)
        self.assertIsNone(decision.task_name, "连续超预算必须被强制释放")
```

- [ ] **Step 2: 跑测试确认失败**（当前实现不会因超预算释放）
- [ ] **Step 3: 实现**

在 `_call` 返回后记录：`self._slow_frames[label] = self._slow_frames.get(label, 0) + 1 if elapsed > limit else 0`；`_slow_frames[label] >= SLOW_STEP_FRAMES` → `errors.append(...)` 并 `self._release(now, errors, "task exceeded the per-step budget", reset_task=True)`。

租约：`_collect_requests` 只在任务返回 RUNNING 时生成带 `ttl=lease_seconds` 的请求；`arbiter.select` 丢弃过期请求 → 若赢家消失且无其他有效请求 → `ArbitrationResult(None, "line", ...)` → 协调器走既有 `_release`（硬停 + `claim(OWNER_LINE)` + 等新线）。**不得**修改 `_release` 的握手内容。

- [ ] **Step 4: 跑测试确认通过**
- [ ] **Step 5: 全量回归 + 提交**

---

### Task 5: 任务时钟（零改成员模块，修两个既有误判）

**Files:**
- Modify: `coordinator.py`（新增 `_task_now` / `_note_stepped` / `_is_frozen`）
- Test: `tests/test_task_clock.py`

**Interfaces:**
- Produces: `TaskCoordinator._task_clock: dict[str, float]`（每个模块的虚拟 now）· `TaskCoordinator.task_now(task, real_now) -> float` · `TaskCoordinator._frozen: set[str]`

- [ ] **Step 1: 写失败测试**

```python
class TaskClockTests(unittest.TestCase):
    def test_a_skipped_module_does_not_age_on_the_wall_clock(self):
        """被跳过的模块，其 now 不得按真实时间前进（否则墙钟超时被误杀）。"""
        task = StubTask("obstacle", [IDLE, RUN], priority=70)
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        harness.feed_line(1.05)
        seen = []
        original = task.step

        def spy(frame, now):
            seen.append(now)
            return original(frame, now)

        task.step = spy
        harness.feed_blank(5.0)                 # 4 秒没有它的事
        harness.feed_line(5.05)
        self.assertLess(seen[-1] - seen[0], 1.0, "虚拟时钟不得跳 4 秒")

    def test_freezing_during_a_veto_does_not_age_the_owner(self):
        light = StubTask("traffic_light", [IDLE, RUN] + [IDLE] * 4, priority=90, order=0)
        task = StubTask("obstacle", [RUN] * 10, priority=70)
        harness = TaskHarness(tasks=(task, light))
        harness.start_line(now=1.0)
        harness.feed_line(1.05)                  # obstacle 接管
        first = task.calls
        for index in range(4):                   # 红灯期间 obstacle 不被 step
            harness.feed_line(1.10 + index * 0.05)
        self.assertEqual(task.calls, first, "红灯期间 owner 必须被冻结（不 step）")
```

- [ ] **Step 2: 跑测试确认失败**
- [ ] **Step 3: 实现**

```python
    def _is_frozen(self, task) -> bool:
        return task.name in self._frozen

    def _note_stepped(self, task, now: float) -> None:
        """推进该模块的虚拟时钟：只累计它真正被调用的时段。"""
        last = self._last_called.get(task.name)
        if last is not None:
            self._task_clock[task.name] = self._task_clock.get(task.name, last) + (now - last)
        else:
            self._task_clock[task.name] = now
        self._last_called[task.name] = now

    def task_now(self, task, real_now: float) -> float:
        """传给 task.step 的 now。钳制：不小于本帧 captured_at、不大于真实 now。"""
        return self._task_clock.get(task.name, real_now)
```

冻结规则：红灯否决 / 抢占暂停时把 owner 放进 `self._frozen` 且**不调用其 step**；解除时从 `_frozen` 移除并**同步 `_last_called`**，避免把冻结时长算进去。

- [ ] **Step 4: 跑测试确认通过**（并确认既有 `tests/test_red_light_veto.py` 仍绿）
- [ ] **Step 5: 提交**

---

### Task 6: `main.py` 控制权变更日志 + 竞争探测文案

**Files:**
- Modify: `main.py`（`ConsoleStatus._update` 内新增一行；`main.py:404-413` 的文案）
- Test: `tests/test_console_task_status.py`（追加）

**Interfaces:**
- Consumes: `CoordinatorDecision.owner_change`（Task 3 产出）
- Produces: 终端行 `[ARB] owner changed: ...`（**只加行，不改控制流**）

- [ ] **Step 1: 写失败测试**

```python
    def test_owner_change_is_printed_once(self):
        console = ConsoleStatus(stream=stream, enabled=True)
        decision = SimpleNamespace(state="TASK_ACTIVE", owner="external", task_name="obstacle",
                                   command=STOP_COMMAND, force_stop=False, errors=(),
                                   line=None, task_update=None, message="",
                                   owner_change="[ARB] owner changed: line -> obstacle  reason=dodging  priority=70 > 10",
                                   claims=())
        console.update(decision, 1.0)
        console.update(decision, 1.1)          # 同一条不许重复打
        text = stream.getvalue()
        self.assertEqual(text.count("[ARB] owner changed"), 1)
```

- [ ] **Step 2: 跑测试确认失败**（`ConsoleStatus.update` 不接受 `owner_change`）
- [ ] **Step 3: 实现**：`_update` 里维护 `self._last_owner_change`，不同则 `self._say(now, decision.owner_change)`；`_competition_text` 把「顺序里第一个想接管的」改成「优先级最高的」。
- [ ] **Step 4: 跑测试确认通过**：`& $PYEXE -m unittest tests.test_console_task_status tests.test_console_competition -v` → `OK`
- [ ] **Step 5: 提交**

---

### Task 7: 文档、总闸门、分支与 PR

**Files:**
- Modify: `coordinator.py` docstring（v0.2 → v0.3 段落）、`MODULE_GUIDE.md`、`README.md`
- Test: 全量 + 总闸

- [ ] **Step 1: 更新文档**：`coordinator.py` 顶部 Ownership rules 改写为"仲裁器 + 优先级 + 租约"；`MODULE_GUIDE.md` 增加"接管顺序 = `TASK_PRIORITIES`（数值见 `task_registry.py`）"与 `[ARB]` 日志格式；`README.md` 公共接口版本 v0.3 段落补一句"仲裁层 v0.3：不新增类型字段"。
- [ ] **Step 2: 跑总闸**：`powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1` → 末行 `OFFLINE_CHECK_OK`
- [ ] **Step 3: 全量回归并逐条核对**：`Ran <n> tests / FAILED (failures=7)`，7 条必须是 `tests/test_route.py::RouteRecoveryTests` 的既有失败；与基线逐条比对失败清单。
- [ ] **Step 4: 推送分支 + 开 PR**

```
python tools\push_files.py feat/control-arbiter --message-file tools\_work\arb_msg.txt <全部改动文件> --work <scratch>
python tools\merge_pr.py <PR号> <head-sha-prefix>        # 按需：合并前先给人看
```

- [ ] **Step 5: 交付清单**（对应指令第二十四节 14 项）：改动文件与逐文件说明 · `ControlRequest`/`ControlArbiter` 位置 · 唯一底盘出口 · 全部模块优先级表 · 谁能抢占 · 谁只能发事件 · TTL/心跳处理 · 恢复处理 · 同优先级抖动处理 · 是否有模块绕过仲裁（结论：无，附证据）· 控制权流程图 · 测试与静态检查原始输出。

---

## 自检（对照 spec）

- **spec 覆盖**：第三节机制 → Task 1/2；第四节单赢家 → Task 3；第五节优先级 → Task 2；第六节抢占 → Task 3/4（非安全级抢占由 `preempt_all` 控制，默认关，spec 4.2 已说明）；第七节 TTL → Task 4；第八节唯一出口 → 全程不改 `motion_output.py`（约束）；第九节数字识别不直接控制 → 既有静态契约已保证；第十节红灯 → Task 3/5；第十一/十二/十三节 RUNNING≠OWNER → Task 3 的 `motion_owner`；第十四节恢复不靠互相调用 → Task 3 测试 `resets == 0`；第十五节日志 → Task 6；第十六节抖动 → Task 1 的 tie-break/min-hold；第十七/十八节不破坏 → 全局约束 + Step 5 回归；第十九节多线程 → 勘察已证单线程，无任务；第二十三节 Test1~6 → Task 3/4；第二十四节交付 → Task 7 Step 5。
- **占位符扫描**：无 TBD/待补；每个代码步骤都有可运行代码。
- **类型一致性**：`ControlRequest` 字段（`module/priority/command/ttl/timestamp/state/order`）在 Task 3 构造处与 Task 1 定义一致；`ArbitrationResult(owner/reason/changed/request)` 在 Task 3/6 使用一致；`priority_of` / `SAFETY_PRIORITY` / `LINE_PRIORITY` 只在 Task 2 定义、Task 3 使用。
