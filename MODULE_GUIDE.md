# 模块接口与协作说明

公共接口版本：**v0.3（2026-09-17 批准）**。v0.3 **只扩展 `GimbalOutput` 的行为**（见第 2 节"云台请求"），
`GimbalCommand` / `TaskUpdate` 的字段、单位和调用顺序**一律不变**，v0.1/v0.2 写成的模块零改动。

版本历史：v0.2（Issue #15 批准）只增加可选云台请求，v0.1 的既有字段、单位和调用顺序保持兼容。

骨架版本：**v0.2-task-skeleton**（第 6 节）。骨架只做了**附加**：新增协调层 `coordinator.py`、
注册表 `task_registry.py`、任务安全参数 `config.TaskConfig` 和模块协议。第 2 节中
v0.1 已有字段、单位和调用顺序**都没有变化**；v0.2 只在 `TaskUpdate` 末尾追加可选
`gimbal` 字段，因此老模块不受影响。

`models.py`、`camera_source.py`、`runtime.py`、`motion_output.py`、`gimbal_output.py` 和 `main.py` 构成公共边界。**普通模块开发者不得自行修改；需要变更时先向项目负责人提出。** 接口冻结不代表巡线或云台参数已经实车验证。

## 1. 当前结构与启动

生产路径：

```text
main.py
  -> LatestFrameSource            单一相机入口
  -> LineFollower.process_frame   检测、容错和控制
  -> MotionOutput.send            唯一底盘出口
```

公共类型位于 `models.py`；配置位于 `config.py`。正式程序只从 `main.py` 启动。安装、离线测试和实车启动命令见 `README.md`。

任务模块不得自行连接相机或机器人，不得调用 `drive_speed()`、`drive_wheels()` 或 `move()`。每次处理一帧后立即返回，不得用长时间循环或 `sleep()` 占住主流程。

## 2. 固定公共接口

### 图像入口

```python
FramePacket(
    image: numpy.ndarray,
    sequence: int,
    captured_at: float,
)
```

- `image`：BGR 图像，尺寸由相机配置决定。
- `sequence`：单调递增帧号。
- `captured_at`：`time.monotonic()` 秒，不是系统日期时间。

相机接口：

```python
LatestFrameSource(camera, strategy: str, read_timeout: float)
start() -> None
wait_after(sequence: int, timeout: float) -> Optional[FramePacket]
age(now: Optional[float] = None) -> float
close() -> None
```

所有检测器接收同一个 `FramePacket.image`。新增模块不得创建第二个 `LatestFrameSource`。

### 路线检测

```python
LineDetector.detect(frame: numpy.ndarray) -> LineDetection
```

```python
LineDetection(
    valid: bool,
    error: float,
    heading: float,
    confidence: float,
    near_point: Optional[tuple[int, int]],
    far_point: Optional[tuple[int, int]],
    roi: tuple[int, int, int, int],
    mask: numpy.ndarray,
    contour: Optional[numpy.ndarray],
)
```

`error` 和 `heading` 以半幅图像宽度归一化；图像右侧为正。所有点及 ROI 都是整幅图像像素坐标，原点在左上，x 向右、y 向下。无有效路线时 `valid=False`，控制端不得使用其误差运动。

### 通用任务检测结果

```python
VisualDetection(
    valid: bool,
    kind: str,
    center: Optional[tuple[int, int]] = None,
    target_id: Optional[str] = None,
    color: Optional[str] = None,
    confidence: float = 0.0,
    box: Optional[tuple[int, int, int, int]] = None,
)
```

`center` 和 `box=(left, top, right, bottom)` 使用整幅图像像素坐标。数字等身份填 `target_id`，灯色等类别填 `color`；不适用字段为 `None`。没有结果统一返回 `VisualDetection.no_result(kind)`，不能用虚构坐标或空字符串代替。

### 运动请求

```python
MotionCommand(
    forward: float = 0.0,
    lateral: float = 0.0,
    yaw: float = 0.0,
)
```

- `forward`：m/s，正值向前。
- `lateral`：m/s，正值向右、负值向左；当前巡线恒为 0。
- `yaw`：deg/s；本项目约定正值右转、负值左转，首次实车接入仍须架空验证符号。

任务只能返回请求，不能发送 SDK 命令。最终命令必须经过：

```python
MotionOutput.claim(owner: str) -> None       # owner 仅为 "line" 或 "external"
MotionOutput.send(owner: str, command: MotionCommand) -> None
MotionOutput.hard_stop() -> bool
```

`claim()` 会先硬停车再切换唯一所有者，错误所有者发送会抛出异常。

### 任务状态

```python
TaskStatus.NOT_TRIGGERED
TaskStatus.RUNNING
TaskStatus.COMPLETED
TaskStatus.FAILED

TaskUpdate(
    status: TaskStatus,
    motion: Optional[MotionCommand] = None,
    detection: Optional[VisualDetection] = None,
    message: str = "",
    gimbal: Optional[GimbalCommand] = None,
)
```

模块采用约定签名：

```python
def step(self, frame: FramePacket, now: float) -> TaskUpdate:
    ...
```

`NOT_TRIGGERED` 表示当前帧不接管；`RUNNING` 表示继续处理；`COMPLETED` 和 `FAILED` 都应停止该任务的运动请求并交回主流程处理。`step()` 必须快速返回。

### 云台请求（v0.2 类型 / v0.3 出口行为）

```python
GimbalCommand(
    pitch: float,
    yaw: float = 0.0,
)
```

角度单位为度，表示相对机器人上电基准的绝对目标角。任务只把请求放入 `TaskUpdate.gimbal`；`GimbalOutput` 是唯一动态云台出口。正式运行保持 `CHASSIS_LEAD`，因此底盘搜索时云台 yaw 跟随底盘，任务主要请求 pitch。

**v0.3（2026-09-17）——为什么改**：SDK 在已有动作执行中再发 `gimbal.moveto()` 会直接抛
`Robot is already performing N action(s)`（`robomaster/action.py`）。旧出口只对"目标完全相同"去重，
换一个目标就撞上这个异常，协调器把它当成任务故障并重置任务；实车表现为
**number_marker 的 ID 4 接管后立刻掉链子**。v0.3 把出口改成"串行化 + 保留最新目标"：

1. **同一时刻只有一个绝对动作在飞**；忙时**不再调用 `moveto`**，只把目标放进 `pending`（**最新覆盖旧的**）；
2. **动作完成后由 `poll()` 收割并补发** `pending`；`poll()` 幂等、**非阻塞**（只读 action 的
   `state / is_completed / has_failed / failure_reason`，绝不 `wait_for_completed` 等待）；
3. **`restore_line_view()` 可重试**：忙时同样只是排队，动作完成后自动补发（它天然是最新目标）；
4. **不再向调用方抛异常**：`already performing` 只记进 `last_error` / `stats()`（`rejected_by_inflight`）；
   动作失败最多重试 2 次，之后放弃该目标并记录（`last_error`），不会无限重试；
5. 出口新增只读面：`busy`、`pending`、`last_error`、`at_line_view`、`stats()`。

**谁负责每帧推进（v0.3 的接口约定）**：`coordinator.py` 在 `step()` 里**每帧恰好一次**调用
`gimbal_output.poll()`（在任何状态分支之前）。这是闭环的一部分：
`coordinator.py` 的释放路径只在任务结束那一帧请求一次恢复，**没有周期性的 poll，
排队的"回巡线视角"就永远发不出去**（车会一直停在 `waiting for gimbal to return to line view`）。
释放期间协调器还会**等 `at_line_view` 为真**才开始 `gimbal_settle_seconds` 计时，并且**有界**：
超过 `release_resume_timeout` 仍未回正就转进"需要人工复位"的既有路径，不无限停车。
（老出口没有 `poll`/`at_line_view` 时，协调器按改造前行为处理，逐字节兼容。）

任务完成、失败、超时、异常、人工停止或视频失效后，协调器恢复 `CONFIG.gimbal_pitch/gimbal_yaw`，
等云台真的到位并 settle 之后才尝试恢复巡线。

#### 实验入口专用的第二个云台出口（2026-09-17 豁免说明）

`route_only_main.py`（直角长断口"云台侧视"实验入口）会注入
`route_gimbal_output.RouteGimbalOutput` —— 它**继承** `GimbalOutput`，增加了"相对车头 yaw +
云台角度遥测"模式。这是**唯一被批准的第二个云台出口**，边界如下：

* **只允许实验入口使用**：默认 `main.py` 仍然只用 `GimbalOutput`（`gimbal_output_factory=None`）；
* 它**不得**直接 import SDK，也不得绕过集中出口的限幅；
* 实验任务 `route_gimbal.py`（`GimbalAlignedRouteTask`）**不登记进 `task_registry.py`**
  （默认接管顺序与实车行为不受影响），只从该入口启用；契约测试用
  `tests/test_task_contract.py::EXPERIMENT_ONLY_MODULES` 显式豁免，并**反向断言它不在注册表里**。

### 得分照片（任务证据，统一层）

**照片张数就是 Final 的分数**，而"画框/画圈/写字/去重/落盘/写进 report.md"全部由证据层
（`evidence.py`，整合负责人维护）负责 —— 模块**只负责在得分那一刻把请求交出来**，
并且**文案由证据层统一生成**（5 个模块各写一套必然口径不一）：

```python
from evidence import make_evidence_photo

photo = make_evidence_photo(
    "obstacle",                       # 见 evidence.ANNOTATION_TEMPLATES
    frame,                            # FramePacket
    detection=detection,              # 带 .box 的 VisualDetection（矩形/圆都用它）
    shape="rect",                     # "rect"=框（障碍/堵车/路线/标识），"circle"=圈（灯）
    side="left", chosen="right",      # 按模板需要填（漏填会当场 KeyError）
)
```

交出去的链路与 `number_marker` / `traffic_light` 一样（`main.service_task_evidence()`
每帧对所有任务轮询，**必须回执**）：

```python
request = task.take_evidence_request()                 # 任务交出
saved = recorder.save_task_evidence(request)           # 证据层画 + 存，返回真实结果
task.acknowledge_evidence(request.request_id, saved)   # 回传真实结果
```

两条硬规矩：

* **尽力而为**：写盘失败绝不允许改变任务行为（不许因此 FAILED、停车或重试到超时）；
* **同一事件只存一张**：`event_key` 默认 = 照片标签，证据层对同一个事件键只写一次
  （红灯与绿灯是两个不同事件，必须分别保存）。

完整对照表（老师后来明确的 5 组要求）见 [`EVIDENCE_PHOTOS.md`](EVIDENCE_PHOTOS.md)。

### 巡线暂停和恢复

```python
LineFollower.process_frame(frame, captured_at: Optional[float] = None) -> RuntimeDecision
LineFollower.process_video_gap(frame_age: float, timestamp: Optional[float] = None) -> RuntimeDecision
LineFollower.pause(timestamp: Optional[float] = None) -> None
LineFollower.reset_fault(timestamp: Optional[float] = None) -> None
LineFollower.resume(timestamp: Optional[float] = None) -> bool
```

接管顺序：`follower.pause()`，随后 `output.claim("external")`。交回顺序：任务零运动、`output.claim("line")`、`follower.reset_fault()`、处理一张新帧确认路线、`follower.resume()`。`resume()` 只有在停止状态且最近 `0.15 s` 内存在有效路线时返回 `True`，并重置控制历史。

普通短时漏检由 `LineFollower` 在 `0.28 s` 内降速容错；超时锁定停车。`RouteTask` 是独立的长断线恢复任务，必须在此后才接管，不能延长短时容错时间冒充实现。第二版仍只完成离线验证，专用视觉辅助在 `route_detector.py`，详见 `ROUTE_RECOVERY.md`。

## 3. 最小模块示例

```python
class ExampleTask:
    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        result = VisualDetection.no_result("example")
        if not result.valid:
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=result)
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(forward=0.0, yaw=10.0),
            detection=result,
        )
```

完整的接管、完成和恢复流程见 `examples/offline_takeover.py`，使用 `python -m examples.offline_takeover` 运行。它用内存中的 `FakeChassis` 实际调用 `MotionOutput.claim()`、`send()` 和 `hard_stop()`，不会连接 SDK，也没有接入正式 `main.py`。

## 4. 独立测试方法

视觉模块优先接受函数参数中的 `numpy.ndarray`，用以下任一种数据测试：

- `cv2.imread("sample.png")` 读取去隐私的场地样图；
- 用 `numpy.full()` 和 `cv2.line()` 生成合成图；
- 构造 `FramePacket(image, sequence, captured_at)` 测时间与状态。

运动测试使用假出口记录 `MotionCommand`，断言方向、上限、完成后零请求及异常情况；不得导入或模拟连接真实机器人。测试放在 `tests/test_<module>.py`，执行：

```powershell
python -m unittest discover -s tests -v
```

## 5. 交回要求

模块交回内容：

- 模块源码及对应的 `tests/test_<module>.py`；
- 必要且可公开的少量样图；
- 实际运行的测试命令、通过数量和仍未验证事项；
- 输入假设、关键阈值和场地依赖；
- 没有虚拟环境、缓存、日志、密钥或大批原始录像。

如果需要修改 `models.py`、`camera_source.py`、`runtime.py`、`motion_output.py` 或 `main.py`，先说明现有接口为何不足，由整合负责人统一协调。不要在任务分支中顺手重构公共代码。

## 6. 任务骨架与模块接入（v0.2 骨架）

骨架已经把"每帧询问模块要不要接管"接进了主循环，所以新增模块**不再需要改 `main.py`**。

### 6.1 文件与登记

**一个文件 = 一个功能模块 = 一个人**，文件名与模块名一一对应：

| 文件 | 类 | 类型 |
|---|---|---|
| `number_marker.py` | `NumberMarkerTask` | 接管型 |
| `traffic_light.py` | `TrafficLightTask` | 接管型 |
| `obstacle.py` | `ObstacleTask` | 接管型 |
| `route.py` | `RouteTask` | 接管型 |
| `route_detector.py` | `RouteVision` | 长断线纯视觉辅助（不登记为任务） |
| `green_junction.py` | `GreenJunctionTask` | 接管型 |
| `free_junction.py` | `FreeJunctionTask` | 接管型 |
| `evidence.py` | `EvidenceRecorder` | 观察型（基础设施，由整合负责人维护，不占名额） |

每个文件已在 `task_registry.py` 里登记好，组员**只替换文件内容**即可：

```python
TASK_PRIORITIES = {"traffic_light": 90, "green_junction": 80, ...}  # 显式接管优先级
OBSERVER_CLASSES = (EvidenceRecorder,)                              # 每帧可见，永不接管
```

漏登记不会静默失效：`tests/test_task_contract.py` 会扫出根目录下所有带
`step(self, frame, now)` 或 `observe(self, frame, now)` 的文件，未登记就直接失败并告诉你加哪一行。

### 6.2 两种协议

接管型（可以拿到运动控制权）：

```python
class YourTask:
    name = "your_task"

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        ...
```

观察型（每帧都能看到，但**永远不能**接管运动）：

```python
class YourObserver:
    name = "your_observer"

    def observe(self, frame: FramePacket, now: float) -> None:
        ...
```

构造函数必须能**无参调用**。`step()` / `observe()` 必须**立刻返回**。

### 6.3 骨架替你保证的事（不要重复实现）

| 事情 | 骨架行为 |
|---|---|
| 唯一运动出口 | 只有 `coordinator.py` 调用 `MotionOutput`；模块只返回 `MotionCommand` |
| 接管时机 | 仅当基础巡线处于 `TRACKING` / `COASTING` / `LINE_LOST` 时可接管；`STOPPED`（人工暂停或复位后）和 `VIDEO_LOST` **一律不许**，模块不可能自己启动车 |
| 接管顺序 | 按 `task_registry.TASK_PRIORITIES` **优先级降序**询问，第一个返回 `RUNNING` 的拿到控制权（数值与"顺序即优先级"时代**完全一致**：`traffic_light` 90 → `green_junction` 80 → `obstacle` 70 → `route` 60 → `free_junction` 50 → `number_marker` 40） |
| 唯一 owner | 一帧只有一个模块能下发运动指令（`control_arbiter.ControlArbiter` 决定）；接管期间只问"优先级更高的模块 + 当前任务" |
| 安全级抢占 | 优先级 ≥ 90 的模块（今天只有红绿灯）**可以随时抢占**任何正在开车的模块，把车停住；其余模块默认不能中途抢占（见下） |
| 租约 TTL | 模块每次返回 `RUNNING` 就是一次续租；连续多帧不被调用或不再返回 `RUNNING` 时控制权会失效并交回（硬停 → 归还巡线） |
| 卡死保护 | 同一模块**连续 5 帧**单步耗时超过 `max_step_seconds` 会被强制释放并硬停车（原来只记一条 error） |
| **熔断（6s / 3s）** | **每个模块一次连续运行最长 6 秒**（`control_arbiter.RUN_SECONDS`，不含被红灯等冻结掉的时段），到点强制释放并硬停；**释放后 3 秒内不许再接管**（`COOLDOWN_SECONDS`）：冷却期内仲裁器直接丢弃它的请求、协调器**连 `step()` 都不调它**，终端打 `[ARB] <模块> locked out for 3.0s`，竞争探测里显示 `LOCKED_OUT`。<br>**冷却只锁刚跑完的那一个模块**，其他模块照常可以立刻接管（这是刻意的：让位而不是让车停摆）。<br>**对每个模块一视同仁，没有豁免**（红绿灯也一样）。数值由实车测试定稿：6 秒上限 + 3 秒冷却。<br>⚠️ 注意：几个模块自己的总预算比 6 秒长（`route` 19.7s、`traffic_light` 15s、`green_junction`/`free_junction` 12s、`obstacle` 11s），这些长动作会被 6 秒提前结束（车硬停 + 该模块冷却 3 秒），需要更长动作时请先调 `RUN_SECONDS` |
| 任务时钟 | 传给 `step()` 的 `now` **只累计该模块真正被调用的时段**。被跳过的帧、被红灯冻结的帧不计入 —— 所以"等红灯/被别人开车"不会把模块自己的墙钟超时耗掉 |
| 命令限幅 | 前进 ≤ 0.30 m/s、横移 ≤ 0.25 m/s、yaw ≤ 90 deg/s（见 `config.TaskConfig`），超限自动裁剪并写入 `decision.errors` |
| 数值安全 | `nan` / `inf` 一律置零 |
| 硬超时 | 单个任务连续接管超过 20 秒被强制释放并硬停车 |
| 异常隔离 | 模块抛异常只记录到 `decision.errors`，不会打断主循环 |
| 归还流程 | `COMPLETED` / `FAILED` → 硬停车 → `claim("line")` → 清巡线历史 → 等一张**新鲜有效**路线 → 自动恢复（**无限期等下去**：2026-09-18 起不再"2 秒等不到就要求人工按 `SPACE`"，线一回来就自己恢复） |
| 丢线处理 | 短时丢线按 `lost_grace_seconds` 滑行；**超过也不再停车**（2026-09-18 取消锁停）：继续低速**直行**找线（forward ≤ `lost_forward_speed`、yaw 衰减到 0），线一回来立刻恢复跟踪。仍保留的兜底：**视频丢失**锁停、人工 `SPACE`、任何任务接管 |
| 视频中断 | 立即结束接管并停车（`VIDEO_LOST` 锁存，不会自动恢复） |
| 人工打断 | `SPACE` / `R` 立即结束任何接管并停车，必须人工再按 `SPACE` 才恢复 |

**一旦返回 `RUNNING`，就必须持续返回 `RUNNING`，直到返回 `COMPLETED` 或 `FAILED`。**
中途返回 `NOT_TRIGGERED` 会被判为失败并强制归还控制权。

**关于"抢占"（`preempt_all`）**：默认 **关闭**。关闭时，一个模块开车期间只有安全级
（优先级 ≥ 90，即红绿灯）能抢它；其他模块必须等它自己结束。打开后，任何**优先级严格更高**
的模块都能中途接管（还有一个 `min_hold_seconds` 最小持有时间防止抖动）。这个开关故意
不写进 `config.TaskConfig`（那是"基础底座"的文件）：要打开得先在 `config.py` 里加一行
`preempt_all: bool = True`，并且**必须实车复验** —— 它会改变实车行为（例如绿灯岔路能在
避障绕行中途夺走控制权，今天不会）。

**控制权日志**：每次换人终端都会打一行，用来现场排查"谁把车交给了谁"：

```text
[  12.3s] [ARB] owner changed: line -> obstacle  reason=obstacle confirmed; stepping aside  priority=70 > 10
[  15.0s] [ARB] owner changed: obstacle -> traffic_light  reason=holding red  priority=90 > 70
[  18.2s] [ARB] owner changed: traffic_light -> obstacle  reason=stepping aside  priority=70 <= 90
```

### 6.4 单独测试一个模块

```powershell
python scripts/check_module.py number_marker    # 静态契约检查 + 自己的测试 + 合成帧实跑轨迹
python scripts/check_module.py --all --quiet    # 全部模块，只报结论
```

每个模块的测试文件是 `tests/test_<模块文件>.py`，前面是必须一直通过的契约测试，
后面是留给你补的用例区。`tests/task_harness.py` 提供 `TaskHarness`：用合成帧和假底盘把
**单个模块**接进真实的协调器跑，不需要机器人、相机或网络。

一条必须永远成立的断言：**合成帧里只有一条蓝线时，任何模块都不许接管。**
如果实现完之后 `test_does_not_take_over_on_a_plain_line_frame` 开始失败，
说明你的模块会误触发，必须先修误触发再谈功能。

## 7. 最简 Git 协作

领取任务后先同步 `integration`，再使用短分支名，例如 `feat/traffic-light` 或 `fix/line-loss`。一次提交只处理一个可说明的问题；提交前运行对应模块测试和完整离线测试。PR 目标必须是 `integration`。不要提交 `.venv`、缓存、日志、密钥或运行截图目录。

完整的新手步骤见 `COLLABORATION_GUIDE.md`。尚不熟悉 PR 的成员可以配对开发，但仍应保留任务分支、源码、测试、样图和验证结果。公共接口和主程序变更都先由整合负责人协调。
