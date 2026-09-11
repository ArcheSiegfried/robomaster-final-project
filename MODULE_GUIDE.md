# 模块接口与协作说明

公共接口版本：**v0.1（2026-09-11 核验冻结）**。本轮没有改变字段或运行语义。

骨架版本：**v0.2-task-skeleton**（第 6 节）。骨架只做了**附加**：新增协调层 `coordinator.py`、
注册表 `task_registry.py`、任务安全参数 `config.TaskConfig` 和模块协议。`models.py` 与
`MODULE_GUIDE.md` 第 2 节里 v0.1 的任何字段、单位、调用顺序**都没有变化**，老模块不受影响。

`models.py`、`camera_source.py`、`runtime.py`、`motion_output.py` 和 `main.py` 构成公共边界。**普通模块开发者不得自行修改；需要变更时先向项目负责人提出。** v0.1 冻结的是现有字段、单位和调用顺序，不代表巡线参数已经实车验证。

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
)
```

模块采用约定签名：

```python
def step(self, frame: FramePacket, now: float) -> TaskUpdate:
    ...
```

`NOT_TRIGGERED` 表示当前帧不接管；`RUNNING` 表示继续处理；`COMPLETED` 和 `FAILED` 都应停止该任务的运动请求并交回主流程处理。`step()` 必须快速返回。

### 巡线暂停和恢复

```python
LineFollower.process_frame(frame, captured_at: Optional[float] = None) -> RuntimeDecision
LineFollower.process_video_gap(frame_age: float, timestamp: Optional[float] = None) -> RuntimeDecision
LineFollower.pause(timestamp: Optional[float] = None) -> None
LineFollower.reset_fault(timestamp: Optional[float] = None) -> None
LineFollower.resume(timestamp: Optional[float] = None) -> bool
```

接管顺序：`follower.pause()`，随后 `output.claim("external")`。交回顺序：任务零运动、`output.claim("line")`、`follower.reset_fault()`、处理一张新帧确认路线、`follower.resume()`。`resume()` 只有在停止状态且最近 `0.15 s` 内存在有效路线时返回 `True`，并重置控制历史。

普通短时漏检由 `LineFollower` 在 `0.28 s` 内降速容错；超时锁定停车。长断线巡回属于未来外部任务，不能延长此时间冒充实现。

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
| `junction.py` | `JunctionTask` | 接管型 |
| `evidence.py` | `EvidenceRecorder` | 观察型（基础设施，由整合负责人维护，不占名额） |

每个文件已在 `task_registry.py` 里登记好，组员**只替换文件内容**即可：

```python
MOTION_TASK_CLASSES = (TrafficLightTask, NumberMarkerTask, ...)  # 可接管运动，顺序即优先级
OBSERVER_CLASSES = (EvidenceRecorder,)                            # 每帧可见，永不接管
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
| 接管顺序 | 按 `MOTION_TASK_CLASSES` 顺序询问，第一个返回 `RUNNING` 的拿到控制权 |
| 唯一 owner | 接管期间其他模块不再被询问 |
| 命令限幅 | 前进 ≤ 0.30 m/s、横移 ≤ 0.25 m/s、yaw ≤ 90 deg/s（见 `config.TaskConfig`），超限自动裁剪并写入 `decision.errors` |
| 数值安全 | `nan` / `inf` 一律置零 |
| 硬超时 | 单个任务连续接管超过 20 秒被强制释放并硬停车 |
| 异常隔离 | 模块抛异常只记录到 `decision.errors`，不会打断主循环 |
| 归还流程 | `COMPLETED` / `FAILED` → 硬停车 → `claim("line")` → 清巡线历史 → 等一张**新鲜有效**路线 → 自动恢复；2 秒内等不到就保持停车，需人工按 `SPACE` |
| 视频中断 | 立即结束接管并停车（`VIDEO_LOST` 锁存，不会自动恢复） |
| 人工打断 | `SPACE` / `R` 立即结束任何接管并停车，必须人工再按 `SPACE` 才恢复 |

**一旦返回 `RUNNING`，就必须持续返回 `RUNNING`，直到返回 `COMPLETED` 或 `FAILED`。**
中途返回 `NOT_TRIGGERED` 会被判为失败并强制归还控制权。

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
