# 模块接口与协作说明

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

完整的接管、完成和恢复流程见 `examples/offline_takeover.py`，使用 `python -m examples.offline_takeover` 运行。该示例没有接入正式 `main.py`。

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

如果需要修改 `models.py`、`runtime.py`、`motion_output.py` 或 `main.py`，先说明现有接口为何不足，由项目维护者统一协调。不要在任务分支中顺手重构公共代码。

## 6. 最简 Git 协作

领取任务后先同步基线，再使用短分支名，例如 `task/traffic-light`。一次提交只处理一个可说明的问题；提交前运行对应模块测试和完整离线测试。不要提交 `.venv`、缓存、日志、密钥或运行截图目录。

熟悉 PR 的成员可以推送分支并提交 PR；尚不熟悉 PR 的成员可以配对开发，或向维护者交回源码、测试、样图和验证结果，由维护者统一集成。无论采用哪种方式，公共接口和主程序变更都先由项目维护者协调。
