"""障碍检测与绕行（WP5 / Issue #5）。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`ObstacleTask.step()`，就自动接入主流程，不需要改 main.py 或协调器。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：绕行动作必须拆成"每次一小步"的状态机，
    不许用一次阻塞的长动作把主循环占住。

必须实现的接口（签名已冻结，不要改）：
    class ObstacleTask:
        name = "obstacle"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没有障碍   -> TaskUpdate(TaskStatus.NOT_TRIGGERED)
  * 绕行中     -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
  * 绕行完成   -> TaskUpdate(TaskStatus.COMPLETED)
  * 失败/超时  -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

骨架已经替你实现的安全护栏（你不用自己写）：
  * 前进 <= 0.30 m/s、横移 <= 0.25 m/s、yaw <= 90 deg/s，超出会被裁掉并记录；
  * nan/inf 会被置零；
  * 单个任务连续接管超过 20 秒会被强制释放并硬停车——但**你自己也要给每一段
    动作设总时长上限**，不要依赖这 20 秒兜底；
  * 视频中断或人工 SPACE 会立刻结束接管并停车。

参考蓝本：竞速工程没有任何障碍检测实现，必须从零写。
轮廓筛选框架可以借 blue_line_following/blue_line_detector.py:186-236。
横移量级参考竞速工程 config.py 的 MAX_LATERAL_SPEED=0.25 m/s。

先看 tests/test_obstacle.py 的用例区再动手。
单独自测：python scripts/check_module.py obstacle
"""

from typing import Optional

from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection

KIND = "obstacle"


class ObstacleTask:
    """TODO(WP5)：把 detect() 换成真实障碍检测，再补 step() 的绕行状态机。"""

    name = "obstacle"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = settings
        self.last_detection = VisualDetection.no_result(KIND)
        self.started_at = None

    def detect(self, frame) -> VisualDetection:
        """TODO(WP5)：返回障碍的整幅图像坐标框；没有障碍返回 no_result。"""
        return VisualDetection.no_result(KIND)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection
        if not detection.valid:
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
        # TODO(WP5)：从这里开始绕行状态机。
        #   每一段动作都要能被打断；用 now 计算已经绕了多久；
        #   超过你自己的动作上限就返回 FAILED 停车，不要原地打转。
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
