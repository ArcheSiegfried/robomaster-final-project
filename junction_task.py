"""两类岔路：绿灯岔路 + 无拥堵岔路（WP6b / Issue #6）。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`JunctionTask.step()`，就自动接入主流程，不需要改 main.py 或协调器。

**这个名额是从零写**：竞速工程里连一点参考都没有——它的地图模型是单条闭合
折线、没有拓扑，lap_manager 是圈数状态机而不是路段状态机，race_v32_planner
自述"仅供显示和日志"。所以动手前先和负责人确认场地几何和判据。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：选路动作必须拆成每帧一小步，不许阻塞。

必须实现的接口（签名已冻结，不要改）：
    class JunctionTask:
        name = "junction"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没有岔路   -> TaskUpdate(TaskStatus.NOT_TRIGGERED)
  * 正在选路   -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
  * 选完/通过   -> TaskUpdate(TaskStatus.COMPLETED)
  * 判据不明/失败 -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

**判据不明确时必须选择停车，不许猜方向。**

骨架已经替你实现的安全护栏（你不用自己写）：
  * 前进 <= 0.30 m/s、横移 <= 0.25 m/s、yaw <= 90 deg/s，超出会被裁掉并记录；
  * 单个任务连续接管超过 20 秒会被强制释放并硬停车；
  * 完成后骨架会硬停车、清巡线历史、等一张新鲜有效路线再恢复。

先看 tests/test_junction_task.py 的用例区再动手。
单独自测：python scripts/check_module.py junction_task
"""

from typing import Optional

from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection

KIND = "junction"
GREEN_BRANCH = "green_branch"
FREE_BRANCH = "free_branch"


class JunctionTask:
    """TODO(WP6b)：把 detect() 换成真实岔路判据，再补 step() 的选路状态机。"""

    name = "junction"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = settings
        self.last_detection = VisualDetection.no_result(KIND)
        self.started_at = None

    def detect(self, frame) -> VisualDetection:
        """TODO(WP6b)：返回岔路的类型与位置；不是岔路就返回 no_result。"""
        return VisualDetection.no_result(KIND)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection
        # 空实现：永远不接管。普通弯道和断口都不该被误判成岔路。
        # TODO(WP6b)：判据成立时从这里开始接管并选路；判据不成立就停车报失败。
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
