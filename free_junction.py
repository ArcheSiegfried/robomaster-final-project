"""无拥堵岔路（WP6b / Issue #6）。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`FreeJunctionTask.step()`，就自动接入主流程，不需要改 main.py 或协调器。

**这个文件只负责"无拥堵岔路"**：走到岔路口 → 判断哪一侧没有拥堵 →
选择并进入 → 完成后归还巡线。"绿灯岔路"是另一个人、另一个文件
（`green_junction.py`），**你不管灯色**，只管拥堵判据。

**这个名额是从零写，而且大概率排在被阻塞的一档**：竞速工程里没有岔路判断，
"拥堵"的判据也没有任何现成来源。先和负责人确认：
  * 靠视觉看另一侧有没有车？还是靠固定规则（例如总是走某一侧）？
  * 判据的可靠性到什么程度才允许自动选择？
**判据没有可靠来源时，正确行为是停车报失败，不是猜。** 在拿到场地规则之前，
先做不依赖场地的部分：假帧夹具、状态机骨架、限幅与超时测试。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：选路动作必须拆成每帧一小步，不许阻塞。

必须实现的接口（签名已冻结，不要改）：
    class FreeJunctionTask:
        name = "free_junction"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没到岔路 / 判据不成立 -> TaskUpdate(TaskStatus.NOT_TRIGGERED)
  * 正在选路             -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
  * 选完并通过           -> TaskUpdate(TaskStatus.COMPLETED)
  * 失败 / 判据不明       -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

骨架已经替你实现的安全护栏（你不用自己写）：
  * 前进 <= 0.30 m/s、横移 <= 0.25 m/s、yaw <= 90 deg/s，超出会被裁掉并记录；
  * nan/inf 一律置零；
  * 单个任务连续接管超过 20 秒会被强制释放并硬停车；
  * 只有基础巡线处于运行或丢线状态时模块才能接管，模块不可能自己启动车；
  * 完成后骨架会硬停车、清巡线历史、等一张新鲜有效路线再恢复。

先看 tests/test_free_junction.py 的用例区再动手。
单独自测：python scripts/check_module.py free_junction
"""

from typing import Optional

from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection

KIND = "free_junction"
BRANCH_LEFT = "left"
BRANCH_RIGHT = "right"


class FreeJunctionTask:
    """TODO(WP6b)：把 detect() 换成真实拥堵判据，再补 step() 的选路状态机。"""

    name = "free_junction"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = settings
        self.last_detection = VisualDetection.no_result(KIND)
        self.started_at = None
        self.chosen_direction = None

    def detect(self, frame) -> VisualDetection:
        """TODO(WP6b)：返回岔路位置与没有拥堵的一侧。

        不是岔路、或拥堵判据不成立时，一律返回 no_result(KIND)。
        """
        return VisualDetection.no_result(KIND)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection
        # 空实现：永远不接管。普通弯道和断口都不该被误判成岔路。
        # TODO(WP6b)：判据成立时从这里开始接管并选路；判据不成立就停车报失败。
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
