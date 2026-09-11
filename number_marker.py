"""数字标识 1~5 识别、筛选与居中（WP2 / Issue #2）。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`NumberMarkerTask.step()`，就自动接入主流程，不需要改 main.py 或协调器。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：不许长循环、不许 sleep、不许在整幅图上跑重算法。

必须实现的接口（签名已冻结，不要改）：
    class NumberMarkerTask:
        name = "number_marker"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没看到目标 -> TaskUpdate(TaskStatus.NOT_TRIGGERED)，不要虚构坐标。
  * 正在处理   -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
                  只有 RUNNING 才会拿到控制权；motion=None 表示接管但要求停车。
  * 处理完成   -> TaskUpdate(TaskStatus.COMPLETED)
  * 失败       -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。
  中途返回 NOT_TRIGGERED 会被协调器判为失败并强制归还控制权。

骨架已经替你实现的安全护栏（你不用自己写）：
  * 前进 <= 0.30 m/s、横移 <= 0.25 m/s、yaw <= 90 deg/s，超出会被裁掉并记录；
  * nan/inf 会被置零；
  * 单个任务连续接管超过 20 秒会被强制释放并硬停车；
  * 只有基础巡线处于运行或丢线状态时模块才能接管，模块不可能自己启动车。

参考蓝本：竞速工程里没有数字识别的任何实现，只能借
  blue_line_following/blue_line_detector.py:186-236 的"多候选打分选最优"框架。

先看 tests/test_number_marker.py 的用例区再动手。
单独自测：python scripts/check_module.py number_marker
"""

from typing import Optional

from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection

KIND = "number_marker"
SUPPORTED_IDS = ("1", "2", "3", "4", "5")


class NumberMarkerTask:
    """TODO(WP2)：把 detect() 换成真实识别，再补 step() 的接管逻辑。"""

    name = "number_marker"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = settings
        self.last_detection = VisualDetection.no_result(KIND)

    def detect(self, frame) -> VisualDetection:
        """TODO(WP2)：返回整幅图像像素坐标的识别结果。

        没有结果一定要用 VisualDetection.no_result(KIND)，不要用假坐标顶替。
        """
        return VisualDetection.no_result(KIND)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection
        if not detection.valid:
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
        # TODO(WP2)：目标已经在画面里，从这里开始接管：
        #   返回 RUNNING + MotionCommand 做居中，做完返回 COMPLETED，出错返回 FAILED。
        #   接管期间不许返回 NOT_TRIGGERED。完成后把控制权交回，骨架会自动处理
        #   硬停车、清巡线历史、等一张新鲜有效路线再恢复。
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
