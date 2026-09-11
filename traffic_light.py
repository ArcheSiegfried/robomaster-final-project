"""红绿灯识别与停车/放行（WP3 / Issue #3）。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`TrafficLightTask.step()`，就自动接入主流程，不需要改 main.py 或协调器。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：不许长循环、不许 sleep、不许在整幅图上跑重算法。

必须实现的接口（签名已冻结，不要改）：
    class TrafficLightTask:
        name = "traffic_light"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没看到灯   -> TaskUpdate(TaskStatus.NOT_TRIGGERED)。**绝不许把"没看到红灯"
                  当成绿灯放行。**
  * 红灯期间   -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand()) 保持停车
  * 绿灯放行   -> 满足连续确认后 TaskUpdate(TaskStatus.COMPLETED)
  * 失败       -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

骨架已经替你实现的安全护栏（你不用自己写）：
  * 前进 <= 0.30 m/s、横移 <= 0.25 m/s、yaw <= 90 deg/s，超出会被裁掉并记录；
  * nan/inf 会被置零；
  * 单个任务连续接管超过 20 秒会被强制释放并硬停车；
  * 只有基础巡线处于运行或丢线状态时模块才能接管，模块不可能自己启动车；
  * 红灯锁停不会被巡线自动恢复覆盖：交回控制权后必须出现新鲜有效路线才恢复。

参考蓝本：竞速工程没有任何灯识别实现。颜色管线可以照抄
  blue_line_following/blue_line_detector.py:354-363
（ROI -> HSV -> inRange -> 开运算/闭运算），把 HSV 区间换成红/绿。
连续确认的迟滞参数模型参考 blue_line_following/race_v32_config.py:96-102
（START_CONFIDENCE / KEEP_CONFIDENCE / START_STABLE_FRAMES）。

先看 tests/test_traffic_light.py 的用例区再动手。
单独自测：python scripts/check_module.py traffic_light
"""

from typing import Optional

from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection

KIND = "traffic_light"
RED = "red"
GREEN = "green"


class TrafficLightTask:
    """TODO(WP3)：把 detect() 换成真实颜色判断，再补 step() 的确认状态机。"""

    name = "traffic_light"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = settings
        self.last_detection = VisualDetection.no_result(KIND)
        self.confirmations = 0

    def detect(self, frame) -> VisualDetection:
        """TODO(WP3)：返回 color=RED / GREEN 的判定，拿不准就返回 no_result。"""
        return VisualDetection.no_result(KIND)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection
        if not detection.valid:
            return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
        # TODO(WP3)：从这里开始你的状态机。
        #   红灯：返回 RUNNING 且 motion=MotionCommand()（零运动），保持停车；
        #   绿灯：连续确认若干帧之后返回 COMPLETED 才算放行；
        #   单帧抖动不许放行；全程不许 sleep，用 now 做假时钟可测的时间判断。
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
