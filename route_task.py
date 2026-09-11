"""长断线巡回（WP6a / Issue #6）。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`RouteTask.step()`，就自动接入主流程，不需要改 main.py 或协调器。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：巡逻和扫线都必须是"每帧一小步"，不许阻塞。

**最重要的一条**：长断线是外部任务，**不许**通过延长基础底座的
lost_grace_seconds（0.28 秒）来冒充。基础底座的短时容错归巡线管，
本模块只处理线真的断了以后的情况。

允许接管的时机（骨架已经强制）：基础巡线处于 TRACKING / COASTING / LINE_LOST。
也就是说线刚断、lock 之前你能接管，lock 之后（LINE_LOST）你也能接管；
但 VIDEO_LOST 和人工暂停（STOPPED）时你绝不能接管，模块不可能自己启动车。

必须实现的接口（签名已冻结，不要改）：
    class RouteTask:
        name = "route"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 线还在     -> TaskUpdate(TaskStatus.NOT_TRIGGERED)，把活留给巡线
  * 正在巡逻   -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
  * 找回线     -> TaskUpdate(TaskStatus.COMPLETED)
  * 找不到/超时 -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续返回 RUNNING，直到 COMPLETED 或 FAILED。

骨架已经替你实现的安全护栏（你不用自己写）：
  * 前进 <= 0.30 m/s、横移 <= 0.25 m/s、yaw <= 90 deg/s，超出会被裁掉并记录；
  * 单个任务连续接管超过 20 秒会被强制释放并硬停车——**你自己也要有硬超时**，
    不许无限扫描、无限转圈；
  * 完成后骨架会硬停车、清巡线历史、等一张新鲜有效路线再恢复。

参考蓝本（全场最赚的一个名额，竞速工程有现成实现，照着改）：
  * blue_line_following/line_following_core.py:81-131 `_search_decision()`
      —— 用最后看到的线路方向决定首扫方向、扫线期间前进强制为 0、
         超过搜索预算硬停、按 sweep_index 奇偶左右交替扫
  * blue_line_following/race_v32_config.py:186-201
      —— MAX_SEARCH_TIME=1.80 / 首扫 0.28s / 全扫 0.56s / 扫线 340 deg/s /
         REACQUIRE_STABLE_FRAMES=3 / REACQUIRE_MAX_ERROR_JUMP=0.45
  * blue_line_following/race_v32_config.py:58-63
      —— 断口两侧真实线段按几何关系续连的参数（GAP_FRAGMENT_*）

先看 tests/test_route_task.py 的用例区再动手。
单独自测：python scripts/check_module.py route_task
"""

from typing import Optional

from models import FramePacket, TaskStatus, TaskUpdate, VisualDetection

KIND = "route"
SEARCHING = "searching"
REACQUIRING = "reacquiring"


class RouteTask:
    """TODO(WP6a)：把 detect() 换成真实线路判断，再补 step() 的巡逻/重获状态机。"""

    name = "route"

    def __init__(self, settings: Optional[object] = None) -> None:
        self.settings = settings
        self.last_detection = VisualDetection.no_result(KIND)
        self.state = SEARCHING
        self.started_at = None
        self.sweep_direction = 1.0
        self.stable_frames = 0

    def detect(self, frame) -> VisualDetection:
        """TODO(WP6a)：返回线路的存在性与位置；线清楚时返回 no_result 交给巡线。"""
        return VisualDetection.no_result(KIND)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        detection = self.detect(frame.image)
        self.last_detection = detection
        # 空实现：永远不接管，正常巡线完全不受影响。
        # TODO(WP6a)：线断到需要外部搜索时从这里开始接管，
        #   左右交替扫线，扫到线后连续确认若干帧才算重获，再返回 COMPLETED。
        return TaskUpdate(TaskStatus.NOT_TRIGGERED, detection=detection)
