"""离线联调演示：巡线 → 岔路接管 → 完成 → 交回巡线。

只使用合成画面和内存里的假底盘：**不连机器人、不连相机、不跑 main.py**。

    python -m examples.green_junction_demo

输出是一串事件名，可以直接粘进 PR 描述作为运行证据。
"""

from typing import List, Optional

import cv2
import numpy as np

from config import CONFIG
from green_junction import (
    Branch,
    GreenJunctionTask,
    LightColor,
    LightReading,
)
from models import FramePacket, TaskStatus
from motion_output import MotionOutput
from runtime import LineFollower

WIDTH, HEIGHT = 640, 360
TAPE = (255, 0, 0)
FRAME_DT = 0.10


class FakeChassis:
    """内存记录命令，没有 SDK，也没有硬件。"""

    def __init__(self) -> None:
        self.calls = []

    def drive_speed(self, **kwargs) -> None:
        self.calls.append(("speed", kwargs))

    def drive_wheels(self, **kwargs) -> None:
        self.calls.append(("wheels", kwargs))


def _ground() -> np.ndarray:
    return np.full((HEIGHT, WIDTH, 3), 200, np.uint8)


def straight_frame(x: int = 320) -> np.ndarray:
    image = _ground()
    cv2.line(image, (x, 350), (x, 180), TAPE, 24)
    return image


def junction_frame(split_y: int = 220) -> np.ndarray:
    image = _ground()
    cv2.line(image, (WIDTH // 2, split_y), (WIDTH // 2, split_y - 90), TAPE, 24)
    for sign in (-1, 1):
        cv2.line(
            image,
            (WIDTH // 2, split_y),
            (WIDTH // 2 + sign * 96, HEIGHT - 2),
            TAPE,
            24,
        )
    return image


def green_right(frame: FramePacket, now: float) -> Optional[LightReading]:
    """假的红绿灯模块：这里永远报"右边是绿灯"。"""
    return LightReading(color=LightColor.GREEN, branch=Branch.RIGHT)


def run_demo(light_probe=green_right) -> List[str]:
    """返回事件轨迹；任何一步不符合预期就抛异常。"""
    follower = LineFollower(CONFIG)
    chassis = FakeChassis()
    output = MotionOutput(chassis, CONFIG)
    task = GreenJunctionTask(light_probe=light_probe)
    trace: List[str] = []

    # 1) 巡线先跑起来
    first = FramePacket(straight_frame(), sequence=1, captured_at=1.00)
    follower.process_frame(first.image, first.captured_at)
    if not follower.resume(1.00):
        raise RuntimeError("demo could not start on a valid line")
    now = 1.05
    tracking = follower.process_frame(straight_frame(340), now)
    trace.append("line:%s" % tracking.state)

    # 2) 车开到岔路口：模块确认几帧后接管
    follower.pause(now)
    output.claim("external")
    trace.append("owner:%s" % output.owner)
    now += FRAME_DT
    update = task.step(FramePacket(junction_frame(), 2, now), now, tracking.detection)
    if update.status is not TaskStatus.NOT_TRIGGERED:
        raise RuntimeError("junction confirmed on the first frame; expected confirmation")
    now += FRAME_DT
    update = task.step(FramePacket(junction_frame(), 3, now), now, tracking.detection)
    now += FRAME_DT
    update = task.step(FramePacket(junction_frame(), 4, now), now, tracking.detection)
    if update.status is not TaskStatus.RUNNING:
        raise RuntimeError("task did not take over at the junction")
    trace.append("task:running")

    # 3) 转向：请求被送到唯一运动出口
    now += FRAME_DT
    update = task.step(FramePacket(junction_frame(), 5, now), now, tracking.detection)
    if task.chosen_branch is None:
        # 没有灯判据时模块只能停着等，最后必然失败；这是设计好的保守行为。
        trace.append("task:waiting:%s" % update.message)
        return trace
    output.send("external", update.motion)
    trace.append("branch:%s" % task.chosen_branch.value)
    trace.append("motion:yaw=%.1f" % update.motion.yaw)

    # 4) 继续转够 turn_min_duration：车的相机在转向时仍能看到岔路，
    #    同时巡线模块已经在左/右分支上重新找到线并居中 → 完成
    sequence = 6
    update = None
    for _ in range(6):
        now += FRAME_DT
        decision = follower.process_frame(straight_frame(), now)
        update = task.step(
            FramePacket(junction_frame(), sequence, now), now, decision.detection
        )
        sequence += 1
        if update.status is TaskStatus.COMPLETED:
            break
    if update.status is not TaskStatus.COMPLETED:
        raise RuntimeError("task did not complete: %s" % update.message)
    trace.append("task:%s" % update.status.value)

    # 5) 交回巡线：先零运动、再交权、清历史、新帧确认、显式恢复
    output.hard_stop()
    output.claim("line")
    trace.append("owner:%s" % output.owner)
    follower.reset_fault(now)
    fresh = FramePacket(straight_frame(), 9, now + 0.01)
    follower.process_frame(fresh.image, fresh.captured_at)
    if not follower.resume(fresh.captured_at):
        raise RuntimeError("line did not resume after the junction")
    trace.append("line:%s" % follower.state)
    return trace


if __name__ == "__main__":
    for event in run_demo():
        print(event)
