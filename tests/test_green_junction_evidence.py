"""绿岔路的"得分照片"回归（集成侧统一拍照层 + 作者新版转向逻辑）。

背景：老师后来明确要求 5.3 岔路选择必须存一张证据照片（**用圆圈标出识别到的灯** +
写明灯在哪条路 / 我们选哪条路）。这一组测试证明：把作者 `feat/two-lights` 新版
（A21~A28 的转向重写）拿来之后，**拍照接入仍然生效、且不影响转向**。

用他 `tests/test_green_junction.py` 里的夹具（`fork_scene` / `two_lamp_frame` /
`green_lamp_frame` / `LampSpotter`），不新造画面。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from green_junction import (  # noqa: E402
    Branch,
    GreenJunctionTask,
    JunctionState,
    LampSpotter,
    make_light_probe,
)
from models import TaskStatus  # noqa: E402
from tests.task_harness import TaskHarness  # noqa: E402
from tests.test_green_junction import fork_scene  # noqa: E402

TEAM = "10"


def drive_until_choice(task, harness, chosen: Branch, lamp_x: float, frames: int = 120):
    """照作者测试的写法把车开到"选边"那一刻；返回 (now, 决策列表)。"""
    now = 1.05
    car_yaw = 0.0
    travel = 0.0
    seen = []
    for _ in range(frames):
        now += 0.05
        image = fork_scene(car_yaw, travel=travel, chosen=chosen, lamp="green", lamp_x=lamp_x)
        decision = harness.feed_image(now, image)
        seen.append(decision)
        command = getattr(decision, "command", None)
        if harness.owner == "external" and command is not None:
            car_yaw += float(command.yaw) * 0.05
            travel += float(command.forward) * 0.05
        if task.chosen_branch is not None:
            break
    return now, seen


class WireEvidencePhotoTests(unittest.TestCase):
    def _task_and_harness(self):
        spotter = LampSpotter()
        task = GreenJunctionTask(light_probe=make_light_probe(spotter))
        harness = TaskHarness(task=task)
        harness.start_line(now=1.0)
        return task, harness

    def test_choosing_the_left_green_branch_queues_one_circled_photo(self):
        task, harness = self._task_and_harness()
        drive_until_choice(task, harness, Branch.LEFT, lamp_x=100.0)
        self.assertEqual(task.chosen_branch, Branch.LEFT, "没走到选边那一刻")

        request = task.take_evidence_request()
        self.assertIsNotNone(request, "选边那一刻没有排队照片")
        self.assertEqual(request.annotation, "Team %s detects a green light on the left way and chooses left" % TEAM)
        self.assertEqual(request.shape, "circle", "岔路照片要用圆圈标灯")
        self.assertIsNotNone(request.image)
        # 第二次取应该没有了（一个事件只排一张）
        self.assertIsNone(task.take_evidence_request())
        # 圈用的是检测器真的量到的灯心（灯在画面左侧）
        if request.detection is not None and request.detection.box is not None:
            left, top, right, bottom = request.detection.box
            self.assertLess((left + right) / 2.0, 320.0, "圈出来的灯应该在画面左侧")

    def test_a_failed_write_never_changes_the_turn(self):
        """照片写盘失败（回执 False）绝不允许改变转向：仍要按他的逻辑转完并完成。"""
        task, harness = self._task_and_harness()
        now, _ = drive_until_choice(task, harness, Branch.LEFT, lamp_x=100.0)
        request = task.take_evidence_request()
        self.assertIsNotNone(request)
        # 两次写盘失败（模块的重试上限是 2）
        self.assertTrue(task.acknowledge_evidence(request.request_id, False))
        retry = task.take_evidence_request()
        if retry is not None:
            task.acknowledge_evidence(retry.request_id, False)

        # 继续开：必须仍然自己转完、进 COMPLETED，而不是因为照片失败停车
        car_yaw = 0.0
        travel = 0.0
        status = None
        for _ in range(200):
            now += 0.05
            image = fork_scene(car_yaw, travel=travel, chosen=Branch.LEFT, lamp="green", lamp_x=100.0)
            decision = harness.feed_image(now, image)
            command = getattr(decision, "command", None)
            if harness.owner == "external" and command is not None:
                car_yaw += float(command.yaw) * 0.05
                travel += float(command.forward) * 0.05
            status = getattr(getattr(decision, "task_update", None), "status", None)
            if task.finished:
                break
        self.assertEqual(task.chosen_branch, Branch.LEFT)
        self.assertEqual(task.state, JunctionState.COMPLETED, "照片失败把转向搞坏了")
        self.assertNotEqual(status, TaskStatus.FAILED)


if __name__ == "__main__":
    unittest.main()
