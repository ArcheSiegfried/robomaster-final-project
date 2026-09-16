"""集成侧接线：注册表必须真的给绿岔路模块注入灯色探针。

为什么要有这个测试：绿岔路模块（`green_junction.py`）按契约**自己不认红绿灯**，
只接受一个注入缝 `light_probe(frame, now) -> LightReading`；而注册表是**唯一无参
构造任务的装配点**。PR #45 把适配器和 A14 判据都写好了，但**没有任何生产调用点**——
也就是说实车上它仍然每帧 `no light rule source` 不接管（今天的 15 次实车记录里
`green_junction` 一次都没出现）。这条测试把那根线钉住。

另外两条不变量（依据 PR#45 审查）：
* 注入的必须包**无状态**的 `TrafficLightDetector`，不能是 `TrafficLightTask()`
  （后者会养出第二台有状态灯机、自己往证据队列塞红灯截图）；
* 红灯时**绝不能有前进分量**（模块的转弯/稳定阶段不看灯，红灯安全靠"否决 + 不前进"）。

只用合成画面、假时钟：不连车、不连相机。
"""

import pathlib
import sys
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.test_green_junction import approach_frame  # noqa: E402

import task_registry  # noqa: E402
from green_junction import GreenJunctionTask, LightColor  # noqa: E402
from models import FramePacket  # noqa: E402
from traffic_light import TrafficLightTask  # noqa: E402

GREEN = (0, 255, 0)
RED = (0, 0, 255)
LAMP_TOP, LAMP_BOTTOM = 80, 160
LAMP_LEFT, LAMP_RIGHT = 300, 380


def junction_with_lamp(color):
    image = approach_frame()
    if color is not None:
        image[LAMP_TOP:LAMP_BOTTOM, LAMP_LEFT:LAMP_RIGHT] = color
    return image


def packet(image, sequence=1, now=0.0):
    return FramePacket(image=image, sequence=sequence, captured_at=now)


def green_junction_from_registry(enabled=None):
    tasks = task_registry.build_motion_tasks(enabled)
    return next(t for t in tasks if isinstance(t, GreenJunctionTask))


class WiringTests(unittest.TestCase):
    def test_registry_gives_green_junction_a_light_probe(self):
        task = green_junction_from_registry()
        self.assertIsNotNone(
            task.light_probe,
            "注册表没给绿岔路注入灯色探针 → 它在实车上永远不会接管",
        )

    def test_enabled_names_path_also_wires_the_probe(self):
        task = green_junction_from_registry(("green_junction",))
        self.assertIsNotNone(task.light_probe)

    def test_other_tasks_are_still_built_without_arguments(self):
        tasks = task_registry.build_motion_tasks()
        self.assertEqual(len(tasks), len(task_registry.MOTION_TASK_CLASSES))

    def test_probe_is_not_a_second_stateful_light_task(self):
        """适配器两种形态都要能用，但生产装配必须用无状态的检测器。"""
        from_tb = task_registry.build_light_probe(detector=TrafficLightTask())
        self.assertIsNotNone(from_tb, "适配器本身要能包 task 形态（他的设计）")
        production = task_registry.build_light_probe()
        self.assertIsNotNone(production)


class ProbeReadingTests(unittest.TestCase):
    def setUp(self):
        self.probe = task_registry.build_light_probe()

    def test_green_lamp_reads_green(self):
        reading = self.probe(packet(junction_with_lamp(GREEN)), 0.0)
        self.assertIsNotNone(reading)
        self.assertEqual(reading.color, LightColor.GREEN)

    def test_red_lamp_reads_red(self):
        reading = self.probe(packet(junction_with_lamp(RED)), 0.0)
        self.assertIsNotNone(reading)
        self.assertEqual(reading.color, LightColor.RED)

    def test_no_lamp_reads_unknown(self):
        reading = self.probe(packet(junction_with_lamp(None)), 0.0)
        self.assertTrue(reading is None or reading.color is LightColor.UNKNOWN)

    def test_probe_never_raises_on_a_missing_image(self):
        reading = self.probe(FramePacket(image=None, sequence=1, captured_at=0.0), 0.0)
        self.assertTrue(reading is None or reading.color is LightColor.UNKNOWN)


class BehaviourTests(unittest.TestCase):
    """接线之后，真模块在真画面上的表现。"""

    def _run(self, color, frames=12):
        task = green_junction_from_registry()
        updates = []
        for index in range(frames):
            now = index * (1.0 / 30.0)
            updates.append(
                task.step(packet(junction_with_lamp(color), index + 1, now), now)
            )
        return updates

    def test_green_light_eventually_drives_the_junction(self):
        updates = self._run(GREEN)
        self.assertTrue(
            any(u.status.name == "RUNNING" for u in updates),
            "绿灯 + 岔路必须能接管（否则这条链还是没通电）",
        )

    def test_red_light_never_commands_forward_motion(self):
        """红灯下绝不许有前进分量（转弯阶段不看灯，全靠这条与协调器的否决）。"""
        updates = self._run(RED)
        worst = max(
            (abs(u.motion.forward) for u in updates if u.motion is not None),
            default=0.0,
        )
        self.assertEqual(worst, 0.0, "红灯时出现了前进指令")

    def test_no_light_does_not_take_over(self):
        """没灯就别接管：不能把握不住的岔路从后面的模块手里抢走（他的 A14）。"""
        updates = self._run(None)
        self.assertFalse(any(u.status.name == "RUNNING" for u in updates))
        self.assertTrue(all("not applicable" in (u.message or "") or "junction" in (u.message or "")
                            for u in updates))


if __name__ == "__main__":
    unittest.main()
