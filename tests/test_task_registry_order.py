"""注册表顺序 = 撞车时的裁判顺序。这里把它钉死，改顺序必须先看见这条红。

2026-09-16 按实车反馈调整：`number_marker` 从第 2 位降到第 5 位
（它在"看到了标识"时不会让车停下，实际表现是抢在岔路/红绿灯前面却不干活）。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import task_registry  # noqa: E402

EXPECTED_ORDER = (
    "traffic_light",
    "green_junction",
    "free_junction",
    "route",
    "number_marker",
    "obstacle",
)


class RegistryOrderTests(unittest.TestCase):
    def test_takeover_order_is_pinned(self):
        names = tuple(cls.name for cls in task_registry.MOTION_TASK_CLASSES)
        self.assertEqual(names, EXPECTED_ORDER)

    def test_number_marker_is_fifth(self):
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertEqual(names.index("number_marker"), 4, "数字标识要排在第 5 位")

    def test_every_registered_task_appears_once(self):
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertEqual(len(names), len(set(names)), "同一个模块不能登记两次")

    def test_built_tasks_follow_the_same_order(self):
        built = [task.name for task in task_registry.build_motion_tasks()]
        self.assertEqual(tuple(built), EXPECTED_ORDER,
                         "装出来的实例顺序必须和注册表一致（终端显示的就是它）")


if __name__ == "__main__":
    unittest.main()
