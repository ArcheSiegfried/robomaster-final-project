"""注册表顺序 = 撞车时的裁判顺序。这里把它钉死，改顺序必须先看见这条红。

2026-09-17：`traffic_light` 恢复（3 号重新提交，治好了"反复误判锁停"），放回 **第 1 位**
（与集成负责人确认过的优先顺序一致：红绿灯 → 红绿灯岔路 → 障碍物绕行 → 短线巡回 →
障碍物岔路 → 数字识别）。它一回来，`coordinator.py` 的"红灯否决权"也随之重新生效。
2026-09-16：`number_marker` 从第 2 位降到最末（它在"看到了标识"时不接管，实际表现是抢在
岔路/红绿灯前面却不干活）。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import task_registry  # noqa: E402

EXPECTED_ORDER = (
    "traffic_light",    # 1 红绿灯（红灯停、绿灯确认后放行）
    "green_junction",   # 2 红绿灯岔路
    "obstacle",         # 3 障碍物绕行
    "route",            # 4 短线巡回
    "free_junction",    # 5 障碍物岔路
    "number_marker",    # 6 数字识别
)


class RegistryOrderTests(unittest.TestCase):
    def test_takeover_order_is_pinned(self):
        names = tuple(cls.name for cls in task_registry.MOTION_TASK_CLASSES)
        self.assertEqual(names, EXPECTED_ORDER)

    def test_traffic_light_is_first(self):
        """红灯排第 1：停车是安全项，不能被任何"正在开车"的模块挡住。"""
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertEqual(names.index("traffic_light"), 0)

    def test_number_marker_is_last(self):
        """数字识别排最后：它目前在实车上"看到标识却不接管"，先让会动作的模块先上。"""
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertEqual(names.index("number_marker"), len(names) - 1)

    def test_obstacle_outranks_the_junctions_and_the_marker(self):
        """障碍物绕行在叉路之前：车前方的障碍必须先处理。"""
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertLess(names.index("obstacle"), names.index("free_junction"))
        self.assertLess(names.index("obstacle"), names.index("route"))
        self.assertLess(names.index("obstacle"), names.index("number_marker"))

    def test_red_light_veto_has_a_module_to_ask(self):
        """红灯否决权靠"注册表里有个叫 traffic_light 的模块"生效，把它钉住。"""
        import coordinator
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertIn(coordinator.LIGHT_TASK_NAME, names)

    def test_every_registered_task_appears_once(self):
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertEqual(len(names), len(set(names)), "同一个模块不能登记两次")

    def test_built_tasks_follow_the_same_order(self):
        built = [task.name for task in task_registry.build_motion_tasks()]
        self.assertEqual(tuple(built), EXPECTED_ORDER,
                         "装出来的实例顺序必须和注册表一致（终端显示的就是它）")


class PriorityTableTests(unittest.TestCase):
    """显式优先级表必须与"顺序即优先级"完全一致（2026-09-18 仲裁层 v0.3）。"""

    def test_priority_table_matches_the_confirmed_order(self):
        ordered = [
            name
            for name, _ in sorted(
                task_registry.TASK_PRIORITIES.items(), key=lambda item: -item[1]
            )
        ]
        self.assertEqual(tuple(ordered), EXPECTED_ORDER)

    def test_safety_priority_constant_equals_the_top_module(self):
        top = max(task_registry.TASK_PRIORITIES, key=task_registry.TASK_PRIORITIES.get)
        self.assertEqual(top, "traffic_light")
        self.assertEqual(task_registry.TASK_PRIORITIES[top], task_registry.SAFETY_PRIORITY)

    def test_every_registered_class_exposes_its_priority(self):
        for cls in task_registry.MOTION_TASK_CLASSES:
            self.assertEqual(cls.priority, task_registry.TASK_PRIORITIES[cls.name])

    def test_ranked_task_classes_is_the_priority_order(self):
        self.assertEqual(
            tuple(cls.name for cls in task_registry.ranked_task_classes()),
            EXPECTED_ORDER,
        )

    def test_priority_of_accepts_a_name_a_class_and_an_instance(self):
        self.assertEqual(task_registry.priority_of("obstacle"), 70)
        self.assertEqual(task_registry.priority_of(task_registry.ObstacleTask), 70)
        self.assertEqual(task_registry.priority_of(task_registry.ObstacleTask()), 70)
        self.assertEqual(task_registry.priority_of("not_registered"), task_registry.LINE_PRIORITY)


if __name__ == "__main__":
    unittest.main()
