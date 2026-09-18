"""注册表顺序 = 撞车时的裁判顺序。这里把它钉死，改顺序必须先看见这条红。

2026-09-18：按**期末考场场景顺序**重排（用户批准）。依据是决赛要求 PDF
（16-Final_Project_Requirements v8）第 41 页的六项现场任务顺序，以及"第一个岔路口
是左红右绿"这个现场事实：`traffic_light` 只认颜色不认位置，排第 1 时红灯 2 帧就原地
锁停，会把 `green_junction` 的选路权整个吃掉（15 分），所以岔路选路必须排在它前面。
新顺序：green_junction → obstacle → free_junction → route → traffic_light → number_marker。
`traffic_light` 移位**不影响红灯停车**：`coordinator.py` 的红灯否决权按 name 查找，
与位次无关（见 `test_red_light_veto_has_a_module_to_ask`）。

2026-09-17：`traffic_light` 恢复（3 号重新提交，治好了"反复误判锁停"），当时放回第 1 位。
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
    "green_junction",   # 1 第一岔路：左红右绿，往绿灯那侧走（赛题 4）
    "obstacle",         # 2 绕障（赛题 3）
    "free_junction",    # 3 第二岔路：避开拥堵（赛题 5）
    "route",            # 4 断线恢复（赛题 2）
    "traffic_light",    # 5 自选地点红灯停绿灯行（赛题 6）
    "number_marker",    # 6 标识 1~5 拍照（赛题 1）
)


class RegistryOrderTests(unittest.TestCase):
    def test_takeover_order_is_pinned(self):
        """精确元组断言：任何一位调换都必须让这条变红。"""
        names = tuple(cls.name for cls in task_registry.MOTION_TASK_CLASSES)
        self.assertEqual(names, EXPECTED_ORDER)

    def test_green_junction_is_first(self):
        """岔路选路排第 1：考场第一岔路是左红右绿，选路权必须先给岔路模块。

        **注意这条只保证"岔路排在第一位被询问"，不保证锁停问题已解决。**
        实测（合成 Y 形岔路 + 左红右绿 + 真协调器，600 帧）：`traffic_light`
        只要 2 帧红灯就接管（`red_confirm_frames=2`），而 `green_junction`
        要 3 帧才确认岔路，所以灯模块仍然赢下这场赛跑；随后 `coordinator.py`
        的红灯否决权（`_step_active` 里先问红灯、否决时**不调用**当前任务的
        `step()`）会把岔路模块按停 —— 实测 599/600 帧零指令。
        排序对这件事**无能为力**，真正的修复要在灯模块/否决权那一层做。
        """
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertEqual(names.index("green_junction"), 0)

    def test_traffic_light_is_after_the_junctions(self):
        """红灯模块排在岔路选路之后：它管的是"自选地点"的红灯停绿灯行，不是岔路选路。

        移位不减少它的得分能力：红灯否决权按 name 查找（见下一条测试），
        任何模块开车时遇到红灯仍会被暂停。
        """
        names = [cls.name for cls in task_registry.MOTION_TASK_CLASSES]
        self.assertGreater(names.index("traffic_light"),
                           names.index("green_junction"))
        self.assertGreater(names.index("traffic_light"),
                           names.index("free_junction"))

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
        """红灯否决权靠"注册表里有个叫 traffic_light 的模块"生效，把它钉住。

        协调器是按 **name** 找它（coordinator.py 的 LIGHT_TASK_NAME），不是按位次，
        所以把它移到第 5 位之后否决权依然生效。
        """
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


if __name__ == "__main__":
    unittest.main()
