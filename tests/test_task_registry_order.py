"""注册表顺序 = 撞车时的裁判顺序。这里把它钉死，改顺序必须先看见这条红。

2026-09-18：按**期末考场场景顺序**重排（用户批准）。**这不是 PDF 的任务编号顺序**：
决赛要求 PDF（16-Final_Project_Requirements v8）第 41 页是按 1 标识 → 2 断线 →
3 绕障 → 4 岔路选绿灯 → 5 岔路避拥堵 → 6 红灯停 逐条描述任务的，并没有规定接管优先级。
本顺序来自用户给出的考场实际遭遇顺序，再叠加两条判断（断线恢复是"线消失"才触发的
独立场景、优先级高于"选路"；标识只拍照不动作、排最后）。另外"第一个岔路口是左红右绿"
这个现场事实要求岔路选路排在 `traffic_light` 前面。
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

        注意这条只保证"岔路排在第一位被询问"。它**不是**岔路锁停问题的修复：
        修复在同分支的 `traffic_light.py`（红绿同框闸门 `fork_light_competition`）。
        修复前的实测（合成 Y 形岔路 + 左红右绿 + 真协调器，600 帧）：`traffic_light`
        只要 2 帧红灯就接管（`red_confirm_frames=2`），而 `green_junction` 要 3 帧
        才确认岔路，灯模块赢下赛跑后，`coordinator.py` 的红灯否决权（否决时
        **不调用**当前任务的 `step()`）把岔路模块按成 599/600 帧零指令。
        加上闸门后同一场景为 4/600，覆盖测试见
        `tests/test_fork_light_competition.py`。
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
