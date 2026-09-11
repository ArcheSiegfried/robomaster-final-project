"""两类岔路：绿灯岔路 + 无拥堵岔路（WP6b / Issue #6）。

契约测试在前，成员用例区在后。骨架已经把本文件注册进 task_registry，
你只要替换上面的实现即可，不需要改 main.py。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.task_harness import (  # noqa: E402
    TaskHarness,
    assert_inert_through_harness,
    assert_module_source_is_clean,
)

from junction import JunctionTask  # noqa: E402


class JunctionContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "junction.py")

    def test_does_not_take_over_on_a_single_line_frame(self):
        """合成帧里只有一条线，没有岔路。

        这条测试在你实现完之后**仍然必须通过**：不能把普通弯道当成岔路。
        """
        assert_inert_through_harness(self, JunctionTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 岔路出现的判据（两条线/分叉几何），以及它与普通弯道、与断口的区分
#   [ ] 两类岔路各自的触发、方向选择、完成与失败条件，都能分开断言
#   [ ] 选择动作有硬超时和限幅，随时可取消
#   [ ] 完成后归还控制权，需要新鲜有效路线才恢复
#   [ ] 判据不明确时**必须选择停车**而不是猜方向
#
# 参考蓝本：**竞速工程里一条都没有**。它的地图模型是单条闭合折线、没有拓扑，
# lap_manager 是圈数状态机而不是路段状态机，race_v32_planner 自述"仅供显示和日志"。
# 所以这个名额是从零写，先和负责人确认场地几何再动手。
#
# 单独自测命令：
#   python scripts/check_module.py junction
# ============================================================================


if __name__ == "__main__":
    unittest.main()
