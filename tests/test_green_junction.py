"""绿灯岔路（WP6b / Issue #6）。

契约测试在前，成员用例区在后。骨架已经把这个文件登记进 task_registry，
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

from green_junction import GreenJunctionTask  # noqa: E402


class GreenJunctionContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "green_junction.py")

    def test_does_not_take_over_on_a_single_line_frame(self):
        """合成帧里只有一条线，没有岔路、也没有灯。

        这条测试在你实现完之后**仍然必须通过**：不能把普通弯道当成岔路。
        """
        assert_inert_through_harness(self, GreenJunctionTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 岔路出现的判据（两条线/分叉几何），以及它与普通弯道、与断口的区分
#   [ ] 绿灯侧的判定：单帧抖动不许选路，要有连续确认
#   [ ] 红灯或判据不明时**停车报失败**，绝不猜方向
#   [ ] 选路动作有硬超时和限幅，随时可取消
#   [ ] 完成后归还控制权，需要新鲜有效路线才恢复
#
# 参考蓝本：**竞速工程里一条都没有**。它的地图模型是单条闭合折线、没有拓扑，
# lap_manager 是圈数状态机而不是路段状态机。这个名额是从零写，
# 动手前先和负责人确认岔路几何、灯的摆放和选择规则。
#
# 依赖 #3 红绿灯：灯色由谁判断，你和负责红绿灯的同学约定好，
# 但两个文件都不要改公共文件。
#
# 单独自测命令：
#   python scripts/check_module.py green_junction
# ============================================================================


if __name__ == "__main__":
    unittest.main()
