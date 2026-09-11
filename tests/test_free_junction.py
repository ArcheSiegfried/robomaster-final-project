"""无拥堵岔路（WP6b / Issue #6）。

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

from free_junction import FreeJunctionTask  # noqa: E402


class FreeJunctionContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "free_junction.py")

    def test_does_not_take_over_on_a_single_line_frame(self):
        """合成帧里只有一条线，没有岔路。

        这条测试在你实现完之后**仍然必须通过**：不能把普通弯道当成岔路。
        """
        assert_inert_through_harness(self, FreeJunctionTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 岔路出现的判据（两条线/分叉几何），以及它与普通弯道、与断口的区分
#   [ ] "无拥堵"的判据来源必须先和负责人确认：靠视觉看另一侧有没有车？
#       还是靠固定规则？**判据没有可靠来源时，正确行为是停车报失败，不是猜。**
#   [ ] 选择动作有硬超时和限幅，随时可取消
#   [ ] 完成后归还控制权，需要新鲜有效路线才恢复
#   [ ] 与"绿灯岔路"（green_junction.py，另一个人负责）的分工不要重叠：
#       你不管灯色，只管拥堵判据
#
# 参考蓝本：**竞速工程里一条都没有**，这个名额是从零写。
# 注意：这条大概率会被负责人标成 needs-field-data（等场地和规则确认），
# 在此之前先做不依赖场地的部分——假帧夹具、状态机骨架、限幅与超时测试。
#
# 单独自测命令：
#   python scripts/check_module.py free_junction
# ============================================================================


if __name__ == "__main__":
    unittest.main()
