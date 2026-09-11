"""数字标识 1~5 识别、筛选与居中（WP2 / Issue #2）。

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

from number_marker import NumberMarkerTask  # noqa: E402


class NumberMarkerContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "number_marker.py")

    def test_does_not_take_over_on_a_plain_line_frame(self):
        """合成帧里只有一条蓝线，没有任何数字标识。

        这条测试在你实现完之后**仍然必须通过**：没有目标就不许接管。
        如果它开始失败，说明模块会误触发，先修误触发再谈功能。
        """
        assert_inert_through_harness(self, NumberMarkerTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 数字 1~5 各自能识别，center/box 坐标符合 MODULE_GUIDE 的整幅像素约定
#   [ ] 无目标时返回 VisualDetection.no_result("number_marker")，不虚构坐标
#   [ ] 非目标图案（其他数字、纯色块）不误检
#   [ ] 同帧多个目标时的选择规则，以及重复目标的去重依据
#   [ ] 居中请求的方向与限幅（用 TaskHarness 断言 command.yaw / forward 符号和上限）
#   [ ] 完成与失败都会归还控制权（harness.owner 回到 "line"）
#   [ ] 统计误检/漏检数量，不要只展示成功样例
#
# 参考蓝本：竞速工程里没有数字识别的任何实现，只能借
#   F:\robomaster\blue_line_following\blue_line_detector.py:186-236
# 的"多候选打分选最优"框架改造。
#
# 单独自测命令：
#   python scripts/check_module.py number_marker
# ============================================================================


if __name__ == "__main__":
    unittest.main()
