"""红绿灯识别与停车/放行（WP3 / Issue #3）。

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

from traffic_light import TrafficLightTask  # noqa: E402


class TrafficLightContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "traffic_light.py")

    def test_does_not_take_over_on_a_plain_line_frame(self):
        """合成帧里只有一条蓝线，没有任何灯。

        这条测试在你实现完之后**仍然必须通过**：没看到灯就不许接管，
        更不许把"没看到红灯"当成绿灯放行。
        """
        assert_inert_through_harness(self, TrafficLightTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 红 / 绿 / 无灯 / 反光或相似颜色四类结果都有测试
#   [ ] 单帧抖动不放行：连续确认（参考 START_CONFIDENCE=0.55 / KEEP_CONFIDENCE=0.40
#       的迟滞思路）与假时钟下的状态转换
#   [ ] 红灯期间输出零运动，且不会被巡线自动恢复覆盖
#   [ ] 绿灯放行后明确归还控制权（harness.owner 回到 "line"）
#   [ ] 超时与失败路径都会停车
#   [ ] 全过程不依赖长时间阻塞（step 必须立刻返回）
#
# 参考蓝本：竞速工程没有任何灯识别实现；颜色管线可以照抄
#   F:\robomaster\blue_line_following\blue_line_detector.py:354-363
# （ROI → HSV → inRange → 开/闭运算），把 HSV 区间换成红/绿即可。
# 连续确认的参数模型参考 race_v32_config.py:96-102。
#
# 单独自测命令：
#   python scripts/check_module.py traffic_light
# ============================================================================


if __name__ == "__main__":
    unittest.main()
