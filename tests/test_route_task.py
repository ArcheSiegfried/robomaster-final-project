"""长断线巡回（WP6a / Issue #6）。

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

from route_task import RouteTask  # noqa: E402


class RouteContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "route_task.py")

    def test_does_not_take_over_on_a_clear_line_frame(self):
        """合成帧里有一条清晰的蓝线。

        这条测试在你实现完之后**仍然必须通过**：能看到线的时候归巡线管，
        长断线任务只处理线真的断了的情况，不许抢正常巡线的活。
        """
        assert_inert_through_harness(self, RouteTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 触发条件明确：必须晚于基础底座的 lost_grace_seconds=0.28s，
#       不得靠延长短时容错冒充长断线
#   [ ] 巡逻动作有硬超时，假时钟下方向、限幅、左右扫描次序可断言
#   [ ] 重新找到线之后要连续确认若干帧才算重获（防止误抓邻近线）
#   [ ] 找不到线时停车并报告失败，不许无限扫描
#   [ ] 任务结束后归还控制权，需要新鲜有效路线才恢复
#
# 参考蓝本（这是全场最赚的一个名额，竞速工程有现成实现）：
#   F:\robomaster\blue_line_following\line_following_core.py:81-131 _search_decision()
#     —— 用最后看到的线路方向决定首扫方向、扫线期间前进强制为 0、
#        超时硬停、按 sweep_index 奇偶左右交替扫
#   F:\robomaster\blue_line_following\race_v32_config.py:186-201
#     —— MAX_SEARCH_TIME=1.80 / 首扫 0.28s / 全扫 0.56s / 扫线 340 deg/s /
#        REACQUIRE_STABLE_FRAMES=3 / REACQUIRE_MAX_ERROR_JUMP=0.45
#   F:\robomaster\blue_line_following\race_v32_config.py:58-63
#     —— 断口两侧真实线段的几何关联参数（GAP_FRAGMENT_*）
#
# 单独自测命令：
#   python scripts/check_module.py route_task
# ============================================================================


if __name__ == "__main__":
    unittest.main()
