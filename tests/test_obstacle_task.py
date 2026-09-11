"""障碍检测与绕行（WP5 / Issue #5）。

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

from obstacle_task import ObstacleTask  # noqa: E402


class ObstacleContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "obstacle_task.py")

    def test_does_not_take_over_on_a_plain_line_frame(self):
        """合成帧里只有一条蓝线，没有任何障碍。

        这条测试在你实现完之后**仍然必须通过**：没有障碍就不许接管，
        否则车会在空跑道上做出绕行动作。
        """
        assert_inert_through_harness(self, ObstacleTask())


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 有障碍 / 无障碍 / 误检候选 / 丢帧四类输入
#   [ ] 假时钟驱动每一步；前进、横移、yaw 都受骨架限幅约束
#       （骨架会强制裁到 config.TaskConfig 的 task_max_*，你不要依赖它放宽）
#   [ ] 每一段动作都有总时长上限，随时可取消
#   [ ] 完成后清巡线历史、要求新鲜有效路线才恢复（harness 已经帮你验证时序）
#   [ ] 失败和超时路径都会停车
#   [ ] 横移量级参考竞速工程的 MAX_LATERAL_SPEED=0.25 m/s
#
# 参考蓝本：竞速工程没有任何障碍检测实现，必须从零写。
# 轮廓筛选框架可以借 blue_line_detector.py:186-236。
#
# 单独自测命令：
#   python scripts/check_module.py obstacle_task
# ============================================================================


if __name__ == "__main__":
    unittest.main()
