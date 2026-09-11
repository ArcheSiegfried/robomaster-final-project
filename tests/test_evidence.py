"""截图标注与任务证据（WP4 / Issue #4）。

这是观察型模块：每帧都会被调用，但**永远不能接管运动**。
契约测试在前，成员用例区在后。
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

from evidence import EvidenceRecorder  # noqa: E402


class EvidenceContractTests(unittest.TestCase):
    def test_module_source_obeys_the_safety_rules(self):
        """不得碰 SDK、相机、MotionOutput，不得阻塞或写死绝对路径。"""
        assert_module_source_is_clean(self, "evidence.py")

    def test_recorder_never_takes_over(self):
        """记录模块没有运动权限，任何帧都不许接管。

        这条测试在你实现完之后**仍然必须通过**。
        """
        assert_inert_through_harness(self, EvidenceRecorder(), observer=True)

    def test_recorder_is_called_on_every_frame(self):
        """骨架每帧都会调 observe()，包括别的任务正在接管时。"""
        recorder = EvidenceRecorder()
        harness = TaskHarness(observer=recorder)
        harness.start_line(now=1.0)
        for step in range(4):
            harness.feed_line(1.10 + step * 0.05, x=320 + step * 3)
        self.assertEqual(harness.owner, "line")


# ============================================================================
# 你的用例区（成员补充）。建议至少覆盖：
#   [ ] 用假 FramePacket + 假检测结果生成截图与结构化记录（临时目录，别写进仓库）
#   [ ] 一次任务只记录一次；重复事件的去重/命名规则明确
#   [ ] 路径是项目内相对路径或运行时目录，不含个人绝对路径
#   [ ] 写盘失败可报告，且不影响安全停车
#   [ ] 主循环调用是短操作：observe() 只入内存队列，异步或节流落盘
#   [ ] .gitignore 能排除运行产物（captures/ 之类）
#
# 参考蓝本（本批最值得搬的一段，已被 60+ 次真实运行验证不阻塞）：
#   F:\robomaster\blue_line_following\race_telemetry.py:109-117 建目录/开文件/DictWriter
#   F:\robomaster\blue_line_following\race_telemetry.py:296-299 每帧只 writerow，
#       flush 节流到 0.5s 一次
#   F:\robomaster\blue_line_following\trajectory_io.py:191-205 原子写（.tmp → replace）
#       加 allow_nan=False
# 注意竞速工程 .gitignore 只忽略了 logs/，track_map.json 和 reports/*.json 都没忽略，
# 别重复这个错。
#
# 单独自测命令：
#   python scripts/check_module.py evidence
# ============================================================================


if __name__ == "__main__":
    unittest.main()
