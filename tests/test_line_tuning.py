"""巡线调参的底线测试：纠偏力度不许悄悄退回"软"的那一档。

背景（2026-09-15）：实车反馈是"偏了纠不回来 / 看到弯就冲出去"。对比同源的
race-v4 参考实现后发现，我们的 `ControlConfig` 一直是 `config.py` 注释里写的
"conservative starting values"，**从没在实车上标定过**：

    参考 race-v4（实车基准 1.50 m/s）: KP 310 / KD 4.5 / heading 前馈 260 / yaw 上限 550 / 变化率 2600
    我们（改动前）                  : KP 115 / KD 3.0 / heading 前馈  70 / yaw 上限 135 / 变化率  500

同样的误差序列下，我们在 403 秒真实轨迹上的平均 |yaw| 只有 22°/s，参考是 39°/s，
而且参考有 13.5% 的帧超过 90°/s，我们只有 1.3%。

这组测试是**地板**：它不规定该怎么调，只保证"中等偏大误差必须给出足够硬的转向"。
谁要把 KP 或转向上限调回软档，就必须先看见这些测试变红，并有意识地改掉它们。

只用假误差和假时钟，不连车、不连相机。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CONFIG  # noqa: E402
from controller import LineController  # noqa: E402

#: 半幅误差（0.5）下，0.2 秒内必须打到这个量级的 yaw。
#: 依据：参考实现 403 秒真实轨迹里 |yaw| 第 85 分位约 90°/s，均值 39°/s；
#: 取 100 表示"比它的平均力度更硬一点"，但远低于它的上限 550。
FIRM_YAW = 100.0

#: 转向变化率下限：0.2 秒内要能爬到 FIRM_YAW。
MIN_YAW_RATE = 900.0


class SteeringAuthorityTests(unittest.TestCase):
    def _after(self, error, frames=6, heading=0.0, dt=1.0 / 30.0):
        controller = LineController(CONFIG.control)
        controller.reset(0.0)
        command = None
        for index in range(frames):
            command = controller.track(error, heading, (index + 1) * dt)
        return command

    def test_half_scale_error_gets_a_firm_steering_command(self):
        command = self._after(0.5)
        self.assertGreaterEqual(
            abs(command.yaw), FIRM_YAW,
            "误差 0.5 时 0.2 秒内应该已经打出至少 %.0f°/s 的转向（当前 KP=%.0f）"
            % (FIRM_YAW, CONFIG.control.kp))

    def test_sign_is_still_correct(self):
        self.assertGreater(self._after(0.5).yaw, 0.0)
        self.assertLess(self._after(-0.5).yaw, 0.0)

    def test_steering_never_exceeds_the_configured_ceiling(self):
        for error in (0.5, 1.0, 1.5, 3.0):
            command = self._after(error)
            self.assertLessEqual(abs(command.yaw), CONFIG.control.max_yaw_speed)

    def test_steering_rate_is_high_enough_to_be_usable(self):
        self.assertGreaterEqual(
            CONFIG.control.max_yaw_rate, MIN_YAW_RATE,
            "转向变化率太小的话，误差再大也要花很久才打到目标值（'纠不回来'）")

    def test_curve_braking_is_stronger_than_acceleration(self):
        self.assertGreater(CONFIG.control.max_forward_deceleration,
                           CONFIG.control.max_forward_acceleration,
                           "入弯要刹得住：减速度必须大于加速度")

    def test_forward_speed_was_left_alone(self):
        """速度不是这次的症状，不许顺手提速。"""
        self.assertLessEqual(CONFIG.control.forward_speed, 0.32)


if __name__ == "__main__":
    unittest.main()
