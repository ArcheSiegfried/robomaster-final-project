"""集成侧回归测试：直角候选的接受必须与"车已经转过多少"无关。

**这条测试的由来（一次争议的定案）**：PR #43 的审查曾把下面这行判成 Critical：

    world_tangent = self._heading_offset + endpoint.tangent_deg

理由是"相机固连车体、gimbal_yaw 恒为 0，图像切向本来就是车体系角度，加累计转角是
多算两倍"，并建议改成 `tangent − heading_offset`。集成侧复核后**不接受这条**：

- 相机固连车体 ⇒ 车体转过 ψ 后，**同一个世界里的特征在图像里的切向会跟着变**
  （世界角 90° 的直角新线，在车体系里表现为 `90° − ψ`）；
- `_old_tangent_world` 是丢线那一刻（ψ≈0）记下的角度，等于世界角；
- 因此 `世界角 = 图像切向 + ψ = (θ_world − ψ) + ψ = θ_world` ✓ **作者原来的"加"是对的**；
- 改成"减"会得到 `θ_world − 2ψ`，扫到 ±30°/±45°/±60° 时真正的直角候选会被拒绝。

实测（`tools/probe_route_frame_formula.py`，两种写法各跑一遍）：
    作者原式（+）：被拒绝的扫角 = 无（全部接受）
    建议改成（−）：被拒绝的扫角 = [30, -30, 45, -45, 60, -60]

所以这条测试断言的是**物理不变量**：让图像切向随车体旋转一起变，接受结论必须不变。
只构造假候选与假端点，不连车、不连相机。
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import route_detector  # noqa: E402
from route import RouteTask  # noqa: E402

#: 旧线在世界坐标里的切向（0° = 与画面水平方向一致）。
OLD_WORLD_TANGENT = 0.0
#: 真正的直角新线：世界坐标 90°。
RIGHT_ANGLE_WORLD = 90.0
#: 扇扫能开到的范围（模块自己的软/硬限位是 ±95°）。
SWEEP_OFFSETS = (0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 90.0, -90.0)


def candidate_seen_at(sweep_deg: float, world_tangent: float = RIGHT_ANGLE_WORLD):
    """车体转过 `sweep_deg` 之后，那条世界角 90° 的新线在**图像里**的样子。"""
    image_tangent = world_tangent - sweep_deg
    endpoint = route_detector.RouteEndpoint(
        point=(300, 200), tangent_deg=image_tangent, internal=True, branch_length=40.0
    )
    return route_detector.RouteCandidate(
        detection=None, angle_deg=image_tangent, near=False, bottom_ratio=0.5,
        upper_point=(300, 180), lower_point=(300, 220), elongation=3.0,
        score=1.0, endpoints=(endpoint,), entry_endpoint=endpoint,
    )


class EndpointFrameTests(unittest.TestCase):
    def _task(self, sweep_deg):
        task = RouteTask()
        task._old_tangent_world = OLD_WORLD_TANGENT
        task._heading_offset = sweep_deg
        return task

    def test_true_right_angle_is_accepted_at_every_sweep_angle(self):
        for sweep in SWEEP_OFFSETS:
            task = self._task(sweep)
            self.assertTrue(
                task._candidate_geometry_ok(candidate_seen_at(sweep)),
                "车体转过 %.0f° 后，画面里那条真正的直角新线被拒绝了"
                "（说明世界角换算把累计转角算错了方向）" % sweep,
            )

    def test_left_and_right_sweeps_are_symmetric(self):
        for world in (RIGHT_ANGLE_WORLD, -RIGHT_ANGLE_WORLD):
            for sweep in (30.0, -30.0, 60.0, -60.0):
                task = self._task(sweep)
                self.assertTrue(
                    task._candidate_geometry_ok(candidate_seen_at(sweep, world)),
                    "世界角 %.0f° 的新线在车体转过 %.0f° 后被拒绝了" % (world, sweep),
                )

    def test_a_straight_continuation_is_still_rejected(self):
        """别把门开穿：与旧线同向（世界角 0°）的候选仍必须被拒。"""
        for sweep in (0.0, 30.0, -30.0):
            task = self._task(sweep)
            self.assertFalse(
                task._candidate_geometry_ok(candidate_seen_at(sweep, OLD_WORLD_TANGENT)),
                "与旧线同向的候选不该被当成直角断口",
            )


if __name__ == "__main__":
    unittest.main()
