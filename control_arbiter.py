"""Central control arbiter: decides which single module may command motion.

纯逻辑，不导入 coordinator / config / runtime（只依赖 `models.MotionCommand`）。
设计见 `docs/superpowers/specs/2026-09-18-control-arbiter-design.md`。

判定顺序（`select`）：

1. 丢弃过期请求（`ttl is None` 视为永不过期）；
2. 优先级降序，同优先级按 `order`（注册序号）升序 —— 确定性 tie-break，天然消除抖动；
3. 当前 owner 若仍有有效请求，只有在**挑战者"愿意且够格"**时才让位：
   * `preempt_all=False`（默认）时，只有安全级（`priority >= safety_priority`）可以抢占；
   * `preempt_all=True` 时，任何 `priority > owner.priority + preempt_margin` 都可以；
   * 安全级另外**无视最小持有时间**（红灯必须能立刻停住车）；
4. 没有有效请求 → 返回 `None`，由调用方停车。

关键性质：**同优先级永远不抢当前 owner**，所以"两个 P50 模块 A/B/A/B 抖动"在判定层面
不可能发生；`min_hold_seconds` / `preempt_margin` 是给 `preempt_all=True` 用的进一步迟滞。
"""

from dataclasses import dataclass
from typing import Optional, Sequence

from models import MotionCommand

#: 巡线底座不是注册表中的任务，用这个优先级参与比较。
LINE_OWNER = "line"
LINE_PRIORITY = 10

#: 安全级：可以随时抢占任何模块（红绿灯红灯停靠它）。
SAFETY_PRIORITY = 90


@dataclass(frozen=True)
class ControlRequest:
    """一个模块在这一帧提出的控制请求。"""

    module: str
    priority: int
    command: Optional[MotionCommand]
    ttl: Optional[float]
    timestamp: float
    state: str = ""
    order: int = 0

    def expired(self, now: float) -> bool:
        return self.ttl is not None and (now - self.timestamp) > self.ttl


@dataclass(frozen=True)
class ArbitrationResult:
    """仲裁结果：谁在开车、为什么、这一帧有没有换人。"""

    request: Optional[ControlRequest]
    owner: str
    reason: str
    changed: bool


class ControlArbiter:
    def __init__(
        self,
        settings=None,
        safety_priority: int = SAFETY_PRIORITY,
        preempt_margin: int = 0,
        min_hold_seconds: float = 0.2,
        preempt_all: bool = False,
    ) -> None:
        self.safety_priority = int(safety_priority)
        self.preempt_margin = int(preempt_margin)
        self.min_hold_seconds = float(min_hold_seconds)
        self.preempt_all = bool(preempt_all)

    @staticmethod
    def _ranked(requests: Sequence[ControlRequest]) -> list:
        return sorted(requests, key=lambda item: (-item.priority, item.order))

    def _willing_to_preempt(self, winner: ControlRequest) -> bool:
        return self.preempt_all or winner.priority >= self.safety_priority

    def select(
        self,
        requests: Sequence[ControlRequest],
        now: float,
        current_owner: str = LINE_OWNER,
        current_owner_since: Optional[float] = None,
    ) -> ArbitrationResult:
        valid = [item for item in requests if not item.expired(now)]
        ranked = self._ranked(valid)
        if not ranked:
            return ArbitrationResult(
                None, LINE_OWNER, "no_valid_request", current_owner != LINE_OWNER
            )

        winner = ranked[0]
        if current_owner != LINE_OWNER:
            holder = next((item for item in valid if item.module == current_owner), None)
            if holder is not None:
                if winner.module == current_owner:
                    return ArbitrationResult(
                        holder, current_owner, holder.state or "running", False
                    )
                if not self._willing_to_preempt(winner):
                    return ArbitrationResult(
                        holder, current_owner, "owner_keeps_control", False
                    )
                if winner.priority <= holder.priority + self.preempt_margin:
                    return ArbitrationResult(
                        holder, current_owner, "owner_keeps_control", False
                    )
                is_safety = winner.priority >= self.safety_priority
                held_long_enough = (
                    current_owner_since is None
                    or (now - current_owner_since) >= self.min_hold_seconds
                )
                if not is_safety and not held_long_enough:
                    return ArbitrationResult(holder, current_owner, "min_hold", False)

        reason = winner.state or "claimed"
        return ArbitrationResult(
            winner, winner.module, reason, winner.module != current_owner
        )

    def change_text(
        self,
        previous: str,
        result: ArbitrationResult,
        previous_priority: int = LINE_PRIORITY,
    ) -> Optional[str]:
        """控制权变更的终端日志行（协调器只返回文本，打印由 main.py 负责）。"""
        if not result.changed or result.request is None:
            return None
        return "[ARB] owner changed: %s -> %s  reason=%s  priority=%d > %d" % (
            previous,
            result.owner,
            result.reason,
            result.request.priority,
            previous_priority,
        )
