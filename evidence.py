"""截图标注与任务证据（基础设施，不占功能模块名额）。

归属：由**整合负责人**维护，不是某个组员的功能模块名额。
最终交付材料需要截图与运行记录（见 `DELIVERABLES.md`），所以这个能力必须有人负责，
但它不属于"功能模块"，因此**不单独占一个人**。

这是**观察型模块**：骨架每一帧都会调用 `observe()`（包括别的任务正在接管
的时候），但它**永远不能接管运动**，没有任何运动权限。

状态：**空实现**。骨架已经把这个文件登记进 task_registry，你只要替换下面的
`EvidenceRecorder.observe()`，就自动接入主流程，不需要改 main.py。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 不返回运动请求，不接触底盘的唯一运动出口。
  * observe() 必须立刻返回：**不许在控制循环里同步写盘**。
  * 不许把大批原始录像、运行日志、个人绝对路径或凭据提交进仓库。

必须实现的接口（签名已冻结，不要改）：
    class EvidenceRecorder:
        name = "evidence"
        def observe(self, frame: FramePacket, now: float) -> None

建议结构（骨架不限制你怎么写，只限制"必须立刻返回"）：
  * observe() 只把"要记录什么"塞进一个内存队列，并做去重；
  * 真正的写盘由节流逻辑触发（例如距上次 flush 超过 0.5 秒，或任务完成时）；
  * 写盘失败要能报告，但**绝不能**因此影响安全停车；
  * 输出目录用被 .gitignore 排除的 captures/ 之类，不要写进仓库根目录。

参考蓝本（本批最值得搬的一段，已被 60+ 次真实运行验证不阻塞）：
  * blue_line_following/race_telemetry.py:109-117
      —— 建目录 / 开文件 / DictWriter，文件名带时间戳
  * blue_line_following/race_telemetry.py:296-299
      —— 每帧只 writerow，flush 被节流到 0.5 秒一次
  * blue_line_following/trajectory_io.py:191-205
      —— 原子写（先写 .tmp 再替换），并且 allow_nan=False
  注意竞速工程的 .gitignore 只忽略了 logs/，track_map.json 和 reports/*.json
  都没忽略，别重复这个错。

先看 tests/test_evidence.py 的用例区再动手。
单独自测：python scripts/check_module.py evidence
"""

from typing import List, Optional

from models import FramePacket

KIND = "evidence"


class EvidenceRecorder:
    """TODO(WP4)：实现内存队列 + 节流落盘，观察型模块永不接管运动。"""

    name = "evidence"

    def __init__(self, directory: Optional[str] = None) -> None:
        self.directory = directory
        self.pending: List[tuple] = []
        self.last_flush = None
        self.write_failures = 0

    def observe(self, frame: FramePacket, now: float) -> None:
        # 空实现：什么都不做，因此也绝不会影响控制循环的时序。
        # TODO(WP4)：只做"入队 + 去重 + 判断该不该 flush"，不要在这里同步写盘。
        #   步骤示例：
        #     1) 判断这一帧有没有值得记录的事件（别每帧都存图）；
        #     2) 只把 (frame.sequence, now, 事件描述) 追加到 self.pending；
        #     3) 距 self.last_flush 超过节流间隔时，写一次盘并更新 last_flush；
        #     4) 写盘异常只累加 self.write_failures，绝不向外抛。
        return None
