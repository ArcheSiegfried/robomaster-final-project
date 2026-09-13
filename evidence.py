"""截图标注与任务证据（基础设施，不占功能模块名额）。

归属：由**整合负责人**维护，不是某个组员的功能模块名额。
最终交付材料需要截图与运行记录（见 `DELIVERABLES.md`），所以这个能力必须有人负责，
但它不属于"功能模块"，因此**不单独占一个人**。

状态：**已实现**。

这是**观察型模块**：骨架每一帧都会调用 `observe()`（包括别的任务正在接管
的时候），但它**永远不能接管运动**，没有任何运动权限。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 不返回运动请求，不接触底盘的唯一运动出口。
  * observe() 必须立刻返回：**不在控制循环里同步写盘**。
  * 不把大批原始录像、运行日志、个人绝对路径或凭据提交进仓库。

必须实现的接口（签名已冻结，不要改）：
    class EvidenceRecorder:
        name = "evidence"
        def observe(self, frame: FramePacket, now: float) -> None

产出什么
--------
每次运行在 `captures/` 下建一个带时间戳的目录（`captures/` 已被 `.gitignore` 排除）：

    captures/run_20260911_153045/
        log.csv        每帧一行：帧号、采集时刻、循环时刻、已运行秒数、画面尺寸、亮度
        frame_000123_0006.15s.jpg   每隔一段时间存一张关键帧
        summary.json   结束时写入：总帧数、截图数、写入失败数、运行时长

设计要点
--------
* **默认关闭**：`EvidenceRecorder()` 不带 directory 时什么都不做。
  真实运行由 `task_registry.build_observers()` 显式打开；测试里构造无参实例不会写任何文件。
* **绝不阻塞**：`observe()` 只做"入队 + 去重 + 判断该不该落盘"，真正的写盘被节流到
  `flush_interval` 秒一次（参考竞速工程 race_telemetry.py 的做法，已被 60+ 次真实运行验证）。
* **绝不抛异常**：任何一步写盘失败只累加 `write_failures`，不影响控制循环和安全停车。
  连目录建不出来时，模块会把自己关掉，而不是让程序起不来。
* **原子写**：summary 先写 `.tmp` 再替换（参考竞速工程 trajectory_io.py 的做法）。

单独自测：python scripts/check_module.py evidence
"""

import csv
import json
import os
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2

from models import FramePacket

KIND = "evidence"

#: 真实运行时默认的输出目录（相对仓库根目录；已在 .gitignore 里排除）。
DEFAULT_CAPTURE_DIRECTORY = "captures"

#: 每帧写一行 CSV，但真正 flush 的频率由它控制（秒）。
DEFAULT_FLUSH_INTERVAL = 0.50

#: 最多每隔这么多秒存一张关键帧（秒）。不是每帧都存图。
DEFAULT_SNAPSHOT_INTERVAL = 2.0

#: 待写队列超过这个长度就立刻 flush，防内存涨。
DEFAULT_QUEUE_LIMIT = 200

#: JPEG 质量。
SNAPSHOT_QUALITY = 85

FIELDNAMES = (
    "sequence",
    "captured_at",
    "loop_time",
    "elapsed_s",
    "width",
    "height",
    "mean_v",
    "note",
)


def _resolve_directory(directory: str) -> Path:
    """相对路径按仓库根目录解析，绝对路径原样使用。

    仓库根目录 = 本文件所在目录（evidence.py 在仓库根）。
    """
    path = Path(directory)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    return path


class EvidenceRecorder:
    """把关键帧和运行记录写到磁盘；永不接管运动。

    用法（真实运行）：``EvidenceRecorder(directory="captures")``
    用法（测试）：``EvidenceRecorder(directory=<临时目录>)``
    不带目录时是**安全的空操作**。
    """

    name = "evidence"

    def __init__(
        self,
        directory: Optional[str] = None,
        flush_interval: float = DEFAULT_FLUSH_INTERVAL,
        snapshot_interval: float = DEFAULT_SNAPSHOT_INTERVAL,
        queue_limit: int = DEFAULT_QUEUE_LIMIT,
    ) -> None:
        self.directory = directory
        self.flush_interval = float(flush_interval)
        self.snapshot_interval = float(snapshot_interval)
        self.queue_limit = max(1, int(queue_limit))

        self.pending: List[dict] = []
        self.last_flush: Optional[float] = None
        self.last_snapshot: Optional[float] = None
        self.write_failures = 0
        self.snapshots = 0
        self.rows_written = 0
        self.duplicate_frames_skipped = 0

        self.run_directory: Optional[Path] = None
        self.log_path: Optional[Path] = None
        self._csv_file = None
        self._writer = None
        self._seen_sequences = set()
        self._started_at: Optional[float] = None
        self._started_wall_clock: Optional[str] = None
        self._closed = False

        if directory is not None:
            # 建目录 / 开文件只做一次，而且失败时只是把自己关掉——
            # 一个记录功能不能把整个程序拖垮。
            try:
                self._open()
            except Exception:
                self.write_failures += 1
                self.directory = None

    # ------------------------------------------------------------------
    # 对外
    # ------------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self.directory is not None and self._writer is not None

    def observe(self, frame: FramePacket, now: float) -> None:
        """每帧调用一次，必须立刻返回。只入队 + 节流，不同步做大 I/O。"""
        if not self.enabled or self._closed:
            return

        sequence = getattr(frame, "sequence", None)
        if sequence is None:
            return
        if sequence in self._seen_sequences:
            self.duplicate_frames_skipped += 1
            return
        self._seen_sequences.add(sequence)

        if self._started_at is None:
            self._started_at = now
        elapsed = max(0.0, now - self._started_at)

        image = getattr(frame, "image", None)
        height, width = (0, 0)
        mean_v = ""
        if image is not None and getattr(image, "ndim", 0) == 3:
            height, width = image.shape[:2]
            try:
                mean_v = round(float(image[::8, ::8].mean()), 2)
            except Exception:
                mean_v = ""

        self.pending.append(
            {
                "sequence": sequence,
                "captured_at": round(float(getattr(frame, "captured_at", now)), 4),
                "loop_time": round(float(now), 4),
                "elapsed_s": round(elapsed, 3),
                "width": width,
                "height": height,
                "mean_v": mean_v,
                "note": "",
            }
        )

        due_snapshot = (
            self.last_snapshot is None
            or now - self.last_snapshot >= self.snapshot_interval
        )
        if due_snapshot and image is not None:
            if self._write_snapshot(image, sequence, elapsed):
                self.last_snapshot = now

        due_flush = (
            self.last_flush is None
            or now - self.last_flush >= self.flush_interval
            or len(self.pending) >= self.queue_limit
        )
        if due_flush:
            self._flush(now)

    def close(self) -> None:
        """收尾：把剩下的行写掉、关文件、写一份 summary。绝不抛异常。"""
        if self._closed:
            return
        self._closed = True
        if self._writer is not None:
            self._flush(None)
        try:
            if self._csv_file is not None:
                self._csv_file.close()
        except Exception:
            self.write_failures += 1
        self._csv_file = None
        self._writer = None
        self._write_summary()

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _open(self) -> None:
        base = _resolve_directory(self.directory)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_directory = base / ("run_%s" % stamp)
        self.run_directory.mkdir(parents=True, exist_ok=True)
        self.log_path = self.run_directory / "log.csv"
        # utf-8-sig 让 Excel 直接打开不乱码；newline="" 是 csv 模块的要求。
        self._csv_file = self.log_path.open("w", encoding="utf-8-sig", newline="")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._started_wall_clock = datetime.now().isoformat(timespec="seconds")

    def _flush(self, now: Optional[float]) -> None:
        if self._writer is None or not self.pending:
            if now is not None:
                self.last_flush = now
            return
        rows, self.pending = self.pending, []
        try:
            for row in rows:
                self._writer.writerow(row)
            self._csv_file.flush()
            self.rows_written += len(rows)
        except Exception:
            # 记录失败绝不能影响车的安全；把没写成的行丢掉，只计数。
            self.write_failures += 1
        if now is not None:
            self.last_flush = now

    def _write_snapshot(self, image, sequence: int, elapsed: float) -> bool:
        if self.run_directory is None:
            return False
        name = "frame_%06d_%07.2fs.jpg" % (int(sequence), elapsed)
        target = self.run_directory / name
        try:
            # 故意不用 OpenCV 自带的写文件接口：它在 Windows 上走窄字符路径，
            # 目录名里有中文（例如 E:\...\机器人期末\）时会**静默失败**，
            # 函数只返回 False，不抛异常，很难查。
            # 这里先编码到内存，再用 Python 的 open() 落盘，路径完全交给 Python。
            ok, buffer = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), SNAPSHOT_QUALITY]
            )
            if not ok:
                self.write_failures += 1
                return False
            target.write_bytes(buffer.tobytes())
        except Exception:
            self.write_failures += 1
            return False
        self.snapshots += 1
        return True

    def _write_summary(self) -> None:
        if self.run_directory is None:
            return
        payload = {
            "started_wall_clock": self._started_wall_clock,
            "finished_wall_clock": datetime.now().isoformat(timespec="seconds"),
            "frames_recorded": self.rows_written + len(self.pending),
            "rows_written": self.rows_written,
            "snapshots": self.snapshots,
            "duplicate_frames_skipped": self.duplicate_frames_skipped,
            "write_failures": self.write_failures,
            "flush_interval_s": self.flush_interval,
            "snapshot_interval_s": self.snapshot_interval,
            "log": os.path.basename(str(self.log_path)) if self.log_path else "",
        }
        target = self.run_directory / "summary.json"
        temporary = target.with_suffix(".json.tmp")
        try:
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
                encoding="utf-8",
            )
            temporary.replace(target)
        except Exception:
            self.write_failures += 1
            try:
                temporary.unlink()
            except OSError:
                pass
