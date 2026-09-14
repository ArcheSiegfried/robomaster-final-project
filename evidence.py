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
        frame_000123_0006.15s.jpg   每隔一段时间存一张关键帧（运行记录用）
        task_2_000123_0006.15s.jpg  任务得分截图（带检测框 + 居中说明文字）
        log.csv / summary.json / report.md
        report.md      运行结束时自动生成的人类可读记录：
                       起止时间、时长、帧数、得分截图清单（含图上那句话）、
                       任务接管时间线、出现过的限幅/超时/异常。
                       **可以直接贴进报告发给别人**，不用再手工整理。

得分截图（任务证据）——这是 Final 真正算分的东西
----------------------------------------------
Final 原文："The count of the saved images will be the final task score"，
而且每张图必须带"检测框/圆 + 指定的一行说明文字"（例如
`Team 03 detects a marker with ID of 2`）。所以**画框写字属于证据层**，
成员模块只负责在得分那一刻把"哪一帧、框在哪、写什么字"交出来：

    request = task.take_evidence_request()              # 任务交出请求
    saved = recorder.save_task_evidence(request)        # 证据层画 + 存，返回真实结果
    task.acknowledge_evidence(request.request_id, saved) # 回传真实结果

**最后一步不能省**：任务只有在收到 True 之后才会算作完成并归还控制权；
不回执 = 任务一直停在"接管中 + 停车"直到超时。所以调用方必须把
`save_task_evidence()` 的返回值原样回传。谁来做这件事见 `main.py` 的
`service_task_evidence()`。

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


def _safe_label(text: object) -> str:
    """把请求里的标识变成可以安全放进文件名的短标签。"""
    raw = str(text or "task")
    cleaned = "".join(
        character if (character.isalnum() or character in "-_") else "_"
        for character in raw
    )
    return cleaned[:48] or "task"


def render_task_evidence(request):
    """在**完整原始画面副本**上画检测框 + 居中说明文字。

    Final 要求存图里必须有"检测到的目标框 + 一行说明文字"，所以画图属于
    证据层，不属于某个功能模块。这里只读请求对象上的公开字段（鸭子类型），
    因此不需要 import 任何成员模块：

    * ``image``       真实的全帧 BGR 画面（只读，不修改原图）
    * ``detection``   带 ``box=(left, top, right, bottom)`` 的检测结果，可为 None
    * ``annotation``  要显示的说明文字
    * ``text_anchor`` 文字锚点 ``(x, y)``，缺省用画面中心

    返回标注后的新画面；字段缺失时退化成未标注的副本。取不到图才抛异常。
    """
    image = getattr(request, "image", None)
    if image is None:
        raise ValueError("evidence request carries no image")
    shown = image.copy()
    height, width = shown.shape[:2]

    box = getattr(getattr(request, "detection", None), "box", None)
    if box:
        left, top, right, bottom = (int(value) for value in box)
        cv2.rectangle(shown, (left, top), (right, bottom), (0, 255, 255), 2)

    annotation = getattr(request, "annotation", "")
    if annotation:
        anchor = getattr(request, "text_anchor", None) or (width // 2, height // 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 2
        (text_width, text_height), _ = cv2.getTextSize(
            str(annotation), font, scale, thickness
        )
        x = max(0, min(width - text_width, int(anchor[0]) - text_width // 2))
        y = max(text_height, min(height - 1, int(anchor[1])))
        cv2.putText(
            shown, str(annotation), (x, y), font, scale, (0, 255, 255), thickness
        )
    return shown


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
        self.task_snapshots = 0
        self.rows_written = 0
        self.duplicate_frames_skipped = 0

        # 本次运行的记录：得分截图清单 + 任务接管时间线。
        # 结束时写成 report.md，可以直接贴进报告发给别人。
        self.task_evidence: List[dict] = []
        self.events: List[dict] = []
        self.event_limit = 500
        self._last_event_key = None
        self._last_errors: tuple = ()
        self._last_frame_at: Optional[float] = None

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
        self._last_frame_at = now

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

    def save_task_evidence(self, request) -> bool:
        """把任务模块交出来的得分截图请求真正落盘，返回**真实写盘结果**。

        调用方必须把这个返回值原样回传到
        ``task.acknowledge_evidence(request.request_id, saved)``：
        任务只有收到 True 才会 COMPLETED 并把控制权还给巡线；收不到回执
        就会一直停在"接管中 + 停车"直到超时。

        这是本模块**唯一**一处同步写盘。它是任务主动发起的，而且任务在等回执
        期间本来就处于"已接管 + 要求停车"状态（Final 要求先停车再截图），
        所以写一张 JPEG 不会额外增加运动风险。

        绝不抛异常：任何失败都只累加 ``write_failures`` 并返回 False。
        """
        if not self.enabled or self._closed or self.run_directory is None:
            return False

        request_id = getattr(request, "request_id", "")
        try:
            sequence = int(getattr(request, "frame_sequence", 0) or 0)
        except (TypeError, ValueError):
            sequence = 0
        annotation = str(getattr(request, "annotation", "") or "")

        try:
            image = render_task_evidence(request)
        except Exception:
            # 取不到画面。宁可如实报失败，也不要写一张充数的图上去——
            # Final 的分数就是按这些图算的。
            self.write_failures += 1
            return False

        try:
            captured_at = float(getattr(request, "captured_at", 0.0) or 0.0)
            elapsed = 0.0
            if self._started_at is not None and captured_at:
                elapsed = max(0.0, captured_at - self._started_at)
            label = _safe_label(
                getattr(request, "marker_id", None) or request_id or "task"
            )
            name = "task_%s_%06d_%07.2fs.jpg" % (label, sequence, elapsed)
            target = self.run_directory / name
            # 和关键帧一样：先编码到内存再用 Python 落盘。故意不用
            # cv2.imwrite —— 它在 Windows 上走窄字符路径，目录名带中文
            # （例如 E:\...\机器人期末\）时会**静默失败**，只返回 False。
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

        self.task_snapshots += 1
        self.task_evidence.append(
            {
                "wall_clock": datetime.now().isoformat(timespec="seconds"),
                "elapsed_s": round(elapsed, 2),
                "label": label,
                "annotation": annotation,
                "file": name,
            }
        )
        self.pending.append(
            {
                "sequence": sequence,
                "captured_at": round(captured_at, 4),
                "loop_time": round(captured_at, 4),
                "elapsed_s": round(elapsed, 3),
                "width": int(image.shape[1]),
                "height": int(image.shape[0]),
                "mean_v": "",
                # 把说明文字写进日志，事后能把图和"哪一分"对上。
                "note": "task_evidence %s %s" % (request_id, annotation),
            }
        )
        # 立刻把这一行刷进 CSV：得分截图是分数本身，不能等节流。
        # 传 None 表示"只刷行，不动节流时钟"。
        self._flush(None)
        return True

    def record_decision(self, decision, now: float) -> None:
        """记录主循环这一帧的结果，供运行报告使用。

        只记**状态发生变化**的帧（以及任何带错误的帧），所以逐帧调用几乎不花钱，
        报告里也不会被"每帧一行"刷屏。这是一次运行最值得留档的东西：
        哪个模块什么时候接管、怎么结束、有没有被限幅/超时/异常。

        绝不抛异常：记录功能不能影响控制循环。
        """
        if self._closed or self.run_directory is None:
            return
        try:
            update = getattr(decision, "task_update", None)
            status = getattr(update, "status", None)
            key = (
                str(getattr(decision, "state", "")),
                str(getattr(decision, "owner", "")),
                str(getattr(decision, "task_name", "") or ""),
                str(getattr(status, "name", "") or ""),
            )
            errors = tuple(str(item) for item in (getattr(decision, "errors", ()) or ()))
            message = str(getattr(decision, "message", "") or "")
        except Exception:
            return

        if key == self._last_event_key and errors == self._last_errors:
            return
        self._last_event_key = key
        self._last_errors = errors
        if len(self.events) >= self.event_limit:
            return

        elapsed = None
        if self._started_at is not None:
            elapsed = max(0.0, float(now) - self._started_at)
        self.events.append(
            {
                "wall_clock": datetime.now().isoformat(timespec="seconds"),
                "elapsed_s": None if elapsed is None else round(elapsed, 2),
                "state": key[0],
                "owner": key[1],
                "task": key[2] or None,
                "task_status": key[3] or None,
                "message": message,
                "errors": errors,
            }
        )

    def _write_report(self) -> None:
        """把本次运行写成一份可以直接贴进报告的 report.md。绝不抛异常。"""
        if self.run_directory is None:
            return

        def clock(seconds):
            if seconds is None:
                return "-"
            return "%02d:%04.1f" % (int(seconds) // 60, seconds % 60)

        frames = self.rows_written + len(self.pending)
        duration = None
        if self._started_at is not None and self._last_frame_at is not None:
            duration = max(0.0, self._last_frame_at - self._started_at)

        lines = [
            "# 运行记录 %s" % self.run_directory.name,
            "",
            "| 项 | 值 |",
            "|---|---|",
            "| 开始（墙钟） | %s |" % (self._started_wall_clock or "-"),
            "| 结束（墙钟） | %s |" % datetime.now().isoformat(timespec="seconds"),
            "| 运行时长 | %s |" % clock(duration),
            "| 记录帧数 | %d |" % frames,
            "| **得分截图** | **%d 张** |" % self.task_snapshots,
            "| 运行记录截图 | %d 张 |" % self.snapshots,
            "| 写入失败 | %d |" % self.write_failures,
            "| 目录 | `%s` |" % self.run_directory.name,
            "",
            "## 得分截图（Final 按这些图算分）",
            "",
        ]
        if self.task_evidence:
            lines += [
                "| 墙钟 | 相对 | 标识 | 图上文字 | 文件 |",
                "|---|---|---|---|---|",
            ]
            for item in self.task_evidence:
                lines.append(
                    "| %s | %s | %s | %s | `%s` |"
                    % (
                        item["wall_clock"],
                        clock(item["elapsed_s"]),
                        item["label"],
                        item["annotation"],
                        item["file"],
                    )
                )
        else:
            lines.append("本次运行没有产生得分截图。")
        lines += ["", "## 任务接管记录", ""]
        if self.events:
            lines += [
                "| 墙钟 | 相对 | 状态 | 接管者 | 任务 | 结果 | 说明 |",
                "|---|---|---|---|---|---|---|",
            ]
            for event in self.events:
                lines.append(
                    "| %s | %s | %s | %s | %s | %s | %s |"
                    % (
                        event["wall_clock"],
                        clock(event["elapsed_s"]),
                        event["state"],
                        event["owner"],
                        event["task"] or "-",
                        event["task_status"] or "-",
                        (event["message"] or "-")[:60],
                    )
                )
            problems = [e for e in self.events if e["errors"]]
            if problems:
                lines += ["", "### 这一轮出现过的问题", ""]
                for event in problems:
                    for error in event["errors"]:
                        lines.append("- `%s` %s" % (clock(event["elapsed_s"]), error))
        else:
            lines.append("本次运行没有任何模块接管运动（全程基础巡线）。")
        lines += [
            "",
            "---",
            "",
            "本文件由 `evidence.py` 在运行结束时自动生成。`captures/` 不在 git 里，",
            "要交作业请把整个 run 目录（图和这份记录）一起复制走。",
            "",
        ]

        target = self.run_directory / "report.md"
        temporary = target.with_suffix(".md.tmp")
        try:
            temporary.write_text("\n".join(lines), encoding="utf-8")
            temporary.replace(target)
        except Exception:
            self.write_failures += 1
            try:
                temporary.unlink()
            except OSError:
                pass

    def close(self) -> None:
        """收尾：把剩下的行写掉、关文件、写一份 summary 与运行记录。绝不抛异常。"""
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
        # 放在 summary 之后：报告要把 write_failures 的最终值写进去。
        self._write_report()

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
            "task_snapshots": self.task_snapshots,
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
