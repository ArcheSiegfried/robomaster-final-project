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
        console.log    终端上打过的那些状态行（`[  12.3s] …`）的副本，带时间戳。
                       由 `main.py` 把 ConsoleStatus 的输出同时引到这里（见
                       `main._TeeStream`）。关掉窗口或程序崩了之后还能查发生了什么。
        frame_000123_0006.15s.jpg   每隔一段时间存一张关键帧（运行记录/调试用）
        scoring/        **得分截图专用目录**：老师按这个目录里的张数算分，
                       交作业只交它，调试关键帧不会混进去。
        scoring/task_2_000123_0006.15s.jpg  得分截图（带检测框 + 居中说明文字）
        log.csv / summary.json / report.md
        report.md      运行结束时自动生成的人类可读记录：
                       起止时间、时长、帧数、得分截图清单（含图上那句话）、
                       任务接管时间线、出现过的限幅/超时/异常。
                       **可以直接贴进报告发给别人**，不用再手工整理。

得分截图（任务证据）——这是 Final 真正算分的东西
----------------------------------------------
Final 原文："The count of the saved images will be the final task score"，
而且每张图必须带"检测框/圆 + 指定的一行说明文字"（例如
`Team 10 detects a marker with ID of 2`）。所以**画框写字属于证据层**，
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
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2

from models import FramePacket

KIND = "evidence"

#: 我们队的编号。老师给的样例文字是 `Team 10 ...`，那是样例队号；我们按队号写 03。
TEAM_NUMBER = "10"

#: 老师后来明确的"证据照片"文案模板（2026-09-17 新增，见 `EVIDENCE_PHOTOS.md`）。
#: **文案属于证据层**：5 个模块各写一套必然口径不一，分数就丢在这上面。
#: `{side}` = 目标（灯/堵车）在哪条路；`{chosen}` = 我们选哪条路/绕哪边。
ANNOTATION_TEMPLATES = {
    "traffic_light:red": "Team {team} detects a red light and the robot stops",
    "traffic_light:green": "Team {team} detects a green light and continues",
    "green_junction:green": "Team {team} detects a green light » {side} way and {chosen}",
    "green_junction:red": "Team {team} detects a red light » {side} way and {chosen}",
    "free_junction": "Team {team} detects traffic jam » {side} way and {chosen}",
    "obstacle": "Team {team} detects obstacle » the {side} side",
    "route": "Team {team} finds correct to follow",
}

#: 真实运行时默认的输出目录（相对仓库根目录；已在 .gitignore 里排除）。
DEFAULT_CAPTURE_DIRECTORY = "captures"

#: 每帧写一行 CSV，但真正 flush 的频率由它控制（秒）。
DEFAULT_FLUSH_INTERVAL = 0.50

#: 最多每隔这么多秒存一张关键帧（秒）。不是每帧都存图。
DEFAULT_SNAPSHOT_INTERVAL = 2.0

#: 关键帧总数上限。关键帧只是**调试/运行记录**，不是算分的图，所以封顶；
#: 得分截图（`scoring/`）**不封顶** —— 老师按那个目录里的张数算分，封顶就是丢分。
#: 一次运行 20 张够复盘了，而且交付时"一次限发 20 张"也放得下。
DEFAULT_MAX_KEYFRAMES = 20

#: 得分截图专用子目录名。把算分的图和一个调试用的关键帧分开：
#: 交作业只交这个目录，不会把调试帧混进去。
SCORING_DIRECTORY = "scoring"

#: 终端状态行副本的文件名（`main.py` 把 ConsoleStatus 的输出 tee 到这里）。
CONSOLE_LOG_FILENAME = "console.log"

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


def annotation_for(kind: str, team: str = TEAM_NUMBER, **fields) -> str:
    """按 `kind` 取模板并填参。模板缺失时**大声失败**（宁可当场红，也不要存一张没字的图）。"""
    template = ANNOTATION_TEMPLATES.get(kind)
    if template is None:
        raise KeyError(
            "unknown evidence kind %r; known: %s"
            % (kind, ", ".join(sorted(ANNOTATION_TEMPLATES)))
        )
    return template.format(team=team, **fields)


#: `cv2.putText` 用的是 Hershey 字体，**只支持 ASCII**：非 ASCII 字符会被逐字节
#: 映射成 `?`，于是图上的 `»` 变成 `??`（字符串本身是对的，只有像素是错的）。
#: 图是要交作业的东西，所以绘制时把这类字符换成等价的 ASCII 写法；
#: **记录/日志里仍然保留原字符**（`report.md` 里就是老师的 `»`）。
DRAWING_FALLBACK = {
    ord("»"): ">>",
    ord("«"): "<<",
    ord("→"): "->",
    ord("←"): "<-",
    ord("×"): "x",
    ord("—"): "-",
    ord("“"): '"',
    ord("”"): '"',
}


def drawing_text(text: object) -> str:
    """把要**画到像素上**的文字转成 ASCII 安全写法（其余字符原样保留）。"""
    return str(text).translate(DRAWING_FALLBACK)


@dataclass(frozen=True)
class EvidencePhoto:
    """一条"得分照片"请求。**成员模块在得分那一刻构造它，证据层负责画与存。**

    这是统一层：模块只管"哪一帧、圈哪个目标、写什么字"，画框/画圆/写字/去重/落盘
    全在证据层，所以 5 个模块的照片口径一定一致。

    字段（与 `render_task_evidence` / `save_task_evidence` 的鸭子类型约定一致）：
      * ``image``        全帧 BGR 画面
      * ``detection``    带 ``box`` 的检测结果（矩形用它；圆也可以用它内切）
      * ``shape``        ``"rect"``（框，例如障碍/堵车/标识）或 ``"circle"``（圈，例如红绿灯）
      * ``circle``       显式 ``(cx, cy, r)``；不给就用 ``detection.box`` 的内切圆
      * ``annotation``   图上那行字（默认由 `annotation_for` 生成）
      * ``label``        文件名/记录里的短标签
      * ``event_key``    **同一事件只存一张**（老师按张数算分，重复存不算）；
                         默认取 ``label``
    """

    kind: str
    image: object
    detection: object = None
    annotation: str = ""
    shape: str = "rect"
    circle: Optional[Tuple[float, float, float]] = None
    text_anchor: Optional[Tuple[int, int]] = None
    frame_sequence: int = 0
    captured_at: float = 0.0
    label: str = ""
    attempt: int = 1
    event_key: str = ""

    @property
    def request_id(self) -> str:
        return "%s:frame:%d:attempt:%d" % (
            self.label or self.kind, int(self.frame_sequence), int(self.attempt)
        )

    @property
    def marker_id(self) -> str:
        return self.label or self.kind


def make_evidence_photo(
    kind: str,
    frame,
    detection=None,
    shape: str = "rect",
    side: Optional[str] = None,
    chosen: Optional[str] = None,
    label: Optional[str] = None,
    team: str = TEAM_NUMBER,
    circle: Optional[Tuple[float, float, float]] = None,
    event_key: Optional[str] = None,
    attempt: int = 1,
):
    """模块侧的入口：一行构造出统一格式的得分照片请求。

    用法（成员模块只需在"得分那一刻"调一次，然后照既有回执链交出去）::

        self._queued_evidence = make_evidence_photo(
            "obstacle", frame, detection=detection, side=self.last_side)

    文案由模板统一生成，模块不需要（也不应该）自己拼字符串。
    """
    fields = {}
    if side is not None:
        fields["side"] = side
    if chosen is not None:
        fields["chosen"] = chosen
    annotation = annotation_for(kind, team=team, **fields)
    short = label or kind.replace(":", "_")
    raw_image = getattr(frame, "image", None)
    # 拷贝一份：队列里的请求可能过几帧才被证据层处理，而相机缓冲会被复用。
    image = raw_image.copy() if hasattr(raw_image, "copy") else raw_image
    return EvidencePhoto(
        kind=kind,
        image=image,
        detection=detection,
        annotation=annotation,
        shape=shape,
        circle=circle,
        frame_sequence=int(getattr(frame, "sequence", 0) or 0),
        captured_at=float(getattr(frame, "captured_at", 0.0) or 0.0),
        label=short,
        attempt=attempt,
        event_key=event_key if event_key is not None else short,
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
    shape = str(getattr(request, "shape", "rect") or "rect").lower()
    circle = getattr(request, "circle", None)
    # 文字默认画在**目标框/圆的下方居中**（老师要求"在圈下方写明"）。
    below = None
    if shape == "circle":
        center = None
        if circle is not None:
            try:
                center = (int(circle[0]), int(circle[1]), max(1, int(circle[2])))
            except (TypeError, ValueError, IndexError):
                center = None
        elif box:
            left, top, right, bottom = (int(value) for value in box)
            center = (
                (left + right) // 2,
                (top + bottom) // 2,
                max(4, max(right - left, bottom - top) // 2),
            )
        if center is not None:
            cv2.circle(shown, (center[0], center[1]), center[2], (0, 255, 255), 2)
            below = (center[0], center[1] + center[2] + 24)
    elif box:
        left, top, right, bottom = (int(value) for value in box)
        cv2.rectangle(shown, (left, top), (right, bottom), (0, 255, 255), 2)
        below = ((left + right) // 2, bottom + 24)

    annotation = getattr(request, "annotation", "")
    if annotation:
        anchor = getattr(request, "text_anchor", None)
        if below is not None:
            anchor = below
        elif anchor is None:
            anchor = (width // 2, height // 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 2
        # 画上去的是 ASCII 安全写法（`»` -> `>>`）；`report.md` 里仍记原始字符串。
        text = drawing_text(annotation)
        (text_width, text_height), _ = cv2.getTextSize(
            text, font, scale, thickness
        )
        x = max(0, min(width - text_width, int(anchor[0]) - text_width // 2))
        y = max(text_height, min(height - 4, int(anchor[1])))
        cv2.putText(
            shown, text, (x, y), font, scale, (0, 255, 255), thickness
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
        max_keyframes: int = DEFAULT_MAX_KEYFRAMES,
    ) -> None:
        self.directory = directory
        self.flush_interval = float(flush_interval)
        self.snapshot_interval = float(snapshot_interval)
        self.queue_limit = max(1, int(queue_limit))
        #: 关键帧上限；<=0 表示不限制。得分截图不受它影响。
        self.max_keyframes = int(max_keyframes)

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
        #: 已经存过照片的"事件键"（同一事件只存一张，见 save_task_evidence）。
        self.saved_events: set = set()
        self.events: List[dict] = []
        # 接线层的自检结果（例如 SDK marker 订阅状态）。运行结束时作为独立
        # 小节写进 report.md，用来回答"某个模块为什么一次都没动"。
        self.diagnostics: List[tuple] = []
        self.event_limit = 500
        self._last_event_key = None
        self._last_errors: tuple = ()
        self._last_frame_at: Optional[float] = None

        self.run_directory: Optional[Path] = None
        self.log_path: Optional[Path] = None
        #: 终端状态行副本（见 `write_console_log` / `main._TeeStream`）。
        self.console_log_path: Optional[Path] = None
        self._console_log = None
        #: 得分截图目录（`run_directory/scoring`）。
        self.scoring_directory: Optional[Path] = None
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
        if due_snapshot and image is not None and self._keyframes_remaining():
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
        # 同一事件只存一张：老师按**保存的张数**算分，同一件事存两张不会多加一分，
        # 只会让报告里的清单变脏。所以这里提前返回 True（= "这件事已经有照片了"），
        # 让任务照常 COMPLETED，而不是误以为写盘失败。
        event_key = str(getattr(request, "event_key", "") or "")
        if event_key and event_key in self.saved_events:
            return True
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
            # 得分截图单独进 scoring/：交作业只交这个目录。
            # 目录建不出来时退回运行目录根，绝不让"存不上分"这种事发生。
            parent = self.scoring_directory
            if parent is None:
                parent = self.run_directory
            try:
                parent.mkdir(parents=True, exist_ok=True)
            except Exception:
                parent = self.run_directory
            target = parent / name
            relative = "%s/%s" % (parent.name, name)
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
        if event_key:
            self.saved_events.add(event_key)
        self.task_evidence.append(
            {
                "wall_clock": datetime.now().isoformat(timespec="seconds"),
                "elapsed_s": round(elapsed, 2),
                "label": label,
                "annotation": annotation,
                "file": relative,
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
            # 协调器的消息只写"task failed"/"task completed"，模块自己给的原因在
            # task_update.message 里。不合并的话，报告里只剩"失败了"而不知道
            # 为什么（实车测 obstacle 那次就是这么丢掉原因的）。
            update_message = str(getattr(update, "message", "") or "")
            if update_message and update_message not in message:
                message = ("%s / %s" % (message, update_message)) if message else update_message
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

    def record_diagnostics(self, title: str, values: dict) -> None:
        """记一段本次运行的诊断信息，结束时作为独立小节写进 report.md。

        给"某个模块为什么没动作"这类问题留证据：时间线只能看出"没接管"，
        看不出原因。调用方通常传接线层的自检结果（例如数字标识的 SDK marker
        订阅状态、回调频率、坐标模式）。

        和本类其它方法一样：**绝不抛异常**，也绝不影响开车。
        """
        if self._closed or self.run_directory is None:
            return
        try:
            self.diagnostics.append((str(title), dict(values)))
        except Exception:
            pass

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
        for title, values in self.diagnostics:
            lines += ["", "## %s" % title, ""]
            if values:
                lines += ["| 项目 | 值 |", "|---|---|"]
                for key, value in values.items():
                    lines.append("| %s | %s |" % (key, value))
            else:
                lines.append("（没有可用的诊断数据。）")
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
        # 终端日志先关：report.md 之前不需要它，但别留着句柄。
        self.close_console_log()
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
        #: 得分截图单独一个目录：交作业只交它，调试关键帧不会混进去。
        self.scoring_directory = self.run_directory / SCORING_DIRECTORY
        self.scoring_directory.mkdir(parents=True, exist_ok=True)
        self.log_path = self.run_directory / "log.csv"
        # 终端状态行的副本。只建**文件**，内容由 main 的 tee 写进来。
        self.console_log_path = self.run_directory / CONSOLE_LOG_FILENAME
        try:
            self._console_log = self.console_log_path.open(
                "w", encoding="utf-8", newline=""
            )
        except Exception:
            self._console_log = None
            self.console_log_path = None
            self.write_failures += 1
        # utf-8-sig 让 Excel 直接打开不乱码；newline="" 是 csv 模块的要求。
        self._csv_file = self.log_path.open("w", encoding="utf-8-sig", newline="")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._started_wall_clock = datetime.now().isoformat(timespec="seconds")

    def write_console_log(self, text: str) -> None:
        """把一行终端输出追加到 `console.log`。

        由 `main._TeeStream` 调用：终端上打过的状态行，磁盘上留一份。
        绝不抛异常、绝不阻塞 —— 一个日志不能影响车的控制循环。
        """
        if self._console_log is None:
            return
        try:
            self._console_log.write(text)
            self._console_log.flush()
        except Exception:
            # 写日志失败只计数，不打断任何东西。
            self.write_failures += 1

    def close_console_log(self) -> None:
        """关闭终端日志。先于 close() 调用，让最后几行也落盘。"""
        handle, self._console_log = self._console_log, None
        if handle is None:
            return
        try:
            handle.close()
        except Exception:
            pass

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

    def _keyframes_remaining(self) -> bool:
        """关键帧还有配额吗？`max_keyframes <= 0` 表示不限制。

        关键帧是**调试/运行记录**用的，不是算分的图，所以封顶；
        得分截图（`scoring/`）走 `save_task_evidence`，不经过这里、不受上限影响。
        """
        if self.max_keyframes <= 0:
            return True
        return self.snapshots < self.max_keyframes

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
            "max_keyframes": self.max_keyframes,
            "log": os.path.basename(str(self.log_path)) if self.log_path else "",
            "console_log": (
                os.path.basename(str(self.console_log_path))
                if self.console_log_path else ""
            ),
            "scoring_directory": SCORING_DIRECTORY,
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
