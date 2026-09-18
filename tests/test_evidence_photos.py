"""统一证据照片层：文案模板、构造器、画圈/画框、同一事件只存一张。

为什么单独一组：老师后来明确了 5 组"证据照片"（见 `EVIDENCE_PHOTOS.md`），
而且**照片张数就是分数**。这一层是集成侧统一实现的：文案模板、圆圈/矩形标注、
事件去重都在这里，5 个模块只负责在得分那一刻把请求交出来。
"""

import pathlib
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import evidence  # noqa: E402
from evidence import (  # noqa: E402
    ANNOTATION_TEMPLATES,
    EvidenceRecorder,
    TEAM_NUMBER,
    annotation_for,
    make_evidence_photo,
    render_task_evidence,
)
from models import FramePacket  # noqa: E402

YELLOW = (0, 255, 255)   # 证据层的标注色（BGR）


def blank_frame(width=640, height=360):
    return np.full((height, width, 3), 40, np.uint8)


def packet(image=None):
    return FramePacket(blank_frame() if image is None else image, 7, 3.5)


class Box:
    """最简检测结果：只要带 .box，证据层就能画。"""

    def __init__(self, box):
        self.box = box


class AnnotationTemplateTests(unittest.TestCase):
    def test_every_required_photo_has_a_template(self):
        # 老师要求的那几张，一个都不能少
        for kind in ("traffic_light:red", "traffic_light:green", "green_junction:green",
                     "green_junction:red", "free_junction", "obstacle", "route"):
            self.assertIn(kind, ANNOTATION_TEMPLATES)

    def test_texts_match_the_teacher_samples(self):
        self.assertEqual(
            annotation_for("traffic_light:red"),
            "Team %s detects a red light and stops the robot" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("traffic_light:green"),
            "Team %s detects a green light and continues" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("green_junction:green", side="left", chosen="left"),
            "Team %s detects a green light on the left way and chooses left" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("free_junction", side="left", chosen="right"),
            "Team %s detects traffic jam on the left way and chooses right" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("obstacle", side="left"),
            "Team %s detects an obstacle and chooses the left side" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("route"),
            "Team %s finds the correct line to follow" % TEAM_NUMBER)

    def test_an_unknown_kind_fails_loudly(self):
        with self.assertRaises(KeyError):
            annotation_for("not-a-real-kind")

    def test_all_templates_are_ascii_so_the_drawn_text_matches_the_record(self):
        """模板必须全是 ASCII：`cv2.putText` 的 Hershey 字体画不了非 ASCII（会变成 `??`），
        而 `report.md` 记的是原始字符串 —— 一旦模板里有非 ASCII，"图上的字"和"记录的字"
        就不一致了。2026-09-18 按老师给的示例原文改过之后模板已全是 ASCII，这条钉住它。
        """
        for kind, template in ANNOTATION_TEMPLATES.items():
            self.assertTrue(
                all(ord(ch) < 128 for ch in template),
                "%s 的模板含非 ASCII 字符：%r" % (kind, template),
            )

    def test_non_ascii_is_still_transliterated_for_drawing(self):
        """安全网：万一将来又有非 ASCII，绘制时必须转写（`»` -> `>>`），不能画成 `??`。"""
        drawn = evidence.drawing_text(
            "Team 10 detects a green light » left way and left"
        )
        self.assertNotIn("»", drawn)
        self.assertIn(">> left way and left", drawn)
        self.assertTrue(all(ord(ch) < 128 for ch in drawn), "画到图上的必须全是 ASCII")

    def test_rendering_a_green_junction_photo_does_not_leave_question_marks(self):
        """真渲染一遍：图上不该出现 `??` 这种被 Hershey 吃掉的字符。"""
        photo = make_evidence_photo(
            "green_junction:green", packet(),
            detection=Box((78, 78, 122, 122)), shape="circle", side="left", chosen="left")
        shown = render_task_evidence(photo)
        self.assertTrue(np.any(shown == YELLOW), "圈/字没画上去")

    def test_rendering_draws_actual_letter_pixels(self):
        """**像素级**验证：图上真的画出了一行字，而不只是框。

        为什么需要：以前只有"文案字符串对不对"的测试 —— 字符串对了但字没画上去
        （或画到画面外、被裁掉）一样拿不到分。做法：数黄色的"字母级"连通块
        （3~600 px）。矩形/圆圈轮廓只会留下少数几个大连通块，一行 40+ 字符的英文
        会留下几十个小块。
        """
        photo = make_evidence_photo(
            "obstacle", packet(), detection=Box((10, 20, 110, 120)), side="left")
        shown = render_task_evidence(photo)

        yellow = np.all(
            np.abs(shown.astype(int) - np.array(YELLOW)) <= 40, axis=2
        ).astype(np.uint8)
        count, _, stats, _ = cv2.connectedComponentsWithStats(yellow, connectivity=8)
        letters = sum(
            1 for index in range(1, count)
            if 3 <= stats[index, cv2.CC_STAT_AREA] <= 600
        )
        self.assertGreaterEqual(
            letters, 20,
            "图上没有画出成句的文字（字母级连通块只有 %d 个）" % letters,
        )

    def test_drawn_text_has_a_dark_outline_so_it_stays_readable(self):
        """黄字必须有黑色描边：否则压在浅色地面/白墙上根本看不清。"""
        photo = make_evidence_photo(
            "route", packet(), detection=Box((10, 20, 110, 120)))
        shown = render_task_evidence(photo)
        dark = np.all(shown.astype(int) <= 60, axis=2)
        self.assertGreater(int(dark.sum()), 50, "没有黑色描边（浅色背景上会看不清）")


class BuilderTests(unittest.TestCase):
    def test_builder_fills_annotation_shape_label_and_event_key(self):
        photo = make_evidence_photo(
            "obstacle", packet(), detection=Box((10, 20, 110, 120)), side="right")
        self.assertEqual(photo.annotation, "Team %s detects an obstacle and chooses the right side" % TEAM_NUMBER)
        self.assertEqual(photo.shape, "rect")
        self.assertEqual(photo.label, "obstacle")
        self.assertEqual(photo.event_key, "obstacle")
        self.assertEqual(photo.frame_sequence, 7)
        self.assertIn("obstacle", photo.request_id)

    def test_builder_copies_the_frame_so_a_reused_buffer_cannot_change_the_photo(self):
        image = blank_frame()
        photo = make_evidence_photo("route", packet(image))
        image[:] = 255          # 相机缓冲被下一帧复用
        self.assertFalse(np.all(photo.image == 255), "请求里存的应该是当时那份拷贝")

    def test_circle_kind_is_selectable(self):
        photo = make_evidence_photo("traffic_light:green", packet(), shape="circle")
        self.assertEqual(photo.shape, "circle")
        self.assertEqual(photo.label, "traffic_light_green")


class RenderTests(unittest.TestCase):
    def test_circle_is_drawn_around_the_box(self):
        photo = make_evidence_photo(
            "traffic_light:red", packet(), detection=Box((300, 80, 360, 140)), shape="circle")
        shown = render_task_evidence(photo)
        # 圆经过 (330, 110±30) 与 (330±30, 110)
        for point in ((330, 80), (360, 110), (330, 140), (300, 110)):
            self.assertTrue(
                np.any(shown[point[1] - 1:point[1] + 2, point[0] - 1:point[0] + 2] == YELLOW),
                "圆没画到 %s" % (point,))

    def test_text_sits_below_the_marked_shape(self):
        photo = make_evidence_photo(
            "obstacle", packet(), detection=Box((200, 60, 300, 160)), side="left")
        shown = render_task_evidence(photo)
        # 框底 160 + 24 = 184 那一行附近应该出现文字（黄色）
        band = shown[170:200, :, :]
        self.assertTrue(np.any(band == YELLOW), "文字没有画在框的下方")

    def test_rect_still_works_for_boxes(self):
        photo = make_evidence_photo("route", packet(), detection=Box((100, 100, 200, 200)))
        shown = render_task_evidence(photo)
        self.assertTrue(np.any(shown[99:102, 100:200] == YELLOW), "矩形上边没画出来")

    def test_without_any_shape_the_text_still_lands_on_the_image(self):
        photo = make_evidence_photo("route", packet())
        shown = render_task_evidence(photo)
        self.assertTrue(np.any(shown == YELLOW), "没有框时文字也应该画出来")


class DedupeTests(unittest.TestCase):
    def test_the_same_event_is_only_saved_once(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = EvidenceRecorder(directory=directory)
            self.addCleanup(recorder.close)
            first = make_evidence_photo("obstacle", packet(), detection=Box((10, 10, 90, 90)), side="left")
            second = make_evidence_photo("obstacle", packet(), detection=Box((20, 20, 99, 99)), side="left")
            self.assertTrue(recorder.save_task_evidence(first))
            # 第二次同事件：返回 True（"已经有照片了"），但不重复写盘
            self.assertTrue(recorder.save_task_evidence(second))
            self.assertEqual(recorder.task_snapshots, 1, "同一事件写了两张")
            files = [p for p in recorder.run_directory.glob("task_*.jpg")]
            self.assertEqual(len(files), 1, "落盘的文件应该只有一张：%s" % files)
            recorder.close()

    def test_different_events_are_saved_separately(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = EvidenceRecorder(directory=directory)
            self.addCleanup(recorder.close)
            red = make_evidence_photo("traffic_light:red", packet(), shape="circle")
            green = make_evidence_photo("traffic_light:green", packet(), shape="circle")
            self.assertTrue(recorder.save_task_evidence(red))
            self.assertTrue(recorder.save_task_evidence(green))
            self.assertEqual(recorder.task_snapshots, 2, "红灯和绿灯必须分别保存")
            recorder.close()


if __name__ == "__main__":
    unittest.main()
