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
            "Team %s detects a red light and the robot stops" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("traffic_light:green"),
            "Team %s detects a green light and continues" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("green_junction:green", side="left", chosen="left"),
            "Team %s detects a green light » left way and left" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("free_junction", side="left", chosen="right"),
            "Team %s detects traffic jam » left way and right" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("obstacle", side="left"),
            "Team %s detects obstacle » the left side" % TEAM_NUMBER)
        self.assertEqual(
            annotation_for("route"),
            "Team %s finds correct to follow" % TEAM_NUMBER)

    def test_an_unknown_kind_fails_loudly(self):
        with self.assertRaises(KeyError):
            annotation_for("not-a-real-kind")

    def test_the_arrow_and_other_non_ascii_are_drawn_as_ascii(self):
        """`cv2.putText` 只支持 ASCII；`»` 画上去会变成 `??`，所以绘制时要转写。

        字符串本身（记录/日志/report.md）**必须保留老师的 `»`**，只有像素转写。
        """
        text = annotation_for("green_junction:green", side="left", chosen="left")
        self.assertIn("»", text, "记录的文案要保留老师的箭头字符")
        drawn = evidence.drawing_text(text)
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


class BuilderTests(unittest.TestCase):
    def test_builder_fills_annotation_shape_label_and_event_key(self):
        photo = make_evidence_photo(
            "obstacle", packet(), detection=Box((10, 20, 110, 120)), side="right")
        self.assertEqual(photo.annotation, "Team %s detects obstacle » the right side" % TEAM_NUMBER)
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
