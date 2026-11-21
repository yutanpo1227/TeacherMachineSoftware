"""MaixPy実機APIを置き換え、前後検出と送信を確認する標準unittest。"""
import contextlib
import io
from pathlib import Path
import runpy
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


class FakeUART:
    UART1 = 1

    def __init__(self, *args, **kwargs):
        self.baud = args[1]
        self.frames = []

    def write(self, frame):
        self.frames.append(frame)


class Blob:
    def __init__(self, x, y, pixels=200):
        self.x, self.y, self.count = x, y, pixels

    def cx(self):
        return self.x

    def cy(self):
        return self.y

    def pixels(self):
        return self.count

    def rect(self):
        return (self.x - 5, self.y - 5, 10, 10)


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.sensor = types.SimpleNamespace(
            reset=lambda **kw: None,
            RGB565=1, QVGA=2,
            set_pixformat=lambda *_: None,
            set_framesize=lambda *_: None,
            set_hmirror=lambda *_: None,
            set_vflip=lambda *_: None,
            skip_frames=lambda **kw: None,
            set_auto_gain=lambda *a, **kw: None,
            set_auto_whitebal=lambda *_: None,
        )
        fm = types.SimpleNamespace(
            register=lambda *a, **kw: None,
            fpioa=types.SimpleNamespace(UART1_RX=1, UART1_TX=2))
        with patch.dict(sys.modules, {
            "sensor": self.sensor,
            "fpioa_manager": types.SimpleNamespace(fm=fm),
            "machine": types.SimpleNamespace(UART=FakeUART),
        }):
            self.module = runpy.run_path(str(ROOT / "camera/V-mirror.py"))
        # run_pathの戻り値と関数のglobalsは別の辞書。
        self.g = self.module["main"].__globals__

    def test_angles_and_wire_format(self):
        encode = self.g["encode_frame"]
        self.assertEqual(encode(None, [None, None], [None, None]),
                         "V3,-1.0,0,0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1,-1\n")
        self.assertEqual(encode({"robot_angle": 359.99, "pixels": 200, "name": "B"},
                                [None, None], [None, None]).split(",")[:4],
                         ["V3", "0.0", "200", "2"])
        self.assertEqual(self.g["uart"].baud, 115200)

    def run_one_frame(self, ball_visible):
        g = self.g

        class Image:
            def __init__(self):
                self.drawn = False
                self.calls = 0

            def find_blobs(image, thresholds, **kw):
                self.assertFalse(image.drawn, "描画前に全色を検出する")
                image.calls += 1
                front = kw["roi"] == g["ROI_FRONT"]
                x = kw["roi"][0] + 40
                if thresholds == g["ORANGE_THRESHOLDS"] and front and ball_visible:
                    return [Blob(x, 120)]
                if thresholds == g["YELLOW_THRESHOLDS"] and front:
                    return [Blob(x, 180)]
                if thresholds == g["BLUE_THRESHOLDS"] and not front:
                    return [Blob(x, 60)]
                return []

            def draw_rectangle(image, *args, **kw):
                image.drawn = True

            draw_cross = draw_rectangle
            draw_line = draw_rectangle

        img = Image()
        snapshots = iter([img])
        self.sensor.snapshot = lambda: next(snapshots)
        g["time"] = types.SimpleNamespace(clock=lambda: types.SimpleNamespace(
            tick=lambda: None, fps=lambda: 30))
        g["PRINT_EVERY_N_FRAMES"] = 100
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(StopIteration):
                g["main"]()
        self.assertEqual(img.calls, 6)
        self.assertTrue(img.drawn)
        return g["uart"].frames

    def test_front_ball_yellow_and_back_blue_in_same_frame(self):
        self.assertEqual(self.run_one_frame(True), ["V3,90.2,200,1,110.3,3.0,-1.0,-1.0,-1.0,-1.0,290.1,3.0,-1,106\n"])

    def test_goals_are_sent_even_without_ball(self):
        self.assertEqual(self.run_one_frame(False), ["V3,-1.0,0,0,110.3,3.0,-1.0,-1.0,-1.0,-1.0,290.1,3.0,-1,106\n"])

    def test_clipped_goal_invalidates_distance_but_keeps_angle(self):
        class Image:
            def find_blobs(image, *args, **kwargs):
                return [Blob(200, 5)]
        det = self.g["detect_view"](Image(), "F", self.g["ROI_FRONT"], 90, 1, None)
        self.assertGreaterEqual(self.g["detection_angle"](det), 0)
        self.assertEqual(self.g["detection_span"](det), -1)

    def test_rear_bottom_gap_accepts_y_cropped_goal(self):
        class RectangleBlob(Blob):
            def __init__(self, rect):
                self.bounds = rect
            def rect(self):
                return self.bounds
        gap = self.g["rear_bottom_gap"]
        goal = RectangleBlob((0, 0, 45, 239))
        det = {"name": "B", "roi": self.g["ROI_BACK"], "blob": goal}
        # 後方ROIは幅150。ゴール矩形の右端44との差が106になる。
        self.assertEqual(gap(det), 106)
        det["blob"] = RectangleBlob((0, 0, 149, 239))
        self.assertEqual(gap(det), -1)
        self.assertEqual(gap(None), -1)

    def test_front_noise_does_not_replace_rear_goal(self):
        class Image:
            def find_blobs(image, *args, **kwargs):
                front = kwargs["roi"] == self.g["ROI_FRONT"]
                return [Blob(200 if front else 40, 120, 10000 if front else 200)]
        views = self.g["detect_views"](Image(), self.g["YELLOW_THRESHOLDS"])
        fields = self.g["encode_frame"](None, views, [None, None]).strip().split(",")
        self.assertEqual(fields[4], "90.2")
        self.assertEqual(fields[6], "270.0")

    def test_goal_center_with_depth_edge_clipped(self):
        class RectBlob(Blob):
            def rect(self):
                return (0, 80, 40, 100)
            def cy(self):
                return 100  # 色画素の重心が矩形の中心からずれた場合。
        class Image:
            def find_blobs(image, *args, **kwargs):
                return [RectBlob(10, 100)]
        det = self.g["detect_view"](Image(), "B", self.g["ROI_BACK"], 270, -1, None)
        self.assertGreater(self.g["detection_span"](det), 0)
        self.assertAlmostEqual(self.g["goal_direction"](det), 266.7, places=1)
        self.assertNotEqual(self.g["goal_direction"](det), self.g["detection_angle"](det))

    def test_clipped_rear_goal_uses_visible_rectangle_center(self):
        class RectBlob(Blob):
            def rect(self):
                return (0, 0, 40, 180)
            def cy(self):
                return 40  # 重心が見えている矩形の中点と違っても中点を採用。
        class Image:
            def find_blobs(image, *args, **kwargs):
                return [RectBlob(10, 40)]
        det = self.g["detect_view"](Image(), "B", self.g["ROI_BACK"], 270, -1, None)
        self.assertEqual(self.g["detection_span"](det), -1)
        self.assertAlmostEqual(self.g["goal_direction"](det), 280.0, places=1)
        fields = self.g["encode_frame"](None, [None, det], [None, None]).split(",")
        self.assertEqual(fields[6:8], ["280.0", "-1.0"])


if __name__ == "__main__":
    unittest.main()
