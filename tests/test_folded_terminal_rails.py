"""保存真实轮廓层，离线验证端部折弯横梁，不依赖现场点云文件。"""
import json
from pathlib import Path
import unittest

import numpy as np

from tests._offline_imports import install_noop_logger_module

install_noop_logger_module()

from model.motionplan.MotionFrameByFramePlanning import (
    ContourRows, _detect_frame_edge_candidates, _detect_frame_edge_sequence,
    _build_three_region_boxes, _build_region_box_sequence,
    _folded_terminal_rail_indices,
)


class FoldedTerminalRailsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture = json.loads((Path(__file__).parent / "fixtures" /
                              "folded_terminal_rails.json").read_text(encoding="utf-8"))
        cls.right = [ContourRows(*np.asarray(a).T) for a in fixture["right"]]
        cls.left = ContourRows(*np.asarray(fixture["left"][0]).T)

    @staticmethod
    def edges(rows):
        return _detect_frame_edge_candidates(
            rows, y_threshold=10, minimum_x_change_mm=50,
            minimum_run_layers=4, minimum_horizontal_layers=1,
        )

    @staticmethod
    def boxes(rows, edges):
        return _build_three_region_boxes(
            rows, edges, y_threshold=10, minimum_middle_coverage=.5,
            horizontal_width_mm=80, minimum_horizontal_layers=1,
            rail_padding_layers=5, middle_trim_layers=5,
        )

    def test_source_frame_384_encloses_both_bends_and_excludes_middle_tabs(self):
        rows = self.right[5]
        edges = self.edges(rows)
        self.assertEqual(edges.detection_mode, "folded_terminal_rails")
        self.assertEqual((edges.in_down.height, edges.in_up.height), (1680, 2690))
        boxes = self.boxes(rows, edges)
        self.assertEqual((boxes.lower.y_min, boxes.lower.y_max), (1460, 1690))
        self.assertEqual((boxes.upper.y_min, boxes.upper.y_max), (2680, 2930))
        for box, low, high in ((boxes.lower, 1460, 1680), (boxes.upper, 2690, 2930)):
            mask = (rows.height >= low) & (rows.height <= high)
            self.assertLessEqual(box.x_min, np.min(rows.v_min[mask]))
            self.assertGreaterEqual(box.x_max, np.max(rows.v_max[mask]))
        self.assertLess(boxes.lower.y_max, 1790)
        self.assertGreater(boxes.upper.y_min, 2570)

    def test_neighbor_smoothing_preserves_detected_bend_envelopes(self):
        edges = _detect_frame_edge_sequence(
            self.right, y_threshold=10, minimum_x_change_mm=50,
            minimum_run_layers=4, minimum_horizontal_layers=1,
        )
        boxes = _build_region_box_sequence(
            self.right, edges, y_threshold=10, smoothing_window=11,
            minimum_middle_coverage=.5, narrow_width_mm=80,
            minimum_horizontal_layers=1, rail_padding_layers=5, middle_trim_layers=5,
        )
        for i in range(2, 9):
            with self.subTest(source_frame=379 + i):
                self.assertEqual(edges[i].detection_mode, "folded_terminal_rails")
                raw = self.boxes(self.right[i], edges[i])
                for actual, expected in ((boxes[i].lower, raw.lower), (boxes[i].upper, raw.upper)):
                    self.assertLessEqual(actual.x_min, expected.x_min)
                    self.assertGreaterEqual(actual.x_max, expected.x_max)
                    self.assertLessEqual(actual.y_min, expected.y_min)
                    self.assertGreaterEqual(actual.y_max, expected.y_max)
        self.assertEqual((boxes[5].lower.y_max, boxes[5].upper.y_min), (1690, 2680))
        self.assertIsNotNone(boxes[5].middle)

    def test_left_frame_keeps_existing_detection(self):
        edges = self.edges(self.left)
        self.assertNotEqual(edges.detection_mode, "folded_terminal_rails")
        self.assertIsNotNone(self.boxes(self.left, edges))

    def test_detection_is_independent_of_absolute_coordinates(self):
        rows = self.right[5]
        translated = ContourRows(rows.height + 400, rows.v_min - 700, rows.v_max - 700)
        edges = self.edges(translated)
        self.assertEqual(edges.detection_mode, "folded_terminal_rails")
        self.assertEqual((edges.in_down.height, edges.in_up.height), (2080, 3090))

    def test_back_side_width_bulges_are_not_folded_rails(self):
        rows = self.right[5]
        widths = rows.v_max - rows.v_min
        outward_only = ContourRows(rows.height, np.full(len(rows), 1670.), 1690 + widths)
        self.assertIsNone(_folded_terminal_rail_indices(outward_only, 10, 50, 4))

    def test_missing_layers_do_not_bridge_disconnected_components(self):
        rows = self.right[5]
        keep = rows.height != 1600
        missing = ContourRows(rows.height[keep], rows.v_min[keep], rows.v_max[keep])
        self.assertIsNone(_folded_terminal_rail_indices(missing, 10, 50, 4))

    def test_isolated_translated_noise_is_not_a_folded_rail(self):
        heights = np.arange(1000, 2500, 10, dtype=float)
        low, high = np.full(150, 1670.), np.full(150, 1690.)
        low[[1, 148]], high[[1, 148]] = 1550, 1570
        self.assertIsNone(_folded_terminal_rail_indices(ContourRows(heights, low, high), 10, 50, 4))


if __name__ == "__main__":
    unittest.main()
