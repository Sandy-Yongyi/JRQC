import math
from collections import Counter
from dataclasses import dataclass

from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper


@dataclass(frozen=True)
class XNUpdown4StructureGeometry:
    has_data: bool = False
    complete: bool = False
    overall_x_min: int | None = None
    yup: int | None = None
    ydown: int | None = None
    ymiddle: int | None = None
    up_boundary: int | None = None
    down_boundary: int | None = None
    up_y_min: int | None = None
    up_y_max: int | None = None
    down_y_min: int | None = None
    down_y_max: int | None = None
    valid_frame_count: int = 0
    required_frame_count: int = 0


@dataclass(frozen=True)
class XNUpdown4RegionGeometry:
    y_min: int | None = None
    y_max: int | None = None
    x_min: int | None = None
    x_max: int | None = None

    @property
    def complete(self) -> bool:
        return None not in (self.y_min, self.y_max, self.x_min, self.x_max)


class XNUpdown4FrameSearchHelper:
    """旧四枪顶底设备（xn_updown4）的横梁识别和区域数据提取。"""

    def __init__(self, z_threshold=10, beam_ratio=0.70):
        self.z_threshold = int(z_threshold)
        self.beam_ratio = float(beam_ratio)
        if self.z_threshold <= 0:
            raise ValueError(f"z_threshold 必须大于 0，当前值: {self.z_threshold}")
        if not 0 < self.beam_ratio <= 1:
            raise ValueError(f"beam_ratio 必须在 (0, 1] 内，当前值: {self.beam_ratio}")
        self.frame_helper = FrameSearchHelper(z_threshold=self.z_threshold)

    def get_frames(self, machine_cfg, frame_queue_manager):
        return self.frame_helper.get_side_frames(machine_cfg, frame_queue_manager)

    def get_search_window(self, machine_cfg, runtime_cfg, frame_count):
        return self._get_window(machine_cfg, runtime_cfg, frame_count, "search_front_z_offset", "search_after_z_offset", 500)

    def get_spray_window(self, machine_cfg, runtime_cfg, frame_count):
        return self._get_window(machine_cfg, runtime_cfg, frame_count, "out_z_front_offset", "out_z_after_offset", 100)

    def identify_structure(self, frames, window, in_up_y_offset, in_down_y_offset):
        if window is None:
            return XNUpdown4StructureGeometry()

        valid_frames = []
        y_counts = Counter()
        overall_x_min_values = []
        all_rows = []
        for frame_idx in range(window[0], window[1] + 1):
            frame_rows = [row for row in self._get_rows(frames, frame_idx) if self.frame_helper.row_has_data(row)]
            if not frame_rows:
                continue
            valid_frames.append(frame_idx)
            all_rows.extend(frame_rows)
            y_counts.update({int(getattr(row, "H_Axis", 0) or 0) for row in frame_rows if int(getattr(row, "H_Axis", 0) or 0) != 0})
            overall_x_min_values.extend(int(getattr(row, "V_Axis_Min", 0) or 0) for row in frame_rows if int(getattr(row, "V_Axis_Min", 0) or 0) != 0)

        valid_frame_count = len(valid_frames)
        if valid_frame_count == 0:
            return XNUpdown4StructureGeometry()

        required_frame_count = math.ceil(valid_frame_count * self.beam_ratio)
        qualifying_y = [y_value for y_value, count in y_counts.items() if count >= required_frame_count]
        overall_x_min = min(overall_x_min_values) if overall_x_min_values else None
        if not qualifying_y:
            return XNUpdown4StructureGeometry(has_data=True, overall_x_min=overall_x_min, valid_frame_count=valid_frame_count, required_frame_count=required_frame_count)

        yup = max(qualifying_y)
        ydown = min(qualifying_y)
        if yup <= ydown:
            return XNUpdown4StructureGeometry(
                has_data=True, overall_x_min=overall_x_min, yup=yup, ydown=ydown, valid_frame_count=valid_frame_count, required_frame_count=required_frame_count
            )

        ymiddle = int((yup + ydown) / 2)
        up_boundary = ymiddle + int(in_up_y_offset)
        down_boundary = ymiddle - int(in_down_y_offset)
        up_y_values = [int(getattr(row, "H_Axis", 0) or 0) for row in all_rows if int(getattr(row, "H_Axis", 0) or 0) >= up_boundary]
        down_y_values = [int(getattr(row, "H_Axis", 0) or 0) for row in all_rows if 0 < int(getattr(row, "H_Axis", 0) or 0) <= down_boundary]
        complete = bool(up_y_values and down_y_values and overall_x_min is not None)
        return XNUpdown4StructureGeometry(has_data=True, complete=complete, overall_x_min=overall_x_min, yup=yup, ydown=ydown, ymiddle=ymiddle, up_boundary=up_boundary,
                                          down_boundary=down_boundary, up_y_min=min(up_y_values) if up_y_values else None, up_y_max=max(up_y_values) if up_y_values else None,
                                          down_y_min=min(down_y_values) if down_y_values else None, down_y_max=max(down_y_values) if down_y_values else None,
                                          valid_frame_count=valid_frame_count, required_frame_count=required_frame_count)

    def collect_region(self, frames, window, y_min=None, y_max=None):
        if window is None:
            return XNUpdown4RegionGeometry()
        y_values = []
        x_min_values = []
        x_max_values = []
        for frame_idx in range(window[0], window[1] + 1):
            for row in self._get_rows(frames, frame_idx):
                if not self.frame_helper.row_has_data(row):
                    continue
                y_value = int(getattr(row, "H_Axis", 0) or 0)
                if y_value == 0 or y_min is not None and y_value < int(y_min) or y_max is not None and y_value > int(y_max):
                    continue
                x_min = int(getattr(row, "V_Axis_Min", 0) or 0)
                x_max = int(getattr(row, "V_Axis_Max", 0) or 0)
                y_values.append(y_value)
                if x_min != 0:
                    x_min_values.append(x_min)
                if x_max != 0:
                    x_max_values.append(x_max)
        return XNUpdown4RegionGeometry(y_min=min(y_values) if y_values else None, y_max=max(y_values) if y_values else None,
                                       x_min=min(x_min_values) if x_min_values else None, x_max=max(x_max_values) if x_max_values else None)

    def has_data_in_y_band(self, frames, window, y_min, y_max):
        if window is None:
            return False
        band_min = min(int(y_min), int(y_max))
        band_max = max(int(y_min), int(y_max))
        for frame_idx in range(window[0], window[1] + 1):
            for row in self._get_rows(frames, frame_idx):
                y_value = int(getattr(row, "H_Axis", 0) or 0)
                if band_min <= y_value <= band_max and self.frame_helper.row_has_data(row):
                    return True
        return False

    def _get_window(self, machine_cfg, runtime_cfg, frame_count, front_key, after_key, default_offset):
        if int(frame_count or 0) <= 0:
            return None
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        front_offset = self._get_config_int(machine_cfg, runtime_cfg, front_key, default_offset)
        after_offset = self._get_config_int(machine_cfg, runtime_cfg, after_key, default_offset)
        first_idx = int((z_position + front_offset) / self.z_threshold)
        second_idx = int((z_position - after_offset) / self.z_threshold)
        raw_start = min(first_idx, second_idx)
        raw_end = max(first_idx, second_idx)
        if raw_end < 0 or raw_start >= frame_count:
            return None
        return max(0, raw_start), min(frame_count - 1, raw_end)

    @staticmethod
    def _get_rows(frames, frame_idx):
        if frame_idx < 0 or frame_idx >= len(frames):
            return []
        return getattr(frames[frame_idx], "FrameData", None) or []

    @staticmethod
    def _get_config_int(machine_cfg, runtime_cfg, key, default):
        value = runtime_cfg.get(key) if isinstance(runtime_cfg, dict) and key in runtime_cfg else machine_cfg.get(key, default)
        return int(default) if value is None else int(value)
