from dataclasses import dataclass
from model.utils.MachineConfigUtil import get_machine_offset


@dataclass(frozen=True)
class FrameWindow:
    """按帧运动使用的 Z 轴窗口索引。"""

    start: int
    center: int
    end: int


class FrameSearchHelper:
    """帧数据搜索辅助类。"""

    def __init__(self, z_threshold: int = 10):
        self.z_threshold = int(z_threshold)
        if self.z_threshold <= 0:
            raise ValueError(f"z_threshold 必须大于 0，当前值: {z_threshold}")

    def get_side_frames(self, machine_cfg, frame_queue_manager):
        direction = self.get_side_direction(machine_cfg)
        return frame_queue_manager.frame_stack.get(direction, [])

    def get_upper_frames(self, machine_cfg, frame_queue_manager):
        direction = self.get_upper_direction(machine_cfg)
        return frame_queue_manager.frame_stack.get(direction, [])

    def get_side_direction(self, machine_cfg):
        return "left" if machine_cfg.get("install_orietation", "left") == "left" else "right"

    def get_upper_direction(self, machine_cfg):
        return "left_upper" if machine_cfg.get("install_orietation", "left") == "left" else "right_upper"

    def create_window(self, start: int, center: int, end: int,
                      frame_count: int) -> FrameWindow:
        """建立单调且不超过帧队列长度的窗口索引。"""
        last_index = max(0, int(frame_count or 0) - 1)
        start = max(0, min(int(start), last_index))
        center = max(start, min(int(center), last_index))
        end = max(center, min(int(end), last_index))
        return FrameWindow(start=start, center=center, end=end)

    def build_window(self, machine_cfg, runtime_cfg, z_cur: int,
                     frame_count: int) -> FrameWindow:
        """根据设备定位、当前 Z 位置和前后偏移建立动态窗口。"""
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        front_offset = get_machine_offset(machine_cfg, "out_z_front_offset", runtime_cfg)
        after_offset = get_machine_offset(machine_cfg, "out_z_after_offset", runtime_cfg)
        center_z = z_position + int(z_cur or 0)
        return self.create_window(
            int((center_z - front_offset) / self.z_threshold),
            int(center_z / self.z_threshold),
            int((center_z + after_offset) / self.z_threshold),
            frame_count,
        )

    def get_frame_by_index(self, frames, index):
        if index < 0 or index >= len(frames):
            return None
        return frames[index]

    def iter_window_indices(self, start_idx, end_idx):
        if start_idx <= end_idx:
            return range(start_idx, end_idx + 1)
        return range(start_idx, end_idx - 1, -1)

    def row_has_data(self, row):
        if row is None:
            return False
        h_axis = getattr(row, "H_Axis", 0)
        v_max = getattr(row, "V_Axis_Max", 0)
        v_min = getattr(row, "V_Axis_Min", 0)
        return not (int(h_axis or 0) == 0 and int(v_max or 0) == 0 and int(v_min or 0) == 0)

    def frame_has_data(self, frame):
        if frame is None or not getattr(frame, "FrameData", None):
            return False
        return any(self.row_has_data(row) for row in frame.FrameData)

    def has_start_signature(self, frames, window: FrameWindow, count: int) -> bool:
        """窗口前端连续 count 帧有数据且其余帧为空时判定开始。"""
        count = int(count or 0)
        window_length = window.end - window.start + 1
        if count <= 0 or count >= window_length:
            return False

        boundary_end = window.start + count
        return (
            all(
                self.frame_has_data(self.get_frame_by_index(frames, index))
                for index in range(window.start, boundary_end)
            )
            and all(
                not self.frame_has_data(self.get_frame_by_index(frames, index))
                for index in range(boundary_end, window.end + 1)
            )
        )

    def has_end_signature(self, frames, window: FrameWindow, count: int) -> bool:
        """窗口后端连续 count 帧有数据且其余帧为空时判定结束。"""
        count = int(count or 0)
        window_length = window.end - window.start + 1
        if count <= 0 or count >= window_length:
            return False

        boundary_start = window.end - count + 1
        return (
            all(
                not self.frame_has_data(self.get_frame_by_index(frames, index))
                for index in range(window.start, boundary_start)
            )
            and all(
                self.frame_has_data(self.get_frame_by_index(frames, index))
                for index in range(boundary_start, window.end + 1)
            )
        )

    def window_is_empty(self, frames, window: FrameWindow) -> bool:
        """判断整个 Z 窗口是否已经没有有效数据。"""
        return all(
            not self.frame_has_data(self.get_frame_by_index(frames, index))
            for index in range(window.start, window.end + 1)
        )

    def frame_has_y_in_band(self, frame, y_lower, y_upper):
        if frame is None or not getattr(frame, "FrameData", None):
            return False
        for row in frame.FrameData:
            if not self.row_has_data(row):
                continue
            y_val = int(getattr(row, "H_Axis", 0) or 0)
            if y_lower <= y_val <= y_upper:
                return True
        return False

    def scan_y_range(self, frames, start_idx, end_idx):
        y_values = []
        for idx in self.iter_window_indices(start_idx, end_idx):
            frame = self.get_frame_by_index(frames, idx)
            if frame is None or not getattr(frame, "FrameData", None):
                continue
            for row in frame.FrameData:
                if self.row_has_data(row):
                    y_values.append(int(row.H_Axis))
        if not y_values:
            return None, None
        return min(y_values), max(y_values)

    def scan_y_min(self, frames, start_idx, end_idx):
        y_min, _ = self.scan_y_range(frames, start_idx, end_idx)
        return y_min

    def collect_x_min_values(self, frames, start_idx, end_idx, y_start, y_end):
        x_values = []
        for idx in self.iter_window_indices(start_idx, end_idx):
            frame = self.get_frame_by_index(frames, idx)
            if frame is None or not getattr(frame, "FrameData", None):
                continue
            for row in frame.FrameData:
                if not self.row_has_data(row):
                    continue
                y_val = int(getattr(row, "H_Axis", 0) or 0)
                if y_start <= y_val <= y_end:
                    x_min = int(getattr(row, "V_Axis_Min", 0) or 0)
                    if x_min != 0:
                        x_values.append(x_min)
        return x_values

    def collect_x_values(self, frames, start_idx, end_idx, y_start, y_end):
        x_values = []
        for idx in self.iter_window_indices(start_idx, end_idx):
            frame = self.get_frame_by_index(frames, idx)
            if frame is None or not getattr(frame, "FrameData", None):
                continue
            for row in frame.FrameData:
                if not self.row_has_data(row):
                    continue
                y_val = int(getattr(row, "H_Axis", 0) or 0)
                if y_start <= y_val <= y_end:
                    x_min = int(getattr(row, "V_Axis_Min", 0) or 0)
                    x_max = int(getattr(row, "V_Axis_Max", 0) or 0)
                    if x_min != 0:
                        x_values.append(x_min)
                    if x_max != 0:
                        x_values.append(x_max)
        return x_values

    def collect_x_range(self, frames, window: FrameWindow,
                        y_min: int, y_max: int):
        """在完整 Z 窗口和指定 Y 区间中收集 X 最小值及最大值。"""
        y_start = min(int(y_min), int(y_max))
        y_end = max(int(y_min), int(y_max))
        values = self.collect_x_values(
            frames,
            window.start,
            window.end,
            y_start,
            y_end,
        )
        if not values:
            return None, None
        return min(values), max(values)

    def scan_vertical_range_by_row_window(self, frame, start_row, end_row):
        if frame is None or not getattr(frame, "FrameData", None):
            return None, None
        if start_row > end_row:
            start_row, end_row = end_row, start_row

        row_start = max(0, int(start_row))
        row_end = min(len(frame.FrameData) - 1, int(end_row))
        if row_start > row_end:
            return None, None

        v_mins = []
        v_maxs = []
        for row_idx in range(row_start, row_end + 1):
            row = frame.FrameData[row_idx]
            if not self.row_has_data(row):
                continue
            v_min = int(getattr(row, "V_Axis_Min", 0) or 0)
            v_max = int(getattr(row, "V_Axis_Max", 0) or 0)
            if v_min != 0:
                v_mins.append(v_min)
            if v_max != 0:
                v_maxs.append(v_max)

        if not v_mins and not v_maxs:
            return None, None

        final_min = min(v_mins) if v_mins else None
        final_max = max(v_maxs) if v_maxs else None
        return final_min, final_max
