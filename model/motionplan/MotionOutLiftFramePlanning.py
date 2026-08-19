import os
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper
from model.plc.MovingFrameData import AxisData
from model.utils.TomlLoader import TomlLoader


class MotionOutLiftFramePlanning:
    """按帧模式二维设备规划。"""

    def __init__(self, read_data_cfg=None):
        if read_data_cfg is None:
            read_data_cfg = TomlLoader.load(os.path.join(os.getcwd(), "model", "tomls", "ReadDataConfig.toml"))
        z_threshold_value = read_data_cfg.get("z_threshold", 10)
        self.z_threshold = 10 if z_threshold_value is None else int(z_threshold_value)
        if self.z_threshold <= 0:
            raise ValueError(f"z_threshold 必须大于 0，当前值: {self.z_threshold}")
        self.frame_search_helper = FrameSearchHelper(z_threshold=self.z_threshold)

    def auto_out_lift_machine_move(self, machine_cfg, runtime_cfg, frame_queue_manager):
        """在设备 Z 窗口内汇总侧面帧，输出 Pos=XMIN、Speed=YMAX、Status=YMIN。"""
        frames = self.frame_search_helper.get_side_frames(machine_cfg, frame_queue_manager)
        start_idx, end_idx = self._get_z_window(machine_cfg, runtime_cfg)
        return self._build_axis_data(frames, start_idx, end_idx)

    def _get_z_window(self, machine_cfg, runtime_cfg):
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        z_front_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_z_front_offset", 100)
        z_after_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_z_after_offset", 100)
        start_z_machine = z_position - z_front_offset
        end_z_machine = z_position + z_after_offset
        start_idx = max(0, int(start_z_machine / self.z_threshold))
        end_idx = max(0, int(end_z_machine / self.z_threshold))
        return start_idx, end_idx

    def _build_axis_data(self, frames, start_idx, end_idx):
        if not frames or start_idx > end_idx or start_idx >= len(frames):
            return AxisData(Pos=0, Speed=0, Status=0)

        x_min_values = []
        y_values = []
        for frame_idx in range(start_idx, min(end_idx, len(frames) - 1) + 1):
            frame = frames[frame_idx]
            for row in getattr(frame, "FrameData", None) or []:
                if not self.frame_search_helper.row_has_data(row):
                    continue
                x_min = int(getattr(row, "V_Axis_Min", 0) or 0)
                y_value = int(getattr(row, "H_Axis", 0) or 0)
                if x_min != 0:
                    x_min_values.append(x_min)
                if y_value != 0:
                    y_values.append(y_value)

        if not x_min_values or not y_values:
            return AxisData(Pos=0, Speed=0, Status=0)
        return AxisData(Pos=min(x_min_values), Speed=max(y_values), Status=min(y_values))

    @staticmethod
    def _get_config_int(machine_cfg, runtime_cfg, key, default):
        value = runtime_cfg.get(key) if isinstance(runtime_cfg, dict) and key in runtime_cfg else machine_cfg.get(key, default)
        return int(default) if value is None else int(value)
