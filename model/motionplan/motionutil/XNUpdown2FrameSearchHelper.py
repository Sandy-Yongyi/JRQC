from dataclasses import dataclass

from model.motionplan.MachineAxisMap import get_axis_position_limits
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper


@dataclass(frozen=True)
class XNUpdown2FrameGeometry:
    """xn_updown2 当前 Z 工作窗口内的原始点云几何。"""

    has_data: bool = False
    raw_x_min: int | None = None
    raw_x_max: int | None = None
    raw_y_min: int | None = None
    band_y_max: int | None = None


class XNUpdown2FrameSearchHelper:
    """两枪顶底设备按帧点云搜索助手，不保存任何运动状态。"""

    def __init__(self, z_threshold: int = 10):
        self.z_threshold = int(z_threshold)
        if self.z_threshold <= 0:
            raise ValueError(f"z_threshold 必须大于 0，当前值: {self.z_threshold}")
        self.frame_helper = FrameSearchHelper(z_threshold=self.z_threshold)

    def get_geometry(self, machine_cfg, runtime_cfg, frame_queue_manager) -> XNUpdown2FrameGeometry:
        """按设备方向和 Z 工作窗口汇总原始 X/Y 几何量。"""
        frames = self.frame_helper.get_side_frames(machine_cfg, frame_queue_manager)
        window = self._get_work_window(machine_cfg, runtime_cfg, len(frames))
        if window is None:
            return XNUpdown2FrameGeometry()

        origin_pos = self._get_int_list(machine_cfg, "origin_pos")
        if len(origin_pos) < 2:
            raise ValueError("origin_pos 至少需要 y1/y2 两个值")
        up_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_up_y_offset", 100)
        _, y2_max_limit = get_axis_position_limits(machine_cfg, "y2")
        y2_band_min = origin_pos[1] - up_offset
        y2_band_max = origin_pos[1] + y2_max_limit
        x_min_values = []
        x_max_values = []
        y_values = []
        band_y_values = []

        for frame_index in range(window[0], window[1] + 1):
            frame = frames[frame_index]
            for row in getattr(frame, "FrameData", None) or []:
                if not self.frame_helper.row_has_data(row):
                    continue

                h_axis = self._to_int(getattr(row, "H_Axis", 0), "H_Axis")
                v_axis_min = self._to_int(getattr(row, "V_Axis_Min", 0), "V_Axis_Min")
                v_axis_max = self._to_int(getattr(row, "V_Axis_Max", 0), "V_Axis_Max")
                y_values.append(h_axis)
                x_min_values.append(v_axis_min)
                x_max_values.append(v_axis_max)
                if y2_band_min <= h_axis <= y2_band_max:
                    band_y_values.append(h_axis)

        if not y_values or not x_min_values or not x_max_values:
            return XNUpdown2FrameGeometry()

        return XNUpdown2FrameGeometry(
            has_data=True,
            raw_x_min=min(x_min_values),
            raw_x_max=max(x_max_values),
            raw_y_min=min(y_values),
            band_y_max=max(band_y_values) if band_y_values else None,
        )

    def _get_work_window(self, machine_cfg, runtime_cfg, frame_count: int):
        if int(frame_count) <= 0:
            return None

        z_position = self._get_config_int(machine_cfg, runtime_cfg, "z_position", 0)
        front_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_z_front_offset", 100)
        after_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_z_after_offset", 100)
        first_index = int((z_position + front_offset) / self.z_threshold)
        second_index = int((z_position - after_offset) / self.z_threshold)
        # 绝对 Z 范围转换后仍按当前方向帧栈边界截断。
        start_index = min(first_index, second_index)
        end_index = max(first_index, second_index)

        if end_index < 0 or start_index >= frame_count:
            return None
        return max(0, start_index), min(frame_count - 1, end_index)

    @staticmethod
    def _get_config_int(machine_cfg, runtime_cfg, key: str, default: int) -> int:
        if key == "z_position":
            value = machine_cfg.get(key, default)
        else:
            value = runtime_cfg.get(key) if isinstance(runtime_cfg, dict) and key in runtime_cfg else machine_cfg.get(key, default)
        return XNUpdown2FrameSearchHelper._to_int(value if value is not None else default, key)

    @staticmethod
    def _get_int_list(machine_cfg, key: str) -> list[int]:
        values = machine_cfg.get(key, [])
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"{key} 必须为列表")
        return [XNUpdown2FrameSearchHelper._to_int(value, key) for value in values]

    @staticmethod
    def _to_int(value, key: str) -> int:
        try:
            return int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} 必须为整数，当前值: {value}") from exc
