"""运动规划配置解析与启动参数校验。"""

from collections.abc import Mapping
from dataclasses import dataclass
import math

from model.motionplan._validation import (
    _require_int,
)


@dataclass(frozen=True)
class FrameGeometryPlanningParameters:
    """逐帧轮廓参数；旧配置默认采用600mm横梁净距、允许缺失一层。"""

    minimum_x_change_mm: float
    minimum_run_layers: int
    smoothing_window: int
    minimum_middle_coverage: float
    narrow_width_mm: float
    minimum_horizontal_layers: int
    rail_padding_layers: int
    middle_trim_layers: int
    gun_obstacle_half_height_mm: int = 30
    minimum_rail_gap_mm: int = 600
    maximum_missing_y_layers: int = 1

    def __post_init__(self) -> None:
        for name in ("minimum_x_change_mm", "narrow_width_mm"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} 必须是数值")
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f"{name} 必须是大于0的有限数")
        for name in (
            "minimum_run_layers",
            "smoothing_window",
            "minimum_horizontal_layers",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} 必须是整数")
            if value <= 0:
                raise ValueError(f"{name} 必须大于0")
        if self.smoothing_window % 2 == 0:
            raise ValueError("smoothing_window 必须是正奇数")
        coverage = self.minimum_middle_coverage
        if isinstance(coverage, bool) or not isinstance(coverage, (int, float)):
            raise TypeError("minimum_middle_coverage 必须是数值")
        if not math.isfinite(float(coverage)) or not 0 <= float(coverage) <= 1:
            raise ValueError("minimum_middle_coverage 必须在0到1之间")
        for name in (
            "rail_padding_layers",
            "middle_trim_layers",
            "gun_obstacle_half_height_mm",
            "minimum_rail_gap_mm",
            "maximum_missing_y_layers",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} 必须是整数")
            if value < 0:
                raise ValueError(f"{name} 不能小于0")


@dataclass(frozen=True)
class FrameByFramePlanningParameters:
    """逐帧运动规划的启动配置。"""

    x_arrival_confirmations: int
    x_position_tolerance_mm: int
    minimum_gun_y_distance_mm: int
    geometry: FrameGeometryPlanningParameters
    short_stroke_x_offset_mm: int = 100
    short_stroke_threshold_mm: int = 40

    @classmethod
    def from_config(cls, planning_config: Mapping) -> "FrameByFramePlanningParameters":
        if not isinstance(planning_config, Mapping):
            raise TypeError("MotionPlanningConfig 必须是映射")
        if "frame_by_frame" not in planning_config:
            raise KeyError("MotionPlanningConfig 缺少 [frame_by_frame]")
        section = planning_config["frame_by_frame"]
        if not isinstance(section, Mapping):
            raise TypeError("MotionPlanningConfig.frame_by_frame 必须是表")

        expected = {
            "x_arrival_confirmations",
            "x_position_tolerance_mm",
            "minimum_gun_y_distance_mm",
            "geometry",
        }
        missing = expected - set(section)
        # 接受旧调用携带的全局窗口字段，但距离仅由设备配置决定。
        extra = set(section) - expected - {
            "short_stroke_x_offset_mm", "short_stroke_threshold_mm",
            "yunque_safety_window_mm", "variable_2d_safety_window_mm",
            "idle_return_distance_mm",  # 旧配置兼容：待料检查整段来料队列，此值不再使用。
        }
        if missing:
            raise KeyError(
                "MotionPlanningConfig.frame_by_frame 缺少参数："
                + ", ".join(sorted(missing))
            )
        if extra:
            raise ValueError(
                "MotionPlanningConfig.frame_by_frame 包含未知参数："
                + ", ".join(sorted(extra))
            )

        geometry_section = section["geometry"]
        if not isinstance(geometry_section, Mapping):
            raise TypeError("MotionPlanningConfig.frame_by_frame.geometry 必须是表")
        geometry_fields = {
            "minimum_x_change_mm",
            "minimum_run_layers",
            "smoothing_window",
            "minimum_middle_coverage",
            "narrow_width_mm",
            "minimum_horizontal_layers",
            "rail_padding_layers",
            "middle_trim_layers",
            "gun_obstacle_half_height_mm",
            "minimum_rail_gap_mm",
            "maximum_missing_y_layers",
        }
        missing_geometry = (
            geometry_fields - {
                "gun_obstacle_half_height_mm", "minimum_rail_gap_mm", "maximum_missing_y_layers",
            } - set(geometry_section)
        )
        legacy_geometry_fields = {"fixture_clearance_layers", "lower_wall_clearance_layers"}
        extra_geometry = set(geometry_section) - geometry_fields - legacy_geometry_fields
        if missing_geometry:
            raise KeyError(
                "MotionPlanningConfig.frame_by_frame.geometry 缺少参数："
                + ", ".join(sorted(missing_geometry))
            )
        if extra_geometry:
            raise ValueError(
                "MotionPlanningConfig.frame_by_frame.geometry 包含未知参数："
                + ", ".join(sorted(extra_geometry))
            )

        x_arrival_confirmations = _require_int(
            "x_arrival_confirmations", section["x_arrival_confirmations"]
        )
        if x_arrival_confirmations <= 0:
            raise ValueError("x_arrival_confirmations 必须大于0")
        x_position_tolerance_mm = _require_int(
            "x_position_tolerance_mm", section["x_position_tolerance_mm"]
        )
        if x_position_tolerance_mm < 0:
            raise ValueError("x_position_tolerance_mm 不能小于0")
        minimum_gun_y_distance_mm = _require_int(
            "minimum_gun_y_distance_mm", section["minimum_gun_y_distance_mm"]
        )
        if minimum_gun_y_distance_mm <= 0:
            raise ValueError("minimum_gun_y_distance_mm 必须大于0")

        short_stroke_x_offset_mm = _require_int(
            "short_stroke_x_offset_mm", section.get("short_stroke_x_offset_mm", 100)
        )
        if short_stroke_x_offset_mm < 0:
            raise ValueError("short_stroke_x_offset_mm 不能小于0")
        short_stroke_threshold_mm = _require_int(
            "short_stroke_threshold_mm", section.get("short_stroke_threshold_mm", 40)
        )
        if short_stroke_threshold_mm < 0:
            raise ValueError("short_stroke_threshold_mm 不能小于0")

        return cls(
            x_arrival_confirmations=x_arrival_confirmations,
            x_position_tolerance_mm=x_position_tolerance_mm,
            minimum_gun_y_distance_mm=minimum_gun_y_distance_mm,
            short_stroke_x_offset_mm=short_stroke_x_offset_mm,
            short_stroke_threshold_mm=short_stroke_threshold_mm,
            geometry=FrameGeometryPlanningParameters(**{
                key: value for key, value in geometry_section.items()
                if key not in legacy_geometry_fields
            }),
        )
