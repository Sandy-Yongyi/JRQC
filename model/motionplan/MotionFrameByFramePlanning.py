from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
import math
from typing import Sequence

import numpy as np

from model.utils.MachineSafetyWindow import machine_window_bounds, machine_window_distances
from model.motionplan.MachineAxisMap import get_axis_map, get_x_axes
from model.plc.MovingFrameData import AxisData as MovingAxisData
from model.plc.MovingFrameData import SendMovingFrameData


@dataclass(frozen=True)
class XAxisMotionCommand:
    """反馈状态机生成的单根X轴命令。"""

    Pos: int
    Speed: int
    Status: int
    Phase: str


DIRECTION_TO_END = "to_end"
DIRECTION_TO_START = "to_start"
PHASE_POSITIONING_TO_START = "positioning_to_start"
PHASE_TO_END = DIRECTION_TO_END
PHASE_TO_START = DIRECTION_TO_START
PHASE_RECOVERING = "recovering"
PHASE_RETURNING_INSIDE = "returning_inside"
PHASE_HOLD = "hold"
PHASE_POSITIONING_TO_FIXED = "positioning_to_fixed"
PHASE_FIXED = "fixed"


def _require_int(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} 必须是整数")
    return value


@dataclass(frozen=True)
class FeedbackXAxisTarget:
    """当前轮廓给单根X轴提供的动态端点、速度和机械限位。"""

    start_pos: int
    end_pos: int
    position_speed: int
    reciprocation_speed: int
    min_limit_pos: int
    max_limit_pos: int

    def __post_init__(self) -> None:
        values = {
            name: _require_int(name, getattr(self, name))
            for name in (
                "start_pos",
                "end_pos",
                "position_speed",
                "reciprocation_speed",
                "min_limit_pos",
                "max_limit_pos",
            )
        }
        if values["min_limit_pos"] > values["max_limit_pos"]:
            raise ValueError("X轴最小限位不能大于最大限位")
        for endpoint_name in ("start_pos", "end_pos"):
            if not (
                values["min_limit_pos"]
                <= values[endpoint_name]
                <= values["max_limit_pos"]
            ):
                raise ValueError(f"{endpoint_name} 超出X轴限位")
        if values["start_pos"] == values["end_pos"]:
            raise ValueError("起点和终点不能相同")
        if values["position_speed"] <= 0:
            raise ValueError("position_speed 必须大于0")
        if values["reciprocation_speed"] <= 0:
            raise ValueError("reciprocation_speed 必须大于0")


@dataclass(frozen=True)
class FixedXAxisTarget:
    """短行程轴的固定喷涂位置及机械约束。"""

    position: int
    position_speed: int
    min_limit_pos: int
    max_limit_pos: int

    def __post_init__(self) -> None:
        values = {
            name: _require_int(name, getattr(self, name))
            for name in (
                "position",
                "position_speed",
                "min_limit_pos",
                "max_limit_pos",
            )
        }
        if values["min_limit_pos"] > values["max_limit_pos"]:
            raise ValueError("X轴最小限位不能大于最大限位")
        if not values["min_limit_pos"] <= values["position"] <= values["max_limit_pos"]:
            raise ValueError("固定点超出X轴限位")
        if values["position_speed"] <= 0:
            raise ValueError("position_speed 必须大于0")


@dataclass(frozen=True)
class FeedbackXAxisSnapshot:
    actual_pos: int
    target_pos: int
    direction: str
    phase: str
    completed_traversals: int
    arrival_confirm_count: int
    held: bool
    hold_reason: str | None
    recovering: bool
    status: int = 0
    fixed_target_pos: int | None = None


class FeedbackXAxisMotion:
    """根据PLC实际位置确认端点并连续换向的单轴状态机。"""

    def __init__(self, arrival_confirmations: int) -> None:
        arrival_confirmations = _require_int(
            "arrival_confirmations", arrival_confirmations
        )
        if arrival_confirmations <= 0:
            raise ValueError("arrival_confirmations 必须大于0")
        self.arrival_confirmations = arrival_confirmations
        self.initialized = False
        self.direction = DIRECTION_TO_END
        self.phase = PHASE_POSITIONING_TO_START
        self.completed_traversals = 0
        self.arrival_confirm_count = 0
        self._arrival_target: int | None = None
        self._recovery_target: int | None = None
        self._recovery_spray = False
        self._target_mode: str | None = None
        self._fixed_target: int | None = None

    def _reset_arrival_confirmation(self) -> None:
        self.arrival_confirm_count = 0
        self._arrival_target = None

    def _observe_arrival(
        self,
        actual_pos: int,
        target_pos: int,
        tolerance: int,
    ) -> bool:
        if (
            self._arrival_target is None
            or abs(target_pos - self._arrival_target) > tolerance
        ):
            self.arrival_confirm_count = 0
        self._arrival_target = target_pos
        self.arrival_confirm_count = (
            self.arrival_confirm_count + 1
            if abs(actual_pos - target_pos) <= tolerance
            else 0
        )
        if self.arrival_confirm_count < self.arrival_confirmations:
            return False
        self._reset_arrival_confirmation()
        return True

    @staticmethod
    def _nearest_endpoint(target: FeedbackXAxisTarget, actual_pos: int) -> int:
        if abs(actual_pos - target.start_pos) <= abs(actual_pos - target.end_pos):
            return target.start_pos
        return target.end_pos

    def _snapshot(
        self,
        *,
        actual_pos: int,
        target_pos: int,
        held: bool = False,
        hold_reason: str | None = None,
    ) -> FeedbackXAxisSnapshot:
        return FeedbackXAxisSnapshot(
            actual_pos=actual_pos,
            target_pos=target_pos,
            direction=self.direction,
            phase=PHASE_HOLD if held else self.phase,
            completed_traversals=self.completed_traversals,
            arrival_confirm_count=self.arrival_confirm_count,
            held=held,
            hold_reason=hold_reason,
            status=int(not held and (
                self.phase in (PHASE_TO_END, PHASE_TO_START, PHASE_FIXED,
                               PHASE_POSITIONING_TO_FIXED, PHASE_RETURNING_INSIDE)
                or (self.phase == PHASE_RECOVERING and self._recovery_spray)
            )),
            recovering=self.phase in (PHASE_RECOVERING, PHASE_RETURNING_INSIDE),
            fixed_target_pos=(
                self._fixed_target if self._target_mode == PHASE_FIXED else None
            ),
        )

    def _advance_fixed(
        self,
        target: FixedXAxisTarget,
        actual_pos: int,
        tolerance: int,
    ) -> tuple[XAxisMotionCommand, FeedbackXAxisSnapshot]:
        target_changed = (
            self._fixed_target is None
            or abs(target.position - self._fixed_target) > tolerance
        )
        if not self.initialized:
            self.initialized = True
            target_changed = True
        if self._target_mode != PHASE_FIXED or target_changed:
            self._target_mode = PHASE_FIXED
            self._fixed_target = target.position
            self._recovery_target = None
            self._recovery_spray = False
            self.phase = PHASE_POSITIONING_TO_FIXED
            self._reset_arrival_confirmation()
        else:
            self._fixed_target = target.position

        if self.phase == PHASE_FIXED and abs(actual_pos - target.position) > tolerance:
            self.phase = PHASE_POSITIONING_TO_FIXED
            self._reset_arrival_confirmation()

        if self.phase == PHASE_POSITIONING_TO_FIXED:
            if not self._observe_arrival(actual_pos, target.position, tolerance):
                command = XAxisMotionCommand(
                    target.position,
                    target.position_speed,
                    1,
                    PHASE_POSITIONING_TO_FIXED,
                )
                return command, self._snapshot(
                    actual_pos=actual_pos,
                    target_pos=target.position,
                )
            self.phase = PHASE_FIXED

        command = XAxisMotionCommand(target.position, 0, 1, PHASE_FIXED)
        return command, self._snapshot(
            actual_pos=actual_pos,
            target_pos=target.position,
        )

    def advance(
        self,
        target: FeedbackXAxisTarget | FixedXAxisTarget | None,
        actual_pos: int,
        *,
        position_tolerance_mm: int,
        hold_reason: str | None = None,
    ) -> tuple[XAxisMotionCommand, FeedbackXAxisSnapshot]:
        actual_pos = _require_int("actual_pos", actual_pos)
        tolerance = _require_int("position_tolerance_mm", position_tolerance_mm)
        if tolerance < 0:
            raise ValueError("position_tolerance_mm 不能小于0")
        if hold_reason is not None and not isinstance(hold_reason, str):
            raise TypeError("hold_reason 必须是字符串或None")

        if target is None or hold_reason is not None:
            self._reset_arrival_confirmation()
            reason = hold_reason or "当前帧没有有效X轴边界"
            command = XAxisMotionCommand(actual_pos, 0, 0, PHASE_HOLD)
            return command, self._snapshot(
                actual_pos=actual_pos,
                target_pos=actual_pos,
                held=True,
                hold_reason=reason,
            )

        if isinstance(target, FixedXAxisTarget):
            return self._advance_fixed(target, actual_pos, tolerance)

        if not self.initialized:
            self.initialized = True
            self.direction = DIRECTION_TO_END
            self.phase = PHASE_POSITIONING_TO_START
            self._reset_arrival_confirmation()
        elif self._target_mode == PHASE_FIXED:
            self._recovery_target = self._nearest_endpoint(target, actual_pos)
            # 固定点切回往复时保持喷涂；首次往复定位仍关枪。
            self._recovery_spray = True
            self.phase = PHASE_RECOVERING
            self._reset_arrival_confirmation()
        self._target_mode = "reciprocating"
        self._fixed_target = None

        lower = min(target.start_pos, target.end_pos)
        upper = max(target.start_pos, target.end_pos)
        outside = actual_pos < lower - tolerance or actual_pos > upper + tolerance
        if self.phase in (PHASE_TO_START, PHASE_TO_END, PHASE_RETURNING_INSIDE):
            if outside or self.phase == PHASE_RETURNING_INSIDE:
                if not lower <= actual_pos <= upper:
                    # 已在往复中的轴越界后，直接穿过近端点向区间内回程。
                    # 连续收缩时不追赶近端点；整个区间来自上游安全包络和
                    # 障碍约束。新边界移到轴的另一侧时重新选择向内方向。
                    destination = upper if actual_pos < lower else lower
                    self.direction = (
                        DIRECTION_TO_START
                        if destination == target.start_pos else DIRECTION_TO_END
                    )
                    self.phase = PHASE_RETURNING_INSIDE
                    self._reset_arrival_confirmation()
                    command = XAxisMotionCommand(
                        destination, target.position_speed, 1, self.phase
                    )
                    return command, self._snapshot(
                        actual_pos=actual_pos, target_pos=destination
                    )
                # 反馈真正进入安全区后沿原方向继续，不在近端点定位确认。
                self.phase = self.direction
                self._reset_arrival_confirmation()
        elif outside:
            # 首次定位和固定喷涂切回往复仍使用最近端点定位。
            self._recovery_target = self._nearest_endpoint(target, actual_pos)
            self.phase = PHASE_RECOVERING
        if self.phase == PHASE_RECOVERING:
            recovery_target = self._recovery_target
            if recovery_target not in (target.start_pos, target.end_pos):
                recovery_target = self._nearest_endpoint(target, actual_pos)
                self._recovery_target = recovery_target
            if not self._observe_arrival(actual_pos, recovery_target, tolerance):
                command = XAxisMotionCommand(
                    recovery_target, target.position_speed,
                    int(self._recovery_spray), PHASE_RECOVERING
                )
                return command, self._snapshot(
                    actual_pos=actual_pos, target_pos=recovery_target
                )
            self._recovery_target = None
            if recovery_target == target.start_pos:
                self.direction = DIRECTION_TO_END
                self.phase = PHASE_TO_END
            else:
                self.direction = DIRECTION_TO_START
                self.phase = PHASE_TO_START

        if self.phase == PHASE_POSITIONING_TO_START:
            if not self._observe_arrival(actual_pos, target.start_pos, tolerance):
                command = XAxisMotionCommand(
                    target.start_pos,
                    target.position_speed,
                    0,
                    PHASE_POSITIONING_TO_START,
                )
                return command, self._snapshot(
                    actual_pos=actual_pos, target_pos=target.start_pos
                )
            self.direction = DIRECTION_TO_END
            self.phase = PHASE_TO_END

        target_pos = (
            target.end_pos if self.direction == DIRECTION_TO_END else target.start_pos
        )
        if self._observe_arrival(actual_pos, target_pos, tolerance):
            self.completed_traversals += 1
            self.direction = (
                DIRECTION_TO_START
                if self.direction == DIRECTION_TO_END
                else DIRECTION_TO_END
            )
            self.phase = (
                PHASE_TO_START if self.direction == DIRECTION_TO_START else PHASE_TO_END
            )
            target_pos = (
                target.end_pos
                if self.direction == DIRECTION_TO_END
                else target.start_pos
            )
        command = XAxisMotionCommand(
            target_pos, target.reciprocation_speed, 1, self.phase
        )
        return command, self._snapshot(actual_pos=actual_pos, target_pos=target_pos)


@dataclass(frozen=True)
class XnSideFeedbackXStep:
    commands: dict[str, XAxisMotionCommand]
    snapshots: dict[str, FeedbackXAxisSnapshot]


class XnSideFeedbackXPlanner:
    """同时维护一侧四根X轴的实际位置反馈状态。"""

    def __init__(self, arrival_confirmations: int) -> None:
        self.states = {
            axis_name: FeedbackXAxisMotion(arrival_confirmations)
            for axis_name in ("x1", "x2", "x3", "x4")
        }

    def advance(
        self,
        targets: Mapping[
            str,
            FeedbackXAxisTarget | FixedXAxisTarget | None,
        ],
        actual_positions: Mapping[str, int],
        *,
        position_tolerance_mm: int,
        hold_reasons: Mapping[str, str | None] | None = None,
    ) -> XnSideFeedbackXStep:
        expected_axes = {"x1", "x2", "x3", "x4"}
        if not isinstance(targets, Mapping) or set(targets) != expected_axes:
            raise ValueError("targets 必须且只能包含 x1、x2、x3、x4")
        if (
            not isinstance(actual_positions, Mapping)
            or set(actual_positions) != expected_axes
        ):
            raise ValueError("actual_positions 必须且只能包含 x1、x2、x3、x4")
        if hold_reasons is not None and not isinstance(hold_reasons, Mapping):
            raise TypeError("hold_reasons 必须是映射或None")
        if set(hold_reasons or ()) - expected_axes:
            raise ValueError("hold_reasons 包含未知X轴")

        for axis_name in expected_axes:
            target = targets[axis_name]
            if target is not None and not isinstance(
                target,
                (FeedbackXAxisTarget, FixedXAxisTarget),
            ):
                raise TypeError(f"{axis_name}目标必须是往复目标、固定点目标或None")
            _require_int(f"{axis_name}.actual_pos", actual_positions[axis_name])
            reason = (hold_reasons or {}).get(axis_name)
            if reason is not None and not isinstance(reason, str):
                raise TypeError(f"{axis_name}保持原因必须是字符串或None")
            if not self.states[axis_name].initialized and (
                target is None or reason is not None
            ):
                raise RuntimeError("首次规划必须提供四根X轴的有效边界")

        commands = {}
        snapshots = {}
        for axis_name in ("x1", "x2", "x3", "x4"):
            command, snapshot = self.states[axis_name].advance(
                targets[axis_name],
                actual_positions[axis_name],
                position_tolerance_mm=position_tolerance_mm,
                hold_reason=(hold_reasons or {}).get(axis_name),
            )
            commands[axis_name] = command
            snapshots[axis_name] = snapshot
        return XnSideFeedbackXStep(commands=commands, snapshots=snapshots)


# SendMovingFrameData.Enable
ENABLE_MASTER = 0x0001
ENABLE_LEFT_XN_SIDE = 0x0002
ENABLE_RIGHT_XN_SIDE = 0x0004
ENABLE_CLEAN_MODE = 0x8000

# PLC -> PC ReceiveMovingFrameData.Operate。该字段在现有协议
# 中以 Int16 解码，使用前必须归一化为无符号 16 位。
PLC_OPERATE_LEFT_XN_SIDE = 0x0002
PLC_OPERATE_RIGHT_XN_SIDE = 0x0004
PLC_OPERATE_CLEAN_MODE = 0x8000

# SendMovingFrameData.Operate
OPERATE_RESERVED = 0x0001
OPERATE_CHAIN_RUN = 0x0002

# HeartBeat 原样回传 PLC 的 Int16 心跳，包含有符号负值。
HEARTBEAT_MIN = -0x8000
HEARTBEAT_MAX = 0x7FFF

# PLC负责人确认：ChainCountCM 每增加1，链条实际前进10mm。
CHAIN_COUNT_CM_STEP_MM = 10

# 业务负责人确认：左右两侧云雀都把逐层轮廓的最小X值视为外前表面。
OUT_FRONT_SURFACE_FIELD_BY_DIRECTION = {
    "left": "V_Axis_Min",
    "right": "V_Axis_Min",
}

# 四组云雀喷枪按物理编号从下到上对应的车架表面。最高枪4高度固定，
# 所以没有y4；枪1、枪2、枪3分别由y1、y2、y3控制。
XN_SIDE_GUN_SURFACE_SEQUENCE = (
    "out_down",
    "in_down",
    "in_up",
    "out_up",
)

# 单台侧面云雀写入PLC前的固定轴顺序。左侧最终映射到 AxisList[0:7]，
# 右侧最终映射到 AxisList[7:14]；本模块暂时只生成按名称索引的离线指令。
XN_SIDE_AXIS_SEQUENCE = (
    "y1",
    "x1",
    "y2",
    "x2",
    "y3",
    "x3",
    "x4",
)
XN_SIDE_X_AXES = ("x1", "x2", "x3", "x4")
XN_SIDE_Y_AXES = ("y1", "y2", "y3")

# X轴前/后端点使用的设备偏移。正式规划完整保存在本模块内，避免
# PlcCommunicationProcess 的运行路径依赖离线工具或其他新增模块。
X_AXIS_OFFSET_KEYS = {
    "x1": ("out_front_x_offset", "out_after_x_offset"),
    "x2": ("in_front_x_offset", "in_after_x_offset"),
    "x3": ("in_front_x_offset", "in_after_x_offset"),
    "x4": ("out_front_x_offset", "out_after_x_offset"),
}

# 业务负责人确认的Y特征点偏移方向。这里修正的是激光坐标中的
# 喷枪到达点；随后再换算成y1/y2/y3相对各自零点的轴位置。
XN_SIDE_SURFACE_Y_OFFSET_RULES = {
    "out_up": ("out_up_y_offset", 1),
    "in_up": ("in_up_y_offset", -1),
    "in_down": ("in_down_y_offset", 1),
    "out_down": ("out_down_y_offset", -1),
}


@dataclass(frozen=True)
class RuntimeQueueGeometry:
    """参考帧矩形（底面Y取安全窗口最小值）及安全包络后的四根X轴行程。"""

    boxes: object | None
    motions: Mapping | None
    leading_edge: bool = False
    rows_by_frame: tuple["ContourRows", ...] = ()
    y_targets: Mapping[str, int] | None = None
    obstacle_constraints: tuple["GunObstacleConstraint", ...] = ()
    target_reasons: tuple[str, ...] = ()
    targets_prepared: bool = False
    trailing_edge: bool = False
    y_targets_valid: bool = True
    arrival_phase: str = "at_station"
    station_index: int | None = None
    reference_index: int | None = None


@dataclass(frozen=True)
class FrameGeometryPlanningParameters:
    """逐帧轮廓识别参数；旧配置缺少喷枪障碍检查半高度时默认使用30mm。"""

    minimum_x_change_mm: float
    minimum_run_layers: int
    smoothing_window: int
    minimum_middle_coverage: float
    narrow_width_mm: float
    minimum_horizontal_layers: int
    rail_padding_layers: int
    middle_trim_layers: int
    gun_obstacle_half_height_mm: int = 30

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
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} 必须是整数")
            if value < 0:
                raise ValueError(f"{name} 不能小于0")


@dataclass(frozen=True)
class FrameByFramePlanningParameters:
    """逐帧运动规划的启动配置。"""

    idle_return_distance_mm: int
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
            "idle_return_distance_mm",
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
        }
        missing_geometry = (
            geometry_fields - {"gun_obstacle_half_height_mm"} - set(geometry_section)
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

        idle_return_distance_mm = _require_int(
            "idle_return_distance_mm", section["idle_return_distance_mm"]
        )
        if (
            idle_return_distance_mm <= 0
            or idle_return_distance_mm % CHAIN_COUNT_CM_STEP_MM != 0
        ):
            raise ValueError("idle_return_distance_mm 必须是10mm的正整数倍")
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
            idle_return_distance_mm=idle_return_distance_mm,
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


@dataclass(frozen=True)
class XAxisPlanningDiagnostic:
    axis_name: str
    detected_start_x: int | None
    detected_end_x: int | None
    safe_start_pos: int | None
    safe_end_pos: int | None
    actual_pos: int
    command_pos: int | None
    command_speed: int | None
    phase: str | None
    held: bool
    hold_reason: str | None
    fixed_target_pos: int | None = None
    obstacle_constraint: "GunObstacleConstraint | None" = None


@dataclass(frozen=True)
class GunObstacleConstraint:
    axis_name: str
    target_height: int
    height_min: int
    height_max: int
    window_min_x: tuple[int | None, ...]
    center_min_x: int | None
    obstacle_min_x: int | None
    limit_pos: int | None
    laser_x_offset: int
    original_start_pos: int | None = None
    original_end_pos: int | None = None
    final_start_pos: int | None = None
    final_end_pos: int | None = None
    constrained: bool = False
    collapsed: bool = False

    @property
    def window_detected(self) -> tuple[bool, ...]:
        return tuple(value is not None for value in self.window_min_x)


@dataclass(frozen=True)
class XnSidePlanningDiagnostic:
    side: str
    boxes: object | None
    axes: tuple[XAxisPlanningDiagnostic, ...]
    y_targets: tuple[tuple[str, int], ...]
    active: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class Variable2DPlanningDiagnostic:
    side: str
    state: str
    x_pos: int | None
    y_pos_min: int | None
    y_pos_max: int | None
    active: bool


@dataclass(frozen=True)
class MotionPlanningDebugSnapshot:
    """当前周期的只读诊断副本，不暴露规划器内部可变状态。"""

    heartbeat: int
    fifo: int | None
    safe: bool
    enable: int
    operate: int
    xn_sides: tuple[XnSidePlanningDiagnostic, ...]
    out_lifts: tuple[Variable2DPlanningDiagnostic, ...]
    active_sides: tuple[str, ...]
    active_2d_sides: tuple[str, ...]
    reasons: tuple[str, ...]
    error: str | None


VARIABLE_2D_WINDOW_EMPTY = "empty"
VARIABLE_2D_WINDOW_MIXED = "mixed"
VARIABLE_2D_WINDOW_VALID = "valid"


@dataclass(frozen=True)
class Variable2DWindowObservation:
    """一台2D变频设备在本次安全窗口中的边界结果。"""

    state: str
    command: MovingAxisData | None = None
    leading_edge: bool = False


class Variable2DReportStateMachine:
    """维护2D上下限报文，避免窗口空洞打断PLC端Y往复方向。

    线协议只发送 ``XPos/YPosMin/YPosMax``，没有Y当前位置或方向字段；
    到达上下端点后的换向由PLC状态机负责。上位机在窗口夹有空帧时
    必须持续发送上一组有效边界，不能插入零报文导致PLC端突然换向。
    """

    def __init__(self) -> None:
        self.last_command: MovingAxisData | None = None
        self.phase = VARIABLE_2D_WINDOW_EMPTY

    @staticmethod
    def _copy(command: MovingAxisData) -> MovingAxisData:
        return MovingAxisData(
            Pos=command.Pos,
            Speed=command.Speed,
            Status=command.Status,
        )

    def reset(self) -> None:
        self.last_command = None
        self.phase = VARIABLE_2D_WINDOW_EMPTY

    def advance(
        self,
        observation: Variable2DWindowObservation,
    ) -> MovingAxisData | None:
        if not isinstance(observation, Variable2DWindowObservation):
            raise TypeError("observation 必须是 Variable2DWindowObservation")
        if observation.state == VARIABLE_2D_WINDOW_EMPTY:
            self.reset()
            return None
        if observation.state == VARIABLE_2D_WINDOW_MIXED:
            self.phase = VARIABLE_2D_WINDOW_MIXED
            return None if self.last_command is None else self._copy(self.last_command)
        if observation.state != VARIABLE_2D_WINDOW_VALID:
            raise ValueError("未知2D安全窗口状态")
        if not isinstance(observation.command, MovingAxisData):
            raise TypeError("有效2D窗口必须包含PLC AxisData")

        self.last_command = self._copy(observation.command)
        self.phase = VARIABLE_2D_WINDOW_VALID
        return self._copy(self.last_command)


def _round_runtime_mm(value: float) -> int:
    """按常规四舍五入转换为PLC整数毫米。"""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("轮廓坐标必须是数值")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("轮廓坐标必须是有限数")
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def _freeze_runtime_value(value):
    """把运行时配置转成可比较签名，用于自动失效几何缓存。"""

    if isinstance(value, Mapping):
        return tuple(
            sorted(
                (str(key), _freeze_runtime_value(item)) for key, item in value.items()
            )
        )
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_runtime_value(item) for item in value)
    if isinstance(value, (str, int, float, bool, type(None))):
        return value
    return repr(value)


def apply_xn_side_surface_y_offsets(
    surface_y: Mapping[str, int],
    machine_config: Mapping,
) -> dict[str, int]:
    """按MachineConfig修正四个面的激光Y到达点。

    规则为：上外顶加、上内顶减、下内底加、下外底减。调用方可以
    只传需要的面，但每个传入面都必须有对应配置。
    """

    if not isinstance(surface_y, Mapping):
        raise TypeError("surface_y 必须是映射")
    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if not surface_y:
        raise ValueError("surface_y 不能为空")

    adjusted: dict[str, int] = {}
    for surface, detected_y in surface_y.items():
        if surface not in XN_SIDE_SURFACE_Y_OFFSET_RULES:
            raise ValueError(f"未知云雀表面：{surface}")
        if isinstance(detected_y, bool) or not isinstance(detected_y, int):
            raise TypeError(f"{surface}检测Y必须是整数毫米值")
        if detected_y <= 0:
            raise ValueError(f"{surface}检测Y必须大于0")

        offset_key, sign = XN_SIDE_SURFACE_Y_OFFSET_RULES[surface]
        if offset_key not in machine_config:
            raise KeyError(f"MachineConfig 缺少参数：{offset_key}")
        offset = machine_config[offset_key]
        if isinstance(offset, bool) or not isinstance(offset, int):
            raise TypeError(f"MachineConfig.{offset_key} 必须是整数")
        if offset < 0:
            raise ValueError(f"MachineConfig.{offset_key} 不能小于0")

        target_y = detected_y + sign * offset
        if target_y <= 0:
            raise ValueError(f"{surface}应用Y偏移后的到达点必须大于0")
        adjusted[surface] = target_y
    return adjusted


def calculate_fixed_gun_sample_y(
    combined_y_max: int,
    y_threshold: int,
) -> int:
    """返回扫描配置半开区间中的最后一个Y层（旧离线辅助接口）。

    逐帧分层使用半开区间，所以2600mm边界对应最后一个10mm数据层
    2590。本函数不参与正式Y轴规划；枪4实际零点始终读取
    ``MachineConfig.origin_pos[3]``。
    """

    values = {
        "combined_y_max": combined_y_max,
        "y_threshold": y_threshold,
    }
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} 必须是整数毫米值")
    if combined_y_max <= 0:
        raise ValueError("combined_y_max 必须大于0")
    if y_threshold <= 0:
        raise ValueError("y_threshold 必须大于0")
    if combined_y_max < y_threshold:
        raise ValueError("combined_y_max 不能小于 y_threshold")
    return combined_y_max - y_threshold


def calculate_history_index(
    z_position_mm: int,
    frame_step_mm: int,
    stack_size: int,
) -> int | None:
    """计算设备应读取的历史Z帧下标。

    ``frame_stack[0]`` 是激光刚采集的最新帧，下标越大表示越早采集、
    已沿链条正方向移动得越远。函数不猜测队列单位，调用方必须明确传入
    每个队列槽对应的毫米数。

    距离超出队列覆盖范围时返回 ``None``，让上层进入关枪安全状态。
    当前阶段不擅自决定非整格距离的取整方向，因此要求距离与步长整除。
    """

    values = {
        "z_position_mm": z_position_mm,
        "frame_step_mm": frame_step_mm,
        "stack_size": stack_size,
    }
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} 必须是整数")

    if z_position_mm < 0:
        raise ValueError("z_position_mm 不能小于0")
    if frame_step_mm <= 0:
        raise ValueError("frame_step_mm 必须大于0")
    if stack_size <= 0:
        raise ValueError("stack_size 必须大于0")
    if z_position_mm % frame_step_mm != 0:
        raise ValueError("z_position_mm 必须能被 frame_step_mm 整除，取整规则尚未确认")

    history_index = z_position_mm // frame_step_mm
    if history_index >= stack_size:
        return None
    return history_index


def select_history_frame(
    frame_stack,
    z_position_mm: int,
    frame_step_mm: int,
):
    """按设备Z距离读取历史帧；队列覆盖不到时返回 ``None``。"""

    if frame_stack is None:
        raise TypeError("frame_stack 不能是None")

    history_index = calculate_history_index(
        z_position_mm=z_position_mm,
        frame_step_mm=frame_step_mm,
        stack_size=len(frame_stack),
    )
    if history_index is None:
        return None
    return frame_stack[history_index]


def select_out_front_detected_x(axis_data, direction: str) -> int:
    """从一个Y层轮廓中取得外前表面的雷达X检测值。

    当前业务规则明确规定：左、右两侧都读取 ``V_Axis_Min``。
    空层使用0表示，因此0和负数都不能进入后续运动计算。
    """

    if direction not in OUT_FRONT_SURFACE_FIELD_BY_DIRECTION:
        raise ValueError("direction 必须是 left 或 right")
    if axis_data is None:
        raise TypeError("axis_data 不能是None")

    field_name = OUT_FRONT_SURFACE_FIELD_BY_DIRECTION[direction]
    if not hasattr(axis_data, field_name):
        raise TypeError(f"axis_data 缺少字段 {field_name}")

    detected_x = getattr(axis_data, field_name)
    if isinstance(detected_x, bool) or not isinstance(detected_x, int):
        raise TypeError(f"{field_name} 必须是整数毫米值")
    if detected_x <= 0:
        raise ValueError(f"{field_name} 必须大于0，当前层可能是空层")
    return detected_x


def validate_xn_side_gun_origin_y(
    origin_pos: Sequence[int],
    minimum_gun_y_distance_mm: int,
) -> tuple[int, int, int, int]:

    minimum_gun_y_distance_mm = _require_int(
        "minimum_gun_y_distance_mm", minimum_gun_y_distance_mm
    )
    if minimum_gun_y_distance_mm <= 0:
        raise ValueError("minimum_gun_y_distance_mm 必须大于0")
    if not isinstance(origin_pos, (list, tuple)):
        raise TypeError("MachineConfig.origin_pos 必须是枪1至枪4的四元素列表")
    if len(origin_pos) != 4:
        raise ValueError("MachineConfig.origin_pos 必须依次包含枪1至枪4的四个Y零点")

    origins = []
    for gun_number, value in enumerate(origin_pos, start=1):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(
                f"MachineConfig.origin_pos[{gun_number - 1}] 必须是整数毫米值"
            )
        if value < 0:
            raise ValueError(f"枪{gun_number}的Y零点不能小于0")
        origins.append(value)

    for lower_index in range(3):
        distance = origins[lower_index + 1] - origins[lower_index]
        if distance < minimum_gun_y_distance_mm:
            raise ValueError(
                f"枪{lower_index + 1}与枪{lower_index + 2}的Y零点距离"
                f"{distance}mm小于{minimum_gun_y_distance_mm}mm"
            )
    return tuple(origins)


def limit_xn_side_y_targets_for_clearance(
    y_targets: Mapping[str, int],
    origin_pos: Sequence[int],
    minimum_gun_y_distance_mm: int,
) -> dict[str, int]:
    """
    PLC位置是喷枪相对自身零点向上的位移，因此喷枪绝对激光坐标为
    ``origin_pos[i] + Pos``。发生冲突时只减小较低喷枪的Pos。
    """

    if not isinstance(y_targets, Mapping):
        raise TypeError("y_targets 必须是映射")
    if set(y_targets) != set(XN_SIDE_Y_AXES):
        raise ValueError("y_targets 必须且只能包含 y1、y2、y3")
    origins = validate_xn_side_gun_origin_y(origin_pos, minimum_gun_y_distance_mm)

    safe_positions = []
    for axis_name in XN_SIDE_Y_AXES:
        value = y_targets[axis_name]
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{axis_name}目标位置必须是整数毫米值")
        if value < 0:
            raise ValueError(f"{axis_name}目标位置不能小于0")
        safe_positions.append(value)

    # 枪4没有Y轴，固定在origin_pos[3]，所以对应的相对位移为0。
    upper_relative_position = 0
    for lower_index in range(2, -1, -1):
        maximum_lower_position = (
            origins[lower_index + 1]
            + upper_relative_position
            - minimum_gun_y_distance_mm
            - origins[lower_index]
        )
        safe_positions[lower_index] = min(
            safe_positions[lower_index],
            maximum_lower_position,
        )
        upper_relative_position = safe_positions[lower_index]

    return {
        axis_name: safe_positions[index]
        for index, axis_name in enumerate(XN_SIDE_Y_AXES)
    }


def calculate_xn_side_y_targets(
    in_up_y: int,
    in_down_y: int,
    out_down_y: int,
    origin_pos: Sequence[int],
    machine_config: Mapping,
    minimum_gun_y_distance_mm: int,
) -> dict[str, int]:
    """把三条可移动边的激光Y坐标换算成y1/y2/y3相对零点偏移。

    枪4负责上外顶且高度固定。枪1、枪2、枪3分别负责下外底、下内底、
    上内顶；先按MachineConfig应用“上内减、下内加、下外减”的Y到达
    点偏移，再减去每把枪各自的安装零点。Y轴正位置表示喷枪从自身
    零点向上移动。最后从枪4向枪1执行防撞限幅。
    """

    adjusted_y = apply_xn_side_surface_y_offsets(
        {
            "in_up": in_up_y,
            "in_down": in_down_y,
            "out_down": out_down_y,
        },
        machine_config,
    )
    if not adjusted_y["in_up"] > adjusted_y["in_down"] > adjusted_y["out_down"]:
        raise ValueError("应用Y偏移后必须保持上内顶 > 下内底 > 下外底")

    gun1_zero_y, gun2_zero_y, gun3_zero_y, _ = validate_xn_side_gun_origin_y(
        origin_pos,
        minimum_gun_y_distance_mm,
    )
    targets = {
        "y1": adjusted_y["out_down"] - gun1_zero_y,
        "y2": adjusted_y["in_down"] - gun2_zero_y,
        "y3": adjusted_y["in_up"] - gun3_zero_y,
    }
    invalid_axes = [axis for axis, position in targets.items() if position < 0]
    if invalid_axes:
        raise ValueError(
            f"Y轴目标位置小于0：{', '.join(invalid_axes)}，请检查轮廓边和喷枪零点"
        )
    return limit_xn_side_y_targets_for_clearance(
        targets,
        origin_pos,
        minimum_gun_y_distance_mm,
    )


def calculate_out_front_target_x(
    detected_x: int,
    x_position: int,
    out_front_x_offset: int,
) -> int:
    """把雷达检测到的工件进深换算成云雀 X 轴目标位置。

    当前项目只喷涂工件外前表面，已确认公式为：
        target_x = detected_x - x_position - out_front_x_offset

    此函数只处理几何关系；轴限位检查将在生成 PLC 运动指令时进行。
    """
    values = {
        "detected_x": detected_x,
        "x_position": x_position,
        "out_front_x_offset": out_front_x_offset,
    }
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} 必须是整数毫米值")
        if value < 0:
            raise ValueError(f"{name} 不能小于0")

    target_x = detected_x - x_position - out_front_x_offset
    if target_x < 0:
        raise ValueError(
            "计算得到的云雀X目标位置小于0，请检查雷达检测值、x_position和out_front_x_offset"
        )
    return target_x


def _xn_side_config_axis_value(
    machine_config: Mapping,
    config_key: str,
    axis_name: str,
) -> int:
    """读取xn_side配置中同类轴共享的Y/X数组值。"""

    if config_key not in machine_config:
        raise KeyError(f"MachineConfig 缺少参数：{config_key}")
    values = machine_config[config_key]
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise ValueError(f"MachineConfig.{config_key} 必须是[y, x]两个整数")
    config_index = 0 if axis_name.startswith("y") else 1
    value = values[config_index]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"MachineConfig.{config_key}[{config_index}] 必须是整数")
    return value


def build_xn_side_axis_commands(
    x_commands: Mapping[str, XAxisMotionCommand],
    y_targets: Mapping[str, int],
    machine_config: Mapping,
) -> dict[str, MovingAxisData]:
    """把已规划的四根X轴和三根Y轴合成单侧7轴离线指令。

    这个函数是几何/状态机与PLC槽位之间的安全适配层：它只生成按轴名
    排列的 ``AxisData``，不写 ``AxisList``、不设置使能、不发送PLC。
    X轴 ``Status`` 传递每把枪的开关粉状态，Y轴保持为0。
    """

    if not isinstance(x_commands, Mapping):
        raise TypeError("x_commands 必须是映射")
    if not isinstance(y_targets, Mapping):
        raise TypeError("y_targets 必须是映射")
    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "xn_side":
        raise ValueError("machine_config.type 必须是 xn_side")

    configured_axis_order = machine_config.get("axis_type")
    if tuple(configured_axis_order or ()) != XN_SIDE_AXIS_SEQUENCE:
        raise ValueError("MachineConfig.axis_type 与侧面云雀7轴顺序不一致")

    if set(x_commands) != set(XN_SIDE_X_AXES):
        raise ValueError("x_commands 必须且只能包含 x1、x2、x3、x4")
    if set(y_targets) != set(XN_SIDE_Y_AXES):
        raise ValueError("y_targets 必须且只能包含 y1、y2、y3")

    x_min = _xn_side_config_axis_value(machine_config, "min_limit_pos", "x1")
    x_max = _xn_side_config_axis_value(machine_config, "max_limit_pos", "x1")
    x_speed_max = _xn_side_config_axis_value(machine_config, "max_limit_speed", "x1")
    y_min = _xn_side_config_axis_value(machine_config, "min_limit_pos", "y1")
    y_max = _xn_side_config_axis_value(machine_config, "max_limit_pos", "y1")
    y_speed_max = _xn_side_config_axis_value(machine_config, "max_limit_speed", "y1")
    if x_min > x_max or y_min > y_max:
        raise ValueError("MachineConfig轴最小限位不能大于最大限位")
    if x_speed_max <= 0 or y_speed_max <= 0:
        raise ValueError("MachineConfig轴最大速度必须大于0")

    y_speed = machine_config.get("y_pos_speed")
    if isinstance(y_speed, bool) or not isinstance(y_speed, int):
        raise TypeError("MachineConfig.y_pos_speed 必须是整数")
    if not 0 < y_speed <= y_speed_max:
        raise ValueError("MachineConfig.y_pos_speed 必须大于0且不超过Y轴最大速度")

    axis_commands: dict[str, MovingAxisData] = {}
    for axis_name in XN_SIDE_AXIS_SEQUENCE:
        if axis_name.startswith("x"):
            command = x_commands[axis_name]
            if not all(
                hasattr(command, field_name)
                for field_name in ("Pos", "Speed", "Status")
            ):
                raise TypeError(f"{axis_name} 必须提供 Pos、Speed、Status")
            if _require_int(f"{axis_name}.Status", command.Status) not in (0, 1):
                raise ValueError(f"{axis_name}.Status 必须为0或1")
            if not x_min <= command.Pos <= x_max:
                raise ValueError(
                    f"{axis_name}.Pos={command.Pos} 超出X轴限位 {x_min}~{x_max}"
                )
            if not 0 <= command.Speed <= x_speed_max:
                raise ValueError(
                    f"{axis_name}.Speed={command.Speed} 超出X轴速度范围 0~{x_speed_max}"
                )
            axis_commands[axis_name] = MovingAxisData(
                Pos=command.Pos,
                Speed=command.Speed,
                Status=command.Status,
            )
            continue

        target = y_targets[axis_name]
        if isinstance(target, bool) or not isinstance(target, int):
            raise TypeError(f"{axis_name}目标位置必须是整数")
        if not y_min <= target <= y_max:
            raise ValueError(f"{axis_name}.Pos={target} 超出Y轴限位 {y_min}~{y_max}")
        axis_commands[axis_name] = MovingAxisData(
            Pos=target,
            Speed=y_speed,
            Status=0,
        )
    return axis_commands


def build_xn_side_return_commands(
    machine_config: Mapping,
    *,
    clean_mode: bool,
) -> dict[str, MovingAxisData]:
    """构造单侧云雀的安全回位或清理位指令。

    普通回位时 X/Y 都使用 ``safe_pos``；清理模式时 Y 使用
    ``safe_pos``，四根 X 轴使用 ``max_limit_pos``。
    """

    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "xn_side":
        raise ValueError("machine_config.type 必须是 xn_side")
    if not isinstance(clean_mode, bool):
        raise TypeError("clean_mode 必须是布尔值")

    y_target = _xn_side_config_axis_value(machine_config, "safe_pos", "y1")
    x_config_key = "max_limit_pos" if clean_mode else "safe_pos"
    x_target = _xn_side_config_axis_value(machine_config, x_config_key, "x1")
    x_speed = machine_config.get("x_pos_speed")
    if isinstance(x_speed, bool) or not isinstance(x_speed, int):
        raise TypeError("MachineConfig.x_pos_speed 必须是整数")

    x_commands = {
        axis_name: XAxisMotionCommand(
            Pos=x_target,
            Speed=x_speed,
            Status=0,
            Phase=PHASE_POSITIONING_TO_FIXED,
        )
        for axis_name in XN_SIDE_X_AXES
    }
    y_targets = {axis_name: y_target for axis_name in XN_SIDE_Y_AXES}
    return build_xn_side_axis_commands(x_commands, y_targets, machine_config)


def build_out_lift_return_command(
    machine_config: Mapping,
    *,
    clean_mode: bool,
) -> MovingAxisData:
    """构造2D变频设备的安全命令。

    该设备的独立槽使用 ``Pos=X, Speed=YMin, Status=YMax``
    的复合编码；回位时 YMin/YMax 都等于 Y 安全位。
    """

    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "out_lift":
        raise ValueError("machine_config.type 必须是 out_lift")
    if not isinstance(clean_mode, bool):
        raise TypeError("clean_mode 必须是布尔值")

    y_safe = _out_lift_limit(machine_config, "safe_pos", "y")
    x_key = "max_limit_pos" if clean_mode else "safe_pos"
    x_target = _out_lift_limit(machine_config, x_key, "x")
    y_min = _out_lift_limit(machine_config, "min_limit_pos", "y")
    y_max = _out_lift_limit(machine_config, "max_limit_pos", "y")
    x_min = _out_lift_limit(machine_config, "min_limit_pos", "x")
    x_max = _out_lift_limit(machine_config, "max_limit_pos", "x")
    if not y_min <= y_safe <= y_max:
        raise ValueError("out_lift Y安全位超出轴限位")
    if not x_min <= x_target <= x_max:
        raise ValueError("out_lift X回位目标超出轴限位")
    return MovingAxisData(Pos=x_target, Speed=y_safe, Status=y_safe)


def build_available_xn_side_moving_frame(
    left_commands: Mapping[str, MovingAxisData] | None,
    right_commands: Mapping[str, MovingAxisData] | None,
    heartbeat: int,
    *,
    chain_running: bool = True,
    left_2d_command: MovingAxisData | None = None,
    right_2d_command: MovingAxisData | None = None,
) -> SendMovingFrameData:
    """把左/右云雀及2D变频指令装入固定110字节PLC数据结构。

    左右设备的 ``z_position`` 可以不同，因此两个命令允许独立为 ``None``。
    没到站的一侧保持7个零轴槽并且不设置该侧使能位；总使能和链条运行
    位仍保持，使工件能够继续向后续设备移动。该函数不连接或发送PLC。
    """

    if isinstance(heartbeat, bool) or not isinstance(heartbeat, int):
        raise TypeError("heartbeat 必须是整数")
    if not HEARTBEAT_MIN <= heartbeat <= HEARTBEAT_MAX:
        raise ValueError(f"heartbeat 必须在 {HEARTBEAT_MIN}~{HEARTBEAT_MAX} 范围内")
    if not isinstance(chain_running, bool):
        raise TypeError("chain_running 必须是布尔值")

    side_commands = {
        "left": left_commands,
        "right": right_commands,
    }
    axis_list = [MovingAxisData() for _ in range(14)]
    enable = ENABLE_MASTER
    for side, commands in side_commands.items():
        if commands is None:
            continue
        if not isinstance(commands, Mapping):
            raise TypeError(f"{side}_commands 必须是映射或None")
        if tuple(commands) != XN_SIDE_AXIS_SEQUENCE:
            raise ValueError(f"{side}_commands 必须按侧面云雀7轴固定顺序排列")
        slot_offset = 0 if side == "left" else 7
        enable |= ENABLE_LEFT_XN_SIDE if side == "left" else ENABLE_RIGHT_XN_SIDE
        for local_index, axis_name in enumerate(XN_SIDE_AXIS_SEQUENCE):
            command = commands[axis_name]
            if not isinstance(command, MovingAxisData):
                raise TypeError(f"{side}.{axis_name} 必须是PLC AxisData")
            status = _require_int(f"{side}.{axis_name}.Status", command.Status)
            allowed_statuses = (0, 1) if axis_name.startswith("x") else (0,)
            if status not in allowed_statuses:
                raise ValueError(f"{side}.{axis_name}.Status 必须为{allowed_statuses}")
            axis_list[slot_offset + local_index] = MovingAxisData(
                Pos=command.Pos,
                Speed=command.Speed,
                Status=status,
            )

    variable_2d_commands = {
        "left": left_2d_command,
        "right": right_2d_command,
    }
    for side, command in variable_2d_commands.items():
        if command is not None and not isinstance(command, MovingAxisData):
            raise TypeError(f"{side}_2d_command 必须是PLC AxisData或None")

    return SendMovingFrameData(
        Enable=enable,
        Gun_Cont1=0,
        Gun_Cont2=0,
        HeartBeat=heartbeat,
        Operate=OPERATE_CHAIN_RUN if chain_running else 0,
        AxisList=axis_list,
        Left2DLiftData=(
            MovingAxisData()
            if left_2d_command is None
            else MovingAxisData(
                Pos=left_2d_command.Pos,
                Speed=left_2d_command.Speed,
                Status=left_2d_command.Status,
            )
        ),
        Right2DLiftData=(
            MovingAxisData()
            if right_2d_command is None
            else MovingAxisData(
                Pos=right_2d_command.Pos,
                Speed=right_2d_command.Speed,
                Status=right_2d_command.Status,
            )
        ),
    )


def build_dual_xn_side_moving_frame(
    left_commands: Mapping[str, MovingAxisData],
    right_commands: Mapping[str, MovingAxisData],
    heartbeat: int,
    *,
    chain_running: bool = True,
) -> SendMovingFrameData:
    """兼容原有离线工具：要求左右两侧同时提供7轴指令。"""

    if left_commands is None or right_commands is None:
        raise TypeError("双侧运动帧必须同时提供left_commands和right_commands")
    return build_available_xn_side_moving_frame(
        left_commands,
        right_commands,
        heartbeat,
        chain_running=chain_running,
    )


@dataclass(frozen=True)
class ContourRows:
    """从一张激光截面中提取并按H_Axis排序的有效轮廓层。"""

    height: np.ndarray
    v_min: np.ndarray
    v_max: np.ndarray

    def __len__(self) -> int:
        return int(self.height.size)


@dataclass(frozen=True)
class EdgeCandidate:
    height: float
    detected_x: float


@dataclass(frozen=True)
class FrameEdgeCandidates:
    out_up: EdgeCandidate | None
    in_up: EdgeCandidate | None
    in_down: EdgeCandidate | None
    out_down: EdgeCandidate | None
    change_threshold: float
    detection_mode: str

    @property
    def detected_count(self) -> int:
        return sum(
            edge is not None
            for edge in (self.out_up, self.in_up, self.in_down, self.out_down)
        )

    @property
    def is_complete(self) -> bool:
        return self.detected_count == 4


@dataclass(frozen=True)
class RegionBox:
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class ThreeRegionBoxes:
    upper: RegionBox
    middle: RegionBox | None
    lower: RegionBox


@dataclass(frozen=True)
class DetectedXAxisStroke:
    start_x: float
    end_x: float


@dataclass(frozen=True)
class PlannedXAxisRange:
    axis_name: str
    detected_start_x: int
    detected_end_x: int
    start_pos: int
    end_pos: int
    min_limit_pos: int
    max_limit_pos: int
    position_speed: int
    reciprocation_speed: int
    status: int = 0
    # 安全窗口内该轴轮廓的最小检测X；不受端点偏移和障碍裁剪影响。
    window_min_detected_x: int | None = None

    @property
    def has_out_of_limit_position(self) -> bool:
        return not (
            self.min_limit_pos <= self.start_pos <= self.max_limit_pos
            and self.min_limit_pos <= self.end_pos <= self.max_limit_pos
        )


def _extract_contour_rows(axis_frame: object) -> ContourRows:
    rows = []
    for item in getattr(axis_frame, "FrameData", None) or ():
        try:
            height = float(item.H_Axis)
            v_min = float(item.V_Axis_Min)
            v_max = float(item.V_Axis_Max)
        except (AttributeError, TypeError, ValueError):
            continue
        if height == 0 or (v_min == 0 and v_max == 0):
            continue
        if not np.isfinite((height, v_min, v_max)).all() or v_min > v_max:
            continue
        rows.append((height, v_min, v_max))

    if not rows:
        empty = np.empty(0, dtype=float)
        return ContourRows(empty, empty.copy(), empty.copy())
    values = np.asarray(rows, dtype=float)
    values = values[np.argsort(values[:, 0], kind="stable")]
    return ContourRows(values[:, 0], values[:, 1], values[:, 2])


OUT_LIFT_LIMIT_INDEX = {"y": 0, "x": 1}


def _out_lift_limit(machine_config: Mapping, key: str, axis_name: str) -> int:
    if axis_name not in OUT_LIFT_LIMIT_INDEX:
        raise ValueError("out_lift轴名称必须是y或x")
    values = machine_config.get(key)
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise ValueError(f"out_lift.{key} 必须按[y,x]包含两个轴限位")
    index = OUT_LIFT_LIMIT_INDEX[axis_name]
    value = values[index]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"out_lift.{key}[{index}] 必须是整数")
    return value


def _out_lift_origin_y(machine_config: Mapping) -> int:
    values = machine_config.get("origin_pos")
    if not isinstance(values, (list, tuple)) or len(values) != 1:
        raise ValueError("out_lift.origin_pos 必须只包含一个Y轴原点")
    value = values[0]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("out_lift.origin_pos[0] 必须是整数")
    return value


def _clamp_axis_position(value: int, minimum: int, maximum: int) -> int:
    if minimum > maximum:
        raise ValueError("min_limit_pos 不能大于 max_limit_pos")
    return max(minimum, min(maximum, value))


def _leading_edge_window_indices(
    occupied: Sequence[bool], center_index: int, *, older_at_end: bool,
    allow_trailing_edge: bool = False,
) -> tuple[int, ...] | None:
    """裁掉前缘空白；云雀可额外裁掉尾缘空白，中心和内部必须有效。"""
    if not 0 <= center_index < len(occupied):
        raise ValueError("中心帧超出窗口范围")
    indices = list(range(len(occupied)))
    if not older_at_end:
        indices.reverse()
    while indices and not occupied[indices[-1]]:
        indices.pop()
    if allow_trailing_edge:
        while indices and not occupied[indices[0]]:
            indices.pop(0)
    if center_index not in indices or not all(occupied[i] for i in indices):
        return None
    return tuple(sorted(indices))


def build_variable_2d_window_observation(
    frames: Sequence[object],
    machine_config: Mapping,
    *,
    _row_extractor=None,
    station_index: int | None = None,
) -> Variable2DWindowObservation:
    """把2D设备安全窗口换算成 ``XPos/YPosMin/YPosMax``。

    X与Y最小值由本侧激光坐标换算为设备本地坐标；X减去外前偏移，
    不使用外下、外上偏移，Y最大值固定使用设备最大限位。窗口全空表示
    工件不在窗口内。运行时显式提供station_index，允许来料前缘提前
    定位；仅出料侧有工件或窗口内部有空洞时返回mixed以保持上一范围。
    不传station_index的旧离线调用保留对称窗口、中心到站判定。
    """

    if not isinstance(frames, Sequence) or isinstance(frames, (str, bytes)):
        raise TypeError("frames 必须是帧序列")
    if not frames:
        raise ValueError("2D安全窗口不能为空")
    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "out_lift":
        raise ValueError("machine_config 必须是 out_lift 设备配置")

    extract = _extract_contour_rows if _row_extractor is None else _row_extractor
    rows_by_frame = [extract(frame) for frame in frames]
    nonempty_rows = [rows for rows in rows_by_frame if len(rows) > 0]
    if not nonempty_rows:
        return Variable2DWindowObservation(VARIABLE_2D_WINDOW_EMPTY)
    if station_index is None:
        selected = _leading_edge_window_indices(
            [len(rows) > 0 for rows in rows_by_frame],
            len(frames) // 2, older_at_end=True,
        )
    else:
        if not 0 <= station_index < len(frames):
            raise ValueError("站位索引超出二维安全窗口")
        occupied = [len(rows) > 0 for rows in rows_by_frame]
        reference = station_index if occupied[station_index] else next(
            (i for i in range(station_index - 1, -1, -1) if occupied[i]), None,
        )
        selected = None if reference is None else _leading_edge_window_indices(
            occupied, reference, older_at_end=True, allow_trailing_edge=True,
        )
    if selected is None:
        return Variable2DWindowObservation(VARIABLE_2D_WINDOW_MIXED)

    x_position = machine_config.get("x_position")
    if isinstance(x_position, bool) or not isinstance(x_position, int):
        raise TypeError("out_lift.x_position 必须是整数")
    front_offset = _require_int(
        "out_lift.out_front_x_offset", machine_config.get("out_front_x_offset")
    )

    y_minimum_limit = _out_lift_limit(machine_config, "min_limit_pos", "y")
    y_maximum_limit = _out_lift_limit(machine_config, "max_limit_pos", "y")
    x_minimum_limit = _out_lift_limit(machine_config, "min_limit_pos", "x")
    x_maximum_limit = _out_lift_limit(machine_config, "max_limit_pos", "x")
    origin_y = _out_lift_origin_y(machine_config)

    detected_x_min = min(float(np.min(rows.v_min)) for rows in nonempty_rows)
    detected_y_min = min(float(np.min(rows.height)) for rows in nonempty_rows)

    x_pos = _round_runtime_mm(detected_x_min) - x_position - front_offset
    y_pos_min = _round_runtime_mm(detected_y_min) - origin_y
    y_pos_max = y_maximum_limit

    x_pos = _clamp_axis_position(x_pos, x_minimum_limit, x_maximum_limit)
    y_pos_min = _clamp_axis_position(
        y_pos_min,
        y_minimum_limit,
        y_maximum_limit,
    )
    y_pos_max = _clamp_axis_position(
        y_pos_max,
        y_minimum_limit,
        y_maximum_limit,
    )
    if y_pos_min > y_pos_max:
        raise ValueError("2D设备换算后的YPosMin不能大于YPosMax")

    return Variable2DWindowObservation(
        VARIABLE_2D_WINDOW_VALID,
        MovingAxisData(Pos=x_pos, Speed=y_pos_min, Status=y_pos_max),
        leading_edge=len(selected) < len(frames),
    )


def _split_contiguous_rows(
    rows: ContourRows,
    y_threshold: float,
) -> list[ContourRows]:
    if len(rows) == 0:
        return []
    maximum_gap = max(
        float(y_threshold) * 1.5, 1.0
    )  # 按 Y 层间距切分轮廓；间距超过 1.5 × y_threshold 视为断层
    height_gaps = np.diff(rows.height)
    split_at = np.flatnonzero((height_gaps <= 0) | (height_gaps > maximum_gap)) + 1
    indices = np.split(np.arange(len(rows)), split_at)
    return [
        ContourRows(rows.height[index], rows.v_min[index], rows.v_max[index])
        for index in indices
        if index.size > 0
    ]


def _median_smooth(values: np.ndarray) -> np.ndarray:
    """三点中值滤波去除孤立噪声"""
    values = np.asarray(values, dtype=float)
    if values.size < 3:
        return values.copy()
    padded = np.pad(values, (1, 1), mode="edge")
    return np.asarray(
        [np.median(padded[index : index + 3]) for index in range(values.size)],
        dtype=float,
    )


def _last_true_run_start(mask: np.ndarray, minimum_run_layers: int) -> int | None:
    true_indices = np.flatnonzero(np.asarray(mask, dtype=bool))
    if true_indices.size == 0:
        return None
    runs = np.split(true_indices, np.flatnonzero(np.diff(true_indices) > 1) + 1)
    valid_runs = [run for run in runs if run.size >= minimum_run_layers]
    return None if not valid_runs else int(valid_runs[-1][0])


def _isolated_rail_rows(
    rows: ContourRows, y_threshold: float, minimum_layers: int,
) -> tuple[ContourRows, ContourRows] | None:
    """两段之间的空隙大于各段高度时，按完整的上下横梁处理。

    小缺层及带长竖梁的轮廓仍交给原有宽度转折识别。
    """
    segments = _split_contiguous_rows(rows, y_threshold)
    if len(segments) != 2 or any(len(s) < max(2, minimum_layers) for s in segments):
        return None
    lower, upper = segments
    gap = upper.height[0] - lower.height[-1]
    if gap <= max(
        y_threshold * 1.5,
        lower.height[-1] - lower.height[0],
        upper.height[-1] - upper.height[0],
    ):
        return None
    return lower, upper


def _validate_edge_detection_parameters(
    y_threshold: float,
    minimum_x_change_mm: float,
    minimum_run_layers: int,
    minimum_horizontal_layers: int | None,
) -> int:
    """校验边缘检测参数，并补齐水平段的最小层数。"""
    if (
        isinstance(y_threshold, bool)
        or not math.isfinite(y_threshold)
        or y_threshold <= 0
    ):
        raise ValueError("y_threshold 必须是大于0的有限数")
    if isinstance(minimum_x_change_mm, bool):
        raise TypeError("minimum_x_change_mm 不能是布尔值")
    if not math.isfinite(minimum_x_change_mm) or minimum_x_change_mm <= 0:
        raise ValueError("minimum_x_change_mm 必须是大于0的有限数")
    if isinstance(minimum_run_layers, bool) or not isinstance(minimum_run_layers, int):
        raise TypeError("minimum_run_layers 必须是整数")
    if minimum_run_layers <= 0:
        raise ValueError("minimum_run_layers 必须大于0")
    if minimum_horizontal_layers is None:
        minimum_horizontal_layers = minimum_run_layers
    if isinstance(minimum_horizontal_layers, bool) or not isinstance(
        minimum_horizontal_layers, int
    ):
        raise TypeError("minimum_horizontal_layers 必须是整数")
    if minimum_horizontal_layers <= 0:
        raise ValueError("minimum_horizontal_layers 必须大于0")
    return minimum_horizontal_layers


def _folded_terminal_rail_indices(
    rows: ContourRows, y_threshold: float, change_mm: float, minimum_layers: int,
) -> tuple[int, int] | None:
    """识别连续竖杆两端的折弯横梁；竖杆背面的凸起不作为内边界。

    V_Min 是左右侧共同的迎枪表面。折弯可表现为整层向较小X平移，
    因而不能仅按 V_Max-V_Min 判定。最多跨越100mm的端部回折间隔，
    不跨越缺层；两端都必须有连续的、整体偏离竖杆的轮廓证据。
    """
    required = max(2, minimum_layers)
    if len(rows) < max(12, required * 4):
        return None
    if np.any(np.diff(rows.height) > y_threshold * 1.5):
        return None
    start, stop = int(len(rows) * .35), int(math.ceil(len(rows) * .65))
    reference = float(np.median(rows.v_min[start:stop]))
    core = rows.v_min[start:stop]
    if np.mean(np.abs(core - reference) < change_mm / 2) < .8:
        return None
    surface = _median_smooth(rows.v_min)
    shifted = np.flatnonzero(surface <= reference - change_mm)
    if shifted.size == 0:
        return None
    groups = np.split(
        shifted, np.flatnonzero(np.diff(rows.height[shifted]) > 100.0) + 1,
    )
    lower, upper = groups[0], groups[-1]
    if len(groups) < 2 or lower[-1] >= start or upper[0] < stop:
        return None
    if (rows.height[lower[0]] - rows.height[0] > 100.0
            or rows.height[-1] - rows.height[upper[-1]] > 100.0):
        return None
    for group in (lower, upper):
        # 整层偏移的连续证据，排除单层噪声及仅宽度增加的普通横梁。
        translated = rows.v_max[group] < reference - change_mm / 2
        indices = group[translated]
        runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
        if not any(run.size >= required for run in runs):
            return None
    lower_end, upper_start = int(lower[-1]), int(upper[0])
    # 用较小的恢复阈值保留折弯过渡，直到重新贴近稳定竖杆。
    while lower_end + 1 < start and surface[lower_end + 1] < reference - change_mm / 2:
        lower_end += 1
    while upper_start - 1 >= stop and surface[upper_start - 1] < reference - change_mm / 2:
        upper_start -= 1
    return lower_end, upper_start


def _detect_frame_edge_candidates(
    rows: ContourRows,
    y_threshold: float = 10.0,
    minimum_x_change_mm: float = 100,
    minimum_run_layers: int = 3,
    minimum_horizontal_layers: int | None = None,
) -> FrameEdgeCandidates:
    minimum_horizontal_layers = _validate_edge_detection_parameters(
        y_threshold,
        minimum_x_change_mm,
        minimum_run_layers,
        minimum_horizontal_layers,
    )
    if len(rows) == 0:
        return FrameEdgeCandidates(
            None, None, None, None, float(minimum_x_change_mm), "empty"
        )

    out_up = EdgeCandidate(float(rows.height[-1]), float(rows.v_min[-1]))
    out_down = EdgeCandidate(float(rows.height[0]), float(rows.v_min[0]))
    isolated = _isolated_rail_rows(rows, y_threshold, minimum_horizontal_layers)
    if isolated is not None:
        lower, upper = isolated
        return FrameEdgeCandidates(
            out_up,
            EdgeCandidate(float(upper.height[0]), float(upper.v_min[0])),
            EdgeCandidate(float(lower.height[-1]), float(lower.v_min[-1])),
            out_down, float(minimum_x_change_mm), "separated_segments",
        )
    segments = [
        segment
        for segment in _split_contiguous_rows(rows, y_threshold)
        if len(segment) >= minimum_horizontal_layers
    ]
    if len(segments) >= 2:
        bottom_segment = segments[0]
        top_segment = segments[-1]
        in_up_index = 0
        top_span = float(top_segment.height[-1] - top_segment.height[0])
        connector_span = float(y_threshold) * max(10, minimum_run_layers * 3)
        detection_mode = "separated_segments"
        if top_span >= connector_span:
            widths = top_segment.v_max - top_segment.v_min
            wide_run_start = _last_true_run_start(
                widths >= float(minimum_x_change_mm),
                minimum_horizontal_layers,
            )
            if wide_run_start is None:
                return FrameEdgeCandidates(
                    out_up,
                    None,
                    EdgeCandidate(
                        float(bottom_segment.height[-1]),
                        float(bottom_segment.v_min[-1]),
                    ),
                    out_down,
                    float(minimum_x_change_mm),
                    "connector_ambiguous",
                )
            in_up_index = wide_run_start
            detection_mode = "upper_width_transition"
        in_up = EdgeCandidate(
            float(top_segment.height[in_up_index]),
            float(top_segment.v_min[in_up_index]),
        )
        in_down = EdgeCandidate(
            float(bottom_segment.height[-1]),
            float(bottom_segment.v_min[-1]),
        )
        if in_up.height > in_down.height:
            return FrameEdgeCandidates(
                out_up,
                in_up,
                in_down,
                out_down,
                float(minimum_x_change_mm),
                detection_mode,
            )

    if len(rows) < max(7, minimum_horizontal_layers * 3):
        return FrameEdgeCandidates(
            out_up,
            None,
            None,
            out_down,
            float(minimum_x_change_mm),
            "insufficient",
        )

    folded = _folded_terminal_rail_indices(
        rows, y_threshold, minimum_x_change_mm, minimum_run_layers,
    )
    if folded is not None:
        lower_end, upper_start = folded
        return FrameEdgeCandidates(
            out_up,
            EdgeCandidate(float(rows.height[upper_start]), float(rows.v_min[upper_start])),
            EdgeCandidate(float(rows.height[lower_end]), float(rows.v_min[lower_end])),
            out_down, float(minimum_x_change_mm), "folded_terminal_rails",
        )

    smooth_min = _median_smooth(rows.v_min)
    smooth_max = _median_smooth(rows.v_max)
    count = len(rows)
    middle_start = max(1, int(count * 0.35))
    middle_stop = min(
        count - 1,
        max(middle_start + 1, int(math.ceil(count * 0.65))),
    )
    middle_slice = slice(middle_start, middle_stop)
    reference_min = float(np.median(smooth_min[middle_slice]))
    reference_max = float(np.median(smooth_max[middle_slice]))
    change_score = np.maximum(
        np.abs(smooth_min - reference_min),
        np.abs(smooth_max - reference_max),
    )
    middle_score = change_score[middle_slice]
    middle_median = float(np.median(middle_score))
    middle_mad = float(np.median(np.abs(middle_score - middle_median)))
    threshold = max(
        float(minimum_x_change_mm),
        middle_median + 4.0 * 1.4826 * middle_mad,
    )
    changed = change_score >= threshold

    in_down_index = None
    for index in range(
        middle_start - 1,
        minimum_horizontal_layers - 2,
        -1,
    ):
        run_start = index - minimum_horizontal_layers + 1
        if np.all(changed[run_start : index + 1]):
            in_down_index = index
            break
    in_up_index = None
    last_start = count - minimum_horizontal_layers
    for index in range(middle_stop, last_start + 1):
        if np.all(changed[index : index + minimum_horizontal_layers]):
            in_up_index = index
            break

    in_down = (
        EdgeCandidate(
            float(rows.height[in_down_index]),
            float(rows.v_min[in_down_index]),
        )
        if in_down_index is not None
        else None
    )
    in_up = (
        EdgeCandidate(
            float(rows.height[in_up_index]),
            float(rows.v_min[in_up_index]),
        )
        if in_up_index is not None
        else None
    )
    if in_up is not None and in_down is not None and in_up.height <= in_down.height:
        in_up = None
        in_down = None
    return FrameEdgeCandidates(
        out_up,
        in_up,
        in_down,
        out_down,
        threshold,
        "profile_change",
    )


def _detect_frame_edge_sequence(
    rows_by_frame: Sequence[ContourRows],
    y_threshold: float = 10.0,
    minimum_x_change_mm: float = 100,
    minimum_run_layers: int = 3,
    minimum_horizontal_layers: int | None = None,
) -> list[FrameEdgeCandidates]:
    results = [
        _detect_frame_edge_candidates(
            rows,
            y_threshold=y_threshold,
            minimum_x_change_mm=minimum_x_change_mm,
            minimum_run_layers=minimum_run_layers,
            minimum_horizontal_layers=minimum_horizontal_layers,
        )
        for rows in rows_by_frame
    ]
    if not results:
        return []
    completed_values = [
        {
            "out_up": result.out_up,
            "in_up": result.in_up,
            "in_down": result.in_down,
            "out_down": result.out_down,
        }
        for result in results
    ]
    filled_indices: set[int] = set()
    frame_indices = np.arange(len(results), dtype=float)
    for edge_name in ("in_up", "in_down"):
        known_indices = np.asarray(
            [
                index
                for index, values in enumerate(completed_values)
                if values[edge_name] is not None
            ],
            dtype=int,
        )
        if known_indices.size == 0:
            continue
        known_heights = np.asarray(
            [completed_values[index][edge_name].height for index in known_indices],
            dtype=float,
        )
        guided_heights = np.interp(frame_indices, known_indices, known_heights)
        for index, rows in enumerate(rows_by_frame):
            if completed_values[index][edge_name] is not None or len(rows) == 0:
                continue
            nearest_row = int(np.argmin(np.abs(rows.height - guided_heights[index])))
            completed_values[index][edge_name] = EdgeCandidate(
                float(rows.height[nearest_row]),
                float(rows.v_min[nearest_row]),
            )
            filled_indices.add(index)

    completed_results = []
    for index, (result, values) in enumerate(zip(results, completed_values)):
        in_up = values["in_up"]
        in_down = values["in_down"]
        out_up = values["out_up"]
        out_down = values["out_down"]
        if (
            out_up is not None
            and in_up is not None
            and in_down is not None
            and out_down is not None
            and not (out_up.height >= in_up.height > in_down.height >= out_down.height)
        ):
            in_up = None
            in_down = None
        completed_results.append(
            FrameEdgeCandidates(
                out_up,
                in_up,
                in_down,
                out_down,
                result.change_threshold,
                "neighbor_guided" if index in filled_indices else result.detection_mode,
            )
        )
    return completed_results


def _build_three_region_boxes(
    rows: ContourRows,
    edges: FrameEdgeCandidates,
    y_threshold: float = 10.0,
    minimum_middle_coverage: float = 0.3,
    horizontal_width_mm: float = 50.0,
    minimum_horizontal_layers: int = 4,
    rail_padding_layers: int = 4,
    middle_trim_layers: int = 4,
) -> ThreeRegionBoxes | None:
    if len(rows) == 0 or not edges.is_complete:
        return None
    if not math.isfinite(y_threshold) or y_threshold <= 0:
        raise ValueError("y_threshold 必须是大于0的有限数")
    if (
        not math.isfinite(minimum_middle_coverage)
        or not 0 <= minimum_middle_coverage <= 1
    ):
        raise ValueError("minimum_middle_coverage 必须在0到1之间")
    if not math.isfinite(horizontal_width_mm) or horizontal_width_mm <= 0:
        raise ValueError("horizontal_width_mm 必须是大于0的有限数")
    if (
        minimum_horizontal_layers <= 0
        or rail_padding_layers < 0
        or middle_trim_layers < 0
    ):
        raise ValueError("横梁连续层数必须大于0，扩展和收缩层数不能小于0")

    def make_box_from_indices(indices: np.ndarray) -> RegionBox | None:
        if indices.size == 0:
            return None
        return RegionBox(
            x_min=float(np.min(rows.v_min[indices])),
            x_max=float(np.max(rows.v_max[indices])),
            y_min=float(np.min(rows.height[indices])),
            y_max=float(np.max(rows.height[indices])),
        )

    isolated = _isolated_rail_rows(rows, y_threshold, minimum_horizontal_layers)
    if isolated is not None:
        lower_rows, upper_rows = isolated
        return ThreeRegionBoxes(
            lower=make_box_from_indices(np.flatnonzero(rows.height <= lower_rows.height[-1])),
            upper=make_box_from_indices(np.flatnonzero(rows.height >= upper_rows.height[0])),
            middle=None,
        )

    def contiguous_runs(mask: np.ndarray, minimum_layers: int) -> list[np.ndarray]:
        indices = np.flatnonzero(mask)
        if indices.size == 0:
            return []
        gaps = (np.diff(indices) > 1) | (
            np.diff(rows.height[indices]) > y_threshold * 1.5
        )
        return [
            run
            for run in np.split(indices, np.flatnonzero(gaps) + 1)
            if run.size >= minimum_layers
        ]

    widths = rows.v_max - rows.v_min
    wide_runs = contiguous_runs(
        widths >= horizontal_width_mm,
        minimum_horizontal_layers,
    )

    def expand_run(run: np.ndarray) -> np.ndarray:
        if run.size == 0:
            return run
        start = int(run[0])
        stop = int(run[-1])
        maximum_gap = y_threshold * 1.5
        for _ in range(rail_padding_layers):
            if (
                start > 0
                and rows.height[start] - rows.height[start - 1] <= maximum_gap
            ):
                start -= 1
            if (
                stop + 1 < len(rows)
                and rows.height[stop + 1] - rows.height[stop] <= maximum_gap
            ):
                stop += 1
        return np.arange(start, stop + 1, dtype=int)

    if edges.detection_mode == "folded_terminal_rails":
        # 已按竖杆恢复位置找到完整端部；扩展最多包含首个竖杆层，
        # 不再把固定padding层数延伸到中间的附件上。
        padding = min(rail_padding_layers, 1) * y_threshold
        lower = make_box_from_indices(np.flatnonzero(
            rows.height <= edges.in_down.height + padding
        ))
        upper = make_box_from_indices(np.flatnonzero(
            rows.height >= edges.in_up.height - padding
        ))
    elif len(wide_runs) >= 2:
        lower_run = wide_runs[0]
        upper_run = wide_runs[-1]
        lower = make_box_from_indices(expand_run(lower_run))
        upper = make_box_from_indices(expand_run(upper_run))
    else:
        upper_mask = (rows.height >= edges.in_up.height) & (
            rows.height <= edges.out_up.height
        )
        lower_mask = (rows.height >= edges.out_down.height) & (
            rows.height <= edges.in_down.height
        )
        # 边界回退分支同样按真实连续层扩展。薄横梁可能只有一层，
        # 若不扩展，零高度框会进入Z向平滑并使邻帧的整组正式框失效。
        upper = make_box_from_indices(expand_run(np.flatnonzero(upper_mask)))
        lower = make_box_from_indices(expand_run(np.flatnonzero(lower_mask)))

    if upper is None or lower is None or upper.y_min <= lower.y_max:
        return None
    middle_mask = (
        (rows.height > lower.y_max)
        & (rows.height < upper.y_min)
        & (widths < horizontal_width_mm)
    )
    middle_runs = contiguous_runs(middle_mask, minimum_layers=2)
    expected_middle_layers = max(
        int(round((upper.y_min - lower.y_max) / y_threshold)) - 1,
        1,
    )
    longest_middle_run = (
        max(middle_runs, key=len) if middle_runs else np.empty(0, dtype=int)
    )
    middle_coverage = longest_middle_run.size / expected_middle_layers
    if (
        middle_coverage >= minimum_middle_coverage
        and longest_middle_run.size > middle_trim_layers * 2
    ):
        middle_indices = longest_middle_run[
            middle_trim_layers : longest_middle_run.size - middle_trim_layers
        ]
        middle_core = make_box_from_indices(middle_indices)
        middle = RegionBox(
            x_min=middle_core.x_min,
            x_max=middle_core.x_max,
            y_min=lower.y_max,
            y_max=upper.y_min,
        )
    else:
        middle = None
    return ThreeRegionBoxes(upper=upper, middle=middle, lower=lower)


def _stabilize_numeric_track(
    values: Sequence[float | None],
    window_size: int = 11,
    quantize_step: float | None = None,
) -> list[float | None]:
    if (
        isinstance(window_size, bool)
        or not isinstance(window_size, int)
        or window_size <= 0
    ):
        raise ValueError("window_size 必须是大于0的整数")
    if window_size % 2 == 0:
        raise ValueError("window_size 必须是奇数")
    if quantize_step is not None and (
        not math.isfinite(quantize_step) or quantize_step <= 0
    ):
        raise ValueError("quantize_step 必须是大于0的有限数")
    if not values:
        return []
    numeric = np.asarray(
        [np.nan if value is None else float(value) for value in values],
        dtype=float,
    )
    known = np.flatnonzero(np.isfinite(numeric))
    if known.size == 0:
        return [None] * len(values)
    frame_indices = np.arange(len(values), dtype=float)
    interpolated = np.interp(frame_indices, known, numeric[known])
    actual_window = min(window_size, len(values))
    if actual_window % 2 == 0:
        actual_window -= 1
    actual_window = max(actual_window, 1)
    half_window = actual_window // 2
    smoothed = np.empty(len(values), dtype=float)
    for index in range(len(values)):
        start = min(max(index - half_window, 0), len(values) - actual_window)
        smoothed[index] = float(np.median(interpolated[start : start + actual_window]))
    if quantize_step is not None:
        smoothed = np.round(smoothed / quantize_step) * quantize_step
    return [float(value) for value in smoothed]


def _build_region_box_sequence(
    rows_by_frame: Sequence[ContourRows],
    edges_by_frame: Sequence[FrameEdgeCandidates],
    y_threshold: float = 10.0,
    smoothing_window: int = 11,
    minimum_middle_coverage: float = 0.3,
    narrow_width_mm: float = 50.0,
    minimum_horizontal_layers: int = 4,
    rail_padding_layers: int = 4,
    middle_trim_layers: int = 4,
) -> list[ThreeRegionBoxes | None]:
    if len(rows_by_frame) != len(edges_by_frame):
        raise ValueError("rows_by_frame 与 edges_by_frame 长度必须一致")
    if not rows_by_frame:
        return []
    raw_boxes = [
        _build_three_region_boxes(
            rows,
            edges,
            y_threshold=y_threshold,
            minimum_middle_coverage=minimum_middle_coverage,
            horizontal_width_mm=narrow_width_mm,
            minimum_horizontal_layers=minimum_horizontal_layers,
            rail_padding_layers=rail_padding_layers,
            middle_trim_layers=middle_trim_layers,
        )
        for rows, edges in zip(rows_by_frame, edges_by_frame)
    ]

    def track(selector, quantize: float | None = None) -> list[float | None]:
        return _stabilize_numeric_track(
            [None if boxes is None else selector(boxes) for boxes in raw_boxes],
            window_size=smoothing_window,
            quantize_step=quantize,
        )

    lower_y_min = track(lambda boxes: boxes.lower.y_min, y_threshold)
    lower_y_max = track(lambda boxes: boxes.lower.y_max, y_threshold)
    upper_y_min = track(lambda boxes: boxes.upper.y_min, y_threshold)
    upper_y_max = track(lambda boxes: boxes.upper.y_max, y_threshold)
    lower_x_min = track(lambda boxes: boxes.lower.x_min)
    lower_x_max = track(lambda boxes: boxes.lower.x_max)
    upper_x_min = track(lambda boxes: boxes.upper.x_min)
    upper_x_max = track(lambda boxes: boxes.upper.x_max)

    results = []
    for index, rows in enumerate(rows_by_frame):
        # 中值平滑不能裁掉已完整识别的独立横梁或端部折弯横梁。
        if (_isolated_rail_rows(rows, y_threshold, minimum_horizontal_layers) is not None
                or edges_by_frame[index].detection_mode == "folded_terminal_rails"):
            current = raw_boxes[index]
            if current is not None:
                for low, high, box in (
                    (lower_y_min, lower_y_max, current.lower),
                    (upper_y_min, upper_y_max, current.upper),
                ):
                    low[index] = min(low[index], box.y_min)
                    high[index] = max(high[index], box.y_max)
                for low, high, box in (
                    (lower_x_min, lower_x_max, current.lower),
                    (upper_x_min, upper_x_max, current.upper),
                ):
                    low[index] = min(low[index], box.x_min)
                    high[index] = max(high[index], box.x_max)
        boundaries = (
            lower_y_min[index],
            lower_y_max[index],
            upper_y_min[index],
            upper_y_max[index],
            lower_x_min[index],
            lower_x_max[index],
            upper_x_min[index],
            upper_x_max[index],
        )
        if any(value is None for value in boundaries):
            results.append(None)
            continue
        if not (
            lower_y_min[index]
            < lower_y_max[index]
            < upper_y_min[index]
            < upper_y_max[index]
            and lower_x_min[index] < lower_x_max[index]
            and upper_x_min[index] < upper_x_max[index]
        ):
            results.append(None)
            continue
        lower = RegionBox(
            lower_x_min[index],
            lower_x_max[index],
            lower_y_min[index],
            lower_y_max[index],
        )
        upper = RegionBox(
            upper_x_min[index],
            upper_x_max[index],
            upper_y_min[index],
            upper_y_max[index],
        )
        middle_mask = (rows.height > lower.y_max) & (rows.height < upper.y_min)
        expected_middle_layers = max(
            int(round((upper.y_min - lower.y_max) / y_threshold)) - 1,
            1,
        )
        middle_indices = np.flatnonzero(middle_mask)
        middle_coverage = middle_indices.size / expected_middle_layers
        middle = None
        if middle_coverage >= minimum_middle_coverage and middle_indices.size >= 2:
            widths = rows.v_max[middle_indices] - rows.v_min[middle_indices]
            narrow_indices = middle_indices[widths <= narrow_width_mm]
            if narrow_indices.size < 2:
                keep_count = max(2, int(math.ceil(middle_indices.size * 0.25)))
                narrow_indices = middle_indices[np.argsort(widths)[:keep_count]]
            middle_x_min = float(np.median(rows.v_min[narrow_indices]))
            middle_x_max = float(np.median(rows.v_max[narrow_indices]))
            if middle_x_min < middle_x_max:
                middle = RegionBox(
                    middle_x_min,
                    middle_x_max,
                    lower.y_max,
                    upper.y_min,
                )
        results.append(ThreeRegionBoxes(upper=upper, middle=middle, lower=lower))
    return results


def _rectangle_x_reciprocation_start_points(
    boxes: ThreeRegionBoxes,
    side: str,
) -> dict[str, EdgeCandidate]:
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    if boxes.middle is not None:
        in_up_x = boxes.middle.x_min
        in_down_x = boxes.middle.x_min
    else:
        in_up_x = boxes.upper.x_max
        in_down_x = boxes.lower.x_max
    return {
        "out_up": EdgeCandidate(boxes.upper.y_max, boxes.upper.x_max),
        "in_up": EdgeCandidate(boxes.upper.y_min, in_up_x),
        "in_down": EdgeCandidate(boxes.lower.y_max, in_down_x),
        "out_down": EdgeCandidate(boxes.lower.y_min, boxes.lower.x_max),
    }


def _rectangle_x_reciprocation_end_points(
    boxes: ThreeRegionBoxes,
    side: str,
) -> dict[str, EdgeCandidate]:
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    return {
        "out_up": EdgeCandidate(boxes.upper.y_max, boxes.upper.x_min),
        "in_up": EdgeCandidate(boxes.upper.y_min, boxes.upper.x_min),
        "in_down": EdgeCandidate(boxes.lower.y_max, boxes.lower.x_min),
        "out_down": EdgeCandidate(boxes.lower.y_min, boxes.lower.x_min),
    }


def _rectangle_x_axis_strokes(
    boxes: ThreeRegionBoxes,
    side: str,
) -> dict[str, DetectedXAxisStroke]:
    axis_by_surface = {
        "out_down": "x1",
        "in_down": "x2",
        "in_up": "x3",
        "out_up": "x4",
    }
    start_points = _rectangle_x_reciprocation_start_points(boxes, side)
    end_points = _rectangle_x_reciprocation_end_points(boxes, side)
    strokes = {
        axis_by_surface[surface]: DetectedXAxisStroke(
            start_x=start_points[surface].detected_x,
            end_x=end_points[surface].detected_x,
        )
        for surface in axis_by_surface
    }
    return {axis: strokes[axis] for axis in XN_SIDE_X_AXES}


def _x_config_value(machine_config: Mapping, key: str) -> int:
    values = machine_config.get(key)
    if not isinstance(values, (list, tuple)) or len(values) <= 1:
        raise ValueError(f"MachineConfig.{key} 必须包含[y, x]两个值")
    value = values[1]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"MachineConfig.{key} 的X轴值必须是整数")
    return value


def _build_x_axis_ranges(
    boxes: ThreeRegionBoxes,
    side: str,
    machine_config: Mapping,
) -> dict[str, PlannedXAxisRange]:
    if machine_config.get("type") != "xn_side":
        raise ValueError("machine_config 必须是 xn_side 设备配置")
    if machine_config.get("install_orietation") != side:
        raise ValueError("machine_config 的安装方向与 side 不一致")
    x_position = machine_config.get("x_position")
    position_speed = machine_config.get("x_pos_speed")
    reciprocation_speed = machine_config.get("x_recip_speed")
    for name, value in {
        "x_position": x_position,
        "x_pos_speed": position_speed,
        "x_recip_speed": reciprocation_speed,
    }.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"MachineConfig.{name} 必须是整数")

    min_limit_pos = _x_config_value(machine_config, "min_limit_pos")
    max_limit_pos = _x_config_value(machine_config, "max_limit_pos")
    strokes = _rectangle_x_axis_strokes(boxes, side)
    results = {}
    for axis_name, stroke in strokes.items():
        start_x = _round_runtime_mm(stroke.start_x)
        end_x = _round_runtime_mm(stroke.end_x)
        front_offset_key, after_offset_key = X_AXIS_OFFSET_KEYS[axis_name]
        front_offset = machine_config.get(front_offset_key)
        after_offset = machine_config.get(after_offset_key)
        for key, value in (
            (front_offset_key, front_offset),
            (after_offset_key, after_offset),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"MachineConfig.{key} 必须是整数")
        results[axis_name] = PlannedXAxisRange(
            axis_name=axis_name,
            detected_start_x=start_x,
            detected_end_x=end_x,
            start_pos=start_x - x_position - front_offset,
            end_pos=end_x - x_position - after_offset,
            min_limit_pos=min_limit_pos,
            max_limit_pos=max_limit_pos,
            position_speed=position_speed,
            reciprocation_speed=reciprocation_speed,
            status=0,
        )
    return {axis: results[axis] for axis in XN_SIDE_X_AXES}


def _build_minimum_x_envelope(
    motions_by_frame: Sequence[dict[str, PlannedXAxisRange] | None],
    half_window_frames: int,
    *,
    contour_occupied: Sequence[bool] | None = None,
) -> list[dict[str, PlannedXAxisRange] | None]:
    if isinstance(half_window_frames, bool) or not isinstance(half_window_frames, int):
        raise TypeError("half_window_frames 必须是整数")
    if half_window_frames < 0:
        raise ValueError("half_window_frames 不能小于0")
    frame_count = len(motions_by_frame)
    if contour_occupied is not None and len(contour_occupied) != frame_count:
        raise ValueError("轮廓标记与行程帧数不一致")
    envelope = []
    for center_index, center_motions in enumerate(motions_by_frame):
        if center_motions is None:
            envelope.append(None)
            continue
        start_index = max(0, center_index - half_window_frames)
        end_index = min(frame_count, center_index + half_window_frames + 1)
        window = motions_by_frame[start_index:end_index]
        if contour_occupied is not None:
            selected = _leading_edge_window_indices(
                contour_occupied[start_index:end_index],
                center_index - start_index, older_at_end=False,
                allow_trailing_edge=True,
            )
            if selected is None:
                envelope.append(None)
                continue
            window = [window[i] for i in selected]
        if any(frame_motions is None for frame_motions in window):
            envelope.append(None)
            continue
        safe_motions = {}
        for axis_name in XN_SIDE_X_AXES:
            candidates = [frame[axis_name] for frame in window if frame is not None]
            safe_motions[axis_name] = replace(
                center_motions[axis_name],
                window_min_detected_x=min(
                    item.window_min_detected_x
                    if item.window_min_detected_x is not None
                    else min(item.detected_start_x, item.detected_end_x)
                    for item in candidates
                ),
                start_pos=min(item.start_pos for item in candidates),
                end_pos=min(item.end_pos for item in candidates),
            )
        envelope.append(safe_motions)
    return envelope


def _build_minimum_x_envelope_at(
    motions_by_frame,
    half_window_frames,
    center_index,
    *,
    contour_occupied=None,
):
    """仅求指定中心的包络；窗口选择与完整序列算法保持一致。"""
    if isinstance(half_window_frames, bool) or not isinstance(half_window_frames, int):
        raise TypeError("half_window_frames 必须是整数")
    if half_window_frames < 0:
        raise ValueError("half_window_frames 不能小于0")
    if contour_occupied is not None and len(contour_occupied) != len(motions_by_frame):
        raise ValueError("轮廓标记与行程帧数不一致")
    center_motions = motions_by_frame[center_index]
    if center_motions is None:
        return None
    start = max(0, center_index - half_window_frames)
    stop = min(len(motions_by_frame), center_index + half_window_frames + 1)
    window = motions_by_frame[start:stop]
    if contour_occupied is not None:
        selected = _leading_edge_window_indices(
            contour_occupied[start:stop], center_index - start,
            older_at_end=False, allow_trailing_edge=True,
        )
        if selected is None:
            return None
        window = [window[i] for i in selected]
    if any(frame is None for frame in window):
        return None
    result = {}
    for axis in XN_SIDE_X_AXES:
        candidates = [frame[axis] for frame in window]
        result[axis] = replace(
            center_motions[axis],
            window_min_detected_x=min(
                item.window_min_detected_x
                if item.window_min_detected_x is not None
                else min(item.detected_start_x, item.detected_end_x)
                for item in candidates
            ),
            start_pos=min(item.start_pos for item in candidates),
            end_pos=min(item.end_pos for item in candidates),
        )
    return result


def _obstacle_min_x_at_height(
    rows: ContourRows, target_height: int, half_height_mm: int,
) -> int | None:
    """按绝对激光高度取闭区间内原始轮廓最小X，不做类别过滤。"""
    _require_int("target_height", target_height)
    _require_int("gun_obstacle_half_height_mm", half_height_mm)
    if half_height_mm < 0:
        raise ValueError("gun_obstacle_half_height_mm 不能小于0")
    mask = (rows.height >= target_height - half_height_mm) & (
        rows.height <= target_height + half_height_mm
    )
    return _round_runtime_mm(float(np.min(rows.v_min[mask]))) if np.any(mask) else None


def _apply_gun_obstacle_constraints(
    motions: Mapping[str, PlannedXAxisRange] | None,
    rows_by_frame: Sequence[ContourRows],
    center_index: int,
    y_targets: Mapping[str, int],
    machine_config: Mapping,
    half_height_mm: int,
) -> tuple[Mapping[str, PlannedXAxisRange] | None, tuple[GunObstacleConstraint, ...]]:
    if not 0 <= center_index < len(rows_by_frame):
        raise ValueError("center_index 超出安全窗口范围")
    if set(y_targets) != set(XN_SIDE_Y_AXES):
        raise ValueError("y_targets 必须且只能包含 y1、y2、y3")
    if motions is not None and set(motions) != set(XN_SIDE_X_AXES):
        raise ValueError("motions 必须且只能包含 x1、x2、x3、x4")
    origins = machine_config.get("origin_pos")
    if not isinstance(origins, (list, tuple)) or len(origins) != 4:
        raise ValueError("origin_pos 必须包含四把枪的Y零点")
    x_position = _require_int("x_position", machine_config.get("x_position"))
    result = None if motions is None else dict(motions)
    diagnostics = []
    for index, axis in enumerate(XN_SIDE_X_AXES):
        height = _require_int("origin_pos", origins[index])
        if index < 3:
            height += _require_int(f"y{index + 1}", y_targets[f"y{index + 1}"])
        values = tuple(
            _obstacle_min_x_at_height(rows, height, half_height_mm)
            for rows in rows_by_frame
        )
        known = [value for value in values if value is not None]
        obstacle_x = min(known) if known else None
        offset_key = X_AXIS_OFFSET_KEYS[axis][0]
        offset = _require_int(offset_key, machine_config.get(offset_key))
        limit = None if obstacle_x is None else obstacle_x - x_position - offset
        original = None if motions is None else motions[axis]
        adjusted = original
        constrained = collapsed = False
        if original is not None and limit is not None:
            low, high = sorted((original.start_pos, original.end_pos))
            constrained = limit < high
            collapsed = constrained and limit < low
            if collapsed:
                adjusted = replace(original, start_pos=limit, end_pos=limit)
            elif constrained:
                adjusted = (
                    replace(original, start_pos=limit)
                    if original.start_pos >= original.end_pos
                    else replace(original, end_pos=limit)
                )
            result[axis] = adjusted
        diagnostics.append(GunObstacleConstraint(
            axis_name=axis, target_height=height,
            height_min=height - half_height_mm, height_max=height + half_height_mm,
            window_min_x=values, center_min_x=values[center_index],
            obstacle_min_x=obstacle_x, limit_pos=limit,
            laser_x_offset=x_position + offset,
            original_start_pos=None if original is None else original.start_pos,
            original_end_pos=None if original is None else original.end_pos,
            final_start_pos=None if adjusted is None else adjusted.start_pos,
            final_end_pos=None if adjusted is None else adjusted.end_pos,
            constrained=constrained, collapsed=collapsed,
        ))
    return result, tuple(diagnostics)


@dataclass
class AxisGroupArrival:
    """整组反馈同时到位才计数；目标改变后重新确认。"""

    targets: dict[str, int] = field(default_factory=dict)
    count: int = 0

    def observe(self, targets, actual, tolerance, confirmations) -> bool:
        if self.targets.keys() != targets.keys() or any(
            abs(value - self.targets[axis]) > tolerance
            for axis, value in targets.items()
        ):
            self.count = 0
        self.targets = dict(targets)
        self.count = self.count + 1 if all(
            abs(actual[axis] - value) <= tolerance
            for axis, value in targets.items()
        ) else 0
        return self.count >= confirmations


@dataclass
class SidePositioningSequence:
    phase: str = "idle"
    held: dict[str, int] = field(default_factory=dict)
    endpoint_names: dict[str, str] = field(default_factory=dict)
    x_arrival: AxisGroupArrival = field(default_factory=AxisGroupArrival)
    y_arrival: AxisGroupArrival = field(default_factory=AxisGroupArrival)


class MotionFrameByFramePlanning:
    """将逐帧规划结果转换为 PLC 发送帧。"""

    SIDES = ("left", "right")

    def __init__(self, planning_config: Mapping) -> None:
        self.parameters = FrameByFramePlanningParameters.from_config(planning_config)
        self.idle_return_distance_mm = self.parameters.idle_return_distance_mm
        self.x_arrival_confirmations = self.parameters.x_arrival_confirmations
        self.position_tolerance_mm = self.parameters.x_position_tolerance_mm
        self.minimum_gun_y_distance_mm = self.parameters.minimum_gun_y_distance_mm
        self.geometry_parameters = self.parameters.geometry
        self.x_planners = {
            side: XnSideFeedbackXPlanner(self.x_arrival_confirmations)
            for side in self.SIDES
        }
        self._arrival_phase = {side: "waiting" for side in self.SIDES}
        self._window_settings = {}
        self.motion_started = {side: False for side in self.SIDES}
        self._positioning = {side: SidePositioningSequence() for side in self.SIDES}
        self._return_sequence = {side: SidePositioningSequence() for side in self.SIDES}
        self.last_y_targets = {side: None for side in self.SIDES}
        self.variable_2d_states = {
            side: Variable2DReportStateMachine() for side in self.SIDES
        }
        self._geometry_cache_key = None
        self._geometry_cache: dict[str, RuntimeQueueGeometry] = {}
        self._prepared_geometry_cache = {}
        # 入队后的帧按不可变快照使用；替换帧而非原地修改其坐标。
        # 强引用防止id复用，LRU上限避免长期运行积累全部历史帧。
        self._contour_cache = OrderedDict()
        self._variable_2d_cache_key = None
        self._variable_2d_cache: dict[str, Variable2DWindowObservation] = {}
        self.last_reasons: tuple[str, ...] = ()
        self.last_error: str | None = None
        self.last_frame_was_safe = True
        self.last_active_sides: tuple[str, ...] = ()
        self.last_active_2d_sides: tuple[str, ...] = ()
        self.last_debug_snapshot: MotionPlanningDebugSnapshot | None = None
        self._last_x_steps: dict[str, XnSideFeedbackXStep | None] = {
            side: None for side in self.SIDES
        }
        self._return_triggered = {side: False for side in self.SIDES}
        self._return_complete = {side: False for side in self.SIDES}
        self._out_lift_return_active = {side: False for side in self.SIDES}
        self._last_clean_mode = False
        self._clean_gate_state = None
        self._clean_occupancy_cache = {}
        self._behind_empty_cache_key = None
        self._behind_empty_cache: dict[tuple[str, str], bool] = {}

    def reset(self) -> None:
        """清空连续运动状态；供换工件、清队列或离线测试显式调用。"""

        self.x_planners = {
            side: XnSideFeedbackXPlanner(self.x_arrival_confirmations)
            for side in self.SIDES
        }
        self._arrival_phase = {side: "waiting" for side in self.SIDES}
        self._window_settings = {}
        self.motion_started = {side: False for side in self.SIDES}
        self._positioning = {side: SidePositioningSequence() for side in self.SIDES}
        self._return_sequence = {side: SidePositioningSequence() for side in self.SIDES}
        self.last_y_targets = {side: None for side in self.SIDES}
        self.variable_2d_states = {
            side: Variable2DReportStateMachine() for side in self.SIDES
        }
        self._geometry_cache_key = None
        self._geometry_cache = {}
        self._prepared_geometry_cache.clear()
        self._contour_cache.clear()
        self._variable_2d_cache_key = None
        self._variable_2d_cache = {}
        self.last_reasons = ()
        self.last_error = None
        self.last_frame_was_safe = True
        self.last_active_sides = ()
        self.last_active_2d_sides = ()
        self.last_debug_snapshot = None
        self._last_x_steps = {side: None for side in self.SIDES}
        self._return_triggered = {side: False for side in self.SIDES}
        self._return_complete = {side: False for side in self.SIDES}
        self._out_lift_return_active = {side: False for side in self.SIDES}
        self._last_clean_mode = False
        self._clean_gate_state = None
        self._clean_occupancy_cache.clear()
        self._behind_empty_cache_key = None
        self._behind_empty_cache = {}

    def _reset_side(self, side: str) -> None:
        """工件离开某侧设备后，单独重置该侧下一工件的往复状态。"""

        self.x_planners[side] = XnSideFeedbackXPlanner(self.x_arrival_confirmations)
        self.motion_started[side] = False
        self._positioning[side] = SidePositioningSequence()
        self.last_y_targets[side] = None
        self._last_x_steps[side] = None

    def _reset_return_state(self, side: str) -> None:
        self._return_sequence[side] = SidePositioningSequence()
        self._return_triggered[side] = False
        self._return_complete[side] = False

    def _handle_clean_mode_transition(self, clean_mode: bool) -> None:
        if clean_mode == self._last_clean_mode:
            return
        for side in self.SIDES:
            self._reset_side(side)
            self._reset_return_state(side)
            self.variable_2d_states[side] = Variable2DReportStateMachine()
            self._out_lift_return_active[side] = False
        self._last_clean_mode = clean_mode

    def _record_clean_gate(self, state: str) -> None:
        if state != getattr(self, "_clean_gate_state", None):
            from model.utils.LoggerUtil import logger
            logger.info(f"清理模式判定：{state}")
            self._clean_gate_state = state

    def _clean_queue_empty(self, manager, read_data_config: Mapping) -> bool:
        """检查所有启用方向的完整历史；缺失或损坏的数据不能证明为空。"""
        stacks = getattr(manager, "frame_stack", None)
        directions = getattr(manager, "active_directions", None)
        if directions is None and isinstance(stacks, Mapping):
            directions = tuple(stacks)  # 兼容离线队列替身。
        size = read_data_config.get("frame_by_frame_queue_size", manager.stack_size)
        if (isinstance(size, bool) or not isinstance(size, int) or size <= 0
                or size != manager.stack_size):
            raise ValueError("清理检查的配置队列长度与实际长度不一致")
        if not isinstance(stacks, Mapping) or not directions:
            raise ValueError("清理检查缺少启用方向队列")
        occupied = False
        seen = {}
        for direction in directions:
            stack = stacks.get(direction)
            if stack is None or len(stack) != size:
                raise ValueError(f"清理检查的{direction}队列缺失或长度错误")
            for frame in stack:
                if id(frame) in seen:
                    occupied |= seen[id(frame)][2]
                    continue
                data = getattr(frame, "FrameData", None)
                if data is None:
                    raise ValueError("清理检查遇到损坏的历史帧")
                cached = self._clean_occupancy_cache.get(id(frame))
                if cached is not None and cached[0] is frame and cached[1] is data:
                    seen[id(frame)] = cached
                    occupied |= cached[2]
                    continue
                # 通用轮廓解析会忽略坏行；此处先校验，防止将坏数据判为空。
                for row in data:
                    values = tuple(float(getattr(row, name)) for name in
                                   ("H_Axis", "V_Axis_Min", "V_Axis_Max"))
                    if not all(math.isfinite(value) for value in values) or values[1] > values[2]:
                        raise ValueError("清理检查遇到无效轮廓坐标")
                has_workpiece = len(self._cached_contour_rows(frame)) > 0
                seen[id(frame)] = (frame, data, has_workpiece)
                occupied |= has_workpiece
        # 仅保留本轮队列中的帧，避免历史缓存无界增长。
        self._clean_occupancy_cache = seen
        return not occupied

    def _safe_frame(
        self,
        heartbeat: int,
        *reasons: str,
        error: Exception | None = None,
    ) -> SendMovingFrameData:
        for side in self.SIDES:
            self._reset_side(side)
            self._reset_return_state(side)
        self.last_reasons = tuple(reason for reason in reasons if reason)
        self.last_error = None if error is None else str(error)
        self.last_frame_was_safe = True
        self.last_active_sides = ()
        self.last_active_2d_sides = ()
        frame = self.build_safe_frame(heartbeat)
        self.last_debug_snapshot = MotionPlanningDebugSnapshot(
            heartbeat=heartbeat,
            fifo=None,
            safe=True,
            enable=frame.Enable,
            operate=frame.Operate,
            xn_sides=(),
            out_lifts=(),
            active_sides=(),
            active_2d_sides=(),
            reasons=self.last_reasons,
            error=self.last_error,
        )
        return frame

    @staticmethod
    def _find_xn_side_machine(
        machine_config: Mapping,
        side: str,
    ) -> Mapping:
        if not isinstance(machine_config, Mapping):
            raise TypeError("machine_config 必须是映射")
        matches = [
            config
            for config in machine_config.values()
            if isinstance(config, Mapping)
            and config.get("type") == "xn_side"
            and config.get("install_orietation") == side
        ]
        if len(matches) != 1:
            raise ValueError(f"MachineConfig中必须恰好有一台{side}侧xn_side设备")
        return matches[0]

    @staticmethod
    def _find_out_lift_machine(
        machine_config: Mapping,
        side: str,
    ) -> Mapping | None:
        if not isinstance(machine_config, Mapping):
            raise TypeError("machine_config 必须是映射")
        matches = [
            config
            for config in machine_config.values()
            if isinstance(config, Mapping)
            and config.get("type") == "out_lift"
            and config.get("install_orietation") == side
        ]
        if len(matches) > 1:
            raise ValueError(f"MachineConfig中最多只能有一台{side}侧out_lift设备")
        return None if not matches else matches[0]

    @staticmethod
    def _actual_x_positions(plc_data, side: str) -> dict[str, int]:
        """按固定PLC槽位读取一侧四根X轴的实际位置。"""

        axis_list = getattr(plc_data, "AxisList", None)
        if not isinstance(axis_list, Sequence) or isinstance(axis_list, (str, bytes)):
            raise TypeError("plc_data.AxisList 必须是轴反馈序列")

        axis_map = get_x_axes("xn_side", side)
        positions: dict[str, int] = {}
        for axis_name in XN_SIDE_X_AXES:
            axis_index = axis_map[axis_name]
            if axis_index >= len(axis_list):
                raise ValueError(f"plc_data.AxisList 缺少{side}.{axis_name}反馈槽位")
            actual_pos = getattr(axis_list[axis_index], "Pos", None)
            if isinstance(actual_pos, bool) or not isinstance(actual_pos, int):
                raise TypeError(f"{side}.{axis_name}实际位置必须是整数")
            positions[axis_name] = actual_pos
        return positions

    @staticmethod
    def _actual_xn_side_positions(plc_data, side: str) -> dict[str, int]:
        """按固定PLC槽位读取一侧云雀全部七根轴的实际位置。"""

        axis_list = getattr(plc_data, "AxisList", None)
        if not isinstance(axis_list, Sequence) or isinstance(axis_list, (str, bytes)):
            raise TypeError("plc_data.AxisList 必须是轴反馈序列")
        positions = {}
        for axis_name, axis_index in get_axis_map("xn_side", side).items():
            if axis_index >= len(axis_list):
                raise ValueError(f"plc_data.AxisList 缺少{side}.{axis_name}反馈槽位")
            actual_pos = getattr(axis_list[axis_index], "Pos", None)
            if isinstance(actual_pos, bool) or not isinstance(actual_pos, int):
                raise TypeError(f"{side}.{axis_name}实际位置必须是整数")
            positions[axis_name] = actual_pos
        return positions

    @staticmethod
    def _plc_operate_word(plc_data) -> int:
        """将PLC返回的有符号Int16 Operate归一化为无符号位字。"""

        value = getattr(plc_data, "Operate", None)
        # 为只构造了部分PLC字段的旧离线调用保留兼容默认；
        # 真实 ReceiveMovingFrameData 始终包含 Operate。
        if value is None:
            return PLC_OPERATE_LEFT_XN_SIDE | PLC_OPERATE_RIGHT_XN_SIDE
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("plc_data.Operate 必须是整数")
        return value & 0xFFFF

    def _behind_empty_by_machine(
        self,
        frame_queue_manager,
        machines_by_key: Mapping[tuple[str, str], Mapping | None],
    ) -> dict[tuple[str, str], bool]:
        """正常回位须来料空闲范围与出料后窗口同时全空。

        来料检查取空闲距离和提前定位距离中的较大者，避免大前窗口
        提前定位时被空闲回位抢占。仅空闲距离可截到激光可见边界；
        设备安全窗口必须完整落在队列中。
        """

        frame_stack = getattr(frame_queue_manager, "frame_stack", None)
        if frame_stack is None:
            # 兼容不包含真实队列的离线几何调用。
            return {
                key: False
                for key, machine in machines_by_key.items()
                if machine is not None
            }
        if not isinstance(frame_stack, Mapping):
            raise TypeError("frame_queue_manager.frame_stack 必须是映射")

        distance_frames = self.idle_return_distance_mm // CHAIN_COUNT_CM_STEP_MM
        ranges: list[tuple[tuple[str, str], str, int, int]] = []
        for key, machine in machines_by_key.items():
            if machine is None:
                continue
            side = key[1]
            if side not in frame_stack:
                raise ValueError(f"FrameQueueManager缺少{side}侧队列")
            history_index = calculate_history_index(
                machine.get("z_position"),
                CHAIN_COUNT_CM_STEP_MM,
                frame_queue_manager.stack_size,
            )
            if history_index is None:
                raise ValueError(f"{side}侧{key[0]}.z_position超出帧队列范围")
            window_start, stop, _ = machine_window_bounds(machine, frame_queue_manager.stack_size)
            start = min(window_start, max(0, history_index - distance_frames))
            ranges.append((key, side, start, stop))

        revision = getattr(frame_queue_manager, "frame_revision", None)
        if isinstance(revision, int) and not isinstance(revision, bool):
            revision_signature = revision
        else:
            revision_signature = tuple(
                (
                    key,
                    tuple(id(frame) for frame in frame_stack[side][start:stop]),
                )
                for key, side, start, stop in ranges
            )
        cache_key = (
            id(frame_queue_manager),
            revision_signature,
            self.idle_return_distance_mm,
            tuple((key, side, start, stop) for key, side, start, stop in ranges),
        )
        if cache_key == self._behind_empty_cache_key:
            return dict(self._behind_empty_cache)

        occupancy_by_id: dict[int, tuple[object, bool]] = {}

        def frame_has_workpiece(frame) -> bool:
            cached = occupancy_by_id.get(id(frame))
            if cached is not None and cached[0] is frame:
                return cached[1]
            occupied = len(self._cached_contour_rows(frame)) > 0
            occupancy_by_id[id(frame)] = (frame, occupied)
            return occupied

        result = {
            key: not any(
                frame_has_workpiece(frame) for frame in frame_stack[side][start:stop]
            )
            for key, side, start, stop in ranges
        }
        self._behind_empty_cache_key = cache_key
        self._behind_empty_cache = dict(result)
        return result

    def _advance_returning_xn_side(
        self,
        side: str,
        machine_config: Mapping,
        plc_data,
    ) -> tuple[Mapping[str, MovingAxisData] | None, tuple[str, ...]]:
        """先确认整侧X回位，再允许Y回位，全程关粉。"""

        if self._return_complete[side]:
            return None, (f"{side}侧已回安全位，设备使能已撤销",)
        commands = build_xn_side_return_commands(
            machine_config,
            clean_mode=False,
        )
        actual_positions = self._actual_xn_side_positions(plc_data, side)
        sequence = self._return_sequence[side]
        x_targets = {axis: commands[axis].Pos for axis in XN_SIDE_X_AXES}
        x_inside = all(abs(actual_positions[a] - p) <= self.position_tolerance_mm
                       for a, p in x_targets.items())
        x_changed = any(abs(p - sequence.x_arrival.targets.get(a, p))
                        > self.position_tolerance_mm for a, p in x_targets.items())
        if sequence.phase == "idle" or (sequence.phase == "y" and (not x_inside or x_changed)):
            sequence.phase = "x"
            sequence.held = {a: actual_positions[a] for a in XN_SIDE_Y_AXES}
            sequence.x_arrival = AxisGroupArrival()
            sequence.y_arrival = AxisGroupArrival()
        if sequence.phase == "x":
            if not sequence.x_arrival.observe(
                x_targets, actual_positions, self.position_tolerance_mm,
                self.x_arrival_confirmations,
            ):
                for axis in XN_SIDE_Y_AXES:
                    commands[axis] = replace(commands[axis], Pos=sequence.held[axis])
                return commands, (f"{side}侧等待X回位",)
            sequence.phase = "y"
        if sequence.y_arrival.observe(
            {a: commands[a].Pos for a in XN_SIDE_Y_AXES}, actual_positions,
            self.position_tolerance_mm, self.x_arrival_confirmations,
        ):
            self._return_complete[side] = True
            return None, (f"{side}侧全七轴已连续确认到达安全位",)
        return commands, (f"{side}侧等待Y回位",)

    def _history_indices(
        self,
        frame_queue_manager,
        machine_by_side: Mapping[str, Mapping],
    ) -> dict[str, int]:
        indices: dict[str, int] = {}
        for side in self.SIDES:
            z_position = machine_by_side[side].get("z_position")
            history_index = calculate_history_index(
                z_position,
                CHAIN_COUNT_CM_STEP_MM,
                frame_queue_manager.stack_size,
            )
            if history_index is None:
                raise ValueError(f"{side}侧z_position超出FrameQueueManager覆盖范围")
            indices[side] = history_index
        return indices

    def _build_variable_2d_observation(
        self,
        frame_queue_manager,
        side: str,
        history_index: int,
        machine_config: Mapping,
    ) -> Variable2DWindowObservation:
        start, stop, _ = machine_window_bounds(
            machine_config, frame_queue_manager.stack_size, history_index=history_index,
        )
        if side not in frame_queue_manager.frame_stack:
            raise ValueError(f"FrameQueueManager缺少{side}侧队列")
        frames = frame_queue_manager.frame_stack[side][start:stop]
        return build_variable_2d_window_observation(
            frames, machine_config, station_index=history_index - start,
            _row_extractor=self._cached_contour_rows,
        )

    def _variable_2d_observations_by_side(
        self,
        frame_queue_manager,
        fifo: int,
        machine_by_side: Mapping[str, Mapping],
    ) -> dict[str, Variable2DWindowObservation]:
        history_indices = self._history_indices(
            frame_queue_manager,
            machine_by_side,
        )
        cache_key = self._geometry_signature(
            frame_queue_manager,
            fifo,
            history_indices,
            machine_by_side,
            0,
        )
        if cache_key == self._variable_2d_cache_key and self._variable_2d_cache:
            return self._variable_2d_cache

        observations = {
            side: self._build_variable_2d_observation(
                frame_queue_manager,
                side,
                history_indices[side],
                machine_by_side[side],
            )
            for side in self.SIDES
        }
        self._variable_2d_cache_key = cache_key
        self._variable_2d_cache = observations
        return observations

    def _geometry_signature(
        self,
        frame_queue_manager,
        fifo: int,
        history_indices: Mapping[str, int],
        machine_by_side: Mapping[str, Mapping],
        y_threshold: int,
    ):
        queue_windows = []
        for side in self.SIDES:
            history_index = history_indices[side]
            start, stop, _ = machine_window_bounds(
                machine_by_side[side], frame_queue_manager.stack_size, history_index=history_index,
            )
            if side not in frame_queue_manager.frame_stack:
                raise ValueError(f"FrameQueueManager缺少{side}侧队列")
            queue_windows.append(
                (
                    side,
                    history_index,
                    tuple(
                        id(frame)
                        for frame in frame_queue_manager.frame_stack[side][start:stop]
                    ),
                )
            )
        return (
            fifo,
            y_threshold,
            tuple(queue_windows),
            _freeze_runtime_value(machine_by_side),
            self.geometry_parameters,
        )

    def _cached_contour_rows(self, frame):
        """复用不可变帧的轮廓；FrameData被替换时重新提取。"""
        key = id(frame)
        frame_data = getattr(frame, "FrameData", None)
        cached = self._contour_cache.get(key)
        if cached is not None and cached[0] is frame and cached[1] is frame_data:
            self._contour_cache.move_to_end(key)
            return cached[2]
        rows = _extract_contour_rows(frame)
        for values in (rows.height, rows.v_min, rows.v_max):
            values.flags.writeable = False
        self._contour_cache[key] = (frame, frame_data, rows)
        self._contour_cache.move_to_end(key)
        if len(self._contour_cache) > 512:
            self._contour_cache.popitem(last=False)
        return rows

    def _build_side_geometry(
        self,
        frame_queue_manager,
        side: str,
        history_index: int,
        y_threshold: int,
        machine_config: Mapping,
    ) -> RuntimeQueueGeometry:
        """从历史索引及配置的前后安全窗口构造单侧安全行程。"""

        stack = frame_queue_manager.frame_stack[side]
        start, stop, _ = machine_window_bounds(
            machine_config, len(stack), history_index=history_index,
        )
        # 队列下标越大越旧；几何序列反转为旧到新，站位位于出料窗口帧数处。
        frames = list(reversed(stack[start:stop]))
        rows_by_frame = [self._cached_contour_rows(frame) for frame in frames]
        station_index = stop - history_index - 1
        occupied = [len(rows) > 0 for rows in rows_by_frame]
        if occupied[station_index]:
            reference_index, arrival_phase = station_index, "at_station"
        else:
            reference_index = next((i for i in range(station_index + 1, len(frames))
                                    if occupied[i]), None)
            arrival_phase = "preposition" if reference_index is not None else "waiting"
        parameters = self.geometry_parameters
        edges_by_frame = _detect_frame_edge_sequence(
            rows_by_frame,
            y_threshold=y_threshold,
            minimum_x_change_mm=parameters.minimum_x_change_mm,
            minimum_run_layers=parameters.minimum_run_layers,
            minimum_horizontal_layers=parameters.minimum_horizontal_layers,
        )
        boxes_by_frame = _build_region_box_sequence(
            rows_by_frame,
            edges_by_frame,
            y_threshold=y_threshold,
            smoothing_window=parameters.smoothing_window,
            minimum_middle_coverage=parameters.minimum_middle_coverage,
            narrow_width_mm=parameters.narrow_width_mm,
            minimum_horizontal_layers=parameters.minimum_horizontal_layers,
            rail_padding_layers=parameters.rail_padding_layers,
            middle_trim_layers=parameters.middle_trim_layers,
        )
        raw_motions = [
            None if boxes is None else _build_x_axis_ranges(boxes, side, machine_config)
            for boxes in boxes_by_frame
        ]
        selected = None if reference_index is None else _leading_edge_window_indices(
            occupied, reference_index, older_at_end=False, allow_trailing_edge=True,
        )
        # 不能跨越内部空洞合并两个工件；参考帧不能用插值补成有效轮廓。
        center_motion = None
        if selected is not None:
            center_motion = _build_minimum_x_envelope_at(
                raw_motions, half_window_frames=len(frames),
                center_index=reference_index, contour_occupied=occupied,
            )
        center_boxes = None if selected is None else boxes_by_frame[reference_index]
        if center_motion is not None:
            # 与X包络共用有效窗口；复制矩形，避免改变原始帧几何。
            center_boxes = replace(
                center_boxes,
                lower=replace(
                    center_boxes.lower,
                    y_min=min(boxes_by_frame[i].lower.y_min for i in selected),
                ),
            )
        return RuntimeQueueGeometry(
            boxes=center_boxes,
            motions=center_motion,
            rows_by_frame=tuple(rows_by_frame),
            leading_edge=center_motion is not None and not occupied[0],
            trailing_edge=center_motion is not None and not occupied[-1],
            arrival_phase=arrival_phase, station_index=station_index,
            reference_index=reference_index,
        )

    def _geometry_by_side(
        self,
        frame_queue_manager,
        fifo: int,
        machine_by_side: Mapping[str, Mapping],
        y_threshold: int,
    ) -> dict[str, RuntimeQueueGeometry]:
        history_indices = self._history_indices(
            frame_queue_manager,
            machine_by_side,
        )
        cache_key = self._geometry_signature(
            frame_queue_manager,
            fifo,
            history_indices,
            machine_by_side,
            y_threshold,
        )
        if cache_key == self._geometry_cache_key and self._geometry_cache:
            return self._geometry_cache

        geometry = {
            side: self._build_side_geometry(
                frame_queue_manager,
                side,
                history_indices[side],
                y_threshold,
                machine_by_side[side],
            )
            for side in self.SIDES
        }
        self._geometry_cache_key = cache_key
        self._geometry_cache = geometry
        return geometry

    def _calculate_side_y_targets(
        self,
        boxes,
        side: str,
        machine_config: Mapping,
    ) -> dict[str, int]:
        points = _rectangle_x_reciprocation_start_points(boxes, side)
        origin_pos = machine_config.get("origin_pos")
        return calculate_xn_side_y_targets(
            in_up_y=_round_runtime_mm(points["in_up"].height),
            in_down_y=_round_runtime_mm(points["in_down"].height),
            out_down_y=_round_runtime_mm(points["out_down"].height),
            origin_pos=origin_pos,
            machine_config=machine_config,
            minimum_gun_y_distance_mm=self.minimum_gun_y_distance_mm,
        )

    def _prepare_side_geometry(self, geometry, machine_config, side):
        """轮廓或配置变化时重算安全目标，其余周期只推进位置状态机。"""
        signature = (
            _freeze_runtime_value(machine_config),
            _freeze_runtime_value(self.last_y_targets[side]),
            self.minimum_gun_y_distance_mm,
            self.geometry_parameters.gun_obstacle_half_height_mm,
        )
        cached = self._prepared_geometry_cache.get(side)
        if cached is not None and cached[0] is geometry and cached[1] == signature:
            return cached[2]
        prepared = self._compute_side_geometry_targets(geometry, machine_config, side)
        # 保留原对象强引用；每侧仅缓存一个不可变轮廓，避免id复用和积累。
        self._prepared_geometry_cache[side] = (geometry, signature, prepared)
        return prepared

    def _compute_side_geometry_targets(self, geometry, machine_config, side):
        if geometry.boxes is None:
            return replace(geometry, targets_prepared=True)
        reasons = []
        targets = self.last_y_targets[side]
        y_targets_valid = True
        try:
            targets = self._calculate_side_y_targets(geometry.boxes, side, machine_config)
        except (KeyError, TypeError, ValueError) as exc:
            y_targets_valid = False
            reasons.append(f"{side}侧Y目标无效：{exc}")
        motions = geometry.motions
        constraints = ()
        if targets is None:
            motions = None
        elif geometry.rows_by_frame:
            motions, constraints = _apply_gun_obstacle_constraints(
                motions, geometry.rows_by_frame,
                (geometry.station_index if geometry.station_index is not None
                 else len(geometry.rows_by_frame) // 2),
                targets, machine_config, self.geometry_parameters.gun_obstacle_half_height_mm,
            )
        else:
            # 缺少原始窗口时不能绕过障碍检查。
            motions = None
            reasons.append(f"{side}侧缺少障碍检查原始轮廓窗口")
        return replace(
            geometry, motions=motions, y_targets=targets,
            obstacle_constraints=constraints, target_reasons=tuple(reasons),
            targets_prepared=True, y_targets_valid=y_targets_valid,
        )

    def _hold_initial_position(self, side, actual_positions):
        sequence = self._positioning[side]
        if sequence.phase != "invalid":
            self._reset_side(side)
            sequence = self._positioning[side]
            sequence.phase = "invalid"
            sequence.held = dict(actual_positions)
        return {
            axis: MovingAxisData(Pos=sequence.held[axis], Speed=0, Status=0)
            for axis in XN_SIDE_AXIS_SEQUENCE
        }

    def _advance_initial_position(self, side, targets, y_targets, actual, tolerance, *, allow_spray=True):
        """首次Y→X组间屏障；完成前不推进单轴往复状态机。"""
        sequence = self._positioning[side]
        if sequence.phase in ("idle", "invalid"):
            sequence = SidePositioningSequence(
                phase="y", held={a: actual[a] for a in XN_SIDE_X_AXES},
            )
            self._positioning[side] = sequence
        y_inside = all(abs(actual[a] - p) <= tolerance for a, p in y_targets.items())
        y_changed = any(abs(p - sequence.y_arrival.targets[a]) > tolerance
                        for a, p in y_targets.items()) if sequence.phase == "x" else False
        if sequence.phase == "x" and (not y_inside or y_changed):
            sequence.phase = "y"
            sequence.held = {a: actual[a] for a in XN_SIDE_X_AXES}
            sequence.y_arrival = AxisGroupArrival()
            sequence.x_arrival = AxisGroupArrival()
            sequence.endpoint_names.clear()
        if sequence.phase == "y":
            if not sequence.y_arrival.observe(
                y_targets, actual, tolerance, self.x_arrival_confirmations,
            ):
                return {
                    a: XAxisMotionCommand(sequence.held[a], 0, 0, "positioning_y")
                    for a in XN_SIDE_X_AXES
                }, "等待Y首次定位"
            sequence.phase = "x"

        positions = {}
        for axis, target in targets.items():
            if isinstance(target, FixedXAxisTarget):
                sequence.endpoint_names[axis] = "position"
                positions[axis] = target.position
            else:
                endpoint = sequence.endpoint_names.get(axis)
                lower, upper = sorted((target.start_pos, target.end_pos))
                if actual[axis] < lower - tolerance or actual[axis] > upper + tolerance:
                    nearest = FeedbackXAxisMotion._nearest_endpoint(target, actual[axis])
                    endpoint = "end_pos" if nearest == target.end_pos else "start_pos"
                elif endpoint not in ("start_pos", "end_pos"):
                    endpoint = "start_pos"
                sequence.endpoint_names[axis] = endpoint
                positions[axis] = getattr(target, endpoint)
        if not sequence.x_arrival.observe(
            positions, actual, tolerance, self.x_arrival_confirmations,
        ):
            return {
                axis: XAxisMotionCommand(
                    positions[axis],
                    targets[axis].position_speed,
                    0,
                    PHASE_POSITIONING_TO_FIXED
                    if isinstance(targets[axis], FixedXAxisTarget)
                    else PHASE_POSITIONING_TO_START,
                )
                for axis in XN_SIDE_X_AXES
            }, "等待X首次定位"

        if not allow_spray:
            return {
                axis: XAxisMotionCommand(positions[axis], 0, 0, "preposition_ready")
                for axis in XN_SIDE_X_AXES
            }, "提前定位完成，等待工件实际到站"

        # 统一放行后从已确认的端点向另一端运动，固定点直接进入喷涂。
        for axis, target in targets.items():
            state = self.x_planners[side].states[axis]
            state.initialized = True
            if isinstance(target, FixedXAxisTarget):
                state.phase = PHASE_FIXED
                state._target_mode = PHASE_FIXED
                state._fixed_target = target.position
            else:
                state._target_mode = "reciprocating"
                state.direction = (DIRECTION_TO_END
                                   if sequence.endpoint_names[axis] == "start_pos"
                                   else DIRECTION_TO_START)
                state.phase = state.direction
        sequence.phase = "complete"
        self.motion_started[side] = True
        return None, None

    def _advance_side(
        self,
        geometry: RuntimeQueueGeometry,
        machine_config: Mapping,
        side: str,
        actual_positions: Mapping[str, int],
        position_tolerance_mm: int,
    ):
        reasons: list[str] = []
        self._last_x_steps[side] = None
        settings = machine_window_distances(machine_config)
        old_settings = self._window_settings.get(side)
        previous_phase = self._arrival_phase[side]
        if ((old_settings is not None and settings != old_settings)
                or (previous_phase == "at_station" and geometry.arrival_phase != "at_station")
                or (previous_phase == "waiting" and geometry.arrival_phase != "waiting")):
            self._reset_side(side)
        self._window_settings[side] = settings
        self._arrival_phase[side] = geometry.arrival_phase
        if geometry.arrival_phase == "waiting":
            return self._hold_initial_position(side, actual_positions), (
                f"{side}侧离站或等待来料，保持位置并关粉，等待收枪条件",
            )
        if geometry.leading_edge:
            reasons.append(f"{side}侧前缘进入安全窗口，使用部分有效安全窗口定位")
        if geometry.trailing_edge:
            reasons.append(f"{side}侧尾缘经过，使用剩余有效安全窗口规划")
        if geometry.boxes is None:
            return self._hold_initial_position(side, actual_positions), (
                f"{side}侧工件尚未到站或已经离站，保持位置并关粉",
            )

        if not geometry.targets_prepared:
            geometry = self._prepare_side_geometry(geometry, machine_config, side)
        reasons.extend(geometry.target_reasons)
        last_y_targets = geometry.y_targets
        self.last_y_targets[side] = last_y_targets
        if last_y_targets is None or (
            not self.motion_started[side] and not geometry.y_targets_valid
        ):
            reasons.append(f"{side}侧尚无有效Y目标")
            return self._hold_initial_position(side, actual_positions), tuple(reasons)

        targets = {}
        hold_reasons = {}
        for axis_name in XN_SIDE_X_AXES:
            if geometry.motions is None:
                targets[axis_name] = None
                hold_reasons[axis_name] = f"出料{settings[0]}mm/来料{settings[1]}mm安全包络无效"
                continue
            motion = geometry.motions[axis_name]
            if motion.has_out_of_limit_position:
                reason = "本帧端点超限，保持上一位置"
                reasons.append(f"{side}侧{axis_name}：{reason}")
            else:
                reason = None
            hold_reasons[axis_name] = reason
            if reason is not None:
                targets[axis_name] = None
            elif abs(motion.end_pos - motion.start_pos) <= self.parameters.short_stroke_threshold_mm:
                detected_x = (
                    motion.window_min_detected_x
                    if motion.window_min_detected_x is not None
                    else min(motion.detected_start_x, motion.detected_end_x)
                )
                # 左右设备的所有枪共用独立的短行程固定点偏移。
                fixed_position = (
                    detected_x
                    - _require_int("x_position", machine_config.get("x_position"))
                    - self.parameters.short_stroke_x_offset_mm
                )
                obstacle = next((item for item in geometry.obstacle_constraints
                                 if item.axis_name == axis_name), None)
                if obstacle is not None and obstacle.limit_pos is not None:
                    fixed_position = min(fixed_position, obstacle.limit_pos)
                if not motion.min_limit_pos <= fixed_position <= motion.max_limit_pos:
                    targets[axis_name] = None
                    hold_reasons[axis_name] = "短行程固定点超限，保持当前位置并关枪"
                    reasons.append(f"{side}侧{axis_name}：{hold_reasons[axis_name]}")
                    continue
                targets[axis_name] = FixedXAxisTarget(
                    position=fixed_position,
                    position_speed=motion.position_speed,
                    min_limit_pos=motion.min_limit_pos,
                    max_limit_pos=motion.max_limit_pos,
                )
            else:
                targets[axis_name] = FeedbackXAxisTarget(
                    start_pos=motion.start_pos,
                    end_pos=motion.end_pos,
                    position_speed=motion.position_speed,
                    reciprocation_speed=motion.reciprocation_speed,
                    min_limit_pos=motion.min_limit_pos,
                    max_limit_pos=motion.max_limit_pos,
                )

        if not self.motion_started[side]:
            if any(
                targets[axis] is None or hold_reasons[axis] for axis in XN_SIDE_X_AXES
            ):
                reasons.append(f"{side}侧尚无可初始化的四轴安全行程")
                return self._hold_initial_position(side, actual_positions), tuple(reasons)
            initial_commands, initial_reason = self._advance_initial_position(
                side, targets, last_y_targets, actual_positions, position_tolerance_mm,
                allow_spray=geometry.arrival_phase == "at_station",
            )
            if initial_commands is not None:
                reasons.append(f"{side}侧{initial_reason}")
                self._last_x_steps[side] = XnSideFeedbackXStep(
                    commands=initial_commands,
                    snapshots={
                        a: replace(
                            self.x_planners[side].states[a]._snapshot(
                                actual_pos=actual_positions[a], target_pos=c.Pos,
                            ),
                            phase=c.Phase, status=0,
                            held=self._positioning[side].phase == "y",
                            hold_reason=initial_reason,
                            fixed_target_pos=(c.Pos if isinstance(targets[a], FixedXAxisTarget)
                                              and self._positioning[side].phase == "x" else None),
                        ) for a, c in initial_commands.items()
                    },
                )
                return build_xn_side_axis_commands(
                    initial_commands, last_y_targets, machine_config,
                ), tuple(reasons)
        step = self.x_planners[side].advance(
            targets,
            {a: actual_positions[a] for a in XN_SIDE_X_AXES},
            position_tolerance_mm=position_tolerance_mm,
            hold_reasons=hold_reasons,
        )
        self._last_x_steps[side] = step
        self.motion_started[side] = True

        if last_y_targets is None:
            reasons.append(f"{side}侧尚无有效Y目标")
            return None, tuple(reasons)
        try:
            commands = build_xn_side_axis_commands(
                step.commands,
                last_y_targets,
                machine_config,
            )
        except (KeyError, TypeError, ValueError) as exc:
            reasons.append(f"{side}侧7轴构造失败：{exc}")
            commands = None
        return commands, tuple(reasons)

    def _build_debug_snapshot(
        self,
        *,
        heartbeat: int,
        fifo: int,
        frame: SendMovingFrameData,
        geometry: Mapping[str, RuntimeQueueGeometry],
        actual_x_positions: Mapping[str, Mapping[str, int]],
        side_commands: Mapping[str, Mapping | None],
        side_reasons: Mapping[str, tuple[str, ...]],
        variable_2d_observations: Mapping[str, Variable2DWindowObservation],
        variable_2d_commands: Mapping[str, MovingAxisData | None],
    ) -> MotionPlanningDebugSnapshot:
        xn_diagnostics: list[XnSidePlanningDiagnostic] = []
        for side in self.SIDES:
            axes: list[XAxisPlanningDiagnostic] = []
            step = self._last_x_steps[side]
            motions = geometry[side].motions
            for axis_name in XN_SIDE_X_AXES:
                motion = (
                    motions.get(axis_name) if isinstance(motions, Mapping) else None
                )
                command = None if step is None else step.commands[axis_name]
                state = None if step is None else step.snapshots[axis_name]
                obstacle = next((item for item in geometry[side].obstacle_constraints
                                 if item.axis_name == axis_name), None)
                axes.append(
                    XAxisPlanningDiagnostic(
                        axis_name=axis_name,
                        obstacle_constraint=obstacle,
                        detected_start_x=(
                            None if motion is None else motion.detected_start_x
                        ),
                        detected_end_x=(
                            None if motion is None else motion.detected_end_x
                        ),
                        safe_start_pos=(None if motion is None else motion.start_pos),
                        safe_end_pos=(None if motion is None else motion.end_pos),
                        actual_pos=actual_x_positions[side][axis_name],
                        command_pos=None if command is None else command.Pos,
                        command_speed=None if command is None else command.Speed,
                        phase=None if state is None else state.phase,
                        held=False if state is None else state.held,
                        hold_reason=(None if state is None else state.hold_reason),
                        fixed_target_pos=(
                            None if state is None else state.fixed_target_pos
                        ),
                    )
                )
            y_targets = self.last_y_targets[side]
            xn_diagnostics.append(
                XnSidePlanningDiagnostic(
                    side=side,
                    boxes=geometry[side].boxes,
                    axes=tuple(axes),
                    y_targets=(
                        ()
                        if y_targets is None
                        else tuple(
                            (name, int(y_targets[name])) for name in XN_SIDE_Y_AXES
                        )
                    ),
                    active=side_commands[side] is not None,
                    reasons=side_reasons[side],
                )
            )

        out_lift_diagnostics: list[Variable2DPlanningDiagnostic] = []
        for side in self.SIDES:
            observation = variable_2d_observations[side]
            command = variable_2d_commands[side]
            out_lift_diagnostics.append(
                Variable2DPlanningDiagnostic(
                    side=side,
                    state=observation.state,
                    x_pos=None if command is None else command.Pos,
                    y_pos_min=None if command is None else command.Speed,
                    y_pos_max=None if command is None else command.Status,
                    active=command is not None,
                )
            )
        return MotionPlanningDebugSnapshot(
            heartbeat=heartbeat,
            fifo=fifo,
            safe=False,
            enable=frame.Enable,
            operate=frame.Operate,
            xn_sides=tuple(xn_diagnostics),
            out_lifts=tuple(out_lift_diagnostics),
            active_sides=self.last_active_sides,
            active_2d_sides=self.last_active_2d_sides,
            reasons=self.last_reasons,
            error=None,
        )

    def _record_override_frame(
        self,
        *,
        heartbeat: int,
        fifo: int,
        frame: SendMovingFrameData,
        reasons: Sequence[str],
        active_sides: Sequence[str],
        active_2d_sides: Sequence[str],
    ) -> None:
        """记录清理/回位覆盖帧的精简诊断。"""

        self.last_reasons = tuple(reasons)
        self.last_error = None
        self.last_frame_was_safe = False
        self.last_active_sides = tuple(active_sides)
        self.last_active_2d_sides = tuple(active_2d_sides)
        self.last_debug_snapshot = MotionPlanningDebugSnapshot(
            heartbeat=heartbeat,
            fifo=fifo,
            safe=False,
            enable=frame.Enable,
            operate=frame.Operate,
            xn_sides=(),
            out_lifts=(),
            active_sides=self.last_active_sides,
            active_2d_sides=self.last_active_2d_sides,
            reasons=self.last_reasons,
            error=None,
        )

    def _advance_out_lift(
        self,
        side: str,
        machine_config: Mapping,
        observation: Variable2DWindowObservation,
        idle_return: bool,
    ) -> tuple[MovingAxisData | None, tuple[str, ...]]:
        """处理单侧二维设备的空闲回位、窗口推进及诊断原因。"""
        reasons: list[str] = []
        if observation.leading_edge:
            reasons.append(f"{side}侧2D前缘进入安全窗口，提前下发定位范围")
        if idle_return != self._out_lift_return_active[side]:
            self.variable_2d_states[side] = Variable2DReportStateMachine()
            self._out_lift_return_active[side] = idle_return
        if idle_return:
            command = build_out_lift_return_command(machine_config, clean_mode=False)
            reasons.append(
                f"{side}侧2D来料方向{self.idle_return_distance_mm}mm及出料后窗口"
                "均无工件，持续保持安全位"
            )
        else:
            waiting = observation.state == VARIABLE_2D_WINDOW_EMPTY
            command = self.variable_2d_states[side].advance(
                Variable2DWindowObservation(VARIABLE_2D_WINDOW_MIXED) if waiting else observation
            )
            if observation.state == VARIABLE_2D_WINDOW_EMPTY:
                reasons.append(f"{side}侧2D等待来料或延后收枪，保持上一有效范围")
            elif observation.state == VARIABLE_2D_WINDOW_MIXED:
                if command is None:
                    reasons.append(f"{side}侧2D安全窗口夹有空帧，尚无有效报文可保持")
                else:
                    reasons.append(f"{side}侧2D安全窗口夹有空帧，保持上一有效报文")
        return command, tuple(reasons)

    def build_moving_frame(
        self,
        *,
        strategy_name: str,
        frame_queue_manager,
        plc_data,
        lidar_status: int,
        raw_data_timeout_active: bool,
        machine_config: Mapping,
        read_data_config: Mapping,
    ) -> SendMovingFrameData:
        """从运行时队列生成14轴及左右2D变频命令；不连接或发送PLC。"""

        heartbeat = getattr(plc_data, "HeartBeat", 0)
        if strategy_name != "frame_by_frame":
            return self._safe_frame(heartbeat, "当前规划器只支持frame_by_frame")
        if frame_queue_manager is None or plc_data is None:
            self._handle_clean_mode_transition(False)
            self._record_clean_gate("逐帧队列或PLC数据缺失，禁止清理")
            return self._safe_frame(heartbeat, "逐帧队列或PLC链条数据尚未初始化")

        try:
            operate_word = self._plc_operate_word(plc_data)
            clean_requested = bool(operate_word & PLC_OPERATE_CLEAN_MODE)
            if int(lidar_status or 0) in (1, 2, 3):
                self._handle_clean_mode_transition(False)
                self._record_clean_gate("激光异常，禁止清理")
                return self._safe_frame(heartbeat, f"激光状态异常：{lidar_status}")
            if raw_data_timeout_active:
                self._handle_clean_mode_transition(False)
                self._record_clean_gate("激光逐帧数据超时，禁止清理")
                return self._safe_frame(heartbeat, "激光逐帧数据超时")

            if not isinstance(machine_config, Mapping):
                raise TypeError("machine_config 必须是映射")
            if not isinstance(read_data_config, Mapping):
                raise TypeError("read_data_config 必须是映射")

            clean_mode = clean_requested and self._clean_queue_empty(
                frame_queue_manager, read_data_config
            )
            self._handle_clean_mode_transition(clean_mode)
            clean_gate_state = (
                "全队列无工件，进入清理" if clean_mode else
                "队列内有工件，继续正常生产并等待清空" if clean_requested else
                "无清理请求"
            )

            fifo = getattr(plc_data, "ChainCountCM", None)
            if isinstance(fifo, bool) or not isinstance(fifo, int):
                raise TypeError("plc_data.ChainCountCM 必须是整数")
            y_threshold = read_data_config.get("y_threshold", 10)
            if isinstance(y_threshold, bool) or not isinstance(y_threshold, int):
                raise TypeError("ReadDataConfig.y_threshold 必须是整数")
            if y_threshold <= 0:
                raise ValueError("ReadDataConfig.y_threshold 必须大于0")
            position_tolerance_mm = self.position_tolerance_mm
            machine_by_side = {
                side: self._find_xn_side_machine(machine_config, side)
                for side in self.SIDES
            }
            out_lift_by_side = {
                side: self._find_out_lift_machine(machine_config, side)
                for side in self.SIDES
            }
            configured_out_lift_sides = tuple(
                side for side in self.SIDES if out_lift_by_side[side] is not None
            )
            if configured_out_lift_sides and len(configured_out_lift_sides) != 2:
                raise ValueError("帧模式的out_lift必须左右两侧成对配置")

            if clean_mode:
                side_commands = {
                    side: build_xn_side_return_commands(
                        machine_by_side[side],
                        clean_mode=True,
                    )
                    for side in self.SIDES
                }
                variable_2d_commands = {
                    side: (
                        None
                        if out_lift_by_side[side] is None
                        else build_out_lift_return_command(
                            out_lift_by_side[side],
                            clean_mode=True,
                        )
                    )
                    for side in self.SIDES
                }
                frame = build_available_xn_side_moving_frame(
                    side_commands["left"],
                    side_commands["right"],
                    heartbeat,
                    chain_running=False,
                    left_2d_command=variable_2d_commands["left"],
                    right_2d_command=variable_2d_commands["right"],
                )
                frame.Enable |= ENABLE_CLEAN_MODE
                self._record_clean_gate(clean_gate_state)
                reasons = ("PLC轴清理模式：Y轴回安全位，X轴到最大位，请求停链",)
                self._record_override_frame(
                    heartbeat=heartbeat,
                    fifo=fifo,
                    frame=frame,
                    reasons=reasons,
                    active_sides=self.SIDES,
                    active_2d_sides=configured_out_lift_sides,
                )
                return frame

            machines_for_empty_check = {
                ("xn_side", side): machine_by_side[side] for side in self.SIDES
            }
            machines_for_empty_check.update(
                {("out_lift", side): out_lift_by_side[side] for side in self.SIDES}
            )
            behind_empty = self._behind_empty_by_machine(
                frame_queue_manager,
                machines_for_empty_check,
            )
            geometry = dict(self._geometry_by_side(
                frame_queue_manager,
                fifo,
                machine_by_side,
                y_threshold,
            ))
            variable_2d_observations = (
                self._variable_2d_observations_by_side(
                    frame_queue_manager,
                    fifo,
                    out_lift_by_side,
                )
                if configured_out_lift_sides
                else {
                    side: Variable2DWindowObservation(VARIABLE_2D_WINDOW_EMPTY)
                    for side in self.SIDES
                }
            )
            actual_x_positions = {
                side: self._actual_xn_side_positions(plc_data, side) for side in self.SIDES
            }
            side_commands = {}
            variable_2d_commands = {}
            reasons_by_side: dict[str, tuple[str, ...]] = {}
            reasons: list[str] = []
            for side in self.SIDES:
                plc_enable_bit = (
                    PLC_OPERATE_LEFT_XN_SIDE
                    if side == "left"
                    else PLC_OPERATE_RIGHT_XN_SIDE
                )
                device_disabled = not bool(operate_word & plc_enable_bit)
                idle_return = behind_empty.get(("xn_side", side), False)
                return_triggered = device_disabled or idle_return or (
                    self._return_triggered[side] and not self._return_complete[side]
                )
                if return_triggered != self._return_triggered[side]:
                    self._reset_side(side)
                    self._reset_return_state(side)
                    self._return_triggered[side] = return_triggered
                    self._return_complete[side] = False

                if return_triggered:
                    side_commands[side], side_reasons = self._advance_returning_xn_side(
                        side,
                        machine_by_side[side],
                        plc_data,
                    )
                    trigger_reasons = []
                    if device_disabled:
                        trigger_reasons.append(f"{side}侧PLC独立设备使能已关闭")
                    if idle_return:
                        trigger_reasons.append(
                            f"{side}侧来料方向{self.idle_return_distance_mm}mm及出料后窗口均无工件"
                        )
                    side_reasons = tuple(trigger_reasons) + tuple(side_reasons)
                else:
                    geometry[side] = self._prepare_side_geometry(
                        geometry[side], machine_by_side[side], side,
                    )
                    side_commands[side], side_reasons = self._advance_side(
                        geometry[side],
                        machine_by_side[side],
                        side,
                        actual_x_positions[side],
                        position_tolerance_mm,
                    )
                reasons_by_side[side] = side_reasons
                reasons.extend(side_reasons)
                if configured_out_lift_sides:
                    variable_2d_commands[side], out_lift_reasons = self._advance_out_lift(
                        side,
                        out_lift_by_side[side],
                        variable_2d_observations[side],
                        behind_empty.get(("out_lift", side), False),
                    )
                    reasons.extend(out_lift_reasons)
                else:
                    variable_2d_commands[side] = None

            frame = build_available_xn_side_moving_frame(
                side_commands["left"],
                side_commands["right"],
                heartbeat,
                left_2d_command=variable_2d_commands["left"],
                right_2d_command=variable_2d_commands["right"],
            )
            self._record_clean_gate(clean_gate_state)
            self.last_reasons = tuple(reasons)
            self.last_error = None
            self.last_frame_was_safe = False
            self.last_active_sides = tuple(
                side for side in self.SIDES if side_commands[side] is not None
            )
            self.last_active_2d_sides = tuple(
                side for side in self.SIDES if variable_2d_commands[side] is not None
            )
            self.last_debug_snapshot = self._build_debug_snapshot(
                heartbeat=heartbeat,
                fifo=fifo,
                frame=frame,
                geometry=geometry,
                actual_x_positions=actual_x_positions,
                side_commands=side_commands,
                side_reasons=reasons_by_side,
                variable_2d_observations=variable_2d_observations,
                variable_2d_commands=variable_2d_commands,
            )
            return frame
        except Exception as exc:
            self._handle_clean_mode_transition(False)
            self._record_clean_gate("数据或规划异常，禁止清理")
            return self._safe_frame(
                heartbeat,
                "逐帧运动规划异常，输出安全零帧",
                error=exc,
            )

    @staticmethod
    def build_safe_frame(heartbeat: int = 0) -> SendMovingFrameData:
        """生成全停、关枪、无使能的安全静止帧。

        心跳只透传，不在这里累加，避免规划器同时承担通信状态管理。
        """
        if isinstance(heartbeat, bool) or not isinstance(heartbeat, int):
            raise TypeError("heartbeat 必须是整数")
        if not HEARTBEAT_MIN <= heartbeat <= HEARTBEAT_MAX:
            raise ValueError(f"heartbeat 必须在 {HEARTBEAT_MIN}~{HEARTBEAT_MAX} 范围内")

        return SendMovingFrameData(
            Enable=0,
            Gun_Cont1=0,
            Gun_Cont2=0,
            HeartBeat=heartbeat,
            Operate=0,
        )
