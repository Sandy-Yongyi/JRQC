"""几何、运行状态与诊断的数据合同。"""

from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np

from model.plc.MovingFrameData import AxisData as MovingAxisData


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


@dataclass(frozen=True)
class Variable2DWindowObservation:
    """一台2D变频设备在本次安全窗口中的边界结果。"""

    state: str
    command: MovingAxisData | None = None
    leading_edge: bool = False


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
