"""仅依赖实际轴位置的 X 轴反馈状态机。"""

from collections.abc import Mapping
from dataclasses import dataclass

from model.motionplan._validation import (
    _require_int,
)
from model.motionplan.constants import (
    DIRECTION_TO_END,
    DIRECTION_TO_START,
    PHASE_FIXED,
    PHASE_HOLD,
    PHASE_POSITIONING_TO_FIXED,
    PHASE_POSITIONING_TO_START,
    PHASE_RECOVERING,
    PHASE_RETURNING_INSIDE,
    PHASE_TO_END,
    PHASE_TO_START,
)


@dataclass(frozen=True)
class XAxisMotionCommand:
    """反馈状态机生成的单根X轴命令。"""

    Pos: int
    Speed: int
    Status: int
    Phase: str


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
