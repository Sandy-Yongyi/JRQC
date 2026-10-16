"""逐帧规划运行时：协调队列、反馈、定位联锁与安全输出。

历史导入路径保留并显式导出原有符号；新代码请从各职责模块导入。"""

from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import replace
from typing import Sequence
import math

from model.motionplan._validation import (
    _freeze_runtime_value as _freeze_runtime_value,
    _require_int as _require_int,
    _round_runtime_mm as _round_runtime_mm,
)
# 显式同名导入保留历史兼容接口；Pyflakes 仍将模块级重导出视为未使用。
# 仅这些兼容导出块忽略 F401，不限制其他源码检查或改变原有 import * 接口。
from model.motionplan.commands import (  # noqa: F401
    _xn_side_config_axis_value as _xn_side_config_axis_value,
    apply_xn_side_surface_y_offsets as apply_xn_side_surface_y_offsets,
    build_available_xn_side_moving_frame as build_available_xn_side_moving_frame,
    build_dual_xn_side_moving_frame as build_dual_xn_side_moving_frame,
    build_out_lift_return_command as build_out_lift_return_command,
    build_xn_side_axis_commands as build_xn_side_axis_commands,
    build_xn_side_return_commands as build_xn_side_return_commands,
    calculate_fixed_gun_sample_y as calculate_fixed_gun_sample_y,
    calculate_out_front_target_x as calculate_out_front_target_x,
    calculate_xn_side_y_targets as calculate_xn_side_y_targets,
    limit_xn_side_y_targets_for_clearance as limit_xn_side_y_targets_for_clearance,
    select_out_front_detected_x as select_out_front_detected_x,
    validate_xn_side_gun_origin_y as validate_xn_side_gun_origin_y,
)
from model.motionplan.constants import (  # noqa: F401
    CHAIN_COUNT_CM_STEP_MM as CHAIN_COUNT_CM_STEP_MM,
    DIRECTION_TO_END as DIRECTION_TO_END,
    DIRECTION_TO_START as DIRECTION_TO_START,
    ENABLE_CLEAN_MODE as ENABLE_CLEAN_MODE,
    ENABLE_LEFT_XN_SIDE as ENABLE_LEFT_XN_SIDE,
    ENABLE_MASTER as ENABLE_MASTER,
    ENABLE_RIGHT_XN_SIDE as ENABLE_RIGHT_XN_SIDE,
    HEARTBEAT_MAX as HEARTBEAT_MAX,
    HEARTBEAT_MIN as HEARTBEAT_MIN,
    OPERATE_CHAIN_RUN as OPERATE_CHAIN_RUN,
    OPERATE_RESERVED as OPERATE_RESERVED,
    OUT_FRONT_SURFACE_FIELD_BY_DIRECTION as OUT_FRONT_SURFACE_FIELD_BY_DIRECTION,
    OUT_LIFT_LIMIT_INDEX as OUT_LIFT_LIMIT_INDEX,
    PHASE_FIXED as PHASE_FIXED,
    PHASE_HOLD as PHASE_HOLD,
    PHASE_POSITIONING_TO_FIXED as PHASE_POSITIONING_TO_FIXED,
    PHASE_POSITIONING_TO_START as PHASE_POSITIONING_TO_START,
    PHASE_RECOVERING as PHASE_RECOVERING,
    PHASE_RETURNING_INSIDE as PHASE_RETURNING_INSIDE,
    PHASE_TO_END as PHASE_TO_END,
    PHASE_TO_START as PHASE_TO_START,
    PLC_OPERATE_CLEAN_MODE as PLC_OPERATE_CLEAN_MODE,
    PLC_OPERATE_LEFT_XN_SIDE as PLC_OPERATE_LEFT_XN_SIDE,
    PLC_OPERATE_RIGHT_XN_SIDE as PLC_OPERATE_RIGHT_XN_SIDE,
    VARIABLE_2D_WINDOW_EMPTY as VARIABLE_2D_WINDOW_EMPTY,
    VARIABLE_2D_WINDOW_MIXED as VARIABLE_2D_WINDOW_MIXED,
    VARIABLE_2D_WINDOW_VALID as VARIABLE_2D_WINDOW_VALID,
    XN_SIDE_AXIS_SEQUENCE as XN_SIDE_AXIS_SEQUENCE,
    XN_SIDE_GUN_SURFACE_SEQUENCE as XN_SIDE_GUN_SURFACE_SEQUENCE,
    XN_SIDE_SURFACE_Y_OFFSET_RULES as XN_SIDE_SURFACE_Y_OFFSET_RULES,
    XN_SIDE_X_AXES as XN_SIDE_X_AXES,
    XN_SIDE_Y_AXES as XN_SIDE_Y_AXES,
    X_AXIS_OFFSET_KEYS as X_AXIS_OFFSET_KEYS,
)
from model.motionplan.contours import (  # noqa: F401
    _build_region_box_sequence as _build_region_box_sequence,
    _build_three_region_boxes as _build_three_region_boxes,
    _detect_frame_edge_candidates as _detect_frame_edge_candidates,
    _detect_frame_edge_sequence as _detect_frame_edge_sequence,
    _extract_contour_rows as _extract_contour_rows,
    _folded_terminal_rail_indices as _folded_terminal_rail_indices,
    _isolated_rail_rows as _isolated_rail_rows,
    _last_true_run_start as _last_true_run_start,
    _median_smooth as _median_smooth,
    _split_contiguous_rows as _split_contiguous_rows,
    _stabilize_numeric_track as _stabilize_numeric_track,
    _validate_edge_detection_parameters as _validate_edge_detection_parameters,
)
from model.motionplan.contracts import (  # noqa: F401
    AxisGroupArrival as AxisGroupArrival,
    ContourRows as ContourRows,
    DetectedXAxisStroke as DetectedXAxisStroke,
    EdgeCandidate as EdgeCandidate,
    FrameEdgeCandidates as FrameEdgeCandidates,
    GunObstacleConstraint as GunObstacleConstraint,
    MotionPlanningDebugSnapshot as MotionPlanningDebugSnapshot,
    PlannedXAxisRange as PlannedXAxisRange,
    RegionBox as RegionBox,
    RuntimeQueueGeometry as RuntimeQueueGeometry,
    SidePositioningSequence as SidePositioningSequence,
    ThreeRegionBoxes as ThreeRegionBoxes,
    Variable2DPlanningDiagnostic as Variable2DPlanningDiagnostic,
    Variable2DWindowObservation as Variable2DWindowObservation,
    XAxisPlanningDiagnostic as XAxisPlanningDiagnostic,
    XnSidePlanningDiagnostic as XnSidePlanningDiagnostic,
)
from model.motionplan.feedback import (  # noqa: F401
    FeedbackXAxisMotion as FeedbackXAxisMotion,
    FeedbackXAxisSnapshot as FeedbackXAxisSnapshot,
    FeedbackXAxisTarget as FeedbackXAxisTarget,
    FixedXAxisTarget as FixedXAxisTarget,
    XAxisMotionCommand as XAxisMotionCommand,
    XnSideFeedbackXPlanner as XnSideFeedbackXPlanner,
    XnSideFeedbackXStep as XnSideFeedbackXStep,
)
from model.motionplan.geometry import (  # noqa: F401
    _apply_gun_obstacle_constraints as _apply_gun_obstacle_constraints,
    _apply_window_bottom,
    _build_minimum_x_envelope as _build_minimum_x_envelope,
    _build_minimum_x_envelope_at as _build_minimum_x_envelope_at,
    _build_x_axis_ranges as _build_x_axis_ranges,
    _obstacle_min_x_at_height as _obstacle_min_x_at_height,
    _rectangle_x_axis_strokes as _rectangle_x_axis_strokes,
    _rectangle_x_reciprocation_end_points as _rectangle_x_reciprocation_end_points,
    _rectangle_x_reciprocation_start_points as _rectangle_x_reciprocation_start_points,
    _x_config_value as _x_config_value,
    calculate_history_index as calculate_history_index,
    select_history_frame as select_history_frame,
)
from model.motionplan.MachineAxisMap import get_axis_map, get_x_axes
from model.motionplan.parameters import (  # noqa: F401
    FrameByFramePlanningParameters as FrameByFramePlanningParameters,
    FrameGeometryPlanningParameters as FrameGeometryPlanningParameters,
)
from model.motionplan.standby import XnSideStandbySequence, incoming_workpiece_window
from model.motionplan.variable_2d import (  # noqa: F401
    Variable2DReportStateMachine as Variable2DReportStateMachine,
    _clamp_axis_position as _clamp_axis_position,
    _leading_edge_window_indices as _leading_edge_window_indices,
    _out_lift_limit as _out_lift_limit,
    _out_lift_origin_y as _out_lift_origin_y,
    build_variable_2d_window_observation as build_variable_2d_window_observation,
)
from model.plc.MovingFrameData import AxisData as MovingAxisData
from model.plc.MovingFrameData import SendMovingFrameData
from model.utils.MachineSafetyWindow import machine_window_bounds, machine_window_distances


class MotionFrameByFramePlanning:
    """将逐帧规划结果转换为 PLC 发送帧。"""

    SIDES = ("left", "right")

    def __init__(self, planning_config: Mapping) -> None:
        self.parameters = FrameByFramePlanningParameters.from_config(planning_config)
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
        self._standby = {side: XnSideStandbySequence() for side in self.SIDES}
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
        self._return_requires_completion = {side: False for side in self.SIDES}
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
        self._standby = {side: XnSideStandbySequence() for side in self.SIDES}
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
        self._return_requires_completion = {side: False for side in self.SIDES}
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
        self._standby[side] = XnSideStandbySequence()
        self.last_y_targets[side] = None
        self._last_x_steps[side] = None

    def _reset_return_state(self, side: str) -> None:
        self._return_sequence[side] = SidePositioningSequence()
        self._return_triggered[side] = False
        self._return_complete[side] = False
        self._return_requires_completion[side] = False

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
        """雷达到设备的整段来料队列及出料front窗口同时全空才正常回位。"""

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
            _, stop, _ = machine_window_bounds(machine, frame_queue_manager.stack_size)
            ranges.append((key, side, 0, stop))

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
            if machine_by_side[side].get("type") == "xn_side":
                start = 0  # 待料Y也依赖安全窗口之外、雷达至站位的下一件。
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
            reference_index, arrival_phase = None, "waiting"
            # 上一件尚在出料front窗口时，不使用下一件轮廓改变Y。
            if not any(occupied[:station_index]):
                incoming = incoming_workpiece_window(
                    stack, history_index, machine_window_distances(machine_config)[1] // CHAIN_COUNT_CM_STEP_MM,
                    lambda frame: len(self._cached_contour_rows(frame)) > 0,
                )
                if incoming is not None:
                    incoming_start, incoming_stop = incoming
                    frames = list(reversed(stack[incoming_start:incoming_stop]))
                    rows_by_frame = [self._cached_contour_rows(frame) for frame in frames]
                    occupied = [True] * len(frames)
                    station_index = reference_index = 0
                    arrival_phase = "preposition"
        parameters = self.geometry_parameters
        edges_by_frame = _detect_frame_edge_sequence(
            rows_by_frame,
            y_threshold=y_threshold,
            minimum_x_change_mm=parameters.minimum_x_change_mm,
            minimum_run_layers=parameters.minimum_run_layers,
            minimum_horizontal_layers=parameters.minimum_horizontal_layers,
            minimum_rail_gap_mm=parameters.minimum_rail_gap_mm,
            maximum_missing_y_layers=parameters.maximum_missing_y_layers,
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
            minimum_rail_gap_mm=parameters.minimum_rail_gap_mm,
            maximum_missing_y_layers=parameters.maximum_missing_y_layers,
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
            center_boxes = _apply_window_bottom(center_boxes, [boxes_by_frame[i] for i in selected])
        return RuntimeQueueGeometry(
            boxes=center_boxes,
            motions=center_motion,
            rows_by_frame=tuple(rows_by_frame),
            leading_edge=center_motion is not None and (arrival_phase == "preposition" or not occupied[0]),
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

    def _advance_initial_position(self, side, targets, y_targets, actual, tolerance):
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

    def _build_side_x_targets(self, geometry, machine_config, side, window_settings):
        """将四枪安全行程转换为往复或短行程固定点目标及保持原因。"""
        targets = {}
        hold_reasons = {}
        reasons = []
        for axis_name in XN_SIDE_X_AXES:
            if geometry.motions is None:
                targets[axis_name] = None
                hold_reasons[axis_name] = f"出料{window_settings[0]}mm/来料{window_settings[1]}mm安全包络无效"
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
        return targets, hold_reasons, reasons

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
            reasons.append(f"{side}侧使用来料前缘有效轮廓定位")
        if geometry.trailing_edge:
            reasons.append(f"{side}侧尾缘经过，使用剩余有效安全窗口规划")
        if geometry.arrival_phase == "preposition" or self._standby[side].phase != "idle":
            if not geometry.targets_prepared:
                geometry = self._prepare_side_geometry(geometry, machine_config, side)
            y_targets = (geometry.y_targets if geometry.boxes is not None
                         and geometry.y_targets_valid else None)
            commands, x_ready = self._standby[side].advance(
                machine_config, actual_positions, y_targets,
                position_tolerance_mm, self.x_arrival_confirmations,
            )
            self.last_y_targets[side] = y_targets
            phase = self._standby[side].phase
            reason = {"x": "待料先等待整侧X回安全位",
                      "y": "X保持安全位，Y定位下一件首喷位，关粉等待实际到站",
                      "waiting_geometry": "来料首喷轮廓无效，X保持安全位，Y保持位置"}[phase]
            x_commands = {a: XAxisMotionCommand(commands[a].Pos, commands[a].Speed, 0, f"standby_{phase}")
                          for a in XN_SIDE_X_AXES}
            self._last_x_steps[side] = XnSideFeedbackXStep(
                commands=x_commands,
                snapshots={a: replace(self.x_planners[side].states[a]._snapshot(
                    actual_pos=actual_positions[a], target_pos=c.Pos,
                ), phase=c.Phase, status=0, held=False, hold_reason=reason)
                    for a, c in x_commands.items()},
            )
            if geometry.arrival_phase != "at_station" or not x_ready:
                return commands, tuple(geometry.target_reasons) + (f"{side}侧{reason}",)
            self._standby[side] = XnSideStandbySequence()
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

        targets, hold_reasons, target_reasons = self._build_side_x_targets(
            geometry, machine_config, side, settings,
        )
        reasons.extend(target_reasons)

        if not self.motion_started[side]:
            if any(
                targets[axis] is None or hold_reasons[axis] for axis in XN_SIDE_X_AXES
            ):
                reasons.append(f"{side}侧尚无可初始化的四轴安全行程")
                return self._hold_initial_position(side, actual_positions), tuple(reasons)
            initial_commands, initial_reason = self._advance_initial_position(
                side, targets, last_y_targets, actual_positions, position_tolerance_mm,
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
                f"{side}侧2D整段来料队列及出料front窗口"
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

    @staticmethod
    def _read_cycle_parameters(plc_data, read_data_config):
        """校验本周期链条计数与轮廓分层精度。"""
        fifo = getattr(plc_data, "ChainCountCM", None)
        if isinstance(fifo, bool) or not isinstance(fifo, int):
            raise TypeError("plc_data.ChainCountCM 必须是整数")
        y_threshold = read_data_config.get("y_threshold", 10)
        if isinstance(y_threshold, bool) or not isinstance(y_threshold, int):
            raise TypeError("ReadDataConfig.y_threshold 必须是整数")
        if y_threshold <= 0:
            raise ValueError("ReadDataConfig.y_threshold 必须大于0")
        return fifo, y_threshold

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

            fifo, y_threshold = self._read_cycle_parameters(plc_data, read_data_config)
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
                # 普通空闲回位可改为收X后直接定位下一件Y；PLC禁用回位必须完成。
                standby_takeover = (geometry[side].arrival_phase == "preposition"
                                    and not self._return_requires_completion[side])
                return_triggered = device_disabled or idle_return or (
                    self._return_triggered[side] and not self._return_complete[side]
                    and not standby_takeover
                )
                if return_triggered != self._return_triggered[side]:
                    self._reset_side(side)
                    self._reset_return_state(side)
                    self._return_triggered[side] = return_triggered
                    self._return_complete[side] = False
                if device_disabled:
                    self._return_requires_completion[side] = True

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
                            f"{side}侧整段来料队列及出料front窗口均无工件"
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
