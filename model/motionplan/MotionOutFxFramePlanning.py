import os
from dataclasses import dataclass, field
from model.utils.LoggerUtil import logger
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_r, clamp_to_limit_yx, clamp_to_limit_z
from model.motionplan.MachineAxisMap import get_axis_map, get_axis_position_limits, get_axis_safe_pos, get_axis_speed_limit
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper
from model.motionplan.motionutil.FrameXMotionHelper import FrameXMotionHelper


@dataclass
class StaticGlobalXResult:
    global_x_min: int | None = None
    global_x_max: int | None = None
    gun_ranges: dict[str, tuple[int, int] | None] = field(default_factory=dict)
    search_y_ranges: dict[str, tuple[int, int]] = field(default_factory=dict)

    @property
    def valid_axes(self) -> set[str]:
        return {
            axis_name
            for axis_name, x_range in self.gun_ranges.items()
            if x_range is not None
        }


@dataclass
class DeviceFrameMotionState:
    stage: str = "idle"
    y_phase: str = "to_min"
    x_phase: str = "to_min"
    y_initialized: bool = False
    x_initialized: bool = False
    y_cycles: int = 0
    x_cycles: int = 0
    start_cycles: int = 0
    end_cycles: int = 0
    interpolation_bins: dict[str, int] = field(default_factory=dict)
    interpolation_y: dict[str, int] = field(default_factory=dict)
    interpolation_targets: dict[str, int] = field(default_factory=dict)
    interpolation_speeds: dict[str, int] = field(default_factory=dict)
    interpolation_base_x: dict[str, int] = field(default_factory=dict)
    tracking_x_pre_target: int | None = None
    tracking_recip_x_min_target: int | None = None
    tracking_recip_x_max_target: int | None = None
    tracking_start_seen: bool = False


@dataclass
class FrameMotionContext:
    sn: int
    machine_cfg: dict
    runtime_cfg: dict
    plc_data: object
    state: DeviceFrameMotionState
    frames: list
    chain_running: bool
    direct_profile: bool
    tracking: int
    window: object | None = None
    preposition_window: object | None = None
    config_error: str | None = None
    preposition_signature: bool = False
    start_signature: bool = False
    end_signature: bool = False
    center_has_data: bool = False
    window_empty: bool = True
    stage_changed: bool = False


@dataclass
class FrameStateResult:
    axis_cmds: dict | None = None
    next_stage: str | None = None
    workpiece_complete: bool = False
    stop_chain: bool = False


class MotionOutFxFramePlanning:
    """外侧仿形自动喷涂规划。"""

    FRAME_CONFIG_DEFAULTS = {
        "stage_detect_frame_count": 8,
        "frame_x_interpolation_enabled": 0,
        "frame_x_slow_in_out_enabled": 0,
        "frame_idle_y_reciprocate_enabled": 1,
        "frame_x_no_data_target_mode": 0,
        "frame_x_dynamic_speed_mode": 1,
    }

    def __init__(self, spray_cfg=None, read_data_cfg=None, motion_to_target=None):
        if spray_cfg is None or read_data_cfg is None:
            from model.utils.TomlLoader import TomlLoader
            config_dir = os.path.join(os.getcwd(), "model", "tomls")
            if spray_cfg is None:
                spray_cfg = TomlLoader.load(os.path.join(config_dir, "SprayConfig.toml"))
            if read_data_cfg is None:
                read_data_cfg = TomlLoader.load(os.path.join(config_dir, "ReadDataConfig.toml"))
        if motion_to_target is None:
            from model.motionplan.MotionToTarget import MotionToTarget
            motion_to_target = MotionToTarget()

        self.spray_cfg = spray_cfg
        self.read_data_cfg = read_data_cfg
        self.motion_to_target = motion_to_target
        self.z_threshold = int(self.read_data_cfg.get("z_threshold", 10))
        self.y_threshold = int(self.read_data_cfg.get("y_threshold", 10) or 10)
        if self.y_threshold <= 0:
            raise ValueError(f"y_threshold 必须大于 0，当前值: {self.y_threshold}")
        self.spray_pos_tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10))
        self.frame_helper = FrameSearchHelper(z_threshold=self.z_threshold)
        self.x_motion_helper = FrameXMotionHelper()
        self._work_states = {}

    def reset_motion_state(self, sn=None):
        if sn is None:
            self._work_states = {}
            return
        self._work_states.pop(int(sn), None)

    def auto_out_fx_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        """构建本周期上下文，并由统一状态分发表生成设备运动命令。"""
        ctx = self._build_motion_context(machine_cfg, runtime_cfg, plc_data, frame_queue_manager)
        self._normalize_initial_stage(ctx)
        result = self._dispatch_state(ctx)
        return result.axis_cmds or {}, result.workpiece_complete, result.stop_chain

    def _build_motion_context(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        """集中计算状态处理函数共用的配置、点云窗口和阶段特征。"""
        sn = int(machine_cfg.get("sn", 0))
        state = self._get_state(sn)
        frames = self.frame_helper.get_side_frames(machine_cfg, frame_queue_manager)
        requested_tracking = int(runtime_cfg.get("tracking", machine_cfg.get("tracking", 0)) or 0)
        direct_profile = self._get_frame_config("frame_x_slow_in_out_enabled") == 0
        tracking = 0 if direct_profile else requested_tracking
        ctx = FrameMotionContext(
            sn=sn,
            machine_cfg=machine_cfg,
            runtime_cfg=runtime_cfg,
            plc_data=plc_data,
            state=state,
            frames=frames,
            chain_running=self._is_chain_running(plc_data),
            direct_profile=direct_profile,
            tracking=tracking,
        )
        if not direct_profile and state.stage in {"start_retract", "end_retract", "return_safe"}:
            return ctx
        if not frames:
            return ctx

        z_cur = self._get_axis_pos(machine_cfg, plc_data, "z")
        ctx.window = self.frame_helper.build_window(machine_cfg, runtime_cfg, z_cur=z_cur, frame_count=len(frames))
        ctx.preposition_window = self._build_tracking_preposition_window(machine_cfg, runtime_cfg, z_cur, len(frames)) if tracking else ctx.window
        ctx.config_error = self._validate_motion_config(machine_cfg, runtime_cfg, ctx.window, tracking)
        if direct_profile or ctx.config_error:
            return ctx

        detect_count = self._get_frame_config("stage_detect_frame_count")
        ctx.preposition_signature = bool(tracking and self.frame_helper.has_start_signature(frames, ctx.preposition_window, detect_count))
        ctx.start_signature = self.frame_helper.has_start_signature(frames, ctx.window, detect_count)
        ctx.end_signature = self.frame_helper.has_end_signature(frames, ctx.window, detect_count)
        ctx.center_has_data = self.frame_helper.frame_has_data(self.frame_helper.get_frame_by_index(frames, ctx.window.center))
        ctx.window_empty = self.frame_helper.window_is_empty(frames, ctx.window)
        return ctx

    def _normalize_initial_stage(self, ctx):
        """慢进慢出开关只决定进入直接仿形状态还是分阶段状态机。"""
        if ctx.direct_profile and ctx.state.stage != "direct_profile":
            self._set_stage(ctx.state, "direct_profile", tracking=0)
        elif not ctx.direct_profile and ctx.state.stage == "direct_profile":
            self._set_stage(ctx.state, "idle", tracking=ctx.tracking)

    def _dispatch_table(self):
        """所有配置共用同一张状态表，配置仅影响状态跳转和状态内部计算策略。"""
        return {
            "idle": self._state_idle,
            "direct_profile": self._state_direct_profile,
            "preposition": self._state_preposition,
            "start": self._state_start,
            "start_retract": self._state_start_retract,
            "middle": self._state_middle,
            "end": self._state_end,
            "end_retract": self._state_end_retract,
            "return_safe": self._state_return_safe,
        }

    def _dispatch_state(self, ctx):
        """统一执行状态、应用状态切换；无命令的判定状态会在本周期继续分发到目标状态。"""
        dispatch_table = self._dispatch_table()
        for _ in range(4):
            handler = dispatch_table.get(ctx.state.stage)
            if handler is None:
                logger.error(f"SN[{ctx.sn}] unknown frame motion state: {ctx.state.stage}")
                return FrameStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data), stop_chain=True)

            result = handler(ctx)
            if result.next_stage:
                self._set_stage(ctx.state, result.next_stage, tracking=ctx.tracking)
                ctx.stage_changed = True
            if result.axis_cmds is not None:
                return result
            if not result.next_stage:
                logger.error(f"SN[{ctx.sn}] frame state [{ctx.state.stage}] returned neither commands nor next state")
                return FrameStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data), stop_chain=True)

        logger.error(f"SN[{ctx.sn}] frame state transition exceeded the single-cycle limit")
        return FrameStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data), stop_chain=True)

    def _frame_state_guard(self, ctx):
        """需要点云的状态统一处理无数据和配置错误，收枪及复位状态不经过此检查。"""
        if not ctx.frames:
            logger.warning(f"SN[{ctx.sn}] frame side data not ready")
            return FrameStateResult(self._build_idle_axis_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state))
        if ctx.config_error:
            logger.error(f"SN[{ctx.sn}] frame motion config error: {ctx.config_error}")
            return FrameStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data), stop_chain=True)
        return None

    def _state_idle(self, ctx):
        guard_result = self._frame_state_guard(ctx)
        if guard_result:
            return guard_result
        if ctx.tracking and (ctx.preposition_signature or ctx.start_signature):
            return FrameStateResult(next_stage="preposition")
        if ctx.start_signature:
            return FrameStateResult(next_stage="start")
        return FrameStateResult(self._build_idle_axis_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state))

    def _state_direct_profile(self, ctx):
        guard_result = self._frame_state_guard(ctx)
        if guard_result:
            return guard_result
        axis_cmds = self._build_basic_nontracking_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state, ctx.frames, ctx.window, ctx.chain_running)
        return FrameStateResult(axis_cmds)

    def _state_preposition(self, ctx):
        guard_result = self._frame_state_guard(ctx)
        if guard_result:
            return guard_result
        if ctx.start_signature:
            ctx.state.tracking_start_seen = True
        axis_cmds, x_ready = self._build_tracking_preposition_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state, ctx.frames, ctx.preposition_window)
        next_stage = "start" if x_ready and ctx.state.tracking_start_seen else None
        return FrameStateResult(axis_cmds, next_stage=next_stage)

    def _state_start(self, ctx):
        guard_result = self._frame_state_guard(ctx)
        if guard_result:
            return guard_result
        if not ctx.stage_changed and not ctx.tracking and ctx.center_has_data:
            return FrameStateResult(next_stage="middle")
        axis_cmds = self._build_active_stage_commands(ctx)
        return FrameStateResult(axis_cmds, next_stage=self._resolve_tracking_completion_stage(ctx))

    def _state_start_retract(self, ctx):
        axis_cmds, x_ready = self._build_tracking_retract_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state)
        return FrameStateResult(axis_cmds, next_stage="middle" if x_ready else None)

    def _state_middle(self, ctx):
        guard_result = self._frame_state_guard(ctx)
        if guard_result:
            return guard_result
        if not ctx.stage_changed and ctx.end_signature:
            return FrameStateResult(next_stage="end")
        return FrameStateResult(self._build_active_stage_commands(ctx))

    def _state_end(self, ctx):
        guard_result = self._frame_state_guard(ctx)
        if guard_result:
            return guard_result
        if not ctx.stage_changed and not ctx.tracking and ctx.window_empty:
            return FrameStateResult(next_stage="return_safe")
        axis_cmds = self._build_active_stage_commands(ctx)
        return FrameStateResult(axis_cmds, next_stage=self._resolve_tracking_completion_stage(ctx))

    def _state_end_retract(self, ctx):
        axis_cmds, x_ready = self._build_tracking_retract_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state)
        return FrameStateResult(axis_cmds, next_stage="return_safe" if x_ready else None)

    def _state_return_safe(self, ctx):
        axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data)
        if all_ready:
            self._work_states[ctx.sn] = DeviceFrameMotionState()
        return FrameStateResult(axis_cmds or {}, workpiece_complete=bool(all_ready))

    def _build_active_stage_commands(self, ctx):
        """生成 start、middle、end 共用的 Y/X/Z/R 命令，状态只决定目标策略和完成条件。"""
        axis_cmds = self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data)
        axis_cmds["y"] = self._build_y_reciprocate_axis(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state)
        interpolation_enabled = self._get_frame_config("frame_x_interpolation_enabled")

        if ctx.tracking and ctx.state.stage in {"start", "end"}:
            static_result = self._calculate_static_global_x(ctx.machine_cfg, ctx.runtime_cfg, ctx.frames, ctx.window)
            self._cache_tracking_pre_target(ctx.state, ctx.machine_cfg, ctx.runtime_cfg, static_result)
            x_cmds = self._build_tracking_reciprocate_x_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state,
                                                                 ctx.frames, ctx.window, static_result, ctx.chain_running)
        else:
            current_x_offset = self._resolve_current_x_offset(ctx.state, ctx.tracking, ctx.frames, ctx.window, ctx.machine_cfg, ctx.runtime_cfg)
            if interpolation_enabled == 1:
                x_cmds = self._build_dynamic_x_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state,
                                                        ctx.frames, ctx.window, current_x_offset, ctx.chain_running)
            else:
                static_result = self._calculate_static_global_x(ctx.machine_cfg, ctx.runtime_cfg, ctx.frames, ctx.window)
                x_cmds = self._build_static_position_x_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.frames,
                                                                ctx.window, static_result, current_x_offset, ctx.chain_running)

        axis_cmds.update(x_cmds)
        axis_cmds["z"] = self._build_z_axis(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state, ctx.tracking)
        axis_cmds.update(self._build_r_axis_commands(ctx.machine_cfg))
        logger.info(f"SN[{ctx.sn}] frame stage={ctx.state.stage}, tracking={ctx.tracking}, chain_running={ctx.chain_running}, "
                    f"z_window=({ctx.window.start}, {ctx.window.center}, {ctx.window.end})")
        return axis_cmds

    def _resolve_tracking_completion_stage(self, ctx):
        """跟踪开始面和结束面达到配置往复次数后，返回对应收枪状态。"""
        if not ctx.tracking or ctx.state.stage not in {"start", "end"}:
            return None
        cycle_axis = str(self.spray_cfg.get("side_2d_cycle_axis", "y")).lower()
        completed_cycles = ctx.state.x_cycles if cycle_axis == "x" else ctx.state.y_cycles
        total_cycles = int(ctx.runtime_cfg.get("outside_total_cycles", ctx.machine_cfg.get("outside_total_cycles", 1)) or 0)
        if ctx.state.stage == "start":
            ctx.state.start_cycles = completed_cycles
            return "start_retract" if ctx.state.start_cycles >= total_cycles else None
        ctx.state.end_cycles = completed_cycles
        return "end_retract" if ctx.state.end_cycles >= total_cycles else None

    def _get_state(self, sn: int) -> DeviceFrameMotionState:
        return self._work_states.setdefault(int(sn), DeviceFrameMotionState())

    def _get_frame_config(self, name):
        """集中读取按帧开关，确保代码回退值与 SprayConfig.toml 一致。"""
        default = self.FRAME_CONFIG_DEFAULTS[name]
        value = self.spray_cfg.get(name, default)
        return default if value is None else int(value)

    def _build_idle_axis_commands(self, machine_cfg, runtime_cfg, plc_data, state):
        axis_cmds, _ = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
        axis_cmds = axis_cmds or {}
        idle_reciprocate_enabled = self._get_frame_config("frame_idle_y_reciprocate_enabled")
        if idle_reciprocate_enabled == 1:
            axis_cmds["y"] = self._build_y_reciprocate_axis(machine_cfg, runtime_cfg, plc_data, state)
        return axis_cmds

    def _build_basic_nontracking_commands(self, machine_cfg, runtime_cfg, plc_data, state, frames, window, chain_running):
        """无状态普通仿形：不做阶段检测和链条跟踪，每帧直接生成逐枪 X 目标。"""
        axis_cmds = self._build_idle_axis_commands(machine_cfg, runtime_cfg, plc_data, state)
        current_x_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 0)) or 0)
        interpolation_enabled = self._get_frame_config("frame_x_interpolation_enabled")
        if interpolation_enabled == 1:
            x_cmds = self._build_dynamic_x_commands(machine_cfg, runtime_cfg, plc_data, state, frames, window, current_x_offset, chain_running)
        else:
            static_result = self._calculate_static_global_x(machine_cfg, runtime_cfg, frames, window)
            x_cmds = self._build_static_position_x_commands(machine_cfg, runtime_cfg, plc_data, frames, window,
                                                            static_result, current_x_offset, chain_running)

        axis_cmds.update(x_cmds)
        axis_cmds["z"] = self._build_z_axis(machine_cfg, runtime_cfg, plc_data, state, tracking=0)
        axis_cmds.update(self._build_r_axis_commands(machine_cfg))
        return axis_cmds

    def _build_tracking_preposition_window(self, machine_cfg, runtime_cfg, z_cur, frame_count):
        window_runtime_cfg = dict(runtime_cfg or {})
        front_offset = int(window_runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 0)) or 0)
        x_status_offset = max(0, int(window_runtime_cfg.get("x_status_offset", machine_cfg.get("x_status_offset", 0)) or 0))
        window_runtime_cfg["out_z_front_offset"] = (front_offset + x_status_offset)
        return self.frame_helper.build_window(machine_cfg, window_runtime_cfg, z_cur=z_cur, frame_count=frame_count)

    def _build_tracking_preposition_commands(self, machine_cfg, runtime_cfg, plc_data, state, frames, window):
        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        axis_cmds["y"] = self._build_y_reciprocate_axis(machine_cfg, runtime_cfg, plc_data, state)
        static_result = self._calculate_static_global_x(machine_cfg, runtime_cfg, frames, window)
        target = self._cache_tracking_pre_target(state, machine_cfg, runtime_cfg, static_result)
        if target is None:
            return axis_cmds, False

        x_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0)
        x_cmds, x_ready = self.motion_to_target.move_x_axes_to_target(machine_cfg, plc_data, target, x_speed)
        axis_cmds.update(x_cmds)
        return axis_cmds, x_ready

    def _build_tracking_retract_commands(self, machine_cfg, runtime_cfg, plc_data, state):
        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        target = state.tracking_x_pre_target
        x_ready = False
        if target is not None:
            x_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0)
            x_cmds, x_ready = self.motion_to_target.move_x_axes_to_target(machine_cfg, plc_data, target, x_speed)
            axis_cmds.update(x_cmds)

        axis_cmds["z"] = self._build_z_axis(machine_cfg, runtime_cfg, plc_data, state, tracking=1)
        axis_cmds.update(self._build_r_axis_commands(machine_cfg))
        return axis_cmds, x_ready

    def _cache_tracking_pre_target(self, state, machine_cfg, runtime_cfg, static_result):
        if state.tracking_x_pre_target is not None:
            return state.tracking_x_pre_target
        if static_result.global_x_min is None:
            return None

        front_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 0)) or 0)
        x_pre_distance = max(0, int(self.spray_cfg.get("x_pre_distance", 0) or 0))
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        state.tracking_x_pre_target = self.x_motion_helper.build_final_x_target(static_result.global_x_min, machine_cfg.get("x_position", 0),
                                                                                front_offset + x_pre_distance, x_min_limit, x_max_limit)
        return state.tracking_x_pre_target

    def _validate_motion_config(self, machine_cfg, runtime_cfg, window, tracking):
        """校验按帧模式开关和当前设备运动参数。"""
        if tracking not in (0, 1):
            return f"tracking must be 0 or 1, current value: {tracking}"

        switch_names = (
            "frame_x_interpolation_enabled",
            "frame_x_slow_in_out_enabled",
            "frame_idle_y_reciprocate_enabled",
            "frame_x_no_data_target_mode",
            "frame_x_dynamic_speed_mode",
        )
        for name in switch_names:
            value = self._get_frame_config(name)
            if value not in (0, 1):
                return f"{name} must be 0 or 1, current value: {value}"

        slow_in_out_enabled = self._get_frame_config("frame_x_slow_in_out_enabled")
        cycle_axis = str(self.spray_cfg.get("side_2d_cycle_axis", "y")).lower()
        if cycle_axis not in {"x", "y"}:
            return f"side_2d_cycle_axis must be x or y, current value: {cycle_axis}"

        requires_stage_detection = slow_in_out_enabled == 1
        if requires_stage_detection:
            detect_count = self._get_frame_config("stage_detect_frame_count")
            window_length = window.end - window.start + 1
            if detect_count <= 0 or detect_count >= window_length:
                return ("stage_detect_frame_count must be greater than 0 and smaller "
                        f"than the Z window length, current values: {detect_count}/{window_length}")

        total_cycles = int(runtime_cfg.get("outside_total_cycles", machine_cfg.get("outside_total_cycles", 1)) or 0)
        if total_cycles <= 0:
            return f"outside_total_cycles must be greater than 0, current value: {total_cycles}"

        y_target_min, y_target_max = self._get_y_targets(machine_cfg, runtime_cfg)
        if y_target_min >= y_target_max:
            return ("y_move_min must be smaller than y_move_max after limit checking, "
                    f"current values: {y_target_min}/{y_target_max}")

        x_axis_names = self._get_x_axis_names(machine_cfg)
        origin_pos = machine_cfg.get("origin_pos", [])
        if len(origin_pos) < len(x_axis_names):
            return ("origin_pos count is smaller than X axis count, "
                    f"current values: {len(origin_pos)}/{len(x_axis_names)}")
        return None

    def _set_stage(self, state, stage, tracking):
        if state.stage == stage:
            return

        state.stage = stage
        self._clear_interpolation_state(state)
        if stage == "preposition":
            self._reset_reciprocation_state(state)
            state.tracking_x_pre_target = None
            state.tracking_start_seen = False
        elif stage == "start":
            self._reset_reciprocation_state(state)
            self._clear_tracking_reciprocate_targets(state)
            state.start_cycles = 0
            state.end_cycles = 0
        elif stage == "middle":
            state.x_phase = "to_min"
            state.x_initialized = False
            state.x_cycles = 0
            self._clear_tracking_reciprocate_targets(state)
        elif stage == "end":
            state.x_phase = "to_min"
            state.x_initialized = False
            state.x_cycles = 0
            self._clear_tracking_reciprocate_targets(state)
            state.end_cycles = 0
            if tracking:
                state.y_phase = "to_min"
                state.y_initialized = False
                state.y_cycles = 0
        elif stage in {"direct_profile", "return_safe", "idle"}:
            self._reset_reciprocation_state(state)
            self._clear_tracking_reciprocate_targets(state)
            state.tracking_x_pre_target = None
            state.tracking_start_seen = False

    @staticmethod
    def _reset_reciprocation_state(state):
        state.y_phase = "to_min"
        state.x_phase = "to_min"
        state.y_initialized = False
        state.x_initialized = False
        state.y_cycles = 0
        state.x_cycles = 0

    @staticmethod
    def _clear_tracking_reciprocate_targets(state):
        state.tracking_recip_x_min_target = None
        state.tracking_recip_x_max_target = None

    @staticmethod
    def _clear_interpolation_state(state):
        state.interpolation_bins.clear()
        state.interpolation_y.clear()
        state.interpolation_targets.clear()
        state.interpolation_speeds.clear()
        state.interpolation_base_x.clear()

    def _get_z_window(self, machine_cfg, runtime_cfg):
        z_position = int(machine_cfg.get("z_position", 0))
        z_front_offset = int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 0)))
        z_after_offset = int(runtime_cfg.get("out_z_after_offset", machine_cfg.get("out_z_after_offset", 0)))
        start_idx = max(0, int((z_position - z_front_offset) / self.z_threshold))
        end_idx = max(0, int((z_position + z_after_offset) / self.z_threshold))
        return start_idx, end_idx

    def _calc_y_motion_range(self, machine_cfg, runtime_cfg, y_min_abs, y_max_abs):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_offset = int(runtime_cfg.get("out_up_y_offset", machine_cfg.get("out_up_y_offset", 0)))
        origin_pos = machine_cfg.get("origin_pos", [])
        origin_values = [int(v or 0) for v in origin_pos]
        y_origin_min = min(origin_values) if origin_values else 0
        y_origin_max = max(origin_values) if origin_values else 0
        gun_distance = abs(origin_values[1] - origin_values[0]) if len(origin_values) >= 2 else 250

        y_min_target = clamp_to_limit_yx(int(y_min_abs) - y_offset - y_origin_min, y_min_limit, y_max_limit)
        y_max_target = clamp_to_limit_yx(int(y_max_abs) + y_offset - y_origin_max, y_min_limit, y_max_limit)
        if y_max_target < gun_distance + y_min_target:
            y_max_target = clamp_to_limit_yx(gun_distance + y_min_target, y_min_limit, y_max_limit)
        return y_min_target, y_max_target

    def _build_y_reciprocate_axis(self, machine_cfg, runtime_cfg, plc_data, state):
        y_speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)))
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        y_target_min, y_target_max = self._get_y_targets(machine_cfg, runtime_cfg)
        y_axis_name = self._get_logical_y_axis_name(machine_cfg)
        y_cur = self._get_axis_pos(machine_cfg, plc_data, y_axis_name)

        if not state.y_initialized:
            if abs(y_cur - y_target_min) <= self.spray_pos_tolerance:
                state.y_initialized = True
                state.y_phase = "to_max"
                target = y_target_max
            else:
                state.y_phase = "to_min"
                target = y_target_min
        else:
            target = y_target_max if state.y_phase == "to_max" else y_target_min
            if abs(y_cur - target) <= self.spray_pos_tolerance:
                if state.y_phase == "to_max":
                    state.y_phase = "to_min"
                    target = y_target_min
                else:
                    state.y_cycles += 1
                    state.y_phase = "to_max"
                    target = y_target_max

        return build_axis(target, y_speed, 0, y_speed_limit)

    def _get_y_targets(self, machine_cfg, runtime_cfg):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_target_min = clamp_to_limit_yx(int(runtime_cfg.get("y_move_min", machine_cfg.get("y_move_min", 0)) or 0),
                                         y_min_limit, y_max_limit)
        y_target_max = clamp_to_limit_yx(int(runtime_cfg.get("y_move_max", machine_cfg.get("y_move_max", 0)) or 0),
                                         y_min_limit, y_max_limit)
        return y_target_min, y_target_max

    @staticmethod
    def _get_logical_y_axis_name(machine_cfg):
        return "y1" if machine_cfg.get("type") == "xn_side" else "y"

    @staticmethod
    def _get_x_axis_names(machine_cfg):
        return [axis_name
                for axis_name in machine_cfg.get("axis_type", [])
                if axis_name.startswith("x")]

    def _calculate_x_ranges(self, machine_cfg, runtime_cfg, frames, window, y_cur=None):
        """按静态 Y 行程或当前 Y 位置提取每把枪的矩形 X 范围，并汇总本帧全局范围。"""
        y_move_min, y_move_max = self._get_y_targets(machine_cfg, runtime_cfg)
        out_down_y_offset = int(runtime_cfg.get("out_down_y_offset", machine_cfg.get("out_down_y_offset", 0)) or 0)
        out_up_y_offset = int(runtime_cfg.get("out_up_y_offset", machine_cfg.get("out_up_y_offset", 0)) or 0)
        origin_pos = machine_cfg.get("origin_pos", [])
        result = StaticGlobalXResult()

        for index, axis_name in enumerate(self._get_x_axis_names(machine_cfg)):
            if y_cur is None:
                search_y_min, search_y_max = self.x_motion_helper.build_static_search_y_range(origin_pos[index], y_move_min, y_move_max, out_down_y_offset, out_up_y_offset)
            else:
                search_y_min, search_y_max = self.x_motion_helper.build_dynamic_search_y_range(origin_pos[index], y_cur, out_down_y_offset, out_up_y_offset)
            result.search_y_ranges[axis_name] = (search_y_min, search_y_max)
            x_min, x_max = self.frame_helper.collect_x_range(frames, window, search_y_min, search_y_max)
            result.gun_ranges[axis_name] = (int(x_min), int(x_max)) if x_min is not None and x_max is not None else None

        result.global_x_min, result.global_x_max = self.x_motion_helper.aggregate_static_x_range(result.gun_ranges.values())
        return result

    def _calculate_static_global_x(self, machine_cfg, runtime_cfg, frames, window):
        return self._calculate_x_ranges(machine_cfg, runtime_cfg, frames, window)

    def _calculate_dynamic_x_ranges(self, machine_cfg, runtime_cfg, plc_data, frames, window):
        y_axis_name = self._get_logical_y_axis_name(machine_cfg)
        y_cur = self._get_axis_pos(machine_cfg, plc_data, y_axis_name)
        return self._calculate_x_ranges(machine_cfg, runtime_cfg, frames, window, y_cur=y_cur), y_cur

    def _build_tracking_reciprocate_x_commands(self, machine_cfg, runtime_cfg, plc_data, state, frames, window, static_result, chain_running):
        """跟踪开始和结束阶段锁定首组有效全局 X 目标，避免 Z 窗口移动改变本次往复行程。"""
        axis_names = self._get_x_axis_names(machine_cfg)
        x_position = int(machine_cfg.get("x_position", 0) or 0)
        front_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 0)) or 0)
        after_offset = int(runtime_cfg.get("out_after_x_offset", machine_cfg.get("out_after_x_offset", 0)) or 0)
        targets_cached = state.tracking_recip_x_min_target is not None and state.tracking_recip_x_max_target is not None
        if not targets_cached and (static_result.global_x_min is None or static_result.global_x_max is None):
            return {axis_name: self._build_no_data_x_command(machine_cfg, runtime_cfg, axis_name, None, front_offset)
                    for axis_name in axis_names}

        if not targets_cached:
            x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
            target_min = self.x_motion_helper.build_final_x_target(static_result.global_x_min, x_position, front_offset, x_min_limit, x_max_limit)
            target_max = self.x_motion_helper.build_final_x_target(static_result.global_x_max, x_position, after_offset, x_min_limit, x_max_limit)
            if target_min > target_max:
                target_min, target_max = target_max, target_min
            state.tracking_recip_x_min_target = target_min
            state.tracking_recip_x_max_target = target_max

        target_min = state.tracking_recip_x_min_target
        target_max = state.tracking_recip_x_max_target

        axis_positions = [self._get_axis_pos(machine_cfg, plc_data, axis_name) for axis_name in axis_names]
        x_pos_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0)
        x_recip_speed = int(runtime_cfg.get("x_recip_speed", machine_cfg.get("x_recip_speed", 0)) or 0)

        if not state.x_initialized:
            if axis_positions and all(abs(position - target_min) <= self.spray_pos_tolerance for position in axis_positions):
                state.x_initialized = True
                state.x_phase = "to_max"
                target = target_max
                speed = x_recip_speed
            else:
                state.x_phase = "to_min"
                target = target_min
                speed = x_pos_speed
        else:
            target = target_max if state.x_phase == "to_max" else target_min
            if axis_positions and all(abs(position - target) <= self.spray_pos_tolerance for position in axis_positions):
                if state.x_phase == "to_max":
                    state.x_phase = "to_min"
                    target = target_min
                else:
                    state.x_cycles += 1
                    state.x_phase = "to_max"
                    target = target_min
            speed = x_recip_speed

        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        axis_cmds = {}
        for axis_name in axis_names:
            search_y_range = static_result.search_y_ranges.get(axis_name)
            status = int(search_y_range is not None and chain_running
                         and self._has_x_status_data(frames, window, search_y_range[0], search_y_range[1], runtime_cfg, machine_cfg))
            axis_cmds[axis_name] = build_axis(target, speed, status, x_speed_limit)
        return axis_cmds

    def _build_static_position_x_commands(self, machine_cfg, runtime_cfg, plc_data, frames, window, static_result, current_x_offset, chain_running):
        """静态 Y 范围下按每把枪自己的矩形 x_min 定位，无数据时按配置选择安全位置或全局 x_min。"""
        x_position = int(machine_cfg.get("x_position", 0) or 0)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0)
        speed_limit = get_axis_speed_limit(machine_cfg, "x")
        axis_cmds = {}

        for axis_name in self._get_x_axis_names(machine_cfg):
            gun_range = static_result.gun_ranges.get(axis_name)
            has_region_data = gun_range is not None
            if has_region_data:
                base_x_min = gun_range[0]
            else:
                axis_cmds[axis_name] = self._build_no_data_x_command(machine_cfg, runtime_cfg, axis_name, static_result.global_x_min, current_x_offset)
                continue

            target = self.x_motion_helper.build_final_x_target(base_x_min, x_position, current_x_offset, x_min_limit, x_max_limit)
            search_y_range = static_result.search_y_ranges.get(axis_name)
            status = int(has_region_data and search_y_range is not None and chain_running
                         and self._has_x_status_data(frames, window, search_y_range[0], search_y_range[1], runtime_cfg, machine_cfg))
            axis_cmds[axis_name] = build_axis(target, speed, status, speed_limit)
        return axis_cmds

    def _resolve_dynamic_x_speed(self, state, axis_name, y_cur, y_bin, base_x_min, target, y_recip_speed, x_speed_limit, x_pos_speed):
        """动态 Y 范围下根据配置选择固定定位速度或逐枪插补速度。"""
        if self._get_frame_config("frame_x_dynamic_speed_mode") == 0:
            return x_pos_speed

        needs_update = (axis_name not in state.interpolation_bins or state.interpolation_bins[axis_name] != y_bin
                        or state.interpolation_base_x.get(axis_name) != base_x_min)
        if needs_update:
            speed = self.x_motion_helper.calculate_interpolation_speed(state.interpolation_y.get(axis_name), y_cur, state.interpolation_targets.get(axis_name),
                                                                       target, y_recip_speed, x_speed_limit, x_pos_speed)
            state.interpolation_bins[axis_name] = y_bin
            state.interpolation_y[axis_name] = y_cur
            state.interpolation_base_x[axis_name] = base_x_min
            state.interpolation_speeds[axis_name] = speed
        else:
            speed = state.interpolation_speeds[axis_name]
        state.interpolation_targets[axis_name] = target
        return speed

    def _build_dynamic_x_commands(self, machine_cfg, runtime_cfg, plc_data, state, frames, window, current_x_offset, chain_running):
        """动态 Y 范围下提取逐枪 x_min，并独立选择目标和速度。"""
        range_result, y_cur = self._calculate_dynamic_x_ranges(machine_cfg, runtime_cfg, plc_data, frames, window)
        y_bin = int(y_cur / self.y_threshold)
        x_position = int(machine_cfg.get("x_position", 0) or 0)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        x_pos_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0)
        y_recip_speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 0)) or 0)
        no_data_mode = self._get_frame_config("frame_x_no_data_target_mode")
        axis_cmds = {}

        for axis_name in self._get_x_axis_names(machine_cfg):
            gun_range = range_result.gun_ranges.get(axis_name)
            has_region_data = gun_range is not None
            if has_region_data:
                base_x_min = gun_range[0]
            elif no_data_mode == 1 and range_result.global_x_min is not None:
                base_x_min = range_result.global_x_min
            else:
                axis_cmds[axis_name] = self._build_no_data_x_command(machine_cfg, runtime_cfg, axis_name, None, current_x_offset)
                continue

            target = self.x_motion_helper.build_final_x_target(base_x_min, x_position, current_x_offset, x_min_limit, x_max_limit)
            speed = self._resolve_dynamic_x_speed(state, axis_name, y_cur, y_bin, base_x_min, target, y_recip_speed, x_speed_limit, x_pos_speed)
            search_y_range = range_result.search_y_ranges.get(axis_name)
            status = int(has_region_data and search_y_range is not None and chain_running
                         and self._has_x_status_data(frames, window, search_y_range[0], search_y_range[1], runtime_cfg, machine_cfg))
            axis_cmds[axis_name] = build_axis(target, speed, status, x_speed_limit)
        return axis_cmds

    def _resolve_current_x_offset(self, state, tracking, frames, window, machine_cfg, runtime_cfg):
        front_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 0)) or 0)
        if tracking:
            return front_offset

        slow_in_out_enabled = self._get_frame_config("frame_x_slow_in_out_enabled")
        if slow_in_out_enabled == 0:
            return front_offset

        populated_indices = [
            index
            for index in range(window.start, window.end + 1)
            if self.frame_helper.frame_has_data(
                self.frame_helper.get_frame_by_index(frames, index)
            )
        ]
        if not populated_indices:
            return 0

        return self.x_motion_helper.resolve_slow_offset(
            max(populated_indices) * self.z_threshold,
            min(populated_indices) * self.z_threshold,
            window.center * self.z_threshold,
            int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 0)) or 0),
            int(runtime_cfg.get("out_z_after_offset", machine_cfg.get("out_z_after_offset", 0)) or 0),
            front_offset,
        )

    def _resolve_follow_z_speed(self, plc_data):
        chain_speed = int(getattr(plc_data, "ChainSpeed", 0) or 0)
        if not self._is_chain_running(plc_data):
            return 0
        return chain_speed if chain_speed != 0 else 0

    def _build_z_axis(self, machine_cfg, runtime_cfg, plc_data, state, tracking):
        z_min_limit, z_max_limit = get_axis_position_limits(machine_cfg, "z")
        z_speed_limit = get_axis_speed_limit(machine_cfg, "z")
        if tracking and state.stage in {"start", "end", "start_retract", "end_retract"}:
            target = clamp_to_limit_z(z_max_limit, z_min_limit, z_max_limit)
            speed = self._resolve_follow_z_speed(plc_data)
        elif tracking:
            target = clamp_to_limit_z(get_axis_safe_pos(machine_cfg, "z"), z_min_limit, z_max_limit)
            speed = int(runtime_cfg.get("z_back_speed", machine_cfg.get("z_back_speed", 0)) or 0)
        else:
            target = clamp_to_limit_z(0, z_min_limit, z_max_limit)
            speed = int(runtime_cfg.get("z_zeroing_speed", machine_cfg.get("z_zeroing_speed", 0)) or 0)
        return build_axis(target, speed, 0, z_speed_limit)

    def _build_r_axis_commands(self, machine_cfg):
        axis_cmds = {}
        for axis_name in machine_cfg.get("axis_type", []):
            if not axis_name.startswith("r"):
                continue
            r_min_limit, r_max_limit = get_axis_position_limits(machine_cfg, axis_name)
            axis_cmds[axis_name] = build_axis(clamp_to_limit_r(0, r_min_limit, r_max_limit), 0, 0, get_axis_speed_limit(machine_cfg, axis_name))
        return axis_cmds

    def _build_no_data_x_command(self, machine_cfg, runtime_cfg, axis_name, global_x_min, current_x_offset):
        """区域无数据时按配置使用安全位置，或使用本帧其他有效区域的全局 x_min；全部无数据始终回安全位置。"""
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, axis_name)
        if self._get_frame_config("frame_x_no_data_target_mode") == 1 and global_x_min is not None:
            target = self.x_motion_helper.build_final_x_target(global_x_min, machine_cfg.get("x_position", 0), current_x_offset, x_min_limit, x_max_limit)
        else:
            target = clamp_to_limit_yx(get_axis_safe_pos(machine_cfg, axis_name), x_min_limit, x_max_limit)

        x_pos_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0)
        return build_axis(target, x_pos_speed, 0, get_axis_speed_limit(machine_cfg, axis_name))

    def _build_hold_x_command(self, machine_cfg, plc_data, axis_name):
        current_position = self._get_axis_pos(machine_cfg, plc_data, axis_name)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, axis_name)
        return build_axis(clamp_to_limit_yx(current_position, x_min_limit, x_max_limit), 0, 0, get_axis_speed_limit(machine_cfg, axis_name))

    def _has_x_status_data(self, frames, window, search_y_min, search_y_max, runtime_cfg, machine_cfg):
        x_status_offset = int(runtime_cfg.get("x_status_offset", machine_cfg.get("x_status_offset", 0)) or 0)
        offset_frames = max(0, int(x_status_offset / self.z_threshold))
        start_index = max(0, window.start - offset_frames)
        end_index = max(start_index, window.end - offset_frames)
        return bool(self.frame_helper.collect_x_min_values(frames, start_index, end_index, min(search_y_min, search_y_max), max(search_y_min, search_y_max)))

    def _build_x_axis_commands(self, machine_cfg, runtime_cfg, chain_running, frames, z_start_idx, z_end_idx, y_min_target, y_max_target):
        x_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 300)))
        x_position = int(machine_cfg.get("x_position", 850))
        x_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 100)))
        fx_mode = int(self.spray_cfg.get("fx_mode", 0) or 0)
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        origin_pos = machine_cfg.get("origin_pos", [])
        x_axis_names = [axis_name for axis_name in machine_cfg.get("axis_type", []) if axis_name.startswith("x")]

        results = []
        valid_targets = []
        for idx, axis_name in enumerate(x_axis_names):
            origin_y = int(origin_pos[idx] or 0) if idx < len(origin_pos) else 0
            y_start_abs = origin_y + y_min_target
            y_end_abs = origin_y + y_max_target
            x_min_values = self.frame_helper.collect_x_min_values(frames, z_start_idx, z_end_idx, y_start_abs, y_end_abs)
            # logger.debug(f"x_min_values={x_min_values}")
            has_x_min_data = len(x_min_values) > 0
            if has_x_min_data:
                x_target = clamp_to_limit_yx(min(x_min_values) - x_position - x_offset, x_min_limit, x_max_limit)
                # logger.debug(f"x_target={x_target}")
                valid_targets.append(x_target)
                results.append((axis_name, x_target, True))
            else:
                results.append((axis_name, 0, False))

        fallback_target = min(valid_targets) if valid_targets else 0

        # 二维模式：先保留原有逐轴搜索逻辑，再将 x1~x5 的目标统一收敛到所有有效目标中的最小值
        if fx_mode == 1 and x_axis_names:
            unified_target = fallback_target
            unified_status = 1 if chain_running and bool(valid_targets) else 0
            axis_cmds = {axis_name: build_axis(unified_target, x_speed, unified_status, x_speed_limit)
                         for axis_name in x_axis_names}
            # logger.debug(
            #     f"out_fx fx_mode=1, unified x target applied: target={unified_target}, "
            #     f"status={unified_status}, valid_targets={valid_targets}"
            # )
            return axis_cmds

        axis_cmds = {}
        for axis_name, x_target, has_x_min_data in results:
            final_target = x_target if has_x_min_data else fallback_target
            status = 1 if chain_running and has_x_min_data else 0
            axis_cmds[axis_name] = build_axis(final_target, x_speed, status, x_speed_limit)
        return axis_cmds

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "left"))
        return self.motion_to_target._get_axis_current_pos(plc_data, axis_map[axis_name])

    def _is_chain_running(self, plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") == "moving_forward"
