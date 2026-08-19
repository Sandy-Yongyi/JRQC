import os
from dataclasses import dataclass

from model.motionplan.MachineAxisMap import get_axis_map, get_axis_position_limits, get_axis_safe_pos, get_axis_speed_limit
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx, clamp_to_limit_z
from model.motionplan.motionutil.MotionUtil import MotionUtil
from model.motionplan.motionutil.WorkpieceMotionHelper import WorkpieceMotionHelper
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader


@dataclass
class OutFxCompleteMotionState:
    stage: str = "wait_front"
    y_phase: str = "to_min"
    x_phase: str = "to_min"
    y_initialized: bool = False
    x_initialized: bool = False
    y_cycles: int = 0
    x_cycles: int = 0
    tracking_x_pre_target: int | None = None
    tracking_start_seen: bool = False


@dataclass
class OutFxCompleteContext:
    sn: int
    machine_cfg: dict
    runtime_cfg: dict
    plc_data: object
    block: object
    outside: object
    gun_group: object
    state: OutFxCompleteMotionState
    direct_profile: bool
    tracking: int
    chain_running: bool
    stage_changed: bool = False


@dataclass
class OutFxCompleteStateResult:
    axis_cmds: dict | None = None
    next_stage: str | None = None
    workpiece_complete: bool = False


class MotionOutFxCompleteWorkpiecePlanning:
    """完整工件模式下的仿形升降机状态机。"""

    CONFIG_DEFAULTS = {
        "frame_x_slow_in_out_enabled": 0,
        "frame_idle_y_reciprocate_enabled": 1,
    }

    def __init__(self, spray_cfg=None, motion_to_target=None, motion_util=None):
        config_dir = os.path.join(os.getcwd(), "model", "tomls")
        self.spray_cfg = spray_cfg if spray_cfg is not None else TomlLoader.load(os.path.join(config_dir, "SprayConfig.toml"))
        self.motion_to_target = motion_to_target if motion_to_target is not None else MotionToTarget()
        self.motion_util = motion_util if motion_util is not None else MotionUtil()
        self.spray_pos_tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10) or 10)
        self._work_states: dict[int, OutFxCompleteMotionState] = {}

    def reset_motion_state(self, sn=None):
        if sn is None:
            self._work_states.clear()
            return
        self._work_states.pop(int(sn), None)

    def auto_out_fx_complete_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue):
        """读取队首完整工件，按配置路径生成 OutFx 的 Z/Y/X 轴命令。"""
        sn = int(machine_cfg.get("sn", 0) or 0)
        state = self._work_states.setdefault(sn, OutFxCompleteMotionState())
        stop_chain = self.motion_util.check_z_limit(plc_data, machine_cfg)
        block = WorkpieceMotionHelper.peek_first_block(frame_queue)
        if block is None:
            if state.stage != "wait_front":
                state = OutFxCompleteMotionState()
                self._work_states[sn] = state
            return self._build_idle_axis_commands(machine_cfg, runtime_cfg, plc_data, state), False, stop_chain

        outside = self._get_outside(block)
        gun_group = self._get_outside_group(block, sn)
        if not self._is_valid_workpiece(outside, gun_group):
            axis_cmds, ready = self._build_return_safe_commands(machine_cfg, runtime_cfg, plc_data, state)
            if ready:
                self.reset_motion_state(sn)
            logger.warning(f"SN[{sn}] OutFx complete workpiece data or distribution is invalid, return safe and drop when ready")
            return axis_cmds, bool(ready), stop_chain

        direct_profile = self._get_config("frame_x_slow_in_out_enabled") == 0
        requested_tracking = int(runtime_cfg.get("tracking", machine_cfg.get("tracking", 0)) or 0)
        tracking = 0 if direct_profile else requested_tracking
        ctx = OutFxCompleteContext(sn, machine_cfg, runtime_cfg, plc_data, block, outside, gun_group, state, direct_profile, tracking, self._is_chain_running(plc_data))
        if state.stage == "wait_front" and self._has_after_outside_over_arrived(ctx):
            axis_cmds, ready = self._build_return_safe_commands(machine_cfg, runtime_cfg, plc_data, state)
            if ready:
                self.reset_motion_state(sn)
            logger.warning(f"SN[{sn}] OutFx complete workpiece already passed the work area, drop when X/Z are safe")
            return axis_cmds, bool(ready), stop_chain

        result = self._dispatch_state(ctx)
        return result.axis_cmds or {}, result.workpiece_complete, stop_chain

    def _dispatch_table(self):
        """所有配置共用一张状态表，配置只决定状态跳转路径。"""
        return {
            "wait_front": self._state_wait_front,
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
        dispatch_table = self._dispatch_table()
        for _ in range(4):
            handler = dispatch_table.get(ctx.state.stage)
            if handler is None:
                logger.error(f"SN[{ctx.sn}] unknown OutFx complete state: {ctx.state.stage}")
                return OutFxCompleteStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data))
            result = handler(ctx)
            if result.next_stage:
                self._set_stage(ctx, result.next_stage)
                ctx.stage_changed = True
            if result.axis_cmds is not None or result.workpiece_complete:
                return result
            if not result.next_stage:
                logger.error(f"SN[{ctx.sn}] OutFx complete state [{ctx.state.stage}] returned no commands and no next state")
                return OutFxCompleteStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data))
        logger.error(f"SN[{ctx.sn}] OutFx complete state transition exceeded the single-cycle limit")
        return OutFxCompleteStateResult(self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data))

    def _state_wait_front(self, ctx):
        if ctx.direct_profile and self._has_front_outside_arrived_or_over(ctx):
            return OutFxCompleteStateResult(next_stage="direct_profile")
        if ctx.tracking and self._has_tracking_preposition_arrived_or_over(ctx):
            return OutFxCompleteStateResult(next_stage="preposition")
        if not ctx.tracking and self._has_front_outside_arrived_or_over(ctx):
            return OutFxCompleteStateResult(next_stage="start")
        return OutFxCompleteStateResult(self._build_idle_axis_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state))

    def _state_direct_profile(self, ctx):
        if not ctx.stage_changed and self._has_after_outside_arrived_or_over(ctx):
            return OutFxCompleteStateResult(next_stage="return_safe")
        x_target, _ = self._get_outside_x_targets(ctx)
        axis_cmds = self._build_profile_axis_commands(ctx, x_target, self._get_x_position_speed(ctx))
        return OutFxCompleteStateResult(axis_cmds)

    def _state_preposition(self, ctx):
        if self._has_front_outside_arrived_or_over(ctx):
            ctx.state.tracking_start_seen = True
        target = self._get_tracking_pre_target(ctx)
        status = int(ctx.chain_running and self._has_front_outside_x_status_arrived_or_over(ctx))
        axis_cmds = self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data)
        axis_cmds["y"] = self._build_active_y_axis(ctx)
        axis_cmds["z"] = self._build_z_axis(ctx, tracking=False)
        x_cmds, x_ready = self._build_x_position_commands(ctx, target, self._get_x_position_speed(ctx), status)
        axis_cmds.update(x_cmds)
        if x_ready and ctx.state.tracking_start_seen:
            return OutFxCompleteStateResult(axis_cmds, next_stage="start")
        return OutFxCompleteStateResult(axis_cmds)

    def _state_start(self, ctx):
        if not ctx.tracking and not ctx.stage_changed and self._has_outside_front_center_arrived_or_over(ctx):
            return OutFxCompleteStateResult(next_stage="middle")
        if ctx.tracking:
            axis_cmds = self._build_tracking_reciprocate_commands(ctx)
            if self._tracking_cycles_complete(ctx):
                return OutFxCompleteStateResult(axis_cmds, next_stage="start_retract")
            return OutFxCompleteStateResult(axis_cmds)
        x_target = self._get_slow_profile_x_target(ctx)
        return OutFxCompleteStateResult(self._build_profile_axis_commands(ctx, x_target, self._get_x_position_speed(ctx)))

    def _state_start_retract(self, ctx):
        axis_cmds, x_ready = self._build_tracking_retract_commands(ctx)
        return OutFxCompleteStateResult(axis_cmds, next_stage="middle" if x_ready else None)

    def _state_middle(self, ctx):
        if not ctx.stage_changed and self._has_after_outside_arrived_or_over(ctx):
            return OutFxCompleteStateResult(next_stage="end" if ctx.tracking else "return_safe")
        x_target = self._get_slow_profile_x_target(ctx) if not ctx.tracking else self._get_outside_x_targets(ctx)[0]
        return OutFxCompleteStateResult(self._build_profile_axis_commands(ctx, x_target, self._get_x_position_speed(ctx)))

    def _state_end(self, ctx):
        axis_cmds = self._build_tracking_reciprocate_commands(ctx)
        if self._tracking_cycles_complete(ctx):
            return OutFxCompleteStateResult(axis_cmds, next_stage="end_retract")
        return OutFxCompleteStateResult(axis_cmds)

    def _state_end_retract(self, ctx):
        axis_cmds, x_ready = self._build_tracking_retract_commands(ctx)
        return OutFxCompleteStateResult(axis_cmds, next_stage="return_safe" if x_ready else None)

    def _state_return_safe(self, ctx):
        axis_cmds, ready = self._build_return_safe_commands(ctx.machine_cfg, ctx.runtime_cfg, ctx.plc_data, ctx.state)
        if not ready:
            return OutFxCompleteStateResult(axis_cmds)
        self.reset_motion_state(ctx.sn)
        logger.info(f"SN[{ctx.sn}] OutFx complete workpiece finished and return-safe condition is satisfied")
        return OutFxCompleteStateResult(axis_cmds, workpiece_complete=True)

    def _set_stage(self, ctx, next_stage):
        state = ctx.state
        if state.stage == next_stage:
            return
        logger.info(f"SN[{ctx.sn}] OutFx complete state: {state.stage} -> {next_stage}, tracking={ctx.tracking}")
        state.stage = next_stage
        if next_stage == "preposition":
            self._reset_reciprocation_state(state)
            state.tracking_x_pre_target = None
            state.tracking_start_seen = False
        elif next_stage == "start":
            self._reset_reciprocation_state(state)
        elif next_stage == "middle":
            state.x_phase = "to_min"
            state.x_initialized = False
            state.x_cycles = 0
        elif next_stage == "end":
            self._reset_reciprocation_state(state)
        elif next_stage in {"direct_profile", "return_safe", "wait_front"}:
            self._reset_reciprocation_state(state)
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

    def _build_profile_axis_commands(self, ctx, x_target, x_speed):
        axis_cmds = self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data)
        axis_cmds["y"] = self._build_active_y_axis(ctx)
        x_cmds, _ = self._build_x_position_commands(ctx, x_target, x_speed, int(ctx.chain_running))
        axis_cmds.update(x_cmds)
        axis_cmds["z"] = self._build_z_axis(ctx, tracking=False)
        return axis_cmds

    def _build_tracking_reciprocate_commands(self, ctx):
        axis_cmds = self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data)
        axis_cmds["y"] = self._build_active_y_axis(ctx)
        x_min, x_max = self._get_outside_x_targets(ctx)
        target, speed = self._resolve_tracking_x_target(ctx, x_min, x_max)
        x_cmds, _ = self._build_x_position_commands(ctx, target, speed, int(ctx.chain_running))
        axis_cmds.update(x_cmds)
        axis_cmds["z"] = self._build_z_axis(ctx, tracking=True)
        return axis_cmds

    def _resolve_tracking_x_target(self, ctx, x_min, x_max):
        enabled_axes = self._get_enabled_x_axis_names(ctx)
        positions = [self._get_axis_pos(ctx.machine_cfg, ctx.plc_data, axis_name) for axis_name in enabled_axes]
        if not ctx.state.x_initialized:
            if positions and all(abs(position - x_min) <= self.spray_pos_tolerance for position in positions):
                ctx.state.x_initialized = True
                ctx.state.x_phase = "to_max"
                return x_max, self._get_x_reciprocate_speed(ctx)
            ctx.state.x_phase = "to_min"
            return x_min, self._get_x_position_speed(ctx)

        target = x_max if ctx.state.x_phase == "to_max" else x_min
        if positions and all(abs(position - target) <= self.spray_pos_tolerance for position in positions):
            if ctx.state.x_phase == "to_max":
                ctx.state.x_phase = "to_min"
                target = x_min
            else:
                ctx.state.x_cycles += 1
                ctx.state.x_phase = "to_max"
                target = x_min
        return target, self._get_x_reciprocate_speed(ctx)

    def _build_tracking_retract_commands(self, ctx):
        axis_cmds = self.motion_to_target.hold_current_position(ctx.machine_cfg, ctx.plc_data)
        x_cmds, x_ready = self._build_x_position_commands(ctx, self._get_tracking_pre_target(ctx), self._get_x_position_speed(ctx), 0)
        axis_cmds.update(x_cmds)
        axis_cmds["y"] = self._build_hold_axis(ctx.machine_cfg, ctx.plc_data, "y")
        axis_cmds["z"] = self._build_z_axis(ctx, tracking=True)
        return axis_cmds, x_ready

    def _build_idle_axis_commands(self, machine_cfg, runtime_cfg, plc_data, state):
        axis_cmds, _ = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
        axis_cmds = axis_cmds or self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        if self._get_config("frame_idle_y_reciprocate_enabled") == 1:
            axis_cmds["y"] = self._build_idle_y_axis(machine_cfg, runtime_cfg, plc_data, state)
        return axis_cmds

    def _build_return_safe_commands(self, machine_cfg, runtime_cfg, plc_data, state):
        """空闲往复关闭时等待全部轴安全；开启时只等待 X/Z，Y持续往复。"""
        axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
        axis_cmds = axis_cmds or self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        if self._get_config("frame_idle_y_reciprocate_enabled") == 0:
            return axis_cmds, bool(all_ready)
        axis_cmds["y"] = self._build_idle_y_axis(machine_cfg, runtime_cfg, plc_data, state)
        return axis_cmds, self._are_xz_axes_safe(machine_cfg, plc_data)

    def _build_active_y_axis(self, ctx):
        y_range = self._get_group_y_range(ctx)
        if y_range is None:
            return self._build_safe_axis(ctx.machine_cfg, "y", self._get_y_position_speed(ctx))
        return self._build_y_reciprocate_axis(ctx.machine_cfg, ctx.plc_data, ctx.state, y_range[0], y_range[1], self._get_y_reciprocate_speed(ctx))

    def _build_idle_y_axis(self, machine_cfg, runtime_cfg, plc_data, state):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_min = clamp_to_limit_yx(int(runtime_cfg.get("y_move_min", machine_cfg.get("y_move_min", 0)) or 0), y_min_limit, y_max_limit)
        y_max = clamp_to_limit_yx(int(runtime_cfg.get("y_move_max", machine_cfg.get("y_move_max", 0)) or 0), y_min_limit, y_max_limit)
        speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)) or 100)
        return self._build_y_reciprocate_axis(machine_cfg, plc_data, state, y_min, y_max, speed)

    def _build_y_reciprocate_axis(self, machine_cfg, plc_data, state, y_min, y_max, speed):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        lower = clamp_to_limit_yx(int(y_min), y_min_limit, y_max_limit)
        upper = clamp_to_limit_yx(int(y_max), y_min_limit, y_max_limit)
        if lower > upper:
            lower, upper = upper, lower
        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")
        if not state.y_initialized:
            if abs(y_cur - lower) <= self.spray_pos_tolerance:
                state.y_initialized = True
                state.y_phase = "to_max"
                target = upper
            else:
                state.y_phase = "to_min"
                target = lower
        else:
            target = upper if state.y_phase == "to_max" else lower
            if abs(y_cur - target) <= self.spray_pos_tolerance:
                if state.y_phase == "to_max":
                    state.y_phase = "to_min"
                    target = lower
                else:
                    state.y_cycles += 1
                    state.y_phase = "to_max"
                    target = lower
        return build_axis(target, speed, 0, get_axis_speed_limit(machine_cfg, "y"))

    def _build_x_position_commands(self, ctx, target, speed, status):
        enabled_ids = self._get_enabled_gun_ids(ctx.gun_group)
        axis_cmds = {}
        all_ready = True
        for gun_id, axis_name in enumerate(self._get_x_axis_names(ctx.machine_cfg)):
            if gun_id in enabled_ids:
                final_target = self._clamp_x_target(ctx.machine_cfg, target)
                final_status = int(status)
            else:
                final_target = self._get_x_safe_target(ctx.machine_cfg)
                final_status = 0
            axis_cmds[axis_name] = build_axis(final_target, speed, final_status, get_axis_speed_limit(ctx.machine_cfg, axis_name))
            if abs(self._get_axis_pos(ctx.machine_cfg, ctx.plc_data, axis_name) - final_target) > self.spray_pos_tolerance:
                all_ready = False
        return axis_cmds, all_ready

    def _get_group_y_range(self, ctx):
        enabled_guns = [gun for gun in getattr(ctx.gun_group, "gundata_list", None) or [] if int(getattr(gun, "gun_y_enable", 0) or 0) == 1]
        if not enabled_guns:
            return None
        lower = max(int(getattr(gun, "gun_y_downer", 0) or 0) for gun in enabled_guns)
        upper = min(int(getattr(gun, "gun_y_upper", 0) or 0) for gun in enabled_guns)
        reduce_distance = max(0, int(ctx.runtime_cfg.get("recip_reduce_distance", ctx.machine_cfg.get("recip_reduce_distance", 0)) or 0))
        upper -= reduce_distance
        y_min_limit, y_max_limit = get_axis_position_limits(ctx.machine_cfg, "y")
        lower = clamp_to_limit_yx(lower, y_min_limit, y_max_limit)
        upper = clamp_to_limit_yx(max(lower, upper), y_min_limit, y_max_limit)
        return lower, upper

    def _get_outside_x_targets(self, ctx):
        x_position = int(ctx.machine_cfg.get("x_position", 0) or 0)
        front_offset = int(ctx.runtime_cfg.get("out_front_x_offset", ctx.machine_cfg.get("out_front_x_offset", 100)) or 100)
        after_offset = int(ctx.runtime_cfg.get("out_after_x_offset", ctx.machine_cfg.get("out_after_x_offset", 100)) or 100)
        x_min = int(getattr(ctx.outside, "outside_x_min", 0) or 0) - x_position - front_offset
        x_max = int(getattr(ctx.outside, "outside_x_max", 0) or 0) - x_position - after_offset
        x_min = self._clamp_x_target(ctx.machine_cfg, x_min)
        x_max = self._clamp_x_target(ctx.machine_cfg, x_max)
        return (x_min, x_max) if x_min <= x_max else (x_max, x_min)

    def _get_slow_profile_x_target(self, ctx):
        x_position = int(ctx.machine_cfg.get("x_position", 0) or 0)
        x_min = int(getattr(ctx.outside, "outside_x_min", 0) or 0)
        return self._clamp_x_target(ctx.machine_cfg, x_min - x_position - self._resolve_slow_x_offset(ctx))

    def _resolve_slow_x_offset(self, ctx):
        """按工件前沿和后沿相对设备中心的位置计算慢进慢出偏移。"""
        front_offset = max(0, int(ctx.runtime_cfg.get("out_front_x_offset", ctx.machine_cfg.get("out_front_x_offset", 100)) or 100))
        front_z_offset = max(0, int(ctx.runtime_cfg.get("out_z_front_offset", ctx.machine_cfg.get("out_z_front_offset", 100)) or 100))
        after_z_offset = max(0, int(ctx.runtime_cfg.get("out_z_after_offset", ctx.machine_cfg.get("out_z_after_offset", 100)) or 100))
        machine_center = int(ctx.machine_cfg.get("z_position", 0) or 0) + self._get_axis_pos(ctx.machine_cfg, ctx.plc_data, "z")
        front_chain = int(ctx.block.fifo_frame_pos or 0) - int(getattr(ctx.outside, "outside_z_min", 0) or 0)
        after_chain = int(ctx.block.fifo_frame_pos or 0) - int(getattr(ctx.outside, "outside_z_max", 0) or 0)
        if front_chain < machine_center:
            if front_z_offset <= 0:
                return front_offset
            offset = front_offset - front_offset / front_z_offset * (machine_center - front_chain)
            return self._clamp_slow_offset(offset, front_offset)
        if after_chain < machine_center:
            return front_offset
        if after_z_offset <= 0:
            return 0
        offset = front_offset - front_offset / after_z_offset * (after_chain - machine_center)
        return self._clamp_slow_offset(offset, front_offset)

    def _get_tracking_pre_target(self, ctx):
        if ctx.state.tracking_x_pre_target is not None:
            return ctx.state.tracking_x_pre_target
        x_position = int(ctx.machine_cfg.get("x_position", 0) or 0)
        front_offset = int(ctx.runtime_cfg.get("out_front_x_offset", ctx.machine_cfg.get("out_front_x_offset", 100)) or 100)
        pre_distance = max(0, int(self.spray_cfg.get("x_pre_distance", 0) or 0))
        base_x_min = int(getattr(ctx.outside, "outside_x_min", 0) or 0)
        ctx.state.tracking_x_pre_target = self._clamp_x_target(ctx.machine_cfg, base_x_min - x_position - front_offset - pre_distance)
        return ctx.state.tracking_x_pre_target

    def _tracking_cycles_complete(self, ctx):
        cycle_axis = str(self.spray_cfg.get("side_2d_cycle_axis", "y") or "y").strip().lower()
        completed = ctx.state.x_cycles if cycle_axis == "x" else ctx.state.y_cycles
        total = int(ctx.runtime_cfg.get("outside_total_cycles", ctx.machine_cfg.get("outside_total_cycles", 1)) or 1)
        return completed >= max(1, total)

    def _build_z_axis(self, ctx, tracking):
        z_min_limit, z_max_limit = get_axis_position_limits(ctx.machine_cfg, "z")
        speed_limit = get_axis_speed_limit(ctx.machine_cfg, "z")
        if tracking:
            target = clamp_to_limit_z(z_max_limit, z_min_limit, z_max_limit)
            speed = self._resolve_follow_z_speed(ctx.plc_data)
        elif ctx.tracking and ctx.state.stage == "middle":
            target = clamp_to_limit_z(get_axis_safe_pos(ctx.machine_cfg, "z"), z_min_limit, z_max_limit)
            speed = int(ctx.runtime_cfg.get("z_back_speed", ctx.machine_cfg.get("z_back_speed", 100)) or 100)
        else:
            target = clamp_to_limit_z(get_axis_safe_pos(ctx.machine_cfg, "z"), z_min_limit, z_max_limit)
            speed = int(ctx.runtime_cfg.get("z_zeroing_speed", ctx.machine_cfg.get("z_zeroing_speed", 100)) or 100)
        return build_axis(target, speed, 0, speed_limit)

    def _has_tracking_preposition_arrived_or_over(self, ctx):
        z_position = int(ctx.machine_cfg.get("z_position", 0) or 0)
        front_offset = int(ctx.runtime_cfg.get("out_z_front_offset", ctx.machine_cfg.get("out_z_front_offset", 100)) or 100)
        x_status_offset = self._get_x_status_offset(ctx)
        machine_z = z_position - front_offset - x_status_offset
        chain_z = int(ctx.block.fifo_frame_pos or 0) - int(getattr(ctx.outside, "outside_z_min", 0) or 0)
        return self._has_z_arrived_or_over(ctx, machine_z, chain_z)

    def _has_front_outside_arrived_or_over(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_front_z_pair(self._build_workpiece_context(ctx), "out_z_front_offset", "outside_z_min")
        return self._has_z_arrived_or_over(ctx, machine_z, chain_z)

    def _has_front_outside_x_status_arrived_or_over(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_front_z_pair(self._build_workpiece_context(ctx), "out_z_front_offset", "outside_z_min")
        return self._has_z_arrived_or_over(ctx, machine_z - self._get_x_status_offset(ctx), chain_z)

    def _has_outside_front_center_arrived_or_over(self, ctx):
        machine_z = int(ctx.machine_cfg.get("z_position", 0) or 0)
        chain_z = int(ctx.block.fifo_frame_pos or 0) - int(getattr(ctx.outside, "outside_z_min", 0) or 0)
        return self._has_z_arrived_or_over(ctx, machine_z, chain_z)

    def _has_after_outside_arrived_or_over(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_after_z_pair(self._build_workpiece_context(ctx), "out_z_after_offset", "outside_z_max")
        return self._has_z_arrived_or_over(ctx, machine_z, chain_z)

    def _has_after_outside_over_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_after_z_pair(self._build_workpiece_context(ctx), "out_z_after_offset", "outside_z_max")
        return self.motion_util.has_over_arrive_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    @staticmethod
    def _build_workpiece_context(ctx):
        return {
            "machine_cfg": ctx.machine_cfg,
            "runtime_cfg": ctx.runtime_cfg,
            "fifo_frame_pos": int(ctx.block.fifo_frame_pos or 0),
            "outside_data": [ctx.outside],
        }

    def _has_z_arrived_or_over(self, ctx, machine_z, chain_z):
        compare_machine_z = self._get_compare_machine_z(ctx, machine_z)
        return self.motion_util.has_arrived_z(compare_machine_z, chain_z) or self.motion_util.has_over_arrive_z(compare_machine_z, chain_z)

    def _get_compare_machine_z(self, ctx, machine_z):
        return int(machine_z or 0) + self._get_axis_pos(ctx.machine_cfg, ctx.plc_data, "z")

    def _are_xz_axes_safe(self, machine_cfg, plc_data):
        for axis_name in machine_cfg.get("axis_type", []):
            if axis_name != "z" and not axis_name.startswith("x"):
                continue
            safe_target = self._get_safe_target(machine_cfg, axis_name)
            if abs(self._get_axis_pos(machine_cfg, plc_data, axis_name) - safe_target) > self.spray_pos_tolerance:
                return False
        return True

    def _build_safe_axis(self, machine_cfg, axis_name, speed):
        return build_axis(self._get_safe_target(machine_cfg, axis_name), speed, 0, get_axis_speed_limit(machine_cfg, axis_name))

    def _build_hold_axis(self, machine_cfg, plc_data, axis_name):
        current = self._get_axis_pos(machine_cfg, plc_data, axis_name)
        min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
        return build_axis(clamp_to_limit_yx(current, min_limit, max_limit), 0, 0, get_axis_speed_limit(machine_cfg, axis_name))

    def _get_safe_target(self, machine_cfg, axis_name):
        min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
        safe_position = get_axis_safe_pos(machine_cfg, axis_name)
        if axis_name == "z":
            return clamp_to_limit_z(safe_position, min_limit, max_limit)
        return clamp_to_limit_yx(safe_position, min_limit, max_limit)

    def _get_x_safe_target(self, machine_cfg):
        return self._get_safe_target(machine_cfg, "x")

    def _clamp_x_target(self, machine_cfg, target):
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        return clamp_to_limit_yx(int(target), x_min_limit, x_max_limit)

    def _get_enabled_x_axis_names(self, ctx):
        enabled_ids = self._get_enabled_gun_ids(ctx.gun_group)
        return [axis_name for gun_id, axis_name in enumerate(self._get_x_axis_names(ctx.machine_cfg)) if gun_id in enabled_ids]

    @staticmethod
    def _get_enabled_gun_ids(gun_group):
        return {
            int(getattr(gun, "gun_id"))
            for gun in getattr(gun_group, "gundata_list", None) or []
            if getattr(gun, "gun_id", None) is not None and int(getattr(gun, "gun_y_enable", 0) or 0) == 1
        }

    @staticmethod
    def _get_x_axis_names(machine_cfg):
        return [axis_name for axis_name in machine_cfg.get("axis_type", []) if axis_name.startswith("x")]

    @staticmethod
    def _get_outside(block):
        outside_data = getattr(block, "outside_data", None) or []
        return outside_data[0] if outside_data else None

    @staticmethod
    def _get_outside_group(block, sn):
        for machine_data in getattr(block, "distribe_gun_list", None) or []:
            machine_id = getattr(machine_data, "machine_id", None)
            if machine_id is None or int(machine_id) != int(sn):
                continue
            for gun_group in getattr(machine_data, "gun_groups", None) or []:
                if getattr(gun_group, "group_type", None) == "outside":
                    return gun_group
        return None

    def _is_valid_workpiece(self, outside, gun_group):
        if outside is None or not self._get_enabled_gun_ids(gun_group):
            return False
        required_values = ("outside_x_min", "outside_x_max", "outside_y_min", "outside_y_max", "outside_z_min", "outside_z_max")
        return all(getattr(outside, name, None) is not None for name in required_values)

    def _get_config(self, name):
        default = self.CONFIG_DEFAULTS[name]
        value = self.spray_cfg.get(name, default)
        return default if value is None else int(value)

    @staticmethod
    def _clamp_slow_offset(offset, max_offset):
        return int(max(0, min(float(max_offset), float(offset))))

    def _get_x_status_offset(self, ctx):
        return int(ctx.runtime_cfg.get("x_status_offset", ctx.machine_cfg.get("x_status_offset", 100)) or 100)

    @staticmethod
    def _get_x_position_speed(ctx):
        return int(ctx.runtime_cfg.get("x_pos_speed", ctx.machine_cfg.get("x_pos_speed", 100)) or 100)

    @staticmethod
    def _get_x_reciprocate_speed(ctx):
        return int(ctx.runtime_cfg.get("x_recip_speed", ctx.machine_cfg.get("x_recip_speed", 100)) or 100)

    @staticmethod
    def _get_y_position_speed(ctx):
        return int(ctx.runtime_cfg.get("y_pos_speed", ctx.machine_cfg.get("y_pos_speed", 100)) or 100)

    @staticmethod
    def _get_y_reciprocate_speed(ctx):
        return int(ctx.runtime_cfg.get("y_recip_speed", ctx.machine_cfg.get("y_recip_speed", 100)) or 100)

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "left"))
        return self.motion_to_target._get_axis_current_pos(plc_data, axis_map[axis_name])

    def _resolve_follow_z_speed(self, plc_data):
        chain_speed = int(getattr(plc_data, "ChainSpeedMM", getattr(plc_data, "ChainSpeed", 0)) or 0)
        return chain_speed if self._is_chain_running(plc_data) else 0

    @staticmethod
    def _is_chain_running(plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") == "moving_forward"
