import os
from model.motionplan.MachineAxisMap import get_axis_map, get_axis_position_limits, get_axis_safe_pos, get_axis_speed_limit
from model.motionplan.MotionReciprocate import MotionReciprocate
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_r, clamp_to_limit_yx, clamp_to_limit_z
from model.motionplan.motionutil.MotionUtil import MotionUtil
from model.motionplan.motionutil.WorkpieceMotionHelper import WorkpieceMotionHelper
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader


class MotionXNSidePlanning:
    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\sprayconfig.toml")
        self.process_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\ProcessConfig.toml")
        self.spray_pos_tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10) or 10)
        self.rect_threshold = int(self.spray_cfg.get("rect_threshold", 100) or 100)
        self.side_back_end_face_enabled = self._normalize_enabled_flag(self.spray_cfg.get("side_back_end_face_enabled", 1))
        self.side_rotate_enabled = self._normalize_enabled_flag(self.spray_cfg.get("side_rotate_enabled", 1))
        self.x_range = int(self.process_cfg.get("x_range", 300) or 300)
        self.y_range = int(self.process_cfg.get("y_range", 300) or 300)
        self.z_range = int(self.process_cfg.get("z_range", 300) or 300)
        self.motiontotarget = MotionToTarget()
        self.recip_manager = MotionReciprocate()
        self.motion_util = MotionUtil()
        self._work_states: dict[int, dict] = {}

    def reset_side_motion_state(self, sn=None):
        WorkpieceMotionHelper.reset_state(self._work_states, sn)
        state_prefix = None if sn is None else f"sn{int(sn)}:"
        self.recip_manager.reset_states(state_prefix)

    def auto_xn_side_machine_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue):
        """
        执行侧面机台的自动运动控制逻辑。

        该函数根据当前的帧队列状态、喷涂模式以及机器配置，计算并返回下一步的运动指令。
        它处理了空闲状态下的归位逻辑、特定模式下的安全预检查以及常规喷涂流程的状态管理。

        Args:
            machine_cfg (dict): 机器配置参数，包含序列号sn等硬件相关配置。
            runtime_cfg (dict): 运行时配置参数。
            plc_data (dict): PLC实时数据，包含传感器状态、轴位置等。
            frame_queue (queue.Queue): 待处理的运动帧队列。

        Returns:
            tuple: 包含以下三个元素的元组:
                - axis_cmds (dict/list): 生成的轴运动指令。
                - done (bool): 标识当前侧喷任务是否已完成。
                - stop_chain (bool): 标识是否触发Z轴限位保护，需要停止链路。
        """
        sn = int(machine_cfg.get("sn", 0) or 0)
        stop_chain = self.motion_util.check_z_limit(plc_data, machine_cfg)
        WorkpieceMotionHelper.ensure_state(self._work_states, sn, self._create_initial_state)

        block = WorkpieceMotionHelper.peek_first_block(frame_queue)
        if block is None:
            self.reset_side_motion_state(sn)
            axis_cmds, _ = self.motiontotarget.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
            if axis_cmds is None:
                axis_cmds = self.motiontotarget.hold_current_position(machine_cfg, plc_data)
            return axis_cmds, False, stop_chain

        spray_mode = self._resolve_spray_mode(block)
        mode_runtime_cfg = self._resolve_mode_runtime_cfg(runtime_cfg, spray_mode)
        self._ensure_mode_state(sn, spray_mode)

        if spray_mode == "cabinet" and self._work_states[sn]["state"] == "wait_front_outside":
            axis_data, _, drop = self.motion_util.precheck_z_safety_and_drop_blocks(
                machine_cfg=machine_cfg,
                runtime_cfg=mode_runtime_cfg,
                plc_data=plc_data,
                block=block,
                reset_state_cb=lambda: self.reset_side_motion_state(sn),
            )
            if drop:
                return axis_data, True, stop_chain

        ctx = self._build_context(machine_cfg, mode_runtime_cfg, plc_data, block, spray_mode)
        done = self._handle_side_spray(ctx)
        axis_cmds = ctx.get("side_machine") or self.motiontotarget.hold_current_position(machine_cfg, plc_data)
        return axis_cmds, done, stop_chain

    @staticmethod
    def _create_initial_state():
        return {"state": "wait_front_outside", "inside_idx": 0, "spray_mode": None, "skip_inside_recip": False}

    def _build_context(self, machine_cfg, runtime_cfg, plc_data, block, spray_mode):
        ctx = WorkpieceMotionHelper.build_block_context(machine_cfg, runtime_cfg, plc_data, block)
        ctx["side_machine"] = None
        ctx["rect_threshold"] = self.rect_threshold
        ctx["spray_mode"] = spray_mode
        return ctx

    def _resolve_spray_mode(self, block):
        if self.motion_util.check_if_cabinet(block, self.x_range, self.y_range, self.z_range):
            # logger.debug("spray_mode is cabinet")
            return "cabinet"
        # logger.debug("spray_mode is flat")
        return "flat"

    @staticmethod
    def _resolve_mode_runtime_cfg(runtime_cfg, spray_mode):
        if not isinstance(runtime_cfg, dict):
            return {}

        mode_runtime_cfg = {key: value for key, value in runtime_cfg.items() if key != "flat"}
        if spray_mode == "flat" and isinstance(runtime_cfg.get("flat"), dict):
            mode_runtime_cfg.update(runtime_cfg["flat"])
        return mode_runtime_cfg

    def _ensure_mode_state(self, sn, spray_mode):
        state = self._work_states[sn]
        last_mode = state.get("spray_mode")
        if last_mode is not None and last_mode != spray_mode:
            self._work_states[sn] = self._create_initial_state()
            state = self._work_states[sn]
        state["spray_mode"] = spray_mode

    def _handle_side_spray(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        state_name = self._work_states[sn].get("state", "wait_front_outside")
        dispatch_table = self._dispatch_table_side() if ctx.get("spray_mode") == "cabinet" else self._dispatch_table_flat_side()
        handler = dispatch_table.get(state_name)
        if handler is None:
            return False

        next_state = handler(ctx)
        if next_state == "finish":
            self._work_states[sn]["state"] = "finish"
            self._state_finish(ctx)
            return True
        if next_state:
            self._reset_recip_state_on_step_change(ctx, state_name, next_state)
            self._work_states[sn]["state"] = next_state
        if state_name == "finish":
            self._state_finish(ctx)
            return True
        return False

    def _reset_recip_state_on_step_change(self, ctx, current_state, next_state):
        if not next_state or next_state == current_state:
            return

        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        current_inside_idx = self._get_current_inside_idx(ctx)
        state_prefix_map = {
            "recip_outside": [self._state_prefix(sn, "flat_outside")],
            "recip_front_outside": [self._state_prefix(sn, "front_outside")],
            "recip_front_inside": [self._state_prefix(sn, f"inside_{current_inside_idx}_front")],
            "recip_inside_end_face": [self._state_prefix(sn, f"inside_{current_inside_idx}_end_face")],
            "recip_after_inside": [self._state_prefix(sn, f"inside_{current_inside_idx}_after")],
            "recip_after_outside": [self._state_prefix(sn, "after_outside")],
        }

        for state_prefix in state_prefix_map.get(next_state, []):
            self.recip_manager.reset_states(state_prefix)

    def _dispatch_table_side(self):
        return {
            "wait_front_outside": self._state_wait_front_outside,
            "recip_front_outside": self._state_recip_front_outside,
            "front_outside_out_gun": self._state_front_outside_out_gun,
            "wait_front_inside": self._state_wait_front_inside,
            "recip_front_inside": self._state_recip_front_inside,
            "front_inside_out_gun": self._state_front_inside_out_gun,
            "recip_inside_end_face": self._state_recip_inside_end_face,
            "inside_end_face_out_gun": self._state_inside_end_face_out_gun,
            "wait_after_inside": self._state_wait_after_inside,
            "recip_after_inside": self._state_recip_after_inside,
            "after_inside_out_gun": self._state_after_inside_out_gun,
            "wait_after_outside": self._state_wait_after_outside,
            "recip_after_outside": self._state_recip_after_outside,
            "return_origin": self._state_return_origin,
            "finish": self._state_finish,
        }

    def _dispatch_table_flat_side(self):
        return {
            "wait_front_outside": self._state_wait_front_outside,
            "recip_outside": self._state_recip_outside,
            "return_origin": self._state_return_origin,
            "finish": self._state_finish,
        }

    def _state_wait_front_outside(self, ctx):
        spray_mode = ctx.get("spray_mode")
        x_status = 1 if spray_mode == "cabinet" and self._has_front_outside_x_status_arrived(ctx) else 0
        ctx["side_machine"] = self._build_position_axes(ctx, self._get_outside_group(ctx), -90, x_status=x_status)
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)

        if self._has_front_outside_arrived(ctx):
            logger.info(f"sn:[{sn}] side arrived front outside")
            return "recip_front_outside" if spray_mode == "cabinet" else "recip_outside"
        if self._has_front_outside_over_arrived(ctx):
            logger.info(f"sn:[{sn}] side over arrive front outside")
            if spray_mode == "flat":
                return "recip_outside"
            if self._switch_to_next_enterable_inside(ctx, 0):
                return "wait_front_inside"
            return "wait_after_outside"
        return None

    def _state_recip_outside(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        group = self._get_outside_group(ctx)
        x_min, x_max = self._get_outside_x_range(ctx)
        enabled_guns = list(self._get_enabled_guns_by_id(group).values())
        y_range = self.recip_manager._get_group_y_range(ctx["machine_cfg"], enabled_guns, ctx["runtime_cfg"])
        if y_range is None:
            y_safe = get_axis_safe_pos(ctx["machine_cfg"], "y", default=0)
            y_lower, y_upper = y_safe, y_safe
        else:
            y_lower, y_upper = y_range
        axis_cmds = self.recip_manager.build_side_end_face_reciprocate(
            machine_cfg=ctx["machine_cfg"],
            runtime_cfg=ctx["runtime_cfg"],
            plc_data=ctx["plc_data"],
            gun_group=group,
            x_target=x_min,
            y_min=y_lower,
            y_max=y_upper,
            r_angle=self._resolve_side_r_angle(180),
            state_key=self._state_prefix(sn, "flat_outside"),
            z_target=self._get_safe_z_target(ctx["machine_cfg"]),
            z_speed=self._get_back_z_speed(ctx),
            x_active_status=3,
        )
        ctx["side_machine"] = axis_cmds
        if self._has_after_outside_arrived(ctx) or self._has_after_outside_over_arrived(ctx):
            logger.info(f"sn:[{sn}] side flat outside reached after outside, return origin")
            return "return_origin"
        return None

    def _state_recip_front_outside(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        group = self._get_outside_group(ctx)
        x_min, x_max = self._get_outside_x_range(ctx)
        axis_cmds, done = self.recip_manager.build_side_reciprocate(
            machine_cfg=ctx["machine_cfg"],
            runtime_cfg=ctx["runtime_cfg"],
            plc_data=ctx["plc_data"],
            gun_group=group,
            x_min=x_min,
            x_max=x_max,
            r_angle=self._resolve_side_r_angle(-90),
            rect_threshold=ctx["rect_threshold"],
            state_key=self._state_prefix(sn, "front_outside"),
            x_recip_status=1,
            keep_x_status_when_chain_stopped=True,
            total_cycles_key="outside_total_cycles",
        )
        ctx["side_machine"] = axis_cmds
        if done:
            logger.info(f"sn:[{sn}] side reciprocate front outside out gun")
            return "front_outside_out_gun"
        return None

    def _state_front_outside_out_gun(self, ctx):
        axis_cmds, ready = self._prepare_next_stage(ctx, self._get_outside_group(ctx), 0)
        ctx["side_machine"] = axis_cmds
        if not ready:
            return None
        if self._switch_to_next_enterable_inside(ctx, 0):
            sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
            inside_idx = self._get_current_inside_idx(ctx)
            inside_column = self._get_current_inside_column(ctx)
            inside_id = None if inside_column is None else getattr(inside_column, "inside_id", None)
            logger.info(f"sn:[{sn}] side front outside out gun switch to inside: inside_idx={inside_idx}, inside_id={inside_id}")
            return "wait_front_inside"
        self._log_no_enterable_inside(ctx, "front_outside_out_gun")
        return "wait_after_outside"

    def _state_wait_front_inside(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        if not self._ensure_current_enterable_inside(ctx):
            return "wait_after_outside"

        ctx["side_machine"] = self._build_position_axes(ctx, self._get_current_inside_group(ctx), 90)
        after_over = self._has_after_inside_over_arrived(ctx)
        front_ready = self._has_front_inside_arrived(ctx) or self._has_front_inside_over_arrived(ctx)
        has_enabled_guns = self._current_inside_has_enabled_guns(ctx)

        if after_over:
            if self._switch_to_next_enterable_inside(ctx, self._get_current_inside_idx(ctx) + 1):
                logger.info(f"sn:[{sn}] side current inside already passed, switch next inside")
                return "wait_front_inside"
            return "wait_after_outside"
        if front_ready:
            if not has_enabled_guns:
                self._work_states[sn]["skip_inside_recip"] = True
                logger.info(f"sn:[{sn}] side current inside has no enabled guns, skip front inside recip")
                if not self.side_back_end_face_enabled:
                    return "wait_after_outside"
                return "recip_inside_end_face"
            self._work_states[sn]["skip_inside_recip"] = False
            logger.info(f"sn:[{sn}] side current inside ready for front spray")
            return "recip_front_inside"
        return None

    def _state_recip_front_inside(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        self._work_states[sn]["skip_inside_recip"] = False
        group = self._get_current_inside_group(ctx)
        x_min, x_max = self._get_inside_spray_x_range(ctx)
        axis_cmds, done = self.recip_manager.build_side_reciprocate(
            machine_cfg=ctx["machine_cfg"],
            runtime_cfg=ctx["runtime_cfg"],
            plc_data=ctx["plc_data"],
            gun_group=group,
            x_min=x_min,
            x_max=x_max,
            r_angle=self._resolve_side_r_angle(90),
            rect_threshold=ctx["rect_threshold"],
            state_key=self._state_prefix(sn, f"inside_{self._get_current_inside_idx(ctx)}_front"),
            x_recip_status=2,
            keep_x_status_when_chain_stopped=True,
            total_cycles_key="inside_total_cycles",
        )
        ctx["side_machine"] = axis_cmds
        if done:
            if self._can_enter_inside_column(ctx, self._get_current_inside_idx(ctx)):
                if self._has_after_inside_over_arrived(ctx):
                    logger.info(f"sn:[{sn}] side front inside done but already over after inside, direct out gun")
                    return "after_inside_out_gun"
                if not self.side_back_end_face_enabled:
                    logger.info(f"sn:[{sn}] side front inside done, not skip inside end face and start out gun")
                    return "front_inside_out_gun"
                logger.info(f"sn:[{sn}] side front inside done, continue inside end face without out gun")
                return "recip_inside_end_face"
            logger.info(f"sn:[{sn}] side front inside done, inside span not enough for end face spray, continue after inside spray")
            return "recip_after_inside"
        return None

    def _state_front_inside_out_gun(self, ctx):
        axis_cmds, ready = self._prepare_next_stage(ctx, self._get_current_inside_group(ctx), 180)
        ctx["side_machine"] = axis_cmds
        if not ready:
            return None
        if not self.side_back_end_face_enabled:
            return "wait_after_inside"
        return "recip_inside_end_face"

    def _state_recip_inside_end_face(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        state = self._work_states[sn]
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        gun_group = self._get_current_inside_group(ctx)
        recip_gun_group = gun_group
        if state.get("skip_inside_recip", False):
            outside = (ctx.get("outside_data") or [None])[0]
            outside_x_min = int(getattr(outside, "outside_x_min", 0) or 0) if outside is not None else 0
            x_target = outside_x_min - int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 100)) or 100) - self._get_x_position(machine_cfg)
            outside_group = self._get_outside_group(ctx)
            recip_gun_group = outside_group
            y_range = self._get_group_y_full_range(outside_group, machine_cfg, runtime_cfg)
            if y_range is None:
                y_safe = get_axis_safe_pos(machine_cfg, "y", default=0)
                y_lower, y_upper = y_safe, y_safe
            else:
                y_lower, y_upper = y_range
        else:
            _, x_max, _, _, _, _ = self._get_current_inside_xyz_range(ctx)
            x_target = int(x_max or 0) - int(runtime_cfg.get("in_after_x_offset", machine_cfg.get("in_after_x_offset", 100)) or 100) - self._get_x_position(machine_cfg)
            y_lower, y_upper = self._get_inside_end_face_y_range(ctx, gun_group)
            if y_lower == y_upper:
                logger.info(f"sn:[{sn}] side inside end face y range is 0, skip reciprocate and go out gun")
                ctx["side_machine"] = self.motiontotarget.hold_current_position(machine_cfg, ctx["plc_data"])
                return "after_inside_out_gun"

        ctx["side_machine"] = self.recip_manager.build_side_end_face_reciprocate(
            machine_cfg=machine_cfg,
            runtime_cfg=runtime_cfg,
            plc_data=ctx["plc_data"],
            gun_group=recip_gun_group,
            x_target=x_target,
            y_min=y_lower,
            y_max=y_upper,
            r_angle=self._resolve_side_r_angle(180),
            state_key=self._state_prefix(sn, f"inside_{self._get_current_inside_idx(ctx)}_end_face"),
            z_target=self._get_safe_z_target(machine_cfg),
            z_speed=self._get_back_z_speed(ctx),
        )

        if self._has_after_inside_arrived(ctx):
            logger.info(f"sn:[{sn}] side inside end face reached after inside")
            if state.get("skip_inside_recip", False):
                state["skip_inside_recip"] = False
                return "after_inside_out_gun"
            return "recip_after_inside"
        if self._has_after_inside_over_arrived(ctx):
            logger.info(f"sn:[{sn}] side inside end face already over after inside, direct out gun")
            if state.get("skip_inside_recip", False):
                state["skip_inside_recip"] = False
            return "after_inside_out_gun"
        return None

    def _state_inside_end_face_out_gun(self, ctx):
        axis_cmds, ready = self._prepare_next_stage(ctx, self._get_current_inside_group(ctx), 180)
        ctx["side_machine"] = axis_cmds
        if not ready:
            return None
        return "recip_after_inside"

    def _state_wait_after_inside(self, ctx):
        if not self._ensure_current_enterable_inside(ctx):
            return "wait_after_outside"

        ctx["side_machine"] = self._build_position_axes(ctx, self._get_current_inside_group(ctx), -90)
        if self._has_after_inside_arrived(ctx):
            return "recip_after_inside"
        if self._has_after_inside_over_arrived(ctx):
            return "after_inside_out_gun"
        return None

    def _state_recip_after_inside(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        group = self._get_current_inside_group(ctx)
        x_min, x_max = self._get_inside_spray_x_range(ctx)
        axis_cmds, done = self.recip_manager.build_side_reciprocate(
            machine_cfg=ctx["machine_cfg"],
            runtime_cfg=ctx["runtime_cfg"],
            plc_data=ctx["plc_data"],
            gun_group=group,
            x_min=x_min,
            x_max=x_max,
            r_angle=self._resolve_side_r_angle(-90),
            rect_threshold=ctx["rect_threshold"],
            state_key=self._state_prefix(sn, f"inside_{self._get_current_inside_idx(ctx)}_after"),
            x_recip_status=2,
            keep_x_status_when_chain_stopped=True,
            total_cycles_key="inside_total_cycles",
        )
        ctx["side_machine"] = axis_cmds
        if done:
            logger.info(f"sn:[{sn}] side reciprocate after inside out gun")
            return "after_inside_out_gun"
        return None

    def _state_after_inside_out_gun(self, ctx):
        axis_cmds, ready = self._prepare_next_stage(ctx, self._get_current_inside_group(ctx), 0)
        ctx["side_machine"] = axis_cmds
        if not ready:
            return None
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        self._work_states[sn]["skip_inside_recip"] = False
        if self._switch_to_next_enterable_inside(ctx, self._get_current_inside_idx(ctx) + 1):
            return "wait_front_inside"
        return "wait_after_outside"

    def _state_wait_after_outside(self, ctx):
        x_status = 1 if self._has_after_outside_x_status_arrived(ctx) else 0
        ctx["side_machine"] = self._build_position_axes(ctx, self._get_outside_group(ctx), 90, x_status=x_status)
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)

        if self._has_after_outside_arrived(ctx):
            logger.info(f"sn:[{sn}] side arrived after outside")
            return "recip_after_outside"
        if self._has_after_outside_over_arrived(ctx):
            logger.info(f"sn:[{sn}] side over arrive after outside")
            return "finish"
        return None

    def _state_recip_after_outside(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        group = self._get_outside_group(ctx)
        x_min, x_max = self._get_outside_x_range(ctx)
        axis_cmds, done = self.recip_manager.build_side_reciprocate(
            machine_cfg=ctx["machine_cfg"],
            runtime_cfg=ctx["runtime_cfg"],
            plc_data=ctx["plc_data"],
            gun_group=group,
            x_min=x_min,
            x_max=x_max,
            r_angle=self._resolve_side_r_angle(90),
            rect_threshold=ctx["rect_threshold"],
            state_key=self._state_prefix(sn, "after_outside"),
            x_recip_status=1,
            keep_x_status_when_chain_stopped=True,
            total_cycles_key="outside_total_cycles",
        )
        ctx["side_machine"] = axis_cmds
        if done:
            logger.info(f"sn:[{sn}] side after outside done, start return origin")
            return "return_origin"
        return None

    def _state_return_origin(self, ctx):
        axis_cmds, done = self.motiontotarget.move_to_origin_safe(
            machine_cfg=ctx["machine_cfg"],
            runtime_cfg=ctx["runtime_cfg"],
            plc_data=ctx["plc_data"],
        )
        if axis_cmds is None:
            axis_cmds = self.motiontotarget.hold_current_position(ctx["machine_cfg"], ctx["plc_data"])
        ctx["side_machine"] = axis_cmds
        if done:
            logger.info(f"sn:[{ctx['machine_cfg']['sn']}] side return origin done")
            return "finish"
        return None

    def _state_finish(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        self.reset_side_motion_state(sn)
        logger.info(f"sn:[{sn}] side finish spraying")
        ctx["side_machine"] = self.motiontotarget.hold_current_position(ctx["machine_cfg"], ctx["plc_data"])
        return None

    def _build_position_axes(self, ctx, gun_group, r_angle, x_status=0):
        r_angle = self._resolve_side_r_angle(r_angle)
        machine_cfg = ctx["machine_cfg"]
        plc_data = ctx["plc_data"]
        runtime_cfg = ctx["runtime_cfg"]
        axis_cmds = self.motiontotarget.hold_current_position(machine_cfg, plc_data)
        axis_cmds["z"] = self._build_z_axis(machine_cfg, self._get_safe_z_target(machine_cfg), self._get_back_z_speed(ctx))

        enabled_by_id = self._get_enabled_guns_by_id(gun_group)
        status_enabled_by_id = self._get_enabled_guns_by_id(gun_group) if x_status else None
        y_speed = int(runtime_cfg.get("y_pos_speed", machine_cfg.get("y_pos_speed", 100)) or 100)
        if self.recip_manager._is_independent_y_mode(machine_cfg):
            gun_ranges = self.recip_manager._get_independent_y_ranges(machine_cfg, gun_group, runtime_cfg)
            spray_num = int(machine_cfg.get("spray_num", 0) or 0)
            axis_cmds.update(self.recip_manager._build_independent_fixed_y_axes(machine_cfg, plc_data, spray_num, gun_ranges, y_speed, y_speed))
        else:
            axis_cmds["y"] = self._build_y_axis(machine_cfg, self._get_group_y_lower(gun_group, machine_cfg), y_speed)
        x_target = self._get_outside_wait_x_target(ctx)
        axis_cmds.update(
            self._build_gun_axes(
                machine_cfg,
                plc_data,
                enabled_by_id,
                x_target,
                x_status,
                hold_current=False,
                x_speed=int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 100)) or 100),
                r_target=r_angle,
                apply_r_to_all=True,
                status_enabled_by_id=status_enabled_by_id,
            )
        )
        return axis_cmds

    def _prepare_next_stage(self, ctx, gun_group, r_angle):
        machine_cfg = ctx["machine_cfg"]
        plc_data = ctx["plc_data"]
        runtime_cfg = ctx["runtime_cfg"]
        # 收枪期间每根 Y/R 轴始终保持各自当前位置，只允许 X 收枪和 Z 跟踪或回安全位置。
        axis_cmds = self.motiontotarget.hold_current_position(machine_cfg, plc_data)
        x_target = self._get_outside_wait_x_target(ctx)
        enabled_by_id = self._get_enabled_guns_by_id(gun_group)

        x_ready = True
        for gun_idx in enabled_by_id:
            axis_name = f"x{gun_idx + 1}"
            if abs(self._get_axis_pos(machine_cfg, plc_data, axis_name) - x_target) > self.spray_pos_tolerance:
                x_ready = False

        if x_ready:
            z_target = self._get_safe_z_target(machine_cfg)
            z_speed = self._get_back_z_speed(ctx)
        else:
            z_target = self._get_work_z_target(machine_cfg)
            z_speed = self._get_work_z_speed(ctx)

        axis_cmds["z"] = self._build_z_axis(machine_cfg, z_target, z_speed)
        gun_axis_cmds = self._build_gun_axes(
            machine_cfg,
            plc_data,
            enabled_by_id,
            x_target,
            0,
            hold_current=False,
            x_speed=int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 100)) or 100),
        )
        axis_cmds.update({axis_name: axis_data for axis_name, axis_data in gun_axis_cmds.items() if axis_name.startswith("x")})

        return axis_cmds, x_ready

    def _build_gun_axes(self, machine_cfg, plc_data, enabled_by_id, x_target, x_status, hold_current, x_speed=None,
                        r_target=None, apply_r_to_all=False, status_enabled_by_id=None):
        axis_cmds = {}
        spray_num = int(machine_cfg.get("spray_num", 0) or 0)
        if x_speed is None:
            x_speed = int(machine_cfg.get("x_pos_speed", 100) or 100)
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        r_speed_limit = get_axis_speed_limit(machine_cfg, "r")
        x_safe = self._get_x_safe(machine_cfg)
        r_safe = self._get_r_safe(machine_cfg)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        r_min_limit, r_max_limit = get_axis_position_limits(machine_cfg, "r")
        x_target = clamp_to_limit_yx(int(x_target or 0), x_min_limit, x_max_limit)
        resolved_r_target = r_safe if r_target is None else clamp_to_limit_r(int(r_target or 0), r_min_limit, r_max_limit)

        for gun_idx in range(spray_num):
            axis_idx = gun_idx + 1
            x_name = f"x{axis_idx}"
            r_name = f"r{axis_idx}"
            x_cur = self._get_axis_pos(machine_cfg, plc_data, x_name)
            if gun_idx not in enabled_by_id:
                axis_cmds[x_name] = build_axis(x_safe, x_speed, 0, x_speed_limit)
                disabled_r_target = resolved_r_target if apply_r_to_all else r_safe
                axis_cmds[r_name] = build_axis(disabled_r_target, r_speed_limit, 0, r_speed_limit)
                continue
            final_target = x_cur if hold_current else x_target
            axis_status = x_status if status_enabled_by_id is None or gun_idx in status_enabled_by_id else 0
            axis_cmds[x_name] = build_axis(final_target, x_speed, axis_status, x_speed_limit)
            axis_cmds[r_name] = build_axis(resolved_r_target, r_speed_limit, 0, r_speed_limit)
        return axis_cmds

    def _ensure_current_enterable_inside(self, ctx):
        current_idx = self._get_current_inside_idx(ctx)
        if self._can_enter_inside_column(ctx, current_idx):
            return True
        return self._switch_to_next_enterable_inside(ctx, current_idx + 1)

    def _switch_to_next_enterable_inside(self, ctx, start_idx):
        next_idx = self._find_next_enterable_inside_idx(ctx, start_idx)
        if next_idx is None:
            return False
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        self._work_states[sn]["inside_idx"] = next_idx
        return True

    def _find_next_enterable_inside_idx(self, ctx, start_idx):
        inside_columns = ctx.get("inside_data") or []
        for inside_idx in range(max(0, int(start_idx or 0)), len(inside_columns)):
            if self._can_enter_inside_column(ctx, inside_idx):
                return inside_idx
        return None

    def _can_enter_inside_column(self, ctx, inside_idx):
        inside_columns = ctx.get("inside_data") or []
        if not (0 <= inside_idx < len(inside_columns)):
            return False

        _, _, _, _, z_min, z_max = self._get_inside_xyz_range_by_idx(ctx, inside_idx)
        inside_z_span = max(0, int(z_max or 0) - int(z_min or 0))
        return self._get_inside_enter_threshold(ctx) < inside_z_span

    def _get_inside_enter_threshold(self, ctx):
        # 进枪需要同时容纳前后偏移、喷枪半径和定位误差，判定与诊断必须共用同一阈值。
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        front_offset = int(runtime_cfg.get("in_z_front_offset", machine_cfg.get("in_z_front_offset", 0)) or 0)
        after_offset = int(runtime_cfg.get("in_z_after_offset", machine_cfg.get("in_z_after_offset", 0)) or 0)
        spray_radius = int(machine_cfg.get("spray_radius", 0) or 0)
        return front_offset + after_offset + 2 * spray_radius + 2 * self.spray_pos_tolerance

    def _log_no_enterable_inside(self, ctx, stage_name):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        inside_columns = ctx.get("inside_data") or []
        threshold = self._get_inside_enter_threshold(ctx)
        column_summaries = []

        for inside_idx, inside_column in enumerate(inside_columns):
            _, _, _, _, z_min, z_max = self._get_inside_xyz_range_by_idx(ctx, inside_idx)
            inside_z_span = max(0, int(z_max or 0) - int(z_min or 0))
            group_id = getattr(inside_column, "inside_id", None)
            if group_id is None:
                group_id = inside_idx
            gun_group = self._get_group(ctx, "inside", group_id)
            enabled_gun_ids = sorted(self._get_enabled_guns_by_id(gun_group).keys())
            column_summaries.append(
                {
                    "inside_idx": inside_idx,
                    "inside_id": getattr(inside_column, "inside_id", None),
                    "z_min": z_min,
                    "z_max": z_max,
                    "z_span": inside_z_span,
                    "enterable": threshold < inside_z_span,
                    "group_found": gun_group is not None,
                    "enabled_gun_ids": enabled_gun_ids,
                }
            )

        logger.warning(
            f"sn:[{sn}] side no enterable inside at {stage_name}, fallback to wait_after_outside: "
            f"inside_count={len(inside_columns)}, current_inside_idx={self._get_current_inside_idx(ctx)}, "
            f"enter_threshold={threshold}, columns={column_summaries}"
        )

    def _get_outside_group(self, ctx):
        return self._get_group(ctx, "outside")

    def _get_current_inside_group(self, ctx):
        inside_idx = self._get_current_inside_idx(ctx)
        inside_column = self._get_current_inside_column(ctx)
        if inside_column is None:
            return None
        group_id = getattr(inside_column, "inside_id", None)
        if group_id is None:
            group_id = inside_idx
        return self._get_group(ctx, "inside", group_id)

    def _get_group(self, ctx, group_type, group_id=None):
        sn_value = ctx["machine_cfg"].get("sn", None)
        sn = -1 if sn_value is None else int(sn_value)
        block = ctx["block_data"]
        for machine_data in getattr(block, "distribe_gun_list", None) or []:
            machine_id = getattr(machine_data, "machine_id", None)
            if machine_id is None or int(machine_id) != sn:
                continue
            for gun_group in getattr(machine_data, "gun_groups", None) or []:
                if getattr(gun_group, "group_type", None) != group_type:
                    continue
                gun_group_id = getattr(gun_group, "group_id", None)
                if group_id is None or (gun_group_id is not None and int(gun_group_id) == int(group_id)):
                    return gun_group
        return None

    def _get_enabled_guns_by_id(self, gun_group):
        enabled = {}
        if gun_group is None:
            return enabled
        for gun in getattr(gun_group, "gundata_list", None) or []:
            if int(getattr(gun, "gun_y_enable", 0) or 0) == 1:
                gun_id = getattr(gun, "gun_id", None)
                if gun_id is None:
                    continue
                enabled[int(gun_id)] = gun
        return enabled

    def _get_group_y_lower(self, gun_group, machine_cfg):
        enabled = list(self._get_enabled_guns_by_id(gun_group).values())
        if not enabled:
            return get_axis_safe_pos(machine_cfg, "y", default=0)
        y_values = [int(getattr(gun, "gun_y_downer", 0) or 0) for gun in enabled]
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        return clamp_to_limit_yx(max(y_values), y_min_limit, y_max_limit)

    def _get_group_y_full_range(self, gun_group, machine_cfg, runtime_cfg=None):
        enabled = list(self._get_enabled_guns_by_id(gun_group).values())
        if not enabled:
            return None
        y_downer = [int(getattr(gun, "gun_y_downer", 0) or 0) for gun in enabled]
        y_upper = [int(getattr(gun, "gun_y_upper", 0) or 0) for gun in enabled]
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_lower = clamp_to_limit_yx(min(y_downer), y_min_limit, y_max_limit)
        raw_y_upper = clamp_to_limit_yx(max(y_upper), y_min_limit, y_max_limit)
        reduce_distance = 0 if runtime_cfg is None else max(
            0,
            int(runtime_cfg.get("recip_reduce_distance", machine_cfg.get("recip_reduce_distance", 0)) or 0),
        )
        reduced_y_upper = int(raw_y_upper) - reduce_distance
        if reduced_y_upper <= y_lower:
            y_upper = y_lower
        else:
            y_upper = clamp_to_limit_yx(reduced_y_upper, y_min_limit, y_max_limit)
        if y_lower > y_upper:
            y_lower, y_upper = y_upper, y_lower
        return y_lower, y_upper

    def _get_inside_end_face_y_range(self, ctx, gun_group):
        enabled_guns = list(self._get_enabled_guns_by_id(gun_group).values())
        y_range = self.recip_manager._get_group_y_range(ctx["machine_cfg"], enabled_guns, ctx["runtime_cfg"])
        if y_range is None:
            y_safe = get_axis_safe_pos(ctx["machine_cfg"], "y", default=0)
            return y_safe, y_safe
        return y_range

    def _current_inside_has_enabled_guns(self, ctx):
        return bool(self._get_enabled_guns_by_id(self._get_current_inside_group(ctx)))

    def _get_current_inside_idx(self, ctx):
        sn = int(ctx["machine_cfg"].get("sn", 0) or 0)
        return int(self._work_states[sn].get("inside_idx", 0) or 0)

    def _get_current_inside_column(self, ctx):
        return self._get_inside_column_by_idx(ctx, self._get_current_inside_idx(ctx))

    def _get_inside_column_by_idx(self, ctx, inside_idx):
        inside_columns = ctx.get("inside_data") or []
        if 0 <= inside_idx < len(inside_columns):
            return inside_columns[inside_idx]
        return None

    def _get_current_inside_xyz_range(self, ctx):
        return self._get_inside_xyz_range_by_idx(ctx, self._get_current_inside_idx(ctx))

    def _get_inside_xyz_range_by_idx(self, ctx, inside_idx):
        inside_column = self._get_inside_column_by_idx(ctx, inside_idx)
        if inside_column is None:
            return 0, 0, 0, 0, 0, 0

        subinside_list = getattr(inside_column, "subinside_datalist", None) or []
        x_mins = [int(getattr(sub, "subinside_x_min", 0) or 0) for sub in subinside_list if getattr(sub, "subinside_x_min", None) is not None]
        x_maxs = [int(getattr(sub, "subinside_x_max", 0) or 0) for sub in subinside_list if getattr(sub, "subinside_x_max", None) is not None]
        y_mins = [int(getattr(sub, "subinside_y_min", 0) or 0) for sub in subinside_list if getattr(sub, "subinside_y_min", None) is not None]
        y_maxs = [int(getattr(sub, "subinside_y_max", 0) or 0) for sub in subinside_list if getattr(sub, "subinside_y_max", None) is not None]
        z_mins = [int(getattr(sub, "subinside_z_min", 0) or 0) for sub in subinside_list if getattr(sub, "subinside_z_min", None) is not None]
        z_maxs = [int(getattr(sub, "subinside_z_max", 0) or 0) for sub in subinside_list if getattr(sub, "subinside_z_max", None) is not None]
        return (
            min(x_mins) if x_mins else 0,
            min(x_maxs) if x_maxs else 0,
            min(y_mins) if y_mins else 0,
            max(y_maxs) if y_maxs else 0,
            max(z_mins) if z_mins else 0,
            min(z_maxs) if z_maxs else 0,
        )

    def _get_outside_x_range(self, ctx):
        outside = (ctx.get("outside_data") or [None])[0]
        if outside is None:
            return 0, 0
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        x_position = self._get_x_position(machine_cfg)
        out_front_x_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 100)) or 100)
        out_after_x_offset = int(runtime_cfg.get("out_after_x_offset", machine_cfg.get("out_after_x_offset", 100)) or 100)
        x_min = int(getattr(outside, "outside_x_min", 0) or 0) - out_front_x_offset - x_position
        x_max = int(getattr(outside, "outside_x_max", 0) or 0) - out_after_x_offset - x_position
        return x_min, x_max

    def _get_outside_wait_x_target(self, ctx):
        x_min, _ = self._get_outside_x_range(ctx)
        x_pre_distance = int(getattr(self.motiontotarget, "x_pre_distance", 0) or 0)
        return x_min - x_pre_distance

    def _get_inside_spray_x_range(self, ctx):
        x_min, x_max, _, _, _, _ = self._get_current_inside_xyz_range(ctx)
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        x_position = self._get_x_position(machine_cfg)
        in_front_x_offset = int(runtime_cfg.get("in_front_x_offset", machine_cfg.get("in_front_x_offset", 100)) or 100)
        in_after_x_offset = int(runtime_cfg.get("in_after_x_offset", machine_cfg.get("in_after_x_offset", 100)) or 100)
        return x_min - in_front_x_offset - x_position, x_max - in_after_x_offset - x_position

    def _get_x_position(self, machine_cfg):
        return int(machine_cfg.get("x_position", 0) or 0)

    def _get_all_guns_by_id(self, machine_cfg):
        spray_num = int(machine_cfg.get("spray_num", 0) or 0)
        return {gun_idx: True for gun_idx in range(spray_num)}

    def _has_front_outside_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_front_z_pair(ctx, "out_z_front_offset", "outside_z_min")
        return self.motion_util.has_arrived_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_front_outside_x_status_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_front_z_pair(ctx, "out_z_front_offset", "outside_z_min")
        machine_z -= self._get_x_status_offset(ctx)
        return self._has_z_arrived_or_over(ctx, machine_z, chain_z)

    def _has_front_outside_over_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_front_z_pair(ctx, "out_z_front_offset", "outside_z_min")
        return self.motion_util.has_over_arrive_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_after_outside_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_after_z_pair(ctx, "out_z_after_offset", "outside_z_max")
        return self.motion_util.has_arrived_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_after_outside_x_status_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_after_z_pair(ctx, "out_z_after_offset", "outside_z_max")
        machine_z -= self._get_x_status_offset(ctx)
        return self._has_z_arrived_or_over(ctx, machine_z, chain_z)

    def _has_after_outside_over_arrived(self, ctx):
        machine_z, chain_z = WorkpieceMotionHelper.get_outside_after_z_pair(ctx, "out_z_after_offset", "outside_z_max")
        return self.motion_util.has_over_arrive_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_front_inside_arrived(self, ctx):
        _, _, _, _, z_min, _ = self._get_current_inside_xyz_range(ctx)
        machine_z, chain_z = WorkpieceMotionHelper.get_inside_front_z_pair(ctx, "in_z_front_offset", z_min)
        return self.motion_util.has_arrived_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_front_inside_over_arrived(self, ctx):
        _, _, _, _, z_min, _ = self._get_current_inside_xyz_range(ctx)
        machine_z, chain_z = WorkpieceMotionHelper.get_inside_front_z_pair(ctx, "in_z_front_offset", z_min)
        return self.motion_util.has_over_arrive_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_after_inside_arrived(self, ctx):
        _, _, _, _, _, z_max = self._get_current_inside_xyz_range(ctx)
        machine_z, chain_z = WorkpieceMotionHelper.get_inside_after_z_pair(ctx, "in_z_after_offset", z_max)
        return self.motion_util.has_arrived_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _has_after_inside_over_arrived(self, ctx):
        _, _, _, _, _, z_max = self._get_current_inside_xyz_range(ctx)
        machine_z, chain_z = WorkpieceMotionHelper.get_inside_after_z_pair(ctx, "in_z_after_offset", z_max)
        return self.motion_util.has_over_arrive_z(self._get_compare_machine_z(ctx, machine_z), chain_z)

    def _get_compare_machine_z(self, ctx, machine_z):
        machine_cfg = ctx["machine_cfg"]
        plc_data = ctx["plc_data"]
        current_z_pos = self._get_axis_pos(machine_cfg, plc_data, "z")
        return int(machine_z or 0) + int(current_z_pos or 0)

    def _has_z_arrived_or_over(self, ctx, machine_z, chain_z):
        compare_machine_z = self._get_compare_machine_z(ctx, machine_z)
        return self.motion_util.has_arrived_z(compare_machine_z, chain_z) or self.motion_util.has_over_arrive_z(compare_machine_z, chain_z)

    def _get_x_status_offset(self, ctx):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        return int(runtime_cfg.get("x_status_offset", machine_cfg.get("x_status_offset", 0)) or 0)

    def _get_work_z_target(self, machine_cfg):
        _, z_max_limit = get_axis_position_limits(machine_cfg, "z")
        return z_max_limit

    def _get_safe_z_target(self, machine_cfg):
        return get_axis_safe_pos(machine_cfg, "z", default=0)

    def _get_work_z_speed(self, ctx):
        return self._resolve_follow_z_speed(ctx["plc_data"])

    def _resolve_follow_z_speed(self, plc_data):
        chain_speed = int(getattr(plc_data, "ChainSpeed", 0) or 0)
        if not self._is_chain_running(plc_data):
            return 0
        return chain_speed if chain_speed != 0 else 0

    @staticmethod
    def _is_chain_running(plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") == "moving_forward"

    def _get_back_z_speed(self, ctx):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        return int(runtime_cfg.get("z_back_speed", machine_cfg.get("z_back_speed", 100)) or 100)

    def _get_safe_z_speed(self, ctx):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        return int(runtime_cfg.get("z_zeroing_speed", machine_cfg.get("z_zeroing_speed", 100)) or 100)

    def _build_z_axis(self, machine_cfg, target, speed):
        z_min_limit, z_max_limit = get_axis_position_limits(machine_cfg, "z")
        z_speed_limit = get_axis_speed_limit(machine_cfg, "z")
        z_target = clamp_to_limit_z(int(target or 0), z_min_limit, z_max_limit)
        return build_axis(z_target, speed, 0, z_speed_limit)

    def _build_y_axis(self, machine_cfg, target, speed):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        y_target = clamp_to_limit_yx(int(target or 0), y_min_limit, y_max_limit)
        return build_axis(y_target, speed, 0, y_speed_limit)

    def _get_x_safe(self, machine_cfg):
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        return clamp_to_limit_yx(get_axis_safe_pos(machine_cfg, "x", default=0), x_min_limit, x_max_limit)

    def _get_r_safe(self, machine_cfg):
        r_min_limit, r_max_limit = get_axis_position_limits(machine_cfg, "r")
        return clamp_to_limit_r(get_axis_safe_pos(machine_cfg, "r", default=0), r_min_limit, r_max_limit)

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "right"))
        if axis_name == "y" and axis_name not in axis_map:
            y_axes = [name for name in axis_map if name.startswith("y")]
            if y_axes:
                axis_name = y_axes[0]
        return self.motiontotarget._get_axis_current_pos(plc_data, axis_map[axis_name])

    @staticmethod
    def _state_prefix(sn, name):
        return f"sn{sn}:{name}"

    @staticmethod
    def _normalize_enabled_flag(value):
        if isinstance(value, bool):
            return value
        if value is None:
            return True
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(int(value))

    def _resolve_side_r_angle(self, r_angle):
        if not self.side_rotate_enabled:
            return 0
        return int(r_angle or 0)
