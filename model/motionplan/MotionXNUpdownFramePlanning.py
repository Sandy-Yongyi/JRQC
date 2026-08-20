import os
from dataclasses import dataclass, field

from model.motionplan.MachineAxisMap import get_axis_map, get_axis_position_limits, get_axis_speed_limit
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.motionutil.XNUpdownFrameSearchHelper import XNUpdownFrameSearchHelper
from model.utils.TomlLoader import TomlLoader


@dataclass
class XNUpdownGunTarget:
    gun_id: int
    y_target: int | None
    x_min_target: int
    x_max_target: int
    spray_allowed: bool
    collision_adjusted: bool = False


@dataclass
class XNUpdownDeviceState:
    targets: dict[int, XNUpdownGunTarget] = field(default_factory=dict)
    x_phases: dict[int, str] = field(default_factory=dict)

    @property
    def latched(self):
        return bool(self.targets)


class MotionXNUpdownFramePlanning:
    """顶底设备按帧目标锁存、独立定位和持续X往复规划。"""

    def __init__(self, read_data_cfg=None, spray_cfg=None, motion_to_target=None):
        config_dir = os.path.join(os.getcwd(), "model", "tomls")
        self.read_data_cfg = read_data_cfg if read_data_cfg is not None else TomlLoader.load(os.path.join(config_dir, "ReadDataConfig.toml"))
        self.spray_cfg = spray_cfg if spray_cfg is not None else TomlLoader.load(os.path.join(config_dir, "SprayConfig.toml"))
        self.motion_to_target = motion_to_target if motion_to_target is not None else MotionToTarget()
        z_threshold_value = self.read_data_cfg.get("z_threshold", 10)
        self.z_threshold = 10 if z_threshold_value is None else int(z_threshold_value)
        if self.z_threshold <= 0:
            raise ValueError(f"z_threshold 必须大于 0，当前值: {self.z_threshold}")
        self.tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10) or 10)
        self.search_helper = XNUpdownFrameSearchHelper(z_threshold=self.z_threshold)
        self._states: dict[int, XNUpdownDeviceState] = {}

    def reset_motion_state(self, sn=None):
        if sn is None:
            self._states.clear()
            return
        self._states.pop(int(sn), None)

    def auto_xn_updown_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        sn = int(machine_cfg.get("sn", 0) or 0)
        state = self._states.setdefault(sn, XNUpdownDeviceState())
        frames = self.search_helper.get_frames(machine_cfg, frame_queue_manager)
        search_window = self.search_helper.get_search_window(machine_cfg, runtime_cfg, len(frames))
        in_up_y_offset = self._get_config_int(machine_cfg, runtime_cfg, "in_up_y_offset", 100)
        in_down_y_offset = self._get_config_int(machine_cfg, runtime_cfg, "in_down_y_offset", 100)
        structure = self.search_helper.identify_structure(frames, search_window, in_up_y_offset, in_down_y_offset)

        if not structure.has_data:
            self.reset_motion_state(sn)
            axis_cmds, _ = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
            return axis_cmds or {}

        if not state.latched:
            targets = self._build_latched_targets(machine_cfg, runtime_cfg, frames, structure)
            if targets is None:
                return self._build_incomplete_commands(machine_cfg, runtime_cfg, plc_data, structure.overall_x_min)
            state.targets = targets
            state.x_phases = {gun_id: "position_min" for gun_id in targets}

        return self._build_latched_commands(machine_cfg, runtime_cfg, plc_data, state)

    def _build_latched_targets(self, machine_cfg, runtime_cfg, frames, structure):
        if not structure.complete:
            return None
        spray_window = self.search_helper.get_spray_window(machine_cfg, runtime_cfg, len(frames))
        lower = self.search_helper.collect_region(frames, spray_window, y_max=structure.down_boundary)
        upper = self.search_helper.collect_region(frames, spray_window, y_min=structure.up_boundary)
        origin_pos = [int(value or 0) for value in machine_cfg.get("origin_pos", [])]
        if not lower.complete or not upper.complete or len(origin_pos) < 4:
            return None

        out_down_y_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_down_y_offset", 100)
        in_down_y_offset = self._get_config_int(machine_cfg, runtime_cfg, "in_down_y_offset", 100)
        in_up_y_offset = self._get_config_int(machine_cfg, runtime_cfg, "in_up_y_offset", 100)
        out_up_y_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_up_y_offset", 100)
        y_targets = [int(lower.y_min) - out_down_y_offset - origin_pos[0], int(lower.y_max) + in_down_y_offset - origin_pos[1],  # type: ignore
                     int(upper.y_min) - in_up_y_offset - origin_pos[2]]  # type: ignore
        y_targets = [self._clamp_y_target(machine_cfg, f"y{index + 1}", target) for index, target in enumerate(y_targets)]
        lower_x_min, lower_x_max, lower_x_valid = self._build_x_range(machine_cfg, runtime_cfg, lower)
        upper_x_min, upper_x_max, upper_x_valid = self._build_x_range(machine_cfg, runtime_cfg, upper)
        y2_allowed = int(lower.y_max) <= int(structure.down_y_max) - in_down_y_offset * 2  # type: ignore
        y3_allowed = int(upper.y_min) >= int(structure.up_y_min) + in_up_y_offset * 2  # type: ignore
        y4_has_collision = self.search_helper.has_data_in_y_band(frames, spray_window, origin_pos[3] - out_up_y_offset, origin_pos[3] + out_up_y_offset)
        targets = {
            1: XNUpdownGunTarget(1, y_targets[0], lower_x_min, lower_x_max, lower_x_valid),
            2: XNUpdownGunTarget(2, y_targets[1], lower_x_min, lower_x_max, lower_x_valid and y2_allowed),
            3: XNUpdownGunTarget(3, y_targets[2], upper_x_min, upper_x_max, upper_x_valid and y3_allowed),
            4: XNUpdownGunTarget(4, None, upper_x_min, upper_x_max, upper_x_valid and not y4_has_collision),
        }
        self._apply_y_collision_rules(targets)
        return targets

    def _build_x_range(self, machine_cfg, runtime_cfg, region):
        x_position = int(machine_cfg.get("x_position", 0) or 0)
        front_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_front_x_offset", 100)
        after_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_after_x_offset", 100)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        x_min_target = clamp_to_limit_yx(int(region.x_min) - front_offset - x_position, x_min_limit, x_max_limit)
        x_max_target = clamp_to_limit_yx(int(region.x_max) - after_offset - x_position, x_min_limit, x_max_limit)
        return x_min_target, max(x_min_target, x_max_target), x_min_target < x_max_target

    @staticmethod
    def _apply_y_collision_rules(targets):
        for gun_id in (2, 3):
            lower_target = targets[gun_id - 1]
            upper_target = targets[gun_id]
            if int(upper_target.y_target) > int(lower_target.y_target):
                continue
            upper_target.y_target = lower_target.y_target
            upper_target.spray_allowed = False
            upper_target.collision_adjusted = True

    def _build_incomplete_commands(self, machine_cfg, runtime_cfg, plc_data, overall_x_min):
        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        if overall_x_min is None:
            return axis_cmds
        x_position = int(machine_cfg.get("x_position", 0) or 0)
        front_offset = self._get_config_int(machine_cfg, runtime_cfg, "out_front_x_offset", 100)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        x_target = clamp_to_limit_yx(int(overall_x_min) - front_offset - x_position, x_min_limit, x_max_limit)
        x_speed = self._get_speed(machine_cfg, runtime_cfg, "x_pos_speed", 300)
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        for gun_id in range(1, 5):
            axis_cmds[f"x{gun_id}"] = build_axis(x_target, x_speed, 0, x_speed_limit)
        return axis_cmds

    def _build_latched_commands(self, machine_cfg, runtime_cfg, plc_data, state):
        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        y_speed = self._get_speed(machine_cfg, runtime_cfg, "y_pos_speed", 100)
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        y_ready = {4: True}
        for gun_id in range(1, 4):
            target = state.targets[gun_id]
            current = self._get_axis_pos(machine_cfg, plc_data, f"y{gun_id}")
            y_ready[gun_id] = self._has_arrived(current, target.y_target)
            axis_cmds[f"y{gun_id}"] = build_axis(target.y_target, 0 if y_ready[gun_id] else y_speed, 0, y_speed_limit)
        for gun_id in range(1, 5):
            axis_cmds[f"x{gun_id}"] = self._build_x_command(machine_cfg, runtime_cfg, plc_data, state, gun_id, y_ready[gun_id])
        return axis_cmds

    def _build_x_command(self, machine_cfg, runtime_cfg, plc_data, state, gun_id, y_ready):
        target = state.targets[gun_id]
        axis_name = f"x{gun_id}"
        current = self._get_axis_pos(machine_cfg, plc_data, axis_name)
        x_pos_speed = self._get_speed(machine_cfg, runtime_cfg, "x_pos_speed", 300)
        x_recip_speed = self._get_speed(machine_cfg, runtime_cfg, "x_recip_speed", 100)
        x_speed_limit = get_axis_speed_limit(machine_cfg, axis_name)
        phase = state.x_phases.get(gun_id, "position_min")

        if not target.spray_allowed or not y_ready:
            state.x_phases[gun_id] = "position_min"
            speed = 0 if self._has_arrived(current, target.x_min_target) else x_pos_speed
            return build_axis(target.x_min_target, speed, 0, x_speed_limit)

        if phase == "position_min":
            if not self._has_arrived(current, target.x_min_target):
                return build_axis(target.x_min_target, x_pos_speed, 0, x_speed_limit)
            phase = "to_max"
        elif phase == "to_max" and self._has_arrived(current, target.x_max_target):
            phase = "to_min"
        elif phase == "to_min" and self._has_arrived(current, target.x_min_target):
            phase = "to_max"

        state.x_phases[gun_id] = phase
        x_target = target.x_max_target if phase == "to_max" else target.x_min_target
        return build_axis(x_target, x_recip_speed, 1, x_speed_limit)

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "left"))
        axis_item = plc_data.AxisList[axis_map[axis_name]]
        if hasattr(axis_item, "Pos"):
            return int(getattr(axis_item, "Pos", 0) or 0)
        if isinstance(axis_item, (list, tuple)) and axis_item:
            return int(axis_item[0] or 0)
        if isinstance(axis_item, dict):
            return int(axis_item.get("Pos", 0) or 0)
        return 0

    def _clamp_y_target(self, machine_cfg, axis_name, target):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, axis_name)
        return clamp_to_limit_yx(int(target), y_min_limit, y_max_limit)

    def _has_arrived(self, current, target):
        return abs(int(current) - int(target)) <= self.tolerance

    @staticmethod
    def _get_config_int(machine_cfg, runtime_cfg, key, default):
        value = runtime_cfg.get(key) if isinstance(runtime_cfg, dict) and key in runtime_cfg else machine_cfg.get(key, default)
        return int(default) if value is None else int(value)

    @staticmethod
    def _get_speed(machine_cfg, runtime_cfg, key, default):
        value = runtime_cfg.get(key) if isinstance(runtime_cfg, dict) and key in runtime_cfg else machine_cfg.get(key, default)
        return int(default) if value is None else int(value)
