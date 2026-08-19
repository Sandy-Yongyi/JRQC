import os
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_speed_limit, get_axis_map
from model.motionplan.MotionToTarget import MotionToTarget


class MotionManualOutFxPlanning:
    """外侧仿形手动模式运动规划。"""

    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.path.join(os.getcwd(), "model", "tomls", "SprayConfig.toml"))
        self.motion_to_target = MotionToTarget()
        self.spray_pos_tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10))
        self._work_states = {}

    def reset_motion_state(self, sn=None):
        if sn is None:
            self._work_states = {}
            return
        self._work_states.pop(int(sn), None)

    def auto_manual_out_fx_move(self, machine_cfg, runtime_cfg, spray_cfg, plc_data):
        sn = int(machine_cfg.get("sn", 0))
        state = self._work_states.setdefault(sn, {"y_phase": "to_max"})
        chain_running = self._is_chain_running(plc_data)
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")

        y_speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)))
        x_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 300)))
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        origin_pos = machine_cfg.get("origin_pos", [])
        origin_values = [int(v or 0) for v in origin_pos]
        gun_distance = abs(origin_values[1] - origin_values[0]) if len(origin_values) >= 2 else 0

        y_min_target = clamp_to_limit_yx(int(spray_cfg.get("size_y_min", 0) or 0), y_min_limit, y_max_limit)
        y_max_target = clamp_to_limit_yx(int(spray_cfg.get("size_y_max", 0) or 0), y_min_limit, y_max_limit)
        if y_max_target < gun_distance + y_min_target:
            y_max_target = clamp_to_limit_yx(gun_distance + y_min_target, y_min_limit, y_max_limit)

        x_target = clamp_to_limit_yx(int(spray_cfg.get("size_x_min", 0) or 0), x_min_limit, x_max_limit)

        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")
        phase = state.get("y_phase", "to_max")
        y_target = y_max_target if phase == "to_max" else y_min_target

        if y_cur > y_max_target:
            state["y_phase"] = "to_min"
            y_target = y_min_target

        if abs(y_cur - y_target) <= self.spray_pos_tolerance:
            phase = "to_min" if phase == "to_max" else "to_max"
            state["y_phase"] = phase
            y_target = y_max_target if phase == "to_max" else y_min_target

        axis_cmds = {
            "y": build_axis(y_target, y_speed, 0, y_speed_limit),
        }
        for axis_name in machine_cfg.get("axis_type", []):
            if axis_name.startswith("x"):
                axis_cmds[axis_name] = build_axis(x_target, x_speed, 1 if chain_running else 0, x_speed_limit)

        logger.debug(
            f"SN[{sn}] manual out_fx active, chain_running={chain_running}, x_target={x_target}, "
            f"y_range=({y_min_target}, {y_max_target}), y_phase={state.get('y_phase', 'to_max')}"
        )
        return axis_cmds

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "left"))
        return self.motion_to_target._get_axis_current_pos(plc_data, axis_map[axis_name])

    def _is_chain_running(self, plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") == "moving_forward"
