from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_speed_limit, get_axis_safe_pos
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_r, clamp_to_limit_yx, clamp_to_limit_z
from model.utils.LoggerUtil import logger


class MotionCleaningPlanning:
    """清理模式运动规划。"""

    CLEAN_MODE_BIT = 1 << 6

    def is_clean_mode_enabled(self, operate: int) -> bool:
        return (int(operate or 0) & self.CLEAN_MODE_BIT) != 0

    def has_any_workpiece(self, frame_queue_manager) -> bool:
        queues = getattr(frame_queue_manager, "queues", {}) or {}
        is_empty_data = getattr(frame_queue_manager, "_is_empty_data", self._is_empty_data)
        for direction_queues in queues.values():
            for queue_data in (direction_queues or {}).values():
                if self._queue_has_workpiece(queue_data, is_empty_data):
                    return True
        return False

    def build_device_axis_cmds(self, machine_cfg, runtime_cfg, clean_ready: bool):
        axis_cmds = {}
        axis_type_list = machine_cfg.get("axis_type", []) or []
        x_pos_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 100)) or 100)
        y_pos_speed = int(runtime_cfg.get("y_pos_speed", machine_cfg.get("y_pos_speed", 100)) or 100)
        z_pos_speed = int(runtime_cfg.get("z_zeroing_speed", machine_cfg.get("z_zeroing_speed", 100)) or 100)

        for axis_name in axis_type_list:
            min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
            speed_limit = get_axis_speed_limit(machine_cfg, axis_name)

            if axis_name.startswith("x"):
                target = max_limit if clean_ready else get_axis_safe_pos(machine_cfg, axis_name, default=0)
                target = clamp_to_limit_yx(int(target or 0), min_limit, max_limit)
                axis_cmds[axis_name] = build_axis(target, x_pos_speed, 0, speed_limit)
                continue

            if axis_name == "y":
                target = clamp_to_limit_yx(get_axis_safe_pos(machine_cfg, axis_name, default=0), min_limit, max_limit)
                axis_cmds[axis_name] = build_axis(target, y_pos_speed, 0, speed_limit)
                continue

            if axis_name == "z":
                target = clamp_to_limit_z(get_axis_safe_pos(machine_cfg, axis_name, default=0), min_limit, max_limit)
                axis_cmds[axis_name] = build_axis(target, z_pos_speed, 0, speed_limit)
                continue

            if axis_name.startswith("r"):
                target = clamp_to_limit_r(get_axis_safe_pos(machine_cfg, axis_name, default=0), min_limit, max_limit)
                axis_cmds[axis_name] = build_axis(target, x_pos_speed, 0, speed_limit)
                continue

            target = clamp_to_limit_yx(get_axis_safe_pos(machine_cfg, axis_name, default=0), min_limit, max_limit)
            axis_cmds[axis_name] = build_axis(target, x_pos_speed, 0, speed_limit)

        return axis_cmds

    def log_clean_mode_blocked(self):
        print("当前内部有工件请关闭清理模式")
        logger.warning("当前内部有工件请关闭清理模式")

    @staticmethod
    def _queue_has_workpiece(queue_data, is_empty_data=None) -> bool:
        if queue_data is None:
            return False
        if isinstance(queue_data, list):
            empty_check = is_empty_data or MotionCleaningPlanning._is_empty_data
            return any(not empty_check(item) for item in queue_data)
        if hasattr(queue_data, "empty"):
            try:
                return not queue_data.empty()
            except Exception:
                return True
        try:
            return len(queue_data) > 0
        except Exception:
            return bool(queue_data)

    @staticmethod
    def _is_empty_data(data) -> bool:
        if data is None:
            return True
        if isinstance(data, dict):
            block = data.get("data")
            if block is None:
                return True
            if hasattr(block, "is_empty") and block.is_empty:
                return True
            return False
        if hasattr(data, "is_empty"):
            return bool(getattr(data, "is_empty"))
        return False
