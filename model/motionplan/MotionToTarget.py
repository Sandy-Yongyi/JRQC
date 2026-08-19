import os
from model.utils.TomlLoader import TomlLoader
from model.motionplan.motionutil.AxisLimits import clamp_to_limit_yx, clamp_to_limit_z, clamp_to_limit_r, build_axis
from model.motionplan.MachineAxisMap import get_axis_map, MACHINE_AXIS_MAP, get_axis_position_limits, get_axis_speed_limit, get_axis_safe_pos


class MotionToTarget:
    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\sprayconfig.toml")
        self.tolerance = self.spray_cfg.get("spray_pos_tolerance", 10)
        self.x_pre_distance = self.spray_cfg.get("x_pre_distance", 30)

    def hold_current_position(self, machine_cfg, plc_data):
        """
        安全维持当前位置：
        - 遍历设备的所有轴，读取当前实际位置作为目标位置
        - 所有轴速度设为 0，设备不会发生任何运动
        - 适用于需要设备"冻结"在当前状态的场景

        Returns:
            dict[str, AxisData]: 轴名称 → 维持当前位置的运动指令
        """
        machine_type = machine_cfg.get("type", "")
        if machine_type not in MACHINE_AXIS_MAP:
            return {}

        orientation = machine_cfg.get("install_orietation", "left")
        axis_type_list = machine_cfg.get("axis_type", [])
        axis_map = get_axis_map(machine_type, orientation)

        axis_cmds = {}
        for axis_name in axis_type_list:
            if axis_name not in axis_map:
                continue
            idx = axis_map[axis_name]
            cur_pos = self._get_axis_current_pos(plc_data, idx)
            min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
            cur_pos = self._clamp_axis_target(axis_name, cur_pos, min_limit, max_limit)
            # 目标位置 = 当前位置，速度 = 0，设备保持不动
            axis_cmds[axis_name] = build_axis(cur_pos, 0, 0)

        return axis_cmds

    def _get_axis_current_pos(self, plc_data, idx):
        axis_list = getattr(plc_data, "AxisList", None)
        if axis_list is None or idx >= len(axis_list):
            return 0

        axis_item = axis_list[idx]
        if hasattr(axis_item, "Pos"):
            return int(getattr(axis_item, "Pos", 0) or 0)
        if isinstance(axis_item, (list, tuple)) and len(axis_item) > 0:
            return int(axis_item[0] or 0)
        if isinstance(axis_item, dict):
            return int(axis_item.get("Pos", 0) or 0)
        return 0

    def move_x_axes_to_target(self, machine_cfg, plc_data, target, speed, status=0):
        """将设备全部 X 轴移动到统一限位目标，并返回是否全部到位。"""
        machine_type = machine_cfg.get("type", "")
        if machine_type not in MACHINE_AXIS_MAP:
            return {}, False

        orientation = machine_cfg.get("install_orietation", "left")
        axis_map = get_axis_map(machine_type, orientation)
        axis_cmds = {}
        all_ready = True

        for axis_name in machine_cfg.get("axis_type", []):
            if not axis_name.startswith("x") or axis_name not in axis_map:
                continue

            min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
            limited_target = clamp_to_limit_yx(int(target or 0), min_limit, max_limit)
            speed_limit = get_axis_speed_limit(machine_cfg, axis_name)
            axis_cmds[axis_name] = build_axis(limited_target, int(speed or 0), int(status or 0), speed_limit)
            current = self._get_axis_current_pos(plc_data, axis_map[axis_name])
            if abs(current - limited_target) > int(self.tolerance or 0):
                all_ready = False

        return axis_cmds, all_ready

    def move_to_origin_safe(self, machine_cfg, runtime_cfg, plc_data):
        """
        安全回安全位：
        基于 axis_type 和 MACHINE_AXIS_MAP 泛型处理
          - 如果 X 型轴未到位 -> 优先让关键轴回到配置 safe_pos，其余轴按安全策略保持/等待
          - 如果 X 型轴已到位 -> 所有轴回到配置 safe_pos
        """
        machine_type = machine_cfg.get("type", "")
        if machine_type not in MACHINE_AXIS_MAP:
            return None, False
        axis_cmds = self._build_device_origin_move(machine_cfg, runtime_cfg, plc_data)
        all_ready = self._check_device_axes_arrived(machine_cfg, plc_data)
        return axis_cmds, all_ready

    def _build_device_origin_move(self, machine_cfg, runtime_cfg, plc_data):
        """
        通用设备回安全位运动指令构建

        默认逻辑 (非 out_rotate / in_rotate):
        - X 型轴 (x, x1~x5): 直接回各自 safe_pos
        - Z 轴:   X 未到位时先朝最大限制位置运动; X 已到位后回 safe_pos
        - Y 轴:   X 未到位时保持当前位置; X 已到位后回 safe_pos
        - R 轴:   X 未到位时保持当前位置; X 已到位后回 safe_pos

        out_rotate / in_rotate 逻辑:
        - Y 轴:   直接回 safe_pos
        - X 轴:   Y 未到位时保持当前位置; Y 已到位后回 safe_pos
        - Z 轴:   Y 未到位时保持当前位置; Y 已到位后回 safe_pos
        - R 轴:   Y 未到位时保持当前位置; Y 已到位后回 safe_pos

        Returns:
            dict[str, AxisData]: 轴名称 → 运动指令
        """
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")
        axis_type_list = machine_cfg.get("axis_type", [])
        x_pos_speed = runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 300))
        y_pos_speed = runtime_cfg.get("y_pos_speed", 100)
        z_zeroing_speed = runtime_cfg.get("z_zeroing_speed", 150)
        chain_speed = self._resolve_follow_z_speed(plc_data)

        axis_map = get_axis_map(machine_type, orientation)
        is_rotate = machine_type in ("out_rotate", "in_rotate")

        axis_cmds = {}

        if is_rotate:
            # out_rotate / in_rotate: 先确保 Y 轴回安全位，再回其余轴
            y_ready = self._check_y_axes_arrived_by_type(machine_cfg, plc_data)

            for axis_name in axis_type_list:
                if axis_name not in axis_map:
                    continue
                idx = axis_map[axis_name]
                cur_pos = self._get_axis_current_pos(plc_data, idx)
                min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
                safe_pos = self._get_axis_safe_pos(machine_cfg, axis_name)
                safe_target = self._clamp_axis_target(axis_name, safe_pos, min_limit, max_limit)
                axis_speed_limit = self._get_axis_speed_limit(machine_cfg, axis_name)

                if axis_name.startswith("y"):
                    # Y 轴始终直接回安全位
                    cmd = build_axis(safe_target, y_pos_speed, 0, axis_speed_limit)
                elif axis_name == "z":
                    if not y_ready:
                        cmd = build_axis(clamp_to_limit_z(cur_pos, min_limit, max_limit), chain_speed, 0, axis_speed_limit)
                    else:
                        cmd = build_axis(safe_target, z_zeroing_speed, 0, axis_speed_limit)
                elif axis_name.startswith("x"):
                    if not y_ready:
                        cmd = build_axis(clamp_to_limit_yx(cur_pos, min_limit, max_limit), x_pos_speed, 0, axis_speed_limit)
                    else:
                        cmd = build_axis(safe_target, x_pos_speed, 0, axis_speed_limit)
                elif axis_name.startswith("r"):
                    if not y_ready:
                        cmd = build_axis(clamp_to_limit_r(cur_pos, min_limit, max_limit), axis_speed_limit, 0, axis_speed_limit)
                    else:
                        cmd = build_axis(safe_target, axis_speed_limit, 0, axis_speed_limit)
                else:
                    cmd = build_axis(safe_target, x_pos_speed, 0, axis_speed_limit)

                axis_cmds[axis_name] = cmd
        else:
            # 默认逻辑: 先确保 X 轴回安全位，再回其余轴
            x_ready = self._check_x_axes_arrived_by_type(machine_cfg, plc_data)

            for axis_name in axis_type_list:
                if axis_name not in axis_map:
                    continue
                idx = axis_map[axis_name]
                cur_pos = self._get_axis_current_pos(plc_data, idx)
                min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
                safe_pos = self._get_axis_safe_pos(machine_cfg, axis_name)
                safe_target = self._clamp_axis_target(axis_name, safe_pos, min_limit, max_limit)
                axis_speed_limit = self._get_axis_speed_limit(machine_cfg, axis_name)

                if axis_name == "z":
                    if not x_ready:
                        cmd = build_axis(safe_target, chain_speed, 0, axis_speed_limit)
                    else:
                        cmd = build_axis(safe_target, z_zeroing_speed, 0, axis_speed_limit)
                elif axis_name.startswith("y"):
                    if not x_ready:
                        cmd = build_axis(clamp_to_limit_yx(cur_pos, min_limit, max_limit), y_pos_speed, 0, axis_speed_limit)
                    else:
                        cmd = build_axis(safe_target, y_pos_speed, 0, axis_speed_limit)
                elif axis_name.startswith("x"):
                    cmd = build_axis(safe_target, x_pos_speed, 0, axis_speed_limit)
                elif axis_name.startswith("r"):
                    if not x_ready:
                        cmd = build_axis(clamp_to_limit_r(cur_pos, min_limit, max_limit), axis_speed_limit, 0, axis_speed_limit)
                    else:
                        cmd = build_axis(safe_target, axis_speed_limit, 0, axis_speed_limit)
                else:
                    cmd = build_axis(safe_target, x_pos_speed, 0, axis_speed_limit)

                axis_cmds[axis_name] = cmd

        return axis_cmds

    def _resolve_follow_z_speed(self, plc_data):
        chain_speed = int(getattr(plc_data, "ChainSpeed", 0) or 0)
        if not self._is_chain_running(plc_data):
            return 0
        return chain_speed if chain_speed != 0 else 0

    @staticmethod
    def _is_chain_running(plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") == "moving_forward"

    def _check_y_axes_arrived_by_type(self, machine_cfg, plc_data):
        """
        检查设备类型的 Y 轴是否已到达配置安全位

        用于 out_rotate / in_rotate 设备回安全位时判断 Y 轴是否先到位
        """
        tol = self.tolerance
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")
        axis_type_list = machine_cfg.get("axis_type", [])
        axis_map = get_axis_map(machine_type, orientation)
        for axis_name in axis_type_list:
            if axis_name.startswith("y") and axis_name in axis_map:
                cur_pos = self._get_axis_current_pos(plc_data, axis_map[axis_name])
                min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
                target_pos = self._clamp_axis_target(axis_name, self._get_axis_safe_pos(machine_cfg, axis_name), min_limit, max_limit)
                if abs(cur_pos - target_pos) > tol:
                    return False
        return True

    def _check_x_axes_arrived_by_type(self, machine_cfg, plc_data):
        """
        检查设备类型的所有 X 型轴是否已到达配置安全位

        X 型轴: axis_type 中以 "x" 开头的轴 (x, x1, x2, ..., x5)
        """
        tol = self.tolerance
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")
        axis_type_list = machine_cfg.get("axis_type", [])
        axis_map = get_axis_map(machine_type, orientation)
        for axis_name in axis_type_list:
            if axis_name.startswith("x") and axis_name in axis_map:
                cur_pos = self._get_axis_current_pos(plc_data, axis_map[axis_name])
                min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
                target_pos = self._clamp_axis_target(axis_name, self._get_axis_safe_pos(machine_cfg, axis_name), min_limit, max_limit)
                if abs(cur_pos - target_pos) > tol:
                    return False
        return True

    def _check_device_axes_arrived(self, machine_cfg, plc_data):
        """
        检查设备类型的所有轴是否均已到达配置安全位
        """
        tol = self.tolerance
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")
        axis_type_list = machine_cfg.get("axis_type", [])
        axis_map = get_axis_map(machine_type, orientation)
        for axis_name in axis_type_list:
            if axis_name in axis_map:
                cur_pos = self._get_axis_current_pos(plc_data, axis_map[axis_name])
                min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
                target_pos = self._clamp_axis_target(axis_name, self._get_axis_safe_pos(machine_cfg, axis_name), min_limit, max_limit)
                if abs(cur_pos - target_pos) > tol:
                    return False
        return True

    def _get_axis_safe_pos(self, machine_cfg, axis_name):
        return get_axis_safe_pos(machine_cfg, axis_name, default=0)

    def _clamp_axis_target(self, axis_name, target_pos, min_limit, max_limit):
        if axis_name == "z":
            return clamp_to_limit_z(target_pos, min_limit, max_limit)
        if axis_name.startswith("r"):
            return clamp_to_limit_r(target_pos, min_limit, max_limit)
        return clamp_to_limit_yx(target_pos, min_limit, max_limit)

    def _get_axis_speed_limit(self, machine_cfg, axis_name):
        return get_axis_speed_limit(machine_cfg, axis_name, default=300)
