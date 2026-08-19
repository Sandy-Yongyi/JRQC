class WorkpieceMotionHelper:
    """complete_workpiece 运动规划的通用功能块。"""

    @staticmethod
    def ensure_state(work_states: dict[int, dict], sn: int, default_factory):
        if sn not in work_states:
            work_states[sn] = default_factory()
        return work_states[sn]

    @staticmethod
    def reset_state(work_states: dict[int, dict], sn=None):
        if sn is None:
            work_states.clear()
            return
        work_states.pop(int(sn), None)

    @staticmethod
    def peek_first_block(frame_queue):
        if not frame_queue:
            return None
        first = frame_queue[0]
        if first is None:
            return None
        if isinstance(first, dict):
            return first.get("data")
        return first

    @staticmethod
    def build_block_context(machine_cfg, runtime_cfg, plc_data, block):
        return {
            "machine_cfg": machine_cfg,
            "runtime_cfg": runtime_cfg,
            "plc_data": plc_data,
            "block_data": block,
            "fifo_frame_pos": int(block.fifo_frame_pos or 0),
            "outside_data": block.outside_data,
            "inside_data": block.inside_data,
        }

    @staticmethod
    def find_enabled_gun(block, sn: int, group_type: str | None = None):
        for machine_data in getattr(block, "distribe_gun_list", None) or []:
            machine_id = getattr(machine_data, "machine_id", None)
            if machine_id is None or int(machine_id) != int(sn):
                continue
            for gun_group in getattr(machine_data, "gun_groups", None) or []:
                if group_type is not None and getattr(gun_group, "group_type", None) != group_type:
                    continue
                for gun in getattr(gun_group, "gundata_list", None) or []:
                    if int(getattr(gun, "gun_y_enable", 0) or 0) == 1:
                        return gun
        return None

    @staticmethod
    def _get_z_position_and_offset(machine_cfg, runtime_cfg, offset_key: str):
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        z_offset = int(runtime_cfg.get(offset_key, machine_cfg.get(offset_key, 0)) or 0)
        spray_radius = int(machine_cfg.get("spray_radius", 0) or 0)
        return z_position, z_offset + spray_radius

    @staticmethod
    def get_outside_front_z_pair(ctx, offset_key: str, z_attr: str):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        outside = ctx["outside_data"][0]
        z_position, z_offset = WorkpieceMotionHelper._get_z_position_and_offset(machine_cfg, runtime_cfg, offset_key)
        front_z_machine = z_position - z_offset
        front_z_chain = ctx["fifo_frame_pos"] - int(getattr(outside, z_attr, 0) or 0)
        return front_z_machine, front_z_chain

    @staticmethod
    def get_outside_after_z_pair(ctx, offset_key: str, z_attr: str):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        outside = ctx["outside_data"][0]
        z_position, z_offset = WorkpieceMotionHelper._get_z_position_and_offset(machine_cfg, runtime_cfg, offset_key)
        after_z_machine = z_position + z_offset
        after_z_chain = ctx["fifo_frame_pos"] - int(getattr(outside, z_attr, 0) or 0)
        return after_z_machine, after_z_chain

    @staticmethod
    def get_inside_front_z_pair(ctx, offset_key: str, z_value: int):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        z_position, z_offset = WorkpieceMotionHelper._get_z_position_and_offset(machine_cfg, runtime_cfg, offset_key)
        front_z_machine = z_position + z_offset
        front_z_chain = ctx["fifo_frame_pos"] - int(z_value or 0)
        return front_z_machine, front_z_chain

    @staticmethod
    def get_inside_after_z_pair(ctx, offset_key: str, z_value: int):
        machine_cfg = ctx["machine_cfg"]
        runtime_cfg = ctx["runtime_cfg"]
        z_position, z_offset = WorkpieceMotionHelper._get_z_position_and_offset(machine_cfg, runtime_cfg, offset_key)
        after_z_machine = z_position - z_offset
        after_z_chain = ctx["fifo_frame_pos"] - int(z_value or 0)
        return after_z_machine, after_z_chain
