import os
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.MachineAxisMap import has_axis, get_axis_index, MACHINE_AXIS_MAP, get_axis_position_limits
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader


class MotionUtil:
    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\sprayconfig.toml")
        self.spray_pos_tolerance = self.spray_cfg.get("spray_pos_tolerance", 10)
        self.spray_width_distance = self.spray_cfg.get("spray_width_distance", 50)
        self.motiontotarget = MotionToTarget()

    def find_cabinet_in_queue(self, queue, x_range, y_range, z_range):
        """
        在队列中查找柜体工件。
        如果找到则返回 block 对象，否则返回 None。
        """
        if not queue or queue[0] is None:
            return None
        for i, item in enumerate(queue):
            if item is None:
                continue
            block = item["data"]
            if self.check_if_cabinet(block, x_range, y_range, z_range):
                # logger.info(f"found cabinet at index {i}, queue:{queue}")
                return block
        logger.info("no cabinet found in queue")
        return None

    def check_z_limit(self, plc_data, machine_cfg):
        """
        检查设备的Z轴是否到达最大限位。
        仅支持 MACHINE_AXIS_MAP 中注册的设备类型。
        """
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")

        if machine_type not in MACHINE_AXIS_MAP:
            return False
        # 使用 MACHINE_AXIS_MAP 查找 Z 轴索引
        if not has_axis(machine_type, orientation, "z"):
            return False
        z_idx = get_axis_index(machine_type, orientation, "z")
        _, z_limit = get_axis_position_limits(machine_cfg, "z")
        current_z_pos = self.motiontotarget._get_axis_current_pos(plc_data, z_idx)
        return abs(current_z_pos - z_limit) <= self.spray_pos_tolerance

    def has_arrived_z(self, machine_z, chain_z):
        """检测Z轴是否到达目标"""
        return (machine_z - self.spray_pos_tolerance) <= chain_z <= (machine_z + self.spray_pos_tolerance)

    def has_over_arrive_z(self, machine_z, chain_z):
        """检测Z轴已经超过目标"""
        return chain_z > (machine_z - self.spray_pos_tolerance)

    def check_if_cabinet(self, blockdata, x_range, y_range, z_range):
        """
        检查是否为柜体:outside_x_range的长宽高必须全部大于配置的x_range, y_range, z_range
        """
        if not blockdata.outside_data or len(blockdata.outside_data) == 0:
            return False

        # 取第一个外侧数据进行检查
        outside_data = blockdata.outside_data[0]

        # 计算实际的长宽高
        actual_x_range = (outside_data.outside_x_max or 0) - (outside_data.outside_x_min or 0)
        actual_y_range = (outside_data.outside_y_max or 0) - (outside_data.outside_y_min or 0)
        actual_z_range = (outside_data.outside_z_max or 0) - (outside_data.outside_z_min or 0)

        # 判断是否全部大于配置的阈值
        return (actual_x_range >= x_range and actual_y_range >= y_range and actual_z_range >= z_range)

    def precheck_z_safety_and_drop_blocks(self, machine_cfg, runtime_cfg, plc_data, block, reset_state_cb,):
        """
        喷涂前 Z 安全校验：
        - 如果当前工件 Z 已进入设备 Z 工作区
        - 直接回原点 + 删除工件 + reset 状态
        """
        # 设备 Z 工作区基准 = 配置定位位置 z_position + 当前设备 Z 实际位置
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        machine_z_pos = self._get_machine_z_pos(machine_cfg, plc_data)
        offset_z = z_position + machine_z_pos
        spray_radius = int(machine_cfg.get("spray_radius", 0) or 0)
        z_distance = int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 100)) or 100) + spray_radius
        start_z_machine = offset_z - z_distance
        end_z_machine = offset_z + z_distance
        # 当前工件 Z
        z_chain = block.fifo_frame_pos - block.outside_data[0].outside_z_min
        # logger.info(f"sn={machine_cfg['sn']}, start_z_machine: {start_z_machine}, end_z_machine: {end_z_machine}, z_chain: {z_chain}")
        if (start_z_machine <= z_chain <= end_z_machine) or (z_chain > end_z_machine):
            logger.warning(
                f"[Z-SAFETY] sn={machine_cfg['sn']} "
                f"block_z={z_chain} in machine_z_range=({start_z_machine},{end_z_machine}), DROP block"
            )

            # 回原点
            axis_data, _ = self.motiontotarget.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
            reset_state_cb()
            # 工件完成，删除
            return axis_data, True, True   # done=True, drop_block=True删除工件
        return None, False, False

    def _get_machine_z_pos(self, machine_cfg, plc_data):
        """
        获取设备当前 Z 轴位置。
        仅支持 MACHINE_AXIS_MAP 中注册的设备类型。
        """
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")

        if machine_type in MACHINE_AXIS_MAP:
            if has_axis(machine_type, orientation, "z"):
                z_idx = get_axis_index(machine_type, orientation, "z")
                return self.motiontotarget._get_axis_current_pos(plc_data, z_idx)
        return 0
