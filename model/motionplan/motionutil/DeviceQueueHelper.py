class DeviceQueueHelper:
    """设备配置与 complete_workpiece 队列访问辅助类。"""

    def iter_machine_cfgs_by_type(self, machine_config: dict, machine_type: str):
        """按设备类型遍历 machine_config。"""
        for sn_str, machine_cfg in machine_config.items():
            if str(machine_cfg.get("type", "") or "").strip() != machine_type:
                continue
            yield int(sn_str), machine_cfg

    def get_device_queue(self, machine_config: dict, frame_queue_manager, sn: int):
        """按设备 SN 获取 complete_workpiece 队列。"""
        machine_cfg = machine_config.get(str(int(sn)))
        if not machine_cfg:
            return None
        direction = str(machine_cfg.get("install_orietation", "") or "").strip()
        if not direction:
            return None
        return frame_queue_manager.queues.get(direction, {}).get(int(sn))
