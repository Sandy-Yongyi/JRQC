from model.motionplan.MachineAxisMap import apply_device_axes_to_list
from model.motionplan.MotionManualOutFxPlanning import MotionManualOutFxPlanning
from model.motionplan.MotionOutFxFramePlanning import MotionOutFxFramePlanning
from model.motionplan.MotionToTarget import MotionToTarget
from model.plc.MovingFrameData import SendMovingFrameData, create_axis_list


class MotionFrameByFramePlanning:
    """frame_by_frame / continuous_bidirectional 模式运动执行。"""

    def __init__(self):
        self.out_fx_planner = MotionOutFxFramePlanning()
        self.manual_out_fx_planner = MotionManualOutFxPlanning()
        self.motion_to_target = MotionToTarget()

    def build_moving_frame(self, proc) -> SendMovingFrameData:
        moving_frame = SendMovingFrameData()
        enable_value = 0
        stop_chain = False

        # 获取使能状态：Operate 的 bit0
        plc_enable = (proc.plc_data.Operate & 0x01) == 1
        axis_list = create_axis_list()
        # 检查伺服状态
        servo_alarm = proc.plc_data.Status != 1

        lidar_abnormal = int(getattr(proc, "lidar_status", 0) or 0) in (1, 2, 3)
        raw_data_timeout = bool(getattr(proc, "raw_data_timeout_active", False))
        force_disable_all = (not plc_enable) or servo_alarm or lidar_abnormal or raw_data_timeout
        force_disable_by_lidar = lidar_abnormal
        if force_disable_by_lidar or raw_data_timeout:
            stop_chain = True

        if self._is_manual_mode_enabled(proc) and not force_disable_all:
            enable_value = self._build_manual_mode_enable_and_axes(proc, axis_list)

        # 自动模式 + 强制回原点场景都统一走该分支，通过 device_operate_enabled=False 实现
        else:
            effective_operate = 0 if force_disable_all else proc.plc_data.Operate
            for sn in range(proc.num_devices):
                machine_cfg = proc.machine_config.get(str(sn))
                if not machine_cfg:
                    continue
                runtime_cfg = proc.runtime_machine_config.get(sn, {})

                device_bit = sn + 1
                device_operate_enabled = (effective_operate & (1 << device_bit)) != 0

                last_device_operate = (proc.last_operate_state & (1 << device_bit)) != 0
                device_just_closed = last_device_operate and not device_operate_enabled

                if not device_operate_enabled:
                    if device_just_closed or proc.device_returning_to_origin[sn]:
                        axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, proc.plc_data)
                        proc.device_returning_to_origin[sn] = not all_ready
                    else:
                        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, proc.plc_data)
                        all_ready = proc.device_origin_complete.get(sn, False)

                    if axis_cmds:
                        apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)

                    proc.device_origin_complete[sn] = all_ready
                    if not all_ready:
                        enable_value |= 1 << device_bit

                else:
                    proc.device_returning_to_origin[sn] = False
                    proc.device_origin_complete[sn] = False

                    axis_cmds, _, device_stop_chain = self.out_fx_planner.auto_out_fx_move(
                        machine_cfg=machine_cfg,
                        runtime_cfg=runtime_cfg,
                        plc_data=proc.plc_data,
                        frame_queue_manager=proc.frame_queue_manager,
                    )

                    stop_chain = device_stop_chain

                    if axis_cmds:
                        apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)

                    enable_value |= 1 << device_bit
            if not force_disable_all:
                enable_value |= 0x01

        proc.last_operate_state = proc.plc_data.Operate

        moving_frame.AxisList = axis_list
        moving_frame.Enable = enable_value
        moving_frame.Gun_Cont1 = 0
        moving_frame.Gun_Cont2 = 0
        moving_frame.HeartBeat = proc.plc_data.HeartBeat
        moving_frame.Operate = 0 if stop_chain else 0x02
        return moving_frame

    def _is_manual_mode_enabled(self, proc) -> bool:
        spray_mode = int(proc.mode_config.get("spray_mode", 0) or 0)
        return spray_mode == 1

    def _build_manual_mode_enable_and_axes(self, proc, axis_list: list) -> int:
        enable_value = 0x01
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg:
                continue
            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            machine_type = machine_cfg.get("type", "")
            device_bit = sn + 1
            device_operate_enabled = (proc.plc_data.Operate & (1 << device_bit)) != 0

            if machine_type == "out_fx" and device_operate_enabled:
                axis_cmds = self.manual_out_fx_planner.auto_manual_out_fx_move(
                    machine_cfg=machine_cfg,
                    runtime_cfg=runtime_cfg,
                    spray_cfg=proc.runtime_spray_config,
                    plc_data=proc.plc_data,
                )
                proc.device_returning_to_origin[sn] = False
                proc.device_origin_complete[sn] = False
                if axis_cmds:
                    apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
                enable_value |= 1 << device_bit
                continue

            axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, proc.plc_data)
            proc.device_returning_to_origin[sn] = not all_ready
            proc.device_origin_complete[sn] = all_ready
            if axis_cmds:
                apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
            if not all_ready:
                enable_value |= 1 << device_bit

        return enable_value
