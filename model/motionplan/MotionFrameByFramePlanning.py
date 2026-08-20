from model.motionplan.MachineAxisMap import apply_device_axes_to_list
from model.motionplan.MotionCleaningPlanning import MotionCleaningPlanning
from model.motionplan.MotionManualOutFxPlanning import MotionManualOutFxPlanning
from model.motionplan.MotionOutFxFramePlanning import MotionOutFxFramePlanning
from model.motionplan.MotionOutLiftFramePlanning import MotionOutLiftFramePlanning
from model.motionplan.MotionXNUpdownFramePlanning import MotionXNUpdownFramePlanning
from model.motionplan.motionutil.DeviceQueueHelper import DeviceQueueHelper
from model.motionplan.MotionToTarget import MotionToTarget
from model.plc.MovingFrameData import SendMovingFrameData, create_axis_list


class MotionFrameByFramePlanning:
    """frame_by_frame 模式运动执行。"""

    def __init__(self):
        self.out_fx_planner = MotionOutFxFramePlanning()
        self.manual_out_fx_planner = MotionManualOutFxPlanning()
        self.motion_to_target = MotionToTarget()
        self.out_lift_planner = MotionOutLiftFramePlanning()
        self.xn_updown_planner = MotionXNUpdownFramePlanning()
        self.device_queue_helper = DeviceQueueHelper()
        self.cleaning_planner = MotionCleaningPlanning()

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

        # 外二维运动
        self._handle_out_lift(proc, moving_frame)

        clean_mode_enabled, clean_mode_ready = self._resolve_clean_mode_state(proc, force_disable_all)
        clean_mode_just_closed = self._is_clean_mode_just_closed(proc, clean_mode_enabled)
        if clean_mode_enabled and clean_mode_ready:
            stop_chain = True
        if clean_mode_enabled:
            enable_value = self._build_clean_mode_enable_and_axes(proc, clean_mode_ready, axis_list)
        elif self._is_manual_mode_enabled(proc) and not force_disable_all:
            enable_value = self._build_manual_mode_enable_and_axes(proc, axis_list)

        # 自动模式 + 强制回原点场景都统一走该分支，通过 device_operate_enabled=False 实现
        else:
            effective_operate = 0 if force_disable_all else proc.plc_data.Operate
            for sn in range(proc.num_devices):
                machine_cfg = proc.machine_config.get(str(sn))
                if not machine_cfg:
                    continue
                machine_type = machine_cfg.get("type", "")
                if machine_type == "out_lift":
                    continue
                runtime_cfg = proc.runtime_machine_config.get(sn, {})

                device_bit = sn + 1
                device_operate_enabled = (effective_operate & (1 << device_bit)) != 0

                last_device_operate = (proc.last_operate_state & (1 << device_bit)) != 0
                device_just_closed = last_device_operate and not device_operate_enabled
                should_return_safe = self._should_return_safe_before_idle(
                    device_operate_enabled=device_operate_enabled,
                    device_just_closed=device_just_closed,
                    clean_mode_just_closed=clean_mode_just_closed,
                    device_returning=proc.device_returning_to_origin[sn],
                )

                if not device_operate_enabled or should_return_safe:
                    if machine_type == "xn_updown":
                        self.xn_updown_planner.reset_motion_state(sn)
                    if should_return_safe:
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

                    if machine_type == "xn_updown":
                        axis_cmds = self.xn_updown_planner.auto_xn_updown_move(machine_cfg, runtime_cfg, proc.plc_data, proc.frame_queue_manager)
                        device_stop_chain = False
                    else:
                        axis_cmds, _, device_stop_chain = self.out_fx_planner.auto_out_fx_move(
                            machine_cfg=machine_cfg,
                            runtime_cfg=runtime_cfg,
                            plc_data=proc.plc_data,
                            frame_queue_manager=proc.frame_queue_manager,
                        )

                    stop_chain = stop_chain or device_stop_chain

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

    def _build_clean_mode_enable_and_axes(self, proc, clean_mode_ready: bool, axis_list: list) -> int:
        enable_value = 0x01 | self.cleaning_planner.CLEAN_MODE_BIT
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg or machine_cfg.get("type") == "out_lift":
                continue

            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            enable_value = self._handle_clean_mode_device(
                proc=proc,
                sn=sn,
                machine_cfg=machine_cfg,
                runtime_cfg=runtime_cfg,
                clean_mode_ready=clean_mode_ready,
                axis_list=axis_list,
                enable_value=enable_value,
            )

        return enable_value

    def _resolve_clean_mode_state(self, proc, force_disable_all):
        clean_mode_enabled = (not force_disable_all) and self.cleaning_planner.is_clean_mode_enabled(proc.plc_data.Operate)
        clean_mode_ready = clean_mode_enabled and not self.cleaning_planner.has_any_frame_data(proc.frame_queue_manager)
        if clean_mode_enabled and not clean_mode_ready:
            self.cleaning_planner.log_clean_mode_blocked("当前按帧队列中仍有点云，请关闭清理模式")
        return clean_mode_enabled, clean_mode_ready

    def _handle_clean_mode_device(self, proc, sn, machine_cfg, runtime_cfg, clean_mode_ready, axis_list, enable_value):
        if machine_cfg.get("type") == "xn_updown":
            self.xn_updown_planner.reset_motion_state(sn)
        proc.device_returning_to_origin[sn] = False
        proc.device_origin_complete[sn] = False
        axis_cmds = self.cleaning_planner.build_device_axis_cmds(machine_cfg, runtime_cfg, clean_mode_ready)
        if axis_cmds:
            apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
        return enable_value | (1 << (sn + 1))

    def _is_clean_mode_just_closed(self, proc, clean_mode_enabled):
        last_clean_mode_enabled = self.cleaning_planner.is_clean_mode_enabled(proc.last_operate_state)
        return last_clean_mode_enabled and not clean_mode_enabled

    @staticmethod
    def _should_return_safe_before_idle(device_operate_enabled, device_just_closed, clean_mode_just_closed, device_returning):
        if device_returning:
            return True
        if device_just_closed:
            return True
        if clean_mode_just_closed:
            return True
        return False

    def _is_manual_mode_enabled(self, proc) -> bool:
        spray_mode = int(proc.mode_config.get("spray_mode", 0) or 0)
        return spray_mode == 1

    def _build_manual_mode_enable_and_axes(self, proc, axis_list: list) -> int:
        enable_value = 0x01
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg:
                continue
            if machine_cfg.get("type") == "out_lift":
                continue
            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            machine_type = machine_cfg.get("type", "")
            device_bit = sn + 1
            device_operate_enabled = (proc.plc_data.Operate & (1 << device_bit)) != 0
            if machine_type == "xn_updown":
                self.xn_updown_planner.reset_motion_state(sn)

            if machine_type == "xn_updown" and device_operate_enabled:
                axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, proc.plc_data)
                proc.device_returning_to_origin[sn] = False
                proc.device_origin_complete[sn] = False
                if axis_cmds:
                    apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
                enable_value |= 1 << device_bit
                continue

            if machine_type == "out_fx" and device_operate_enabled:
                axis_cmds = self.manual_out_fx_planner.auto_manual_out_fx_move(
                    machine_cfg=machine_cfg, runtime_cfg=runtime_cfg, spray_cfg=proc.runtime_spray_config, plc_data=proc.plc_data
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

    def _handle_out_lift(self, proc, moving_frame):
        for lift_sn, lift_machine_cfg in self.device_queue_helper.iter_machine_cfgs_by_type(proc.machine_config, "out_lift"):
            lift_direction = str(lift_machine_cfg.get("install_orietation", "") or "").strip()
            if lift_direction not in ("left", "right"):
                continue
            runtime_cfg = proc.runtime_machine_config.get(lift_sn, {})
            lift_axis = self.out_lift_planner.auto_out_lift_machine_move(lift_machine_cfg, runtime_cfg, proc.frame_queue_manager)
            if lift_direction == "left":
                moving_frame.Left2DLiftData = lift_axis
            elif lift_direction == "right":
                moving_frame.Right2DLiftData = lift_axis
