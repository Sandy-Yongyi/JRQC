from __future__ import annotations

from model.motionplan.motionutil.DeviceQueueHelper import DeviceQueueHelper
from model.motionplan.MachineAxisMap import apply_device_axes_to_list
from model.motionplan.MotionCleaningPlanning import MotionCleaningPlanning
from model.motionplan.MotionOutFxCompleteWorkpiecePlanning import MotionOutFxCompleteWorkpiecePlanning
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.MotionXNSidePlanning import MotionXNSidePlanning
from model.plc.MovingFrameData import SendMovingFrameData, create_axis_list
from model.utils.LoggerUtil import logger


class MotionCompleteWorkpiecePlanning:
    """完整工件模式下的统一运动规划入口。"""

    def __init__(self):
        self.device_queue_helper = DeviceQueueHelper()
        self.cleaning_planner = MotionCleaningPlanning()
        self.out_fx_planner = MotionOutFxCompleteWorkpiecePlanning()
        self.xn_side_planner = MotionXNSidePlanning()
        self.motion_to_target = MotionToTarget()

    def build_moving_frame(self, proc) -> SendMovingFrameData:
        """
        根据接收的报文 ReceiveMovingFrameData 生成发送的报文 SendMovingFrameData

        逻辑流程：
        1. 当 Operate 的 bit0（使能位）为 0 时，所有设备都需要回原点
        2. 当使能打开时，需要判断 Status 是否为 1（伺服正常）
           - 如果伺服报警（Status != 1），所有设备回原点
           - 如果伺服正常，根据 Operate 的各设备位来控制设备运动
        3. 根据设备的回原点状态设置 Enable 的对应位：
           - 如果设备全部轴回0，对应位发 0
           - 如果设备轴未回0，对应位发 1
        4. 将所有设备的轴数据拼接到 AxisList 中
        5. 当任意设备返回当前工件可删除时，立即调用 `after_spray_complete()` 删除当前工件

        Returns:
            SendMovingFrameData: 包含运动指令的发送帧
        """
        moving_frame = SendMovingFrameData()
        enable_value = 0
        stop_chain = False

        # 获取使能状态：Operate 的 bit0
        plc_enable = (proc.plc_data.Operate & 0x01) == 1
        axis_list = create_axis_list()
        # 检查伺服状态
        servo_alarm = proc.plc_data.Status != 1
        operate_u16 = proc.plc_data.Operate & 0xFFFF
        logger.debug(f"plc_enable={plc_enable}, servo_alarm={servo_alarm}, Operate={operate_u16}, {operate_u16:016b}")
        lidar_abnormal = int(getattr(proc, "lidar_status", 0) or 0) in (1, 2, 3)
        force_disable_all = (not plc_enable) or servo_alarm or lidar_abnormal
        force_disable_by_lidar = lidar_abnormal
        if force_disable_by_lidar:
            stop_chain = True

        # 外二维运动
        effective_operate = 0 if force_disable_all else proc.plc_data.Operate
        clean_mode_enabled, clean_mode_ready = self._resolve_clean_mode_state(proc, force_disable_all)
        clean_mode_just_closed = self._is_clean_mode_just_closed(proc, clean_mode_enabled)
        if clean_mode_enabled and clean_mode_ready:
            stop_chain = True

        # logger.info("使能打开且伺服正常，处理各设备运动")
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg:
                continue
            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            direction = str(machine_cfg.get("install_orietation", "") or "").strip()
            device_queue = self.device_queue_helper.get_device_queue(proc.machine_config, proc.frame_queue_manager, sn)
            done = False

            if clean_mode_enabled:
                enable_value = self._handle_clean_mode_device(
                    proc=proc,
                    sn=sn,
                    machine_cfg=machine_cfg,
                    runtime_cfg=runtime_cfg,
                    clean_mode_ready=clean_mode_ready,
                    axis_list=axis_list,
                    enable_value=enable_value,
                )
                continue

            # 检查该设备的 Operate 位是否开启
            # SN 与位顺序：
            #   sn0 -> out_fx
            #   sn1 -> xn_side
            #   sn2 -> xn_side
            # 因此 sn 0-5 依次对应 bit1-bit6
            device_bit = sn + 1
            device_operate_enabled = (effective_operate & (1 << device_bit)) != 0
            machine_type = machine_cfg.get("type", "")

            # 检查设备是否从开启变为关闭（用于对刚才关闭的设备进行回原点处理）
            last_device_operate = (proc.last_operate_state & (1 << device_bit)) != 0
            device_just_closed = last_device_operate and not device_operate_enabled
            if device_just_closed:
                self._reset_device_motion_state(machine_type, sn)
                logger.info(f"SN[{sn}] device closed, reset complete-workpiece motion state")
            should_return_safe = self._should_return_safe_before_idle(
                device_operate_enabled=device_operate_enabled,
                device_just_closed=device_just_closed,
                clean_mode_just_closed=clean_mode_just_closed,
                device_returning=proc.device_returning_to_origin[sn],
            )
            # logger.debug(f"SN[{sn}] device_operate_enabled={device_operate_enabled}, device_just_closed={device_just_closed}")

            # 设备运动已关闭或刚刚关闭 → 回原点或保持位置
            if not device_operate_enabled or should_return_safe:
                if should_return_safe:
                    # 设备刚从开启变为关闭，或已在回原点途中 → 持续回原点
                    # logger.info(f"SN[{sn}] 设备关闭，持续回原点")
                    axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, proc.plc_data)
                    proc.device_returning_to_origin[sn] = not all_ready
                else:
                    # 设备一直都是关闭状态 → 保持当前位置
                    # logger.info(f"SN[{sn}] 设备保持当前位置")
                    axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, proc.plc_data)
                    all_ready = proc.device_origin_complete.get(sn, False)

                if axis_cmds:
                    apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)

                # 更新设备回原点状态
                proc.device_origin_complete[sn] = all_ready
                if not all_ready:
                    enable_value |= 1 << device_bit

            # 设备运动已开启 → 执行自动运动逻辑
            else:
                # logger.info(f"SN[{sn}] 设备运动开启，执行自动运动")
                proc.device_returning_to_origin[sn] = False
                proc.device_origin_complete[sn] = False

                device_stop_chain = False
                if machine_type == "out_fx":
                    axis_cmds, done, device_stop_chain = self.out_fx_planner.auto_out_fx_complete_move(
                        machine_cfg=machine_cfg,
                        runtime_cfg=runtime_cfg,
                        plc_data=proc.plc_data,
                        frame_queue=device_queue,
                    )
                elif machine_type == "xn_side":
                    axis_cmds, done, device_stop_chain = self.xn_side_planner.auto_xn_side_machine_move(
                        machine_cfg=machine_cfg,
                        runtime_cfg=runtime_cfg,
                        plc_data=proc.plc_data,
                        frame_queue=device_queue,
                    )
                else:
                    axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, proc.plc_data)

                stop_chain = stop_chain or device_stop_chain

                if axis_cmds:
                    apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)

                if done and direction:
                    proc.after_spray_complete(direction, sn)

                # 设备正在运动，Enable 对应位发1
                enable_value |= 1 << device_bit
        if not force_disable_all:
            enable_value |= 0x01
        # 保存当前 Operate 状态作为下一次的 last_operate_state
        proc.last_operate_state = proc.plc_data.Operate

        moving_frame.AxisList = axis_list
        moving_frame.Enable = enable_value
        moving_frame.Gun_Cont1 = 0
        moving_frame.Gun_Cont2 = 0
        moving_frame.HeartBeat = proc.plc_data.HeartBeat
        moving_frame.Operate = 0 if stop_chain else 0x02
        # logger.info(f"生成 SendMovingFrameData: Enable={moving_frame.Enable:032b}, Operate={moving_frame.Operate}")
        return moving_frame

    def _resolve_clean_mode_state(self, proc, force_disable_all):
        clean_mode_enabled = (not force_disable_all) and self.cleaning_planner.is_clean_mode_enabled(proc.plc_data.Operate)
        clean_mode_ready = clean_mode_enabled and not self.cleaning_planner.has_any_workpiece(proc.frame_queue_manager)
        if clean_mode_enabled and not clean_mode_ready:
            self.cleaning_planner.log_clean_mode_blocked()
        return clean_mode_enabled, clean_mode_ready

    def _handle_clean_mode_device(self, proc, sn, machine_cfg, runtime_cfg, clean_mode_ready, axis_list, enable_value):
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

    def _reset_device_motion_state(self, machine_type, sn):
        """设备关闭时清理当前工件的主状态与往复子状态，避免重新开启后续喷旧工件。"""
        if machine_type == "out_fx":
            self.out_fx_planner.reset_motion_state(sn)
        elif machine_type == "xn_side":
            self.xn_side_planner.reset_side_motion_state(sn)
