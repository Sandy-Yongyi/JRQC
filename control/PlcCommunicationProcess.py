import os
import sys
import time
import multiprocessing
from typing import Any, cast
from model.utils.LoggerUtil import check_and_rotate_log, logger
from model.plc.PlcCommon import PlcManager
from model.utils.TomlLoader import TomlLoader
from model.plc.MovingFrameData import SendMovingFrameData
from model.utils.FrameQueueManager import FrameQueueManager
from model.formats.frame_by_frame.AxisFrameDataFormat import AxisFrameData
from model.utils.StrategyUtil import validate_runtime_strategy
from model.utils.MachineConfigUtil import (
    MACHINE_OFFSET_KEYS,
    get_machine_config_path,
    normalize_machine_config_offsets,
    normalize_machine_offset_values,
)
from model.utils.ProcessHeartbeat import ProcessHeartbeat
from model.utils.PlcCycleTiming import PlcCycleTiming


class SimulatedPlcData:
    def __init__(self):
        self.ChainPulse = 0.0
        self.ChainSpeed = 0
        self.ChainStatus = "stopped"
        self.ChainCountCM = 0
        self.Status = 1
        # 内置无PLC仿真默认启用左右云雀（PLC→PC bit1/bit2）。
        self.Operate = 0x0006
        self.AxisList = []


class PlcCommunicationProcess(multiprocessing.Process):
    def __init__(
        self,
        raw_data_queue,
        pulse_queue,
        control_queue,
        machine_data_queue=None,
        strategy_name="frame_by_frame",
        heartbeat=None,
        heartbeat_interval_s: float = 1.0,
    ):
        super().__init__()
        self.raw_data_queue = raw_data_queue
        self.pulse_queue = pulse_queue
        self.control_queue = control_queue
        self.machine_data_queue = (
            machine_data_queue  # 保留旧调用签名；完整工件模式在入口拒绝
        )
        self.strategy_name = validate_runtime_strategy(strategy_name)
        self._heartbeat = ProcessHeartbeat(heartbeat, heartbeat_interval_s)

        self.config_dir = os.path.join(os.getcwd(), "model", "tomls")
        self.plc_config_path = os.path.join(self.config_dir, "PlcConfig.toml")
        self.read_data_config_path = os.path.join(
            self.config_dir, "ReadDataConfig.toml"
        )
        self.machine_config_path = get_machine_config_path(
            self.config_dir, self.strategy_name
        )
        self.spray_config_path = os.path.join(self.config_dir, "SprayConfig.toml")
        self.system_config_path = os.path.join(self.config_dir, "SystemConfig.toml")
        self.mode_config_path = os.path.join(self.config_dir, "ModeConfig.toml")
        self.motion_planning_config_path = os.path.join(
            self.config_dir, "MotionPlanningConfig.toml"
        )

        self.plc_config = cast(dict[str, Any], {})
        self.plc_manager = cast(PlcManager, None)
        self.read_data_config = cast(dict[str, Any], {})
        self.machine_config = cast(dict[str, Any], {})
        self.runtime_spray_config = cast(dict[str, Any], {})
        self.mode_config = cast(dict[str, Any], {})
        self.motion_planning_config = cast(dict[str, Any], {})
        self.frame_queue_manager = cast(FrameQueueManager, None)

        self.max_fifo = 0
        self.max_retries = 3
        self.diff_start_pulse = 7
        self.pulse_to_mm = 1
        self.fifo_unit_mm = 2
        self.workpiece_fifo_step_mm = 5
        self.plc_connected = False
        self.pulse_history = []
        self.chain_motion_status = "stopped"
        self.lidar_status = 0
        self.last_lidar_status_log_time = 0.0
        self.last_synced_chain_fifo = None
        self.current_cycle_raw_shift_steps = 0
        self.last_raw_frame_time = 0.0
        self.last_raw_frame_fifo = None
        self.raw_data_timeout_active = False
        self.raw_data_timeout_s = 0.5
        self.total_mm = 0.0
        self.last_workpiece_chain_mm = {}
        self.last_workpiece_chain_mm_residual = {}

        # 运行时设备参数
        self.runtime_machine_config = {}
        self.runtime_param_keys = {
            "yunque_safety_window_front_mm", "yunque_safety_window_back_mm",
            "variable_2d_safety_window_front_mm", "variable_2d_safety_window_back_mm",
            "tracking",
            "y_move_min",
            "y_move_max",
            "x_pos_speed",
            "x_recip_speed",
            "y_pos_speed",
            "y_recip_speed",
            "z_zeroing_speed",
            "x_status_offset",
            "outside_total_cycles",
            "inside_total_cycles",
            "recip_reduce_distance",
            "out_front_x_offset",
            "out_after_x_offset",
            "in_front_x_offset",
            "in_after_x_offset",
            "origin_pos",
            "out_up_y_offset",
            "out_down_y_offset",
            "in_up_y_offset",
            "in_down_y_offset",
            "out_z_front_offset",
            "out_z_after_offset",
            "search_front_z_offset",
            "search_after_z_offset",
            "in_z_front_offset",
            "in_z_after_offset",
            "z_back_speed",
        }

        # 计算设备数量
        self.num_devices = 0

        # 模拟模式相关
        self.simulation_mode = False  # 启用模拟模式
        self.last_simulation_time = 0  # 记录上次模拟处理时间

        # 设备运动状态跟踪
        self.device_origin_complete = {}  # 跟踪每个设备是否全部轴回0
        self.last_operate_state = 0  # 记录上一次的Operate状态，用于检测设备状态变化
        self.device_returning_to_origin = {}  # 跟踪每个设备是否正在回原点
        # 标准输出重定向
        self.stdout = sys.stdout
        self.stderr = sys.stderr

    def _pulse_to_chaincountcm(self, pulse):
        """将 pulse 换算为 chaincountcm。"""
        pulse_to_mm = float(self.pulse_to_mm or 1)
        return int(round(float(pulse) / pulse_to_mm / 10))

    def _resolve_max_fifo(self):
        """基于 pulse 重置阈值推导 chaincountcm 的最大值。"""
        return max(1, self._pulse_to_chaincountcm(self.max_pulse))

    def _get_fifo_step_delta(self, last_fifo: int, current_fifo: int) -> int:
        """计算 FIFO 的有符号步数，支持 max_fifo -> 0 的正常回绕。"""
        ring_size = self.max_fifo + 1
        last_fifo = int(last_fifo) % ring_size
        current_fifo = int(current_fifo) % ring_size
        forward_gap = (current_fifo - last_fifo) % ring_size

        if forward_gap == 0:
            return 0

        # 例如 max_fifo=3018 时：3018 -> 0 的 forward_gap 为 1，视为正常前进一帧。
        if forward_gap <= ring_size // 2:
            return forward_gap

        return forward_gap - ring_size

    @staticmethod
    def create(
        strategy_name,
        raw_data_queue,
        pulse_queue,
        control_queue,
        machine_data_queue=None,
        heartbeat=None,
        heartbeat_interval_s: float = 1.0,
    ):
        """
        工厂方法：根据采集策略创建 PlcCommunicationProcess 实例
        Args:
            strategy_name: 采集策略名称 ('continuous_bidirectional', 'frame_by_frame', 'complete_workpiece')
            raw_data_queue: 原始数据队列
            pulse_queue: 脉冲数据队列
            machine_data_queue: 机器数据队列（仅用于 complete_workpiece 模式）
        Returns:
            PlcCommunicationProcess 实例
        """
        validate_runtime_strategy(strategy_name)
        return PlcCommunicationProcess(
            raw_data_queue,
            pulse_queue,
            control_queue,
            strategy_name=strategy_name,
            heartbeat=heartbeat,
            heartbeat_interval_s=heartbeat_interval_s,
        )

    def run(self):
        logger.info(f"PlcCommunicationProcess.run() entered, pid={os.getpid()}")
        self._heartbeat.touch(force=True)
        # 重定向输出
        if hasattr(self, "stdout"):
            sys.stdout = self.stdout
        if hasattr(self, "stderr"):
            sys.stderr = self.stderr

        try:
            self._initialize_runtime_state()
            # 如果不是模拟模式，初始化PLC连接
            if not self.simulation_mode:
                if not self._initialize_connection():
                    return  # 连接失败退出进程
            # time.sleep(5)  # TODO: 调试时使用，正式环境可移除或调整为更短时间，确保PLC连接稳定后再进入主循环
            # 主处理循环
            self._run_main_loop()

        except Exception as e:
            logger.error(f"Critical error in PLC handler: {str(e)}")
            self._heartbeat.sleep(1)
        finally:
            if self.plc_connected:
                self.plc.disconnect(self.connection_type)

    def _initialize_runtime_state(self):
        """在子进程内初始化重对象，避免 spawn 时在父进程构造/序列化大量数据。"""
        logger.info(f"Initialize PLC runtime with strategy_name={self.strategy_name}")
        self.plc_config = TomlLoader.load(self.plc_config_path)
        self.plc_manager = PlcManager(self.plc_config_path)
        self.read_data_config = TomlLoader.load(self.read_data_config_path)
        self.machine_config = normalize_machine_config_offsets(
            TomlLoader.load(self.machine_config_path)
        )
        self.runtime_spray_config = TomlLoader.load(self.spray_config_path)
        self.mode_config = TomlLoader.load(self.mode_config_path)
        self.motion_planning_config = TomlLoader.load(self.motion_planning_config_path)
        spray_mode = int(self.mode_config.get("spray_mode", 0) or 0)
        mode_text = "手动模式" if spray_mode == 1 else "自动模式"
        logger.info(f"PLC runtime spray mode: {mode_text} (spray_mode={spray_mode})")

        self.max_pulse = self.read_data_config.get("max_pulse", 160000)
        self.max_retries = self.read_data_config.get("plc_send_max_retries", 3)
        self.diff_start_pulse = self.read_data_config.get("diff_start_pulse", 7)
        self.pulse_to_mm = self.read_data_config.get("pulse_to_mm", 1)
        self.fifo_unit_mm = self.read_data_config.get("fifo_unit_mm", 2)
        self.workpiece_fifo_step_mm = float(
            self.read_data_config.get("workpiece_fifo_step_mm", 5) or 5
        )
        self.max_fifo = self._resolve_max_fifo()
        self.raw_data_timeout_s = float(
            self.read_data_config.get("raw_data_timeout_s", 10) or 10
        )
        self.total_mm = float(self.max_pulse) / float(self.pulse_to_mm or 1)
        self.last_raw_frame_time = time.time()
        self.last_raw_frame_fifo = None
        self.last_synced_chain_fifo = None
        self.current_cycle_raw_shift_steps = 0
        self.raw_data_timeout_active = False
        self.last_workpiece_chain_mm = {}
        self.last_workpiece_chain_mm_residual = {}

        self.frame_queue_manager = self._create_frame_queue_manager()

        self.runtime_machine_config = {}
        for sn_str, cfg in self.machine_config.items():
            sn = int(sn_str)
            self.runtime_machine_config[sn] = self._build_runtime_machine_config(cfg)
            runtime_offsets = {
                key: self.runtime_machine_config[sn][key] for key in MACHINE_OFFSET_KEYS
            }
            logger.info(f"SN[{sn}] runtime offsets initialized: {runtime_offsets}")
        self.num_devices = (
            max(int(k) for k in self.machine_config) + 1 if self.machine_config else 0
        )
        self.device_origin_complete = {sn: True for sn in range(self.num_devices)}
        self.device_returning_to_origin = {sn: False for sn in range(self.num_devices)}
        self.last_operate_state = 0

    def _build_runtime_machine_config(self, cfg: dict) -> dict:
        runtime_cfg = {}
        for key in self.runtime_param_keys:
            if key in cfg:
                runtime_cfg[key] = cfg[key]

        flat_cfg = cfg.get("flat")
        if isinstance(flat_cfg, dict):
            runtime_cfg["flat"] = {
                key: value
                for key, value in flat_cfg.items()
                if key in self.runtime_param_keys
            }
        return runtime_cfg

    def _create_frame_queue_manager(self) -> FrameQueueManager:
        """根据当前喷涂策略创建对应参数的 FrameQueueManager。"""
        stack_size = int(
            self.read_data_config.get(f"{self.strategy_name}_queue_size", 1500)
        )
        y_min = int(self.read_data_config.get("combined_y_min", 1100))
        y_max = int(self.read_data_config.get("combined_y_max", 3800))
        y_threshold = int(self.read_data_config.get("y_threshold", 10))
        x_min = int(self.read_data_config.get("combined_x_min", 0))
        x_max = int(self.read_data_config.get("combined_x_max", 900))
        x_threshold = int(self.read_data_config.get("x_threshold", 10))

        logger.info(
            "Create FrameQueueManager with strategy=%s, stack_size=%s, y=[%s,%s,%s], x=[%s,%s,%s]",
            self.strategy_name,
            stack_size,
            y_min,
            y_max,
            y_threshold,
            x_min,
            x_max,
            x_threshold,
        )

        return FrameQueueManager(
            system_config_path=self.system_config_path,
            stack_size=stack_size,
            y_min=y_min,
            y_max=y_max,
            y_threshold=y_threshold,
            x_min=x_min,
            x_max=x_max,
            x_threshold=x_threshold,
            strategy_name=self.strategy_name,
            machine_config=self.machine_config,
            compact_frames=True,
        )

    def _initialize_connection(self):
        """初始化PLC连接"""
        try:
            # 获取配置信息
            self.connection_type = self.plc_config["1"]["connection_type"]
            self.plc = self.plc_manager.get_plc("1")

            # 初始连接
            if not self.plc.connect(self.connection_type):
                # 初始连接失败后尝试重连
                if not self._handle_connection_error(self.plc):
                    logger.error(
                        "PLC initial connection failed permanently. Exiting process."
                    )
                    print("PLC连接失败，请检查连接并重新运行软件")
                    return False
            print("PLC连接成功.")
            self.plc_connected = True

            return True

        except Exception as e:
            logger.error(f"Connection initialization failed: {str(e)}")
            return False

    def _run_main_loop(self):
        """主处理循环，同时处理模拟和实际模式"""
        next_cycle = time.perf_counter()
        while True:
            # 20ms是最短周期，不在接收/规划耗时之外再固定追加90ms。
            # 慢周期结束后立即读取新反馈，不累积赶周期任务。
            remaining = next_cycle - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            next_cycle = time.perf_counter() + 0.02
            self._heartbeat.touch()
            timing = PlcCycleTiming()
            with timing.stage("receive_update"):
                updated = self._update_plc_data(timing)
            if not updated:
                timing.report(logger, 0.0, None)
                continue
            processing_start = time.perf_counter()
            self.current_cycle_raw_shift_steps = 0
            with timing.stage("raw_total"):
                self._process_frame_data(timing)
            with timing.stage("fifo_sync"):
                self._sync_frame_queue_with_chain_fifo()
            with timing.stage("control_update"):
                self._update_enable_value()
            with timing.stage("log_rotate"):
                check_and_rotate_log()
            # 处理数据发送，发送给PLC
            if not self.simulation_mode:
                # logger.info(f"PLC Operate=0b{int(self.plc_data.Operate):016b}")
                with timing.stage("motion_plan"):
                    moving_frame = self._axis_to_moving_frame()  # 转换为运动帧数据
                processing_ms = (time.perf_counter() - processing_start) * 1000
                with timing.stage("send"):
                    self._send_moving_frame(moving_frame)
                timing.report(
                    logger, processing_ms, getattr(self.plc_data, "ChainCountCM", None)
                )

    def _send_moving_frame(self, moving_frame: SendMovingFrameData):
        """发送前落实停链关枪和清理模式停链联锁。"""
        from model.motionplan.MotionFrameByFramePlanning import (
            OPERATE_CHAIN_RUN,
            ENABLE_CLEAN_MODE,
        )

        # PLC清理位只是请求；规划器确认全队列为空才允许进入清理。
        if moving_frame.Enable & ENABLE_CLEAN_MODE:
            moving_frame.Operate &= ~OPERATE_CHAIN_RUN
        # 链条脉冲尚未确认运动或已经停止时，每帧强制关闭所有出粉输出。
        # 保留链条运行请求，避免启动时因关枪联锁而无法启动链条。
        if getattr(self, "chain_motion_status", "stopped") not in (
            "moving_forward",
            "moving_reverse",
        ):
            moving_frame.Gun_Cont1 = 0
            moving_frame.Gun_Cont2 = 0
            for axis in moving_frame.AxisList:
                axis.Status = 0
            # 2D专用槽的Status实际是YMax坐标，不能当作开枪位清零。
        logger.debug(f"moving now send moving_frame {moving_frame}")
        self.plc.send_frame(moving_frame)

    def _update_plc_data(self, timing=None):
        timing = timing if timing is not None else PlcCycleTiming()
        try:
            # 模拟模式处理逻辑
            if self.simulation_mode:
                current_time = time.time()
                # 每0.04秒处理一次, 对应链速1.5
                if current_time - self.last_simulation_time >= 0.3:
                    if not hasattr(self, "simulated_plc_data"):
                        self.simulated_plc_data = SimulatedPlcData()

                    # 当前 fifo 使用 pulse 换算后的 chaincountcm，需与 pulse 同步回绕
                    pulse_step = max(
                        1,
                        int(round(float(self.fifo_unit_mm) * float(self.pulse_to_mm))),
                    )

                    self.simulated_plc_data.ChainPulse = self._advance_simulated_pulse(
                        self.simulated_plc_data.ChainPulse, pulse_step
                    )
                    self.simulated_plc_data.ChainCountCM = self._pulse_to_chaincountcm(
                        self.simulated_plc_data.ChainPulse
                    )

                    self.plc_data = self.simulated_plc_data
                    self.last_simulation_time = current_time
                    self._update_chain_status(
                        self.plc_data.ChainCountCM, self.plc_data.ChainPulse
                    )

            # 实际PLC模式处理逻辑
            else:
                # 接收并处理PLC数据
                with timing.stage("scan"):
                    self.plc_data = self.plc.scan(self.connection_type)
                chaincountcm = self._pulse_to_chaincountcm(self.plc_data.ChainPulse)
                setattr(self.plc_data, "ChainCountCM", chaincountcm)
                self._update_chain_status(chaincountcm, self.plc_data.ChainPulse)

                logger.info(
                    f"plc tcp recv data: self.plc_data.AxisList: {self.plc_data.AxisList}"
                )
                # # 处理脉冲和定位数据
                # self.pulse_queue.put({'pulse': self.plc_data.ChainPulse, 'fifo': self.plc_data.ChainCountCM})

            return True

        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            # 处理连接异常
            if not self.simulation_mode:
                logger.error(f"Connection error during operation: {str(e)}")
                # 尝试重连
                if not self._handle_connection_error(self.plc):
                    logger.error(
                        "Failed to reconnect after connection loss. Exiting process."
                    )
            return False  # 本轮通信失败，不使用旧PLC数据继续计算或发送

        except Exception as e:
            # 处理其他异常
            logger.error(f"Unexpected error: {str(e)}")
            return False

    def _advance_simulated_pulse(self, current_pulse, pulse_step):
        """仿真脉冲：递增到 max_pulse 时重置为 0。"""
        next_pulse = int(current_pulse) + int(pulse_step)
        reset_threshold = int(self.max_pulse)
        return 0 if next_pulse >= reset_threshold else next_pulse

    def _reset_raw_data_timeout_timer(self, fifo=None):
        """重置激光数据超时计时，供链条重新启动或收到新帧时调用。"""
        self.last_raw_frame_time = time.time()
        if fifo is not None:
            self.last_raw_frame_fifo = fifo
        self.raw_data_timeout_active = False

    def _update_chain_status(self, fifo, pulse):
        self.pulse_history.append(pulse)
        if len(self.pulse_history) > 5:
            self.pulse_history.pop(0)

        status = "stopped"
        if len(self.pulse_history) == 5:
            pulse_delta = float(self.pulse_history[-1]) - float(self.pulse_history[0])
            half_pulse_range = float(self.max_pulse) / 2.0
            # 脉冲计数由最大值回到0时仍是正向运动，例如159997 -> 4的实际增量为7。
            if pulse_delta < -half_pulse_range:
                pulse_delta += float(self.max_pulse)
            elif pulse_delta > half_pulse_range:
                pulse_delta -= float(self.max_pulse)
            # logger.info(f"pulse_history: {self.pulse_history}")
            # 停止
            if len(set(self.pulse_history)) == 1:
                status = "stopped"
            elif pulse_delta > self.diff_start_pulse:
                status = "moving_forward"
            elif pulse_delta < -self.diff_start_pulse:
                status = "moving_reverse"

        last_status = getattr(self, "chain_motion_status", None)
        self.chain_motion_status = status
        if hasattr(self, "plc_data") and self.plc_data is not None:
            setattr(self.plc_data, "ChainStatus", status)

        if last_status != status:
            if (
                status in ("moving_forward", "moving_reverse")
                and last_status == "stopped"
            ):
                self._reset_raw_data_timeout_timer(fifo=fifo)
        logger.info(
            f"PLC chain status changed: pulse={pulse}, fifo={fifo}, status={status}"
        )
        self.pulse_queue.put({"pulse": pulse, "fifo": fifo, "status": status})

    def _update_lidar_status_from_packet(self, fifo_data: dict):
        if "lidar_status" in fifo_data:
            self._update_lidar_status(int(fifo_data.get("lidar_status", 0) or 0))

    def _should_skip_raw_packet(self) -> bool:
        return int(getattr(self, "lidar_status", 0) or 0) in (1, 2, 3)

    def _process_frame_data(self, timing=None):
        """frame_by_frame / continuous_bidirectional 模式处理激光采集队列。"""
        timing = timing if timing is not None else PlcCycleTiming()
        try:
            while True:
                with timing.stage("queue_empty"):
                    empty = self.raw_data_queue.empty()
                if empty:
                    break
                with timing.stage("queue_get", track_gc=True):
                    fifo_data = self.raw_data_queue.get_nowait()
                timing.packets += 1
                if not isinstance(fifo_data, dict):
                    continue
                with timing.stage("lidar_status"):
                    self._update_lidar_status_from_packet(fifo_data)
                if self._should_skip_raw_packet():
                    continue
                if bool(fifo_data.get("reset_queue", False)):
                    with timing.stage("queue_reset"):
                        self._clear_all_frame_queues()

                repeat_count = int(fifo_data.get("repeat_count", 1) or 1)
                frame_packet_received = False
                for direction in self.frame_queue_manager.active_directions:
                    frame = fifo_data.get(direction)
                    if not isinstance(frame, AxisFrameData):
                        continue
                    with timing.stage(f"push_{direction}", track_gc=True):
                        self.frame_queue_manager.push_frame(
                            direction, frame, repeat_count=repeat_count
                        )
                    frame_packet_received = True
                    if direction == "left":
                        with timing.stage("raw_log"):
                            logger.debug(
                                f"get raw_data_queue fifo is {fifo_data.get('fifo', 1)}"
                            )
                if frame_packet_received:
                    with timing.stage("packet_finalize"):
                        self._handle_frame_packet_received(fifo_data, repeat_count)
        except Exception as e:
            logger.error(f"Error processing raw data from queue: {e}")

    def _handle_frame_packet_received(self, fifo_data: dict, repeat_count: int):
        self.current_cycle_raw_shift_steps += repeat_count
        was_timeout_active = self.raw_data_timeout_active
        self._reset_raw_data_timeout_timer(fifo=fifo_data.get("fifo", None))
        if was_timeout_active:
            msg = f"激光采样数据已恢复，最新FIFO={self.last_raw_frame_fifo}，超时阈值={self.raw_data_timeout_s}s"
            print(msg)
            logger.info(msg)
            self.raw_data_timeout_active = False

    def _push_empty_frames(self, repeat_count: int):
        """链条在走但没有新激光帧时，补空帧推动队列前进，避免旧数据停在原位。"""
        if repeat_count <= 0:
            return

        for direction in tuple(self.frame_queue_manager.frame_stack.keys()):
            empty_frame = (
                self.frame_queue_manager.create_empty_frame_x()
                if direction in ("left_upper", "right_upper")
                else self.frame_queue_manager.create_empty_frame_y()
            )
            self.frame_queue_manager.push_frame(
                direction, empty_frame, repeat_count=repeat_count
            )

    def _sync_frame_queue_with_chain_fifo(self):
        """以 PLC 当前 FIFO 为准同步 frame_by_frame 帧队列。"""

        if not hasattr(self, "plc_data") or self.plc_data is None:
            return

        current_fifo = int(getattr(self.plc_data, "ChainCountCM", 0) or 0)
        if self.last_synced_chain_fifo is None:
            self.last_synced_chain_fifo = current_fifo
            return

        fifo_delta = self._get_fifo_step_delta(
            self.last_synced_chain_fifo, current_fifo
        )
        if fifo_delta < 0:
            logger.warning(
                f"PLC fifo reversed unexpectedly: last_synced={self.last_synced_chain_fifo}, cur={current_fifo}, reset sync base"
            )
            self.last_synced_chain_fifo = current_fifo
            self.current_cycle_raw_shift_steps = 0
            return

        if fifo_delta > 0:
            missing_shift = max(0, fifo_delta - self.current_cycle_raw_shift_steps)
            if missing_shift > 0:
                elapsed = time.time() - self.last_raw_frame_time
                if elapsed >= self.raw_data_timeout_s:
                    self._push_empty_frames(missing_shift)
                    logger.warning(
                        f"Chain fifo advanced without enough lidar frames: last_synced={self.last_synced_chain_fifo}, "
                        f"cur={current_fifo}, fifo_delta={fifo_delta}, raw_shift={self.current_cycle_raw_shift_steps}, "
                        f"inject_empty_shift={missing_shift}, elapsed={elapsed:.3f}s"
                    )
            self.last_synced_chain_fifo = current_fifo

        self.current_cycle_raw_shift_steps = 0
        self._update_raw_data_watchdog()

    def _update_raw_data_watchdog(self):
        """frame_by_frame 模式下，长时间没有收到激光帧时触发停链保护。"""

        chain_running = (
            getattr(self, "chain_motion_status", "stopped") == "moving_forward"
        )
        if not chain_running:
            if self.raw_data_timeout_active:
                msg = "链条已停止，清除激光采样超时保护状态"
                print(msg)
                logger.info(msg)
            self.raw_data_timeout_active = False
            return

        elapsed = time.time() - self.last_raw_frame_time
        if elapsed <= self.raw_data_timeout_s:
            return

        if not self.raw_data_timeout_active:
            self.raw_data_timeout_active = True
            self._clear_all_frame_queues()
            msg = f"链条运行中已有 {elapsed:.3f}s 未收到新的激光帧，已清空帧队列并请求停链保护，最后一次FIFO={self.last_raw_frame_fifo}"
            print(msg)
            logger.error(msg)

    def _update_lidar_status(self, new_status: int):
        """更新激光状态，并按要求输出节流日志。"""
        new_status = int(new_status or 0)
        old_status = int(getattr(self, "lidar_status", 0) or 0)
        self.lidar_status = new_status

        if new_status in (1, 2, 3):
            if old_status not in (1, 2, 3):
                self._clear_all_frame_queues()
                cleared_count = self._clear_pending_raw_data_queue()
                logger.warning(
                    f"Lidar abnormal, cleared frame queues and dropped {cleared_count} pending raw packets"
                )
            now = time.time()
            if new_status != old_status:
                self.last_lidar_status_log_time = 0.0
            if now - self.last_lidar_status_log_time >= 10:
                if new_status == 1:
                    msg = "激光有遮挡，已清空数据队列并禁止设备运动"
                elif new_status == 2:
                    msg = "激光异常，已清空数据队列并禁止设备运动"
                else:
                    msg = "激光断连，已清空数据队列并禁止设备运动"
                print(msg)
                logger.warning(msg)
                self.last_lidar_status_log_time = now
        elif old_status in (1, 2, 3):
            msg = "激光正常了，设备恢复正常运动"
            print(msg)
            logger.info(msg)
            self.last_lidar_status_log_time = 0.0

    def _clear_all_frame_queues(self):
        for direction in self.frame_queue_manager.active_directions:
            self.frame_queue_manager.clear(direction)
        self.last_workpiece_chain_mm = {}
        self.last_workpiece_chain_mm_residual = {}

    def _clear_pending_raw_data_queue(self):
        cleared_count = 0
        try:
            while True:
                self.raw_data_queue.get_nowait()
                cleared_count += 1
        except Exception:
            pass
        return cleared_count

    def _update_enable_value(self):
        try:
            while not self.control_queue.empty():
                msg = self.control_queue.get_nowait()
                logger.info(f"control_queue recv: {msg}")
                if not isinstance(msg, dict):
                    logger.warning("control_queue msg is not dict, skipped")
                    continue

                # machine config
                if "machine" in msg:
                    machine_msg = msg.get("machine")
                    if not isinstance(machine_msg, dict):
                        logger.warning("machine msg is not dict, skipped")
                        continue
                    sn = machine_msg.get("sn")
                    if sn is None:
                        logger.warning("machine msg without sn, skipped")
                        continue
                    config = {k: v for k, v in machine_msg.items() if k != "sn"}
                    self._handle_config_update(sn=sn, config=config)
                    continue

                # spray config
                if "spray" in msg:
                    spray_cfg = msg.get("spray")
                    if not isinstance(spray_cfg, dict):
                        logger.warning("spray msg is not dict, skipped")
                        continue
                    self._handle_spray_update(spray_cfg)
                    continue
                logger.warning(f"Unknown control message type: {msg}")

        except Exception as e:
            logger.error(f"deal control queue error: {str(e)}")

    def _handle_config_update(self, sn: int, config: dict):
        config = normalize_machine_offset_values(config, fill_missing=False)
        sn_key = str(sn)
        if sn_key in self.machine_config:
            self.machine_config[sn_key].update(config)
        else:
            logger.warning(f"SN[{sn}] not found in machine_config, create one")
            self.machine_config[sn_key] = {"sn": sn, **config}

        if sn not in self.runtime_machine_config:
            logger.warning(f"SN[{sn}] not found, create runtime config")
            self.runtime_machine_config[sn] = {}
        runtime_cfg = self.runtime_machine_config[sn]
        for key, value in config.items():
            if key == "flat" and isinstance(value, dict):
                flat_runtime_cfg = runtime_cfg.setdefault("flat", {})
                for flat_key, flat_value in value.items():
                    if flat_key in self.runtime_param_keys:
                        flat_runtime_cfg[flat_key] = flat_value
                continue
            if key not in self.runtime_param_keys:
                continue
            runtime_cfg[key] = value
        logger.info(f"SN[{sn}] runtime config updated: {runtime_cfg}")

    def _handle_spray_update(self, spray_cfg: dict):
        if not hasattr(self, "runtime_spray_config"):
            self.runtime_spray_config = {}
        for key, value in spray_cfg.items():
            self.runtime_spray_config[key] = value
        logger.info(f"Spray runtime config updated: {self.runtime_spray_config}")

    def _axis_to_moving_frame(self) -> SendMovingFrameData:
        """把运行时逐帧数据交给独立运动规划器生成110字节数据帧。"""

        from model.motionplan.MotionFrameByFramePlanning import (
            MotionFrameByFramePlanning,
        )

        planner = getattr(self, "_frame_by_frame_motion_planner", None)
        if planner is None:
            planner = MotionFrameByFramePlanning(self.motion_planning_config)
            self._frame_by_frame_motion_planner = planner

        # 设备结构、安装方向、z_position和轴限位来自启动配置；界面运行时
        # 更新的速度/偏移覆盖同名字段。这样规划器仍只接收一份完整配置，
        # 且不会依赖控制层以外的辅助文件。
        effective_machine_config = self.machine_config
        if self.runtime_machine_config:
            effective_machine_config = {}
            for sn_key, static_config in self.machine_config.items():
                merged_config = dict(static_config)
                try:
                    runtime_config = self.runtime_machine_config.get(int(sn_key), {})
                except (TypeError, ValueError):
                    runtime_config = {}
                if runtime_config:
                    merged_config.update(runtime_config)
                effective_machine_config[sn_key] = merged_config

        return planner.build_moving_frame(
            strategy_name=self.strategy_name,
            frame_queue_manager=self.frame_queue_manager,
            plc_data=getattr(self, "plc_data", None),
            lidar_status=self.lidar_status,
            raw_data_timeout_active=self.raw_data_timeout_active,
            machine_config=effective_machine_config,
            read_data_config=self.read_data_config,
        )

    def _handle_connection_error(self, plc):
        """处理连接异常，支持初始连接和已连接状态的断开重连"""
        self.plc_connected = False
        max_retries = self.read_data_config["plc_connect_max_retries"]
        for attempt in range(max_retries):
            try:
                self._heartbeat.touch()
                print(f"PLC连接失败，正在尝试第{attempt+1}次重连...")
                try:
                    plc.disconnect(connection_type=self.connection_type)
                except Exception:
                    pass
                self._heartbeat.sleep(10)
                # 尝试重连
                if plc.connect(connection_type=self.connection_type):
                    logger.info("PLC reconnected successfully")
                    print("PLC重连成功.")
                    self.plc_connected = True
                    return True
            except Exception as e:
                logger.error(f"Reconnect attempt {attempt+1} failed: {str(e)}")

        logger.error(f"Failed to reconnect to PLC after {max_retries} attempts")
        return False
