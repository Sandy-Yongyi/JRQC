import os
import wx
import sys
import queue
import time
import numpy as np
import multiprocessing
from control.DataProcessWorker import DataProcessWorker
from view.MainFrame import MainFrame
from model.utils.TomlLoader import TomlLoader
from model.utils.LidarDirectionUtil import get_active_directions, get_active_lidar_config
from model.utils.StrategyUtil import is_complete_workpiece_mode, strategy_name_from_code
from model.plot.Draw import PointCloudVisualizer
from model.dataprocess.DataFilter import DataFilter
from view.MachineConfigFrame import MachineConfigFrame
from view.PasswordDialog import PasswordDialog
from model.utils.PasswordConfig import is_password_required
from control.LidarAcquisitionProcess import LidarAcquisitionProcess
from control.PlcCommunicationProcess import PlcCommunicationProcess
from model.utils.LoggerUtil import logger, manage_log_files, log_directory


class RedirectText:
    def __init__(self, text_ctrl):
        self.text_ctrl = text_ctrl

    def write(self, string):
        wx.CallAfter(self.text_ctrl.AppendText, string)

    def flush(self):
        pass


class SubprocessRedirect:
    def __init__(self, queue, is_stderr=False):
        self.queue = queue
        self.is_stderr = is_stderr

    def write(self, s):
        self.queue.put(("stderr" if self.is_stderr else "stdout", s))

    def flush(self):
        pass


class MainFrameController(MainFrame):
    def __init__(self, parent):
        super().__init__(parent)

        self.running = False
        self.processes = []
        self.filter = DataFilter()
        self.log_queue = multiprocessing.Queue()  # 用于收集子进程输出
        self.control_queue = multiprocessing.Queue()  # 专门用于传输按钮状态的队列
        self.control_type = 2  # 1: 调用数据处理, 2: 实际采数联调, 3: 只进行画图

        self.toml_path = os.getcwd() + "\\model\\tomls"
        self.mode_config_path = f"{self.toml_path}\\ModeConfig.toml"
        self.read_data_config = TomlLoader.load(f"{self.toml_path}\\ReadDataConfig.toml")
        self.process_heartbeat_interval_s = float(self.read_data_config.get("process_heartbeat_interval_s", 1))
        self.process_unresponsive_timeout_s = float(self.read_data_config.get("process_unresponsive_timeout_s", 60))
        if self.process_heartbeat_interval_s <= 0 or self.process_unresponsive_timeout_s <= 0:
            raise ValueError("子进程心跳配置必须为正数")
        self.process_heartbeats = {}
        self._process_fault_handled = False
        self.sys_config = TomlLoader.load(f"{self.toml_path}\\SystemConfig.toml")
        self.spray_config = TomlLoader.load(f"{self.toml_path}\\SprayConfig.toml")
        self.mode_config = TomlLoader.load(self.mode_config_path)
        self.strategy_name = strategy_name_from_code(self.mode_config.get("strategy_name"))
        self._ensure_password_config()

        # 重定向标准输出
        sys.stdout = RedirectText(self.status_text)
        sys.stderr = RedirectText(self.status_text)

        # 绑定按钮事件
        self.start_btn.Bind(wx.EVT_BUTTON, self._handle_start_with_password)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._handle_stop_with_password)
        self.left_out_fx_btn.Bind(wx.EVT_BUTTON, lambda e: self._open_machine_config_with_password(0))
        self.left_xn_side_btn.Bind(wx.EVT_BUTTON, lambda e: self._open_machine_config_with_password(1))
        self.right_xn_side_btn.Bind(wx.EVT_BUTTON, lambda e: self._open_machine_config_with_password(2))
        self.Bind(wx.EVT_CLOSE, self.on_close)

    def _ensure_password_config(self):
        password = str(self.mode_config.get("auth_password", "") or "").strip()
        if password:
            return

        default_password = "123456"
        TomlLoader.save({"auth_password": default_password}, self.mode_config_path)
        self.mode_config["auth_password"] = default_password

    def _get_configured_password(self):
        self.mode_config = TomlLoader.load(self.mode_config_path)
        password = str(self.mode_config.get("auth_password", "123456") or "123456")
        return password

    def _is_password_required(self):
        return is_password_required(self.mode_config_path)

    def _verify_button_password(self):
        if not self._is_password_required():
            return True

        dialog = PasswordDialog(self, account_name="河村电器")
        result = dialog.ShowModal()
        input_password = dialog.get_password()
        dialog.Destroy()

        if result != wx.ID_OK:
            return False

        if input_password != self._get_configured_password():
            error_message = "密码输入错误"
            wx.MessageBox(error_message, "报警", wx.OK | wx.ICON_ERROR)
            print(error_message)
            logger.warning(error_message)
            return False

        return True

    def _handle_start_with_password(self, event):
        if self._verify_button_password():
            self.on_start(event)

    def _handle_stop_with_password(self, event):
        if self._verify_button_password():
            self.on_stop(event)

    def _open_machine_config_with_password(self, sn: int):
        if self._verify_button_password():
            self.open_machine_config(sn)

    def open_machine_config(self, sn: int):
        """打开设备配置对话框"""
        if self.control_queue is None:
            wx.MessageBox("请先启动程序后再修改参数", "提示", wx.OK | wx.ICON_INFORMATION)
            return

        # 传递控制队列给对话框
        dlg = MachineConfigFrame(
            self,
            sn,
            self.control_queue,
            strategy_name=self.strategy_name,
        )
        dlg.ShowModal()
        dlg.Destroy()

    @staticmethod
    def _new_process_heartbeat():
        return multiprocessing.Value('d', time.monotonic())

    def _register_process(self, process, heartbeat):
        self.processes.append(process)
        self.process_heartbeats[process] = heartbeat

    def on_start(self, event):
        if self.running:
            print("系统已在运行中")
            return

        print("now start main.exe version is 2026/7/27 PM")
        print("启动系统...")
        self.running = True
        self._process_fault_handled = False
        self.process_heartbeats = {}
        self.on_program_started()
        # 清理日志
        manage_log_files(log_directory)
        self.mode_config = TomlLoader.load(self.mode_config_path)
        spray_mode = int(self.mode_config.get("spray_mode", 0) or 0)
        mode_text = "手动模式" if spray_mode == 1 else "自动模式"
        print(f"当前喷涂模式：{self.strategy_name}")
        logger.info(f"Current spray mode: {mode_text} (spray_mode={spray_mode})")

        # 启动激光采集进程，支持三种策略:
        # continuous_bidirectional：为往复采集，0-3000或者3000-0，不间断采数，只在端点停止采数。（沙特项目）
        # frame_by_frame：为逐帧采集并逐帧数据发送，可跟进fifo进行数据递增堆栈，（星沙、新疆大道等项目）
        # complete_workpiece：为完成工件采集，每个工件采集完后进行分区及分枪处理再进行整个工件的数据发送。（上海展会、河村、欧瑞等项目）
        strategy_name = self.strategy_name

        if self.control_type == 1:
            # --------------------------调试数据处理----------------------------------------
            data_paths = r"D:\draw_points\temp"
            data_name = "20260716_102537.txt"  # 陕西中集

            # 创建进程通信队列
            raw_data_queue = multiprocessing.Queue()
            pulse_queue = multiprocessing.Queue()
            viz_queue = multiprocessing.Queue()
            machine_data_queue = None

            # 启动调试数据导入进程：直接按 frame_by_frame 方式向 PLC 通讯进程提供帧数据
            process_worker_heartbeat = self._new_process_heartbeat()
            process_worker = DataProcessWorker(raw_data_queue,
                                               pulse_queue,
                                               viz_queue,
                                               data_paths,
                                               strategy_name=strategy_name,
                                               config_dir=self.toml_path,
                                               heartbeat=process_worker_heartbeat,
                                               heartbeat_interval_s=self.process_heartbeat_interval_s)
            process_worker.daemon = True
            # 重定向控制器进程输出
            process_worker.stdout = SubprocessRedirect(self.log_queue)
            process_worker.stderr = SubprocessRedirect(self.log_queue, is_stderr=True)
            process_worker.start()
            self._register_process(process_worker, process_worker_heartbeat)
            logger.info(f"process_worker process ID: {process_worker.pid}")

            if is_complete_workpiece_mode(strategy_name):
                from control.DataProcessingProcess import DataProcessingProcess
                machine_data_queue = multiprocessing.Queue()
                data_processing_heartbeat = self._new_process_heartbeat()
                data_processing = DataProcessingProcess(raw_data_queue=raw_data_queue,
                                                        machine_data_queue=machine_data_queue,
                                                        viz_queue=viz_queue,
                                                        config_dir=self.toml_path,
                                                        heartbeat=data_processing_heartbeat,
                                                        heartbeat_interval_s=self.process_heartbeat_interval_s)
                data_processing.daemon = True
                data_processing.stdout = SubprocessRedirect(self.log_queue)
                data_processing.stderr = SubprocessRedirect(self.log_queue, is_stderr=True)
                data_processing.start()
                self._register_process(data_processing, data_processing_heartbeat)
                logger.info(f"start data_processing process ID: {data_processing.pid}")

            # 启动PLC通信进程
            plc_heartbeat = self._new_process_heartbeat()
            plc_handler = PlcCommunicationProcess.create(strategy_name=strategy_name,
                                                         raw_data_queue=raw_data_queue,
                                                         pulse_queue=pulse_queue,
                                                         control_queue=self.control_queue,
                                                         machine_data_queue=machine_data_queue,
                                                         heartbeat=plc_heartbeat,
                                                         heartbeat_interval_s=self.process_heartbeat_interval_s)
            plc_handler.daemon = True
            # 重定向PLC进程输出
            plc_handler.stdout = SubprocessRedirect(self.log_queue)
            plc_handler.stderr = SubprocessRedirect(self.log_queue, is_stderr=True)
            plc_handler.start()
            self._register_process(plc_handler, plc_heartbeat)
            logger.info(f"plc_handler process ID: {plc_handler.pid}")
            # --------------------------调试数据处理----------------------------------------
        elif self.control_type == 2:
            # --------------------------系统软件运行----------------------------------------
            # 创建进程通信队列
            pulse_queue = multiprocessing.Queue()
            raw_data_queue = multiprocessing.Queue()
            viz_queue = multiprocessing.Queue()

            # 激光雷达配置结构
            lidar_config = get_active_lidar_config(self.sys_config)

            lidar_acquisition = None
            if spray_mode == 0:
                t0 = time.perf_counter()
                lidar_heartbeat = self._new_process_heartbeat()
                lidar_acquisition = LidarAcquisitionProcess(pulse_queue=pulse_queue,
                                                            raw_data_queue=raw_data_queue,
                                                            viz_queue=viz_queue,
                                                            lidar_config=lidar_config,
                                                            config_dir=self.toml_path,
                                                            strategy_name=strategy_name,
                                                            heartbeat=lidar_heartbeat,
                                                            heartbeat_interval_s=self.process_heartbeat_interval_s)
                logger.info(f"create lidar_acquisition object took {time.perf_counter() - t0:.3f}s")
                lidar_acquisition.daemon = True
                # 重定向激光采集进程输出
                lidar_acquisition.stdout = SubprocessRedirect(self.log_queue)
                lidar_acquisition.stderr = SubprocessRedirect(self.log_queue, is_stderr=True)
                t1 = time.perf_counter()
                lidar_acquisition.start()
                logger.info(f"lidar_acquisition.start() returned in {time.perf_counter() - t1:.3f}s")
                self._register_process(lidar_acquisition, lidar_heartbeat)
                logger.info(f"start lidar_acquisition process ID: {lidar_acquisition.pid}")
            else:
                logger.info("spray_mode=1 manual mode, skip starting lidar_acquisition process")

            machine_data_queue = None
            if is_complete_workpiece_mode(strategy_name):
                from control.DataProcessingProcess import DataProcessingProcess
                machine_data_queue = multiprocessing.Queue()
                # 启动数据处理进程
                data_processing_heartbeat = self._new_process_heartbeat()
                data_processing = DataProcessingProcess(raw_data_queue=raw_data_queue,
                                                        machine_data_queue=machine_data_queue,
                                                        viz_queue=viz_queue,
                                                        config_dir=self.toml_path,
                                                        heartbeat=data_processing_heartbeat,
                                                        heartbeat_interval_s=self.process_heartbeat_interval_s)
                data_processing.daemon = True
                # 重定向数据处理进程输出
                data_processing.stdout = SubprocessRedirect(self.log_queue)
                data_processing.stderr = SubprocessRedirect(self.log_queue, is_stderr=True)
                data_processing.start()
                self._register_process(data_processing, data_processing_heartbeat)
                logger.info(f"start data_processing process ID: {data_processing.pid}")

            # 使用工厂方法创建 PLC 通信进程
            t2 = time.perf_counter()
            plc_heartbeat = self._new_process_heartbeat()
            plc_handler = PlcCommunicationProcess.create(strategy_name=strategy_name,
                                                         raw_data_queue=raw_data_queue,
                                                         pulse_queue=pulse_queue,
                                                         control_queue=self.control_queue,
                                                         machine_data_queue=machine_data_queue,
                                                         heartbeat=plc_heartbeat,
                                                         heartbeat_interval_s=self.process_heartbeat_interval_s)
            logger.info(f"create plc_handler object took {time.perf_counter() - t2:.3f}s")
            plc_handler.daemon = True
            # 重定向PLC进程输出
            plc_handler.stdout = SubprocessRedirect(self.log_queue)
            plc_handler.stderr = SubprocessRedirect(self.log_queue, is_stderr=True)
            t3 = time.perf_counter()
            plc_handler.start()
            logger.info(f"plc_handler.start() returned in {time.perf_counter() - t3:.3f}s")
            self._register_process(plc_handler, plc_heartbeat)
            logger.info(f"start plc_handler process ID: {plc_handler.pid}")
            # # --------------------------系统软件运行----------------------------------------
        else:
            data_paths = r"D:\draw_points\SXZJ"
            data_name = "20260405_104810.txt"
            # data_name = "20250723_095550.txt"  # 河村试喷大
            # data_name = "20250723_095239.txt"  # 河村试喷中
            # data_name = "20250805_112904.txt"  # 河村试喷小

            # 画图
            visualizer = PointCloudVisualizer()
            for direction in get_active_directions(self.sys_config):
                direction_path = os.path.join(data_paths, f"{direction}_{data_name}")
                if not os.path.exists(direction_path):
                    logger.warning(f"Visualization file not found, skip direction '{direction}': {direction_path}")
                    continue
                data = np.loadtxt(direction_path)
                point_cloud = np.asarray(data)[:, :3]
                visualizer.draw_point_cloud(point_cloud)

        # 启动可视化更新线程
        self.viz_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, lambda e: self.update_visualization(viz_queue), self.viz_timer)
        self.viz_timer.Start(100)  # 每100ms检查一次可视化更新

        # 启动日志收集定时器
        self.log_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.collect_subprocess_output, self.log_timer)
        self.log_timer.Start(200)  # 每200ms检查一次子进程输出

        # 启动进程监控定时器
        self.proc_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.monitor_subprocesses, self.proc_timer)
        self.proc_timer.Start(500)  # 每500ms检查一次子进程状态

    def collect_subprocess_output(self, event):
        """从日志队列中收集子进程输出并显示"""
        try:
            count = 0
            while not self.log_queue.empty() and count < 20:
                source, output = self.log_queue.get_nowait()
                if source == "stdout" and self.status_text:
                    self.status_text.AppendText(output)
                count += 1
        except Exception as e:
            logger.error(f"collect content error {str(e)}")

    def monitor_subprocesses(self, event):
        """监控子进程退出及存活但连续无进展的状态。"""
        if not self.running or self._process_fault_handled:
            return

        dead_processes = [process for process in self.processes if not process.is_alive()]
        if dead_processes:
            dead_names = ", ".join(type(process).__name__ for process in dead_processes)
            self._handle_process_fault("检测到子进程异常退出", f"退出进程：{dead_names}")
            return

        now = time.monotonic()
        stale_processes = []
        for process, heartbeat in self.process_heartbeats.items():
            elapsed = now - float(heartbeat.value)
            if elapsed > self.process_unresponsive_timeout_s:
                stale_processes.append(f"{type(process).__name__}（{elapsed:.1f}秒无响应）")

        if stale_processes:
            detail = "无响应进程：" + "，".join(stale_processes)
            self._handle_process_fault("检测到子进程无响应", detail)

    def _handle_process_fault(self, title: str, detail: str):
        """同一故障只报警一次，并在弹窗前完成有界安全停机。"""
        if self._process_fault_handled:
            return
        self._process_fault_handled = True
        message = f"{title}：{detail}。系统已主动停机，请检查激光/PLC连接、日志和硬件状态后重新启动。"
        logger.error(message)
        self.status_text.AppendText(f"\n\n--- {title} ---\n{detail}\n")
        self.status_text.AppendText("系统已主动停机，避免链条继续运动导致数据错位或撞枪\n")
        self.status_text.AppendText("请检查激光/PLC连接、日志和硬件状态后重新启动系统\n")
        self.on_stop(None)
        wx.MessageBox(message, "子进程异常", wx.OK | wx.ICON_ERROR)

    def update_visualization(self, viz_queue):
        """从队列获取点云数据并更新GL画布"""
        try:
            while not viz_queue.empty():
                data = viz_queue.get_nowait()
                if "points" in data:
                    points = data["points"]
                    if points.shape[0] > 200000:
                        print(f"点数 {points.shape[0]} 超过阈值，进行降采样...")
                        points = self.filter.voxelReductionFilter(points, voxel_size=10.0)
                    wx.CallAfter(self.gl_canvas.set_points, points)
                # 如果有方框数据，设置方框
                if "boxes" in data:
                    boxes_data = data["boxes"]
                    wx.CallAfter(self.gl_canvas.set_boxes, boxes_data)
                # 如果没有方框数据，清除方框
                else:
                    wx.CallAfter(self.gl_canvas.set_boxes, None)
        except queue.Empty:
            pass

    def on_stop(self, event):
        if not self.running:
            return

        print("停止系统...")
        self.running = False

        # 先通知全部子进程停止，再逐个有界等待，避免主界面卡在join()。
        for process in self.processes:
            if process.is_alive():
                try:
                    process.terminate()
                except Exception as e:
                    logger.error(f"Failed to terminate {type(process).__name__}: {str(e)}")

        for process in self.processes:
            try:
                process.join(timeout=2.0)
                if process.is_alive() and hasattr(process, "kill"):
                    logger.error(f"{type(process).__name__} did not exit after terminate, forcing kill")
                    process.kill()
                    process.join(timeout=1.0)
            except Exception as e:
                logger.error(f"Failed to stop {type(process).__name__}: {str(e)}")
        self.processes = []
        self.process_heartbeats = {}

        # 停止定时器
        if hasattr(self, "viz_timer"):
            self.viz_timer.Stop()
        if hasattr(self, "log_timer"):
            self.log_timer.Stop()
        if hasattr(self, "proc_timer"):
            self.proc_timer.Stop()

        # 清空画布
        wx.CallAfter(self.gl_canvas.set_points, np.empty((0, 3)))
        self.on_program_stopped()

    def on_close(self, event):
        if event is not None:
            if not self._verify_button_password():
                event.Veto()
                return

        self.on_stop(None)
        self.Destroy()


class MainApp(wx.App):
    def OnInit(self):
        frame = MainFrameController(None)
        frame.Show()
        frame.Maximize(True)
        return True
