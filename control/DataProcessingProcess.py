import os
import sys
import numpy as np
import multiprocessing
from model.dataprocess.complete_workpiece.DataFindBlocks import DataFindBlocks
from model.formats.complete_workpiece.BlockDataFormat import BlockData
from model.utils.TomlLoader import TomlLoader
from model.utils.WorkpieceOriginUtil import get_origin_side, transform_points_for_origin
from model.utils.LoggerUtil import logger
from model.utils.LoggerUtil import check_and_rotate_log
from model.utils.ProcessHeartbeat import ProcessHeartbeat


class DataProcessingProcess(multiprocessing.Process):
    def __init__(self, raw_data_queue, machine_data_queue, viz_queue=None, config_dir=None,
                 heartbeat=None, heartbeat_interval_s: float = 1.0):
        super().__init__()
        self.raw_data_queue = raw_data_queue
        self.machine_data_queue = machine_data_queue
        self.viz_queue = viz_queue
        self.config_dir = config_dir or (os.getcwd() + "\\model\\tomls")
        self._heartbeat = ProcessHeartbeat(heartbeat, heartbeat_interval_s)
        self.process_config = TomlLoader.load(f"{self.config_dir}\\ProcessConfig.toml")
        self.read_data_config = TomlLoader.load(f"{self.config_dir}\\ReadDataConfig.toml")
        self.draw_type = int(self.read_data_config.get("draw_type", 1))
        self.machine_data_queue.put(0)  # 初始状态
        self.shutdown_flag = False
        # 标准输出重定向
        self.stdout = sys.stdout
        self.stderr = sys.stderr

    def run(self):
        self._heartbeat.touch(force=True)
        if hasattr(self, 'stdout'):
            sys.stdout = self.stdout
        if hasattr(self, 'stderr'):
            sys.stderr = self.stderr
        # 初始化数据处理控制器
        self.data_processor = DataFindBlocks()

        while not self.shutdown_flag:
            try:
                self._heartbeat.touch()
                # 获取原始数据
                if not self.raw_data_queue.empty():
                    raw_data = self.raw_data_queue.get()
                    logger.info("DataProcessingProcess receive Raw data")
                    self._process_data(raw_data)
                    check_and_rotate_log()
                else:
                    self._heartbeat.sleep(0.05)

            except Exception as e:
                logger.error(f"Data processing error: {str(e)}")
                self._heartbeat.sleep(0.1)

    def _process_data(self, raw_data):
        """
        machine_data_queue数据格式：
        [方向1 stop_pulse, 方向1 xyz_data, 方向2 stop_pulse, 方向2 xyz_data, ...]
        """
        result = {}
        lidar_status = int(raw_data.get("lidar_status", 0))
        result["lidar_status"] = lidar_status
        translate_data_origin = int(raw_data.get("translate_data_origin", 1 if "all_data" in raw_data else 2) or 2)

        directions = {"left", "right"}
        if translate_data_origin == 1:
            all_data = raw_data.get("all_data")
            if self._has_valid_points(all_data):
                origin_data_dir = 2 if get_origin_side(self.read_data_config) == "right" else 1
                block_data = self._build_block_data(all_data, lidar_status, data_dir=origin_data_dir, same_origin=True)
                if isinstance(block_data, BlockData):
                    self._push_visualization_data(all_data, block_data)
                    target_directions = [
                        direction for direction in sorted(directions)
                        if f"{direction}_stop_pulse" in raw_data
                    ]
                    if not target_directions:
                        target_directions = sorted(directions)

                    default_stop_pulse = raw_data.get("all_stop_pulse", 0)
                    for direction in target_directions:
                        result[direction] = {
                            "stop_pulse": raw_data.get(f"{direction}_stop_pulse", default_stop_pulse),
                            "data": block_data,
                        }

            self.machine_data_queue.put(result)
            logger.info(f"Processed same-origin data sent to PLC queue:{result}")
            return

        # 按方向处理
        for idx, direction in enumerate(sorted(directions), start=1):
            stop_pulse = raw_data.get(f"{direction}_stop_pulse", 0)
            data = raw_data.get(f"{direction}_data")

            if self._has_valid_points(data):
                block_data = self._build_block_data(data, lidar_status, data_dir=idx, same_origin=False)
                if isinstance(block_data, BlockData):
                    self._push_visualization_data(data, block_data)
                    result[direction] = {"stop_pulse": stop_pulse, "data": block_data}
                    # time.sleep(30)  # TODO：调试模式下看数据

        # 发送处理结果
        self.machine_data_queue.put(result)
        logger.info(f"Processed data sent to PLC queue:{result}")

    @staticmethod
    def _has_valid_points(data):
        if data is None:
            return False
        if isinstance(data, np.ndarray):
            return data.size > 0
        return len(data) > 0

    def _build_block_data(self, data, lidar_status, data_dir, same_origin=False):
        data_array = np.array(data)
        process_points = data_array[:, :3]
        if same_origin:
            process_points = transform_points_for_origin(process_points, self.read_data_config)

        self._heartbeat.touch(force=True)
        processed = self.data_processor.start_process(process_points, lidar_status, data_dir=1)
        self._heartbeat.touch(force=True)
        print(f"data_dir: {data_dir}, processed:{processed}")
        if isinstance(processed, BlockData):
            return processed
        return None

    def _push_visualization_data(self, points, block_data):
        if self.draw_type != 2 or self.viz_queue is None or not isinstance(block_data, BlockData):
            return

        points_array = np.asarray(points, dtype=float)
        if points_array.ndim == 1:
            points_array = points_array.reshape(1, -1)
        if points_array.shape[1] < 3:
            return

        display_points = points_array[:, :3]
        if int(self.read_data_config.get("translate_data_origin", 1) or 1) == 1:
            display_points = transform_points_for_origin(
                display_points,
                self.read_data_config,
            )

        viz_data = {
            "points": display_points,
            "boxes": block_data,
        }

        try:
            self.viz_queue.put(viz_data, block=False)
            logger.info("Processed block visualization data sent to queue")
        except Exception as e:
            logger.warning(f"Failed to send processed visualization data: {str(e)}")
