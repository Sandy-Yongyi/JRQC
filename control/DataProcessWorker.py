import os
import queue
import sys
import time
import numpy as np
import multiprocessing
from contextlib import nullcontext
from model.dataprocess.DataSplitting import DataSplitting
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader
from model.utils.LidarDirectionUtil import ALL_DIRECTIONS, get_active_directions
from model.utils.StrategyUtil import is_complete_workpiece_mode, is_frame_by_frame_mode, validate_strategy_name
from model.utils.WorkpieceOriginUtil import get_origin_side, transform_points_for_origin
from model.dataprocess.frame_by_frame.BuildSideFrame import build_side_frame
from model.utils.ProcessHeartbeat import ProcessHeartbeat


# 调试数据处理
class DataProcessWorker(multiprocessing.Process):
    DIRECTIONS = ALL_DIRECTIONS

    def __init__(self, machine_data_queue, pulse_queue, viz_queue, data_paths, data_name=None,
                 strategy_name="frame_by_frame", config_dir=None, heartbeat=None, heartbeat_interval_s: float = 1.0):
        super().__init__()
        self.pulse_queue = pulse_queue
        self.viz_queue = viz_queue
        self.machine_data_queue = machine_data_queue
        self.data_paths = data_paths
        self.data_name = data_name
        self.strategy_name = validate_strategy_name(strategy_name)
        self.config_dir = config_dir or (os.getcwd() + "\\model\\tomls")
        self._heartbeat = ProcessHeartbeat(heartbeat, heartbeat_interval_s)
        # 添加标准输出重定向
        self.stdout = sys.stdout
        self.stderr = sys.stderr

        self.data_split = DataSplitting()
        self.process_config = TomlLoader.load(f"{self.config_dir}\\ProcessConfig.toml")
        self.read_data_config = TomlLoader.load(f"{self.config_dir}\\ReadDataConfig.toml")
        self.system_config = TomlLoader.load(f"{self.config_dir}\\SystemConfig.toml")
        self.draw_type = int(self.read_data_config.get("draw_type", 1))
        self.translate_data_origin = int(self.read_data_config.get("translate_data_origin", 1) or 1)
        self.directions = tuple(get_active_directions(self.system_config))
        # 存储所有帧数据
        self.all_frames = {}
        self.current_frame_index = 0
        self.max_frame_index = 0
        self.current_pulse = 0
        self.current_fifo = 0
        self.max_fifo = self._get_max_chaincountcm()
        self.accum = {direction: [] for direction in self.directions}
        self.lidar_status = 0

    def _get_max_chaincountcm(self):
        """基于 pulse 重置阈值推导 chaincountcm 的最大值。"""
        max_pulse = float(self.read_data_config.get("max_pulse", 160000) or 160000)
        pulse_to_mm = float(self.read_data_config.get("pulse_to_mm", 1) or 1)
        return max(1, int(round(max_pulse / pulse_to_mm / 10)))

    def _heartbeat_guard(self):
        heartbeat = getattr(self, "_heartbeat", None)
        if heartbeat is None:
            return nullcontext()
        return heartbeat.keep_alive(max_duration_s=30)

    def _heartbeat_touch(self):
        heartbeat = getattr(self, "_heartbeat", None)
        if heartbeat is not None:
            heartbeat.touch()

    def _heartbeat_sleep(self, seconds):
        heartbeat = getattr(self, "_heartbeat", None)
        if heartbeat is None:
            time.sleep(seconds)
            return
        heartbeat.sleep(seconds)

    def run(self):
        self._heartbeat.touch(force=True)
        # 重定向输出
        if hasattr(self, 'stdout'):
            sys.stdout = self.stdout
        if hasattr(self, 'stderr'):
            sys.stderr = self.stderr

        if is_frame_by_frame_mode(self.strategy_name):
            # 预先加载所有帧数据
            self._load_all_frames()

            # 主处理循环
            self._process_frames()
            return

        if is_complete_workpiece_mode(self.strategy_name):
            self._process_complete_workpieces()
            return

        logger.warning(f"Unsupported debug strategy: {self.strategy_name}")

    def _load_all_frames(self):
        """按原点配置加载 frame_by_frame 调试点云并保持 Z 空帧。"""
        datasets = self._discover_point_cloud_datasets()
        if not datasets:
            logger.warning(f"No debug point cloud dataset found in: {self.data_paths}")
            return

        dataset = datasets[0]
        if len(datasets) > 1:
            logger.info(
                f"frame_by_frame debug selected first dataset '{dataset['name']}' "
                f"from {len(datasets)} available datasets"
            )
        points_by_source = self._load_dataset_points(dataset)

        if self._uses_same_origin():
            combined_points = points_by_source.get("combined", self._empty_points())
            with self._heartbeat_guard():
                combined_frames = self._split_points_into_frames(combined_points)
            for direction in self.directions:
                self.all_frames[direction] = list(combined_frames)
            visualization_points = combined_points
        else:
            for direction in self.directions:
                direction_points = points_by_source.get(direction, self._empty_points())
                with self._heartbeat_guard():
                    self.all_frames[direction] = self._split_points_into_frames(direction_points)
            visualization_side = get_origin_side(self.read_data_config)
            visualization_points = points_by_source.get(visualization_side, self._empty_points())
            if visualization_points.size == 0:
                logger.warning(f"No frame_by_frame visualization points found for configured side: {visualization_side}")

        if visualization_points.size > 0:
            self._push_visualization_data(visualization_points)

        frame_counts = [len(self.all_frames.get(direction, [])) for direction in self.directions]
        self.max_frame_index = max(frame_counts, default=0) - 1
        logger.info(
            f"Loaded debug dataset '{dataset['name']}', translate_data_origin={self.translate_data_origin}: "
            + ", ".join(
                f"{direction}={len(self.all_frames.get(direction, []))}"
                for direction in self.directions
            )
            + f". Max frame index: {self.max_frame_index}"
        )

    def _uses_same_origin(self):
        return int(
            getattr(
                self,
                "translate_data_origin",
                self.read_data_config.get("translate_data_origin", 1),
            )
            or 1
        ) == 1

    def _get_point_cloud_sources(self):
        if self._uses_same_origin():
            return ("combined",)
        return tuple(self.directions)

    def _normalize_dataset_name(self, data_name):
        name = os.path.basename(str(data_name or "").strip())
        if not name:
            return ""

        source_names = ("combined",) + tuple(
            sorted(self.DIRECTIONS, key=len, reverse=True)
        )
        lower_name = name.lower()
        for source in source_names:
            prefix = f"{source}_"
            if lower_name.startswith(prefix.lower()):
                return name[len(prefix):]
        return name

    def _discover_point_cloud_datasets(self):
        """按共同文件后缀把 combined/各方向点云组织成统一数据集。"""
        if not os.path.isdir(self.data_paths):
            logger.warning(f"Debug point cloud directory not found: {self.data_paths}")
            return []

        file_names = [
            file_name
            for file_name in os.listdir(self.data_paths)
            if os.path.isfile(os.path.join(self.data_paths, file_name))
        ]
        file_lookup = {file_name.lower(): file_name for file_name in file_names}
        sources = self._get_point_cloud_sources()

        if self.data_name:
            dataset_names = [self._normalize_dataset_name(self.data_name)]
        else:
            dataset_names = set()
            for file_name in file_names:
                lower_file_name = file_name.lower()
                if not lower_file_name.endswith(".txt"):
                    continue
                for source in sorted(sources, key=len, reverse=True):
                    prefix = f"{source}_"
                    if lower_file_name.startswith(prefix.lower()):
                        dataset_names.add(file_name[len(prefix):])
                        break
            dataset_names = sorted(dataset_names)

        datasets = []
        for dataset_name in dataset_names:
            if not dataset_name:
                continue
            paths = {}
            for source in sources:
                candidates = [f"{source}_{dataset_name}"]
                if not str(dataset_name).lower().endswith(".txt"):
                    candidates.append(f"{source}_{dataset_name}.txt")
                for candidate in candidates:
                    actual_name = file_lookup.get(candidate.lower())
                    if actual_name:
                        paths[source] = os.path.join(self.data_paths, actual_name)
                        break
            if paths:
                datasets.append({"name": dataset_name, "paths": paths})

        return datasets

    def _load_dataset_points(self, dataset):
        points_by_source = {}
        for source in self._get_point_cloud_sources():
            path = dataset["paths"].get(source)
            if not path:
                logger.warning(
                    f"Dataset '{dataset['name']}' has no {source}_ point cloud file"
                )
                points_by_source[source] = self._empty_points()
                continue
            with self._heartbeat_guard():
                points_by_source[source] = self._safe_load_points(path)
            logger.info(
                f"Loaded debug point cloud: source={source}, path={path}, "
                f"points={points_by_source[source].shape[0]}"
            )
        return points_by_source

    def _safe_load_points(self, path):
        """安全加载点云文件，缺失时返回空点云。"""
        if not os.path.exists(path):
            logger.warning(f"Point cloud file not found: {path}")
            return self._empty_points()

        try:
            data = np.loadtxt(path)
        except (OSError, ValueError) as e:
            logger.warning(f"Failed to load point cloud file '{path}': {e}")
            return self._empty_points()
        data = np.asarray(data, dtype=float)
        if data.size == 0:
            return self._empty_points()
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 3:
            padded = np.zeros((data.shape[0], 3), dtype=float)
            padded[:, :data.shape[1]] = data
            data = padded
        return data[:, :3]

    def _load_direction_frames(self, path):
        """按 Z 轴分帧加载单个方向点云。"""
        with self._heartbeat_guard():
            data = self._safe_load_points(path)
        logger.info(f"Loading data from: {path}, points={data.shape[0]}")
        with self._heartbeat_guard():
            return self._split_points_into_frames(data)

    def _split_points_into_frames(self, data):
        """按 Z 间隔分帧，缺失的 Z 区间保留为空帧。"""
        data = np.asarray(data, dtype=float)
        if data.size == 0:
            return []
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 3:
            return []

        z_threshold = int(
            self.read_data_config.get(
                "z_threshold",
                self.process_config.get("z_threshold", 10),
            )
            or 0
        )
        if z_threshold <= 0:
            raise ValueError(f"z_threshold must be greater than 0, current value: {z_threshold}")

        points = data[:, :3]
        relative_z = points[:, 2] - np.min(points[:, 2])
        z_bins = np.floor(relative_z / z_threshold).astype(int)
        max_z_bin = int(np.max(z_bins))
        frames = []
        for z_bin in range(max_z_bin + 1):
            frame_points = points[z_bins == z_bin]
            if frame_points.size == 0:
                frames.append(self._empty_points())
                continue
            sort_order = np.lexsort((frame_points[:, 0], frame_points[:, 1]))
            frames.append(frame_points[sort_order])
        return frames

    @staticmethod
    def _empty_points():
        return np.empty((0, 3), dtype=float)

    def _process_frames(self):
        """处理所有帧数据，每收到一个脉冲处理一帧"""
        last_fifo = None

        while True:
            try:
                self._heartbeat_touch()
                # 获取脉冲数据
                current_pulse, current_fifo = self._update_pulse_data()

                if last_fifo is None:
                    # 初始化last_fifo为当前fifo
                    last_fifo = current_fifo
                    logger.info(f"Initial FIFO set to {last_fifo}")
                    continue

                # 检查 fifo 是否前进，支持 chaincountcm 随 pulse 回绕后的 max -> 0
                sent_count = self._process_fifo_advance(last_fifo, current_fifo)
                if sent_count > 0:
                    last_fifo = current_fifo
                    if self.current_frame_index > self.max_frame_index:
                        logger.info("All frames processed. Continue sending zero frames while FIFO increments...")

            except Exception as e:
                logger.error(f"Error in frame processing: {str(e)}")
                self._heartbeat_sleep(1)

    def _process_fifo_advance(self, last_fifo, current_fifo):
        """FIFO 每前进一格发送下一帧；FIFO 不变或倒退时不发送。"""
        step_count = self._get_fifo_step_delta(last_fifo, current_fifo)
        if step_count <= 0:
            return 0

        ring_size = self.max_fifo + 1
        for step in range(1, step_count + 1):
            frame_fifo = (int(last_fifo) + step) % ring_size
            self._process_current_frame(real_fifo_key=frame_fifo, repeat_count=1)
            self.current_frame_index += 1
        return step_count

    def _get_fifo_step_delta(self, last_fifo, current_fifo):
        """计算 FIFO 前进了多少步，支持 max_fifo -> 0 的正常回绕。"""
        ring_size = self.max_fifo + 1
        last_fifo = int(last_fifo) % ring_size
        current_fifo = int(current_fifo) % ring_size
        forward_gap = (current_fifo - last_fifo) % ring_size

        if forward_gap == 0:
            return 0

        if forward_gap <= ring_size // 2:
            return forward_gap

        return 0

    def _update_pulse_data(self):
        """从队列获取脉冲数据"""
        latest_pulse = self.current_pulse
        latest_fifo = self.current_fifo
        while not self.pulse_queue.empty():
            try:
                pulse_data = self.pulse_queue.get_nowait()
                latest_pulse = pulse_data['pulse']
                latest_fifo = pulse_data['fifo']
                logger.info(f"latest_pulse is {latest_fifo}")
            except queue.Empty:
                break
        if latest_pulse != self.current_pulse:
            self.current_pulse = latest_pulse
        if latest_fifo != self.current_fifo:
            self.current_fifo = latest_fifo
        return self.current_pulse, self.current_fifo

    def _process_current_frame(self, real_fifo_key, repeat_count):
        """处理当前帧数据：按 FIFO 前进打包四个方向的 1 个数据包发送给 PLC。"""

        # 取出当前帧四个方向点云（xyz）
        direction_data = {}
        for direction in self.directions:
            direction_frames = self.all_frames.get(direction, [])
            if self.current_frame_index < len(direction_frames):
                direction_data[direction] = direction_frames[self.current_frame_index]
            else:
                direction_data[direction] = self._empty_points()

        logger.info(f"Accumulating frame {self.current_frame_index}...")

        for direction, points in direction_data.items():
            if points.size > 0:
                self.accum[direction].append(points)

        frame_packet = {
            "fifo": real_fifo_key,
            "repeat_count": int(repeat_count),
            "lidar_status": int(getattr(self, "lidar_status", 0) or 0),
        }
        frame_config = dict(self.process_config)
        frame_config.update(self.read_data_config)
        for direction in self.directions:
            frame_packet[direction] = build_side_frame(self.accum, direction, frame_config)

        # 清除累积器（开启下一轮）
        self.accum = {direction: [] for direction in self.directions}

        # 发送给 PLC 进程
        self.machine_data_queue.put(frame_packet)

        logger.info(
            f"Sent FIFO {real_fifo_key}, repeat_count={repeat_count}, lidar_status={frame_packet['lidar_status']}: "
            f"{', '.join(self.directions)} frames to PLC"
        )

    def _process_complete_workpieces(self):
        """完整工件调试：按原点模式模拟实时采数进程发送原始数据。"""
        datasets = self._discover_point_cloud_datasets()
        if not datasets:
            logger.warning(f"No complete workpiece debug dataset found in: {self.data_paths}")
            return

        for dataset in datasets:
            self._heartbeat_touch()
            points_by_source = self._load_dataset_points(dataset)
            raw_data, _ = self._build_complete_raw_data(
                points_by_source
            )
            if raw_data is None:
                logger.warning(
                    f"Empty complete workpiece dataset skipped: {dataset['name']}"
                )
                continue

            self.machine_data_queue.put(raw_data)
            logger.info(
                f"Sent complete workpiece debug data: dataset={dataset['name']}, "
                f"translate_data_origin={raw_data['translate_data_origin']}"
            )

            if self.draw_type != 2:
                visualization_sources = (
                    ("combined",)
                    if self._uses_same_origin()
                    else self.directions
                )
                for source in visualization_sources:
                    visualization_points = points_by_source.get(
                        source,
                        self._empty_points(),
                    )
                    if visualization_points.size == 0:
                        continue
                    self._push_visualization_data(visualization_points)
                    # time.sleep(30)
        self._heartbeat_sleep(10000000)  # 正式进程持续刷新心跳，兼容旧调试调用

    def _build_complete_raw_data(self, points_by_source):
        raw_data = {
            "lidar_status": int(getattr(self, "lidar_status", 0) or 0),
            "translate_data_origin": 1 if self._uses_same_origin() else 2,
        }

        if self._uses_same_origin():
            points = points_by_source.get("combined", self._empty_points())
            if points.size == 0:
                return None, self._empty_points()
            raw_data["all_data"] = points[:, :3]
            raw_data["all_stop_pulse"] = 0
            for direction in self.directions:
                raw_data[f"{direction}_stop_pulse"] = 0
            return raw_data, points[:, :3]

        visualization_arrays = []
        for direction in self.directions:
            points = points_by_source.get(direction, self._empty_points())
            if points.size == 0:
                continue
            raw_data[f"{direction}_data"] = points[:, :3]
            raw_data[f"{direction}_stop_pulse"] = 0
            visualization_arrays.append(points[:, :3])

        if not visualization_arrays:
            return None, self._empty_points()
        return raw_data, np.vstack(visualization_arrays)

    def _push_visualization_data(self, points):
        points_array = np.asarray(points, dtype=float)
        if points_array.ndim == 1:
            points_array = points_array.reshape(1, -1)
        if points_array.shape[1] < 3:
            return

        visualization_points = points_array[:, :3]
        if self._uses_same_origin():
            visualization_points = transform_points_for_origin(
                visualization_points,
                self.read_data_config,
            )

        viz_data = {
            "points": visualization_points,
            "boxes": None,
        }
        try:
            self.viz_queue.put(viz_data, block=False)
            logger.info("Debug visualization data sent to queue")
        except Exception as e:
            logger.warning(f"Failed to send debug visualization data: {str(e)}")
