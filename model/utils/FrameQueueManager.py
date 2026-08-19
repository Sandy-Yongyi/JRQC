import copy
from typing import Any, Callable, Dict, List
from model.utils.LoggerUtil import logger
from model.utils.StrategyUtil import is_complete_workpiece_mode, validate_strategy_name
from model.utils.TomlLoader import TomlLoader
from model.utils.LidarDirectionUtil import SIDE_DIRECTIONS, UPPER_DIRECTIONS, get_active_lidar_config
from model.formats.frame_by_frame.AxisFrameDataFormat import AxisFrameData, AxisData


class FrameQueueManager:
    """
    激光数据帧队列管理器（left / right / left_upper / right_upper）
    功能：
        - frame_by_frame / continuous_bidirectional:
            - 根据 SystemConfig.toml 自动创建方向（left / right / left_upper / right_upper）
            - 每个方向一个固定长度 FIFO 队列
            - left / right 按 Y 轴范围初始化全零帧
            - left_upper / right_upper 按 X 轴范围初始化全零帧
        - complete_workpiece:
            - 根据 machine_config 的安装方向动态创建方向队列
            - 每个方向下按设备 sn 建立固定长度的工件队列，初始值为 None
    """

    # 侧面方向（使用 Y 轴范围）
    _SIDE_DIRECTIONS = SIDE_DIRECTIONS
    # 顶面方向（使用 X 轴范围）
    _UPPER_DIRECTIONS = UPPER_DIRECTIONS

    def __init__(self, system_config_path: str, stack_size: int = 1500, y_min: int = 1100, y_max: int = 3800,
                 y_threshold: int = 10, x_min: int = 0, x_max: int = 900, x_threshold: int = 10,
                 strategy_name: str = "frame_by_frame", machine_config: dict | None = None):
        self.system_config_path = system_config_path
        self.stack_size = stack_size
        self.y_min = y_min
        self.y_max = y_max
        self.y_threshold = y_threshold
        self.x_min = x_min
        self.x_max = x_max
        self.x_threshold = x_threshold
        self.strategy_name = validate_strategy_name(strategy_name)
        self.machine_config = machine_config or {}
        self.frame_stack: Dict[str, List[AxisFrameData]] = {}
        self.queues: Dict[str, Dict[int, List[Any]]] = {}
        self.active_directions = ()
        self._initialize_stacks()

    def _initialize_stacks(self):
        """按策略初始化队列结构。"""
        if is_complete_workpiece_mode(self.strategy_name):
            self._initialize_complete_workpiece_queues()
            return

        self._initialize_frame_stacks()

    def _initialize_frame_stacks(self):
        """初始化 frame_by_frame / continuous_bidirectional 模式的帧队列。"""

        try:
            system_cfg = TomlLoader.load(self.system_config_path)
        except Exception as e:
            logger.error(f"Failed to load SystemConfig.toml: {e}")
            return

        active_lidar_config = get_active_lidar_config(system_cfg)
        self.active_directions = tuple(active_lidar_config.keys())

        for direction in self.active_directions:
            creator = self.create_empty_frame_x if direction in self._UPPER_DIRECTIONS else self.create_empty_frame_y
            self.frame_stack[direction] = self._create_empty_stack(creator)

        if not self.frame_stack:
            logger.warning("SystemConfig.toml has no lidar directions; no stacks created.")

    def _initialize_complete_workpiece_queues(self):
        """根据 machine_config 初始化 complete_workpiece 模式的按方向/设备队列。"""
        direction_map: Dict[str, Dict[int, List[Any]]] = {}

        for sn_str, cfg in self.machine_config.items():
            orientation = str(cfg.get("install_orietation", "") or "").strip()
            if not orientation:
                logger.warning(f"SN {sn_str} missing install_orietation, skipped in complete_workpiece queues")
                continue

            sn = int(sn_str)
            direction_map.setdefault(orientation, {})[sn] = [None for _ in range(self.stack_size)]

        self.queues = direction_map
        self.active_directions = tuple(direction_map.keys())

        if not self.queues:
            logger.warning("machine_config has no valid orientations; no complete_workpiece queues created.")

    def _create_empty_stack(self, frame_creator: Callable) -> List[AxisFrameData]:
        """生成一个 stack_size 长度的空 AxisFrameData 队列"""
        return [frame_creator() for _ in range(self.stack_size)]

    def create_empty_frame_y(self) -> AxisFrameData:
        """
        创建空 AxisFrameData（Y 轴范围，用于 left / right）：
        - 按 y_min ~ y_max, y_threshold 步长生成行
        - 所有值初始化为 0
        """
        frame_list = [AxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0) for _ in range(self.y_min, self.y_max, self.y_threshold)]
        return AxisFrameData(FrameData=frame_list)

    def create_empty_frame_x(self) -> AxisFrameData:
        """
        创建空 AxisFrameData（X 轴范围，用于 left_upper / right_upper）：
        - 按 x_min ~ x_max, x_threshold 步长生成行
        - 所有值初始化为 0
        """
        frame_list = [AxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0) for _ in range(self.x_min, self.x_max, self.x_threshold)]
        return AxisFrameData(FrameData=frame_list)

    def create_empty_frame(self) -> AxisFrameData:
        """向后兼容：等同于 create_empty_frame_y"""
        return self.create_empty_frame_y()

    def push_frame(self, direction: str, frame_data: Any, repeat_count: int = 1):
        """
        向 frame_by_frame 队列写入新帧。

        Args:
            direction: 方向
            frame_data: AxisFrameData
            repeat_count: 需要连续补入的帧数。
                - 正常情况下为 1
                - 若 FIFO 跳变，则可将当前帧重复补齐多个位置
        """
        if direction not in self.frame_stack:
            logger.error(f"push_frame failed: direction '{direction}' not initialized.")
            return

        if repeat_count <= 0:
            return

        stack = self.frame_stack[direction]
        shift_count = min(int(repeat_count), self.stack_size)

        # 批量前插当前帧，再截断尾部，避免逐帧循环右移
        self.frame_stack[direction] = [frame_data] * shift_count + stack[: self.stack_size - shift_count]

    def push_workpiece(self, direction: str, sn: int, data: Any):
        """complete_workpiece 模式下向指定方向/设备队列追加一个工件数据。"""
        if direction not in self.queues:
            logger.error(f"Direction {direction} not found in queues")
            return False

        if sn not in self.queues[direction]:
            logger.error(f"SN {sn} not found in {direction} queues")
            return False

        data_copy = copy.deepcopy(data) if data is not None else None
        queue = self.queues[direction][sn]

        all_empty = all(item is None or self._is_empty_data(item) for item in queue)

        if all_empty:
            queue[0] = data_copy
            return True

        if queue[-1] is not None and not self._is_empty_data(queue[-1]):
            self.shift_queue(direction, sn)
            queue[-1] = data_copy
            return True

        for i in range(len(queue)):
            if queue[i] is None or self._is_empty_data(queue[i]):
                queue[i] = data_copy
                return True

        return False

    def _is_empty_data(self, data: Any) -> bool:
        if data is None:
            return True
        if isinstance(data, dict):
            block = data.get("data")
            if block is None:
                return True
            if hasattr(block, "is_empty") and block.is_empty:
                return True
            return False
        if hasattr(data, "is_empty"):
            return bool(getattr(data, "is_empty"))
        return False

    def shift_queue(self, direction: str, sn: int):
        """complete_workpiece 模式下队列整体左移，丢弃最旧工件。"""
        if direction not in self.queues or sn not in self.queues[direction]:
            return False

        queue = self.queues[direction][sn]
        for i in range(len(queue) - 1):
            queue[i] = queue[i + 1]
        queue[-1] = None
        return True

    def clear(self, direction: str):
        """按当前模式清空指定方向队列。"""
        if is_complete_workpiece_mode(self.strategy_name):
            self.clear_workpieces(direction)
            return

        self.clear_frames(direction)

    def clear_frames(self, direction: str):
        """清空 frame_by_frame / continuous_bidirectional 模式的帧队列。"""
        if is_complete_workpiece_mode(self.strategy_name):
            logger.error("clear_frames is not available in complete_workpiece mode")
            return

        if direction in self.frame_stack:
            if direction in self._UPPER_DIRECTIONS:
                creator = self.create_empty_frame_x
            else:
                creator = self.create_empty_frame_y
            self.frame_stack[direction] = self._create_empty_stack(creator)

    def clear_workpieces(self, direction: str):
        """清空 complete_workpiece 模式的某个方向工件队列。"""
        if direction in self.queues:
            for sn in self.queues[direction]:
                self.queues[direction][sn] = [None for _ in range(self.stack_size)]
