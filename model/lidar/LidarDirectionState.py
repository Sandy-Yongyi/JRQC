import numpy as np


class LidarDirectionState:
    """管理每个激光雷达方向的状态"""

    def __init__(self, direction, config):
        self.direction = direction
        self.config = config
        self.scanning_started = False
        self.has_scanned = False
        self.start_pulse = None
        self.stop_pulse = None
        self.frame_counts = []
        self.xyz_data = np.empty((0, 3))
        self.stop_reason = None        # "points" / "max_length" / "linked"
        self.is_link_anchor = False
        self.last_diff_origin_filtered = np.empty((0, 3))
        self.last_same_origin_filtered = np.empty((0, 3))
        # 方向特定阈值
        self.points_threshold = config[f"{direction}_scan_points_threshold"]

    def should_start_scanning(self, frame_counts):
        """判断是否应该开始扫描"""
        return len(frame_counts) == 5 and all(c >= self.points_threshold for c in frame_counts)

    def should_stop_scanning(self, frame_counts):
        """判断是否应该停止扫描"""
        return len(frame_counts) == 5 and all(c < self.points_threshold for c in frame_counts)

    def update_frame_count(self, frame_count):
        """更新帧计数"""
        self.frame_counts.append(frame_count)
        if len(self.frame_counts) > 5:
            self.frame_counts.pop(0)

    def reset(self):
        """重置状态"""
        self.scanning_started = False
        self.has_scanned = False
        self.start_pulse = None
        self.stop_pulse = None
        self.frame_counts = []
        self.xyz_data = np.empty((0, 3))
        self.stop_reason = None
        self.is_link_anchor = False
        self.last_diff_origin_filtered = np.empty((0, 3))
        self.last_same_origin_filtered = np.empty((0, 3))
