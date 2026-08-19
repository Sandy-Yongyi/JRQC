import os
import socket
import time
import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Dict
from dataclasses import dataclass
# import sys
# current_dir = os.path.dirname(os.path.abspath(__file__))

# # 项目根目录（假设你的结构是 project_root/model/...）
# project_root = os.path.dirname(os.path.dirname(current_dir))
# model_root = os.path.dirname(current_dir)

# # 添加到 sys.path 以便 import 可用
# if project_root not in sys.path:
#     sys.path.append(project_root)
#     sys.path.append(model_root)
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader


AsciiCommon = b'\x02sRN LMDscandata\x03'
LoginCommon = b'\x02sMN SetAccessMode 03 F4724744\x03'
RebootCommon = b'\x02sMN mSCreboot\x03'


@dataclass
class LidarData:
    lidar_id: str  # 序号
    radius: np.ndarray  # 半径
    angles: np.ndarray  # 角度
    same_cartesian: np.ndarray  # 同一笛卡尔坐标
    diff_cartesian: np.ndarray  # 不同笛卡尔坐标

    def __str__(self):
        return f"Lidar {self.lidar_id} - Points: {len(self.radius)}"


class BaseLidar(ABC):
    def __init__(self, lidar_id: str, config: Dict):
        self.id = lidar_id
        self.host = config["host"]
        self.port = config["port"]
        self.lidar_type = config["type"]
        self.same_origin_offset_x = config["same_origin_offset_x"]
        self.same_origin_offset_y = config["same_origin_offset_y"]
        self.diff_origin_offset_x = config["diff_origin_offset_x"]
        self.diff_origin_offset_y = config["diff_origin_offset_y"]
        self.install_orietation = config["install_orietation"]
        self.start_angle_direction = config["start_angle_direction"]
        self.start_angle = config["start_angle"]
        self.stop_angle = config["stop_angle"]
        self._socket: Optional[socket.socket] = None
        self.lidar_status = 0  # 0=正常，1=有遮挡，2=异常需重启

        read_data_config = TomlLoader.load(os.getcwd() + "\\model\\tomls\\ReadDataConfig.toml")
        self.radius_num = read_data_config["radius_num"]
        self.radius_threshold = read_data_config["radius_threshold"]
        poll_timeout_ms = float(read_data_config.get("socket_recv_poll_timeout_ms", 50))
        max_attempts = int(read_data_config.get("socket_recv_max_attempts", 3))
        if poll_timeout_ms <= 0 or max_attempts <= 0:
            raise ValueError("Socket接收超时配置必须为正数")
        self.socket_recv_poll_timeout_s = poll_timeout_ms / 1000.0
        self.socket_recv_max_attempts = max_attempts

    def connect(self) -> bool:
        max_retries = 1
        for attempt in range(max_retries):
            try:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.connect((self.host, self.port))
                logger.info(f"Lidar {self.id} connected successfully")
                return True
            except socket.error as e:
                if attempt < max_retries - 1:
                    logger.warning(f"Lidar {self.id} connection attempt {attempt+1} failed: {str(e)}. Retrying...")
                    time.sleep(1)  # 等待1秒后重试
                else:
                    logger.error(f"Lidar {self.id} connection failed after {max_retries} attempts: {str(e)}")
                    return False
        return False

    def disconnect(self):
        if self._socket:
            self._socket.close()
            self._socket = None

    def scan(self):
        raw_data = self._read_data()
        if not raw_data:
            self.lidar_status = 2
            logger.error(f"Lidar {self.id} returned empty raw data")
            empty = np.array([], dtype=float)
            empty_points = np.empty((0, 2), dtype=float)
            return LidarData(self.id, empty, empty, empty_points, empty_points), self.lidar_status

        radius, angles = self._parse_data(raw_data)
        if self.lidar_status == 2:
            empty_points = np.empty((0, 2), dtype=float)
            return LidarData(self.id, radius, angles, empty_points, empty_points), self.lidar_status

        # 根据开始角度和结束角度来截取数据（仅type=120需要）
        if self.lidar_type == "120":
            mask = (angles >= self.start_angle) & (angles <= self.stop_angle)
            angles = angles[mask]
            radius = radius[mask]

        r_num = np.sum((radius > 0) & (radius < self.radius_threshold))
        self.lidar_status = 1 if r_num > self.radius_num else 0

        radian = angles * np.pi / 180.0
        same_cartesian = self._to_cartesian_same_origin(radius, radian)
        diff_cartesian = self._to_cartesian_diff_origin(radius, radian)

        return LidarData(self.id, radius, angles, same_cartesian, diff_cartesian), self.lidar_status

    def _read_data(self) -> bytes:
        if self._socket is None:
            raise ConnectionError("Lidar socket not initialized")

        active_socket = self._socket
        data = bytearray()
        total_timeout = self.socket_recv_poll_timeout_s * self.socket_recv_max_attempts
        deadline = time.monotonic() + total_timeout
        previous_timeout = active_socket.gettimeout()
        try:
            active_socket.sendall(AsciiCommon)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout(f"激光{self.id}响应超时，等待约{total_timeout:.3f}秒")

                active_socket.settimeout(min(self.socket_recv_poll_timeout_s, remaining))
                try:
                    chunk = active_socket.recv(4096)
                except socket.timeout:
                    if time.monotonic() >= deadline:
                        raise socket.timeout(f"激光{self.id}响应超时，等待约{total_timeout:.3f}秒")
                    continue

                if not chunk:
                    raise ConnectionError(f"Lidar {self.id} connection closed")

                data.extend(chunk)
                start_index = data.find(b"\x02sRA")
                if start_index >= 0 and data.find(b"\x03", start_index) >= 0:
                    return bytes(data)
        finally:
            active_socket.settimeout(previous_timeout)

    def _parse_data(self, raw: bytes) -> tuple[np.ndarray, np.ndarray]:
        """解析数据"""
        try:
            data_str = raw[raw.find(b"\x02sRA") + 1: raw.find(b"\x03")].decode()
            parts = data_str.split()
            error_status = parts[5]

            if not self._check_error_status(error_status):
                logger.error('lidar error, now restart')
                self.lidar_status = 2
                return np.array([], dtype=float), np.array([], dtype=float)

            dist_index = parts.index("DIST1")
            start_angle = self._get_start_angle(parts, dist_index)
            angular_step = float(int(parts[dist_index + 4], 16) / 10000.0)
            data_count = int(parts[dist_index + 5], 16)

            radius = np.array([int(x, 16) for x in parts[dist_index + 6: dist_index + 6 + data_count]])
            angles = np.linspace(start_angle, start_angle + angular_step * data_count, data_count, dtype=float)

            return radius, angles
        except Exception as e:
            self.lidar_status = 2
            logger.error(f"Parse data error: {str(e)}")
            return np.array([], dtype=float), np.array([], dtype=float)

    @abstractmethod
    def _check_error_status(self, error_status: str) -> bool:
        """检查错误状态，由子类实现"""
        pass

    @abstractmethod
    def _get_start_angle(self, parts: list, dist_index: int) -> float:
        """获取起始角度，由子类实现"""
        pass

    def _to_cartesian_diff_origin(self, radius: np.ndarray, angles: np.ndarray) -> np.ndarray:  # 不同安装方式激光使用不同原点转换方法
        if self.start_angle_direction == 1:  # 0为0度在下方, 1为0度在上方, 2为0度朝右边, 3为0度朝左边, 都是面对激光正面逆时针0-180
            if self.install_orietation == "left":
                x = radius * np.sin(angles) + self.diff_origin_offset_x
                y = radius * np.cos(angles) + self.diff_origin_offset_y
            elif self.install_orietation == "down":
                x = -(radius * np.cos(angles)) + self.diff_origin_offset_x
                y = radius * np.sin(angles) + self.diff_origin_offset_y
            elif self.install_orietation == "right":
                x = radius * np.sin(angles) + self.diff_origin_offset_x
                y = -(radius * np.cos(angles)) + self.diff_origin_offset_y
            elif self.install_orietation == "left_upper":
                x = -(radius * np.cos(angles)) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
            elif self.install_orietation == "right_upper":
                x = radius * np.cos(angles) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
        elif self.start_angle_direction == 0:
            if self.install_orietation == "left":
                x = radius * np.sin(angles) + self.diff_origin_offset_x
                y = -(radius * np.cos(angles)) + self.diff_origin_offset_y
            elif self.install_orietation == "right":
                x = -(radius * np.sin(angles)) + self.diff_origin_offset_x
                y = radius * np.cos(angles) + self.diff_origin_offset_y
            elif self.install_orietation == "down":
                x = radius * np.cos(angles) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
            elif self.install_orietation == "left_upper":
                x = radius * np.cos(angles) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
            elif self.install_orietation == "right_upper":
                x = -(radius * np.cos(angles)) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
        elif self.start_angle_direction == 2:
            if self.install_orietation == "left":
                x = -(radius * np.cos(angles)) + self.diff_origin_offset_x
                y = radius * np.sin(angles) + self.diff_origin_offset_y
            elif self.install_orietation == "right":
                x = radius * np.cos(angles) + self.diff_origin_offset_x
                y = radius * np.sin(angles) + self.diff_origin_offset_y
        elif self.start_angle_direction == 3:
            if self.install_orietation == "left":
                x = radius * np.cos(angles) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
            elif self.install_orietation == "right":
                x = -(radius * np.cos(angles)) + self.diff_origin_offset_x
                y = -(radius * np.sin(angles)) + self.diff_origin_offset_y
        return np.column_stack((x, y))

    def _to_cartesian_same_origin(self, radius: np.ndarray, angles: np.ndarray) -> np.ndarray:  # 不同安装方式激光使用同一原点转换方法
        if self.start_angle_direction == 1:  # 0为0度在下方, 1为0度在上方, 2为0度朝右边, 3为0度朝左边, 都是面对激光正面逆时针0-180
            if self.install_orietation == "left":
                x = radius * np.sin(angles) + self.same_origin_offset_x
                y = radius * np.cos(angles) + self.same_origin_offset_y
            elif self.install_orietation == "down":
                x = -(radius * np.cos(angles)) + self.same_origin_offset_x
                y = radius * np.sin(angles) + self.same_origin_offset_y
            elif self.install_orietation == "right":
                x = -(radius * np.sin(angles)) + self.same_origin_offset_x
                y = -(radius * np.cos(angles)) + self.same_origin_offset_y
            elif self.install_orietation == "left_upper":
                x = radius * np.cos(angles) + self.same_origin_offset_x
                y = -(radius * np.sin(angles)) + self.same_origin_offset_y
            elif self.install_orietation == "right_upper":
                x = radius * np.cos(angles) + self.same_origin_offset_x
                y = -(radius * np.sin(angles)) + self.same_origin_offset_y
        elif self.start_angle_direction == 0:
            if self.install_orietation == "left":
                x = radius * np.sin(angles) + self.same_origin_offset_x
                y = -(radius * np.cos(angles)) + self.same_origin_offset_y
            elif self.install_orietation == "right":
                x = -(radius * np.sin(angles)) + self.same_origin_offset_x
                y = radius * np.cos(angles) + self.same_origin_offset_y
            elif self.install_orietation == "down":
                x = radius * np.cos(angles) + self.same_origin_offset_x
                y = radius * np.sin(angles) + self.same_origin_offset_y
            elif self.install_orietation == "left_upper":
                x = -(radius * np.cos(angles)) + self.same_origin_offset_x
                y = -(radius * np.sin(angles)) + self.same_origin_offset_y
            elif self.install_orietation == "right_upper":
                x = -(radius * np.cos(angles)) + self.same_origin_offset_x
                y = -(radius * np.sin(angles)) + self.same_origin_offset_y
        elif self.start_angle_direction == 2:
            x = -(radius * np.cos(angles)) + self.same_origin_offset_x
            y = radius * np.sin(angles) + self.same_origin_offset_y
        elif self.start_angle_direction == 3:
            x = radius * np.cos(angles) + self.same_origin_offset_x
            y = -(radius * np.sin(angles)) + self.same_origin_offset_y
        return np.column_stack((x, y))


class LidarSick111(BaseLidar):
    """SICK 111型激光雷达"""
    def _check_error_status(self, error_status: str) -> bool:
        return error_status != '1'

    def _get_start_angle(self, parts: list, dist_index: int) -> float:
        return float(int(parts[dist_index + 3], 16) / 10000.0)


class LidarSick120(BaseLidar):
    """SICK 120型激光雷达"""
    def _check_error_status(self, error_status: str) -> bool:
        return error_status == '0'

    def _get_start_angle(self, parts: list, dist_index: int) -> float:
        return -48.0


class LidarSick483(BaseLidar):
    """SICK 483型激光雷达"""
    def _check_error_status(self, error_status: str) -> bool:
        return error_status == '0'

    def _get_start_angle(self, parts: list, dist_index: int) -> float:
        return float(int(parts[dist_index + 3], 16) / 10000.0)


class LidarManager:
    def __init__(self, config_path: str):
        self.config = TomlLoader.load(config_path)
        self.lidars: Dict[str, BaseLidar] = {}
        self._init_lidars()

    def _init_lidars(self):
        lidar_class_map = {
            "111": LidarSick111,
            "120": LidarSick120,
            "483": LidarSick483,
        }
        for lidar_id, cfg in self.config.items():
            lidar_type = cfg.get("type", "111")
            lidar_class = lidar_class_map.get(lidar_type, LidarSick111)
            self.lidars[lidar_id] = lidar_class(lidar_id, cfg)

    def get_lidar(self, lidar_id: str) -> BaseLidar:
        if lidar_id not in self.lidars:
            raise ValueError(f"Invalid lidar ID: {lidar_id}")
        return self.lidars[lidar_id]


if __name__ == "__main__":
    lidar_manager = LidarManager(os.getcwd() + "\\model\\tomls\\LidarConfig.toml")
    lidar = lidar_manager.get_lidar("5")
    lidar.connect()
    lidar.scan()
