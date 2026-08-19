from queue import Empty, Queue
import threading
import time
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import open3d as o3d
from model.utils.LoggerUtil import logger


class PointCloudVisualizer:
    def __init__(self, x_range=None, y_range=None, z_range=None):
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.fig = None
        self.ax = None
        self.scatter = None
        self.current_data = np.empty((0, 4))

        # 存储原始范围（参数名对应原始数据轴）
        self.original_ranges = {"x": self._validate_range(x_range), "y": self._validate_range(y_range), "z": self._validate_range(z_range)}
        # 原X轴 → 新Z轴（取反）  # 原Y轴 → 新Y轴  # 原Z轴 → 新X轴

        self.visualization_thread = None
        self.geometry_queue = Queue(maxsize=1)  # 用于传递几何体数据
        self.shutdown_flag = threading.Event()  # 用于安全关闭线程
        self.lock = threading.Lock()  # 线程锁
        self.window_closed = True  # 初始状态为已关闭

    def plot_lidar_comparison_with_energy(self, data, passthrough_filtered_data, energy_filtered_data):
        # 提取 x 和 y 值
        x_data = data[:, 0]
        y_data = data[:, 1]
        energy_data = data[:, 2]

        x_energy_filtered = energy_filtered_data[:, 0]
        y_energy_filtered = energy_filtered_data[:, 1]
        energy_filtered = energy_filtered_data[:, 2]

        x_passthrough_filtered = passthrough_filtered_data[:, 0]
        y_passthrough_filtered = passthrough_filtered_data[:, 1]
        energy_passthrough_filtered = passthrough_filtered_data[:, 2]

        # 创建图形，设置为左右两图
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 6))

        # 第一个子图：原始数据
        scatter1 = ax1.scatter(x_data, y_data, c=energy_data, cmap="viridis", alpha=0.7)
        ax1.set_title("Original Lidar Data")
        ax1.set_xlabel("X")
        ax1.set_ylabel("Y")
        fig.colorbar(scatter1, ax=ax1, label="Energy Value")  # 显示颜色条

        # 第二个子图：过滤能量值后的数据
        scatter2 = ax2.scatter(x_passthrough_filtered, y_passthrough_filtered, c=energy_passthrough_filtered, cmap="viridis", alpha=0.7)
        ax2.set_title("PassThrough Energy Data")
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        fig.colorbar(scatter2, ax=ax2, label="Energy Value")  # 显示颜色条

        # 第三个子图：直通过滤后的数据
        scatter3 = ax3.scatter(x_energy_filtered, y_energy_filtered, c=energy_filtered, cmap="viridis", alpha=0.7)
        ax3.set_title("AxisSorting_yx Filtered Data")
        ax3.set_xlabel("X")
        ax3.set_ylabel("Y")
        fig.colorbar(scatter3, ax=ax3, label="Energy Value")  # 显示颜色条

        # 调整布局
        plt.tight_layout()

        # 显示图形
        plt.show()

    def _validate_range(self, range_input):
        """增强参数兼容性：支持单值自动转范围"""
        if range_input is None:
            return None
        try:
            # 情况1：已经是范围元组/列表
            if isinstance(range_input, (list, tuple)) and len(range_input) == 2:
                return (float(range_input[0]), float(range_input[1]))
            # 情况2：单值（自动转为0到该值的范围）
            elif isinstance(range_input, (int, float)):
                return (0.0, float(range_input))
        except Exception:
            pass
        raise ValueError(f"无效的范围参数格式: {range_input}. 应为 [min, max] 或单个数值")

    def _remap_coordinates(self, data):
        """执行坐标轴重映射"""
        if data.size == 0:
            return data

        remapped = np.empty_like(data)
        # 新X轴 = 原Z取反（向左）
        remapped[:, 0] = -data[:, 2]
        # 新Y轴 = 原X
        remapped[:, 1] = data[:, 0]
        # 新Z轴 = 原Y
        remapped[:, 2] = data[:, 1]
        remapped[:, 3] = data[:, 3]  # 保留能量值
        return remapped

    def close(self):
        if self.fig is not None and plt.fignum_exists(self.fig.number):
            plt.close(self.fig)
        self.fig = None

    def draw_point_cloud(self, points):
        """
        绘制原始点云
        :param points: 点云数据
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd], window_name="原始点云")  # type: ignore

    def draw_point_cloud_color(self, points):
        """
        绘制原始点云，根据Z轴数值渲染颜色
        :param points: 点云数据，shape为(n, 3)
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        # 获取Z轴数值
        z_values = points[:, 0]
        # 归一化Z值到[0, 1]范围
        z_min, z_max = z_values.min(), z_values.max()
        if z_max > z_min:  # 避免除零
            z_normalized = (z_values - z_min) / (z_max - z_min)
        else:
            z_normalized = np.zeros_like(z_values)
        # 使用matplotlib的颜色映射（例如viridis, jet, hot等）
        colormap = cm.viridis  # type: ignore # 可以改为其他colormap如：cm.jet, cm.hot, cm.plasma等
        # 将归一化的Z值映射到颜色
        colors = colormap(z_normalized)[:, :3]  # 只取RGB，忽略Alpha通道
        # 设置点云颜色
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd], window_name="原始点云（Z轴颜色渲染）")  # type: ignore

    def _create_box_geometries(self, boxes_data):
        """生成方块几何体"""
        geometries = []

        # Jig数据（红色）
        for jig in boxes_data["jig_data"]:
            center = [
                (jig["x_max"] + jig["x_min"]) / 2,
                (jig["y_max"] + jig["y_min"]) / 2,
                (jig["z_end"] + jig["z_start"]) / 2
            ]
            extent = [
                jig["x_max"] - jig["x_min"],
                jig["y_max"] - jig["y_min"],
                jig["z_end"] - jig["z_start"]
            ]
            bbox = o3d.geometry.OrientedBoundingBox(center, np.eye(3), extent)
            bbox.color = (1, 0, 0)
            geometries.append(bbox)

        # 外侧数据（蓝色）
        out = boxes_data["outside_data"]
        if out["x_max"] > -np.inf:
            center = [
                (out["x_max"] + out["x_min"]) / 2,
                (out["y_max"] + out["y_min"]) / 2,
                (out["z_max"] + out["z_min"]) / 2
            ]
            extent = [
                out["x_max"] - out["x_min"],
                out["y_max"] - out["y_min"],
                out["z_max"] - out["z_min"]
            ]
            bbox = o3d.geometry.OrientedBoundingBox(center, np.eye(3), extent)
            bbox.color = (0, 0, 1)
            geometries.append(bbox)

        # 内侧数据（绿色）
        for part in boxes_data["inside_data"]:
            center = [
                (part["x_max"] + part["x_min"]) / 2,
                (part["y_max"] + part["y_min"]) / 2,
                (part["z_end"] + part["z_start"]) / 2
            ]
            extent = [
                part["x_max"] - part["x_min"],
                part["y_max"] - part["y_min"],
                part["z_end"] - part["z_start"]
            ]
            bbox = o3d.geometry.OrientedBoundingBox(center, np.eye(3), extent)
            bbox.color = (0, 1, 0)
            geometries.append(bbox)

        return geometries

    def plot_layer(self, layer_points, layer_index=0):
        """
        可视化指定层的点云
        Args:
            layer_points (list): 分层后的点云列表，每个元素可能是多个数组的列表
            layer_index (int): 要可视化的层索引
        """
        if not layer_points or layer_index >= len(layer_points):
            print(f"警告：无效的层索引 {layer_index} 或空点云列表")
            return

        # 合并该层所有数组为一个数组
        target_layer = np.vstack(layer_points[layer_index]) if isinstance(layer_points[layer_index], list) else layer_points[layer_index]

        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection="3d")

        # 提取坐标
        x = target_layer[:, 0]
        y = target_layer[:, 1]
        z = target_layer[:, 2]

        # 绘制散点图
        ax.scatter(x, y, z, c="r", marker="o", s=10)  # type: ignore
        ax.set_title(f"Layer {layer_index} (Points: {len(target_layer)})")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")  # type: ignore
        plt.show()

    def plot_multiple_layers(self, layer_points, end_layer_index=6, start_layer_index=0):
        """
        可视化多层点云（从start_layer_index到end_layer_index）
        Args:
            layer_points (list): 分层后的点云列表
            end_layer_index (int): 结束层索引（包含）
            start_layer_index (int): 起始层索引（默认为0）
        """
        if not layer_points or end_layer_index >= len(layer_points):
            print(f"警告：无效的层索引范围 {start_layer_index}-{end_layer_index}")
            return

        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")

        # 定义颜色映射
        colors = plt.cm.viridis(np.linspace(0, 1, end_layer_index - start_layer_index + 1))  # type: ignore

        total_points = 0
        for i in range(start_layer_index, end_layer_index + 1):
            # 合并当前层的所有子数组
            current_layer = np.vstack(layer_points[i]) if isinstance(layer_points[i], list) else layer_points[i]
            total_points += len(current_layer)

            # 绘制当前层（使用不同颜色区分）
            ax.scatter(current_layer[:, 0], current_layer[:, 1], current_layer[:, 2], c=[colors[i - start_layer_index]],  # type: ignore
                       label=f"Layer {i}", s=8, alpha=0.7)  # type: ignore  # 每层不同颜色

        ax.set_title(f"Layers {start_layer_index}-{end_layer_index} (Total Points: {total_points})")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")  # type: ignore
        plt.legend()
        plt.tight_layout()
        plt.show()

    def draw_combined(self, points, boxes_data=None):
        """非阻塞方式绘制点云（可选的带方块数据）"""
        with self.lock:
            # 如果窗口已关闭，重置状态准备重新打开
            if self.window_closed:
                self.shutdown_flag.clear()  # 重置关闭标志
                self.window_closed = False  # 标记窗口为未关闭状态
                logger.info("Preparing to reopen visualization window")

            # 创建点云对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # 设置点云颜色：有boxes_data用灰色，没有则用x轴高度彩色
            if boxes_data is None:
                # 彩色模式 - 基于x轴高度
                x_values = points[:, 0]  # 使用x轴高度作为颜色映射
                x_min, x_max = np.min(x_values), np.max(x_values)
                if x_max - x_min > 1e-6:  # 避免除以零
                    x_normalized = (x_values - x_min) / (x_max - x_min)
                    colors = plt.cm.viridis(x_normalized)[:, :3]  # type: ignore
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                else:
                    pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 所有点高度相同时使用灰色
            else:
                # 灰色模式（与方块数据一起显示）
                pcd.paint_uniform_color([0.5, 0.5, 0.5])

            # 准备几何体列表
            geometries = [pcd]

            # 如果有方块数据，添加到几何体列表
            if boxes_data is not None:
                box_geometries = self._create_box_geometries(boxes_data)
                geometries.extend(box_geometries)

            # 将几何体放入队列（如果队列已满则替换旧数据）
            if self.geometry_queue.full():
                try:
                    self.geometry_queue.get_nowait()
                except Exception as e:
                    logger.error(f"Error geometry_queue get_nowait: {str(e)}")
            self.geometry_queue.put(geometries)

            # 如果可视化线程不存在或已停止，启动新线程
            if self.visualization_thread is None or not self.visualization_thread.is_alive():
                self.visualization_thread = threading.Thread(
                    target=self._visualization_loop,
                    daemon=True
                )
                self.visualization_thread.start()
                logger.info("Visualization thread started")

    def _visualization_loop(self):
        """可视化主循环"""
        try:
            logger.info("Opening visualization window")
            vis = o3d.visualization.Visualizer()  # type: ignore
            vis.create_window(window_name="3D Point Cloud Viewer", width=1024, height=768)

            # 初始视角设置
            ctr = vis.get_view_control()
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, 1, 0])

            while not self.shutdown_flag.is_set():
                try:
                    # 尝试获取新几何体
                    new_geometries = self.geometry_queue.get_nowait()
                    vis.clear_geometries()

                    # 添加新几何体
                    for geom in new_geometries:
                        try:
                            vis.add_geometry(geom)
                        except Exception as e:
                            logger.error(f"Error adding geometry: {str(e)}")
                except Empty:
                    pass  # 队列为空是正常情况

                try:
                    # 处理窗口事件
                    if not vis.poll_events():
                        logger.info("Visualization window closed by user")
                        break
                    vis.update_renderer()
                except Exception as e:
                    logger.error(f"Rendering error: {str(e)}")
                    break

                # 控制刷新率
                time.sleep(0.05)

        except Exception as e:
            logger.error(f"Fatal error in visualization loop: {str(e)}")
            import traceback
            logger.error(traceback.format_exc())

        finally:
            # 清理资源
            try:
                vis.destroy_window()
                logger.info("Visualization window closed")
            except Exception as e:
                logger.error(f"error destroy_window: {str(e)}")

            with self.lock:
                self.window_closed = True
                self.visualization_thread = None

    def close_visualization(self):
        """安全关闭可视化窗口"""
        with self.lock:
            self.shutdown_flag.set()
            if self.visualization_thread and self.visualization_thread.is_alive():
                try:
                    self.visualization_thread.join(timeout=1.0)
                except Exception as e:
                    logger.error(f"error visualization_thread join: {str(e)}")
            self.visualization_thread = None
            try:
                while not self.geometry_queue.empty():
                    self.geometry_queue.get_nowait()
            except Exception as e:
                logger.error(f"error geometry_queue get_nowait: {str(e)}")
            self.window_closed = True

    def __del__(self):
        """析构时确保关闭可视化"""
        self.close_visualization()


if __name__ == "__main__":
    visualizer = PointCloudVisualizer(3000, 4800, 16000)
    # data_paths = r"D:\ReadDataProcess\data\points\20250331_111130.txt"
    data_paths = r"D:\ReadDataProcess\data\points\20250331_114009.txt"
    data = np.loadtxt(data_paths)
    point_cloud = data[:, :3]
    print("point_cloud:", len(point_cloud))
    visualizer.draw_point_cloud(point_cloud)  # 绘制原始点云
