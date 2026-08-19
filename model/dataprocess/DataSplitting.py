import numpy as np


class DataSplitting:
    """
    点云数据切割处理模块
    """

    def energy_filter_data(self, data, energy_percentage):
        # 计算所有点的能量值的平均值
        energy_values = data[:, 2]  # 取出第三列的能量值
        avg_energy = np.mean(energy_values)  # 计算平均能量值
        # 设置过滤条件：能量值小于平均能量值的energy_percentage
        threshold = avg_energy * energy_percentage
        # 过滤数据：保留能量值大于等于阈值的点
        filtered_data = data[energy_values >= threshold]
        return filtered_data

    def AxisSorting_yx(self, points, y_threshold=10):
        """
        Y轴分层二维排序（Y轴从小到大，层内按X→Z排序）
        返回结构：按Y轴分组的点云列表，每层为np.ndarray
        """
        # 确保是numpy数组
        if isinstance(points, list):
            points = np.array(points)

        # 处理空数据
        if len(points) == 0 or points.size == 0:
            return np.empty((0, 3))

        # 确保至少有3列 (x, y, z)，不足则补零
        if points.ndim == 1:
            points = points.reshape(1, -1)
        if points.shape[1] < 3:
            padding = np.zeros((points.shape[0], 3 - points.shape[1]))
            points = np.hstack([points, padding])

        point_list = []
        for i in range(points.shape[0]):
            arr = [float(points[i, 0]), float(points[i, 1]), float(points[i, 2])]  # x, y, z
            point_list.append(arr)

        sorted_points = sorted(point_list, key=lambda p: p[1])

        y_groups = []
        if sorted_points:
            current_group = [sorted_points[0]]
            base_y = sorted_points[0][1]

            for i in range(1, len(sorted_points)):
                current_y = sorted_points[i][1]

                if abs(current_y - base_y) <= y_threshold:
                    current_group.append(sorted_points[i])
                else:
                    y_groups.append(current_group)
                    current_group = [sorted_points[i]]
                    base_y = current_y

            y_groups.append(current_group)

        result = []
        for group in y_groups:
            sorted_group = sorted(group, key=lambda p: p[0])
            result.extend([[p[0], p[1], p[2]] for p in sorted_group])  # 保持数据

        # 确保返回3D数组，不是2D数组
        if not result:
            return np.empty((0, 3))
        return np.array(result)

    def AxisSorting_yxz(self, points, y_threshold=10):
        """
        Y轴分层二维排序（Y轴从小到大，层内按X→Z排序）
        返回结构：按Y轴分组的点云列表，每层为np.ndarray
        """
        # 确保是numpy数组
        if isinstance(points, list):
            points = np.array(points)

        # 确保至少有3列 (x, y, z)，不足则补零
        if points.ndim == 1:
            points = points.reshape(1, -1)
        if points.shape[1] < 3:
            padding = np.zeros((points.shape[0], 3 - points.shape[1]))
            points = np.hstack([points, padding])

        point_list = [[float(p[0]), float(p[1]), float(p[2])] for p in points]

        # 按 Y 排序
        sorted_points = sorted(point_list, key=lambda p: p[1])

        y_groups = []
        if sorted_points:
            current_group = [sorted_points[0]]
            base_y = sorted_points[0][1]

            for i in range(1, len(sorted_points)):
                current_y = sorted_points[i][1]

                # 是否与当前层的基准 Y 足够接近
                if abs(current_y - base_y) <= y_threshold:
                    current_group.append(sorted_points[i])
                else:
                    y_groups.append(current_group)
                    current_group = [sorted_points[i]]
                    base_y = current_y

            y_groups.append(current_group)

        # 每层内再按 X 排序，并转为 np.ndarray
        result = []
        for group in y_groups:
            sorted_group = sorted(group, key=lambda p: (p[0], p[2]))  # 先按X，再按Z
            result.append(np.array(sorted_group, dtype=float))

        return result

    def normalize_xyz_points(self, points, z_reset_threshold=160000):
        """
        将点云数据的 Z 轴归一化（减去最小值，从 0 开始）
        :param points: 三维点云数据
        :param z_reset_threshold: Z 轴重置阈值（用于修正 Z 轴跳变）
        :return: 归一化后的点云数据
        """
        # 处理空列表
        if isinstance(points, list):
            # 过滤空数组
            points = [p for p in points if isinstance(p, np.ndarray) and len(p) > 0]
            if not points:
                return np.empty((0, 3))
            points = np.vstack(points)

        if len(points) == 0:
            return points.copy() if isinstance(points, np.ndarray) else np.empty((0, 3))

        # 确保至少有3列 (x, y, z)，否则填补
        if points.shape[1] < 3:
            z_col = np.zeros((points.shape[0], 3 - points.shape[1]))
            points = np.hstack([points, z_col])

        # 提取 Z 轴数据
        z_data = points[:, 2].copy()
        corrected_z = []
        offset = 0

        # 修正 Z 轴跳变
        for i in range(len(z_data)):
            if i > 0 and z_data[i] < z_data[i - 1] and z_data[i - 1] >= z_reset_threshold * 0.9:
                offset += z_reset_threshold
            corrected_z.append(z_data[i] + offset)

        # 归一化 Z 轴（减去最小值，从 0 开始）
        corrected_z = np.array(corrected_z)
        min_z = np.min(corrected_z)
        normalized_z = corrected_z - min_z

        # 组合结果（保持 XY 不变）
        normalized_points = points.copy()
        normalized_points[:, 2] = normalized_z

        return normalized_points

    def AxisSorting_zyx(self, points, z_threshold=10, y_threshold=10):
        """
        Z-Y-X多级分层排序
        返回结构：List[List[np.ndarray]]
          - 外层列表：Z层（按Z升序排列）
          - 中层列表：每个Z层内的Y层（按Y升序排列）
          - 内层数组：每个Y层内的点按X升序排列
        """
        if not isinstance(points, np.ndarray) or points.shape[1] != 3:
            raise ValueError("输入点云必须是形状为(N,3)的numpy数组")

        # 按Z升序排序
        z_sorted = points[points[:, 2].argsort()]
        # 计算Z层索引
        z_bins = (z_sorted[:, 2] // z_threshold).astype(int)
        z_layers = [z_sorted[z_bins == layer] for layer in np.unique(z_bins)]

        # 在每个Z层内处理Y-X
        full_structure = []
        for z_layer in z_layers:
            if z_layer.size == 0:
                continue

            # 按Y升序排序
            y_sorted = z_layer[z_layer[:, 1].argsort()]
            # 计算Y层索引
            y_bins = (y_sorted[:, 1] // y_threshold).astype(int)
            y_layers = []

            # 按Y层分组并排序X
            for y_bin in np.unique(y_bins):
                y_subset = y_sorted[y_bins == y_bin]
                # 按X升序排序
                x_sorted = y_subset[y_subset[:, 0].argsort()]
                y_layers.append(x_sorted)

            full_structure.append(y_layers)

        return full_structure

    def AxisSorting_yzx(self, points, y_threshold=10, z_threshold=10):
        """
        Y-Z-X多级分层排序
        返回结构：List[List[np.ndarray]]
        - 外层列表：Y层（按Y升序排列）
        - 中层列表：每个Y层内的Z层（按Z升序排列）
        - 内层数组：每个Z层内的点按X升序排列
        """
        if not isinstance(points, np.ndarray) or points.shape[1] != 3:
            raise ValueError("输入必须是形状为(N,3)的numpy数组")

        # 按Y升序排序
        y_sorted = points[points[:, 1].argsort()]
        # 计算Y层索引
        y_bins = (y_sorted[:, 1] // y_threshold).astype(int)
        y_layers = [y_sorted[y_bins == layer] for layer in np.unique(y_bins)]

        # 在每个Y层内处理Z-X
        full_structure = []
        for y_layer in y_layers:
            if y_layer.size == 0:
                continue

            # 按Z升序排序
            z_sorted = y_layer[y_layer[:, 2].argsort()]
            # 计算Z层索引
            z_bins = (z_sorted[:, 2] // z_threshold).astype(int)
            z_layers = []

            # 按Z层分组并排序X
            for z_bin in np.unique(z_bins):
                z_subset = z_sorted[z_bins == z_bin]
                # 按X升序排序
                x_sorted = z_subset[z_subset[:, 0].argsort()]
                z_layers.append(x_sorted)

            full_structure.append(z_layers)

        return full_structure
