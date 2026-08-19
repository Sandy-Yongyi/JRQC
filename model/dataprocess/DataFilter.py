import numpy as np
import open3d as o3d


class DataFilter:
    '''
    点云滤波处理模块
    '''
    def points_to_pcd(self, points):
        '''
        将三维数组转成点云
        :param points: 三维点云数据(N, 3), 其中N是点数
        :return: 点云对象
        '''
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    def remove_statistical_outliers(self, points, nb_neighbors=10, std_ratio=5.0):
        '''
        统计滤波
        :param points: 三维点云数据
        :param nb_neighbors: 每个点周围的邻点数量
        :param std_ratio: 标准差比率
        :return: 滤波后的点云数据
        '''
        pcd = self.points_to_pcd(points)
        cl, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        filtered_pcd = pcd.select_by_index(ind)
        return np.asarray(filtered_pcd.points)

    def PassThroughFilter(self, points, x_min, x_max, y_min, y_max):
        '''
        直通滤波
        :param points: 三维点云数据
        :param x_min: x最小值
        :param x_max: x最大值
        '''
        if points is None:
            return np.empty((0, 2), dtype=np.float64)

        points = np.asarray(points)
        if points.size == 0:
            return np.empty((0, 2), dtype=np.float64)

        if points.ndim == 1:
            points = points.reshape(1, -1)

        if points.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float64)

        xy_points = points[:, :2]
        mask = (
            (xy_points[:, 0] >= x_min)
            & (xy_points[:, 0] <= x_max)
            & (xy_points[:, 1] >= y_min)
            & (xy_points[:, 1] <= y_max)
        )

        if not np.any(mask):
            return np.empty((0, 2), dtype=np.float64)

        return xy_points[mask].astype(np.float64, copy=False)

    def radiusFilter(self, points, radiusValue=40.0, radiusNearest=3):
        """
        半径滤波
        """
        if points.shape[0] > 0:
            pcd = self.points_to_pcd(points)
            cl, ind = pcd.remove_radius_outlier(nb_points=int(radiusNearest), radius=radiusValue)
            radius_cloud = pcd.select_by_index(ind)
            return np.asarray(radius_cloud.points)
        else:
            return np.empty((0, 3))

    def voxelReductionFilter(self, points, voxel_size=10.0):
        """
        体素降采
        voxel_size:表示体素大小,长宽高
        """
        if points.shape[0] > 0:
            pcd = self.points_to_pcd(points)
            downpcd = pcd.voxel_down_sample(voxel_size)
            return np.asarray(downpcd.points)
        else:
            return np.empty((0, 3))
