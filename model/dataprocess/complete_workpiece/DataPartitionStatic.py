import numpy as np


class DataPartitionStatic:
    '''
    固定数据分区处理模块
    '''
    def find_boundary_layers_static(self, y_groups, upper_y, lower_y, x_middle, last_surface_edge):
        """
        找到每帧的最上边缘和最下边缘xyz坐标, 以及进行分区检测
        """
        # lower_layer, upper_layer = self._find_boundary_edges(y_groups, lower_y, upper_y)

        # if upper_layer is None or lower_layer is None:
        #     raise ValueError("upper_layer or lower_layer is None, cannot generate partition bounds.")
        y_bounds = self._generate_partition_bounds(upper_y, lower_y)
        partitions = []
        x_max = 0.0
        x_min = float('inf')
        # 遍历每个分区计算参数
        for partition_id in range(6):
            start_y, end_y = y_bounds[partition_id], y_bounds[partition_id+1]
            partition_layers = self._get_partition_layers(y_groups, start_y, end_y)
            prev_state = last_surface_edge[partition_id]
            partition_params = self._calculate_partition_params(
                partition_layers, x_middle, partition_id, prev_state
            )
            x_max = max(x_max, partition_params["x_max"])
            x_min = min(x_min, partition_params["x_min"])
            partitions.append(partition_params)
        merge_partitions = self._merge_partition(x_middle, partitions)

        return (x_max, x_min, partitions, merge_partitions)

    def _find_boundary_edges(self, y_groups, lower_y, upper_y):
        """查找lower_layer和upper_layer"""
        lower_layer = upper_layer = None
        for i, layer in enumerate(reversed(y_groups)):
            if len(layer) == 0:
                continue
            min_y_value = np.min(layer[:, 1])
            max_y_value = np.max(layer[:, 1])

            # 查找 lower_layer（第一个差值<10mm的分组）
            if lower_layer is None:
                min_y_index = np.argmin(layer[:, 1])
                if i == len(y_groups) - 1 and min_y_value > lower_y:
                    lower_layer = layer[min_y_index][1]
                elif 0 < min_y_value - lower_y < 15.0:
                    lower_layer = layer[min_y_index][1]

            # 查找 upper_layer（最后一个差值<10mm的分组）
            if upper_layer is None:
                max_y_index = np.argmax(layer[:, 1])
                if i == 0 and max_y_value < upper_y:
                    upper_layer = layer[max_y_index][1]
                elif 0 < upper_y - max_y_value < 15.0:
                    upper_layer = layer[max_y_index][1]
        if lower_layer is None:
            lower_layer = lower_y
        if upper_layer is None:
            upper_layer = upper_y
        return lower_layer, upper_layer

    def _generate_partition_bounds(self, upper_y, lower_y):
        """生成6个分区的y边界"""
        return np.linspace(upper_y, lower_y, 7)

    def _get_partition_layers(self, y_groups, start_y, end_y):
        """收集属于当前分区的层数据"""
        partition_layers = []
        for layer in y_groups:
            if len(layer) == 0:
                continue
            layer_y = layer[0, 1]
            if end_y <= layer_y <= start_y:
                partition_layers.append(layer)

        return partition_layers

    def _calculate_partition_params(self, layers, x_middle, partition_id, last_surface_edge):
        """计算单个分区的参数"""
        params = {
            "partition_id": partition_id,
            "surface_edge": 0,
            "x_min": float('inf'),
            "x_max": 0.0,
            "up_edge_y": 0.0,
            "up_edge_x_max": 0.0,
            "up_edge_x_min": float('inf'),
            "do_edge_y": float('inf'),
            "do_edge_x_max": 0.0,
            "do_edge_x_min": float('inf'),
            "odd_surface_edge": 0,
            "even_surface_edge": 0
        }

        less_count = 0  # 统计x < x_middle的点数
        has_less_points = False  # 标记是否存在x < x_middle的点
        candidate_less_points = []  # 收集所有x < x_middle的点用于边界计算

        for layer in layers:
            if len(layer) == 0:
                continue
            x_values = layer[:, 0]
            # 更新x_min及x_max
            params["x_min"] = min(params["x_min"], np.min(x_values))
            params["x_max"] = max(params["x_max"], np.max(x_values))
            # 统计小于x_middle的点数
            layer_less_mask = x_values < x_middle
            less_count += np.sum(layer_less_mask)
            # 收集x < x_middle的点用于边界计算
            if np.any(layer_less_mask):
                has_less_points = True
                candidate_less_points.append(layer[layer_less_mask])

        if has_less_points:
            self._set_edges_with_less_points(candidate_less_points, params)
        else:
            self._set_edges_without_less_points(layers, params)

        params["surface_edge"] = 1 if (params["up_edge_x_min"] < x_middle and params["do_edge_x_min"] < x_middle) else 0

        # 根据前序状态设置标志位
        self._determine_edge_flags(params, last_surface_edge)
        return params

    def _set_edges_with_less_points(self, candidate_less_points, params):
        """处理存在x<x_middle点的情况，直接遍历候选点列表"""
        # 初始化极值
        max_y = -float('inf')
        min_y = float('inf')
        max_y_x_list = []
        min_y_x_list = []

        # 遍历所有层中的点
        for layer in candidate_less_points:
            if len(layer) == 0:
                continue
            # 提取当前层所有点的Y值
            y_values = layer[:, 1]
            # 更新最大Y及其X值
            current_max_y = np.max(y_values)
            if current_max_y > max_y:
                max_y = current_max_y
                max_y_x_list = layer[:, 0].tolist()
            elif current_max_y == max_y:
                max_y_x_list.extend(layer[:, 0].tolist())
            # 更新最小Y及其X值
            current_min_y = np.min(y_values)
            if current_min_y < min_y:
                min_y = current_min_y
                min_y_x_list = layer[:, 0].tolist()
            elif current_min_y == min_y:
                min_y_x_list.extend(layer[:, 0].tolist())

        # 设置参数
        params.update({
            "up_edge_y": max_y if max_y != -float('inf') else 0.0,
            "up_edge_x_max": max(max_y_x_list) if max_y_x_list else 0.0,
            "up_edge_x_min": min(max_y_x_list) if max_y_x_list else 0.0,
            "do_edge_y": min_y if min_y != float('inf') else 0.0,
            "do_edge_x_max": max(min_y_x_list) if min_y_x_list else 0.0,
            "do_edge_x_min": min(min_y_x_list) if min_y_x_list else 0.0,
        })

    def _set_edges_without_less_points(self, layers, params):
        """处理无x<x_middle点的情况"""
        if not layers:
            return
        max_y = -float('inf')
        min_y = float('inf')
        max_y_layer = None
        min_y_layer = None
        for layer in layers:
            if len(layer) == 0:
                continue
            layer_y = layer[0, 1]  # 假设层已按y分组，取第一个点的y值
            if layer_y > max_y:
                max_y = layer_y
                max_y_layer = layer
            if layer_y < min_y:
                min_y = layer_y
                min_y_layer = layer
        # 设置边界
        if max_y_layer is not None:
            params["up_edge_y"] = max_y
            params["up_edge_x_max"] = np.max(max_y_layer[:, 0])
            params["up_edge_x_min"] = np.min(max_y_layer[:, 0])
        if min_y_layer is not None:
            params["do_edge_y"] = min_y
            params["do_edge_x_max"] = np.max(min_y_layer[:, 0])
            params["do_edge_x_min"] = np.min(min_y_layer[:, 0])

    def _determine_edge_flags(self, params, last_surface_edge):
        """根据前序状态设置odd/even标志"""
        current = params["surface_edge"]
        if last_surface_edge == 0 and current == 1:
            params["odd_surface_edge"] = 1
        elif last_surface_edge == 1 and current == 0:
            params["even_surface_edge"] = 2

    def _merge_partition(self, x_middle, partitions):
        '''
        找每个分区的x_min，如果x_min小于x_middle则继续寻找下一个分区，如果x_min大于x_middle，则记录上一个分区的do_edge_y为merge_ymax，为合并分区的开始。
        当ymax有数值则继续下一个分区寻找x_min小于x_middle的分区，记录x_min小于x_middle的分区的up_edge_y为merge_ymin，为合并分区的结束。
        找到合并分区的x_min大于x_middle对应的x_min，并保留最小的x_min为merge_x_max
        所有分区的x_min的最小值为merge_x_min
        以上为合并分区merge_partition_id的merge_ymax，merge_ymin，merge_x_max，merge_x_min
        再继续往下寻找合并分区的开始和结束以及数值，如果所有分区都存在x_min小于x_middle则合并分区所有都为0
        '''
        merge_partitions = []
        current_merge = None
        for i in range(6):
            if i == 0:
                continue
            curr_part = partitions[i]
            if curr_part["x_min"] > x_middle and curr_part["x_min"] != float('inf'):
                # 开始或继续合并分区
                if current_merge is None:
                    if partitions[i-1]["do_edge_y"] == float('inf'):
                        cur_y = curr_part["up_edge_y"]
                    else:
                        cur_y = partitions[i-1]["do_edge_y"]
                    current_merge = {
                        "merge_partition_id": len(merge_partitions),
                        "merge_partition_y_max": cur_y,
                        "merge_partition_y_min": float('inf'),
                        "merge_partition_x_max": curr_part["x_min"],
                        "merge_partition_x_min": partitions[i-1]["x_min"],
                        "surface_edge": 0,
                        "merge_partition_surface_edge": 0
                    }
                else:
                    # 更新当前合并分区的x范围
                    current_merge["merge_partition_x_max"] = min(current_merge["merge_partition_x_max"], curr_part["x_min"])
                    current_merge["merge_partition_x_min"] = min(current_merge["merge_partition_x_min"], curr_part["x_min"])
            else:
                # 结束当前合并分区
                if current_merge is not None:
                    current_merge["merge_partition_y_min"] = curr_part["up_edge_y"]
                    current_merge["merge_partition_x_min"] = min(current_merge["merge_partition_x_min"], curr_part["x_min"])

                    merge_partitions.append(current_merge)
                    current_merge = None

        # 处理最后未结束的合并分区
        if current_merge is not None:
            current_merge["merge_partition_y_min"] = partitions[-1]["do_edge_y"] if partitions else 0.0
            merge_partitions.append(current_merge)

        return merge_partitions
