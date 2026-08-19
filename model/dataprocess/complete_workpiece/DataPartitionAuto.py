import numpy as np


class DataPartitionAuto:
    '''
    自动化分区处理模块
    '''
    def find_boundary_layers_auto(self, y_groups, upper_y, lower_y, x_threshold, last_surface_edge, outside_x_max, outside_x_min, inside_partition_min_y_range=50):
        """
        自动进行分区检测:
        循环遍历每一组y_groups，当所有组的layer中所有x点的最小值小于x_threshold则只返回当前layer的x_max和x_min，merge_partitions=[]
        如果当这一组的y_groups，循环layer直到最小x大于x_threshold的时候则从当前的组为开始，记录start_y = np.min(layer[:, 1])和x_max=np.min(layer[:, 0]),
        有了开始，还没有结束的过程中刷新x_max=min(x_max,np.min(layer[:, 0]))
        开始后直到找到最小x小于x_threshold的时候则为结束，记录end_y = np.max(layer[:, 1])和刷新x_max=min(x_max,np.min(layer[:, 0]))
        判断该开始到结束的分区start_y-end_y是否大于100，是则merge_partitions.append，否则舍弃
        最后返回return (x_max, x_min, None, merge_partitions)
        """
        reversed_y_groups = list(reversed(y_groups))  # 创建逆序数组副本
        self.x_threshold = x_threshold
        self.inside_partition_min_y_range = inside_partition_min_y_range
        x_max = 0.0
        x_min = float('inf')
        merge_partitions = []
        current_partition = None

        # lower_layer, upper_layer = self._find_boundary_edges(y_groups, lower_y, upper_y)

        # if upper_layer is None or lower_layer is None:
        #     raise ValueError("upper_layer or lower_layer is None, cannot generate partition bounds.")

        for i, layer in enumerate(reversed_y_groups):
            if len(layer) == 0:
                continue

            current_min_x = np.min(layer[:, 0])
            current_max_x = np.max(layer[:, 0])
            current_min_y = np.min(layer[:, 1])
            current_max_y = np.max(layer[:, 1])
            idx_temp = len(y_groups)-i-2
            if idx_temp > 0:
                idx = idx_temp
            else:
                idx = 0
            next_max_y = np.max(y_groups[idx][:, 1])
            x_max = max(x_max, current_max_x)
            x_min = min(x_min, current_min_x)

            if current_max_y > upper_y or current_min_y < lower_y:
                continue

            if current_min_x >= x_threshold:
                if current_partition is None:
                    current_partition = self._create_partition(
                        start_index=i,
                        start_y=current_min_y,
                        end_y=current_max_y,
                        x_max=current_min_x,
                        is_no_back=False,
                    )
                else:
                    if current_partition.get("is_no_back", False):
                        current_partition["end_index"] = max(i - 1, current_partition.get("start_index", i))
                        merge_partitions = self._finalize_current_partition(x_min, current_partition, merge_partitions, reversed_y_groups)
                        current_partition = self._create_partition(
                            start_index=i,
                            start_y=current_min_y,
                            end_y=current_max_y,
                            x_max=current_min_x,
                            is_no_back=False,
                        )
                    else:
                        # 更新分区信息
                        current_partition["end_y"] = current_max_y
                        current_partition["x_max"] = min(current_partition["x_max"], current_min_x)

            if current_min_y - next_max_y > 200:  # 考虑背板是镂空
                if current_partition is None:
                    current_partition = self._create_partition(
                        start_index=i,
                        start_y=current_min_y,
                        end_y=next_max_y,
                        x_max=outside_x_max,
                        is_no_back=True,
                    )
                else:
                    if current_partition.get("is_no_back", False):
                        current_partition["end_y"] = next_max_y
                        current_partition["x_max"] = outside_x_max
                    else:
                        current_partition["end_index"] = max(i - 1, current_partition.get("start_index", i))
                        merge_partitions = self._finalize_current_partition(x_min, current_partition, merge_partitions, reversed_y_groups)
                        current_partition = self._create_partition(
                            start_index=i,
                            start_y=current_min_y,
                            end_y=next_max_y,
                            x_max=outside_x_max,
                            is_no_back=True,
                        )

            if current_min_x < x_threshold and current_min_y - next_max_y <= 200:
                if current_partition is not None:
                    current_partition["end_index"] = i
                    merge_partitions = self._finalize_current_partition(x_min, current_partition, merge_partitions, reversed_y_groups)
                    current_partition = None

        # 最后检查是否有未完成的分区
        if current_partition is not None:
            # 设置结束索引为最后一层
            current_partition["end_index"] = len(reversed_y_groups) - 1
            merge_partitions = self._finalize_current_partition(x_min, current_partition, merge_partitions, reversed_y_groups)
        if x_max == 0.0:
            x_max = outside_x_max
        if x_min == float('inf'):
            x_min = outside_x_min
        return (x_max, x_min, None, merge_partitions)

    def _create_partition(self, start_index, start_y, end_y, x_max, is_no_back):
        return {
            "start_index": start_index,
            "start_y": start_y,
            "end_y": end_y,
            "x_max": x_max,
            "is_no_back": is_no_back,
        }

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

    def _finalize_current_partition(self, x_min, current_partition, merge_partitions, reversed_y_groups):
        """结束当前分区并添加到列表（增加边界调整）"""
        # 计算调整后的边界
        start_index = current_partition.get("start_index", 0)
        end_index = current_partition.get("end_index", 0)

        if start_index > len(reversed_y_groups) - 2 or end_index < 2:
            # 如果分区范围过小，直接返回空列表
            return merge_partitions
        if end_index - start_index < 4:  # 为背板镂空情况
            partition_height = current_partition["start_y"] - current_partition["end_y"]
            if partition_height > self.inside_partition_min_y_range:
                merge_partitions.append({
                    "merge_partition_id": len(merge_partitions),
                    "merge_partition_x_min": x_min,
                    "merge_partition_x_max": current_partition["x_max"],
                    "merge_partition_y_max": current_partition["start_y"],
                    "merge_partition_y_min": current_partition["end_y"],
                    "merge_partition_surface_edge": 0
                })
            return merge_partitions
        else:
            # 上边界下移两帧（逆序数组中索引增加）
            adj_start_index = max(start_index + 2, 0)
            adjusted_start_y = np.max(reversed_y_groups[adj_start_index][:, 1])

            # 下边界上移两帧（逆序数组中索引减少）
            adj_end_index = min(end_index - 2, len(reversed_y_groups) - 1)
            adjusted_end_y = np.min(reversed_y_groups[adj_end_index][:, 1])

            partition_x_max = float('inf')
            for idx in range(adj_start_index, adj_end_index + 1):
                if len(reversed_y_groups[idx]) > 0:
                    layer_min_x = np.min(reversed_y_groups[idx][:, 0])
                    partition_x_max = min(partition_x_max, layer_min_x)

            # 计算调整后的分区高度
            partition_height = adjusted_start_y - adjusted_end_y
            if partition_height > self.inside_partition_min_y_range:
                merge_partitions.append({
                    "merge_partition_id": len(merge_partitions),
                    "merge_partition_x_min": x_min,
                    "merge_partition_x_max": partition_x_max,
                    "merge_partition_y_max": adjusted_start_y,  # 使用调整后的上边界
                    "merge_partition_y_min": adjusted_end_y,    # 使用调整后的下边界
                    "merge_partition_surface_edge": 0
                })
            return merge_partitions
