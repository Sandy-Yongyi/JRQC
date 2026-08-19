from collections import defaultdict
from itertools import count
import os
from typing import List
import numpy as np
from model.utils.LoggerUtil import logger
# from model.plot.draw import PointCloudVisualizer
from model.utils.MachineConfigUtil import normalize_offset_value
from model.utils.TomlLoader import TomlLoader
from model.dataprocess.DataFilter import DataFilter
from model.dataprocess.DataSplitting import DataSplitting
from model.dataprocess.complete_workpiece.DataPartitionStatic import DataPartitionStatic
from model.dataprocess.complete_workpiece.DataPartitionAuto import DataPartitionAuto
from model.formats.complete_workpiece.FrameDataFormat import MergePartitionData, PartitionData, LateralData
from model.formats.complete_workpiece.BlockDataFormat import JigData, OutsideData, InsideData, BlockData, SubInsideData


class DataFindBlocks:
    def __init__(self):
        # self.visualizer = PointCloudVisualizer()
        self.data_filter = DataFilter()
        self.data_split = DataSplitting()
        self.data_partition_static = DataPartitionStatic()
        self.data_partition_auto = DataPartitionAuto()
        self.process_config = TomlLoader.load(os.getcwd() + "\\model\\tomls\\ProcessConfig.toml")
        self.find_y_plane = self.process_config["find_y_plane"]
        # if self.process_config["draw_type"] != 0:
        #     self.visualizer = PointCloudVisualizer()
        #     self.visualizer.close_visualization = self.visualizer.close_visualization

    def start_process(self, point_cloud, lidar_status, data_dir):
        try:
            self.data_dir = data_dir
            self.lidar_status = lidar_status
            # self.filtered_data = self.data_filter.remove_statistical_outliers(points=point_cloud, nb_neighbors=30, std_ratio=0.1)  # 预编程统计滤波
            self.filtered_data = self.data_filter.remove_statistical_outliers(points=point_cloud, nb_neighbors=40, std_ratio=2.0)  # 自动化统计滤波
            logger.info(f"points after filtered data len: {len(self.filtered_data)}")
            # self.visualizer.draw_point_cloud(self.filtered_data)  # 绘制滤波后的点云
            # self.filtered_data = self.point_cloud

            # 根据z轴每间隔10mm进行切割，排序为zyx顺序，绘制切割后图像
            z_threshold = self.process_config["z_threshold"]
            z_groups = self.data_split.AxisSorting_zyx(self.filtered_data, z_threshold)
            logger.info(f"data spilt z_groups len: {len(z_groups)}")
            # self.visualizer.plot_multiple_layers(z_groups, 60, 16)  # 绘制切割后7-0的拼接图像
            # self.visualizer.plot_layer(z_groups, layer_index=2)  # 绘制切割后图像

            if len(self.filtered_data) < 20:
                return []

            # 根据y轴每间隔10mm进行切割，排序为yzx顺序，绘制切割后图像
            y_threshold = self.process_config["y_threshold"]
            y_groups = self.data_split.AxisSorting_yzx(self.filtered_data, y_threshold)
            logger.info(f"data spilt y_groups len: {len(y_groups)}")
            # self.visualizer.plot_layer(y_groups, layer_index=150)

            y_max, y_min, work_type = self.find_yplane(y_groups, z_groups)  # 找到最上边缘和最下边缘y平面

            allsendpointdata = self.process_z_group(y_max, y_min, z_groups, work_type)  # 分区处理
            logger.info("finish process data")
            # print("finish process data")
            return allsendpointdata
        except Exception:
            return []

    def judge_shape(self, x_range, y_range, z_range):
        # 自动化工件
        if x_range > self.process_config["x_range"] and\
                y_range > self.process_config["y_range"] and z_range > self.process_config["z_range"]:
            work_type = 2  # 柜体
        else:
            work_type = 1  # 平板

        # -----------预编程工件----------------
        # #  1：X:1070, Y:1900，Z:600
        # #  2：X:350, Y:1150，Z:420
        # #  3：X:300, Y:550，Z:330
        # #  4：X:660, Y:1675，Z:1200
        # #  5：X:260, Y:380，Z:300
        # x_difference = 160
        # difference = 110
        # if 1900 - difference < y_range < 1900 + difference and\
        #         600 - difference < self.z_range < 600 + difference and\
        #         1070 - difference < x_range < 1070 + x_difference:
        #     work_type = 1
        # elif 1150 - difference < y_range < 1150 + difference and\
        #         420 - difference < self.z_range < 420 + difference and\
        #         350 - difference < x_range < 350 + x_difference:
        #     work_type = 2
        # elif 500 < y_range < 550 + difference and\
        #         330 - difference < self.z_range < 330 + difference and\
        #         300 - difference < x_range < 300 + x_difference:
        #     work_type = 3
        # elif 1675 - difference < y_range < 1675 + difference and\
        #         1200 - difference < self.z_range < 1200 + difference and\
        #         660 - difference < x_range < 660 + x_difference:
        #     work_type = 4
        # elif 380 - difference < y_range < 380 + difference and\
        #         300 - difference < self.z_range < 400 and\
        #         260 - difference < x_range < 260 + x_difference:
        #     work_type = 5
        # else:
        #     work_type = 0
        # return work_type, x_max, x_min, y_max, y_min
        # ------------------------------
        return work_type

    def find_yplane(self, y_groups, z_groups):
        first_complete_layer = None
        last_complete_layer = None
        z_range = np.max(self.filtered_data[:, 2]) - np.min(self.filtered_data[:, 2])

        x_range = 0
        for ii in range(len(z_groups)):
            merged_group = np.vstack(z_groups[ii])
            temp_x_max = np.max(merged_group[:, 0])
            temp_x_min = np.min(merged_group[:, 0])
            temp_x_range = temp_x_max - temp_x_min
            x_range = max(temp_x_range, x_range)
        self.x_max = np.max(self.filtered_data[:, 0])
        self.x_min = np.min(self.filtered_data[:, 0])
        self.x_middle = (self.x_max + self.x_min) / 2
        self.auto_partition_x_threshold = self._resolve_auto_partition_x_threshold()

        y_max_temp = np.max(self.filtered_data[:, 1])
        y_min_temp = np.min(self.filtered_data[:, 1])
        y_range = y_max_temp - y_min_temp

        logger.info(f"x_range: {x_range}, y_range: {y_range}, z_range: {z_range}")
        logger.info(f"outside x range: {self.x_max - self.x_min}")
        work_type = self.judge_shape(x_range, y_range, z_range)
        logger.info(f"judge shape work_type is : {work_type}")

        self.fifo_frame_pos = z_range
        if work_type == 1:  # 平板
            y_max = y_max_temp
            y_min = y_min_temp
        else:
            for ii in reversed(range(len(y_groups))):
                if len(y_groups[ii]) > z_range / 10 * self.find_y_plane:
                    if first_complete_layer is None:
                        first_complete_layer = ii
                last_complete_layer = ii
            logger.info(f'lower plane idx: {last_complete_layer}, upper plane idx: {first_complete_layer}')
            if last_complete_layer is not None and first_complete_layer is not None:
                upper_yplane = y_groups[first_complete_layer]
                lower_yplane = y_groups[last_complete_layer]
                y_max = upper_yplane[0][0][1]
                y_min = lower_yplane[0][0][1]
                logger.info(f"upper plane Y : {y_max}, lower plane Y : {y_min}")
            else:
                y_max = np.max(self.filtered_data[:, 1])
                y_min = np.min(self.filtered_data[:, 1])
                logger.info(f"no find upper plane Y, lower plane Y, y_max is: {y_max}, y_min is: {y_min}")

        return y_max, y_min, work_type

    def _resolve_auto_partition_x_threshold(self):
        use_middle = int(self.process_config.get("inside_x_use_middle", 1))
        if use_middle not in (0, 1):
            raise ValueError("inside_x_use_middle must be 0 or 1")
        if use_middle == 1:
            return self.x_middle

        raw_offset = float(self.process_config.get("inside_x_min_offset", 100) or 0)
        if raw_offset < 0:
            raise ValueError("inside_x_min_offset must be greater than or equal to 0")
        offset = float(normalize_offset_value(raw_offset))

        threshold = self.x_min + offset
        if threshold > self.x_max:
            raise ValueError("inside_x_min_offset exceeds the workpiece X range")
        return threshold

    def process_z_group(self, y_max, y_min, z_group, work_type):
        """
        循环遍历z_group的每一分隔
        找到第一个Y平面所在的平面，往上找y轴如果还有数据，则是挂具
        """
        allsendpointdata = []
        merge_state_history = []
        last_surface_edge = []
        for ii in range(len(z_group)):
            # 1. 初始化帧基础参数
            point_id, odd_surface_edge, even_surface_edge, \
                work_v_middle, last_surface_edge = self._init_frame_params(ii, len(z_group), last_surface_edge)

            # 2. 检测挂具数据
            jig_dat = self._detect_jig_data(z_group[ii], y_max)

            # self.visualizer.plot_layer(z_group, layer_index=ii)  # 绘制切割后图像
            if work_type == 2:
                # 3. 处理分区数据
                (x_max, x_min, formatted_partitions,
                    formatted_merge_partitions) = self._process_partitions(z_group[ii], y_max, y_min,
                                                                           last_surface_edge, self.x_max,
                                                                           self.x_min)
                merge_state_history.append(formatted_merge_partitions)

                # 4. 更新固定6个分区状态
                partitions = self._update_partitions(formatted_partitions, last_surface_edge)

                # 5. 更新合并后分区状态
                merge_partitions = self._update_merge_partiitons(formatted_merge_partitions)

                # 6. 构建最终数据
                sendpointdata = LateralData(point_id, self.fifo_frame_pos, jig_dat, work_type, work_v_middle, odd_surface_edge, even_surface_edge,
                                            int(x_max), int(x_min), y_max, y_min, partitions, merge_partitions)
                allsendpointdata.append(sendpointdata)
            else:
                # 平板件直接构建最终数据
                sendpointdata = LateralData(point_id, self.fifo_frame_pos, jig_dat, work_type, work_v_middle, odd_surface_edge, even_surface_edge,
                                            self.x_max, self.x_min, y_max, y_min)
                allsendpointdata.append(sendpointdata)
        # logger.info(f"All send points data: {allsendpointdata}")

        # 7. 查找合并分区up_y和do_y的特征值
        upper_y_value, down_y_value = self._cal_y_value(merge_state_history)

        # 8、根据特征值修改数据
        resetallsendpointdata = self._reset_send_data(upper_y_value, down_y_value, allsendpointdata)
        # logger.info(f"Y-axis boundary reset completed: {resetallsendpointdata}")

        # 9、处理成空间方块数据
        rconcatenatedata = self._data_concatenation(resetallsendpointdata)
        # logger.info(f"data concatenation: {rconcatenatedata}")
        # print(f"data concatenation: {rconcatenatedata}")

        # 10. 构建方块数据并构建方块数据BlockData
        blockdata = self._update_block_data(rconcatenatedata)
        logger.info(f"data blockdata: {blockdata}")

        if self.process_config["send_data_type"] == 1:  # 1=分帧发送，2=整方块数据发送
            return resetallsendpointdata
        else:
            return blockdata

    def _init_frame_params(self, frame_idx, total_frames, last_surface_edge):
        """初始化帧参数"""
        if frame_idx == 0:
            return (frame_idx, 1, 0, 0, [1, 1, 1, 1, 1, 1])
        elif frame_idx == total_frames - 1:
            return (frame_idx, 0, 2, 0, [0, 0, 0, 0, 0, 0])
        elif frame_idx == int(total_frames / 2):
            return (frame_idx, 0, 0, 1, last_surface_edge)
        else:
            return (frame_idx, 0, 0, 0, last_surface_edge)

    def _detect_jig_data(self, frame_data, y_max):
        """检测挂具数据"""
        top_layer = frame_data[-1] if frame_data else None
        if top_layer is not None and np.max(top_layer[:, 1]) > y_max + self.process_config["jig_threshold"]:
            return np.max(top_layer[:, 1])
        return 0

    def _process_partitions(self, frame_data, y_max, y_min, last_surface_edge, x_max, x_min):
        """处理分区数据"""
        if self.process_config["find_boundary_type"] == 1:
            # 固定6个分区数据
            result = self.data_partition_static.find_boundary_layers_static(
                frame_data, y_max, y_min, self.x_middle, last_surface_edge
            )
        else:
            # 自动化查找分区数据
            result = self.data_partition_auto.find_boundary_layers_auto(
                frame_data, y_max, y_min, self.auto_partition_x_threshold, last_surface_edge, x_max, x_min
            )
        return result

    def _update_partitions(self, partitions, last_surface_edge):
        """更新固定6个分区状态"""
        if partitions is None:
            return None
        updated = []
        for p in partitions:
            if p.get("partition_id") is not None:
                last_surface_edge[p["partition_id"]] = p.get("surface_edge")
            updated.append(PartitionData(
                partition_id=p.get("partition_id"),
                partition_odd_surface_edge=p.get("odd_surface_edge"),
                partition_even_surface_edge=p.get("even_surface_edge"),
                partition_x_min=int(p.get("x_min", 0)),
                partition_up_edge_y=int(p.get("up_edge_y", 0)),
                partition_up_edge_x_max=int(p.get("up_edge_x_max", 0)),
                partition_do_edge_y=int(p.get("do_edge_y", 0)),
                partition_do_edge_x_max=int(p.get("do_edge_x_max", 0))
            ))
        return updated

    def _update_merge_partiitons(self, formatted_merge_partitions):
        if formatted_merge_partitions is None:
            return None
        updated_merge_partitions = []
        for mp in formatted_merge_partitions:
            updated_merge_partitions.append(MergePartitionData(
                merge_partition_id=mp.get("merge_partition_id"),
                merge_partition_surface_edge=mp.get("merge_partition_surface_edge", 0),
                merge_partition_x_min=mp.get("merge_partition_x_min", 0.0),
                merge_partition_x_max=mp.get("merge_partition_x_max", 0.0),
                merge_partition_y_min=mp.get("merge_partition_y_min", 0.0),
                merge_partition_y_max=mp.get("merge_partition_y_max", 0.0)
            ))
        return updated_merge_partitions

    def _cal_y_value(self, merge_state_history):
        if not merge_state_history:
            return None, None

        column_partitions = self._group_merge_state_history(merge_state_history)
        upper_y_value, down_y_value = self._calculate_y_values(column_partitions)
        # logger.info(f"Final Y values with overlap handling: upper={upper_y_value}, down={down_y_value}")
        return upper_y_value, down_y_value

    def _group_merge_state_history(self, merge_state_history):
        column_partitions = []
        current_partition = []

        for row in merge_state_history:
            if not row:
                if current_partition and len(current_partition) >= 5:
                    column_partitions.append(current_partition)
                current_partition = []
            else:
                current_partition.append(row)

        if current_partition and len(current_partition) >= 5:
            column_partitions.append(current_partition)

        return column_partitions

    def _calculate_y_values(self, column_partitions):
        upper_y_value = []
        down_y_value = []
        threshold = self.process_config["y_partition_threshold"]

        for col_idx, column in enumerate(column_partitions):
            y_group_dict = self._group_column_data(column, threshold)
            final_groups = self._filter_and_sort_groups(y_group_dict, threshold)
            upper_y_value, down_y_value = self._compute_final_y_values(final_groups, col_idx, upper_y_value, down_y_value, threshold)

        return upper_y_value, down_y_value

    def _group_column_data(self, column, threshold):
        y_group_dict = {}
        group_counter = 0
        # 记录每个组最后出现的行索引
        group_last_row = {}  # group_id -> last row index

        # 遍历每一行（帧）
        for row_idx, row in enumerate(column):
            # 当前帧中出现的所有组ID
            current_frame_groups = set()

            # 遍历当前帧中的所有分区
            for entry_idx, entry in enumerate(row):
                y_max = entry["merge_partition_y_max"]
                y_min = entry["merge_partition_y_min"]

                found_group = None
                # 只考虑在上一帧出现过的组（保证连续性）
                for gid, group in y_group_dict.items():
                    last_row = group_last_row.get(gid, -1)
                    # 检查是否连续（上一帧出现）且y值在阈值内
                    if last_row == row_idx - 1 and \
                        abs(y_max - group["current_ymax"]) <= threshold and \
                            abs(y_min - group["current_ymin"]) <= threshold:
                        found_group = gid
                        break

                if found_group is not None:
                    # 更新现有组
                    group = y_group_dict[found_group]
                    group["y_maxs"].append(y_max)
                    group["y_mins"].append(y_min)
                    group["current_ymax"] = max(y_max, group["current_ymax"])
                    group["current_ymin"] = min(y_min, group["current_ymin"])
                    group["count"] += 1
                    group_last_row[found_group] = row_idx
                    current_frame_groups.add(found_group)
                else:
                    # 创建新组
                    group_counter += 1
                    y_group_dict[group_counter] = {
                        "entry_idx": entry_idx,
                        "y_maxs": [y_max],
                        "y_mins": [y_min],
                        "current_ymax": y_max,
                        "current_ymin": y_min,
                        "count": 1
                    }
                    group_last_row[group_counter] = row_idx
                    current_frame_groups.add(group_counter)

            # 检查是否有组在本帧中断（上一帧出现过但本帧未出现）
            for gid in list(group_last_row.keys()):
                if group_last_row[gid] == row_idx - 1 and gid not in current_frame_groups:
                    # 组中断，移除其最后一行记录
                    del group_last_row[gid]

        return y_group_dict

    def _filter_and_sort_groups(self, y_group_dict, threshold):
        entry_groups = defaultdict(list)
        for group_id, group_data in y_group_dict.items():
            entry_idx = 0
            group_count = len(group_data["y_mins"])

            if group_count >= 5:
                entry_groups[entry_idx].append({
                    "group_id": group_id,
                    "min": min(group_data["y_mins"]),
                    "max": max(group_data["y_maxs"]),
                    "count": group_count,
                    "data": group_data
                })
                entry_idx += 1

        final_groups = {}
        for entry_idx, groups in entry_groups.items():
            sorted_groups = sorted(groups, key=lambda x: x["count"], reverse=True)
            selected_groups = []
            for group in sorted_groups:
                overlap = any(not (group["max"] < selected["min"] or group["min"] > selected["max"]) for selected in selected_groups)
                if not overlap:
                    selected_groups.append({
                        "min": group["min"],
                        "max": group["max"],
                        "count": group["count"]
                    })
                    final_groups[group["group_id"]] = group["data"]

        return final_groups

    def _compute_final_y_values(self, final_groups, col_idx, upper_y_value, down_y_value, threshold):
        for group_id, group_data in final_groups.items():
            entry_idx = group_data["entry_idx"]
            y_maxs = group_data["y_maxs"]
            y_mins = group_data["y_mins"]

            avg_y_max = self._trimmed_mean(y_maxs)
            avg_y_min = self._trimmed_mean(y_mins)

            filtered_max = [v for v in y_maxs if abs(v - avg_y_max) <= threshold]
            filtered_min = [v for v in y_mins if abs(v - avg_y_min) <= threshold]

            if not filtered_max or not filtered_min:
                continue

            final_avg_max = self._trimmed_mean(filtered_max)
            final_avg_min = self._trimmed_mean(filtered_min)

            upper_y_value.append({
                "col_idx": col_idx,
                "entry_idx": entry_idx,
                "group_id": group_id,
                "average": round(final_avg_max, 4),
                "max": max(filtered_max),
                "min": min(filtered_max)
            })

            down_y_value.append({
                "col_idx": col_idx,
                "entry_idx": entry_idx,
                "group_id": group_id,
                "average": round(final_avg_min, 4),
                "max": max(filtered_min),
                "min": min(filtered_min)
            })
        return upper_y_value, down_y_value

    def _trimmed_mean(self, values):
        if len(values) < 3:
            return np.mean(values)
        sorted_vals = sorted(values)
        return np.mean(sorted_vals[1:-1])

    def _reset_send_data(self, upper_y_value, down_y_value, allsendpointdata):
        '''
        更新所有分区的最大y和最小y统一
        确认边缘逻辑：
        如果当前data_obj.merge_partitions[i]在upper_stats["min"] <= current_y_max <= upper_stats["max"] and
        down_stats["min"] <= current_y_min <= down_stats["max"]范围内，则记录此时的upper_y_value[col_idx]的col_idx，
        对比上一帧数据data_obj【-1】.merge_partitions是否存在data_obj.merge_partitions[i]在upper_y_value[col_idx]的范围内，
        如果存在则last_flag = 1，否则为0。同时还要对比下一帧数据data_obj【+1】.merge_partitions是否存在data_obj.merge_partitions[i]
        在upper_y_value[col_idx]的范围内，如果存在则next_flag = 1，否则为0，下一帧数据data_obj【+1】.merge_partitions为空也
        是next_flag = 0。根据last_flag和next_flag确认current_flag，data_obj，
        更新merge_partitions[i].merge_partition_surface_edge=current_flag
        last_flag          next_flag          current_flag
            1                    0                   1
            1                    1                   0
            0                    1                   2
            0                    0                   0
        '''
        if upper_y_value is None or down_y_value is None:
            return allsendpointdata

        stat_configs = self._generate_stat_configs(upper_y_value, down_y_value)
        last_2_list = []

        for data_idx, data_obj in enumerate(allsendpointdata):
            if not data_obj.merge_partitions:
                continue

            for part in data_obj.merge_partitions:
                matched_configs = self._adjust_partition_values(part, stat_configs)
                self._update_partition_flags(data_idx, part, matched_configs, allsendpointdata, last_2_list)

        return allsendpointdata

    def _generate_stat_configs(self, upper_y_value, down_y_value):
        return [
            {
                "upper_min": stats["min"] - self.process_config["y_partition_threshold"],
                "upper_max": stats["max"] + self.process_config["y_partition_threshold"],
                "down_min": down_stats["min"] - self.process_config["y_partition_threshold"],
                "down_max": down_stats["max"] + self.process_config["y_partition_threshold"],
                "set_upper": stats["min"],
                "set_down": down_stats["max"]
            }
            for stats, down_stats in zip(upper_y_value, down_y_value)
        ]

    def _adjust_partition_values(self, part, stat_configs):
        matched_configs = []
        current_y_max = part.merge_partition_y_max
        current_y_min = part.merge_partition_y_min

        for config in stat_configs:
            upper_match = config["upper_min"] <= current_y_max <= config["upper_max"]
            down_match = config["down_min"] <= current_y_min <= config["down_max"]

            if upper_match:
                part.merge_partition_y_max = config["set_upper"]
            if down_match:
                part.merge_partition_y_min = config["set_down"]

            if upper_match and down_match:
                matched_configs.append(config)

        return matched_configs

    def _update_partition_flags(self, data_idx, part, matched_configs, allsendpointdata, last_2_list):
        last_flag, next_flag = self._check_adjacent_frames(data_idx, part, matched_configs, allsendpointdata)

        if last_flag and not next_flag:
            part.merge_partition_surface_edge = 1
        elif last_flag and next_flag:
            part.merge_partition_surface_edge = 0
        elif not last_flag and next_flag:
            part.merge_partition_surface_edge = 2
        else:
            part.merge_partition_surface_edge = 0

        self._handle_short_intervals(data_idx, part, last_2_list)

    def _check_adjacent_frames(self, data_idx, part, matched_configs, allsendpointdata):
        last_flag = self._check_frame(data_idx - 1, part, matched_configs, allsendpointdata) if data_idx > 0 else 0
        next_flag = self._check_frame(data_idx + 1, part, matched_configs, allsendpointdata) if data_idx < len(allsendpointdata) - 1 else 0
        return last_flag, next_flag

    def _check_frame(self, frame_idx, part, matched_configs, allsendpointdata):
        frame_data = allsendpointdata[frame_idx]
        for frame_part in frame_data.merge_partitions:
            if any(
                config["upper_min"] <= frame_part.merge_partition_y_max <= config["upper_max"] and
                config["down_min"] <= frame_part.merge_partition_y_min <= config["down_max"]
                for config in matched_configs
            ):
                return 1
        return 0

    def _handle_short_intervals(self, data_idx, part, last_2_list):
        current_y_max = part.merge_partition_y_max
        current_y_min = part.merge_partition_y_min
        if part.merge_partition_surface_edge == 2:
            last_2_list.append({
                "y_max": current_y_max,
                "y_min": current_y_min,
                "data_idx": data_idx,
                "part": part
            })
        elif part.merge_partition_surface_edge == 1:
            for info in last_2_list:
                if (abs(current_y_max - info["y_max"]) <= self.process_config["y_partition_threshold"] and
                        abs(current_y_min - info["y_min"]) <= self.process_config["y_partition_threshold"]):
                    # 检查帧间隔是否 ≤10
                    if (data_idx - info["data_idx"]) <= self.process_config["z_partition_threshold"] / 10 or\
                            (info["y_max"] - info["y_min"]) <= self.process_config["y_partition_threshold"]:
                        # 仅修改这两个part的标志位
                        info["part"].merge_partition_surface_edge = 0
                        part.merge_partition_surface_edge = 0
                    # 从列表中移除已处理的记录（避免重复处理）
                    last_2_list.remove(info)
                    break

    def _data_concatenation(self, resetallsendpointdata):
        '''
        轮询每一帧数据：
        1、当jig_dat不为0则代表是jig的开始，直到再次为0是结束，记录开始到结束的z轴范围z_min和z_max，在这区间内的x_max的最大值和x_min的最小值，
        同时在这个区间内的最大jig_dat为y_max,y_min为0，第一个分区的idx为0，最多两个分区。
        2、outside_data即外侧数据为所有数据的x_max,x_min,y_max,y_min,z_max,z_min。
        3、inside_data即内测数据在partitions中查找：当遇到merge_partition_surface_edge=2则代表开始，merge_partition_surface_edge=1则代表结束，记录为1个分区，记录当前分区的为idx0
        的x_min,x_max, y_min,y_max,z_min,z_max。下一个分区为idx1的x_min,x_max, y_min,y_max,z_min,z_max以此类推，上限为10个分区
        '''
        jig_data = []
        outside_data = {
            "x_max": -np.inf, "x_min": np.inf,
            "y_max": -np.inf, "y_min": np.inf,
            "z_max": self.fifo_frame_pos,  # 总高度
            "z_min": 0
        }
        inside_data = []
        active_partitions = []
        current_jig = None

        for frame_idx, data_obj in enumerate(resetallsendpointdata):
            z_pos = frame_idx * 10

            # ========== 外侧数据更新 ==========
            outside_data["x_max"] = max(outside_data["x_max"], data_obj.x_max)
            outside_data["x_min"] = min(outside_data["x_min"], data_obj.x_min)
            outside_data["y_max"] = max(outside_data["y_max"], data_obj.up_edge_y)
            outside_data["y_min"] = min(outside_data["y_min"], data_obj.do_edge_y)

            # ========== Jig数据处理 ==========
            current_jig, jig_data = self.jig_data_process(data_obj, z_pos, current_jig, jig_data)

            # --- 处理内侧数据 ---
            if data_obj.merge_partitions:
                for part in data_obj.merge_partitions:
                    # 判断是否匹配已有活跃分区（y范围重叠阈值100mm）
                    matched_part = None
                    for p in active_partitions:
                        if (part.merge_partition_y_max < p["y_max"] + self.process_config["y_partition_threshold"]) and \
                                (part.merge_partition_y_min > p["y_min"] - self.process_config["y_partition_threshold"]):
                            matched_part = p
                            break

                    # 处理分区的开始(surface_edge=2)
                    if part.merge_partition_surface_edge == 2:
                        if self.data_dir == 2:
                            active_partitions.append(self.handle_new_partition(part, z_pos))
                        elif self.data_dir == 1 and frame_idx > 1:
                            if self.check_previous_frames(resetallsendpointdata, frame_idx, part):
                                active_partitions.append(self.handle_new_partition(part, z_pos))
                    elif matched_part:
                        matched_part["x_min"] = min(matched_part["x_min"], part.merge_partition_x_min)
                        matched_part["x_max"] = min(matched_part["x_max"], part.merge_partition_x_max)
                        matched_part["y_min"] = max(matched_part["y_min"], part.merge_partition_y_min)
                        matched_part["y_max"] = min(matched_part["y_max"], part.merge_partition_y_max)
                        matched_part["z_end"] = z_pos
                        # 处理分区的结束(surface_edge=1)
                        if part.merge_partition_surface_edge == 1:
                            if self.data_dir == 2:
                                inside_data.append({
                                    "idx": len(inside_data),
                                    "x_min": matched_part["x_min"],
                                    "x_max": matched_part["x_max"],
                                    "y_min": matched_part["y_min"],
                                    "y_max": matched_part["y_max"],
                                    "z_start": matched_part["z_start"],
                                    "z_end": matched_part["z_end"]
                                })
                            elif self.data_dir == 1 and frame_idx < len(resetallsendpointdata) - 2:
                                if self.check_next_frames(resetallsendpointdata, frame_idx, part):
                                    inside_data.append({
                                        "idx": len(inside_data),
                                        "x_min": matched_part["x_min"],
                                        "x_max": matched_part["x_max"],
                                        "y_min": matched_part["y_min"],
                                        "y_max": matched_part["y_max"],
                                        "z_start": matched_part["z_start"],
                                        "z_end": matched_part["z_end"]
                                    })
                            active_partitions.remove(matched_part)

        # ========== 后处理未闭合数据 ==========
        # 处理未结束的Jig
        if current_jig is not None:
            current_jig["z_end"] = z_pos
            if len(jig_data) < 2:
                jig_data.append(current_jig)

        # 处理未闭合的内侧分区（超过10帧视为有效）
        for p in active_partitions:
            if (p["z_end"] - p["z_start"]) >= self.process_config["z_partition_threshold"]:
                inside_data.append({
                    "idx": len(inside_data),
                    "x_min": p["x_min"],
                    "x_max": p["x_max"],
                    "y_min": p["y_min"],
                    "y_max": p["y_max"],
                    "z_start": p["z_start"],
                    "z_end": p["z_end"]
                })
        inside_data = inside_data[:20]  # 最多保留20个分区
        new_inside_data = self.handle_inside_data(inside_data)

        return {
            "jig_data": jig_data,
            "outside_data": outside_data,
            "inside_data": new_inside_data
        }

    def handle_inside_data(self, inside_data):
        '''
        1、找到inside_data中的z_min和z_max最大的分区，求出这个分区的中心点z_middle，
        #如果存在其他分区的z_max、z_min和z_middle差值小于50，则去掉该分区，保留剩下的分区
        2、在剩余分区上执行“找平”操作：如果分区之间存在重叠，则所有分区的 z_start 取最大值，z_end 取最小值。
        '''
        if not inside_data:
            return []
        # ------------------------------元信信高情况，需要去掉中间挂件两侧分区
        # max_z_partition = max(inside_data, key=lambda x: x["z_end"] - x["z_start"])
        # z_middle = (max_z_partition["z_start"] + max_z_partition["z_end"]) / 2
        # filtered = []
        # for part in inside_data:
        #     if abs(part["z_start"] - z_middle) < 50 or abs(part["z_end"] - z_middle) < 50:
        #         continue
        #     filtered.append(part)
        # if not filtered:
        #     return []

        # -------------------------------找平内分区Z轴
        n = len(inside_data)
        parent = list(range(n))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        # 检查每对分区是否有重叠
        for i in range(n):
            for j in range(i + 1, n):
                # 检查 Z 轴是否有重叠
                if max(inside_data[i]["z_start"], inside_data[j]["z_start"]) < min(inside_data[i]["z_end"], inside_data[j]["z_end"]):
                    union(i, j)

        # 分组
        groups = {}
        for i in range(n):
            r = find(i)
            groups.setdefault(r, []).append(i)

        # 对每组分区计算统一的 Z 轴范围
        inside_counter = count(0)
        sub_counter = count(0)
        inside_objs: List[InsideData] = []

        for root, idxs in groups.items():
            # 计算该组的统一 Z 轴范围
            group_max_start = max(inside_data[i]["z_start"] for i in idxs)
            group_min_end = min(inside_data[i]["z_end"] for i in idxs)

            # 如果组内分区有共同的重叠区间，则使用统一的范围
            if group_max_start < group_min_end:
                z_start, z_end = group_max_start, group_min_end
                # logger.info(f"Aligned group {idxs} to z_start={z_start}, z_end={z_end}")
            else:
                # 如果没有共同重叠区间，则使用各自的范围
                z_start = min(inside_data[i]["z_start"] for i in idxs)
                z_end = max(inside_data[i]["z_end"] for i in idxs)
                logger.debug(f"Group {idxs} has no common overlap, using union range {z_start}-{z_end}")

            # 创建子分区列表，使用统一的 Z 轴范围
            sub_list: List[SubInsideData] = []
            for i in idxs:
                p = inside_data[i]
                sub_list.append(
                    SubInsideData(
                        subinside_id=next(sub_counter),
                        subinside_x_min=p.get("x_min"),
                        subinside_x_max=p.get("x_max"),
                        subinside_y_min=p.get("y_min"),
                        subinside_y_max=p.get("y_max"),
                        subinside_z_min=z_start,
                        subinside_z_max=z_end
                    )
                )

            sub_list.sort(
                key=lambda sub: (
                    float(sub.subinside_y_max or 0),
                    float(sub.subinside_y_min or 0),
                ),
                reverse=True,
            )

            inside_objs.append(InsideData(inside_id=next(inside_counter), subinside_datalist=sub_list))

        # 先按 Z 起始位置排序；若 Z 相同，再按 Y 从大到小排序
        inside_objs.sort(
            key=lambda x: (
                float(x.subinside_datalist[0].subinside_z_min or 0) if x.subinside_datalist else 0,
                -float(x.subinside_datalist[0].subinside_y_max or 0) if x.subinside_datalist else 0,
                -float(x.subinside_datalist[0].subinside_y_min or 0) if x.subinside_datalist else 0,
            ),
        )
        return inside_objs

    def jig_data_process(self, data_obj, z_pos, current_jig, jig_data):
        if data_obj.jig_dat != 0:
            if current_jig is None:  # Jig开始
                current_jig = {
                    "z_start": z_pos,
                    "z_end": None,
                    "x_min": data_obj.x_min,
                    "x_max": data_obj.x_max,
                    "y_max": data_obj.jig_dat,
                    "y_min": data_obj.up_edge_y,
                    "idx": len(jig_data)
                }
            else:  # 更新jig参数
                current_jig["x_min"] = min(current_jig["x_min"], data_obj.x_min)
                current_jig["x_max"] = max(current_jig["x_max"], data_obj.x_max)
                current_jig["y_max"] = max(current_jig["y_max"], data_obj.jig_dat)
        else:
            if current_jig is not None:  # Jig结束
                current_jig["z_end"] = z_pos
                jig_data.append(current_jig)
                current_jig = None

        return current_jig, jig_data

    def check_y_overlap(self, part_a, part_b):
        """检查两个分区在y轴上是否有重叠"""
        return (part_b.merge_partition_y_min < part_a.merge_partition_y_max < part_b.merge_partition_y_max) or \
            (part_b.merge_partition_y_min < part_a.merge_partition_y_min < part_b.merge_partition_y_max)

    def handle_new_partition(self, part, z_pos):
        """创建新分区字典"""
        return {
            "x_min": part.merge_partition_x_min,
            "x_max": part.merge_partition_x_max,
            "y_min": part.merge_partition_y_min,
            "y_max": part.merge_partition_y_max,
            "z_start": z_pos,
            "z_end": z_pos
        }

    def check_previous_frames(self, resetallsendpointdata, frame_idx, part):
        """检查之前帧中是否存在非重叠分区"""
        for last_temp_frame_idx in range(0, frame_idx - 1):
            if resetallsendpointdata[last_temp_frame_idx].merge_partitions == []:
                return True
            for last_part in resetallsendpointdata[last_temp_frame_idx].merge_partitions:
                if not self.check_y_overlap(part, last_part):
                    return True
        return False

    def check_next_frames(self, resetallsendpointdata, frame_idx, part):
        """检查后续帧中是否存在非重叠分区"""
        for next_temp_frame_idx in range(frame_idx + 1, len(resetallsendpointdata) - 1):
            if resetallsendpointdata[next_temp_frame_idx].merge_partitions == []:
                return True
            for next_part in resetallsendpointdata[next_temp_frame_idx].merge_partitions:
                if not self.check_y_overlap(part, next_part):
                    return True
        return False

    def _update_block_data(self, concatenated_data):
        """构建BlockData数据对象"""
        if concatenated_data is None:
            return None
        jig_data_list = []
        for idx, jig in enumerate(concatenated_data.get("jig_data", [])[:5]):  # 最多五组
            jig_data_list.append(
                JigData(
                    jig_id=idx,
                    jig_y_min=int(round(float(jig.get("y_min", 0)))),
                    jig_y_max=int(round(float(jig.get("y_max", 0)))),
                    jig_z_min=int(round(float(jig.get("z_start", 0)))),
                    jig_z_max=int(round(float(jig.get("z_end", 0))))
                )
            )

        # 转换OutsideData（仅一组）
        outside_dict = concatenated_data.get("outside_data", {})
        outside_data_list = []
        if outside_dict:
            outside_data_list.append(
                OutsideData(
                    outside_x_min=int(round(float(outside_dict.get("x_min", 0)))),
                    outside_x_max=int(round(float(outside_dict.get("x_max", 0)))),
                    outside_y_min=int(round(float(outside_dict.get("y_min", 0)))),
                    outside_y_max=int(round(float(outside_dict.get("y_max", 0)))),
                    outside_z_min=int(round(float(outside_dict.get("z_min", 0)))),
                    outside_z_max=int(round(float(outside_dict.get("z_max", 0))))
                )
            )

        # 转换InsideData（最多十组）
        inside_data_list = []
        for idx, inside in enumerate(concatenated_data.get("inside_data", [])[:10]):
            subinside_list = []
            # 检查subinside_datalist是否存在且不为空
            if inside.subinside_datalist:
                for sub_idx, sub in enumerate(inside.subinside_datalist[:20]):  # 子分区数量限制为20
                    subinside_list.append(
                        SubInsideData(
                            subinside_id=sub_idx,
                            subinside_x_min=int(round(float(sub.subinside_x_min or 0))),
                            subinside_x_max=int(round(float(sub.subinside_x_max or 0))),
                            subinside_y_min=int(round(float(sub.subinside_y_min or 0))),
                            subinside_y_max=int(round(float(sub.subinside_y_max or 0))),
                            subinside_z_min=int(round(float(sub.subinside_z_min or 0))),
                            subinside_z_max=int(round(float(sub.subinside_z_max or 0)))
                        )
                    )
            inside_data_list.append(
                InsideData(
                    inside_id=idx,
                    subinside_datalist=subinside_list if subinside_list else None
                )
            )

        # 构建BlockData对象
        block_data = BlockData(
            lidar_status=int(self.lidar_status),
            fifo_frame_pos=int(round(self.fifo_frame_pos)),
            data_dir=self.data_dir,
            jig_data=jig_data_list if jig_data_list else None,
            outside_data=outside_data_list if outside_data_list else None,
            inside_data=inside_data_list if inside_data_list else None
        )

        # logger.info(f"BlockData Build Finish: {block_data}")
        return block_data

    # def __del__(self):
    #     """析构时确保关闭可视化"""
    #     self.visualizer.close_visualization()
