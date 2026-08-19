from typing import List, Optional
from dataclasses import dataclass


@dataclass
class PartitionData:
    partition_id: Optional[int] = None  # 所在分区ID
    partition_odd_surface_edge: Optional[int] = None  # 第partition_id分区垂直内侧边缘：1=最左边，0=非内侧边缘
    partition_even_surface_edge: Optional[int] = None  # 第partition_id分区垂直内侧边缘：2=最右边，0=非内侧边缘
    partition_x_min: Optional[int] = None  # 第partition_id分区垂直面最小x
    partition_up_edge_y: Optional[int] = None  # 第partition_id分区最上边缘Y值，所有帧一样，取最小值
    partition_up_edge_x_max: Optional[int] = None  # 第partition_id分区最上边缘Y值所对应的X最大值
    partition_do_edge_y: Optional[int] = None  # 第partition_id分区最下边缘Y值，所有帧一样，取最大值
    partition_do_edge_x_max: Optional[int] = None  # 第partition_id分区最下边缘Y值所对应的X最大值


@dataclass
class MergePartitionData:
    merge_partition_id: Optional[int] = None  # 合并分区后所在分区ID
    merge_partition_surface_edge: Optional[int] = None  # 第merge_partition_id分区垂直内侧边缘：1=最左边，2=最右边，0=非内侧边缘
    merge_partition_x_min: Optional[int] = None  # 第merge_partition_id分区X最小值
    merge_partition_x_max: Optional[int] = None  # 第merge_partition_id分区X最大值
    merge_partition_y_max: Optional[int] = None  # 第merge_partition_id分区最上边缘y值
    merge_partition_y_min: Optional[int] = None  # 第merge_partition_id分区最下边缘y值


@dataclass
class LateralData:
    point_id: Optional[int] = None  # 帧数
    fifo_frame_pos: Optional[int] = None  # 开始发送数据时此时的工件最前面与激光的距离，单位mm
    jig_dat: Optional[int] = None  # 0=非挂具，其他数=Y值（距离地面），单位mm
    work_type: int = 0  # 工件类型：1=平板，2=柜体，0=工件类型异常
    work_v_middle: Optional[int] = None  # 是否到工件中心：1=中心帧，0=非中心帧
    odd_surface_edge: Optional[int] = None  # 垂直外侧边缘：1=最左边，0=非外侧边缘
    even_surface_edge: Optional[int] = None  # 垂直外侧边缘：2=最右边，0=非外侧边缘
    x_max: Optional[int] = None  # 一帧X最大值，单位mm
    x_min: Optional[int] = None  # 一帧X最小值，单位mm
    up_edge_y: Optional[int] = None  # 所有帧外侧最上边缘的y坐标，单位mm
    do_edge_y: Optional[int] = None  # 所有帧外侧最下边缘的y坐标，单位mm
    partitions: Optional[List[PartitionData]] = None  # 分区数据，存储多个分区的信息
    merge_partitions: Optional[List[MergePartitionData]] = None  # 合并分区数据，存储多个分区的信息
