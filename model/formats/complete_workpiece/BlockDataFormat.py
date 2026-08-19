from typing import List, Optional
from dataclasses import dataclass


@dataclass
class DistribeGunData:
    gun_id: Optional[int] = None                                    # 枪ID
    gun_y_enable: Optional[int] = None                              # 枪Y轴是否被选通
    gun_y_upper: Optional[int] = None                               # 枪所在Y轴往复最上位置
    gun_y_downer: Optional[int] = None                              # 枪所在Y轴往复最下位置
    gun_r_angle: Optional[int] = None                               # 枪所在R轴转角角度


@dataclass
class GunGroupData:
    group_type: Optional[str] = None                                 # "outside" / "inside" / "up" / "down"
    group_id: Optional[int] = None                                   # 分组ID；inside 场景下用于保存列ID/列序号
    gundata_list: Optional[List[DistribeGunData]] = None


@dataclass
class SingleMachineData:
    machine_id: Optional[int] = None                                 # 设备ID
    gun_groups: Optional[List[GunGroupData]] = None                  # 分枪数据列表


@dataclass
class JigData:
    jig_id: Optional[int] = None                                     # 挂具所在方块ID
    jig_y_min: Optional[int] = None                                  # 挂具方块y最小值
    jig_y_max: Optional[int] = None                                  # 挂具方块y最大值
    jig_z_min: Optional[int] = None                                  # 挂具方块z最小值
    jig_z_max: Optional[int] = None                                  # 挂具方块z最大值


@dataclass
class OutsideData:
    outside_x_min: Optional[int] = None                              # 外侧方块X最小值
    outside_x_max: Optional[int] = None                              # 外侧方块X最大值
    outside_y_min: Optional[int] = None                              # 外侧方块y最小值
    outside_y_max: Optional[int] = None                              # 外侧方块y最大值
    outside_z_min: Optional[int] = None                              # 外侧方块z最小值
    outside_z_max: Optional[int] = None                              # 外侧方块z最大值


@dataclass
class SubInsideData:
    # 每一列内侧分区的子内侧方块，按行排序
    subinside_id: Optional[int] = None
    subinside_x_min: Optional[int] = None                            # 子内侧方块X最小值，单位mm
    subinside_x_max: Optional[int] = None                            # 子内侧方块X最大值，单位mm
    subinside_y_min: Optional[int] = None                            # 子内侧方块y最小值，单位mm
    subinside_y_max: Optional[int] = None                            # 子内侧方块y最大值，单位mm
    subinside_z_min: Optional[int] = None                            # 子内侧方块z最小值，单位mm
    subinside_z_max: Optional[int] = None                            # 子内侧方块z最大值，单位mm


@dataclass
class InsideData:
    # 内侧分区，按列排序
    inside_id: Optional[int] = None                                  # 内侧方块ID
    subinside_datalist: Optional[List[SubInsideData]] = None         # 子内侧方块数据列表


@dataclass
class BlockData:
    lidar_status: Optional[int] = None                               # 0=激光正常，1=激光有异物遮挡，2=激光严重异常需重启
    fifo_frame_pos: Optional[int] = None                             # 开始发送数据时此时的工件最前面与激光的距离，单位mm
    data_dir: Optional[int] = None                                   # 1=左侧激光读数数据，2=右侧激光读数数据
    jig_data: Optional[List[JigData]] = None                         # 挂具数据
    outside_data: Optional[List[OutsideData]] = None                 # 外侧数据
    inside_data: Optional[List[InsideData]] = None                   # 内侧数据
    distribe_gun_list: Optional[List[SingleMachineData]] = None      # 第一阶段仅保存当前sn对应的一项分枪数据
