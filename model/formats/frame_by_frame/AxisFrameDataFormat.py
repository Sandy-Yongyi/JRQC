from dataclasses import dataclass
from typing import List, Optional


@dataclass
class AxisData:
    """单个水平轴的数据结构"""
    H_Axis: Optional[int] = None        # 水平轴定位位置Y/X坐标
    V_Axis_Max: Optional[int] = None    # 水平轴中对应的垂直轴的最大值，即水平轴是Y的话则对应X的最大值，X_Max/Y_Max
    V_Axis_Min: Optional[int] = None    # 水平轴中对应的垂直轴的最小值，即水平轴是Y的话则对应X的最小值，X_Min/Y_Min


@dataclass
class AxisFrameData:
    """每帧完整的坐标数据结构"""
    FrameData: Optional[List[AxisData]] = None        # 数据列表
