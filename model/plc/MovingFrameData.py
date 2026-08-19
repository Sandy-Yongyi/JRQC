"""PLC 运动帧数据结构定义。"""

from dataclasses import dataclass, field
from typing import Annotated, ClassVar

from model.plc.PlcFrame import Int16, Int32, PlcFrame, Repeat


# 字节序配置: '>' = 大端序 (Big-Endian), '<' = 小端序 (Little-Endian)
_BYTE_ORDER = ">"
SEND_AXIS_LIST_COUNT = 14
AXIS_LIST_COUNT = 16


@dataclass
class AxisData(PlcFrame):
    """单轴数据。"""

    BYTE_ORDER = _BYTE_ORDER
    Pos: Annotated[int, Int16()] = 0
    Speed: Annotated[int, Int16()] = 0
    Status: Annotated[int, Int16()] = 0


def create_axis_list() -> list[AxisData]:
    """创建发送帧中 SN0/SN1 使用的 14 个普通轴。"""
    return [AxisData() for _ in range(SEND_AXIS_LIST_COUNT)]


def create_receive_axis_list() -> list[AxisData]:
    """创建接收帧中的 16 个反馈轴，末两轴属于左右二维设备。"""
    return [AxisData() for _ in range(AXIS_LIST_COUNT)]


@dataclass
class SendMovingFrameData(PlcFrame):
    """发送帧数据结构。"""

    BYTE_ORDER = _BYTE_ORDER
    FRAME_SIZE: ClassVar[int] = 110              # 固定帧长度(字节), 0表示不填充，需减掉开始头和结束头的长度
    Enable: Annotated[int, Int32()] = 0          # bit0总使能, bit1左顶底, bit2右顶底, bit15清理模式
    Gun_Cont1: Annotated[int, Int32()] = 0       # 开枪控制1（保留）
    Gun_Cont2: Annotated[int, Int16()] = 0       # 开枪控制2（保留）
    HeartBeat: Annotated[int, Int16()] = 0       # 心跳
    Operate: Annotated[int, Int16()] = 0         # 远程操作位（bit0不用，bit1控制链条：0停止，1运动）
    AxisList: Annotated[list[AxisData], Repeat(SEND_AXIS_LIST_COUNT)] = field(default_factory=create_axis_list)
    Left2DLiftData: AxisData = field(default_factory=AxisData)
    Right2DLiftData: AxisData = field(default_factory=AxisData)


@dataclass
class ReceiveMovingFrameData(PlcFrame):
    """接收帧数据结构。"""

    BYTE_ORDER = _BYTE_ORDER
    FRAME_SIZE: ClassVar[int] = 110              # 固定帧长度(字节), 0表示不填充
    ChainPulse: Annotated[int, Int32()] = 0      # 链条脉冲数
    ChainSpeed: Annotated[int, Int32()] = 0      # 链条速度
    HeartBeat: Annotated[int, Int16()] = 0       # 心跳
    Status: Annotated[int, Int16()] = 0          # 公共状态位（00000000 00000001，0是伺服轴报警，1是伺服正常）
    Operate: Annotated[int, Int16()] = 0         # bit0总使能, bit1左顶底, bit2右顶底, bit15清理模式
    AxisList: Annotated[list[AxisData], Repeat(AXIS_LIST_COUNT)] = field(default_factory=create_receive_axis_list)
