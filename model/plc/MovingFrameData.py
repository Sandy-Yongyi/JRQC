"""PLC 运动帧数据结构定义。"""

from dataclasses import dataclass, field
from typing import Annotated, ClassVar

from model.plc.PlcFrame import Int16, Int32, PlcFrame, Repeat


# 字节序配置: '>' = 大端序 (Big-Endian), '<' = 小端序 (Little-Endian)
_BYTE_ORDER = ">"
AXIS_LIST_COUNT = 48


@dataclass
class AxisData(PlcFrame):
    """单轴数据。"""

    BYTE_ORDER = _BYTE_ORDER
    Pos: Annotated[int, Int16()] = 0
    Speed: Annotated[int, Int16()] = 0
    Status: Annotated[int, Int16()] = 0


def create_axis_list() -> list[AxisData]:
    return [AxisData() for _ in range(AXIS_LIST_COUNT)]


@dataclass
class SendMovingFrameData(PlcFrame):
    """发送帧数据结构。"""

    BYTE_ORDER = _BYTE_ORDER
    FRAME_SIZE: ClassVar[int] = 302              # 固定帧长度(字节), 0表示不填充，需减掉开始头和结束头的长度
    Enable: Annotated[int, Int32()] = 0          # 使能/各设备运动,bit0总使能,bit1右边内顶,bit2右边侧面云雀,bit3左边外底,bit4右外顶,bit5左外顶,bit6各轴清理模式
    Gun_Cont1: Annotated[int, Int32()] = 0       # 开枪控制1（保留）
    Gun_Cont2: Annotated[int, Int16()] = 0       # 开枪控制2（保留）
    HeartBeat: Annotated[int, Int16()] = 0       # 心跳
    Operate: Annotated[int, Int16()] = 0         # 远程操作位（bit0不用，bit1控制链条：0停止，1运动）
    AxisList: Annotated[list[AxisData], Repeat(AXIS_LIST_COUNT)] = field(default_factory=create_axis_list)


@dataclass
class ReceiveMovingFrameData(PlcFrame):
    """接收帧数据结构。"""

    BYTE_ORDER = _BYTE_ORDER
    FRAME_SIZE: ClassVar[int] = 302              # 固定帧长度(字节), 0表示不填充
    ChainPulse: Annotated[int, Int32()] = 0      # 链条脉冲数
    ChainSpeed: Annotated[int, Int32()] = 0      # 链条速度
    HeartBeat: Annotated[int, Int16()] = 0       # 心跳
    Status: Annotated[int, Int16()] = 0          # 公共状态位（00000000 00000001，0是伺服轴报警，1是伺服正常）
    Operate: Annotated[int, Int16()] = 0         # 远程操作位,bit0总使能,bit1右边内顶,bit2右边侧面云雀,bit3左边外底,bit4右外顶,bit5左外顶,bit6各轴清理模式
    AxisList: Annotated[list[AxisData], Repeat(AXIS_LIST_COUNT)] = field(default_factory=create_axis_list)
