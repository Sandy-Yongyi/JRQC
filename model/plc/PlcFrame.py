"""
通用PLC二进制帧序列化框架
基于 dataclass + Annotated 类型注解，自动处理序列化/反序列化。
支持: int16, uint16, int32, uint32, float32, 定长字符串, 嵌套帧, 帧列表

使用方式:
    @dataclass
    class MyFrame(PlcFrame):
        value: Annotated[int, Int16()] = 0
        big_value: Annotated[int, Int32()] = 0
        name: Annotated[str, FixedStr(10)] = ''
        sub: SubFrame = field(default_factory=SubFrame)
        items: Annotated[List[SubFrame], Repeat(3)] = field(default_factory=list)

    # 序列化
    frame = MyFrame(value=42, big_value=100000, name="hello")
    raw = frame.to_bytes()

    # 反序列化
    frame, offset = MyFrame.from_bytes(raw)

    # 动态列表长度 (覆盖 Repeat 默认值)
    frame, offset = MyFrame.from_bytes(raw, items_count=5)
"""

import struct
from dataclasses import fields
from typing import ClassVar, Union, get_type_hints, get_origin, get_args


class WireType:
    """线协议基本类型基类"""
    fmt: str = ''
    size: int = 0


class Int16(WireType):
    """有符号16位整数 (2字节)"""
    fmt = 'h'
    size = 2


class UInt16(WireType):
    """无符号16位整数 (2字节)"""
    fmt = 'H'
    size = 2


class Int32(WireType):
    """有符号32位整数 (4字节)"""
    fmt = 'i'
    size = 4


class UInt32(WireType):
    """无符号32位整数 (4字节)"""
    fmt = 'I'
    size = 4


class Float32(WireType):
    """32位浮点数 (4字节)"""
    fmt = 'f'
    size = 4


class FixedStr(WireType):
    """定长字符串 (UTF-8编码, \\x00 填充)"""
    def __init__(self, length: int):
        self.fmt = f'{length}s'
        self.size = length
        self.length = length


class Repeat:
    """标记列表字段的默认重复次数，仅在 from_bytes 时使用"""
    def __init__(self, count: int):
        self.count = count


def _extract_wire_meta(hint):
    """从 Annotated 类型中提取 WireType 和 Repeat 元数据"""
    wire_type = None
    repeat = None
    if hasattr(hint, '__metadata__'):
        for meta in hint.__metadata__:
            if isinstance(meta, WireType):
                wire_type = meta
            elif isinstance(meta, Repeat):
                repeat = meta
    return wire_type, repeat


def _unwrap_hint(hint):
    """去除 Annotated / Optional 包装，获取裸类型"""
    # Annotated[X, ...] → X
    if hasattr(hint, '__metadata__'):
        hint = get_args(hint)[0]
    # Optional[X] (即 Union[X, None]) → X
    if get_origin(hint) is Union:
        args = [a for a in get_args(hint) if a is not type(None)]
        if args:
            return args[0]
    return hint


def _is_list_hint(hint):
    """判断类型是否为 List[X]"""
    return get_origin(_unwrap_hint(hint)) is list


def _get_list_elem_type(hint):
    """从 List[X] 中提取元素类型"""
    bare = _unwrap_hint(hint)
    if get_origin(bare) is list:
        args = get_args(bare)
        return args[0] if args else None
    return None


class PlcFrame:
    """
    通用PLC二进制帧基类 — 基于 @dataclass + Annotated 自动序列化

    子类必须同时使用 @dataclass 装饰器。

    字节序控制:
        在顶层帧类中设置 BYTE_ORDER 类属性即可控制整个帧的字节序，
        嵌套子帧自动继承父帧的字节序。
        BYTE_ORDER = '>'  # 大端序 (Big-Endian)
        BYTE_ORDER = '<'  # 小端序 (Little-Endian)

    字段类型注解决定序列化格式:

    基本类型:
        field: Annotated[int, Int16()]       → 有符号16位
        field: Annotated[int, UInt16()]      → 无符号16位
        field: Annotated[int, Int32()]       → 有符号32位
        field: Annotated[int, UInt32()]      → 无符号32位
        field: Annotated[float, Float32()]   → 32位浮点
        field: Annotated[str, FixedStr(N)]   → N字节定长字符串

    嵌套帧 (自动递归):
        field: SubFrame = field(default_factory=SubFrame)

    帧列表:
        field: Annotated[List[SubFrame], Repeat(N)] = field(default_factory=list)
        或 from_bytes() 时通过 kwargs: <field_name>_count=N
    """

    BYTE_ORDER: ClassVar[str] = '>'  # 默认大端序，子类可覆盖

    def to_bytes(self, _byte_order: str = '') -> bytes:
        """
        序列化为 bytes
        Args:
            _byte_order: 内部传播参数，外部调用无需传入，自动使用类属性 BYTE_ORDER
        """
        bo = _byte_order or type(self).BYTE_ORDER
        buf = bytearray()
        hints = get_type_hints(type(self), include_extras=True)

        for f in fields(self):  # type: ignore[arg-type]
            value = getattr(self, f.name)
            hint = hints.get(f.name)
            wire_type, _ = _extract_wire_meta(hint)

            if wire_type is not None:
                # 基本类型字段
                if isinstance(wire_type, FixedStr):
                    raw = (value or '').encode('utf-8')[:wire_type.length]
                    raw = raw.ljust(wire_type.length, b'\x00')
                    buf.extend(struct.pack(f'{bo}{wire_type.fmt}', raw))
                else:
                    buf.extend(struct.pack(f'{bo}{wire_type.fmt}', value or 0))
            elif isinstance(value, PlcFrame):
                # 嵌套 PlcFrame
                buf.extend(value.to_bytes(_byte_order=bo))
            elif isinstance(value, list):
                # List[PlcFrame]
                for item in value:
                    if isinstance(item, PlcFrame):
                        buf.extend(item.to_bytes(_byte_order=bo))

        return bytes(buf)

    @classmethod
    def from_bytes(cls, data: bytes, offset: int = 0, _byte_order: str = '', **kwargs):
        """
        从 bytes 反序列化

        Args:
            data: 原始字节数据
            offset: 起始偏移量
            _byte_order: 内部传播参数，外部调用无需传入，自动使用类属性 BYTE_ORDER
            **kwargs: 动态列表长度, 格式: <字段名>_count=N (覆盖 Repeat 默认值)

        Returns:
            (实例, 下一个偏移量)
        """
        bo = _byte_order or cls.BYTE_ORDER
        hints = get_type_hints(cls, include_extras=True)
        init_kwargs = {}
        idx = offset

        for f in fields(cls):  # type: ignore[arg-type]
            hint = hints.get(f.name)
            wire_type, repeat = _extract_wire_meta(hint)

            if wire_type is not None:
                # 基本类型
                val, = struct.unpack_from(f'{bo}{wire_type.fmt}', data, idx)
                if isinstance(wire_type, FixedStr):
                    val = val.rstrip(b'\x00').decode('utf-8')
                init_kwargs[f.name] = val
                idx += wire_type.size

            elif _is_list_hint(hint):
                # List[SubFrame]
                elem_type = _get_list_elem_type(hint)
                if elem_type is None:
                    continue
                count = kwargs.get(f'{f.name}_count', repeat.count if repeat else 0)
                items = []
                for _ in range(count):
                    sub, idx = elem_type.from_bytes(data, idx, _byte_order=bo, **kwargs)
                    items.append(sub)
                init_kwargs[f.name] = items

            else:
                # 嵌套 PlcFrame
                bare_type = _unwrap_hint(hint)
                if isinstance(bare_type, type) and issubclass(bare_type, PlcFrame):
                    sub, idx = bare_type.from_bytes(data, idx, _byte_order=bo, **kwargs)
                    init_kwargs[f.name] = sub

        return cls(**init_kwargs), idx
