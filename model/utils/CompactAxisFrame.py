"""PLC历史帧的只读紧凑存储，避免保留数十万个逐行对象及字典。

入队帧作为快照使用。FrameData仍支持长度、索引和迭代，读取一行时才
构造AxisData；标量保持原类型，不舍入、不筛除空行、不改变行序。
"""

from collections.abc import Sequence
from copy import copy

from model.formats.frame_by_frame.AxisFrameDataFormat import AxisData, AxisFrameData


class CompactAxisRows(Sequence):
    __slots__ = ("_values",)

    def __init__(self, values):
        self._values = tuple(values)
        if len(self._values) % 3:
            raise ValueError("每行必须包含H_Axis、V_Axis_Max、V_Axis_Min")

    @classmethod
    def zeros(cls, count):
        return cls((0, 0, 0) * count)

    def __len__(self):
        return len(self._values) // 3

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        if index < 0:
            index += len(self)
        if not 0 <= index < len(self):
            raise IndexError(index)
        offset = index * 3
        return AxisData(
            H_Axis=self._values[offset],
            V_Axis_Max=self._values[offset + 1],
            V_Axis_Min=self._values[offset + 2],
        )

    def __eq__(self, other):
        if isinstance(other, CompactAxisRows):
            return self._values == other._values
        if isinstance(other, Sequence):
            return list(self) == list(other)
        return NotImplemented


def compact_axis_frame(frame):
    """复制帧外壳，释放历史存储对原逐行对象的引用；保留未知格式。"""
    if not isinstance(frame, AxisFrameData):
        return frame
    rows = frame.FrameData
    if rows is None or isinstance(rows, CompactAxisRows):
        return frame
    # 仅压缩已知线性数据；扩展行类型/额外字段继续走原存储路径。
    values = []
    for row in rows:
        if type(row) is not AxisData or set(vars(row)) != {"H_Axis", "V_Axis_Max", "V_Axis_Min"}:
            return frame
        scalars = (row.H_Axis, row.V_Axis_Max, row.V_Axis_Min)
        if any(type(value) not in (int, float, type(None)) for value in scalars):
            return frame
        values.extend(scalars)
    result = copy(frame)
    result.FrameData = CompactAxisRows(values)
    return result
