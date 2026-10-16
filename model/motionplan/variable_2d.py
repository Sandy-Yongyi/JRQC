"""二维安全窗口观测与边界报文保持状态机。"""

from collections.abc import Mapping
from typing import Sequence

import numpy as np

from model.motionplan._validation import (
    _require_int,
    _round_runtime_mm,
)
from model.motionplan.constants import (
    OUT_LIFT_LIMIT_INDEX,
    VARIABLE_2D_WINDOW_EMPTY,
    VARIABLE_2D_WINDOW_MIXED,
    VARIABLE_2D_WINDOW_VALID,
)
from model.motionplan.contours import (
    _extract_contour_rows,
)
from model.motionplan.contracts import (
    Variable2DWindowObservation,
)
from model.plc.MovingFrameData import AxisData as MovingAxisData


class Variable2DReportStateMachine:
    """维护2D上下限报文，避免窗口空洞打断PLC端Y往复方向。

    线协议只发送 ``XPos/YPosMin/YPosMax``，没有Y当前位置或方向字段；
    到达上下端点后的换向由PLC状态机负责。上位机在窗口夹有空帧时
    必须持续发送上一组有效边界，不能插入零报文导致PLC端突然换向。
    """

    def __init__(self) -> None:
        self.last_command: MovingAxisData | None = None
        self.phase = VARIABLE_2D_WINDOW_EMPTY

    @staticmethod
    def _copy(command: MovingAxisData) -> MovingAxisData:
        return MovingAxisData(
            Pos=command.Pos,
            Speed=command.Speed,
            Status=command.Status,
        )

    def reset(self) -> None:
        self.last_command = None
        self.phase = VARIABLE_2D_WINDOW_EMPTY

    def advance(
        self,
        observation: Variable2DWindowObservation,
    ) -> MovingAxisData | None:
        if not isinstance(observation, Variable2DWindowObservation):
            raise TypeError("observation 必须是 Variable2DWindowObservation")
        if observation.state == VARIABLE_2D_WINDOW_EMPTY:
            self.reset()
            return None
        if observation.state == VARIABLE_2D_WINDOW_MIXED:
            self.phase = VARIABLE_2D_WINDOW_MIXED
            return None if self.last_command is None else self._copy(self.last_command)
        if observation.state != VARIABLE_2D_WINDOW_VALID:
            raise ValueError("未知2D安全窗口状态")
        if not isinstance(observation.command, MovingAxisData):
            raise TypeError("有效2D窗口必须包含PLC AxisData")

        self.last_command = self._copy(observation.command)
        self.phase = VARIABLE_2D_WINDOW_VALID
        return self._copy(self.last_command)


def _out_lift_limit(machine_config: Mapping, key: str, axis_name: str) -> int:
    if axis_name not in OUT_LIFT_LIMIT_INDEX:
        raise ValueError("out_lift轴名称必须是y或x")
    values = machine_config.get(key)
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise ValueError(f"out_lift.{key} 必须按[y,x]包含两个轴限位")
    index = OUT_LIFT_LIMIT_INDEX[axis_name]
    value = values[index]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"out_lift.{key}[{index}] 必须是整数")
    return value


def _out_lift_origin_y(machine_config: Mapping) -> int:
    values = machine_config.get("origin_pos")
    if not isinstance(values, (list, tuple)) or len(values) != 1:
        raise ValueError("out_lift.origin_pos 必须只包含一个Y轴原点")
    value = values[0]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("out_lift.origin_pos[0] 必须是整数")
    return value


def _clamp_axis_position(value: int, minimum: int, maximum: int) -> int:
    if minimum > maximum:
        raise ValueError("min_limit_pos 不能大于 max_limit_pos")
    return max(minimum, min(maximum, value))


def _leading_edge_window_indices(
    occupied: Sequence[bool], center_index: int, *, older_at_end: bool,
    allow_trailing_edge: bool = False,
) -> tuple[int, ...] | None:
    """裁掉前缘空白；云雀可额外裁掉尾缘空白，中心和内部必须有效。"""
    if not 0 <= center_index < len(occupied):
        raise ValueError("中心帧超出窗口范围")
    indices = list(range(len(occupied)))
    if not older_at_end:
        indices.reverse()
    while indices and not occupied[indices[-1]]:
        indices.pop()
    if allow_trailing_edge:
        while indices and not occupied[indices[0]]:
            indices.pop(0)
    if center_index not in indices or not all(occupied[i] for i in indices):
        return None
    return tuple(sorted(indices))


def build_variable_2d_window_observation(
    frames: Sequence[object],
    machine_config: Mapping,
    *,
    _row_extractor=None,
    station_index: int | None = None,
) -> Variable2DWindowObservation:
    """把2D设备安全窗口换算成 ``XPos/YPosMin/YPosMax``。

    X与Y最小值由本侧激光坐标换算为设备本地坐标；X减去外前偏移，
    不使用外下、外上偏移，Y最大值固定使用设备最大限位。窗口全空表示
    工件不在窗口内。运行时显式提供station_index，允许来料前缘提前
    定位；仅出料侧有工件或窗口内部有空洞时返回mixed以保持上一范围。
    不传station_index的旧离线调用保留对称窗口、中心到站判定。
    """

    if not isinstance(frames, Sequence) or isinstance(frames, (str, bytes)):
        raise TypeError("frames 必须是帧序列")
    if not frames:
        raise ValueError("2D安全窗口不能为空")
    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "out_lift":
        raise ValueError("machine_config 必须是 out_lift 设备配置")

    extract = _extract_contour_rows if _row_extractor is None else _row_extractor
    rows_by_frame = [extract(frame) for frame in frames]
    nonempty_rows = [rows for rows in rows_by_frame if len(rows) > 0]
    if not nonempty_rows:
        return Variable2DWindowObservation(VARIABLE_2D_WINDOW_EMPTY)
    if station_index is None:
        selected = _leading_edge_window_indices(
            [len(rows) > 0 for rows in rows_by_frame],
            len(frames) // 2, older_at_end=True,
        )
    else:
        if not 0 <= station_index < len(frames):
            raise ValueError("站位索引超出二维安全窗口")
        occupied = [len(rows) > 0 for rows in rows_by_frame]
        reference = station_index if occupied[station_index] else next(
            (i for i in range(station_index - 1, -1, -1) if occupied[i]), None,
        )
        selected = None if reference is None else _leading_edge_window_indices(
            occupied, reference, older_at_end=True, allow_trailing_edge=True,
        )
    if selected is None:
        return Variable2DWindowObservation(VARIABLE_2D_WINDOW_MIXED)

    x_position = machine_config.get("x_position")
    if isinstance(x_position, bool) or not isinstance(x_position, int):
        raise TypeError("out_lift.x_position 必须是整数")
    front_offset = _require_int(
        "out_lift.out_front_x_offset", machine_config.get("out_front_x_offset")
    )

    y_minimum_limit = _out_lift_limit(machine_config, "min_limit_pos", "y")
    y_maximum_limit = _out_lift_limit(machine_config, "max_limit_pos", "y")
    x_minimum_limit = _out_lift_limit(machine_config, "min_limit_pos", "x")
    x_maximum_limit = _out_lift_limit(machine_config, "max_limit_pos", "x")
    origin_y = _out_lift_origin_y(machine_config)

    detected_x_min = min(float(np.min(rows.v_min)) for rows in nonempty_rows)
    detected_y_min = min(float(np.min(rows.height)) for rows in nonempty_rows)

    x_pos = _round_runtime_mm(detected_x_min) - x_position - front_offset
    y_pos_min = _round_runtime_mm(detected_y_min) - origin_y
    y_pos_max = y_maximum_limit

    x_pos = _clamp_axis_position(x_pos, x_minimum_limit, x_maximum_limit)
    y_pos_min = _clamp_axis_position(
        y_pos_min,
        y_minimum_limit,
        y_maximum_limit,
    )
    y_pos_max = _clamp_axis_position(
        y_pos_max,
        y_minimum_limit,
        y_maximum_limit,
    )
    if y_pos_min > y_pos_max:
        raise ValueError("2D设备换算后的YPosMin不能大于YPosMax")

    return Variable2DWindowObservation(
        VARIABLE_2D_WINDOW_VALID,
        MovingAxisData(Pos=x_pos, Speed=y_pos_min, Status=y_pos_max),
        leading_edge=len(selected) < len(frames),
    )
