"""帧历史定位、X 轴范围、安全包络和喷枪障碍约束。"""

from collections.abc import Mapping
from dataclasses import replace
from typing import Sequence

import numpy as np

from model.motionplan._validation import (
    _require_int,
    _round_runtime_mm,
)
from model.motionplan.constants import (
    XN_SIDE_X_AXES,
    XN_SIDE_Y_AXES,
    X_AXIS_OFFSET_KEYS,
)
from model.motionplan.contracts import (
    ContourRows,
    DetectedXAxisStroke,
    EdgeCandidate,
    GunObstacleConstraint,
    PlannedXAxisRange,
    ThreeRegionBoxes,
)
from model.motionplan.variable_2d import (
    _leading_edge_window_indices,
)


def calculate_history_index(
    z_position_mm: int,
    frame_step_mm: int,
    stack_size: int,
) -> int | None:
    """计算设备应读取的历史Z帧下标。

    ``frame_stack[0]`` 是激光刚采集的最新帧，下标越大表示越早采集、
    已沿链条正方向移动得越远。函数不猜测队列单位，调用方必须明确传入
    每个队列槽对应的毫米数。

    距离超出队列覆盖范围时返回 ``None``，让上层进入关枪安全状态。
    当前阶段不擅自决定非整格距离的取整方向，因此要求距离与步长整除。
    """

    values = {
        "z_position_mm": z_position_mm,
        "frame_step_mm": frame_step_mm,
        "stack_size": stack_size,
    }
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} 必须是整数")

    if z_position_mm < 0:
        raise ValueError("z_position_mm 不能小于0")
    if frame_step_mm <= 0:
        raise ValueError("frame_step_mm 必须大于0")
    if stack_size <= 0:
        raise ValueError("stack_size 必须大于0")
    if z_position_mm % frame_step_mm != 0:
        raise ValueError("z_position_mm 必须能被 frame_step_mm 整除，取整规则尚未确认")

    history_index = z_position_mm // frame_step_mm
    if history_index >= stack_size:
        return None
    return history_index


def select_history_frame(
    frame_stack,
    z_position_mm: int,
    frame_step_mm: int,
):
    """按设备Z距离读取历史帧；队列覆盖不到时返回 ``None``。"""

    if frame_stack is None:
        raise TypeError("frame_stack 不能是None")

    history_index = calculate_history_index(
        z_position_mm=z_position_mm,
        frame_step_mm=frame_step_mm,
        stack_size=len(frame_stack),
    )
    if history_index is None:
        return None
    return frame_stack[history_index]


def _rectangle_x_reciprocation_start_points(
    boxes: ThreeRegionBoxes,
    side: str,
) -> dict[str, EdgeCandidate]:
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    if boxes.middle is not None:
        in_up_x = boxes.middle.x_min
        in_down_x = boxes.middle.x_min
    else:
        in_up_x = boxes.upper.x_max
        in_down_x = boxes.lower.x_max
    return {
        "out_up": EdgeCandidate(boxes.upper.y_max, boxes.upper.x_max),
        "in_up": EdgeCandidate(boxes.upper.y_min, in_up_x),
        "in_down": EdgeCandidate(boxes.lower.y_max, in_down_x),
        "out_down": EdgeCandidate(boxes.lower.y_min, boxes.lower.x_max),
    }


def _rectangle_x_reciprocation_end_points(
    boxes: ThreeRegionBoxes,
    side: str,
) -> dict[str, EdgeCandidate]:
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    return {
        "out_up": EdgeCandidate(boxes.upper.y_max, boxes.upper.x_min),
        "in_up": EdgeCandidate(boxes.upper.y_min, boxes.upper.x_min),
        "in_down": EdgeCandidate(boxes.lower.y_max, boxes.lower.x_min),
        "out_down": EdgeCandidate(boxes.lower.y_min, boxes.lower.x_min),
    }


def _rectangle_x_axis_strokes(
    boxes: ThreeRegionBoxes,
    side: str,
) -> dict[str, DetectedXAxisStroke]:
    axis_by_surface = {
        "out_down": "x1",
        "in_down": "x2",
        "in_up": "x3",
        "out_up": "x4",
    }
    start_points = _rectangle_x_reciprocation_start_points(boxes, side)
    end_points = _rectangle_x_reciprocation_end_points(boxes, side)
    strokes = {
        axis_by_surface[surface]: DetectedXAxisStroke(
            start_x=start_points[surface].detected_x,
            end_x=end_points[surface].detected_x,
        )
        for surface in axis_by_surface
    }
    return {axis: strokes[axis] for axis in XN_SIDE_X_AXES}


def _x_config_value(machine_config: Mapping, key: str) -> int:
    values = machine_config.get(key)
    if not isinstance(values, (list, tuple)) or len(values) <= 1:
        raise ValueError(f"MachineConfig.{key} 必须包含[y, x]两个值")
    value = values[1]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"MachineConfig.{key} 的X轴值必须是整数")
    return value


def _build_x_axis_ranges(
    boxes: ThreeRegionBoxes,
    side: str,
    machine_config: Mapping,
) -> dict[str, PlannedXAxisRange]:
    if machine_config.get("type") != "xn_side":
        raise ValueError("machine_config 必须是 xn_side 设备配置")
    if machine_config.get("install_orietation") != side:
        raise ValueError("machine_config 的安装方向与 side 不一致")
    x_position = machine_config.get("x_position")
    position_speed = machine_config.get("x_pos_speed")
    reciprocation_speed = machine_config.get("x_recip_speed")
    for name, value in {
        "x_position": x_position,
        "x_pos_speed": position_speed,
        "x_recip_speed": reciprocation_speed,
    }.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"MachineConfig.{name} 必须是整数")

    min_limit_pos = _x_config_value(machine_config, "min_limit_pos")
    max_limit_pos = _x_config_value(machine_config, "max_limit_pos")
    strokes = _rectangle_x_axis_strokes(boxes, side)
    results = {}
    for axis_name, stroke in strokes.items():
        start_x = _round_runtime_mm(stroke.start_x)
        end_x = _round_runtime_mm(stroke.end_x)
        front_offset_key, after_offset_key = X_AXIS_OFFSET_KEYS[axis_name]
        front_offset = machine_config.get(front_offset_key)
        after_offset = machine_config.get(after_offset_key)
        for key, value in (
            (front_offset_key, front_offset),
            (after_offset_key, after_offset),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"MachineConfig.{key} 必须是整数")
        results[axis_name] = PlannedXAxisRange(
            axis_name=axis_name,
            detected_start_x=start_x,
            detected_end_x=end_x,
            start_pos=start_x - x_position - front_offset,
            end_pos=end_x - x_position - after_offset,
            min_limit_pos=min_limit_pos,
            max_limit_pos=max_limit_pos,
            position_speed=position_speed,
            reciprocation_speed=reciprocation_speed,
            status=0,
        )
    return {axis: results[axis] for axis in XN_SIDE_X_AXES}


def _build_minimum_x_envelope(
    motions_by_frame: Sequence[dict[str, PlannedXAxisRange] | None],
    half_window_frames: int,
    *,
    contour_occupied: Sequence[bool] | None = None,
) -> list[dict[str, PlannedXAxisRange] | None]:
    if isinstance(half_window_frames, bool) or not isinstance(half_window_frames, int):
        raise TypeError("half_window_frames 必须是整数")
    if half_window_frames < 0:
        raise ValueError("half_window_frames 不能小于0")
    frame_count = len(motions_by_frame)
    if contour_occupied is not None and len(contour_occupied) != frame_count:
        raise ValueError("轮廓标记与行程帧数不一致")
    envelope = []
    for center_index, center_motions in enumerate(motions_by_frame):
        if center_motions is None:
            envelope.append(None)
            continue
        start_index = max(0, center_index - half_window_frames)
        end_index = min(frame_count, center_index + half_window_frames + 1)
        window = motions_by_frame[start_index:end_index]
        if contour_occupied is not None:
            selected = _leading_edge_window_indices(
                contour_occupied[start_index:end_index],
                center_index - start_index, older_at_end=False,
                allow_trailing_edge=True,
            )
            if selected is None:
                envelope.append(None)
                continue
            window = [window[i] for i in selected]
        if any(frame_motions is None for frame_motions in window):
            envelope.append(None)
            continue
        safe_motions = {}
        for axis_name in XN_SIDE_X_AXES:
            candidates = [frame[axis_name] for frame in window if frame is not None]
            safe_motions[axis_name] = replace(
                center_motions[axis_name],
                window_min_detected_x=min(
                    item.window_min_detected_x
                    if item.window_min_detected_x is not None
                    else min(item.detected_start_x, item.detected_end_x)
                    for item in candidates
                ),
                start_pos=min(item.start_pos for item in candidates),
                end_pos=min(item.end_pos for item in candidates),
            )
        envelope.append(safe_motions)
    return envelope


def _build_minimum_x_envelope_at(
    motions_by_frame,
    half_window_frames,
    center_index,
    *,
    contour_occupied=None,
):
    """仅求指定中心的包络；窗口选择与完整序列算法保持一致。"""
    if isinstance(half_window_frames, bool) or not isinstance(half_window_frames, int):
        raise TypeError("half_window_frames 必须是整数")
    if half_window_frames < 0:
        raise ValueError("half_window_frames 不能小于0")
    if contour_occupied is not None and len(contour_occupied) != len(motions_by_frame):
        raise ValueError("轮廓标记与行程帧数不一致")
    center_motions = motions_by_frame[center_index]
    if center_motions is None:
        return None
    start = max(0, center_index - half_window_frames)
    stop = min(len(motions_by_frame), center_index + half_window_frames + 1)
    window = motions_by_frame[start:stop]
    if contour_occupied is not None:
        selected = _leading_edge_window_indices(
            contour_occupied[start:stop], center_index - start,
            older_at_end=False, allow_trailing_edge=True,
        )
        if selected is None:
            return None
        window = [window[i] for i in selected]
    if any(frame is None for frame in window):
        return None
    result = {}
    for axis in XN_SIDE_X_AXES:
        candidates = [frame[axis] for frame in window]
        result[axis] = replace(
            center_motions[axis],
            window_min_detected_x=min(
                item.window_min_detected_x
                if item.window_min_detected_x is not None
                else min(item.detected_start_x, item.detected_end_x)
                for item in candidates
            ),
            start_pos=min(item.start_pos for item in candidates),
            end_pos=min(item.end_pos for item in candidates),
        )
    return result


def _apply_window_bottom(
    boxes: ThreeRegionBoxes, window_boxes: Sequence[ThreeRegionBoxes],
) -> ThreeRegionBoxes:
    """以有效安全窗口最低的下外边构造正式框，保留输入框不变。"""
    return replace(
        boxes,
        lower=replace(boxes.lower, y_min=min(item.lower.y_min for item in window_boxes)),
    )


def _obstacle_min_x_at_height(
    rows: ContourRows, target_height: int, half_height_mm: int,
) -> int | None:
    """按绝对激光高度取闭区间内原始轮廓最小X，不做类别过滤。"""
    _require_int("target_height", target_height)
    _require_int("gun_obstacle_half_height_mm", half_height_mm)
    if half_height_mm < 0:
        raise ValueError("gun_obstacle_half_height_mm 不能小于0")
    mask = (rows.height >= target_height - half_height_mm) & (
        rows.height <= target_height + half_height_mm
    )
    return _round_runtime_mm(float(np.min(rows.v_min[mask]))) if np.any(mask) else None


def _apply_gun_obstacle_constraints(
    motions: Mapping[str, PlannedXAxisRange] | None,
    rows_by_frame: Sequence[ContourRows],
    center_index: int,
    y_targets: Mapping[str, int],
    machine_config: Mapping,
    half_height_mm: int,
) -> tuple[Mapping[str, PlannedXAxisRange] | None, tuple[GunObstacleConstraint, ...]]:
    if not 0 <= center_index < len(rows_by_frame):
        raise ValueError("center_index 超出安全窗口范围")
    if set(y_targets) != set(XN_SIDE_Y_AXES):
        raise ValueError("y_targets 必须且只能包含 y1、y2、y3")
    if motions is not None and set(motions) != set(XN_SIDE_X_AXES):
        raise ValueError("motions 必须且只能包含 x1、x2、x3、x4")
    origins = machine_config.get("origin_pos")
    if not isinstance(origins, (list, tuple)) or len(origins) != 4:
        raise ValueError("origin_pos 必须包含四把枪的Y零点")
    x_position = _require_int("x_position", machine_config.get("x_position"))
    result = None if motions is None else dict(motions)
    diagnostics = []
    for index, axis in enumerate(XN_SIDE_X_AXES):
        height = _require_int("origin_pos", origins[index])
        if index < 3:
            height += _require_int(f"y{index + 1}", y_targets[f"y{index + 1}"])
        values = tuple(
            _obstacle_min_x_at_height(rows, height, half_height_mm)
            for rows in rows_by_frame
        )
        known = [value for value in values if value is not None]
        obstacle_x = min(known) if known else None
        offset_key = X_AXIS_OFFSET_KEYS[axis][0]
        offset = _require_int(offset_key, machine_config.get(offset_key))
        limit = None if obstacle_x is None else obstacle_x - x_position - offset
        original = None if motions is None else motions[axis]
        adjusted = original
        constrained = collapsed = False
        if original is not None and limit is not None:
            low, high = sorted((original.start_pos, original.end_pos))
            constrained = limit < high
            collapsed = constrained and limit < low
            if collapsed:
                adjusted = replace(original, start_pos=limit, end_pos=limit)
            elif constrained:
                adjusted = (
                    replace(original, start_pos=limit)
                    if original.start_pos >= original.end_pos
                    else replace(original, end_pos=limit)
                )
            result[axis] = adjusted
        diagnostics.append(GunObstacleConstraint(
            axis_name=axis, target_height=height,
            height_min=height - half_height_mm, height_max=height + half_height_mm,
            window_min_x=values, center_min_x=values[center_index],
            obstacle_min_x=obstacle_x, limit_pos=limit,
            laser_x_offset=x_position + offset,
            original_start_pos=None if original is None else original.start_pos,
            original_end_pos=None if original is None else original.end_pos,
            final_start_pos=None if adjusted is None else adjusted.start_pos,
            final_end_pos=None if adjusted is None else adjusted.end_pos,
            constrained=constrained, collapsed=collapsed,
        ))
    return result, tuple(diagnostics)
