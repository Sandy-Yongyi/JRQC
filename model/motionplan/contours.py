"""轮廓提取、折边识别、边缘稳定和三区域矩形构建。"""

from typing import Sequence
from dataclasses import replace
import math

import numpy as np

from model.motionplan.contracts import (
    ContourRows,
    EdgeCandidate,
    FrameEdgeCandidates,
    RegionBox,
    ThreeRegionBoxes,
)
from model.motionplan._validation import _require_int


def _maximum_y_gap(y_threshold: float, maximum_missing_y_layers: int) -> float:
    """连续性容差与采样步长分开；不插值、不增加原始轮廓层。"""
    missing = _require_int("maximum_missing_y_layers", maximum_missing_y_layers)
    if missing < 0:
        raise ValueError("maximum_missing_y_layers 不能小于0")
    return max(float(y_threshold) * max(1.5, missing + 1), 1.0)


def _validate_contour_gaps(
    y_threshold: float, minimum_rail_gap_mm: int, maximum_missing_y_layers: int,
) -> float:
    minimum = _require_int("minimum_rail_gap_mm", minimum_rail_gap_mm)
    if minimum < 0:
        raise ValueError("minimum_rail_gap_mm 不能小于0")
    maximum_gap = _maximum_y_gap(y_threshold, maximum_missing_y_layers)
    if minimum and maximum_gap >= minimum:
        raise ValueError("允许缺层的最大间隔必须小于横梁最小净距")
    return maximum_gap


def _enforce_minimum_rail_gap(
    edges: FrameEdgeCandidates, minimum_rail_gap_mm: int,
) -> FrameEdgeCandidates:
    """净距检查针对扩展前的真实内边候选，不能用规划矩形的间距代替。"""
    minimum = _require_int("minimum_rail_gap_mm", minimum_rail_gap_mm)
    if minimum < 0:
        raise ValueError("minimum_rail_gap_mm 不能小于0")
    if (
        edges.in_up is not None and edges.in_down is not None
        and edges.in_up.height - edges.in_down.height < minimum
    ):
        return replace(edges, in_up=None, in_down=None, detection_mode="rail_gap_too_small")
    return edges


def _extract_contour_rows(axis_frame: object) -> ContourRows:
    rows = []
    for item in getattr(axis_frame, "FrameData", None) or ():
        try:
            height = float(item.H_Axis)
            v_min = float(item.V_Axis_Min)
            v_max = float(item.V_Axis_Max)
        except (AttributeError, TypeError, ValueError):
            continue
        if height == 0 or (v_min == 0 and v_max == 0):
            continue
        if not np.isfinite((height, v_min, v_max)).all() or v_min > v_max:
            continue
        rows.append((height, v_min, v_max))

    if not rows:
        empty = np.empty(0, dtype=float)
        return ContourRows(empty, empty.copy(), empty.copy())
    values = np.asarray(rows, dtype=float)
    values = values[np.argsort(values[:, 0], kind="stable")]
    return ContourRows(values[:, 0], values[:, 1], values[:, 2])


def _split_contiguous_rows(
    rows: ContourRows,
    y_threshold: float,
    maximum_missing_y_layers: int = 0,
) -> list[ContourRows]:
    if len(rows) == 0:
        return []
    maximum_gap = _maximum_y_gap(y_threshold, maximum_missing_y_layers)
    height_gaps = np.diff(rows.height)
    split_at = np.flatnonzero((height_gaps <= 0) | (height_gaps > maximum_gap)) + 1
    indices = np.split(np.arange(len(rows)), split_at)
    return [
        ContourRows(rows.height[index], rows.v_min[index], rows.v_max[index])
        for index in indices
        if index.size > 0
    ]


def _median_smooth(values: np.ndarray) -> np.ndarray:
    """三点中值滤波去除孤立噪声"""
    values = np.asarray(values, dtype=float)
    if values.size < 3:
        return values.copy()
    padded = np.pad(values, (1, 1), mode="edge")
    return np.asarray(
        [np.median(padded[index : index + 3]) for index in range(values.size)],
        dtype=float,
    )


def _last_true_run_start(mask: np.ndarray, minimum_run_layers: int) -> int | None:
    true_indices = np.flatnonzero(np.asarray(mask, dtype=bool))
    if true_indices.size == 0:
        return None
    runs = np.split(true_indices, np.flatnonzero(np.diff(true_indices) > 1) + 1)
    valid_runs = [run for run in runs if run.size >= minimum_run_layers]
    return None if not valid_runs else int(valid_runs[-1][0])


def _isolated_rail_rows(
    rows: ContourRows, y_threshold: float, minimum_layers: int,
    minimum_rail_gap_mm: int = 0, maximum_missing_y_layers: int = 0,
) -> tuple[ContourRows, ContourRows] | None:
    """两段之间的空隙大于各段高度时，按完整的上下横梁处理。

    小缺层及带长竖梁的轮廓仍交给原有宽度转折识别。
    """
    segments = _split_contiguous_rows(rows, y_threshold, maximum_missing_y_layers)
    if len(segments) != 2 or any(len(s) < max(2, minimum_layers) for s in segments):
        return None
    lower, upper = segments
    gap = upper.height[0] - lower.height[-1]
    if gap < minimum_rail_gap_mm:
        return None
    if gap <= max(
        _maximum_y_gap(y_threshold, maximum_missing_y_layers),
        lower.height[-1] - lower.height[0],
        upper.height[-1] - upper.height[0],
    ):
        return None
    return lower, upper


def _validate_edge_detection_parameters(
    y_threshold: float,
    minimum_x_change_mm: float,
    minimum_run_layers: int,
    minimum_horizontal_layers: int | None,
) -> int:
    """校验边缘检测参数，并补齐水平段的最小层数。"""
    if (
        isinstance(y_threshold, bool)
        or not math.isfinite(y_threshold)
        or y_threshold <= 0
    ):
        raise ValueError("y_threshold 必须是大于0的有限数")
    if isinstance(minimum_x_change_mm, bool):
        raise TypeError("minimum_x_change_mm 不能是布尔值")
    if not math.isfinite(minimum_x_change_mm) or minimum_x_change_mm <= 0:
        raise ValueError("minimum_x_change_mm 必须是大于0的有限数")
    if isinstance(minimum_run_layers, bool) or not isinstance(minimum_run_layers, int):
        raise TypeError("minimum_run_layers 必须是整数")
    if minimum_run_layers <= 0:
        raise ValueError("minimum_run_layers 必须大于0")
    if minimum_horizontal_layers is None:
        minimum_horizontal_layers = minimum_run_layers
    if isinstance(minimum_horizontal_layers, bool) or not isinstance(
        minimum_horizontal_layers, int
    ):
        raise TypeError("minimum_horizontal_layers 必须是整数")
    if minimum_horizontal_layers <= 0:
        raise ValueError("minimum_horizontal_layers 必须大于0")
    return minimum_horizontal_layers


def _folded_terminal_rail_indices(
    rows: ContourRows, y_threshold: float, change_mm: float, minimum_layers: int,
    maximum_missing_y_layers: int = 0,
) -> tuple[int, int] | None:
    """识别连续竖杆两端的折弯横梁；竖杆背面的凸起不作为内边界。

    V_Min 是左右侧共同的迎枪表面。折弯可表现为整层向较小X平移，
    因而不能仅按 V_Max-V_Min 判定。合并100mm内的回折证据，逐组验证
    整体平移和端部归属；仅跨越配置允许的小缺层。
    """
    required = max(2, minimum_layers)
    if len(rows) < max(12, required * 4):
        return None
    if np.any(np.diff(rows.height) > _maximum_y_gap(y_threshold, maximum_missing_y_layers)):
        return None
    start, stop = int(len(rows) * .35), int(math.ceil(len(rows) * .65))
    reference = float(np.median(rows.v_min[start:stop]))
    core = rows.v_min[start:stop]
    if np.mean(np.abs(core - reference) < change_mm / 2) < .8:
        return None
    surface = _median_smooth(rows.v_min)
    shifted = np.flatnonzero(surface <= reference - change_mm)
    if shifted.size == 0:
        return None
    recovered = np.abs(surface - reference) < change_mm / 2
    groups = np.split(
        shifted, np.flatnonzero(np.diff(rows.height[shifted]) > 100.0) + 1,
    )
    qualified = []
    for group in groups:
        # 整层偏移的连续证据，排除单层噪声及仅宽度增加的普通横梁。
        translated = rows.v_max[group] < reference - change_mm / 2
        indices = group[translated]
        runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
        if any(run.size >= required for run in runs):
            qualified.append(group)
    # 碎片证据不足时继续寻找主折弯；已被稳定竖杆隔开的中间附件
    # 不能借助最外沿的碎片冒充端部横梁。
    lower = next((
        group for group in qualified if group[-1] < start
        and _last_true_run_start(recovered[:group[0]], required) is None
    ), None)
    upper = next((
        group for group in reversed(qualified) if group[0] >= stop
        and _last_true_run_start(recovered[group[-1] + 1:], required) is None
    ), None)
    if lower is None or upper is None:
        return None
    lower_end, upper_start = int(lower[-1]), int(upper[0])
    # 用较小的恢复阈值保留折弯过渡，直到重新贴近稳定竖杆。
    while lower_end + 1 < start and surface[lower_end + 1] < reference - change_mm / 2:
        lower_end += 1
    while upper_start - 1 >= stop and surface[upper_start - 1] < reference - change_mm / 2:
        upper_start -= 1
    return lower_end, upper_start


def _detect_frame_edge_candidates(
    rows: ContourRows,
    y_threshold: float = 10.0,
    minimum_x_change_mm: float = 100,
    minimum_run_layers: int = 3,
    minimum_horizontal_layers: int | None = None,
    minimum_rail_gap_mm: int = 0,
    maximum_missing_y_layers: int = 0,
) -> FrameEdgeCandidates:
    """识别四边；旧直接调用可用0保留通用小尺寸轮廓合同。"""
    _validate_contour_gaps(y_threshold, minimum_rail_gap_mm, maximum_missing_y_layers)
    edges = _detect_frame_edge_candidates_core(
        rows, y_threshold, minimum_x_change_mm, minimum_run_layers,
        minimum_horizontal_layers, minimum_rail_gap_mm, maximum_missing_y_layers,
    )
    return _enforce_minimum_rail_gap(edges, minimum_rail_gap_mm)


def _detect_frame_edge_candidates_core(
    rows: ContourRows,
    y_threshold: float,
    minimum_x_change_mm: float,
    minimum_run_layers: int,
    minimum_horizontal_layers: int | None,
    minimum_rail_gap_mm: int,
    maximum_missing_y_layers: int,
) -> FrameEdgeCandidates:
    minimum_horizontal_layers = _validate_edge_detection_parameters(
        y_threshold,
        minimum_x_change_mm,
        minimum_run_layers,
        minimum_horizontal_layers,
    )
    if len(rows) == 0:
        return FrameEdgeCandidates(
            None, None, None, None, float(minimum_x_change_mm), "empty"
        )

    out_up = EdgeCandidate(float(rows.height[-1]), float(rows.v_min[-1]))
    out_down = EdgeCandidate(float(rows.height[0]), float(rows.v_min[0]))
    isolated = _isolated_rail_rows(
        rows, y_threshold, minimum_horizontal_layers,
        minimum_rail_gap_mm, maximum_missing_y_layers,
    )
    if isolated is not None:
        lower, upper = isolated
        return FrameEdgeCandidates(
            out_up,
            EdgeCandidate(float(upper.height[0]), float(upper.v_min[0])),
            EdgeCandidate(float(lower.height[-1]), float(lower.v_min[-1])),
            out_down, float(minimum_x_change_mm), "separated_segments",
        )
    segments = [
        segment
        for segment in _split_contiguous_rows(rows, y_threshold, maximum_missing_y_layers)
        if len(segment) >= minimum_horizontal_layers
    ]
    # 缺测产生的短断口不能单独作为上下横梁分离的证据。
    separated = any(
        upper.height[0] - lower.height[-1] >= minimum_rail_gap_mm
        for lower, upper in zip(segments, segments[1:])
    )
    if len(segments) >= 2 and separated:
        bottom_segment = segments[0]
        top_segment = segments[-1]
        in_up_index = 0
        top_span = float(top_segment.height[-1] - top_segment.height[0])
        connector_span = float(y_threshold) * max(10, minimum_run_layers * 3)
        detection_mode = "separated_segments"
        if top_span >= connector_span:
            widths = top_segment.v_max - top_segment.v_min
            wide_run_start = _last_true_run_start(
                widths >= float(minimum_x_change_mm),
                minimum_horizontal_layers,
            )
            if wide_run_start is None:
                return FrameEdgeCandidates(
                    out_up,
                    None,
                    EdgeCandidate(
                        float(bottom_segment.height[-1]),
                        float(bottom_segment.v_min[-1]),
                    ),
                    out_down,
                    float(minimum_x_change_mm),
                    "connector_ambiguous",
                )
            in_up_index = wide_run_start
            detection_mode = "upper_width_transition"
        in_up = EdgeCandidate(
            float(top_segment.height[in_up_index]),
            float(top_segment.v_min[in_up_index]),
        )
        in_down = EdgeCandidate(
            float(bottom_segment.height[-1]),
            float(bottom_segment.v_min[-1]),
        )
        if in_up.height > in_down.height:
            return FrameEdgeCandidates(
                out_up,
                in_up,
                in_down,
                out_down,
                float(minimum_x_change_mm),
                detection_mode,
            )

    if len(rows) < max(7, minimum_horizontal_layers * 3):
        return FrameEdgeCandidates(
            out_up,
            None,
            None,
            out_down,
            float(minimum_x_change_mm),
            "insufficient",
        )

    folded = _folded_terminal_rail_indices(
        rows, y_threshold, minimum_x_change_mm, minimum_run_layers,
        maximum_missing_y_layers,
    )
    if folded is not None:
        lower_end, upper_start = folded
        return FrameEdgeCandidates(
            out_up,
            EdgeCandidate(float(rows.height[upper_start]), float(rows.v_min[upper_start])),
            EdgeCandidate(float(rows.height[lower_end]), float(rows.v_min[lower_end])),
            out_down, float(minimum_x_change_mm), "folded_terminal_rails",
        )

    smooth_min = _median_smooth(rows.v_min)
    smooth_max = _median_smooth(rows.v_max)
    count = len(rows)
    middle_start = max(1, int(count * 0.35))
    middle_stop = min(
        count - 1,
        max(middle_start + 1, int(math.ceil(count * 0.65))),
    )
    middle_slice = slice(middle_start, middle_stop)
    reference_min = float(np.median(smooth_min[middle_slice]))
    reference_max = float(np.median(smooth_max[middle_slice]))
    change_score = np.maximum(
        np.abs(smooth_min - reference_min),
        np.abs(smooth_max - reference_max),
    )
    middle_score = change_score[middle_slice]
    middle_median = float(np.median(middle_score))
    middle_mad = float(np.median(np.abs(middle_score - middle_median)))
    threshold = max(
        float(minimum_x_change_mm),
        middle_median + 4.0 * 1.4826 * middle_mad,
    )
    changed = change_score >= threshold

    in_down_index = None
    for index in range(
        middle_start - 1,
        minimum_horizontal_layers - 2,
        -1,
    ):
        run_start = index - minimum_horizontal_layers + 1
        if np.all(changed[run_start : index + 1]):
            in_down_index = index
            break
    in_up_index = None
    last_start = count - minimum_horizontal_layers
    for index in range(middle_stop, last_start + 1):
        if np.all(changed[index : index + minimum_horizontal_layers]):
            in_up_index = index
            break

    in_down = (
        EdgeCandidate(
            float(rows.height[in_down_index]),
            float(rows.v_min[in_down_index]),
        )
        if in_down_index is not None
        else None
    )
    in_up = (
        EdgeCandidate(
            float(rows.height[in_up_index]),
            float(rows.v_min[in_up_index]),
        )
        if in_up_index is not None
        else None
    )
    if in_up is not None and in_down is not None and in_up.height <= in_down.height:
        in_up = None
        in_down = None
    return FrameEdgeCandidates(
        out_up,
        in_up,
        in_down,
        out_down,
        threshold,
        "profile_change",
    )


def _detect_frame_edge_sequence(
    rows_by_frame: Sequence[ContourRows],
    y_threshold: float = 10.0,
    minimum_x_change_mm: float = 100,
    minimum_run_layers: int = 3,
    minimum_horizontal_layers: int | None = None,
    minimum_rail_gap_mm: int = 0,
    maximum_missing_y_layers: int = 0,
) -> list[FrameEdgeCandidates]:
    results = [
        _detect_frame_edge_candidates(
            rows,
            y_threshold=y_threshold,
            minimum_x_change_mm=minimum_x_change_mm,
            minimum_run_layers=minimum_run_layers,
            minimum_horizontal_layers=minimum_horizontal_layers,
            minimum_rail_gap_mm=minimum_rail_gap_mm,
            maximum_missing_y_layers=maximum_missing_y_layers,
        )
        for rows in rows_by_frame
    ]
    if not results:
        return []
    completed_values = [
        {
            "out_up": result.out_up,
            "in_up": result.in_up,
            "in_down": result.in_down,
            "out_down": result.out_down,
        }
        for result in results
    ]
    filled_indices: set[int] = set()
    frame_indices = np.arange(len(results), dtype=float)
    for edge_name in ("in_up", "in_down"):
        known_indices = np.asarray(
            [
                index
                for index, values in enumerate(completed_values)
                if values[edge_name] is not None
            ],
            dtype=int,
        )
        if known_indices.size == 0:
            continue
        known_heights = np.asarray(
            [completed_values[index][edge_name].height for index in known_indices],
            dtype=float,
        )
        guided_heights = np.interp(frame_indices, known_indices, known_heights)
        for index, rows in enumerate(rows_by_frame):
            if completed_values[index][edge_name] is not None or len(rows) == 0:
                continue
            nearest_row = int(np.argmin(np.abs(rows.height - guided_heights[index])))
            completed_values[index][edge_name] = EdgeCandidate(
                float(rows.height[nearest_row]),
                float(rows.v_min[nearest_row]),
            )
            filled_indices.add(index)

    completed_results = []
    for index, (result, values) in enumerate(zip(results, completed_values)):
        in_up = values["in_up"]
        in_down = values["in_down"]
        out_up = values["out_up"]
        out_down = values["out_down"]
        if (
            out_up is not None
            and in_up is not None
            and in_down is not None
            and out_down is not None
            and not (out_up.height >= in_up.height > in_down.height >= out_down.height)
        ):
            in_up = None
            in_down = None
        completed_results.append(
            _enforce_minimum_rail_gap(
                FrameEdgeCandidates(
                    out_up,
                    in_up,
                    in_down,
                    out_down,
                    result.change_threshold,
                    "neighbor_guided" if index in filled_indices else result.detection_mode,
                ),
                minimum_rail_gap_mm,
            )
        )
    return completed_results


def _build_three_region_boxes(
    rows: ContourRows,
    edges: FrameEdgeCandidates,
    y_threshold: float = 10.0,
    minimum_middle_coverage: float = 0.3,
    horizontal_width_mm: float = 50.0,
    minimum_horizontal_layers: int = 4,
    rail_padding_layers: int = 4,
    middle_trim_layers: int = 4,
    minimum_rail_gap_mm: int = 0,
    maximum_missing_y_layers: int = 0,
) -> ThreeRegionBoxes | None:
    edges = _enforce_minimum_rail_gap(edges, minimum_rail_gap_mm)
    if len(rows) == 0 or not edges.is_complete:
        return None
    if not math.isfinite(y_threshold) or y_threshold <= 0:
        raise ValueError("y_threshold 必须是大于0的有限数")
    if (
        not math.isfinite(minimum_middle_coverage)
        or not 0 <= minimum_middle_coverage <= 1
    ):
        raise ValueError("minimum_middle_coverage 必须在0到1之间")
    if not math.isfinite(horizontal_width_mm) or horizontal_width_mm <= 0:
        raise ValueError("horizontal_width_mm 必须是大于0的有限数")
    if (
        minimum_horizontal_layers <= 0
        or rail_padding_layers < 0
        or middle_trim_layers < 0
    ):
        raise ValueError("横梁连续层数必须大于0，扩展和收缩层数不能小于0")

    def make_box_from_indices(indices: np.ndarray) -> RegionBox | None:
        if indices.size == 0:
            return None
        return RegionBox(
            x_min=float(np.min(rows.v_min[indices])),
            x_max=float(np.max(rows.v_max[indices])),
            y_min=float(np.min(rows.height[indices])),
            y_max=float(np.max(rows.height[indices])),
        )

    maximum_gap = _validate_contour_gaps(
        y_threshold, minimum_rail_gap_mm, maximum_missing_y_layers,
    )
    isolated = _isolated_rail_rows(
        rows, y_threshold, minimum_horizontal_layers,
        minimum_rail_gap_mm, maximum_missing_y_layers,
    )
    if isolated is not None:
        lower_rows, upper_rows = isolated
        return ThreeRegionBoxes(
            lower=make_box_from_indices(np.flatnonzero(rows.height <= lower_rows.height[-1])),
            upper=make_box_from_indices(np.flatnonzero(rows.height >= upper_rows.height[0])),
            middle=None,
        )

    def contiguous_runs(mask: np.ndarray, minimum_layers: int) -> list[np.ndarray]:
        indices = np.flatnonzero(mask)
        if indices.size == 0:
            return []
        gaps = (np.diff(indices) > 1) | (
            np.diff(rows.height[indices]) > maximum_gap
        )
        return [
            run
            for run in np.split(indices, np.flatnonzero(gaps) + 1)
            if run.size >= minimum_layers
        ]

    widths = rows.v_max - rows.v_min
    wide_runs = contiguous_runs(
        widths >= horizontal_width_mm,
        minimum_horizontal_layers,
    )

    def expand_run(run: np.ndarray) -> np.ndarray:
        if run.size == 0:
            return run
        start = int(run[0])
        stop = int(run[-1])
        for _ in range(rail_padding_layers):
            if (
                start > 0
                and rows.height[start] - rows.height[start - 1] <= maximum_gap
            ):
                start -= 1
            if (
                stop + 1 < len(rows)
                and rows.height[stop + 1] - rows.height[stop] <= maximum_gap
            ):
                stop += 1
        return np.arange(start, stop + 1, dtype=int)

    if edges.detection_mode == "folded_terminal_rails":
        # 已按竖杆恢复位置找到完整端部；扩展最多包含首个竖杆层，
        # 不再把固定padding层数延伸到中间的附件上。
        padding = min(rail_padding_layers, 1) * y_threshold
        lower = make_box_from_indices(np.flatnonzero(
            rows.height <= edges.in_down.height + padding
        ))
        upper = make_box_from_indices(np.flatnonzero(
            rows.height >= edges.in_up.height - padding
        ))
    elif (
        len(wide_runs) >= 2
        and rows.height[wide_runs[-1][0]] - rows.height[wide_runs[0][-1]] >= minimum_rail_gap_mm
        # 宽段只辅助构框，不能截短已经确认的内边界。
        and rows.height[wide_runs[0][-1]] >= edges.in_down.height
        and rows.height[wide_runs[-1][0]] <= edges.in_up.height
    ):
        lower_run = wide_runs[0]
        upper_run = wide_runs[-1]
        lower = make_box_from_indices(expand_run(lower_run))
        upper = make_box_from_indices(expand_run(upper_run))
    else:
        upper_mask = (rows.height >= edges.in_up.height) & (
            rows.height <= edges.out_up.height
        )
        lower_mask = (rows.height >= edges.out_down.height) & (
            rows.height <= edges.in_down.height
        )
        # 边界回退分支同样按真实连续层扩展。薄横梁可能只有一层，
        # 若不扩展，零高度框会进入Z向平滑并使邻帧的整组正式框失效。
        upper = make_box_from_indices(expand_run(np.flatnonzero(upper_mask)))
        lower = make_box_from_indices(expand_run(np.flatnonzero(lower_mask)))

    if upper is None or lower is None or upper.y_min <= lower.y_max:
        return None
    middle_mask = (
        (rows.height > lower.y_max)
        & (rows.height < upper.y_min)
        & (widths < horizontal_width_mm)
    )
    middle_runs = contiguous_runs(middle_mask, minimum_layers=2)
    expected_middle_layers = max(
        int(round((upper.y_min - lower.y_max) / y_threshold)) - 1,
        1,
    )
    longest_middle_run = (
        max(middle_runs, key=len) if middle_runs else np.empty(0, dtype=int)
    )
    middle_coverage = longest_middle_run.size / expected_middle_layers
    if (
        middle_coverage >= minimum_middle_coverage
        and longest_middle_run.size > middle_trim_layers * 2
    ):
        middle_indices = longest_middle_run[
            middle_trim_layers : longest_middle_run.size - middle_trim_layers
        ]
        middle_core = make_box_from_indices(middle_indices)
        middle = RegionBox(
            x_min=middle_core.x_min,
            x_max=middle_core.x_max,
            y_min=lower.y_max,
            y_max=upper.y_min,
        )
    else:
        middle = None
    return ThreeRegionBoxes(upper=upper, middle=middle, lower=lower)


def _stabilize_numeric_track(
    values: Sequence[float | None],
    window_size: int = 11,
    quantize_step: float | None = None,
) -> list[float | None]:
    if (
        isinstance(window_size, bool)
        or not isinstance(window_size, int)
        or window_size <= 0
    ):
        raise ValueError("window_size 必须是大于0的整数")
    if window_size % 2 == 0:
        raise ValueError("window_size 必须是奇数")
    if quantize_step is not None and (
        not math.isfinite(quantize_step) or quantize_step <= 0
    ):
        raise ValueError("quantize_step 必须是大于0的有限数")
    if not values:
        return []
    numeric = np.asarray(
        [np.nan if value is None else float(value) for value in values],
        dtype=float,
    )
    known = np.flatnonzero(np.isfinite(numeric))
    if known.size == 0:
        return [None] * len(values)
    frame_indices = np.arange(len(values), dtype=float)
    interpolated = np.interp(frame_indices, known, numeric[known])
    actual_window = min(window_size, len(values))
    if actual_window % 2 == 0:
        actual_window -= 1
    actual_window = max(actual_window, 1)
    half_window = actual_window // 2
    smoothed = np.empty(len(values), dtype=float)
    for index in range(len(values)):
        start = min(max(index - half_window, 0), len(values) - actual_window)
        smoothed[index] = float(np.median(interpolated[start : start + actual_window]))
    if quantize_step is not None:
        smoothed = np.round(smoothed / quantize_step) * quantize_step
    return [float(value) for value in smoothed]


def _build_region_box_sequence(
    rows_by_frame: Sequence[ContourRows],
    edges_by_frame: Sequence[FrameEdgeCandidates],
    y_threshold: float = 10.0,
    smoothing_window: int = 11,
    minimum_middle_coverage: float = 0.3,
    narrow_width_mm: float = 50.0,
    minimum_horizontal_layers: int = 4,
    rail_padding_layers: int = 4,
    middle_trim_layers: int = 4,
    minimum_rail_gap_mm: int = 0,
    maximum_missing_y_layers: int = 0,
) -> list[ThreeRegionBoxes | None]:
    if len(rows_by_frame) != len(edges_by_frame):
        raise ValueError("rows_by_frame 与 edges_by_frame 长度必须一致")
    if not rows_by_frame:
        return []
    raw_boxes = [
        _build_three_region_boxes(
            rows,
            edges,
            y_threshold=y_threshold,
            minimum_middle_coverage=minimum_middle_coverage,
            horizontal_width_mm=narrow_width_mm,
            minimum_horizontal_layers=minimum_horizontal_layers,
            rail_padding_layers=rail_padding_layers,
            middle_trim_layers=middle_trim_layers,
            minimum_rail_gap_mm=minimum_rail_gap_mm,
            maximum_missing_y_layers=maximum_missing_y_layers,
        )
        for rows, edges in zip(rows_by_frame, edges_by_frame)
    ]

    def track(selector, quantize: float | None = None) -> list[float | None]:
        return _stabilize_numeric_track(
            [None if boxes is None else selector(boxes) for boxes in raw_boxes],
            window_size=smoothing_window,
            quantize_step=quantize,
        )

    lower_y_min = track(lambda boxes: boxes.lower.y_min, y_threshold)
    lower_y_max = track(lambda boxes: boxes.lower.y_max, y_threshold)
    upper_y_min = track(lambda boxes: boxes.upper.y_min, y_threshold)
    upper_y_max = track(lambda boxes: boxes.upper.y_max, y_threshold)
    lower_x_min = track(lambda boxes: boxes.lower.x_min)
    lower_x_max = track(lambda boxes: boxes.lower.x_max)
    upper_x_min = track(lambda boxes: boxes.upper.x_min)
    upper_x_max = track(lambda boxes: boxes.upper.x_max)

    results = []
    for index, rows in enumerate(rows_by_frame):
        # 中值平滑不能裁掉已完整识别的独立横梁或端部折弯横梁。
        if (
            _isolated_rail_rows(
                rows, y_threshold, minimum_horizontal_layers,
                minimum_rail_gap_mm, maximum_missing_y_layers,
            ) is not None
            or edges_by_frame[index].detection_mode == "folded_terminal_rails"
        ):
            current = raw_boxes[index]
            if current is not None:
                for low, high, box in (
                    (lower_y_min, lower_y_max, current.lower),
                    (upper_y_min, upper_y_max, current.upper),
                ):
                    low[index] = min(low[index], box.y_min)
                    high[index] = max(high[index], box.y_max)
                for low, high, box in (
                    (lower_x_min, lower_x_max, current.lower),
                    (upper_x_min, upper_x_max, current.upper),
                ):
                    low[index] = min(low[index], box.x_min)
                    high[index] = max(high[index], box.x_max)
        boundaries = (
            lower_y_min[index],
            lower_y_max[index],
            upper_y_min[index],
            upper_y_max[index],
            lower_x_min[index],
            lower_x_max[index],
            upper_x_min[index],
            upper_x_max[index],
        )
        if any(value is None for value in boundaries):
            results.append(None)
            continue
        if not (
            lower_y_min[index]
            < lower_y_max[index]
            < upper_y_min[index]
            < upper_y_max[index]
            and lower_x_min[index] < lower_x_max[index]
            and upper_x_min[index] < upper_x_max[index]
        ):
            results.append(None)
            continue
        lower = RegionBox(
            lower_x_min[index],
            lower_x_max[index],
            lower_y_min[index],
            lower_y_max[index],
        )
        upper = RegionBox(
            upper_x_min[index],
            upper_x_max[index],
            upper_y_min[index],
            upper_y_max[index],
        )
        middle_mask = (rows.height > lower.y_max) & (rows.height < upper.y_min)
        expected_middle_layers = max(
            int(round((upper.y_min - lower.y_max) / y_threshold)) - 1,
            1,
        )
        middle_indices = np.flatnonzero(middle_mask)
        middle_coverage = middle_indices.size / expected_middle_layers
        middle = None
        if middle_coverage >= minimum_middle_coverage and middle_indices.size >= 2:
            widths = rows.v_max[middle_indices] - rows.v_min[middle_indices]
            narrow_indices = middle_indices[widths <= narrow_width_mm]
            if narrow_indices.size < 2:
                keep_count = max(2, int(math.ceil(middle_indices.size * 0.25)))
                narrow_indices = middle_indices[np.argsort(widths)[:keep_count]]
            middle_x_min = float(np.median(rows.v_min[narrow_indices]))
            middle_x_max = float(np.median(rows.v_max[narrow_indices]))
            if middle_x_min < middle_x_max:
                middle = RegionBox(
                    middle_x_min,
                    middle_x_max,
                    lower.y_max,
                    upper.y_min,
                )
        results.append(ThreeRegionBoxes(upper=upper, middle=middle, lower=lower))
    return results
