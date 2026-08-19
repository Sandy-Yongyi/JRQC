from functools import lru_cache
from typing import Dict, List

import numpy as np
from model.formats.frame_by_frame.AxisFrameDataFormat import AxisData, AxisFrameData

# "upper" 方向使用 X 分层找 Y 最大/最小值，其余方向使用 Y 分层找 X 最大/最小值
_UPPER_SIDES = ("left_upper", "right_upper")


@lru_cache(maxsize=8)
def _get_h_centers(h_min: int, h_max: int, h_threshold: int) -> np.ndarray:
    return np.arange(h_min, h_max, h_threshold, dtype=int)


def build_side_frame(accum: Dict[str, List[np.ndarray]], side: str, process_config) -> AxisFrameData:
    """
    根据累积的点云数据(accum)构建某一侧的 AxisFrameData。

    对于 left / right:
        - 使用 combined_y_min / combined_y_max 与 y_threshold 做 Y 分层
        - 每层输出: H_Axis=Y, V_Axis_Max=X_max, V_Axis_Min=X_min

    对于 left_upper / right_upper:
        - 使用 combined_x_min / combined_x_max 与 x_threshold 做 X 分层
        - 每层输出: H_Axis=X, V_Axis_Max=Y_max, V_Axis_Min=Y_min
    """
    is_upper = side in _UPPER_SIDES

    if is_upper:
        # X 分层参数
        h_threshold = int(process_config.get("x_threshold", 10))
        h_min = int(process_config.get("combined_x_min", 0))
        h_max = int(process_config.get("combined_x_max", 1500))
        # 点云列索引: H=X(col0), V=Y(col1)
        h_col, v_col = 0, 1
    else:
        # Y 分层参数（原有逻辑）
        h_threshold = int(process_config.get("y_threshold", 10))
        h_min = int(process_config.get("combined_y_min", 1100))
        h_max = int(process_config.get("combined_y_max", 3800))
        # 点云列索引: H=Y(col1), V=X(col0)
        h_col, v_col = 1, 0

    data_key = side

    # 合并所有帧数据
    main_arrays = accum.get(data_key, [])
    if not main_arrays:
        main_combined = np.empty((0, 3), dtype=float)
    elif len(main_arrays) == 1:
        main_combined = np.asarray(main_arrays[0])
    else:
        main_combined = np.vstack(main_arrays)

    # 构建分层中心值
    h_centers = _get_h_centers(h_min, h_max, h_threshold)
    layer_count = len(h_centers)

    frame_list: List[AxisData] = []

    # 若无数据，则每层全 0
    if main_combined.size == 0:
        for _ in h_centers:
            frame_list.append(AxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0))
    else:
        if main_combined.ndim == 1:
            main_combined = main_combined.reshape(1, -1)

        if main_combined.shape[1] <= max(h_col, v_col):
            return AxisFrameData(FrameData=[AxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0) for _ in range(layer_count)])

        h_values = main_combined[:, h_col]
        v_values = main_combined[:, v_col]
        valid_mask = (h_values >= h_min) & (h_values < h_max)

        if not np.any(valid_mask):
            frame_list = [AxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0) for _ in range(layer_count)]
        else:
            h_values = h_values[valid_mask]
            v_values = v_values[valid_mask]
            bin_indices = ((h_values - h_min) // h_threshold).astype(np.int32)

            v_max = np.full(layer_count, -np.inf)
            v_min = np.full(layer_count, np.inf)
            np.maximum.at(v_max, bin_indices, v_values)
            np.minimum.at(v_min, bin_indices, v_values)

            filled_mask = np.isfinite(v_min)
            frame_list = [
                AxisData(
                    H_Axis=int(h_centers[idx]) if filled_mask[idx] else 0,
                    V_Axis_Max=int(v_max[idx]) if filled_mask[idx] else 0,
                    V_Axis_Min=int(v_min[idx]) if filled_mask[idx] else 0,
                )
                for idx in range(layer_count)
            ]

    axis_frame = AxisFrameData(FrameData=frame_list)
    return axis_frame
