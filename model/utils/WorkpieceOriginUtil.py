import copy
from typing import Any

import numpy as np


LEFT_ORIGIN = "left"
RIGHT_ORIGIN = "right"


def normalize_origin_side(value: Any) -> str:
    side = str(value or LEFT_ORIGIN).strip().lower()
    if side not in {LEFT_ORIGIN, RIGHT_ORIGIN}:
        return LEFT_ORIGIN
    return side


def get_origin_side(config: dict | None) -> str:
    if not isinstance(config, dict):
        return LEFT_ORIGIN
    return normalize_origin_side(config.get("workpiece_origin_side", LEFT_ORIGIN))


def get_origin_reference_x(config: dict | None, default: float = 1540.0) -> float:
    if not isinstance(config, dict):
        return float(default)
    return float(config.get("workpiece_origin_reference_x", default) or default)


def transform_points_for_origin(points, config: dict | None):
    side = get_origin_side(config)
    arr = np.asarray(points, dtype=float)
    if side != RIGHT_ORIGIN or arr.ndim != 2 or arr.shape[1] < 1:
        return np.array(arr, copy=True)

    transformed = np.array(arr, copy=True)
    reference_x = get_origin_reference_x(config)
    transformed[:, 0] = reference_x - transformed[:, 0]
    return transformed


def transform_block_data_for_origin(block_data, config: dict | None):
    side = get_origin_side(config)
    if side != RIGHT_ORIGIN or block_data is None:
        return block_data

    transformed = copy.deepcopy(block_data)
    reference_x = get_origin_reference_x(config)

    for outside in getattr(transformed, "outside_data", []) or []:
        x_min = getattr(outside, "outside_x_min", None)
        x_max = getattr(outside, "outside_x_max", None)
        if x_min is None or x_max is None:
            continue
        outside.outside_x_min = reference_x - x_max
        outside.outside_x_max = reference_x - x_min

    for inside in getattr(transformed, "inside_data", []) or []:
        for subinside in getattr(inside, "subinside_datalist", []) or []:
            x_min = getattr(subinside, "subinside_x_min", None)
            x_max = getattr(subinside, "subinside_x_max", None)
            if x_min is None or x_max is None:
                continue
            subinside.subinside_x_min = reference_x - x_max
            subinside.subinside_x_max = reference_x - x_min

    return transformed
