"""规划器共用的数值校验、毫米取整和不可变快照工具。"""

from collections.abc import Mapping
import math


def _require_int(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} 必须是整数")
    return value


def _round_runtime_mm(value: float) -> int:
    """按常规四舍五入转换为PLC整数毫米。"""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("轮廓坐标必须是数值")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("轮廓坐标必须是有限数")
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def _freeze_runtime_value(value):
    """把运行时配置转成可比较签名，用于自动失效几何缓存。"""

    if isinstance(value, Mapping):
        return tuple(
            sorted(
                (str(key), _freeze_runtime_value(item)) for key, item in value.items()
            )
        )
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_runtime_value(item) for item in value)
    if isinstance(value, (str, int, float, bool, type(None))):
        return value
    return repr(value)
