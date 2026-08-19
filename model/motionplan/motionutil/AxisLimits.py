from model.plc.MovingFrameData import AxisData


def clamp_speed(speed: int, max_speed: int) -> int:
    """
    保证速度在 [0, max_speed] 范围内
    """
    if speed is None:
        return 0
    if max_speed is None:
        return max(0, speed)
    return max(0, min(speed, max_speed))


def clamp_to_limit_yx(value: int, min_limit: int = 0, max_limit: int | None = None) -> int:
    """
    保证 Y/X 轴位置 value 在 [min_limit, max_limit] 范围内
    """
    if value is None:
        return 0
    if max_limit is None:
        max_limit = min_limit
        min_limit = 0
    return max(min_limit, min(value, max_limit))


def clamp_to_limit_z(value: int, min_limit: int, max_limit: int | None = None) -> int:
    """
    保证 Z 轴位置 value 在 [min_limit, max_limit] 范围内
    """
    if value is None:
        return 0
    if max_limit is None:
        max_limit = min_limit
        min_limit = 0
    return max(min_limit, min(value, max_limit))


def clamp_to_limit_r(value: int, min_limit: int, max_limit: int | None = None) -> int:
    """
    保证旋转角度 value 在 [min_limit, max_limit] 范围内
    """
    if value is None:
        return 0
    if max_limit is None:
        max_limit = abs(min_limit)
        min_limit = -abs(min_limit)
    return max(min_limit, min(value, max_limit))


def build_axis(target, speed, status, max_speed=None):
    """
    生成 AxisData 指令
    自动对速度做安全限制
    """
    if max_speed is not None:
        speed = clamp_speed(speed, max_speed)
    return AxisData(Pos=target, Speed=speed, Status=status)
