"""设备轴索引和设备限位数组映射。

轴槽属于已经确认的PLC线协议。侧面云雀左右各占7个普通轴槽，
左右2D变频设备分别使用两个专用AxisData槽位。
"""

from model.utils.LoggerUtil import logger


# max_limit_speed / min_limit_pos / max_limit_pos / safe_pos 数组顺序。
AXIS_LIMIT_KEY: dict[str, dict[str, int]] = {
    "xn_side": {"y": 0, "x": 1},
    "out_lift": {"y": 0, "x": 1},
}


def get_axis_config_index(machine_type: str, axis_name: str) -> int | None:
    """获取轴在设备限速、限位数组中的索引。"""

    limit_key = AXIS_LIMIT_KEY.get(machine_type, {})
    if axis_name.startswith("x"):
        lookup = "x"
    elif axis_name.startswith("y"):
        lookup = "y"
    else:
        lookup = axis_name
    return limit_key.get(lookup)


def get_axis_config_value(
    machine_config: dict,
    axis_name: str,
    config_key: str,
    default: int = 0,
) -> int:
    """按设备轴类别读取限速、限位或安全位。"""

    index = get_axis_config_index(machine_config.get("type", ""), axis_name)
    values = machine_config.get(config_key, [])
    if (
        index is not None
        and isinstance(values, (list, tuple))
        and index < len(values)
    ):
        return int(values[index] or 0)
    return default


def get_axis_speed_limit(
    machine_config: dict,
    axis_name: str,
    default: int = 300,
) -> int:
    """获取设备指定轴的最大速度。"""

    return get_axis_config_value(
        machine_config,
        axis_name,
        "max_limit_speed",
        default,
    )


def get_axis_position_limits(
    machine_config: dict,
    axis_name: str,
    default_min: int = 0,
    default_max: int = 1000,
) -> tuple[int, int]:
    """获取设备指定轴的位置最小、最大限位。"""

    return (
        get_axis_config_value(
            machine_config, axis_name, "min_limit_pos", default_min
        ),
        get_axis_config_value(
            machine_config, axis_name, "max_limit_pos", default_max
        ),
    )


def get_axis_safe_pos(
    machine_config: dict,
    axis_name: str,
    default: int = 0,
) -> int:
    """获取设备指定轴的安全位。"""

    return get_axis_config_value(
        machine_config,
        axis_name,
        "safe_pos",
        default,
    )


MACHINE_AXIS_MAP = {
    # 喷枪从下往上排列，枪4只有X轴。
    # axis_type: ["y1", "x1", "y2", "x2", "y3", "x3", "x4"]
    "xn_side": {
        "left": dict(
            y1=0, x1=1,
            y2=2, x2=3,
            y3=4, x3=5,
            x4=6,
        ),
        "right": dict(
            y1=7, x1=8,
            y2=9, x2=10,
            y3=11, x3=12,
            x4=13,
        ),
    },
    # axis_type: ["y", "x"]；线协议中Y/X共用一个二维专用槽位。
    "out_lift": {
        "left": dict(y=14),
        "right": dict(y=15),
    },
}


def get_axis_map(machine_type: str, orientation: str) -> dict:
    """获取指定设备类型和安装方向的完整轴槽映射。"""

    if machine_type not in MACHINE_AXIS_MAP:
        raise ValueError(
            f"未知设备类型: {machine_type}, "
            f"可用类型: {list(MACHINE_AXIS_MAP)}"
        )
    type_map = MACHINE_AXIS_MAP[machine_type]
    if orientation not in type_map:
        raise ValueError(
            f"设备 '{machine_type}' 不支持方向 '{orientation}', "
            f"可用方向: {list(type_map)}"
        )
    return type_map[orientation]


def get_axis_index(
    machine_type: str,
    orientation: str,
    axis_name: str,
) -> int:
    """获取一个设备轴在PLC AxisList中的索引。"""

    axis_map = get_axis_map(machine_type, orientation)
    if axis_name not in axis_map:
        raise ValueError(
            f"轴 '{axis_name}' 不存在于 {machine_type}/{orientation}, "
            f"可用轴: {list(axis_map)}"
        )
    return axis_map[axis_name]


def get_x_axes(machine_type: str, orientation: str) -> dict:
    """获取设备所有X轴的名称到PLC索引映射。"""

    return {
        name: index
        for name, index in get_axis_map(machine_type, orientation).items()
        if name.startswith("x")
    }


def has_axis(machine_type: str, orientation: str, axis_name: str) -> bool:
    """判断指定设备是否具有某个轴。"""

    return axis_name in get_axis_map(machine_type, orientation)


def get_all_axis_indices(machine_type: str, orientation: str) -> list[int]:
    """获取指定设备占用的全部PLC轴索引。"""

    return list(get_axis_map(machine_type, orientation).values())


def apply_to_axis_list(
    machine_data: dict,
    machine_type: str,
    orientation: str,
    axis_list: list,
) -> None:
    """把按轴名组织的普通轴指令写入PLC AxisList。"""

    axis_map = get_axis_map(machine_type, orientation)
    for axis_name, axis_data in machine_data.items():
        if axis_name in axis_map:
            axis_list[axis_map[axis_name]] = axis_data


def apply_device_axes_to_list(
    machine_config: dict,
    sn: int,
    axis_commands: dict,
    axis_list: list,
) -> None:
    """根据设备SN和安装方向写入普通AxisList槽位。"""

    config = machine_config.get(str(sn))
    if not config:
        return
    machine_type = config.get("type", "")
    if machine_type == "out_lift":
        return
    try:
        apply_to_axis_list(
            axis_commands,
            machine_type,
            config.get("install_orietation", "left"),
            axis_list,
        )
    except Exception as exc:
        logger.error(f"将 SN[{sn}] 的轴数据应用到 AxisList 时出错: {exc}")
