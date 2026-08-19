"""
设备轴索引映射配置模块。

根据当前策略对应 MachineConfig1/2/3.toml 的 type 和 install_orietation，查找 PLC AxisList
中对应的轴索引。当前 Lab_Auto 第一阶段设备顺序：

    sn=0 out_fx left     : 0~9   (z, y, x1~x8)
    sn=1 xn_side left    : 10~28 (z, y1, x1, r1, ..., y6, x6, r6)
    sn=2 xn_side right   : 29~47 (z, y1, x1, r1, ..., y6, x6, r6)
"""

from model.utils.LoggerUtil import logger
from model.motionplan.motionutil.AxisLimits import clamp_speed, clamp_to_limit_yx
from model.plc.MovingFrameData import AxisData


AXIS_LIMIT_KEY: dict[str, dict[str, int]] = {
    "out_fx": {"z": 0, "y": 1, "x": 2},
    "xn_side": {"z": 0, "y": 1, "x": 2, "r": 3},
}


def get_axis_config_index(machine_type: str, axis_name: str) -> int | None:
    """获取轴在 max_limit_speed / min_limit_pos / max_limit_pos 中的索引。"""
    limit_key = AXIS_LIMIT_KEY.get(machine_type, {})
    if axis_name.startswith("x"):
        lookup = "x"
    elif axis_name.startswith("y"):
        lookup = "y"
    elif axis_name.startswith("r"):
        lookup = "r"
    else:
        lookup = axis_name
    return limit_key.get(lookup)


def get_axis_config_value(machine_cfg: dict, axis_name: str, config_key: str, default: int = 0) -> int:
    """按设备通用轴定义读取配置值，如 safe_pos / max_limit_speed / min_limit_pos / max_limit_pos。"""
    machine_type = machine_cfg.get("type", "")
    idx = get_axis_config_index(machine_type, axis_name)
    values = machine_cfg.get(config_key, [])
    if idx is not None and isinstance(values, (list, tuple)) and idx < len(values):
        return int(values[idx] or 0)
    return default


def get_axis_speed_limit(machine_cfg: dict, axis_name: str, default: int = 300) -> int:
    """获取设备指定轴的最大速度。"""
    return get_axis_config_value(machine_cfg, axis_name, "max_limit_speed", default)


def get_axis_position_limits(machine_cfg: dict, axis_name: str, default_min: int = 0, default_max: int = 1000) -> tuple[int, int]:
    """获取设备指定轴的位置最小/最大限位。"""
    min_limit = get_axis_config_value(machine_cfg, axis_name, "min_limit_pos", default_min)
    max_limit = get_axis_config_value(machine_cfg, axis_name, "max_limit_pos", default_max)
    return min_limit, max_limit


def get_axis_safe_pos(machine_cfg: dict, axis_name: str, default: int = 0) -> int:
    """获取设备指定轴的安全位置。"""
    return get_axis_config_value(machine_cfg, axis_name, "safe_pos", default)


MACHINE_AXIS_MAP = {
    "out_fx": {
        "left": dict(
            z=0,
            y=1,
            x1=2,
            x2=3,
            x3=4,
            x4=5,
            x5=6,
            x6=7,
            x7=8,
            x8=9,
        ),
    },
    "xn_side": {
        "left": dict(
            z=10,
            y1=11, x1=12, r1=13,
            y2=14, x2=15, r2=16,
            y3=17, x3=18, r3=19,
            y4=20, x4=21, r4=22,
            y5=23, x5=24, r5=25,
            y6=26, x6=27, r6=28,
        ),
        "right": dict(
            z=29,
            y1=30, x1=31, r1=32,
            y2=33, x2=34, r2=35,
            y3=36, x3=37, r3=38,
            y4=39, x4=40, r4=41,
            y5=42, x5=43, r5=44,
            y6=45, x6=46, r6=47,
        ),
    },
}


def get_axis_map(machine_type: str, orientation: str) -> dict:
    """获取指定设备类型和安装方向的完整轴映射字典。"""
    if machine_type not in MACHINE_AXIS_MAP:
        raise ValueError(f"未知设备类型: {machine_type}, 可用类型: {list(MACHINE_AXIS_MAP.keys())}")
    type_map = MACHINE_AXIS_MAP[machine_type]
    if orientation not in type_map:
        raise ValueError(f"设备 '{machine_type}' 不支持方向 '{orientation}', 可用方向: {list(type_map.keys())}")
    return type_map[orientation]


def get_axis_index(machine_type: str, orientation: str, axis_name: str) -> int:
    """获取指定设备某个轴在 PLC AxisList 中的索引。"""
    axis_map = get_axis_map(machine_type, orientation)
    if axis_name not in axis_map:
        raise ValueError(f"轴 '{axis_name}' 不存在于 {machine_type}/{orientation}, 可用轴: {list(axis_map.keys())}")
    return axis_map[axis_name]


def get_x_axes(machine_type: str, orientation: str) -> dict:
    """获取所有 X 类轴的名称到 PLC 索引映射。"""
    axis_map = get_axis_map(machine_type, orientation)
    return {k: v for k, v in axis_map.items() if k.startswith("x")}


def has_axis(machine_type: str, orientation: str, axis_name: str) -> bool:
    """判断指定设备是否拥有某个轴。"""
    axis_map = get_axis_map(machine_type, orientation)
    return axis_name in axis_map


def get_all_axis_indices(machine_type: str, orientation: str) -> list:
    """获取指定设备所有轴的 PLC 索引列表。"""
    return list(get_axis_map(machine_type, orientation).values())


def _limit_axis_command(machine_cfg: dict, axis_name: str, axis_data: AxisData) -> AxisData:
    """在写入 PLC 报文前统一限制单轴位置和速度。"""
    min_position, max_position = get_axis_position_limits(machine_cfg, axis_name)
    if min_position > max_position:
        raise ValueError(f"轴 {axis_name} 的位置限位无效: {min_position} > {max_position}")
    max_speed = get_axis_speed_limit(machine_cfg, axis_name)
    return AxisData(
        Pos=clamp_to_limit_yx(int(axis_data.Pos), min_position, max_position),
        Speed=clamp_speed(int(axis_data.Speed), max_speed),
        Status=int(axis_data.Status),
    )


def apply_to_axis_list(machine_data: dict, machine_cfg: dict, axis_list: list):
    """将设备运动指令字典应用到 PLC AxisList 中。"""
    machine_type = machine_cfg.get("type", "")
    orientation = machine_cfg.get("install_orietation", "left")
    axis_map = get_axis_map(machine_type, orientation)
    for axis_name, axis_data in machine_data.items():
        if axis_name in axis_map:
            idx = axis_map[axis_name]
            axis_list[idx] = _limit_axis_command(machine_cfg, axis_name, axis_data)
        elif axis_name == "y":
            for mapped_axis, idx in axis_map.items():
                if mapped_axis.startswith("y"):
                    axis_list[idx] = _limit_axis_command(machine_cfg, mapped_axis, axis_data)


def apply_device_axes_to_list(machine_config: dict, sn: int, axis_cmds: dict, axis_list: list):
    """按设备配置将单台设备的轴运动指令写入 AxisList。"""
    machine_cfg = machine_config.get(str(sn))
    if not machine_cfg:
        return

    try:
        apply_to_axis_list(axis_cmds, machine_cfg, axis_list)
    except Exception as e:
        logger.error(f"将 SN[{sn}] 的轴数据应用到 AxisList 时出错: {e}")
