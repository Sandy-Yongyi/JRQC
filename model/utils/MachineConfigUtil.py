import os

from model.utils.StrategyUtil import validate_strategy_name


MACHINE_CONFIG_FILE_BY_STRATEGY = {
    "frame_by_frame": "MachineConfig1.toml",
    "complete_workpiece": "MachineConfig2.toml",
    "continuous_bidirectional": "MachineConfig3.toml",
}

DEFAULT_MACHINE_OFFSET = 100
MACHINE_OFFSET_KEYS = (
    "x_status_offset",
    "out_front_x_offset",
    "out_after_x_offset",
    "in_front_x_offset",
    "in_after_x_offset",
    "out_up_y_offset",
    "out_down_y_offset",
    "in_up_y_offset",
    "in_down_y_offset",
    "out_z_front_offset",
    "out_z_after_offset",
    "in_z_front_offset",
    "in_z_after_offset",
)


def normalize_offset_value(value, default: int = DEFAULT_MACHINE_OFFSET) -> int:
    """偏移缺失、无效或不大于 0 时使用安全默认值。"""
    try:
        offset = int(value)
    except (TypeError, ValueError):
        return int(default)
    return offset if offset > 0 else int(default)


def get_machine_offset(machine_cfg: dict, key: str, runtime_cfg: dict | None = None,
                       default: int = DEFAULT_MACHINE_OFFSET) -> int:
    """优先读取运行时偏移，并统一处理无效值。"""
    if runtime_cfg is not None and key in runtime_cfg:
        return normalize_offset_value(runtime_cfg.get(key), default)
    return normalize_offset_value(machine_cfg.get(key), default)


def normalize_machine_offset_values(config: dict, fill_missing: bool = True) -> dict:
    """复制单台设备配置，并将主配置和 flat 中的偏移统一为安全值。"""
    normalized = dict(config or {})
    for key in MACHINE_OFFSET_KEYS:
        if fill_missing or key in normalized:
            normalized[key] = normalize_offset_value(normalized.get(key))

    flat_cfg = normalized.get("flat")
    if isinstance(flat_cfg, dict):
        normalized["flat"] = normalize_machine_offset_values(
            flat_cfg,
            fill_missing=fill_missing,
        )
    return normalized


def normalize_machine_config_offsets(machine_config: dict) -> dict:
    """归一化 MachineConfig 中每台设备的全部偏移参数。"""
    return {
        str(sn): normalize_machine_offset_values(machine_cfg)
        for sn, machine_cfg in (machine_config or {}).items()
    }


def get_machine_config_filename(strategy_name: str) -> str:
    """根据运动策略返回对应的设备配置文件名。"""
    strategy_name = validate_strategy_name(strategy_name)
    return MACHINE_CONFIG_FILE_BY_STRATEGY[strategy_name]


def get_machine_config_path(config_dir: str, strategy_name: str) -> str:
    """根据运动策略返回设备配置文件完整路径。"""
    return os.path.join(config_dir, get_machine_config_filename(strategy_name))


def validate_machine_params(values: dict, param_range_rules: dict):
    """校验界面参数范围及 Y 轴最小、最大位置关系。"""
    for key, value in values.items():
        if isinstance(value, dict):
            validate_machine_params(value, param_range_rules)
            continue
        if key not in param_range_rules:
            continue
        min_value, max_value = param_range_rules[key]
        if not (min_value <= value <= max_value):
            raise ValueError(f"{key} 超出范围 {min_value} ~ {max_value}")

    if "y_move_min" in values and "y_move_max" in values:
        if int(values["y_move_min"]) >= int(values["y_move_max"]):
            raise ValueError("y_move_min 必须小于 y_move_max")
