import os
from model.utils.TomlLoader import TomlLoader
from model.utils.LoggerUtil import logger
from model.utils.MachineConfigUtil import get_machine_config_path, normalize_machine_offset_values, validate_machine_params
from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_speed_limit


def _load_range(cfg: dict, key: str, default_min: int, default_max: int):
    return cfg.get(f"{key}_min", default_min), cfg.get(f"{key}_max", default_max)


def _get_param_range_rules(sn: int | None = None, strategy_name="frame_by_frame"):
    """加载参数范围。距离类来自 SprayConfig，速度类来自当前设备 max_limit_speed。"""
    spray_config_path = os.path.join(os.getcwd(), "model", "tomls", "SprayConfig.toml")
    machine_cfg = load_machine_config_for_ui(sn, strategy_name) if sn is not None else {}
    x_speed_max = get_axis_speed_limit(machine_cfg, "x", 620) if machine_cfg else 620
    y_speed_max = get_axis_speed_limit(machine_cfg, "y", 590) if machine_cfg else 590
    z_speed_max = get_axis_speed_limit(machine_cfg, "z", 240) if machine_cfg else 240
    y_pos_min, y_pos_max = get_axis_position_limits(machine_cfg, "y", 0, 430) if machine_cfg else (0, 430)

    try:
        spray_cfg = TomlLoader.load(spray_config_path)
        return {
            "tracking": (0, 1),
            "y_move_min": (y_pos_min, y_pos_max),
            "y_move_max": (y_pos_min, y_pos_max),
            "out_front_x_offset": _load_range(spray_cfg, "out_front_x_offset", 0, 300),
            "out_after_x_offset": _load_range(spray_cfg, "out_after_x_offset", 0, 300),
            "in_front_x_offset": _load_range(spray_cfg, "in_front_x_offset", 0, 300),
            "in_after_x_offset": _load_range(spray_cfg, "in_after_x_offset", 50, 300),
            "x_pos_speed": (0, x_speed_max),
            "x_recip_speed": (0, x_speed_max),
            "x_status_offset": _load_range(spray_cfg, "x_status_offset", 0, 300),
            "origin_pos": _load_range(spray_cfg, "origin_pos", 2500, 4000),
            "out_up_y_offset": _load_range(spray_cfg, "out_up_y_offset", 0, 300),
            "out_down_y_offset": _load_range(spray_cfg, "out_down_y_offset", 0, 300),
            "in_up_y_offset": _load_range(spray_cfg, "in_up_y_offset", 50, 300),
            "in_down_y_offset": _load_range(spray_cfg, "in_down_y_offset", 50, 300),
            "y_pos_speed": (0, y_speed_max),
            "y_recip_speed": (0, y_speed_max),
            "out_z_front_offset": _load_range(spray_cfg, "out_z_front_offset", 50, 300),
            "out_z_after_offset": _load_range(spray_cfg, "out_z_after_offset", 50, 300),
            "in_z_front_offset": _load_range(spray_cfg, "in_z_front_offset", 50, 300),
            "in_z_after_offset": _load_range(spray_cfg, "in_z_after_offset", 50, 300),
            "z_back_speed": (0, z_speed_max),
            "z_zeroing_speed": (0, z_speed_max),
            "outside_total_cycles": _load_range(spray_cfg, "outside_total_cycles", 1, 5),
            "inside_total_cycles": _load_range(spray_cfg, "inside_total_cycles", 1, 5),
            "recip_reduce_distance": _load_range(spray_cfg, "recip_reduce_distance", 0, 300),
        }
    except Exception as e:
        # 如果加载失败，使用默认值
        logger.info(f"load SprayConfig.toml failed, use default param ranges: {e}")
        return {
            "tracking": (0, 1),
            "y_move_min": (y_pos_min, y_pos_max),
            "y_move_max": (y_pos_min, y_pos_max),
            "out_front_x_offset": (0, 300),
            "out_after_x_offset": (0, 300),
            "in_front_x_offset": (0, 300),
            "in_after_x_offset": (50, 300),
            "x_pos_speed": (0, x_speed_max),
            "x_recip_speed": (0, x_speed_max),
            "x_status_offset": (0, 300),
            "out_up_y_offset": (0, 300),
            "out_down_y_offset": (0, 300),
            "in_up_y_offset": (50, 300),
            "in_down_y_offset": (50, 300),
            "origin_pos": (2500, 4000),
            "y_pos_speed": (0, y_speed_max),
            "y_recip_speed": (0, y_speed_max),
            "out_z_front_offset": (50, 300),
            "out_z_after_offset": (50, 300),
            "in_z_front_offset": (50, 300),
            "in_z_after_offset": (50, 300),
            "z_back_speed": (0, z_speed_max),
            "z_zeroing_speed": (0, z_speed_max),
            "outside_total_cycles": (1, 5),
            "inside_total_cycles": (1, 5),
            "recip_reduce_distance": (0, 300),
        }


FLAT_CONFIG_KEY = "flat"


def _get_toml_path(strategy_name: str) -> str:
    config_dir = os.path.join(os.getcwd(), "model", "tomls")
    return get_machine_config_path(config_dir, strategy_name)


def _normalize_origin_pos_for_ui(cfg: dict) -> dict:
    normalized = dict(cfg)
    origin_pos = normalized.get("origin_pos")
    if isinstance(origin_pos, list) and len(origin_pos) == 1:
        normalized["origin_pos"] = origin_pos[0]
    return normalized


def _normalize_config_for_ui(cfg: dict) -> dict:
    normalized = normalize_machine_offset_values(_normalize_origin_pos_for_ui(cfg))
    flat_cfg = normalized.get(FLAT_CONFIG_KEY)
    if isinstance(flat_cfg, dict):
        normalized[FLAT_CONFIG_KEY] = normalize_machine_offset_values(_normalize_origin_pos_for_ui(flat_cfg))
    return normalized


def _normalize_origin_pos_for_save(values: dict) -> dict:
    normalized = dict(values)
    if "origin_pos" in normalized:
        normalized["origin_pos"] = [normalized["origin_pos"]]
    return normalized


def _normalize_ui_values_for_save(values: dict) -> dict:
    normalized = normalize_machine_offset_values(_normalize_origin_pos_for_save(values))
    flat_values = normalized.get(FLAT_CONFIG_KEY)
    if isinstance(flat_values, dict):
        normalized[FLAT_CONFIG_KEY] = normalize_machine_offset_values(_normalize_origin_pos_for_save(flat_values))
    return normalized


def load_machine_config_for_ui(sn: int, strategy_name="frame_by_frame") -> dict:
    cfg = TomlLoader.load(_get_toml_path(strategy_name))
    return _normalize_config_for_ui(cfg.get(str(sn), {}))


def save_machine_config_from_ui(sn: int, values: dict, control_queue=None,
                                strategy_name="frame_by_frame"):
    """保存机器配置并通知PLC进程"""
    param_range_rules = _get_param_range_rules(sn, strategy_name)
    _validate_params(values, param_range_rules)
    normalized_values = _normalize_ui_values_for_save(values)
    toml_path = _get_toml_path(strategy_name)
    _save_to_toml(sn, normalized_values, toml_path)
    if control_queue is not None:
        _notify_plc(sn, normalized_values, control_queue, toml_path)


def _validate_params(values: dict, param_range_rules: dict):
    validate_machine_params(values, param_range_rules)


def _save_to_toml(sn: int, values: dict, toml_path: str):
    cfg = TomlLoader.load(toml_path)
    sn_key = str(sn)
    if sn_key not in cfg:
        raise RuntimeError(f"{os.path.basename(toml_path)} 中不存在 SN[{sn}] 配置")
    existing_config = cfg.get(sn_key, {})
    merged_config = {**existing_config, **values}
    # 更新配置
    cfg[sn_key] = merged_config
    TomlLoader.save(cfg, toml_path)


def _notify_plc(sn: int, values: dict, control_queue, toml_path: str):
    """通知PLC进程配置已更新"""
    try:
        cfg = TomlLoader.load(toml_path)
        sn_key = str(sn)
        if sn_key in cfg:
            full_config = cfg[sn_key].copy()
            full_config.update(values)  # UI值覆盖配置文件值
            # 构建通知消息
            config_update_msg = {"machine": {"sn": sn, **full_config}}
            control_queue.put(config_update_msg)
            logger.info(f"{sn} send config to plc: {config_update_msg}")
            print(f"已通知PLC进程更新SN[{sn}]的配置：{config_update_msg}")
        else:
            # 如果配置文件中不存在该SN，只发送UI传过来的值
            config_update_msg = {"machine": {"sn": sn, **values}}
            control_queue.put(config_update_msg)
            logger.info(f"{sn} send config to plc (new SN): {config_update_msg}")
            print(f"已通知PLC进程更新SN[{sn}]的配置(新SN): {config_update_msg}")
    except Exception as e:
        logger.error(f"send config to plc failed: {e}")
        print(f"通知PLC进程失败: {e}")
