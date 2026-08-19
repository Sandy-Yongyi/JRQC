from model.utils.TomlLoader import TomlLoader


def is_password_required(mode_config_path: str) -> bool:
    """读取 ModeConfig.toml 的 require_password，读取失败时默认需要密码。"""
    try:
        config = TomlLoader.load(mode_config_path)
    except Exception:
        return True

    value = config.get("require_password", True)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in {"false", "0", "no", "off"}
    return bool(value)
