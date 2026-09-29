"""设备安全距离：云雀front来料/back出料，二维front出料/back来料。"""

WINDOW_KEYS = {
    "xn_side": ("yunque_safety_window_front_mm", "yunque_safety_window_back_mm"),
    "out_lift": ("variable_2d_safety_window_front_mm", "variable_2d_safety_window_back_mm"),
}
WINDOW_DEFAULTS = {"xn_side": 200, "out_lift": 300}


def machine_window_distances(machine):
    """统一返回(出料距离, 来料距离)，与机型的字段命名解耦。"""
    kind = machine["type"]
    values = []
    for key in WINDOW_KEYS[kind]:
        value = machine.get(key, WINDOW_DEFAULTS[kind])
        if isinstance(value, bool) or not isinstance(value, int) or value < 0 or value % 10:
            raise ValueError(f"{key} 必须是10mm的非负整数倍")
        values.append(value)
    return tuple(reversed(values)) if kind == "xn_side" else tuple(values)


def machine_window_bounds(machine, stack_size, *, history_index=None):
    """返回队列 [start, stop) 和站位索引，禁止静默截断安全窗口。"""
    outgoing, incoming = machine_window_distances(machine)
    if isinstance(stack_size, bool) or not isinstance(stack_size, int) or stack_size <= 0:
        raise ValueError("帧队列长度必须是正整数")
    if history_index is None:
        z = machine.get("z_position")
        if isinstance(z, bool) or not isinstance(z, int) or z < 0 or z % 10:
            raise ValueError("设备z_position必须是10mm的非负整数倍")
        history_index = z // 10
    start, stop = history_index - incoming // 10, history_index + outgoing // 10 + 1
    if start < 0 or stop > stack_size or not 0 <= history_index < stack_size:
        raise ValueError(
            f"设备前后安全窗口超出帧队列范围：来料距离最大{history_index * 10}mm，"
            f"出料距离最大{max(0, stack_size - history_index - 1) * 10}mm"
        )
    return start, stop, history_index


def with_machine_window_defaults(machine):
    result = dict(machine)
    if result.get("type") in WINDOW_KEYS:
        machine_window_distances(result)  # 校验已配置字段
        for key in WINDOW_KEYS[result["type"]]:
            result.setdefault(key, WINDOW_DEFAULTS[result["type"]])
    return result
