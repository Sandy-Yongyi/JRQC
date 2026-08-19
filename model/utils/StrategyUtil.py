"""喷涂策略判断工具。"""

COMPLETE_WORKPIECE_STRATEGY = "complete_workpiece"
FRAME_QUEUE_STRATEGY = "frame_by_frame"
CONTINUOUS_BIDIRECTIONAL_STRATEGY = "continuous_bidirectional"
SUPPORTED_STRATEGIES = {
    COMPLETE_WORKPIECE_STRATEGY,
    FRAME_QUEUE_STRATEGY,
    CONTINUOUS_BIDIRECTIONAL_STRATEGY,
}
STRATEGY_CODE_MAP = {
    1: FRAME_QUEUE_STRATEGY,
    2: COMPLETE_WORKPIECE_STRATEGY,
    3: CONTINUOUS_BIDIRECTIONAL_STRATEGY,
}


def strategy_name_from_code(value: object) -> str:
    """把 ModeConfig.toml 的整数策略映射为内部策略名称。"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"strategy_name 必须是整数 1/2/3，当前值: {value!r}")
    try:
        return STRATEGY_CODE_MAP[value]
    except KeyError as exc:
        raise ValueError(f"strategy_name 只支持 1/2/3，当前值: {value}") from exc


def validate_strategy_name(strategy_name: str) -> str:
    """校验策略名称是否合法，不合法时直接抛错。"""
    if strategy_name not in SUPPORTED_STRATEGIES:
        raise ValueError(
            f"未知喷涂策略: {strategy_name}, 可用策略: {sorted(SUPPORTED_STRATEGIES)}"
        )
    return strategy_name


def is_complete_workpiece_mode(strategy_name: str) -> bool:
    """是否为完整工件模式。"""
    validate_strategy_name(strategy_name)
    return strategy_name == COMPLETE_WORKPIECE_STRATEGY


def is_frame_by_frame_mode(strategy_name: str) -> bool:
    """是否为帧队列模式。"""
    validate_strategy_name(strategy_name)
    return strategy_name == FRAME_QUEUE_STRATEGY


def is_continuous_bidirectional_mode(strategy_name: str) -> bool:
    """是否为连续双向模式。"""
    validate_strategy_name(strategy_name)
    return strategy_name == CONTINUOUS_BIDIRECTIONAL_STRATEGY
