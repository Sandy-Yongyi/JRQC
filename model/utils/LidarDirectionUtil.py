ALL_DIRECTIONS = ("left", "right", "left_upper", "right_upper")
SIDE_DIRECTIONS = ("left", "right")
UPPER_DIRECTIONS = ("left_upper", "right_upper")

SYSTEM_CONFIG_DIRECTION_KEYS = {
    "left": "left_lidar_ids",
    "right": "right_lidar_ids",
    "left_upper": "left_upper_lidar_ids",
    "right_upper": "right_upper_lidar_ids",
}


def normalize_lidar_ids(lidar_ids):
    if not isinstance(lidar_ids, (list, tuple)):
        return []
    return [str(lidar_id).strip() for lidar_id in lidar_ids if str(lidar_id).strip()]


def filter_active_direction_map(direction_map):
    active_direction_map = {}
    for direction in ALL_DIRECTIONS:
        lidar_ids = normalize_lidar_ids(direction_map.get(direction, []))
        if lidar_ids:
            active_direction_map[direction] = lidar_ids
    return active_direction_map


def get_direction_lidar_config(system_config):
    direction_map = {
        direction: system_config.get(config_key, [])
        for direction, config_key in SYSTEM_CONFIG_DIRECTION_KEYS.items()
    }
    return {
        direction: normalize_lidar_ids(lidar_ids)
        for direction, lidar_ids in direction_map.items()
    }


def get_active_lidar_config(system_config):
    return filter_active_direction_map(get_direction_lidar_config(system_config))


def get_active_directions(system_config):
    return list(get_active_lidar_config(system_config).keys())
