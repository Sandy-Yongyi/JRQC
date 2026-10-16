"""设备坐标转换、回位命令和 PLC 运动帧组装。"""

from collections.abc import Mapping
from typing import Sequence

from model.motionplan._validation import (
    _require_int,
)
from model.motionplan.constants import (
    ENABLE_LEFT_XN_SIDE,
    ENABLE_MASTER,
    ENABLE_RIGHT_XN_SIDE,
    HEARTBEAT_MAX,
    HEARTBEAT_MIN,
    OPERATE_CHAIN_RUN,
    OUT_FRONT_SURFACE_FIELD_BY_DIRECTION,
    PHASE_POSITIONING_TO_FIXED,
    XN_SIDE_AXIS_SEQUENCE,
    XN_SIDE_SURFACE_Y_OFFSET_RULES,
    XN_SIDE_X_AXES,
    XN_SIDE_Y_AXES,
)
from model.motionplan.feedback import (
    XAxisMotionCommand,
)
from model.motionplan.variable_2d import (
    _out_lift_limit,
)
from model.plc.MovingFrameData import AxisData as MovingAxisData
from model.plc.MovingFrameData import SendMovingFrameData


def apply_xn_side_surface_y_offsets(
    surface_y: Mapping[str, int],
    machine_config: Mapping,
) -> dict[str, int]:
    """按MachineConfig修正四个面的激光Y到达点。

    规则为：上外顶加、上内顶减、下内底加、下外底减。调用方可以
    只传需要的面，但每个传入面都必须有对应配置。
    """

    if not isinstance(surface_y, Mapping):
        raise TypeError("surface_y 必须是映射")
    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if not surface_y:
        raise ValueError("surface_y 不能为空")

    adjusted: dict[str, int] = {}
    for surface, detected_y in surface_y.items():
        if surface not in XN_SIDE_SURFACE_Y_OFFSET_RULES:
            raise ValueError(f"未知云雀表面：{surface}")
        if isinstance(detected_y, bool) or not isinstance(detected_y, int):
            raise TypeError(f"{surface}检测Y必须是整数毫米值")
        if detected_y <= 0:
            raise ValueError(f"{surface}检测Y必须大于0")

        offset_key, sign = XN_SIDE_SURFACE_Y_OFFSET_RULES[surface]
        if offset_key not in machine_config:
            raise KeyError(f"MachineConfig 缺少参数：{offset_key}")
        offset = machine_config[offset_key]
        if isinstance(offset, bool) or not isinstance(offset, int):
            raise TypeError(f"MachineConfig.{offset_key} 必须是整数")
        if offset < 0:
            raise ValueError(f"MachineConfig.{offset_key} 不能小于0")

        target_y = detected_y + sign * offset
        if target_y <= 0:
            raise ValueError(f"{surface}应用Y偏移后的到达点必须大于0")
        adjusted[surface] = target_y
    return adjusted


def calculate_fixed_gun_sample_y(
    combined_y_max: int,
    y_threshold: int,
) -> int:
    """返回扫描配置半开区间中的最后一个Y层（旧离线辅助接口）。

    逐帧分层使用半开区间，所以2600mm边界对应最后一个10mm数据层
    2590。本函数不参与正式Y轴规划；枪4实际零点始终读取
    ``MachineConfig.origin_pos[3]``。
    """

    values = {
        "combined_y_max": combined_y_max,
        "y_threshold": y_threshold,
    }
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} 必须是整数毫米值")
    if combined_y_max <= 0:
        raise ValueError("combined_y_max 必须大于0")
    if y_threshold <= 0:
        raise ValueError("y_threshold 必须大于0")
    if combined_y_max < y_threshold:
        raise ValueError("combined_y_max 不能小于 y_threshold")
    return combined_y_max - y_threshold


def select_out_front_detected_x(axis_data, direction: str) -> int:
    """从一个Y层轮廓中取得外前表面的雷达X检测值。

    当前业务规则明确规定：左、右两侧都读取 ``V_Axis_Min``。
    空层使用0表示，因此0和负数都不能进入后续运动计算。
    """

    if direction not in OUT_FRONT_SURFACE_FIELD_BY_DIRECTION:
        raise ValueError("direction 必须是 left 或 right")
    if axis_data is None:
        raise TypeError("axis_data 不能是None")

    field_name = OUT_FRONT_SURFACE_FIELD_BY_DIRECTION[direction]
    if not hasattr(axis_data, field_name):
        raise TypeError(f"axis_data 缺少字段 {field_name}")

    detected_x = getattr(axis_data, field_name)
    if isinstance(detected_x, bool) or not isinstance(detected_x, int):
        raise TypeError(f"{field_name} 必须是整数毫米值")
    if detected_x <= 0:
        raise ValueError(f"{field_name} 必须大于0，当前层可能是空层")
    return detected_x


def validate_xn_side_gun_origin_y(
    origin_pos: Sequence[int],
    minimum_gun_y_distance_mm: int,
) -> tuple[int, int, int, int]:

    minimum_gun_y_distance_mm = _require_int(
        "minimum_gun_y_distance_mm", minimum_gun_y_distance_mm
    )
    if minimum_gun_y_distance_mm <= 0:
        raise ValueError("minimum_gun_y_distance_mm 必须大于0")
    if not isinstance(origin_pos, (list, tuple)):
        raise TypeError("MachineConfig.origin_pos 必须是枪1至枪4的四元素列表")
    if len(origin_pos) != 4:
        raise ValueError("MachineConfig.origin_pos 必须依次包含枪1至枪4的四个Y零点")

    origins = []
    for gun_number, value in enumerate(origin_pos, start=1):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(
                f"MachineConfig.origin_pos[{gun_number - 1}] 必须是整数毫米值"
            )
        if value < 0:
            raise ValueError(f"枪{gun_number}的Y零点不能小于0")
        origins.append(value)

    for lower_index in range(3):
        distance = origins[lower_index + 1] - origins[lower_index]
        if distance < minimum_gun_y_distance_mm:
            raise ValueError(
                f"枪{lower_index + 1}与枪{lower_index + 2}的Y零点距离"
                f"{distance}mm小于{minimum_gun_y_distance_mm}mm"
            )
    return tuple(origins)


def limit_xn_side_y_targets_for_clearance(
    y_targets: Mapping[str, int],
    origin_pos: Sequence[int],
    minimum_gun_y_distance_mm: int,
) -> dict[str, int]:
    """
    PLC位置是喷枪相对自身零点向上的位移，因此喷枪绝对激光坐标为
    ``origin_pos[i] + Pos``。发生冲突时只减小较低喷枪的Pos。
    """

    if not isinstance(y_targets, Mapping):
        raise TypeError("y_targets 必须是映射")
    if set(y_targets) != set(XN_SIDE_Y_AXES):
        raise ValueError("y_targets 必须且只能包含 y1、y2、y3")
    origins = validate_xn_side_gun_origin_y(origin_pos, minimum_gun_y_distance_mm)

    safe_positions = []
    for axis_name in XN_SIDE_Y_AXES:
        value = y_targets[axis_name]
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{axis_name}目标位置必须是整数毫米值")
        if value < 0:
            raise ValueError(f"{axis_name}目标位置不能小于0")
        safe_positions.append(value)

    # 枪4没有Y轴，固定在origin_pos[3]，所以对应的相对位移为0。
    upper_relative_position = 0
    for lower_index in range(2, -1, -1):
        maximum_lower_position = (
            origins[lower_index + 1]
            + upper_relative_position
            - minimum_gun_y_distance_mm
            - origins[lower_index]
        )
        safe_positions[lower_index] = min(
            safe_positions[lower_index],
            maximum_lower_position,
        )
        upper_relative_position = safe_positions[lower_index]

    return {
        axis_name: safe_positions[index]
        for index, axis_name in enumerate(XN_SIDE_Y_AXES)
    }


def calculate_xn_side_y_targets(
    in_up_y: int,
    in_down_y: int,
    out_down_y: int,
    origin_pos: Sequence[int],
    machine_config: Mapping,
    minimum_gun_y_distance_mm: int,
) -> dict[str, int]:
    """把三条可移动边的激光Y坐标换算成y1/y2/y3相对零点偏移。

    枪4负责上外顶且高度固定。枪1、枪2、枪3分别负责下外底、下内底、
    上内顶；先按MachineConfig应用“上内减、下内加、下外减”的Y到达
    点偏移，再减去每把枪各自的安装零点。Y轴正位置表示喷枪从自身
    零点向上移动。最后从枪4向枪1执行防撞限幅。
    """

    adjusted_y = apply_xn_side_surface_y_offsets(
        {
            "in_up": in_up_y,
            "in_down": in_down_y,
            "out_down": out_down_y,
        },
        machine_config,
    )
    if not adjusted_y["in_up"] > adjusted_y["in_down"] > adjusted_y["out_down"]:
        raise ValueError("应用Y偏移后必须保持上内顶 > 下内底 > 下外底")

    gun1_zero_y, gun2_zero_y, gun3_zero_y, _ = validate_xn_side_gun_origin_y(
        origin_pos,
        minimum_gun_y_distance_mm,
    )
    targets = {
        "y1": adjusted_y["out_down"] - gun1_zero_y,
        "y2": adjusted_y["in_down"] - gun2_zero_y,
        "y3": adjusted_y["in_up"] - gun3_zero_y,
    }
    invalid_axes = [axis for axis, position in targets.items() if position < 0]
    if invalid_axes:
        raise ValueError(
            f"Y轴目标位置小于0：{', '.join(invalid_axes)}，请检查轮廓边和喷枪零点"
        )
    return limit_xn_side_y_targets_for_clearance(
        targets,
        origin_pos,
        minimum_gun_y_distance_mm,
    )


def calculate_out_front_target_x(
    detected_x: int,
    x_position: int,
    out_front_x_offset: int,
) -> int:
    """把雷达检测到的工件进深换算成云雀 X 轴目标位置。

    当前项目只喷涂工件外前表面，已确认公式为：
        target_x = detected_x - x_position - out_front_x_offset

    此函数只处理几何关系；轴限位检查将在生成 PLC 运动指令时进行。
    """
    values = {
        "detected_x": detected_x,
        "x_position": x_position,
        "out_front_x_offset": out_front_x_offset,
    }
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} 必须是整数毫米值")
        if value < 0:
            raise ValueError(f"{name} 不能小于0")

    target_x = detected_x - x_position - out_front_x_offset
    if target_x < 0:
        raise ValueError(
            "计算得到的云雀X目标位置小于0，请检查雷达检测值、x_position和out_front_x_offset"
        )
    return target_x


def _xn_side_config_axis_value(
    machine_config: Mapping,
    config_key: str,
    axis_name: str,
) -> int:
    """读取xn_side配置中同类轴共享的Y/X数组值。"""

    if config_key not in machine_config:
        raise KeyError(f"MachineConfig 缺少参数：{config_key}")
    values = machine_config[config_key]
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise ValueError(f"MachineConfig.{config_key} 必须是[y, x]两个整数")
    config_index = 0 if axis_name.startswith("y") else 1
    value = values[config_index]
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"MachineConfig.{config_key}[{config_index}] 必须是整数")
    return value


def build_xn_side_axis_commands(
    x_commands: Mapping[str, XAxisMotionCommand],
    y_targets: Mapping[str, int],
    machine_config: Mapping,
) -> dict[str, MovingAxisData]:
    """把已规划的四根X轴和三根Y轴合成单侧7轴离线指令。

    这个函数是几何/状态机与PLC槽位之间的安全适配层：它只生成按轴名
    排列的 ``AxisData``，不写 ``AxisList``、不设置使能、不发送PLC。
    X轴 ``Status`` 传递每把枪的开关粉状态，Y轴保持为0。
    """

    if not isinstance(x_commands, Mapping):
        raise TypeError("x_commands 必须是映射")
    if not isinstance(y_targets, Mapping):
        raise TypeError("y_targets 必须是映射")
    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "xn_side":
        raise ValueError("machine_config.type 必须是 xn_side")

    configured_axis_order = machine_config.get("axis_type")
    if tuple(configured_axis_order or ()) != XN_SIDE_AXIS_SEQUENCE:
        raise ValueError("MachineConfig.axis_type 与侧面云雀7轴顺序不一致")

    if set(x_commands) != set(XN_SIDE_X_AXES):
        raise ValueError("x_commands 必须且只能包含 x1、x2、x3、x4")
    if set(y_targets) != set(XN_SIDE_Y_AXES):
        raise ValueError("y_targets 必须且只能包含 y1、y2、y3")

    x_min = _xn_side_config_axis_value(machine_config, "min_limit_pos", "x1")
    x_max = _xn_side_config_axis_value(machine_config, "max_limit_pos", "x1")
    x_speed_max = _xn_side_config_axis_value(machine_config, "max_limit_speed", "x1")
    y_min = _xn_side_config_axis_value(machine_config, "min_limit_pos", "y1")
    y_max = _xn_side_config_axis_value(machine_config, "max_limit_pos", "y1")
    y_speed_max = _xn_side_config_axis_value(machine_config, "max_limit_speed", "y1")
    if x_min > x_max or y_min > y_max:
        raise ValueError("MachineConfig轴最小限位不能大于最大限位")
    if x_speed_max <= 0 or y_speed_max <= 0:
        raise ValueError("MachineConfig轴最大速度必须大于0")

    y_speed = machine_config.get("y_pos_speed")
    if isinstance(y_speed, bool) or not isinstance(y_speed, int):
        raise TypeError("MachineConfig.y_pos_speed 必须是整数")
    if not 0 < y_speed <= y_speed_max:
        raise ValueError("MachineConfig.y_pos_speed 必须大于0且不超过Y轴最大速度")

    axis_commands: dict[str, MovingAxisData] = {}
    for axis_name in XN_SIDE_AXIS_SEQUENCE:
        if axis_name.startswith("x"):
            command = x_commands[axis_name]
            if not all(
                hasattr(command, field_name)
                for field_name in ("Pos", "Speed", "Status")
            ):
                raise TypeError(f"{axis_name} 必须提供 Pos、Speed、Status")
            if _require_int(f"{axis_name}.Status", command.Status) not in (0, 1):
                raise ValueError(f"{axis_name}.Status 必须为0或1")
            if not x_min <= command.Pos <= x_max:
                raise ValueError(
                    f"{axis_name}.Pos={command.Pos} 超出X轴限位 {x_min}~{x_max}"
                )
            if not 0 <= command.Speed <= x_speed_max:
                raise ValueError(
                    f"{axis_name}.Speed={command.Speed} 超出X轴速度范围 0~{x_speed_max}"
                )
            axis_commands[axis_name] = MovingAxisData(
                Pos=command.Pos,
                Speed=command.Speed,
                Status=command.Status,
            )
            continue

        target = y_targets[axis_name]
        if isinstance(target, bool) or not isinstance(target, int):
            raise TypeError(f"{axis_name}目标位置必须是整数")
        if not y_min <= target <= y_max:
            raise ValueError(f"{axis_name}.Pos={target} 超出Y轴限位 {y_min}~{y_max}")
        axis_commands[axis_name] = MovingAxisData(
            Pos=target,
            Speed=y_speed,
            Status=0,
        )
    return axis_commands


def build_xn_side_return_commands(
    machine_config: Mapping,
    *,
    clean_mode: bool,
) -> dict[str, MovingAxisData]:
    """构造单侧云雀的安全回位或清理位指令。

    普通回位时 X/Y 都使用 ``safe_pos``；清理模式时 Y 使用
    ``safe_pos``，四根 X 轴使用 ``max_limit_pos``。
    """

    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "xn_side":
        raise ValueError("machine_config.type 必须是 xn_side")
    if not isinstance(clean_mode, bool):
        raise TypeError("clean_mode 必须是布尔值")

    y_target = _xn_side_config_axis_value(machine_config, "safe_pos", "y1")
    x_config_key = "max_limit_pos" if clean_mode else "safe_pos"
    x_target = _xn_side_config_axis_value(machine_config, x_config_key, "x1")
    x_speed = machine_config.get("x_pos_speed")
    if isinstance(x_speed, bool) or not isinstance(x_speed, int):
        raise TypeError("MachineConfig.x_pos_speed 必须是整数")

    x_commands = {
        axis_name: XAxisMotionCommand(
            Pos=x_target,
            Speed=x_speed,
            Status=0,
            Phase=PHASE_POSITIONING_TO_FIXED,
        )
        for axis_name in XN_SIDE_X_AXES
    }
    y_targets = {axis_name: y_target for axis_name in XN_SIDE_Y_AXES}
    return build_xn_side_axis_commands(x_commands, y_targets, machine_config)


def build_out_lift_return_command(
    machine_config: Mapping,
    *,
    clean_mode: bool,
) -> MovingAxisData:
    """构造2D变频设备的安全命令。

    该设备的独立槽使用 ``Pos=X, Speed=YMin, Status=YMax``
    的复合编码；回位时 YMin/YMax 都等于 Y 安全位。
    """

    if not isinstance(machine_config, Mapping):
        raise TypeError("machine_config 必须是映射")
    if machine_config.get("type") != "out_lift":
        raise ValueError("machine_config.type 必须是 out_lift")
    if not isinstance(clean_mode, bool):
        raise TypeError("clean_mode 必须是布尔值")

    y_safe = _out_lift_limit(machine_config, "safe_pos", "y")
    x_key = "max_limit_pos" if clean_mode else "safe_pos"
    x_target = _out_lift_limit(machine_config, x_key, "x")
    y_min = _out_lift_limit(machine_config, "min_limit_pos", "y")
    y_max = _out_lift_limit(machine_config, "max_limit_pos", "y")
    x_min = _out_lift_limit(machine_config, "min_limit_pos", "x")
    x_max = _out_lift_limit(machine_config, "max_limit_pos", "x")
    if not y_min <= y_safe <= y_max:
        raise ValueError("out_lift Y安全位超出轴限位")
    if not x_min <= x_target <= x_max:
        raise ValueError("out_lift X回位目标超出轴限位")
    return MovingAxisData(Pos=x_target, Speed=y_safe, Status=y_safe)


def build_available_xn_side_moving_frame(
    left_commands: Mapping[str, MovingAxisData] | None,
    right_commands: Mapping[str, MovingAxisData] | None,
    heartbeat: int,
    *,
    chain_running: bool = True,
    left_2d_command: MovingAxisData | None = None,
    right_2d_command: MovingAxisData | None = None,
) -> SendMovingFrameData:
    """把左/右云雀及2D变频指令装入固定110字节PLC数据结构。

    左右设备的 ``z_position`` 可以不同，因此两个命令允许独立为 ``None``。
    没到站的一侧保持7个零轴槽并且不设置该侧使能位；总使能和链条运行
    位仍保持，使工件能够继续向后续设备移动。该函数不连接或发送PLC。
    """

    if isinstance(heartbeat, bool) or not isinstance(heartbeat, int):
        raise TypeError("heartbeat 必须是整数")
    if not HEARTBEAT_MIN <= heartbeat <= HEARTBEAT_MAX:
        raise ValueError(f"heartbeat 必须在 {HEARTBEAT_MIN}~{HEARTBEAT_MAX} 范围内")
    if not isinstance(chain_running, bool):
        raise TypeError("chain_running 必须是布尔值")

    side_commands = {
        "left": left_commands,
        "right": right_commands,
    }
    axis_list = [MovingAxisData() for _ in range(14)]
    enable = ENABLE_MASTER
    for side, commands in side_commands.items():
        if commands is None:
            continue
        if not isinstance(commands, Mapping):
            raise TypeError(f"{side}_commands 必须是映射或None")
        if tuple(commands) != XN_SIDE_AXIS_SEQUENCE:
            raise ValueError(f"{side}_commands 必须按侧面云雀7轴固定顺序排列")
        slot_offset = 0 if side == "left" else 7
        enable |= ENABLE_LEFT_XN_SIDE if side == "left" else ENABLE_RIGHT_XN_SIDE
        for local_index, axis_name in enumerate(XN_SIDE_AXIS_SEQUENCE):
            command = commands[axis_name]
            if not isinstance(command, MovingAxisData):
                raise TypeError(f"{side}.{axis_name} 必须是PLC AxisData")
            status = _require_int(f"{side}.{axis_name}.Status", command.Status)
            allowed_statuses = (0, 1) if axis_name.startswith("x") else (0,)
            if status not in allowed_statuses:
                raise ValueError(f"{side}.{axis_name}.Status 必须为{allowed_statuses}")
            axis_list[slot_offset + local_index] = MovingAxisData(
                Pos=command.Pos,
                Speed=command.Speed,
                Status=status,
            )

    variable_2d_commands = {
        "left": left_2d_command,
        "right": right_2d_command,
    }
    for side, command in variable_2d_commands.items():
        if command is not None and not isinstance(command, MovingAxisData):
            raise TypeError(f"{side}_2d_command 必须是PLC AxisData或None")

    return SendMovingFrameData(
        Enable=enable,
        Gun_Cont1=0,
        Gun_Cont2=0,
        HeartBeat=heartbeat,
        Operate=OPERATE_CHAIN_RUN if chain_running else 0,
        AxisList=axis_list,
        Left2DLiftData=(
            MovingAxisData()
            if left_2d_command is None
            else MovingAxisData(
                Pos=left_2d_command.Pos,
                Speed=left_2d_command.Speed,
                Status=left_2d_command.Status,
            )
        ),
        Right2DLiftData=(
            MovingAxisData()
            if right_2d_command is None
            else MovingAxisData(
                Pos=right_2d_command.Pos,
                Speed=right_2d_command.Speed,
                Status=right_2d_command.Status,
            )
        ),
    )


def build_dual_xn_side_moving_frame(
    left_commands: Mapping[str, MovingAxisData],
    right_commands: Mapping[str, MovingAxisData],
    heartbeat: int,
    *,
    chain_running: bool = True,
) -> SendMovingFrameData:
    """兼容原有离线工具：要求左右两侧同时提供7轴指令。"""

    if left_commands is None or right_commands is None:
        raise TypeError("双侧运动帧必须同时提供left_commands和right_commands")
    return build_available_xn_side_moving_frame(
        left_commands,
        right_commands,
        heartbeat,
        chain_running=chain_running,
    )
