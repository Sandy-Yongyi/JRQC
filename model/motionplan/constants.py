"""运动规划与 PLC 协议常量；兼容既有位定义和轴顺序。"""

DIRECTION_TO_END = "to_end"
DIRECTION_TO_START = "to_start"
PHASE_POSITIONING_TO_START = "positioning_to_start"
PHASE_TO_END = DIRECTION_TO_END
PHASE_TO_START = DIRECTION_TO_START
PHASE_RECOVERING = "recovering"
PHASE_RETURNING_INSIDE = "returning_inside"
PHASE_HOLD = "hold"
PHASE_POSITIONING_TO_FIXED = "positioning_to_fixed"
PHASE_FIXED = "fixed"
# SendMovingFrameData.Enable
ENABLE_MASTER = 0x0001
ENABLE_LEFT_XN_SIDE = 0x0002
ENABLE_RIGHT_XN_SIDE = 0x0004
ENABLE_CLEAN_MODE = 0x8000
# PLC -> PC ReceiveMovingFrameData.Operate。该字段在现有协议
# 中以 Int16 解码，使用前必须归一化为无符号 16 位。
PLC_OPERATE_LEFT_XN_SIDE = 0x0002
PLC_OPERATE_RIGHT_XN_SIDE = 0x0004
PLC_OPERATE_CLEAN_MODE = 0x8000
# SendMovingFrameData.Operate
OPERATE_RESERVED = 0x0001
OPERATE_CHAIN_RUN = 0x0002
# HeartBeat 原样回传 PLC 的 Int16 心跳，包含有符号负值。
HEARTBEAT_MIN = -0x8000
HEARTBEAT_MAX = 0x7FFF
# PLC负责人确认：ChainCountCM 每增加1，链条实际前进10mm。
CHAIN_COUNT_CM_STEP_MM = 10
# 业务负责人确认：左右两侧云雀都把逐层轮廓的最小X值视为外前表面。
OUT_FRONT_SURFACE_FIELD_BY_DIRECTION = {
    "left": "V_Axis_Min",
    "right": "V_Axis_Min",
}
# 四组云雀喷枪按物理编号从下到上对应的车架表面。最高枪4高度固定，
# 所以没有y4；枪1、枪2、枪3分别由y1、y2、y3控制。
XN_SIDE_GUN_SURFACE_SEQUENCE = (
    "out_down",
    "in_down",
    "in_up",
    "out_up",
)
# 单台侧面云雀写入PLC前的固定轴顺序。左侧最终映射到 AxisList[0:7]，
# 右侧最终映射到 AxisList[7:14]；本模块暂时只生成按名称索引的离线指令。
XN_SIDE_AXIS_SEQUENCE = (
    "y1",
    "x1",
    "y2",
    "x2",
    "y3",
    "x3",
    "x4",
)
XN_SIDE_X_AXES = ("x1", "x2", "x3", "x4")
XN_SIDE_Y_AXES = ("y1", "y2", "y3")
# X轴前/后端点使用的设备偏移；规划逻辑位于模型层，不依赖离线工具。
X_AXIS_OFFSET_KEYS = {
    "x1": ("out_front_x_offset", "out_after_x_offset"),
    "x2": ("in_front_x_offset", "in_after_x_offset"),
    "x3": ("in_front_x_offset", "in_after_x_offset"),
    "x4": ("out_front_x_offset", "out_after_x_offset"),
}
# 业务负责人确认的Y特征点偏移方向。这里修正的是激光坐标中的
# 喷枪到达点；随后再换算成y1/y2/y3相对各自零点的轴位置。
XN_SIDE_SURFACE_Y_OFFSET_RULES = {
    "out_up": ("out_up_y_offset", 1),
    "in_up": ("in_up_y_offset", -1),
    "in_down": ("in_down_y_offset", 1),
    "out_down": ("out_down_y_offset", -1),
}
VARIABLE_2D_WINDOW_EMPTY = "empty"
VARIABLE_2D_WINDOW_MIXED = "mixed"
VARIABLE_2D_WINDOW_VALID = "valid"
OUT_LIFT_LIMIT_INDEX = {"y": 0, "x": 1}
