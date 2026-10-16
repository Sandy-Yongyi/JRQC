"""来料首帧选择及云雀收X、定位Y的关粉待料状态。"""

from dataclasses import dataclass, field

from model.motionplan.commands import build_xn_side_axis_commands, build_xn_side_return_commands
from model.motionplan.constants import XN_SIDE_X_AXES, XN_SIDE_Y_AXES
from model.motionplan.contracts import AxisGroupArrival


def incoming_workpiece_window(stack, station_index, back_frames, has_workpiece):
    """找站位前最近一件的前缘，只取该件已采集的连续轮廓，不跨空隙。"""
    leading = next((i for i in range(station_index - 1, -1, -1)
                    if has_workpiece(stack[i])), None)
    if leading is None:
        return None
    start = leading
    while start > max(0, leading - back_frames) and has_workpiece(stack[start - 1]):
        start -= 1
    return start, leading + 1


@dataclass
class XnSideStandbySequence:
    """整侧X连续确认回安全位后才放行Y；X偏离时重新冻结Y。"""

    phase: str = "idle"
    held_y: dict[str, int] = field(default_factory=dict)
    x_arrival: AxisGroupArrival = field(default_factory=AxisGroupArrival)

    def advance(self, machine, actual, y_targets, tolerance, confirmations):
        safe = build_xn_side_return_commands(machine, clean_mode=False)
        x_targets = {axis: safe[axis].Pos for axis in XN_SIDE_X_AXES}
        x_inside = all(abs(actual[a] - p) <= tolerance for a, p in x_targets.items())
        changed = self.x_arrival.targets != x_targets
        if self.phase == "idle" or (self.phase != "x" and (not x_inside or changed)):
            self.phase = "x"
            self.held_y = {a: actual[a] for a in XN_SIDE_Y_AXES}
            self.x_arrival = AxisGroupArrival()
        x_ready = self.x_arrival.observe(x_targets, actual, tolerance, confirmations)
        if not x_ready:
            target_y = self.held_y
        elif y_targets is None:
            if self.phase != "waiting_geometry":
                self.held_y = {a: actual[a] for a in XN_SIDE_Y_AXES}
            self.phase = "waiting_geometry"
            target_y = self.held_y
        else:
            self.phase = "y"
            target_y = y_targets
        commands = build_xn_side_axis_commands(
            {a: safe[a] for a in XN_SIDE_X_AXES}, target_y, machine,
        )
        return commands, x_ready
