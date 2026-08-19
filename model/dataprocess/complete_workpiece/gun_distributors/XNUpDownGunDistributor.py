from model.motionplan.MachineAxisMap import get_axis_position_limits
from model.formats.complete_workpiece.BlockDataFormat import GunGroupData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor


class XNUpDownGunDistributor(BaseGunDistributor):
    def distribute(self, blockdata, mcfg, spray_width_distance):
        if not blockdata.outside_data or not blockdata.inside_data:
            return self._build_updown_zero_groups()

        out = blockdata.outside_data[0]
        inside_blocks = blockdata.inside_data[0].subinside_datalist
        if not inside_blocks:
            return self._build_updown_zero_groups()

        origin_pos = mcfg.get("origin_pos", [675, 930, 1180, 1430])
        _, limit_y = get_axis_position_limits(mcfg, "y", 0, 400)
        y_distance = mcfg.get("y_distance", 100)
        safety_y_distance = spray_width_distance + y_distance

        guns = [self._zero_gun(i) for i in range(4)]

        if out.outside_y_max is not None:
            up_locate_y = self._calc_y_offset(out.outside_y_max + safety_y_distance, origin_pos[3], limit_y)
            if up_locate_y is not None:
                guns[3] = self._enable_gun(3, up_locate_y, up_locate_y, 0)
                guns[2] = self._inside_guard(
                    gun_id=2,
                    y_offset=up_locate_y,
                    origin_y=origin_pos[2],
                    inside=inside_blocks[0],
                    safety_y_distance=safety_y_distance,
                    angle=135,
                )

        if out.outside_y_min is not None:
            down_locate_y = self._calc_y_offset(out.outside_y_min - safety_y_distance, origin_pos[0], limit_y)
            if down_locate_y is not None:
                guns[0] = self._enable_gun(0, down_locate_y, down_locate_y, 0)
                guns[1] = self._inside_guard(
                    gun_id=1,
                    y_offset=down_locate_y,
                    origin_y=origin_pos[1],
                    inside=inside_blocks[-1],
                    safety_y_distance=safety_y_distance,
                    angle=45,
                )

        return [
            GunGroupData(group_type="up", group_id=0, gundata_list=[guns[2], guns[3]]),
            GunGroupData(group_type="down", group_id=0, gundata_list=[guns[0], guns[1]]),
        ]
