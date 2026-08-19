from model.formats.complete_workpiece.BlockDataFormat import GunGroupData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor

"""外底分枪逻辑。

规则：
1. 外底按配置轴数分枪，当前场景通常只有 1 把枪。
2. 只使用 `blockdata.outside_data`，不参与 `inside_data` 判断。
3. 目标位置按 `outside_y_min - out_down_y_offset - spray_radius` 计算。
4. `gun_y_downer` 与 `gun_y_upper` 取同一个值：`target_y - origin_pos[i]`。
5. 当该值 >= 0 时使能喷枪，否则保持关闭。
6. `gun_r_angle` 固定为 0。
"""


class OutDownGunDistributor(BaseGunDistributor):
    def distribute(self, blockdata, mcfg):
        axis_num = int(mcfg.get("spray_num", 1) or 1)
        guns = [self._zero_gun(i) for i in range(axis_num)]
        if not blockdata.outside_data:
            return [GunGroupData(group_type="down", group_id=0, gundata_list=guns)]

        out = blockdata.outside_data[0]
        if out.outside_y_min is None:
            return [GunGroupData(group_type="down", group_id=0, gundata_list=guns)]

        origin_pos = mcfg.get("origin_pos", [0])
        out_down_y_offset = int(mcfg.get("out_down_y_offset", 0) or 0)
        spray_radius = int(mcfg.get("spray_radius", 0) or 0)

        for idx in range(min(axis_num, len(origin_pos))):
            locate_y = int(out.outside_y_min) - out_down_y_offset - spray_radius - int(origin_pos[idx] or 0)
            if locate_y >= 0:
                guns[idx] = self._enable_gun(idx, locate_y, locate_y, 0)

        return [GunGroupData(group_type="down", group_id=0, gundata_list=guns)]
