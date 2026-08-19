from model.formats.complete_workpiece.BlockDataFormat import GunGroupData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor

"""外顶分枪逻辑。

规则：
1. 只判断当前工件是否为柜体。
2. 是柜体时，唯一喷枪使能；不是柜体时保持关闭。
3. 外顶只有一个轴，因此 `gun_y_downer` / `gun_y_upper` / `gun_r_angle` 固定为 0。
"""


class OutUpGunDistributor(BaseGunDistributor):
    def distribute(self, blockdata, mcfg):
        axis_num = int(mcfg.get("spray_num", 1) or 1)
        guns = [self._zero_gun(i) for i in range(axis_num)]
        guns[0] = self._enable_gun(0, 0, 0, 0)
        return [GunGroupData(group_type="default", group_id=0, gundata_list=guns)]
