from model.formats.complete_workpiece.BlockDataFormat import GunGroupData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor

"""内顶分枪逻辑。

规则：
1. 内顶按配置轴数分枪，当前场景通常只有 1 把枪。
2. 只检查 `blockdata.inside_data` 是否存在；若为空则全部关闭并直接返回。
3. 对每一列 `inside_data`，使用该列自己的内侧 Y 最小/最大值计算选枪窗口。
    若一列中存在多个子分区，则只取该列的第一个子分区参与计算。
    当 `origin_pos[i]` 落在 `[inside_y_min + in_down_y_offset + spray_radius, inside_y_max - in_up_y_offset - spray_radius]` 范围内时选中该枪。
4. `gun_y_downer` / `gun_y_upper` 固定为 0，`gun_r_angle` 固定为 0。
"""


class InUpGunDistributor(BaseGunDistributor):
    def distribute(self, blockdata, mcfg):
        axis_num = int(mcfg.get("spray_num", 1) or 1)
        zero_guns = [self._zero_gun(i) for i in range(axis_num)]

        if not blockdata.inside_data or not blockdata.outside_data:
            return [GunGroupData(group_type="inside", group_id=0, gundata_list=zero_guns)]

        out = blockdata.outside_data[0]
        if out.outside_y_min is None or out.outside_y_max is None:
            return [GunGroupData(group_type="inside", group_id=0, gundata_list=zero_guns)]

        origin_pos = mcfg.get("origin_pos", [0])
        in_up_y_offset = int(mcfg.get("in_up_y_offset", 0) or 0)
        in_down_y_offset = int(mcfg.get("in_down_y_offset", 0) or 0)
        spray_radius = int(mcfg.get("spray_radius", 0) or 0)

        gun_groups = []
        for column_index, inside_column in enumerate(blockdata.inside_data):
            guns = [self._zero_gun(i) for i in range(axis_num)]
            inside_y_min, inside_y_max = self._get_inside_column_y_range(inside_column)
            if inside_y_min is None or inside_y_max is None:
                gun_groups.append(
                    GunGroupData(
                        group_type="inside",
                        group_id=inside_column.inside_id if inside_column.inside_id is not None else column_index,
                        gundata_list=guns,
                    )
                )
                continue

            select_y_min = inside_y_min + in_down_y_offset + spray_radius
            select_y_max = inside_y_max - in_up_y_offset - spray_radius
            for idx in range(min(axis_num, len(origin_pos))):
                origin_y = int(origin_pos[idx] or 0)
                if select_y_min <= origin_y <= select_y_max:
                    guns[idx] = self._enable_gun(idx, 0, 0, 0)
            gun_groups.append(
                GunGroupData(
                    group_type="inside",
                    group_id=inside_column.inside_id if inside_column.inside_id is not None else column_index,
                    gundata_list=guns,
                )
            )

        return gun_groups

    @staticmethod
    def _get_inside_column_y_range(inside_column):
        subinside_list = getattr(inside_column, "subinside_datalist", None) or []
        if not subinside_list:
            return None, None
        first_subinside = subinside_list[0]
        if getattr(first_subinside, "subinside_y_min", None) is None or getattr(first_subinside, "subinside_y_max", None) is None:
            return None, None
        return int(first_subinside.subinside_y_min), int(first_subinside.subinside_y_max)
