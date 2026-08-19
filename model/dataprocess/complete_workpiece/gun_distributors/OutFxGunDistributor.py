from model.formats.complete_workpiece.BlockDataFormat import DistribeGunData, GunGroupData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor


class OutFxGunDistributor(BaseGunDistributor):
    """仿形升降机完整工件模式的外侧共享 Y 分枪器。"""

    def distribute(self, blockdata, machine_cfg, gun_distance):
        axis_num = int(machine_cfg.get("spray_num", 0) or 0)
        origin_pos = [int(value or 0) for value in machine_cfg.get("origin_pos", [])]
        zero_guns = [self._zero_gun(index) for index in range(axis_num)]
        if axis_num <= 0 or not origin_pos or not getattr(blockdata, "outside_data", None):
            return [GunGroupData(group_type="outside", group_id=0, gundata_list=zero_guns)]

        outside = blockdata.outside_data[0]
        outside_y_min = getattr(outside, "outside_y_min", None)
        outside_y_max = getattr(outside, "outside_y_max", None)
        if outside_y_min is None or outside_y_max is None:
            return [GunGroupData(group_type="outside", group_id=0, gundata_list=zero_guns)]

        down_offset = int(machine_cfg.get("out_down_y_offset", 100) or 100)
        up_offset = int(machine_cfg.get("out_up_y_offset", 100) or 100)
        target_y_min = int(outside_y_min) - down_offset
        target_y_max = int(outside_y_max) + up_offset
        scan_span = self._resolve_scan_span(origin_pos, gun_distance)
        guns = self._build_outside_guns(origin_pos, axis_num, target_y_min, target_y_max, scan_span)
        return [GunGroupData(group_type="outside", group_id=0, gundata_list=guns)]

    def _build_outside_guns(self, origin_pos, axis_num, target_y_min, target_y_max, scan_span):
        guns = [self._zero_gun(index) for index in range(axis_num)]
        if target_y_max <= target_y_min:
            return guns

        max_index = min(axis_num, len(origin_pos)) - 1
        if max_index < 0:
            return guns
        top_index = max_index if target_y_max > origin_pos[max_index] + scan_span else self._find_top_down_point_gun(target_y_max, origin_pos, axis_num, scan_span)
        bottom_index = 0 if target_y_min < origin_pos[0] else self._find_top_down_point_gun(target_y_min, origin_pos, axis_num, scan_span)
        if top_index is None or bottom_index is None:
            return guns

        for index in range(min(top_index, bottom_index), max(top_index, bottom_index) + 1):
            guns[index] = DistribeGunData(gun_id=index, gun_y_enable=1, gun_y_downer=0, gun_y_upper=max(0, scan_span), gun_r_angle=0)
        return guns

    @staticmethod
    def _resolve_scan_span(origin_pos, gun_distance):
        differences = [abs(origin_pos[index + 1] - origin_pos[index]) for index in range(len(origin_pos) - 1)]
        positive_differences = [difference for difference in differences if difference > 0]
        return min(positive_differences) if positive_differences else int(gun_distance or 430)

    @staticmethod
    def _find_top_down_point_gun(point, origin_pos, axis_num, scan_span):
        for index in range(min(axis_num, len(origin_pos)) - 1, -1, -1):
            if origin_pos[index] <= point <= origin_pos[index] + scan_span:
                return index
        return None
