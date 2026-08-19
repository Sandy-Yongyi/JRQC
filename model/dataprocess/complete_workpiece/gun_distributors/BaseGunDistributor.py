from model.formats.complete_workpiece.BlockDataFormat import DistribeGunData, GunGroupData


class BaseGunDistributor:
    def _enable_gun(self, gun_id, y_upper, y_downer, angle):
        return DistribeGunData(
            gun_id=gun_id,
            gun_y_enable=1,
            gun_y_upper=y_upper,
            gun_y_downer=y_downer,
            gun_r_angle=angle,
        )

    def _zero_gun(self, gun_id):
        return DistribeGunData(gun_id=gun_id, gun_y_enable=0, gun_y_upper=0, gun_y_downer=0, gun_r_angle=0)

    def _build_side_zero_groups(self, axis_num):
        zero = [self._zero_gun(i) for i in range(axis_num)]
        return [
            GunGroupData(group_type="outside", group_id=0, gundata_list=zero),
            GunGroupData(group_type="inside", group_id=0, gundata_list=zero),
        ]

    def _build_default_zero_groups(self, axis_num):
        zero = [self._zero_gun(i) for i in range(axis_num)]
        return [GunGroupData(group_type="default", group_id=0, gundata_list=zero)]

    def _build_updown_zero_groups(self):
        return [
            GunGroupData(group_type="up", group_id=0, gundata_list=[self._zero_gun(2), self._zero_gun(3)]),
            GunGroupData(group_type="down", group_id=0, gundata_list=[self._zero_gun(0), self._zero_gun(1)]),
        ]

    def _calc_y_offset(self, target_y, origin_y, limit_y):
        y = target_y - origin_y
        if y <= 0 or y > limit_y:
            return None
        return y

    def _inside_guard(self, gun_id, y_offset, origin_y, inside, safety_y_distance, angle):
        y_real = origin_y + y_offset
        y_max = inside.subinside_y_max - safety_y_distance
        y_min = inside.subinside_y_min + safety_y_distance

        if y_min < y_real < y_max:
            return self._enable_gun(gun_id, y_offset, y_offset, angle)
        return self._zero_gun(gun_id)

    @staticmethod
    def check_if_cabinet(blockdata, x_range, y_range, z_range):
        if not blockdata.outside_data:
            return False

        outside_data = blockdata.outside_data[0]
        actual_x_range = (outside_data.outside_x_max or 0) - (outside_data.outside_x_min or 0)
        actual_y_range = (outside_data.outside_y_max or 0) - (outside_data.outside_y_min or 0)
        actual_z_range = (outside_data.outside_z_max or 0) - (outside_data.outside_z_min or 0)

        return actual_x_range >= x_range and actual_y_range >= y_range and actual_z_range >= z_range
