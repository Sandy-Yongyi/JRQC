import os
from model.formats.complete_workpiece.BlockDataFormat import DistribeGunData, GunGroupData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor
from model.utils.TomlLoader import TomlLoader

"""
xn_side 侧面设备分枪逻辑整理：

1. outside 分枪
   - 扫描窗口按 [origin_pos[i], origin_pos[i] + scan_span] 计算。
   - 使用 outside_y_max + y_offset 与 outside_y_min - y_offset。
   - 从上往下分别找到首枪和尾枪。
   - 首尾之间的枪全部使能。
   - 所有选中枪统一：
       gun_y_downer = 0
       gun_y_upper  = scan_span
       gun_r_angle  = 0

2. inside 分枪
   - 这里的 inside_data 是“按列”组织的，必须逐列计算。
   - 每一列都有自己的 subinside_datalist，且每列的分枪结果都可能不同。
   - 因此 inside 分枪结果需要按列分别保存，而不能只保留一组总的 inside 分枪。

   对单列 inside 的规则：
   2.1 span >= scan_span
       - 每个子分区单独计算。
       - 该子分区内部的选枪连续；不同子分区之间允许不连续。
       - 统一：gun_y_downer = 0, gun_y_upper = scan_spant。

   2.2 min_recip_distance < span < scan_span
       - 先找“最小可覆盖分区”作为基准分区。
       - 基准分区要求 adjusted_y_min 与 adjusted_y_max 同时落在同一把枪的原始窗口内。
       - 由该枪确定 temp = adjusted_y_min - origin_pos[i]。
       - 统一：
           gun_y_downer = temp
           gun_y_upper  = temp + span
       - 其余分区再按新的窗口 [origin_pos[i] + temp, origin_pos[i] + scan_span] 查找是否可进枪。

   2.3 0 < span <= min_recip_distance
       - 取中间值 center。
       - 先找从上往下第一把能命中 center 的枪，作为基准枪。
       - temp = center - origin_pos[i]。
       - 其余分区按 [origin_pos[i] + temp, origin_pos[i] + scan_span] 查找是否可进枪。
       - 统一：gun_y_downer = temp, gun_y_upper = temp。
"""


class XNSharedYGunDistributor(BaseGunDistributor):
    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.path.join(os.getcwd(), "model", "tomls", "SprayConfig.toml"))

    def distribute(self, blockdata, mcfg, gun_distance):
        axis_num = int(mcfg.get("spray_num", 5))
        origin_pos = mcfg.get("origin_pos", [1550, 2180, 2780, 3410, 4040])
        outside_up_y_offset = int(mcfg.get("out_up_y_offset", 100) or 100)
        outside_down_y_offset = int(mcfg.get("out_down_y_offset", 100) or 100)
        inside_up_y_offset = int(mcfg.get("in_up_y_offset", 100) or 100)
        inside_down_y_offset = int(mcfg.get("in_down_y_offset", 100) or 100)
        min_recip_distance = int(self.spray_cfg.get("min_recip_distance", 60) or 60)
        scan_span = self._resolve_scan_span(origin_pos, gun_distance)

        if not origin_pos or not blockdata.outside_data:
            return self._build_side_zero_groups(axis_num)

        out = blockdata.outside_data[0]
        inside_columns = blockdata.inside_data or []
        outside_guns = self._build_side_outside_guns(
            out,
            origin_pos,
            axis_num,
            outside_up_y_offset,
            outside_down_y_offset,
            scan_span,
        )

        if not inside_columns:
            return [
                GunGroupData(group_type="outside", group_id=0, gundata_list=outside_guns),
                GunGroupData(group_type="inside", group_id=0, gundata_list=[self._zero_gun(i) for i in range(axis_num)]),
            ]

        inside_groups = []
        for column_index, inside_column in enumerate(inside_columns):
            subinside_list = inside_column.subinside_datalist or []
            inside_guns = self._build_side_inside_guns(
                subinside_list,
                origin_pos,
                axis_num,
                inside_up_y_offset,
                inside_down_y_offset,
                min_recip_distance,
                scan_span,
            )
            inside_groups.append(
                GunGroupData(
                    group_type="inside",
                    group_id=inside_column.inside_id if inside_column.inside_id is not None else column_index,
                    gundata_list=inside_guns,
                )
            )

        return [
            GunGroupData(group_type="outside", group_id=0, gundata_list=outside_guns),
            *inside_groups,
        ]

    @staticmethod
    def _resolve_scan_span(origin_pos, gun_distance):
        if len(origin_pos) >= 2:
            diffs = [abs(int(origin_pos[i + 1]) - int(origin_pos[i])) for i in range(len(origin_pos) - 1)]
            diffs = [diff for diff in diffs if diff > 0]
            if diffs:
                return min(diffs)
        return int(gun_distance or 430)

    def _build_side_outside_guns(self, out, origin_pos, axis_num, up_y_offset, down_y_offset, scan_span):
        guns = [self._zero_gun(i) for i in range(axis_num)]

        if out.outside_y_max is None or out.outside_y_min is None:
            return guns

        target_y_max = out.outside_y_max + up_y_offset
        target_y_min = out.outside_y_min - down_y_offset
        if target_y_max <= target_y_min:
            return guns

        max_idx = min(axis_num, len(origin_pos)) - 1
        if max_idx < 0:
            return guns
        top_scan_upper = origin_pos[max_idx] + scan_span
        if target_y_max > top_scan_upper:
            top_idx = max_idx
        else:
            top_idx = self._find_top_down_point_gun(target_y_max, origin_pos, axis_num, scan_span)
        if target_y_min < origin_pos[0]:
            bottom_idx = 0
        else:
            bottom_idx = self._find_top_down_point_gun(target_y_min, origin_pos, axis_num, scan_span)
        if top_idx is None or bottom_idx is None:
            return guns

        selected_indices = range(min(top_idx, bottom_idx), max(top_idx, bottom_idx) + 1)
        gun_y_upper = max(0, scan_span)
        for idx in selected_indices:
            guns[idx] = DistribeGunData(gun_id=idx, gun_y_enable=1, gun_y_downer=0, gun_y_upper=gun_y_upper, gun_r_angle=0)
        return guns

    def _build_side_inside_guns(self, inside_blocks, origin_pos, axis_num, up_y_offset, down_y_offset, min_recip_distance, scan_span):
        guns = [self._zero_gun(i) for i in range(axis_num)]
        normalized_blocks = self._normalize_inside_blocks(inside_blocks, up_y_offset, down_y_offset)
        if not normalized_blocks:
            return guns

        min_span = min(block["span"] for block in normalized_blocks)

        if min_span >= scan_span:
            return self._build_inside_large_guns(normalized_blocks, origin_pos, axis_num, scan_span)

        if min_recip_distance < min_span < scan_span:
            return self._build_inside_medium_guns(normalized_blocks, origin_pos, axis_num, scan_span)

        if 0 < min_span <= min_recip_distance:
            return self._build_inside_small_guns(normalized_blocks, origin_pos, axis_num, scan_span)

        return guns

    @staticmethod
    def _normalize_inside_blocks(inside_blocks, up_y_offset, down_y_offset):
        normalized = []
        for block in inside_blocks:
            if block.subinside_y_min is None or block.subinside_y_max is None:
                continue
            y_min = block.subinside_y_min + down_y_offset
            y_max = block.subinside_y_max - up_y_offset
            if y_max <= y_min:
                continue
            normalized.append({
                "y_min": y_min,
                "y_max": y_max,
                "span": y_max - y_min,
                "center": (y_max + y_min) / 2,
            })
        return normalized

    def _build_inside_large_guns(self, inside_blocks, origin_pos, axis_num, scan_span):
        enabled_indices = set()

        for block in inside_blocks:
            for idx in range(min(axis_num, len(origin_pos)) - 1, -1, -1):
                gun_y_min = origin_pos[idx]
                gun_y_max = origin_pos[idx] + scan_span
                if block["y_min"] < gun_y_min and gun_y_max < block["y_max"]:
                    enabled_indices.add(idx)

        gun_y_upper = max(0, scan_span)
        return self._build_fixed_guns(axis_num, enabled_indices, 0, gun_y_upper)

    def _build_inside_medium_guns(self, inside_blocks, origin_pos, axis_num, scan_span):
        anchor_block = None
        anchor_idx = None
        for block in sorted(inside_blocks, key=lambda item: item["span"]):
            gun_idx = self._find_top_down_cover_gun(block["y_min"], block["y_max"], origin_pos, axis_num, scan_span)
            if gun_idx is not None:
                anchor_block = block
                anchor_idx = gun_idx
                break

        if anchor_block is None or anchor_idx is None:
            return [self._zero_gun(i) for i in range(axis_num)]

        gun_y_downer = max(0, anchor_block["y_min"] - origin_pos[anchor_idx])
        gun_y_upper = max(gun_y_downer, gun_y_downer + anchor_block["span"])

        enabled_indices = set()
        for block in inside_blocks:
            gun_idx = self._find_top_down_shifted_cover_gun(
                y_min=block["y_min"],
                y_max=block["y_max"],
                origin_pos=origin_pos,
                axis_num=axis_num,
                lower_shift=gun_y_downer,
                upper_shift=gun_y_upper,
                excluded_indices=enabled_indices,
            )
            if gun_idx is not None:
                enabled_indices.add(gun_idx)

        return self._build_fixed_guns(axis_num, enabled_indices, gun_y_downer, gun_y_upper)

    def _build_inside_small_guns(self, inside_blocks, origin_pos, axis_num, scan_span):
        guns = [self._zero_gun(i) for i in range(axis_num)]
        ordered_blocks = sorted(inside_blocks, key=lambda block: block["y_max"], reverse=True)

        anchor_idx = None
        anchor_temp = None
        for block in ordered_blocks:
            gun_idx = self._find_top_down_point_gun(block["center"], origin_pos, axis_num, scan_span)
            if gun_idx is not None:
                anchor_idx = gun_idx
                anchor_temp = max(0, block["center"] - origin_pos[gun_idx])
                break

        if anchor_idx is None or anchor_temp is None:
            return guns

        enabled_indices = {anchor_idx}
        for block in ordered_blocks:
            gun_idx = self._find_top_down_fixed_point_gun(
                y_min=block["y_min"],
                y_max=block["y_max"],
                origin_pos=origin_pos,
                axis_num=axis_num,
                fixed_shift=anchor_temp,
                excluded_indices=enabled_indices,
            )
            if gun_idx is not None:
                enabled_indices.add(gun_idx)

        return self._build_fixed_guns(axis_num, enabled_indices, anchor_temp, anchor_temp)

    def _build_fixed_guns(self, axis_num, enabled_indices, gun_y_downer, gun_y_upper):
        guns = [self._zero_gun(i) for i in range(axis_num)]
        for idx in enabled_indices:
            if 0 <= idx < axis_num:
                guns[idx] = DistribeGunData(
                    gun_id=idx,
                    gun_y_enable=1,
                    gun_y_downer=int(gun_y_downer),
                    gun_y_upper=int(gun_y_upper),
                    gun_r_angle=0,
                )
        return guns

    @staticmethod
    def _find_top_down_point_gun(point, origin_pos, axis_num, scan_span, lower_shift=0):
        for idx in range(min(axis_num, len(origin_pos)) - 1, -1, -1):
            start = origin_pos[idx] + lower_shift
            end = origin_pos[idx] + scan_span
            if start <= point <= end:
                return idx
        return None

    @staticmethod
    def _find_top_down_cover_gun(y_min, y_max, origin_pos, axis_num, scan_span):
        for idx in range(min(axis_num, len(origin_pos)) - 1, -1, -1):
            start = origin_pos[idx]
            end = origin_pos[idx] + scan_span
            if start <= y_min and y_max <= end:
                return idx
        return None

    @staticmethod
    def _find_top_down_shifted_cover_gun(y_min, y_max, origin_pos, axis_num, lower_shift, upper_shift, excluded_indices=None):
        excluded = excluded_indices or set()
        for idx in range(min(axis_num, len(origin_pos)) - 1, -1, -1):
            if idx in excluded:
                continue
            start = origin_pos[idx] + lower_shift
            end = origin_pos[idx] + upper_shift
            if start >= y_min and y_max >= end:
                return idx
        return None

    @staticmethod
    def _find_top_down_fixed_point_gun(y_min, y_max, origin_pos, axis_num, fixed_shift, excluded_indices=None):
        excluded = excluded_indices or set()
        for idx in range(min(axis_num, len(origin_pos)) - 1, -1, -1):
            if idx in excluded:
                continue
            fixed_y = origin_pos[idx] + fixed_shift
            if y_min <= fixed_y <= y_max:
                return idx
        return None
