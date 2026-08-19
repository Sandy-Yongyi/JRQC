"""
xn_side 独立 Y 轴分枪方案。

共享Y和独立Y只在外侧、内侧分枪范围的生成方式上不同，输入输出结构及后续完整工件流程保持一致。
当前分枪器每次只处理调用方传入的一台 xn_side 设备，并返回当前设备的 GunGroupData 列表。

外侧分枪：
1. 只使用 outside_data[0]，按设备的 out_down_y_offset 和 out_up_y_offset 扩大目标Y范围。
2. 目标下限不能低于第一把枪原点，目标上限不能超过最后一把枪原点加Y轴限位。
3. 按 floor((y_max - y_min) / gun_distance) 计算枪数，最少一把且不超过设备可用喷枪数。
4. 从最高原点向最低原点选枪；如果最低选中枪仍高于目标下限，则删除最高枪并继续向下补枪。
5. 选中枪按原点从低到高分别计算 downer/upper，并保证后续喷枪范围不小于前一把枪。
6. 最低选中枪下方的未选中枪保持零位；上方未选中枪移动到 top_offset，但保持禁用。
7. 当最高运动位置超过 y_limit 时，不减少枪数，按枪数均分超出量并缩小 move_range。

内侧分枪：
1. 每个 InsideData 列独立生成一个 inside 分枪组，子分区按 subinside_y_min 从低到高处理。
2. 每个子分区使用 in_down_y_offset 和 in_up_y_offset 缩小安全范围，无效范围直接跳过。
3. 每个子分区根据自身Y范围和剩余喷枪数计算枪数及独立 move_range。
4. 第一个子分区按 origin + gun_distance > y_min 查找起始枪，后续子分区连续使用剩余喷枪。
5. 启用枪分别计算 downer/upper；下方未选中枪保持零位，上方未选中枪使用最后一个子分区的 top_offset。
"""
import math
import os

from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor
from model.formats.complete_workpiece.BlockDataFormat import DistribeGunData, GunGroupData
from model.utils.TomlLoader import TomlLoader


class XNIndependentYGunDistributor(BaseGunDistributor):
    """为每把 xn_side 喷枪分别计算独立的 Y 轴运动范围。"""

    def __init__(self, spray_config=None):
        config_path = os.path.join(os.getcwd(), "model", "tomls", "SprayConfig.toml")
        self.spray_cfg = spray_config if spray_config is not None else TomlLoader.load(config_path)

    def distribute(self, blockdata, mcfg, gun_distance):
        axis_num = int(mcfg.get("spray_num", 6))
        origin_pos = mcfg.get("origin_pos", [1120, 1420, 1720, 2020, 2320, 2620])
        outside_up_y_offset = int(mcfg.get("out_up_y_offset", 100) or 100)
        outside_down_y_offset = int(mcfg.get("out_down_y_offset", 100) or 100)
        inside_up_y_offset = int(mcfg.get("in_up_y_offset", 100) or 100)
        inside_down_y_offset = int(mcfg.get("in_down_y_offset", 100) or 100)
        min_recip_distance = int(self.spray_cfg.get("min_recip_distance", 60) or 60)
        y_limit = self._resolve_y_limit(mcfg)
        scan_span = self._resolve_scan_span(origin_pos, gun_distance)

        if not origin_pos or not blockdata.outside_data:
            return self._build_side_zero_groups(axis_num)

        out = blockdata.outside_data[0]
        inside_columns = blockdata.inside_data or []
        outside_guns = self._build_outside_guns(
            out,
            origin_pos,
            axis_num,
            outside_up_y_offset,
            outside_down_y_offset,
            y_limit,
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
            inside_guns = self._build_inside_guns(
                subinside_list,
                origin_pos,
                axis_num,
                inside_up_y_offset,
                inside_down_y_offset,
                min_recip_distance,
                y_limit,
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

    def _build_outside_guns(self, out, origin_pos, axis_num, up_y_offset, down_y_offset, y_limit, scan_span):
        guns = [self._zero_gun(index) for index in range(axis_num)]
        if out.outside_y_min is None or out.outside_y_max is None:
            return guns

        target_y_max = out.outside_y_max + up_y_offset
        target_y_min = out.outside_y_min - down_y_offset
        if target_y_max <= target_y_min:
            return guns
        max_idx = min(axis_num, len(origin_pos)) - 1
        if max_idx < 0:
            return guns

        raw_gun = math.floor((target_y_max - target_y_min) / scan_span)
        gun_num = max(1, min(max_idx + 1, raw_gun))
        selected = self._select_outside_guns(
            indexed_origins=list(enumerate(origin_pos))[:max_idx + 1],
            target_y_min=target_y_min,
            target_y_max=target_y_max,
            gun_distance=scan_span,
            gun_num=gun_num,
        )
        if not selected:
            return guns

        move_range = math.floor((target_y_max - target_y_min) / gun_num)
        top_offset = self._calculate_top_offset(selected, target_y_min, move_range)
        if top_offset > y_limit:
            exceed = math.ceil((top_offset - y_limit) / gun_num)
            move_range = max(0, move_range - exceed)
            top_offset = y_limit

        return self._build_independent_guns(
            axis_num=axis_num,
            origin_pos=origin_pos,
            selected=selected,
            base_y_min_by_gun={gun_id: target_y_min for gun_id, _ in selected},
            move_range_by_gun={gun_id: move_range for gun_id, _ in selected},
            index_in_region_by_gun={gun_id: index for index, (gun_id, _) in enumerate(selected)},
            top_offset=top_offset,
            y_limit=y_limit,
            range_constraint="outside",
        )

    def _build_inside_guns(self, inside_blocks, origin_pos, axis_num, up_y_offset, down_y_offset, min_recip_distance, y_limit, scan_span):
        guns = [self._zero_gun(index) for index in range(axis_num)]
        if not inside_blocks or scan_span <= 0:
            return guns

        indexed_origins = self._sorted_indexed_origins(origin_pos, axis_num)
        if not indexed_origins:
            return guns

        normalized_blocks = self._normalize_inside_blocks(inside_blocks, up_y_offset, down_y_offset)
        if not normalized_blocks:
            return guns

        block_plans = []
        remaining_gun_count = len(indexed_origins)
        for block in normalized_blocks:
            if remaining_gun_count <= 0:
                continue

            block_height = block["y_max"] - block["y_min"]
            fixed_at_center = block_height < min_recip_distance
            raw_gun = 1 if fixed_at_center else math.floor(block_height / scan_span)
            gun_num = min(remaining_gun_count, max(1, raw_gun))
            if gun_num <= 0:
                continue

            block_plans.append({"block": block, "gun_num": gun_num, "fixed_at_center": fixed_at_center})
            remaining_gun_count -= gun_num

        if not block_plans:
            return guns

        first_block = block_plans[0]["block"]
        natural_start_index = next(
            (
                origin_index
                for origin_index, (_, position) in enumerate(indexed_origins)
                if position + scan_span > first_block["y_min"]
            ),
            None,
        )
        if natural_start_index is None:
            return guns

        total_required = sum(plan["gun_num"] for plan in block_plans)
        max_start_index = len(indexed_origins) - total_required
        start_index = min(natural_start_index, max_start_index)
        selected = []
        base_y_min_by_gun = {}
        move_range_by_gun = {}
        index_in_region_by_gun = {}
        fixed_center_gun_ids = set()
        top_offset = 0

        for plan in block_plans:
            block = plan["block"]
            requested_gun_num = plan["gun_num"]
            sub_selected = indexed_origins[start_index:start_index + requested_gun_num]
            start_index += len(sub_selected)
            if not sub_selected:
                continue

            selected.extend(sub_selected)
            actual_gun_num = len(sub_selected)
            if plan["fixed_at_center"]:
                base_y_min = round((block["y_min"] + block["y_max"]) / 2)
                move_range = 0
            else:
                base_y_min = block["y_min"]
                move_range = round((block["y_max"] - block["y_min"]) / actual_gun_num)

            for index_in_region, (gun_id, _) in enumerate(sub_selected):
                base_y_min_by_gun[gun_id] = base_y_min
                move_range_by_gun[gun_id] = move_range
                index_in_region_by_gun[gun_id] = index_in_region
                if plan["fixed_at_center"]:
                    fixed_center_gun_ids.add(gun_id)

            last_gun_id, last_position = sub_selected[-1]
            last_downer = max(
                0,
                base_y_min + index_in_region_by_gun[last_gun_id] * move_range - last_position,
            )
            top_offset = min(last_downer + move_range, y_limit)

        if not selected:
            return guns

        return self._build_independent_guns(
            axis_num=axis_num,
            origin_pos=origin_pos,
            selected=selected,
            base_y_min_by_gun=base_y_min_by_gun,
            move_range_by_gun=move_range_by_gun,
            index_in_region_by_gun=index_in_region_by_gun,
            top_offset=top_offset,
            y_limit=y_limit,
            range_constraint="inside",
            fixed_center_gun_ids=fixed_center_gun_ids,
        )

    @staticmethod
    def _sorted_indexed_origins(origin_pos, axis_num):
        usable_axis_num = min(axis_num, len(origin_pos))
        return sorted(
            [(index, int(origin_pos[index])) for index in range(usable_axis_num)],
            key=lambda item: item[1],
        )

    @staticmethod
    def _select_outside_guns(indexed_origins, target_y_min, target_y_max, gun_distance, gun_num):
        candidates = [
            (gun_id, position)
            for gun_id, position in reversed(indexed_origins)
            if position + gun_distance <= target_y_max
        ]
        selected = candidates[:gun_num]
        next_candidate_index = gun_num

        while selected and selected[-1][1] > target_y_min and next_candidate_index < len(candidates):
            selected.pop(0)
            selected.append(candidates[next_candidate_index])
            next_candidate_index += 1

        return sorted(selected, key=lambda item: item[1])

    @staticmethod
    def _calculate_top_offset(selected, target_y_min, move_range):
        _, last_position = selected[-1]
        last_index = len(selected) - 1
        last_downer = max(0, target_y_min + last_index * move_range - last_position)
        return last_downer + move_range

    def _build_independent_guns(
        self, axis_num, origin_pos, selected, base_y_min_by_gun, move_range_by_gun, index_in_region_by_gun, top_offset, y_limit, range_constraint, fixed_center_gun_ids=None
    ):
        guns = [self._zero_gun(index) for index in range(axis_num)]
        selected_ids = {gun_id for gun_id, _ in selected}
        fixed_center_gun_ids = set(fixed_center_gun_ids or [])
        min_selected_position = min(position for _, position in selected)
        indexed_origins = self._sorted_indexed_origins(origin_pos, axis_num)
        raw_ranges = {}

        for gun_id, position in indexed_origins:
            if gun_id not in selected_ids:
                continue
            move_range = move_range_by_gun[gun_id]
            gun_y_downer = max(
                0,
                base_y_min_by_gun[gun_id] + index_in_region_by_gun[gun_id] * move_range - position,
            )
            raw_ranges[gun_id] = (gun_y_downer, min(gun_y_downer + move_range, y_limit))

        range_lengths = {gun_y_upper - gun_y_downer for gun_y_downer, gun_y_upper in raw_ranges.values()}
        prevent_overlap = range_constraint == "inside" and len(range_lengths) > 1
        prev_downer = -1
        prev_upper = -1

        for gun_id, position in indexed_origins:
            if gun_id in selected_ids:
                raw_downer, raw_upper = raw_ranges[gun_id]
                gun_y_downer, gun_y_upper = self._apply_y_range_constraint(
                    raw_downer,
                    raw_upper,
                    prev_downer=prev_downer,
                    prev_upper=prev_upper,
                    prevent_overlap=prevent_overlap,
                )
                gun_y_enable = 1
                if gun_id in fixed_center_gun_ids and (gun_y_downer != raw_downer or gun_y_upper != raw_upper):
                    # 短行程枪偏离工件中心后禁止喷涂，但仍停在防撞调整后的安全位置。
                    safe_position = max(gun_y_downer, gun_y_upper)
                    gun_y_downer = safe_position
                    gun_y_upper = safe_position
                    gun_y_enable = 0
                prev_downer = gun_y_downer
                prev_upper = gun_y_upper

                guns[gun_id] = DistribeGunData(
                    gun_id=gun_id,
                    gun_y_enable=gun_y_enable,
                    gun_y_downer=int(round(gun_y_downer)),
                    gun_y_upper=int(round(gun_y_upper)),
                    gun_r_angle=0,
                )
            elif position >= min_selected_position:
                guns[gun_id] = DistribeGunData(
                    gun_id=gun_id,
                    gun_y_enable=0,
                    gun_y_downer=int(round(top_offset)),
                    gun_y_upper=int(round(top_offset)),
                    gun_r_angle=0,
                )

        return guns

    @staticmethod
    def _apply_y_range_constraint(gun_y_downer, gun_y_upper, prev_downer, prev_upper, prevent_overlap):
        if prevent_overlap:
            gun_y_downer = max(gun_y_downer, prev_upper)
            gun_y_upper = max(gun_y_upper, gun_y_downer)
        else:
            gun_y_downer = max(gun_y_downer, prev_downer)
            gun_y_upper = max(gun_y_upper, prev_upper)

        return gun_y_downer, gun_y_upper

    @staticmethod
    def _normalize_inside_blocks(inside_blocks, in_up_y_offset, in_down_y_offset):
        normalized = []
        for block in inside_blocks:
            if block.subinside_y_min is None or block.subinside_y_max is None:
                continue
            adjusted_y_min = block.subinside_y_min + in_down_y_offset
            adjusted_y_max = block.subinside_y_max - in_up_y_offset
            if adjusted_y_max <= adjusted_y_min:
                continue
            normalized.append({"y_min": adjusted_y_min, "y_max": adjusted_y_max})
        return sorted(normalized, key=lambda item: item["y_min"])

    @staticmethod
    def _resolve_y_limit(mcfg):
        max_limit_pos = list(mcfg.get("max_limit_pos", []) or [])
        if len(max_limit_pos) > 1:
            return max(0, int(max_limit_pos[1] or 0))
        return 0

    @staticmethod
    def _resolve_scan_span(origin_pos, gun_distance):
        if len(origin_pos) >= 2:
            diffs = [abs(int(origin_pos[i + 1]) - int(origin_pos[i])) for i in range(len(origin_pos) - 1)]
            diffs = [diff for diff in diffs if diff > 0]
            if diffs:
                return min(diffs)
        return int(gun_distance or 430)
