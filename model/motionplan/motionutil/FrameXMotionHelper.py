from model.motionplan.motionutil.AxisLimits import clamp_speed, clamp_to_limit_yx


class FrameXMotionHelper:
    """按帧 X 轴范围、目标和插补速度计算辅助类。"""

    @staticmethod
    def build_static_search_y_range(origin_pos, y_move_min, y_move_max, out_down_y_offset, out_up_y_offset):
        """计算静态模式下单把喷枪的固定点云 Y 搜索区间。"""
        search_y_min = (int(origin_pos) + int(y_move_min) - int(out_down_y_offset))
        search_y_max = (int(origin_pos) + int(y_move_max) + int(out_up_y_offset))
        return search_y_min, search_y_max

    @staticmethod
    def build_dynamic_search_y_range(origin_pos, y_cur, out_down_y_offset, out_up_y_offset):
        """计算动态插补模式下单把喷枪当前的点云 Y 搜索区间。"""
        search_y_min = (int(origin_pos) + int(y_cur) - int(out_down_y_offset))
        search_y_max = (int(origin_pos) + int(y_cur) + int(out_up_y_offset))
        return search_y_min, search_y_max

    @staticmethod
    def aggregate_static_x_range(x_ranges):
        """汇总各喷枪的有效 X 范围，得到静态全局最小值和最大值。"""
        valid_ranges = []
        for x_range in x_ranges:
            if not x_range or len(x_range) != 2:
                continue
            x_min, x_max = x_range
            if x_min is None or x_max is None:
                continue
            x_min = int(x_min)
            x_max = int(x_max)
            if x_min > x_max:
                raise ValueError(f"X 范围无效: {x_min} > {x_max}")
            valid_ranges.append((x_min, x_max))

        if not valid_ranges:
            return None, None
        return (
            min(x_min for x_min, _ in valid_ranges),
            max(x_max for _, x_max in valid_ranges),
        )

    @staticmethod
    def calculate_interpolation_speed(previous_y, current_y, previous_target, current_target, y_speed, max_speed, initial_speed):
        """根据相邻 Y 位置和最终 X 目标差计算插补速度。"""
        max_speed = int(max_speed or 0)
        if previous_y is None or previous_target is None:
            return clamp_speed(int(initial_speed or 0), max_speed)

        y_distance = abs(int(current_y) - int(previous_y))
        if y_distance == 0:
            return 0

        x_distance = abs(int(current_target) - int(previous_target))
        speed = round(
            x_distance * abs(int(y_speed or 0)) / y_distance
        )
        return clamp_speed(speed, max_speed)

    @staticmethod
    def build_final_x_target(base_x_min, x_position, current_x_offset, x_min_limit, x_max_limit):
        """减去设备定位和当前慢进慢退偏移，并限制最终 X 目标。"""
        x_min_limit = int(x_min_limit)
        x_max_limit = int(x_max_limit)
        if x_min_limit > x_max_limit:
            raise ValueError(f"X 轴位置限位无效: {x_min_limit} > {x_max_limit}")
        target = (int(base_x_min) - int(x_position or 0) - int(current_x_offset or 100))
        return clamp_to_limit_yx(target, x_min_limit, x_max_limit)

    @staticmethod
    def resolve_slow_offset(start_z_chain, end_z_chain, center_z, front_offset, after_offset, max_x_offset):
        """沿用原完整工件逻辑计算慢退、保持和慢进的当前 X 偏移。"""
        max_x_offset = max(0, int(max_x_offset or 0))
        if max_x_offset == 0:
            return 0

        start_z_chain = int(start_z_chain)
        end_z_chain = int(end_z_chain)
        center_z = int(center_z)
        front_offset = int(front_offset or 0)
        after_offset = int(after_offset or 0)

        if start_z_chain < center_z:
            if front_offset <= 0:
                return max_x_offset
            remaining = center_z - start_z_chain
            value = max_x_offset - max_x_offset / front_offset * remaining
            return FrameXMotionHelper._clamp_slow_offset(value, max_x_offset)

        if end_z_chain < center_z:
            return max_x_offset

        if after_offset <= 0:
            return 0
        passed = end_z_chain - center_z
        value = max_x_offset - max_x_offset / after_offset * passed
        return FrameXMotionHelper._clamp_slow_offset(value, max_x_offset)

    @staticmethod
    def _clamp_slow_offset(value, max_x_offset):
        return int(max(0, min(float(max_x_offset), float(value))))
