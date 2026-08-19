import os
from model.motionplan.motionutil.MotionUtil import MotionUtil
from model.plc.MovingFrameData import AxisData
from model.utils.TomlLoader import TomlLoader


class MotionOutLiftCompleteWorkpiecePlanning:
    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\sprayconfig.toml")
        self.process_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\ProcessConfig.toml")
        self.read_data_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\ReadDataConfig.toml")
        self.x_range = int(self.process_cfg.get("x_range", 300) or 300)
        self.y_range = int(self.process_cfg.get("y_range", 300) or 300)
        self.z_range = int(self.process_cfg.get("z_range", 300) or 300)
        self.motion_util = MotionUtil()

    def auto_out_lift_machine_move(self, machine_cfg, runtime_cfg, queue):
        """
        Handle the out_lift queue.
        Returns:
            axis_data
            finished
        """
        if not queue or queue[0] is None:
            return AxisData(Pos=0, Speed=0, Status=0), False

        block = queue[0]["data"]
        spray_mode = self._resolve_spray_mode(block)
        mode_runtime_cfg = self._resolve_mode_runtime_cfg(runtime_cfg, spray_mode)
        if spray_mode == "cabinet":
            return self._build_cabinet_reciprocate_axis_from_block(machine_cfg, mode_runtime_cfg, block)
        return self._build_reciprocate_axis_from_block(machine_cfg, mode_runtime_cfg, block)

    def _build_reciprocate_axis_from_block(self, machine_cfg, runtime_cfg, block, apply_x_offset=True):
        """
        Build AxisData from one block.
        Returns:
            axis_data
            finished: whether the workpiece should be removed
        """
        fifo_frame_pos = block.fifo_frame_pos
        outside = block.outside_data[0]
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        x_position = int(machine_cfg.get("x_position", 0) or 0) if apply_x_offset else 0
        z_front_offset = int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 100)) or 100)
        z_after_offset = int(runtime_cfg.get("out_z_after_offset", machine_cfg.get("out_z_after_offset", 100)) or 100)
        out_front_x_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 0)) or 0) if apply_x_offset else 0

        start_z_machine = z_position - z_front_offset
        end_z_machine = z_position + z_after_offset

        start_z_chain = fifo_frame_pos - outside.outside_z_min
        end_z_chain = fifo_frame_pos - outside.outside_z_max

        if end_z_chain > end_z_machine:
            return AxisData(Pos=0, Speed=0, Status=0), True

        if start_z_chain > start_z_machine:
            workpiece_origin_reference_x = self._get_workpiece_origin_reference_x()
            return AxisData(Pos=int(workpiece_origin_reference_x - int(outside.outside_x_max or 0) - out_front_x_offset - x_position),
                            Speed=outside.outside_y_max,
                            Status=outside.outside_y_min), False

        return AxisData(Pos=0, Speed=0, Status=0), False

    def _build_cabinet_reciprocate_axis_from_block(self, machine_cfg, runtime_cfg, block):
        fifo_frame_pos = block.fifo_frame_pos
        outside = block.outside_data[0]
        z_position = int(machine_cfg.get("z_position", 0) or 0)
        x_position = int(machine_cfg.get("x_position", 0) or 0)
        z_front_offset = int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 100)) or 100)
        z_after_offset = int(runtime_cfg.get("out_z_after_offset", machine_cfg.get("out_z_after_offset", 100)) or 100)
        out_front_x_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 0)) or 0)

        start_z_machine = z_position - z_front_offset
        end_z_machine = z_position + z_after_offset

        start_z_chain = fifo_frame_pos - outside.outside_z_min
        end_z_chain = fifo_frame_pos - outside.outside_z_max

        if end_z_chain > end_z_machine:
            return AxisData(Pos=0, Speed=0, Status=0), True

        if self._has_cabinet_start_arrived(start_z_machine, start_z_chain):
            x_target = self._build_cabinet_x_target(
                outside=outside,
                out_front_x_offset=out_front_x_offset,
                x_position=x_position,
                start_z_chain=start_z_chain,
                end_z_chain=end_z_chain,
                z_position=z_position,
                z_front_offset=z_front_offset,
                z_after_offset=z_after_offset,
            )
            return AxisData(Pos=x_target,
                            Speed=outside.outside_y_max,
                            Status=outside.outside_y_min), False

        return AxisData(Pos=0, Speed=0, Status=0), False

    def _build_cabinet_x_target(self, outside, out_front_x_offset, x_position,
                                start_z_chain, end_z_chain, z_position, z_front_offset, z_after_offset):
        workpiece_origin_reference_x = self._get_workpiece_origin_reference_x()
        x_dis = self._resolve_cabinet_x_dis(
            out_front_x_offset=out_front_x_offset,
            start_z_chain=start_z_chain,
            end_z_chain=end_z_chain,
            z_position=z_position,
            z_front_offset=z_front_offset,
            z_after_offset=z_after_offset,
        )
        return int(workpiece_origin_reference_x - int(outside.outside_x_max or 0) - x_position - x_dis)

    def _resolve_cabinet_x_dis(self, out_front_x_offset, start_z_chain, end_z_chain,
                               z_position, z_front_offset, z_after_offset):
        out_front_x_offset = max(0, int(out_front_x_offset or 0))
        if out_front_x_offset == 0:
            return 0

        if start_z_chain < z_position:
            if z_front_offset <= 0:
                return out_front_x_offset
            z_cur_dis = z_position - start_z_chain
            x_dis = out_front_x_offset - out_front_x_offset / z_front_offset * z_cur_dis
            return self._clamp_x_dis(x_dis, out_front_x_offset)

        if end_z_chain < z_position:
            return out_front_x_offset

        if z_after_offset <= 0:
            return 0
        z_cur_dis = end_z_chain - z_position
        x_dis = out_front_x_offset - out_front_x_offset / z_after_offset * z_cur_dis
        return self._clamp_x_dis(x_dis, out_front_x_offset)

    @staticmethod
    def _clamp_x_dis(x_dis, max_offset):
        return int(max(0, min(float(max_offset), float(x_dis))))

    def _has_cabinet_start_arrived(self, start_z_machine, start_z_chain):
        return self.motion_util.has_arrived_z(start_z_machine, start_z_chain) or start_z_chain > start_z_machine

    def _resolve_spray_mode(self, block):
        if self.motion_util.check_if_cabinet(block, self.x_range, self.y_range, self.z_range):
            return "cabinet"
        return "flat"

    @staticmethod
    def _resolve_mode_runtime_cfg(runtime_cfg, spray_mode):
        if not isinstance(runtime_cfg, dict):
            return {}

        mode_runtime_cfg = {key: value for key, value in runtime_cfg.items() if key != "flat"}
        if spray_mode == "flat" and isinstance(runtime_cfg.get("flat"), dict):
            mode_runtime_cfg.update(runtime_cfg["flat"])
        return mode_runtime_cfg

    def _get_workpiece_origin_reference_x(self):
        return float(self.read_data_cfg.get("workpiece_origin_reference_x", 2040) or 2040)
