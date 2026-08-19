from copy import deepcopy
import os
from typing import Optional
from model.utils.TomlLoader import TomlLoader
from model.utils.MachineConfigUtil import get_machine_config_path, normalize_machine_config_offsets
from model.formats.complete_workpiece.BlockDataFormat import BlockData, SingleMachineData
from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.DefaultGunDistributor import DefaultGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.InUpGunDistributor import InUpGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.OutDownGunDistributor import OutDownGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.OutFxGunDistributor import OutFxGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.OutUpGunDistributor import OutUpGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.XNIndependentYGunDistributor import XNIndependentYGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.XNSharedYGunDistributor import XNSharedYGunDistributor
from model.dataprocess.complete_workpiece.gun_distributors.XNUpDownGunDistributor import XNUpDownGunDistributor


class GunDistributor:
    IN_UP_MACHINE_TYPES = {"in_up"}
    XN_UPDOWN_MACHINE_TYPES = {"xn_updown"}
    OUT_DOWN_MACHINE_TYPES = {"out_down"}
    OUT_UP_MACHINE_TYPES = {"out_up"}
    OUT_FX_MACHINE_TYPES = {"out_fx"}
    XN_SIDE_MACHINE_TYPES = {"xn_side"}
    NO_DISTRIBUTE_MACHINE_TYPES = {"out_lift", "out_rotate", "in_rotate", "in_lift"}

    def __init__(self, machine_cfg=None):
        machine_config_path = get_machine_config_path(
            os.path.join(os.getcwd(), "model", "tomls"),
            "complete_workpiece",
        )
        loaded_machine_cfg = machine_cfg or TomlLoader.load(machine_config_path)
        self.machine_cfg = normalize_machine_config_offsets(loaded_machine_cfg)
        self.spray_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\SprayConfig.toml")
        self.process_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\ProcessConfig.toml")
        self.base_distributor = BaseGunDistributor()
        self.in_up_distributor = InUpGunDistributor()
        self.shared_side_distributor = XNSharedYGunDistributor()
        self.independent_side_distributor = XNIndependentYGunDistributor()
        self.updown_distributor = XNUpDownGunDistributor()
        self.out_down_distributor = OutDownGunDistributor()
        self.out_fx_distributor = OutFxGunDistributor()
        self.out_up_distributor = OutUpGunDistributor()
        self.default_distributor = DefaultGunDistributor()

    def distribute_for_machine(self, blockdata: BlockData, machine_cfg: dict, machine_id: Optional[int] = None) -> BlockData:
        if not blockdata:
            return blockdata

        if machine_id is None:
            machine_id = int(machine_cfg.get("sn", -1))

        gun_distance = self.spray_cfg.get("gun_distance", 450)
        spray_width_distance = self.spray_cfg.get("spray_width_distance", 30)
        x_range = self.process_cfg.get("x_range", 300)
        y_range = self.process_cfg.get("y_range", 300)
        z_range = self.process_cfg.get("z_range", 300)

        is_cabinet = self.base_distributor.check_if_cabinet(blockdata, x_range, y_range, z_range)
        machine_result = self._build_machine_distribution(
            blockdata=blockdata,
            machine_cfg=machine_cfg,
            machine_id=machine_id,
            is_cabinet=is_cabinet,
            spray_width_distance=spray_width_distance,
            gun_distance=gun_distance,
        )
        blockdata.distribe_gun_list = [machine_result] if machine_result is not None else []
        return blockdata

    def _get_xn_side_distributor(self):
        raw_mode = self.spray_cfg.get("xn_side_y_mode", 0)
        try:
            y_mode = int(raw_mode)
        except (TypeError, ValueError):
            y_mode = 0

        if y_mode == 1:
            return self.independent_side_distributor
        return self.shared_side_distributor

    def _build_machine_distribution(self, blockdata, machine_cfg, machine_id, is_cabinet, spray_width_distance, gun_distance):
        machine_type = str(machine_cfg.get("type", "") or "").strip()
        if machine_type in self.NO_DISTRIBUTE_MACHINE_TYPES:
            return None

        if not is_cabinet:
            if machine_type in self.OUT_FX_MACHINE_TYPES:
                gun_groups = self.out_fx_distributor.distribute(blockdata, machine_cfg, gun_distance)
                return SingleMachineData(machine_id=machine_id, gun_groups=gun_groups)
            if machine_type in self.XN_SIDE_MACHINE_TYPES:
                flat_blockdata = deepcopy(blockdata)
                flat_blockdata.inside_data = []
                gun_groups = self._get_xn_side_distributor().distribute(flat_blockdata, machine_cfg, gun_distance)
                return SingleMachineData(machine_id=machine_id, gun_groups=gun_groups)
            gun_groups = self.default_distributor.distribute(machine_cfg)
            return SingleMachineData(machine_id=machine_id, gun_groups=gun_groups)

        if machine_type in self.IN_UP_MACHINE_TYPES:
            gun_groups = self.in_up_distributor.distribute(blockdata, machine_cfg)
        elif machine_type in self.XN_UPDOWN_MACHINE_TYPES:
            gun_groups = self.updown_distributor.distribute(blockdata, machine_cfg, spray_width_distance)
        elif machine_type in self.XN_SIDE_MACHINE_TYPES:
            gun_groups = self._get_xn_side_distributor().distribute(blockdata, machine_cfg, gun_distance)
        elif machine_type in self.OUT_UP_MACHINE_TYPES:
            gun_groups = self.out_up_distributor.distribute(blockdata, machine_cfg)
        elif machine_type in self.OUT_DOWN_MACHINE_TYPES:
            gun_groups = self.out_down_distributor.distribute(blockdata, machine_cfg)
        elif machine_type in self.OUT_FX_MACHINE_TYPES:
            gun_groups = self.out_fx_distributor.distribute(blockdata, machine_cfg, gun_distance)
        else:
            gun_groups = self.default_distributor.distribute(machine_cfg)

        return SingleMachineData(machine_id=machine_id, gun_groups=gun_groups)
