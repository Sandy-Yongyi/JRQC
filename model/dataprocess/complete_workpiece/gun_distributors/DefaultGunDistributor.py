from model.dataprocess.complete_workpiece.gun_distributors.BaseGunDistributor import BaseGunDistributor


class DefaultGunDistributor(BaseGunDistributor):
    def distribute(self, mcfg):
        axis_num = int(mcfg.get("spray_num", 1) or 1)
        return self._build_default_zero_groups(axis_num)
