from robopy.config.robot_config.rakuda_config import RakudaArmObs, RakudaConfig

from ..common.composed import ComposedRobot
from .rakuda_pair_sys import RakudaPairSys


class Rakuda_Robot(ComposedRobot):
    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._pair_sys = RakudaPairSys(cfg)

    def connect(self) -> None:
        self._pair_sys.connect()

    def disconnect(self) -> None:
        self._pair_sys.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._pair_sys.is_connected

    def get_observation(self) -> RakudaArmObs:
        return self._pair_sys.get_observation()

    def record(self):
        pass
