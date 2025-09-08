import logging
import pickle
from typing import Any

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelBus

from ..common.robot import Robot
from .rakuda_follower import RakudaFollower
from .rakuda_leader import RakudaLeader

logger = logging.getLogger(__name__)


class RakudaPairSys(Robot):
    """Class representing the Rakuda robotic system with both leader and follower arms."""

    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._leader = RakudaLeader(cfg)
        self._follower = RakudaFollower(cfg)
        self._is_connected = False

    def connect(self) -> None:
        """Connect to both leader and follower arms."""
        if self.is_connected:
            logger.info("Successfully connected to both leader and follower arms.")
            return

        try:
            self._leader.connect()
            self._follower.connect()
            logger.info("Successfully connected to both leader and follower arms.")

            self._is_connected = True

        except (OSError, IOError, PermissionError) as e:
            logger.error(f"Failed to connect to arms: {e}")
            raise ConnectionError(f"Failed to connect to arms: {e}")
        except (pickle.PickleError, EOFError) as e:
            logger.error(f"Calibration data corrupted: {e}")
            raise ConnectionError(f"Calibration data error: {e}")

    def disconnect(self) -> None:
        """Disconnect from both leader and follower arms."""
        self.leader.disconnect()
        self.follower.disconnect()

    def get_observation(self) -> Any:
        return super().get_observation()

    @property
    def is_connected(self) -> bool:
        """Check if both arms are connected."""
        return self._is_connected

    @property
    def port(self) -> str:
        """Get the ports of both arms."""
        return f"Leader: {self.leader.port}, Follower: {self.follower.port}"

    @property
    def motors(self) -> dict[str, DynamixelBus]:
        """Get the motor buses of both arms."""
        return {"leader": self.leader.motors, "follower": self.follower.motors}

    @property
    def leader(self) -> RakudaLeader:
        return self._leader

    @property
    def follower(self) -> RakudaFollower:
        return self._follower
