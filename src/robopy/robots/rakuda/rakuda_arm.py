import logging
from abc import abstractmethod

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)


class RakudaArm(Arm):
    """Base class for Rakuda robotic arms (leader and follower)."""

    def __init__(self, cfg: RakudaConfig, port: str):
        self.config = cfg
        self._port = port
        self._is_connected = False
        self._motors = DynamixelBus(port=self._port, motors=self._create_motors())

    @abstractmethod
    def _create_motors(self) -> dict[str, DynamixelMotor]:
        """Create motor configuration specific to each arm type."""
        pass

    @abstractmethod
    def _init_control_mode(self) -> None:
        """Initialize control mode specific to each arm type."""
        pass

    def connect(self) -> None:
        if self._is_connected:
            logger.info(f"Already connected to the {self.__class__.__name__}.")
            return
        try:
            self._motors.open()
            self._init_control_mode()
            self._is_connected = True
            print(f"Connected to the {self.__class__.__name__}.")
        except Exception as e:
            logger.error(f"Failed to connect to the {self.__class__.__name__}: {e}")
            raise ConnectionError(f"Failed to connect to the {self.__class__.__name__}: {e}")

    def disconnect(self) -> None:
        if not self._is_connected:
            logger.info(f"Not connected to the {self.__class__.__name__}.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            logger.info(f"Disconnected from the {self.__class__.__name__}.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the {self.__class__.__name__}: {e}")
            raise ConnectionError(f"Failed to disconnect from the {self.__class__.__name__}: {e}")

    @property
    def port(self) -> str:
        return self._port

    @property
    def motors(self) -> DynamixelBus:
        return self._motors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def motor_names(self) -> list[str]:
        return list(self._motors.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [motor.model_name for motor in self._motors.motors.values()]
