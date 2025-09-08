import logging

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)


class RakudaFollower(Arm):
    """Class representing the follower arm of the Rakuda robotic system."""

    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._port = cfg.follower_port
        # Initialize motors for the follower arm
        self._is_connected = False

        self._motors = DynamixelBus(
            port=self._port,
            motors={
                # right
                "r_arm_sh_pitch1": DynamixelMotor(13, "xm430-w350"),
            },
        )

    def connect(self) -> None:
        if self._is_connected:
            logger.info("Already connected to the Rakuda Follower arm.")
            return
        try:
            self._motors.open()
            self._is_connected = True
            logger.info("Connected to the Rakuda Follower arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the Rakuda Follower arm: {e}")
            raise ConnectionError(f"Failed to connect to the Rakuda Follower arm: {e}")

    def disconnect(self) -> None:
        # Implementation to disconnect from the Rakuda Follower arm
        if not self._is_connected:
            logger.info("Not connected to the Rakuda Follower arm.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            logger.info("Disconnected from the Rakuda Follower arm.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the Rakuda Follower arm: {e}")
            raise ConnectionError(f"Failed to disconnect from the Rakuda Follower arm: {e}")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def motor_names(self) -> list[str]:
        return list(self._motors.motors.keys())

    @property
    def port(self) -> str:
        return self._port

    @property
    def motor_models(self) -> list[str]:
        return [motor.model_name for motor in self._motors.motors.values()]

    @property
    def motors(self) -> DynamixelBus:
        return self._motors
