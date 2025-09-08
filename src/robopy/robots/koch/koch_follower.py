import logging

from robopy.config.robot_config.koch_config import KochConfig
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)


class KochFollower(Arm):
    """Class representing the Koch follower robotic arm with config and extended error handling."""

    def __init__(self, config: KochConfig, motor_ids: list[int]):
        self.config = config
        self._port = config.follower_port

        if len(motor_ids) != 6:
            logger.error("Koch follower requires exactly 6 motor IDs.")
            raise ValueError("Koch follower requires exactly 6 motor IDs.")
        self._motors = DynamixelBus(
            port=self._port,
            motors={
                "shoulder_pan": DynamixelMotor(motor_ids[0], "shoulder_pan", "xl330-m077"),
                "shouder_lift": DynamixelMotor(motor_ids[1], "shoulder_lift", "xl330-m077"),
                "elbow": DynamixelMotor(motor_ids[2], "elbow", "xl330-m077"),
                "wrist_flex": DynamixelMotor(motor_ids[3], "wrist_flex", "xl330-m077"),
                "wrist_roll": DynamixelMotor(motor_ids[4], "wrist_roll", "xl330-m077"),
                "gripper": DynamixelMotor(motor_ids[5], "gripper", "xl330-m077"),
                #
            },
        )
        self._is_connected = False

    @property
    def port(self) -> str:
        return self._port

    @property
    def motors(self) -> DynamixelBus:
        return self._motors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        if self._is_connected:
            logger.info("Already connected to the Koch follower arm.")
            return
        try:
            self._motors.open()
            self._is_connected = True
            logger.info("Connected to the Koch follower arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the Koch follower arm: {e}")
            raise ConnectionError(f"Failed to connect to the Koch follower arm: {e}")

    def disconnect(self) -> None:
        # Implementation to disconnect from the Koch follower arm
        if not self._is_connected:
            logger.info("Not connected to the Koch follower arm.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            logger.info("Disconnected from the Koch follower arm.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the Koch follower arm: {e}")
            raise ConnectionError(f"Failed to disconnect from the Koch follower arm: {e}")

    @property
    def motor_names(self) -> list[str]:
        return list(self._motors.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [motor.model_name for motor in self._motors.motors.values()]
