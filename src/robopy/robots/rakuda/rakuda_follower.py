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
                # head
                "torso_yaw": DynamixelMotor(27, "torso_yaw", "xm430-w350"),
                "head_yaw": DynamixelMotor(28, "head_yaw", "xm430-w350"),
                "head_pitch": DynamixelMotor(29, "head_pitch", "xm430-w350"),
                # right
                "r_arm_sh_pitch1": DynamixelMotor(1, "r_arm_sh_pitch1", "xm430-w350"),
                "r_arm_sh_roll": DynamixelMotor(3, "r_arm_sh_roll", "xm430-w350"),
                "r_arm_sh_pitch2": DynamixelMotor(5, "r_arm_sh_pitch2", "xm430-w350"),
                "r_arm_el_yaw": DynamixelMotor(7, "r_arm_el_yaw", "xm430-w350"),
                "r_arm_wr_roll": DynamixelMotor(9, "r_arm_wr_roll", "xm430-w350"),
                "r_arm_wr_yaw": DynamixelMotor(11, "r_arm_wr_yaw", "xm430-w350"),
                "r_arm_grip": DynamixelMotor(31, "r_arm_grip", "xm430-w350"),
                # left
                "l_arm_sh_pitch1": DynamixelMotor(2, "l_arm_sh_pitch1", "xm430-w350"),
                "l_arm_sh_roll": DynamixelMotor(4, "l_arm_sh_roll", "xm430-w350"),
                "l_arm_sh_pitch2": DynamixelMotor(6, "l_arm_sh_pitch2", "xm430-w350"),
                "l_arm_el_yaw": DynamixelMotor(8, "l_arm_el_yaw", "xm430-w350"),
                "l_arm_wr_roll": DynamixelMotor(10, "l_arm_wr_roll", "xm430-w350"),
                "l_arm_wr_yaw": DynamixelMotor(12, "l_arm_wr_yaw", "xm430-w350"),
            },
        )

    def connect(self) -> None:
        if self._is_connected:
            logger.info("Already connected to the Rakuda Follower arm.")
            return
        try:
            self._motors.open()
            self.motors.torque_enabled()
            self._is_connected = True
            print("Connected to the Rakuda Follower arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the Rakuda Follower arm: {e}")
            raise ConnectionError(f"Failed to connect to the Rakuda Follower arm: {e}")

    def disconnect(self) -> None:
        # Implementation to disconnect from the Rakuda Follower arm
        if not self._is_connected:
            logger.info("Not connected to the Rakuda Follower arm.")
            return
        try:
            self.motors.torque_disabled()
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
