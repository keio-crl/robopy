import logging

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)


class RakudaLeader(Arm):
    """Class representing the leader arm of the Rakuda robotic system."""

    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._port = cfg.leader_port
        # Initialize motors for the leader arm
        self._is_connected = False

        self._mortors = DynamixelBus(
            port=self._port,
            motors={
                # head
                "torso_yaw": DynamixelMotor(27, "torso_yaw", "xc330-t288"),
                "head_yaw": DynamixelMotor(28, "head_yaw", "xc330-t288"),
                "head_pitch": DynamixelMotor(29, "head_pitch", "xc330-t288"),
                # right
                "r_arm_sh_pitch1": DynamixelMotor(1, "r_arm_sh_pitch1", "xc330-t288"),
                "r_arm_sh_roll": DynamixelMotor(3, "r_arm_sh_roll", "xc330-t288"),
                "r_arm_sh_pitch2": DynamixelMotor(5, "r_arm_sh_pitch2", "xc330-t288"),
                "r_arm_el_yaw": DynamixelMotor(7, "r_arm_el_yaw", "xc330-t288"),
                "r_arm_wr_roll": DynamixelMotor(9, "r_arm_wr_roll", "xc330-t288"),
                "r_arm_wr_yaw": DynamixelMotor(11, "r_arm_wr_yaw", "xc330-t288"),
                "r_arm_grip": DynamixelMotor(31, "r_arm_grip", "xc330-t288"),
                # left
                "l_arm_sh_pitch1": DynamixelMotor(2, "l_arm_sh_pitch1", "xc330-t288"),
                "l_arm_sh_roll": DynamixelMotor(4, "l_arm_sh_roll", "xc330-t288"),
                "l_arm_sh_pitch2": DynamixelMotor(6, "l_arm_sh_pitch2", "xc330-t288"),
                "l_arm_el_yaw": DynamixelMotor(8, "l_arm_el_yaw", "xc330-t288"),
                "l_arm_wr_roll": DynamixelMotor(10, "l_arm_wr_roll", "xc330-t288"),
                "l_arm_wr_yaw": DynamixelMotor(12, "l_arm_wr_yaw", "xc330-t288"),
                "l_arm_grip": DynamixelMotor(30, "l_arm_grip", "xc330-t288"),
            },
        )

    def connect(self) -> None:
        if self._is_connected:
            logger.info("Already connected to the Rakuda Leader arm.")
            return
        try:
            self._mortors.open()
            self._is_connected = True
            print("Connected to the Rakuda Leader arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the Rakuda Leader arm: {e}")
            raise ConnectionError(f"Failed to connect to the Rakuda Leader arm: {e}")

    def disconnect(self) -> None:
        # Implementation to disconnect from the Rakuda Leader arm
        if not self._is_connected:
            logger.info("Not connected to the Rakuda Leader arm.")
            return
        try:
            self._mortors.close()
            self._is_connected = False
            logger.info("Disconnected from the Rakuda Leader arm.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the Rakuda Leader arm: {e}")
            raise ConnectionError(f"Failed to disconnect from the Rakuda Leader arm: {e}")

    @property
    def port(self) -> str:
        return self._port

    @property
    def motors(self) -> DynamixelBus:
        return self._mortors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def motor_names(self) -> list[str]:
        return list(self._mortors.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [motor.model_name for motor in self._mortors.motors.values()]
