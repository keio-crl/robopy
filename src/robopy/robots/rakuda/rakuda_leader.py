import logging

from robopy.config import RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelMotor

from .rakuda_arm import RakudaArm

logger = logging.getLogger(__name__)


class RakudaLeader(RakudaArm):
    """Class representing the leader arm of the Rakuda robotic system."""

    def __init__(self, cfg: RakudaConfig):
        super().__init__(cfg, cfg.leader_port)

    def _create_motors(self) -> dict[str, DynamixelMotor]:
        """Create motor configuration for the leader arm using xc330-t288 motors."""
        return {
            # head
            "torso_yaw": DynamixelMotor(27, "torso_yaw", "xm430-w350"),  # different model
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
        }

    def _init_control_mode(self) -> None:
        """Initialize control mode for leader arm (no special initialization needed)."""
        return super()._init_control_mode()

    def connect(self) -> None:
        super().connect()
        if self._is_connected:
            self._motors.torque_enabled(specific_motor_names=["l_arm_grip", "r_arm_grip"])

    def disconnect(self) -> None:
        if self._is_connected:
            self._motors.torque_disabled()
        super().disconnect()
