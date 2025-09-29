import logging

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelMotor

from .rakuda_arm import RakudaArm

logger = logging.getLogger(__name__)


class RakudaFollower(RakudaArm):
    """Class representing the follower arm of the Rakuda robotic system."""

    def __init__(self, cfg: RakudaConfig):
        super().__init__(cfg, cfg.follower_port)

    def _create_motors(self) -> dict[str, DynamixelMotor]:
        """Create motor configuration for the follower arm using xm430-w350 motors."""
        return {
            # head
            "torso_yaw": DynamixelMotor(27, "torso_yaw", "xm540-w270"),
            "head_yaw": DynamixelMotor(28, "head_yaw", "xm430-w350"),
            "head_pitch": DynamixelMotor(29, "head_pitch", "xm430-w350"),
            # right
            "r_arm_sh_pitch1": DynamixelMotor(1, "r_arm_sh_pitch1", "xm540-w270"),
            "r_arm_sh_roll": DynamixelMotor(3, "r_arm_sh_roll", "xm540-w270"),
            "r_arm_sh_pitch2": DynamixelMotor(5, "r_arm_sh_pitch2", "xm430-w350"),
            "r_arm_el_yaw": DynamixelMotor(7, "r_arm_el_yaw", "xm430-w350"),
            "r_arm_wr_roll": DynamixelMotor(9, "r_arm_wr_roll", "xm430-w350"),
            "r_arm_wr_yaw": DynamixelMotor(11, "r_arm_wr_yaw", "xm430-w350"),
            "r_arm_grip": DynamixelMotor(31, "r_arm_grip", "xm430-w350"),
            # left
            "l_arm_sh_pitch1": DynamixelMotor(2, "l_arm_sh_pitch1", "xm540-w270"),
            "l_arm_sh_roll": DynamixelMotor(4, "l_arm_sh_roll", "xm540-w270"),
            "l_arm_sh_pitch2": DynamixelMotor(6, "l_arm_sh_pitch2", "xm430-w350"),
            "l_arm_el_yaw": DynamixelMotor(8, "l_arm_el_yaw", "xm430-w350"),
            "l_arm_wr_roll": DynamixelMotor(10, "l_arm_wr_roll", "xm430-w350"),
            "l_arm_wr_yaw": DynamixelMotor(12, "l_arm_wr_yaw", "xm430-w350"),
            "l_arm_grip": DynamixelMotor(30, "l_arm_grip", "xm430-w350"),
        }

    def _init_control_mode(self) -> None:
        """Initialize control mode for follower arm with gripper configuration."""
        # Set 2 gripper motors to Current-based position control mode: 5
        # details:https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode

        return super()._init_control_mode()

    def connect(self) -> None:
        """Connect to the follower arm and enable torque."""
        super().connect()
        if self._is_connected:
            self._motors.torque_enabled()

    def disconnect(self) -> None:
        """Disconnect from the follower arm and disable torque."""
        if self._is_connected:
            try:
                self.motors.torque_disabled()
            except Exception as e:
                logger.warning(f"Failed to disable torque during disconnect: {e}")
        super().disconnect()
