import logging

from robopy.config.robot_config.so101_config import So101Config
from robopy.motor.feetech_bus import FeetechBus, FeetechMotor
from robopy.motor.feetech_control_table import STSControlTable
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)


class So101Follower(Arm):
    """Class representing the SO-101 Follower arm with Feetech STS3215 motors.

    The follower arm has 6 joints (motor IDs 1-6):
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
    """

    def __init__(self, config: So101Config) -> None:
        self.config = config
        self._port = config.follower_port

        self._motors = FeetechBus(
            port=self._port,
            motors={
                "shoulder_pan": FeetechMotor(1, "shoulder_pan", "sts3215"),
                "shoulder_lift": FeetechMotor(2, "shoulder_lift", "sts3215"),
                "elbow_flex": FeetechMotor(3, "elbow_flex", "sts3215"),
                "wrist_flex": FeetechMotor(4, "wrist_flex", "sts3215"),
                "wrist_roll": FeetechMotor(5, "wrist_roll", "sts3215"),
                "gripper": FeetechMotor(6, "gripper", "sts3215"),
            },
        )
        self._is_connected = False

    @property
    def port(self) -> str:
        return self._port

    @property
    def motors(self) -> FeetechBus:
        return self._motors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def torque_enable(self) -> None:
        self._motors.sync_write(
            STSControlTable.TORQUE_ENABLE, {name: 1 for name in self.motor_names}
        )

    def torque_disable(self) -> None:
        self._motors.sync_write(
            STSControlTable.TORQUE_ENABLE, {name: 0 for name in self.motor_names}
        )

    def connect(self) -> None:
        if self._is_connected:
            logger.info("Already connected to the SO-101 Follower arm.")
            return
        try:
            self._motors.open()
            self._is_connected = True
            logger.info("Connected to the SO-101 Follower arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the SO-101 Follower arm: {e}")
            raise ConnectionError(f"Failed to connect to the SO-101 Follower arm: {e}")

        # Configure PID gains for stable position control
        # Lower P to avoid shakiness, matching lerobot's defaults
        for motor_name in self.motor_names:
            self._motors.write(STSControlTable.P_COEFFICIENT, motor_name, 16)
            self._motors.write(STSControlTable.I_COEFFICIENT, motor_name, 0)
            self._motors.write(STSControlTable.D_COEFFICIENT, motor_name, 32)

        # Limit gripper torque and current to protect objects
        self._motors.write(STSControlTable.MAX_TORQUE_LIMIT, "gripper", 500)
        self._motors.write(STSControlTable.PROTECTION_CURRENT, "gripper", 250)
        self._motors.write(STSControlTable.OVERLOAD_TORQUE, "gripper", 25)

    def disconnect(self) -> None:
        self.torque_disable()

        if not self._is_connected:
            logger.info("Not connected to the SO-101 Follower arm.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            logger.info("Disconnected from the SO-101 Follower arm.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the SO-101 Follower arm: {e}")
            raise ConnectionError(f"Failed to disconnect from the SO-101 Follower arm: {e}")

    @property
    def motor_names(self) -> list[str]:
        return list(self._motors.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [motor.model_name for motor in self._motors.motors.values()]
