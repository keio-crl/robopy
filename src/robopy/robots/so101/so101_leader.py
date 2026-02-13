import logging

from robopy.config.robot_config.so101_config import So101Config
from robopy.motor.feetech_bus import FeetechBus, FeetechMotor
from robopy.motor.feetech_control_table import STSControlTable
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)


class So101Leader(Arm):
    """Class representing the SO-101 Leader arm with Feetech STS3215 motors.

    The leader arm has 6 joints (motor IDs 1-6):
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
    """

    def __init__(self, config: So101Config) -> None:
        self.config = config
        self._port = config.leader_port

        if self._port is None:
            logger.warning("Leader port is not specified. Leader arm will not be initialized.")
            self._motors: FeetechBus | None = None
        else:
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
    def port(self) -> str | None:
        return self._port

    @property
    def motors(self) -> FeetechBus | None:
        return self._motors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def torque_enable(self) -> None:
        if self._motors is None:
            logger.warning("Leader motors are not initialized. Cannot enable torque.")
            return
        # Enable torque only on gripper for leader (to provide resistance)
        self._motors.write(STSControlTable.TORQUE_ENABLE, "gripper", 1)

    def torque_disable(self) -> None:
        if self._motors is None:
            return
        self._motors.torque_disabled()

    def connect(self) -> None:
        if self._port is None or self._motors is None:
            logger.info("Leader port is not specified. Skipping leader arm connection.")
            return
        if self._is_connected:
            logger.info("Already connected to the SO-101 Leader arm.")
            return
        try:
            self._motors.open()
            self._is_connected = True
            logger.info("Connected to the SO-101 Leader arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the SO-101 Leader arm: {e}")
            raise ConnectionError(f"Failed to connect to the SO-101 Leader arm: {e}")

    def disconnect(self) -> None:
        if self._motors is None:
            return
        self.torque_disable()

        if not self._is_connected:
            logger.info("Not connected to the SO-101 Leader arm.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            logger.info("Disconnected from the SO-101 Leader arm.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the SO-101 Leader arm: {e}")
            raise ConnectionError(f"Failed to disconnect from the SO-101 Leader arm: {e}")

    @property
    def motor_names(self) -> list[str]:
        if self._motors is None:
            return []
        return list(self._motors.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        if self._motors is None:
            return []
        return [motor.model_name for motor in self._motors.motors.values()]
