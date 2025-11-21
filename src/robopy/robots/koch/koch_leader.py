import logging
from typing import Optional

from robopy.config.robot_config.koch_config import KochConfig
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.common.arm import Arm
from robopy.motor.control_table import XControlTable

logger = logging.getLogger(__name__)


class KochLeader(Arm):
    """Class representing the Koch Leader robotic arm with config and extended error handling."""

    def __init__(self, config: KochConfig, motor_ids: list[int]):
        self.config = config
        self._port = config.leader_port

        if len(motor_ids) != 6:
            logger.error("Koch Leader requires exactly 6 motor IDs.")
            raise ValueError("Koch Leader requires exactly 6 motor IDs.")
        
        if self._port is None:
            logger.warning("Leader port is not specified. Leader arm will not be initialized.")
            self._motors: Optional[DynamixelBus] = None
        else:
            self._motors = DynamixelBus(
                port=self._port,
                motors={
                    "shoulder_pan": DynamixelMotor(motor_ids[0], "shoulder_pan", "xl330-m077"),
                    "shoulder_lift": DynamixelMotor(motor_ids[1], "shoulder_lift", "xl330-m077"),
                    "elbow": DynamixelMotor(motor_ids[2], "elbow", "xl330-m077"),
                    "wrist_flex": DynamixelMotor(motor_ids[3], "wrist_flex", "xl330-m077"),
                    "wrist_roll": DynamixelMotor(motor_ids[4], "wrist_roll", "xl330-m077"),
                    "gripper": DynamixelMotor(motor_ids[5], "gripper", "xl330-m077"),
                },
            )
        self._is_connected = False

    @property
    def port(self) -> Optional[str]:
        return self._port

    @property
    def motors(self) -> Optional[DynamixelBus]:
        return self._motors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def torque_enable(self):
        if self._motors is None:
            logger.warning("Leader motors are not initialized. Cannot enable torque.")
            return
        self._motors.write(XControlTable.TORQUE_ENABLE, "gripper", 1)
        self._motors.write(XControlTable.GOAL_POSITION, "gripper", 148.00)

    def torque_disable(self):
        if self._motors is None:
            return
        self._motors.write(XControlTable.TORQUE_ENABLE, "gripper", 0)

    def connect(self) -> None:
        if self._port is None or self._motors is None:
            logger.info("Leader port is not specified. Skipping leader arm connection.")
            return
        if self._is_connected:
            logger.info("Already connected to the Koch Leader arm.")
            return
        try:
            self._motors.open()
            self._is_connected = True
            logger.info("Connected to the Koch Leader arm.")
        except Exception as e:
            logger.error(f"Failed to connect to the Koch Leader arm: {e}")
            raise ConnectionError(f"Failed to connect to the Koch Leader arm: {e}")
        # self.torque_enable()

    def disconnect(self) -> None:
        if self._motors is None:
            return
        self.torque_disable()

        # Implementation to disconnect from the Koch Leader arm
        if not self._is_connected:
            logger.info("Not connected to the Koch Leader arm.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            logger.info("Disconnected from the Koch Leader arm.")
        except Exception as e:
            logger.error(f"Failed to disconnect from the Koch Leader arm: {e}")
            raise ConnectionError(f"Failed to disconnect from the Koch Leader arm: {e}")

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
