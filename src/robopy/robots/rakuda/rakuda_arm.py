import logging
from abc import abstractmethod

from robopy.config.robot_config import RAKUDA_CONTROLTABLE_VALUES, RakudaConfig
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.motor.dynamixel_control_table import XControlTable
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

    #: Gripper motors, configured separately from the arm joints.
    GRIPPER_MOTORS: tuple[str, ...] = ("l_arm_grip", "r_arm_grip")

    @abstractmethod
    def _init_control_mode(self) -> None:
        """Initialize control mode specific to each arm type.

        Order matters on a DYNAMIXEL: ``OPERATING_MODE`` is an EEPROM item and is
        rejected while torque is enabled, and changing the mode resets the goal
        values and the position gains.  So torque is disabled first, then the
        mode is set, then the gains -- not the other way round.
        """
        # Torque must be off before OPERATING_MODE (an EEPROM item) is written.
        self.motors.torque_disabled(list(self.GRIPPER_MOTORS))

        gains = (
            RAKUDA_CONTROLTABLE_VALUES.GRIP_PID_SLOW
            if self.config.slow_mode
            else RAKUDA_CONTROLTABLE_VALUES.GRIP_PID
        )
        gain_items = (
            XControlTable.POSITION_P_GAIN,
            XControlTable.POSITION_I_GAIN,
            XControlTable.POSITION_D_GAIN,
        )

        for motor_name in self.GRIPPER_MOTORS:
            # Set gripper motors to current-based position control mode (5).
            # https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
            self.motors.write(
                XControlTable.OPERATING_MODE,
                motor_name,
                RAKUDA_CONTROLTABLE_VALUES.CURRENT_BASED_OPERATING_MODE,
            )
            # The gains are written after the mode change, which resets them.
            # This used to be a `map(...)` whose result was never consumed, so
            # none of these writes actually happened.
            for item, value in zip(gain_items, gains):
                self.motors.write(item, motor_name, value)

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
