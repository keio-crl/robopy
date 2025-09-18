import logging
import pickle
import time

import numpy as np

from robopy.config.robot_config.rakuda_config import (
    RAKUDA_MOTOR_MAPPING,
    RakudaArmObs,
    RakudaConfig,
)
from robopy.motor.control_table import XControlTable
from robopy.motor.dynamixel_bus import DynamixelBus

from ..common.robot import Robot
from .rakuda_follower import RakudaFollower
from .rakuda_leader import RakudaLeader

logger = logging.getLogger(__name__)


class RakudaPairSys(Robot):
    """Class representing the Rakuda robotic system with both leader and follower arms."""

    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._leader = RakudaLeader(cfg)
        self._follower = RakudaFollower(cfg)
        self._is_connected = False
        self._motor_mapping = (
            RAKUDA_MOTOR_MAPPING  # key: leader motor name, value: follower motor name
        )

    def connect(self) -> None:
        """Connect to both leader and follower arms."""
        if self.is_connected:
            logger.info("Successfully connected to both leader and follower arms.")
            return

        try:
            self._leader.connect()
            self._follower.connect()
            logger.info("Successfully connected to both leader and follower arms.")
            print("Successfully connected to both leader and follower arms.")
            self._is_connected = True
        except (OSError, IOError, PermissionError) as e:
            logger.error(f"Failed to connect to arms: {e}")
            raise ConnectionError(f"Failed to connect to arms: {e}")
        except (pickle.PickleError, EOFError) as e:
            logger.error(f"Calibration data corrupted: {e}")
            raise ConnectionError(f"Calibration data error: {e}")

    def disconnect(self) -> None:
        """Disconnect from both leader and follower arms."""
        self.leader.disconnect()
        self.follower.disconnect()

    def get_observation(self) -> RakudaArmObs:
        """Get the current observation from both arms."""
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        leader_motor_names = list(self._leader.motors.motors.keys())
        follower_motor_names = list(self._follower.motors.motors.keys())
        leader_obs = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )

        follower_obs = self._follower.motors.sync_read(
            XControlTable.PRESENT_POSITION, follower_motor_names
        )

        leader_obs_array = np.array(list(leader_obs.values()), dtype=np.float32)
        follower_obs_array = np.array(list(follower_obs.values()), dtype=np.float32)
        return RakudaArmObs(leader=leader_obs_array, follower=follower_obs_array)

    def teleoperate(self, max_seconds: float | None = None) -> None:
        """
        Leader controls follower. If max_seconds is set,
        run for that many seconds then return.
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        logger.info("Starting teleoperation. Leader will control follower.")
        start_time = time.time()
        try:
            while True:
                # Get current positions from leader arm
                leader_positions = self.get_leader_action()
                # Map leader positions to follower positions
                follower_positions = {}
                for leader_name, position in leader_positions.items():
                    follower_name = self._motor_mapping.get(leader_name)
                    if follower_name:
                        follower_positions[follower_name] = position

                # Send positions to follower arm
                try:
                    self.send_follower_action(follower_positions)
                    logger.info("Sent follower action.")
                except Exception:
                    logger.exception("Failed to send follower action; continuing loop.")

                # Check for max_seconds
                if max_seconds is not None and (time.time() - start_time) >= max_seconds:
                    logger.info("Reached max_seconds; exiting teleoperate.")
                    break

            # TODO: add a better way to stop smoothly, eg. set a home position
        except KeyboardInterrupt:
            self.follower.motors.torque_disabled()
            logger.info("Teleoperation stopped by user.")
        except Exception:
            logger.exception("Error during teleoperation.")
            raise

    def teleoperate_step(self) -> RakudaArmObs:
        """teleoperate_step performs one iteration of teleoperation.

        Args:
            if_record (bool, optional): _description_. Defaults to True.

        Raises:
            ConnectionError: if not connected.

        Returns:
            RakudaArmObs | None: Current observation if if_record is True, else None.
                            - leader: np.ndarray of leader arm positions
                            - follower: np.ndarray of follower arm positions
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        # Get current positions from leader arm
        leader_positions = self.get_leader_action()

        # Map leader positions to follower positions
        follower_goal_positions = {}
        for leader_name, position in leader_positions.items():
            follower_name = self._motor_mapping.get(leader_name)
            if follower_name:
                follower_goal_positions[follower_name] = position

        # Send positions to follower arm
        try:
            self.send_follower_action(follower_goal_positions)
        except Exception:
            logger.exception("Failed to send follower action; continuing.")

        leader_obs = np.array(list(leader_positions.values()), dtype=np.float32)
        follower_obs = np.array(list(follower_goal_positions.values()), dtype=np.float32)
        return RakudaArmObs(leader=leader_obs, follower=follower_obs)

    def get_leader_action(self) -> dict:
        """Get the current action (positions) from the leader arm."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        leader_motor_names = list(self._leader.motors.motors.keys())
        leader_positions = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )
        return leader_positions

    def get_follower_action(self) -> dict:
        """Get the current action (positions) from the follower arm."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        follower_motor_names = list(self._follower.motors.motors.keys())
        follower_positions = self._follower.motors.sync_read(
            XControlTable.PRESENT_POSITION, follower_motor_names
        )
        return follower_positions

    def send_follower_action(self, action: dict) -> None:
        """Send action to the follower arm only."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        self._follower.motors.sync_write(XControlTable.GOAL_POSITION, action)

    @property
    def is_connected(self) -> bool:
        """Check if both arms are connected."""
        return self._is_connected

    @property
    def port(self) -> str:
        """Get the ports of both arms."""
        return f"Leader: {self.leader.port}, Follower: {self.follower.port}"

    @property
    def motors(self) -> dict[str, DynamixelBus]:
        """Get the motor buses of both arms."""
        return {"leader": self.leader.motors, "follower": self.follower.motors}

    @property
    def leader(self) -> RakudaLeader:
        return self._leader

    @property
    def follower(self) -> RakudaFollower:
        return self._follower
