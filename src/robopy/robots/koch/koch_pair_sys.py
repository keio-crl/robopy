# robopy/robots/koch_pair_sys.py

import logging
import os
import pickle
import time
from typing import Dict, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.config.robot_config.koch_config import KOCH_MOTOR_MAPPING, KochConfig
from robopy.motor.control_table import XControlTable
from robopy.robots.koch.calibration import run_arm_calibration

from ..common.robot import Robot
from .koch_follower import KochFollower
from .koch_leader import KochLeader

logger = logging.getLogger(__name__)


class KochPairSys(Robot):
    """Class representing a pair of Koch robotic arms: Leader and Follower"""

    def __init__(self, cfg: KochConfig):
        motor_ids = list(range(1, 13))
        self._leader: KochLeader | None
        if cfg.leader_port is not None:
            self._leader = KochLeader(cfg, motor_ids[:6])
        else:
            self._leader = None
            logger.info("Leader port is not specified. Leader arm will not be available.")
        self._follower = KochFollower(cfg, motor_ids[6:])

        self.calibration_path = cfg.calibration_path
        self._is_connected = False

        # Define explicit motor mapping between leader and follower
        self._motor_mapping = (
            KOCH_MOTOR_MAPPING  # key: leader motor name, value: follower motor name
        )

    def connect(self) -> None:
        """Connects to all devices and handles calibration."""
        if self._is_connected:
            logger.info("KochPairSys is already connected.")
            return

        try:
            if self._leader is not None:
                logger.info("Connecting to leader arm...")
                self._leader.connect()
            else:
                logger.info("Leader arm is not available. Skipping leader connection.")
            logger.info("Connecting to follower arm...")
            self._follower.connect()

            # --- Calibration Logic ---
            try:
                if os.path.exists(self.calibration_path):
                    logger.info(f"Loading calibration from '{self.calibration_path}'...")
                    with open(self.calibration_path, "rb") as f:
                        calibration = pickle.load(f)
                else:
                    logger.info("Calibration file not found. Starting new calibration procedure.")
                    calibration = self.run_calibration()

                    # Create directory if it doesn't exist
                    os.makedirs(os.path.dirname(self.calibration_path), exist_ok=True)
                    logger.info(f"Saving calibration to '{self.calibration_path}'...")
                    with open(self.calibration_path, "wb") as f:
                        pickle.dump(calibration, f)
            except (OSError, IOError, PermissionError) as e:
                logger.error(f"File operation error: {e}")
                raise ConnectionError(f"Calibration file error: {e}")
            except (pickle.PickleError, EOFError) as e:
                logger.error(f"Calibration data corrupted: {e}")
                raise ConnectionError(f"Calibration data error: {e}")

            # Apply calibration to each arm
            if self._leader is not None and self._leader.motors is not None:
                self._leader.motors.set_calibration(calibration["leader"])
            self._follower.motors.set_calibration(calibration["follower"])
            logger.info("Calibration successfully applied to both arms.")
            # --- End of Calibration Logic ---

            self.follower.torque_enable()
            if self.leader is not None:
                self.leader.torque_enable()

            self._is_connected = True
            logger.info("Connected to KochPairSys successfully")

        except ConnectionError:
            # Re-raise ConnectionError as is
            self.disconnect()
            raise
        except (OSError, TimeoutError, RuntimeError) as e:
            logger.error(f"Hardware connection error: {e}")
            self.disconnect()
            raise ConnectionError(f"Connection failed due to hardware error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error during connection: {e}")
            self.disconnect()
            raise ConnectionError(f"Connection failed due to unexpected error: {e}")

    def run_calibration(self) -> Dict[str, Dict[str, Tuple[int, bool]]]:
        """Orchestrates the calibration process for both arms."""
        all_calibration_data = {}

        # Calibrate leader arm
        if self._leader is not None:
            leader_calib = run_arm_calibration(self._leader, arm_type="leader")
            all_calibration_data["leader"] = leader_calib
        else:
            # Create dummy calibration data for leader if not available
            all_calibration_data["leader"] = {}

        # Calibrate follower arm
        follower_calib = run_arm_calibration(self._follower, arm_type="follower")
        all_calibration_data["follower"] = follower_calib

        logger.info("Both arms have been calibrated.")
        return all_calibration_data

    def disconnect(self) -> None:
        """Disconnects from all devices."""
        if self._is_connected:
            logger.info("Disconnecting KochPairSys...")
            if self._leader is not None:
                self._leader.disconnect()
            self._follower.disconnect()
            self._is_connected = False
            logger.info("Disconnected from KochPairSys")

    def get_observation(
        self, leader_obs: Dict[str, NDArray[np.float32]] | None = None
    ) -> Dict[str, NDArray[np.float32]]:
        """Gets the current observation from both arms and sensors."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        follower_motor_names = list(self._follower.motors.motors.keys())

        # Get leader observation if available
        if self._leader is not None and self._leader.motors is not None:
            leader_motor_names = list(self._leader.motors.motors.keys())
            # 直接Enumを使用
            if leader_obs is None:
                leader_obs = self._leader.motors.sync_read(
                    XControlTable.PRESENT_POSITION, leader_motor_names
                )
            leader_obs_array = np.array(list(leader_obs.values()), dtype=np.float32)
        else:
            # Return empty array if leader is not available
            leader_obs_array = np.array([], dtype=np.float32)
            logger.debug("Leader arm is not available. Returning empty leader observation.")

        follower_obs = self._follower.motors.sync_read(
            XControlTable.PRESENT_POSITION, follower_motor_names
        )

        follower_obs_array = np.array(list(follower_obs.values()), dtype=np.float32)
        logger.debug(f"Leader positions: {leader_obs_array}")
        logger.debug(f"Follower positions: {follower_obs_array}")

        return {
            "leader": leader_obs_array,
            "follower": follower_obs_array,
        }

    def teleoperate(self) -> None:
        """Teleoperation: Leader controls the Follower arm movements."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        if self._leader is None or self._leader.motors is None:
            raise ConnectionError("Leader arm is not available. Cannot start teleoperation.")

        logger.info("Starting teleoperation. Leader will control follower.")
        logger.info("Press Ctrl+C to stop teleoperation.")

        try:
            while True:
                # Get current positions from leader arm
                leader_motor_names = list(self._leader.motors.motors.keys())
                leader_positions = self._leader.motors.sync_read(
                    XControlTable.PRESENT_POSITION, leader_motor_names
                )

                # Map leader positions to follower using explicit mapping
                follower_goals = {}
                for leader_motor, leader_pos in leader_positions.items():
                    if leader_motor in self._motor_mapping:
                        follower_motor = self._motor_mapping[leader_motor]
                        # Verify that follower motor exists
                        if follower_motor in self._follower.motors.motors:
                            follower_goals[follower_motor] = leader_pos
                        else:
                            logger.warning(f"Follower motor '{follower_motor}' not found")
                    else:
                        logger.warning(f"No mapping found for leader motor '{leader_motor}'")

                # Send action to follower
                if follower_goals:
                    self._follower.motors.sync_write(XControlTable.GOAL_POSITION, follower_goals)

                time.sleep(0.01)  # 100Hz update rate

        except KeyboardInterrupt:
            logger.info("Teleoperation stopped by user.")
        except Exception as e:
            logger.error(f"Error during teleoperation: {e}")
            raise

    def teleope_step(self, if_record: bool = True) -> None | Dict[str, NDArray[np.float32]]:
        """Teleoperation step: Leader controls the Follower arm movements."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        if self._leader is None or self._leader.motors is None:
            logger.warning("Leader arm is not available. Cannot perform teleoperation step.")
            if if_record:
                return self.get_observation(leader_obs=None)
            return None

        # Get current positions from leader arm
        leader_motor_names = list(self._leader.motors.motors.keys())
        leader_positions = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )

        # Map leader positions to follower using explicit mapping
        follower_goals = {}
        for leader_motor, leader_pos in leader_positions.items():
            if leader_motor in self._motor_mapping:
                follower_motor = self._motor_mapping[leader_motor]
                # Verify that follower motor exists
                if follower_motor in self._follower.motors.motors:
                    follower_goals[follower_motor] = leader_pos
                else:
                    logger.warning(f"Follower motor '{follower_motor}' not found")
            else:
                logger.warning(f"No mapping found for leader motor '{leader_motor}'")

        # Send action to follower
        if follower_goals:
            self._follower.motors.sync_write(XControlTable.GOAL_POSITION, follower_goals)

        if if_record:
            return self.get_observation(leader_obs=leader_positions)
        return None

    def get_leader_action(self) -> dict:
        """Get the current action (positions) from the leader arm."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        if self._leader is None or self._leader.motors is None:
            logger.warning("Leader arm is not available. Returning empty action.")
            return {}

        leader_motor_names = list(self._leader.motors.motors.keys())
        leader_positions = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )
        return leader_positions

    def send_follower_action(self, action: dict) -> None:
        """Send action to the follower arm only."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        self._follower.motors.sync_write(XControlTable.GOAL_POSITION, action)

    @property
    def leader(self) -> KochLeader | None:
        return self._leader

    @property
    def follower(self) -> KochFollower:
        return self._follower
