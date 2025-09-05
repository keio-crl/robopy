# robopy/robots/koch_pair_sys.py

import logging
import os
import pickle
import time
from typing import Dict, Tuple

from robopy.config.robot_config.koch_config import KochConfig
from robopy.motor.control_table import XControlTable  # 直接インポート
from robopy.robots.koch.calibration import run_arm_calibration

from ..common.robot import Robot
from .koch_follower import KochFollower
from .koch_leader import KochLeader

logger = logging.getLogger(__name__)


class KochPairSys(Robot):
    """Class representing a pair of Koch robotic arms: Leader and Follower"""

    def __init__(self, cfg: KochConfig):
        motor_ids = list(range(1, 13))
        self._leader = KochLeader(cfg, motor_ids[:6])
        self._follower = KochFollower(cfg, motor_ids[6:])

        self.calibration_path = cfg.calibration_path
        self._is_connected = False

        # Define explicit motor mapping between leader and follower
        self._motor_mapping = {
            "shoulder_pan": "shoulder_pan",
            "shoulder_lift": "shoulder_lift",
            "elbow_flex": "elbow_flex",
            "wrist_flex": "wrist_flex",
            "wrist_roll": "wrist_roll",
            "gripper": "gripper",
        }

    def connect(self) -> None:
        """Connects to all devices and handles calibration."""
        if self._is_connected:
            logger.info("KochPairSys is already connected.")
            return

        try:
            logger.info("Connecting to leader arm...")
            self._leader.connect()
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
            self._leader.motors.set_calibration(calibration["leader"])
            self._follower.motors.set_calibration(calibration["follower"])
            logger.info("Calibration successfully applied to both arms.")
            # --- End of Calibration Logic ---

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
        leader_calib = run_arm_calibration(self._leader, arm_type="leader")
        all_calibration_data["leader"] = leader_calib

        # Calibrate follower arm
        follower_calib = run_arm_calibration(self._follower, arm_type="follower")
        all_calibration_data["follower"] = follower_calib

        logger.info("Both arms have been calibrated.")
        return all_calibration_data

    def disconnect(self) -> None:
        """Disconnects from all devices."""
        if self._is_connected:
            logger.info("Disconnecting KochPairSys...")
            self._leader.disconnect()
            self._follower.disconnect()
            self._is_connected = False
            logger.info("Disconnected from KochPairSys")

    def get_observation(self):
        """Gets the current observation from both arms and sensors."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        leader_motor_names = list(self._leader.motors.motors.keys())
        follower_motor_names = list(self._follower.motors.motors.keys())

        # 直接Enumを使用
        leader_obs = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )
        follower_obs = self._follower.motors.sync_read(
            XControlTable.PRESENT_POSITION, follower_motor_names
        )
        return {
            "leader": leader_obs,
            "follower": follower_obs,
        }

    def send_action(self, action: dict) -> None:
        """Sends action (goal positions) to both arms.
        action: {'leader': {...}, 'follower': {...}}"""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        leader_action = action.get("leader", {})
        follower_action = action.get("follower", {})

        # 直接Enumを使用
        self._leader.motors.sync_write(XControlTable.GOAL_POSITION, leader_action)
        self._follower.motors.sync_write(XControlTable.GOAL_POSITION, follower_action)

    def teleoperate(self) -> None:
        """Teleoperation: Leader controls the Follower arm movements."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

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

    def get_leader_action(self) -> dict:
        """Get the current action (positions) from the leader arm."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

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
    def leader(self) -> KochLeader:
        return self._leader

    @property
    def follower(self) -> KochFollower:
        return self._follower
