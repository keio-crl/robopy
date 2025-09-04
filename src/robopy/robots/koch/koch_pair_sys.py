# robopy/robots/koch_pair_sys.py

import logging
import os
import pickle
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
            if os.path.exists(self.calibration_path):
                logger.info(f"Loading calibration from '{self.calibration_path}'...")
                with open(self.calibration_path, "rb") as f:
                    calibration = pickle.load(f)
            else:
                logger.info("Calibration file not found. Starting new calibration procedure.")
                calibration = self.run_calibration()
                logger.info(f"Saving calibration to '{self.calibration_path}'...")
                with open(self.calibration_path, "wb") as f:
                    pickle.dump(calibration, f)

            # Apply calibration to each arm
            self._leader.motors.set_calibration(calibration["leader"])
            self._follower.motors.set_calibration(calibration["follower"])
            logger.info("Calibration successfully applied to both arms.")
            # --- End of Calibration Logic ---

            self._is_connected = True
            logger.info("Connected to KochPairSys successfully")

        except Exception as e:
            logger.error("Failed to connect KochPairSys. Disconnecting all devices.")
            self.disconnect()
            raise ConnectionError(f"Connection failed due to: {e}")

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

        leader_action = action.get("leader", {})
        follower_action = action.get("follower", {})

        # 直接Enumを使用
        self._leader.motors.sync_write(XControlTable.GOAL_POSITION, leader_action)
        self._follower.motors.sync_write(XControlTable.GOAL_POSITION, follower_action)
