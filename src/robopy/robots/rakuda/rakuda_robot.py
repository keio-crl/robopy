import time
from logging import getLogger
from typing import DefaultDict, Dict, List, override

import numpy as np
from numpy.typing import NDArray

from robopy.config.robot_config.rakuda_config import (
    RakudaArmObs,
    RakudaConfig,
    RakudaObs,
    RakudaSensorConfigs,
    RakudaSensorObs,
)
from robopy.config.sensor_config.params_config import TactileParams
from robopy.config.sensor_config.sensors import Sensors
from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.tactile.digit_sensor import DigitSensor
from robopy.sensors.visual.realsense_camera import RealsenseCamera

from ..common.composed import ComposedRobot
from .rakuda_pair_sys import RakudaPairSys

logger = getLogger(__name__)


class RakudaRobot(ComposedRobot):
    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._pair_sys = RakudaPairSys(cfg)
        self._sensor_configs: RakudaSensorConfigs = self._init_config()
        self._sensors: Sensors = self._init_sensors()

    @override
    def connect(self) -> None:
        try:
            self._pair_sys.connect()
        except Exception as e:
            self._pair_sys.disconnect()
            raise e

    @override
    def disconnect(self) -> None:
        self._pair_sys.disconnect()

        for cam in self._sensors.cameras or []:
            cam.disconnect()

        for tac in self._sensors.tactile or []:
            tac.disconnect()

    @override
    def teleoperation(self, max_seconds: float | None = None) -> None:
        """Start teleoperation for Rakuda robot."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        if max_seconds is not None and max_seconds > 0:
            self._pair_sys.teleoperate(max_seconds=max_seconds)
        else:
            self._pair_sys.teleoperate()

    def record(self, max_frame: int, fps: int = 5) -> RakudaObs:
        if not self.is_connected:
            self.connect()

        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")

        leader_obs = []
        follower_obs = []
        camera_obs: Dict[str, List] = DefaultDict(list)
        tactile_obs: Dict[str, List] = DefaultDict(list)

        get_obs_interval = 1.0 / fps
        frame_count = 0
        interval_start = time.time()
        while frame_count < max_frame:
            self.robot_system.teleoperate_step()

            if time.time() - interval_start < get_obs_interval:
                continue

            arm_obs = self.get_arm_observation()

            leader_obs.append(arm_obs["leader"])
            follower_obs.append(arm_obs["follower"])

            sensor_data = self.sensors_observation()
            camera_data = sensor_data["cameras"]
            tactile_data = sensor_data["tactile"]

            for cam_name, cam_frame in camera_data.items():
                camera_obs[cam_name].append(cam_frame)

            for tac_name, tac_frame in tactile_data.items():
                tactile_obs[tac_name].append(tac_frame)

            frame_count += 1
            interval_start = time.time()
        # proccess observations to numpy arrays
        leader_obs_np = np.array(leader_obs)
        follower_obs_np = np.array(follower_obs)
        arms: RakudaArmObs = {"leader": leader_obs_np, "follower": follower_obs_np}

        # process camera observations
        camera_obs_np: Dict[str, NDArray[np.float32] | None] = {}
        for cam_name, frames in camera_obs.items():
            if frames:
                camera_obs_np[cam_name] = np.array(frames)
            else:
                camera_obs_np[cam_name] = None

        # process tactile observations
        tactile_obs_np: Dict[str, NDArray[np.float32] | None] = {}
        for tac_name, frames in tactile_obs.items():
            if frames:
                tactile_obs_np[tac_name] = np.array(frames)
            else:
                tactile_obs_np[tac_name] = None

        sensors_obs = RakudaSensorObs(cameras=camera_obs_np, tactile=tactile_obs_np)
        self.disconnect()
        return RakudaObs(arms=arms, sensors=sensors_obs)

    @override
    def get_observation(self) -> RakudaObs:
        """get_observation get the current observation from the robot system and sensors."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        arm_obs = self.get_arm_observation()
        sensor_obs = self.sensors_observation()
        return RakudaObs(arms=arm_obs, sensors=sensor_obs)

    def get_arm_observation(self) -> RakudaArmObs:
        """Get the current observation from the robot system (leader and follower positions)."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        return self._pair_sys.get_observation()

    def sensors_observation(self) -> RakudaSensorObs:
        """Get the current observation from the sensors."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        if self._sensors is None:
            raise RuntimeError("Sensors are not initialized.")

        # Get camera data
        camera_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.cameras is not None:
            for cam in self._sensors.cameras:
                if cam.is_connected:
                    camera_data[cam.name] = cam.read()
                else:
                    logger.warning(f"Camera {cam.name} is not connected.")
                    camera_data[cam.name] = None
        else:
            logger.warning("No cameras are initialized in sensors.")
            camera_data = {}

        # Get tactile data
        tactile_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.tactile is not None:
            for tac in self._sensors.tactile:
                if tac.is_connected:
                    tactile_data[tac.name] = tac.read()
                else:
                    tactile_data[tac.name] = None
        else:
            logger.warning("No tactile sensors are initialized in sensors.")
            tactile_data = {}

        return RakudaSensorObs(cameras=camera_data, tactile=tactile_data)

    def _init_config(self) -> RakudaSensorConfigs:
        """Initialize sensor configurations based on the provided robot configuration."""

        # if sensors config is provided in RakudaConfig, use it
        if self.config.sensors is not None:
            camera_params = self.config.sensors.cameras
            camera_configs = []

            if camera_params is None:
                camera_configs.append(RealsenseCameraConfig())
            else:
                for cam_param in camera_params:
                    came_cfg = RealsenseCameraConfig()
                    came_cfg.name = cam_param.name
                    came_cfg.width = cam_param.width
                    came_cfg.height = cam_param.height
                    came_cfg.fps = cam_param.fps

                    camera_configs.append(came_cfg)

            tactile_configs = []
            if self.config.sensors.tactile is not None:
                tactile_params = self.config.sensors.tactile
                for tac_param in tactile_params:
                    tactile_configs.append(tac_param)
            else:
                tactile_configs.append(TactileParams(name="main", serial_num="123", fps=30))

        # if no sensors config is provided, use default configs
        else:
            camera_configs = [RealsenseCameraConfig()]
            tactile_configs = [TactileParams(serial_num="123")]

        sensor_configs = RakudaSensorConfigs(cameras=camera_configs, tactile=tactile_configs)
        return sensor_configs

    def _init_sensors(self) -> Sensors:
        if self._sensor_configs is None:
            raise RuntimeError("Failed to initialize sensor configurations.")

        cameras = []
        for cam_cfg in self._sensor_configs.cameras:
            cam = RealsenseCamera(cam_cfg)
            cam.connect()
            cameras.append(cam)
        tactiles = []

        for tac_cfg in self._sensor_configs.tactile:
            digit = DigitSensor(tac_cfg)
            digit.connect()
            tactiles.append(digit)

        sensors = Sensors(cameras=cameras)
        self._sensors = sensors
        return sensors

    @property
    def sensor_configs(self) -> RakudaSensorConfigs:
        if self._sensor_configs is None:
            raise RuntimeError("Failed to initialize sensor configurations.")
        return self._sensor_configs

    @property
    def is_connected(self) -> bool:
        return self._pair_sys.is_connected

    @property
    def sensors(self) -> Sensors:
        if self._sensors is None:
            raise RuntimeError("Failed to initialize sensors.")
        return self._sensors

    @property
    def robot_system(self) -> RakudaPairSys:
        return self._pair_sys
