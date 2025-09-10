from robopy.config.robot_config.rakuda_config import (
    RakudaArmObs,
    RakudaConfig,
    RakudaSensorConfigs,
    RakudaSensorType,
)
from robopy.config.sensor_config.params_config import TactileParams
from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.visual.realsense_camera import RealsenseCamera

from ..common.composed import ComposedRobot
from .rakuda_pair_sys import RakudaPairSys


class Rakuda_Robot(ComposedRobot):
    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._pair_sys = RakudaPairSys(cfg)
        self._sensor_configs: RakudaSensorConfigs = self._init_config()
        self._sensors: RakudaSensorType = self._init_sensors()

    def connect(self) -> None:
        try:
            self._pair_sys.connect()
        except Exception as e:
            self._pair_sys.disconnect()
            raise e

    def disconnect(self) -> None:
        self._pair_sys.disconnect()

    def get_observation(self) -> RakudaArmObs:
        return self._pair_sys.get_observation()

    def record(self):
        pass

    def _init_config(self) -> RakudaSensorConfigs:
        if self.config.sensors is not None:
            camera_params = self.config.sensors.cameras
            camera_configs = []
            for cam_param in camera_params:
                came_cfg = RealsenseCameraConfig()
                came_cfg.name = cam_param.name
                came_cfg.width = cam_param.width
                came_cfg.height = cam_param.height
                came_cfg.fps = cam_param.fps

                camera_configs.append(came_cfg)

            tactile_configs = []
            tactile_params = self.config.sensors.tactile
            for tac_param in tactile_params:
                tactile_configs.append(tac_param)
        else:
            camera_configs = [RealsenseCameraConfig()]
            tactile_configs = [TactileParams(name="main", fps=30)]
        sensor_configs = RakudaSensorConfigs(cameras=camera_configs, tactile=tactile_configs)
        return sensor_configs

    def _init_sensors(self) -> RakudaSensorType:
        if self._sensor_configs is None:
            raise RuntimeError("Failed to initialize sensor configurations.")

        cameras = []
        for cam_cfg in self._sensor_configs.cameras:
            cam = RealsenseCamera(cam_cfg)
            cam.connect()
            cameras.append(cam)

        sensors = RakudaSensorType(cameras=cameras)
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
    def sensors(self) -> RakudaSensorType:
        if self._sensors is None:
            raise RuntimeError("Failed to initialize sensors.")
        return self._sensors

