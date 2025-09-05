from robopy.config.robot_config.koch_config import KochConfig
from robopy.config.types import Sensors
from robopy.config.visual_config.camera_config import RealsenseCameraConfig, WebCameraConfig
from robopy.robots.common.composed import ComposedRobot
from robopy.visual.web_camera import WebCamera

from .koch_pair_sys import KochPairSys


class KochRobot(ComposedRobot):
    def __init__(self, cfg: KochConfig) -> None:
        super().__init__()
        self.cfg = cfg
        self.camera_cfg = cfg.camera
        self._robot_system = KochPairSys(cfg)
        self._init_sensors()

    def teleoperation(self) -> None:
        """Start teleoperation mode where leader controls follower."""
        if not self._robot_system:
            raise RuntimeError("Robot system is not initialized.")

        self._robot_system.teleoperate()

    @property
    def robot_system(self) -> KochPairSys:
        return self._robot_system

    @property
    def sensors(self) -> Sensors:
        return self._sensors

    def get_observation(self):
        """Get the current observation from the robot system and sensors."""
        # Get robot observation (leader and follower positions)
        robot_obs = self._robot_system.get_observation()

        # Get camera data
        camera_data = []
        for cam in self._cameras:
            if cam.is_connected:
                camera_data.append(cam.read())
            else:
                camera_data.append(None)

        return {"robot": robot_obs, "cameras": camera_data, "sensors": self._sensors}

    def _init_sensors(self) -> None:
        """_init_sensors Initializes sensors based on the provided configuration.

        Raises:
            ValueError: If an unsupported camera configuration type is encountered.
        """
        self._cameras = []
        index = 0
        for name, cam_cfg in self.camera_cfg.items():
            if isinstance(cam_cfg, WebCameraConfig):
                cam = WebCamera(index, name, cam_cfg)
            elif isinstance(cam_cfg, RealsenseCameraConfig):
                # cam = RealsenseCamera(index,name, cam_cfg)
                cam = WebCamera(index, name, None, is_realsense=True)
            else:
                raise ValueError(f"Unsupported camera config type: {type(cam_cfg)}")
            self._cameras.append(cam)
            index += 1
        self._sensors = Sensors(CAMERA=self._cameras)

    def connect(self) -> None:
        self._robot_system.connect()
        for cam in self._cameras:
            cam.connect()

    def disconnect(self) -> None:
        self._robot_system.disconnect()
        for cam in self._cameras:
            cam.disconnect()
