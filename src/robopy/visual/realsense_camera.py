from robopy.config.visual_config.camera_config import RealsenseCameraConfig

from .camera import Camera

try:
    import pyrealsense2 as rs  # type: ignore[import]
except ImportError:
    rs = None


class RealsenseCamera(Camera):
    """Implementation class for Intel RealSense cameras using pyrealsense2"""

    def __init__(self, index: int, name: str, config: RealsenseCameraConfig | None) -> None:
        super().__init__()
        self.index = index
        self.name = name
        if rs is None:
            err = (
                """pyrealsense2 is not installed.
                Please make sure you installed pyrealsense2.""",
            )
            raise ImportError(err)
        self.config = config if config is not None else RealsenseCameraConfig()
        self.pipeline = rs.pipeline()
        self.is_connected = False

    def connect(self) -> None:
        # TODO: Implement actual connection logic
        pass
