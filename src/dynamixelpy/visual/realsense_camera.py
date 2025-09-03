from dynamixelpy.config.visual_config.camera_config import RealsenseCameraConfig

from .camera import Camera

try:
    import pyrealsense2 as rs  # type: ignore[import]
except ImportError:
    rs = None


class RealsenseCamera(Camera):
    """Implementation class for Intel RealSense cameras using pyrealsense2"""

    def __init__(self, config: RealsenseCameraConfig | None) -> None:
        super().__init__()
        if rs is None:
            err = (
                """pyrealsense2 is not installed.
                Please make sure you installed pyrealsense2.""",
            )
            raise ImportError(err)
        self.config = config if config is not None else RealsenseCameraConfig()
        self.pipeline = rs.pipeline()
