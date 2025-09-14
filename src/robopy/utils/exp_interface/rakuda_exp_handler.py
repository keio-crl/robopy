from robopy.config import RakudaConfig
from robopy.config.robot_config.rakuda_config import RakudaObs, RakudaSensorParams
from robopy.config.sensor_config import TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot


class RakudaExpHandler:
    """This class handles the experimental interface for the Rakuda robot.\n
    **This Rakuda experiment handler hard codes the sensors type and configuration
    to take tactile data**.
    """

    def __init__(
        self,
        leader_port: str,
        follower_port: str,
        left_digit_serial: str,
        right_digit_serial: str,
        fps: int = 10,
    ) -> None:
        config = self._init_config(
            leader_port_num=leader_port,
            follower_port_num=follower_port,
            left_digit_serial=left_digit_serial,
            right_digit_serial=right_digit_serial,
        )
        if 30 < fps or fps < 1:
            raise ValueError("FPS must be between 1 and 30")
        else:
            self.fps = fps

        self.robot = RakudaRobot(config)

        try:
            self.robot.connect()
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Rakuda robot: {e}")

    def record(self, max_frames: int, if_async: bool = True) -> RakudaObs:
        """record data from Rakuda robot

        Args:
            max_frames (int): maximum number of frames to record
            if_async (bool, optional): if a . Defaults to True.

        Raises:
            RuntimeError: _failed to record from Rakuda robot

        Returns:
            RakudaObs: _observation data from Rakuda robot
        """
        try:
            if if_async:
                obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps)
            else:
                obs = self.robot.record(max_frame=max_frames, fps=self.fps)
        except Exception as e:
            raise RuntimeError(f"Failed to record from Rakuda robot: {e}")
        return obs

    def _init_config(
        self,
        leader_port_num: str,
        follower_port_num: str,
        left_digit_serial: str,
        right_digit_serial: str,
    ) -> RakudaConfig:
        return RakudaConfig(
            leader_port=leader_port_num,
            follower_port=follower_port_num,
            sensors=RakudaSensorParams(
                tactile=[
                    TactileParams(
                        serial_num=left_digit_serial,
                        name="left_digit",
                    ),
                    TactileParams(
                        serial_num=right_digit_serial,
                        name="right_digit",
                    ),
                ]
            ),
        )
