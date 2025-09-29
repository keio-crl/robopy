import os
from time import sleep
from typing import override
from venv import logger

from numpy import float32
from numpy.typing import NDArray

from robopy.config import RakudaConfig, RakudaObs, RakudaSensorParams, TactileParams
from robopy.robots import RakudaRobot
from robopy.utils import RakudaSaveWorker

from .exp_handler import ExpHandler


class RakudaExpHandler(ExpHandler):
    """This class handles the experimental interface for the Rakuda robot.

    Sensors:
    1x Realsense, 2x Digit,

    Example:
    ```python
    from robopy.utils.exp_interface import RakudaExpHandler
    handler = RakudaExpHandler(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        left_digit_serial="D20542",
        right_digit_serial="D20537",
        fps=30,
    )
    handler.recode_save(max_frames=150, save_path="test_01", if_async=True)
    ```
    """

    def __init__(
        self,
        leader_port: str,
        follower_port: str,
        left_digit_serial: str,
        right_digit_serial: str,
        fps: int = 10,
    ) -> None:
        """__init__ initialize Rakuda experimental handler

        Args:
            leader_port (str): leader serial port
            follower_port (str): follower serial port
            left_digit_serial (str): left digit serial number
            right_digit_serial (str): right digit serial number
            fps (int, optional): The frequency to capture obs. Defaults to 10

        Raises:
            ValueError: fps must be between 1 and 30
            RuntimeError: failed to connect to Rakuda robot
        """

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
        self.save_worker = RakudaSaveWorker(config, worker_num=6, fps=self.fps)
        try:
            self.robot.connect()
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Rakuda robot: {e}")

    @override
    def record(self, max_frames: int, if_async: bool = True) -> RakudaObs:
        """record data from Rakuda robot

        Args:
            max_frames (int): maximum number of frames to record
            if_async (bool, optional): If use parallel. Defaults to True.

        Raises:
            RuntimeError: failed to record from Rakuda robot

        Returns:
            RakudaObs: recorded observation
        """
        try:
            if if_async:
                obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps)
            else:
                obs = self.robot.record(max_frame=max_frames, fps=self.fps)
        except Exception as e:
            raise RuntimeError(f"Failed to record from Rakuda robot: {e}")
        except KeyboardInterrupt:
            logger.info("Recording stopped by user...")
            sleep(0.5)
            self.robot.disconnect()
            raise RuntimeError("Recording stopped by user")
        return obs

    @override
    def recode_save(
        self,
        max_frames: int,
        save_path: str,
        if_async: bool = True,
        save_gif: bool = True,
        warmup_time: int = 5,
    ) -> None:
        """record and save data from Rakuda robot

        Args:
            max_frames (int): maximum number of frames to record
            save_path (str): path to save the recorded data:
            data will be saved to `data/{save_path}`
            if_async (bool, optional): if a . Defaults to True.
            save_gif (bool, optional): if save gif. Defaults to True.
            warmup_time (int, optional): warm up time before recording. Defaults to 5.

        Raises:
            RuntimeError: failed to record from Rakuda robot
            RuntimeError: failed to save data
        """
        try:
            print("Starting recording...")
            if not self.robot.is_connected:
                self.robot.connect()

            while True:
                print("Press 'Enter' to warm up , or 'q' to quit")
                input_str = input()
                if input_str.lower() == "q":
                    print("Exiting...")
                    self.robot.disconnect()
                    sleep(0.5)
                    return
                print(f"Warming up for default {warmup_time} seconds...")
                self.robot.teleoperation(warmup_time)
                print("Press 'Enter' to start recording...")
                input_str = input()
                print("Recording...")

                if if_async:
                    print("Using asynchronous recording...")
                    obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps)
                else:
                    obs = self.robot.record(max_frame=max_frames, fps=self.fps)
                print(
                    "Recording finished. print 1~9 to save data,",
                    " or 'e' to record again",
                )
                input_str = input()

                # handle user input
                if input_str.lower() == "e":
                    print("Recording again...")
                    continue
                # save data
                elif input_str in [str(i) for i in range(1, 10)]:
                    save_dir = os.path.join("data", f"{save_path}", f"{input_str}")
                    count = 1
                    unique_save_dir = save_dir + f"_{count}"
                    while os.path.exists(unique_save_dir):
                        unique_save_dir = f"{save_dir}_{count}"
                        count += 1
                    os.makedirs(unique_save_dir)
                    self.save_worker.save_all_obs(obs, unique_save_dir, save_gif)
                # disconnect and exit
                else:
                    print("Invalid input. Exiting...")
                    self.robot.disconnect()
                    return
        except Exception as e:
            self.robot.disconnect()
            raise RuntimeError(f"Failed to record from Rakuda robot: {e}")
        except KeyboardInterrupt:
            logger.info("Recording stopped by user...")
            sleep(0.5)
            self.robot.disconnect()

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

    @override
    def send(self, max_frame: int, fps: int, leader_action: NDArray[float32]) -> None:
        """send leader action to Rakuda robot"""
        self.robot.send(max_frame, fps, leader_action)
