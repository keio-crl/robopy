import os
from time import sleep
from venv import logger

from robopy.config import RakudaConfig
from robopy.config.robot_config.rakuda_config import RakudaObs, RakudaSensorParams
from robopy.config.sensor_config import TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot
from robopy.utils.worker.rakuda_save_woker import RakudaSaveWorker


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
        self.save_worker = RakudaSaveWorker(config, worker_num=6, fps=self.fps)
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

    def recode_save(self, max_frames: int, save_path: str, if_async: bool = True) -> None:
        """record and save data from Rakuda robot

        Args:
            max_frames (int): maximum number of frames to record
            save_path (str): path to save the recorded data
            if_async (bool, optional): if a . Defaults to True.

        Raises:
            RuntimeError: _failed to record from Rakuda robot
            RuntimeError: _failed to save data
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
                print("Warming up for 5 seconds...")
                self.robot.teleoperation(5)
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
                    unique_save_dir = save_dir + str(count)
                    while os.path.exists(unique_save_dir):
                        unique_save_dir = f"{save_dir}_{count}"
                        count += 1
                    os.makedirs(unique_save_dir)
                    self.save_worker.save_all_obs(obs, unique_save_dir, save_gif=False)
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
