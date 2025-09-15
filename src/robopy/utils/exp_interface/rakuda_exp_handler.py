import concurrent
import concurrent.futures
import os
from time import sleep
from venv import logger

from robopy.config import RakudaConfig
from robopy.config.robot_config.rakuda_config import RakudaObs, RakudaSensorParams
from robopy.config.sensor_config import TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot
from robopy.utils import visualize_rakuda_obs
from robopy.utils.blosc_handler import BLOSCHandler


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
                input()

                if if_async:
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
                    unique_save_dir = save_dir
                    count = 1
                    while os.path.exists(unique_save_dir):
                        unique_save_dir = f"{save_dir}_{count}"
                        count += 1
                    os.makedirs(unique_save_dir)
                    try:
                        save_rakuda_obs(obs, unique_save_dir, self.fps)
                        print(f"Data saved to {unique_save_dir}")
                    except Exception as e:
                        raise RuntimeError(f"Failed to save data: {e}")
                # disconnect and exit
                else:
                    print("Invalid input. Exiting...")
                    self.robot.disconnect()
                    return
        except Exception as e:
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


def save_rakuda_obs(obs: RakudaObs, base_path: str, fps: int) -> None:
    """save Rakuda observation data and visualize it

    Args:
        obs (RakudaObs): observation data from Rakuda robot
    """
    if not os.path.exists(base_path):
        os.makedirs(base_path)

    arms_obs = obs["arms"]
    sensors_obs = obs["sensors"]
    if sensors_obs is None:
        raise ValueError("sensors_obs is None, cannot save tactile data")

    leader_obs = arms_obs["leader"]
    follower_obs = arms_obs["follower"]
    visual_obs = sensors_obs["cameras"]
    tactile_obs = sensors_obs["tactile"]

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        futures = []

        futures.append(
            executor.submit(
                BLOSCHandler.save, leader_obs, os.path.join(base_path, "leader_obs.blosc2")
            )
        )
        futures.append(
            executor.submit(
                BLOSCHandler.save, follower_obs, os.path.join(base_path, "follower_obs.blosc2")
            )
        )
        for cam_name, cam_data in visual_obs.items():
            if cam_data is not None:
                futures.append(
                    executor.submit(
                        BLOSCHandler.save,
                        cam_data,
                        os.path.join(base_path, f"{cam_name}_obs.blosc2"),
                    )
                )
        for tactile_name, tactile_data in tactile_obs.items():
            if tactile_data is not None:
                futures.append(
                    executor.submit(
                        BLOSCHandler.save,
                        tactile_data,
                        os.path.join(base_path, f"{tactile_name}_obs.blosc2"),
                    )
                )

        futures.append(executor.submit(visualize_rakuda_obs, obs, base_path, fps))

        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"Error occurred while saving data: {e}")
