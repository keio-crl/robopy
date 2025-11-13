import logging

from robopy.config.robot_config.koch_config import KochConfig, KochSensorConfig
from robopy.config import RealsenseCameraConfig
from robopy.robots.koch.koch_robot import KochRobot
from robopy.robots.koch.koch_pair_sys import KochPairSys

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def main():
    try:
        koch_config = KochConfig(
            leader_port="/dev/ttyACM1",
            follower_port="/dev/ttyACM0",
            calibration_path="/home/fujii/.cache/calibration/curtain_new.pkl",
            sensors=KochSensorConfig(
                cameras={
                    "main": RealsenseCameraConfig(
                        width=640,
                        height=480,
                        fps=30,
                        is_depth_camera=True
                    )
                }
            ),

        )
        koch = KochRobot(cfg=koch_config)
        koch.connect()
        obs = koch.get_observation()
        print(obs.keys())
        print(obs["robot"])
        print(obs["cameras"])
        # koch.teleoperation(max_seconds=5)
        koch.disconnect()
    except Exception as e:
        raise e


if __name__ == "__main__":
    main()
