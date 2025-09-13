import logging

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.robots.rakuda.rakuda_pair_sys import RakudaPairSys

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def main():
    try:
        rakuda_config = RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
        )
        rakuda = RakudaPairSys(cfg=rakuda_config)
        rakuda.connect()
        rakuda.teleoperate(max_seconds=5)
        rakuda.disconnect()
    except Exception as e:
        raise e


if __name__ == "__main__":
    main()
