import logging

from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.robots.rakuda.rakuda_pair_sys import RakudaPairSys

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def main():
    rakuda_config = RakudaConfig(
        leader_port="/dev/cu.usbserial-FT7923HA",
        follower_port="/dev/cu.usbserial-FT1JD4T5",
    )
    rakuda = RakudaPairSys(cfg=rakuda_config)
    rakuda.connect()

    print(rakuda.motors)
    rakuda.teleoperate()
    #for i in range(100):
    #    print("leader")
    #    print(rakuda.get_leader_action())
    #    print("follower")
    #    print(rakuda.get_follower_action())

    #    sleep(0.1)

    rakuda.disconnect()


if __name__ == "__main__":
    main()
