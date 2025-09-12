from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(),
    slow_mode=False,
)


def test_rakuda_record():
    rakuda = RakudaRobot(config)
    rakuda.connect()
    obs = rakuda.record(max_frame=15, fps=5)
    print(obs["arms"]["leader"].shape)


if __name__ == "__main__":
    test_rakuda_record()
