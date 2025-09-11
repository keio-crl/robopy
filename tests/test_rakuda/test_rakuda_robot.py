from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(),
    slow_mode=False,
)


def test_rakuda_robot_initialization():
    robot = RakudaRobot(config)
    if robot.is_connected is False:
        robot.connect()

    robot.teleoperation(max_seconds=5)
    if robot.is_connected:
        robot.disconnect()


def test_rakuda_robot_get_observation():
    robot = RakudaRobot(config)
    robot.connect()
    obs = robot.get_observation()
    print(obs)
    assert "arms" in obs
    assert "leader" in obs["arms"]
    assert "follower" in obs["arms"]
    assert obs["arms"]["leader"].shape == (17,)
    assert obs["arms"]["follower"].shape == (17,)

    if robot.is_connected:
        robot.disconnect()


if __name__ == "__main__":
    test_rakuda_robot_get_observation()
