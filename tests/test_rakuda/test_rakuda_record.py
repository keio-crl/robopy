from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(
        cameras=None,
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
    slow_mode=False,
)


def test_rakuda_record():
    rakuda = RakudaRobot(config)
    rakuda.connect()
    obs = rakuda.record(max_frame=50, fps=5)

    arm_obs = obs["arms"]
    sensor_obs = obs["sensors"]

    assert arm_obs["leader"].shape[-1] == 17
    assert arm_obs["follower"].shape[-1] == 17

    assert sensor_obs is not None
    print(sensor_obs)


if __name__ == "__main__":
    test_rakuda_record()
