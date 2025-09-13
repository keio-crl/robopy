import time

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
    start_time = time.time()
    obs = rakuda.record_parallel(max_frame=150, fps=30)
    print(f"Recording took {time.time() - start_time:.2f} seconds")
    arm_obs = obs["arms"]
    sensor_obs = obs["sensors"]
    print(f"Leader arm obs shape: {arm_obs['leader'].shape}")
    print(f"Follower arm obs shape: {arm_obs['follower'].shape}")
    print(sensor_obs)

    assert arm_obs["leader"].shape[-1] == 17
    assert arm_obs["follower"].shape[-1] == 17

    assert sensor_obs is not None


if __name__ == "__main__":
    test_rakuda_record()
