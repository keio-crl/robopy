from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import TactileParams
from robopy.utils.exp_interface import RakudaExpHandler


def test_exp_handler_import():
    from robopy.utils.exp_interface.exp_handler import ExpHandler
    from robopy.utils.exp_interface.rakuda_exp_handler import RakudaExpHandler

    assert ExpHandler is not None
    assert RakudaExpHandler is not None


def test_rakuda_exp_send():
    handler = RakudaExpHandler(
        rakuda_config=RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
        ),
        fps=10,
    )

    try:
        action = handler.record(max_frames=100, if_async=True).arms.leader
        handler.send(max_frame=100, fps=10, leader_action=action)
    except Exception as e:
        raise e


if __name__ == "__main__":
    hander = RakudaExpHandler(
        rakuda_config=RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
            sensors=RakudaSensorParams(
                tactile=[
                    TactileParams(serial_num="D20542", name="left_digit", fps=30),
                    TactileParams(serial_num="D20537", name="right_digit", fps=30),
                ],
            ),
        ),
        fps=10,
    )
    hander.recode_save(max_frames=20, save_path="test_01", if_async=False)
