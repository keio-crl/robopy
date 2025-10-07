from robopy.config import RakudaConfig
from robopy.utils import RakudaExpHandler


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
        action = handler.record(max_frames=200, if_async=True).arms.leader
        handler.send(max_frame=200, fps=10, leader_action=action)
    except Exception as e:
        raise e
    except KeyboardInterrupt:
        handler.robot.disconnect()


if __name__ == "__main__":
    test_rakuda_exp_send()
