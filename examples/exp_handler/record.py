from logging import INFO, basicConfig

from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
from robopy.utils import MetaDataConfig, RakudaExpHandler

basicConfig(level=INFO)


def exp_handler_import():
    from robopy.utils.exp_interface.exp_handler import ExpHandler
    from robopy.utils.exp_interface.rakuda_exp_handler import RakudaExpHandler

    assert ExpHandler is not None
    assert RakudaExpHandler is not None


def rakuda_exp_send():
    handler = RakudaExpHandler(
        rakuda_config=RakudaConfig(
            leader_port="/dev/ttyUSB1",
            follower_port="/dev/ttyUSB0",
        ),
        fps=10,
        metadata_config=MetaDataConfig(
            task_name="test_task",
            description="This is a test task",
            date="2024-06-01",
        ),
    )

    try:
        action = handler.record(max_frames=200).arms.leader
        handler.send(max_frame=200, fps=10, leader_action=action)
    except Exception as e:
        raise e
    except KeyboardInterrupt:
        handler.robot.disconnect()


if __name__ == "__main__":
    handler = RakudaExpHandler(
        rakuda_config=RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
            sensors=RakudaSensorParams(
                tactile=[
                    TactileParams(serial_num="D20542", name="left"),
                    TactileParams(serial_num="D20537", name="right"),
                ],
            ),
        ),
        metadata_config=MetaDataConfig(
            task_name="test_task",
            description="This is a test task of no tactile sensors",
            date="2024-06-01",
        ),
        fps=10,
    )
    handler.record_save(max_frames=100, save_path="test_01")
