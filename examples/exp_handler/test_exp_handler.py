from logging import INFO, getLogger

from robopy.config import RakudaConfig
from robopy.utils import H5Handler, MetaDataConfig, RakudaExpHandler

logger = getLogger(__name__)
logger.setLevel(INFO)


def test_exp_handler_import():
    from robopy.utils.exp_interface.exp_handler import ExpHandler
    from robopy.utils.exp_interface.rakuda_exp_handler import RakudaExpHandler

    assert ExpHandler is not None
    assert RakudaExpHandler is not None


def test_rakuda_exp_send():
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
        handler.close()
        raise e
    except KeyboardInterrupt:
        handler.close()


if __name__ == "__main__":
    handler = RakudaExpHandler(
        rakuda_config=RakudaConfig(
            leader_port="/dev/ttyUSB1",
            follower_port="/dev/ttyUSB0",
            # sensors=RakudaSensorParams(
            #    tactile=[
            #        TactileParams(serial_num="D20542", name="left"),
            #        TactileParams(serial_num="D20537", name="right"),
            #    ],
            # ),
        ),
        metadata_config=MetaDataConfig(
            task_name="test_task",
            description="This is a test task of no tactile sensors",
            date="2024-06-01",
        ),
        fps=10,
    )
    try:
        action = H5Handler.load_single_array(
            file_path="./data/test_01/1_1/rakuda_observations.h5", dataset_name="arm/leader"
        )
        handler.record_save_with_fixed_leader(
            max_frame=action.shape[0], leader_action=action, save_path="test_no_tactile"
        )
    except Exception as e:
        handler.close()
        raise e
