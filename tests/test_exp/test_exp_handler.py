from robopy.utils.exp_interface import RakudaExpHandler


def test_exp_handler_import():
    from robopy.utils.exp_interface.exp_handler import ExpHandler
    from robopy.utils.exp_interface.rakuda_exp_handler import RakudaExpHandler

    assert ExpHandler is not None
    assert RakudaExpHandler is not None


def test_rakuda_exp_send():
    handler = RakudaExpHandler(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        left_digit_serial="D20542",
        right_digit_serial="D20537",
        fps=10,
    )

    try:
        action = handler.record(max_frames=100, if_async=True)["arms"]["leader"]
        handler.send(max_frame=100, fps=10, leader_action=action)
    except Exception as e:
        raise e


if __name__ == "__main__":
    test_rakuda_exp_send()
