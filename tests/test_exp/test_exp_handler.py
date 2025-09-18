from robopy.utils.exp_interface import RakudaExpHandler


def test_rakuda_exp_handler():
    handler = RakudaExpHandler(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        left_digit_serial="D20542",
        right_digit_serial="D20537",
        fps=30,
    )
    handler.recode_save(max_frames=150, save_path="test_01", if_async=True)
    

if __name__ == "__main__":
    test_rakuda_exp_handler()
