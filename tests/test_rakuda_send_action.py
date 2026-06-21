from collections import OrderedDict
from unittest.mock import MagicMock

import numpy as np
import pytest

from robopy.robots.rakuda.rakuda_robot import RakudaRobot
from robopy.utils.exp_interface.rakuda_exp_handler import RakudaExpHandler


def _make_handler(start: np.ndarray, obs: object) -> tuple[RakudaExpHandler, MagicMock]:
    handler = object.__new__(RakudaExpHandler)
    handler.fps = 10
    handler.control_hz = 100
    handler._last_follower_action = None

    robot = MagicMock()
    robot.is_connected = True
    robot.robot_system.follower.motors.motors = OrderedDict(
        (f"joint_{index}", object()) for index in range(17)
    )
    robot.get_follower_frame_action.return_value = start
    robot.get_observation.return_value = obs
    handler._robot = robot
    return handler, robot


def test_send_action_interpolates_from_measured_pose_and_returns_obs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("robopy.utils.exp_interface.rakuda_exp_handler.time.sleep", lambda _: None)
    monkeypatch.setattr(
        "robopy.utils.exp_interface.rakuda_exp_handler.time.perf_counter", lambda: 0.0
    )
    start = np.zeros(17, dtype=np.float32)
    target = np.full(17, 10.0, dtype=np.float32)
    expected_obs = object()
    handler, robot = _make_handler(start, expected_obs)

    obs = handler.send_action(target)

    assert obs is expected_obs
    robot.get_follower_frame_action.assert_called_once_with()
    assert robot.send_follower_frame_action.call_count == 10
    np.testing.assert_allclose(
        robot.send_follower_frame_action.call_args_list[0].args[0],
        np.ones(17, dtype=np.float32),
    )
    np.testing.assert_allclose(robot.send_follower_frame_action.call_args_list[-1].args[0], target)


def test_send_action_uses_previous_target_on_next_call(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("robopy.utils.exp_interface.rakuda_exp_handler.time.sleep", lambda _: None)
    monkeypatch.setattr(
        "robopy.utils.exp_interface.rakuda_exp_handler.time.perf_counter", lambda: 0.0
    )
    handler, robot = _make_handler(np.zeros(17, dtype=np.float32), object())
    first_target = np.full(17, 10.0, dtype=np.float32)
    second_target = np.full(17, 20.0, dtype=np.float32)

    handler.send_action(first_target)
    robot.send_follower_frame_action.reset_mock()
    handler.send_action(second_target)

    robot.get_follower_frame_action.assert_called_once_with()
    np.testing.assert_allclose(
        robot.send_follower_frame_action.call_args_list[0].args[0],
        np.full(17, 11.0, dtype=np.float32),
    )


@pytest.mark.parametrize(
    "action",
    [np.zeros(16, dtype=np.float32), np.zeros((1, 17), dtype=np.float32), [np.nan] * 17],
)
def test_send_action_rejects_invalid_action(action: object) -> None:
    handler, robot = _make_handler(np.zeros(17, dtype=np.float32), object())

    with pytest.raises(ValueError):
        handler.send_action(action)  # type: ignore[arg-type]

    robot.send_follower_frame_action.assert_not_called()


def test_send_failure_resets_interpolation_state(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("robopy.utils.exp_interface.rakuda_exp_handler.time.sleep", lambda _: None)
    handler, robot = _make_handler(np.zeros(17, dtype=np.float32), object())
    robot.send_follower_frame_action.side_effect = RuntimeError("write failed")

    with pytest.raises(RuntimeError, match="write failed"):
        handler.send_action(np.ones(17, dtype=np.float32))

    assert handler._last_follower_action is None


def test_robot_sends_follower_array_in_motor_order() -> None:
    robot = object.__new__(RakudaRobot)
    pair_sys = MagicMock()
    pair_sys.follower.motors.motors = OrderedDict([("joint_b", object()), ("joint_a", object())])
    robot._pair_sys = pair_sys

    robot.send_follower_frame_action(np.asarray([1.5, 2.5], dtype=np.float32))

    pair_sys.send_follower_action.assert_called_once_with({"joint_b": 1.5, "joint_a": 2.5})


@pytest.mark.parametrize("control_hz", [0, 9])
def test_handler_rejects_control_hz_below_fps(control_hz: int) -> None:
    with pytest.raises(ValueError, match="control_hz"):
        RakudaExpHandler(
            rakuda_config=MagicMock(),
            metadata_config=MagicMock(),
            fps=10,
            control_hz=control_hz,
        )
