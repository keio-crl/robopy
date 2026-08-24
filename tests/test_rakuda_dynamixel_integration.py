"""Rakuda leader/follower against two simulated Dynamixel buses.

This exercises the whole stack -- config, arms, pair system, transport -- on the
pty based protocol simulator, so the teleoperation path can be regression tested
without the robot.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Dict, Iterator, Tuple

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from robopy.config.robot_config.rakuda_config import RakudaConfig  # noqa: E402
from robopy.motor.dynamixel_transport import native_available  # noqa: E402
from robopy.robots.rakuda.rakuda_pair_sys import RakudaPairSys  # noqa: E402
from support.fake_dynamixel import FakeDynamixelBus  # noqa: E402

PRESENT_POSITION = 132
RETURN_DELAY_TIME = 9

# Ids used by RakudaLeader/RakudaFollower (both arms share the layout).
RAKUDA_IDS = list(range(1, 13)) + [27, 28, 29, 30, 31]

BACKENDS = ["python"] + (["native"] if native_available() else [])


@pytest.fixture(params=BACKENDS)
def pair(
    request: pytest.FixtureRequest, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> Iterator[Tuple[RakudaPairSys, FakeDynamixelBus, FakeDynamixelBus]]:
    # apply_rakuda_dotconfig writes .robopy/ into the cwd; keep it in tmp_path.
    monkeypatch.chdir(tmp_path)

    leader_device = FakeDynamixelBus(RAKUDA_IDS).start()
    follower_device = FakeDynamixelBus(RAKUDA_IDS).start()
    for offset, motor_id in enumerate(RAKUDA_IDS):
        leader_device.set_value(motor_id, PRESENT_POSITION, 2048 + offset, 4)
        follower_device.set_value(motor_id, PRESENT_POSITION, 1024 + offset, 4)
        leader_device.set_value(motor_id, RETURN_DELAY_TIME, 250, 1)
        follower_device.set_value(motor_id, RETURN_DELAY_TIME, 250, 1)

    system = RakudaPairSys(
        RakudaConfig(
            leader_port=leader_device.port,
            follower_port=follower_device.port,
            motor_backend=request.param,
            usb_latency_timer_ms=None,  # a pty has no USB latency timer
            return_delay_time=0,
        )
    )
    system.connect()
    try:
        yield system, leader_device, follower_device
    finally:
        system.disconnect()
        leader_device.stop()
        follower_device.stop()


def test_connect_zeroes_return_delay_time(
    pair: Tuple[RakudaPairSys, FakeDynamixelBus, FakeDynamixelBus],
) -> None:
    _, leader_device, follower_device = pair

    for device in (leader_device, follower_device):
        assert all(device.get_value(motor_id, RETURN_DELAY_TIME, 1) == 0 for motor_id in RAKUDA_IDS)


def test_get_observation_reads_both_arms(
    pair: Tuple[RakudaPairSys, FakeDynamixelBus, FakeDynamixelBus],
) -> None:
    system, _, _ = pair

    observation = system.get_observation()

    assert observation.leader.shape == (len(RAKUDA_IDS),)
    assert observation.follower.shape == (len(RAKUDA_IDS),)

    # Values follow motor-name order, not id order; check against the seeds.
    seeds = {motor_id: offset for offset, motor_id in enumerate(RAKUDA_IDS)}
    leader_ids = [system.leader.motors.motors[name].id for name in system._leader_motor_names]
    follower_ids = [system.follower.motors.motors[name].id for name in system._follower_motor_names]
    assert observation.leader.tolist() == [2048 + seeds[i] for i in leader_ids]
    assert observation.follower.tolist() == [1024 + seeds[i] for i in follower_ids]


def test_control_step_mirrors_leader_onto_follower(
    pair: Tuple[RakudaPairSys, FakeDynamixelBus, FakeDynamixelBus],
) -> None:
    system, _, follower_device = pair

    leader_positions = system.control_step()

    assert set(leader_positions) == set(system._leader_motor_names)

    # Sync write is tx-only, so wait for the simulator to apply it.
    goal_position = 116
    written: Dict[int, int] = {}
    deadline = time.perf_counter() + 2.0
    while time.perf_counter() < deadline:
        written = {
            i: follower_device.get_value(i, goal_position, 4, signed=True) for i in RAKUDA_IDS
        }
        if all(written.values()):
            break
        time.sleep(0.001)

    assert all(written.values()), f"follower never received the goal positions: {written}"
    for name, position in leader_positions.items():
        follower_id = system.follower.motors.motors[name].id
        assert written[follower_id] == pytest.approx(position)


def test_observation_is_stable_across_repeated_reads(
    pair: Tuple[RakudaPairSys, FakeDynamixelBus, FakeDynamixelBus],
) -> None:
    system, _, _ = pair

    first = system.get_observation()
    for _ in range(20):
        again = system.get_observation()
        assert (again.leader == first.leader).all()
        assert (again.follower == first.follower).all()
