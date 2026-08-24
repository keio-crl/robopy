"""End-to-end tests for the Dynamixel transports against a simulated bus.

Everything here runs against :class:`tests.support.fake_dynamixel.FakeDynamixelBus`,
a protocol 2.0 device simulator on a pty, so no hardware is required.  The same
assertions are run for both backends, which is what keeps the optional C++
transport honest: it has to be observationally identical to the Python one.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Callable, Dict, Iterator, List

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from robopy.motor.dynamixel_bus import (  # noqa: E402
    DynamixelBus,
    DynamixelCommError,
    DynamixelMotor,
    native_available,
    sync_read_parallel,
)
from robopy.motor.dynamixel_control_table import XControlTable  # noqa: E402
from support.fake_dynamixel import (  # noqa: E402
    INST_FAST_SYNC_READ,
    INST_SYNC_READ,
    FakeDynamixelBus,
)

MOTOR_IDS = [1, 2, 3, 5, 7]
PRESENT_POSITION = 132
GOAL_POSITION = 116

BACKENDS = ["python"] + (["native"] if native_available() else [])


def make_motors() -> Dict[str, DynamixelMotor]:
    return {
        f"joint_{motor_id}": DynamixelMotor(motor_id, f"joint_{motor_id}", "xm430-w350")
        for motor_id in MOTOR_IDS
    }


@pytest.fixture
def fake() -> Iterator[FakeDynamixelBus]:
    device = FakeDynamixelBus(MOTOR_IDS).start()
    try:
        yield device
    finally:
        device.stop()


@pytest.fixture(params=BACKENDS)
def bus(request: pytest.FixtureRequest, fake: FakeDynamixelBus) -> Iterator[DynamixelBus]:
    dynamixel_bus = DynamixelBus(fake.port, make_motors(), backend=request.param)
    # The pty has no USB latency timer; skip the tuning attempt.
    dynamixel_bus.open(latency_timer_ms=None)
    try:
        yield dynamixel_bus
    finally:
        dynamixel_bus.close()


def joint_names() -> List[str]:
    return [f"joint_{motor_id}" for motor_id in MOTOR_IDS]


def wait_until(predicate: Callable[[], bool], timeout: float = 2.0) -> None:
    """Sync write is tx-only, so give the simulator a moment to apply it."""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if predicate():
            return
        time.sleep(0.001)
    assert predicate(), "timed out waiting for the simulated bus to catch up"


def test_backend_is_the_requested_one(bus: DynamixelBus, request: pytest.FixtureRequest) -> None:
    assert bus.backend in BACKENDS


def test_sync_read_returns_raw_steps_without_calibration(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    for offset, motor_id in enumerate(MOTOR_IDS):
        fake.set_value(motor_id, PRESENT_POSITION, 2048 + offset, 4)

    positions = bus.sync_read(XControlTable.PRESENT_POSITION, joint_names())

    assert positions == {name: 2048 + i for i, name in enumerate(joint_names())}


def test_sync_read_sign_extends_negative_values(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    fake.set_value(MOTOR_IDS[0], PRESENT_POSITION, -1234 & 0xFFFFFFFF, 4)

    positions = bus.sync_read(XControlTable.PRESENT_POSITION, joint_names())

    assert positions["joint_1"] == -1234


def test_sync_write_round_trips_through_the_bus(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    goals = {name: 1000 + 3 * i for i, name in enumerate(joint_names())}

    bus.sync_write(XControlTable.GOAL_POSITION, goals)

    wait_until(lambda: fake.get_value(MOTOR_IDS[-1], GOAL_POSITION, 4, signed=True) != 0)
    for i, motor_id in enumerate(MOTOR_IDS):
        assert fake.get_value(motor_id, GOAL_POSITION, 4, signed=True) == 1000 + 3 * i


def test_calibration_converts_steps_to_degrees_and_back(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    # joint_1 is inverted with a homing offset; joint_2 is plain.
    bus.set_calibration({"joint_1": (100, True), "joint_2": (-50, False)})
    fake.set_value(MOTOR_IDS[0], PRESENT_POSITION, 1024, 4)
    fake.set_value(MOTOR_IDS[1], PRESENT_POSITION, 1024, 4)

    positions = bus.sync_read(XControlTable.PRESENT_POSITION, joint_names())

    # (-1024 + 100) * 360/4096  and  (1024 - 50) * 360/4096
    assert positions["joint_1"] == pytest.approx(-924 * 360 / 4096, abs=1e-3)
    assert positions["joint_2"] == pytest.approx(974 * 360 / 4096, abs=1e-3)

    bus.sync_write(XControlTable.GOAL_POSITION, {"joint_1": positions["joint_1"]})
    wait_until(lambda: fake.get_value(MOTOR_IDS[0], GOAL_POSITION, 4, signed=True) == 1024)


def test_single_read_and_write(bus: DynamixelBus, fake: FakeDynamixelBus) -> None:
    bus.write(XControlTable.TORQUE_ENABLE, "joint_3", 1)
    wait_until(lambda: fake.get_value(3, 64, 1) == 1)
    assert bus.read(XControlTable.TORQUE_ENABLE, "joint_3") == 1


def test_torque_helpers_touch_every_motor(bus: DynamixelBus, fake: FakeDynamixelBus) -> None:
    bus.torque_enabled()
    wait_until(lambda: all(fake.get_value(motor_id, 64, 1) == 1 for motor_id in MOTOR_IDS))

    bus.torque_disabled(specific_motor_names=["joint_1"])
    wait_until(lambda: fake.get_value(1, 64, 1) == 0)
    assert fake.get_value(2, 64, 1) == 1


def test_set_return_delay_time(bus: DynamixelBus, fake: FakeDynamixelBus) -> None:
    fake.set_all(9, 250, 1)

    bus.set_return_delay_time(0)

    wait_until(lambda: all(fake.get_value(motor_id, 9, 1) == 0 for motor_id in MOTOR_IDS))


def test_missing_motor_raises_instead_of_shortening_the_result(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    """A partial answer must not silently shrink the observation vector."""
    names = joint_names() + ["ghost"]
    bus.motors["ghost"] = DynamixelMotor(99, "ghost", "xm430-w350")

    with pytest.raises(DynamixelCommError):
        bus.sync_read(XControlTable.PRESENT_POSITION, names)


def test_groups_are_reused_across_calls(bus: DynamixelBus) -> None:
    names = joint_names()
    bus.sync_read(XControlTable.PRESENT_POSITION, names)
    handles = dict(bus._read_handles)

    bus.sync_read(XControlTable.PRESENT_POSITION, names)

    assert dict(bus._read_handles) == handles
    assert len(handles) == 1


def test_parallel_read_matches_sequential_read(fake: FakeDynamixelBus) -> None:
    other = FakeDynamixelBus(MOTOR_IDS).start()
    try:
        for offset, motor_id in enumerate(MOTOR_IDS):
            fake.set_value(motor_id, PRESENT_POSITION, 100 + offset, 4)
            other.set_value(motor_id, PRESENT_POSITION, 200 + offset, 4)

        bus_a = DynamixelBus(fake.port, make_motors())
        bus_b = DynamixelBus(other.port, make_motors())
        bus_a.open(latency_timer_ms=None)
        bus_b.open(latency_timer_ms=None)
        try:
            names = joint_names()
            first, second = sync_read_parallel(
                [
                    (bus_a, XControlTable.PRESENT_POSITION, names),
                    (bus_b, XControlTable.PRESENT_POSITION, names),
                ]
            )
            assert first == bus_a.sync_read(XControlTable.PRESENT_POSITION, names)
            assert second == bus_b.sync_read(XControlTable.PRESENT_POSITION, names)
            assert first["joint_1"] == 100
            assert second["joint_1"] == 200
        finally:
            bus_a.close()
            bus_b.close()
    finally:
        other.stop()


@pytest.mark.skipif(not native_available(), reason="robopy_dxl is not installed")
def test_native_backend_uses_fast_sync_read() -> None:
    device = FakeDynamixelBus(MOTOR_IDS).start()
    try:
        bus = DynamixelBus(device.port, make_motors(), backend="native")
        bus.open(latency_timer_ms=None)
        try:
            bus.sync_read(XControlTable.PRESENT_POSITION, joint_names())
            assert device.instruction_counts.get(INST_FAST_SYNC_READ) == 1
            assert device.instruction_counts.get(INST_SYNC_READ) is None
            assert bus.uses_fast_sync_read(XControlTable.PRESENT_POSITION, joint_names())
        finally:
            bus.close()
    finally:
        device.stop()


@pytest.mark.skipif(not native_available(), reason="robopy_dxl is not installed")
def test_native_backend_downgrades_on_firmware_without_fast_sync_read() -> None:
    device = FakeDynamixelBus(MOTOR_IDS, supports_fast_sync_read=False).start()
    try:
        for offset, motor_id in enumerate(MOTOR_IDS):
            device.set_value(motor_id, PRESENT_POSITION, 300 + offset, 4)

        bus = DynamixelBus(device.port, make_motors(), backend="native")
        bus.open(latency_timer_ms=None)
        try:
            names = joint_names()
            positions = bus.sync_read(XControlTable.PRESENT_POSITION, names)
            assert positions == {name: 300 + i for i, name in enumerate(names)}
            assert not bus.uses_fast_sync_read(XControlTable.PRESENT_POSITION, names)

            # The downgrade is permanent: no further 0x8A attempts.
            attempted = device.instruction_counts[INST_FAST_SYNC_READ]
            bus.sync_read(XControlTable.PRESENT_POSITION, names)
            assert device.instruction_counts[INST_FAST_SYNC_READ] == attempted
        finally:
            bus.close()
    finally:
        device.stop()


def test_registered_groups_survive_close_and_reopen(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    fake.set_value(MOTOR_IDS[0], PRESENT_POSITION, 777, 4)
    names = joint_names()
    assert bus.sync_read(XControlTable.PRESENT_POSITION, names)["joint_1"] == 777
    handles = dict(bus._read_handles)

    bus.close()
    bus.open(latency_timer_ms=None)

    assert bus.sync_read(XControlTable.PRESENT_POSITION, names)["joint_1"] == 777
    assert dict(bus._read_handles) == handles


def test_parallel_read_of_the_same_bus_twice_does_not_deadlock(
    bus: DynamixelBus, fake: FakeDynamixelBus
) -> None:
    fake.set_value(MOTOR_IDS[0], PRESENT_POSITION, 555, 4)
    names = joint_names()

    first, second = sync_read_parallel(
        [
            (bus, XControlTable.PRESENT_POSITION, names),
            (bus, XControlTable.PRESENT_POSITION, names[:2]),
        ]
    )

    assert first["joint_1"] == 555
    assert list(second) == names[:2]
