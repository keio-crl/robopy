"""State transitions, command generations and lease exclusivity."""

from __future__ import annotations

import pytest

from robopy.control.mode_manager import ModeManager, ModeTransitionError
from robopy.control.types import ControlMode, ServoState


def _ready(manager: ModeManager) -> None:
    manager.transition(ServoState.CONFIGURING)
    manager.transition(ServoState.READY)


def _running(manager: ModeManager) -> None:
    _ready(manager)
    manager.transition(ServoState.ALIGNING)
    manager.transition(ServoState.RUNNING)


class TestTransitions:
    def test_starts_disconnected_and_cannot_command(self) -> None:
        manager = ModeManager()
        assert manager.state is ServoState.DISCONNECTED
        assert not manager.can_command

    def test_the_normal_sequence_reaches_running(self) -> None:
        manager = ModeManager()
        _running(manager)
        assert manager.state is ServoState.RUNNING
        assert manager.can_command

    def test_skipping_configuration_is_refused(self) -> None:
        manager = ModeManager()
        with pytest.raises(ModeTransitionError, match="Cannot go from disconnected to running"):
            manager.transition(ServoState.RUNNING)

    def test_every_transition_bumps_the_generation(self) -> None:
        manager = ModeManager()
        generations = [manager.generation]
        for state in (ServoState.CONFIGURING, ServoState.READY, ServoState.ALIGNING):
            generations.append(manager.transition(state))
        assert generations == sorted(set(generations))

    def test_a_transition_to_the_same_state_is_a_no_op(self) -> None:
        manager = ModeManager()
        _ready(manager)
        before = manager.generation
        assert manager.transition(ServoState.READY) == before

    def test_listeners_see_each_transition(self) -> None:
        manager = ModeManager()
        seen: list[tuple[str, str]] = []
        manager.add_listener(lambda old, new: seen.append((old.value, new.value)))
        _ready(manager)
        assert seen == [("disconnected", "configuring"), ("configuring", "ready")]


class TestFaults:
    def test_a_fault_latches_and_only_clears_through_configuring(self) -> None:
        manager = ModeManager()
        _running(manager)
        record = manager.fault("bus_timeout", "the follower stopped answering", source="follower")
        assert manager.state is ServoState.FAULT
        assert record.reason == "bus_timeout"

        # Resuming straight back into RUNNING is not allowed: recovery must
        # re-validate through CONFIGURING.
        with pytest.raises(ModeTransitionError):
            manager.transition(ServoState.RUNNING)
        manager.transition(ServoState.CONFIGURING)
        assert manager.state is ServoState.CONFIGURING

    def test_clearing_the_history_does_not_leave_the_fault_state(self) -> None:
        manager = ModeManager()
        _running(manager)
        manager.fault("overheat")
        manager.clear_faults()
        assert manager.faults == ()
        assert manager.state is ServoState.FAULT

    def test_repeated_faults_accumulate_without_extra_transitions(self) -> None:
        manager = ModeManager()
        _running(manager)
        manager.fault("first")
        generation = manager.generation
        manager.fault("second")
        assert len(manager.faults) == 2
        assert manager.generation == generation


class TestLeases:
    def test_a_lease_is_exclusive(self) -> None:
        manager = ModeManager()
        _running(manager)
        manager.acquire("follower_command")
        with pytest.raises(ModeTransitionError, match="already held"):
            manager.acquire("follower_command")

    def test_closing_a_lease_frees_the_path(self) -> None:
        manager = ModeManager()
        _running(manager)
        lease = manager.acquire("follower_command")
        lease.close()
        assert not manager.is_held("follower_command")
        manager.acquire("follower_command").close()

    def test_a_lease_cannot_be_taken_outside_a_commandable_state(self) -> None:
        manager = ModeManager()
        _ready(manager)
        with pytest.raises(ModeTransitionError, match="Cannot acquire"):
            manager.acquire("follower_command")

    def test_a_transition_invalidates_every_open_lease(self) -> None:
        manager = ModeManager()
        _running(manager)
        lease = manager.acquire("follower_command")
        assert lease.is_valid
        manager.transition(ServoState.STOPPING)
        assert not lease.is_valid
        with pytest.raises(ModeTransitionError, match="closed"):
            lease.require_valid()

    def test_a_superseded_generation_is_named_in_the_error(self) -> None:
        manager = ModeManager()
        _running(manager)
        lease = manager.acquire("follower_command")
        lease.generation -= 1  # pretend it was computed a generation ago
        with pytest.raises(ModeTransitionError, match="discarded, not delivered late"):
            lease.require_valid()

    def test_a_lease_works_as_a_context_manager(self) -> None:
        manager = ModeManager()
        _running(manager)
        with manager.acquire("follower_command") as lease:
            lease.require_valid()
        assert not manager.is_held("follower_command")


class TestModeChanges:
    def test_the_mode_cannot_change_while_commanding(self) -> None:
        manager = ModeManager(ControlMode.POSITION_TELEOP)
        _running(manager)
        with pytest.raises(ModeTransitionError, match="Stop first"):
            manager.set_mode(ControlMode.BILATERAL_JOINT)

    def test_changing_the_mode_bumps_the_generation(self) -> None:
        manager = ModeManager(ControlMode.POSITION_TELEOP)
        _ready(manager)
        before = manager.generation
        after = manager.set_mode(ControlMode.BILATERAL_JOINT)
        assert after > before
        assert manager.mode is ControlMode.BILATERAL_JOINT

    def test_a_lease_from_the_previous_mode_is_dead(self) -> None:
        manager = ModeManager(ControlMode.POSITION_TELEOP)
        _running(manager)
        lease = manager.acquire("follower_command")
        manager.transition(ServoState.STOPPING)
        manager.transition(ServoState.READY)
        manager.set_mode(ControlMode.BILATERAL_JOINT)
        assert not lease.is_valid
