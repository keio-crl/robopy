"""Mode transitions, command exclusion and fault latching.

Two jobs, both about making sure only one thing commands a motor at a time:

* the **state machine** -- which of :class:`~robopy.control.types.ServoState`
  the system is in, and which transitions are legal;
* the **command generation** -- a counter that increments on every transition,
  so a command computed from a state captured before a mode change is rejected
  rather than delivered late into the new mode.

A :class:`CommandLease` is the token that grants the right to write.  Only one
lease can be open per command path at a time, and a lease from an older
generation is dead.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, FrozenSet, List, Tuple

from .types import ControlMode, ServoState, monotonic_ns

__all__ = [
    "CommandLease",
    "FaultRecord",
    "ModeManager",
    "ModeTransitionError",
]


class ModeTransitionError(RuntimeError):
    """Raised on an illegal state transition or an illegal command attempt."""


#: Legal transitions.  ``FAULT`` is reachable from everywhere and only leaves
#: through ``CONFIGURING``, so recovery always re-validates rather than picking
#: up where it left off.
_ALLOWED: Dict[ServoState, FrozenSet[ServoState]] = {
    ServoState.DISCONNECTED: frozenset({ServoState.CONFIGURING, ServoState.FAULT}),
    ServoState.CONFIGURING: frozenset(
        {ServoState.READY, ServoState.DISCONNECTED, ServoState.FAULT}
    ),
    ServoState.READY: frozenset(
        {ServoState.ALIGNING, ServoState.CONFIGURING, ServoState.STOPPING, ServoState.FAULT}
    ),
    ServoState.ALIGNING: frozenset({ServoState.RUNNING, ServoState.STOPPING, ServoState.FAULT}),
    ServoState.RUNNING: frozenset({ServoState.STOPPING, ServoState.FAULT}),
    ServoState.STOPPING: frozenset({ServoState.READY, ServoState.DISCONNECTED, ServoState.FAULT}),
    ServoState.FAULT: frozenset({ServoState.CONFIGURING, ServoState.DISCONNECTED}),
}

#: States in which motion commands may be written to a bus.
_COMMANDABLE: FrozenSet[ServoState] = frozenset({ServoState.ALIGNING, ServoState.RUNNING})


@dataclass(frozen=True)
class FaultRecord:
    """One latched fault.

    Attributes:
        reason: Short machine-readable reason, e.g. ``"bus_timeout"``.
        detail: Human-readable description.
        timestamp_ns: Monotonic time the fault was raised.
        generation: Command generation in force when it was raised.
        source: Which subsystem raised it, e.g. ``"leader_bus"``.
    """

    reason: str
    detail: str
    timestamp_ns: int
    generation: int
    source: str = ""


@dataclass
class CommandLease:
    """The exclusive right to issue commands on one command path.

    Attributes:
        path: Name of the command path, e.g. ``"follower_position"``.
        mode: The control mode the lease was opened for.
        generation: Command generation at the time the lease was opened.
        opened_ns: Monotonic time the lease was opened.
    """

    path: str
    mode: ControlMode
    generation: int
    opened_ns: int
    _manager: "ModeManager" = field(repr=False, compare=False)
    _closed: bool = field(default=False, repr=False, compare=False)

    @property
    def is_valid(self) -> bool:
        """Whether this lease may still be used to write."""
        return (
            not self._closed
            and self._manager.generation == self.generation
            and self._manager.state in _COMMANDABLE
            and self._manager.mode is self.mode
        )

    def require_valid(self) -> None:
        """Raise unless this lease may still be used to write.

        Raises:
            ModeTransitionError: With the specific reason -- closed, superseded
                generation, wrong mode, or a state that must not command.
        """
        if self._closed:
            raise ModeTransitionError(f"The '{self.path}' command lease is closed.")
        if self._manager.generation != self.generation:
            raise ModeTransitionError(
                f"The '{self.path}' command lease is from generation {self.generation}, but the "
                f"current generation is {self._manager.generation}. Commands computed before a "
                "mode change are discarded, not delivered late."
            )
        if self._manager.mode is not self.mode:
            raise ModeTransitionError(
                f"The '{self.path}' lease was opened for {self.mode.value} but the current mode "
                f"is {self._manager.mode.value}."
            )
        if self._manager.state not in _COMMANDABLE:
            raise ModeTransitionError(
                f"State {self._manager.state.value} does not permit commands on '{self.path}'."
            )

    def close(self) -> None:
        """Release the lease so another path may take it."""
        if not self._closed:
            self._closed = True
            self._manager._release(self.path)  # noqa: SLF001 - deliberate coupling

    def __enter__(self) -> "CommandLease":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


class ModeManager:
    """Owns the servo state, the active control mode and the command leases.

    All methods are safe to call from several threads.
    """

    def __init__(self, mode: ControlMode = ControlMode.POSITION_TELEOP) -> None:
        """Start disconnected, in ``mode``."""
        self._lock = threading.RLock()
        self._state = ServoState.DISCONNECTED
        self._mode = mode
        self._generation = 0
        self._faults: List[FaultRecord] = []
        self._leases: Dict[str, CommandLease] = {}
        self._listeners: List[Callable[[ServoState, ServoState], None]] = []

    # -- observation --------------------------------------------------------

    @property
    def state(self) -> ServoState:
        """The current servo state."""
        with self._lock:
            return self._state

    @property
    def mode(self) -> ControlMode:
        """The active control mode."""
        with self._lock:
            return self._mode

    @property
    def generation(self) -> int:
        """The current command generation."""
        with self._lock:
            return self._generation

    @property
    def faults(self) -> Tuple[FaultRecord, ...]:
        """Every latched fault, oldest first."""
        with self._lock:
            return tuple(self._faults)

    @property
    def can_command(self) -> bool:
        """Whether the current state permits writing motion commands."""
        with self._lock:
            return self._state in _COMMANDABLE

    def add_listener(self, callback: Callable[[ServoState, ServoState], None]) -> None:
        """Register ``callback(old_state, new_state)``, called on every transition."""
        with self._lock:
            self._listeners.append(callback)

    # -- transitions --------------------------------------------------------

    def transition(self, new_state: ServoState, *, detail: str = "") -> int:
        """Move to ``new_state`` and bump the command generation.

        Every transition invalidates every open lease, because every transition
        changes what may be written.

        Args:
            new_state: Target state.
            detail: Optional note, used in the error when the move is illegal.

        Returns:
            The new command generation.

        Raises:
            ModeTransitionError: If the transition is not legal from the current
                state.  In particular, ``FAULT`` is only left through
                ``CONFIGURING``: a fault is never cleared by simply resuming.
        """
        with self._lock:
            old = self._state
            if new_state is old:
                return self._generation
            if new_state not in _ALLOWED[old]:
                suffix = f" ({detail})" if detail else ""
                allowed = ", ".join(sorted(s.value for s in _ALLOWED[old]))
                raise ModeTransitionError(
                    f"Cannot go from {old.value} to {new_state.value}{suffix}. "
                    f"Allowed from {old.value}: {allowed}."
                )
            self._state = new_state
            self._generation += 1
            for lease in self._leases.values():
                lease._closed = True  # noqa: SLF001 - deliberate coupling
            self._leases.clear()
            listeners = list(self._listeners)
        for callback in listeners:
            callback(old, new_state)
        return self.generation

    def set_mode(self, mode: ControlMode) -> int:
        """Change the control mode.

        Only legal from a state that is not issuing commands, so a mode switch
        can never land halfway through a control cycle.

        Returns:
            The new command generation.

        Raises:
            ModeTransitionError: If the system is currently commanding.
        """
        with self._lock:
            if self._state in _COMMANDABLE:
                raise ModeTransitionError(
                    f"Cannot change mode while in {self._state.value}. Stop first, so that the "
                    "outgoing mode's commands are finished before the new mode configures."
                )
            if mode is self._mode:
                return self._generation
            self._mode = mode
            self._generation += 1
            self._leases.clear()
            return self._generation

    def fault(self, reason: str, detail: str = "", *, source: str = "") -> FaultRecord:
        """Latch a fault and move to :attr:`ServoState.FAULT`.

        Faults latch: the system stays in ``FAULT`` until an explicit
        reconfiguration, so a condition that keeps recurring cannot be papered
        over by an automatic resume.

        Returns:
            The recorded fault.
        """
        with self._lock:
            record = FaultRecord(
                reason=reason,
                detail=detail,
                timestamp_ns=monotonic_ns(),
                generation=self._generation,
                source=source,
            )
            self._faults.append(record)
            old = self._state
            if old is not ServoState.FAULT:
                self._state = ServoState.FAULT
                self._generation += 1
                for lease in self._leases.values():
                    lease._closed = True  # noqa: SLF001 - deliberate coupling
                self._leases.clear()
            listeners = list(self._listeners)
        if old is not ServoState.FAULT:
            for callback in listeners:
                callback(old, ServoState.FAULT)
        return record

    def clear_faults(self) -> None:
        """Discard the latched fault history.

        This does *not* leave :attr:`ServoState.FAULT` -- call
        :meth:`transition` to ``CONFIGURING`` for that, which forces the state,
        the command generation, the alignment and the initial goal values to be
        re-validated before running again.
        """
        with self._lock:
            self._faults.clear()

    # -- command leases -----------------------------------------------------

    def acquire(self, path: str) -> CommandLease:
        """Take exclusive command rights on ``path``.

        Args:
            path: Name of the command path.  One name per (bus, command kind):
                for instance ``"follower_position"`` and ``"follower_current"``
                are different paths on the same bus and must never be open at
                once -- the caller enforces that by using the same path name for
                anything that writes goal values to that bus.

        Returns:
            An open :class:`CommandLease`.

        Raises:
            ModeTransitionError: If the state does not permit commands, or a
                lease on ``path`` is already open.
        """
        with self._lock:
            if self._state not in _COMMANDABLE:
                raise ModeTransitionError(
                    f"Cannot acquire '{path}' in state {self._state.value}; commands are only "
                    f"permitted in {', '.join(sorted(s.value for s in _COMMANDABLE))}."
                )
            existing = self._leases.get(path)
            if existing is not None and not existing._closed:  # noqa: SLF001
                raise ModeTransitionError(
                    f"Command path '{path}' is already held by a lease opened at "
                    f"{existing.opened_ns}. Two writers on one path is exactly what this "
                    "prevents; close the first lease before opening another."
                )
            lease = CommandLease(
                path=path,
                mode=self._mode,
                generation=self._generation,
                opened_ns=monotonic_ns(),
                _manager=self,
            )
            self._leases[path] = lease
            return lease

    def _release(self, path: str) -> None:
        with self._lock:
            self._leases.pop(path, None)

    def is_held(self, path: str) -> bool:
        """Whether an open lease exists on ``path``."""
        with self._lock:
            lease = self._leases.get(path)
            return lease is not None and not lease._closed  # noqa: SLF001
