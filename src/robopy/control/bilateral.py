"""Joint-space bilateral coupling: a virtual spring and damper between two arms.

The control law, with leader ``m`` and follower ``s`` in each machine's own
calibrated joint coordinates:

.. math::

    e &= q_s - (S q_m + b) \\\\
    \\dot e &= v_s - S v_m \\\\
    u &= K e + D \\dot e \\\\
    \\tau_s^{cmd} &= \\hat g_s(q_s) - u \\\\
    \\tau_m^{cmd} &= \\hat g_m(q_m) + S^\\top u

``S`` is the fixed angle ratio and axis correspondence, ``b`` the calibrated
offset.  Each machine's own rotation-sign convention already lives in its
:class:`~robopy.control.joint_mapping.JointCalibration`; ``S`` carries only the
*mirroring between the two machines*, so a sign is never applied twice.

With a fixed ``S`` and ``b``, in ideal continuous time and with no delay, the
coupling term stores energy :math:`\\tfrac12 e^\\top K e`.  That identity is why
``K`` and ``D`` are non-negative diagonals here.  It is **not** a stability
guarantee: discretisation, communication delay and independent saturation on
each side all break it, which is why the simulated-contact tests record the gain
range that was actually checked instead of claiming a general bound.

This module performs no serial I/O and converts nothing to amperes.  Torque to
current happens in the motor-side adapter, against a per-joint validated model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Protocol, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from .types import BilateralOutput, JointState, LimitFlags

__all__ = [
    "BilateralController",
    "BilateralGains",
    "GravityProvider",
    "UnvalidatedGravityProvider",
    "coupling_stored_energy",
]


class GravityProvider(Protocol):
    """Supplies gravity-compensation torques for one specific machine.

    A provider is bound to one side.  The leader and the follower are different
    machines with different masses and different payloads, so a follower model
    is never acceptable as a leader model -- :class:`BilateralController`
    enforces that by checking :attr:`side`.
    """

    @property
    def side(self) -> str:
        """``"leader"`` or ``"follower"``."""
        ...

    @property
    def validated(self) -> bool:
        """Whether this model was verified against the actual machine."""
        ...

    def gravity_torque_nm(
        self,
        joint_names: Sequence[str],
        positions_rad: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Gravity torque for each of ``joint_names`` at ``positions_rad``."""
        ...


@dataclass(frozen=True)
class UnvalidatedGravityProvider:
    """A provider that returns zero torque and says so.

    Use it only where the absence of compensation is acceptable and stated: a
    simulated plant, or a joint that is mechanically supported.  It reports
    ``validated = False``, so :class:`BilateralController` refuses it unless the
    caller explicitly opted into running uncompensated.

    Attributes:
        side: The machine this stands in for.
        justification: Why running without compensation is acceptable here.
    """

    side: str
    justification: str = ""

    @property
    def validated(self) -> bool:
        """Always ``False``: zero torque is not a validated gravity model."""
        return False

    def gravity_torque_nm(
        self,
        joint_names: Sequence[str],
        positions_rad: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Zero torque for every joint."""
        del positions_rad
        return np.zeros(len(joint_names), dtype=np.float64)


@dataclass
class BilateralGains:
    """Coupling gains and limits, per joint or scalar.

    Attributes:
        stiffness_nm_per_rad: ``K``.  Non-negative.
        damping_nm_s_per_rad: ``D``.  Non-negative.
        scale: ``S``, the leader-to-follower angle ratio, per joint.  ``1.0``
            means the two joints move through the same angle.
        offset_rad: ``b``, the calibrated joint offset per joint.
        max_torque_nm: Per-joint ceiling on the commanded torque magnitude.
        max_torque_rate_nm_s: Per-joint ceiling on how fast a torque command may
            change.
        leader_feedback_scale: Overall multiplier on the torque fed back to the
            leader.  Anything other than ``1.0`` breaks the energy identity
            above and is treated as a separate, separately validated feature --
            see :meth:`BilateralController.__init__`.
        velocity_filter_hz: Cut-off of the first-order low-pass applied to the
            measured velocities before differencing.  ``None`` disables it.
        ramp_time_s: Time over which the coupling gains fade in after
            :meth:`BilateralController.engage`.
    """

    stiffness_nm_per_rad: Mapping[str, float] | float = 1.0
    damping_nm_s_per_rad: Mapping[str, float] | float = 0.05
    scale: Mapping[str, float] | float = 1.0
    offset_rad: Mapping[str, float] | float = 0.0
    max_torque_nm: Mapping[str, float] | float = 0.5
    max_torque_rate_nm_s: Mapping[str, float] | float = 20.0
    leader_feedback_scale: float = 1.0
    velocity_filter_hz: float | None = 20.0
    ramp_time_s: float = 1.0


def _expand(
    value: Mapping[str, float] | float,
    joint_names: Sequence[str],
    *,
    name: str,
    non_negative: bool = False,
) -> NDArray[np.float64]:
    if isinstance(value, Mapping):
        missing = [j for j in joint_names if j not in value]
        if missing:
            raise ValueError(f"{name} is missing entries for {missing}.")
        array = np.asarray([float(value[j]) for j in joint_names], dtype=np.float64)
    else:
        array = np.full(len(joint_names), float(value), dtype=np.float64)
    if non_negative and np.any(array < 0.0):
        offenders = [joint_names[i] for i in np.flatnonzero(array < 0.0)]
        raise ValueError(f"{name} must be non-negative; negative for {offenders}.")
    return array


def coupling_stored_energy(
    position_error_rad: NDArray[np.float64],
    stiffness_nm_per_rad: NDArray[np.float64],
) -> float:
    """Energy stored in the coupling spring, ``0.5 e^T K e``.

    This is the ideal continuous-time value.  It is reported for diagnostics and
    is not evidence that a discretised, delayed, saturating loop is passive.
    """
    e = np.asarray(position_error_rad, dtype=np.float64)
    k = np.asarray(stiffness_nm_per_rad, dtype=np.float64)
    return 0.5 * float(e @ (k * e))


@dataclass
class _FilterState:
    """First-order low-pass state for one side's velocities."""

    value: NDArray[np.float64] | None = None

    def update(self, measurement: NDArray[np.float64], alpha: float | None) -> NDArray[np.float64]:
        if alpha is None:
            self.value = np.array(measurement, copy=True)
            return self.value
        if self.value is None:
            self.value = np.array(measurement, copy=True)
        else:
            self.value = self.value + alpha * (measurement - self.value)
        return self.value

    def reset(self) -> None:
        self.value = None


class BilateralController:
    """Stateful joint-space coupling between a leader and a follower arm.

    The state is explicit and resettable: the velocity filters, the previous
    torque command used by the rate limiter, and the engagement ramp.  All three
    are cleared by :meth:`reset`, which every mode change and every clutch must
    call so that a stale filter or a stale command cannot cross the boundary.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        gains: BilateralGains | None = None,
        *,
        leader_gravity: GravityProvider | None = None,
        follower_gravity: GravityProvider | None = None,
        allow_uncompensated: bool = False,
        allow_asymmetric_feedback: bool = False,
    ) -> None:
        """Configure the coupling.

        Args:
            joint_names: Coupled joints, in array order.  These are the follower
                joint names; the leader's corresponding joints are addressed by
                the same names through :meth:`compute`.
            gains: Coupling gains and limits.
            leader_gravity: Gravity model for the leader.
            follower_gravity: Gravity model for the follower.
            allow_uncompensated: Permit an unvalidated (or absent) gravity model.
                Intended for a simulated plant or a mechanically supported
                joint, and it must be an explicit choice.
            allow_asymmetric_feedback: Permit
                :attr:`BilateralGains.leader_feedback_scale` other than ``1.0``.
                Scaling one side's torque alone breaks the coupling's energy
                balance, so it is off by default.

        Raises:
            ValueError: On an empty joint list, a negative gain, a gravity model
                bound to the wrong side, or an unvalidated model without
                ``allow_uncompensated``.
        """
        if not joint_names:
            raise ValueError("A bilateral controller needs at least one coupled joint.")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("Coupled joint names must be unique.")

        self._joint_names: Tuple[str, ...] = tuple(joint_names)
        self._gains = gains or BilateralGains()

        self._K = _expand(
            self._gains.stiffness_nm_per_rad,
            self._joint_names,
            name="stiffness_nm_per_rad",
            non_negative=True,
        )
        self._D = _expand(
            self._gains.damping_nm_s_per_rad,
            self._joint_names,
            name="damping_nm_s_per_rad",
            non_negative=True,
        )
        self._S = _expand(self._gains.scale, self._joint_names, name="scale")
        self._b = _expand(self._gains.offset_rad, self._joint_names, name="offset_rad")
        self._tau_max = _expand(
            self._gains.max_torque_nm,
            self._joint_names,
            name="max_torque_nm",
            non_negative=True,
        )
        self._tau_rate_max = _expand(
            self._gains.max_torque_rate_nm_s,
            self._joint_names,
            name="max_torque_rate_nm_s",
            non_negative=True,
        )

        if self._gains.leader_feedback_scale != 1.0 and not allow_asymmetric_feedback:
            raise ValueError(
                "leader_feedback_scale != 1.0 scales one side's torque on its own, which breaks "
                "the coupling's energy balance. Adjust the feel with the coupling gains instead, "
                "or opt in with allow_asymmetric_feedback=True after validating saturation and "
                "stability for that configuration."
            )

        self._leader_gravity = self._check_gravity(leader_gravity, "leader", allow_uncompensated)
        self._follower_gravity = self._check_gravity(
            follower_gravity, "follower", allow_uncompensated
        )

        self._leader_filter = _FilterState()
        self._follower_filter = _FilterState()
        self._previous_leader_torque: NDArray[np.float64] | None = None
        self._previous_follower_torque: NDArray[np.float64] | None = None
        self._engaged_elapsed_s = 0.0
        self._engaged = False

    @staticmethod
    def _check_gravity(
        provider: GravityProvider | None,
        side: str,
        allow_uncompensated: bool,
    ) -> GravityProvider:
        if provider is None:
            if not allow_uncompensated:
                raise ValueError(
                    f"No gravity model was supplied for the {side}. Running a current-controlled "
                    "joint without compensation can drop it under its own weight. Supply a "
                    "validated provider, or pass allow_uncompensated=True for a simulated plant "
                    "or a mechanically supported joint."
                )
            return UnvalidatedGravityProvider(
                side=side, justification="no provider supplied; uncompensated run was allowed"
            )
        if provider.side != side:
            raise ValueError(
                f"A gravity model built for the '{provider.side}' was passed as the '{side}' "
                "model. The two machines have different masses and payloads; a model is not "
                "transferable between them."
            )
        if not provider.validated and not allow_uncompensated:
            raise ValueError(
                f"The {side} gravity model reports validated=False. Verify it against the actual "
                "machine, or pass allow_uncompensated=True for a simulated or supported setup."
            )
        return provider

    # -- properties ---------------------------------------------------------

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Coupled joint names in array order."""
        return self._joint_names

    @property
    def gains(self) -> BilateralGains:
        """The configured gains."""
        return self._gains

    @property
    def is_engaged(self) -> bool:
        """Whether the coupling is currently being applied."""
        return self._engaged

    @property
    def coupling_scale(self) -> float:
        """Current ramp factor in ``[0, 1]``."""
        if not self._engaged:
            return 0.0
        if self._gains.ramp_time_s <= 0.0:
            return 1.0
        return min(1.0, self._engaged_elapsed_s / self._gains.ramp_time_s)

    # -- lifecycle ----------------------------------------------------------

    def reset(self) -> None:
        """Clear every piece of internal state and disengage the coupling."""
        self._leader_filter.reset()
        self._follower_filter.reset()
        self._previous_leader_torque = None
        self._previous_follower_torque = None
        self._engaged_elapsed_s = 0.0
        self._engaged = False

    def engage(self) -> None:
        """Start applying the coupling, ramping the gains in from zero."""
        self._engaged = True
        self._engaged_elapsed_s = 0.0

    def clutch(self) -> None:
        """Release the coupling and clear the filters, keeping the calibration.

        A clutch removes the spring without changing ``b``.  Re-baselining ``b``
        is a separate, explicit act performed with the force feedback released,
        via :meth:`rebaseline_offset`.
        """
        self.reset()

    def alignment_error_rad(self, leader: JointState, follower: JointState) -> NDArray[np.float64]:
        """The position error ``e`` that engaging right now would produce.

        Use it before :meth:`engage` to check that the two machines are aligned;
        engaging with a large ``e`` would apply a large spring torque instantly.
        """
        q_m, q_s = self._extract_positions(leader, follower)
        return q_s - (self._S * q_m + self._b)

    def rebaseline_offset(self, leader: JointState, follower: JointState) -> None:
        """Redefine ``b`` so that the current poses are the zero-error pose.

        This changes the calibration, so it must only be done with the coupling
        released -- the method enforces that.

        Raises:
            RuntimeError: If the coupling is still engaged.
        """
        if self._engaged:
            raise RuntimeError(
                "Refusing to re-baseline the offset while the coupling is engaged. Release the "
                "force feedback first, so the change does not appear as a torque step."
            )
        q_m, q_s = self._extract_positions(leader, follower)
        self._b = q_s - self._S * q_m
        self._gains.offset_rad = {
            name: float(self._b[i]) for i, name in enumerate(self._joint_names)
        }

    # -- control law --------------------------------------------------------

    def compute(
        self,
        leader: JointState,
        follower: JointState,
        dt: float,
    ) -> BilateralOutput:
        """Evaluate the control law for one cycle.

        Args:
            leader: Measured leader state.  Must cover every coupled joint and
                every coupled joint must be valid.
            follower: Measured follower state, likewise.
            dt: Elapsed time since the previous call, in seconds.

        Returns:
            The commanded torques and the diagnostics for this cycle.

        Raises:
            ValueError: If ``dt`` is not positive, or a coupled joint is missing
                or invalid on either side.  A stale or partial measurement never
                produces a torque command.
        """
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}.")

        q_m, q_s = self._extract_positions(leader, follower)
        v_m_raw, v_s_raw = self._extract_velocities(leader, follower)

        alpha = self._filter_alpha(dt)
        v_m = self._leader_filter.update(v_m_raw, alpha)
        v_s = self._follower_filter.update(v_s_raw, alpha)

        if self._engaged:
            self._engaged_elapsed_s += dt

        e = q_s - (self._S * q_m + self._b)
        e_dot = v_s - self._S * v_m
        scale = self.coupling_scale
        u = scale * (self._K * e + self._D * e_dot)

        g_s = self._follower_gravity.gravity_torque_nm(self._joint_names, q_s)
        g_m = self._leader_gravity.gravity_torque_nm(self._joint_names, q_m)

        tau_s = g_s - u
        # Transposed coupling: with a diagonal S the transpose is the same
        # diagonal, but the transpose is what makes a non-diagonal S correct, so
        # it is written that way here.
        tau_m = g_m + self._gains.leader_feedback_scale * (self._S * u)

        tau_s, s_sat, s_rate = self._limit(tau_s, self._previous_follower_torque, dt)
        tau_m, m_sat, m_rate = self._limit(tau_m, self._previous_leader_torque, dt)
        self._previous_follower_torque = tau_s
        self._previous_leader_torque = tau_m

        limits = LimitFlags(
            torque_saturated={
                name: bool(s_sat[i] or m_sat[i]) for i, name in enumerate(self._joint_names)
            },
            rate_limited={
                name: bool(s_rate[i] or m_rate[i]) for i, name in enumerate(self._joint_names)
            },
            current_limited={},
        )
        return BilateralOutput(
            joint_names=self._joint_names,
            leader_torque_nm=tau_m,
            follower_torque_nm=tau_s,
            coupling_torque_nm=u,
            position_error_rad=e,
            velocity_error_rad_s=e_dot,
            coupling_scale=scale,
            limits=limits,
            feedback_active=self._engaged and scale > 0.0,
            dt_s=dt,
        )

    def stored_energy_j(self, output: BilateralOutput) -> float:
        """Ideal coupling energy for an output, ``0.5 e^T K e``.

        See the module docstring for what this does and does not imply.
        """
        return coupling_stored_energy(output.position_error_rad, self._K)

    # -- helpers ------------------------------------------------------------

    def _filter_alpha(self, dt: float) -> float | None:
        cutoff = self._gains.velocity_filter_hz
        if cutoff is None or cutoff <= 0.0:
            return None
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return float(dt / (tau + dt))

    def _extract_positions(
        self, leader: JointState, follower: JointState
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        self._require_valid(leader, "leader")
        self._require_valid(follower, "follower")
        q_m = np.asarray(
            [leader.position_rad[leader.index(n)] for n in self._joint_names], dtype=np.float64
        )
        q_s = np.asarray(
            [follower.position_rad[follower.index(n)] for n in self._joint_names],
            dtype=np.float64,
        )
        return q_m, q_s

    def _extract_velocities(
        self, leader: JointState, follower: JointState
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        v_m = np.asarray(
            [leader.velocity_rad_s[leader.index(n)] for n in self._joint_names], dtype=np.float64
        )
        v_s = np.asarray(
            [follower.velocity_rad_s[follower.index(n)] for n in self._joint_names],
            dtype=np.float64,
        )
        return v_m, v_s

    def _require_valid(self, state: JointState, side: str) -> None:
        missing = [n for n in self._joint_names if n not in state.joint_names]
        if missing:
            raise ValueError(f"The {side} state does not cover coupled joint(s) {missing}.")
        invalid = [n for n in self._joint_names if not state.valid[state.index(n)]]
        if invalid:
            raise ValueError(
                f"The {side} state is invalid for joint(s) {invalid}; no torque is produced from "
                "a partial measurement."
            )

    def _limit(
        self,
        torque: NDArray[np.float64],
        previous: NDArray[np.float64] | None,
        dt: float,
    ) -> Tuple[NDArray[np.float64], NDArray[np.bool_], NDArray[np.bool_]]:
        rate_limited = np.zeros(len(self._joint_names), dtype=bool)
        if previous is not None:
            max_delta = self._tau_rate_max * dt
            delta = np.clip(torque - previous, -max_delta, max_delta)
            rate_limited = np.abs(torque - previous) > max_delta + 1e-12
            torque = previous + delta
        saturated = np.abs(torque) > self._tau_max
        torque = np.clip(torque, -self._tau_max, self._tau_max)
        return torque, saturated, rate_limited
