"""Keep a gripper's approach axis pointing along a chosen direction.

A full pose task pins three rotational degrees of freedom; a position task
pins none.  Grasping usually wants the thing in between: the approach axis of
the gripper must point at the object, while turning *about* that axis is free.
That is a task on the sphere :math:`S^2`, two degrees of freedom, and it is
written here as such -- never as "zero one Euler component", which is neither
continuous nor a direction.

Conventions match Pink's so the task can be summed with its
:class:`pink.tasks.FrameTask`: the contribution to the QP objective is
:math:`\\tfrac{1}{2}\\|J\\,\\Delta q + \\alpha e\\|_W^2`, so the step the task
asks for is :math:`J\\,\\Delta q = -\\alpha e`.

For the current world direction of the axis :math:`a` and the target direction
:math:`d`, the rotation taking :math:`a` onto :math:`d` is about
:math:`n = a \\times d / \\|a \\times d\\|` by the angle
:math:`\\theta = \\mathrm{atan2}(\\|a \\times d\\|, a \\cdot d)`.  With the
frame's angular velocity :math:`\\omega` (world), :math:`\\dot a = \\omega
\\times a`, and only the part of :math:`\\omega` perpendicular to :math:`a`
moves it, so the task Jacobian is the projection :math:`(I - a a^T) J_\\omega`
and the error is :math:`e = -\\theta n`.  Both live in the tangent plane at
:math:`a`; the third row is zero, which the QP does not mind.

Anti-parallel is the degenerate point: :math:`a \\times d` vanishes while the
angle is :math:`\\pi`.  Any axis perpendicular to :math:`a` will do to leave it,
and the one chosen is the perpendicular closest to a fixed world axis, so the
choice is deterministic and the error norm (the angle) stays continuous.
"""

from __future__ import annotations

from typing import Any, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

__all__ = ["AxisAlignmentTask", "axis_rotation_error"]


def axis_rotation_error(
    current: NDArray[np.float64], target: NDArray[np.float64]
) -> Tuple[NDArray[np.float64], float]:
    """Rotation vector ``theta * n`` taking unit vector ``current`` onto ``target``.

    Returns:
        ``(rotation_vector, angle)``; the angle is in ``[0, pi]``.
    """
    a = np.asarray(current, dtype=np.float64)
    d = np.asarray(target, dtype=np.float64)
    a = a / np.linalg.norm(a)
    d = d / np.linalg.norm(d)
    cross = np.cross(a, d)
    sin = float(np.linalg.norm(cross))
    cos = float(np.clip(a @ d, -1.0, 1.0))
    angle = float(np.arctan2(sin, cos))
    if sin > 1e-9:
        return cross / sin * angle, angle
    if cos > 0.0:
        return np.zeros(3), 0.0
    # Anti-parallel: pick the perpendicular to ``a`` nearest the world axis
    # least aligned with it, so the choice is deterministic near the pole.
    basis = np.eye(3)[int(np.argmin(np.abs(a)))]
    n = np.cross(a, basis)
    n /= np.linalg.norm(n)
    return n * angle, angle


class AxisAlignmentTask:
    """Point a frame's body axis along a world direction, free about that axis."""

    def __init__(
        self,
        model: Any,
        frame: str,
        axis_in_frame: Sequence[float],
        *,
        cost: float,
        gain: float = 1.0,
        lm_damping: float = 0.0,
    ) -> None:
        """Bind the task to a frame axis.

        Args:
            model: A :class:`~robopy.kinematics.urdf_model.WholeBodyModel`.
            frame: Frame whose axis is aligned (the TCP).
            axis_in_frame: The axis in the frame's own coordinates, e.g. the
                gripper's approach direction.  Stated, never assumed.
            cost: Task weight, in ``[cost]/rad``.
            gain: Error correction per step, ``alpha``.
            lm_damping: Levenberg-Marquardt damping, as in Pink.
        """
        axis = np.asarray(axis_in_frame, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            raise ValueError("axis_in_frame must be a non-zero vector.")
        if cost < 0.0:
            raise ValueError("cost must be non-negative.")
        self._model = model
        self.frame = frame
        self.axis_in_frame = axis / norm
        self.cost = float(cost)
        self.gain = float(gain)
        self.lm_damping = float(lm_damping)
        self._target: NDArray[np.float64] | None = None

    def set_target_direction(self, direction_world: Sequence[float]) -> None:
        """Set the world direction the axis should point along."""
        d = np.asarray(direction_world, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(d))
        if norm < 1e-9:
            raise ValueError("The target direction must be a non-zero vector.")
        self._target = d / norm

    def set_target_from_pose(self, T_world: NDArray[np.float64]) -> None:
        """Take the direction from a target pose: the frame's axis as that pose would carry it."""
        R = np.asarray(T_world, dtype=np.float64)[:3, :3]
        self.set_target_direction([float(v) for v in R @ self.axis_in_frame])

    @property
    def target_direction(self) -> NDArray[np.float64] | None:
        """The target direction, if set."""
        return None if self._target is None else self._target.copy()

    def current_axis(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """The axis in world coordinates at ``q``."""
        R = self._model.frame_pose(q, self.frame)[:3, :3]
        return np.asarray(R @ self.axis_in_frame, dtype=np.float64)

    def compute_error(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """Task error ``e = -theta n`` (world), so that ``J dq = -alpha e`` reduces the angle."""
        if self._target is None:
            raise RuntimeError(f"no target direction set for axis task on '{self.frame}'")
        rotation, _ = axis_rotation_error(self.current_axis(q), self._target)
        return -rotation

    def angle_error(self, q: NDArray[np.float64]) -> float:
        """The angle between the axis and its target, radians."""
        if self._target is None:
            raise RuntimeError(f"no target direction set for axis task on '{self.frame}'")
        _, angle = axis_rotation_error(self.current_axis(q), self._target)
        return angle

    def compute_jacobian(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """``(3, nv)`` Jacobian: the frame's world angular velocity, projected off the axis."""
        J = self._model.frame_jacobian(q, self.frame, local=False)[3:, :]
        a = self.current_axis(q)
        projector = np.eye(3) - np.outer(a, a)
        return np.asarray(projector @ J, dtype=np.float64)

    def compute_qp_objective(
        self, q: NDArray[np.float64]
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(H, c)`` such that the task adds ``1/2 dq^T H dq + c^T dq`` to the QP."""
        J = self.compute_jacobian(q)
        minus_gain_error = -self.gain * self.compute_error(q)
        weighted_J = self.cost * J
        weighted_error = self.cost * minus_gain_error
        mu = self.lm_damping * float(weighted_error @ weighted_error)
        H = weighted_J.T @ weighted_J + mu * np.eye(J.shape[1])
        c = -weighted_error @ weighted_J
        return H, c

    def weighted_rows(
        self, q: NDArray[np.float64]
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(A, b)`` with the task residual ``A dq + b`` in cost units (for the hierarchy)."""
        J = self.compute_jacobian(q)
        return self.cost * J, self.cost * self.gain * self.compute_error(q)
