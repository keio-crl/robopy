"""Kinematic chain with forward kinematics and numerical Jacobian."""

from dataclasses import dataclass
from typing import List

import numpy as np
from numpy.typing import NDArray

from .transforms import pose_from_matrix, rot_x, rot_y, rot_z

_ROT_FUNCS = {"x": rot_x, "y": rot_y, "z": rot_z}


@dataclass(frozen=True)
class RevoluteJoint:
    """A single revolute joint in the kinematic chain.

    Attributes:
        name: Joint name (e.g., "shoulder_pan").
        parent_to_joint: 4x4 static transform from parent frame to this joint's
            rotation center (before any rotation is applied).
        axis: Rotation axis as a string: "x", "y", or "z".
        lower_limit_rad: Minimum joint angle in radians.
        upper_limit_rad: Maximum joint angle in radians.
    """

    name: str
    parent_to_joint: NDArray[np.float64]
    axis: str
    lower_limit_rad: float
    upper_limit_rad: float

    def __post_init__(self) -> None:
        if self.axis not in _ROT_FUNCS:
            raise ValueError(f"axis must be 'x', 'y', or 'z', got '{self.axis}'")


class KinematicChain:
    """Forward kinematics and numerical Jacobian for a serial revolute chain.

    The chain is a sequence of :class:`RevoluteJoint` objects followed by a
    fixed transform from the last joint frame to the end-effector frame.
    """

    def __init__(
        self,
        joints: List[RevoluteJoint],
        ee_fixed_transform: NDArray[np.float64],
    ) -> None:
        self._joints = list(joints)
        self._ee_fixed = np.array(ee_fixed_transform, dtype=np.float64)
        self._n_joints = len(self._joints)

    @property
    def n_joints(self) -> int:
        return self._n_joints

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self._joints]

    @property
    def lower_limits_rad(self) -> NDArray[np.float64]:
        return np.array([j.lower_limit_rad for j in self._joints])

    @property
    def upper_limits_rad(self) -> NDArray[np.float64]:
        return np.array([j.upper_limit_rad for j in self._joints])

    def forward_kinematics_matrix(
        self, joint_angles_rad: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Compute the 4x4 homogeneous transform from base to end-effector.

        Args:
            joint_angles_rad: (n_joints,) array of joint angles in radians.

        Returns:
            4x4 homogeneous transformation matrix.
        """
        if len(joint_angles_rad) != self._n_joints:
            raise ValueError(
                f"Expected {self._n_joints} joint angles, got {len(joint_angles_rad)}"
            )

        T = np.eye(4)
        for i, joint in enumerate(self._joints):
            rot_fn = _ROT_FUNCS[joint.axis]
            T = T @ joint.parent_to_joint @ rot_fn(float(joint_angles_rad[i]))
        T = T @ self._ee_fixed
        return T

    def forward_kinematics(
        self, joint_angles_rad: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Compute end-effector pose as (x, y, z, pitch, roll).

        Args:
            joint_angles_rad: (n_joints,) array of joint angles in radians.

        Returns:
            (5,) array [x, y, z, pitch, roll].
        """
        T = self.forward_kinematics_matrix(joint_angles_rad)
        return pose_from_matrix(T)

    def jacobian(
        self, joint_angles_rad: NDArray[np.float64], delta: float = 1e-6
    ) -> NDArray[np.float64]:
        """Numerical Jacobian of the 5-DOF pose w.r.t. joint angles.

        Uses central finite differences for O(delta^2) accuracy.

        Returns:
            (5, n_joints) Jacobian matrix.
        """
        J = np.zeros((5, self._n_joints))

        for i in range(self._n_joints):
            q_plus = joint_angles_rad.copy()
            q_minus = joint_angles_rad.copy()
            q_plus[i] += delta
            q_minus[i] -= delta

            pose_plus = self.forward_kinematics(q_plus)
            pose_minus = self.forward_kinematics(q_minus)

            diff = pose_plus - pose_minus
            # Wrap angular components to [-pi, pi]
            for k in (3, 4):
                diff[k] = (diff[k] + np.pi) % (2 * np.pi) - np.pi

            J[:, i] = diff / (2 * delta)

        return J

    def clamp_to_limits(
        self, joint_angles_rad: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Clamp joint angles to their limits."""
        return np.clip(joint_angles_rad, self.lower_limits_rad, self.upper_limits_rad)
