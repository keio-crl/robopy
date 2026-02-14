"""Damped least-squares inverse kinematics solver (numpy only)."""

from dataclasses import dataclass
from logging import getLogger

import numpy as np
from numpy.typing import NDArray

from .chain import KinematicChain

logger = getLogger(__name__)


@dataclass
class IKConfig:
    """Configuration for the IK solver.

    Attributes:
        max_iterations: Maximum solver iterations.
        position_tolerance: Convergence threshold for position error (meters).
        orientation_tolerance: Convergence threshold for orientation error (radians).
        damping: Damping factor lambda for DLS. Higher = more stable, less accurate.
        step_scale: Scale factor for each iteration step (0 < step_scale <= 1).
        position_weight: Weight for position (x, y, z) components in the error.
        orientation_weight: Weight for orientation (pitch, roll) components.
        joint_limit_margin_rad: Stay this far inside joint limits.
    """

    max_iterations: int = 100
    position_tolerance: float = 1e-4  # 0.1 mm
    orientation_tolerance: float = 1e-3  # ~0.06 degrees
    damping: float = 0.05
    step_scale: float = 0.5
    position_weight: float = 1.0
    orientation_weight: float = 0.1
    joint_limit_margin_rad: float = 0.01  # ~0.57 degrees


@dataclass
class IKResult:
    """Result from the IK solver.

    Attributes:
        joint_angles_rad: (n_joints,) solution joint angles in radians.
        success: Whether the solver converged within tolerance.
        iterations: Number of iterations used.
        position_error: Final Euclidean position error in meters.
        orientation_error: Final orientation error in radians.
    """

    joint_angles_rad: NDArray[np.float64]
    success: bool
    iterations: int
    position_error: float
    orientation_error: float


class IKSolver:
    """Damped Least-Squares (Levenberg-Marquardt style) IK solver.

    Uses the formulation::

        dq = J_w^T (J_w J_w^T + lambda^2 I)^{-1} e_w

    where *J_w* is the weighted Jacobian and *e_w* is the weighted task-space
    error.  The 5x5 linear system is solved with ``np.linalg.solve`` — no
    scipy required.
    """

    def __init__(self, chain: KinematicChain, config: IKConfig | None = None) -> None:
        self._chain = chain
        self._config = config or IKConfig()

    @property
    def config(self) -> IKConfig:
        return self._config

    def solve(
        self,
        target_pose: NDArray[np.float64],
        initial_angles_rad: NDArray[np.float64],
    ) -> IKResult:
        """Solve IK for a target 5-DOF pose.

        Args:
            target_pose: (5,) array [x, y, z, pitch, roll].
            initial_angles_rad: (n_joints,) initial guess in radians.

        Returns:
            IKResult with solution and convergence information.
        """
        cfg = self._config
        q = initial_angles_rad.copy().astype(np.float64)

        W = np.diag(
            [
                cfg.position_weight,
                cfg.position_weight,
                cfg.position_weight,
                cfg.orientation_weight,
                cfg.orientation_weight,
            ]
        )

        lower = self._chain.lower_limits_rad + cfg.joint_limit_margin_rad
        upper = self._chain.upper_limits_rad - cfg.joint_limit_margin_rad

        for iteration in range(cfg.max_iterations):
            current_pose = self._chain.forward_kinematics(q)
            error = target_pose - current_pose

            # Wrap angular errors to [-pi, pi]
            for k in (3, 4):
                error[k] = (error[k] + np.pi) % (2 * np.pi) - np.pi

            pos_err = float(np.linalg.norm(error[:3]))
            ori_err = float(np.linalg.norm(error[3:]))

            if pos_err < cfg.position_tolerance and ori_err < cfg.orientation_tolerance:
                return IKResult(
                    joint_angles_rad=q,
                    success=True,
                    iterations=iteration + 1,
                    position_error=pos_err,
                    orientation_error=ori_err,
                )

            # Weighted error and Jacobian
            e_w = W @ error
            J = self._chain.jacobian(q)
            J_w = W @ J  # (5, n_joints)

            # DLS: dq = J_w^T (J_w J_w^T + lambda^2 I)^{-1} e_w
            JJT = J_w @ J_w.T  # (5, 5)
            damped = JJT + (cfg.damping**2) * np.eye(5)
            y = np.linalg.solve(damped, e_w)
            dq = J_w.T @ y

            q = q + cfg.step_scale * dq
            q = np.clip(q, lower, upper)

        # Did not converge — compute final error
        current_pose = self._chain.forward_kinematics(q)
        error = target_pose - current_pose
        pos_err = float(np.linalg.norm(error[:3]))
        ori_err = float(np.linalg.norm(error[3:]))

        logger.debug(
            "IK did not converge after %d iterations. "
            "Position error: %.6f m, Orientation error: %.6f rad",
            cfg.max_iterations,
            pos_err,
            ori_err,
        )

        return IKResult(
            joint_angles_rad=q,
            success=False,
            iterations=cfg.max_iterations,
            position_error=pos_err,
            orientation_error=ori_err,
        )
