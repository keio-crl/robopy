"""Tests for robopy.kinematics.ik_solver."""

import numpy as np

from robopy.kinematics.chain import KinematicChain, RevoluteJoint
from robopy.kinematics.ik_solver import IKConfig, IKResult, IKSolver
from robopy.kinematics.robot_chains import koch_chain, so101_chain
from robopy.kinematics.transforms import translation


def _simple_3dof_chain() -> KinematicChain:
    """3-DOF chain with X/Y/Z rotation — enough to reach 3D positions."""
    joints = [
        RevoluteJoint(
            name="j0",
            parent_to_joint=translation(0.0, 0.0, 0.05),
            axis="y",
            lower_limit_rad=-np.pi,
            upper_limit_rad=np.pi,
        ),
        RevoluteJoint(
            name="j1",
            parent_to_joint=translation(0.0, 0.0, 0.0),
            axis="x",
            lower_limit_rad=-np.pi / 2,
            upper_limit_rad=np.pi / 2,
        ),
        RevoluteJoint(
            name="j2",
            parent_to_joint=translation(-0.15, 0.0, 0.0),
            axis="x",
            lower_limit_rad=-np.pi / 2,
            upper_limit_rad=np.pi / 2,
        ),
    ]
    ee = translation(-0.1, 0.0, 0.0)
    return KinematicChain(joints=joints, ee_fixed_transform=ee)


class TestIKSolverSimple:
    """IK solver tests with a simple 3-DOF chain."""

    def test_solve_at_home(self):
        """IK at the home (zero-angle) pose should converge trivially."""
        chain = _simple_3dof_chain()
        solver = IKSolver(chain)
        home_pose = chain.forward_kinematics(np.zeros(3))
        result = solver.solve(home_pose, np.zeros(3))
        assert result.success
        assert result.position_error < 1e-4

    def test_roundtrip(self):
        """FK → IK → FK should recover the original pose."""
        chain = _simple_3dof_chain()
        solver = IKSolver(chain, IKConfig(max_iterations=200))

        rng = np.random.default_rng(42)
        for _ in range(5):
            q_rand = rng.uniform(
                chain.lower_limits_rad * 0.5,
                chain.upper_limits_rad * 0.5,
            )
            target_pose = chain.forward_kinematics(q_rand)
            # Use a slightly perturbed starting point
            q_init = q_rand + rng.normal(0, 0.1, size=3)
            q_init = chain.clamp_to_limits(q_init)

            result = solver.solve(target_pose, q_init)
            recovered_pose = chain.forward_kinematics(result.joint_angles_rad)

            np.testing.assert_allclose(
                recovered_pose[:3],
                target_pose[:3],
                atol=1e-3,
                err_msg="Position mismatch in FK→IK→FK roundtrip",
            )

    def test_result_fields(self):
        chain = _simple_3dof_chain()
        solver = IKSolver(chain)
        home_pose = chain.forward_kinematics(np.zeros(3))
        result = solver.solve(home_pose, np.zeros(3))
        assert isinstance(result, IKResult)
        assert isinstance(result.success, bool)
        assert isinstance(result.iterations, int)
        assert result.iterations >= 1


class TestIKSolverSO101:
    """IK tests using the real SO-101 chain definition."""

    def test_home_pose_roundtrip(self):
        chain = so101_chain()
        solver = IKSolver(chain)
        home_pose = chain.forward_kinematics(np.zeros(5))
        result = solver.solve(home_pose, np.zeros(5))
        assert result.success
        assert result.position_error < 1e-3

    def test_random_roundtrip(self):
        chain = so101_chain()
        solver = IKSolver(chain, IKConfig(max_iterations=200))
        rng = np.random.default_rng(123)

        for _ in range(5):
            q_rand = rng.uniform(
                chain.lower_limits_rad * 0.3,
                chain.upper_limits_rad * 0.3,
            )
            target_pose = chain.forward_kinematics(q_rand)
            q_init = q_rand + rng.normal(0, 0.05, size=5)
            q_init = chain.clamp_to_limits(q_init)

            result = solver.solve(target_pose, q_init)
            recovered_pose = chain.forward_kinematics(result.joint_angles_rad)

            np.testing.assert_allclose(
                recovered_pose[:3],
                target_pose[:3],
                atol=5e-3,
                err_msg="SO-101 FK→IK→FK position mismatch",
            )

    def test_joint_limits_respected(self):
        chain = so101_chain()
        config = IKConfig(joint_limit_margin_rad=0.02)
        solver = IKSolver(chain, config)

        home_pose = chain.forward_kinematics(np.zeros(5))
        result = solver.solve(home_pose, np.zeros(5))

        lower = chain.lower_limits_rad + config.joint_limit_margin_rad
        upper = chain.upper_limits_rad - config.joint_limit_margin_rad
        assert np.all(result.joint_angles_rad >= lower - 1e-10)
        assert np.all(result.joint_angles_rad <= upper + 1e-10)


class TestIKSolverKoch:
    """IK tests using the Koch chain definition (estimated parameters)."""

    def test_home_pose_roundtrip(self):
        chain = koch_chain()
        solver = IKSolver(chain)
        home_pose = chain.forward_kinematics(np.zeros(5))
        result = solver.solve(home_pose, np.zeros(5))
        assert result.success
        assert result.position_error < 1e-3

    def test_random_roundtrip(self):
        chain = koch_chain()
        solver = IKSolver(chain, IKConfig(max_iterations=200))
        rng = np.random.default_rng(456)

        for _ in range(5):
            q_rand = rng.uniform(
                chain.lower_limits_rad * 0.3,
                chain.upper_limits_rad * 0.3,
            )
            target_pose = chain.forward_kinematics(q_rand)
            q_init = q_rand + rng.normal(0, 0.05, size=5)
            q_init = chain.clamp_to_limits(q_init)

            result = solver.solve(target_pose, q_init)
            recovered_pose = chain.forward_kinematics(result.joint_angles_rad)

            np.testing.assert_allclose(
                recovered_pose[:3],
                target_pose[:3],
                atol=5e-3,
                err_msg="Koch FK→IK→FK position mismatch",
            )
