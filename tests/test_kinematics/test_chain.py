"""Tests for robopy.kinematics.chain."""

import numpy as np
import pytest

from robopy.kinematics.chain import KinematicChain, RevoluteJoint
from robopy.kinematics.transforms import translation


def _simple_2dof_chain() -> KinematicChain:
    """A simple 2-joint planar chain for testing.

    Joint layout (all rotations about Z):
      base → translate(0.1, 0, 0) → rot_z(q0) → translate(0.2, 0, 0) → rot_z(q1) → EE at (0.15, 0, 0)
    """
    joints = [
        RevoluteJoint(
            name="j0",
            parent_to_joint=translation(0.1, 0.0, 0.0),
            axis="z",
            lower_limit_rad=-np.pi,
            upper_limit_rad=np.pi,
        ),
        RevoluteJoint(
            name="j1",
            parent_to_joint=translation(0.2, 0.0, 0.0),
            axis="z",
            lower_limit_rad=-np.pi,
            upper_limit_rad=np.pi,
        ),
    ]
    ee = translation(0.15, 0.0, 0.0)
    return KinematicChain(joints=joints, ee_fixed_transform=ee)


class TestKinematicChain:
    def test_properties(self):
        chain = _simple_2dof_chain()
        assert chain.n_joints == 2
        assert chain.joint_names == ["j0", "j1"]

    def test_fk_zeros(self):
        """All joints at zero → EE at sum of offsets along X."""
        chain = _simple_2dof_chain()
        T = chain.forward_kinematics_matrix(np.array([0.0, 0.0]))
        # x = 0.1 + 0.2 + 0.15 = 0.45, y = 0, z = 0
        np.testing.assert_allclose(T[:3, 3], [0.45, 0.0, 0.0], atol=1e-14)

    def test_fk_pose_zeros(self):
        chain = _simple_2dof_chain()
        pose = chain.forward_kinematics(np.array([0.0, 0.0]))
        assert pose.shape == (5,)
        np.testing.assert_allclose(pose[:3], [0.45, 0.0, 0.0], atol=1e-14)

    def test_fk_90_first_joint(self):
        """Rotate first joint 90° about Z → second link points in +Y."""
        chain = _simple_2dof_chain()
        T = chain.forward_kinematics_matrix(np.array([np.pi / 2, 0.0]))
        # After q0=90deg: the 0.2 and 0.15 offsets now go in Y direction
        # x = 0.1, y = 0.2 + 0.15 = 0.35
        np.testing.assert_allclose(T[0, 3], 0.1, atol=1e-14)
        np.testing.assert_allclose(T[1, 3], 0.35, atol=1e-14)
        np.testing.assert_allclose(T[2, 3], 0.0, atol=1e-14)

    def test_wrong_number_of_angles(self):
        chain = _simple_2dof_chain()
        with pytest.raises(ValueError, match="Expected 2"):
            chain.forward_kinematics(np.array([0.0]))

    def test_jacobian_shape(self):
        chain = _simple_2dof_chain()
        J = chain.jacobian(np.array([0.0, 0.0]))
        assert J.shape == (5, 2)

    def test_jacobian_finite_difference_consistency(self):
        """Jacobian should be consistent with finite-difference check."""
        chain = _simple_2dof_chain()
        q = np.array([0.3, -0.5])
        J = chain.jacobian(q, delta=1e-6)

        # Verify with a larger delta for comparison
        J2 = chain.jacobian(q, delta=1e-4)
        # Should agree to within O(delta^2)
        np.testing.assert_allclose(J, J2, atol=1e-5)

    def test_clamp_to_limits(self):
        chain = _simple_2dof_chain()
        q = np.array([5.0, -5.0])
        clamped = chain.clamp_to_limits(q)
        np.testing.assert_allclose(clamped, [np.pi, -np.pi], atol=1e-14)


class TestRevoluteJoint:
    def test_invalid_axis(self):
        with pytest.raises(ValueError, match="axis must be"):
            RevoluteJoint(
                name="bad",
                parent_to_joint=np.eye(4),
                axis="w",
                lower_limit_rad=0,
                upper_limit_rad=1,
            )
