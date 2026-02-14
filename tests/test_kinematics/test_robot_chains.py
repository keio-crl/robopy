"""Smoke tests for robot-specific kinematic chain factories."""

import numpy as np

from robopy.kinematics.ee_pose import EEPose
from robopy.kinematics.robot_chains import koch_chain, so101_chain


class TestSO101Chain:
    def test_creation(self):
        chain = so101_chain()
        assert chain.n_joints == 5
        assert chain.joint_names == [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        ]

    def test_fk_home_position(self):
        """At zero angles the EE should be at the sum of all offsets."""
        chain = so101_chain()
        T = chain.forward_kinematics_matrix(np.zeros(5))
        # The EE should be at a reasonable position (non-zero, within arm reach)
        pos = T[:3, 3]
        reach = np.linalg.norm(pos)
        assert 0.01 < reach < 0.6, f"Unexpected reach {reach} m at home"

    def test_fk_home_position_values(self):
        """Home position must match the URDF FK (arm extended forward/up)."""
        chain = so101_chain()
        T = chain.forward_kinematics_matrix(np.zeros(5))
        pos = T[:3, 3]
        # Expected from URDF: arm extended forward in +X, raised in +Z.
        np.testing.assert_allclose(pos[0], 0.391, atol=0.005)  # ~0.39 m forward
        assert pos[0] > 0.3, "Home x must be positive (arm forward)"
        np.testing.assert_allclose(pos[1], 0.0, atol=0.005)    # centred in Y
        np.testing.assert_allclose(pos[2], 0.227, atol=0.005)  # ~0.23 m high

    def test_fk_home_all_axes_z(self):
        """All joints in the SO-101 URDF rotate about local Z."""
        chain = so101_chain()
        for j in chain._joints:
            assert j.axis == "z", f"Joint {j.name} axis should be 'z', got '{j.axis}'"

    def test_shoulder_pan_swings_y(self):
        """Shoulder pan should swing the EE left/right (±Y)."""
        chain = so101_chain()
        home = chain.forward_kinematics(np.zeros(5))
        q_pos = np.array([0.3, 0, 0, 0, 0])
        q_neg = np.array([-0.3, 0, 0, 0, 0])
        pose_pos = chain.forward_kinematics(q_pos)
        pose_neg = chain.forward_kinematics(q_neg)
        # +pan → -Y, -pan → +Y (URDF convention)
        assert pose_pos[1] < home[1], "Positive pan should move EE in -Y"
        assert pose_neg[1] > home[1], "Negative pan should move EE in +Y"
        # X/Z should change less than Y
        assert abs(pose_pos[1] - home[1]) > abs(pose_pos[0] - home[0])

    def test_shoulder_lift_moves_z(self):
        """Shoulder lift should move the EE up/down (±Z)."""
        chain = so101_chain()
        home = chain.forward_kinematics(np.zeros(5))
        q_neg = np.array([0, -0.3, 0, 0, 0])
        pose_neg = chain.forward_kinematics(q_neg)
        # Negative lift → higher Z (arm lifts)
        assert pose_neg[2] > home[2], "Negative lift should raise EE"

    def test_fk_returns_ee_pose(self):
        chain = so101_chain()
        pose = chain.forward_kinematics(np.zeros(5))
        assert pose.shape == (5,)
        ee = EEPose.from_array(pose)
        assert isinstance(ee, EEPose)

    def test_joint_limits(self):
        chain = so101_chain()
        lower = chain.lower_limits_rad
        upper = chain.upper_limits_rad
        assert len(lower) == 5
        assert len(upper) == 5
        assert np.all(lower < upper)


class TestKochChain:
    def test_creation(self):
        chain = koch_chain()
        assert chain.n_joints == 5
        assert chain.joint_names == [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_flex",
            "wrist_roll",
        ]

    def test_fk_home_position(self):
        chain = koch_chain()
        T = chain.forward_kinematics_matrix(np.zeros(5))
        pos = T[:3, 3]
        reach = np.linalg.norm(pos)
        assert 0.01 < reach < 0.5, f"Unexpected reach {reach} m at home"

    def test_joint_limits(self):
        chain = koch_chain()
        lower = chain.lower_limits_rad
        upper = chain.upper_limits_rad
        assert len(lower) == 5
        assert len(upper) == 5
        assert np.all(lower < upper)
