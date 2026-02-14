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
