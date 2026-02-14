"""Tests for robopy.kinematics.transforms."""

import numpy as np
import pytest

from robopy.kinematics.transforms import (
    pose_from_matrix,
    rot_x,
    rot_y,
    rot_z,
    translation,
    urdf_transform,
)


class TestRotationMatrices:
    """Basic properties of rotation matrices."""

    @pytest.mark.parametrize("rot_fn", [rot_x, rot_y, rot_z])
    def test_identity_at_zero(self, rot_fn):
        np.testing.assert_allclose(rot_fn(0.0), np.eye(4), atol=1e-15)

    @pytest.mark.parametrize("rot_fn", [rot_x, rot_y, rot_z])
    def test_orthogonality(self, rot_fn):
        R = rot_fn(0.7)[:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-14)

    @pytest.mark.parametrize("rot_fn", [rot_x, rot_y, rot_z])
    def test_determinant_is_one(self, rot_fn):
        R = rot_fn(1.2)[:3, :3]
        assert abs(np.linalg.det(R) - 1.0) < 1e-14

    def test_rot_x_90(self):
        T = rot_x(np.pi / 2)
        # Y-axis → Z-axis, Z-axis → -Y-axis
        np.testing.assert_allclose(T[:3, :3] @ [0, 1, 0], [0, 0, 1], atol=1e-14)
        np.testing.assert_allclose(T[:3, :3] @ [0, 0, 1], [0, -1, 0], atol=1e-14)

    def test_rot_y_90(self):
        T = rot_y(np.pi / 2)
        # Z-axis → X-axis, X-axis → -Z-axis... actually:
        # rot_y: X→Z component positive, Z→-X
        np.testing.assert_allclose(T[:3, :3] @ [1, 0, 0], [0, 0, -1], atol=1e-14)
        np.testing.assert_allclose(T[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-14)

    def test_rot_z_90(self):
        T = rot_z(np.pi / 2)
        np.testing.assert_allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-14)
        np.testing.assert_allclose(T[:3, :3] @ [0, 1, 0], [-1, 0, 0], atol=1e-14)


class TestTranslation:
    def test_basic(self):
        T = translation(1.0, 2.0, 3.0)
        assert T.shape == (4, 4)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(T[:3, :3], np.eye(3))

    def test_zero(self):
        np.testing.assert_allclose(translation(0, 0, 0), np.eye(4))


class TestPoseFromMatrix:
    def test_identity(self):
        pose = pose_from_matrix(np.eye(4))
        np.testing.assert_allclose(pose, [0, 0, 0, 0, 0], atol=1e-14)

    def test_pure_translation(self):
        T = translation(0.1, 0.2, 0.3)
        pose = pose_from_matrix(T)
        np.testing.assert_allclose(pose[:3], [0.1, 0.2, 0.3])
        np.testing.assert_allclose(pose[3:], [0, 0], atol=1e-14)

    def test_pitch_only(self):
        # Pitch = rotation about Y-axis
        angle = 0.5
        T = rot_y(angle)
        pose = pose_from_matrix(T)
        np.testing.assert_allclose(pose[3], angle, atol=1e-14)  # pitch
        np.testing.assert_allclose(pose[4], 0.0, atol=1e-14)  # roll

    def test_roll_only(self):
        # Roll = rotation about X-axis
        angle = 0.3
        T = rot_x(angle)
        pose = pose_from_matrix(T)
        np.testing.assert_allclose(pose[3], 0.0, atol=1e-14)  # pitch
        np.testing.assert_allclose(pose[4], angle, atol=1e-14)  # roll


class TestUrdfTransform:
    """Tests for the URDF-style transform builder."""

    def test_zero_rpy_is_pure_translation(self):
        T = urdf_transform(xyz=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.0))
        np.testing.assert_allclose(T, translation(1.0, 2.0, 3.0), atol=1e-14)

    def test_zero_xyz_is_pure_rotation(self):
        T = urdf_transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, np.pi / 2, 0.0))
        expected = rot_y(np.pi / 2)
        np.testing.assert_allclose(T, expected, atol=1e-14)

    def test_rpy_order_is_zyx(self):
        """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
        roll, pitch, yaw = 0.1, 0.2, 0.3
        T = urdf_transform(xyz=(0.0, 0.0, 0.0), rpy=(roll, pitch, yaw))
        R_expected = (rot_z(yaw) @ rot_y(pitch) @ rot_x(roll))[:3, :3]
        np.testing.assert_allclose(T[:3, :3], R_expected, atol=1e-14)

    def test_identity(self):
        T = urdf_transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0))
        np.testing.assert_allclose(T, np.eye(4), atol=1e-14)
