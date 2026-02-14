"""Homogeneous transformation utilities using only numpy."""

import numpy as np
from numpy.typing import NDArray


def rot_x(theta: float) -> NDArray[np.float64]:
    """4x4 rotation matrix about X-axis. theta in radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_y(theta: float) -> NDArray[np.float64]:
    """4x4 rotation matrix about Y-axis. theta in radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_z(theta: float) -> NDArray[np.float64]:
    """4x4 rotation matrix about Z-axis. theta in radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def translation(x: float, y: float, z: float) -> NDArray[np.float64]:
    """4x4 pure translation matrix."""
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def urdf_transform(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> NDArray[np.float64]:
    """Create a 4x4 transform from URDF joint origin (xyz, rpy).

    The URDF convention is ``T = Translation(xyz) @ Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
    """
    x, y, z = xyz
    roll, pitch, yaw = rpy
    return translation(x, y, z) @ rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def pose_from_matrix(T: NDArray[np.float64]) -> NDArray[np.float64]:
    """Extract (x, y, z, pitch, roll) from a 4x4 homogeneous matrix.

    Convention (Y-X intrinsic / extrinsic ZYX subset):
        pitch = atan2(-R[2,0], sqrt(R[0,0]^2 + R[1,0]^2))  -- rotation about Y
        roll  = atan2(R[2,1], R[2,2])                        -- rotation about X

    Yaw is omitted because the 5-DOF arm cannot independently control it.
    """
    x, y_val, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    R = T[:3, :3]
    pitch = float(np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)))
    roll = float(np.arctan2(R[2, 1], R[2, 2]))
    return np.array([x, y_val, z, pitch, roll])
