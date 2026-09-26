"""Frame conventions between WebXR and the robot base, and the operator frame.

WebXR reference spaces are right-handed with **+Y up, -Z forward, +X right**
(``local-floor`` puts the origin on the floor under the headset at session
start).  The robot base used throughout :mod:`robopy.control` is right-handed
with **+Z up, +X forward, +Y left**.  The change of basis is the constant
rotation :data:`XR_TO_ROBOT`::

    robot_x = -xr_z        (forward)
    robot_y = -xr_x        (left)
    robot_z =  xr_y        (up)

A pose is converted by conjugation, ``T_robot = C T_xr C^T`` with ``C`` the
4x4 embedding of :data:`XR_TO_ROBOT`, so that a body's *local* axes are
re-expressed too: the headset's own forward (local -Z in WebXR) becomes local
+X, and every function here that talks about "forward" means the local +X
column of a rotation matrix.

Nothing here knows where the operator stands relative to the robot; that is
:class:`OperatorFrame`, which is re-centred by the operator (yaw only -- the
floor is the floor on both sides, and pitch/roll of the reference would be a
mistake, not a calibration).
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.control.types import quat_xyzw_to_matrix, se3_from_quat_xyzw

__all__ = [
    "XR_TO_ROBOT",
    "OperatorFrame",
    "axis_angle_of",
    "azimuth_elevation",
    "matrix_to_quat_xyzw",
    "rotation_z",
    "wrap_to_pi",
    "xr_pose_to_robot",
    "xr_rotation_to_robot",
    "xr_vector_to_robot",
    "yaw_pitch_of_forward",
]

#: Rotation taking WebXR world coordinates to robot base coordinates.
XR_TO_ROBOT: NDArray[np.float64] = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)
_C4 = np.eye(4)
_C4[:3, :3] = XR_TO_ROBOT


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle into ``(-pi, pi]``."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def rotation_z(yaw: float) -> NDArray[np.float64]:
    """Rotation about +Z by ``yaw`` radians (positive turns +X towards +Y)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def xr_vector_to_robot(v_xr: Sequence[float]) -> NDArray[np.float64]:
    """Re-express a free vector (position, velocity, axis) in robot coordinates."""
    v = np.asarray(v_xr, dtype=np.float64)
    if v.shape != (3,):
        raise ValueError("A WebXR vector has 3 components.")
    return XR_TO_ROBOT @ v


def xr_rotation_to_robot(q_xr_xyzw: Sequence[float]) -> NDArray[np.float64]:
    """Rotation matrix, in robot coordinates, of a WebXR orientation quaternion.

    The result's columns are the body's local axes re-expressed in the robot
    convention: column 0 is the body's forward (WebXR local -Z), column 1 its
    left (WebXR local -X) and column 2 its up (WebXR local +Y).
    """
    return XR_TO_ROBOT @ quat_xyzw_to_matrix(q_xr_xyzw) @ XR_TO_ROBOT.T


def xr_pose_to_robot(
    position_xr: Sequence[float],
    q_xr_xyzw: Sequence[float],
) -> NDArray[np.float64]:
    """Homogeneous transform, in robot coordinates, of a WebXR pose."""
    T = se3_from_quat_xyzw(position_xr, q_xr_xyzw)
    return _C4 @ T @ _C4.T


def yaw_pitch_of_forward(R: NDArray[np.float64]) -> Tuple[float, float]:
    """Yaw and pitch of a body's forward axis (column 0 of ``R``).

    Yaw is positive turning left (towards +Y); pitch is positive looking up
    (towards +Z).  Roll is ignored: a two-axis head cannot reproduce it.
    """
    f = np.asarray(R, dtype=np.float64)[:3, 0]
    return azimuth_elevation(f)


def azimuth_elevation(v: Sequence[float] | NDArray[np.float64]) -> Tuple[float, float]:
    """Azimuth (about +Z from +X, positive towards +Y) and elevation of a vector."""
    x, y, z = (float(c) for c in np.asarray(v, dtype=np.float64)[:3])
    horizontal = math.hypot(x, y)
    if horizontal < 1e-12 and abs(z) < 1e-12:
        raise ValueError("Cannot take the direction of a zero vector.")
    return math.atan2(y, x), math.atan2(z, horizontal)


def axis_angle_of(R: NDArray[np.float64]) -> Tuple[NDArray[np.float64], float]:
    """Unit axis and angle in ``[0, pi]`` of a rotation matrix.

    Uses the trace for the angle and the antisymmetric part for the axis,
    falling back to the symmetric part near ``pi`` where the antisymmetric part
    vanishes.  Returns the +Z axis with angle 0 for the identity.
    """
    R = np.asarray(R, dtype=np.float64)
    cos_angle = min(1.0, max(-1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    if angle < math.pi - 1e-6:
        axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        return axis / (2.0 * math.sin(angle)), angle
    # Near pi: R ~ 2 a a^T - I, so the axis is the dominant column of (R + I).
    M = R + np.eye(3)
    column = int(np.argmax(np.linalg.norm(M, axis=0)))
    axis = M[:, column]
    return axis / float(np.linalg.norm(axis)), angle


def matrix_to_quat_xyzw(R: NDArray[np.float64]) -> Tuple[float, float, float, float]:
    """Unit quaternion ``(x, y, z, w)`` of a rotation matrix (Shepperd's method)."""
    R = np.asarray(R, dtype=np.float64)
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


class OperatorFrame:
    """Where the operator faces, so that "forward" means the robot's +X.

    WebXR's reference space is oriented by where the headset pointed when the
    session started, which has nothing to do with the robot.  The operator
    re-centres by looking in the direction they want to call "the robot's
    front"; from then on every pose is rotated about +Z by minus that yaw.
    Only the yaw is captured: pitch and roll of the headset at the moment of
    re-centring are not a calibration of anything.

    The origin is also moved to the headset's floor projection at re-centring,
    so positions are expressed relative to where the operator stands, and the
    headset's height there is kept: in the operator frame the head was at
    ``(0, 0, head_height_m)`` when re-centring.  The absolute arm mapping
    (see :mod:`robopy.vr.arm_teleop`) makes that point correspond to the
    robot's head, and the page's robot twin is placed the same way.
    """

    def __init__(self) -> None:
        self._yaw_offset = 0.0
        self._origin = np.zeros(3)
        self._head_height = 0.0
        self._recentred = False

    @property
    def recentred(self) -> bool:
        """Whether :meth:`recenter` has been called at least once."""
        return self._recentred

    @property
    def yaw_offset_rad(self) -> float:
        """Yaw of the operator's forward in WebXR-converted robot coordinates."""
        return self._yaw_offset

    @property
    def origin_m(self) -> NDArray[np.float64]:
        """The operator frame's origin in WebXR-converted robot coordinates."""
        return self._origin.copy()

    @property
    def head_height_m(self) -> float:
        """Height of the headset above the floor when re-centring."""
        return self._head_height

    @property
    def head_position_m(self) -> NDArray[np.float64]:
        """Where the headset was, in the operator frame, when re-centring."""
        return np.array([0.0, 0.0, self._head_height])

    def from_operator(self, pose_operator: NDArray[np.float64]) -> NDArray[np.float64]:
        """Inverse of :meth:`to_operator`."""
        T = np.asarray(pose_operator, dtype=np.float64)
        out = np.eye(4)
        Rz = rotation_z(self._yaw_offset)
        out[:3, :3] = Rz @ T[:3, :3]
        out[:3, 3] = Rz @ T[:3, 3] + self._origin
        return out

    def recenter(self, head_pose_robot: NDArray[np.float64]) -> None:
        """Make the headset's current forward the operator's +X.

        Args:
            head_pose_robot: Headset pose already converted with
                :func:`xr_pose_to_robot` (robot axes, WebXR origin).
        """
        T = np.asarray(head_pose_robot, dtype=np.float64)
        yaw, _ = yaw_pitch_of_forward(T[:3, :3])
        self._yaw_offset = yaw
        self._origin = np.array([T[0, 3], T[1, 3], 0.0])
        self._head_height = float(T[2, 3])
        self._recentred = True

    def to_operator(self, pose_robot: NDArray[np.float64]) -> NDArray[np.float64]:
        """Express a converted WebXR pose in the operator frame."""
        T = np.asarray(pose_robot, dtype=np.float64)
        out = np.eye(4)
        Rz = rotation_z(-self._yaw_offset)
        out[:3, :3] = Rz @ T[:3, :3]
        out[:3, 3] = Rz @ (T[:3, 3] - self._origin)
        return out
