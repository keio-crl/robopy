"""Frame math, head tracking and arm teleoperation of :mod:`robopy.vr`.

Everything here is pure numpy; the tests that derive the head mapping from a
model are skipped without the ``kinematics`` extra.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from robopy.control.types import TorsoPolicy, quat_xyzw_to_matrix
from robopy.vr.arm_teleop import (
    ArmTeleop,
    ArmTeleopConfig,
    ControllerSample,
    DualArmTeleop,
)
from robopy.vr.head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig
from robopy.vr.xr_math import (
    XR_TO_ROBOT,
    OperatorFrame,
    axis_angle_of,
    matrix_to_quat_xyzw,
    rotation_z,
    wrap_to_pi,
    xr_pose_to_robot,
    xr_rotation_to_robot,
    xr_vector_to_robot,
    yaw_pitch_of_forward,
)


def quat_about(axis: str, angle: float) -> list[float]:
    """``xyzw`` quaternion of a rotation about a WebXR world axis."""
    half = angle / 2.0
    s = math.sin(half)
    return {
        "x": [s, 0.0, 0.0, math.cos(half)],
        "y": [0.0, s, 0.0, math.cos(half)],
        "z": [0.0, 0.0, s, math.cos(half)],
    }[axis]


class TestFrameConversion:
    def test_basis_change_is_a_proper_rotation_with_the_documented_axes(self) -> None:
        assert np.linalg.det(XR_TO_ROBOT) == pytest.approx(1.0)
        assert np.allclose(XR_TO_ROBOT @ XR_TO_ROBOT.T, np.eye(3))
        # WebXR forward (-Z) is robot forward (+X); WebXR up (+Y) is robot up (+Z);
        # WebXR right (+X) is robot right (-Y).
        assert np.allclose(xr_vector_to_robot([0, 0, -1]), [1, 0, 0])
        assert np.allclose(xr_vector_to_robot([0, 1, 0]), [0, 0, 1])
        assert np.allclose(xr_vector_to_robot([1, 0, 0]), [0, -1, 0])

    def test_headset_yaw_left_is_positive_robot_yaw(self) -> None:
        # Turning left in WebXR is a positive rotation about +Y (up).
        R = xr_rotation_to_robot(quat_about("y", math.radians(30)))
        yaw, pitch = yaw_pitch_of_forward(R)
        assert yaw == pytest.approx(math.radians(30))
        assert pitch == pytest.approx(0.0)

    def test_headset_looking_up_is_positive_pitch(self) -> None:
        # Tilting the head back is a positive rotation about WebXR +X (right).
        R = xr_rotation_to_robot(quat_about("x", math.radians(20)))
        yaw, pitch = yaw_pitch_of_forward(R)
        assert yaw == pytest.approx(0.0)
        assert pitch == pytest.approx(math.radians(20))

    def test_pose_conversion_is_a_similarity_that_preserves_composition(self) -> None:
        p1, q1 = [0.1, 1.6, -0.5], quat_about("y", 0.4)
        p2, q2 = [0.3, -0.2, 0.7], quat_about("x", -0.3)
        T1, T2 = xr_pose_to_robot(p1, q1), xr_pose_to_robot(p2, q2)
        raw1 = np.eye(4)
        raw1[:3, :3] = quat_xyzw_to_matrix(q1)
        raw1[:3, 3] = p1
        raw2 = np.eye(4)
        raw2[:3, :3] = quat_xyzw_to_matrix(q2)
        raw2[:3, 3] = p2
        combined = xr_pose_to_robot(
            (raw1 @ raw2)[:3, 3], matrix_to_quat_xyzw((raw1 @ raw2)[:3, :3])
        )
        assert np.allclose(T1 @ T2, combined)
        assert np.allclose(T1[:3, 3], xr_vector_to_robot(p1))

    def test_axis_angle_and_quaternion_round_trips(self) -> None:
        rng = np.random.default_rng(0)
        for _ in range(50):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            angle = rng.uniform(1e-4, math.pi - 1e-4)
            K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
            got_axis, got_angle = axis_angle_of(R)
            assert got_angle == pytest.approx(angle, abs=1e-9)
            assert np.allclose(got_axis, axis, atol=1e-7)
            q = matrix_to_quat_xyzw(R)
            assert np.allclose(quat_xyzw_to_matrix(q), R, atol=1e-12)
        axis, angle = axis_angle_of(np.eye(3))
        assert angle == 0.0
        # A half turn about Y: the antisymmetric part vanishes.
        axis, angle = axis_angle_of(np.diag([-1.0, 1.0, -1.0]))
        assert angle == pytest.approx(math.pi)
        assert np.allclose(np.abs(axis), [0, 1, 0])

    def test_wrap_to_pi(self) -> None:
        assert wrap_to_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
        assert wrap_to_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
        assert wrap_to_pi(0.3) == pytest.approx(0.3)


class TestOperatorFrame:
    def test_recentring_makes_the_headset_forward_the_robot_x(self) -> None:
        frame = OperatorFrame()
        assert not frame.recentred
        # Operator stands at XR (1, 1.7, 2) looking along WebXR +X (i.e. yawed -90 degrees).
        head = xr_pose_to_robot([1.0, 1.7, 2.0], quat_about("y", -math.pi / 2))
        frame.recenter(head)
        assert frame.recentred
        assert frame.yaw_offset_rad == pytest.approx(-math.pi / 2)
        head_op = frame.to_operator(head)
        yaw, _ = yaw_pitch_of_forward(head_op[:3, :3])
        assert yaw == pytest.approx(0.0)
        # The origin moves to the headset's floor projection: only height remains.
        assert np.allclose(head_op[:3, 3], [0.0, 0.0, 1.7])
        # Something 1 m ahead of the operator (WebXR +X here) is at operator +X.
        ahead = xr_pose_to_robot([2.0, 1.2, 2.0], [0, 0, 0, 1])
        assert np.allclose(frame.to_operator(ahead)[:3, 3], [1.0, 0.0, 1.2])


def _mapping(yaw_sign: int = 1, pitch_sign: int = 1) -> HeadJointMapping:
    return HeadJointMapping(
        yaw_joint="head_yaw_dof",
        pitch_joint="head_pitch_dof",
        yaw_sign=yaw_sign,
        pitch_sign=pitch_sign,
        yaw_neutral_rad=0.1,
        pitch_neutral_rad=-0.05,
        yaw_limits_rad=(-1.0, 1.0),
        pitch_limits_rad=(-0.5, 0.5),
    )


def _head(yaw: float, pitch: float) -> np.ndarray:
    R = rotation_z(yaw)
    c, s = math.cos(pitch), math.sin(pitch)
    Ry = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])  # forward tilts up for pitch > 0
    return R @ Ry


class TestHeadTracker:
    def test_first_update_recentres_and_targets_follow_signs(self) -> None:
        tracker = HeadTracker(
            _mapping(yaw_sign=-1, pitch_sign=1), HeadTrackingConfig(filter_hz=None)
        )
        tracker.reset({"head_yaw_dof": 0.1, "head_pitch_dof": -0.05})
        first = tracker.update(_head(0.7, 0.1), now_s=0.0)  # arbitrary starting gaze
        assert first.recentred_now
        assert first.yaw_input_rad == pytest.approx(0.0)
        assert first.targets_rad == pytest.approx({"head_yaw_dof": 0.1, "head_pitch_dof": -0.05})
        # Turn left 0.3 and look up 0.2 relative to the reference; a generous
        # dt lets the rate limiter (2.5 rad/s) get there in one step.
        cmd = tracker.update(_head(1.0, 0.3), now_s=1.0)
        assert cmd.yaw_input_rad == pytest.approx(0.3)
        assert cmd.pitch_input_rad == pytest.approx(0.2)
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(0.1 - 0.3)
        assert cmd.targets_rad["head_pitch_dof"] == pytest.approx(-0.05 + 0.2)
        assert cmd.at_limit == ()

    def test_limits_clamp_with_margin_and_are_reported(self) -> None:
        tracker = HeadTracker(_mapping(), HeadTrackingConfig(filter_hz=None, limit_margin_rad=0.02))
        tracker.update(_head(0.0, 0.0), now_s=0.0)
        cmd = tracker.update(_head(2.0, -1.5), now_s=10.0)
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(1.0 - 0.02)
        assert cmd.targets_rad["head_pitch_dof"] == pytest.approx(-0.5 + 0.02)
        assert set(cmd.at_limit) == {"head_yaw_dof", "head_pitch_dof"}

    def test_rate_limit_and_filter_slow_a_jump(self) -> None:
        tracker = HeadTracker(_mapping(), HeadTrackingConfig(filter_hz=None, max_rate_rad_s=1.0))
        tracker.update(_head(0.0, 0.0), now_s=0.0)
        cmd = tracker.update(_head(0.8, 0.0), now_s=0.1)
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(0.1 + 0.1)  # 1 rad/s * 0.1 s
        filtered = HeadTracker(_mapping(), HeadTrackingConfig(filter_hz=2.0, max_rate_rad_s=100.0))
        filtered.update(_head(0.0, 0.0), now_s=0.0)
        cmd = filtered.update(_head(0.5, 0.0), now_s=0.05)
        alpha = 1.0 - math.exp(-2.0 * math.pi * 2.0 * 0.05)
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(0.1 + alpha * 0.5)

    def test_recenter_and_hold(self) -> None:
        tracker = HeadTracker(_mapping(), HeadTrackingConfig(filter_hz=None))
        assert tracker.hold() == {"head_yaw_dof": 0.1, "head_pitch_dof": -0.05}
        tracker.update(_head(0.0, 0.0), now_s=0.0)
        tracker.update(_head(0.4, 0.0), now_s=1.0)
        tracker.recenter(_head(0.4, 0.0))
        cmd = tracker.update(_head(0.4, 0.0), now_s=2.0)
        assert cmd.yaw_input_rad == pytest.approx(0.0)
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(0.1)
        assert tracker.hold() == cmd.targets_rad

    def test_torso_compensation_is_added_to_the_yaw_command(self) -> None:
        mapping = HeadJointMapping(
            yaw_joint="head_yaw_dof",
            pitch_joint="head_pitch_dof",
            yaw_sign=1,
            pitch_sign=1,
            yaw_neutral_rad=0.0,
            pitch_neutral_rad=0.0,
            yaw_limits_rad=(-2.0, 2.0),
            pitch_limits_rad=(-1.0, 1.0),
            torso_joint="torso_yaw_dof",
            torso_coupling=-1.0,
            torso_reference_rad=0.1,
        )
        tracker = HeadTracker(mapping, HeadTrackingConfig(filter_hz=None, max_rate_rad_s=100.0))
        tracker.update(_head(0.0, 0.0), now_s=0.0, torso_angle_rad=0.1)
        # Operator turns left 0.3 while the torso has turned left 0.5 past its
        # reference: the head must turn back by 0.5 to keep the heading.
        cmd = tracker.update(_head(0.3, 0.0), now_s=1.0, torso_angle_rad=0.6)
        assert cmd.torso_compensation_rad == pytest.approx(-0.5)
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(0.3 - 0.5)
        # Without a torso reading no compensation is applied.
        cmd = tracker.update(_head(0.3, 0.0), now_s=2.0)
        assert cmd.torso_compensation_rad == 0.0
        assert cmd.targets_rad["head_yaw_dof"] == pytest.approx(0.3)
        assert mapping.describe()["torso_coupling"] == -1.0

    def test_config_and_mapping_validation(self) -> None:
        with pytest.raises(ValueError):
            HeadTrackingConfig(max_rate_rad_s=0.0)
        with pytest.raises(ValueError):
            HeadJointMapping("a", "b", 2, 1, 0.0, 0.0, (-1, 1), (-1, 1))
        with pytest.raises(ValueError):
            HeadJointMapping("a", "b", 1, 1, 0.0, 0.0, (-1, 1), (1, -1))


class TestHeadMappingFromModel:
    @pytest.fixture
    def synthetic_model(self, synthetic_urdf: Path):  # type: ignore[no-untyped-def]
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.kinematics.urdf_model import WholeBodyModel

        return WholeBodyModel.from_urdf(synthetic_urdf)

    def test_synthetic_head_axes_give_plus_yaw_minus_pitch(self, synthetic_model) -> None:
        mapping = HeadJointMapping.from_model(
            synthetic_model, "head_yaw_dof", "head_pitch_dof", camera_frame="head_camera_link"
        )
        assert mapping.yaw_sign == 1  # axis +Z: positive turns left
        assert mapping.pitch_sign == -1  # axis +Y: positive looks down
        assert mapping.yaw_neutral_rad == pytest.approx(0.0, abs=1e-6)
        assert mapping.pitch_neutral_rad == pytest.approx(0.0, abs=1e-6)
        assert mapping.forward_source == "camera_frame:head_camera_link/x (auto)"
        assert mapping.notes == ()
        no_camera = HeadJointMapping.from_model(synthetic_model, "head_yaw_dof", "head_pitch_dof")
        assert no_camera.forward_source == "pitch_axis"
        assert any("assumed level" in n for n in no_camera.notes)
        assert no_camera.torso_joint is None and no_camera.torso_coupling == 0.0

    def test_torso_coupling_keeps_the_camera_heading_fixed(self, synthetic_model) -> None:
        mapping = HeadJointMapping.from_model(
            synthetic_model,
            "head_yaw_dof",
            "head_pitch_dof",
            camera_frame="head_camera_link",
            torso_joint="torso_yaw_dof",
        )
        # Both axes are +Z on the fixture: one radian of torso needs minus one of head.
        assert mapping.torso_joint == "torso_yaw_dof"
        assert mapping.torso_coupling == pytest.approx(-1.0, abs=1e-6)
        assert mapping.torso_reference_rad == 0.0
        for torso in (-0.7, 0.0, 0.4):
            positions = {name: 0.0 for name in synthetic_model.movable_joint_names}
            positions["torso_yaw_dof"] = torso
            positions["head_yaw_dof"] = mapping.yaw_neutral_rad + mapping.torso_compensation_rad(
                torso
            )
            R = synthetic_model.frame_pose(
                synthetic_model.q_from_positions(positions), "head_camera_link"
            )[:3, :3]
            assert np.allclose(R[:, 0], [1.0, 0.0, 0.0], atol=1e-9)  # still looking along +X
        with pytest.raises(ValueError, match="not a movable joint"):
            HeadJointMapping.from_model(
                synthetic_model, "head_yaw_dof", "head_pitch_dof", torso_joint="nope"
            )

    def test_rejects_joints_without_limits_or_wrong_geometry(self, synthetic_model) -> None:
        with pytest.raises(ValueError, match="not a movable joint"):
            HeadJointMapping.from_model(synthetic_model, "nope", "head_pitch_dof")
        # The torso yaw is continuous: no finite limit.
        with pytest.raises(ValueError, match="no finite position limit"):
            HeadJointMapping.from_model(synthetic_model, "torso_yaw_dof", "head_pitch_dof")
        synthetic_model.set_soft_limits({"torso_yaw_dof": (-1.0, 1.0)})
        # A shoulder pitch joint (horizontal axis) is not a yaw joint.
        synthetic_model.set_soft_limits({"shoulder_pitch_left_dof": (-1.0, 1.0)})
        with pytest.raises(ValueError, match="not vertical"):
            HeadJointMapping.from_model(
                synthetic_model, "shoulder_pitch_left_dof", "head_pitch_dof"
            )

    def test_real_rakuda_export_yaws_the_other_way_and_starts_turned(self) -> None:
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.kinematics.urdf_model import WholeBodyModel
        from robopy.models import find_rakuda_model

        rakuda = find_rakuda_model()
        if rakuda is None:
            pytest.skip("committed Rakuda model not found")
        model = WholeBodyModel.from_urdf(
            rakuda.convex_collision_urdf, package_dirs=rakuda.package_dirs, geometry_only=True
        )
        mapping = HeadJointMapping.from_model(
            model,
            "head_yaw_dof",
            "head_pitch_dof",
            camera_frame="head_camera_link",
            torso_joint="torso_yaw_dof",
        )
        # Both yaw axes are world -Z, so the head command is (signal - torso_yaw).
        assert mapping.torso_coupling == pytest.approx(-1.0, abs=1e-6)
        # Measured on the export: yaw axis is world -Z, the URDF zero has the
        # head turned ~22 degrees right and the camera looking ~6 degrees up.
        assert mapping.yaw_sign == -1
        assert mapping.pitch_sign == 1
        assert mapping.yaw_neutral_rad == pytest.approx(-0.381, abs=2e-3)
        assert mapping.pitch_neutral_rad == pytest.approx(-0.104, abs=2e-3)
        assert mapping.forward_source == "camera_frame:head_camera_link/z (auto)"
        positions = {name: 0.0 for name in model.movable_joint_names}
        positions.update(mapping.neutral_positions())
        R_cam = model.frame_pose(model.q_from_positions(positions), "head_camera_link")[:3, :3]
        assert np.allclose(R_cam[:, 2], [1.0, 0.0, 0.0], atol=1e-6)
        # ... and it keeps looking along +X when the torso turns and the head compensates.
        positions["torso_yaw_dof"] = 0.5
        positions["head_yaw_dof"] = mapping.yaw_neutral_rad + mapping.torso_compensation_rad(0.5)
        R_cam = model.frame_pose(model.q_from_positions(positions), "head_camera_link")[:3, :3]
        assert np.allclose(R_cam[:, 2], [1.0, 0.0, 0.0], atol=1e-6)


def _sample(
    x: float, y: float, z: float, *, clutch: bool, R: np.ndarray | None = None
) -> ControllerSample:
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    if R is not None:
        T[:3, :3] = R
    return ControllerSample(pose=T, clutch=clutch)


RELATIVE = ArmTeleopConfig(mapping="relative")


class TestArmTeleop:
    def test_clutch_engages_moves_relatively_and_releases_into_a_hold(self) -> None:
        arm = ArmTeleop(
            "left", ArmTeleopConfig(mapping="relative", position_scale=2.0, max_speed_m_s=100.0)
        )
        hand = np.eye(4)
        hand[:3, 3] = [0.3, 0.2, 0.1]
        idle = arm.update(_sample(0, 0, 1.2, clutch=False), hand, 0.0)
        assert not idle.enabled and idle.target is None and not idle.clutched
        engaged = arm.update(_sample(0, 0, 1.2, clutch=True), hand, 0.01)
        assert engaged.engaged_now and engaged.clutched and engaged.enabled
        assert np.allclose(engaged.target, hand)
        moved = arm.update(_sample(0.05, 0.0, 1.2, clutch=True), hand, 0.02)
        assert np.allclose(moved.target[:3, 3], [0.3 + 0.10, 0.2, 0.1])  # scale 2
        assert np.allclose(moved.target[:3, :3], np.eye(3))
        released = arm.update(_sample(0.05, 0.0, 1.2, clutch=False), hand, 0.03)
        assert released.released_now and not released.enabled and not released.clutched
        assert np.allclose(released.target[:3, 3], [0.40, 0.2, 0.1])  # latched
        # Re-engaging anchors to the *current* hand pose, not the old target.
        hand2 = np.eye(4)
        hand2[:3, 3] = [0.0, 0.0, 0.5]
        again = arm.update(_sample(0.0, 0.0, 1.2, clutch=True), hand2, 0.04)
        assert np.allclose(again.target, hand2)

    def test_engage_gate_waits_until_the_hand_is_at_the_robot_hand(self) -> None:
        arm = ArmTeleop(
            "left", ArmTeleopConfig(mapping="absolute", engage_radius_m=0.05, max_speed_m_s=100.0)
        )
        arm.set_anchor([0.0, 0.0, 1.0], [0.0, 0.0, 1.6])
        hand = np.eye(4)
        hand[:3, 3] = [0.3, 0.2, 0.7]  # 0.3 ahead, 0.2 left, 0.3 below the anchor
        # The controller 20 cm above the corresponding spot: pressing waits.
        far = arm.update(_sample(0.3, 0.2, 1.6 - 0.3 + 0.2, clutch=True), hand, 0.0)
        assert far.waiting and not far.clutched and not far.enabled and far.target is None
        assert far.engage_distance_m == pytest.approx(0.2)
        assert far.engage_radius_m == 0.05
        # Not pressing: the distance is still reported, nothing waits.
        idle = arm.update(_sample(0.3, 0.2, 1.6 - 0.3 + 0.2, clutch=False), hand, 0.1)
        assert not idle.waiting and idle.engage_distance_m == pytest.approx(0.2)
        # Within the radius: engages, anchored at the current hand.
        near = arm.update(_sample(0.3, 0.2, 1.6 - 0.3 + 0.03, clutch=True), hand, 0.2)
        assert near.engaged_now and near.clutched and not near.waiting
        assert near.engage_distance_m is None
        assert near.target is not None and np.allclose(near.target, hand)
        # A sample may carry its own radius, which wins over the arm's.
        arm.release()
        near_pose = np.eye(4)
        near_pose[:3, 3] = [0.3, 0.2, 1.6 - 0.3 + 0.03]
        strict = ControllerSample(pose=near_pose, clutch=True, engage_radius_m=0.01)
        assert arm.update(strict, hand, 0.3).waiting
        # The relative mapping never gates.
        relative = ArmTeleop("left", ArmTeleopConfig(mapping="relative", engage_radius_m=0.05))
        assert relative.update(_sample(5.0, 5.0, 5.0, clutch=True), hand, 0.0).clutched
        with pytest.raises(ValueError, match="engage_radius_m"):
            ArmTeleopConfig(engage_radius_m=0.0)

    def test_untracked_controller_releases_the_clutch(self) -> None:
        arm = ArmTeleop("right", RELATIVE)
        hand = np.eye(4)
        arm.update(_sample(0, 0, 1, clutch=True), hand, 0.0)
        lost = arm.update(ControllerSample(pose=None, clutch=True), hand, 0.1)
        assert lost.released_now and not lost.clutched and not lost.tracked
        gone = arm.update(None, hand, 0.2)
        assert not gone.clutched and not gone.tracked

    def test_speed_limit_slews_a_jump(self) -> None:
        arm = ArmTeleop("left", ArmTeleopConfig(mapping="relative", max_speed_m_s=0.5))
        hand = np.eye(4)
        arm.update(_sample(0, 0, 1, clutch=True), hand, 0.0)
        cmd = arm.update(_sample(1.0, 0, 1, clutch=True), hand, 0.1)  # a 1 m leap in 0.1 s
        assert np.linalg.norm(cmd.target[:3, 3]) == pytest.approx(0.05)
        for i in range(2, 60):
            cmd = arm.update(_sample(1.0, 0, 1, clutch=True), hand, 0.1 * i)
        assert np.allclose(cmd.target[:3, 3], [1.0, 0.0, 0.0], atol=1e-9)

    def test_orientation_follows_and_stays_a_rotation_over_many_steps(self) -> None:
        # Regression: re-multiplying the previous rotation by a step derived
        # from it doubled the rounding error every step and blew up after ~50.
        arm = ArmTeleop("left", ArmTeleopConfig(mapping="relative", max_angular_speed_rad_s=0.5))
        hand = np.eye(4)
        arm.update(_sample(0, 0, 1, clutch=True), hand, 0.0)
        cmd = None
        for i in range(1, 500):
            R = rotation_z(0.3 * math.sin(i / 20.0))
            cmd = arm.update(_sample(0, 0, 1, clutch=True, R=R), hand, i / 60.0)
            Rt = cmd.target[:3, :3]
            assert np.allclose(Rt @ Rt.T, np.eye(3), atol=1e-9)
            assert np.linalg.det(Rt) == pytest.approx(1.0, abs=1e-9)
        assert cmd is not None
        # Once settled, the target orientation is the controller's delta applied to the hand.
        settled = arm.update(_sample(0, 0, 1, clutch=True, R=rotation_z(0.2)), hand, 100.0)
        assert np.allclose(settled.target[:3, :3], rotation_z(0.2), atol=1e-9)
        fixed = ArmTeleop("left", ArmTeleopConfig(mapping="relative", orientation_enabled=False))
        fixed.update(_sample(0, 0, 1, clutch=True), hand, 0.0)
        cmd = fixed.update(_sample(0, 0, 1, clutch=True, R=rotation_z(0.5)), hand, 1.0)
        assert np.allclose(cmd.target[:3, :3], np.eye(3))

    def test_workspace_clamp(self) -> None:
        arm = ArmTeleop(
            "left",
            ArmTeleopConfig(
                mapping="relative",
                workspace_min_m=(-0.1, -0.1, 0.0),
                workspace_max_m=(0.2, 0.1, 0.5),
                max_speed_m_s=100.0,
            ),
        )
        hand = np.eye(4)
        hand[:3, 3] = [0.1, 0.0, 0.2]
        arm.update(_sample(0, 0, 1, clutch=True), hand, 0.0)
        cmd = arm.update(_sample(1.0, 0, 1, clutch=True), hand, 1.0)
        assert cmd.target[0, 3] == pytest.approx(0.2)

    def test_gripper_needs_measured_travel(self) -> None:
        unmeasured = ArmTeleopConfig(gripper_motor="l_arm_grip", gripper_open_rad=0.0)
        assert not unmeasured.gripper_available
        assert unmeasured.gripper_target(1.0) is None
        measured = ArmTeleopConfig(
            gripper_motor="l_arm_grip", gripper_open_rad=0.1, gripper_closed_rad=1.1
        )
        assert measured.gripper_target(0.0) == pytest.approx(0.1)
        assert measured.gripper_target(0.5) == pytest.approx(0.6)
        assert measured.gripper_target(7.0) == pytest.approx(1.1)  # clamped
        arm = ArmTeleop("left", measured)
        arm.set_anchor([0.0, 0.0, 1.0], [0.0, 0.0, 1.6])
        cmd = arm.update(
            ControllerSample(pose=np.eye(4), clutch=False, trigger=0.25), np.eye(4), 0.0
        )
        assert cmd.gripper_rad == pytest.approx(0.35)

    def test_config_validation(self) -> None:
        with pytest.raises(ValueError):
            ArmTeleopConfig(position_scale=0.0)
        with pytest.raises(ValueError):
            ArmTeleopConfig(workspace_min_m=(0, 0, 0))
        with pytest.raises(ValueError):
            ArmTeleopConfig(workspace_min_m=(0, 0, 0), workspace_max_m=(0, 1, 1))
        with pytest.raises(ValueError):
            ArmTeleop("middle")
        with pytest.raises(ValueError):
            ArmTeleopConfig(mapping="sideways")  # type: ignore[arg-type]

    def test_absolute_mapping_needs_an_anchor(self) -> None:
        arm = ArmTeleop("left")  # the default mapping is absolute
        with pytest.raises(RuntimeError, match="set_anchor"):
            arm.update(_sample(0, 0, 1.2, clutch=True), np.eye(4), 0.0)
        with pytest.raises(ValueError):
            arm.set_anchor([0.0, 0.0, float("nan")], [0.0, 0.0, 1.6])


class TestAbsoluteArmTeleop:
    """The operator's head at (0, 0, 1.6); the robot's head anchor at (0.1, 0, 1.2)."""

    ROBOT = np.array([0.1, 0.0, 1.2])
    OPERATOR = np.array([0.0, 0.0, 1.6])

    def _arm(self, **overrides: object) -> ArmTeleop:
        cfg = ArmTeleopConfig(max_speed_m_s=100.0, **overrides)  # type: ignore[arg-type]
        arm = ArmTeleop("right", cfg)
        arm.set_anchor(self.ROBOT, self.OPERATOR)
        return arm

    def test_pressing_sends_the_hand_to_the_controller_about_the_head(self) -> None:
        arm = self._arm()
        hand = np.eye(4)
        hand[:3, 3] = [0.3, -0.2, 0.9]  # wherever the robot's hand happens to be
        # Controller 40 cm ahead of and 30 cm below the operator's head, 25 cm right.
        engaged = arm.update(_sample(0.4, -0.25, 1.3, clutch=True), hand, 0.0)
        assert engaged.engaged_now and np.allclose(engaged.target, hand)  # starts where the hand is
        cmd = arm.update(_sample(0.4, -0.25, 1.3, clutch=True), hand, 0.1)
        assert np.allclose(cmd.target[:3, 3], self.ROBOT + [0.4, -0.25, -0.3])
        # Holding still does not drift; releasing latches; re-pressing goes back.
        again = arm.update(_sample(0.4, -0.25, 1.3, clutch=True), hand, 0.2)
        assert np.allclose(again.target, cmd.target)
        released = arm.update(_sample(0.9, 0.0, 1.3, clutch=False), hand, 0.3)
        assert released.released_now and np.allclose(released.target, cmd.target)
        hand2 = np.eye(4)
        hand2[:3, 3] = [0.5, -0.3, 1.0]
        arm.update(_sample(0.9, 0.0, 1.3, clutch=True), hand2, 0.4)
        back = arm.update(_sample(0.9, 0.0, 1.3, clutch=True), hand2, 0.5)
        assert np.allclose(back.target[:3, 3], self.ROBOT + [0.9, 0.0, -0.3])

    def test_the_approach_is_slew_limited(self) -> None:
        cfg = ArmTeleopConfig(max_speed_m_s=0.5)
        arm = ArmTeleop("right", cfg)
        arm.set_anchor(self.ROBOT, self.OPERATOR)
        hand = np.eye(4)
        hand[:3, 3] = self.ROBOT + [0.0, -0.25, -0.3]
        arm.update(_sample(0.4, -0.25, 1.3, clutch=True), hand, 0.0)
        step = arm.update(_sample(0.4, -0.25, 1.3, clutch=True), hand, 0.1)
        assert np.linalg.norm(step.target[:3, 3] - hand[:3, 3]) == pytest.approx(0.05)
        assert step.target[0, 3] > hand[0, 3]  # towards the controller, +X
        cmd = step
        for i in range(2, 40):
            cmd = arm.update(_sample(0.4, -0.25, 1.3, clutch=True), hand, 0.1 * i)
        assert np.allclose(cmd.target[:3, 3], self.ROBOT + [0.4, -0.25, -0.3], atol=1e-9)

    def test_scale_is_about_the_head_and_orientation_stays_relative(self) -> None:
        arm = self._arm(position_scale=0.5)
        hand = np.eye(4)
        hand[:3, :3] = rotation_z(1.0)
        arm.update(_sample(0.4, 0.0, 1.6, clutch=True), hand, 0.0)
        cmd = arm.update(_sample(0.4, 0.0, 1.6, clutch=True, R=rotation_z(0.2)), hand, 1.0)
        assert np.allclose(cmd.target[:3, 3], self.ROBOT + [0.2, 0.0, 0.0])
        # Orientation: the controller turned 0.2 rad since the press, so the
        # hand turns 0.2 rad from where it was -- not to the controller's own attitude.
        assert np.allclose(cmd.target[:3, :3], rotation_z(0.2) @ rotation_z(1.0), atol=1e-9)

    def test_dual_arm_anchor_and_description(self) -> None:
        teleop = DualArmTeleop(ArmTeleopConfig(), ArmTeleopConfig(mapping="relative"))
        assert teleop.needs_anchor
        teleop.set_anchor(self.ROBOT, self.OPERATOR)
        described = teleop.describe()
        assert described["left"]["mapping"] == "absolute"
        assert described["left"]["anchor"]["robot_m"] == pytest.approx(list(self.ROBOT))
        assert described["right"]["mapping"] == "relative"
        neither = DualArmTeleop(
            ArmTeleopConfig(mapping="relative"), ArmTeleopConfig(mapping="relative")
        )
        assert not neither.needs_anchor


class TestDualArmTeleop:
    def test_builds_a_dual_arm_target_with_expiry_and_grippers(self) -> None:
        left = ArmTeleopConfig(
            mapping="relative",
            gripper_motor="l_arm_grip",
            gripper_open_rad=0.0,
            gripper_closed_rad=1.0,
        )
        teleop = DualArmTeleop(left, RELATIVE, torso_policy=TorsoPolicy.OPTIMIZE, target_ttl_s=0.5)
        hands = {"left": np.eye(4), "right": np.eye(4)}
        out = teleop.update(
            {
                "left": ControllerSample(pose=np.eye(4), clutch=True, trigger=0.5),
                "right": None,
            },
            hands,
            0.0,
        )
        assert out.target.left_enabled and not out.target.right_enabled
        assert out.target.right_target is None
        assert out.target.torso_policy is TorsoPolicy.OPTIMIZE
        assert out.target.expiry_ns is not None
        assert out.target.expiry_ns - out.target.created_ns == pytest.approx(0.5e9, rel=1e-6)
        assert out.gripper_targets_rad == pytest.approx({"l_arm_grip": 0.5})
        teleop.release_all()
        held = teleop.update({"left": None, "right": None}, hands, 1.0)
        assert not held.target.left_enabled and not held.target.right_enabled
        assert held.gripper_targets_rad == {}
        with pytest.raises(ValueError):
            DualArmTeleop(target_ttl_s=0.0)
