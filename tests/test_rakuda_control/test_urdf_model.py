"""Whole-body model: configuration handling, FK, Jacobians and collision.

Skipped when the optional ``kinematics`` extra is absent.
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")

from robopy.kinematics.synthetic_dual_arm import (  # noqa: E402
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_HEAD_JOINTS,
    SYNTHETIC_SPEC,
    SYNTHETIC_TCP_FRAMES,
    SYNTHETIC_TORSO_JOINT,
)
from robopy.kinematics.urdf_model import WholeBodyModel  # noqa: E402

ALL_ARM_JOINTS = SYNTHETIC_ARM_JOINTS["left"] + SYNTHETIC_ARM_JOINTS["right"]


class TestConfigurationSpace:
    def test_nq_and_nv_differ_because_of_continuous_joints(self, whole_body_model) -> None:
        # Three continuous joints occupy two configuration entries each.
        assert whole_body_model.nv == 15
        assert whole_body_model.nq == 18

    def test_continuous_joints_are_identified(self, whole_body_model) -> None:
        assert whole_body_model.is_continuous(SYNTHETIC_TORSO_JOINT)
        assert whole_body_model.is_continuous("shoulder_pitch_left_dof")
        assert not whole_body_model.is_continuous("elbow_yaw_left_dof")

    def test_a_fixed_joint_is_a_frame_but_not_a_movable_joint(self, whole_body_model) -> None:
        assert whole_body_model.has_frame("gripper_left_dof")
        assert not whole_body_model.has_joint("gripper_left_dof")
        with pytest.raises(KeyError, match="not evidence of a degree of freedom"):
            whole_body_model.joint_v_index("gripper_left_dof")

    def test_positions_round_trip_through_the_configuration_vector(
        self, whole_body_model
    ) -> None:
        wanted = {
            name: 0.1 * (index + 1)
            for index, name in enumerate(whole_body_model.movable_joint_names)
        }
        recovered = whole_body_model.positions_from_q(whole_body_model.q_from_positions(wanted))
        for name, value in wanted.items():
            assert recovered[name] == pytest.approx(value)

    def test_a_missing_joint_is_refused_by_default(self, whole_body_model) -> None:
        with pytest.raises(KeyError, match="Missing joint position"):
            whole_body_model.q_from_positions({SYNTHETIC_TORSO_JOINT: 0.1})

    def test_an_unknown_joint_is_refused(self, whole_body_model) -> None:
        with pytest.raises(KeyError, match="Unknown movable joint"):
            whole_body_model.q_from_positions({"nope": 0.1}, require_all=False)

    def test_a_velocity_cannot_be_passed_where_a_configuration_is_expected(
        self, whole_body_model
    ) -> None:
        with pytest.raises(ValueError, match=r"nq != nv"):
            whole_body_model.positions_from_q(np.zeros(whole_body_model.nv))

    def test_integrate_uses_the_manifold_not_plain_addition(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        v = np.zeros(whole_body_model.nv)
        v[whole_body_model.joint_v_index(SYNTHETIC_TORSO_JOINT)] = 0.25
        positions = whole_body_model.positions_from_q(whole_body_model.integrate(q, v))
        assert positions[SYNTHETIC_TORSO_JOINT] == pytest.approx(0.25)

    def test_difference_is_the_inverse_of_integrate(self, whole_body_model) -> None:
        q0 = whole_body_model.neutral_q()
        v = np.linspace(-0.2, 0.2, whole_body_model.nv)
        q1 = whole_body_model.integrate(q0, v)
        np.testing.assert_allclose(whole_body_model.difference(q0, q1), v, atol=1e-9)


class TestForwardKinematics:
    def test_zero_pose_matches_the_hand_computed_geometry(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        for side in ("left", "right"):
            pose = whole_body_model.frame_pose(q, SYNTHETIC_TCP_FRAMES[side])
            expected = SYNTHETIC_SPEC.tcp_position_at_zero(side)
            np.testing.assert_allclose(pose[:3, 3], expected, atol=1e-12)
            np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-12)

    def test_a_torso_yaw_rotates_both_hands_about_the_base_z_axis(
        self, whole_body_model
    ) -> None:
        angle = 0.5
        q = whole_body_model.q_from_positions(
            {name: 0.0 for name in whole_body_model.movable_joint_names}
        )
        q_yawed = whole_body_model.q_from_positions(
            {
                name: (angle if name == SYNTHETIC_TORSO_JOINT else 0.0)
                for name in whole_body_model.movable_joint_names
            }
        )
        for side in ("left", "right"):
            frame = SYNTHETIC_TCP_FRAMES[side]
            before = whole_body_model.frame_pose(q, frame)[:3, 3]
            after = whole_body_model.frame_pose(q_yawed, frame)[:3, 3]
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            # The torso axis sits at z = torso_height, so only x and y rotate.
            np.testing.assert_allclose(after, rotation @ before, atol=1e-12)

    def test_an_added_tcp_frame_sits_at_its_declared_offset(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        parent = whole_body_model.frame_pose(q, SYNTHETIC_TCP_FRAMES["left"])
        tcp = whole_body_model.frame_pose(q, "left_tcp")
        np.testing.assert_allclose(tcp[:3, 3] - parent[:3, 3], [0.0, 0.0, -0.02], atol=1e-12)

    def test_redefining_an_existing_frame_is_refused(self, whole_body_model) -> None:
        with pytest.raises(ValueError, match="refusing to redefine"):
            whole_body_model.add_fixed_frame("left_tcp", "gripper_left_dof", np.eye(4))

    def test_an_unknown_parent_frame_is_refused(self, whole_body_model) -> None:
        with pytest.raises(KeyError, match="does not exist"):
            whole_body_model.add_fixed_frame("x", "no_such_frame", np.eye(4))


class TestJacobian:
    @staticmethod
    def _numerical_jacobian(model: WholeBodyModel, q, frame, delta=1e-6):  # type: ignore[no-untyped-def]
        """Body-frame Jacobian by central differences, for comparison."""
        import pinocchio as pin

        columns = []
        for index in range(model.nv):
            step = np.zeros(model.nv)
            step[index] = delta
            plus = model.frame_pose(model.integrate(q, step), frame)
            minus = model.frame_pose(model.integrate(q, -step), frame)
            se3_plus = pin.SE3(plus[:3, :3], plus[:3, 3])
            se3_minus = pin.SE3(minus[:3, :3], minus[:3, 3])
            base = model.frame_pose(q, frame)
            se3_base = pin.SE3(base[:3, :3], base[:3, 3])
            twist = (
                pin.log(se3_base.actInv(se3_plus)).vector
                - pin.log(se3_base.actInv(se3_minus)).vector
            ) / (2 * delta)
            columns.append(twist)
        return np.asarray(columns).T

    def test_analytic_jacobian_matches_finite_differences(self, whole_body_model) -> None:
        rng = np.random.default_rng(0)
        positions = {
            name: float(rng.uniform(-0.5, 0.5))
            for name in whole_body_model.movable_joint_names
        }
        q = whole_body_model.q_from_positions(positions)
        for frame in ("left_tcp", "right_tcp"):
            analytic = whole_body_model.frame_jacobian(q, frame, local=True)
            numeric = self._numerical_jacobian(whole_body_model, q, frame)
            np.testing.assert_allclose(analytic, numeric, atol=1e-6)

    def test_the_torso_column_is_non_zero_for_both_hands(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        column = whole_body_model.joint_v_index(SYNTHETIC_TORSO_JOINT)
        for frame in ("left_tcp", "right_tcp"):
            jacobian = whole_body_model.frame_jacobian(q, frame)
            assert np.linalg.norm(jacobian[:, column]) > 1e-6

    def test_a_left_arm_joint_does_not_move_the_right_hand(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        jacobian = whole_body_model.frame_jacobian(q, "right_tcp")
        for name in SYNTHETIC_ARM_JOINTS["left"]:
            column = whole_body_model.joint_v_index(name)
            np.testing.assert_allclose(jacobian[:, column], 0.0, atol=1e-12)

    def test_head_joints_do_not_move_either_hand(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        for frame in ("left_tcp", "right_tcp"):
            jacobian = whole_body_model.frame_jacobian(q, frame)
            for name in SYNTHETIC_HEAD_JOINTS:
                column = whole_body_model.joint_v_index(name)
                np.testing.assert_allclose(jacobian[:, column], 0.0, atol=1e-12)


class TestLimits:
    def test_a_continuous_joint_is_unbounded_until_a_soft_limit_is_given(
        self, synthetic_urdf
    ) -> None:
        model = WholeBodyModel.from_urdf(synthetic_urdf)
        lower, upper = model.position_limits([SYNTHETIC_TORSO_JOINT])
        # Not the +/-1.01 Pinocchio stores for the (cos, sin) pair.
        assert not np.isfinite(lower[0]) and not np.isfinite(upper[0])
        assert model.unbounded_joints([SYNTHETIC_TORSO_JOINT]) == [SYNTHETIC_TORSO_JOINT]

    def test_a_soft_limit_applies_to_a_continuous_joint(self, whole_body_model) -> None:
        lower, upper = whole_body_model.position_limits([SYNTHETIC_TORSO_JOINT])
        assert (lower[0], upper[0]) == (-1.5, 1.5)
        assert whole_body_model.unbounded_joints([SYNTHETIC_TORSO_JOINT]) == []

    def test_a_soft_limit_only_tightens_a_urdf_limit(self, whole_body_model) -> None:
        whole_body_model.set_soft_limits({"elbow_yaw_left_dof": (-10.0, 10.0)})
        lower, upper = whole_body_model.position_limits(["elbow_yaw_left_dof"])
        assert (lower[0], upper[0]) == (-2.8, 2.8)

    def test_an_inverted_soft_limit_is_refused(self, whole_body_model) -> None:
        with pytest.raises(ValueError, match="exceeds upper limit"):
            whole_body_model.set_soft_limits({"elbow_yaw_left_dof": (1.0, -1.0)})


class TestCollision:
    def test_pairs_are_classified_rather_than_blanket_excluded(self, whole_body_model) -> None:
        groups = whole_body_model.classify_collision_pairs(whole_body_model.neutral_q())
        assert set(groups) == {"same_body", "parent_child", "interfering_at_q", "other"}
        # The genuine self-collision candidates are the large group, and they
        # survive classification rather than being excluded with the rest.
        assert len(groups["other"]) > len(groups["parent_child"])
        assert ("left_hand_link_0", "gripper_left_dof_0") in groups["same_body"]

    def test_the_resting_pose_is_collision_free_after_the_recorded_exclusions(
        self, whole_body_model
    ) -> None:
        report = whole_body_model.collision_report(whole_body_model.neutral_q())
        assert report.min_distance > 0.0

    def test_swinging_the_arms_inward_reduces_the_distance_and_then_collides(
        self, whole_body_model
    ) -> None:
        def distance(roll: float) -> float:
            positions = {name: 0.0 for name in whole_body_model.movable_joint_names}
            positions["shoulder_roll_left_dof"] = -roll
            positions["shoulder_roll_right_dof"] = roll
            return whole_body_model.collision_report(
                whole_body_model.q_from_positions(positions)
            ).min_distance

        resting = whole_body_model.collision_report(whole_body_model.neutral_q()).min_distance
        assert distance(0.3) < resting
        # Far enough in, the forearms interpenetrate the torso: a negative
        # distance, not merely a small one.
        assert distance(0.6) < 0.0

    def test_the_distance_gradient_matches_a_finite_difference(self, whole_body_model) -> None:
        positions = {name: 0.0 for name in whole_body_model.movable_joint_names}
        positions["shoulder_roll_left_dof"] = 0.8
        positions["shoulder_roll_right_dof"] = -0.8
        q = whole_body_model.q_from_positions(positions)
        report = whole_body_model.collision_report(q)
        index = int(np.argmin(report.distances))
        row = whole_body_model.distance_jacobian_row(
            q,
            report.witness_a[index],
            report.joint_a[index],
            report.witness_b[index],
            report.joint_b[index],
            report.normals[index],
        )

        column = whole_body_model.joint_v_index("shoulder_roll_left_dof")
        delta = 1e-5
        step = np.zeros(whole_body_model.nv)
        step[column] = delta
        forward = whole_body_model.collision_report(whole_body_model.integrate(q, step))
        backward = whole_body_model.collision_report(whole_body_model.integrate(q, -step))
        pair = report.pair_names[index]
        numeric = (
            forward.distances[forward.pair_names.index(pair)]
            - backward.distances[backward.pair_names.index(pair)]
        ) / (2 * delta)
        assert row[column] == pytest.approx(numeric, abs=1e-3)

    def test_a_model_without_collision_geometry_says_so(self, synthetic_urdf) -> None:
        model = WholeBodyModel.from_urdf(synthetic_urdf, build_collision=False)
        with pytest.raises(RuntimeError, match="without collision geometry"):
            model.collision_report(model.neutral_q())


class TestAmbiguousFrameNames:
    """The real export names a fixed joint and its child link identically.

    Pinocchio then holds a ``FIXED_JOINT`` and a ``BODY`` frame with the same
    name and a plain ``getFrameId`` raises. The fixture reproduces the naming
    by default, so every FK test above already runs through the resolver; these
    tests pin the behaviour down explicitly.
    """

    def test_the_fixture_reproduces_the_duplicate_frame_names(self, whole_body_model) -> None:
        for name in ("gripper_left_dof", "gripper_right_dof", "head_camera_link"):
            assert len(whole_body_model.frame_ids(name)) == 2

    def test_a_duplicate_name_resolves_to_the_body_frame(self, whole_body_model) -> None:
        import pinocchio as pin

        index = whole_body_model.frame_id("gripper_left_dof")
        assert whole_body_model.model.frames[index].type == pin.FrameType.BODY

    def test_both_frames_of_a_duplicate_name_coincide(self, whole_body_model) -> None:
        # In URDF a fixed joint's frame and its child link's frame are the same
        # place, so the choice between them is numerically immaterial.
        a, b = whole_body_model.frame_ids("gripper_left_dof")
        frames = whole_body_model.model.frames
        np.testing.assert_allclose(
            frames[a].placement.homogeneous, frames[b].placement.homogeneous, atol=1e-12
        )

    def test_fk_and_jacobian_accept_a_duplicate_name(self, whole_body_model) -> None:
        q = whole_body_model.neutral_q()
        pose = whole_body_model.frame_pose(q, "gripper_left_dof")
        np.testing.assert_allclose(
            pose[:3, 3], SYNTHETIC_SPEC.tcp_position_at_zero("left"), atol=1e-12
        )
        assert whole_body_model.frame_jacobian(q, "gripper_left_dof").shape == (6, 15)

    def test_a_unique_name_resolves_directly(self, whole_body_model) -> None:
        assert len(whole_body_model.frame_ids("left_tcp")) == 1
        assert whole_body_model.frame_id("left_tcp") == whole_body_model.frame_ids("left_tcp")[0]

    def test_an_unknown_name_raises_key_error(self, whole_body_model) -> None:
        with pytest.raises(KeyError, match="Unknown frame"):
            whole_body_model.frame_id("no_such_frame")

    def test_genuinely_different_frames_of_one_name_are_refused(
        self, whole_body_model
    ) -> None:
        import pinocchio as pin

        # Manufacture a real ambiguity: two frames of one name at different
        # places. Pinocchio's addFrame silently returns the existing index when
        # the name *and type* already exist, so the two must differ in type.
        model = whole_body_model.model
        parent = model.frames[model.getFrameId("root")]
        model.addFrame(
            pin.Frame(
                "genuinely_ambiguous",
                parent.parentJoint,
                model.getFrameId("root"),
                pin.SE3(np.eye(3), np.array([0.1, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            )
        )
        model.addFrame(
            pin.Frame(
                "genuinely_ambiguous",
                parent.parentJoint,
                model.getFrameId("root"),
                pin.SE3(np.eye(3), np.array([0.2, 0.0, 0.0])),
                pin.FrameType.SENSOR,
            )
        )
        assert len(whole_body_model.frame_ids("genuinely_ambiguous")) == 2
        with pytest.raises(ValueError, match="do not coincide"):
            whole_body_model.frame_id("genuinely_ambiguous")

    def test_unique_link_names_can_still_be_generated(self, tmp_path) -> None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        path = write_synthetic_dual_arm_urdf(tmp_path / "u.urdf", joint_named_child_links=False)
        model = WholeBodyModel.from_urdf(path)
        assert len(model.frame_ids("gripper_left_dof")) == 1


class TestCuratedCollisionPairs:
    """Group-to-group registration, for CAD assemblies where all-pairs is unusable."""

    def test_geometry_names_can_be_selected_by_joint(self, whole_body_model) -> None:
        left = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["left"])
        assert left and all("left" in name for name in left)
        assert len(whole_body_model.geometry_names()) > len(left)

    def test_cross_group_pairs_only(self, whole_body_model) -> None:
        left = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["left"])
        right = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["right"])
        count = whole_body_model.add_collision_pairs_between(left, right)
        assert count == len(left) * len(right)
        report = whole_body_model.collision_report(whole_body_model.neutral_q())
        for a, b in report.pair_names:
            assert (a in left) != (a in right)
            assert (a in left and b in right) or (a in right and b in left)

    def test_adjacent_bodies_are_skipped_structurally(self, whole_body_model) -> None:
        # Hand and gripper hang off the same joint; the wrist-yaw body is the
        # hand's direct parent. Neither is a collision candidate and neither
        # needs a distance evaluation to know that.
        hand = whole_body_model.geometry_names(["wrist_pitch_left_dof"])
        wrist = whole_body_model.geometry_names(["wrist_yaw_left_dof"])
        assert whole_body_model.add_collision_pairs_between(hand, wrist) == 0
        assert whole_body_model.add_collision_pairs_between(
            hand, wrist, skip_adjacent=False
        ) == len(hand) * len(wrist)
        # Two joints apart (forearm vs hand) is not adjacent and is kept.
        forearm = whole_body_model.geometry_names(["elbow_pitch_left_dof"])
        assert whole_body_model.add_collision_pairs_between(hand, forearm) == len(hand) * len(
            forearm
        )

    def test_append_keeps_earlier_registrations(self, whole_body_model) -> None:
        left = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["left"])
        right = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["right"])
        torso = whole_body_model.geometry_names([SYNTHETIC_TORSO_JOINT])
        first = whole_body_model.add_collision_pairs_between(left, right)
        second = whole_body_model.add_collision_pairs_between(left, torso, append=True)
        assert second > 0
        report = whole_body_model.collision_report(whole_body_model.neutral_q())
        assert len(report.pair_names) == first + second

    def test_recorded_exclusions_are_honoured(self, whole_body_model) -> None:
        left = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["left"])
        right = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["right"])
        all_pairs = whole_body_model.add_collision_pairs_between(left, right)
        fewer = whole_body_model.add_collision_pairs_between(
            left, right, excluded=[(right[0], left[0])]
        )
        assert fewer == all_pairs - 1

    def test_close_pairs_are_reported_closest_first(self, whole_body_model) -> None:
        left = whole_body_model.geometry_names(SYNTHETIC_ARM_JOINTS["left"])
        torso = whole_body_model.geometry_names([SYNTHETIC_TORSO_JOINT])
        whole_body_model.add_collision_pairs_between(left, torso)
        positions = {name: 0.0 for name in whole_body_model.movable_joint_names}
        positions["shoulder_roll_left_dof"] = -0.3  # swings the arm into the torso
        q = whole_body_model.q_from_positions(positions)
        close = whole_body_model.pairs_closer_than(q, 0.05)
        assert close
        distances = [d for _, _, d in close]
        assert distances == sorted(distances)
        assert all(d < 0.05 for d in distances)

    def test_an_unknown_geometry_name_is_refused(self, whole_body_model) -> None:
        with pytest.raises(KeyError, match="Unknown collision geometry"):
            whole_body_model.add_collision_pairs_between(["nope_0"], ["torso_link_0"])
