"""The one place joint ranges are resolved, and what everything reads from it.

Pure numpy except the model-backed cases, which are skipped without the
``kinematics`` extra.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from robopy.kinematics.joint_limits import (
    JointLimitOverride,
    SoftLimit,
    check_home_pose,
    normalised_singular_values,
    resolve_joint_limits,
    unwrap_towards,
)

URDF = {
    "revolute": (-2.0, 2.0),
    "continuous": None,
    "on_the_stop": (-2.7925, 0.0),
}


class TestResolve:
    def test_urdf_range_stands_when_nothing_else_is_given(self) -> None:
        profile = resolve_joint_limits(list(URDF), urdf_limits=URDF)
        rev = profile["revolute"]
        assert (rev.lower, rev.upper, rev.source, rev.validated) == (-2.0, 2.0, "urdf", True)
        assert rev.finite and not rev.display_only and rev.urdf == (-2.0, 2.0)
        cont = profile["continuous"]
        assert cont.continuous and cont.display_only and cont.source == "display"
        assert (cont.lower, cont.upper) == (-math.pi, math.pi)
        assert not cont.validated and not cont.finite
        # The display range is for sliders; the real bound is still infinite.
        lower, upper = profile.bounds(["continuous"])
        assert lower[0] == -math.inf and upper[0] == math.inf
        assert profile.unbounded(list(URDF)) == ["continuous"]
        assert profile.slider_bounds(["continuous"]) == ([-math.pi], [math.pi])

    def test_a_soft_limit_narrows_and_carries_its_validation(self) -> None:
        profile = resolve_joint_limits(
            list(URDF),
            urdf_limits=URDF,
            soft_limits={
                "revolute": SoftLimit(-1.0, 3.0, validated=True, note="measured"),
                "continuous": (-1.5, 1.5),
            },
        )
        rev = profile["revolute"]
        # Narrowed on the lower side, not widened on the upper.
        assert (rev.lower, rev.upper) == (-1.0, 2.0)
        assert rev.source == "soft" and rev.validated is True
        assert any("wider than" in n for n in rev.notes)
        cont = profile["continuous"]
        assert (cont.lower, cont.upper) == (-1.5, 1.5)
        assert cont.source == "soft" and cont.validated is False and cont.finite
        assert any("provisional" in n for n in cont.notes)
        assert profile.unvalidated(list(URDF)) == ["continuous", "on_the_stop"] or set(
            profile.unvalidated(list(URDF))
        ) >= {"continuous"}

    def test_a_soft_limit_that_does_not_bite_leaves_the_urdf_source(self) -> None:
        profile = resolve_joint_limits(
            ["revolute"], urdf_limits=URDF, soft_limits={"revolute": (-5.0, 5.0)}
        )
        rev = profile["revolute"]
        assert rev.source == "urdf" and (rev.lower, rev.upper) == (-2.0, 2.0)

    def test_an_override_replaces_the_urdf_range_and_may_widen_it(self) -> None:
        profile = resolve_joint_limits(
            list(URDF),
            urdf_limits=URDF,
            overrides={
                "on_the_stop": JointLimitOverride(-2.7925, 0.1, reason="measured 6 deg beyond"),
                "continuous": {"lower": -3.0, "upper": 3.0, "reason": "cable stop"},
            },
            soft_limits={"on_the_stop": (-1.0, 5.0)},
        )
        stop = profile["on_the_stop"]
        # Override sets [-2.79, 0.1]; the soft limit then narrows the lower side.
        assert stop.source == "soft" and (stop.lower, stop.upper) == (-1.0, 0.1)
        assert stop.override is not None and stop.override.reason == "measured 6 deg beyond"
        assert any("replaced by an override" in n for n in stop.notes)
        cont = profile["continuous"]
        assert cont.source == "override" and cont.validated and cont.finite
        assert (cont.lower, cont.upper) == (-3.0, 3.0)

    def test_bad_inputs_are_refused(self) -> None:
        with pytest.raises(KeyError, match="unknown joint"):
            resolve_joint_limits(["revolute"], urdf_limits=URDF, soft_limits={"nope": (0, 1)})
        with pytest.raises(ValueError):
            SoftLimit(1.0, -1.0)
        with pytest.raises(ValueError):
            JointLimitOverride(0.0, 0.0)
        with pytest.raises(ValueError, match="does not overlap"):
            resolve_joint_limits(
                ["revolute"], urdf_limits=URDF, soft_limits={"revolute": (3.0, 4.0)}
            )
        with pytest.raises(ValueError, match="inverted"):
            resolve_joint_limits(["bad"], urdf_limits={"bad": (1.0, -1.0)})

    def test_violations_and_containment(self) -> None:
        profile = resolve_joint_limits(list(URDF), urdf_limits=URDF)
        assert profile.violations({"revolute": 2.5, "on_the_stop": 0.0, "continuous": 9.0}) == {
            "revolute": pytest.approx(0.5)
        }
        assert profile["on_the_stop"].contains(0.0)
        assert not profile["on_the_stop"].contains(0.0, margin=0.01)
        described = profile.describe()
        assert described["sources"]["display"] == ["continuous"]
        assert any("NOT validated" in line for line in profile.summary_lines())

    def test_unwrap_keeps_a_command_on_the_measured_turn(self) -> None:
        assert unwrap_towards(-3.1, 3.1) == pytest.approx(-3.1 + 2 * math.pi)
        assert unwrap_towards(3.1, -3.1) == pytest.approx(3.1 - 2 * math.pi)
        assert unwrap_towards(0.2, 0.1) == pytest.approx(0.2)
        assert unwrap_towards(0.0, 4 * math.pi) == pytest.approx(4 * math.pi)

    def test_normalised_singular_values_put_metres_and_radians_on_one_scale(self) -> None:
        J = np.zeros((6, 2))
        J[0, 0] = 1e-3  # one millimetre per radian of joint 0
        J[3, 1] = 1e-2  # ten milliradians per radian of joint 1
        sigma = normalised_singular_values(J, position_scale_m=1e-3, orientation_scale_rad=1e-2)
        assert sigma == pytest.approx([1.0, 1.0])
        with pytest.raises(ValueError):
            normalised_singular_values(
                np.zeros((5, 2)), position_scale_m=1, orientation_scale_rad=1
            )


pink = pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.kinematics.synthetic_dual_arm import (  # noqa: E402
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_TORSO_JOINT,
)
from robopy.kinematics.urdf_model import WholeBodyModel  # noqa: E402


class TestModelIntegration:
    def test_the_model_profile_is_what_position_limits_returns(self, synthetic_urdf: Path) -> None:
        model = WholeBodyModel.from_urdf(synthetic_urdf)
        model.set_joint_limit_overrides(
            {"elbow_pitch_left_dof": {"lower": -2.0, "upper": 2.9, "reason": "test"}}
        )
        model.set_soft_limits({SYNTHETIC_TORSO_JOINT: (-1.5, 1.5)})
        model.set_soft_limits({"elbow_yaw_left_dof": (-1.0, 1.0)}, validated=True)
        profile = model.limit_profile()
        for name in ("elbow_pitch_left_dof", SYNTHETIC_TORSO_JOINT, "elbow_yaw_left_dof"):
            lower, upper = model.position_limits([name])
            assert (lower[0], upper[0]) == (profile[name].lower, profile[name].upper)
        assert profile["elbow_pitch_left_dof"].source == "override"
        assert profile["elbow_yaw_left_dof"].validated is True
        assert profile[SYNTHETIC_TORSO_JOINT].validated is False
        assert model.unbounded_joints(SYNTHETIC_ARM_JOINTS["left"]) == ["shoulder_pitch_left_dof"]
        with pytest.raises(KeyError):
            model.set_joint_limit_overrides({"nope": (0.0, 1.0)})

    def test_positions_from_q_unwraps_towards_a_reference(self, synthetic_urdf: Path) -> None:
        model = WholeBodyModel.from_urdf(synthetic_urdf)
        positions = {name: 0.0 for name in model.movable_joint_names}
        positions[SYNTHETIC_TORSO_JOINT] = math.pi + 0.05  # decodes to -pi + 0.05
        q = model.q_from_positions(positions)
        wrapped = model.positions_from_q(q)[SYNTHETIC_TORSO_JOINT]
        assert wrapped == pytest.approx(-math.pi + 0.05)
        unwrapped = model.positions_from_q(q, reference={SYNTHETIC_TORSO_JOINT: math.pi})
        assert unwrapped[SYNTHETIC_TORSO_JOINT] == pytest.approx(math.pi + 0.05)

    def test_home_pose_check_reports_limits_and_singularity(self, synthetic_urdf: Path) -> None:
        model = WholeBodyModel.from_urdf(synthetic_urdf)
        model.set_soft_limits(
            {
                SYNTHETIC_TORSO_JOINT: (-1.5, 1.5),
                "shoulder_pitch_left_dof": (-2.0, 2.0),
                "shoulder_pitch_right_dof": (-2.0, 2.0),
            }
        )
        for side in ("left", "right"):
            model.add_fixed_frame(f"{side}_tcp", f"gripper_{side}_dof", np.eye(4))
        profile = model.limit_profile()
        arms = {s: (SYNTHETIC_ARM_JOINTS[s], f"{s}_tcp") for s in ("left", "right")}
        zero = {name: 0.0 for name in model.movable_joint_names}
        # Straight down, every arm fully extended: singular.
        straight = check_home_pose(model, profile, zero, arms=arms, singular_threshold=1.0)
        assert not straight.ok and any("singular" in p for p in straight.problems)
        bent = dict(zero, elbow_pitch_left_dof=0.8, elbow_pitch_right_dof=-0.8)
        assert check_home_pose(model, profile, bent, arms=arms, singular_threshold=1.0).ok
        outside = dict(bent, elbow_yaw_left_dof=5.0)
        report = check_home_pose(model, profile, outside, arms=arms)
        assert any("elbow_yaw_left_dof" in p and "outside" in p for p in report.problems)
        assert (
            check_home_pose(model, profile, {"torso_yaw_dof": 0.0})
            .problems[0]
            .startswith("missing joint")
        )
