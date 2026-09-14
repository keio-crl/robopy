"""The raw/SI conversion boundary, and the joint-map consistency checks."""

from __future__ import annotations

import math

import numpy as np
import pytest

from robopy.control.joint_mapping import (
    RAW_VELOCITY_UNIT_RAD_S,
    JointCalibration,
    JointMap,
    JointMapError,
    ValidationLevel,
    check_torque_policy,
)

from .conftest import make_calibration


def _cal(**overrides: object) -> JointCalibration:
    fields: dict[str, object] = {
        "motor_name": "j",
        "motor_id": 1,
        "model": "xm430-w350",
        "zero_count": 2048,
    }
    fields.update(overrides)
    return JointCalibration(**fields)  # type: ignore[arg-type]


class TestUnitConversion:
    def test_count_rad_round_trip_is_within_half_a_count(self) -> None:
        calibration = _cal()
        half_count = math.pi / 4096  # half of 2*pi/4096
        for angle in np.linspace(-3.0, 3.0, 401):
            recovered = calibration.count_to_rad(calibration.rad_to_count(float(angle)))
            assert abs(recovered - angle) <= half_count

    def test_zero_count_maps_to_zero_radians(self) -> None:
        assert _cal(zero_count=1234).count_to_rad(1234) == 0.0

    def test_direction_inverts_position_velocity_and_current_alike(self) -> None:
        forward = _cal(direction=1)
        reversed_axis = _cal(direction=-1)
        assert reversed_axis.count_to_rad(3048) == -forward.count_to_rad(3048)
        assert reversed_axis.raw_velocity_to_rad_s(100) == -forward.raw_velocity_to_rad_s(100)
        assert reversed_axis.raw_current_to_a(50) == -forward.raw_current_to_a(50)

    def test_velocity_uses_the_documented_unit_and_ignores_the_position_offset(self) -> None:
        near = _cal(zero_count=0)
        far = _cal(zero_count=4096)
        assert near.raw_velocity_to_rad_s(100) == far.raw_velocity_to_rad_s(100)
        assert near.raw_velocity_to_rad_s(1) == pytest.approx(RAW_VELOCITY_UNIT_RAD_S)
        assert RAW_VELOCITY_UNIT_RAD_S == pytest.approx(0.229 * 2 * math.pi / 60)

    def test_velocity_round_trip(self) -> None:
        calibration = _cal(direction=-1)
        for raw in (-1000, -1, 0, 1, 1000):
            assert calibration.rad_s_to_raw_velocity(calibration.raw_velocity_to_rad_s(raw)) == raw

    def test_current_unit_is_per_model(self) -> None:
        xm = _cal(model="xm430-w350")
        xc = _cal(model="xc330-t288")
        assert xm.raw_current_to_a(100) == pytest.approx(0.269)
        assert xc.raw_current_to_a(100) == pytest.approx(0.1)
        assert xm.a_to_raw_current(-0.269) == -100

    def test_multi_turn_positions_are_not_wrapped(self) -> None:
        calibration = _cal()
        angle = calibration.count_to_rad(2048 + 3 * 4096)
        assert angle == pytest.approx(3 * 2 * math.pi)
        assert angle > math.pi

    def test_external_gear_ratio_scales_position_and_velocity(self) -> None:
        direct = _cal()
        geared = _cal(external_gear_ratio=2.0)
        assert geared.count_to_rad(3072) == pytest.approx(direct.count_to_rad(3072) / 2)
        assert geared.raw_velocity_to_rad_s(100) == pytest.approx(
            direct.raw_velocity_to_rad_s(100) / 2
        )

    def test_uncalibrated_zero_point_is_refused(self) -> None:
        with pytest.raises(JointMapError, match="zero_count is not calibrated"):
            _cal(zero_count=None).count_to_rad(2048)

    def test_torque_conversion_refuses_an_unmeasured_constant(self) -> None:
        with pytest.raises(JointMapError, match="torque_constant_nm_per_a is not calibrated"):
            _cal().torque_to_current_a(0.1)

    def test_torque_conversion_uses_the_measured_constant(self) -> None:
        assert _cal(torque_constant_nm_per_a=2.0).torque_to_current_a(1.0) == pytest.approx(0.5)


class TestJointMapValidation:
    def test_duplicate_motor_name_is_rejected(self) -> None:
        with pytest.raises(JointMapError, match="registered more than once"):
            JointMap([_cal(motor_name="a", motor_id=1), _cal(motor_name="a", motor_id=2)])

    def test_duplicate_motor_id_is_rejected(self) -> None:
        with pytest.raises(JointMapError, match="is used by both"):
            JointMap([_cal(motor_name="a", motor_id=1), _cal(motor_name="b", motor_id=1)])

    def test_two_motors_cannot_claim_one_urdf_joint(self) -> None:
        with pytest.raises(JointMapError, match="is assigned to both"):
            JointMap(
                [
                    _cal(motor_name="a", motor_id=1, urdf_joint="shoulder"),
                    _cal(motor_name="b", motor_id=2, urdf_joint="shoulder"),
                ]
            )

    def test_unknown_urdf_joint_is_rejected(self) -> None:
        with pytest.raises(JointMapError, match="Unknown or non-movable URDF joint"):
            JointMap(
                [_cal(urdf_joint="gripper_left_dof")],
                known_urdf_joints=["torso_yaw_dof", "shoulder_pitch_left_dof"],
            )

    def test_bad_direction_and_inverted_limits_are_rejected(self) -> None:
        with pytest.raises(JointMapError, match="direction must be"):
            _cal(direction=0)
        with pytest.raises(JointMapError, match="exceeds upper_limit_rad"):
            _cal(lower_limit_rad=1.0, upper_limit_rad=-1.0)

    def test_unknown_model_is_rejected_at_configuration_time(self) -> None:
        with pytest.raises(ValueError, match="No verified physical-unit data"):
            _cal(model="xl430-w250")

    def test_empty_map_is_rejected(self) -> None:
        with pytest.raises(JointMapError, match="at least one calibration"):
            JointMap([])

    def test_lookup_ignores_dictionary_order(self) -> None:
        mapping = make_calibration(("a", "b", "c"))
        counts = {"c": 3072, "a": 2048, "b": 1024}
        forward = mapping.counts_to_rad(counts, ["a", "b", "c"])
        backward = mapping.counts_to_rad(counts, ["c", "b", "a"])
        np.testing.assert_allclose(forward, backward[::-1])

    def test_missing_entry_raises_rather_than_defaulting(self) -> None:
        mapping = make_calibration(("a", "b"))
        with pytest.raises(KeyError):
            mapping.counts_to_rad({"a": 2048}, ["a", "b"])

    def test_unknown_motor_lookup_raises(self) -> None:
        with pytest.raises(JointMapError, match="Unknown motor"):
            make_calibration(("a",))["zzz"]


class TestValidationLevels:
    def test_geometry_level_ignores_torque_fields(self) -> None:
        calibration = _cal(lower_limit_rad=-1.0, upper_limit_rad=1.0)
        assert calibration.is_complete(ValidationLevel.GEOMETRY)
        assert not calibration.is_complete(ValidationLevel.HARDWARE)

    def test_hardware_level_lists_every_missing_field(self) -> None:
        missing = _cal(lower_limit_rad=-1.0, upper_limit_rad=1.0).missing_fields(
            ValidationLevel.HARDWARE
        )
        assert missing == [
            "current_limit_a",
            "max_velocity_rad_s",
            "torque_constant_nm_per_a",
            "validated",
        ]

    def test_require_names_the_gaps(self) -> None:
        mapping = JointMap([_cal(lower_limit_rad=-1.0, upper_limit_rad=1.0)])
        with pytest.raises(JointMapError, match="must be measured, not guessed"):
            mapping.require(ValidationLevel.HARDWARE)

    def test_a_validated_map_passes_both_levels(self) -> None:
        mapping = make_calibration(("a", "b"))
        mapping.require(ValidationLevel.GEOMETRY)
        mapping.require(ValidationLevel.HARDWARE)

    def test_validated_flag_alone_is_required_for_hardware(self) -> None:
        mapping = make_calibration(("a",), validated=False)
        assert mapping.validation_report(ValidationLevel.HARDWARE) == {"a": ["validated"]}


class TestCommandPathExclusion:
    def test_two_maps_cannot_claim_the_same_urdf_joint(self) -> None:
        first = JointMap([_cal(motor_name="a", motor_id=1, urdf_joint="torso_yaw_dof")])
        second = JointMap([_cal(motor_name="b", motor_id=2, urdf_joint="torso_yaw_dof")])
        with pytest.raises(JointMapError, match="claimed by two command paths"):
            first.assert_disjoint_from(second)

    def test_disjoint_maps_are_accepted(self) -> None:
        first = JointMap([_cal(motor_name="a", motor_id=1, urdf_joint="torso_yaw_dof")])
        second = JointMap([_cal(motor_name="b", motor_id=2, urdf_joint="head_yaw_dof")])
        first.assert_disjoint_from(second)


class TestTorquePolicyCheck:
    def test_a_coupled_joint_that_is_torque_off_is_a_configuration_error(self) -> None:
        with pytest.raises(JointMapError, match="not adjusted"):
            check_torque_policy(
                ["torso_yaw"],
                ["l_arm_grip"],
                all_motor_names=["torso_yaw", "l_arm_grip"],
                side="follower",
            )

    def test_unknown_coupled_motor_is_rejected(self) -> None:
        with pytest.raises(JointMapError, match="unknown coupled motor"):
            check_torque_policy(["nope"], None, all_motor_names=["torso_yaw"], side="leader")

    def test_none_policy_skips_the_check(self) -> None:
        check_torque_policy(["torso_yaw"], None, all_motor_names=["torso_yaw"], side="leader")
