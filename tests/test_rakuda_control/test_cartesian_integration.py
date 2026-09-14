"""Cartesian teleoperation end to end: config -> model -> IK -> simulated bus.

Skipped when the optional ``kinematics`` extra is absent.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.config.robot_config.rakuda_config import (  # noqa: E402
    RakudaControlConfig,
    RakudaJointCalibrationSpec,
    RakudaModelConfig,
    RakudaTcpSpec,
)
from robopy.control.types import DualArmTarget, TorsoPolicy  # noqa: E402
from robopy.kinematics.synthetic_dual_arm import (  # noqa: E402
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_HEAD_JOINTS,
    SYNTHETIC_TCP_FRAMES,
    SYNTHETIC_TORSO_JOINT,
)
from robopy.motor.dynamixel_bus import DynamixelMotor  # noqa: E402
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint  # noqa: E402
from robopy.robots.rakuda.rakuda_control import RakudaControlSystem  # noqa: E402

# Fictional motor names for the fixture, mapped onto the synthetic URDF joints.
# On the real machine this correspondence is a measurement, not a naming rule.
MOTOR_TO_URDF = {
    "torso_yaw": SYNTHETIC_TORSO_JOINT,
    **{f"l_arm_{i}": name for i, name in enumerate(SYNTHETIC_ARM_JOINTS["left"])},
    **{f"r_arm_{i}": name for i, name in enumerate(SYNTHETIC_ARM_JOINTS["right"])},
    "head_yaw": SYNTHETIC_HEAD_JOINTS[0],
    "head_pitch": SYNTHETIC_HEAD_JOINTS[1],
}
MOTORS = tuple(MOTOR_TO_URDF)


def _bus() -> SimulatedDynamixelBus:
    motors = {
        name: DynamixelMotor(index + 1, name, "xm430-w350")
        for index, name in enumerate(MOTORS)
    }
    joints = {name: SimulatedJoint() for name in MOTORS}
    return SimulatedDynamixelBus(motors, joints=joints, auto_step=False)


def _config(urdf: Path) -> RakudaControlConfig:
    calibration = {
        motor: RakudaJointCalibrationSpec(
            urdf_joint=urdf_joint,
            direction=1,
            zero_count=2048,
            lower_limit_rad=-3.0,
            upper_limit_rad=3.0,
            max_velocity_rad_s=3.0,
            validated=True,
        )
        for motor, urdf_joint in MOTOR_TO_URDF.items()
    }
    return RakudaControlConfig(
        mode="cartesian_teleop",
        control_period_s=0.01,
        ik_period_s=5.0,
        # The loop is stepped by hand here, so wall time between a read and its
        # check is not the configured period. See test_integration.py.
        max_state_age_s=5.0,
        max_cross_bus_skew_s=5.0,
        leader_joint_calibration=dict(calibration),
        follower_joint_calibration=dict(calibration),
        model=RakudaModelConfig(
            urdf_path=str(urdf),
            torso_joint=SYNTHETIC_TORSO_JOINT,
            left_arm_joints=list(SYNTHETIC_ARM_JOINTS["left"]),
            right_arm_joints=list(SYNTHETIC_ARM_JOINTS["right"]),
            head_joints=list(SYNTHETIC_HEAD_JOINTS),
            left_tcp=RakudaTcpSpec(
                parent_frame=SYNTHETIC_TCP_FRAMES["left"],
                translation_m=(0.0, 0.0, -0.02),
                validated=True,
            ),
            right_tcp=RakudaTcpSpec(
                parent_frame=SYNTHETIC_TCP_FRAMES["right"],
                translation_m=(0.0, 0.0, -0.02),
                validated=True,
            ),
            soft_limits_rad={
                SYNTHETIC_TORSO_JOINT: (-1.5, 1.5),
                "shoulder_pitch_left_dof": (-2.0, 2.0),
                "shoulder_pitch_right_dof": (-2.0, 2.0),
            },
            build_collision=False,
            geometry_only=True,
        ),
    )


@pytest.fixture
def cartesian_system(synthetic_urdf: Path):  # type: ignore[no-untyped-def]
    leader, follower = _bus(), _bus()
    system = RakudaControlSystem.from_buses(_config(synthetic_urdf), leader, follower)
    return system, leader, follower


class TestCartesianTeleoperation:
    def test_the_follower_converges_on_a_reachable_pair_of_targets(
        self, cartesian_system
    ) -> None:
        system, _leader, follower = cartesian_system
        model = system.model

        goal = {name: 0.0 for name in model.movable_joint_names}
        goal.update(
            {
                "shoulder_roll_left_dof": 0.3,
                "elbow_pitch_left_dof": -0.4,
                "shoulder_roll_right_dof": -0.3,
                "elbow_pitch_right_dof": -0.4,
            }
        )
        q_goal = model.q_from_positions(goal)
        left = model.frame_pose(q_goal, "left_tcp")
        right = model.frame_pose(q_goal, "right_tcp")

        system.configure()
        system.align()
        system.prepare_running()
        system.set_target(
            DualArmTarget(
                left_target=left, right_target=right, torso_policy=TorsoPolicy.OPTIMIZE
            )
        )

        dt = 0.01
        for _ in range(600):
            follower.step(dt)
            system.loop.run_once(dt)

        reached = {
            urdf: follower.joint(motor).position_rad for motor, urdf in MOTOR_TO_URDF.items()
        }
        pose = model.frame_pose(model.q_from_positions(reached), "left_tcp")
        np.testing.assert_allclose(pose[:3, 3], left[:3, 3], atol=5e-3)
        assert system.manager.faults == ()

    def test_no_target_means_no_new_motion(self, cartesian_system) -> None:
        system, _leader, follower = cartesian_system
        system.configure()
        system.align()
        system.prepare_running()
        before = follower.registers("torso_yaw").goal_position_count
        for _ in range(5):
            system.loop.run_once(0.01)
        assert follower.registers("torso_yaw").goal_position_count == before
        assert system.last_ik_result is None

    def test_an_expired_target_issues_no_new_motion(self, cartesian_system) -> None:
        from robopy.control.types import monotonic_ns

        system, _leader, follower = cartesian_system
        system.configure()
        system.align()
        system.prepare_running()
        before = follower.registers("torso_yaw").goal_position_count
        system.set_target(
            DualArmTarget(
                left_target=np.eye(4),
                right_target=np.eye(4),
                expiry_ns=monotonic_ns() - 1_000_000,
            )
        )
        system.loop.run_once(0.01)
        assert follower.registers("torso_yaw").goal_position_count == before

    def test_the_head_motors_are_never_commanded(self, cartesian_system) -> None:
        system, _leader, follower = cartesian_system
        model = system.model
        goal = {name: 0.0 for name in model.movable_joint_names}
        goal["shoulder_roll_left_dof"] = 0.3
        q_goal = model.q_from_positions(goal)

        system.configure()
        system.align()
        system.prepare_running()
        head_goal_before = follower.registers("head_yaw").goal_position_count
        system.set_target(
            DualArmTarget(
                left_target=model.frame_pose(q_goal, "left_tcp"),
                right_target=model.frame_pose(q_goal, "right_tcp"),
                torso_policy=TorsoPolicy.OPTIMIZE,
            )
        )
        for _ in range(50):
            follower.step(0.01)
            system.loop.run_once(0.01)

        assert follower.registers("head_yaw").goal_position_count == head_goal_before
        assert "head_yaw" not in system.last_ik_result.joint_targets_rad

    def test_the_model_is_built_with_the_declared_tcp_offset(self, cartesian_system) -> None:
        system, _leader, _follower = cartesian_system
        q = system.model.neutral_q()
        parent = system.model.frame_pose(q, SYNTHETIC_TCP_FRAMES["left"])[:3, 3]
        tcp = system.model.frame_pose(q, "left_tcp")[:3, 3]
        np.testing.assert_allclose(tcp - parent, [0.0, 0.0, -0.02], atol=1e-12)


class TestConfigurationErrors:
    def test_a_urdf_joint_claimed_by_no_motor_is_reported(self, synthetic_urdf: Path) -> None:
        config = _config(synthetic_urdf)
        config.follower_joint_calibration["torso_yaw"].urdf_joint = None
        system = RakudaControlSystem.from_buses(config, _bus(), _bus())
        with pytest.raises(Exception, match="No follower motor is mapped"):
            system._urdf_targets_to_motors({SYNTHETIC_TORSO_JOINT: 0.1})  # noqa: SLF001

    def test_a_fixed_joint_cannot_be_named_as_a_urdf_joint(self, synthetic_urdf: Path) -> None:
        config = _config(synthetic_urdf)
        config.follower_joint_calibration["torso_yaw"].urdf_joint = "gripper_left_dof"
        with pytest.raises(Exception, match="Unknown or non-movable URDF joint"):
            RakudaControlSystem.from_buses(config, _bus(), _bus())

    def test_a_missing_urdf_path_is_reported(self, synthetic_urdf: Path) -> None:
        config = _config(synthetic_urdf)
        config.model.urdf_path = None
        with pytest.raises(ValueError, match="urdf_path is not set"):
            RakudaControlSystem.from_buses(config, _bus(), _bus())
