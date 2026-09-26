"""Dual-arm Cartesian teleoperation on a simulated Rakuda.

Both hands are driven from TCP targets solved together with the shared torso
yaw, in a single quadratic program.  Run as-is, nothing is connected: the buses
are simulated and the kinematic model is the synthetic fixture, because the
Rakuda CAD export is not part of this repository.

    uv run --extra kinematics python examples/robot/rakuda_cartesian_teleop.py
    uv run --extra kinematics python examples/robot/rakuda_cartesian_teleop.py --torso optimize

Needs the optional extra::

    uv sync --extra kinematics

To point it at the real model instead, pass ``--urdf`` -- but note the model
alone is not enough for the real machine:

* the motor-to-URDF-joint correspondence is a measurement, not a naming rule;
* the offset from ``gripper_left_dof`` / ``gripper_right_dof`` to the actual
  grasp centre has to be measured;
* the continuous joints (torso yaw, both shoulder pitches) carry no URDF range,
  so their real travel limits have to be supplied;
* the CAD export's masses are placeholders, so it is a geometry-only model.

Audit any model first::

    python -m robopy.kinematics.urdf_audit path/to/robot.urdf --package-dir path/to/pkgs
"""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path
from typing import Dict, List

import numpy as np

from robopy.config.robot_config.rakuda_config import (
    RakudaControlConfig,
    RakudaJointCalibrationSpec,
    RakudaModelConfig,
    RakudaTcpSpec,
    RakudaTrajectoryConfig,
)
from robopy.control.types import DualArmTarget, TorsoPolicy
from robopy.kinematics.synthetic_dual_arm import (
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_HEAD_JOINTS,
    SYNTHETIC_TCP_FRAMES,
    SYNTHETIC_TORSO_JOINT,
    write_synthetic_dual_arm_urdf,
)
from robopy.motor.dynamixel_bus import DynamixelMotor
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint
from robopy.robots.rakuda.rakuda_control import RakudaControlSystem

# Motor names for the simulated machine, and the URDF joint each one drives.
# On a real Rakuda this correspondence is established and checked on the
# machine; motor names such as `r_arm_sh_pitch2` do not map onto URDF names by
# any mechanical rule.
MOTOR_TO_URDF: Dict[str, str] = {
    "torso_yaw": SYNTHETIC_TORSO_JOINT,
    **{f"l_arm_{i}": name for i, name in enumerate(SYNTHETIC_ARM_JOINTS["left"])},
    **{f"r_arm_{i}": name for i, name in enumerate(SYNTHETIC_ARM_JOINTS["right"])},
    "head_yaw": SYNTHETIC_HEAD_JOINTS[0],
    "head_pitch": SYNTHETIC_HEAD_JOINTS[1],
}


def make_bus() -> SimulatedDynamixelBus:
    """A simulated bus carrying every motor in :data:`MOTOR_TO_URDF`."""
    motors = {
        name: DynamixelMotor(index + 1, name, "xm430-w350")
        for index, name in enumerate(MOTOR_TO_URDF)
    }
    joints = {name: SimulatedJoint() for name in MOTOR_TO_URDF}
    return SimulatedDynamixelBus(motors, joints=joints, auto_step=False)


def collision_exclusions(urdf: Path) -> List[tuple[str, str, str]]:
    """Work out which self-collision pairs to exclude, and record why.

    This is the intended workflow: the classifier groups candidate pairs by
    *reason*, and each group is excluded deliberately.  Excluding every adjacent
    or every nearby pair in one go would remove real self-collision checks along
    with the spurious ones, so each exclusion carries its justification into the
    configuration.
    """
    from robopy.kinematics.urdf_model import WholeBodyModel

    model = WholeBodyModel.from_urdf(urdf, build_collision=True, geometry_only=True)
    groups = model.classify_collision_pairs(model.neutral_q())
    reasons = {
        "same_body": "both geometries hang off the same joint",
        "parent_child": "directly connected bodies, touching at their shared joint",
        "interfering_at_q": "convex hulls overlap at the resting pose (hull over-approximation)",
    }
    excluded = [
        (pair[0], pair[1], reason)
        for group, reason in reasons.items()
        for pair in groups[group]
    ]
    print(
        f"collision pairs: {len(groups['other'])} kept, {len(excluded)} excluded "
        f"({', '.join(f'{len(groups[g])} {g}' for g in reasons)})"
    )
    return excluded


def make_config(
    urdf: Path,
    *,
    build_collision: bool,
    exclusions: List[tuple[str, str, str]] | None = None,
) -> RakudaControlConfig:
    """A configuration for the *simulated* machine and the synthetic model."""
    calibration = {
        motor: RakudaJointCalibrationSpec(
            urdf_joint=urdf_joint,
            direction=1,
            zero_count=2048,
            lower_limit_rad=-3.0,
            upper_limit_rad=3.0,
            max_velocity_rad_s=3.0,
            validated=True,
            notes="simulated plant; not a measurement of any real machine",
        )
        for motor, urdf_joint in MOTOR_TO_URDF.items()
    }
    return RakudaControlConfig(
        mode="cartesian_teleop",
        control_period_s=0.01,
        ik_period_s=5.0,
        leader_joint_calibration=dict(calibration),
        follower_joint_calibration=dict(calibration),
        model=RakudaModelConfig(
            urdf_path=str(urdf),
            torso_joint=SYNTHETIC_TORSO_JOINT,
            left_arm_joints=list(SYNTHETIC_ARM_JOINTS["left"]),
            right_arm_joints=list(SYNTHETIC_ARM_JOINTS["right"]),
            # The head is modelled -- it moves collision geometry -- but is
            # never a decision variable of the arm IK.
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
            # Continuous joints have no URDF range, so a real limit is required.
            soft_limits_rad={
                SYNTHETIC_TORSO_JOINT: (-1.5, 1.5),
                "shoulder_pitch_left_dof": (-2.0, 2.0),
                "shoulder_pitch_right_dof": (-2.0, 2.0),
            },
            build_collision=build_collision,
            collision_exclusions=list(exclusions or []),
            geometry_only=True,
        ),
        # The hands track a reference that moves under these ceilings; the
        # machine refuses Cartesian mode without them. Simulated plant values:
        # measure the real arm's before using them there.
        trajectory=RakudaTrajectoryConfig(
            max_linear_velocity_m_s=0.25,
            max_linear_acceleration_m_s2=1.0,
            max_angular_velocity_rad_s=1.5,
            max_angular_acceleration_rad_s2=6.0,
            lag_tolerance_m=0.02,
        ),
    )


def main(argv: List[str] | None = None) -> int:
    """Run the simulated Cartesian teleoperation demonstration."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--torso",
        choices=[policy.value for policy in TorsoPolicy],
        default="optimize",
        help="how the shared torso yaw is treated",
    )
    parser.add_argument("--urdf", type=Path, default=None, help="use this URDF instead")
    parser.add_argument("--collision", action="store_true", help="load collision geometry")
    parser.add_argument("--seconds", type=float, default=4.0, help="simulated duration")
    args = parser.parse_args(argv)

    with tempfile.TemporaryDirectory() as tmp:
        urdf = args.urdf or write_synthetic_dual_arm_urdf(Path(tmp) / "synthetic.urdf")
        if args.urdf is None:
            print(f"using the synthetic fixture at {urdf}")
            print("  (it has Rakuda's topology, not Rakuda's geometry)")

        config = make_config(
            urdf,
            build_collision=args.collision,
            exclusions=collision_exclusions(urdf) if args.collision else None,
        )
        follower_bus = make_bus()
        system = RakudaControlSystem.from_buses(config, make_bus(), follower_bus)
        model = system.model

        # Pick a reachable pose with forward kinematics, then ask the IK to get
        # back to it from zero. A target invented by hand would confound "the
        # solver failed" with "the target was unreachable".
        goal = {name: 0.0 for name in model.movable_joint_names}
        goal.update(
            {
                "shoulder_roll_left_dof": 0.30,
                "elbow_pitch_left_dof": -0.45,
                "shoulder_roll_right_dof": -0.30,
                "elbow_pitch_right_dof": -0.45,
            }
        )
        q_goal = model.q_from_positions(goal)
        left_target = model.frame_pose(q_goal, "left_tcp")
        right_target = model.frame_pose(q_goal, "right_tcp")

        system.configure()
        system.align()
        system.prepare_running()

        policy = TorsoPolicy(args.torso)
        system.set_target(
            DualArmTarget(
                left_target=left_target,
                right_target=right_target,
                torso_policy=policy,
                torso_velocity_rad_s=0.1 if policy is TorsoPolicy.MANUAL else 0.0,
            )
        )

        dt = system.loop.control_period_s
        steps = int(args.seconds / dt)
        print(f"torso policy: {policy.value}; running {steps} cycles of {dt * 1e3:.0f} ms ...")
        for _ in range(steps):
            follower_bus.step(dt)
            system.loop.run_once(dt)

        reached = {
            urdf_joint: follower_bus.joint(motor).position_rad
            for motor, urdf_joint in MOTOR_TO_URDF.items()
        }
        q_reached = model.q_from_positions(reached)
        result = system.last_ik_result

        print(f"IK status          : {result.status.value}")
        for side in ("left", "right"):
            achieved = model.frame_pose(q_reached, f"{side}_tcp")[:3, 3]
            wanted = (left_target if side == "left" else right_target)[:3, 3]
            error = float(np.linalg.norm(achieved - wanted))
            print(f"{side:<6} TCP error  : {error * 1e3:7.3f} mm  at {np.round(achieved, 4)}")
        print(f"torso angle        : {reached[SYNTHETIC_TORSO_JOINT]:+.4f} rad")
        print(
            "head angles        : "
            f"{[round(reached[name], 6) for name in SYNTHETIC_HEAD_JOINTS]} "
            "(never commanded by the arm IK)"
        )
        if result.min_collision_distance_m is not None:
            print(f"min self-distance  : {result.min_collision_distance_m * 1e3:.1f} mm")
        else:
            print("min self-distance  : not modelled (pass --collision)")
        print(f"IK solve time      : {result.compute_time_s * 1e3:.2f} ms")
        print("stopping:", "; ".join(system.stop()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
