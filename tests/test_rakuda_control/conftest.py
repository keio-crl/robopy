"""Shared fixtures for the Rakuda control tests.

Nothing here touches a serial port.  The tests that need a kinematic model are
skipped when the optional ``kinematics`` extra is not installed, which is also
how the "robopy still works without the extra" requirement is exercised.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Sequence

import numpy as np
import pytest

from robopy.control.joint_mapping import JointCalibration, JointMap
from robopy.control.types import JointState, monotonic_ns
from robopy.motor.dynamixel_bus import DynamixelMotor
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint

COUPLED_MOTORS = ("torso_yaw", "r_arm_sh_pitch1", "l_arm_sh_pitch1")


def make_joint_state(
    joint_names: Sequence[str],
    positions: Dict[str, float] | None = None,
    velocities: Dict[str, float] | None = None,
    currents: Dict[str, float] | None = None,
    *,
    valid: Dict[str, bool] | None = None,
    generation: int = 0,
    sequence: int = 0,
    age_ns: int = 0,
) -> JointState:
    """Build a :class:`JointState` for tests, defaulting every array to zero."""
    names = tuple(joint_names)
    now = monotonic_ns() - age_ns
    return JointState(
        joint_names=names,
        position_rad=np.asarray([(positions or {}).get(n, 0.0) for n in names], dtype=float),
        velocity_rad_s=np.asarray([(velocities or {}).get(n, 0.0) for n in names], dtype=float),
        current_a=np.asarray([(currents or {}).get(n, 0.0) for n in names], dtype=float),
        valid=np.asarray([(valid or {}).get(n, True) for n in names], dtype=bool),
        read_start_ns=now,
        read_end_ns=now,
        sequence=sequence,
        mode_generation=generation,
    )


def make_calibration(
    motor_names: Sequence[str],
    model: str = "xm430-w350",
    *,
    validated: bool = True,
    direction: int = 1,
) -> JointMap:
    """A fully measured joint map, for tests that need current output allowed."""
    return JointMap(
        [
            JointCalibration(
                motor_name=name,
                motor_id=index + 1,
                model=model,
                direction=direction,
                zero_count=2048,
                lower_limit_rad=-2.0,
                upper_limit_rad=2.0,
                max_velocity_rad_s=3.0,
                max_acceleration_rad_s2=20.0,
                torque_constant_nm_per_a=1.5,
                current_limit_a=1.0,
                validated=validated,
            )
            for index, name in enumerate(motor_names)
        ]
    )


def make_sim_bus(
    motor_names: Sequence[str] = COUPLED_MOTORS,
    model: str = "xm430-w350",
) -> SimulatedDynamixelBus:
    """A simulated bus carrying ``motor_names``, with the plant held still."""
    motors = {
        name: DynamixelMotor(index + 1, name, model) for index, name in enumerate(motor_names)
    }
    joints = {name: SimulatedJoint() for name in motor_names}
    return SimulatedDynamixelBus(motors, joints=joints, auto_step=False)


@pytest.fixture
def sim_bus() -> SimulatedDynamixelBus:
    """A simulated follower bus."""
    return make_sim_bus()


@pytest.fixture
def joint_map() -> JointMap:
    """A fully measured joint map for :data:`COUPLED_MOTORS`."""
    return make_calibration(COUPLED_MOTORS)


@pytest.fixture(scope="session")
def synthetic_urdf(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The synthetic dual-arm URDF, written once per session."""
    from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

    return write_synthetic_dual_arm_urdf(tmp_path_factory.mktemp("model") / "synthetic.urdf")


@pytest.fixture
def whole_body_model(synthetic_urdf: Path):  # type: ignore[no-untyped-def]
    """A loaded model with TCP frames, soft limits and collision geometry.

    Skipped when the ``kinematics`` extra is not installed.
    """
    pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
    from robopy.kinematics.synthetic_dual_arm import (
        SYNTHETIC_ARM_JOINTS,
        SYNTHETIC_TCP_FRAMES,
        SYNTHETIC_TORSO_JOINT,
    )
    from robopy.kinematics.urdf_model import WholeBodyModel

    model = WholeBodyModel.from_urdf(synthetic_urdf, build_collision=True)
    offset = np.eye(4)
    offset[2, 3] = -0.02
    model.add_fixed_frame("left_tcp", SYNTHETIC_TCP_FRAMES["left"], offset)
    model.add_fixed_frame("right_tcp", SYNTHETIC_TCP_FRAMES["right"], offset)
    model.set_soft_limits(
        {
            SYNTHETIC_TORSO_JOINT: (-1.5, 1.5),
            SYNTHETIC_ARM_JOINTS["left"][0]: (-2.0, 2.0),
            SYNTHETIC_ARM_JOINTS["right"][0]: (-2.0, 2.0),
        }
    )
    groups = model.classify_collision_pairs(model.neutral_q())
    model.add_all_collision_pairs(
        excluded=groups["parent_child"] + groups["same_body"] + groups["interfering_at_q"]
    )
    return model


@pytest.fixture
def dual_arm_ik(whole_body_model):  # type: ignore[no-untyped-def]
    """A solver bound to :func:`whole_body_model`."""
    pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")
    from robopy.kinematics.dual_arm_ik import DualArmIK, DualArmIKConfig
    from robopy.kinematics.synthetic_dual_arm import (
        SYNTHETIC_ARM_JOINTS,
        SYNTHETIC_HEAD_JOINTS,
        SYNTHETIC_TORSO_JOINT,
    )

    return DualArmIK(
        whole_body_model,
        left_frame="left_tcp",
        right_frame="right_tcp",
        torso_joint=SYNTHETIC_TORSO_JOINT,
        left_arm_joints=SYNTHETIC_ARM_JOINTS["left"],
        right_arm_joints=SYNTHETIC_ARM_JOINTS["right"],
        head_joints=SYNTHETIC_HEAD_JOINTS,
        config=DualArmIKConfig(max_joint_step_rad=0.08, compute_budget_s=5.0),
    )
