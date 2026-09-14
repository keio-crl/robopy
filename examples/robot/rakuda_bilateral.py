"""Joint-space bilateral coupling on a simulated Rakuda pair.

Run it as-is and nothing is connected and no current reaches a motor: both
buses are :class:`~robopy.motor.sim_dynamixel_bus.SimulatedDynamixelBus`
instances with a simple plant behind them.

    uv run python examples/robot/rakuda_bilateral.py
    uv run python examples/robot/rakuda_bilateral.py --contact

Running it against real hardware is deliberately *not* a flag on this script.
It needs, at minimum:

1. a measured ``zero_count``, travel limit, torque constant and current limit
   for every coupled joint, written into ``.robopy/rakuda/config.yaml`` with
   ``validated: true``;
2. ``control.allow_hardware_current_output: true``, set knowingly;
3. a validated gravity model for each machine, or joints that are mechanically
   supported -- current control without compensation can drop a joint under its
   own weight;
4. a stop policy chosen against the actual mechanism.

With those in place the same stack runs from
``RakudaPairSys.start_control()``; see ``docs/robots/rakuda.md``.
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List

from robopy.config.robot_config.rakuda_config import (
    RakudaBilateralConfig,
    RakudaControlConfig,
    RakudaJointCalibrationSpec,
)
from robopy.motor.dynamixel_bus import DynamixelMotor
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint
from robopy.robots.rakuda.rakuda_control import RakudaControlSystem

# A small subset of the thirteen arm-and-torso joints, enough to show the
# coupling without turning the example into a wall of numbers.
COUPLED = ("torso_yaw", "r_arm_sh_pitch1", "l_arm_sh_pitch1")

LEADER_MODEL = "xc330-t288"
FOLLOWER_MODEL = "xm430-w350"


def make_bus(model: str, *, contact_at_rad: float | None = None) -> SimulatedDynamixelBus:
    """Build a simulated bus carrying :data:`COUPLED`.

    Args:
        model: DYNAMIXEL model name for every motor on this bus.
        contact_at_rad: When set, the torso meets a stiff obstacle at this angle.
    """
    motors = {name: DynamixelMotor(index + 1, name, model) for index, name in enumerate(COUPLED)}
    joints: Dict[str, SimulatedJoint] = {name: SimulatedJoint() for name in COUPLED}
    if contact_at_rad is not None:
        joints["torso_yaw"] = SimulatedJoint(
            contact_position_rad=contact_at_rad,
            contact_stiffness_nm_per_rad=120.0,
        )
    return SimulatedDynamixelBus(motors, joints=joints, auto_step=False)


def make_config() -> RakudaControlConfig:
    """A fully measured configuration -- for the *simulated* machine.

    Every number here describes the simulated plant above.  None of it is a
    measurement of any real Rakuda, which is why the real thing needs its own
    calibration before any of this can be used.
    """
    calibration = {
        name: RakudaJointCalibrationSpec(
            direction=1,
            zero_count=2048,
            lower_limit_rad=-2.0,
            upper_limit_rad=2.0,
            max_velocity_rad_s=3.0,
            torque_constant_nm_per_a=1.5,
            current_limit_a=1.0,
            validated=True,
            notes="simulated plant; not a measurement of any real machine",
        )
        for name in COUPLED
    }
    return RakudaControlConfig(
        mode="bilateral_joint",
        control_period_s=0.002,
        leader_joint_calibration=dict(calibration),
        follower_joint_calibration=dict(calibration),
        bilateral=RakudaBilateralConfig(
            coupled_motors=list(COUPLED),
            stiffness_nm_per_rad=2.0,
            damping_nm_s_per_rad=0.05,
            max_torque_nm=1.0,
            max_torque_rate_nm_s=50.0,
            ramp_time_s=0.2,
            velocity_filter_hz=50.0,
            leader_current_limit_a={name: 0.5 for name in COUPLED},
            follower_current_limit_a={name: 1.0 for name in COUPLED},
            # The simulated plant has no gravity, so running uncompensated is
            # correct here -- and it is stated rather than assumed.
            allow_uncompensated=True,
        ),
        # Safe only because both "machines" are simulated.
        allow_hardware_current_output=True,
    )


def main(argv: List[str] | None = None) -> int:
    """Run the simulated bilateral demonstration."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--contact",
        action="store_true",
        help="give the follower's torso a stiff obstacle at 0.12 rad",
    )
    parser.add_argument("--seconds", type=float, default=3.0, help="simulated duration")
    args = parser.parse_args(argv)

    leader_bus = make_bus(LEADER_MODEL)
    follower_bus = make_bus(FOLLOWER_MODEL, contact_at_rad=0.12 if args.contact else None)

    system = RakudaControlSystem.from_buses(
        make_config(),
        leader_bus,
        follower_bus,
        leader_torque_enabled=list(COUPLED),
        follower_torque_enabled=list(COUPLED),
    )

    system.configure()
    print("configured:", system.manager.state.value)
    alignment = system.align()
    print(f"worst alignment error: {alignment['worst_alignment_error_rad']:.6f} rad")

    system.prepare_running()
    dt = system.loop.control_period_s
    steps = int(args.seconds / dt)
    print(f"running {steps} cycles of {dt * 1e3:.1f} ms ...")

    for step in range(steps):
        # Stand in for the operator: sweep the leader's torso, then hold.
        elapsed = step * dt
        leader_bus.joint("torso_yaw").position_rad = 0.3 * min(1.0, elapsed / 1.0)
        leader_bus.joint("torso_yaw").velocity_rad_s = 0.3 if elapsed < 1.0 else 0.0
        leader_bus.step(dt)
        follower_bus.step(dt)
        system.loop.run_once(dt)

    leader_angle = leader_bus.joint("torso_yaw").position_rad
    follower_angle = follower_bus.joint("torso_yaw").position_rad
    leader_current_raw = leader_bus.registers("torso_yaw").goal_current_raw
    leader_current_a = leader_current_raw * leader_bus.capabilities("torso_yaw").current_unit_a

    print(f"leader torso   : {leader_angle:+.4f} rad ({math.degrees(leader_angle):+.2f} deg)")
    print(f"follower torso : {follower_angle:+.4f} rad ({math.degrees(follower_angle):+.2f} deg)")
    print(f"tracking error : {follower_angle - leader_angle:+.4f} rad")
    print(f"leader current : {leader_current_a:+.4f} A (the force fed back to the operator)")
    if args.contact:
        print("  the obstacle stops the follower, so the operator feels a resisting torque.")

    for record in system.loop.drain_logs()[-1:]:
        print("last cycle:", record)
    print("stopping:", "; ".join(system.stop()))

    report = system.report()
    print(f"configured rate : {report['timing']['configured_rate_hz']:.0f} Hz")
    print(
        "measured rate   : not meaningful here -- this example steps the loop by hand rather "
        "than running it against a clock."
    )
    print("faults:", report["faults"] or "none")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
