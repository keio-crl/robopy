"""Factory functions creating KinematicChain for each supported robot."""

from .chain import KinematicChain, RevoluteJoint
from .transforms import translation


def so101_chain() -> KinematicChain:
    """Create the kinematic chain for the SO-101 robot (5-DOF, excluding gripper).

    Parameters are derived from the SO-101 URDF
    (``SO-ARM100/Simulation/SO101/so101_new_calib.urdf``).

    Joint axes follow the URDF convention:
        shoulder_pan / wrist_roll → Y-axis rotation,
        shoulder_lift / elbow_flex / wrist_flex → X-axis rotation.
    """
    joints = [
        RevoluteJoint(
            name="shoulder_pan",
            parent_to_joint=translation(0.0388, 0.0, 0.0624),
            axis="y",
            lower_limit_rad=-1.91986,  # -110 deg
            upper_limit_rad=1.91986,  # +110 deg
        ),
        RevoluteJoint(
            name="shoulder_lift",
            parent_to_joint=translation(-0.0304, -0.0183, -0.0542),
            axis="x",
            lower_limit_rad=-1.74533,  # -100 deg
            upper_limit_rad=1.74533,  # +100 deg
        ),
        RevoluteJoint(
            name="elbow_flex",
            parent_to_joint=translation(-0.11257, -0.028, 0.0),
            axis="x",
            lower_limit_rad=-1.69,  # -96.8 deg
            upper_limit_rad=1.69,  # +96.8 deg
        ),
        RevoluteJoint(
            name="wrist_flex",
            parent_to_joint=translation(-0.1349, 0.0052, 0.0),
            axis="x",
            lower_limit_rad=-1.65806,  # -95 deg
            upper_limit_rad=1.65806,  # +95 deg
        ),
        RevoluteJoint(
            name="wrist_roll",
            parent_to_joint=translation(0.0, -0.0611, 0.0181),
            axis="y",
            lower_limit_rad=-2.74385,  # -157.2 deg
            upper_limit_rad=2.84121,  # +162.8 deg
        ),
    ]

    # Fixed transform from wrist_roll frame to end-effector tip
    # (gripper_frame_joint offset from the URDF).
    ee_fixed = translation(-0.0079, 0.0, -0.0981)

    return KinematicChain(joints=joints, ee_fixed_transform=ee_fixed)


def koch_chain() -> KinematicChain:
    """Create the kinematic chain for the Koch v1.1 robot (5-DOF, excluding gripper).

    .. warning::

        Koch has **no official URDF**.  The parameters below are rough
        approximations from MuJoCo menagerie and CAD drawings.  Every numeric
        constant is marked with ``TODO(physical-params)`` and **must** be
        validated against the physical robot before production use.

    TODO(physical-params): Validate shoulder_pan offset against real robot.
    TODO(physical-params): Measure exact upper_arm length (currently ~110 mm est.).
    TODO(physical-params): Measure exact forearm length (currently ~100 mm est.).
    TODO(physical-params): Validate wrist offset and EE frame position.
    TODO(physical-params): Measure and confirm joint axis directions.
    TODO(physical-params): Calibrate joint limit values against real hardware.
    """
    joints = [
        RevoluteJoint(
            name="shoulder_pan",
            # TODO(physical-params): Approximate. Measure base-to-shoulder offset.
            parent_to_joint=translation(0.0, 0.0, 0.05),
            axis="y",  # TODO(physical-params): Confirm axis direction
            lower_limit_rad=-2.61799,  # -150 deg (estimate)
            upper_limit_rad=2.61799,  # +150 deg (estimate)
        ),
        RevoluteJoint(
            name="shoulder_lift",
            # TODO(physical-params): Approximate shoulder-to-upper-arm offset.
            parent_to_joint=translation(0.0, 0.0, -0.04),
            axis="x",  # TODO(physical-params): Confirm axis direction
            lower_limit_rad=-1.74533,  # -100 deg (estimate)
            upper_limit_rad=1.74533,  # +100 deg (estimate)
        ),
        RevoluteJoint(
            name="elbow",
            # Upper arm length ~110 mm
            # TODO(physical-params): Measure precisely.
            parent_to_joint=translation(-0.110, 0.0, 0.0),
            axis="x",  # TODO(physical-params): Confirm axis direction
            lower_limit_rad=-1.74533,  # -100 deg (estimate)
            upper_limit_rad=1.74533,  # +100 deg (estimate)
        ),
        RevoluteJoint(
            name="wrist_flex",
            # Forearm length ~100 mm
            # TODO(physical-params): Measure precisely.
            parent_to_joint=translation(-0.100, 0.0, 0.0),
            axis="x",  # TODO(physical-params): Confirm axis direction
            lower_limit_rad=-1.74533,  # -100 deg (estimate)
            upper_limit_rad=1.74533,  # +100 deg (estimate)
        ),
        RevoluteJoint(
            name="wrist_roll",
            # TODO(physical-params): Approximate wrist-to-gripper offset.
            parent_to_joint=translation(0.0, -0.04, 0.0),
            axis="y",  # TODO(physical-params): Confirm axis direction
            lower_limit_rad=-3.14159,  # -180 deg (estimate)
            upper_limit_rad=3.14159,  # +180 deg (estimate)
        ),
    ]

    # TODO(physical-params): Measure precise EE offset from wrist_roll.
    ee_fixed = translation(0.0, 0.0, -0.06)

    return KinematicChain(joints=joints, ee_fixed_transform=ee_fixed)
