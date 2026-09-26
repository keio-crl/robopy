"""VR teleoperation of the Rakuda model, driven by a scripted "operator".

The real thing is the ``robopy-vr`` command plus a headset.  This script shows
the same pipeline without either: it feeds :class:`~robopy.vr.server.TeleopSession`
the messages the page would send -- a headset that turns its head, a left
controller whose X button is held while it sits in front of the operator --
and prints what the model does.  Everything is simulated; nothing here talks
to a motor.

The arm mapping is *absolute*: the headset position at re-centring stands for
the robot's head, and while X is held the left hand goes to where the
controller is in that correspondence (at a bounded speed).  Releasing X holds
the hand where it got to.

    uv run --extra kinematics python examples/robot/rakuda_vr_teleop.py
    uv run --extra kinematics python examples/robot/rakuda_vr_teleop.py --synthetic
    uv run --extra kinematics python examples/robot/rakuda_vr_teleop.py --hands

With ``--hands`` the operator uses a bare hand instead of the controller:
the page then sends the hand's joints (WebXR Hand Input) and the server reads
the pinch of thumb and index as the clutch.

To operate for real::

    robopy-vr --host 0.0.0.0 --cert cert.pem --key key.pem          # simulated robot
    robopy-vr --host 0.0.0.0 --cert cert.pem --key key.pem --config --hardware

and open ``https://<pc>:8766/vr`` in the headset's browser.
"""

from __future__ import annotations

import argparse
import math
from typing import Any, Dict, List

from robopy.viewer.cli import add_model_arguments, load_model
from robopy.vr.__main__ import DEFAULT_START_POSE, STREAMING_IK_OVERRIDES
from robopy.vr.arm_teleop import ArmTeleopConfig, DualArmTeleop
from robopy.vr.backend import SimulationBackend
from robopy.vr.head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig
from robopy.vr.server import TeleopSession, VRServerConfig


def headset(yaw: float, pitch: float = 0.0) -> Dict[str, Any]:
    """A WebXR head pose: yaw about +Y (left positive), pitch about +X (up positive)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return {"p": [0.0, 1.6, 0.0], "q": [cy * sp, sy * cp, -sy * sp, cy * cp]}


def controller(z: float, *, clutch: bool) -> Dict[str, Any]:
    """A left controller 15 cm to the operator's left, 1.3 m up; WebXR -Z is forward.

    ``clutch`` is what the page sends while the X button is held.  The offsets
    are small because the Rakuda is small: its hands hang about 35 cm below
    its head, a person's about 70 cm (``--position-scale 0.5`` on the command
    line makes the two agree).
    """
    return {"p": [-0.15, 1.3, z], "q": [0, 0, 0, 1], "clutch": clutch, "trigger": 0.0}


def tracked_hand(z: float, *, pinch: bool) -> Dict[str, Any]:
    """A tracked left hand in place of the controller: what the page sends per frame.

    Only the joints the default gestures need are laid out here (the real page
    sends all 25): the wrist with its orientation, the thumb and index tips
    (5 mm apart when pinching, 8 cm otherwise), the middle knuckle for the
    palm reference point, and the middle, ring and little fingers straight,
    i.e. the gripper open.  The palm centre lands where the controller was.
    """
    w = [-0.15, 1.3, z + 0.045]  # palm = midpoint(wrist, middle knuckle) = the controller's spot
    joints: Dict[str, Any] = {"wrist": {"p": w, "q": [0, 0, 0, 1]}}
    for finger, x in (("index", -0.03), ("middle", 0.0), ("ring", 0.03), ("pinky", 0.06)):
        knuckle = [w[0] + x, w[1], w[2] - 0.09]
        for name, dz in (
            ("phalanx-proximal", 0.0),
            ("phalanx-intermediate", -0.04),
            ("phalanx-distal", -0.07),
            ("tip", -0.10),
        ):
            joints[f"{finger}-finger-{name}"] = {"p": [knuckle[0], knuckle[1], knuckle[2] + dz]}
    index_tip = joints["index-finger-tip"]["p"]
    joints["thumb-tip"] = {
        "p": [index_tip[0] - (0.005 if pinch else 0.08), index_tip[1], index_tip[2]]
    }
    return {"hand": {"joints": joints}}


def main(argv: List[str] | None = None) -> int:
    """Run the scripted session."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_model_arguments(parser)
    parser.add_argument(
        "--hands", action="store_true", help="drive the arm with a tracked hand (a pinch)"
    )
    args = parser.parse_args(argv)

    loaded = load_model(args, parser, ik_overrides=STREAMING_IK_OVERRIDES)
    try:
        bundle = loaded.bundle
        model = bundle.model
        start = {k: v for k, v in DEFAULT_START_POSE.items() if k in model.movable_joint_names}
        backend = SimulationBackend(bundle, loaded.ik, initial_positions=start)

        head_joints = [n for n in model.movable_joint_names if "head" in n]
        yaw_joint = next(n for n in head_joints if "yaw" in n)
        pitch_joint = next(n for n in head_joints if "pitch" in n)
        camera = "head_camera_link" if model.has_frame("head_camera_link") else None
        torso_joint = next((n for n in model.movable_joint_names if "torso" in n), None)
        mapping = HeadJointMapping.from_model(
            model, yaw_joint, pitch_joint, camera_frame=camera, torso_joint=torso_joint
        )
        print(
            f"head mapping: {yaw_joint} sign {mapping.yaw_sign:+d} neutral "
            f"{mapping.yaw_neutral_rad:+.3f}; {pitch_joint} sign {mapping.pitch_sign:+d} "
            f"neutral {mapping.pitch_neutral_rad:+.3f}  ({mapping.forward_source})"
        )
        for note in mapping.notes:
            print(f"  note: {note}")
        print(
            f"torso compensation: head yaw += {mapping.torso_coupling:+.2f} x {mapping.torso_joint}"
        )

        session = TeleopSession(
            backend,
            head_tracker=HeadTracker(mapping, HeadTrackingConfig(filter_hz=None)),
            arm_teleop=DualArmTeleop(ArmTeleopConfig(), ArmTeleopConfig()),
            config=VRServerConfig(state_hz=1000.0),
            bundle=bundle,
        )
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None
        print(f"backend: {hello['backend']}  arms: {'yes' if hello['arms'] else 'no'}")
        anchor = hello["robot_anchor_m"]
        print(
            f"arm mapping: {hello['arms']['left']['mapping']}, clutch button "
            f"{hello['clutch_button'].upper()}/X; robot head anchor at "
            f"({anchor[0]:+.3f}, {anchor[1]:+.3f}, {anchor[2]:+.3f}) m in the base frame"
        )

        # 1. The operator looks around: 30 degrees left, then 15 degrees down.
        t = 0.0
        session.handle({"type": "pose", "head": headset(0.0), "left": None, "right": None}, t)
        for target_yaw, target_pitch in ((math.radians(30), 0.0), (0.0, math.radians(-15))):
            for _ in range(60):
                t += 1 / 60
                state = session.handle(
                    {
                        "type": "pose",
                        "head": headset(target_yaw, target_pitch),
                        "left": None,
                        "right": None,
                    },
                    t,
                )
            assert state is not None
            joints = state["joints"]
            print(
                f"headset yaw {math.degrees(target_yaw):+5.1f} "
                f"pitch {math.degrees(target_pitch):+5.1f} deg  ->  "
                f"{yaw_joint}={joints[yaw_joint]:+.3f} {pitch_joint}={joints[pitch_joint]:+.3f} rad"
            )

        # 2. Hold X (or pinch) with the controller (or palm) 20 cm ahead of,
        #    15 cm left of and 30 cm below the headset (which is 1.6 m up).
        #    The hand target is the same offset from the robot's head anchor;
        #    the hand slews there.
        hand0 = backend.hand_pose("left")[:3, 3].copy()
        for _ in range(180):
            t += 1 / 60
            state = session.handle(
                {
                    "type": "pose",
                    "head": headset(0.0),
                    "left": tracked_hand(-0.2, pinch=True)
                    if args.hands
                    else controller(-0.2, clutch=True),
                    "right": None,
                },
                t,
            )
        assert state is not None
        if args.hands:
            left = state["arms"]["left"]
            print(f"input: {left['input']}  gesture: {left['hand']}")
        hand = backend.hand_pose("left")[:3, 3]
        target = state["arms"]["left"]["target"]["p"]
        ik = state["ik"]
        print(
            f"target = anchor + (+0.20, +0.15, -0.30) = ({target[0]:+.3f}, {target[1]:+.3f}, "
            f"{target[2]:+.3f}) m; hand went from "
            f"({hand0[0]:+.3f}, {hand0[1]:+.3f}, {hand0[2]:+.3f}) "
            f"to ({hand[0]:+.3f}, {hand[1]:+.3f}, {hand[2]:+.3f})  (IK {ik['status']}, residual "
            f"{(ik['errors']['left_position_m'] or 0) * 1e3:.1f} mm)"
        )

        # 3. Release X (open the pinch): the hand holds where it is, wherever
        #    the controller goes.
        state = session.handle(
            {
                "type": "pose",
                "head": headset(0.0),
                "left": tracked_hand(-0.9, pinch=False)
                if args.hands
                else controller(-0.9, clutch=False),
                "right": None,
            },
            t + 0.02,
        )
        assert state is not None
        left = state["arms"]["left"]
        print(f"released: clutched={left['clutched']} enabled={left['enabled']}")
        session.close()
    finally:
        loaded.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
