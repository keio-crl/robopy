"""VR teleoperation of the Rakuda model, driven by a scripted "operator".

The real thing is the ``robopy-vr`` command plus a headset.  This script shows
the same pipeline without either: it feeds :class:`~robopy.vr.server.TeleopSession`
the messages the page would send -- a headset that turns its head, a left
controller that squeezes the clutch and pushes forward -- and prints what the
model does.  Everything is simulated; nothing here talks to a motor.

    uv run --extra kinematics python examples/robot/rakuda_vr_teleop.py
    uv run --extra kinematics python examples/robot/rakuda_vr_teleop.py --synthetic

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
from robopy.vr.__main__ import STREAMING_IK_OVERRIDES, DEFAULT_START_POSE
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
    """A left controller 30 cm to the operator's left; WebXR -Z is forward."""
    return {"p": [-0.3, 1.2, z], "q": [0, 0, 0, 1], "clutch": clutch, "trigger": 0.0}


def main(argv: List[str] | None = None) -> int:
    """Run the scripted session."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_model_arguments(parser)
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

        # 1. The operator looks around: 30 degrees left, then 15 degrees down.
        t = 0.0
        session.handle({"type": "pose", "head": headset(0.0), "left": None, "right": None}, t)
        for target_yaw, target_pitch in ((math.radians(30), 0.0), (0.0, math.radians(-15))):
            for _ in range(60):
                t += 1 / 60
                state = session.handle(
                    {"type": "pose", "head": headset(target_yaw, target_pitch), "left": None, "right": None},
                    t,
                )
            assert state is not None
            joints = state["joints"]
            print(
                f"headset yaw {math.degrees(target_yaw):+5.1f} pitch {math.degrees(target_pitch):+5.1f} deg"
                f"  ->  {yaw_joint}={joints[yaw_joint]:+.3f} {pitch_joint}={joints[pitch_joint]:+.3f} rad"
            )

        # 2. Squeeze the left clutch and push 8 cm forward over one second.
        hand0 = backend.hand_pose("left")[:3, 3].copy()
        for i in range(120):
            t += 1 / 60
            z = -0.4 - 0.08 * min(i, 60) / 60
            state = session.handle(
                {"type": "pose", "head": headset(0.0), "left": controller(z, clutch=True), "right": None},
                t,
            )
        assert state is not None
        moved = backend.hand_pose("left")[:3, 3] - hand0
        ik = state["ik"]
        print(
            f"left hand moved {1e3 * moved[0]:+.1f} mm forward, {1e3 * moved[1]:+.1f} left, "
            f"{1e3 * moved[2]:+.1f} up  (IK {ik['status']}, residual "
            f"{(ik['errors']['left_position_m'] or 0) * 1e3:.1f} mm)"
        )

        # 3. Release: the hand holds where it is.
        state = session.handle(
            {"type": "pose", "head": headset(0.0), "left": controller(-0.48, clutch=False), "right": None},
            t + 0.02,
        )
        assert state is not None
        print(f"released: clutched={state['arms']['left']['clutched']} enabled={state['arms']['left']['enabled']}")
        session.close()
    finally:
        loaded.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
