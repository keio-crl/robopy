"""Head-only VR demo: the headset turns the real Rakuda's head, its camera fills the view.

The arms are not driven at all.  The follower bus is written for the two head
motors only, straight in encoder counts around the pose the robot has when
the script starts, so no joint calibration and no leader are needed.  The
RealSense on the head streams into the headset.

Real machine (the head WILL move; the arms keep the torque the config gives
them):

    uv run --extra kinematics robopy-vr --hardware-head --follower-port /dev/ttyUSB1 \\
        --camera realsense --host 0.0.0.0 --self-signed

This script is the same wiring through the API, on a simulated follower bus
and a test-pattern camera so it runs anywhere:

    uv run --extra kinematics python examples/robot/rakuda_vr_head_camera.py

Pass ``--follower-port /dev/ttyUSB1`` to use the real follower instead (and
``--camera realsense`` for the real camera); ``--serve`` keeps the WebXR
server up instead of running the scripted operator.
"""

from __future__ import annotations

import argparse
import math
from typing import Any, Dict, List

from robopy.viewer.cli import add_model_arguments, load_model
from robopy.vr.__main__ import DEFAULT_START_POSE, STREAMING_IK_OVERRIDES
from robopy.vr.head_only import HeadMotor, HeadOnlyFollowerBackend, head_motor_mapping
from robopy.vr.head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig
from robopy.vr.server import TeleopSession, VRServerConfig


def headset(yaw: float, pitch: float = 0.0) -> Dict[str, Any]:
    """A WebXR head pose: yaw about +Y (left positive), pitch about +X (up positive)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return {"p": [0.0, 1.6, 0.0], "q": [cy * sp, sy * cp, -sy * sp, cy * cp]}


def simulated_follower_bus() -> Any:
    """A follower bus that exists only in memory, head motors at mid travel."""
    from robopy.config.robot_config.rakuda_config import RAKUDA_MOTOR_MAPPING
    from robopy.motor.dynamixel_bus import DynamixelMotor
    from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint

    names = list(RAKUDA_MOTOR_MAPPING)
    motors = {name: DynamixelMotor(i + 1, name, "xm430-w350") for i, name in enumerate(names)}
    return SimulatedDynamixelBus(motors, joints={n: SimulatedJoint() for n in names})


def main(argv: List[str] | None = None) -> int:
    """Run the demo."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_model_arguments(parser)
    parser.add_argument("--follower-port", default=None, help="real follower; simulated otherwise")
    parser.add_argument("--camera", default="synthetic", help="synthetic, none, realsense[:i]")
    parser.add_argument("--head-signs", default="auto", help="YAW,PITCH motor signs, or auto")
    parser.add_argument("--serve", action="store_true", help="serve the WebXR page (no script)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args(argv)

    loaded = load_model(args, parser, ik_overrides=STREAMING_IK_OVERRIDES)
    follower = None
    try:
        bundle = loaded.bundle
        model = bundle.model
        if args.follower_port:
            from robopy.config.dotrobopy import apply_rakuda_dotconfig
            from robopy.config.robot_config.rakuda_config import RakudaConfig
            from robopy.robots.rakuda.rakuda_follower import RakudaFollower

            cfg = apply_rakuda_dotconfig(
                RakudaConfig(leader_port="", follower_port=args.follower_port)
            )
            cfg.follower_port = args.follower_port
            print("connecting to the follower; the HEAD WILL MOVE")
            follower = RakudaFollower(cfg)
            follower.connect()
            bus = follower.motors
        else:
            bus = simulated_follower_bus()

        # The model gives the sign of each head joint per headset radian and the
        # forward-looking neutral (for the twin); the motors' start pose is
        # "looking ahead" on the machine.
        urdf = HeadJointMapping.from_model(
            model, "head_yaw_dof", "head_pitch_dof", camera_frame="head_camera_link"
        )
        yaw = HeadMotor(
            "head_yaw", urdf_joint="head_yaw_dof", urdf_neutral_rad=urdf.yaw_neutral_rad
        )
        pitch = HeadMotor(
            "head_pitch",
            urdf_joint="head_pitch_dof",
            urdf_neutral_rad=urdf.pitch_neutral_rad,
            range_rad=math.radians(35),
        )
        signs = None
        if args.head_signs != "auto":
            y, p = (int(v) for v in args.head_signs.split(","))
            signs = (y, p)
        mapping = head_motor_mapping(
            yaw,
            pitch,
            yaw_sign_urdf=urdf.yaw_sign,
            pitch_sign_urdf=urdf.pitch_sign,
            sign_overrides=signs,
        )
        rest = {k: v for k, v in DEFAULT_START_POSE.items() if k in model.movable_joint_names}
        backend = HeadOnlyFollowerBackend(
            bus,
            model=model,
            yaw=yaw,
            pitch=pitch,
            tcp_frames=bundle.tcp_frames,
            rest_positions_rad=rest,
        )
        print(f"head motors at start: {backend.start_units}  ({backend.describe()['units']})")
        for note in mapping.notes:
            print(f"  note: {note}")
        tracker = HeadTracker(mapping, HeadTrackingConfig(filter_hz=None))

        if args.serve:
            from robopy.vr.__main__ import _make_camera
            from robopy.vr.server import VRServer, serve_vr

            camera_args = argparse.Namespace(
                camera=args.camera,
                camera_size="640x480",
                camera_fps=30.0,
                jpeg_quality=75,
                camera_max_width=960,
            )
            server = VRServer(
                bundle,
                ik=loaded.ik,
                backend=backend,
                head_tracker=tracker,
                arm_teleop=None,
                camera=_make_camera(camera_args, lambda: "head-only"),
                host=args.host,
                port=args.port,
                config=VRServerConfig(),
            )
            serve_vr(server)
            return 0

        # Scripted operator: look 30 degrees left, then 15 degrees down.
        session = TeleopSession(
            backend,
            head_tracker=tracker,
            arm_teleop=None,
            config=VRServerConfig(state_hz=1000.0),
            bundle=bundle,
        )
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
            goals = backend.describe()["motors"]
            yaw_goal, pitch_goal = goals["head_yaw"], goals["head_pitch"]
            print(
                f"headset yaw {math.degrees(target_yaw):+5.1f} pitch "
                f"{math.degrees(target_pitch):+5.1f} deg  ->  head_yaw goal "
                f"{yaw_goal['goal']:.0f} (start {yaw_goal['start']:.0f}), head_pitch goal "
                f"{pitch_goal['goal']:.0f} (start {pitch_goal['start']:.0f})"
            )
        session.close()
    finally:
        if follower is not None:
            follower.disconnect()
        loaded.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
