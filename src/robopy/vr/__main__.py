"""Command-line entry point: ``robopy-vr`` / ``python -m robopy.vr``.

Serves the WebXR teleoperation page.  Without ``--hardware`` everything is
simulated: the headset drives the model's head, the controllers drive the
model's arms through the solver, and the camera is a test pattern unless
``--camera`` names a real one.  With ``--hardware`` the same page drives the
follower arm through :class:`~robopy.robots.rakuda.rakuda_control.RakudaControlSystem`.
"""

from __future__ import annotations

import argparse
import logging
import os
import shutil
import ssl
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Sequence, Tuple

#: Solver settings for stepping once per pose sample (rather than jogging to
#: convergence as the viewer does).  A bounded step per sample, a short
#: compute budget and no acceleration window -- the operator's hand is the
#: trajectory generator here.
STREAMING_IK_OVERRIDES: Dict[str, Any] = {
    "max_joint_step_rad": 0.05,
    "compute_budget_s": 0.05,
    "max_state_age_s": 2.0,
    "max_joint_acceleration_rad_s2": None,
    "damping": 1e-3,
}


def build_parser() -> argparse.ArgumentParser:
    """The command-line interface."""
    from robopy.viewer.cli import add_model_arguments

    parser = argparse.ArgumentParser(
        prog="robopy-vr",
        description=(
            "VR teleoperation of the Rakuda from a WebXR headset: the headset drives the "
            "head, the controllers drive the arms, the head camera is shown in front of the "
            "operator. Simulated unless --hardware is given."
        ),
    )
    add_model_arguments(parser)
    net = parser.add_argument_group("network")
    net.add_argument(
        "--host", default="127.0.0.1", help="bind address; 0.0.0.0 for a headset on the LAN"
    )
    net.add_argument("--port", type=int, default=8766)
    net.add_argument(
        "--cert", type=Path, help="TLS certificate (PEM); WebXR needs a secure context"
    )
    net.add_argument("--key", type=Path, help="TLS private key (PEM)")
    net.add_argument(
        "--self-signed",
        action="store_true",
        help="serve HTTPS with a self-signed certificate, generating it with openssl when "
        "the files do not exist yet (default cert.pem/key.pem in the current directory, or "
        "--cert/--key). The headset's browser will ask once to accept it.",
    )
    net.add_argument(
        "--open-browser", action="store_true", help="open the page in a desktop browser"
    )
    net.add_argument(
        "--state-hz", type=float, default=30.0, help="state messages per second to the page"
    )

    cam = parser.add_argument_group("camera")
    cam.add_argument(
        "--camera",
        default="synthetic",
        help="synthetic (default), none, or opencv:<index|/dev/videoN|url>",
    )
    cam.add_argument("--camera-fps", type=float, default=30.0)
    cam.add_argument("--jpeg-quality", type=int, default=75)
    cam.add_argument("--camera-max-width", type=int, default=960, help="downscale wider frames")
    cam.add_argument(
        "--camera-fov",
        type=float,
        default=69.0,
        help="horizontal field of view in degrees used to size the image (D435 colour: 69)",
    )

    head = parser.add_argument_group("head")
    head.add_argument("--no-head", action="store_true", help="do not drive the head")
    head.add_argument(
        "--head-joints",
        metavar="YAW,PITCH",
        help="URDF joints of the head (default: inferred from the model's head group)",
    )
    head.add_argument(
        "--camera-frame",
        default="auto",
        help="URDF frame of the head camera for the forward-looking neutral pose; 'auto' picks "
        "head_camera_link when present, 'none' assumes pitch is level at the URDF zero",
    )
    head.add_argument("--camera-forward-axis", default="auto", help="x|y|z|-x|-y|-z|auto")
    head.add_argument(
        "--no-torso-compensation",
        action="store_true",
        help="do not subtract the torso yaw from the head yaw command (by default the camera "
        "heading follows the headset in the base frame whatever the torso does)",
    )
    head.add_argument(
        "--head-scale", type=float, default=1.0, help="joint radians per headset radian"
    )
    head.add_argument("--head-rate", type=float, default=2.5, help="max head joint speed, rad/s")

    arms = parser.add_argument_group("arms")
    arms.add_argument("--no-arms", action="store_true", help="do not drive the arms")
    arms.add_argument(
        "--mapping",
        choices=["absolute", "relative"],
        default="absolute",
        help="absolute (default): while the clutch is held the hand goes to where the "
        "controller is, with the operator's head at the robot's head; relative: only the "
        "controller's motion since the press is applied",
    )
    arms.add_argument(
        "--arm-anchor",
        default="auto",
        metavar="FRAME",
        help="model frame that stands for the robot's head in the absolute mapping "
        "(auto: head_camera_link when present)",
    )
    arms.add_argument(
        "--clutch",
        choices=["a", "grip", "stick"],
        default="a",
        help="controller button held to drive an arm: a (A on the right, X on the left; "
        "default), grip (the squeeze) or stick (the thumbstick click)",
    )
    arms.add_argument(
        "--position-scale", type=float, default=1.0, help="robot metres per operator metre"
    )
    arms.add_argument("--no-orientation", action="store_true", help="translation-only hand targets")
    arms.add_argument(
        "--max-hand-speed", type=float, default=0.6, help="m/s slew limit of the targets"
    )
    arms.add_argument("--torso", choices=["fixed", "optimize", "manual"], default="fixed")
    arms.add_argument(
        "--gripper",
        action="append",
        default=[],
        metavar="SIDE=MOTOR:OPEN,CLOSED",
        help="measured gripper travel in radians, e.g. left=l_arm_grip:0.0,1.2 (repeatable). "
        "Without it the trigger does nothing: the travel is a measurement, not a default.",
    )
    arms.add_argument("--target-ttl", type=float, default=0.25, help="seconds a target stays valid")

    hw = parser.add_argument_group("hardware")
    hw.add_argument(
        "--hardware",
        action="store_true",
        help="drive the real follower via .robopy/rakuda/config.yaml (needs --config and a "
        "control section in cartesian_teleop mode). THE ROBOT WILL MOVE.",
    )
    hw.add_argument("--leader-port", default=None, help="leader serial port (config default)")
    hw.add_argument("--follower-port", default=None, help="follower serial port (config default)")

    parser.add_argument(
        "--start-pose",
        action="append",
        default=[],
        metavar="JOINT=RAD",
        help="simulation start configuration (repeatable). Default: elbows bent 0.8 rad, because "
        "the Rakuda export's zero pose is fully extended with the right elbow on its limit, "
        "where no solver step is feasible. Ignored with --hardware (the machine is where it is).",
    )
    parser.add_argument(
        "--twin-offset",
        default="auto",
        metavar="X,Y,Z",
        help="where the page draws the robot base (m, robot axes from the WebXR floor "
        "origin). Default auto: the robot's head where the operator's head was at "
        "re-centring, so the twin's hands and the operator's agree",
    )
    rec = parser.add_argument_group("recording")
    rec.add_argument(
        "--record-dir",
        type=Path,
        default=Path("recordings"),
        help="where session recordings and their videos go (default: ./recordings)",
    )
    rec.add_argument("--no-record", action="store_true", help="disable recording")
    rec.add_argument(
        "--no-render",
        action="store_true",
        help="keep the recording log but do not render videos after each recording "
        "(render later with robopy-vr-render)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser


#: Where distributions keep the OpenSSL configuration.  Some ``openssl`` builds
#: are compiled with a prefix that does not exist on the machine (e.g. a binary
#: in /usr/bin looking for /usr/local/ssl/openssl.cnf); ``openssl req`` then
#: fails even though a perfectly good config sits in one of these places.
OPENSSL_CONF_CANDIDATES: Tuple[str, ...] = (
    "/etc/ssl/openssl.cnf",
    "/usr/lib/ssl/openssl.cnf",
    "/etc/pki/tls/openssl.cnf",
    "/usr/local/etc/openssl/openssl.cnf",
    "/opt/homebrew/etc/openssl@3/openssl.cnf",
)

SELF_SIGNED_HINT = (
    "generate one with\n"
    "  openssl req -x509 -newkey rsa:2048 -nodes -keyout key.pem -out cert.pem -days 365 "
    '-subj "/CN=robopy"\n'
    "or pass --self-signed to let robopy-vr do it"
)


def generate_self_signed_certificate(
    cert: Path, key: Path, *, days: int = 365, common_name: str = "robopy"
) -> None:
    """Write a self-signed certificate and key with ``openssl``.

    WebXR only runs in a secure context, so a headset on the LAN needs HTTPS
    even for a simulation.  The certificate is for the browser to accept once,
    nothing more.  When the ``openssl`` binary cannot find its own configuration
    (a broken compile-time prefix) the command is retried with the first config
    found in :data:`OPENSSL_CONF_CANDIDATES`.

    Raises:
        RuntimeError: ``openssl`` is not installed or failed both times.
    """
    if shutil.which("openssl") is None:
        raise RuntimeError("openssl is not installed; cannot generate a self-signed certificate")
    command = [
        "openssl",
        "req",
        "-x509",
        "-newkey",
        "rsa:2048",
        "-nodes",
        "-keyout",
        str(key),
        "-out",
        str(cert),
        "-days",
        str(days),
        "-subj",
        f"/CN={common_name}",
    ]
    cert.parent.mkdir(parents=True, exist_ok=True)
    key.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode == 0:
        return
    fallback = next((c for c in OPENSSL_CONF_CANDIDATES if Path(c).is_file()), None)
    if "openssl.cnf" in result.stderr and fallback is not None and "OPENSSL_CONF" not in os.environ:
        env = dict(os.environ, OPENSSL_CONF=fallback)
        retry = subprocess.run(command, capture_output=True, text=True, env=env)
        if retry.returncode == 0:
            print(f"  note: openssl could not find its config; used OPENSSL_CONF={fallback}")
            return
        result = retry
    raise RuntimeError("openssl failed to generate the certificate:\n" + result.stderr.strip())


def _resolve_tls(
    args: argparse.Namespace, parser: argparse.ArgumentParser
) -> Tuple[Path, Path] | None:
    """The certificate and key to serve with, or ``None`` for plain HTTP.

    Checked before the model is loaded so a missing file stops the command at
    once, with the way out, instead of a traceback after the start-up banner.
    """
    if (args.cert is None) != (args.key is None):
        parser.error("--cert and --key go together")
    if args.cert is None and not args.self_signed:
        return None
    cert = args.cert if args.cert is not None else Path("cert.pem")
    key = args.key if args.key is not None else Path("key.pem")
    missing = [str(p) for p in (cert, key) if not p.is_file()]
    if missing and args.self_signed:
        try:
            generate_self_signed_certificate(cert, key)
        except RuntimeError as exc:
            parser.error(str(exc))
        print(f"Self-signed certificate written: {cert} / {key}")
    elif missing:
        parser.error(f"TLS file(s) not found: {', '.join(missing)}\n{SELF_SIGNED_HINT}")
    return cert, key


def _parse_triplet(
    text: str, parser: argparse.ArgumentParser, flag: str
) -> Tuple[float, float, float]:
    try:
        x, y, z = (float(v) for v in text.split(","))
    except ValueError:
        parser.error(f"{flag} expects X,Y,Z, got {text!r}")
    return x, y, z


def _parse_grippers(
    entries: Sequence[str], parser: argparse.ArgumentParser
) -> Dict[str, Dict[str, Any]]:
    out: Dict[str, Dict[str, Any]] = {}
    for entry in entries:
        try:
            side, rest = entry.split("=", 1)
            motor, travel = rest.split(":", 1)
            open_rad, closed_rad = (float(v) for v in travel.split(","))
        except ValueError:
            parser.error(f"--gripper expects SIDE=MOTOR:OPEN,CLOSED, got {entry!r}")
        if side not in ("left", "right"):
            parser.error(f"--gripper side must be left or right, got {side!r}")
        out[side] = {
            "gripper_motor": motor,
            "gripper_open_rad": open_rad,
            "gripper_closed_rad": closed_rad,
        }
    return out


def _infer_head_joints(names: Sequence[str], parser: argparse.ArgumentParser) -> Tuple[str, str]:
    head = [n for n in names if "head" in n]
    yaw = [n for n in head if "yaw" in n]
    pitch = [n for n in head if "pitch" in n]
    if len(yaw) != 1 or len(pitch) != 1:
        parser.error(f"could not infer the head joints from {head}; pass --head-joints YAW,PITCH")
    return yaw[0], pitch[0]


#: Bent-elbow start for the simulation: the Rakuda export's zero pose is fully
#: extended (singular) and puts the right elbow exactly on its upper limit.
DEFAULT_START_POSE: Dict[str, float] = {
    "elbow_pitch_left_dof": 0.8,
    "elbow_pitch_right_dof": -0.8,
}


def _start_pose(
    args: argparse.Namespace, parser: argparse.ArgumentParser, joints: Sequence[str]
) -> Dict[str, float]:
    if args.start_pose:
        out: Dict[str, float] = {}
        for entry in args.start_pose:
            try:
                joint, value = entry.split("=", 1)
                out[joint.strip()] = float(value)
            except ValueError:
                parser.error(f"--start-pose expects JOINT=RAD, got {entry!r}")
            if joint.strip() not in joints:
                parser.error(f"--start-pose: '{joint.strip()}' is not a joint of the model")
        return out
    return {k: v for k, v in DEFAULT_START_POSE.items() if k in joints}


def _make_camera(args: argparse.Namespace, caption: Any) -> Any:
    from .camera import FrameStreamer, JpegEncoder, OpenCVFrameSource, SyntheticFrameSource

    spec = str(args.camera).strip().lower()
    if spec == "none":
        return None
    if spec == "synthetic":
        source: Any = SyntheticFrameSource(caption=caption)
    elif spec.startswith("opencv:"):
        raw = str(args.camera)[len("opencv:") :]
        source = OpenCVFrameSource(int(raw) if raw.isdigit() else raw)
    else:
        raise SystemExit(
            f"--camera must be synthetic, none or opencv:<source>, got {args.camera!r}"
        )
    encoder = JpegEncoder(args.jpeg_quality, max_width=args.camera_max_width)
    return FrameStreamer(source, fps=args.camera_fps, encoder=encoder)


def main(argv: Sequence[str] | None = None) -> int:
    """Parse arguments, build the session and serve until interrupted."""
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    tls = _resolve_tls(args, parser)
    if args.hardware and not args.config:
        parser.error("--hardware needs --config so the page shows the model the controller uses")
    if args.no_head and args.no_arms:
        parser.error("--no-head with --no-arms leaves nothing to teleoperate")

    from robopy.control.types import TorsoPolicy
    from robopy.viewer.cli import load_model

    from .arm_teleop import ArmTeleopConfig, DualArmTeleop
    from .backend import ControlSystemBackend, SimulationBackend
    from .head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig
    from .server import VRServer, VRServerConfig, serve_vr

    loaded = load_model(args, parser, ik_overrides=STREAMING_IK_OVERRIDES)
    bundle = loaded.bundle
    pair = None
    try:
        # -- backend --------------------------------------------------------
        if args.hardware:
            from robopy.config.dotrobopy import apply_rakuda_dotconfig
            from robopy.config.robot_config.rakuda_config import RakudaConfig
            from robopy.robots.rakuda.rakuda_pair_sys import RakudaPairSys

            base = RakudaConfig(
                leader_port=args.leader_port or "", follower_port=args.follower_port or ""
            )
            cfg = apply_rakuda_dotconfig(base)
            if args.leader_port:
                cfg.leader_port = args.leader_port
            if args.follower_port:
                cfg.follower_port = args.follower_port
            if cfg.control is None or cfg.control.mode != "cartesian_teleop":
                parser.error(
                    "--hardware needs control.mode: cartesian_teleop in .robopy/rakuda/config.yaml"
                )
            print("HARDWARE MODE: connecting to the arms and starting Cartesian control.")
            pair = RakudaPairSys(cfg)
            pair.connect()
            system = pair.start_control()
            backend: Any = ControlSystemBackend(system, target_ttl_s=args.target_ttl)
            model = system.model
        else:
            model = bundle.model
            start = _start_pose(args, parser, model.movable_joint_names)
            backend = SimulationBackend(bundle, loaded.ik, initial_positions=start)
            if start:
                print(
                    "Simulation start pose: "
                    + ", ".join(f"{k}={v:+.2f}" for k, v in sorted(start.items()))
                )

        # -- head -----------------------------------------------------------
        head_tracker = None
        if not args.no_head:
            if args.head_joints:
                try:
                    yaw_joint, pitch_joint = (s.strip() for s in args.head_joints.split(","))
                except ValueError:
                    parser.error("--head-joints expects YAW,PITCH")
            else:
                inferred = (loaded.ik.groups.get("head") if loaded.ik is not None else None) or []
                yaw_joint, pitch_joint = _infer_head_joints(
                    inferred or list(model.movable_joint_names), parser
                )
            camera_frame: str | None
            if args.camera_frame == "auto":
                camera_frame = "head_camera_link" if model.has_frame("head_camera_link") else None
            elif args.camera_frame == "none":
                camera_frame = None
            else:
                camera_frame = args.camera_frame
            torso_joint = None
            if not args.no_torso_compensation:
                torso_joint = (
                    loaded.ik.groups.get("torso") if loaded.ik is not None else None
                ) or next((n for n in model.movable_joint_names if "torso" in n), None)
            try:
                mapping = HeadJointMapping.from_model(
                    model,
                    yaw_joint,
                    pitch_joint,
                    camera_frame=camera_frame,
                    camera_forward_axis=args.camera_forward_axis,
                    torso_joint=torso_joint,
                )
            except ValueError as exc:
                print(f"Head tracking disabled: {exc}", file=sys.stderr)
            else:
                head_tracker = HeadTracker(
                    mapping,
                    HeadTrackingConfig(
                        yaw_scale=args.head_scale,
                        pitch_scale=args.head_scale,
                        max_rate_rad_s=args.head_rate,
                    ),
                )
                print(
                    f"Head: {mapping.yaw_joint} sign {mapping.yaw_sign:+d} neutral "
                    f"{mapping.yaw_neutral_rad:+.3f} rad; {mapping.pitch_joint} sign "
                    f"{mapping.pitch_sign:+d} neutral {mapping.pitch_neutral_rad:+.3f} rad "
                    f"(forward from {mapping.forward_source})"
                )
                if mapping.torso_joint is not None:
                    print(
                        f"  head yaw command = signal {mapping.torso_coupling:+.2f} x "
                        f"({mapping.torso_joint} - {mapping.torso_reference_rad:.2f}): the camera "
                        "heading follows the headset in the base frame whatever the torso does"
                    )
                for note in mapping.notes:
                    print(f"  note: {note}")

        # -- arms -----------------------------------------------------------
        arm_teleop = None
        if not args.no_arms:
            if backend.name == "simulation" and loaded.ik is None:
                print("Arms disabled: no solver (see above).", file=sys.stderr)
            else:
                grippers = _parse_grippers(args.gripper, parser)
                if backend.name == "simulation" and grippers:
                    print(
                        "  note: the model has no gripper joints; gripper commands are recorded, "
                        "not simulated."
                    )
                configs = {
                    side: ArmTeleopConfig(
                        mapping=args.mapping,
                        position_scale=args.position_scale,
                        orientation_enabled=not args.no_orientation,
                        max_speed_m_s=args.max_hand_speed,
                        **grippers.get(side, {}),
                    )
                    for side in ("left", "right")
                }
                arm_teleop = DualArmTeleop(
                    configs["left"],
                    configs["right"],
                    torso_policy=TorsoPolicy(args.torso),
                    target_ttl_s=args.target_ttl,
                )
                for side in ("left", "right"):
                    if not configs[side].gripper_available:
                        print(
                            f"  {side} gripper: travel not given (--gripper); trigger does nothing."
                        )
                clutch_name = {"a": "A / X", "grip": "grip", "stick": "thumbstick click"}[
                    args.clutch
                ]
                print(f"Arms: {args.mapping} mapping; hold {clutch_name} to drive an arm.")

        if head_tracker is None and arm_teleop is None:
            print("Nothing to teleoperate.", file=sys.stderr)
            return 1

        # -- camera, TLS, server -------------------------------------------
        def caption() -> str:
            joints = backend.joint_positions()
            if head_tracker is None:
                return "head tracking off"
            m = head_tracker.mapping
            return (
                f"{m.yaw_joint}={joints.get(m.yaw_joint, 0.0):+.2f}  "
                f"{m.pitch_joint}={joints.get(m.pitch_joint, 0.0):+.2f} rad"
            )

        camera = _make_camera(args, caption)
        ssl_context = None
        if tls is not None:
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(str(tls[0]), str(tls[1]))

        server = VRServer(
            bundle,
            ik=loaded.ik,
            backend=backend,
            head_tracker=head_tracker,
            arm_teleop=arm_teleop,
            camera=camera,
            host=args.host,
            port=args.port,
            ssl_context=ssl_context,
            config=VRServerConfig(
                state_hz=args.state_hz,
                camera_fov_deg=args.camera_fov,
                twin_offset_m=None
                if args.twin_offset == "auto"
                else _parse_triplet(args.twin_offset, parser, "--twin-offset"),
                arm_anchor_frame=args.arm_anchor,
                clutch_button=args.clutch,
                record_dir=None if args.no_record else args.record_dir,
                render_videos=not args.no_render,
            ),
        )
        if not args.no_record:
            renderer = "off (--no-render)"
            if not args.no_render:
                try:
                    import mujoco  # noqa: F401

                    renderer = "MuJoCo"
                except ImportError:
                    renderer = "UNAVAILABLE: install mujoco (uv run --with mujoco ...)"
            print(
                f"Recording: B / Y or the page's Record button; files in "
                f"{args.record_dir.resolve()}; video renderer {renderer}"
            )
        serve_vr(server, open_browser=args.open_browser)
    finally:
        if pair is not None:
            try:
                print("stopping control:", "; ".join(pair.stop_control()))
            finally:
                pair.disconnect()
        loaded.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
