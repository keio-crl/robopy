"""Command-line entry point: ``python -m robopy.viewer``."""

from __future__ import annotations

import argparse
import logging
import sys
import tempfile
from pathlib import Path
from typing import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    """Parse arguments, load a model and serve the viewer."""
    parser = argparse.ArgumentParser(
        prog="python -m robopy.viewer",
        description=(
            "Browser viewer / simulator for the Rakuda model. Drive joint angles or "
            "end-effector targets and watch the 3D model -- no hardware involved."
        ),
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        help=(
            "URDF to load. Default: the Rakuda model committed under models/rakuda; the "
            "synthetic fixture if no model directory is found."
        ),
    )
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="serve the synthetic fixture even when the committed Rakuda model is present",
    )
    parser.add_argument(
        "--package-dir",
        type=Path,
        action="append",
        default=[],
        dest="package_dirs",
        help="directory resolving package:// mesh URIs (repeatable)",
    )
    parser.add_argument(
        "--config",
        action="store_true",
        help="take the model, soft limits and TCPs from .robopy/rakuda/config.yaml",
    )
    parser.add_argument(
        "--soft-limit",
        action="append",
        default=[],
        metavar="JOINT=LOWER,UPPER",
        help="soft limit in radians for a continuous joint (repeatable)",
    )
    parser.add_argument(
        "--geometry",
        choices=["auto", "visual", "collision"],
        default="auto",
        help=(
            "which URDF elements to draw: visual meshes, collision geometry (the convex hulls), "
            "or auto -- visual when its meshes are present, collision when they are LFS pointers"
        ),
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true", help="do not open a browser tab")
    parser.add_argument("--no-ik", action="store_true", help="joint-space only; skip the solver")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from .model_bundle import ModelBundle
    from .server import IKSetup, serve

    soft_limits: dict[str, tuple[float, float]] = {}
    for entry in args.soft_limit:
        try:
            joint, bounds = entry.split("=", 1)
            lower, upper = (float(v) for v in bounds.split(","))
        except ValueError:
            parser.error(f"--soft-limit expects JOINT=LOWER,UPPER, got {entry!r}")
        soft_limits[joint] = (lower, upper)

    tcp_offsets = None
    groups: dict[str, object] = {}
    urdf = args.urdf
    package_dirs = list(args.package_dirs)

    if args.config:
        from robopy.config.dotrobopy import apply_rakuda_dotconfig
        from robopy.config.robot_config.rakuda_config import RakudaConfig
        from robopy.control.types import se3_from_quat_xyzw

        cfg = apply_rakuda_dotconfig(RakudaConfig(leader_port="", follower_port=""))
        if cfg.control is None or not cfg.control.model.urdf_path:
            parser.error(
                "--config given but .robopy/rakuda/config.yaml has no control.model.urdf_path"
            )
        spec = cfg.control.model
        urdf = urdf or Path(spec.urdf_path)  # type: ignore[arg-type]
        package_dirs = package_dirs or [Path(d) for d in spec.package_dirs]
        soft_limits = {**spec.soft_limits_rad, **soft_limits}
        if spec.left_tcp and spec.right_tcp:
            tcp_offsets = {
                side: (tcp.parent_frame, se3_from_quat_xyzw(tcp.translation_m, tcp.quaternion_xyzw))
                for side, tcp in (("left", spec.left_tcp), ("right", spec.right_tcp))
            }
        groups = {
            "torso_joint": spec.torso_joint,
            "left_arm_joints": spec.left_arm_joints or None,
            "right_arm_joints": spec.right_arm_joints or None,
            "head_joints": spec.head_joints or None,
        }

    tmpdir = None
    if urdf is None and not args.synthetic:
        from robopy.models import find_rakuda_model

        rakuda = find_rakuda_model()
        if rakuda is not None:
            # The convex URDF carries both the visual meshes (Git LFS, optional)
            # and the convex hulls (plain git); ModelBundle picks whichever is present.
            urdf = rakuda.convex_collision_urdf
            package_dirs = package_dirs or [rakuda.package_dir]
            print(f"Serving the committed Rakuda model: {urdf}")
            hint = rakuda.visual_mesh_hint()
            if hint:
                print(f"  {hint}")
            if not soft_limits:
                # The three continuous joints have no URDF range. These bounds are
                # NOT measured on the machine; they only let the solver build.
                # Measured values belong in .robopy/rakuda/config.yaml (--config).
                soft_limits = {
                    "torso_yaw_dof": (-1.57, 1.57),
                    "shoulder_pitch_left_dof": (-3.14, 3.14),
                    "shoulder_pitch_right_dof": (-3.14, 3.14),
                }
                print(
                    "  Soft limits for the continuous joints are provisional (not measured); "
                    "pass --soft-limit or use --config for the real ranges."
                )
    if urdf is None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        tmpdir = tempfile.TemporaryDirectory()
        urdf = write_synthetic_dual_arm_urdf(Path(tmpdir.name) / "synthetic_dual_arm.urdf")
        print("Serving the synthetic fixture (Rakuda's topology, not its geometry).")
        soft_limits = {
            "torso_yaw_dof": (-1.5, 1.5),
            "shoulder_pitch_left_dof": (-2.0, 2.0),
            "shoulder_pitch_right_dof": (-2.0, 2.0),
            **soft_limits,
        }

    try:
        bundle = ModelBundle.load(
            urdf,
            package_dirs=package_dirs,
            soft_limits=soft_limits,
            tcp_offsets=tcp_offsets,
            geometry_source=args.geometry,
        )
    except Exception as exc:  # noqa: BLE001 - report and exit with a clear message
        print(f"Could not load {urdf}: {exc}", file=sys.stderr)
        return 1

    ik = None
    if not args.no_ik:
        try:
            ik = IKSetup(bundle, **groups)  # type: ignore[arg-type]
            if ik.geometric_study_only:
                print(
                    "IK: continuous joint(s) "
                    f"{ik.groups['unbounded_continuous']} have no soft limit -- the solver runs "
                    "as a geometric study only. Pass --soft-limit JOINT=LOWER,UPPER for the "
                    "real range."
                )
        except Exception as exc:  # noqa: BLE001 - the viewer is still useful without IK
            print(f"IK not available: {exc}", file=sys.stderr)

    try:
        serve(bundle, host=args.host, port=args.port, open_browser=not args.no_browser, ik=ik)
    finally:
        if tmpdir is not None:
            tmpdir.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
