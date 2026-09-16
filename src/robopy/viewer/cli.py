"""Model-loading options shared by ``robopy-viewer`` and ``robopy-vr``.

Both commands need the same thing -- a :class:`ModelBundle`, the joint groups
and (optionally) the jog solver -- from the same flags: ``--urdf``,
``--package-dir``, ``--config``, ``--soft-limit``, ``--geometry`` and the
fallbacks to the bundled Rakuda model or the synthetic fixture.  Keeping the
logic here means the two commands cannot drift apart.
"""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple

from .model_bundle import ModelBundle
from .server import IKSetup

__all__ = ["LoadedModel", "add_model_arguments", "load_model"]


def add_model_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the model selection flags to ``parser``."""
    parser.add_argument(
        "--urdf",
        type=Path,
        help=(
            "URDF to load. Default: the Rakuda model bundled with robopy (robopy/models/rakuda); "
            "the synthetic fixture if no model directory is found."
        ),
    )
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="serve the synthetic fixture even when the bundled Rakuda model is present",
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
    parser.add_argument("--no-ik", action="store_true", help="joint-space only; skip the solver")


@dataclass
class LoadedModel:
    """What :func:`load_model` produced.

    Attributes:
        bundle: The loaded model.
        ik: The jog solver, or ``None`` when disabled or unavailable.
        groups: Joint-group keyword arguments given to :class:`IKSetup`.
        urdf: The URDF path that was loaded.
        package_dirs: Package directories used to resolve meshes.
        soft_limits: Soft limits applied.
        synthetic: Whether the synthetic fixture was served.
        provisional_soft_limits: Whether the continuous-joint limits are the
            unmeasured stand-ins rather than configured values.
        config: The ``RakudaControlConfig`` when ``--config`` was given.
    """

    bundle: ModelBundle
    ik: IKSetup | None
    groups: Dict[str, Any]
    urdf: Path
    package_dirs: List[Path]
    soft_limits: Dict[str, Tuple[float, float]]
    synthetic: bool = False
    provisional_soft_limits: bool = False
    config: Any = None
    _tmpdir: Any = field(default=None, repr=False)

    def cleanup(self) -> None:
        """Remove the temporary synthetic URDF, if one was written."""
        if self._tmpdir is not None:
            self._tmpdir.cleanup()
            self._tmpdir = None


def parse_soft_limits(
    entries: List[str], parser: argparse.ArgumentParser
) -> Dict[str, Tuple[float, float]]:
    """``JOINT=LOWER,UPPER`` strings to a dict; errors go through ``parser.error``."""
    soft_limits: Dict[str, Tuple[float, float]] = {}
    for entry in entries:
        try:
            joint, bounds = entry.split("=", 1)
            lower, upper = (float(v) for v in bounds.split(","))
        except ValueError:
            parser.error(f"--soft-limit expects JOINT=LOWER,UPPER, got {entry!r}")
        soft_limits[joint] = (lower, upper)
    return soft_limits


def load_model(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    *,
    ik_overrides: Mapping[str, Any] | None = None,
    quiet: bool = False,
) -> LoadedModel:
    """Resolve the flags added by :func:`add_model_arguments` into a model.

    Args:
        args: Parsed arguments.
        parser: The parser, for error reporting.
        ik_overrides: Solver configuration overrides (see :class:`IKSetup`).
        quiet: Suppress the progress lines.

    Returns:
        The loaded model.  Call :meth:`LoadedModel.cleanup` when done.
    """

    def say(text: str) -> None:
        if not quiet:
            print(text)

    soft_limits = parse_soft_limits(list(args.soft_limit), parser)
    tcp_offsets = None
    groups: Dict[str, Any] = {}
    urdf: Path | None = args.urdf
    package_dirs = list(args.package_dirs)
    control_config = None
    provisional = False

    if args.config:
        from robopy.config.dotrobopy import apply_rakuda_dotconfig
        from robopy.config.robot_config.rakuda_config import RakudaConfig
        from robopy.control.types import se3_from_quat_xyzw

        cfg = apply_rakuda_dotconfig(RakudaConfig(leader_port="", follower_port=""))
        if cfg.control is None or not cfg.control.model.urdf_path:
            parser.error(
                "--config given but .robopy/rakuda/config.yaml has no control.model.urdf_path"
            )
        assert cfg.control is not None
        control_config = cfg.control
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

    # Rakuda's joints are driven over the whole DYNAMIXEL count range in
    # leader-follower teleoperation, which is wider than several of the ranges
    # its CAD export declares. The sliders show that travel, so the viewer and
    # the machine speak of the same angles; the synthetic fixture has no motors
    # and keeps its own URDF ranges.
    from robopy.config.robot_config import RAKUDA_MOTOR_TRAVEL_RAD

    joint_travel: Tuple[float, float] | None = RAKUDA_MOTOR_TRAVEL_RAD

    tmpdir = None
    synthetic = False
    if urdf is None and not args.synthetic:
        from robopy.models import find_rakuda_model

        rakuda = find_rakuda_model()
        if rakuda is not None:
            # The convex URDF carries both the visual meshes (Git LFS, optional)
            # and the convex hulls (plain git); ModelBundle picks whichever is present.
            urdf = rakuda.convex_collision_urdf
            package_dirs = package_dirs or list(rakuda.package_dirs)
            say(f"Serving the bundled Rakuda model: {urdf}")
            hint = rakuda.visual_mesh_hint()
            if hint:
                say(f"  {hint}")
            if not soft_limits:
                # The three continuous joints have no URDF range at all, and the
                # solver needs a finite one. The motor travel is what the
                # machine is driven over, so that is what they get -- it is not
                # a measurement of where they actually stop, and a measured
                # range belongs in .robopy/rakuda/config.yaml (--config).
                soft_limits = {
                    joint: RAKUDA_MOTOR_TRAVEL_RAD
                    for joint in (
                        "torso_yaw_dof",
                        "shoulder_pitch_left_dof",
                        "shoulder_pitch_right_dof",
                    )
                }
                provisional = True
                say(
                    "  Continuous joints get the motor travel as their solver range; it is not "
                    "measured. Pass --soft-limit or use --config for the real ranges."
                )
            low, high = RAKUDA_MOTOR_TRAVEL_RAD
            say(
                f"  Joint sliders span the motor travel ({math.degrees(low):.0f} to "
                f"{math.degrees(high):.0f} deg about the count zero), as in leader-follower "
                "position teleoperation; the solver still obeys the URDF range."
            )
    if urdf is None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        tmpdir = tempfile.TemporaryDirectory()
        urdf = write_synthetic_dual_arm_urdf(Path(tmpdir.name) / "synthetic_dual_arm.urdf")
        synthetic = True
        joint_travel = None  # no motors behind it; its URDF ranges are all there is
        say("Serving the synthetic fixture (Rakuda's topology, not its geometry).")
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
            joint_travel_rad=joint_travel,
            tcp_offsets=tcp_offsets,
            geometry_source=args.geometry,
        )
    except Exception as exc:  # noqa: BLE001 - report and exit with a clear message
        if tmpdir is not None:
            tmpdir.cleanup()
        print(f"Could not load {urdf}: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    ik = None
    if not args.no_ik:
        try:
            ik = IKSetup(bundle, config_overrides=ik_overrides, **groups)  # type: ignore[arg-type]
            if ik.geometric_study_only:
                say(
                    "IK: continuous joint(s) "
                    f"{ik.groups['unbounded_continuous']} have no soft limit -- the solver runs "
                    "as a geometric study only. Pass --soft-limit JOINT=LOWER,UPPER for the "
                    "real range."
                )
        except Exception as exc:  # noqa: BLE001 - the viewer is still useful without IK
            print(f"IK not available: {exc}", file=sys.stderr)

    return LoadedModel(
        bundle=bundle,
        ik=ik,
        groups=groups,
        urdf=urdf,
        package_dirs=package_dirs,
        soft_limits=soft_limits,
        synthetic=synthetic,
        provisional_soft_limits=provisional,
        config=control_config,
        _tmpdir=tmpdir,
    )
