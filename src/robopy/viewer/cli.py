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
        soft_limits: Soft limits applied, as ``{joint: (lower, upper)}``.
        synthetic: Whether the synthetic fixture was served.
        provisional_soft_limits: Whether the continuous-joint limits are the
            unmeasured stand-ins rather than configured values.
        config: The ``RakudaControlConfig`` when ``--config`` was given.
        ik_overrides: The solver settings the session was built with, for
            the start-up log.
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
    ik_overrides: Dict[str, Any] = field(default_factory=dict)

    def provenance_lines(self) -> List[str]:
        """What was loaded and where every range came from, for the start-up log.

        The lines name the code revision, the model file and its hash, the
        source and validation state of every joint's limit, and the TCPs, so a
        session's behaviour can be traced back to what it ran on.
        """
        lines = [f"code: {_code_revision()}"]
        lines.append(f"model: {self.urdf} (sha256 {_file_hash(self.urdf)})")
        if self.synthetic:
            lines.append("  the synthetic fixture: Rakuda's topology, not its geometry")
        lines.append(f"  package dirs: {[str(d) for d in self.package_dirs]}")
        profile = self.bundle.limit_profile
        lines.append("joint limits (resolved; sliders, solver and adapter all read these):")
        for line in profile.summary_lines():
            lines.append(f"  {line}")
        if self.provisional_soft_limits:
            lines.append(
                "  NOTE: the continuous joints' limits are provisional stand-ins, not measured."
            )
        tcp = "validated" if self.bundle.tcp_validated else "NOT validated (placeholder offset)"
        lines.append(f"TCP frames: {self.bundle.tcp_frames} -- {tcp}")
        if self.bundle.home_positions_rad:
            lines.append("home pose: configured and checked")
        else:
            lines.append("home pose: none configured (the page offers zero all only)")
        if self.ik is not None:
            lines.append(f"IK groups: {self.ik.groups}")
            lines.append(f"IK settings: {self.ik.describe_config()}")
            lines.append(
                f"trajectory profile: {self.ik.trajectory_profile} "
                f"(sample {self.ik.sample_period_s} s, {self.ik.trajectory_limits.describe()})"
            )
            lines.append(
                "self-collision: "
                + ("evaluated" if self.ik.collision_modelled else "NOT evaluated (no pairs)")
            )
        else:
            lines.append("IK: not available")
        return lines

    _tmpdir: Any = field(default=None, repr=False)

    def cleanup(self) -> None:
        """Remove the temporary synthetic URDF, if one was written."""
        if self._tmpdir is not None:
            self._tmpdir.cleanup()
            self._tmpdir = None


def _trajectory_from_config(trajectory: Any) -> Tuple[Dict[str, Any], str | None]:
    """``IKSetup`` keyword arguments for the configuration's ``control.trajectory``.

    Ceilings the configuration sets are used as given; any it leaves unset
    falls back to the viewer's simulation profile, and the note says which,
    so a page never runs a profile nobody can account for.  The machine does
    not fall back: it refuses Cartesian mode with a ceiling it was not given.
    """
    from robopy.kinematics.cartesian_trajectory import TrajectoryLimits  # noqa: PLC0415

    from .server import SIMULATION_SAMPLE_PERIOD_S, SIMULATION_TRAJECTORY_LIMITS  # noqa: PLC0415

    defaults = SIMULATION_TRAJECTORY_LIMITS
    missing = trajectory.missing()
    values = {
        name: getattr(trajectory, name)
        if getattr(trajectory, name) is not None
        else getattr(defaults, name)
        for name in (
            "max_linear_velocity_m_s",
            "max_linear_acceleration_m_s2",
            "max_angular_velocity_rad_s",
            "max_angular_acceleration_rad_s2",
        )
    }
    lag = trajectory.lag_tolerance_m
    limits = TrajectoryLimits(
        **values, lag_tolerance_m=defaults.lag_tolerance_m if lag is None else lag
    )
    period = trajectory.sample_period_s or SIMULATION_SAMPLE_PERIOD_S
    if len(missing) == 5:
        profile, note = (
            "simulation",
            (
                "control.trajectory sets no ceiling: the viewer runs its simulation profile "
                f"({limits.describe()}, sample {period} s)."
            ),
        )
    elif missing:
        profile, note = (
            "config+simulation",
            (
                f"control.trajectory leaves {missing} unset: those take the viewer's simulation "
                "values. The machine will refuse Cartesian mode until they are set."
            ),
        )
    else:
        profile, note = "config", None
    return (
        {"trajectory_limits": limits, "sample_period_s": period, "trajectory_profile": profile},
        note,
    )


def _code_revision() -> str:
    """``branch@commit`` of the running checkout, or the package version."""
    import subprocess  # noqa: PLC0415

    root = Path(__file__).resolve()
    for parent in root.parents:
        if (parent / ".git").exists():
            try:
                commit = subprocess.run(
                    ["git", "-C", str(parent), "rev-parse", "--short", "HEAD"],
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                ).stdout.strip()
                branch = subprocess.run(
                    ["git", "-C", str(parent), "rev-parse", "--abbrev-ref", "HEAD"],
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                ).stdout.strip()
                if commit:
                    return f"{branch}@{commit}"
            except (OSError, subprocess.SubprocessError):
                break
            break
    try:
        from importlib.metadata import version  # noqa: PLC0415

        return f"robopy {version('robopy')}"
    except Exception:  # noqa: BLE001
        return "robopy (unknown revision)"


def _file_hash(path: Path) -> str:
    """Short SHA-256 of a file, or ``?`` when it cannot be read."""
    import hashlib  # noqa: PLC0415

    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()[:12]
    except OSError:
        return "?"


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

    # Soft limits from the command line are unvalidated: nobody said they
    # were measured.  Those from the configuration carry their own flag.
    soft_entries: Dict[str, Any] = {
        joint: {"lower": lo, "upper": hi, "validated": False, "note": "--soft-limit"}
        for joint, (lo, hi) in parse_soft_limits(list(args.soft_limit), parser).items()
    }
    overrides: Dict[str, Any] = {}
    home: Dict[str, float] = {}
    tcp_offsets = None
    tcp_validated = False
    groups: Dict[str, Any] = {}
    urdf: Path | None = args.urdf
    package_dirs = list(args.package_dirs)
    control_config = None
    provisional = False
    config_ik: Dict[str, Any] = {}
    trajectory_kwargs: Dict[str, Any] = {}
    trajectory_note: str | None = None

    if args.config:
        from robopy.config.dotrobopy import apply_rakuda_dotconfig
        from robopy.config.robot_config.rakuda_config import RakudaConfig
        from robopy.control.types import se3_from_quat_xyzw

        cfg = apply_rakuda_dotconfig(RakudaConfig(leader_port="", follower_port=""))
        if cfg.control is None:
            parser.error("--config given but .robopy/rakuda/config.yaml has no control section")
        assert cfg.control is not None
        control_config = cfg.control
        spec = cfg.control.model
        if spec.urdf_path:
            urdf = urdf or Path(spec.urdf_path)
        # urdf_path: null means the bundled model, exactly as the machine
        # resolves it; the fallback below does the same lookup.
        package_dirs = package_dirs or [Path(d) for d in spec.package_dirs]
        soft_entries = {**spec.soft_limit_entries(), **soft_entries}
        overrides = spec.override_entries()
        home = dict(spec.home_positions_rad)
        if spec.left_tcp and spec.right_tcp:
            tcp_offsets = {
                side: (tcp.parent_frame, se3_from_quat_xyzw(tcp.translation_m, tcp.quaternion_xyzw))
                for side, tcp in (("left", spec.left_tcp), ("right", spec.right_tcp))
            }
            tcp_validated = bool(spec.left_tcp.validated and spec.right_tcp.validated)
        groups = {
            "torso_joint": spec.torso_joint,
            "left_arm_joints": spec.left_arm_joints or None,
            "right_arm_joints": spec.right_arm_joints or None,
            "head_joints": spec.head_joints or None,
        }
        config_ik = cfg.control.ik.solver_overrides()
        trajectory_kwargs, trajectory_note = _trajectory_from_config(cfg.control.trajectory)
        if trajectory_note:
            say(f"  {trajectory_note}")

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
            continuous = ("torso_yaw_dof", "shoulder_pitch_left_dof", "shoulder_pitch_right_dof")
            missing = [j for j in continuous if j not in soft_entries]
            if missing:
                # The three continuous joints have no URDF range at all, and the
                # solver needs a finite one. The motor's own travel is a
                # stand-in for simulation -- it is not a measurement of where
                # the machine stops -- and it is recorded as exactly that. A
                # measured range belongs in .robopy/rakuda/config.yaml.
                from robopy.config.robot_config import RAKUDA_MOTOR_TRAVEL_RAD

                low, high = RAKUDA_MOTOR_TRAVEL_RAD
                for joint in missing:
                    soft_entries[joint] = {
                        "lower": low,
                        "upper": high,
                        "validated": False,
                        "note": "provisional: the servo's travel, not a measured stop",
                    }
                provisional = True
                say(
                    f"  Continuous joints {missing} get the servo travel "
                    f"({math.degrees(low):.0f} to {math.degrees(high):.0f} deg) as a "
                    "PROVISIONAL, simulation-only range; it is not measured. Pass --soft-limit "
                    "or use --config for the real ranges."
                )
    if urdf is None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        tmpdir = tempfile.TemporaryDirectory()
        urdf = write_synthetic_dual_arm_urdf(Path(tmpdir.name) / "synthetic_dual_arm.urdf")
        synthetic = True
        say("Serving the synthetic fixture (Rakuda's topology, not its geometry).")
        for joint, bounds in (
            ("torso_yaw_dof", (-1.5, 1.5)),
            ("shoulder_pitch_left_dof", (-2.0, 2.0)),
            ("shoulder_pitch_right_dof", (-2.0, 2.0)),
        ):
            soft_entries.setdefault(
                joint,
                {
                    "lower": bounds[0],
                    "upper": bounds[1],
                    "validated": False,
                    "note": "synthetic fixture",
                },
            )

    try:
        bundle = ModelBundle.load(
            urdf,
            package_dirs=package_dirs,
            soft_limits=soft_entries,
            joint_limit_overrides=overrides,
            tcp_offsets=tcp_offsets,
            tcp_validated=tcp_validated,
            home_positions_rad=home,
            geometry_source=args.geometry,
        )
    except Exception as exc:  # noqa: BLE001 - report and exit with a clear message
        if tmpdir is not None:
            tmpdir.cleanup()
        print(f"Could not load {urdf}: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    ik = None
    # The configuration's control.ik section is the shared behaviour; a
    # command's own overrides (the VR streaming profile) come on top of it.
    merged_overrides: Dict[str, Any] = {**config_ik, **(ik_overrides or {})}
    if not args.no_ik:
        try:
            ik = IKSetup(
                bundle,
                config_overrides=merged_overrides,
                **trajectory_kwargs,
                **groups,  # type: ignore[arg-type]
            )
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
        soft_limits=dict(bundle.soft_limits),
        synthetic=synthetic,
        provisional_soft_limits=provisional,
        config=control_config,
        ik_overrides=merged_overrides,
        _tmpdir=tmpdir,
    )
