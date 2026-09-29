"""Compare the dual-arm solver's profiles numerically on the synthetic model.

The plan for natural hand motion (weighted vs. hierarchical task priority,
position-only vs. pose vs. axis-aligned orientation) is a claim about
numbers: final error, how closely the hand follows its reference path, joint
velocity and acceleration, distance to the joint limits, torso use and
compute time.  This script measures them on the synthetic dual-arm fixture
(Rakuda's topology, not its geometry) with the viewer's simulation ceilings,
so the comparison is reproducible without hardware.  Nothing here is a
measurement of the real machine::

    uv run --extra kinematics python scripts/evaluate_ik_profiles.py
    uv run --extra kinematics python scripts/evaluate_ik_profiles.py --json out.json

Each scenario is run once per profile through the same
:func:`robopy.kinematics.cartesian_trajectory.run_trajectory` the viewer
uses, from the same start pose, and the table reports per run:

``status``
    How the run ended (``converged``, or a stall class).
``goal``
    Final position (mm) and orientation (deg) error at the goal, per hand.
``path``
    Mean and maximum distance (mm) from the actual TCP to the reference
    along the way -- how faithfully the hand rides its reference.
``dq/dt`` / ``d2q/dt2``
    Largest joint speed (rad/s) and acceleration (rad/s^2) over the run.
``margin``
    Smallest distance (rad) of any active joint to a position limit.
``torso``
    Total torso travel (rad).
``ms/step``
    Mean solve time per sample.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np

PROFILES: Dict[str, Dict[str, Any]] = {
    "weighted/position_only": {
        "task_priority_mode": "weighted",
        "orientation_mode": "position_only",
    },
    "hierarchical/position_only": {
        "task_priority_mode": "hierarchical",
        "orientation_mode": "position_only",
    },
    "weighted/pose": {"task_priority_mode": "weighted", "orientation_mode": "pose"},
    "hierarchical/pose": {"task_priority_mode": "hierarchical", "orientation_mode": "pose"},
    "hierarchical/axis_aligned": {
        "task_priority_mode": "hierarchical",
        "orientation_mode": "axis_aligned",
        "approach_axis_tcp": (0.0, 0.0, -1.0),
    },
}

SOFT_LIMITS = {
    "torso_yaw_dof": (-1.5, 1.5),
    "shoulder_pitch_left_dof": (-2.0, 2.0),
    "shoulder_pitch_right_dof": (-2.0, 2.0),
}


@dataclass
class Scenario:
    """A start pose, goals as offsets of the hands' start poses, and a torso policy."""

    name: str
    start: Dict[str, float]
    offsets_m: Dict[str, Sequence[float]]
    torso_policy: str = "optimize"
    tilt_rad: float = 0.0  # rotate each goal about world Y by this much
    max_duration_s: float = 6.0


def _scenarios(joint_names: Sequence[str]) -> List[Scenario]:
    bent = {name: 0.0 for name in joint_names}
    bent["elbow_pitch_left_dof"] = -0.6
    bent["elbow_pitch_right_dof"] = -0.6
    return [
        Scenario("left 6 cm forward", bent, {"left": (0.06, 0.0, 0.0)}),
        Scenario("left 12 cm across the body", bent, {"left": (0.0, -0.12, 0.0)}),
        Scenario("left 6 cm forward, torso fixed", bent, {"left": (0.06, 0.0, 0.0)}, "fixed"),
        Scenario(
            "both hands 5 cm up",
            bent,
            {"left": (0.0, 0.0, 0.05), "right": (0.0, 0.0, 0.05)},
        ),
        Scenario(
            "left 4 cm forward, tilted 0.3 rad", bent, {"left": (0.04, 0.0, 0.0)}, tilt_rad=0.3
        ),
        Scenario("left 40 cm forward (beyond reach)", bent, {"left": (0.40, 0.0, 0.0)}),
    ]


def _rotate_y(T: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    out = T.copy()
    out[:3, :3] = R @ T[:3, :3]
    return out


def _evaluate(setup: Any, bundle: Any, scenario: Scenario) -> Dict[str, Any]:
    from robopy.viewer.model_bundle import matrix_to_pose

    model = bundle.model
    q0 = bundle.positions_to_q(scenario.start)
    targets: Dict[str, Any] = {}
    for side, offset in scenario.offsets_m.items():
        T = model.frame_pose(q0, bundle.tcp_frames[side]).copy()
        T[:3, 3] += np.asarray(offset, dtype=float)
        if scenario.tilt_rad:
            T = _rotate_y(T, scenario.tilt_rad)
        targets[side] = matrix_to_pose(T)
    setup.reset(scenario.start)
    result = setup.solve_trajectory(
        scenario.start,
        targets,
        torso_policy=scenario.torso_policy,
        max_duration_s=scenario.max_duration_s,
    )
    samples = result["samples"]
    dt = result["dt_s"]
    active = [name for name in setup.solver.active_joints if name in samples[0]["joints"]]
    joints = np.array([[s["joints"][name] for name in active] for s in samples])
    velocity = np.diff(joints, axis=0) / dt if len(joints) > 1 else np.zeros((1, len(active)))
    acceleration = np.diff(velocity, axis=0) / dt if len(velocity) > 1 else np.zeros_like(velocity)
    lower_bounds, upper_bounds = bundle.limit_profile.bounds(active)
    margins = []
    for row in joints:
        for value, lower, upper in zip(row, lower_bounds, upper_bounds):
            if math.isfinite(lower) and math.isfinite(upper):
                margins.append(min(value - lower, upper - value))
    torso = setup.groups["torso"]
    torso_travel = (
        float(np.sum(np.abs(np.diff([s["joints"][torso] for s in samples]))))
        if torso in samples[0]["joints"]
        else 0.0
    )
    path = [e for s in samples for e in s["reference_error_m"].values()]
    return {
        "scenario": scenario.name,
        "status": result["status"],
        "duration_s": result["duration_s"],
        "n_samples": len(samples),
        "goal_error_mm": {k: v * 1e3 for k, v in result["goal_error_m"].items()},
        "goal_orientation_deg": {
            k: math.degrees(v) for k, v in result["goal_orientation_error_rad"].items()
        },
        # The approach-axis angle, which is what axis_aligned mode is judged by
        # (the full orientation angle above is then only informative).
        "axis_deg": {
            side: math.degrees(result["errors"][f"{side}_axis_rad"])
            for side in result["goal_error_m"]
            if result["errors"].get(f"{side}_axis_rad") is not None
        },
        "orientation_mode": result["orientation_mode"],
        "path_mean_mm": 1e3 * float(np.mean(path)) if path else 0.0,
        "path_max_mm": 1e3 * float(np.max(path)) if path else 0.0,
        "max_joint_velocity_rad_s": float(np.max(np.abs(velocity))),
        "max_joint_acceleration_rad_s2": float(np.max(np.abs(acceleration))),
        "min_limit_margin_rad": float(min(margins)) if margins else float("nan"),
        "torso_travel_rad": torso_travel,
        "ms_per_step": 1e3 * float(np.mean([s["compute_time_s"] for s in samples[1:]] or [0.0])),
        "active_limits": result["active_limits"],
        "message": result["message"],
    }


def _fmt_goal(row: Dict[str, Any]) -> str:
    parts = []
    for side in ("left", "right"):
        if side in row["goal_error_mm"]:
            angle = (
                f"axis {row['axis_deg'][side]:.1f}°"
                if row["orientation_mode"] == "axis_aligned" and side in row["axis_deg"]
                else f"{row['goal_orientation_deg'][side]:.1f}°"
            )
            parts.append(f"{side[0].upper()} {row['goal_error_mm'][side]:.1f} mm / {angle}")
    return "; ".join(parts)


def _table(rows: List[Dict[str, Any]]) -> str:
    header = (
        "| scenario | profile | status | t (s) | goal | path mean/max (mm) | dq/dt | d2q/dt2 "
        "| margin (rad) | torso (rad) | ms/step |"
    )
    lines = [header, "|" + "---|" * 11]
    for row in rows:
        lines.append(
            f"| {row['scenario']} | {row['profile']} | {row['status']} | {row['duration_s']:.2f} "
            f"| {_fmt_goal(row)} | {row['path_mean_mm']:.2f} / {row['path_max_mm']:.2f} "
            f"| {row['max_joint_velocity_rad_s']:.2f} | {row['max_joint_acceleration_rad_s2']:.1f} "
            f"| {row['min_limit_margin_rad']:.2f} | {row['torso_travel_rad']:.3f} "
            f"| {row['ms_per_step']:.2f} |"
        )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """Run every scenario under every profile and print the comparison."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", type=Path, default=None, help="also write the rows as JSON")
    parser.add_argument(
        "--profiles",
        nargs="*",
        default=list(PROFILES),
        choices=list(PROFILES),
        help="which profiles to run (default: all)",
    )
    args = parser.parse_args(argv)
    try:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf
        from robopy.viewer.model_bundle import ModelBundle
        from robopy.viewer.server import IKSetup
    except ImportError as exc:  # pragma: no cover - environment dependent
        print(f"needs the 'kinematics' optional extra: {exc}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory() as tmp:
        urdf = write_synthetic_dual_arm_urdf(Path(tmp) / "synthetic_dual_arm.urdf")
        bundle = ModelBundle.load(urdf, soft_limits=SOFT_LIMITS)
        rows: List[Dict[str, Any]] = []
        setups = {name: IKSetup(bundle, config_overrides=PROFILES[name]) for name in args.profiles}
        first = next(iter(setups.values()))
        print(
            "synthetic dual-arm fixture; simulation ceilings "
            f"{first.trajectory_limits.describe()}, sample {first.sample_period_s} s; "
            f"self-collision {'evaluated' if first.collision_modelled else 'NOT evaluated'}"
        )
        for scenario in _scenarios(bundle.joint_order):
            for name, setup in setups.items():
                row = _evaluate(setup, bundle, scenario)
                row["profile"] = name
                rows.append(row)
        print(_table(rows))
        if args.json:
            args.json.write_text(json.dumps(rows, indent=2))
            print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
