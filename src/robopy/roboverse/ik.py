"""Scripted arm control: differential IK, and a way to drive a task with it.

This is not a policy.  It is the machinery a demonstration script needs to put
the Rakuda's hand somewhere and film the result, factored out of
``examples/robot/rakuda_lift_block.py`` once a second script wanted it.

Three things about this arm are easy to get wrong, and each cost real debugging:

Solve on a scratch state, never the live one.
    ``env.handler.physics.data`` is the running simulation.  Writing ``qpos``
    into it to evaluate a candidate pose -- which is what an IK loop does on
    every iteration -- teleports the robot and destroys every contact it had.
    The grip then appears to fail for mysterious physical reasons.  :class:`Arm`
    keeps its own :class:`mujoco.MjData` for this.

Do not pin the roll.
    The arm has six joints plus the waist and cannot reach every orientation.
    Asking for a full pose leaves the IK 17 cm short; asking only for the finger
    *direction*, and leaving the rotation about it free, converges to 0.1 mm.
    For a square block the roll does not matter anyway.

Move the hand along a path, not the joints along a line.
    Interpolating joint angles between two solved poses sweeps the arm through
    whatever is in between -- the table, usually, or the object it is reaching
    for.  :meth:`ScriptedRun.move` re-solves at every waypoint of a straight
    line in space instead.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Sequence

__all__ = [
    "ARM_JOINTS",
    "FINGERS_DOWN",
    "PALM_BODIES",
    "Arm",
    "ScriptedRun",
    "solve_ik",
]

#: Joints the IK is allowed to move, per side.  The waist is shared -- it turns
#: the whole torso -- and the head is never touched.
ARM_JOINTS: Dict[str, Sequence[str]] = {
    "right": (
        "torso_yaw_dof",
        "shoulder_pitch_right_dof",
        "shoulder_roll_right_dof",
        "elbow_yaw_right_dof",
        "elbow_pitch_right_dof",
        "wrist_yaw_right_dof",
        "wrist_pitch_right_dof",
    ),
    "left": (
        "torso_yaw_dof",
        "shoulder_pitch_left_dof",
        "shoulder_roll_left_dof",
        "elbow_yaw_left_dof",
        "elbow_pitch_left_dof",
        "wrist_yaw_left_dof",
        "wrist_pitch_left_dof",
    ),
}

#: Candidate bodies to treat as the hand, best first.  ``*_gripper_base`` is the
#: borrowed Panda jaw's palm and only exists on ``rakuda_gripper``;
#: ``gripper_*_dof`` is the CAD's own hand frame, which every variant has.
PALM_BODIES: Dict[str, Sequence[str]] = {
    "right": ("right_gripper_base", "gripper_right_dof"),
    "left": ("left_gripper_base", "gripper_left_dof"),
}


def _numpy():
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError("scripted control needs NumPy") from exc
    return np


#: Direction the fingers are asked to point.  Straight down; the rotation about
#: that axis is left free.
def _fingers_down():
    return _numpy().array([0.0, 0.0, -1.0])


FINGERS_DOWN = _fingers_down()


class Arm:
    """The parts of a running task's MuJoCo model a scripted arm needs.

    Args:
        env: A constructed task env, already reset.
        robot: The robot config, i.e. ``env.scenario.robots[0]``.
        side: ``"right"`` or ``"left"``.
        palm_body: Override the body treated as the hand.  By default the first
            of :data:`PALM_BODIES` for that side which the model actually has.

    Raises:
        KeyError: If ``side`` is not a side, or the model has none of the
            candidate palm bodies -- which would mean the asset has drifted.
    """

    def __init__(self, env, robot, side: str = "right", palm_body: str | None = None) -> None:
        import mujoco

        if side not in ARM_JOINTS:
            raise KeyError(f"side must be 'right' or 'left', not {side!r}")
        self.side = side
        self.arm_joints = tuple(ARM_JOINTS[side])
        self.model = env.handler.physics.model.ptr
        self.live = env.handler.physics.data.ptr
        # Scratch state for IK.  Never solve on `live`: see the module docstring.
        self.scratch = mujoco.MjData(self.model)
        self.prefix = env.handler._mujoco_robot_names[0]
        self.joints = env.handler.get_joint_names(robot.name, sort=True)
        self.fingers = [j for j in self.joints if f"{side}_finger" in j]
        self.home = {j: float(robot.default_joint_positions[j]) for j in self.joints}

        ident = {
            j: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.prefix + j)
            for j in self.joints
        }
        self.qadr = {j: self.model.jnt_qposadr[i] for j, i in ident.items()}
        self.dofadr = {j: self.model.jnt_dofadr[i] for j, i in ident.items()}
        self.limits = {j: tuple(self.model.jnt_range[i]) for j, i in ident.items()}

        candidates = [palm_body] if palm_body else list(PALM_BODIES[side])
        self.palm = -1
        for name in candidates:
            found = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.prefix + name)
            if found >= 0:
                self.palm_name = name
                self.palm = found
                break
        if self.palm < 0:
            raise KeyError(
                f"the model has none of {candidates} to use as the {side} hand; "
                "re-export it with robopy.sim.mjcf_export"
            )

    def palm_pose(self, data):
        """``(position, rotation)`` of the hand frame in ``data``."""
        return data.xpos[self.palm].copy(), data.xmat[self.palm].reshape(3, 3).copy()

    def forward(self, q: Dict[str, float]):
        """Hand position for a set of joint angles, evaluated on the scratch state."""
        import mujoco

        for joint, value in q.items():
            self.scratch.qpos[self.qadr[joint]] = value
        mujoco.mj_kinematics(self.model, self.scratch)
        return self.scratch.xpos[self.palm].copy()

    def action(self, q: Dict[str, float], grip: float):
        """A full action vector: arm from ``q``, this side's fingers at ``grip``."""
        import torch

        row = [grip if j in self.fingers else q.get(j, self.home[j]) for j in self.joints]
        return torch.tensor([row], dtype=torch.float32)


def solve_ik(
    arm: Arm,
    target,
    seed: Dict[str, float],
    iterations: int = 600,
    direction=None,
):
    """Damped least squares for the hand position and the finger direction.

    Runs entirely on ``arm.scratch``.  The orientation term only asks that the
    fingers point along ``direction``; the rotation about that axis is left
    free, which is what makes an otherwise unreachable pose reachable.

    Args:
        arm: The model wrapper.
        target: Where the hand should end up, in world coordinates.
        seed: Joint angles to start from.
        iterations: Cap on the solver loop.
        direction: Unit vector the fingers should point along.  Defaults to
            :data:`FINGERS_DOWN`.

    Returns:
        ``(joint angles, residual in metres)``.  A large residual means the
        target is out of reach, not that the solver failed.
    """
    import mujoco

    np = _numpy()
    down = FINGERS_DOWN if direction is None else np.asarray(direction, dtype=float)
    data = arm.scratch
    q = dict(seed)
    jacp, jacr = np.zeros((3, arm.model.nv)), np.zeros((3, arm.model.nv))
    cols = [arm.dofadr[j] for j in arm.arm_joints]

    for _ in range(iterations):
        for joint, value in q.items():
            data.qpos[arm.qadr[joint]] = value
        mujoco.mj_kinematics(arm.model, data)
        mujoco.mj_comPos(arm.model, data)
        position, rotation = arm.palm_pose(data)

        error_p = target - position
        error_w = np.cross(rotation[:, 2], down)
        if np.linalg.norm(error_p) < 1e-4 and np.linalg.norm(error_w) < 5e-3:
            break

        mujoco.mj_jacBody(arm.model, data, jacp, jacr, arm.palm)
        jacobian = np.vstack([jacp[:, cols], jacr[:, cols]])
        step = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 0.05**2 * np.eye(6), np.concatenate([error_p, error_w])
        )
        for index, joint in enumerate(arm.arm_joints):
            low, high = arm.limits[joint]
            q[joint] = float(
                np.clip(q[joint] + np.clip(step[index], -0.08, 0.08), low + 1e-3, high - 1e-3)
            )

    for joint, value in q.items():
        data.qpos[arm.qadr[joint]] = value
    mujoco.mj_kinematics(arm.model, data)
    return q, float(np.linalg.norm(target - data.xpos[arm.palm]))


class ScriptedRun:
    """Drives a task env along a hand path, recording frames as it goes.

    Args:
        env: The task env.
        arm: Its :class:`Arm`.
        camera: A ``MjvCamera``, or ``None`` not to render.
        renderer: A ``mujoco.Renderer``, or ``None`` not to render.
        note: Called as ``note(states)`` after each leg, returning a string to
            append to :attr:`log`.  Tasks report progress differently, so this
            is where a script says what it wants recorded.
    """

    def __init__(self, env, arm: Arm, camera=None, renderer=None, note=None) -> None:
        self.env = env
        self.arm = arm
        self.camera = camera
        self.renderer = renderer
        self.note: Callable | None = note
        self.frames: List = []
        self.log: List[str] = []
        self.q = dict(arm.home)
        self.states = None
        self.succeeded = False

    def _step(self, q, grip):
        self.states, reward, terminated, _, _ = self.env.step(self.arm.action(q, grip))
        if bool(terminated[0]):
            self.succeeded = True
        if self.renderer is not None:
            self.renderer.update_scene(self.arm.live, self.camera)
            self.frames.append(self.renderer.render())
        return reward

    def _record(self, label: str, extra: str = "") -> None:
        detail = self.note(self.states) if self.note is not None else ""
        self.log.append(f"{label:9} {detail}{extra}")

    def hold(self, grip: float, steps: int, label: str) -> None:
        """Keep one target while the servos settle."""
        for _ in range(steps):
            self._step(self.q, grip)
        self._record(label)

    def move(self, target, grip: float, label: str, waypoints: int = 14, dwell: int = 8) -> None:
        """Follow a straight line in space, re-solving IK at every waypoint."""
        np = _numpy()
        here = self.arm.forward(self.q)
        worst = 0.0
        for index in range(1, waypoints + 1):
            point = here + (np.asarray(target) - here) * (index / waypoints)
            self.q, residual = solve_ik(self.arm, point, self.q)
            worst = max(worst, residual)
            for _ in range(dwell if index < waypoints else dwell * 3):
                self._step(self.q, grip)
        self._record(label, f"  worst IK {worst:.4f} m")

    def correct(
        self, target, grip: float, label: str, rounds: int = 4, dwell: int = 25, tol: float = 2e-3
    ):
        """Close the loop: aim off by however far the real hand is missing by.

        :meth:`move` commands a pose the IK says is right, but the servos hold it
        against gravity with a steady-state error -- leaning out over a table,
        0.02 m of it.  That is enough for a jaw to close beside a 0.04 m block
        instead of around it, shoving it sideways.

        So this reads the *live* hand position, not the commanded one, and biases
        the next command by the difference.  A few rounds and the hand is where
        it was asked to be.

        Returns:
            The remaining error in metres.
        """
        import mujoco

        np = _numpy()
        target = np.asarray(target)
        error = None
        for _ in range(rounds):
            mujoco.mj_forward(self.arm.model, self.arm.live)
            live = self.arm.live.xpos[self.arm.palm].copy()
            offset = target - live
            error = float(np.linalg.norm(offset))
            if error < tol:
                break
            self.q, _ = solve_ik(self.arm, self.arm.forward(self.q) + offset, self.q)
            for _ in range(dwell):
                self._step(self.q, grip)
        mujoco.mj_forward(self.arm.model, self.arm.live)
        error = float(np.linalg.norm(target - self.arm.live.xpos[self.arm.palm]))
        self._record(label, f"  hand off by {error:.4f} m")
        return error
