"""Solve ``rakuda.lift_block``: reach for the block, close on it, pick it up.

Not a policy -- a scripted sequence with differential inverse kinematics, run
through the real task env so the reward and the success flag are the task's own.
It exists to show the environment works end to end, and to be a starting point
for something that learns.

    python examples/robot/rakuda_lift_block.py
    python examples/robot/rakuda_lift_block.py --video docs/robots/assets/lift.mp4
    python examples/robot/rakuda_lift_block.py --seed 3

Three things about this task are easy to get wrong, and all three cost real
debugging here, so they are called out where they happen:

Solve IK on a scratch state, never the live one.
    ``env.handler.physics.data`` is the running simulation.  Writing ``qpos``
    into it to evaluate a candidate pose -- which is what an IK loop does on
    every iteration -- teleports the robot and destroys every contact it had.
    The grip looks like it fails for mysterious physical reasons.  See
    :func:`solve_ik`.

Do not pin the roll.
    The arm has six joints and cannot reach every orientation.  Asking for a
    full pose leaves the IK 17 cm short; asking only for the finger *direction*
    and leaving the rotation about it free converges to 0.1 mm.  The block is
    square, so the roll does not matter anyway.

Move the hand along a path, not the joints along a line.
    Interpolating joint angles from the home pose to a pose above the block
    sweeps the arm through the table and knocks the block over before the hand
    arrives.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List

os.environ.setdefault("MUJOCO_GL", "egl")  # no DISPLAY on most machines

try:
    import mujoco
    import numpy as np
    import torch
    from metasim.task.registry import get_task_class
except ImportError as exc:  # pragma: no cover - depends on the environment
    raise SystemExit(
        f"this example needs RoboVerse/MetaSim and MuJoCo ({exc}).\n"
        "From a robopy checkout:  pip install -e '.[roboverse]'"
    ) from exc

TASK = "rakuda.lift_block"
ROBOT = "rakuda_gripper"

#: Arm joints the IK is allowed to move.  The head is left alone and the left
#: arm stays where it started.
ARM_JOINTS = (
    "torso_yaw_dof",
    "shoulder_pitch_right_dof",
    "shoulder_roll_right_dof",
    "elbow_yaw_right_dof",
    "elbow_pitch_right_dof",
    "wrist_yaw_right_dof",
    "wrist_pitch_right_dof",
)

#: Fingers point straight down; the rotation about that axis is left free.
FINGERS_DOWN = np.array([0.0, 0.0, -1.0])

#: How far along the fingers to hold the block.
#:
#: Two constraints meet here.  The pads have to reach past the block's middle to
#: grip it, and the palm has to stay clear of the block's top face or the hand
#: drives into it on the way down.  The block is 100 mm tall, so anything under
#: 50 mm puts the palm inside it.
GRASP_DEPTH_M = 0.068


class Arm:
    """The bits of the MuJoCo model this script pokes at."""

    def __init__(self, env, robot) -> None:
        self.model = env.handler.physics.model.ptr
        self.live = env.handler.physics.data.ptr
        # Scratch state for IK.  Never solve on `live`: see the module docstring.
        self.scratch = mujoco.MjData(self.model)
        self.prefix = env.handler._mujoco_robot_names[0]
        self.joints = env.handler.get_joint_names(robot.name, sort=True)
        self.fingers = [j for j in self.joints if "right_finger" in j]
        self.home = {j: float(robot.default_joint_positions[j]) for j in self.joints}

        ident = {
            j: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.prefix + j)
            for j in self.joints
        }
        self.qadr = {j: self.model.jnt_qposadr[i] for j, i in ident.items()}
        self.dofadr = {j: self.model.jnt_dofadr[i] for j, i in ident.items()}
        self.limits = {j: tuple(self.model.jnt_range[i]) for j, i in ident.items()}
        self.palm = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, self.prefix + "right_gripper_base"
        )

    def palm_pose(self, data):
        return data.xpos[self.palm].copy(), data.xmat[self.palm].reshape(3, 3).copy()

    def action(self, q: Dict[str, float], grip: float) -> torch.Tensor:
        """A full action vector: arm from ``q``, both right fingers at ``grip``."""
        row = [grip if j in self.fingers else q.get(j, self.home[j]) for j in self.joints]
        return torch.tensor([row], dtype=torch.float32)


def solve_ik(arm: Arm, target: np.ndarray, seed: Dict[str, float], iterations: int = 600):
    """Damped least squares for the palm position and the finger direction.

    Runs entirely on ``arm.scratch``.  The orientation term only asks that the
    fingers point along :data:`FINGERS_DOWN`; the rotation about that axis is
    left free, which is what makes an otherwise unreachable pose reachable.

    Args:
        arm: The model wrapper.
        target: Where the palm should end up, in world coordinates.
        seed: Joint angles to start from.
        iterations: Cap on the solver loop.

    Returns:
        ``(joint angles, residual in metres)``.
    """
    data = arm.scratch
    q = dict(seed)
    jacp, jacr = np.zeros((3, arm.model.nv)), np.zeros((3, arm.model.nv))
    cols = [arm.dofadr[j] for j in ARM_JOINTS]

    for _ in range(iterations):
        for joint, value in q.items():
            data.qpos[arm.qadr[joint]] = value
        mujoco.mj_kinematics(arm.model, data)
        mujoco.mj_comPos(arm.model, data)
        position, rotation = arm.palm_pose(data)

        error_p = target - position
        error_w = np.cross(rotation[:, 2], FINGERS_DOWN)
        if np.linalg.norm(error_p) < 1e-4 and np.linalg.norm(error_w) < 5e-3:
            break

        mujoco.mj_jacBody(arm.model, data, jacp, jacr, arm.palm)
        jacobian = np.vstack([jacp[:, cols], jacr[:, cols]])
        step = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 0.05**2 * np.eye(6), np.concatenate([error_p, error_w])
        )
        for index, joint in enumerate(ARM_JOINTS):
            low, high = arm.limits[joint]
            q[joint] = float(
                np.clip(q[joint] + np.clip(step[index], -0.08, 0.08), low + 1e-3, high - 1e-3)
            )

    for joint, value in q.items():
        data.qpos[arm.qadr[joint]] = value
    mujoco.mj_kinematics(arm.model, data)
    return q, float(np.linalg.norm(target - data.xpos[arm.palm]))


class Run:
    """Drives the env, records frames, and remembers whether the task succeeded."""

    def __init__(self, env, arm: Arm, camera, renderer) -> None:
        self.env = env
        self.arm = arm
        self.camera = camera
        self.renderer = renderer
        self.frames: List[np.ndarray] = []
        self.log: List[str] = []
        self.q = dict(arm.home)
        self.states = None
        self.succeeded = False

    def _step(self, q, grip):
        self.states, reward, terminated, _, _ = self.env.step(self.arm.action(q, grip))
        if bool(terminated[0]):
            self.succeeded = True
        if self.renderer is not None and len(self.frames) % 1 == 0:
            self.renderer.update_scene(self.arm.live, self.camera)
            self.frames.append(self.renderer.render())
        return reward

    def _note(self, label, extra=""):
        self.log.append(
            f"{label:9} lift {float(self.env.lift(self.states)[0]):+.4f} m  "
            f"in_hand {bool(self.env.in_hand(self.states)[0])}{extra}"
        )

    def hold(self, grip, steps, label):
        """Keep one target while the servos settle."""
        for _ in range(steps):
            self._step(self.q, grip)
        self._note(label)

    def move(self, target, grip, label, waypoints=14, dwell=8):
        """Follow a straight line in space, re-solving IK at every waypoint."""
        _, _ = self.arm.palm_pose(self.arm.scratch)
        for joint, value in self.q.items():
            self.arm.scratch.qpos[self.arm.qadr[joint]] = value
        mujoco.mj_kinematics(self.arm.model, self.arm.scratch)
        here = self.arm.scratch.xpos[self.arm.palm].copy()

        worst = 0.0
        for index in range(1, waypoints + 1):
            point = here + (target - here) * (index / waypoints)
            self.q, residual = solve_ik(self.arm, point, self.q)
            worst = max(worst, residual)
            for _ in range(dwell if index < waypoints else dwell * 3):
                self._step(self.q, grip)
        self._note(label, f"  worst IK {worst:.4f} m")


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--video", default=None, help="write an mp4 here")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args(argv)

    task_cls = get_task_class(TASK)
    scenario = task_cls.scenario.update(num_envs=1, headless=True)
    env = task_cls(scenario=scenario, device="cpu")
    robot = scenario.robots[0]
    states, _ = env.reset(seed=args.seed)

    arm = Arm(env, robot)
    block = states.objects["cube"].root_state[0, :3].numpy().copy()
    print(f"robot {robot.name}, {robot.num_joints} joints, asset {robot.asset_source}")
    print(f"block at {np.round(block, 4)}")

    # Where the hand has to be, and a path to get there that misses the table.
    for joint, value in arm.home.items():
        arm.scratch.qpos[arm.qadr[joint]] = value
    mujoco.mj_kinematics(arm.model, arm.scratch)
    start = arm.scratch.xpos[arm.palm].copy()

    grasp = block + np.array([0.0, 0.0, GRASP_DEPTH_M])
    above = grasp + np.array([0.0, 0.0, 0.10])
    ceiling = grasp[2] + 0.16
    rise = np.array([(start[0] + above[0]) / 2, (start[1] + above[1]) / 2, ceiling])
    over = np.array([above[0], above[1], ceiling])
    lifted = grasp + np.array([0.0, 0.0, 0.12])

    renderer = camera = None
    if args.video:
        renderer = mujoco.Renderer(arm.model, height=args.height, width=args.width)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [0.22, block[1] * 0.5, 0.85]
        camera.distance = 1.0
        camera.azimuth = 148
        camera.elevation = -16

    run = Run(env, arm, camera, renderer)
    run.states = states

    open_q = float(robot.gripper_open_q[0])
    run.move(rise, open_q, "rise")
    run.move(over, open_q, "traverse")
    run.move(above, open_q, "descend")
    run.move(grasp, open_q, "settle", waypoints=8)
    run.hold(0.0, 90, "close")
    run.move(lifted, 0.0, "lift", waypoints=10, dwell=10)
    run.hold(0.0, 60, "hold")

    for line in run.log:
        print(" ", line)

    lift = float(env.lift(run.states)[0])
    in_hand = bool(env.in_hand(run.states)[0])
    print(f"\nlifted {lift:+.4f} m, still in the hand: {in_hand}")
    print(f"task reports success: {run.succeeded}")

    if args.video:
        import imageio

        os.makedirs(os.path.dirname(args.video) or ".", exist_ok=True)
        imageio.mimwrite(args.video, run.frames, fps=30, quality=7, macro_block_size=1)
        size = os.path.getsize(args.video) / 1e6
        print(f"wrote {args.video} ({len(run.frames)} frames, {size:.2f} MB)")

    env.close()
    return 0 if run.succeeded else 1


if __name__ == "__main__":
    sys.exit(main())
