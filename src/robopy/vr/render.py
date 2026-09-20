"""Render a recorded VR session (:mod:`robopy.vr.recording`) as video.

Two views, each an MP4:

* **third person** -- a fixed camera in front of the robot, with the
  operator's controllers drawn where the mapping put them in the robot's
  frame (green while that clutch is held, grey otherwise) and a line from
  the hand to the controller it is chasing;
* **first person** -- the robot's head camera, i.e. what the operator was
  looking at, with the same markers.

MuJoCo does the drawing (``pip install mujoco``, or ``uv run --with mujoco``);
the model is the committed MJCF export of the Rakuda with a camera added at
``head_camera_link`` whose optical axis is derived from the forward-looking
neutral of the head, not assumed.  ``robopy-vr-render recording.json`` runs
this from the shell; the VR server runs it after each recording.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from .recording import RECORDING_FORMAT

__all__ = [
    "RenderSettings",
    "RakudaVideoRenderer",
    "render_recording",
    "select_gl_backend",
    "main",
]

VIEWS = ("third_person", "first_person")

#: Camera axes in the base frame when the head looks forward: right = -Y,
#: up = +Z, and MuJoCo cameras look along their -Z, so back = -X.
_CAMERA_AXES_FORWARD = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


@dataclass
class RenderSettings:
    """Video parameters.

    Attributes:
        fps: Output frame rate; the log is resampled onto it.
        width: Frame width in pixels.
        height: Frame height in pixels.
        fovy_deg: Vertical field of view of the head camera.  The default is
            the RealSense D435 colour sensor's 69 deg horizontal at 4:3, not a
            calibration of this camera.
        third_person_distance_m: Distance of the third-person camera from the
            robot.
        third_person_azimuth_deg: Its azimuth (MuJoCo convention; 180 looks at
            the robot's front head-on).
        third_person_elevation_deg: Its elevation (negative looks down).
        marker_radius_m: Radius of the controller markers.
        overlay: Whether to burn the time and clutch state into the frames.
    """

    fps: float = 30.0
    width: int = 640
    height: int = 480
    fovy_deg: float = 54.5
    third_person_distance_m: float = 1.6
    third_person_azimuth_deg: float = 150.0
    third_person_elevation_deg: float = -12.0
    marker_radius_m: float = 0.02
    overlay: bool = True

    def __post_init__(self) -> None:
        if self.fps <= 0.0 or self.width <= 0 or self.height <= 0:
            raise ValueError("fps, width and height must be positive.")


def select_gl_backend() -> str | None:
    """Choose MuJoCo's GL backend before it is imported; returns the setting.

    MuJoCo reads ``MUJOCO_GL`` once, when the package is first imported, and
    its default on Linux is GLFW, which needs an X display.  The server is a
    headless process (it runs in a terminal, often over SSH, while the
    operator is in a headset), so on Linux the default here is EGL, which
    renders off-screen on the GPU.  An explicit ``MUJOCO_GL`` is respected.
    """
    if "MUJOCO_GL" not in os.environ and sys.platform.startswith("linux"):
        os.environ["MUJOCO_GL"] = "egl"
    return os.environ.get("MUJOCO_GL")


def _require_mujoco() -> Any:
    backend = select_gl_backend()
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise RuntimeError(
            "rendering needs MuJoCo: pip install mujoco, or uv run --with mujoco ..."
        ) from exc
    except Exception as exc:  # pragma: no cover - a GL backend that cannot start
        raise RuntimeError(
            f"MuJoCo could not start its {backend or 'default'} GL backend: {exc}. "
            "Set MUJOCO_GL=egl (GPU, headless), osmesa (software) or glfw (needs a display)."
        ) from exc
    return mujoco


def rakuda_mjcf_path() -> Path:
    """The committed MJCF export of the Rakuda."""
    from robopy.models import find_rakuda_model

    found = find_rakuda_model()
    if found is None:  # pragma: no cover - the model ships with the package
        raise RuntimeError("the Rakuda model files were not found")
    path = found.convex_collision_urdf.parent.parent / "mjcf" / "rakuda.xml"
    if not path.is_file():
        raise RuntimeError(f"MJCF export missing: {path} (run python -m robopy.sim.mjcf_export)")
    return path


class RakudaVideoRenderer:
    """Draw frames of the Rakuda at given joint positions from either view.

    Args:
        head_neutral_rad: ``{joint: rad}`` where the head looks straight
            ahead; the head camera's orientation is solved from it.
        settings: Video parameters.
        mjcf: The model file; the committed export by default.
    """

    def __init__(
        self,
        head_neutral_rad: Mapping[str, float] | None = None,
        settings: RenderSettings | None = None,
        *,
        mjcf: Path | None = None,
    ) -> None:
        mujoco = _require_mujoco()
        self.settings = settings or RenderSettings()
        self.mujoco = mujoco
        spec = mujoco.MjSpec.from_file(str(mjcf or rakuda_mjcf_path()))
        neutral = dict(head_neutral_rad or {})
        quat = self._head_camera_quat(spec, neutral)
        spec.body("head_camera_link").add_camera(
            name="head_camera", pos=[0.0, 0.0, 0.0], quat=list(quat), fovy=self.settings.fovy_deg
        )
        directional = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        spec.worldbody.add_light(
            name="key",
            pos=[1.0, -1.0, 2.0],
            dir=[-0.5, 0.5, -1.0],
            type=directional,
            diffuse=[0.8, 0.8, 0.8],
            specular=[0.2, 0.2, 0.2],
            castshadow=0,
        )
        spec.worldbody.add_light(
            name="fill",
            pos=[-1.0, 1.0, 1.5],
            dir=[0.5, -0.5, -0.8],
            type=directional,
            diffuse=[0.4, 0.4, 0.4],
            castshadow=0,
        )
        spec.worldbody.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[10.0, 10.0, 0.05],
            pos=[0.0, 0.0, -0.7],
            rgba=[0.55, 0.58, 0.62, 1.0],
            contype=0,
            conaffinity=0,
        )
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._qpos_index = {
            self.model.joint(i).name: int(self.model.joint(i).qposadr[0])
            for i in range(self.model.njnt)
        }
        self._sites = {
            side: self.model.site(f"gripper_{side}").id
            for side in ("left", "right")
            if any(self.model.site(i).name == f"gripper_{side}" for i in range(self.model.nsite))
        }
        try:
            self._renderer = mujoco.Renderer(
                self.model, height=self.settings.height, width=self.settings.width
            )
        except Exception as exc:  # mujoco.FatalError is not an ImportError
            raise RuntimeError(
                f"MuJoCo could not create an off-screen GL context (MUJOCO_GL="
                f"{os.environ.get('MUJOCO_GL', 'unset')}): {exc}. On a headless Linux box "
                "use MUJOCO_GL=egl (GPU) or MUJOCO_GL=osmesa (software)."
            ) from exc
        self._third = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self._third)
        self._third.lookat[:] = [0.1, 0.0, 0.0]
        self._third.distance = self.settings.third_person_distance_m
        self._third.azimuth = self.settings.third_person_azimuth_deg
        self._third.elevation = self.settings.third_person_elevation_deg
        self._first = mujoco.MjvCamera()
        self._first.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self._first.fixedcamid = self.model.camera("head_camera").id

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Joints the model knows (the URDF names)."""
        return tuple(self._qpos_index)

    def _head_camera_quat(self, spec: Any, neutral: Mapping[str, float]) -> NDArray[np.float64]:
        """Orientation of a camera in ``head_camera_link`` that looks forward at ``neutral``."""
        mujoco = self.mujoco
        model = spec.compile()
        data = mujoco.MjData(model)
        for name, value in neutral.items():
            for i in range(model.njnt):
                if model.joint(i).name == name:
                    data.qpos[model.joint(i).qposadr[0]] = value
        mujoco.mj_forward(model, data)
        R_body = np.array(data.body("head_camera_link").xmat).reshape(3, 3)
        R_body_cam = R_body.T @ _CAMERA_AXES_FORWARD
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_body_cam.flatten())
        return quat

    def close(self) -> None:
        """Free the GL context."""
        self._renderer.close()

    def set_joints(self, positions_rad: Mapping[str, float]) -> None:
        """Pose the model (unknown joints are ignored) and run forward kinematics."""
        for name, value in positions_rad.items():
            index = self._qpos_index.get(name)
            if index is not None:
                self.data.qpos[index] = float(value)
        self.mujoco.mj_forward(self.model, self.data)

    def hand_position(self, side: str) -> NDArray[np.float64] | None:
        """The gripper site's position after :meth:`set_joints`, if the model has it."""
        site = self._sites.get(side)
        if site is None:
            return None
        return np.array(self.data.site_xpos[site], dtype=np.float64)

    def render(
        self,
        view: str,
        *,
        controllers: Mapping[str, Mapping[str, Any] | None] | None = None,
        targets: Mapping[str, Sequence[float] | None] | None = None,
    ) -> NDArray[np.uint8]:
        """One RGB frame of the posed model.

        Args:
            view: ``"third_person"`` or ``"first_person"``.
            controllers: ``{side: {"p_base": [x, y, z], "clutched": bool} | None}``
                markers to draw.
            targets: ``{side: [x, y, z] | None}`` hand targets to draw.
        """
        if view not in VIEWS:
            raise ValueError(f"view must be one of {VIEWS}, got {view!r}")
        scene = self._renderer.scene
        self._renderer.update_scene(
            self.data, self._third if view == "third_person" else self._first
        )
        radius = self.settings.marker_radius_m
        for side, entry in (controllers or {}).items():
            if not entry or entry.get("p_base") is None:
                continue
            p = np.asarray(entry["p_base"], dtype=np.float64)
            clutched = bool(entry.get("clutched"))
            colour = (0.2, 0.9, 0.3, 0.95) if clutched else (0.6, 0.6, 0.6, 0.45)
            self._sphere(scene, p, radius, colour)
            hand = self.hand_position(side)
            if clutched and hand is not None:
                self._line(scene, hand, p, (0.2, 0.9, 0.3, 0.8))
        for side, target in (targets or {}).items():
            if target is None:
                continue
            self._sphere(
                scene, np.asarray(target, dtype=np.float64), radius * 0.6, (0.3, 0.5, 1.0, 0.9)
            )
        frame = self._renderer.render()
        return np.ascontiguousarray(frame, dtype=np.uint8)

    def _sphere(
        self, scene: Any, p: NDArray[np.float64], radius: float, rgba: Sequence[float]
    ) -> None:
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco = self.mujoco
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, radius, radius]),
            p,
            np.eye(3).flatten(),
            np.asarray(rgba, dtype=np.float32),
        )
        scene.ngeom += 1

    def _line(
        self, scene: Any, a: NDArray[np.float64], b: NDArray[np.float64], rgba: Sequence[float]
    ) -> None:
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco = self.mujoco
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).flatten(),
            np.asarray(rgba, dtype=np.float32),
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.004, a, b)
        scene.ngeom += 1


# ---------------------------------------------------------------- recording -> video


def load_recording(path: Path) -> Dict[str, Any]:
    """Read and check a recording file."""
    document = json.loads(Path(path).read_text())
    if document.get("format") != RECORDING_FORMAT:
        raise ValueError(f"{path}: not a {RECORDING_FORMAT} file")
    if not document.get("frames"):
        raise ValueError(f"{path}: the recording has no frames")
    return document


def resample(frames: Sequence[Mapping[str, Any]], fps: float) -> List[Mapping[str, Any]]:
    """The frame in force at each output instant (sample-and-hold)."""
    if not frames:
        return []
    out: List[Mapping[str, Any]] = []
    duration = float(frames[-1]["t"])
    count = max(1, int(math.floor(duration * fps)) + 1)
    j = 0
    for k in range(count):
        t = k / fps
        while j + 1 < len(frames) and float(frames[j + 1]["t"]) <= t:
            j += 1
        out.append(frames[j])
    return out


def _overlay(frame: NDArray[np.uint8], lines: Sequence[str]) -> NDArray[np.uint8]:
    try:
        import cv2
    except ImportError:  # pragma: no cover - opencv is a base dependency
        return frame
    out = np.ascontiguousarray(frame)
    for i, text in enumerate(lines):
        y = 22 + 20 * i
        cv2.putText(
            out, text, (9, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA
        )
        cv2.putText(
            out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
        )
    return out


class _Mp4Writer:
    """OpenCV's writer (``mp4v``), the encoder every install of robopy has."""

    def __init__(self, path: Path, fps: float, width: int, height: int) -> None:
        import cv2

        self._cv2 = cv2
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # type: ignore[attr-defined]
        self._writer = cv2.VideoWriter(str(path), fourcc, fps, (width, height))
        if not self._writer.isOpened():
            raise RuntimeError(f"could not open {path} for writing")

    def write(self, rgb: NDArray[np.uint8]) -> None:
        self._writer.write(self._cv2.cvtColor(rgb, self._cv2.COLOR_RGB2BGR))

    def close(self) -> None:
        self._writer.release()


def render_recording(
    recording: Path,
    out_dir: Path | None = None,
    *,
    settings: RenderSettings | None = None,
    views: Sequence[str] = VIEWS,
    progress: Callable[[float], None] | None = None,
) -> List[Path]:
    """Write one MP4 per view next to (or in ``out_dir`` for) a recording.

    Args:
        recording: The JSON file written by :class:`~robopy.vr.recording.SessionRecorder`.
        out_dir: Output directory; the recording's by default.
        settings: Video parameters.
        views: Which of :data:`VIEWS` to render.
        progress: Called with the fraction done, for a progress display.

    Returns:
        The video paths, in the order of ``views``.
    """
    recording = Path(recording)
    document = load_recording(recording)
    settings = settings or RenderSettings()
    for view in views:
        if view not in VIEWS:
            raise ValueError(f"unknown view {view!r}; choose from {VIEWS}")
    joint_names = list(document.get("joint_names") or [])
    head = document.get("head") or {}
    neutral = {}
    if head.get("yaw_joint"):
        neutral[head["yaw_joint"]] = float(head.get("yaw_neutral_rad") or 0.0)
    if head.get("pitch_joint"):
        neutral[head["pitch_joint"]] = float(head.get("pitch_neutral_rad") or 0.0)
    frames = resample(document["frames"], settings.fps)
    directory = Path(out_dir) if out_dir is not None else recording.parent
    directory.mkdir(parents=True, exist_ok=True)
    renderer = RakudaVideoRenderer(neutral, settings)
    outputs: List[Path] = []
    total = len(frames) * len(views)
    done = 0
    try:
        for view in views:
            path = directory / f"{recording.stem}_{view}.mp4"
            writer = _Mp4Writer(path, settings.fps, settings.width, settings.height)
            try:
                for k, frame in enumerate(frames):
                    q = frame.get("q") or []
                    renderer.set_joints(dict(zip(joint_names, q)))
                    image = renderer.render(
                        view, controllers=frame.get("controllers"), targets=frame.get("targets")
                    )
                    if settings.overlay:
                        image = _overlay(image, _caption(k / settings.fps, frame))
                    writer.write(image)
                    done += 1
                    if progress is not None and (done % 10 == 0 or done == total):
                        progress(done / total)
            finally:
                writer.close()
            outputs.append(path)
    finally:
        renderer.close()
    return outputs


def _caption(t: float, frame: Mapping[str, Any]) -> List[str]:
    parts = [f"t = {t:6.2f} s"]
    controllers = frame.get("controllers") or {}
    ik = frame.get("ik") or {}
    for side in ("left", "right"):
        entry = controllers.get(side)
        if not entry:
            state = "no controller"
        elif entry.get("clutched"):
            residual = ik.get(f"{side}_position_m")
            state = "CLUTCHED" + ("" if residual is None else f"  {residual * 1e3:.0f} mm")
        else:
            state = "idle"
        parts.append(f"{side[0].upper()}: {state}")
    return ["    ".join(parts)]


def main(argv: Sequence[str] | None = None) -> int:
    """``robopy-vr-render``: recordings in, videos out."""
    parser = argparse.ArgumentParser(
        prog="robopy-vr-render",
        description="Render a robopy VR recording as third-person and first-person MP4s.",
    )
    parser.add_argument("recording", type=Path, nargs="+", help="recording JSON file(s)")
    parser.add_argument("--out-dir", type=Path, default=None, help="output directory")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--size", default="640x480", metavar="WxH")
    parser.add_argument(
        "--view", action="append", choices=VIEWS, default=None, help="render only this view"
    )
    parser.add_argument("--azimuth", type=float, default=150.0, help="third-person camera azimuth")
    parser.add_argument("--elevation", type=float, default=-12.0)
    parser.add_argument("--distance", type=float, default=1.6)
    parser.add_argument(
        "--fovy",
        type=float,
        default=54.5,
        help="vertical field of view of the first-person view, degrees (D435 colour at 4:3)",
    )
    parser.add_argument("--no-overlay", action="store_true", help="no burnt-in captions")
    args = parser.parse_args(argv)
    try:
        width, height = (int(v) for v in args.size.lower().split("x"))
    except ValueError:
        parser.error(f"--size expects WxH, got {args.size!r}")
    settings = RenderSettings(
        fps=args.fps,
        width=width,
        height=height,
        third_person_azimuth_deg=args.azimuth,
        third_person_elevation_deg=args.elevation,
        third_person_distance_m=args.distance,
        fovy_deg=args.fovy,
        overlay=not args.no_overlay,
    )
    for recording in args.recording:
        outputs = render_recording(
            recording, args.out_dir, settings=settings, views=args.view or VIEWS
        )
        for path in outputs:
            print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
