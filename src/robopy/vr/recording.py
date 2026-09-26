"""Record a teleoperation session so it can be replayed as video.

The page cannot show the operator what the robot did: in VR they stand inside
the twin, and a headset recording shows the camera image, not the machine.
So the server keeps a log of every pose step -- joint positions, where the
controllers were (already mapped into the robot's base frame), which clutches
were held, the hand targets and the solver's residuals -- and writes it as
one JSON file when recording stops.  :mod:`robopy.vr.render` turns that file
into third-person and first-person videos; the server does so on its own
after each recording when a renderer is given.

The log is plain JSON so anything can read it::

    {"format": "robopy-vr-recording/1", "started_at": "...", "duration_s": 12.3,
     "joint_names": [...], "head": {...} | null, "anchor_m": [...] | null,
     "arms": {"mapping": "absolute", "position_scale": 1.0} | null,
     "frames": [{"t": 0.0, "q": [...], "head": {"yaw_rad": .., "pitch_rad": ..} | null,
                 "controllers": {"left": {"p_base": [...], "clutched": bool} | null, ...},
                 "targets": {"left": [...] | null, ...},
                 "ik": {"status": "...", "left_position_m": .., "right_position_m": ..} | null},
                ...]}
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Sequence

__all__ = ["RECORDING_FORMAT", "CameraTap", "PushedFrames", "SessionRecorder"]

RECORDING_FORMAT = "robopy-vr-recording/1"

logger = logging.getLogger(__name__)

#: Called with the finished recording and a progress callback (0..1); returns
#: the files it produced.
Renderer = Callable[[Path, Callable[[float], None]], Sequence[Path]]


@dataclass(frozen=True)
class _PushedFrame:
    seq: int
    data: bytes
    stamp_s: float


class PushedFrames:
    """A frame source fed by :meth:`push` -- the operator's view, sent by the page.

    Looks like a :class:`~robopy.vr.camera.FrameStreamer` to :class:`CameraTap`
    (``latest`` with ``seq`` and ``data``), so the same tap writes it to video.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: _PushedFrame | None = None
        self.pushed = 0

    def push(self, data: bytes) -> None:
        """Take one JPEG."""
        with self._lock:
            seq = 1 if self._latest is None else self._latest.seq + 1
            self._latest = _PushedFrame(seq, bytes(data), time.monotonic())
            self.pushed += 1

    @property
    def latest(self) -> _PushedFrame | None:
        """The newest frame, if any."""
        with self._lock:
            return self._latest


class CameraTap:
    """Write the camera stream to an MP4 while a recording runs.

    With a real camera on the head, the first-person video should be what
    the camera saw, not a rendering.  The tap samples the streamer's newest
    JPEG at a fixed rate from the moment the recording starts (so the video's
    clock is the recording's), decodes it and hands it to the same H.264
    writer the renderer uses.  Frames are held when the camera is slower
    than the video rate, as the renderer does with the pose log.

    Args:
        streamer: The :class:`~robopy.vr.camera.FrameStreamer` being served.
        fps: Video rate.
    """

    def __init__(self, streamer: Any, *, fps: float = 30.0) -> None:
        if fps <= 0.0:
            raise ValueError("fps must be positive.")
        self.streamer = streamer
        self.fps = fps
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._path: Path | None = None
        self._frames = 0
        self._error: str | None = None

    @property
    def path(self) -> Path | None:
        """The file being (or last) written."""
        return self._path

    @property
    def frames(self) -> int:
        """Frames written so far."""
        return self._frames

    def start(self, path: Path) -> None:
        """Begin writing to ``path``."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("the camera tap is already running")
        self._path = path
        self._frames = 0
        self._error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="vr-camera-tap", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> Dict[str, Any]:
        """Finish the file and report ``{"path", "frames", "error"}``."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout_s)
        return {
            "path": None if self._path is None else str(self._path),
            "frames": self._frames,
            "error": self._error,
        }

    def _run(self) -> None:
        import cv2
        import numpy as np

        from .render import open_video_writer

        assert self._path is not None
        writer = None
        decoded = None
        seq = 0
        period = 1.0 / self.fps
        started = time.monotonic()
        tick = 0
        try:
            while not self._stop.is_set():
                latest = self.streamer.latest
                if latest is not None and latest.seq != seq:
                    image = cv2.imdecode(np.frombuffer(latest.data, np.uint8), cv2.IMREAD_COLOR)
                    if image is not None:
                        decoded = np.ascontiguousarray(image[:, :, ::-1])  # BGR -> RGB
                        seq = latest.seq
                if decoded is not None:
                    if writer is None:
                        h, w = decoded.shape[:2]
                        writer = open_video_writer(self._path, self.fps, w, h)
                    writer.write(decoded)
                    self._frames += 1
                tick += 1
                delay = started + tick * period - time.monotonic()
                if delay > 0.0:
                    self._stop.wait(delay)
        except Exception as exc:  # noqa: BLE001 - reported, never raised into the loop
            logger.exception("camera tap failed")
            self._error = f"{type(exc).__name__}: {exc}"
        finally:
            if writer is not None:
                try:
                    writer.close()
                except Exception as exc:  # noqa: BLE001
                    self._error = self._error or f"{type(exc).__name__}: {exc}"


class SessionRecorder:
    """Collect pose-step frames between ``start()`` and ``stop()``.

    Thread-safe: the teleop socket thread adds frames while an HTTP status
    request describes the recorder and a render thread reports progress.

    Args:
        directory: Where recordings (and their videos) go; created on the
            first recording.
        render: Optional renderer run in a background thread after each
            recording is written.  Its failures are reported, never raised
            into the teleop loop.
        camera: Optional :class:`CameraTap`; the camera stream is then written
            as ``<recording>_first_person.mp4`` while recording, and the
            renderer leaves that view alone.
        operator_view: Optional :class:`CameraTap` over a :class:`PushedFrames`
            that the page feeds with what the headset shows; written as
            ``<recording>_operator_view.mp4`` while recording.
        prefix: File name prefix.
    """

    def __init__(
        self,
        directory: Path,
        *,
        render: Renderer | None = None,
        camera: CameraTap | None = None,
        operator_view: CameraTap | None = None,
        prefix: str = "rakuda-vr",
    ) -> None:
        self.directory = Path(directory)
        self.prefix = prefix
        self._render = render
        self._camera = camera
        self._operator = operator_view
        self._stem = ""
        self._lock = threading.Lock()
        self._active = False
        self._started_s = 0.0
        self._started_at = ""
        self._metadata: Dict[str, Any] = {}
        self._frames: List[Dict[str, Any]] = []
        self._last_path: Path | None = None
        self._render_state: Dict[str, Any] | None = None
        self._render_thread: threading.Thread | None = None

    # -- control ------------------------------------------------------------

    @property
    def active(self) -> bool:
        """Whether frames are being collected."""
        return self._active

    def start(self, metadata: Mapping[str, Any], now_s: float) -> Dict[str, Any]:
        """Begin a recording (a no-op when one is running).

        Args:
            metadata: Session description stored in the file's header.
            now_s: Monotonic time; frame times are relative to it.
        """
        with self._lock:
            if not self._active:
                self._active = True
                self._started_s = now_s
                started_at = datetime.now().astimezone()
                self._started_at = started_at.isoformat(timespec="seconds")
                self._stem = f"{self.prefix}-{started_at.strftime('%Y%m%d-%H%M%S')}"
                self._metadata = dict(metadata)
                self._frames = []
                if self._camera is not None or self._operator is not None:
                    self.directory.mkdir(parents=True, exist_ok=True)
                if self._camera is not None:
                    self._camera.start(self.directory / f"{self._stem}_first_person.mp4")
                if self._operator is not None:
                    self._operator.start(self.directory / f"{self._stem}_operator_view.mp4")
            return self._describe_locked()

    def push_operator_frame(self, data: bytes) -> None:
        """Take one JPEG of the operator's view from the page (ignored when not recording)."""
        if self._operator is None or not self._active:
            return
        source = self._operator.streamer
        if hasattr(source, "push"):
            source.push(data)

    def add(self, frame: Mapping[str, Any], now_s: float) -> None:
        """Append one pose step (ignored when not recording)."""
        with self._lock:
            if not self._active:
                return
            entry = dict(frame)
            entry["t"] = round(now_s - self._started_s, 4)
            self._frames.append(entry)

    def stop(self, now_s: float) -> Path | None:
        """End the recording, write it, and start rendering if configured.

        Returns:
            The recording's path, or ``None`` when nothing was recording.
        """
        with self._lock:
            if not self._active:
                return None
            self._active = False
            frames, metadata = self._frames, self._metadata
            self._frames = []
            duration = round(now_s - self._started_s, 4)
            path = self.directory / f"{self._stem}.json"
            camera_video = None
            if self._camera is not None:
                tap = self._camera.stop()
                if tap["frames"] and tap["path"] and tap["error"] is None:
                    camera_video = Path(tap["path"]).name
                    logger.info("camera video written: %s (%d frames)", tap["path"], tap["frames"])
                else:
                    logger.warning(
                        "camera video not kept (%d frames, %s)", tap["frames"], tap["error"]
                    )
            operator_video = None
            if self._operator is not None:
                tap = self._operator.stop()
                if tap["frames"] and tap["path"] and tap["error"] is None:
                    operator_video = Path(tap["path"]).name
                    logger.info("operator view written: %s (%d frames)", tap["path"], tap["frames"])
                else:
                    logger.info(
                        "operator view not kept (%d frames, %s)", tap["frames"], tap["error"]
                    )
            document = {
                "format": RECORDING_FORMAT,
                "started_at": self._started_at,
                "duration_s": duration,
                **metadata,
                "camera_video": camera_video,
                "operator_view_video": operator_video,
                "frames": frames,
            }
            self.directory.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(document, separators=(",", ":")))
            self._last_path = path
            logger.info("recording written: %s (%d frames, %.1f s)", path, len(frames), duration)
            if self._render is not None:
                self._render_state = {
                    "status": "rendering",
                    "progress": 0.0,
                    "recording": str(path),
                    "outputs": [],
                    "error": None,
                }
                self._render_thread = threading.Thread(
                    target=self._run_render, args=(path,), name="vr-render", daemon=True
                )
                self._render_thread.start()
            return path

    def wait_for_render(self, timeout_s: float | None = None) -> bool:
        """Block until the current render finishes; ``True`` if it did."""
        thread = self._render_thread
        if thread is None:
            return True
        thread.join(timeout_s)
        return not thread.is_alive()

    # -- reporting ----------------------------------------------------------

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly state: what is recording, and what was rendered."""
        with self._lock:
            return self._describe_locked()

    def _describe_locked(self) -> Dict[str, Any]:
        seconds = 0.0
        if self._active and self._frames:
            seconds = float(self._frames[-1]["t"])
        return {
            "active": self._active,
            "frames": len(self._frames) if self._active else 0,
            "seconds": seconds,
            "camera_frames": None if self._camera is None else self._camera.frames,
            "operator_frames": None if self._operator is None else self._operator.frames,
            "directory": str(self.directory),
            "last_recording": None if self._last_path is None else str(self._last_path),
            "render": None if self._render_state is None else dict(self._render_state),
        }

    def _run_render(self, path: Path) -> None:
        assert self._render is not None

        def progress(fraction: float) -> None:
            with self._lock:
                if self._render_state is not None:
                    self._render_state["progress"] = max(0.0, min(1.0, float(fraction)))

        started = time.perf_counter()
        try:
            outputs = [str(p) for p in self._render(path, progress)]
        except Exception as exc:  # noqa: BLE001 - reported to the page, never raised
            logger.exception("rendering %s failed", path)
            with self._lock:
                if self._render_state is not None:
                    self._render_state.update(status="failed", error=f"{type(exc).__name__}: {exc}")
            return
        with self._lock:
            if self._render_state is not None:
                self._render_state.update(
                    status="done",
                    progress=1.0,
                    outputs=outputs,
                    seconds=round(time.perf_counter() - started, 1),
                )
        logger.info("videos written: %s", ", ".join(outputs))
