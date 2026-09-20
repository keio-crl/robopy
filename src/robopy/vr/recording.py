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
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Sequence

__all__ = ["RECORDING_FORMAT", "SessionRecorder"]

RECORDING_FORMAT = "robopy-vr-recording/1"

logger = logging.getLogger(__name__)

#: Called with the finished recording and a progress callback (0..1); returns
#: the files it produced.
Renderer = Callable[[Path, Callable[[float], None]], Sequence[Path]]


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
        prefix: File name prefix.
    """

    def __init__(
        self, directory: Path, *, render: Renderer | None = None, prefix: str = "rakuda-vr"
    ) -> None:
        self.directory = Path(directory)
        self.prefix = prefix
        self._render = render
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
                self._started_at = datetime.now().astimezone().isoformat(timespec="seconds")
                self._metadata = dict(metadata)
                self._frames = []
            return self._describe_locked()

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
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            path = self.directory / f"{self.prefix}-{stamp}.json"
            document = {
                "format": RECORDING_FORMAT,
                "started_at": self._started_at,
                "duration_s": duration,
                **metadata,
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
