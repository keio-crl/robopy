"""Camera frames -> JPEG stream for the headset.

The page shows whatever :class:`FrameStreamer` produced last; frames are never
queued, because a frame that is late is worse than a frame that is dropped when
the picture is what the operator steers by.  Any camera can feed it through
:class:`FrameSource`: OpenCV devices, the repository's ``WebCamera`` /
``RealsenseCamera`` classes via :class:`CallableFrameSource`, or the
:class:`SyntheticFrameSource` test pattern when nothing is plugged in.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Protocol

import cv2
import numpy as np
from numpy.typing import NDArray

logger = logging.getLogger(__name__)

__all__ = [
    "CallableFrameSource",
    "EncodedFrame",
    "FrameSource",
    "FrameStreamer",
    "JpegEncoder",
    "OpenCVFrameSource",
    "SyntheticFrameSource",
    "to_bgr_uint8",
]


class FrameSource(Protocol):
    """Anything that yields BGR ``uint8`` images of shape ``(H, W, 3)``."""

    def read(self) -> NDArray[np.uint8] | None:
        """Return the newest frame, or ``None`` when none is available."""
        ...

    def close(self) -> None:
        """Release the device."""
        ...


def to_bgr_uint8(frame: NDArray[Any], *, color: str = "rgb") -> NDArray[np.uint8]:
    """Normalise an image array to ``(H, W, 3)`` BGR ``uint8``.

    Accepts ``(H, W, 3)`` or ``(3, H, W)`` arrays, integer or float.  Float
    images in ``[0, 1]`` are scaled by 255; other floats are clipped to
    ``[0, 255]``.  ``color`` says which channel order the *input* uses.
    """
    array = np.asarray(frame)
    if array.ndim != 3:
        raise ValueError(f"Expected a 3-D image array, got shape {array.shape}.")
    if array.shape[0] == 3 and array.shape[2] != 3:
        array = np.transpose(array, (1, 2, 0))
    if array.shape[2] != 3:
        raise ValueError(f"Expected 3 colour channels, got shape {array.shape}.")
    if np.issubdtype(array.dtype, np.floating):
        scale = 255.0 if float(np.nanmax(array, initial=0.0)) <= 1.0 else 1.0
        array = np.clip(array * scale, 0.0, 255.0)
    out = np.ascontiguousarray(array).astype(np.uint8, copy=False)
    if color.lower() == "rgb":
        out = np.ascontiguousarray(out[:, :, ::-1])
    elif color.lower() != "bgr":
        raise ValueError("color must be 'rgb' or 'bgr'.")
    return out


class SyntheticFrameSource:
    """A moving test pattern with a clock, for sessions without a camera.

    Args:
        width: Image width in pixels.
        height: Image height in pixels.
        caption: Optional callable returning a line of text to overlay (the
            server passes the current head angles so the operator can see the
            picture react to their head).
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        *,
        caption: Callable[[], str] | None = None,
    ) -> None:
        if width < 32 or height < 32:
            raise ValueError("The synthetic frame must be at least 32x32.")
        self.width = int(width)
        self.height = int(height)
        self._caption = caption
        self._t0 = time.monotonic()
        self._count = 0

    def read(self) -> NDArray[np.uint8] | None:
        """Draw the next frame."""
        t = time.monotonic() - self._t0
        h, w = self.height, self.width
        frame = np.full((h, w, 3), (40, 44, 52), dtype=np.uint8)
        step = max(20, w // 16)
        for x in range(0, w, step):
            cv2.line(frame, (x, 0), (x, h), (70, 76, 88), 1)
        for y in range(0, h, step):
            cv2.line(frame, (0, y), (w, y), (70, 76, 88), 1)
        cx = int(w / 2 + 0.35 * w * np.sin(0.7 * t))
        cy = int(h / 2 + 0.30 * h * np.cos(0.5 * t))
        cv2.circle(frame, (cx, cy), max(8, h // 20), (255, 194, 76), -1)
        cv2.putText(
            frame,
            f"robopy synthetic camera  t={t:6.1f}s  #{self._count}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        if self._caption is not None:
            try:
                caption = self._caption()
            except Exception as exc:  # noqa: BLE001 - a caption must never kill the stream
                caption = f"caption error: {exc}"
            cv2.putText(
                frame,
                caption,
                (12, h - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (120, 220, 255),
                1,
                cv2.LINE_AA,
            )
        self._count += 1
        return frame

    def close(self) -> None:
        """Nothing to release."""


class OpenCVFrameSource:
    """A camera opened with ``cv2.VideoCapture``.

    Args:
        source: Device index, ``/dev/video*`` path, or stream URL.
        width: Requested capture width, or ``None`` to leave the default.
        height: Requested capture height.
    """

    def __init__(
        self, source: int | str, *, width: int | None = None, height: int | None = None
    ) -> None:
        self.source = source
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise OSError(f"Could not open camera {source!r}.")
        if width is not None:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        if height is not None:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))

    def read(self) -> NDArray[np.uint8] | None:
        """Grab the newest frame (BGR, as OpenCV delivers it)."""
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        return np.asarray(frame, dtype=np.uint8)

    def close(self) -> None:
        """Release the device."""
        self._cap.release()


class CallableFrameSource:
    """Adapt any zero-argument frame getter, e.g. ``camera.read``.

    Args:
        getter: Returns an image array in any layout :func:`to_bgr_uint8`
            accepts, or ``None``.
        color: Channel order the getter produces (``"rgb"`` or ``"bgr"``).
        close: Optional callable to release the device.
    """

    def __init__(
        self,
        getter: Callable[[], Any],
        *,
        color: str = "rgb",
        close: Callable[[], None] | None = None,
    ) -> None:
        self._getter = getter
        self._color = color
        self._close = close

    def read(self) -> NDArray[np.uint8] | None:
        """Fetch and normalise one frame."""
        frame = self._getter()
        return None if frame is None else to_bgr_uint8(frame, color=self._color)

    def close(self) -> None:
        """Release the underlying device, if a closer was given."""
        if self._close is not None:
            self._close()


class JpegEncoder:
    """Encode BGR frames as JPEG, optionally downscaling first.

    Args:
        quality: JPEG quality, 1..100.
        max_width: Downscale frames wider than this (keeping the aspect ratio).
    """

    def __init__(self, quality: int = 75, *, max_width: int | None = None) -> None:
        if not 1 <= quality <= 100:
            raise ValueError("JPEG quality must be within 1..100.")
        if max_width is not None and max_width < 16:
            raise ValueError("max_width must be at least 16.")
        self.quality = int(quality)
        self.max_width = max_width

    def encode(self, frame_bgr: NDArray[np.uint8]) -> tuple[bytes, int, int]:
        """Return ``(jpeg_bytes, width, height)`` of the encoded image."""
        image: NDArray[Any] = frame_bgr
        h, w = image.shape[:2]
        if self.max_width is not None and w > self.max_width:
            scale = self.max_width / w
            image = cv2.resize(image, (self.max_width, max(1, int(round(h * scale)))))
            h, w = image.shape[:2]
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            raise RuntimeError("JPEG encoding failed.")
        return bytes(buffer), int(w), int(h)


@dataclass(frozen=True)
class EncodedFrame:
    """One JPEG frame as delivered to clients.

    Attributes:
        seq: Monotonic frame counter (starts at 1).
        data: The JPEG bytes.
        stamp_s: Monotonic capture time.
        width: Encoded width.
        height: Encoded height.
    """

    seq: int
    data: bytes
    stamp_s: float
    width: int
    height: int


class FrameStreamer:
    """Capture and encode on a thread, keeping only the newest frame.

    Args:
        source: Where frames come from.
        fps: Capture rate ceiling.
        encoder: JPEG encoder; a default one when ``None``.
    """

    def __init__(
        self, source: FrameSource, *, fps: float = 30.0, encoder: JpegEncoder | None = None
    ) -> None:
        if fps <= 0.0:
            raise ValueError("fps must be positive.")
        self.source = source
        self.fps = float(fps)
        self.encoder = encoder or JpegEncoder()
        self._latest: EncodedFrame | None = None
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._read_failures = 0
        self._encode_seconds = 0.0
        self._frames = 0

    @property
    def latest(self) -> EncodedFrame | None:
        """The newest frame, if any."""
        with self._condition:
            return self._latest

    @property
    def running(self) -> bool:
        """Whether the capture thread is alive."""
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        """Start capturing."""
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="robopy-camera", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        """Stop capturing and release the source."""
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout_s)
            self._thread = None
        try:
            self.source.close()
        except Exception:  # noqa: BLE001 - closing must not raise into the server
            logger.exception("camera source close failed")

    def wait_for(self, after_seq: int, timeout_s: float) -> EncodedFrame | None:
        """Block until a frame newer than ``after_seq`` exists, or time out."""
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._latest is None or self._latest.seq <= after_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or self._stop.is_set():
                    return None
                self._condition.wait(remaining)
            return self._latest

    def capture_once(self) -> EncodedFrame | None:
        """Read and encode one frame synchronously (also used by the thread)."""
        try:
            frame = self.source.read()
        except Exception:  # noqa: BLE001 - a flaky device must not kill the stream
            logger.exception("camera read failed")
            frame = None
        if frame is None:
            self._read_failures += 1
            return None
        started = time.perf_counter()
        data, width, height = self.encoder.encode(frame)
        self._encode_seconds += time.perf_counter() - started
        with self._condition:
            seq = 1 if self._latest is None else self._latest.seq + 1
            self._latest = EncodedFrame(seq, data, time.monotonic(), width, height)
            self._frames += 1
            self._condition.notify_all()
            return self._latest

    def _run(self) -> None:
        period = 1.0 / self.fps
        next_time = time.monotonic()
        while not self._stop.is_set():
            self.capture_once()
            next_time += period
            delay = next_time - time.monotonic()
            if delay > 0.0:
                self._stop.wait(delay)
            else:
                next_time = time.monotonic()

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly statistics."""
        latest = self.latest
        return {
            "running": self.running,
            "fps_ceiling": self.fps,
            "frames": self._frames,
            "read_failures": self._read_failures,
            "mean_encode_ms": (1e3 * self._encode_seconds / self._frames if self._frames else None),
            "latest": None
            if latest is None
            else {
                "seq": latest.seq,
                "bytes": len(latest.data),
                "width": latest.width,
                "height": latest.height,
                "age_s": time.monotonic() - latest.stamp_s,
            },
            "jpeg_quality": self.encoder.quality,
            "max_width": self.encoder.max_width,
            "source": type(self.source).__name__,
        }
