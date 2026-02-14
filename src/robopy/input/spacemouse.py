"""Thread-safe SpaceMouse reader using the pyspacemouse library."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class SpaceMouseState:
    """Snapshot of the SpaceMouse 6-DOF axes and button states.

    All axis values are normalised to ``[-1.0, 1.0]``.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    buttons: list[bool] = field(default_factory=lambda: [False, False])
    timestamp: float = 0.0


class SpaceMouseReader:
    """Thread-based wrapper around *pyspacemouse*.

    A background daemon thread polls ``pyspacemouse.read()`` at high frequency
    and stores the latest state behind a lock so that callers can retrieve it
    without blocking.

    Usage::

        reader = SpaceMouseReader()
        reader.start()
        state = reader.get_state()
        reader.stop()
    """

    _POLL_INTERVAL: float = 1.0 / 200.0  # ~200 Hz

    def __init__(self) -> None:
        self._state = SpaceMouseState()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._is_running = False

    @property
    def is_running(self) -> bool:
        return self._is_running

    def start(self) -> None:
        """Open the SpaceMouse device and start the polling thread."""
        if self._is_running:
            logger.warning("SpaceMouseReader is already running.")
            return

        try:
            import pyspacemouse
        except ImportError as exc:
            raise ImportError(
                "pyspacemouse is required for SpaceMouse support. "
                "Install it with: pip install pyspacemouse"
            ) from exc

        success = pyspacemouse.open()
        if not success:
            raise RuntimeError(
                "Failed to open SpaceMouse device. "
                "Check that the device is connected and permissions are set."
            )

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        self._is_running = True
        logger.info("SpaceMouseReader started.")

    def stop(self) -> None:
        """Stop the polling thread and close the device."""
        if not self._is_running:
            return

        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        try:
            import pyspacemouse

            pyspacemouse.close()
        except Exception:
            pass

        self._is_running = False
        logger.info("SpaceMouseReader stopped.")

    def get_state(self) -> SpaceMouseState:
        """Return the latest SpaceMouse state (non-blocking)."""
        with self._lock:
            return SpaceMouseState(
                x=self._state.x,
                y=self._state.y,
                z=self._state.z,
                roll=self._state.roll,
                pitch=self._state.pitch,
                yaw=self._state.yaw,
                buttons=list(self._state.buttons),
                timestamp=self._state.timestamp,
            )

    def _poll_loop(self) -> None:
        """Background polling loop."""
        import pyspacemouse

        while not self._stop_event.is_set():
            try:
                raw = pyspacemouse.read()

                buttons = [False, False]
                if hasattr(raw, "buttons") and raw.buttons:
                    for i, val in enumerate(raw.buttons):
                        if i < 2:
                            buttons[i] = bool(val)

                with self._lock:
                    self._state.x = float(raw.x)
                    self._state.y = float(raw.y)
                    self._state.z = float(raw.z)
                    self._state.roll = float(raw.roll)
                    self._state.pitch = float(raw.pitch)
                    self._state.yaw = float(raw.yaw)
                    self._state.buttons = buttons
                    self._state.timestamp = time.time()

            except Exception as exc:
                logger.debug("SpaceMouse read error: %s", exc)

            time.sleep(self._POLL_INTERVAL)
