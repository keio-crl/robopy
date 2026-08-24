"""USB serial port tuning for Dynamixel/Feetech buses.

On Linux the single largest source of latency in a servo control loop is not
the wire time of the packets, it is the **USB latency timer** of the USB-serial
bridge (the U2D2 uses an FTDI chip).  Its default is 16 ms, so every round trip
is padded with up to 16 ms of pure waiting.  Both the Python and the C++
DynamixelSDK hard-code that value in their timeout maths (``LATENCY_TIMER =
16``) but neither of them changes it -- lowering it is left to the application.

Two mechanisms are tried, because availability depends on driver and
permissions:

1. ``/sys/bus/usb-serial/devices/<tty>/latency_timer`` -- the ``ftdi_sio``
   attribute.  Needs write permission (root, or a udev rule).
2. ``TIOCSSERIAL`` with ``ASYNC_LOW_LATENCY`` -- needs no privileges and has the
   same practical effect for ``ftdi_sio``.

To make it permanent, install a udev rule::

    echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \\
      | sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
"""

from __future__ import annotations

import logging
import os
import sys
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# From <linux/serial.h>; the flags field is the 5th int of `struct serial_struct`,
# which is how pyserial reaches it as well.
_TIOCGSERIAL = 0x541E
_TIOCSSERIAL = 0x541F
_ASYNC_LOW_LATENCY = 0x2000
_SERIAL_STRUCT_FLAGS_INDEX = 4


@dataclass(frozen=True)
class PortTuning:
    """What actually happened when lowering a port's latency timer."""

    sysfs_written: bool = False
    low_latency_flag_set: bool = False
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.sysfs_written or self.low_latency_flag_set


def _tty_name(port: str) -> str:
    return port.rsplit("/", 1)[-1]


def _latency_sysfs_path(port: str) -> str:
    return f"/sys/bus/usb-serial/devices/{_tty_name(port)}/latency_timer"


def get_latency_timer(port: str) -> int:
    """Current ftdi_sio latency timer in ms, or ``-1`` when it cannot be read."""
    try:
        with open(_latency_sysfs_path(port)) as handle:
            return int(handle.read().strip())
    except (OSError, ValueError):
        return -1


def set_latency_timer(port: str, latency_ms: int = 1) -> PortTuning:
    """Best-effort lowering of a port's USB latency timer. Never raises.

    Args:
        port: Serial device path, e.g. ``/dev/ttyUSB0``.
        latency_ms: Target latency in ms. ``1`` is the useful value; the
            ``ASYNC_LOW_LATENCY`` fallback is only attempted for ``<= 1``.

    Returns:
        A :class:`PortTuning` describing which mechanisms succeeded.
    """
    if not sys.platform.startswith("linux"):
        return PortTuning(detail="latency timer tuning is only implemented on Linux")

    sysfs_written = False
    detail = ""
    path = _latency_sysfs_path(port)
    try:
        with open(path, "w") as handle:
            handle.write(str(latency_ms))
        sysfs_written = True
    except OSError as error:
        detail = f"{path}: {error}"

    low_latency_flag_set = False
    if latency_ms <= 1:
        low_latency_flag_set, ioctl_detail = _set_low_latency_flag(port)
        if not low_latency_flag_set and not detail:
            detail = ioctl_detail

    result = PortTuning(sysfs_written, low_latency_flag_set, detail)
    if result.ok:
        logger.info(f"Lowered USB latency timer of {port} to {latency_ms}ms ({result}).")
    else:
        logger.warning(
            f"Could not lower the USB latency timer of {port} ({detail}). "
            f"Expect up to 16ms of extra latency per transaction; see "
            f"robopy.motor.port_tuning for the udev rule that fixes this permanently."
        )
    return result


def _set_low_latency_flag(port: str) -> tuple[bool, str]:
    """Set ASYNC_LOW_LATENCY via TIOCSSERIAL. Returns (succeeded, detail)."""
    import array
    import fcntl

    fd = None
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        buffer = array.array("i", [0] * 32)
        fcntl.ioctl(fd, _TIOCGSERIAL, buffer, True)
        buffer[_SERIAL_STRUCT_FLAGS_INDEX] |= _ASYNC_LOW_LATENCY
        fcntl.ioctl(fd, _TIOCSSERIAL, buffer)
        return True, ""
    except OSError as error:
        return False, f"TIOCSSERIAL on {port}: {error}"
    finally:
        if fd is not None:
            os.close(fd)
