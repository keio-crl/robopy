"""Conftest: stub hardware-only modules that cannot be installed in test env.

This must execute before any robopy submodule is imported, because the import
chain (robopy.config -> sensors -> hardware SDKs) pulls in native dependencies
that may not be available in the CI / test environment.
"""

import sys
from unittest.mock import MagicMock


def _ensure_mock(module_name: str) -> None:
    """Add a MagicMock module to sys.modules if it cannot be imported."""
    if module_name in sys.modules:
        return
    try:
        __import__(module_name)
    except (ImportError, ModuleNotFoundError):
        sys.modules[module_name] = MagicMock()


# Everything that might be missing on a minimal test runner
_STUBS = [
    "digit_interface",
    "pyaudio",
    "feetech_servo_sdk",
    "pyrealsense2",
    "dynamixel_sdk",
    "dynamixel_sdk.robotis_def",
    "pyspacemouse",
    "librosa",
    "serial",
    "serial.tools",
    "serial.tools.list_ports",
    "blosc2",
    "imageio",
    "imageio.v2",
    "scservo_sdk",
]

for _mod in _STUBS:
    _ensure_mock(_mod)

# Python 3.11 doesn't have typing.override (added in 3.12)
import typing as _typing

if not hasattr(_typing, "override"):
    _typing.override = lambda fn: fn  # type: ignore[attr-defined]
