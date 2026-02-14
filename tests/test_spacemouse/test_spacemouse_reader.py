"""Tests for SpaceMouseReader and SpaceMouseState."""

from __future__ import annotations

import sys
import time
import types
from collections import namedtuple
from unittest.mock import MagicMock, patch

import pytest

from robopy.input.spacemouse import SpaceMouseReader, SpaceMouseState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_RawState = namedtuple(
    "_RawState", ["x", "y", "z", "roll", "pitch", "yaw", "buttons", "t"]
)


def _make_mock_pyspacemouse(
    raw_state: _RawState | None = None,
    open_success: bool = True,
) -> MagicMock:
    """Create a mock ``pyspacemouse`` module."""
    mock = MagicMock()
    mock.open.return_value = open_success

    if raw_state is None:
        raw_state = _RawState(0.1, -0.2, 0.3, 0.4, -0.5, 0.6, [1, 0], 0.0)
    mock.read.return_value = raw_state
    mock.close.return_value = None
    return mock


# ---------------------------------------------------------------------------
# SpaceMouseState
# ---------------------------------------------------------------------------


class TestSpaceMouseState:
    def test_default_state(self) -> None:
        s = SpaceMouseState()
        assert s.x == 0.0
        assert s.y == 0.0
        assert s.z == 0.0
        assert s.roll == 0.0
        assert s.pitch == 0.0
        assert s.yaw == 0.0
        assert s.buttons == [False, False]
        assert s.timestamp == 0.0

    def test_custom_state(self) -> None:
        s = SpaceMouseState(x=0.5, y=-0.3, z=1.0, roll=0.1, pitch=-0.2, yaw=0.8)
        assert s.x == 0.5
        assert s.y == -0.3
        assert s.z == 1.0


# ---------------------------------------------------------------------------
# SpaceMouseReader
# ---------------------------------------------------------------------------


class TestSpaceMouseReader:
    def test_start_imports_pyspacemouse(self) -> None:
        """start() should raise ImportError when pyspacemouse is missing."""
        # Temporarily ensure pyspacemouse is not available
        with patch.dict(sys.modules, {"pyspacemouse": None}):
            reader = SpaceMouseReader()
            with pytest.raises(ImportError, match="pyspacemouse"):
                reader.start()

    def test_start_stop_lifecycle(self) -> None:
        mock_psm = _make_mock_pyspacemouse()
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            assert not reader.is_running

            reader.start()
            assert reader.is_running
            mock_psm.open.assert_called_once()

            # Let the poll loop run a few cycles
            time.sleep(0.05)

            reader.stop()
            assert not reader.is_running

    def test_start_open_failure(self) -> None:
        mock_psm = _make_mock_pyspacemouse(open_success=False)
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            with pytest.raises(RuntimeError, match="Failed to open"):
                reader.start()
            assert not reader.is_running

    def test_double_start(self) -> None:
        mock_psm = _make_mock_pyspacemouse()
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            reader.start()
            reader.start()  # should just warn
            assert mock_psm.open.call_count == 1
            reader.stop()

    def test_get_state_returns_latest(self) -> None:
        raw = _RawState(0.1, -0.2, 0.3, 0.4, -0.5, 0.6, [1, 0], 0.0)
        mock_psm = _make_mock_pyspacemouse(raw_state=raw)
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            reader.start()
            time.sleep(0.05)

            state = reader.get_state()
            assert abs(state.x - 0.1) < 1e-6
            assert abs(state.y - (-0.2)) < 1e-6
            assert abs(state.z - 0.3) < 1e-6
            assert abs(state.roll - 0.4) < 1e-6
            assert abs(state.pitch - (-0.5)) < 1e-6
            assert abs(state.yaw - 0.6) < 1e-6
            assert state.buttons == [True, False]
            assert state.timestamp > 0

            reader.stop()

    def test_get_state_before_start(self) -> None:
        """get_state() should return defaults before start."""
        reader = SpaceMouseReader()
        state = reader.get_state()
        assert state.x == 0.0
        assert state.buttons == [False, False]

    def test_get_state_returns_copy(self) -> None:
        """Successive calls should return independent objects."""
        mock_psm = _make_mock_pyspacemouse()
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            reader.start()
            time.sleep(0.05)

            s1 = reader.get_state()
            s2 = reader.get_state()
            assert s1 is not s2
            assert s1.buttons is not s2.buttons

            reader.stop()

    def test_buttons_empty(self) -> None:
        """Handle devices that report no buttons."""
        raw = _RawState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [], 0.0)
        mock_psm = _make_mock_pyspacemouse(raw_state=raw)
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            reader.start()
            time.sleep(0.05)

            state = reader.get_state()
            assert state.buttons == [False, False]

            reader.stop()

    def test_buttons_no_attr(self) -> None:
        """Handle raw state objects without a buttons attribute."""
        RawNoButtons = namedtuple("RawNoButtons", ["x", "y", "z", "roll", "pitch", "yaw", "t"])
        raw = RawNoButtons(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        mock_psm = _make_mock_pyspacemouse(raw_state=raw)
        with patch.dict(sys.modules, {"pyspacemouse": mock_psm}):
            reader = SpaceMouseReader()
            reader.start()
            time.sleep(0.05)

            state = reader.get_state()
            assert state.buttons == [False, False]

            reader.stop()
