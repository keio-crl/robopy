"""The uint8 contract for camera frames, and its round trip through storage.

Camera frames are 8 bit at the sensor, so robopy keeps them 8 bit: widening them
to float32 costs ~2.3ms and 2.8MB per 640x480 frame and quadruples every
recording without adding information. These tests pin that down end to end --
frame post-processing, HDF5 round trip and animation preprocessing -- none of
which needs a camera attached.
"""

from __future__ import annotations

import numpy as np
import pytest

from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.visual.frame_ops import ColorMode, to_chw_color, to_chw_depth
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.utils.h5_handler import H5Handler
from robopy.utils.worker.rakuda_save_worker import _to_displayable

HEIGHT, WIDTH = 12, 16


def rgb_frame() -> np.ndarray:
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, (HEIGHT, WIDTH, 3), dtype=np.uint8)


# --------------------------------------------------------------------------- #
# frame_ops
# --------------------------------------------------------------------------- #


def test_to_chw_color_keeps_uint8_and_reorders_axes() -> None:
    frame = rgb_frame()

    result = to_chw_color(frame, source_color="rgb", target_color="rgb")

    assert result.dtype == np.uint8
    assert result.shape == (3, HEIGHT, WIDTH)
    assert result.flags.c_contiguous
    np.testing.assert_array_equal(result, frame.transpose(2, 0, 1))


def test_to_chw_color_swaps_channels_when_modes_differ() -> None:
    frame = rgb_frame()

    result = to_chw_color(frame, source_color="rgb", target_color="bgr")

    assert result.dtype == np.uint8
    # RGB -> BGR reverses the channel axis.
    np.testing.assert_array_equal(result, frame.transpose(2, 0, 1)[::-1])


def test_to_chw_color_handles_single_channel_frames() -> None:
    frame = np.arange(HEIGHT * WIDTH, dtype=np.uint8).reshape(HEIGHT, WIDTH)

    result = to_chw_color(frame, source_color="rgb", target_color="rgb")

    assert result.shape == (1, HEIGHT, WIDTH)
    assert result.dtype == np.uint8


def test_to_chw_color_clips_instead_of_wrapping_a_wider_input() -> None:
    frame = np.full((HEIGHT, WIDTH, 3), 300, dtype=np.int32)

    result = to_chw_color(frame, source_color="rgb", target_color="rgb")

    assert result.dtype == np.uint8
    assert result.max() == 255  # 300 saturates rather than wrapping to 44


def test_to_chw_depth_keeps_uint16_and_clips() -> None:
    depth = np.array([[0, 500, 5000]], dtype=np.uint16)

    result = to_chw_depth(depth, max_depth=2000.0)

    assert result.dtype == np.uint16
    assert result.shape == (1, 1, 3)
    np.testing.assert_array_equal(result[0, 0], [0, 500, 2000])


def test_to_chw_depth_accepts_a_trailing_channel_axis() -> None:
    depth = np.zeros((HEIGHT, WIDTH, 1), dtype=np.uint16)

    result = to_chw_depth(depth, max_depth=2000.0)

    assert result.shape == (1, HEIGHT, WIDTH)


def test_to_chw_depth_rejects_unexpected_shapes() -> None:
    with pytest.raises(ValueError):
        to_chw_depth(np.zeros((2, HEIGHT, WIDTH), dtype=np.uint16), max_depth=2000.0)


# --------------------------------------------------------------------------- #
# RealsenseCamera post-processing (no device needed)
# --------------------------------------------------------------------------- #


def realsense_stub(color_mode: ColorMode = "rgb") -> RealsenseCamera:
    """A RealsenseCamera with only what _postprocess_image touches."""
    camera = object.__new__(RealsenseCamera)
    camera.config = RealsenseCameraConfig(
        name="test", width=WIDTH, height=HEIGHT, color_mode=color_mode
    )
    # __del__ runs on collection and reads these.
    camera.name = "test"
    camera._is_connected = False
    return camera


def test_realsense_postprocess_returns_uint8_chw() -> None:
    frame = rgb_frame()

    result = realsense_stub()._postprocess_image(frame)

    assert result.dtype == np.uint8
    assert result.shape == (3, HEIGHT, WIDTH)
    np.testing.assert_array_equal(result, frame.transpose(2, 0, 1))


def test_realsense_postprocess_converts_to_bgr_when_configured() -> None:
    frame = rgb_frame()

    result = realsense_stub(color_mode="bgr")._postprocess_image(frame)

    np.testing.assert_array_equal(result, frame.transpose(2, 0, 1)[::-1])


def test_realsense_postprocess_rejects_a_wrong_resolution() -> None:
    frame = np.zeros((HEIGHT + 1, WIDTH, 3), dtype=np.uint8)

    with pytest.raises(OSError):
        realsense_stub()._postprocess_image(frame)


# --------------------------------------------------------------------------- #
# Storage round trip
# --------------------------------------------------------------------------- #


def test_h5_preserves_frame_dtypes(tmp_path) -> None:
    path = str(tmp_path / "obs.h5")
    data = {
        "camera": {"main": np.zeros((2, 3, HEIGHT, WIDTH), dtype=np.uint8)},
        "camera_depth": {"main": np.zeros((2, 1, HEIGHT, WIDTH), dtype=np.uint16)},
        "arm": {"leader": np.zeros((2, 17), dtype=np.float32)},
    }

    H5Handler.save_hierarchical(data, path)
    loaded = H5Handler.load_hierarchical(path)

    assert loaded["camera"]["main"].dtype == np.uint8
    assert loaded["camera_depth"]["main"].dtype == np.uint16
    assert loaded["arm"]["leader"].dtype == np.float32


def test_h5_uint8_frames_are_a_quarter_of_the_float32_size(tmp_path) -> None:
    frames = np.random.default_rng(1).integers(0, 256, (4, 3, 32, 32), dtype=np.uint8)

    uint8_path = str(tmp_path / "uint8.h5")
    float_path = str(tmp_path / "float32.h5")
    H5Handler.save_hierarchical({"camera": {"main": frames}}, uint8_path, compress=False)
    H5Handler.save_hierarchical(
        {"camera": {"main": frames.astype(np.float32)}}, float_path, compress=False
    )

    assert H5Handler.get_info(uint8_path)["camera/main"]["size_bytes"] * 4 == (
        H5Handler.get_info(float_path)["camera/main"]["size_bytes"]
    )


def test_h5_single_array_round_trips_uint8(tmp_path) -> None:
    path = str(tmp_path / "frames.h5")
    frames = np.arange(24, dtype=np.uint8).reshape(2, 3, 2, 2)

    H5Handler.save_single_array(frames, path)
    loaded = H5Handler.load_single_array(path)

    assert loaded.dtype == np.uint8
    np.testing.assert_array_equal(loaded, frames)


def test_h5_still_defaults_plain_lists_to_float32(tmp_path) -> None:
    path = str(tmp_path / "list.h5")

    H5Handler.save_hierarchical({"arm": {"leader": [[1.0, 2.0], [3.0, 4.0]]}}, path)

    assert H5Handler.load_hierarchical(path)["arm"]["leader"].dtype == np.float32


# --------------------------------------------------------------------------- #
# Animation preprocessing
# --------------------------------------------------------------------------- #


def test_displayable_passes_uint8_through_without_widening() -> None:
    frames = np.random.default_rng(2).integers(0, 256, (2, 3, HEIGHT, WIDTH), dtype=np.uint8)

    result = _to_displayable(frames)

    assert result.dtype == np.uint8
    assert result.shape == (2, HEIGHT, WIDTH, 3)


def test_displayable_normalises_float_frames_in_float32() -> None:
    frames = np.full((2, 3, HEIGHT, WIDTH), 255.0, dtype=np.float32)

    result = _to_displayable(frames)

    assert result.dtype == np.float32
    assert result.max() == pytest.approx(1.0)
