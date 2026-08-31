"""Shared frame post-processing for the camera implementations.

Cameras hand out **uint8** colour frames in CHW layout, i.e. exactly the bytes
the sensor produced, only reordered.  The previous behaviour was to widen every
frame to float32, which cost about 2.3 ms and 2.8 MB of extra memory traffic per
640x480 frame and quadrupled the size of every recording, without adding any
information: the samples are 8 bit to begin with.

Normalisation is deliberately left to the consumer.  Training code wants
different things (0..1, ImageNet statistics, ...) and doing it here forces the
most expensive representation on everybody, including the recording pipeline
that only ever writes the frames to disk.

Depth frames stay uint16, which is the ``z16`` format the RealSense reports and
is already in millimetres.
"""

from typing import Literal

import cv2
import numpy as np
from numpy.typing import NDArray

ColorMode = Literal["rgb", "bgr"]

#: A colour frame: uint8, CHW (channels, height, width).
ColorFrame = NDArray[np.uint8]
#: A depth frame: uint16 millimetres, CHW with a single channel.
DepthFrame = NDArray[np.uint16]

_COLOR_CONVERSIONS = {
    ("rgb", "bgr"): cv2.COLOR_RGB2BGR,
    ("bgr", "rgb"): cv2.COLOR_BGR2RGB,
}


def to_chw_color(
    image: NDArray[np.generic],
    source_color: ColorMode,
    target_color: ColorMode,
) -> ColorFrame:
    """Convert a raw HWC camera frame to a contiguous uint8 CHW frame.

    Args:
        image: Frame as delivered by the driver, HWC with 1 or 3 channels.
        source_color: Channel order the driver produced.
        target_color: Channel order the caller wants.

    Returns:
        A C-contiguous ``uint8`` array in CHW layout.
    """
    frame = np.asarray(image)
    if frame.dtype != np.uint8:
        # Drivers hand out uint8; anything else would silently change the value
        # range, so clip into range rather than wrapping around.
        frame = np.clip(frame, 0, 255).astype(np.uint8)

    conversion = _COLOR_CONVERSIONS.get((source_color, target_color))
    if conversion is not None and frame.ndim == 3 and frame.shape[-1] == 3:
        frame = cv2.cvtColor(frame, conversion)

    if frame.ndim == 2:
        frame = frame[np.newaxis, ...]
    elif frame.shape[-1] in (1, 3):
        frame = frame.transpose(2, 0, 1)

    # transpose() returns a view; copy once so consumers (h5py, blosc2, torch)
    # do not each have to.
    return np.ascontiguousarray(frame)


def to_chw_depth(depth: NDArray[np.generic], max_depth: float) -> DepthFrame:
    """Convert a raw HW depth frame to a clipped uint16 CHW frame.

    Args:
        depth: Depth frame in millimetres, HW or HWC with a single channel.
        max_depth: Upper clipping bound in millimetres.

    Returns:
        A C-contiguous ``uint16`` array of shape ``(1, H, W)``.
    """
    frame = np.asarray(depth)
    if frame.ndim == 3 and frame.shape[-1] == 1:
        frame = frame[..., 0]
    if frame.ndim != 2:
        raise ValueError(f"Expected a 2D depth frame, got shape {frame.shape}.")

    # Clip before narrowing so that out-of-range samples saturate instead of
    # wrapping around, and keep the bound an int so numpy does not promote.
    bound = int(min(max(max_depth, 0.0), float(np.iinfo(np.uint16).max)))
    clipped = np.clip(frame, 0, bound).astype(np.uint16)
    return np.ascontiguousarray(clipped[np.newaxis, ...])
