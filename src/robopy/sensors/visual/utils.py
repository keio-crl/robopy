import platform
from pathlib import Path

import cv2

MAX_OPENCV_INDEX = 60


def find_camera_indices(
    max_index_search_range: int = MAX_OPENCV_INDEX,
) -> list[int]:
    """find_camera_indices _summary_

    Args:
        raise_when_empty (bool, optional): _description_. Defaults to False.
        max_index_search_range (_type_, optional): _description_. Defaults to MAX_OPENCV_INDEX.

    Raises:
        OSError: _no camera found_

    Returns:
        list[int]: _list of camera indices_

    """
    if platform.system() == "Linux":
        # Linux uses camera ports
        print(
            "Linux detected. Finding available camera indices through scanning '/dev/video*' ports",
        )
        possible_camera_ids = []
        print("Scanning '/dev/video*' ports")
        for port in Path("/dev").glob("video*"):
            camera_idx = int(str(port).replace("/dev/video", ""))
            possible_camera_ids.append(camera_idx)
    else:
        print(
            "Mac or Windows detected. Finding available camera indices through "
            f"scanning all indices from 0 to {MAX_OPENCV_INDEX}",
        )
        possible_camera_ids = list(range(max_index_search_range))

    print(f"Possible camera indices: {possible_camera_ids}")
    camera_ids: list[int] = []

    for camera_idx in possible_camera_ids:
        camera = cv2.VideoCapture(camera_idx)
        is_open = camera.isOpened()
        camera.release()

        if is_open:
            print(f"Camera found at index {camera_idx}")
            camera_ids.append(camera_idx)

    if len(camera_ids) == 0:
        err = """Not a single camera was detected. Try re-plugging, or re-installing `opencv2`,
            or your camera driver, or make sure your camera is compatible with opencv2."""
        raise OSError(err)

    return camera_ids
