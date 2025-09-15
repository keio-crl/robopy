import blosc2
from numpy.typing import NDArray


class BLOSCHandler:
    def __init__(self) -> None:
        pass

    @staticmethod
    def save(data: NDArray, path: str) -> None:
        """Save data dictionary to a Blosc2 file."""
        packed = blosc2.pack_array2(data)
        if isinstance(packed, bytes):
            with open(path, "wb") as f:
                f.write(packed)
        else:
            raise ValueError("Failed to pack array: blosc2.pack_array2 returned non-bytes result")

    @staticmethod
    def load(path: str) -> NDArray:
        """Load data dictionary from a Blosc2 file."""
        with open(path, "rb") as f:
            packed = f.read()
        data = blosc2.unpack_array2(packed)
        return data
