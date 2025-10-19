import logging
from typing import Any, Dict

import h5py
import numpy as np
from numpy.typing import NDArray

logger = logging.getLogger(__name__)


class H5Handler:
    """Handler for saving and loading data using HDF5 format with h5py."""

    @staticmethod
    def save_hierarchical(data_dict: Dict[str, Any], file_path: str, compress: bool = True) -> None:
        """Save hierarchical data structure to HDF5 file.

        Args:
            data_dict (Dict[str, Any]): Hierarchical dictionary of data to save.
                Can contain nested dicts and numpy arrays.
            file_path (str): Path to save the HDF5 file.
            compress (bool): Whether to use compression. Defaults to True.

        Example:
            data = {
                'camera': {
                    'main': array_data,
                    'side': array_data,
                },
                'tactile': {
                    'left': array_data,
                    'right': array_data,
                },
                'arm': {
                    'leader': array_data,
                    'follower': array_data,
                }
            }
            H5Handler.save_hierarchical(data, 'output.h5')
        """
        compression = "gzip" if compress else None
        compression_opts = 4 if compress else None

        with h5py.File(file_path, "w") as f:
            H5Handler._save_dict_to_group(f, data_dict, compression, compression_opts)
            logger.info(f"Data saved to {file_path}")

    @staticmethod
    def _save_dict_to_group(
        group: h5py.Group,
        data_dict: Dict[str, Any],
        compression: str | None = None,
        compression_opts: int | None = None,
    ) -> None:
        """Recursively save dictionary to HDF5 group.

        Args:
            group (h5py.Group): HDF5 group to save to.
            data_dict (Dict[str, Any]): Dictionary containing data.
            compression (str | None): Compression algorithm.
            compression_opts (int | None): Compression options.
        """
        for key, value in data_dict.items():
            if isinstance(value, dict):
                # Create a subgroup for nested dictionaries
                subgroup = group.create_group(key)
                H5Handler._save_dict_to_group(subgroup, value, compression, compression_opts)
            elif isinstance(value, (np.ndarray, list)):
                # Save numpy arrays or lists as datasets
                arr = np.asarray(value, dtype=np.float32)
                if not arr.flags.c_contiguous:
                    arr = np.ascontiguousarray(arr)
                group.create_dataset(
                    key,
                    data=arr,
                    compression=compression,
                    compression_opts=compression_opts,
                )
            elif isinstance(value, (int, float, str, bytes)):
                # Save scalar values as attributes
                group.attrs[key] = value
            else:
                logger.warning(f"Skipping unsupported data type for key '{key}': {type(value)}")

    @staticmethod
    def load_hierarchical(file_path: str) -> Dict[str, Any]:
        """Load hierarchical data structure from HDF5 file.

        Args:
            file_path (str): Path to the HDF5 file.

        Returns:
            Dict[str, Any]: Hierarchical dictionary containing loaded data.

        Example:
            data = H5Handler.load_hierarchical('output.h5')
            camera_data = data['camera']['main']  # Access nested data
        """
        data_dict: Dict[str, Any] = {}

        with h5py.File(file_path, "r") as f:
            H5Handler._load_group_to_dict(f, data_dict)
            logger.info(f"Data loaded from {file_path}")

        return data_dict

    @staticmethod
    def _load_group_to_dict(group: h5py.Group, data_dict: Dict[str, Any]) -> None:
        """Recursively load HDF5 group to dictionary.

        Args:
            group (h5py.Group): HDF5 group to load from.
            data_dict (Dict[str, Any]): Dictionary to populate.
        """
        for key in group.keys():
            item = group[key]
            if isinstance(item, h5py.Group):
                # Recursively load subgroups
                data_dict[key] = {}
                H5Handler._load_group_to_dict(item, data_dict[key])
            elif isinstance(item, h5py.Dataset):
                # Load datasets as numpy arrays
                data_dict[key] = np.array(item, dtype=np.float32)

    @staticmethod
    def save_single_array(
        data: NDArray[np.float32],
        file_path: str,
        dataset_name: str = "data",
        compress: bool = True,
    ) -> None:
        """Save a single numpy array to HDF5 file.

        Args:
            data (NDArray[np.float32]): Array to save.
            file_path (str): Path to save the HDF5 file.
            dataset_name (str): Name of the dataset. Defaults to 'data'.
            compress (bool): Whether to use compression. Defaults to True.
        """
        if not data.flags.c_contiguous:
            data = np.ascontiguousarray(data)

        compression = "gzip" if compress else None
        compression_opts = 4 if compress else None

        with h5py.File(file_path, "w") as f:
            f.create_dataset(
                dataset_name,
                data=data,
                compression=compression,
                compression_opts=compression_opts,
            )
            logger.info(f"Array saved to {file_path}")

    @staticmethod
    def load_single_array(
        file_path: str,
        dataset_name: str = "data",
    ) -> NDArray[np.float32]:
        """Load a single numpy array from HDF5 file.

        Args:
            file_path (str): Path to the HDF5 file.
            dataset_name (str): Name of the dataset. Defaults to 'data'.

        Returns:
            NDArray[np.float32]: Loaded array.
        """
        with h5py.File(file_path, "r") as f:
            data = np.array(f[dataset_name], dtype=np.float32)
            logger.info(f"Array loaded from {file_path}")

        return data

    @staticmethod
    def get_info(file_path: str) -> Dict[str, Any]:
        """Get information about HDF5 file structure.

        Args:
            file_path (str): Path to the HDF5 file.

        Returns:
            Dict[str, Any]: Information about file structure and sizes.
        """
        info: Dict[str, Any] = {}

        def visit_func(name: str, obj: Any) -> None:
            if isinstance(obj, h5py.Dataset):
                info[name] = {
                    "shape": obj.shape,
                    "dtype": str(obj.dtype),
                    "size_bytes": obj.nbytes,
                }

        with h5py.File(file_path, "r") as f:
            f.visititems(visit_func)

        return info
