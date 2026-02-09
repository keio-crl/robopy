from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .blosc_handler import BLOSCHandler
from .exp_interface.meta_data_config import MetaDataConfig
from .find_usb_port import find_port
from .h5_handler import H5Handler
from .worker.koch_save_worker import KochSaveWorker
from .worker.rakuda_save_worker import RakudaSaveWorker
from .worker.save_worker import SaveWorker

if TYPE_CHECKING:
    from .exp_interface.exp_handler import ExpHandler
    from .exp_interface.koch_exp_handler import KochExpHandler
    from .exp_interface.rakuda_exp_handler import RakudaExpHandler

__all__ = [
    "find_port",
    "BLOSCHandler",
    "H5Handler",
    "SaveWorker",
    "KochSaveWorker",
    "RakudaSaveWorker",
    "KochExpHandler",
    "RakudaExpHandler",
    "MetaDataConfig",
]


def __getattr__(name: str) -> type[ExpHandler[Any, Any, Any, Any]]:
    """Lazy import for exp_interface modules to avoid circular imports."""
    if name == "KochExpHandler":
        from .exp_interface.koch_exp_handler import KochExpHandler

        return KochExpHandler
    if name == "RakudaExpHandler":
        from .exp_interface.rakuda_exp_handler import RakudaExpHandler

        return RakudaExpHandler
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
