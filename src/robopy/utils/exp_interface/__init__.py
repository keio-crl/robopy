"""Experiment interface modules."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .exp_handler import ExpHandler
    from .koch_exp_handler import KochExpHandler
    from .rakuda_exp_handler import RakudaExpHandler

__all__ = ["KochExpHandler", "RakudaExpHandler"]


def __getattr__(name: str) -> type[ExpHandler[Any, Any, Any, Any]]:
    """Lazy import for exp_handler modules to avoid circular imports."""
    if name == "KochExpHandler":
        from .koch_exp_handler import KochExpHandler

        return KochExpHandler
    if name == "RakudaExpHandler":
        from .rakuda_exp_handler import RakudaExpHandler

        return RakudaExpHandler
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
