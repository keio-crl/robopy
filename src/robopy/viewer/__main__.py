"""Command-line entry point: ``python -m robopy.viewer``."""

from __future__ import annotations

import argparse
import logging
from typing import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    """Parse arguments, load a model and serve the viewer."""
    parser = argparse.ArgumentParser(
        prog="python -m robopy.viewer",
        description=(
            "Browser viewer / simulator for the Rakuda model. Drive joint angles or "
            "end-effector targets and watch the 3D model -- no hardware involved."
        ),
    )
    from .cli import add_model_arguments, load_model

    add_model_arguments(parser)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true", help="do not open a browser tab")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from .server import serve

    loaded = load_model(args, parser)
    try:
        serve(
            loaded.bundle,
            host=args.host,
            port=args.port,
            open_browser=not args.no_browser,
            ik=loaded.ik,
        )
    finally:
        loaded.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
