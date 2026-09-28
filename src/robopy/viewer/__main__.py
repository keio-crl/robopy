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
    parser.add_argument(
        "--mirror-follower",
        metavar="PORT",
        default=None,
        help="read the real follower's joint angles on PORT (needs --config) so the page can "
        "draw the model in the machine's pose. READ-ONLY: nothing is written to a motor",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    if args.mirror_follower and not args.config:
        parser.error("--mirror-follower needs --config (the follower's joint calibration)")
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from .server import serve

    loaded = load_model(args, parser)
    machine = None
    try:
        if args.mirror_follower:
            from .machine_mirror import open_follower_mirror

            try:
                machine = open_follower_mirror(
                    args.mirror_follower,
                    loaded.config.follower_joint_calibration,
                    known_urdf_joints=loaded.bundle.model.movable_joint_names,
                )
            except Exception as exc:  # noqa: BLE001 - a clear message, then exit
                parser.exit(1, f"cannot mirror the follower on {args.mirror_follower}: {exc}\n")
            machine.start()
        serve(
            loaded.bundle,
            host=args.host,
            port=args.port,
            open_browser=not args.no_browser,
            ik=loaded.ik,
            machine=machine,
        )
    finally:
        if machine is not None:
            machine.stop()
        loaded.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
