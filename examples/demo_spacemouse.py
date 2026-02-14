#!/usr/bin/env python3
"""Demo script to test SpaceMouse input.

This script continuously reads and displays SpaceMouse state until interrupted.
Press Ctrl+C to stop.

Usage:
    python examples/demo_spacemouse.py
"""

from __future__ import annotations

import sys
import time

try:
    from robopy.input.spacemouse import SpaceMouseReader
except ImportError:
    print("Error: Could not import SpaceMouseReader.")
    print("Make sure robopy is installed: pip install -e .")
    sys.exit(1)


def main() -> None:
    """Run the SpaceMouse demo."""
    print("SpaceMouse Demo")
    print("=" * 60)
    print("Starting SpaceMouse reader...")
    print("Move the SpaceMouse to see values.")
    print("Press Ctrl+C to stop.\n")

    reader = SpaceMouseReader()

    try:
        # Start the reader
        reader.start()
        print("SpaceMouse reader started successfully!\n")

        # Continuously read and display state
        while True:
            state = reader.get_state()

            # Clear line and print state
            print(
                f"\r"
                f"X: {state.x:+.3f} | "
                f"Y: {state.y:+.3f} | "
                f"Z: {state.z:+.3f} | "
                f"Roll: {state.roll:+.3f} | "
                f"Pitch: {state.pitch:+.3f} | "
                f"Yaw: {state.yaw:+.3f} | "
                f"Buttons: {state.buttons} | "
                f"Time: {state.timestamp:.3f}",
                end="",
                flush=True,
            )

            time.sleep(0.05)  # 20 Hz update rate

    except KeyboardInterrupt:
        print("\n\nStopping SpaceMouse reader...")

    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        reader.stop()
        print("SpaceMouse reader stopped.")
        print("Demo finished.")


if __name__ == "__main__":
    main()
