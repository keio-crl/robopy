#!/usr/bin/env python3
"""Debug script to diagnose SpaceMouse connection issues.

This script helps identify why SpaceMouse input might not be accessible.
"""

from __future__ import annotations

import sys


def check_pyspacemouse_installed() -> bool:
    """Check if pyspacemouse is installed."""
    try:
        import pyspacemouse

        print("✓ pyspacemouse is installed")
        print(f"  Version info: {pyspacemouse.__file__}")
        return True
    except ImportError:
        print("✗ pyspacemouse is NOT installed")
        print("  Install with: pip install pyspacemouse")
        return False


def check_device_list() -> None:
    """List available HID devices."""
    try:
        import hid

        print("\n✓ hid library is available")
        print("\nListing HID devices (looking for 3Dconnexion devices):")
        print("-" * 80)

        devices = hid.enumerate()
        spacemouse_found = False

        for device in devices:
            vendor_id = device.get("vendor_id", 0)
            product_id = device.get("product_id", 0)
            manufacturer = device.get("manufacturer_string", "")
            product = device.get("product_string", "")

            # 3Dconnexion vendor ID is 0x046d (Logitech) or 0x256f
            if vendor_id in [0x046D, 0x256F] or "3Dconnexion" in manufacturer:
                spacemouse_found = True
                print(f"\n*** SPACEMOUSE DEVICE FOUND ***")
                print(f"  Vendor ID:     0x{vendor_id:04X}")
                print(f"  Product ID:    0x{product_id:04X}")
                print(f"  Manufacturer:  {manufacturer}")
                print(f"  Product:       {product}")
                print(f"  Path:          {device.get('path', b'').decode('utf-8', errors='ignore')}")
                print(f"  Usage Page:    {device.get('usage_page', 'N/A')}")
                print(f"  Usage:         {device.get('usage', 'N/A')}")

        if not spacemouse_found:
            print("\n⚠ No 3Dconnexion/SpaceMouse devices found in HID enumeration")

    except ImportError:
        print("\n✗ hid library is NOT installed")
        print("  Install with: pip install hidapi")


def check_pyspacemouse_open() -> None:
    """Try to open SpaceMouse with pyspacemouse."""
    try:
        import pyspacemouse

        print("\n" + "=" * 80)
        print("Attempting to open SpaceMouse with pyspacemouse...")
        print("=" * 80)

        # List supported devices
        if hasattr(pyspacemouse, "list_devices"):
            print("\nSupported devices:")
            devices = pyspacemouse.list_devices()
            for dev in devices:
                print(f"  - {dev}")

        device = pyspacemouse.open()

        print("\n✓ Successfully opened SpaceMouse!")
        print("\nTrying to read a few samples...")

        for i in range(5):
            state = device.read()
            print(
                f"  Sample {i+1}: "
                f"x={state.x:.3f}, y={state.y:.3f}, z={state.z:.3f}, "
                f"roll={state.roll:.3f}, pitch={state.pitch:.3f}, yaw={state.yaw:.3f}"
            )

        device.close()
        print("\n✓ Read successful! SpaceMouse is working.")

    except Exception as e:
        print(f"\n✗ Error while trying to open SpaceMouse: {e}")
        import traceback

        traceback.print_exc()


def check_running_processes() -> None:
    """Check if 3Dconnexion processes are running."""
    import subprocess

    print("\n" + "=" * 80)
    print("Checking for 3Dconnexion processes...")
    print("=" * 80)

    try:
        result = subprocess.run(
            ["ps", "aux"],
            capture_output=True,
            text=True,
        )

        lines = [line for line in result.stdout.split("\n") if "3dconnexion" in line.lower()]

        if lines:
            print("\n⚠ Found 3Dconnexion processes running:")
            for line in lines:
                print(f"  {line}")
            print("\nThese processes may be intercepting SpaceMouse input.")
            print("To stop them, run:")
            print("  killall 3DconnexionHelper")
            print("  sudo killall 3Dconnexiond")
        else:
            print("\n✓ No 3Dconnexion processes found running")

    except Exception as e:
        print(f"\n✗ Error checking processes: {e}")


def main() -> None:
    """Run all diagnostic checks."""
    print("SpaceMouse Debug Script")
    print("=" * 80)

    # Check 1: pyspacemouse installation
    if not check_pyspacemouse_installed():
        return

    # Check 2: HID devices
    check_device_list()

    # Check 3: Running processes
    check_running_processes()

    # Check 4: Try to open
    check_pyspacemouse_open()

    print("\n" + "=" * 80)
    print("Debug complete!")
    print("=" * 80)


if __name__ == "__main__":
    main()
