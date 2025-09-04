import time

import serial.tools.list_ports as list_ports
from rich import print as rprint


def find_available_ports() -> list[str]:
    """Find available USB ports on the system.

    Returns:
        list[str]: A list of available USB port names.
    """
    ports = list_ports.comports()

    available_ports = [port.device for port in ports]
    return available_ports


def find_port() -> None:
    ports_before = find_available_ports()
    rprint(f"Available ports: {ports_before}")
    rprint(
        "[red]Remove the usb cable[/] from your DynamixelMotorsBus "
        "and [red]press Enter[/] when done."
    )
    input()

    time.sleep(0.5)
    ports_after = find_available_ports()
    ports_diff = list(set(ports_before) - set(ports_after))

    if len(ports_diff) == 1:
        port = ports_diff[0]
        print(f"The port of this DynamixelMotorsBus is '{port}'")
        print("Reconnect the usb cable.")
    elif len(ports_diff) == 0:
        raise OSError(f"Could not detect the port. No difference was found ({ports_diff}).")
    else:
        raise OSError(f"Could not detect the port. More than one port was found ({ports_diff}).")


if __name__ == "__main__":
    find_port()
