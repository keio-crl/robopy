"""GELLO leader diagnostic — read raw joint values without the pair_sys alignment check.

Run this with the GELLO held at the intended start_joints posture
([0, -90, 90, -90, -90, 0, 0] deg). The upstream sim (`launch_nodes.py`)
does NOT need to be running for this script.
"""

import numpy as np

from robopy.config.robot_config.xarm_config import XArmConfig
from robopy.robots.xarm.xarm_leader import XArmLeader

LEADER_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0VKE-if00-port0"
EXPECTED = np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32)


def run_case_a() -> None:
    cfg = XArmConfig(leader_port=LEADER_PORT)
    leader = XArmLeader(cfg)
    leader.connect()
    try:
        print("=== (A) default offsets, NO start_joints align ===")
        print(f"offsets    : {leader._joint_offsets[:7]}")
        print(f"leader raw : {leader.get_joint_state()[:7]}")
        print(f"expected   : {EXPECTED}")
        print(f"delta      : {leader.get_joint_state()[:7] - EXPECTED}")
    finally:
        leader.disconnect()


def run_case_b() -> None:
    cfg = XArmConfig(leader_port=LEADER_PORT, start_joints=EXPECTED)
    leader = XArmLeader(cfg)
    leader.connect()
    try:
        print("=== (B) after _align_offsets_to_start_joints (2π rounding) ===")
        print(f"offsets    : {leader._joint_offsets[:7]}")
        print(f"leader raw : {leader.get_joint_state()[:7]}")
        print(f"expected   : {EXPECTED}")
        print(f"delta      : {leader.get_joint_state()[:7] - EXPECTED}")
    finally:
        leader.disconnect()


if __name__ == "__main__":
    run_case_a()
    print()
    run_case_b()
