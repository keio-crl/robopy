"""SimXArmFollower: ZMQ client that connects to GELLO's ``sim_xarm`` simulator.

Usage:
    1. Start the GELLO simulator in a separate terminal:
       ``cd gello_software && python experiments/launch_nodes.py --robot sim_xarm``

    2. Use robopy with ``XArmConfig(sim_mode=True)`` — the follower will
       automatically connect to the simulator via ZMQ instead of the real xArm.

The ZMQ protocol is pickle-based REQ/REP and is wire-compatible with
``xarm_modules/src/robot_node.py:ZMQClientRobot``.
"""

from __future__ import annotations

import logging
import pickle
from typing import Any, Dict

import numpy as np
from numpy.typing import NDArray

from robopy.robots.xarm.xarm_arm import XArmArm

logger = logging.getLogger(__name__)

DEFAULT_SIM_PORT = 6000


class SimXArmFollower(XArmArm):
    """XArmArm-compatible follower that talks to GELLO's sim_xarm via ZMQ."""

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_SIM_PORT) -> None:
        self._host = host
        self._port = port
        self._socket: Any | None = None
        self._context: Any | None = None
        self._is_connected = False

    def connect(self) -> None:
        if self._is_connected:
            logger.info("SimXArmFollower already connected.")
            return
        try:
            import zmq
        except ImportError as exc:
            raise ImportError(
                "pyzmq is required for SimXArmFollower. Install via `uv add pyzmq`."
            ) from exc

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        addr = f"tcp://{self._host}:{self._port}"
        self._socket.connect(addr)
        self._socket.setsockopt(zmq.RCVTIMEO, 5000)
        self._socket.setsockopt(zmq.SNDTIMEO, 5000)

        ndofs = self._call("num_dofs")
        self._is_connected = True
        logger.info("Connected to GELLO sim_xarm at %s (DOFs=%s).", addr, ndofs)

    def disconnect(self) -> None:
        if not self._is_connected:
            return
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if self._context is not None:
            self._context.term()
            self._context = None
        self._is_connected = False
        logger.info("Disconnected SimXArmFollower.")

    def get_joint_state(self) -> NDArray[np.float32]:
        """Return 8-DOF: 7 joints [rad] + gripper [0, 1]."""
        result = self._call("get_joint_state")
        return np.asarray(result, dtype=np.float32)

    def get_ee_pos_quat(self) -> NDArray[np.float32]:
        """Return [x, y, z, qx, qy, qz, qw] from sim observations."""
        obs = self._call("get_observations")
        return np.asarray(obs.get("ee_pos_quat", np.zeros(7)), dtype=np.float32)

    def command_joint_state(self, joint_state: NDArray[np.float32]) -> None:
        """Send 8-DOF joint command to the simulator."""
        joint_state = np.asarray(joint_state, dtype=np.float32)
        self._call("command_joint_state", joint_state=joint_state)

    def command_cartesian_absolute(
        self, pos_aa: NDArray[np.float32], gripper: float | None = None
    ) -> None:
        raise NotImplementedError(
            "GELLO sim_xarm only supports joint-space commands. Use command_joint_state() instead."
        )

    def command_cartesian_relative(
        self, delta: NDArray[np.float32], gripper: float | None = None
    ) -> None:
        raise NotImplementedError(
            "GELLO sim_xarm only supports joint-space commands. Use command_joint_state() instead."
        )

    def get_observations(self) -> Dict[str, NDArray[np.float32]]:
        """Return the full observation dict from the simulator."""
        obs = self._call("get_observations")
        return {k: np.asarray(v, dtype=np.float32) for k, v in obs.items()}

    def _call(self, method: str, **args: Any) -> Any:
        if self._socket is None:
            raise ConnectionError("SimXArmFollower is not connected.")
        request: Dict[str, Any] = {"method": method}
        if args:
            request["args"] = args
        self._socket.send(pickle.dumps(request))
        result = pickle.loads(self._socket.recv())
        return result

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def port(self) -> str:
        return f"{self._host}:{self._port}"
