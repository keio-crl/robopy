import logging
import pickle
import time
from typing import TYPE_CHECKING, Dict

import numpy as np
from rich import print

from robopy.config.dotrobopy import apply_rakuda_dotconfig
from robopy.config.robot_config.rakuda_config import (
    RAKUDA_MOTOR_MAPPING,
    RakudaArmObs,
    RakudaConfig,
)
from robopy.control.types import ControlMode
from robopy.motor.dynamixel_bus import DynamixelBus
from robopy.motor.dynamixel_control_table import XControlTable

from ..common.robot import Robot
from .rakuda_follower import RakudaFollower
from .rakuda_leader import RakudaLeader

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .rakuda_control import RakudaControlSystem

logger = logging.getLogger(__name__)


def _filter_action_by_enabled_joints(
    action: Dict[str, float],
    enabled_joints: set[str],
) -> Dict[str, float]:
    return {name: value for name, value in action.items() if name in enabled_joints}


class RakudaPairSys(Robot):
    """Class representing the Rakuda robotic system with both leader and follower arms."""

    def __init__(self, cfg: RakudaConfig) -> None:
        cfg = apply_rakuda_dotconfig(cfg)
        self.config = cfg
        self._leader = RakudaLeader(cfg)
        self._follower = RakudaFollower(cfg)
        self._is_connected = False
        self._motor_mapping = (
            RAKUDA_MOTOR_MAPPING  # key: leader motor name, value: follower motor name
        )
        self._leader_motor_names = list(self._leader.motors.motors.keys())
        self._follower_motor_names = list(self._follower.motors.motors.keys())

        # Cache torque-enabled joints for safe write filtering.
        self._leader_torque_enabled: set[str] = (
            {"l_arm_grip", "r_arm_grip"}
            if cfg.leader_torque_enabled is None
            else set(cfg.leader_torque_enabled)
        )
        self._follower_torque_enabled: set[str] = (
            set(self._follower_motor_names)
            if cfg.follower_torque_enabled is None
            else set(cfg.follower_torque_enabled)
        )

        # Set while a RakudaControlSystem owns the buses. The legacy write paths
        # below refuse to run then, so the old teleoperation loop and the new
        # servo loop can never both be writing goal values.
        self._control_system: "RakudaControlSystem | None" = None

    def connect(self) -> None:
        """Connect to both leader and follower arms."""
        if self.is_connected:
            logger.info("Successfully connected to both leader and follower arms.")
            return

        try:
            self._leader.connect()
            self._follower.connect()
            logger.info("Successfully connected to both leader and follower arms.")
            print("[cyan]Successfully connected to both leader and follower arms.[/cyan]")
            self._is_connected = True
        except (OSError, IOError, PermissionError) as e:
            logger.error(f"Failed to connect to arms: {e}")
            raise ConnectionError(f"Failed to connect to arms: {e}")
        except (pickle.PickleError, EOFError) as e:
            logger.error(f"Calibration data corrupted: {e}")
            raise ConnectionError(f"Calibration data error: {e}")

    def disconnect(self) -> None:
        """Disconnect from both leader and follower arms."""
        self.stop_control()
        self.leader.disconnect()
        self.follower.disconnect()

    def get_observation(self) -> RakudaArmObs:
        """Get the current observation from both arms.

        The array shape, ordering and units are unchanged: degrees, in bus motor
        order, exactly as before.  The only difference is where the numbers come
        from -- while a control system owns the buses this reads its cached
        snapshot instead of issuing its own SyncRead, because a second reader on
        a port is what the single-owner rule exists to prevent.  Richer state
        (velocity, current, timestamps, validity) is available through
        :meth:`detailed_state`.
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        cached = self._cached_observation()
        if cached is not None:
            return cached

        leader_motor_names = self._leader_motor_names
        follower_motor_names = self._follower_motor_names
        leader_obs = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )

        follower_obs = self._follower.motors.sync_read(
            XControlTable.PRESENT_POSITION, follower_motor_names
        )

        leader_obs_array = np.array(list(leader_obs.values()), dtype=np.float32)
        follower_obs_array = np.array(list(follower_obs.values()), dtype=np.float32)
        return RakudaArmObs(leader=leader_obs_array, follower=follower_obs_array)

    def detailed_state(self) -> Dict[str, object]:
        """Full SI-unit snapshots from the running control system.

        Returns:
            ``{"leader": JointState | None, "follower": JointState | None}``.
            Both are ``None`` when no control system is running -- the legacy
            path publishes no such snapshot.
        """
        if self._control_system is None:
            return {"leader": None, "follower": None}
        return {
            "leader": self._control_system.leader.latest_state(),
            "follower": self._control_system.follower.latest_state(),
        }

    def _cached_observation(self) -> RakudaArmObs | None:
        """Build a legacy-shaped observation from the servo cache, if running."""
        if self._control_system is None:
            return None
        leader_state = self._control_system.leader.latest_state()
        follower_state = self._control_system.follower.latest_state()
        if leader_state is None or follower_state is None:
            return None

        def as_degrees(state: object, motor_names: list[str]) -> np.ndarray:
            positions = state.positions_dict()  # type: ignore[attr-defined]
            return np.array(
                [np.degrees(positions.get(name, float("nan"))) for name in motor_names],
                dtype=np.float32,
            )

        return RakudaArmObs(
            leader=as_degrees(leader_state, self._leader_motor_names),
            follower=as_degrees(follower_state, self._follower_motor_names),
        )

    def teleoperate(self, max_seconds: float | None = None) -> None:
        """
        Leader controls follower. If max_seconds is set,
        run for that many seconds then return.
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        logger.info("Starting teleoperation. Leader will control follower.")
        start_time = time.time()
        try:
            while True:
                # Get current positions from leader arm
                leader_positions = self.get_leader_action()
                # Map leader positions to follower positions
                follower_positions: Dict[str, float] = {}
                for leader_name, position in leader_positions.items():
                    follower_name = self._motor_mapping.get(leader_name)
                    if follower_name:
                        follower_positions[follower_name] = position

                # Send positions to follower arm
                try:
                    self.send_follower_action(follower_positions)
                    logger.info(f"Sent follower action: {follower_positions}")
                except Exception:
                    logger.exception("Failed to send follower action; continuing loop.")

                # Check for max_seconds
                if max_seconds is not None and (time.time() - start_time) >= max_seconds:
                    logger.info("Reached max_seconds; exiting teleoperate.")
                    break

            # TODO: add a better way to stop smoothly, eg. set a home position
        except KeyboardInterrupt:
            self.follower.motors.torque_disabled()
            logger.info("Teleoperation stopped by user.")
        except Exception:
            logger.exception("Error during teleoperation.")
            raise

    def teleoperate_step(self) -> RakudaArmObs:
        """
        Legacy teleoperate_step (deprecated).

        Performs one iteration of teleoperation with full observation.
        This method is SLOW (40ms) due to 4 Dynamixel communications.

        Use control_step() for high-frequency control instead.

        Returns:
            RakudaArmObs: Current observation
                - leader: np.ndarray of leader arm positions
                - follower: np.ndarray of follower arm positions
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        # Get current positions from leader arm
        leader_positions = self.get_leader_action()
        # Map leader positions to follower positions
        follower_goal_positions: Dict[str, float] = {}
        for leader_name, position in leader_positions.items():
            follower_name = self._motor_mapping.get(leader_name)
            if follower_name:
                follower_goal_positions[follower_name] = position
        # Send positions to follower arm
        try:
            self.send_follower_action(follower_goal_positions)
            self.send_leader_action({"l_arm_grip": 2400, "r_arm_grip": 2400})
        except Exception:
            logger.exception("Failed to send follower action; continuing.")

        follower_observations = self.get_follower_action()
        leader_obs = np.array(list(leader_positions.values()), dtype=np.float32)
        follower_obs = np.array(list(follower_observations.values()), dtype=np.float32)
        return RakudaArmObs(leader=leader_obs, follower=follower_obs)

    def control_step(self) -> Dict[str, float]:
        """
        LeRobot-style high-frequency control step.

        Performs minimal necessary operations for teleoperation:
        1. Read leader positions
        2. Map to follower
        3. Send to follower

        This is FAST (~10-16ms) and suitable for 60Hz control loops.

        Returns:
            Dict[str, float]: Leader positions that were sent to follower
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")
        self._require_no_control_system("control_step")

        # Read leader positions
        leader_positions = self.get_leader_action()

        # Map leader positions to follower goal positions
        follower_goal_positions: Dict[str, float] = {}
        for leader_name, position in leader_positions.items():
            follower_name = self._motor_mapping.get(leader_name)
            if follower_name:
                follower_goal_positions[follower_name] = position

        # Send to follower
        self.send_follower_action(follower_goal_positions)

        return leader_positions

    def get_observation_with_leader(self, leader_positions: Dict[str, float]) -> RakudaArmObs:
        """
        Get observation using pre-read leader positions.

        This is useful when you already have leader positions from control_step()
        and want to avoid redundant communication.

        Args:
            leader_positions: Pre-read leader positions

        Returns:
            RakudaArmObs: Current arm observation
        """
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        # Read follower positions
        follower_positions = self.get_follower_action()

        # Convert to arrays
        leader_obs = np.array(list(leader_positions.values()), dtype=np.float32)
        follower_obs = np.array(list(follower_positions.values()), dtype=np.float32)

        return RakudaArmObs(leader=leader_obs, follower=follower_obs)

    def get_leader_action(self) -> Dict[str, float]:
        """Get the current action (positions) from the leader arm."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        leader_motor_names = list(self._leader.motors.motors.keys())
        leader_positions = self._leader.motors.sync_read(
            XControlTable.PRESENT_POSITION, leader_motor_names
        )
        return leader_positions

    def send_leader_action(self, action: Dict[str, float]) -> None:
        """Send action to the leader arm only."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")
        self._require_no_control_system("send_leader_action")
        filtered = _filter_action_by_enabled_joints(action, self._leader_torque_enabled)
        if not filtered:
            return
        self._leader.motors.sync_write(XControlTable.GOAL_POSITION, filtered)

    def get_follower_action(self) -> Dict[str, float]:
        """Get the current action (positions) from the follower arm."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")

        follower_motor_names = self._follower_motor_names

        follower_positions = self._follower.motors.sync_read(
            XControlTable.PRESENT_POSITION, follower_motor_names
        )
        return follower_positions

    def send_follower_action(self, action: Dict[str, float]) -> None:
        """Send action to the follower arm only."""
        if not self._is_connected:
            raise ConnectionError("KochPairSys is not connected. Call connect() first.")
        self._require_no_control_system("send_follower_action")
        filtered = _filter_action_by_enabled_joints(action, self._follower_torque_enabled)
        if not filtered:
            return

        self._follower.motors.sync_write(XControlTable.GOAL_POSITION, filtered)

    # ------------------------------------------------------------------
    # Mode-based control (dual-arm IK / bilateral)
    #
    # `control_step()` and `teleoperate()` above remain the position
    # teleoperation path and are unchanged. They are simply refused while a
    # control system holds the buses.
    # ------------------------------------------------------------------

    @property
    def control_system(self) -> "RakudaControlSystem | None":
        """The attached control system, or ``None`` for plain teleoperation."""
        return self._control_system

    def _require_no_control_system(self, what: str) -> None:
        """Raise if a control system currently owns the buses."""
        if self._control_system is not None:
            raise RuntimeError(
                f"{what}() is refused while a {type(self._control_system).__name__} owns these "
                "buses. Exactly one writer per port: call stop_control() before using the legacy "
                "position teleoperation path."
            )

    def build_control_system(
        self,
        *,
        leader_gravity: object | None = None,
        follower_gravity: object | None = None,
    ) -> "RakudaControlSystem":
        """Construct the control system described by ``config.control``.

        Args:
            leader_gravity: Validated leader gravity model, if any.
            follower_gravity: Validated follower gravity model, if any.

        Returns:
            The control system, not yet configured or started.

        Raises:
            ValueError: If no ``control:`` section is configured.
            ConnectionError: If the arms are not connected.
        """
        from .rakuda_control import RakudaControlSystem

        if self.config.control is None:
            raise ValueError(
                "No control section is configured. Add a `control:` block to "
                ".robopy/rakuda/config.yaml, or keep using the position teleoperation path."
            )
        if not self.is_connected:
            raise ConnectionError("RakudaPairSys is not connected. Call connect() first.")

        return RakudaControlSystem.from_buses(
            self.config.control,
            self._leader.motors,
            self._follower.motors,
            leader_torque_enabled=sorted(self._leader_torque_enabled),
            follower_torque_enabled=sorted(self._follower_torque_enabled),
            leader_gravity=leader_gravity,  # type: ignore[arg-type]
            follower_gravity=follower_gravity,  # type: ignore[arg-type]
        )

    def start_control(self, system: "RakudaControlSystem | None" = None) -> "RakudaControlSystem":
        """Configure, align and start a control system, taking the buses over.

        Args:
            system: An already-built system, or ``None`` to build one from the
                configuration.

        Returns:
            The running system.

        Raises:
            RuntimeError: If a control system is already running.
        """
        if self._control_system is not None:
            raise RuntimeError("A control system is already running; stop it first.")
        system = system or self.build_control_system()
        system.configure()
        system.align()
        system.start()
        self._control_system = system
        logger.info("Started %s control for Rakuda.", system.mode.value)
        return system

    def stop_control(self) -> list[str]:
        """Stop the running control system and hand the buses back.

        Returns:
            What each servo did while stopping.
        """
        if self._control_system is None:
            return []
        results = self._control_system.stop()
        self._control_system = None
        return results

    @property
    def control_mode(self) -> ControlMode:
        """The running mode, or position teleoperation when nothing is running."""
        if self._control_system is None:
            return ControlMode.POSITION_TELEOP
        return self._control_system.mode

    @property
    def is_connected(self) -> bool:
        """Check if both arms are connected."""
        return self._is_connected

    @property
    def port(self) -> str:
        """Get the ports of both arms."""
        return f"Leader: {self.leader.port}, Follower: {self.follower.port}"

    @property
    def motors(self) -> dict[str, DynamixelBus]:
        """Get the motor buses of both arms."""
        return {"leader": self.leader.motors, "follower": self.follower.motors}

    @property
    def leader(self) -> RakudaLeader:
        return self._leader

    @property
    def follower(self) -> RakudaFollower:
        return self._follower
