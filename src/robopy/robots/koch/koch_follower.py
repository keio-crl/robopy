from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.common.arm import Arm


class KochFollower(Arm):
    """Class representing the Koch follower robotic arm"""

    def __init__(self, port: str, motor_ids: list[int]):
        self._port = port

        # Ensure there are exactly 6 motor IDs for the Koch follower arm
        if len(motor_ids) != 6:
            raise ValueError("Koch follower requires exactly 6 motor IDs.")
        self._motors = DynamixelBus(
            port=port,
            motors={
                "shoulder_pan": DynamixelMotor(motor_ids[0], "xl330-m077"),
                "shoulder_lift": DynamixelMotor(motor_ids[1], "xl330-m077"),
                "elbow": DynamixelMotor(motor_ids[2], "xl330-m077"),
                "wrist_flex": DynamixelMotor(motor_ids[3], "xl330-m077"),
                "wrist_roll": DynamixelMotor(motor_ids[4], "xl330-m077"),
                "gripper": DynamixelMotor(motor_ids[5], "xl330-m077"),
            },
        )
        self._is_connected = False

    @property
    def port(self) -> str:
        return self._port

    @property
    def motors(self) -> DynamixelBus:
        return self._motors

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        # Implementation to connect to the Koch follower arm
        if self._is_connected:
            print("Already connected to the Koch follower arm.")
            return
        try:
            self._motors.open()
            self._is_connected = True
            print("Connected to the Koch follower arm.")
        except Exception as e:
            err = f"Failed to connect to the Koch follower arm: {e}"
            raise ConnectionError(err)

    def disconnect(self) -> None:
        # Implementation to disconnect from the Koch follower arm
        if not self._is_connected:
            print("Not connected to the Koch follower arm.")
            return
        try:
            self._motors.close()
            self._is_connected = False
            print("Disconnected from the Koch follower arm.")
        except Exception as e:
            err = f"Failed to disconnect from the Koch follower arm: {e}"
            raise ConnectionError(err)
