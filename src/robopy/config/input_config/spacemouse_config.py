from dataclasses import dataclass


@dataclass
class SpaceMouseConfig:
    """Configuration for SpaceMouse input device.

    Attributes:
        linear_speed: Maximum linear velocity in m/s when the SpaceMouse axis
            is at full deflection (1.0).
        angular_speed: Maximum angular velocity in rad/s when the SpaceMouse
            axis is at full deflection (1.0).
        gripper_speed: Gripper movement speed in degrees/s per button press.
        deadzone: Axes with absolute values below this threshold are zeroed.
        control_hz: Control loop frequency in Hz.
        input_smoothing: Exponential moving average factor for SpaceMouse axes.
            0.0 = no smoothing (raw input), 1.0 = maximum smoothing.
            Higher values reduce jitter but add latency.
        max_joint_delta_deg: Maximum allowed joint angle change per control
            step in degrees.  Prevents vibration from large IK solution jumps.
    """

    linear_speed: float = 0.10
    angular_speed: float = 0.5
    gripper_speed: float = 50.0
    deadzone: float = 0.05
    control_hz: int = 50
    input_smoothing: float = 0.5
    max_joint_delta_deg: float = 5.0
