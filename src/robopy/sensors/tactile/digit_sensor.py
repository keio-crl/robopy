from robopy.config.sensor_config.params_config import TactileParams

from ..common.sensor import Sensor


class DigitSensor(Sensor):
    def __init__(self, config: TactileParams):
        self.config = config
        self.name = config.name
        self.fps = config.fps
        self._is_connected = False

    def connect(self) -> None:
        self._is_connected = True
        print(f"Connected to Digit sensor: {self.name}")

    def disconnect(self) -> None:
        self._is_connected = False
        print(f"Disconnected from Digit sensor: {self.name}")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def get_observation(self) -> None:
        if not self.is_connected:
            raise RuntimeError(f"Digit sensor {self.name} is not connected.")
        # Placeholder for actual tactile data retrieval logic
        print(f"Getting observation from Digit sensor: {self.name}")
        return None

    def record(self) -> None:
        """record function for Digit sensor. Not implemented."""
        pass
