# Robopy
[日本語](README.md) | ENG  

**Robopy** is a Python interface for robot control. It supports Rakuda and Koch robots (under development) with integrated data collection from cameras and tactile sensors.

## 🚀 Quick Start

### Installation

```bash
uv add git+https://github.com/keio-crl/robopy.git --tag v0.3.0
# RealSense support (Linux)
uv add pyrealsense2
```

### Basic Usage Example

```python
from robopy.utils.exp_interface import RakudaExpHandler

# Create experiment handler
handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",      # Leader arm port
    follower_port="/dev/ttyUSB1",    # Follower arm port
    left_digit_serial="D20542",      # Left tactile sensor serial number
    right_digit_serial="D20537",     # Right tactile sensor serial number
    fps=20                           # Recording frequency
)

# Interactive recording and saving
    handler.record_save(
    max_frames=1000,                 # Number of frames to record
    save_path="experiment_001",      # Save path: data/experiment_001/...
    if_async=True                    # High-speed parallel recording
)
```

## 🤖 Key Features

- **🔄 Multi-robot Support**: Supports [`RakudaRobot`](src/robopy/robots/rakuda/rakuda_robot.py) and KochRobot
- **📷 Sensor Integration**: Unified interface for RealSense cameras and DIGIT tactile sensors
- **⚡ High-performance Data Collection**: 30Hz high-speed data capture with parallel processing
- **🎬 Visualization Features**: Data animation generation functionality
- **🛠 Simple Dependencies**: No need for C/C++ based libraries like ROS

## 📋 Basic Usage

### 1. Configuration Setup

```python
    from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
    from robopy.utils import RakudaExpHandler



    config=RakudaConfig(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
    )

    handler = RakudaExpHandler(
        rakuda_config=config,
        fps=10 # Data collection frame rate (max 30)
    )

    # データ記録と保存
    handler.record_save(
        max_frames=150, # The number of frames to collect
        save_path="experiment_001", # Save directory: data/experiment_001/...
    )
```

### 2. Robot Control

```python
from robopy import RakudaRobot

robot = RakudaRobot(config)

try:
    robot.connect()
    
    # Teleoperation (5 seconds)
    robot.teleoperation(duration=5)
    
finally:
    robot.disconnect()
```


## 📊 Recorded Data Structure

The recorded data is of type [`RakudaObs`](src/robopy/config/robot_config/rakuda_config.py) with the following structure:

```python
{
    "arms": {
        "leader": np.ndarray,    # (frames, 17) - Joint angles
        "follower": np.ndarray,  # (frames, 17) - Joint angles
    },
    "sensors": {
        "cameras": {
            "main": np.ndarray,  # (frames, C, H, W) - RGB images
        },
        "tactile": {
            "left": np.ndarray,  # (frames, C, H, W) - Tactile data
            "right": np.ndarray, # (frames, C, H, W) - Tactile data
        }
    }
}
```

## 📁 Project Structure

```
robopy/
├── src/robopy/
│   ├── robots/          # Robot control classes
│   │   ├── rakuda/      # Rakuda robot
│   │   └── koch/        # Koch robot
│   ├── sensors/         # Sensor control
│   │   ├── visual/      # Cameras
│   │   └── tactile/     # Tactile sensors
│   ├── config/          # Configuration classes
│   └── utils/           # Utilities
│       ├── exp_interface/  # Experiment interface
│       └── worker/         # Data saving and processing
├── docs/                # Documentation
├── examples/            # Sample code
└── tests/              # Test code
```

## 📚 Documentation

For detailed documentation, please refer to:

- [Documentation](https://keio-crl.github.io/robopy/)