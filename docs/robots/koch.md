# Koch Robot

[`KochRobot`](../api/robots.md#robopy.robots.koch.koch_robot.KochRobot)は、二腕ロボットシステムでカメラセンサーを統合したロボットです。

## :material-robot-outline: 構成要素

### ロボットアーム

- **Leader Arm**: 操作者が制御するアーム
- **Follower Arm**: Leaderの動きを追従するアーム

### センサー統合

- **カメラ**: Webカメラ または Intel RealSenseカメラ対応

## :material-cog: 基本的な使用方法

### 設定の作成

```python
from robopy.config.robot_config.koch_config import KochConfig

# 基本設定
config = KochConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    camera_config=None  # カメラなし
)
```

### カメラ付きの設定

```python
from robopy.config.robot_config.koch_config import KochConfig
from robopy.config.sensor_config.visual_config import WebCameraConfig

# Webカメラ付き設定
config = KochConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    camera_config=WebCameraConfig(
        device_id=0,
        fps=20,
        width=640,
        height=480
    )
)
```

### ロボットの操作

```python
from robopy.robots.koch.koch_robot import KochRobot

robot = KochRobot(config)

try:
    # 接続
    robot.connect()
    
    # テレオペレーション（10秒間）
    robot.teleoperation(duration=10)
    
finally:
    robot.disconnect()
```

## :material-database: データ記録

```python
# データ記録
obs = robot.record(max_frame=100, fps=5)

# 記録されたデータの確認
print(f"Leader アーム: {obs['arms']['leader'].shape}")
print(f"Follower アーム: {obs['arms']['follower'].shape}")

if obs['camera'] is not None:
    print(f"カメラ: {obs['camera'].shape}")
```

### 記録データ構造

記録されるデータは[`KochObs`](../api/config.md#robopy.config.robot_config.koch_config.KochObs)型で、以下の構造を持ちます：

```python
{
    "arms": {
        "leader": np.ndarray,    # (frames, joint_dims) - 関節角度など
        "follower": np.ndarray,  # (frames, joint_dims) - 関節角度など
    },
    "camera": np.ndarray | None,  # (frames, H, W, C) - RGB画像 or None
}
```

## :material-link-variant: 関連クラス

- [`KochConfig`](../api/config.md#robopy.config.robot_config.koch_config.KochConfig) - ロボット設定
- [`KochObs`](../api/config.md#robopy.config.robot_config.koch_config.KochObs) - 観測データ型