# Koch Robot

[`KochRobot`](../api/robots.md#robopy.robots.koch.koch_robot.KochRobot)は、Dynamixel X-Seriesサーボモーターを使用した6軸二腕ロボットシステムです。

## :material-robot-outline: 構成要素

### ロボットアーム

- **Leader Arm**: 操作者が制御するアーム
- **Follower Arm**: Leaderの動きを追従するアーム

各アームは6つの関節で構成されています:

| 関節名 | 機能 |
|--------|------|
| `shoulder_pan` | 肩の水平回転 |
| `shoulder_lift` | 肩の上下回転 |
| `elbow` | 肘の屈伸 |
| `wrist_flex` | 手首の屈伸 |
| `wrist_roll` | 手首の回転 |
| `gripper` | グリッパーの開閉 |

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
    calibration_path="calibration/koch_calib.pkl",
)
```

### カメラ付きの設定

```python
from robopy.config.robot_config.koch_config import KochConfig, KochSensorConfig
from robopy.config.sensor_config.visual_config.camera_config import WebCameraConfig

config = KochConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    calibration_path="calibration/koch_calib.pkl",
    sensors=KochSensorConfig(
        cameras={
            "top": WebCameraConfig(fps=30, width=640, height=480),
        }
    ),
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
    robot.teleoperation(max_seconds=10)

finally:
    robot.disconnect()
```

## :material-database: データ記録

```python
# データ記録
obs = robot.record(max_frame=100, fps=5)

# 記録されたデータの確認
print(f"Leader アーム: {obs.arms.leader.shape}")     # (frames, 6)
print(f"Follower アーム: {obs.arms.follower.shape}")  # (frames, 6)
```

### 高速並列記録

```python
obs = robot.record_parallel(
    max_frame=500,
    fps=20,
    teleop_hz=25,
)
```

## :material-play: アクションの送信

### 関節空間でのアクション送信

```python
import numpy as np

# 関節角度の軌道を再生
robot.send(
    max_frame=100,
    fps=20,
    leader_action=recorded_joint_trajectory,
    teleop_hz=100,
)

# 1フレームだけ送信
robot.send_frame_action(np.array([0.0, 30.0, -45.0, 20.0, 0.0, 50.0]))
```

### エンドエフェクタ空間でのアクション送信

Inverse Kinematics (IK) を使用して、エンドエフェクタの姿勢を指定してロボットを制御できます。詳細は[キネマティクス](#kinematics)セクションを参照してください。

```python
import numpy as np

# エンドエフェクタ姿勢: [x, y, z, pitch, roll, gripper_deg]
ee_action = np.array([0.1, 0.0, 0.1, 0.0, 0.0, 50.0], dtype=np.float32)
robot.send_ee_frame_action(ee_action)
```

## :material-axis-arrow: キネマティクス {#kinematics}

KochRobotにもForward Kinematics (FK) と Inverse Kinematics (IK) が組み込まれています。使い方は[SO-101](so101.md#kinematics)と同じインターフェースです。

!!! warning "物理パラメータは推定値です"
    Koch v1.1には公式のURDFが存在しません。現在のキネマティクスパラメータはMuJoCo menagerieとCAD図面からの**推定値**です。実機で使用する前に、実測値で置き換えてください。対応するパラメータは `robopy.kinematics.robot_chains.koch_chain` に `TODO(physical-params)` マーカーで示されています。

### Forward Kinematics (FK)

```python
import numpy as np
from robopy.robots.koch.koch_robot import KochRobot

# @classmethod: ハードウェア接続なしで使用可能
joint_angles = np.array([0.0, 30.0, -45.0, 20.0, 0.0, 50.0], dtype=np.float32)
ee_pose = KochRobot.forward_kinematics(joint_angles)

print(f"位置: x={ee_pose.x:.4f} m, y={ee_pose.y:.4f} m, z={ee_pose.z:.4f} m")
print(f"姿勢: pitch={np.rad2deg(ee_pose.pitch):.1f}°, roll={np.rad2deg(ee_pose.roll):.1f}°")
```

### Inverse Kinematics (IK)

```python
from robopy.kinematics import EEPose

target = EEPose(x=0.1, y=0.0, z=0.1, pitch=0.0, roll=0.0)
current_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

result = robot.inverse_kinematics(target, current_joints)
print(f"収束: {result.success}")
print(f"位置誤差: {result.position_error:.6f} m")
```

### キネマティックチェーンの直接利用

```python
from robopy.kinematics import koch_chain, IKSolver
import numpy as np

chain = koch_chain()
solver = IKSolver(chain)

# FK: 関節角度（ラジアン）→ 姿勢
q = np.zeros(5)
pose = chain.forward_kinematics(q)

# IK: 姿勢 → 関節角度
result = solver.solve(target_pose=pose, initial_angles_rad=q)
```

### Koch のキネマティックパラメータ（推定値）

| 関節 | オフセット (m) | 回転軸 | 可動範囲 |
|------|---------------|--------|---------|
| shoulder_pan | (0, 0, 0.05) | Y | -150° ~ +150° |
| shoulder_lift | (0, 0, -0.04) | X | -100° ~ +100° |
| elbow | (-0.110, 0, 0) | X | -100° ~ +100° |
| wrist_flex | (-0.100, 0, 0) | X | -100° ~ +100° |
| wrist_roll | (0, -0.04, 0) | Y | -180° ~ +180° |

!!! note "5自由度の制約"
    Koch もgripperを除くと5自由度です。エンドエフェクタの位置（x, y, z）と姿勢のうち2成分（pitch, roll）を制御できますが、yaw（ヨー）は独立に制御できません。

## :material-link-variant: 関連クラス

- [`KochConfig`](../api/config.md#robopy.config.robot_config.koch_config.KochConfig) - ロボット設定
- [`EEPose`](../api/kinematics.md) - エンドエフェクタ姿勢
- [`IKSolver`](../api/kinematics.md) - 逆運動学ソルバー
