# キネマティクス API

このページでは、Robopyのキネマティクス（運動学）モジュールに関連するAPIを説明します。

## 概要

`robopy.kinematics` モジュールは、5自由度ロボットアームのForward Kinematics (FK) と Inverse Kinematics (IK) を提供します。

- **FK**: 関節角度 → エンドエフェクタ姿勢（位置 + 姿勢）
- **IK**: エンドエフェクタ姿勢 → 関節角度（数値解法）

外部依存は **numpy のみ** です。scipy は不要です。

## :material-axis-arrow: エンドエフェクタ姿勢

::: robopy.kinematics.ee_pose.EEPose
    options:
      show_root_heading: true
      show_source: true
      members:
        - to_array
        - from_array

## :material-link: キネマティックチェーン

::: robopy.kinematics.chain.RevoluteJoint
    options:
      show_root_heading: true
      show_source: true

::: robopy.kinematics.chain.KinematicChain
    options:
      show_root_heading: true
      show_source: true
      members:
        - n_joints
        - joint_names
        - lower_limits_rad
        - upper_limits_rad
        - forward_kinematics_matrix
        - forward_kinematics
        - jacobian
        - clamp_to_limits

## :material-calculator: IKソルバー

::: robopy.kinematics.ik_solver.IKConfig
    options:
      show_root_heading: true
      show_source: true

::: robopy.kinematics.ik_solver.IKResult
    options:
      show_root_heading: true
      show_source: true

::: robopy.kinematics.ik_solver.IKSolver
    options:
      show_root_heading: true
      show_source: true
      members:
        - config
        - solve

## :material-robot: ロボット固有のチェーン定義

::: robopy.kinematics.robot_chains.so101_chain
    options:
      show_root_heading: true
      show_source: true

::: robopy.kinematics.robot_chains.koch_chain
    options:
      show_root_heading: true
      show_source: true

## アルゴリズム

### Damped Least-Squares (DLS) IK

IKソルバーは **Damped Least-Squares** アルゴリズムを使用しています:

$$
\Delta q = J_w^T (J_w J_w^T + \lambda^2 I)^{-1} e_w
$$

ここで:

- $J_w$: 重み付きヤコビアン行列 (5 x n_joints)
- $e_w$: 重み付きタスク空間誤差 (5,)
- $\lambda$: ダンピング係数（`IKConfig.damping`）
- $I$: 単位行列

この手法の特徴:

- 特異姿勢（腕が完全に伸びた状態など）でも安定
- 位置と姿勢に独立した重みを設定可能
- 5x5の線形方程式を `np.linalg.solve` で解くため高速
- scipy 不要（numpy のみ）

### 数値ヤコビアン

ヤコビアンは中心差分法で計算されます:

$$
J_{ij} = \frac{f(q + \delta e_j) - f(q - \delta e_j)}{2\delta}
$$

5関節の場合、1回のヤコビアン計算に10回のFK評価が必要ですが、各FKは5回の4x4行列乗算のみなので100Hzの制御ループでも十分高速です。

## 使用例

### オフラインでのFK/IK計算

ハードウェア接続なしでFK/IK計算を行う例:

```python
import numpy as np
from robopy.kinematics import so101_chain, koch_chain, IKSolver, IKConfig

# SO-101のチェーンを作成
chain = so101_chain()

# 全関節0度でのEE位置を確認
home_pose = chain.forward_kinematics(np.zeros(5))
print(f"Home位置: x={home_pose[0]:.4f}, y={home_pose[1]:.4f}, z={home_pose[2]:.4f}")

# IKで目標姿勢に到達する関節角度を求める
solver = IKSolver(chain, IKConfig(max_iterations=200))
result = solver.solve(
    target_pose=home_pose,
    initial_angles_rad=np.zeros(5),
)
print(f"収束: {result.success}, 反復回数: {result.iterations}")
```

### ロボットクラスからの利用

```python
from robopy.robots.so101.so101_robot import So101Robot
from robopy.kinematics import EEPose
import numpy as np

# FK（classmethodなのでインスタンス不要）
joint_angles = np.array([0.0, 30.0, -45.0, 20.0, 0.0, 50.0], dtype=np.float32)
ee_pose = So101Robot.forward_kinematics(joint_angles)

# IK（接続済みロボットで）
# target = EEPose(x=0.1, y=0.0, z=0.15, pitch=0.0, roll=0.0)
# result = robot.inverse_kinematics(target, current_joints)

# EEアクション送信（接続済みロボットで）
# robot.send_ee_frame_action(np.array([0.1, 0.0, 0.15, 0.0, 0.0, 50.0]))
```
