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

---

## :material-robot-industrial: 全身モデルと双腕IK（optional extra）

既存の `KinematicChain` / `IKSolver`（5自由度直列アーム、numpyのみ）はそのままです。
以下は **URDF全体**を扱い、左右TCPのSE(3)目標を**1つのQP**で同時に解くための追加APIです。

依存は optional extra `kinematics` にまとめてあり、**遅延import**されます。
未導入でも従来のrobopyはそのまま使えます。

```bash
uv sync --extra kinematics
# または
uv pip install 'robopy[kinematics]'
```

導入されるのは [Pinocchio](https://github.com/stack-of-tasks/pinocchio)（PyPI名 `pin`）、
[Pink](https://github.com/stephane-caron/pink)（PyPI名 `pin-pink`）、`qpsolvers` と `quadprog` です。
**PyPI上の同名別パッケージに注意してください。正しい名前は `pin` と `pin-pink` です。**

### URDF監査

```bash
python -m robopy.kinematics.urdf_audit robot.urdf --package-dir pkgs --json
```

::: robopy.kinematics.urdf_audit.UrdfAudit
    options:
      show_root_heading: true
      members:
        - n_links
        - n_joints
        - n_movable
        - joint
        - usable_for_dynamics
        - summary
        - to_json

::: robopy.kinematics.urdf_audit.audit_urdf
    options:
      show_root_heading: true

### 全身モデル

`nq`（配置次元）と `nv`（速度次元）は continuous 関節があると一致しません。
位置は `positions_from_q` / `q_from_positions`、配置の更新は `integrate` を通してください。

::: robopy.kinematics.urdf_model.WholeBodyModel
    options:
      show_root_heading: true
      members:
        - from_urdf
        - nq
        - nv
        - movable_joint_names
        - has_joint
        - is_continuous
        - joint_v_index
        - joint_q_slice
        - neutral_q
        - positions_from_q
        - q_from_positions
        - integrate
        - difference
        - set_soft_limits
        - position_limits
        - unbounded_joints
        - add_fixed_frame
        - frame_pose
        - frame_jacobian
        - classify_collision_pairs
        - add_all_collision_pairs
        - collision_report
        - distance_jacobian_row

### 双腕IK

胴体yawは左右の共通祖先にある**1変数**として扱われます。左右を別々に解いて胴体角を平均する
実装は行いません。頭部はモデルには入りますが（衝突形状を動かすため）、QPの決定変数からは
外れているので、IKが頭部を動かすことはありません。

::: robopy.kinematics.dual_arm_ik.DualArmIK
    options:
      show_root_heading: true
      members:
        - active_joints
        - solve_step
        - reset
        - set_posture_reference

::: robopy.kinematics.dual_arm_ik.DualArmIKConfig
    options:
      show_root_heading: true

::: robopy.kinematics.dual_arm_ik.DualArmIKResult
    options:
      show_root_heading: true
      members:
        - is_commandable

::: robopy.kinematics.dual_arm_ik.DualArmIKStatus
    options:
      show_root_heading: true

### 合成モデル（テスト用）

Rakuda本体のURDFが手元にない場合でも、同じトポロジ（共有胴体1 + 左右各6 + 頭部2、
`*_dof` という名前の固定グリッパフレーム付き）を持つ合成モデルで実装とテストを進められます。
**リンク長・軸・質量は架空の値**であり、実機の幾何や動力学の主張ではありません。

::: robopy.kinematics.synthetic_dual_arm.synthetic_dual_arm_urdf
    options:
      show_root_heading: true

::: robopy.kinematics.synthetic_dual_arm.write_synthetic_dual_arm_urdf
    options:
      show_root_heading: true

### 使用例

```python
import numpy as np
from robopy.control.types import DualArmTarget, TorsoPolicy
from robopy.kinematics.dual_arm_ik import DualArmIK, DualArmIKConfig
from robopy.kinematics.urdf_model import WholeBodyModel

model = WholeBodyModel.from_urdf("robot.urdf", build_collision=True, geometry_only=True)

# TCPは既存フレームからの「測定した」固定変換で定義する。
tcp = np.eye(4)
tcp[2, 3] = -0.02
model.add_fixed_frame("left_tcp", "gripper_left_dof", tcp)
model.add_fixed_frame("right_tcp", "gripper_right_dof", tcp)

# continuous関節はURDFに範囲がないので、実測のソフト制限が必須。
model.set_soft_limits({"torso_yaw_dof": (-1.5, 1.5)})

# 除外する衝突ペアは分類して、理由つきで明示的に決める。
groups = model.classify_collision_pairs(model.neutral_q())
model.add_all_collision_pairs(
    excluded=groups["parent_child"] + groups["same_body"] + groups["interfering_at_q"]
)

ik = DualArmIK(
    model,
    left_frame="left_tcp",
    right_frame="right_tcp",
    torso_joint="torso_yaw_dof",
    left_arm_joints=[...],   # 肩→手首の6関節
    right_arm_joints=[...],
    head_joints=["head_yaw_dof", "head_pitch_dof"],
    config=DualArmIKConfig(),
)

result = ik.solve_step(state, DualArmTarget(
    left_target=T_left, right_target=T_right, torso_policy=TorsoPolicy.OPTIMIZE
), dt=0.02)

if result.is_commandable:
    send(result.joint_targets_rad)
else:
    # 計算失敗・時間超過・古い状態・到達不能・開始時衝突は区別される。
    # 無効な結果は新しい運動目標として発行しない。
    log(result.status, result.message)
```

### 胴体方針

| `TorsoPolicy` | 動作 |
| --- | --- |
| `FIXED` | 腕IKへの胴体速度をゼロに拘束する |
| `MANUAL` | 制限した胴体速度を既知量として両手タスクへ反映する（腕が補償する） |
| `OPTIMIZE` | 胴体を自由変数にする |

片腕だけを駆動する場合、反対側TCPは**操作開始時または明示的な再基準化時**に保存した
保持目標を追い続けます。毎周期の測定姿勢で保持目標を上書きすることはありません。
保持は有限重みの目標なので、「厳密な固定」とは表示せず残差
（`left_hold_residual_m` / `right_hold_residual_m`）を返します。
