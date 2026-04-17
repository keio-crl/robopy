# xArm Robot

[`XArmRobot`](../api/robots.md#robopy.robots.xarm.xarm_robot.XArmRobot) は UFactory xArm7 を GELLO (Dynamixel ベースのテレオペコントローラ) で操作する leader/follower 構成のロボットです。`RakudaRobot` / `KochRobot` と同じ `ComposedRobot` インターフェースを実装しており、`connect`/`teleoperation`/`record`/`record_parallel` が同じ手順で使えます。

## :material-robot-outline: 構成要素

### ロボットアーム

- **Leader Arm** ([`XArmLeader`](../api/robots.md#robopy.robots.xarm.xarm_leader.XArmLeader)): GELLO Dynamixel テレオペコントローラ。robopy の `DynamixelBus` 経由で接続され、GELLO submodule 依存はありません。
- **Follower Arm** ([`XArmFollower`](../api/robots.md#robopy.robots.xarm.xarm_follower.XArmFollower)): UFactory xArm7 本体。`xarm-python-sdk` で TCP/IP 経由接続、50 Hz の背景スレッドで速度制限付きに指令を送ります。
- **Pair System** ([`XArmPairSys`](../api/robots.md#robopy.robots.xarm.xarm_pair_sys.XArmPairSys)): Leader→Follower 伝搬と 0.8 rad 不整合チェック + 25 ステップ安全アライメントを実行。

### センサー統合

- **カメラ**: Intel RealSense
- **タクタイル**: DIGIT
- **オーディオ**: PyAudio

## :material-cog: 基本的な使用方法

### 設定の作成

```python
import numpy as np
from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds

config = XArmConfig(
    follower_ip="192.168.1.240",         # xArm 本体の IP
    leader_port=None,                    # None なら /dev/serial/by-id から自動検出
    workspace_bounds=XArmWorkspaceBounds(),   # mm 単位のクリッピング
    start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
    control_frequency=50.0,              # xArmAPI 制御スレッド Hz
    max_delta=0.05,                      # 1 ステップあたり最大ジョイント移動量 [rad]
)
```

### ロボットの操作

```python
from robopy.robots.xarm import XArmRobot

robot = XArmRobot(config)
try:
    robot.connect()
    robot.teleoperation(max_seconds=30.0)   # GELLO → xArm を 30 秒間
finally:
    robot.disconnect()
```

## :material-database: データ記録

```python
# 並列記録（高 fps 対応）
obs = robot.record_parallel(
    max_frame=500,
    fps=20,
    teleop_hz=100,
    max_processing_time_ms=40.0,
)

print(obs.arms.leader.shape)        # (500, 8)  — 7 joints [rad] + gripper [0, 1]
print(obs.arms.follower.shape)      # (500, 8)
print(obs.arms.ee_pos_quat.shape)   # (500, 7)  — xyz [m] + quat [xyzw]
```

## :material-test-tube: シミュレータ上で先に動作確認する (強く推奨)

実機の xArm7 を GELLO で動かす前に、**UFactory Studio / UFactory Simulator を使って動作確認** することを強く推奨します。

GELLO 側の挙動 (joint offset, gripper 正規化, 速度制限) やワークスペース境界のクリッピングが意図通りかを、衝突・落下などのリスクなしで検証できます。

!!! info "シミュレータとは"
    UFactory Studio は UFactory 公式の無料デスクトップアプリで、**実機コントローラ Box と同じ xArm プロトコル (TCP/IP)** を話す仮想 xArm を提供します。`xarm-python-sdk` から見ると実機と区別がつかないため、robopy 側のコードは **一切変更不要** で、`follower_ip` をシミュレータのアドレスに向けるだけで動きます。

### 1. UFactory Studio のインストール

UFactory 公式ダウンロードページから、利用 OS 向けのインストーラを取得します。

- 公式ダウンロード: <https://www.ufactory.cc/download-ufactory-studio/>
- 対応 OS: Windows / macOS / Linux (Ubuntu)
- xArm7 を選択してインストール

!!! tip "ヘッドレスサーバで使う場合"
    Studio GUI が使えない環境 (ヘッドレス Linux など) では、UFactory が提供する **xArm Firmware Simulator (Docker)** が利用できる場合があります。詳細は UFactory Developer 向けドキュメント (<https://github.com/xArm-Developer>) を参照してください。

### 2. Studio 内で Simulation/Virtual Robot を有効化

Studio を起動後、**Simulation Mode** または **Virtual Robot** を有効にし、ロボットモデルに **xArm7** を選択します。Studio の起動画面で IP アドレス (通常は `127.0.0.1` またはホスト PC の LAN IP) が表示されるのでメモします。

UI の詳細は Studio のバージョンによって異なるため、公式マニュアル ([Studio User Manual](https://www.ufactory.cc/ufactory-studio-software/)) を参照してください。

### 3. robopy 側の設定変更

`XArmConfig.follower_ip` をシミュレータ側に向けるだけで、他は実機と同じ設定のまま動作します。

```python
config = XArmConfig(
    follower_ip="127.0.0.1",             # ← シミュレータの IP に変更
    leader_port=None,                    # GELLO は実機を USB 接続
    workspace_bounds=XArmWorkspaceBounds(),
    start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
)

robot = XArmRobot(config)
robot.connect()          # 内部で XArmAPI("127.0.0.1", is_radian=True) が呼ばれる
robot.teleoperation(max_seconds=30.0)
robot.disconnect()
```

### 4. 接続確認 (sanity check)

シミュレータ接続だけを先に検証したい場合は、`XArmFollower` 単体で確認できます。

```python
from robopy.config.robot_config import XArmConfig
from robopy.robots.xarm import XArmFollower

cfg = XArmConfig(follower_ip="127.0.0.1")
follower = XArmFollower(cfg)
follower.connect()
print("joints:", follower.get_joint_state())
print("ee_pos_quat:", follower.get_ee_pos_quat())
follower.disconnect()
```

Studio の 3D ビュー上で xArm が表示されていれば、`get_servo_angle` / `get_position_aa` の値がそれと一致するはずです。

### 5. GELLO 連携の検証フロー

```python
# 1. GELLO のみ接続、フォロワーはシミュレータ
config = XArmConfig(follower_ip="127.0.0.1")
robot = XArmRobot(config)
robot.connect()

# 2. 1 ステップだけ実行してログを確認
obs = robot.robot_system.teleoperate_step()
print("leader:  ", obs.leader)      # GELLO の状態
print("follower:", obs.follower)    # シミュレータ xArm の状態

# 3. 短時間のテレオペで 3D ビュー上の動きを確認
robot.teleoperation(max_seconds=10.0)

robot.disconnect()
```

Studio の 3D ビュー上で GELLO の動きに xArm モデルが追従していれば OK です。

### :material-arrow-right: 実機に切り替える前のチェックリスト

シミュレータでの確認後、実機に戻す際は以下を確認してください:

- [ ] `follower_ip` を実機の IP (既定値 `"192.168.1.240"`) に戻した
- [ ] 非常停止スイッチが手の届く位置にある
- [ ] `workspace_bounds` が設置環境に対して安全に設定されている
- [ ] `start_joints` がホームポジション相当になっている
- [ ] `max_delta` が小さすぎ/大きすぎない (既定 0.05 rad/step)
- [ ] GELLO と xArm 本体の初期姿勢差が 0.8 rad 未満 ( `_align_leader_follower` で自動検出)
- [ ] 初回は `max_seconds=5` 程度の短時間テレオペで動作確認

### :material-information-outline: シミュレータと実機の差分

| 項目 | シミュレータ | 実機 |
|---|---|---|
| FK (`get_forward_kinematics`) | 同じ | 同じ |
| `set_position` / `get_servo_angle` | 理想追従、遅延ほぼなし | 実際のモータ応答、ギア誤差あり |
| `set_collision_sensitivity` | Studio 設定に依存 | 実機キャリブレーションに依存 |
| `set_collision_rebound` | 視覚的リバウンドのみ | 実機で物理的に戻る |
| グリッパ (`get/set_gripper_position`) | Studio 設定により擬似動作 | 実機の電動グリッパ |
| collision / 力覚 | 物理衝突は発生しない | 安全設定に従い停止 |

シミュレータでは物理衝突が発生しないため、GELLO 側で無理な姿勢 (シンギュラリティ直近など) を取っても止まりません。実機では `_clear_error_states` による復帰ループが走ることがあるため、最初の実機テストは **必ず有人監視下** で行ってください。

## :material-file-document: 最小テレオペ例

完全な動作例は [`examples/robot/xarm_teleoperate.py`](https://github.com/keio-crl/robopy/blob/main/examples/robot/xarm_teleoperate.py) にあります。

```python
from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds
from robopy.robots.xarm import XArmRobot
import numpy as np

def xarm_teleoperate() -> None:
    config = XArmConfig(
        follower_ip="127.0.0.1",                                     # シミュレータ
        workspace_bounds=XArmWorkspaceBounds(),
        start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
    )
    robot = XArmRobot(config)
    try:
        robot.connect()
        robot.teleoperation(max_seconds=30.0)
    finally:
        robot.disconnect()

if __name__ == "__main__":
    xarm_teleoperate()
```

## :material-alert-circle: トラブルシューティング

### シミュレータに接続できない

- Studio が起動しており、仮想ロボットが xArm7 として有効になっているか確認
- `XArmConfig.follower_ip` が Studio 表示の IP と一致しているか確認
- ファイアウォールで TCP 接続がブロックされていないか確認 (特に Windows)
- `127.0.0.1` で繋がらない場合はホスト PC の LAN IP を試す

### GELLO のポートが自動検出されない

```python
# 明示的にポート指定
config = XArmConfig(
    leader_port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXXXXXX-if00-port0",
    follower_ip="127.0.0.1",
)
```

`ls /dev/serial/by-id/` で GELLO のデバイス ID を確認できます。

### 初期姿勢差が大きすぎる

`_align_leader_follower` が `RuntimeError: Leader/Follower initial mismatch > 0.8 rad` で失敗する場合、GELLO を `start_joints` (既定 `[0, -90, 90, -90, -90, 0, 0]` deg) に近い姿勢に手で動かしてから `connect()` を呼び直してください。

## :material-link-variant: 関連クラス

- [`XArmConfig`](../api/config.md#robopy.config.robot_config.xarm_config.XArmConfig) — ロボット設定
- [`XArmPairSys`](../api/robots.md#robopy.robots.xarm.xarm_pair_sys.XArmPairSys) — Leader/Follower 協調制御
- [`XArmLeader`](../api/robots.md#robopy.robots.xarm.xarm_leader.XArmLeader) — GELLO テレオペコントローラ
- [`XArmFollower`](../api/robots.md#robopy.robots.xarm.xarm_follower.XArmFollower) — xArm7 本体ラッパ
- [`GelloArmConfig`](../api/config.md#robopy.config.robot_config.xarm_config.GelloArmConfig) — GELLO キャリブレーション
- [`XArmWorkspaceBounds`](../api/config.md#robopy.config.robot_config.xarm_config.XArmWorkspaceBounds) — Cartesian ワークスペース境界
