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

## :material-tools: セットアップ

xArm 本体および GELLO リーダーを robopy から利用するまでのハードウェア・ソフトウェア準備手順です。既にセットアップ済みのロボットを借用する場合は、本節をスキップして構いません。

!!! note "出典"
    本節の手順は `xArm_Modules` リポジトリ (<https://github.com/keio-crl/xArm_Modules>) の README に準拠しています。

### 1. 環境 (uv)

robopy は `uv` で管理されています。未インストールの場合は公式インストーラまたは `pip` で導入してください。

```bash
# 公式インストーラ
curl -LsSf https://astral.sh/uv/install.sh | sh

# もしくは pip 経由
pip install uv
```

インストール後、`uv` コマンドが実行できることを確認します。

```bash
$ uv
An extremely fast Python package manager.
Usage: uv [OPTIONS] <COMMAND>
...
```

### 2. GELLO ハードウェア (Dynamixel モーター ID 割り当て)

GELLO 本体の組み立てと、Dynamixel モーターへの ID 割り当てを行います。

1. GELLO の各種部品 (3D プリント品・購入品) を用意して組み立てます。
2. Dynamixel Wizard を導入します。インストールガイドは ROBOTIS 公式マニュアルを参照してください: <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/>
   **インストール時に記載されているコマンドはすべて実行してください** (飛ばすとエラーの原因になります)。
3. Dynamixel Wizard を起動し、まず **U2D2 単体** で接続確認を行います。
4. 一度完全に切断しソフトも閉じた上で、**モーターを 1 つだけ U2D2 に接続** → ソフト再起動 → ポートスキャンを行い、検出されたモーターに **ID 1** を割り当てます。
5. 以降、同様の手順で 2 個目以降のモーターにも順次 ID を割り当てます。
6. 全モーターに ID を割り当て終えたら、通常手順で GELLO 本体へ組み込みます。

### 3. GELLO ソフトウェアキャリブレーション

GELLO 本体を PC に接続した状態で、シリアル ID を確認します。

```bash
ls /dev/serial/by-id
# 例: usb-FTDI_USB__-__Serial_Converter_XXXXXXXX-if00-port0
```

表示された ID を控えておきます。

次に、GELLO 用の `gello_get_offset.py` で各モーターのオフセットとグリッパ開閉角度を取得します (`gello_software` リポジトリのスクリプトを使用)。

```bash
python scripts/gello_get_offset.py \
  --start-joints 0 0 0 0 0 0 0 \
  --joint-signs 1 1 1 1 1 1 1 \
  --port /dev/serial/by-id/<先ほど控えた ID>
```

実行すると、次のような出力が得られます。

```
best offsets : ['3.142' ..... ]
best offsets function of pi : [2*np.pi/2, ..... ]
gripper open(degrees)  202.xxxxxx
gripper close(degrees) 160.xxxxxx
```

これらの値は、GELLO で読み取ったモーター値を xArm に転送する際の補正量です。`gello/agents/gello_agent.py` の `PORT_CONFIG_MAP` に以下のようなエントリを追加してください (閉じ `}` の直前)。

```python
"/dev/serial/by-id/<控えた ID>": DynamixelRobotConfig(
    joint_ids=(1, 2, 3, 4, 5, 6, 7),
    joint_offsets=(
        # best offsets function of pi をそのまま並べる
        2 * np.pi / 2,
        ...
    ),
    joint_signs=(1, 1, 1, 1, 1, 1, 1),
    # gripper_config は (モーター ID, gripper open, gripper close) で、全て整数
    gripper_config=(8, 202, 160),
),
```

!!! info "robopy 側での利用"
    robopy ではこれらの値を [`GelloArmConfig`](../api/config.md#robopy.config.robot_config.xarm_config.GelloArmConfig) (`joint_offsets` / `joint_signs` / `gripper_open_deg` / `gripper_close_deg`) に設定します。

### 4. シミュレータ上での動作確認

GUI アプリを起動するため、**SSH 接続ではなくロボット実機が繋がっている PC 上で** 以下を実行します。ターミナルを 2 つ開いてください。

1 つ目 — シミュレータ起動:

```bash
python experiments/launch_nodes.py --robot sim_xarm
```

GUI が開いたら、もう一方のターミナルで:

```bash
python experiments/run_env.py --agent=gello
```

`Going to start position` で実行が止まる場合は、GELLO の姿勢がホームポジションから外れているのが原因です。できるだけホームに近い姿勢へ手で戻してからやり直してください。

実行が進むと、シミュレータ上の xArm が GELLO に追従します。モーターの向きや動きが明らかにおかしい場合は、`gello/agents/gello_agent.py` の `joint_offsets` を直接微調整してください。
(例: モーター ID 4 が 90° ずれていたケースでは、ラジアンで `1/2 π` ずれているため `joint_offsets` の 4 番目から `np.pi / 2` を引いて補正)

ここまで一致させれば、GELLO キャリブレーションは完了です。

### 5. xArm 本体の接続

#### 5.1 ケーブル類

1. xArm 本体とコントローラ Box を 2 本のケーブルで接続します。
2. Box の電源ケーブルを接続します。
3. 上部の非常停止スイッチを**回して押せる状態**にします。
4. **LAN ケーブルは、PC に直結ではなくルーター (またはハブ) に接続します** (純正の取扱説明書は PC 直結を指示していますが、無視してください)。
5. Box の電源ボタンを押して起動します。

#### 5.2 WebUI での疎通確認

コントローラ Box の LAN 端子の下に貼られているシールで、Box の IP アドレスを確認します (`192.168.1.XXX` の形式)。

ブラウザで以下にアクセスすると UFactory WebUI が起動し、xArm の状態確認・操作が行えます。

```
http://192.168.1.XXX:18333
```

ここまで疎通できれば、xArm のハードウェア側セットアップは完了です。`XArmConfig(follower_ip="192.168.1.XXX", ...)` としてこの IP を指定すれば、robopy から接続できます。

### 6. カメラ権限 (RealSense / Web カメラ)

RealSense や USB Web カメラから画像が取得できない場合、`/dev/video*` の権限不足が原因であることが多いです。全てのカメラを接続した状態で次を実行してください (sudo 権限が必要)。

```bash
sudo bash -c 'for device in /dev/video*; do chmod 666 "$device"; done'
```

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

## :material-database: データ収集

xArm では legacy `xarm_modules/saver.py` 相当のデータ収集を `XArmRobot` の 3 種類の `record*` API で行います。`RakudaExpHandler` のような専用ハンドラはまだ無いため、**記録は `XArmRobot` を直接使い、保存は numpy で手動**という形が基本パターンです。

### API 一覧

| メソッド | 用途 | リーダー | Follower | sensors |
|---|---|---|---|---|
| [`record`](../api/robots.md#robopy.robots.xarm.xarm_robot.XArmRobot.record) | シーケンシャル記録 (低 fps, シンプル) | GELLO ライブ | GELLO 追従 | 毎フレーム同期読み |
| [`record_parallel`](../api/robots.md#robopy.robots.xarm.xarm_robot.XArmRobot.record_parallel) | 並列記録 (推奨、高 fps) | GELLO ライブ | GELLO 追従 | スレッドプール並列読み |
| [`record_with_fixed_leader`](../api/robots.md#robopy.robots.xarm.xarm_robot.XArmRobot.record_with_fixed_leader) | 記録済み leader 軌道で再生しつつ収集 | 配列から補間再生 | 追従 | 並列読み |
| [`send`](../api/robots.md#robopy.robots.xarm.xarm_robot.XArmRobot.send) | 記録なしで軌道を再生のみ | 配列から補間再生 | 追従 | 読まない |

### 記録データの構造 ([`XArmObs`](../api/config.md#robopy.config.robot_config.xarm_config.XArmObs))

```python
obs.arms.leader        # (frames, 8)   — 7 joints [rad] + gripper [0, 1]
obs.arms.follower      # (frames, 8)   — 同上 (xArm 本体 or sim の状態)
obs.arms.ee_pos_quat   # (frames, 7)   — [x, y, z (m), qx, qy, qz, qw]

obs.sensors.cameras    # Dict[str, (frames, H, W, 3) | None]  — RealSense RGB
obs.sensors.tactile    # Dict[str, (frames, 3, H, W) | None]  — DIGIT (CHW に transpose 済)
obs.sensors.audio      # Dict[str, (frames, channels, samples) | None]
```

センサーは `XArmConfig.sensors` に設定したもののみ、辞書に **名前キー**で入ります。未接続センサーは `None`、`XArmConfig.sensors=None` なら辞書は空です。

### シーケンシャル記録 (`record`)

メインスレッドで「テレオペ 1 ステップ → センサー読み」を交互に行う単純ループ。**低 fps の動作確認**に向きます。センサー処理時間が長いと fps が維持できないため、高 fps には `record_parallel` を使ってください。

```python
obs = robot.record(max_frame=100, fps=5)
```

### 並列記録 (`record_parallel`) — 推奨

テレオペを独立スレッドで `teleop_hz` Hz で動かし、メインループは `fps` Hz でキュー最新値 + センサーを取得します。センサー読みも `ThreadPoolExecutor` で並列化。

```python
obs = robot.record_parallel(
    max_frame=500,
    fps=20,                      # 記録 (保存) のフレームレート
    teleop_hz=100,               # 内部テレオペループの Hz (fps 以上にする)
    max_processing_time_ms=40.0, # 1 フレーム処理の上限 (超えるとスキップ検知)
)
```

- `teleop_hz >= fps` が前提。通常 `teleop_hz = 4 × fps` 程度が安定。
- `max_processing_time_ms` を超えると `skipped_frames` にカウントされ、終了時に `logger.info` で出ます。sensor が重い場合はここを上げる。
- ログ: `record_parallel completed: frames=N, skipped=K, avg_proc=XXms`

### 軌道再生つき収集 (`record_with_fixed_leader`)

過去に記録した leader 軌道 `(max_frame, 8)` を時間線形補間しつつ follower に送り、実機応答 + センサーを記録します。**方策評価・再現実験**に使います。

```python
import numpy as np

# 過去データの leader 軌道を読み込み (例: 前回の record_parallel で保存した配列)
prior = np.load("data/trajectory_001/leader.npy")  # (N, 8)
obs = robot.record_with_fixed_leader(
    max_frame=prior.shape[0],
    leader_action=prior,
    fps=20,
    teleop_hz=100,
)
```

### 記録なし再生 (`send`)

保存は不要だが軌道だけ再生したい場合 (デモ、微調整ループなど):

```python
robot.send(max_frame=prior.shape[0], fps=20, leader_action=prior, teleop_hz=100)
```

終了時に Rich の `Table` で送信フレーム数・実効 Hz を出力します。

## :material-camera-plus-outline: センサー設定つきの完全例

`XArmConfig.sensors` に `XArmSensorParams` を渡すと、`connect()` 内でカメラ・触覚・音声を初期化し、記録時に同期取得されます。

=== "カメラのみ"

    ```python
    import numpy as np
    from robopy.config.robot_config import (
        XArmConfig, XArmSensorParams, XArmWorkspaceBounds,
    )
    from robopy.config.sensor_config.params_config import CameraParams
    from robopy.robots.xarm import XArmRobot

    config = XArmConfig(
        follower_ip="192.168.1.240",
        workspace_bounds=XArmWorkspaceBounds(),
        start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
        sensors=XArmSensorParams(
            cameras=[
                CameraParams(name="wrist", width=640, height=480, fps=30),
                CameraParams(name="env",   width=640, height=480, fps=30),
            ],
        ),
    )

    robot = XArmRobot(config)
    try:
        robot.connect()
        obs = robot.record_parallel(max_frame=300, fps=20, teleop_hz=80)
    finally:
        robot.disconnect()
    ```

=== "カメラ + 触覚 + 音声"

    ```python
    from robopy.config.sensor_config.params_config import (
        CameraParams, TactileParams, AudioParams,
    )

    config = XArmConfig(
        follower_ip="192.168.1.240",
        sensors=XArmSensorParams(
            cameras=[CameraParams(name="wrist", width=640, height=480, fps=30)],
            tactile=[TactileParams(serial_num="D20542", name="tip", fps=30)],
            audio=[AudioParams(name="mic", sample_rate=16000, channels=1)],
        ),
    )
    ```

## :material-content-save-outline: 記録データの保存

xArm 向け専用の保存ハンドラは現状未実装です。legacy `saver.py` と同じ形式で保存するなら、以下のようにして numpy で書き出します。

```python
from pathlib import Path
import numpy as np

def save_xarm_obs(obs, save_dir: str | Path, seq_id: int = 1) -> None:
    """legacy xarm_modules/saver.py の出力形式に合わせて保存する。

    Layout:
        save_dir/action_state_001.npy      # (T, 8) leader (= teleop 時の GELLO)
        save_dir/observation_state_001.npy # (T, 8) follower (xArm 実際の状態)
        save_dir/hand_001.npy              # (T, 7) ee_pos_quat
        save_dir/robot_image_001.npy       # (T, H, W, 3) 手先カメラ (あれば)
        save_dir/env_image_001.npy         # (T, H, W, 3) 環境カメラ (あれば)
        save_dir/tactile_<name>_001.npy    # (T, 3, H, W)
        save_dir/audio_<name>_001.npy      # (T, C, N)
    """
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    tag = f"{seq_id:03d}"

    np.save(save_dir / f"action_state_{tag}.npy",      obs.arms.leader)
    np.save(save_dir / f"observation_state_{tag}.npy", obs.arms.follower)
    np.save(save_dir / f"hand_{tag}.npy",              obs.arms.ee_pos_quat)

    if obs.sensors is not None:
        for name, arr in obs.sensors.cameras.items():
            if arr is not None:
                np.save(save_dir / f"camera_{name}_{tag}.npy", arr)
        for name, arr in obs.sensors.tactile.items():
            if arr is not None:
                np.save(save_dir / f"tactile_{name}_{tag}.npy", arr)
        for name, arr in obs.sensors.audio.items():
            if arr is not None:
                np.save(save_dir / f"audio_{name}_{tag}.npy", arr)
```

呼び出し例:

```python
obs = robot.record_parallel(max_frame=500, fps=20, teleop_hz=100)
save_xarm_obs(obs, "data/experiment_001", seq_id=1)
```

!!! tip "統合保存ハンドラ (将来)"
    Rakuda の `RakudaExpHandler` 相当 (`record_save` / HDF5 統合 / メタデータ自動付与) が必要な場合は、`src/robopy/utils/exp_interface/rakuda_exp_handler.py` を参考に `xarm_exp_handler.py` を追加するのが今後の改善方針です。

## :material-tune-variant: 性能チューニング

=== "fps が出ない"

    - `fps` を下げる、または `teleop_hz` を上げる (テレオペスレッドの更新頻度不足)
    - `max_processing_time_ms` を `1/fps * 1000 * 0.8` 程度に設定して、超過頻度を logger で観察
    - 重いセンサー (例: 高解像度カメラ 4 台) を同時記録する場合は解像度を落とす

=== "スキップが多い"

    - 終了時ログの `skipped=` の割合が 10% 超なら、`max_processing_time_ms` を緩めるか fps を下げる
    - USB 帯域 (RealSense 複数台) が原因の場合は、USB3 ポートを分散する

=== "GELLO 追従が遅い"

    - `XArmConfig.max_delta` を上げると追従速度 ↑ / 安全度 ↓ (既定 0.05 rad/step)
    - `XArmConfig.control_frequency` を上げると内部スレッドの指令頻度 ↑ (既定 50 Hz)
    - `cartesian_speed` / `cartesian_mvacc` は xArm SDK への引数で、これも上げると速くなる

## :material-file-document-multiple: データ記録の最小例

```python
import numpy as np
from robopy.config.robot_config import (
    XArmConfig, XArmSensorParams, XArmWorkspaceBounds,
)
from robopy.config.sensor_config.params_config import CameraParams
from robopy.robots.xarm import XArmRobot

def collect() -> None:
    config = XArmConfig(
        follower_ip="192.168.1.240",
        workspace_bounds=XArmWorkspaceBounds(),
        start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
        sensors=XArmSensorParams(
            cameras=[CameraParams(name="wrist", width=640, height=480, fps=30)],
        ),
    )
    robot = XArmRobot(config)
    try:
        robot.connect()
        # 初期姿勢を GELLO で合わせたあと Enter を待つ
        input("GELLO を xArm の姿勢に合わせたら Enter で開始...")
        obs = robot.record_parallel(
            max_frame=500,
            fps=20,
            teleop_hz=100,
            max_processing_time_ms=40.0,
        )
        print("leader:  ", obs.arms.leader.shape)
        print("follower:", obs.arms.follower.shape)
        print("ee:      ", obs.arms.ee_pos_quat.shape)
        # save_xarm_obs(obs, "data/experiment_001", seq_id=1)
    finally:
        robot.disconnect()

if __name__ == "__main__":
    collect()
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
