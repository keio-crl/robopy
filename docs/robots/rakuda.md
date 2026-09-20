# Rakuda Robot

[`RakudaRobot`](../api/robots.md#robopy.robots.rakuda.rakuda_robot.RakudaRobot)は、二腕ロボットシステムでLeader/Followerアーム構成を持つロボットです。テレオペレーションとデータ収集機能を提供します。

## :material-robot-outline: 構成要素

### ロボットアーム

- **Leader Arm** ([`RakudaLeader`](../api/robots.md#robopy.robots.rakuda.rakuda_leader.RakudaLeader)): 操作者が制御するアーム
- **Follower Arm** ([`RakudaFollower`](../api/robots.md#robopy.robots.rakuda.rakuda_follower.RakudaFollower)): Leaderの動きを追従するアーム
- **Pair System** ([`RakudaPairSys`](../api/robots.md#robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys)): 両アームの協調制御

### センサー統合

- **カメラ**: Intel RealSenseカメラ対応
- **タクタイルセンサー**: DIGIT触覚センサー対応

## :material-cog: 基本的な使用方法

### 設定の作成

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(
        cameras=None,
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
    slow_mode=False,  # 高速モード
)
```

### `.robopy/rakuda/config.yaml` による関節トルク設定

Rakudaを使用すると、実行ディレクトリ直下に `.robopy/rakuda/config.yaml` が自動生成されます。
このYAMLで、leader/followerそれぞれ「トルクONにする関節」を関節名で指定できます。

- **read（観測）は常に全関節**
- **write（目標位置送信）はトルクON関節のみ**

例：followerは頭だけトルクON、leaderはgripperだけトルクON

```yaml
leader:
    torque_enabled:
        - l_arm_grip
        - r_arm_grip

follower:
    torque_enabled:
        - torso_yaw
        - head_yaw
        - head_pitch
```

`torque_enabled: null`（またはキー自体を省略）にするとデフォルト挙動になります：

- leader: gripperのみトルクON
- follower: 全関節トルクON

`torque_enabled: []` を指定すると、全関節トルクOFFになります。

`torque_enabled: all` を指定すると、全関節トルクONになります。

利用可能な関節名は、生成された `config.yaml` 内コメント（Available joint names）を参照してください。

### ロボットの操作

```python
from robopy import RakudaRobot

robot = RakudaRobot(config)

try:
    # 接続
    robot.connect()

    # テレオペレーション（10秒間）
    robot.teleoperation(duration=10)

    # データ記録
    obs = robot.record_parallel(max_frame=500, fps=30)

finally:
    robot.disconnect()
```

## :material-database: データ記録機能

### 標準記録

```python
# シーケンシャル記録（低fps推奨）
obs = robot.record(max_frame=100, fps=5)
```

### 並列記録

```python
# 並列記録（高fps対応）
obs = robot.record_parallel(
    max_frame=1000,
    fps=20,
    max_processing_time_ms=25.0
)
```

### 記録データ構造

記録されるデータは[`RakudaObs`](../api/config.md#robopy.config.robot_config.rakuda_config.RakudaObs)型で、以下の構造を持ちます：

```python
{
    "arms": {
        "leader": np.ndarray,    # (frames, 17) - 関節角度など
        "follower": np.ndarray,  # (frames, 17) - 関節角度など
    },
    "sensors": {
        "cameras": {
            "main": np.ndarray,  # (frames, H, W, C) - RGB画像
        },
        "tactile": {
            "left": np.ndarray,  # (frames, H, W, C) - 触覚データ
            "right": np.ndarray, # (frames, H, W, C) - 触覚データ
        }
    }
}
```

## :material-flask: 実験ハンドラー

[`RakudaExpHandler`](../api/utils.md#robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler)を使用すると、より簡単に実験を実行できます：

```python
from robopy.utils.exp_interface import RakudaExpHandler

handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",
    right_digit_serial="D20537",
    fps=30
)

# インタラクティブな記録・保存
handler.record_save(
       max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```

## :material-speedometer: パフォーマンス最適化

### 並列処理

`record_parallel()`メソッドでは、アーム制御とセンサー読み取りを並列化して高速化しています：

```python
# 最大25msの処理時間制限で30Hz記録
obs = robot.record_parallel(
       max_frame=1000,
    fps=30,
    max_processing_time_ms=25.0
)
```

### メモリ効率

- フレームバッファリングなし
- 最新フレームのみ保持
- 効率的な numpy 配列変換

## :material-alert-circle: トラブルシューティング

### 接続エラー

```python
try:
    robot.connect()
except RuntimeError as e:
    print(f"接続失敗: {e}")
    # ポートやボーレート設定を確認
```

### センサー読み取りエラー

```python
# センサーの個別確認
if robot.sensors.tactile:
    for sensor in robot.sensors.tactile:
        if sensor.is_connected:
            print(f"{sensor.name}: 接続OK")
```

### パフォーマンス問題

```python
# フレームスキップ確認
obs = robot.record_parallel(max_frame=100, fps=20)
# ログでスキップされたフレーム数を確認
```

## :material-link-variant: 関連クラス

- [`RakudaConfig`](../api/config.md#robopy.config.robot_config.rakuda_config.RakudaConfig) - ロボット設定
- [`RakudaPairSys`](../api/robots.md#robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys) - アーム協調制御
- [`RakudaLeader`](../api/robots.md#robopy.robots.rakuda.rakuda_leader.RakudaLeader) - リーダーアーム
- [`RakudaFollower`](../api/robots.md#robopy.robots.rakuda.rakuda_follower.RakudaFollower) - フォロワーアーム

## :material-arm-flex: 制御モード（双腕IK / バイラテラル）

`.robopy/rakuda/config.yaml` に `control:` セクションを書くと、従来の関節位置テレオペに加えて
2つのモードが使えます。**セクションを書かなければ挙動は一切変わりません**（モデルも読み込まず、
電流も一切出力しません）。

| モード | 入力と動作 | IK | 使用する制御モード |
| --- | --- | --- | --- |
| `position_teleop` | 既存どおり、リーダ関節位置をフォロワへ送る | 不要 | 位置制御(3) |
| `cartesian_teleop` | 左右TCP目標からフォロワ関節目標を生成する | 双腕IK | 位置制御(3) |
| `bilateral_joint` | 両側の関節位置・速度から仮想ばね・ダンパのトルクを計算する | 使用しない | 電流制御(0) |

### 実機なしで動かす

どちらのモードも、模擬バス（[`SimulatedDynamixelBus`](../api/robots.md)）で最後まで実行できます。

```bash
# バイラテラル（模擬。接触反力の例は --contact）
uv run python examples/robot/rakuda_bilateral.py --contact

# 双腕IK（kinematics extra が必要）
uv sync --extra kinematics
uv run --extra kinematics python examples/robot/rakuda_cartesian_teleop.py --collision
```

### 起動

```python
from robopy.robots.rakuda import RakudaPairSys

pair = RakudaPairSys(config)
pair.connect()

system = pair.start_control()      # configure → align → start
...
pair.stop_control()                # 停止方針を適用してバスを返す
```

`start_control()` 実行中は両バスの指令権を制御システムが保持し、
`control_step()` / `send_follower_action()` などの旧経路は明示的に拒否されます
（1ポート1書き手）。観測API (`get_observation()`) は配列の形・順序・単位（degree）を
変えずに、サーボのキャッシュを読みます。SI単位の詳細状態は `pair.detailed_state()` で取れます。

## :material-cube-scan: 3Dビューア／シミュレータ（実機不要）

![robopy viewer: 実モデル、End effector タブ](assets/rakuda_viewer.png)

UFactory Studio の「モデルだけを動かして確認する」に相当するブラウザUIです。
関節角度（スライダ／数値／±ジョグ）またはエンドエフェクタ姿勢（xyz・roll/pitch/yaw のスライダ／数値／±ジョグ）で
モデルを動かし、3D表示で確認できます。3D表示上のTCPの球を**直接ドラッグ**（クリック、またはタップ＆ホールド）しても動かせます。
**ページ上の何もモータへは送られません。**

```bash
uv sync --extra kinematics

# 引数なし: パッケージ同梱の実モデル（robopy/models/rakuda）で起動。
#   視覚メッシュ（`robopy-models fetch` または `git lfs pull`）があればそれを、なければ凸包（<collision>）を描画
uv run robopy-viewer
uv run robopy-viewer --geometry collision   # 凸包を明示的に描く

# 合成モデル（Rakudaのトポロジ、幾何は架空）
uv run robopy-viewer --synthetic

# 別の場所にあるURDFを指定
uv run robopy-viewer --urdf <path>/assembly_2.urdf --package-dir <models> \
    --soft-limit torso_yaw_dof=-1.57,1.57

# .robopy/rakuda/config.yaml の control.model（URDF・ソフト制限・TCP・関節グループ）を使う
uv run robopy-viewer --config
```

**関節範囲は実機 leader-follower と同じ「モータの可動範囲」に揃えてあります**: スライダは
0〜4095カウント（ゼロ点2048）に対応する **±180°**、初期値は 0°（＝ゼロカウント）です。
CADエクスポート（URDF）が宣言する範囲ではありません — URDF側は `shoulder_roll_left_dof` が
−22.5°〜197.5°、`elbow_yaw_left_dof` が −139.1°〜220.9° のように非対称で、実機で動かせる範囲と一致しません。
ただしこれは**サーボが許す範囲**であって可動域の実測ではありません。実際に止まる位置は計測値なので、
`.robopy/rakuda/config.yaml`（`--config`）か `--soft-limit` でソフト制限として与えてください（与えればスライダも狭まります）。
ソルバ（IK）は従来どおりURDFの範囲＋ソフト制限を守ります。その範囲外へスライダで動かしてから solve すると、
実機と同じく「limits を既に逸脱している」と言って動きません。

`http://127.0.0.1:8765` を開きます（`--port`, `--host`, `--no-browser`, `--no-ik` あり）。

| タブ | 内容 |
| --- | --- |
| Joints | 胴体／左腕／右腕／頭部ごとのスライダ・数値入力・±ジョグ（deg/rad切替、ステップ幅）。範囲はモータ可動範囲 ±180°（初期値0°）、ツールチップに出典とソルバ側の範囲を表示。`zero all`、`copy JSON`（rad） |
| End effector | 左右TCPの現在姿勢（mm / deg）と目標。3Dビュー上の球を直接ドラッグ、各成分のスライダ（ドラッグ中も逐次IK）・数値入力・±ジョグ、`capture`、左右の `track TCP`、非操作腕の follow torso / hold TCP in world、胴体方針 fixed/manual/optimize、姿勢モード soft/keep/free、`solve`／ジョグごとに自動solve |
| Info | URDFパス、nq/nv、IKの関節グループ（名前から推定した場合もここに明示）、モデル監査の警告 |

片腕操作では、反対側の `track TCP` を外してください。既定の `follow torso (keep joints)`
では非操作腕の6関節にIKによる運動を指令せず、TCPのワールド座標は共有胴体と一緒に変化します。
両方の `track TCP` が有効なら、両手の目標を追跡します。再度有効にした腕の目標は、
その時点のTCP姿勢で取り直します。

**胴体の既定は `optimize`** です。片腕の届く範囲を決めているのは共有胴体で、`fixed` にすると
その腕の6関節だけで到達しなければならず、「もう片腕が動かなくて済む範囲」までしか手先が動きません
（実モデルで計測。右手を +y へ動かす指令に対する残差）:

| 指令量 | `torso: fixed` | `torso: optimize` |
| --- | --- | --- |
| 50 mm | 0.7 mm | 1.0 mm（胴体 −7.0°） |
| 100 mm | 8.1 mm | 0.6 mm（胴体 −21.7°） |
| 200 mm | 72.3 mm | 0.9 mm（胴体 −83.4°） |
| 300 mm | 156.5 mm | 0.9 mm（胴体 −126.2°） |

どちらも非操作腕の関節変化は 0.00° です（`optimize` ではTCPが胴体に運ばれて動きます）。
`fixed` のまま届かなかった場合は、ステータスにその旨と `optimize` の案内を出します。

Python APIも同じ既定動作です。例えば右腕のみなら
`DualArmTarget(left_enabled=False, right_target=T_right, torso_policy=TorsoPolicy.OPTIMIZE)`
とします。従来の非操作TCPのワールド座標保持が必要な場合だけ、
`inactive_arm_policy=InactiveArmPolicy.HOLD_WORLD` を指定してください。
`InactiveArmPolicy` は `robopy.control` からインポートできます。
非操作腕の衝突形状・関節範囲のチェックは残ります。この設定はモータのトルクOFFを意味しません。

設計上のポイント:

- **運動学は全てサーバ側**（`WholeBodyModel` / `DualArmIK`）。ページはFK/IKの結果を描くだけなので、
  表示と制御スタックの解が食い違いません。Pinocchio/Pinkが必要（`kinematics` extra）。
- 3D描画は three.js を**同梱**（`src/robopy/viewer/static/vendor/`、MIT）。オフラインの実験室でも動きます。
- **関節範囲の出典**: `/api/model` は各関節について、スライダ範囲（`lower`/`upper`）とその出典（`limit_source`:
  `motor` / `soft` / `urdf` / `display`）、そしてソルバが守る範囲（`model_lower`/`model_upper`）を別々に返します。
  「実機で動かせる範囲」と「モデルが認める範囲」を混ぜないための区別です。`--synthetic` の合成モデルには
  モータが無いので、従来どおりURDFの範囲を使います。
- **接地面**: グリッドは床です。関節で動かないジオメトリ（＝土台）の最下面に置きます。モデル原点ではありません —
  Rakudaのエクスポートでは原点は土台の底から約 26 cm 上にあり、原点に描いていたグリッドは胴体を突き抜けていました。
  土台を描かないモデルでは従来どおり root フレーム（z = 0）に置きます。
- **手先の直接ドラッグ**: 各TCPに置いた球が目標そのものです。掴むとカメラに正対する平面上を追従し（残り1軸は視点を回すか、
  Shiftを押しながら掴んで鉛直移動）、ドラッグ中は逐次IK、離した時点でフル反復でもう一度解きます。姿勢（roll/pitch/yaw）は
  変更せず位置のみ動かします。ポインタイベントで実装しているのでタップ＆ホールドでも同じで、球以外を掴んだときは
  従来どおりカメラが回ります（OrbitControls より先に capture フェーズで判定しているため、掴んだ瞬間に視点が回ることはありません）。
- **目標値のスライダ**: xyz の範囲は「肩から腕を伸ばしきった長さ」（サーバが `ik.workspace` として返す上界）で、
  到達可能性の主張ではありません。ドラッグ中は反復150回・トゥイーンなしで解き、離した時点でフル反復でもう一度解きます。
  リクエストはFKと同じく合流（coalesce）するので、速くドラッグしても古い解が溜まりません。
- 実モデルの視覚メッシュ（53 MB / 137個）があればサーバから配信し、初回ロードに数秒かかります。なければ凸包（2.9 MB）を描画します。
- メッシュは非同期にダウンロードされ、FK の応答より後に届いたものにも最新の姿勢が適用されます（以前は遅く届いたパーツが原点に置かれたままになり、回線が遅いとロボットがバラバラに見えました）。ブラウザ実機の回帰テストは
  `uv run --extra kinematics --with playwright pytest tests/test_rakuda_control/test_viewer_browser.py`（Playwright は任意依存）。
- TCPは `gripper_*_dof` からのオフセット**ゼロ**で置かれ、Infoタブにその旨の警告が出ます（実測が必要）。
- continuous関節にソフト制限を与えない場合、スライダ範囲は表示用の ±π、IKは「幾何学的検討のみ」と表示。
- **姿勢モード（orientation）**: Rakudaの手首は2軸（yaw・pitch）で球面手首ではないため、
  「姿勢を完全に保ったまま平行移動」は6自由度あっても一般に到達不能です（実モデルで確認:
  垂れた腕から +30 mm の平行移動は大域探索でも最良 7.4 mm 残る）。この場合ソルバは重み付きの妥協点で
  止まり、APIは `stalled: true` を返し、ページは「これ以上変わらない」と明示します。
  `free`（姿勢重み0）にすると位置だけを追うので、xArmのCartesianジョグに相当する使い方ができます。
  `soft`（0.15、既定）は位置優先、`keep`（1.0）は両方同等です。
- **初期姿勢の注意**: 全関節0の姿勢では腕が伸びきって垂れており、手先は最大到達距離の約96%（実モデルで計測）
  にあります。高さを保った平行移動の多くは作業空間の外なので、Joints タブで肘を曲げてから
  Cartesian ジョグしてください（UFactory Studio の home 姿勢に相当）。肘は片方向にしか曲がりません —
  この URDF では `elbow_pitch_left_dof` は正（0〜+2.31 rad）、`elbow_pitch_right_dof` は負（−2.79〜0 rad）。
  例えば左 +0.8 / 右 −0.8 rad に曲げた姿勢からは、実モデルで ±x/±y/±z の 30 mm ジョグが
  soft/free とも 9〜34 反復・1 mm 未満で収束します（計測済み）。到達できない場合、ページは
  「限界に座っている関節」か「作業空間の外」かを区別して説明します。
- `/api/fk`, `/api/ik`, `/api/model` はJSONのSI単位APIです。DORA等の外部ノードから叩くこともできます。

実機の状態をこのページに**ミラー表示**する機能は未実装です（サーボループのスナップショットを
`/api/fk` 相当の入力にすれば実現できますが、初回では対象外）。

## :material-virtual-reality: VR テレオペ（Meta Quest） {: #vr }

ヘッドセットの向きで `head_yaw` / `head_pitch` を、左右のコントローラで左右の腕を操作し、頭部カメラの画像を
ヘッドセット内に投影します。ヘッドセット側にアプリは不要で、Quest のブラウザで WebXR ページを開くだけです。
モジュールは `robopy.vr`、コマンドは `robopy-vr` です（API は [VR テレオペ API](../api/vr.md)）。

```bash
# 実機なし（モデルとソルバのみ、カメラはテストパターン）
uv run --extra kinematics robopy-vr --host 0.0.0.0 --cert cert.pem --key key.pem

# 頭部カメラを OpenCV デバイスから配信
uv run --extra kinematics robopy-vr --host 0.0.0.0 --cert cert.pem --key key.pem --camera opencv:0

# 実機（.robopy/rakuda/config.yaml の control.mode: cartesian_teleop が必要。ロボットが動きます）
uv run --extra kinematics robopy-vr --host 0.0.0.0 --cert cert.pem --key key.pem --config --hardware

# 実機の頭だけ（腕は動かさない）＋頭部 RealSense の投影。制御系も校正も不要
uv run --extra kinematics robopy-vr --hardware-head --follower-port /dev/ttyUSB1 --camera realsense \
    --host 0.0.0.0 --self-signed
```

Quest のブラウザで `https://<PCのIP>:8766/vr` を開き、**Enter VR** を押します。

### 実機の頭だけを動かし、頭部 RealSense を投影する（`--hardware-head`） {: #vr-head-only }

腕は一切動かさず、ヘッドセットの向きで `head_yaw` / `head_pitch` の 2 モータだけを動かし、頭部の
RealSense の映像をヘッドセットに投影するデモです。`--hardware` と違って制御系（双腕IK・リーダ・校正済み
joint map）を起動せず、フォロワのバスに直接、モータの生カウントで書きます。校正値（`zero_count` 等）は
不要で、**起動時の頭の姿勢を「正面」**とし、ヘッドセットでリセンターした向きに対応させます。

```bash
uv sync --extra kinematics --extra realsense     # pyrealsense2 を入れる（一度だけ）
uv run --frozen --extra kinematics robopy-vr --hardware-head --follower-port /dev/ttyUSB1 \
    --camera realsense --host 0.0.0.0 --self-signed
```

- 起動時に頭の 2 モータの現在値を読み、それを基準に `--head-range`（既定 yaw ±60°、pitch ±35°）の範囲で
  動かします。頭以外のモータには何も書きません。フォロワの他の関節のトルクは `.robopy/rakuda/config.yaml` の
  `follower.torque_enabled` に従います（既定は全関節ON＝腕はその場で保持。`[head_yaw, head_pitch]` にすると
  腕は脱力）。終了時は従来どおりフォロワ全体のトルクを切ります。
- **軸の向き**: 既定 `--head-signs 1,-1` は研究室の Rakuda で測った値です（頭の 2 モータとも URDF の軸と
  逆向きに回る）。別の機体で逆に回る場合は `--head-signs -1,1` のように yaw, pitch の符号を指定してください。
  `--head-signs auto` にすると URDF から導いた符号に config の `direction` を掛けた値になります。
- **カメラ**: `--camera realsense`（複数台なら `realsense:1`）で色ストリームを配信します。`--camera-size`
  と `--camera-fps` で解像度とレートを、`--camera-fov` で投影サイズを変えられます。D435 の色カメラの
  水平画角 69° が既定です。研究室の Rakuda は RealSense が上下逆に付いているので `--camera-rotate` の既定は
  180 です（正立なら 0、90 / 270 も可）。回転は配信・録画の両方に効きます。
- **録画**: 実カメラ（`realsense` / `opencv:`）を使っているときは、録画中にそのカメラの画像を
  `*_first_person.mp4` に直接書き、描画時は MuJoCo の 1 人称ではなくそれを 1 人称動画として扱います。
  3 人称は従来どおり MuJoCo で描きます。テストパターン（`synthetic`）のときは従来どおり MuJoCo の 1 人称です。
- **配信が止まったとき**: RealSense からフレームが 5 秒来なければパイプラインを再起動し、カメラ用の
  WebSocket が切れればページが自動で再接続します（再接続中は画像が灰色になります）。status 欄の
  `camera ... last N s ago` で最終フレームからの経過時間が見えます。
- ページのツイン／ミラーは、頭だけがモータの読み値に従って動き、腕は起動姿勢のまま描かれます。録画も
  同様に使えます。

API から同じ構成を組む例が `examples/robot/rakuda_vr_head_camera.py` です。引数なしでは模擬バスと
テストパターンで動くので、実機なしでも配線を確認できます（`--follower-port` で実機、`--serve` でページを
配信）。

```bash
uv run --frozen --extra kinematics python examples/robot/rakuda_vr_head_camera.py
```

### セキュアコンテキスト（HTTPS）

WebXR は https か localhost でしか動きません。方法は 2 つあります。

1. 自己署名証明書で HTTPS 配信。Quest のブラウザで一度警告を受け入れます。
   `--self-signed` を付けると `robopy-vr` がカレントディレクトリに `cert.pem` / `key.pem` を作って
   （既にあればそれを使って）配信します。
   ```bash
   uv run --extra kinematics robopy-vr --host 0.0.0.0 --self-signed
   ```
   自分で作る場合は `--cert/--key` で渡します。
   ```bash
   openssl req -x509 -newkey rsa:2048 -nodes -keyout key.pem -out cert.pem -days 365 -subj "/CN=robopy"
   ```
   `openssl` が `Can't open "/usr/local/ssl/openssl.cnf"` と言って失敗する（コンパイル時のプレフィックスが
   実機に無い）環境では `OPENSSL_CONF=/etc/ssl/openssl.cnf` を付けて実行します。`--self-signed` はこの
   回避を自動で行います。`--cert/--key` のファイルが無ければ、モデルを読む前に対処法付きで停止します。
2. `adb reverse tcp:8766 tcp:8766` で Quest の `localhost:8766` を PC に転送し、`http://localhost:8766/vr` を開く
   （ブラウザは localhost をセキュア扱いします。TLS 不要）。

### 操作

| 入力 | 動作 |
| --- | --- |
| ヘッドセットの向き | 頭部の yaw / pitch（roll は 2 軸首では再現できないので無視） |
| A / X（親指） | **クラッチ**。押している間だけ、そのコントローラが同じ側の腕を駆動する（`--clutch grip` でグリップ、`--clutch stick` でスティック押し込みに変更可） |
| トリガ | グリッパ（`--gripper SIDE=MOTOR:OPEN,CLOSED` で開閉角を与えたときのみ） |
| 両スティック同時クリック | リセンター（いま向いている方向をロボットの正面 +X にし、頭の位置をロボットの頭に対応させる） |
| B / Y | **録画の開始／停止**（下記） |
| ページのチェックボックス | ツイン表示、ミラー表示（距離も指定可）、カメラ画像の頭追従 |

腕は既定で**絶対**マッピングです（`--mapping absolute`）。リセンター時のヘッドセット位置をロボットの頭
（`--arm-anchor`、既定は `head_camera_link`）に対応させ、A / X を押している間は「頭から見たコントローラの
位置」をそのまま「ロボットの頭から見た手先目標」にします。押した瞬間から手先はコントローラの位置へ、
スルーリミット（`--max-hand-speed`）の速さで寄っていき、離すとその場で保持されます。片側だけ押している
間は、押していない側の腕の関節は固定され、ソルバは押している側の腕だけを解きます（胴体 yaw は
`--torso optimize` を与えない限り固定）。`--position-scale` は頭を中心とした倍率です。
手先の**向き**は押した時点からのコントローラの回転を相対的に加えます（グリップの軸と 2 軸手首の TCP に
自然な対応がないため）。`--no-orientation` で切れます。Rakuda の手首は 2 軸なので、向きを保った並進は
届かないことが多く、その場合ソルバは重み付きの妥協解に落ちます（Info 表示の残差を見てください）。

`--mapping relative` にすると、押した瞬間の手先を基準にコントローラの移動量だけを加える方式になります
（押しても動かず、離して腕を戻し、また押して続ける操作）。

ページのツイン表示は、既定ではロボットの頭がリセンター時の自分の頭の位置に来るように置かれるので、
腕を押して動かすとツインの手が自分のコントローラの位置に来るのが見えます。従来どおり前方に置くには
`--twin-offset 0,0,1` のように指定します。自分がロボットの中に立つ配置では機体そのものは見えないので、
ページは既定で**ミラー**も描きます。ロボットの頭の前方 1.5 m（`mirror` の距離欄で変更可）の鉛直面で
ツインを鏡映したもので、鏡を見るのと同じく自分の左手側にロボットの左腕が映ります。

### 録画と動画の保存

操作中にどう動いていたかを後から確かめるために、サーバがセッションを記録して動画にします。
ページの **● Record** ボタン、または VR 中は **B / Y** で開始し、もう一度押すと停止します。停止すると
`--record-dir`（既定 `./recordings`）に JSON のログ（関節角、コントローラの位置（ロボット座標系）、
クラッチ状態、手先目標、IK 残差、毎ステップ）が書かれ、続けて MuJoCo で 2 本の MP4 が描画されます。

- `*_third_person.mp4`: 正面やや上からの 3 人称視点。コントローラの位置を球で描き（押している側は緑、
  離している側は灰色）、押している手先から目標への線を引きます。
- `*_first_person.mp4`: 1 人称視点。実カメラを配信しているときはそのカメラの画像そのもの（録画中に書かれる）、
  シミュレーションではロボットの頭部カメラ（`head_camera_link`、光軸は頭の中立姿勢から導出）の描画。

進捗と保存先はページの status 欄／VR 内の HUD に出ます。描画には MuJoCo が必要です（無ければログだけが
残り、後から描画できます）。動画は `ffmpeg`（libx264 付き。Ubuntu なら `apt install ffmpeg`）があれば
H.264 で書かれ、ブラウザや VS Code でも再生できます。無い場合は OpenCV の MPEG-4 part 2（`mp4v`）になり、
VLC 以外の多くのプレイヤーで再生できないので、その旨を警告します。

```bash
uv pip install mujoco                                     # 描画に必要（一度だけ）
uv run --frozen --extra kinematics robopy-vr --host 0.0.0.0 --self-signed   # 録画は既定で有効
uv run --frozen --extra kinematics robopy-vr-render recordings/rakuda-vr-*.json   # 後から／再描画
```

`robopy-vr-render` は `--view third_person|first_person`、`--fps`、`--size WxH`、`--azimuth/--elevation/--distance`
（3 人称カメラ）、`--fovy`（1 人称の縦画角）を受け付けます。`robopy-vr --no-render` で描画を後回しに、
`--no-record` で録画機能自体を切れます。操作者の接続が切れた時点で録画中なら、その時点までを保存します。

### モデルから導出するもの（推定しないもの）

- **頭部関節の符号**: URDF の軸方向から求めます。実機エクスポートでは `head_yaw_dof` の軸はワールド −Z で、
  正の角度で**右**を向きます（ヘッドセットの左回りが負の関節角）。`head_pitch_dof` は正で上向きです。
- **胴体 yaw の補償**: 頭は胴体の上に載っており、胴体 yaw は腕 IK の変数です。操作者の頭の向きは
  **ベース座標系**での向きとして扱い、頭部 yaw の指令は `信号 + k × (torso_yaw − 基準)` で補償します。
  `k` は「胴体 1 rad あたりカメラ方位を保つのに必要な頭部 yaw 変化」をモデルの FK から有限差分で求めた値で、
  実機エクスポートでは両軸が同じ向き（ワールド −Z）なので `k = −1`、すなわち **指令 = −torso_yaw + 信号** です。
  `--no-torso-compensation` で切れます。
- **正面を向く中立姿勢**: 実機エクスポートの URDF ゼロは頭が右に約 22° 回っており、カメラは約 6° 上を
  向いています。`HeadJointMapping.from_model()` が `head_camera_link` の光軸（自動判定で z 軸）が
  ベース +X を水平に向く関節角（yaw −0.381 rad、pitch −0.104 rad）を解き、これをヘッドセットの
  リセンター姿勢に対応させます。
- **グリッパの開閉角**: URDF ではグリッパ関節は fixed で、開閉角は測定値です。与えなければトリガは何もしません。
- **カメラの画角**: `--camera-fov` の既定 69° は D435 カラーの公称値で、このカメラの校正値ではありません。

### 安全側の設計

- 操作者の WebSocket が切れる／無応答になると、両クラッチを解放し腕をホールドします（`teleop_timeout_s`、既定 5 s）。
  姿勢はVR中かデスクトッププレビュー中しか流れないので、ページはそれ以外の間（Enter VR 前など）1 秒ごとに
  `ping` を送って接続を保ち、切れた場合は自動で再接続します。
- 目標には有効期限（`--target-ttl`、既定 0.25 s）があり、ストリームが止まれば新しい動作は出ません。
- 同時に操作できるのは 1 人だけです。2 本目の接続は拒否されます。
- コントローラのトラッキングが外れた瞬間にクラッチを解放します（測っていない姿勢で動き続けない）。
- 実機モードでは `RakudaControlSystem` の Cartesian モードを使い、頭・グリッパは `set_direct_targets()`
  で IK の対象外関節としてのみ指令します（IK が扱う関節に直接指令すると `ValueError`）。

### 実機なしで確認する

```bash
uv run --extra kinematics python examples/robot/rakuda_vr_teleop.py      # スクリプト操作者
uv run --extra kinematics robopy-vr --open-browser                          # デスクトップでプレビュー
```

スクリプト操作者は、X を押したままコントローラを頭の前 20 cm・左 15 cm・下 30 cm に置き、左手先が
ロボットの頭から同じオフセットの位置へ寄っていくのを表示します。
ページの **Desktop preview** はウィンドウのカメラの向きを頭部姿勢として送るので、ヘッドセットなしで
頭部追従とツイン描画を確認できます。シミュレーションの初期姿勢は肘を 0.8 rad 曲げた姿勢です
（実機エクスポートのゼロ姿勢は腕が伸び切り右肘が可動域端にあるため、そこからはソルバの一歩も取れません。
`--start-pose JOINT=RAD` で変更可）。

## :material-ruler: 単位と校正

### 電流定数の表記

`RAKUDA_CONTROLTABLE_VALUES` の電流値（`FOLLOWER_GRIP_GOAL_CURRENT = 128` など）は
**raw制御テーブル値**です。以前のコメントは "mA" と書かれていましたが、コードは一貫して
rawを送っていました。**挙動は維持**してあり、128 は今も raw 128（フォロワXM430で約0.34 A）です。
アンペアが必要な場合は `RAKUDA_CONTROLTABLE_VALUES.raw_current_to_a(128, "xm430-w350")` を使います。

| 型番 | 電流1 count | 電流測定位置 |
| --- | --- | --- |
| XM430-W350 | 0.00269 A | モータ巻線 |
| XM540-W270 | 0.00269 A | モータ巻線 |
| XC330-T288 | 0.001 A | 電源入力側（XMと同じトルク観測モデルは流用不可） |

### 変換境界

raw値とSI単位の変換は [`robopy.control.joint_mapping`](../api/robots.md) の1箇所だけで行います。

- 位置: `q = direction * (count - zero_count) * 2*pi/4096`
- 速度: `v = direction * raw * 0.229 * 2*pi/60`（位置オフセットは加えない）
- 電流: `i = direction * raw * (型番ごとの単位)`

multi-turn位置は `[-pi, pi]` へ折り返しません。`zero_count` は
**モータ内の Drive Mode / Homing Offset 適用後の `PRESENT_POSITION` 読出し値**に対して定義し、
その2つのレジスタ値も校正と一緒に記録します。

## :material-alert: 実機で必要な未確定値

以下は**測定値であり、推測で埋めてはいけません**。未測定の項目は `null` のままにしてください。
`JointMap.require()` が不足項目を列挙して停止します。

| 項目 | 影響 |
| --- | --- |
| モータ名 ↔ URDF関節名の対応 | 名前から推定しない（`*_sh_pitch2`, `*_el_yaw`, `*_wr_roll` 等） |
| `zero_count` / `direction` / 可動範囲 | 位置指令すべて |
| `torque_constant_nm_per_a` / `current_limit_a` | 電流出力（バイラテラル） |
| TCP（`gripper_*_dof` からのオフセット） | Cartesian精度 |
| continuous関節の実可動域（胴体yaw・両肩pitch） | URDFに範囲がない。ソフト制限必須 |
| 重力補償モデル（リーダ・フォロワ別） | 電流制御時の自重落下 |

`control.allow_hardware_current_output` は既定で `false` です。上記が測定され
`validated: true` になるまで、バイラテラルモードは `configure()` で拒否されます。

### モデルの配置（`robopy/models/rakuda/`、パッケージ同梱）

robopy はライブラリなので、動作に必要なものは `pip install robopy` で入るパッケージの中に入っています。
`Rakuda-2_simulation_ready.zip` の展開レイアウトをそのまま `robopy/models/rakuda/assembly_2/` に
パッケージデータとして同梱し、`robopy.models.find_rakuda_model()` が場所を返します（`git clone` は不要です）。

| 内容 | wheel | 用途 |
| --- | --- | --- |
| `urdf/*.urdf`（3種）、`collision_meshes/`（凸包137個、2.9 MB） | **同梱** | 運動学・IK・衝突判定・凸包表示。**pip install だけで動く** |
| `meshes/`（視覚メッシュ137個、53 MB） | 含まない（リポジトリでは Git LFS） | ビューア／VR の見た目のみ。`robopy-models fetch` で取得 |

```bash
robopy-models status     # モデルの場所と視覚メッシュの有無
robopy-models fetch      # 視覚メッシュをキャッシュ（~/.cache/robopy/models/rakuda/）に取得
```

`fetch` は GitHub Release `rakuda-visual-meshes-v1` に添付された `rakuda_visual_meshes.zip`（137 個の STL と
SHA-256 の `MANIFEST.json`）を 1 リクエストで取得し、各ファイルが STL であること（LFS ポインタやエラーページ
ではないこと）とチェックサム、URDF が参照する 137 個が揃っていることを確認してキャッシュに展開します。
Git LFS の帯域クォータは消費しません。取得先は `ROBOPY_CACHE_DIR`（既定 `$XDG_CACHE_HOME/robopy`）で
変えられ、`--tag` / `--url`（ミラーや研究室のファイルサーバ、`file://` も可）/ `--token`（非公開リポジトリは
API 経由で解決）を指定できます。チェックアウトで開発している場合は `git lfs install && git lfs pull` でも
同じ状態になります。

アセットの作り方（CAD を再エクスポートしたときだけ）: LFS を pull したチェックアウトで
`uv run python scripts/build_visual_mesh_asset.py` が zip とマニフェストを作ります。タグ
`rakuda-visual-meshes-v*` を push すると GitHub Actions（`release-visual-meshes`）が同じものを Release に
添付します。既定タグは `robopy.models.RAKUDA_VISUAL_MESH_RELEASE_TAG` で、URDF を更新したときに合わせて上げます。

`find_rakuda_model()` は視覚メッシュの状態を `visual_mesh_status` で
`PRESENT`（実体あり）/ `LFS_POINTERS`（チェックアウトで `git lfs pull` 前）/ `ABSENT`（wheel で未取得）と
区別します。どちらの不在でもビューアは同じ URDF の `<collision>`（同梱の凸包）を描画し、Info タブと起動ログに
理由と対処が出ます（`--geometry visual|collision|auto` で明示もできます）。
注意: `assembly_2_convex_collision.urdf` は `<visual>` に元の視覚メッシュ、`<collision>` に凸包を
持つので、「凸包 URDF を読めば凸包が表示される」わけではありません。
コードからは次のように参照します。

```python
from robopy.models import find_rakuda_model, fetch_visual_meshes

m = find_rakuda_model()            # None なら models データが見つからない（通常はあり得ない）
m.convex_collision_urdf, m.package_dirs, m.visual_mesh_status
m.visual_mesh_hint()               # 不在時の対処を 1 行で返す（PRESENT なら None）
fetch_visual_meshes()              # 視覚メッシュをキャッシュへ（FetchReport を返す）
```

`package://assembly_2/...` の解決には `m.package_dirs`（キャッシュ → 同梱ディレクトリの順）を渡してください。
探索順は環境変数 `ROBOPY_MODELS_DIR` → 引数 → パッケージデータで、別のモデルディレクトリを使いたいときだけ
`ROBOPY_MODELS_DIR` を指定します。`.robopy/rakuda/config.yaml` の `control.model.urdf_path` を `null` に
しておけば、制御系（Cartesian モード）も同梱モデルを使います。

### 実モデル（`Rakuda-2_simulation_ready.zip`）の監査結果

`assembly_2/urdf/assembly_2_convex_collision.urdf` を `python -m robopy.kinematics.urdf_audit` で
監査した結果です（生データ: `docs/robots/assets/rakuda_urdf_audit.json`）。

| 項目 | 結果 |
| --- | --- |
| ルートリンク | `root` |
| リンク／関節 | 153 / 152（fixed=137, revolute=12, continuous=3） |
| 可動自由度 | 15 = 胴体1 + 左腕6 + 右腕6 + 頭部2 |
| メッシュ | 274参照。`--package-dir <展開先>` で全て解決（視覚メッシュ未追加のチェックアウトでは `meshes/` の137参照が未解決になる） |
| 総質量 | **0.0020693 kg → 幾何専用。動力学モデルとして採用不可** |
| `effort` / `velocity` | revolute 12関節すべて `1 / 1` のプレースホルダ |
| continuous関節 | `torso_yaw_dof`, `shoulder_pitch_left_dof`, `shoulder_pitch_right_dof`（範囲なし） |
| 同名の関節とリンク | `gripper_left_dof`, `gripper_right_dof`, `head_camera_link` |
| ゼロ姿勢が可動限界上 | `elbow_pitch_right_dof` の範囲は `(-2.7925, 0.0000)`。**全関節0はこの関節の上限そのもの** |

構造の確認（Jacobianから）: `torso_yaw_dof` は両手に効き、各腕の6関節は自分の手だけに効き、
頭部2関節はどちらの手にも効きません。双腕IKの前提と一致します。

実モデルで確認できたこと（合成モデルではなく本体）:

- 双腕IKが FK生成の到達可能目標へ収束: **位置 0.80 / 0.85 mm、姿勢 < 0.01 rad、1回 0.7〜0.9 ms**、
  `fixed` / `manual` / `optimize` の3方針すべて。頭部への指令なし。
- `examples/robot/rakuda_cartesian_teleop.py --urdf <実URDF>` がそのまま動作（0.06 / 0.10 mm）。

実モデルで見つかり修正した不具合:

- 上記の同名フレーム: Pinocchio が `FIXED_JOINT` と `BODY` の2フレームを同名で持ち、名前解決が例外を出す。
  `WholeBodyModel.frame_id()` が両者の一致を確認して解決するようにし、TCPは必ず一意名の追加フレームで
  定義する（監査でも警告）。
- 可動限界上のゼロ姿勢: 限界に向かう関節で**加速度制限と位置制限が矛盾し「減速して止まる」解が消える**
  不具合。加速度窓を位置窓へクリップする形に修正（安全側の制限が常に優先）。

**衝突判定の計算コスト（実測）**: 凸包274個の全ペア（7472組）で距離計算に **約2.3秒**。
腕同士＋腕と胴体に絞った1494組でも **約0.4秒**、経路検証を含むIK 1回で **約1.2秒**。
このCAD出力の粒度（部品ごとの凸包）は制御周期には乗りません。選択肢は
(a) リンクごとに数個のプリミティブへ縮約した衝突モデルを別途用意する（モデリング作業）、
(b) 衝突判定を低周期の事後検証に限定する、のどちらかで、ソフトウェア側で誤魔化せる量ではありません。
中立姿勢で20 mm以内のペアは、ベアリング・ワッシャ・カバー等の恒久的な機構的近接（全ペアで36組）です。
`WholeBodyModel.pairs_closer_than()` で列挙し、理由つきで除外に記録してください。

設定の雛形: `examples/config/rakuda_control.example.yaml`（`urdf_path` は同梱モデルを指し、未測定値はすべて `null`）。

### モデルの監査

URDFは使う前に監査します（kinematics extra は不要）。

```bash
python -m robopy.kinematics.urdf_audit path/to/robot.urdf --package-dir path/to/pkgs
```

総質量が非現実的（CAD出力では数mgになることがあります）、`effort=1 velocity=1` の
プレースホルダ、`*_dof` という名前の固定関節、範囲のないcontinuous関節などを警告します。
**質量を書き換えて「動力学検証済み」にしないでください。** 幾何専用モデルとして扱います。

## :material-stop-circle: 停止方針と異常時

`control.stop_policy` は機構と支持条件に応じて選びます。万能な既定値はありません。

| 方針 | 内容 | 前提 |
| --- | --- | --- |
| `zero_current` | 電流ゼロ、トルクはON | バックドライバブルで支持されている、または水平 |
| `torque_off` | トルクOFF | 落下するものがない |
| `hold_position` | 測定姿勢で位置制御に切替えて保持 | その姿勢を自力で保持できる |

- 片側の読取り／書込み失敗、状態の古さ、周期超過、温度・電圧異常、範囲超過、不正数値で
  両側を `FAULT` にします。`FAULT` からの復帰は `CONFIGURING` 経由のみで、再検証が必須です。
- 通信不能側には書き込めません。その場合はモータ側の `BUS_WATCHDOG`
  （`control.bus_watchdog_counts`、20 ms/count）に委ねます。
- Bus Watchdog は**全Instruction Packet**が対象なので、読取りを続けている限り
  Goal更新を止めただけでは発火しないことがあります。読取りだけ生きている状況も別途検出します。

## :material-chart-line: 周期の報告

設定周期と実測周期は別に表示されます。

```python
report = system.report()
report["timing"]["configured_rate_hz"]        # 設定値
report["timing"]["measured_period_s"]         # p50/p95/p99/max、期限超過数
report["timing"]["servos"]["follower"]["read"]
```

数百Hzでの動作は、この実測値が出るまで保証されません。
