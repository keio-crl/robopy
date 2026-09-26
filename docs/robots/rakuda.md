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
| `cartesian_teleop` | 左右TCP目標からフォロワ関節目標を生成する（目標へは飛ばず、`control.trajectory` の上限で動く参照を追従） | 双腕IK | 位置制御(3) |
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

`cartesian_teleop` では、操作者の目標（`set_target()`）は**そのままIKへ渡されません**。駆動する手ごとに
**実測TCPから始まる参照姿勢**を作り、`control.trajectory` の速度・加速度上限のもとで目標へ進め、
その参照を1周期ずつ解きます。実測の手が参照から `lag_tolerance_m` 以上遅れると参照は待ちます（目標を
手の位置で上書きして誤差を隠すことはしません）。上限が1つでも未設定なら `cartesian_teleop` は
起動を拒否します（ビューアの模擬プロファイルにフォールバックしません）。参照の状態は
`system.last_reference` / `system.report()["reference"]` で読めます。

`start_control()` 実行中は両バスの指令権を制御システムが保持し、
`control_step()` / `send_follower_action()` などの旧経路は明示的に拒否されます
（1ポート1書き手）。観測API (`get_observation()`) は配列の形・順序・単位（degree）を
変えずに、サーボのキャッシュを読みます。SI単位の詳細状態は `pair.detailed_state()` で取れます。

## :material-cube-scan: 3Dビューア／シミュレータ（実機不要）

![robopy viewer: 実モデル、End effector タブ](assets/rakuda_viewer.png)

UFactory Studio の「モデルだけを動かして確認する」に相当するブラウザUIです。
関節角度（スライダ／数値／±ジョグ）またはエンドエフェクタ姿勢（xyz・roll/pitch/yaw のスライダ／数値／±ジョグ）で
モデルを動かし、3D表示で確認できます。3D表示上のTCPの球を**直接ドラッグ**（クリック、またはタップ＆ホールド）しても動かせます。
手先の移動は**時刻付きの軌道**として計算され、その時間で再生されます（最終姿勢へ飛びません）。
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

# .robopy/rakuda/config.yaml の control.model（URDF・上書き・ソフト制限・TCP・関節グループ・ホーム姿勢）、
# control.ik（ソルバ設定）、control.trajectory（参照軌道の上限）を使う。urdf_path: null なら同梱モデル
uv run robopy-viewer --config
```

起動時に**出所ログ**を出します: コードのリビジョン、モデルファイルとそのSHA-256、関節ごとの範囲の出典
（URDF / 上書きとその理由 / ソフト制限と検証済みか / 表示専用）、TCPの検証状態、ホーム姿勢、IKの関節グループと設定、
軌道プロファイル、自己衝突の評価有無。実機での挙動をどの設定に遡れるかを残すためのものです。

**関節範囲は1か所で解決され、スライダ・ソルバ・実機アダプタが同じ値を読みます**
（`robopy.kinematics.joint_limits.resolve_joint_limits`）。優先順位は
URDF → `joint_limit_overrides_rad`（URDFの範囲を置き換える。理由の記載が必須）→ ソフト制限
（**狭めることしかできません**。`validated: true` で実機で確認済みと宣言、それ以外は暫定）です。
continuous 関節（胴体yaw・両肩pitch）はURDFに範囲が無いので、同梱モデルではサーボの可動範囲 ±180° を
**暫定・シミュレーション専用**の範囲として与え、その旨をログとツールチップに出します。実際に止まる位置は計測値なので、
`.robopy/rakuda/config.yaml`（`--config`）か `--soft-limit` で与えてください（与えればスライダも狭まります）。
範囲外へスライダで動かしてから move すると、実機と同じく「limits を既に逸脱している」と言い、
**限界上の関節を無理に動かすことはしません**（境界上では境界から離れる向きの運動だけを許し、逸脱は関節名と量で報告）。
URDFの範囲外へ出た continuous 関節の角度は前回値に連続になるよう unwrap されるので、±π で跳ぶことはありません。

`http://127.0.0.1:8765` を開きます（`--port`, `--host`, `--no-browser`, `--no-ik` あり）。

| タブ | 内容 |
| --- | --- |
| Joints | 胴体／左腕／右腕／頭部ごとのスライダ・数値入力・±ジョグ（deg/rad切替、ステップ幅）。範囲は解決済みの関節範囲（出典と検証状態をツールチップに表示）。`zero all`、`home`（`home_positions_rad` が設定され、範囲・特異性の検査を通ったとき）、`copy JSON`（rad） |
| End effector | **arm: left / right / both**（起動時は片腕）、胴体方針 fixed/manual/optimize、非操作腕の follow torso / hold TCP in world、**orientation: position_only / pose / axis_aligned** と重み、左右TCPの現在姿勢（mm / deg）と目標。3Dビュー上の球を直接ドラッグ、各成分のスライダ・数値入力・±ジョグ、`capture`、`move`／ジョグごとに自動move、`reset session`。セッション行（駆動している手・方針・モード・衝突評価の有無・軌道プロファイル）と、状態語彙で書かれたステータス |
| Info | URDFパス、nq/nv、関節範囲の出典と上書きの理由、ホーム姿勢・TCPの検証状態、IKの関節グループ・優先モード・姿勢モード・接近軸、衝突評価の有無、軌道プロファイル、ソルバ設定、モデル監査の警告 |

操作対象は **arm** セレクタで `left` / `right` / `both` と明示します（起動時は片腕）。UIの表示と
送信する目標の `enabled` は常に一致し、セッション行に「driving: left / idle: right」のように出ます。
既定の `follow torso (keep joints)` では非操作腕の6関節にIKによる運動を指令せず、TCPのワールド座標は
共有胴体と一緒に変化します。非操作腕も関節範囲と（登録されていれば）衝突判定の対象のままです。
腕を切り替えると、新たに駆動する手の目標をその時点のTCP姿勢で取り直し、セッションをリセットします。

### 時刻付き軌道と再生

`move`（ジョグ・ドラッグ・スライダも同じ）は `/api/ik` の `mode: "trajectory"` を呼びます。サーバは
手先の**参照姿勢**を現在のTCPから目標へ、`control.trajectory`（未設定ならビューアの模擬プロファイル:
0.25 m/s・1 m/s²・1.5 rad/s・6 rad/s²、20 ms）の上限で進め — 静止からの1区間は直線＋回転の測地線
（Euler角の成分差は使いません）— 各サンプルで微分IKを1ステップ解いて、`time_from_start_s`・関節角・TCP・
参照・目標誤差・active limits を時刻付きで返します。ページは**その時刻どおり**に再生します
（350 ms のトゥイーンも、ドラッグ時の最終姿勢への瞬間移動もありません）。描画周期と制御周期は別で、
再生クロックはサンプル周期のタイマ、描画は間に合う分だけです。隣り合うサンプル間は関節角を線形補間し
（回転関節では `integrate` に一致）、補間で表示した姿勢のTCPと計算済みTCPのずれを「display path deviation」
として報告します。

- **再目標化**: 再生中のジョグ・ドラッグは `resume: {seq, t}` を付けて送られ、参照はその時点の姿勢と速度から、
  ソルバはその時点の関節速度から続きます。動きは**曲がる**のであって、止まってやり直しません。
- **順序と取消**: 要求には連番（`seq`）が付き、古い応答は捨てられます。手動で関節を編集した後に届いた応答も捨てます。
- **継続**: 予算（2 s）で切れた軌道は `truncated` で返り、ページが再生の終わる前に続きを取りに行きます。
- **手の遅れ**: 速度上限などで手が参照から `lag_tolerance_m` 以上遅れると、参照は減速・停止して待ちます。
  目標をTCPの位置で上書きすることはありません。
- **セッション状態**: 速度履歴と姿勢参照は操作の間保持されます。リセットは起動時、arm／orientation／torso の
  切り替え、手動の関節編集（編集の後の最初の move で1回、その姿勢に錨を下ろす）、`reset session` に限ります。
  `/api/ik/reset` がその入口で、`/api/model` の `ik.resets` で回数が読めます。
- 3Dビューでは、計算済みTCPの**軌跡**（腕の色）、追従中の**参照**（白い小さな印）、**目標**（球）を描きます（`path` で表示切替）。

### 状態語彙

| status | 意味 |
| --- | --- |
| `tracking` | まだ動いている（予算切れなら `truncated` も true） |
| `converged` | 参照が到着し、手が許容誤差内 |
| `locally_stalled` | 拘束は効いていないのに残差が減らない。局所法がこの姿勢から進めないだけで、目標が到達不能である証明ではない |
| `limits_blocked` | 位置限界が効いた状態で残差が減らない（効いている関節名を併記） |
| `collision_blocked` | 衝突対（安全距離）が効いた状態で残差が減らない |
| `stale` / `solver_error` / `infeasible` / `limit_violation` / `collision_at_start` | 要求が使えない、QPが失敗、この姿勢から拘束を全て満たす一歩が無い、既に範囲外、既に衝突。姿勢は変えない |

衝突対がモデルに登録されていなければ、End effector タブとセッション行に「self-collision NOT evaluated」と
出します。**collision URDF を読み込んだだけでは回避は有効になりません。**

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
- **関節範囲の出典**: `/api/model` は各関節について、解決済みの範囲（`lower`/`upper`）、その出典（`limit_source`:
  `urdf` / `override` / `soft` / `display`）、検証済みか（`validated`）、URDF側の値、上書きの理由、注記を返します。
  スライダ・ソルバ・実機アダプタは同じ解決済み範囲を読みます。
- **接地面**: グリッドは床です。関節で動かないジオメトリ（＝土台）の最下面に置きます。モデル原点ではありません —
  Rakudaのエクスポートでは原点は土台の底から約 26 cm 上にあり、原点に描いていたグリッドは胴体を突き抜けていました。
  土台を描かないモデルでは従来どおり root フレーム（z = 0）に置きます。
- **手先の直接ドラッグ**: 各TCPに置いた球が目標そのものです。掴むとカメラに正対する平面上を追従し（残り1軸は視点を回すか、
  Shiftを押しながら掴んで鉛直移動）、ドラッグ中は短い予算（0.6 s）で再目標化を繰り返し、離した時点でフル予算（2 s）でもう一度
  計画します。姿勢（roll/pitch/yaw）は変更せず位置のみ動かします。ポインタイベントで実装しているのでタップ＆ホールドでも同じで、
  球以外を掴んだときは従来どおりカメラが回ります（OrbitControls より先に capture フェーズで判定しているため、掴んだ瞬間に視点が回ることはありません）。
- **目標値のスライダ**: xyz の範囲は「肩から腕を伸ばしきった長さ」（サーバが `ik.workspace` として返す上界）で、
  到達可能性の主張ではありません。リクエストはFKと同じく合流（coalesce）するので、速くドラッグしても古い解が溜まりません。
- 実モデルの視覚メッシュ（53 MB / 137個）があればサーバから配信し、初回ロードに数秒かかります。なければ凸包（2.9 MB）を描画します。
- メッシュは非同期にダウンロードされ、FK の応答より後に届いたものにも最新の姿勢が適用されます（以前は遅く届いたパーツが原点に置かれたままになり、回線が遅いとロボットがバラバラに見えました）。ブラウザ実機の回帰テストは
  `uv run --extra kinematics --with playwright pytest tests/test_rakuda_control/test_viewer_browser.py`（Playwright は任意依存）。
- TCPは `gripper_*_dof` からのオフセット**ゼロ**で置かれ、Infoタブにその旨の警告が出ます（実測が必要）。
- continuous関節にソフト制限を与えない場合、スライダ範囲は表示用の ±π、IKは「幾何学的検討のみ」と表示。
- **姿勢モード（orientation）**: `position_only`（位置だけ。既定）、`pose`（位置＋完全姿勢、重み付き）、
  `axis_aligned`（位置＋グリッパの接近軸の向き。軸まわりの回転は自由。`control.ik.approach_axis_tcp` で接近軸を
  明示したときだけ選べます — 軸は仮定しません）。`axis_aligned` は S² 上の2自由度タスクとして書かれており、
  Euler角の1成分を0にする実装ではありません（反平行では決定的な逃げ方向を選び、角度誤差は連続）。
  「姿勢を保ったまま平行移動できるか」は手首だけの性質ではなく、腕全体の配置・その点のヤコビアン・関節範囲で決まり、
  姿勢ごとに変わります。できない要求は `locally_stalled` / `limits_blocked` / `collision_blocked` のどれかで終わり、
  何が効いていたかを併記します。
- **タスク優先度**: `control.ik.task_priority_mode` が `hierarchical`（既定）なら手のタスクを第1段で解き、
  姿勢参照・限界回避・平滑化・運動コスト（胴体は腕より高い）を第2段で**手のタスクの零空間**だけで解きます
  （第2段が第1段の残差を増やしたら第1段の解へ戻し、理由を記録）。`weighted` は従来どおり1つの重み付きQPで、
  二次目的が手のタスクと競合します。数値比較は下の表を参照。
- **初期姿勢の注意**: 全関節0の姿勢では腕が伸びきって垂れており、手先は最大到達距離の約96%（実モデルで計測）
  にあります。高さを保った平行移動の多くは作業空間の外なので、Joints タブで肘を曲げてから
  Cartesian ジョグしてください（UFactory Studio の home 姿勢に相当）。肘は片方向にしか曲がりません —
  この URDF では `elbow_pitch_left_dof` は正（0〜+2.31 rad）、`elbow_pitch_right_dof` は負（−2.79〜0 rad）。
  `home_positions_rad` を設定すると `home` ボタンで戻れます（起動時に範囲内・特異性を検査し、
  通らなければ理由付きで拒否）。到達できない場合、ページは状態語彙で何が効いていたかを説明します。
- `/api/fk`, `/api/ik`, `/api/model` はJSONのSI単位APIです。DORA等の外部ノードから叩くこともできます。

### 自然な手先動作の設定（`control.ik` / `control.trajectory`）

ビューアと実機は**同じ** `control.ik` から `DualArmIKConfig` を作ります（ビューアだけの値はサンプル周期と模擬プロファイルのみ）。
`examples/config/rakuda_control.example.yaml` に全項目のコメントがあります。主なもの:

| 項目 | 意味 |
| --- | --- |
| `task_priority_mode` | `hierarchical`（既定）/ `weighted` |
| `orientation_mode`, `approach_axis_tcp` | 姿勢モードと、`axis_aligned` に必要な接近軸（TCP座標） |
| `preferred_posture_rad` | 二次目的が引き戻す姿勢。未設定なら整列時／リセット時の姿勢 |
| `posture_cost`, `joint_motion_cost`, `velocity_smoothing_cost`, `limit_avoidance_*` | 二次目的の重み（胴体の運動コストを腕より高く） |
| `gain_time_constant_s` | 誤差を1ステップで消さず、時定数で追う（加速度上限と両立） |
| `max_joint_velocity_rad_s`, `max_joint_acceleration_rad_s2`, `position_limit_margin_rad` | 関節側の上限（加速度は前回ステップの速度に対して） |
| `singularity_sigma_min`, `singularity_damping` | 正規化した特異値に応じた減衰 |
| `trajectory.*` | 参照軌道の速度・加速度・角速度・角加速度の上限、`lag_tolerance_m`。**実機は全て必須** |

### 数値評価（合成モデル）

`scripts/evaluate_ik_profiles.py` が、同じ開始姿勢・同じ目標を優先モード×姿勢モードで走らせて比較します
（合成モデル、模擬プロファイル。実機の計測ではありません）。抜粋:

| シナリオ | プロファイル | status | 時間 (s) | 目標誤差 | 経路誤差 平均/最大 (mm) | 胴体 (rad) | ms/step |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 左 6 cm 前 | weighted / position_only | converged | 1.00 | 1.0 mm | 5.5 / 14.6 | 0.000 | 1.0 |
| 左 6 cm 前 | hierarchical / position_only | converged | 0.62 | 1.0 mm | 6.8 / 13.9 | 0.001 | 1.3 |
| 左 12 cm 体の前を横切る | weighted / pose | tracking（6 s 予算切れ） | 6.00 | 5.5 mm / 1.8° | 7.3 / 20.4 | 0.004 | 1.0 |
| 左 12 cm 体の前を横切る | hierarchical / pose | converged | 1.56 | 1.0 mm / 0.4° | 8.3 / 20.1 | 0.263 | 1.3 |
| 左 12 cm 体の前を横切る | hierarchical / axis_aligned | converged | 0.88 | 0.9 mm / 軸 0.2°（完全姿勢は 43.5° 自由） | 9.7 / 18.0 | 0.007 | 1.9 |
| 両手 5 cm 上 | hierarchical / pose | converged | 0.72 | 0.9 mm / 0.1°（両手） | 10.1 / 22.5 | 0.000 | 1.5 |
| 左 40 cm 前（到達不能） | weighted / position_only | tracking（6 s 予算切れ） | 6.00 | 296 mm | 19.4 / 21.9 | 0.030 | 1.1 |
| 左 40 cm 前（到達不能） | hierarchical / position_only | locally_stalled | 2.86 | 144 mm | 18.7 / 20.8 | 1.144 | 1.3 |

読み方: 経路誤差は手が参照に対して遅れている距離で、関節速度上限（1 rad/s）と加速度上限（8 rad/s²）に当たると
2 cm（`lag_tolerance_m`）まで遅れ、参照が待ちます。`weighted` は姿勢タスクと二次目的が競合して 12 cm の移動を
5.5 mm 残したまま這い続け、到達不能な目標でも胴体をほとんど使わず 6 s 這い続けます。`hierarchical` は
手のタスクを優先するので胴体を回して届く所まで行き、進まなくなった時点で `locally_stalled` と言います。
計算時間はどのプロファイルも 1〜2 ms/step（合成モデル、衝突評価なし）です。

### 未計測のパラメータ（実機で決めるもの）

コードは合成モデルと同梱URDFで検証しており、以下は**実機の計測値が入るまで暫定**です:

- continuous 関節（`torso_yaw_dof`、両 `shoulder_pitch_*_dof`）の実際の可動範囲（現在はサーボ可動範囲 ±180° の暫定値）
- URDFが宣言する非対称な範囲が実機と一致しない関節の上書き（`joint_limit_overrides_rad`、理由付き）
- TCPオフセット（`gripper_*_dof` から把持中心まで。現在はゼロのプレースホルダ）
- 接近軸 `approach_axis_tcp`（グリッパの実際の向き）
- `home_positions_rad` / `preferred_posture_rad`（実機で確認した姿勢）
- `control.trajectory` の速度・加速度上限（実機は未設定だと Cartesian モードを拒否）
- 関節速度・加速度上限、`gain_time_constant_s`（サーボの応答に合わせる）
- 衝突対の登録（凸包URDFを読むだけでは評価されません）

実機の状態をこのページに**ミラー表示**する機能は未実装です（サーボループのスナップショットを
`/api/fk` 相当の入力にすれば実現できますが、初回では対象外）。

## :material-virtual-reality: VR テレオペ（Meta Quest） {: #vr }

ヘッドセットの向きで `head_yaw` / `head_pitch` を、左右のコントローラ**または素手（ハンドトラッキング）**で
左右の腕を操作し、頭部カメラの画像をヘッドセット内に投影します。ヘッドセット側にアプリは不要で、Quest の
ブラウザで WebXR ページを開くだけです。
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

- 操作者（ヘッドセット）が接続すると、追従を始める前に頭の 2 モータを**リセット位置**へ動かします。
  `--head-home YAW,PITCH`（エンコーダのカウント、既定 `2048,2048` = 可動域の中央）で指定し、到達
  （または 3 秒のタイムアウト）後にその位置を「正面」の基準にします。`--head-home current` にすると
  従来どおり起動時の姿勢を基準にします。バイラテラル時は制御ループがバスを持つ前に同じことを行います。
- 基準位置から `--head-range`（既定 yaw ±60°、pitch ±35°）の範囲で動かします。頭以外のモータには何も書きません。フォロワの他の関節のトルクは `.robopy/rakuda/config.yaml` の
  `follower.torque_enabled` に従います（既定は全関節ON＝腕はその場で保持。`[head_yaw, head_pitch]` にすると
  腕は脱力）。終了時は従来どおりフォロワ全体のトルクを切ります。
- **軸の向き**: 既定 `--head-signs 1,-1` は研究室の Rakuda で測った値です（頭の 2 モータとも URDF の軸と
  逆向きに回る）。別の機体で逆に回る場合は `--head-signs -1,1` のように yaw, pitch の符号を指定してください。
  `--head-signs auto` にすると URDF から導いた符号に config の `direction` を掛けた値になります。
- **カメラ**: `--camera realsense`（複数台なら `realsense:1`）で色ストリームを配信します。`--camera-size`
  と `--camera-fps` で解像度とレートを、`--camera-fov` で投影サイズを変えられます。D435 の色カメラの
  水平画角 69° が既定です。カメラが上下逆や鏡像で取り付けられている場合は `--camera-rotate 90|180|270` と
  `--camera-mirror on` で補正します（既定はどちらも無補正。研究室の Rakuda のカメラは正しく付いています）。
  これらの補正は**カメラから画像を受け取った直後に 1 回だけ**行い、それ以降の
  すべて（配信、ヘッドセット表示、録画の動画、描画）は補正済みの正立画像しか受け取りません。下流のどこにも
  回転の設定はなく、二重に回ることはありません。
- **録画**: 実カメラ（`realsense` / `opencv:`）を使っているときは、録画中にそのカメラの画像を
  `*_first_person.mp4` に直接書き、描画時は MuJoCo の 1 人称ではなくそれを 1 人称動画として扱います。
  3 人称は従来どおり MuJoCo で描きます。テストパターン（`synthetic`）のときは従来どおり MuJoCo の 1 人称です。
- **配信が止まったとき**: RealSense からフレームが 5 秒来なければパイプラインを再起動し、カメラ用の
  WebSocket が切れればページが自動で再接続します（再接続中は画像が灰色になります）。status 欄の
  `camera ... last N s ago` で最終フレームからの経過時間が見えます。
- ページのツイン／ミラーは、頭だけがモータの読み値に従って動き、腕は起動姿勢のまま描かれます。録画も
  同様に使えます。

**マスター（リーダ）も使う**: `--leader-port /dev/ttyUSB0` を足す（または config の `leader_port`）と、
頭の 2 関節はヘッドセット、それ以外の関節（胴体・両腕・グリッパ）はリーダの現在位置をそのままフォロワの
目標にする位置テレオペになります。リーダの頭の読み値は無視します。書き込む関節は `follower.torque_enabled`
に従い、頭はリーダからは決して書きません。リーダのグリッパは既定で**トルク OFF** にします（リーダは手で動かす
マスタ装置なので）。位置テレオペと同じ戻りばね（トルク ON、目標 2400）が欲しい場合は `--leader-grip-hold` を
付けてください。

```bash
uv run --extra kinematics --extra realsense robopy-vr --hardware-head \
    --follower-port /dev/ttyUSB1 --leader-port /dev/ttyUSB0 --camera realsense --host 0.0.0.0 --self-signed
```

ページのツインとミラー、録画ログの関節角は、頭以外は起動姿勢のままです（リーダ追従の腕の角度を URDF に
直すには校正済み joint map が要るため）。実機の動きは頭部カメラの映像と 1 人称動画で確認してください。

バスの読み書きはバックエンド専用のスレッドが固定周期（50 Hz）で行い、1 周期は **リーダ 17 モータ読み → フォロワ
1 回書き**（リーダ由来の 15 関節 + ヘッドセット由来の頭 2 関節）の 2 往復だけです。頭の現在値は起動時の基準姿勢を
1 回読むほかは読み戻さず、書いた目標値をツインとログに使います。ヘッドセットの姿勢メッセージは最新の頭目標を
置くだけです。Linux の USB シリアルは 1 往復 16 ms 前後かかるので、これを受信スレッドで行うと 60 Hz の姿勢
メッセージに追いつけず頭の遅れが増え続けます。1 周期が周期の 2 倍を超えると起動ログに警告が出ます。その場合は
USB シリアルのレイテンシタイマを 1 ms にしてください。

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # ttyUSB1 も同様
```

**バイラテラル**: さらに `--bilateral` を付けると、腕はリーダの位置を写すのではなく
[バイラテラル関節制御](#制御モード双腕ik-バイラテラル)（仮想ばね・ダンパ、電流制御）で結合され、頭はヘッドセットに
追従します。制御系（`RakudaControlSystem`）を `bilateral_joint` モードで起動するので、`.robopy/rakuda/config.yaml` の
`control:` セクション（結合する関節の校正値、`bilateral:` のゲイン、`allow_hardware_current_output: true`）が
そのまま必要です。`control.mode` はこのフラグで `bilateral_joint` に上書きされます。制御系が動いている間はバスの
書き込み権を制御ループが持つため、頭の目標は制御系の `set_direct_goal_counts()` に渡し、制御ループが自分の
周期の中で書きます（別スレッドからバスに触ることはありません）。頭のモータは結合の対象外（位置モードのまま）で、
リーダ側の頭は無視されます。

```bash
uv run --extra kinematics --extra realsense robopy-vr --hardware-head --bilateral \
    --follower-port /dev/ttyUSB1 --leader-port /dev/ttyUSB0 --camera realsense --host 0.0.0.0 --self-signed
```

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
| 素手：親指と人差し指のピンチ | **クラッチ**（コントローラの A / X に相当）。つまんでいる間、その手が同じ側の腕を駆動する（[ハンドトラッキング](#vr-hands)） |
| 素手：中指・薬指・小指を握る | グリッパ（トリガに相当。`--gripper` で開閉角を与えたときのみ） |
| 素手：両手で親指と中指のピンチ | リセンター（両スティック同時クリックに相当）。中断中なら**再開**も兼ねる |
| 素手：片手で親指と中指のピンチを 1 秒保持 | 録画の開始／停止（B / Y に相当） |
| 素手：両手のひらをヘッドセットに向ける（指は伸ばす） | 0.5 秒で**中断**（腕を止めて保持）、2.5 秒保持で**終了**（VR セッションを閉じる） |
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

### ハンドトラッキング（コントローラなしで両腕を操作する） {: #vr-hands }

Quest のハンドトラッキングを有効にしてコントローラを置くと、ブラウザは WebXR Hand Input として両手の
25 関節の姿勢を渡してきます。ページはそれをそのままサーバに送り（関節ごとの位置と、手首の向き）、
**ジェスチャの判定はすべてサーバ側**（`robopy.vr.hand_tracking`）で行います。腕の駆動はコントローラと
同じ `ArmTeleop` を通るので、絶対／相対マッピング、`--position-scale`、スルーリミット、トラッキングが
外れた瞬間の解放、録画はすべて共通です。同じセッションで片手はコントローラ、片手は素手、という混在もできます。
追加の設定は不要で、素手を出せばそのまま使えます（`--no-hands` で無視できます）。

| ジェスチャ | 既定の意味 | オプション |
| --- | --- | --- |
| 親指と人差し指の先をつまむ（ピンチ） | **クラッチ**。つまんでいる間だけ腕が手に追従し、離すとその場で保持 | `--hand-clutch always` で「手が見えている間は常に追従」 |
| 中指・薬指・小指を手のひらに握り込む | **グリッパ**。伸ばして開、握って閉（連続値） | `--hand-gripper pinch`（`always` クラッチ時のみ。ピンチの強さがグリッパ）／`none` |
| 両手で親指と中指の先をつまむ | **リセンター**。中断中なら腕を**再開** | — |
| 片手だけで親指と中指の先をつまんで 1 秒保持 | **録画の開始／停止** | — |
| 両手のひらをヘッドセットに向け、指を伸ばす（ストップの合図） | 0.5 秒で**中断**、2.5 秒保持で**終了** | `--stop-hold PAUSE,END` |

手の「位置」は既定で手のひらの中心（手首と中指の付け根の中点）で、`--hand-reference wrist`（手首関節）や
`--hand-reference pinch`（親指と人差し指の先の中点。つまんだ点が手先目標になるので、把持中心を意識して
操作しやすい）に変えられます。向きは手首関節の向きを、コントローラと同様に**つまんだ時点からの相対回転**
として加えます（`--no-orientation` で切れます）。

ピンチはヒステリシス付きで判定します。指先間の距離が `--pinch-on`（既定 2 cm）を切るとつまんだと見なし、
`--pinch-off`（既定 3.5 cm）を超えるまで離したと見なしません。指先の球の半径分だけ距離が残るので、
実際に触れ合っている状態はおおむね 1〜1.5 cm です。指の握りは、各指の「指先〜付け根の距離 ÷ 指の長さ」
（伸ばすと約 1、握ると 0.3 程度）を 0.9 → 0.45 の範囲で 0〜1 に写した 3 本の平均です。

#### 同期の開始（手をロボットの手先の位置に持って行ってからつまむ）

つまんだ瞬間にロボットの手先が離れた場所へ飛び出さないよう、絶対マッピングでは**手がロボットの手先に
対応する位置に来るまでクラッチを噛ませません**。サーバはロボットの現在の手先位置を「操作者の頭を
ロボットの頭に重ねた対応」で操作者側に写し、そこに球のマーカーを描きます（半径は `--hand-engage-radius`、
既定 5 cm。範囲内に入ると橙から緑に変わります）。手のひら（`--hand-reference` の点）をマーカーに入れて
つまむと、その位置を基準に同期が始まり、以後はロボットの手先が手に追従します。マーカーの外でつまんでいる
間は HUD に「WAITING – マーカーまで xx cm」と出て、腕は一切動きません。つまんだまま手をマーカーへ持って
行けば、入った瞬間に噛みます。離すと保持、次につまむときも同じゲートを通ります（追従中の手先は手の近くに
あるので、通常はそのまま噛みます）。相対マッピングは押した位置が基準なので飛ぶことがなく、ゲートはありません。
コントローラでも同じゲートを使いたいときは `--engage-radius M` を与えます（既定ではコントローラは従来どおり、
押した瞬間から手先が寄っていきます）。

#### 中断と終了（トラッキング中にジェスチャだけで止める）

素手のときはコントローラのボタンもページも使えないので、**両手のひらをヘッドセットに向けて指を伸ばす**
（ストップの合図。ピンチしていない開いた手で、手のひらの法線がヘッドセットの方を向いていること）を
ジェスチャとして読みます。

- **0.5 秒**保持: **中断**。両クラッチを解放して腕を止め（`arms_enabled` オフ）、その後どんなピンチをしても
  腕は動きません。頭部追従とカメラ映像はそのままです。
- **2.5 秒**保持: **終了**。腕を止めたまま、ページが WebXR セッションを終了して VR から抜けます。
- **再開**: リセンター操作（両手で親指と中指のピンチ）が再開を兼ねます。再開後もマーカーのゲートを
  通るので、手を置いた位置から急に動くことはありません。ページの `arms` チェックボックスでも再開できます。

保持時間は `--stop-hold PAUSE,END` で変えられます。HUD には保持中の秒数と「あと何秒で中断／終了」が出ます。

VR 内では追跡中の関節が小さな球で描かれ（手首は橙、クラッチ中は緑）、HUD にピンチ距離・握り具合・
判定できていない関節が出ます。手が視野から外れるなどして追跡が切れると、その瞬間にクラッチを解放して
手先を保持します（コントローラのトラッキングロスと同じ扱い）。`--hand-clutch always`（手が見えている間は
常に追従）でもゲートは効くので、手をマーカーに持って行くまで動きませんが、実機ではまず既定のピンチ
クラッチで試してください。

```bash
# 素手で操作（既定で有効。コントローラも同時に使える）
uv run --extra kinematics robopy-vr --host 0.0.0.0 --self-signed
# つまんだ点を手先目標に、ピンチの強さでグリッパ、手が見えていれば常に追従
uv run --extra kinematics robopy-vr --host 0.0.0.0 --self-signed \
    --hand-clutch always --hand-gripper pinch --hand-reference pinch --mapping relative
# ゲートを 3 cm に絞り、中断 1 秒／終了 3 秒に
uv run --extra kinematics robopy-vr --host 0.0.0.0 --self-signed --hand-engage-radius 0.03 --stop-hold 1,3
```

録画の JSON では各ステップの `controllers.<side>.input` が `"hand"` か `"controller"` かを示し、
動画では手も同じ球として（手のひらの位置に）描かれます。

### 録画と動画の保存

操作中にどう動いていたかを後から確かめるために、サーバがセッションを記録して動画にします。
ページの **● Record** ボタン、または VR 中は **B / Y** で開始し、もう一度押すと停止します。停止すると
`--record-dir`（既定 `./recordings`）に JSON のログ（関節角、コントローラの位置（ロボット座標系）、
クラッチ状態、手先目標、IK 残差、毎ステップ）が書かれ、続けて MuJoCo で 2 本の MP4 が描画されます。

- `*_third_person.mp4`: 正面やや上からの 3 人称視点。コントローラの位置を球で描き（押している側は緑、
  離している側は灰色）、押している手先から目標への線を引きます。
- `*_first_person.mp4`: 1 人称視点。実カメラを配信しているときはそのカメラの画像そのもの（録画中に書かれる）、
  シミュレーションではロボットの頭部カメラ（`head_camera_link`、光軸は頭の中立姿勢から導出）の描画。
- `*_operator_view.mp4`: **操作者がヘッドセットで見ていた画面**。ページが録画中に自分の視点でシーン（ツイン、
  ミラー、カメラ画像、HUD）を 10 fps でオフスクリーン描画して JPEG でサーバに送り、サーバが動画に書きます。
  デスクトッププレビュー中はウィンドウの視点になります。

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
- コントローラや手のトラッキングが外れた瞬間にクラッチを解放します（測っていない姿勢で動き続けない）。
  手は必要な関節が 1 つでも欠ければ「追跡なし」として扱います（どの関節かは HUD に出ます）。
- 素手の絶対マッピングでは、手がロボットの手先に対応する位置（マーカー）に来るまでクラッチが噛みません
  （`--hand-engage-radius`）。つまんだ瞬間に腕が飛ぶことはありません。
- 両手のひらをヘッドセットに向けるストップの合図で、腕を止め（0.5 秒）、セッションを終えられます（2.5 秒）。
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

### 校正コマンド（`robopy-rakuda-calibrate`）

上の表のうちバイラテラルに必要な関節ごとの値を、実機で測って `.robopy/rakuda/config.yaml` の `control:`
セクションに書き込むコマンドです。数値はすべて機械から読むか操作者が測るもので、データシートからは
何も埋めません。

```bash
uv run robopy-rakuda-calibrate --leader-port /dev/ttyUSB1 --follower-port /dev/ttyUSB0
uv run robopy-rakuda-calibrate --simulate     # 模擬バスと自動操作者で流れを見る
```

対象は既定で胴体＋両腕の 13 モータ（`--motors` で絞れます。頭とグリッパは結合対象外なので指定できません）。
リーダ、フォロワの順に、各腕で次を行います。**対象モータのトルクは切れる**ので腕を支えてください。

1. **レジスタ読み取り**（自動）: モデル、`DRIVE_MODE`、`HOMING_OFFSET`、`VELOCITY_LIMIT`（→ `max_velocity_rad_s`）、
   `CURRENT_LIMIT`（その `--current-fraction`、既定 0.5 倍 → `current_limit_a`）。
2. **モータ ↔ URDF 関節の対応**: チェーン順の提案を表示し、Enter で確定、違えば入力します（名前からの推定は
   しません）。
3. **ゼロ点**: 指示された基準姿勢（`--zero-pose` の文、既定はモデルのゼロ姿勢）に手で合わせて Enter →
   `zero_count`。
4. **向き**: 関節ごとに、モデルが正とする向きへ手で動かす → カウントの増減から `direction`。
5. **可動域**: 両端へ手で動かして Enter ずつ → `lower_limit_rad` / `upper_limit_rad`。
6. **トルク定数**（任意、`--no-torque-constant` で省略）: 関節軸を水平・リンクを水平にして保持させ、
   無負荷時と既知の質量 `m` を腕長 `r` に吊るした時の保持電流の差から
   `torque_constant_nm_per_a = m g r / |I_load − I_free|`。

すべて揃った関節だけ `validated: true` になり `bilateral.coupled_motors` に入ります（片腕でも欠ければ結合
しません）。`--allow-current` を付けたときだけ、かつ結合関節がすべて完全なときだけ
`allow_hardware_current_output: true` を書きます。既存の `control:` のゲイン・周期・モデル設定は保持し、
`leader.torque_enabled` / `follower.torque_enabled` は結合関節を含むよう広げ、元のファイルは
`config.yaml.bak-<日時>` に残します（コメントは失われます）。書いた後に通常のローダで読み戻し、
`JointMap.require("hardware")` を通ることを確認してから成功を報告します。

`bilateral.allow_uncompensated` は既定 `false` のままです。電流制御には腕ごとの検証済み重力補償モデルも
必要で、これはこのコマンドでは測れません。重力が載らない関節だけを結合するなら、承知のうえで `true` に
してください。


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
