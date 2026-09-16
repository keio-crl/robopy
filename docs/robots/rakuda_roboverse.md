# Rakuda を RoboVerse で動かす

`pip install robopy` だけで、[RoboVerse](https://github.com/RoboVerseOrg/RoboVerse) / MetaSim の
シーンに Rakuda-2 を置いて動かせます。**RoboVerse のチェックアウト側を書き換える必要はありません。**

robopy は MetaSim の `metasim.packages` エントリポイントで自身のコンテンツパックを登録しているため、
インストールするだけで `get_robot("rakuda")` と `rakuda.*` のタスク名が解決されます。

```bash
# RoboVerse が入っている環境で
pip install -e ".[roboverse]"

python -c "from metasim.utils.setup_util import get_robot; print(get_robot('rakuda').mjcf_path)"
```

```python
from metasim.scenario.scenario import ScenarioCfg
from metasim.utils.setup_util import get_handler

scenario = ScenarioCfg(robots=["rakuda"], simulator="mujoco", num_envs=1, headless=True)
handler = get_handler(scenario)
```

動かして確かめる例:

```bash
python examples/robot/rakuda_roboverse.py --demo wave
python examples/robot/rakuda_roboverse.py --demo task --task rakuda.push_cube --video out.mp4
```

## :material-alert: 最初に知っておくべき制約

| 事実 | 影響 |
| --- | --- |
| **グリッパは関節ではない** | CAD エクスポートは `gripper_left_dof` / `gripper_right_dof` を **fixed** フレームとして持っています。名前に `_dof` と付いていますが自由度ではありません。既定の `rakuda`（15 関節）では**把持できません**。把持が要るときは `rakuda_gripper`（19 関節、後述）を使いますが、その手は**借り物**です。 |
| **自分が取り付けられている面には手が届かない** | 肩は台座面から 0.41 m、腕長は約 0.30 m。手先は取り付け面から **0.112 m** より下に行けません。机に直接載せると机上の物すべての上を素通りします。→ 擬似台に載せて解決（後述）。 |
| **前方リーチは約 0.37 m** | 台座が `x = 0.178` まで占有するので、作業面はそれより先に置きます。 |
| **質量は仮定を含む** | CAD の質量値は実は体積（後述）。密度を掛けて質量にしています。相対的な分布は CAD 由来、絶対値は仮定です。 |

## :material-elevation-rise: 擬似台への設置（CALVIN 方式）

Rakuda は**自分が取り付けられている面に手が届きません**。そのため、床に置くのではなく
**作業面を基準に高さを決めた擬似台**の上に載せます。これは CALVIN が Franka に対して
やっていることと同じです。`calvin_D` ではアームの `robot_base_position` が `z = 0.24`、
プレイテーブル天板が `z = 0.46` — **ベースは作業面の 0.22 m 下**に固定されています。

robopy でも同じ構成を [`robopy.roboverse.mount`](#) が提供します。違いは2点だけで、
オフセットをこのロボット自身の到達集合から**実測**していること、擬似台が実体のある箱
であること（CALVIN は幾何を持たない暗黙のマウント）です。

### オフセットの根拠

関節範囲をサンプリングし、「水平面の直上 8 cm の帯に手先が入る割合」を、その面が取り付け面から
どれだけ上にあるかの関数として測った結果です。

| 取り付け面からの高さ | 到達率（全方向） | 到達率（正面のみ） |
| --- | --- | --- |
| 0.12 | 1.0% | 0.2% |
| 0.18 | 4.2% | 1.1% |
| 0.22 | 7.1% | 2.0% |
| 0.30 | 11.5% | 3.3% |
| **0.34** | **12.4%** | **3.4%** |
| 0.40 | 12.1% | 3.7% |

手先高さの中央値が取り付け面から 0.407 m なので、そのあたりに作業面を置くと上からも下からも
アプローチでき、0.35 付近で頭打ちになります。0.15 未満はほぼ使えず、**0.112 未満は物理的に不可能**です。

### 使い方

作業面の高さを決めれば、他はすべて決まります。

```python
from robopy.roboverse.mount import RakudaMount

mount = RakudaMount(work_surface_z=0.75)   # 普通の机の高さ
mount.describe()
# {'work_surface_z': 0.75, 'offset_above_mount': 0.34, 'pedestal_top_z': 0.41,
#  'robot_base_z': 0.66752, 'table_near_edge_x': 0.1976}

scenario.objects = [mount.pedestal(), mount.table()]
init = {"robots": {"rakuda": {"pos": torch.tensor(list(mount.base_position)), ...}}}
```

- `mount.base_position` … CALVIN の `robot_base_position` に相当
- `mount.pedestal()` / `mount.pedestal_position()` … 擬似台。ベースプレート（0.30 × 0.30、
  中心は原点から (+0.0276, −0.0284) ずれている）を覆う大きさで床に置かれます
- `mount.table()` / `mount.table_position()` … 天板が作業面になるテーブル。擬似台と重ならない
  位置（`x ≥ 0.1976`）から始まります
- 手が届かない高さを指定すると `ValueError` で**拒否されます**（黙って不可能なシーンを作らない）

擬似台の天面はロボットの底面より 2 mm 低くしてあります。どちらも固定されているので接触しても
害はありませんが、シーンが最初から接触状態だと後で問題が起きたときに切り分けづらいためです。

## :material-cube-outline: モデル

MJCF は CAD エクスポート（`models/rakuda/assembly_2/urdf/assembly_2_convex_collision.urdf`）から
`robopy.sim.mjcf_export` が生成し、リポジトリにコミットしてあります。再生成は次の通りです。

```bash
pip install -e ".[sim]"
python -m robopy.sim.mjcf_export
```

2 つのモデルが出力されます。どちらも**同じロボット**（同じボディ・関節・質量・サイト）で、
違うのは「何を描画するか」だけです。

| ファイル | 管理 | 内容 |
| --- | --- | --- |
| `models/rakuda/assembly_2/mjcf/rakuda.xml` | git（79 KB） | STL を参照。視覚メッシュ（Git LFS）があれば綺麗に描画される |
| `models/rakuda/assembly_2/mjcf/rakuda_gripper.xml` | git | 同上 + 借り物のグリッパ |
| `src/robopy/roboverse/assets/rakuda/rakuda.xml` | git（2.8 MB） | 形状を XML に埋め込んだ自己完結版。**wheel に同梱される**ので `pip install` だけで動く |
| `src/robopy/roboverse/assets/rakuda/rakuda_gripper.xml` | git | 同上 + 借り物のグリッパ |

`RakudaCfg` は次の順で選びます。

1. 環境変数 `ROBOPY_RAKUDA_MJCF`
2. チェックアウトの `models/…/mjcf/rakuda.xml`
3. wheel 同梱の自己完結版

### 変換で対処していること

CAD エクスポートをそのまま MuJoCo に読ませると壊れます。以下は実際に踏んだ問題です。

- **メッシュ名の衝突**: 視覚メッシュ `meshes/X.stl` と凸包 `collision_meshes/X.stl` は同じベース名で、
  MuJoCo はベース名でアセットを命名します。そのまま読むと 2 つが 1 つに潰れ、**衝突形状が
  視覚メッシュ（8232 面）に置き換わって凸包が捨てられます**。別名でステージングして回避しています。
- **`fusestatic` は使えない**: 153 ボディを 16 に畳めますが、`MjSpec.to_xml()` は融合済みモデルを
  正しく書き出せません。ジオメトリは移動するのに各ボディの `pos` と `<inertial>` は元のまま出力され、
  保存して読み直すと**部品が最大 27 cm ずれ、質量が 1/10 になります**。融合なしなら float32 丸め
  （2e-7 m）で往復します。
- **質量は質量ではない**: すべての `<mass>` は **m³ 単位の体積**です（密度 1 で出力されている）。
  各部品の質量値と、その部品自身の衝突メッシュの体積を比べると、比は中実部品（ベースプレート、
  サーボ本体）で 1.008 が下限、中空部品でのみ 1 を超えます — 凸包を取ればそうなります。
  合計 2.069 リットル。既定では **2700 kg/m³（アルミ）** を掛けて 5.587 kg としています。
  これは**測定値ではなく仮定**です。`--density` で変えられます。
- **200k 面制限**: RealSense D435 の視覚メッシュは 203,886 面で MuJoCo の上限を超えるため、
  その部品だけ凸包を描画します。
- **組み立て済みの接触**: 腰スラストベアリングとその受けなど 3 ペアは設計上接触しているので、
  MJCF 内で `<contact><exclude>` しています。

### 追加している情報

- **位置サーボ 15 個**: 力の上限は DYNAMIXEL の公称ストールトルク
  （XM430-W350 = 4.1 N·m、XM540-W270 = 7.3 N·m、`robopy.motor.dynamixel_control_table` 由来）。
  どの関節をどのサーボが回すかは CAD の部品名から読み取っています。
- **サイト**: `gripper_left` / `gripper_right` / `head_camera`。
- **ボディ名**: 可動ボディは関節名（`elbow_pitch_left` など）、台座は `base`。固定リンクは CAD 名のままです。
- **連続関節の範囲**: `torso_yaw_dof` と両 `shoulder_pitch_*_dof` は URDF に範囲がありません。
  ビューアと同じ規約で**モータ可動域 ±π** を与えています。これは
  「モータが許す範囲」であって「機体が実際に止まる位置の測定値」ではありません。

## :material-hand-back-right: グリッパ付き `rakuda_gripper`

Rakuda の CAD は**グリッパを形状のない fixed フレームとして出力**しているため、実機のグリッパ形状は
このリポジトリにありません。把持タスクを組むために、uvCalvin の `panda_longer_finger` から**指だけ**を
借りて両腕に取り付けた別ロボットを用意しています。

```python
robot = get_robot("rakuda_gripper")   # 19 関節（15 + 指 2 本 × 2 腕）
get_robot("rakuda")                   # 15 関節のまま。変更していません
```

### これは実機の Rakuda ではありません

- 指の形・ストローク・把持力のいずれも実機と一致しません。**実機転移は期待できません。**
- さらに `gripper_*_dof` から実際の把持点までの TCP オフセットは
  `examples/config/rakuda_control.example.yaml` で**未測定**（`validated: false`）なので、
  「どこに付けるか」自体が仮定です。
- ライセンスは Apache 2.0。メッシュは `models/gripper_panda/` に原本のまま同梱しています。

### 何を借りて、何を借りなかったか

| | 採用 | 理由 |
| --- | --- | --- |
| 指メッシュ `longer_finger_v2` | ✅ | CALVIN と同一。21 × 26 × **96 mm** |
| ストローク 0〜0.04 m / 指 | ✅ | CALVIN の `gripper_joint_limits`。指形状がこの範囲を前提に作られている |
| `panda_hand` 本体 | ❌ | **0.81 kg・幅 204 mm**。Rakuda の前腕は 0.126 kg、手首リンクは 65 mm 幅で、運搬リンクの 6.4 倍・手首の 3 倍になり破綻します |
| 指の質量 0.1 kg | ❌ | 同じ理由で 0.02 kg に変更。**これは選択であって測定値ではありません** |

### 挟める寸法（実測）

| 指令 | パッド間隔 |
| --- | --- |
| 全閉 | **8 mm** |
| 全開 | **88 mm** |

全閉でも 8 mm 残るので、**8 mm より薄いものは掴めません**。また物体は指の**手前 2/3**に
挟む必要があります（30 mm キューブを 35〜50 mm の位置で保持すれば手首を 0.9 rad 振っても
落ちませんが、65 mm の位置だと先端から抜けます）。

### 1 腕 1 モータ

実機の `l_arm_grip` / `r_arm_grip` は XM430-W350 **1 個**なので、2 本の指は MuJoCo の
equality 拘束で連結してあります。アクチュエータは MetaSim が**関節名で引く**ため各指に
1 個ずつ置いていますが、物理的には 1 自由度です。両方をまとめて動かすには:

```python
from robopy.roboverse.robots import gripper_targets
targets.update(gripper_targets(0.0, "right"))   # 右手を閉じる
```

把持力は導出できませんでした。実機は電流 raw 128（= 0.344 A、XM430 で約 0.61 N·m）に
制限されていますが、サーボトルクからジョー力への換算には機構のレバー比が必要で、
その機構はモデル化されていません（指は直動ジョイントで動いています）。20 N は
**もっともらしい値であって導出値ではありません**。ゲイン 800 は実測で決めました
（80 ではどの深さでも落とし、300 では 35 mm のみ保持、800 で 35/50 mm とも保持）。

### 再生成

```bash
python -m robopy.sim.mjcf_export                                  # 4 モデルすべて
python -m robopy.sim.mjcf_export -o out.xml --gripper panda_longer_finger
```

## :material-run: タスク

| 名前 | 内容 |
| --- | --- |
| `rakuda.reach` | 右手を目標点へ |
| `rakuda.bimanual_reach` | 両手をそれぞれの目標点へ |
| `rakuda.push_cube` | テーブル上のキューブを目標領域まで押す |
| `rakuda.lift_cube` | キューブを掴んで持ち上げる（`rakuda_gripper` を使用） |

3つとも、ロボットは擬似台（`rakuda_mount`）の上に立ちます。`push_cube` はさらに
作業面 `z = 0.75` のテーブルを正面に置き、キューブは実測した到達領域
`x ∈ [0.24, 0.32]`、`y ∈ [-0.12, 0.12]` に配置されます。

```python
from metasim.task.registry import get_task_class

cls = get_task_class("rakuda.push_cube")
env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
states, info = env.reset(seed=0)
states, reward, terminated, timeout, _ = env.step(action)
```

目標のサンプリング範囲は、モデルの到達可能集合を実際にサンプリングして決めています
（`robopy.roboverse.tasks._common`）。高さは**取り付け面基準**で書かれているので、
擬似台を上げ下げすれば目標もロボットと一緒に動きます。テストが同じ値を再計算するので、
モデルを作り直して形状が変われば失敗します — タスクが黙って解けなくなることはありません。

テストには「実際にキューブに手が届くか」を関節空間の探索で確かめるものも含まれています
（擬似台が無ければ解が存在しません）。

把持を伴うタスクは `rakuda_gripper` の上に組んでください。`rakuda.lift_cube` がその例です
（成功判定は「キューブが 6 cm 上がっている」かつ「まだ手の中にある」の両方 — 片方だけだと
腕で弾き飛ばしただけでも成功になってしまいます）。

## :material-cog-outline: 自己衝突の設定

`RakudaCfg.enabled_self_collisions` は `"mujoco_default"` です。`True` / `False` はどちらも不適切です。

- `True` → MetaSim が `filterparent` を全体で無効化します。この 153 部品の CAD 組立では、
  カバーはその中身に、ヒンジホーンはハウジングに**設計上めり込んでいる**ため、
  静止状態で 264 接触・最大 43 mm 貫通が発生し、サーボが飽和してロボットが固まります。
- `False` → MetaSim が全ボディ対（11,628 組）に exclude を追加し、腕が胴体を通り抜けます。

`"mujoco_default"` なら親子対のみ除外され、それ以外は正しく衝突します。素の
`mujoco.MjModel.from_xml_path` と同じ挙動になり、検証もその条件で行っています。

## :material-test-tube: テスト

```bash
pytest tests/test_roboverse/
```

MetaSim / MuJoCo が無い環境では自動でスキップされます。テストは MJCF の順運動学を
**URDF から手計算した基準**と突き合わせます（変換コードも MuJoCo も使わずに）。
生成物が「読み込めるが別のロボットになっている」状態を検出するためです。
