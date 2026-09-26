# `rakuda.lift_block` を RoboVerse でインスタンス化する

グリッパ付き Rakuda（`rakuda_gripper`）でテーブル上のブロックを掴んで持ち上げる環境です。
このページだけ見れば環境を立ち上げて動かせるようにしてあります。

実際に掴んで持ち上がるところまで動作確認済みです。

<video controls width="640" src="../assets/rakuda_lift_block.mp4"></video>

```bash
python examples/robot/rakuda_lift_block.py --seed 1 \
    --video docs/robots/assets/rakuda_lift_block.mp4
# lifted +0.1161 m, still in the hand: True
# task reports success: True
```

`examples/robot/rakuda_lift_block.py` は**方策ではなく**、逆運動学で手先の経路を作る
スクリプト制御です。環境が端から端まで成立することを示すためのもので、学習の出発点として
使えます。全配置を解けるわけではありません（後述）。

!!! warning "このグリッパは実機の Rakuda のものではありません"
    Rakuda-2 の CAD エクスポートはグリッパを**形状のない fixed フレーム**として出力しており、
    実機グリッパの形状データはこのリポジトリにありません。ここで使っているのは
    [CALVIN](https://github.com/mees/calvin) の `panda_longer_finger` から借りた指です。
    指の形・ストローク・把持力のいずれも実機と一致せず、**実機転移は期待できません**。
    詳細は [`robopy.sim.panda_gripper`](rakuda_roboverse.md#グリッパ付き-rakuda_gripper) を参照してください。

## 1. インストール

RoboVerse / MetaSim が入っている環境に robopy を入れるだけです。**RoboVerse 側の変更は不要**で、
robopy が `metasim.packages` エントリポイントでコンテンツパックを登録します。

```bash
pip install -e ".[sim]"     # robopy のチェックアウトから
# あるいは MetaSim と MuJoCo が既にあるなら
pip install robopy
```

確認:

```bash
python -c "
from metasim.utils.setup_util import get_robot
from metasim.task.registry import get_task_class
r = get_robot('rakuda_gripper')
print(r.name, r.num_joints, 'joints')
print(get_task_class('rakuda.lift_block'))
"
```

## 2. 環境を作って 1 ステップ回す

```python
import torch
from metasim.task.registry import get_task_class

task_cls = get_task_class("rakuda.lift_block")      # "rakuda.lift_cube" でも同じ
scenario = task_cls.scenario.update(num_envs=1, headless=True)
env = task_cls(scenario=scenario, device="cpu")

states, info = env.reset(seed=0)

robot = scenario.robots[0]
joints = env.handler.get_joint_names(robot.name, sort=True)   # 19 個（アルファベット順）
action = torch.zeros(1, len(joints))
for i, j in enumerate(joints):
    action[0, i] = robot.default_joint_positions[j]

states, reward, terminated, timeout, _ = env.step(action)
env.close()
```

!!! note "行動ベクトルの順序"
    `env.handler.get_joint_names(robot.name, sort=True)` が返す**アルファベット順**です。
    `RAKUDA_ACTUATED_JOINTS` の並び（胴→頭→右腕→左腕）とは**異なります**。
    必ず `get_joint_names` の結果で添字を作ってください。

## 3. グリッパの開閉

実機は 1 腕 1 モータ（`l_arm_grip` / `r_arm_grip`、XM430-W350）なので、2 本の指は
equality 拘束で連結されています。MetaSim がアクチュエータを関節名で引く都合上
アクチュエータは指ごとにありますが、**まとめて指令してください**。

```python
from robopy.roboverse.robots import gripper_targets

targets = {j: float(robot.default_joint_positions[j]) for j in joints}
targets.update(gripper_targets(0.04, "right"))   # 全開
targets.update(gripper_targets(0.0,  "right"))   # 全閉

action = torch.tensor([[targets[j] for j in joints]])
```

| 指令値 | パッド間隔 |
| --- | --- |
| `0.00`（全閉） | **7.8 mm** |
| `0.04`（全開） | **87.6 mm** |

`pad_gap(opening)` で計算できます。**7.8 mm より薄いものは掴めません。**

## 4. シーンの構成

| 物体 | 内容 |
| --- | --- |
| `rakuda_mount` | ロボットを載せる擬似台。天面 `z = 0.60` |
| `table` | 作業台。天面 `z = 0.75`、`x ≥ 0.1976` から奥へ |
| `cube` | 掴む対象のブロック |

ロボットのベースは `z = 0.8575`（擬似台天面 + 0.25752）に固定されます。

### 作業面の高さが把持タスクだけ違う理由

`rakuda.reach` / `push_cube` は取り付け面から **0.34 m** 上に作業面を置きます（手が最も届く高さ）。
`lift_block` は **0.15 m** です。掴むには手先が対象を**上から**向いている必要があり、それは
肩よりかなり低い位置でしか成立しないためです。実測値:

| 取り付け面からの高さ | 正面に届く姿勢 | うち指が下を向くもの |
| --- | --- | --- |
| 0.15 | 9735 | **408** |
| 0.25 | 7343 | 15 |
| 0.34（リーチ用） | 2746 | **0** |

リーチ用の高さでは真上からの把持は困難ではなく**不可能**です。
`RakudaMount(offset=...)` で切り替えます。

```python
from robopy.roboverse.mount import GRASP_OFFSET_ABOVE_MOUNT, RakudaMount
mount = RakudaMount(work_surface_z=0.75, offset=GRASP_OFFSET_ABOVE_MOUNT)
mount.describe()
```

## 5. 成功判定と報酬

```python
env.lift(states)      # ブロックが初期高さから何 m 上がったか
env.in_hand(states)   # ブロックが手から 9 cm 以内か
```

`terminated` は **「6 cm 以上上がった」かつ「まだ手の中」** の両方が要ります。
片方だけだと、腕で弾き飛ばしただけでも成功になってしまうためです。

報酬は `-手とブロックの距離 + 5 × 持ち上げ量`（持ち上げ項は手の中にあるときだけ）。

## 6. 対象物の形が決まっている理由

ブロックは **高さ 100 mm** です。これは選択ではなく幾何的な帰結です。

借りた指は 101 mm あります。真上から挟むには、指先がテーブルを突き抜けないよう
掌をテーブルから 101 mm 以上上げる必要があります。30 mm の立方体だと中心は 15 mm なので、
把持点は指の 86 mm 地点＝**指先**になり、腕を動かすと抜けます。安定する 35〜50 mm の位置で
掴もうとすると、今度は指先がテーブル面より 40 mm 下に来ます。

高さ 100 mm なら、中心を掴んでも指先がテーブルを越えません。
**背の低い物体はこのタスクの難しい版ではなく、成立しない版です。**

## 7. 描画と録画

このマシンには `DISPLAY` がないので、オフスクリーン描画には EGL を使います。

```python
import os
os.environ["MUJOCO_GL"] = "egl"

import imageio, mujoco
renderer = mujoco.Renderer(env.handler.physics.model.ptr, height=480, width=640)
cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = [0.22, -0.02, 0.85]; cam.distance = 1.0
cam.azimuth = 148; cam.elevation = -16

frames = []
# ... env.step() のループ内で ...
renderer.update_scene(env.handler.physics.data.ptr, cam)
frames.append(renderer.render())

imageio.mimwrite("out.mp4", frames, fps=30, quality=7, macro_block_size=1)
```

既定のオフスクリーンバッファは 640 × 480 です。大きくするには MJCF に
`<visual><global offwidth="..." offheight="..."/></visual>` が要ります。

## 8. スクリプト制御の成功率

`examples/robot/rakuda_lift_block.py` はシード 0〜5 のうち **2 つ（seed 1, 3）** で成功します。
失敗の多くは接近中にブロックを倒すもので、**環境が解けないのではなく、この素朴な制御則が
配置に対して頑健でない**ということです。方策を学習させる余地がここにあります。

```bash
for s in 0 1 2 3 4 5; do
  python examples/robot/rakuda_lift_block.py --seed $s | tail -1
done
```

`reset(seed=N)` は配置まで再現します（同じシードなら毎回同じ位置にブロックが出ます）。

## 9. ここで踏んだ落とし穴

環境を動かすまでに実際に時間を溶かした3点です。同じことをやると同じところで止まります。

### IK は必ず別の `MjData` で解く

```python
scratch = mujoco.MjData(model)      # ← これを使う
live = env.handler.physics.data.ptr # ← 絶対に書き込まない
```

IK は候補姿勢の評価のために毎反復 `qpos` を書き込みます。それを**動いている
シミュレーションの状態**に対してやると、ロボットが毎回テレポートして接触が全部消えます。
症状は「掴んでいるのに持ち上がらない」で、物理の問題に見えるので原因に辿り着きにくいです。

### 姿勢は 6 自由度で固定しない

腕は 6 関節しかなく、任意の姿勢には到達できません。完全な姿勢を要求すると IK が
**17 cm 手前で止まり**、指の向きだけ拘束してロールを自由にすると **0.1 mm** まで収束します。
対象が正方形ならロールはどうでもよいはずです。

### 関節空間で補間しない

ホーム姿勢からブロック上空へ関節角を直線補間すると、腕がテーブルを薙いで
**手が着く前にブロックを倒します**。手先のデカルト経路を作り、各経由点で IK を解いてください。

## 10. その他よくある症状

| 症状 | 原因 |
| --- | --- |
| `get_robot('rakuda_gripper')` が見つからない | robopy を入れ直してエントリポイントを登録する |
| 掴んだのに持ち上げで落ちる | 把持位置が指先寄り。指の手前 2/3 で掴む |
| 掌がブロックにめり込む | `GRASP_DEPTH` がブロック半高より小さい。高さ 100 mm なら 50 mm 以上必要 |
