# CALVIN のシーンの Rakuda — `rakuda.calvin_table` と `rakuda.calvin_pick`

`../vapo/scripts/aif_collection_uv.py` がインスタンス化するのと同じ **CALVIN scene D**
（play table + 3 つのブロック）を RoboVerse 上に再現した環境が 2 つあります。
どちらも家具・ブロック・スケールは同一で、**違うのは Rakuda の立ち位置だけ**です。

| タスク | 立ち位置 | 解けるか |
| --- | --- | --- |
| `rakuda.calvin_table` | `calvin_scene_D.yaml` の `robot_base_position` そのまま | **解けない**（届かない） |
| `rakuda.calvin_pick` | Rakuda 自身の可作業領域から逆算した位置 | 赤ブロックを掴んで持ち上げる |

## 動いている様子

<video controls width="960" src="../assets/rakuda_calvin_motion.mp4"></video>

前半が `rakuda.calvin_table`（Panda のベース位置。腕を伸ばしきっても届かない）、
後半が `rakuda.calvin_pick`（届く位置に置き直して実際に掴んで持ち上げる）です。

![pick](../assets/rakuda_calvin_pick.png)

```bash
python examples/roboverse/rakuda_calvin_motion.py \
    --video docs/robots/assets/rakuda_calvin_motion.mp4 \
    --still docs/robots/assets/rakuda_calvin_pick.png
# act one -- where CALVIN puts its Panda
#   block is 0.520 m from the base body
#   furthest the hand can hold: [-0.0922 -0.3136  0.4826]
#   still 0.242 m short of the block, horizontally
# act two -- mounted where its workspace says it can work
#   block is 0.293 m from the base body
#   lifted +0.1072 m, in the hand: True
#   task reports success: True
```

方策ではなく、`examples/roboverse/ik.py` の逆運動学によるスクリプト制御です。
成功判定は環境自身のものを使っています。

!!! warning "`rakuda.calvin_table` の配置では手はテーブルに届きません"
    シーンは Panda 用に作られています。Rakuda の手は **ベースから 0.507 m** までしか
    届かず（`rakuda_gripper` の全関節を 20 万点サンプルして計測）、
    Panda のベース位置からは一番近いブロックでも **0.519 m** 先にあります。
    つまり最良でも約 1 cm 足りず、実際には机の高さではもっと届きません。

    これは想定どおりです。この環境は「両者を同じスケールで並べて見る」ためのもので、
    解くためのものではありません。同じ机で実際に作業させたい場合は
    `rakuda.calvin_pick` を使ってください。

## シーンだけを見る

<video controls width="960" src="../assets/rakuda_calvin_table.mp4"></video>

![front](../assets/rakuda_calvin_front.png)

```bash
python examples/roboverse/rakuda_calvin_scene.py \
    --stills docs/robots/assets \
    --video docs/robots/assets/rakuda_calvin_table.mp4
```

## 使い方

```python
from metasim.task.registry import get_task_class

# Panda のベース位置に立たせるだけの環境
cls = get_task_class("rakuda.calvin_table")
env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
states, _ = env.reset()

# 同じ机で実際に掴める環境
cls = get_task_class("rakuda.calvin_pick")
env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
states, _ = env.reset()
# env.lift(states) / env.in_hand(states) で進捗が取れる
```

インストールは [`rakuda.lift_block` のページ](rakuda_lift_block.md#1-インストール) と同じで、
`pip install -e ".[sim]"` だけです。RoboVerse 側の変更は要りません。

テーブルの MJCF は生成物なのでリポジトリに入っていますが、作り直すこともできます。

```bash
python examples/roboverse/calvin_table_asset.py
# wrote examples/roboverse/assets/calvin_table/mjcf/calvin_table.xml
#   9 bodies, 4 joints, 60 geoms, 11 meshes
#   4.81 kg, 0.880 x 0.449 x 0.697 m at scale 0.8
#   45 boxes in place of base_link's concave mesh
#   52 collision geoms moved to group 3 so they stop being drawn
#   textured: mat_base_link, mat_slide_link, mat_drawer_link, mat_plank_link
#   work surface at z = 0.440
#   moving parts: base__button, base__switch, base__slide, base__drawer
# wrote examples/roboverse/assets/calvin_table/urdf/calvin_table_scaled.urdf
#   （同じ 9 bodies / 4.81 kg / 天板 z=0.440。MuJoCo 以外のバックエンド用）
```

床と空は別アセットです。こちらも生成物（USD 側だけ）です。

```bash
python examples/roboverse/calvin_room_asset.py
# wrote examples/roboverse/assets/calvin_room/calvin_room.usda
#   30 x 30 m floor, tile 0.5 m (15 texture repeats per side)
#   collider: yes
```

## 見た目

最初のバージョンは**背景が真っ黒で、机が灰色の塊**でした。原因は 3 つあり、
どれもメッシュではなく読み込み方の問題です。

### 1. 衝突形状が視覚メッシュの上に描かれていた

MuJoCo の URDF インポータは `<visual>` を geom group 1 に、`<collision>` を
group 0 に置きます。そして **MuJoCo のレンダラは group 0 / 1 / 2 をどれも描きます**。
つまり後述の「[詰まったところ: `base_link` の凹形状](#base_link)」で作る箱 45 個と、引き出し・引き戸・スイッチの凸包が、
本来の木目メッシュの上に白い塊として重ね描きされていました。
あの「灰色のブロックの山」はこれです。

エクスポート時に衝突 geom を **group 3**（MuJoCo が既定で隠す、MuJoCo 自身の
モデルが使っている慣習）へ移しています。物理は一切変わりません。group は描画の
ヒントであって、ビューアでは今も表示に切り替えられます。

### 2. テクスチャを誰も読んでいなかった

`calvin_table_D` の OBJ は Blender 由来で UV を持っていて、隣に `.mtl` があり、
`map_Kd ../textures/dark_wood.png` と書いてあります。PyBullet はこれを読みます。
**MuJoCo の URDF インポータは `mtllib` を無視します。**

そこでエクスポータが MTL を自分で読み、MuJoCo の `<texture>` / `<material>` を
書き出して視覚 geom に割り当てるようにしました。

| リンク | テクスチャ |
| --- | --- |
| `base_link`, `plank_link` | `dark_wood.png` |
| `slide_link`, `drawer_link` | `dark_wood__gray_handle.png` |
| `switch_link` | なし（MTL に `map_Kd` がない）。URDF の灰色のまま |
| `button_link`, `led_link`, `light_link` | STL で UV がない。URDF の色のまま |

テクスチャ 2 枚（730 KB）は `examples/roboverse/assets/calvin_table/textures/`
に calvin_env からそのまま持ってきています（MIT）。

### 3. 床も空も照明もなかった

`ScenarioCfg` に `scene` を渡していなかったので MetaSim の既定の灰色チェッカー板が
敷かれ、その上は何もないので黒、照明は MuJoCo のカメラヘッドライト —
カメラの位置から照らすので陰影が出ず、影も落ちません。

いまは `examples/roboverse/staging.py` が全タスク共通で用意しています。

- **床と空** = `assets/calvin_room/`。床は CALVIN 自身の `checker_blue.png` を、
  `plane.obj` / `plane.mtl` から実測したタイル 0.5 m・`Kd 0.588` で貼っています。
  空はグラデーションのスカイボックス。壁は一度入れて外しました（理由は
  `assets/calvin_room/README.md`）。
- **照明** = `staging.LIGHTS`。ドーム（空の回り込み）＋ キー（右前上方）＋
  フィル（反対側、1/4 の強度）の 3 灯。

照明を MJCF ではなく `ScenarioCfg.lights` に書いているのが要点です。MetaSim の
ライト設定は UsdLux の用語で書かれていて、**バックエンドごとに翻訳されます** —
Isaac Sim は本物の UsdLux プリムを作り、MuJoCo は固定機能 OpenGL ライトで近似します
（対応表と較正済みの露出定数は `metasim/sim/mujoco/lights.py`）。
MJCF に `<light>` と書くと MuJoCo でしか効きません。

## Isaac Lab で動かす

シーンの記述はバックエンド非依存なので、`--sim` を変えるだけで切り替わります。

```bash
python examples/roboverse/rakuda_calvin_views.py --sim mujoco
python examples/roboverse/rakuda_calvin_views.py --sim isaacsim
```

`rakuda_calvin_views.py` は `ScenarioCfg.cameras` と `states.cameras[...].rgb`
だけを使うので、どのバックエンドでも同じ 4 視点が出ます。
隣の `rakuda_calvin_scene.py` のほうが速くて動画も撮れますが、あちらは
`env.handler.physics.model` を直接触って `mujoco.Renderer` を回しているので
MuJoCo 専用です。

### 何が用意してあるか

| 要素 | MuJoCo | Isaac Sim |
| --- | --- | --- |
| プレイテーブル | `mjcf/calvin_table.xml` | `urdf/calvin_table_scaled.urdf` → Isaac Lab の `UrdfConverter` が USD 化してキャッシュ |
| 床・空 | `calvin_room.xml` | `calvin_room.usda`（`calvin_room_asset.py` が生成。コライダ付き） |
| Rakuda | `rakuda.xml` | 同じ MJCF を MetaSim が `convert_mjcf_to_usd_cached` で USD 化 |
| ブロック・台座 | プリミティブ | プリミティブ |
| 照明 | `LIGHTS` を OpenGL ライトに近似 | `LIGHTS` を UsdLux プリムに |

テーブルの URDF は**上流のものではなく生成物**です。上流の URDF は等倍で、
`base_link` の衝突が凹メッシュのままなので、そのまま読むと「スケール済みの
ブロック位置の下に等倍の机」が「くさび形の凸包」の上に立ちます。
MJCF と URDF は同じ `_prepared_urdf()` から出ていて、
`test_calvin_table.py::TestUrdfExport` が 9 bodies / 4.81 kg / 天板 z=0.440 /
バウンディングボックスの一致を毎回確認します。

シーンの USD だけは自動変換がありません。MetaSim はオブジェクトやロボットの
アセットは要求時に USD 化しますが、**シーンはしません** —
`IsaacsimHandler._load_scene` は `usd_path` が `None` なら警告して戻り、
しかも `scene` が `None` でないので自前の terrain も作らないため、
**床がまったくない状態**で立ち上がります。だから `calvin_room.usda` を生成して
`SceneCfg` に両方渡しています。

### 導入

Isaac Sim 5.0 は PyPI にありますが Isaac Lab はありません。RoboVerse の
`tools/install/isaacsim5.sh` が、動く順番で手順をまとめています。

```bash
# Python 3.11 の venv が要ります（Isaac Sim 5.0 は 3.11 専用）
uv venv --python 3.11 .venv-isaacsim && source .venv-isaacsim/bin/activate
RoboVerse/tools/install/isaacsim5.sh          # isaacsim 5.0.0 + Isaac Lab v2.2.1 + cu128 torch
pip install -e .                              # robopy 自身（Rakuda を登録する）
ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y     python -m metasim doctor --backend isaacsim
```

!!! warning "このマシン（Ubuntu 20.04）には入りません"
    NVIDIA が配布している Isaac Sim の wheel は、**4.0 以降すべて
    `manylinux_2_34` 以上**（Isaac Sim 5.0 は `manylinux_2_35`）です。
    つまり **glibc 2.34+、実質 Ubuntu 22.04 以降**が必要で、
    このホストは Ubuntu 20.04 / glibc 2.31 なので、どのバージョンも
    `pip install` できません（ディスク容量の問題ではありません）。

    ```bash
    $ ldd --version | head -1
    ldd (Ubuntu GLIBC 2.31-0ubuntu9.18) 2.31
    ```

    現実的な選択肢は 3 つです。

    1. **NGC のコンテナを使う**（`nvcr.io/nvidia/isaac-sim:5.0.0`）。イメージが
       自前の glibc を持っているのでホストが 20.04 でも動きます。
       `sudo usermod -aG docker $USER` と nvidia-container-toolkit の導入が要ります。
    2. **Ubuntu 22.04 以降のマシンで動かす。**
    3. **ホストを上げる。**

    どの道を通っても、リポジトリ側で追加で必要なことはありません。
    上の表のアセットはすべて揃っていて、`--sim isaacsim` を渡すだけです。

## `calvin_scene_D.yaml` から持ってきたもの

| 項目 | 値 | 出典 |
| --- | --- | --- |
| `global_scaling` | 0.8 | シーン全体に適用。MJCF に焼き込み済み |
| `robot_base_position` | `[-0.34, -0.46, 0.24]` | Rakuda の**足裏**をここに置く |
| `robot_base_orientation` | `[0, 0, 0]` | 単位回転。こちらで決めた値ではない |
| テーブル位置 / 姿勢 | `[0, 0, 0]` / `[0, 0, 0]` | `fixed_objects.table` |
| 関節初期値 | すべて 0 | `base__button` / `base__switch` / `base__slide` / `base__drawer` |
| 作業面 | `[[0.0, -0.15], [0.35, -0.03]]` | `surfaces.table` |
| ブロック | `block_red_middle` / `block_blue_small` / `block_pink_big` | 寸法・色は URDF どおり（質量は後述） |

Rakuda のモデル原点は腰にあるため、`base` ボディは `robot_base_position` より
`STAND_HEIGHT = 0.25752 m` 高い位置に置かれます（足裏が 0.24 m）。
CALVIN の Panda は 0.24 m の高さに**何の形状もなく**浮いていますが、
このシーンでは台座を描いているので Rakuda は浮いて見えません。

## 再現しなかったもの

- **LED と電球は光りません。** `effect: led` / `effect: lightbulb` は calvin_env の Python 側の
  処理で、URDF には形状しかありません。点灯させたい場合は関節を読んで geom の `rgba` を
  書き換えてください。
- **背景と照明は CALVIN のものではありません。** CALVIN 自体は 30 m の板 1 枚しか持たず、
  それ以外はビューポートのクリア色（＝黒）です。床は CALVIN のテクスチャをそのまま
  使っていますが、空と照明はこちらで足しています（「見た目」の節）。
- **ブロックの初期位置はランダムではありません。** CALVIN は `initial_pos: any` で作業面に
  ランダム配置しますが、ここでは比較しやすいように決め打ちで並べています。
- **ブロックの質量は CALVIN と違います。** URDF はどれも 1 kg で、PyBullet の
  `globalScaling` は形状だけ縮めて質量は変えないため CALVIN のブロックは実際その重さです。
  `mass=1.0` は渡していますが **MetaSim の MuJoCo バックエンドはこれを見ておらず**、
  実測すると体積 × 既定密度 1000 の **0.0896 kg** になります。把持が成立する理由の
  一つなので明記しておきます。

## 詰まったところ: `base_link` の凹形状 { #base_link }

CALVIN の `calvin_table_D.urdf` は `base_link` の `<collision>` を
`concave="yes"` 付きのメッシュで指定しています。PyBullet はこれを尊重しますが、
**MuJoCo はメッシュを必ず凸包に変換します**。この机の凸包は、作業面の手前の縁から
背面パネルの上端まで伸びる「くさび形」になり、机の上に置いたものは
数 cm めり込んだ状態で生成されて弾き飛ばされます。実際、最初はブロックが
`y = -0.93` まで吹き飛んでいました。

そこで、この 1 つの衝突メッシュだけを箱の集合に置き換えています
（`calvin_table_asset.decompose_to_boxes`）。

この机は CAD 由来でほぼ軸平行なので、**メッシュ自身の面が乗る平面**を延長して空間を
セルに切ると、各セルは完全に内側か完全に外側のどちらかになります。セルごとに 1 点だけ
内外判定（真上に飛ばしたレイが三角形を何回横切るか）を行い、隣接する内側セルを
x → y → z の順に貪欲にまとめると、7200 セルが **45 個の箱** になります。1 秒かかりません。

結果として机はちゃんと中空になり、引き出しと引き戸の通り道も空きます。
作業面の高さも視覚メッシュと一致します（`z = 0.440`）。
ハンドルやボタンの丸い部分だけは軸平行でないため、わずかに角張って（実際より大きめに）出ます。

!!! note "`surfaces.table` の `z = 0.46` は机の高さではありません"
    シーンファイルの 0.46 は物体を**落とす**高さです。実際の天板は `z = 0.440`
    （スケール前 0.55 × 0.8）で、CALVIN は 2 cm 上から落として置いています。
    この環境では `CALVIN_WORK_SURFACE_Z = 0.44` にブロックの半分の高さを足して直接置いています。

## `rakuda.calvin_pick` の立ち位置の決め方

数字はすべて計測値から逆算していて、手で選んでいません。

- **高さ**: 下向きに掴む動作はこの腕の肩より十分下でしか成立しないため、作業面は
  取付面の `GRASP_OFFSET_ABOVE_MOUNT = 0.15 m` 上に来る必要があります。
  CALVIN の天板は `z = 0.44` なので足裏は `0.44 - 0.15 = 0.29`。
  **Panda のベース（0.24）より高くなります**。
- **向き**: Rakuda の可作業領域は左右方向に狭いので、机の正面に正対させます
  （world `+y` を向く、ヨー 90°）。
- **前後**: 赤ブロックが `OBJECT_ZONE`（ロボット座標で x 0.24〜0.32、y −0.12〜0.12）の
  中央に来る位置に立たせます。

結果として足裏は `[0.0525, -0.42, 0.29]` です。

## 詰まったところ その 2: 机が自分で引き出しを開ける

箱分解は上流の凸分解（`*_vhacd2.obj`）とわずかに重なります。放っておくと
**誰も触っていないのに引き出しが 0.17 m 出てきて**、引き戸が 0.02 m 動き、
スイッチが下限を 0.015 m 突き抜けます。

引き出しを止めるのは関節のリミットであって筐体との接触ではないので
（CALVIN が使う PyBullet は親子リンク間の衝突を既定で無効にします）、
エクスポート時に `<contact><exclude>` を書き出しています。
**ボタンだけは除外しません** — ボタンは筐体に支えられていて、
接触を切ると 2 cm 落ちて押しっぱなしに見えるためです。

## 詰まったところ その 3: 指が空を掴む

逆運動学が解けていても、サーボは重力に対して定常偏差を残します。机の上に
腕を伸ばした姿勢では **約 0.02 m** ずれ、ジョー（開き 0.088 m）が
56 mm のブロックの横で閉じてブロックを弾き飛ばします。

`ScriptedRun.correct()` が**実機側のハンド位置**を読んでその差分だけ指令を
ずらし直します。4 回で 0.006 m まで詰まり、そこから閉じれば掴めます。

## テスト

```bash
pytest tests/test_roboverse/test_calvin_table.py tests/test_roboverse/test_calvin_room.py -q
# 37 passed
```

箱分解そのもの（重なりがないこと、中空であること、天板の高さが視覚メッシュと一致すること）、
エクスポートしたモデル（スケール・引き出しのストローク・レイキャストした作業面高さ）、
シーン（ロボットが Panda のベース位置に立つこと、300 ステップ後もブロックが机の上に
載ったままで、机の 4 関節が動かないこと）、そして `rakuda.calvin_pick` の立ち位置
（掴める高さであること、対象ブロックが可作業領域に入ること、逆運動学が把持姿勢に
到達すること、そして Panda のベース位置からは**届かない**こと）を検証します。
