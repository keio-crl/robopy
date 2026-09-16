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
python examples/robot/rakuda_calvin_motion.py \
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

方策ではなく、`robopy.roboverse.ik` の逆運動学によるスクリプト制御です。
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
python examples/robot/rakuda_calvin_table.py \
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
`pip install -e ".[roboverse]"` だけです。RoboVerse 側の変更は要りません。

テーブルの MJCF は生成物なのでリポジトリに入っていますが、作り直すこともできます。

```bash
python -m robopy.sim.calvin_table
#   9 bodies, 4 joints, 60 geoms, 11 meshes
#   4.81 kg, 0.880 x 0.449 x 0.697 m at scale 0.8
#   45 boxes in place of base_link's concave mesh
#   work surface at z = 0.440
#   moving parts: base__button, base__switch, base__slide, base__drawer
```

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
- **テクスチャは入りません。** URDF のフラットな色で描画されます。
- **ブロックの初期位置はランダムではありません。** CALVIN は `initial_pos: any` で作業面に
  ランダム配置しますが、ここでは比較しやすいように決め打ちで並べています。
- **ブロックの質量は CALVIN と違います。** URDF はどれも 1 kg で、PyBullet の
  `globalScaling` は形状だけ縮めて質量は変えないため CALVIN のブロックは実際その重さです。
  `mass=1.0` は渡していますが **MetaSim の MuJoCo バックエンドはこれを見ておらず**、
  実測すると体積 × 既定密度 1000 の **0.0896 kg** になります。把持が成立する理由の
  一つなので明記しておきます。

## 詰まったところ: `base_link` の凹形状

CALVIN の `calvin_table_D.urdf` は `base_link` の `<collision>` を
`concave="yes"` 付きのメッシュで指定しています。PyBullet はこれを尊重しますが、
**MuJoCo はメッシュを必ず凸包に変換します**。この机の凸包は、作業面の手前の縁から
背面パネルの上端まで伸びる「くさび形」になり、机の上に置いたものは
数 cm めり込んだ状態で生成されて弾き飛ばされます。実際、最初はブロックが
`y = -0.93` まで吹き飛んでいました。

そこで、この 1 つの衝突メッシュだけを箱の集合に置き換えています
（`robopy.sim.calvin_table.decompose_to_boxes`）。

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
pytest tests/test_roboverse/test_calvin_table.py -q
# 17 passed
```

箱分解そのもの（重なりがないこと、中空であること、天板の高さが視覚メッシュと一致すること）、
エクスポートしたモデル（スケール・引き出しのストローク・レイキャストした作業面高さ）、
シーン（ロボットが Panda のベース位置に立つこと、300 ステップ後もブロックが机の上に
載ったままで、机の 4 関節が動かないこと）、そして `rakuda.calvin_pick` の立ち位置
（掴める高さであること、対象ブロックが可作業領域に入ること、逆運動学が把持姿勢に
到達すること、そして Panda のベース位置からは**届かない**こと）を検証します。
