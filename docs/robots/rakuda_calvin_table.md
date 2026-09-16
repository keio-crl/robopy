# `rakuda.calvin_table` — CALVIN のシーンに Rakuda を立たせる

`../vapo/scripts/aif_collection_uv.py` がインスタンス化するのと同じ **CALVIN scene D**
（play table + 3 つのブロック）を RoboVerse 上に再現し、Panda のベースがある位置に
Rakuda を立たせた環境です。

<video controls width="960" src="../assets/rakuda_calvin_table.mp4"></video>

![front](../assets/rakuda_calvin_front.png)

```bash
python examples/robot/rakuda_calvin_table.py \
    --stills docs/robots/assets \
    --video docs/robots/assets/rakuda_calvin_table.mp4
# robot base   [-0.34   -0.46    0.4975]  (feet at z=0.240)
# panda base   [-0.34 -0.46  0.24]  from calvin_scene_D.yaml
# table corners are [0.25  0.635 0.796 0.987] m away; the hands reach 0.428 m
```

!!! warning "この配置では Rakuda の手はテーブルに届きません"
    シーンは Panda 用に作られています。Panda はベースから 0.46〜0.81 m の範囲で作業しますが、
    Rakuda の手は取付面から **0.428 m** までしか届きません。
    作業面の最も近い角でもそれより遠いため、**この位置からは作業できません**。

    これは想定どおりです。この環境は「両者を同じスケールで並べて見る」ためのもので、
    解くためのものではありません。Rakuda が実際に作業できる環境は
    [`rakuda.lift_block`](rakuda_lift_block.md) です。

## 使い方

```python
from metasim.task.registry import get_task_class

cls = get_task_class("rakuda.calvin_table")
env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
states, _ = env.reset()
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
| ブロック | `block_red_middle` / `block_blue_small` / `block_pink_big` | 寸法・色・質量 1 kg とも URDF どおり |

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

## テスト

```bash
pytest tests/test_roboverse/test_calvin_table.py -q
# 12 passed
```

箱分解そのもの（重なりがないこと、中空であること、天板の高さが視覚メッシュと一致すること）、
エクスポートしたモデル（スケール・引き出しのストローク・レイキャストした作業面高さ）、
そしてシーン（ロボットが Panda のベース位置に立つこと、120 ステップ後もブロックが
机の上に載ったままであること）を検証します。
