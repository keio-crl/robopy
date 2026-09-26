# CALVIN プレイテーブル（`calvin_table_D`）

[CALVIN](https://github.com/mees/calvin) の `calvin_env` から、シーン D のプレイテーブルを
そのまま持ってきたものです。RoboVerse 側で Rakuda と同じシーンに置くために使います。

| パス | 内容 |
| --- | --- |
| `urdf/calvin_table_D.urdf` | 上流のまま。8 リンク、可動関節 4（すべて prismatic） |
| `meshes/*.obj`, `*.STL` | URDF が参照する 11 個だけ。348 KB |
| `meshes/*.mtl` | 上流のまま。OBJ が `mtllib` で指すマテリアル 5 個 |
| `textures/*.png` | 上流のまま。MTL が `map_Kd` で指す 2 枚、730 KB |
| `mjcf/calvin_table.xml` | **生成物**。MuJoCo バックエンドが読む |
| `urdf/calvin_table_scaled.urdf` | **生成物**。それ以外のバックエンド（Isaac Sim など）が読む |
| `LICENSE` | MIT License（Copyright (c) 2021 Oier Mees）。上流のまま |

**変更点**: 上流から持ってきたファイルは 1 バイトも変更していません。
生成物 2 つは `python examples/roboverse/calvin_table_asset.py` で作り直せます。

## 生成物が 2 つある理由

MetaSim で MJCF を読むのは MuJoCo だけです。Isaac Sim / PyBullet / SAPIEN /
Genesis は URDF を読みます（Isaac Sim は Isaac Lab の `UrdfConverter` で一度
USD に変換してキャッシュします）。同じ机が両方に要るので、両方を**同じ前処理**
（`calvin_table_asset._prepared_urdf`）から出しています。

上流の URDF をそのまま渡さないのは 2 点のためです。

- **スケール**。`ArticulationObjCfg.scale` は Isaac Sim の `UsdFileCfg` には
  届きますが MuJoCo の MJCF ローダには届きません。片方だけ 0.8 倍になって
  しまうので、両方に焼き込んでいます。
- **`base_link` の衝突形状**（後述）。PyBullet 以外はメッシュを凸包にするので、
  箱分解に置き換えたものを両方に入れています。

`tests/test_roboverse/test_calvin_table.py::TestUrdfExport` が、2 つのファイルが
同じ 9 bodies / 4 joints / 4.81 kg / 天板 z=0.440 / 同じ寸法になることを毎回
確認します。

## 可動部

| 関節 | 種類 | 可動域 |
| --- | --- | --- |
| `base__button` | prismatic | 0 〜 0.025 m |
| `base__switch` | prismatic | 0 〜 0.11 m |
| `base__slide` | prismatic | 0 〜 0.35 m（スライド扉） |
| `base__drawer` | prismatic | 0 〜 0.275 m（引き出し） |

`calvin_scene_D.yaml` は `base__switch` を "Revolute" とコメントしていますが、
URDF 上は prismatic です。ここでは URDF に従っています。

## テクスチャ

MuJoCo の URDF インポータは `mtllib` を読みません。そのため最初は木目がまったく
出ず、机が灰色の塊に見えていました。エクスポータが MTL を自分で読んで
MuJoCo の `<texture>` / `<material>` を書き出します。

| リンク | MTL の `map_Kd` |
| --- | --- |
| `base_link`, `plank_link` | `dark_wood.png` |
| `slide_link`, `drawer_link` | `dark_wood__gray_handle.png` |
| `switch_link` | なし。URDF の灰色 (`rgba 0.4 0.4 0.4`) のまま |
| `button_link`, `led_link`, `light_link` | STL なので UV がない。URDF の色のまま |

上流が持っている 9 枚のうち、この机が実際に使う 2 枚だけを持ってきています
（明るい木目や黒ハンドルの版は `calvin_table_A/B/C` 用）。

Isaac Sim 側はこの回り道が要りません。URDF → OBJ → MTL → PNG を素直に辿ります。

## 持ってきていないもの

- **ライト／LED の挙動**。`calvin_scene_D.yaml` の `effect: led` / `effect: lightbulb` は
  URDF ではなく calvin_env の Python 側の処理です（ボタンを押すと LED、スイッチで電球）。
  `led_link` / `light_link` のジオメトリはありますが、光りません。
- **`calvin_table_A/B/C`**。シーン D だけです。

## 衝突形状の注意

可動部（switch, slide, drawer）は上流が用意した `*_vhacd2.obj`（凸分解）を衝突に使います。
一方 **`base_link` は視覚メッシュをそのまま衝突にも使っており**、凸分解がありません。
PyBullet は静的物体の凹メッシュをそのまま扱えますが、**MuJoCo は凸包にします**。
その結果、MuJoCo 上ではテーブル本体が中身の詰まった塊になり、引き出しの中や
スライダ棚の内部に物を入れることはできません。

これは箱分解を入れる前の話です。いまは `decompose_to_boxes` が `base_link` の
衝突メッシュを 45 個の軸平行な箱に置き換えているので、机はちゃんと中空で、
引き出しの中にもスライダ棚の中にも物を入れられます。詳細は
[`docs/robots/rakuda_calvin_table.md`](../../../../docs/robots/rakuda_calvin_table.md) を参照。

その箱は geom group 3 に入れてあります。MuJoCo は group 0/1/2 を描くので、
group 0 のままだと 45 個の白い箱が木目メッシュの上に重ね描きされます
（それが「灰色のブロックの山」の正体でした）。物理は変わりません。
