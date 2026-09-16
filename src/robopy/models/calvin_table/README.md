# CALVIN プレイテーブル（`calvin_table_D`）

[CALVIN](https://github.com/mees/calvin) の `calvin_env` から、シーン D のプレイテーブルを
そのまま持ってきたものです。RoboVerse 側で Rakuda と同じシーンに置くために使います。

| パス | 内容 |
| --- | --- |
| `urdf/calvin_table_D.urdf` | 上流のまま。8 リンク、可動関節 4（すべて prismatic） |
| `meshes/*.obj`, `*.STL` | URDF が参照する 11 個だけ。348 KB |
| `mjcf/calvin_table.xml` | 生成物。`python -m robopy.sim.calvin_table` で作り直せます |
| `LICENSE` | MIT License（Copyright (c) 2021 Oier Mees）。上流のまま |

**変更点**: ファイルは 1 バイトも変更していません。

## 可動部

| 関節 | 種類 | 可動域 |
| --- | --- | --- |
| `base__button` | prismatic | 0 〜 0.025 m |
| `base__switch` | prismatic | 0 〜 0.11 m |
| `base__slide` | prismatic | 0 〜 0.35 m（スライド扉） |
| `base__drawer` | prismatic | 0 〜 0.275 m（引き出し） |

`calvin_scene_D.yaml` は `base__switch` を "Revolute" とコメントしていますが、
URDF 上は prismatic です。ここでは URDF に従っています。

## 持ってきていないもの

- **テクスチャ**（`dark_wood.png` など、3.6 MB）。URDF の `rgba`（黒・灰・白）で描画しています。
- **ライト／LED の挙動**。`calvin_scene_D.yaml` の `effect: led` / `effect: lightbulb` は
  URDF ではなく calvin_env の Python 側の処理です（ボタンを押すと LED、スイッチで電球）。
  `led_link` / `light_link` のジオメトリはありますが、光りません。

## 衝突形状の注意

可動部（switch, slide, drawer）は上流が用意した `*_vhacd2.obj`（凸分解）を衝突に使います。
一方 **`base_link` は視覚メッシュをそのまま衝突にも使っており**、凸分解がありません。
PyBullet は静的物体の凹メッシュをそのまま扱えますが、**MuJoCo は凸包にします**。
その結果、MuJoCo 上ではテーブル本体が中身の詰まった塊になり、引き出しの中や
スライダ棚の内部に物を入れることはできません。

**眺める・並べる用途には十分ですが、収納を伴う操作タスクには足りません。**
必要になったら `base_link.obj` を凸分解して差し替えてください。
