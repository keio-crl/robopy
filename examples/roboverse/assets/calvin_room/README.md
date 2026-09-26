# CALVIN の床と空 (`calvin_room`)

`examples/roboverse/tasks/rakuda_calvin.py` が `ScenarioCfg.scene` として読み込む
シーン MJCF です。テーブルやロボットはこの MJCF を根として組み立てられます。

| パス | 内容 |
| --- | --- |
| `calvin_room.xml` | 手書き。床 1 枚とグラデーション天空のみ |
| `textures/checker_blue.png` | [calvin_env](https://github.com/mees/calvin) の `data/plane/checker_blue.png`。1 バイトも変更していません（MIT License, Copyright (c) 2021 Oier Mees。全文は `../calvin_table/LICENSE`） |

## なぜ床だけなのか

CALVIN 本体のシーンは `plane/plane.urdf`（30 m 四方の板）1 枚きりで、それ以外の
背景はありません。PyBullet のビューポートがクリアする色、つまり黒がそのまま出ます。
方策側は手首カメラと固定カメラをテーブル周りに切り出した画しか見ないので、
ベンチマークとしてはそれで正しいのですが、**人間が見るための画にはならない**。

そこで床は CALVIN のものをそのまま使い（タイル 1 辺 0.5 m、`Kd 0.588`。どちらも
`plane.obj` / `plane.mtl` から実測）、その上に空を足しています。

壁は一度入れて外しました。8 m 先に高さ 2.6 m の壁を立てると、多くのアングルでは
画面上端を覆うのに四隅からは覆いきれず、カメラを回すと空の楔形が出たり入ったり
します。壁を高くすると今度は空があった場所が一様な灰色になるだけです。
床と空が直接出会えば地平線になり、継ぎ目もありません。

## ライトはここにない

照明は `ScenarioCfg.lights`（`tasks/rakuda_calvin.py` の `CALVIN_LIGHTS`）にあります。
MuJoCo と Isaac Sim の両方が読む唯一の照明記述がそちらなので、`<light>` をこの
ファイルに書くと MuJoCo でしか効きません。
