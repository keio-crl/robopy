# robopy — プロジェクト固有の注意（Rakuda 実機まわり）

コードから読み取れない、実機で痛い目を見て学んだことだけを書く。一般的な構成は `docs/robots/rakuda.md`。

## 校正値・可動域は「一本の経路」しかない

- モータの count → 関節角は `JointCalibration.count_to_rad`（`control.follower_joint_calibration` の
  `zero_count` / `direction` / `urdf_joint`）**だけ**。viewer の `--mirror-follower`、`robopy-vr --hardware`、
  `ServoLoop` は全部これを通る。関節の可動域も `ModelBundle` / `rakuda_control` の
  `limit_profile`（URDF ∩ `control.model.soft_limits_rad`、`joint_limit_overrides` は広げる）で共通。
- したがって「viewer では合うのに VR / 実機制御では違う」ときに、校正の読み方を疑って別実装を足さない。
  差が出るのは **判定の厳しさ**（viewer は表示するだけ、IK・サーボは範囲外を拒否する）か、
  経路の外にあるもの（ヘッドの neutral、開始姿勢、操作座標系）である。まずログの数値と
  config の数値を突き合わせて、どの層が拒否しているかを特定する。

## 可動域は「止まっている実機」が必ず満たす値にする

- `soft_limits_rad` は「測った travel − 2° マージン」で書かれる。実機はストッパに当たって休むとき
  travel をさらに 1° ほど超えて読める（例：右肘が伸び切りで 0.2439、travel 上限 0.227、soft 0.1921）。
  つまり **soft limit は静止した実機に破られる前提**で扱うこと。
- IK が「保持中（動かしていない）関節が範囲外」を理由に**全体を infeasible** にすると、片腕のせいで
  両腕とも一切動かなくなる。保持関節の範囲外は「限界に置いて解く」か「内向きにだけ動かす」で処理し、
  範囲外の警告は出しても解は返す。`HELD_JOINT_LIMIT_TOLERANCE_RAD` のような固定の許容値で
  ごまかすと、値が数 mrad 超えただけで再発する（今回 0.0518 > 0.05）。
- 校正を変えたら（zero, direction, travel, URDF 幅）、実機を脱力／静止させた状態で **制御に使う
  limit profile で全関節が範囲内か**を確かめる。viewer で見た目が合うことは確認にならない。

## IK のプロファイルは一つ（`teleop_solver_settings`）。別の場所に既定値を書かない

- viewer の `IKSetup`、VR シミュレーション、実機の `rakuda_control.build_model_and_ik` は全部
  `kinematics/dual_arm_ik.py` の `teleop_solver_settings` から `DualArmIKConfig` を組み、その上に
  `control.ik` を乗せる。以前は実機側が `DualArmIKConfig` の素の既定値（weighted・pose・向き重み 0.15）で
  動いていて、シミュレーション（hierarchical・position_only）で検証した挙動と別物だった。
  「シミュレーションでは腕が理想的なのに実機は追従が悪い」が出たら、まず両方の起動ログの `IK` 行を
  突き合わせる。ソルバの既定値を変えるときは `teleop_solver_settings` だけを触る。意図した差は一つだけ:
  実機は `gain_time_constant_s=None`（参照ガバナ `control.trajectory` が加減速を担うので二重の遅れを作らない）。
- `robopy-vr --orientation-mode / --orientation-weight` はシミュレーションと `--hardware` の両方に渡る
  （実機側は `cfg.control.ik` に書き込んでからシステムを組む）。position_only では重みは無意味。
- 既定の向きモードは `axis_aligned`（位置＋グリッパの接近軸、軸まわりのロールは自由）。接近軸は左右で違う
  （TCP フレームの向きが左右で別）ので `approach_axis_tcp` は `{left:, right:}` の形も取れる。未設定なら
  `approach_axes_from_model`（手首ピッチ関節→TCP 原点、ゼロ姿勢）で読む。TCP はまだ placeholder
  （validated false）なので、接近軸も「モデルの手の長軸」であって実測ではない。
- 向き（axis_aligned / pose）は hierarchical の**第二段**に置く（`orientation_priority: secondary`）。第一段に
  置くと、上腕ロールと前腕ロールを ±90° 逆回しにした形でも指す方向がほぼ同じため、一度巻いたロールは
  巻き戻すと指す方向がわずかに変わる＝第一段が禁じるので、ロールが ±90° のまま固まる（実機で確認済み）。
- グリッパはその腕のクラッチ中だけトリガに追従し、離すと最後の値を保持する（`ArmTeleop._last_gripper`）。
- 素手の既定（robopy-vr の CLI 既定）はクラッチ＝中指・薬指・小指の握り込み（`grip`）、グリッパ＝親指と人差し指の
  距離（`pinch`）。`HandTrackingConfig` 自体の既定は旧方式（pinch / curl）のままで、既存テストはそれを前提にしている。
  グリップ中は中指ピンチ（リセンター・録画）を読まず、両拳の終了サインも無効（拳＝グリップ）。
- Rakuda の腕は 6 軸で、`elbow_yaw_*`（上腕ロール、軸は 20° 傾斜）と `wrist_yaw_*`（前腕ロール）が
  ロール軸。position_only では 3 自由度の零空間にこの 2 軸が入り、姿勢コストが無いと左右対称に外向きへ
  ±30〜70° 流れる。pose モードでは向きを保つためにこの 2 軸を 45〜65° 回し、位置誤差が数 cm 残る。
  プロファイルはロール軸に `posture_cost` 0.3・基準 0 を掛けている（`ROLL_POSTURE_COST`）。
- 保持中（動かしていない）関節が限界を超えて読めても IK は止まらない: その関節を限界に置いて解き、
  限界値を指令して内側へ戻す（警告は一度）。動かしている関節の限界超過も内向きの動きだけ許して続行する。

## direction / zero を触るときの整合

- direction を反転したら、`lower_limit_rad` / `upper_limit_rad`（測った travel）と対応する
  `soft_limits_rad` を**同時に**符号反転して入れ替える。zero を動かしたら両方を同じ量ずらす。
  URDF を広げた関節は URDF と MJCF も一緒に直す。config の `note` に何をしたか残す。
- **方向の正否はミラー（`robopy-viewer --mirror-follower`）で関節を手で動かして確かめる以外に決めない。**
  「記録した travel が URDF の範囲に収まるか」で推定しない: 右 wrist_pitch は travel が URDF 上限を
  0.6 rad 超えるが、方向は正しかった（URDF の範囲の方が実機より狭い）。推定で反転して外した実例あり。
  前腕ロールが 90° 近く回っていると手首ピッチの向きは目視で横になり、逆と誤認しやすい。
- 片方だけ直すと、URDF ∩ soft が空になったり、実機が届く範囲が禁止域になったりして、
  症状は「追従しない」「ゼロがずれて見える」として現れる（校正自体は正しいことが多い）。

## 名前の対応

- モータ名（`r_arm_el_yaw`, `r_arm_wr_roll`, `r_arm_sh_pitch2` …）と URDF 関節名
  （`elbow_pitch_right_dof`, `wrist_yaw_right_dof`, `elbow_yaw_right_dof` …）は**名前で対応しない**。
  対応は config の `urdf_joint` が唯一の正。ログ・報告では必ず両方を併記する。

## 実機の運用メモ

- FTDI の `latency_timer` が 16 ms のままだと 17 モータの一括読み出しが時々 50 ms 予算を超える。
  超えた読み取り（タイムアウト・通信エラー・サンプル時刻の広がり過多）は 1 回ならその周期を指令なしで
  飛ばし、`max_slow_reads`（既定 5）回連続で初めてフォールトする。根本対策は起動前に
  `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`。
- 変更後の確認は `robopy-vr --hardware --hardware-check`（トルクを入れずに構成だけ検証）。
