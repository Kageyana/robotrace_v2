# 固定カメラによる独立XY基準

## 目的と前提

schema 10の正常な一次ログに固定カメラの独立位置・方位を同期し、まず現在の`x/y`とジャイロ方位の誤差を測る。その後、学習走5本だけで横ICR係数`K_lat`を推定し、別に固定した検証走5本で候補の中点積分を評価する。撮影・解析ではファームウェア、36 Bログ形式、SD設定を変更しない。

実機上面の前後マーカーから機体原点へのオフセット、右マーカーセンサーの位置、マーカー高さは現物で測る必要がある。ログ12678の動画・実測値は`analysis/camera_xy/12678/`に記録した。係数の採否には別走行を含む学習5本と検証5本が必要である。

機体座標はファームウェアの定義に合わせ、初期進行方向を`+Y`、機体右を`+X`、時計回りを正とする。`pathFollower.c`は`x += ds·sin(heading)`、`y += ds·cos(heading)`で積分し、方位を`atan2(dx,dy)`で計算する。

## 機体マーカーと位置オフセット

機体上面の前後に、色が異なる軽いマーカーを同じ前後線上へ固定する。中心間距離と高さをノギス・高さゲージで測る。画像から得る2点の中点から機体原点までの差を、機体座標の右・前方成分[mm]で測る。右マーカーセンサーは受光部中心を基準に、同じ原点からの右・前方成分を測る。

照合に使うファイルは`Machine/robotrace_v2 v86.step`、`robotrace_v2/Core/Src/markerSensor.c`、`robotrace_v2/Core/Inc/markerSensor.h`、`robotrace_v2/Core/Src/pathFollower.c`、`robotrace_v2/Core/Inc/encoder.h`とする。ソースでは右側検出が`RIGHTMARKER`として扱われ、開始時の検出とゴール時の`goalMarkerOnset_p`に使われる。STEPは部品配置の照合用であり、実機の取付誤差を確定しない。受光部中心、マーカー高さ、機体原点の実測値は別に記録する。

計測値は[`camera_setup.json`](../analysis/camera_xy_reference/templates/camera_setup.json)へ記録する。`marker_midpoint_to_origin_*`は2点中点から機体原点、`right_marker_sensor_*`は機体原点から右側受光部までの位置である。`front_marker_lateral_offset_mm`は前側赤丸中心が後側黄丸中心より機体の右へずれた量で、右を正、左を負とする。カメラが検出した2点の線の方位から、この実測オフセットによる角度を引いて機体方位とする。寸法の符号は右・前を正とする。

## カメラ校正

60 fps以上でコース全体を固定撮影する。撮影中にカメラを動かさず、前後マーカーと床の固定基準点が見える照明にする。カメラ座標と床XYの変換は、コース全域に分散させた8点以上の`fit`点から射影変換で求め、別の4点以上の`check`点で検査する。点を8点の一角へ偏らせず、走行領域全体を囲む。4つの検査点は撮影中も動かない固定点に選ぶ。床点と上面マーカーの高さが異なる場合は、両面の高さとカメラの床面からの高さを実測し、上面位置を床面へ投影補正する。

点の中心画素と床座標を[`calibration_points.csv`](../analysis/camera_xy_reference/templates/calibration_points.csv)へ記録する。`floor_x_mm`は初期進行方向に向かって右、`floor_y_mm`は初期進行方向を正とする。独立4点の最大位置誤差が5 mmを超える場合、解析器は参照CSVを出さない。レンズ歪み、反射、マーカー面高さ、校正点の配置を直して撮り直す。校正点の凸包外へ変換を外挿しない。

撮影日、担当者、カメラ・レンズ、取付高さ・傾き、露光、ゲイン、照明、補足条件もカメラ設定の`capture_conditions`へ残す。映像ファイル自体の解像度・コーデック・実測fps・フレーム時刻は診断JSONへ自動記録される。通常は実測60 fps以上を要求する。`allow_isolated_frame_drops=true`の場合のみ、名目60 fps、中央値59.9 fps以上、実平均59 fps以上、長い間隔が全体の0.1%以下、最大間隔が3.05フレーム周期以下の孤立欠落を許す。欠落分の時刻を詰めずPTSをそのまま使う。

端末の回転メタデータがある動画では、画素座標を測る向きとFFmpegのデコード向きを一致させる。符号化フレームの座標で床点・車体マーカーを測る場合は`camera_setup.json`に`decode_without_rotation=true`を指定し、FFmpegの自動回転を止める。既存12678のように表示方向で校正した動画は既定の`false`のまま使う。向きが違う12点座標を流用しない。

固定点を同時追跡し、カメラ移動を検出する。既定は床面換算5 mmを超える移動で無効とする。やむを得ず画角が動いた動画では`camera_motion_mode=register`を指定し、床の独立4点で各フレームを初期フレームへ射影登録する。この4点以外の8点を各フレームの検査に使い、最大残差が5 mmを超えたフレームは無効とする。固定点や機体マーカーの欠測行は`track_valid=0`にし、`x_mm`、`y_mm`、方位を空欄にする。追跡失敗を補間して正解値にはしない。ログ時刻0の基準姿勢だけは、欠測のない隣接フレーム間に限り時刻補間する。

停車して撮影する区間があれば`stationary_check_video_interval_ms`に動画時刻の開始・終了を記録する。診断JSONはその区間の位置RMSと方位RMSを出す。区間を設定しない場合は`configured=false`となる。

## 動画とログの同期

まず動画の全フレームPTSを出す。

```powershell
python analysis/script/camera_xy_reference.py probe F:\path\run.mp4 --output analysis/camera_xy/run/video_timestamps.csv
```

動画を見て、右マーカーセンサーの受光位置がスタート線・ゴール線の先端を通るフレームを選び、その`video_time_ms`を[`sync_events.json`](../analysis/camera_xy_reference/templates/sync_events.json)へ入力する。イベントは機体中心や上面マーカーではなく、センサーの床面投影位置が線へ入る時刻として定義する。

ログの`timing.startResetDelay_ms`は右スタートマーカー検出から走行距離原点のリセットまでの遅れである。動画上のスタート検出時刻をログ時刻`-startResetDelay_ms`へ対応させる。ゴール時刻は`goalMarkerOnset_p`を`encTotalOptimal`の累積パルスと`cntlog`の展開時刻から線形補間する。両者から求めた動画・ログ時刻差が半フレームを超えた場合、時計ずれやイベント選択を確認して撮り直す。ログ時刻はU16の`cntlog`を差分展開して得る。

## 参照軌跡の生成

カメラ設定テンプレートの色範囲はPillowのHSV値（Hは0〜255）で指定する。各色マーカーの初期画素位置、固定点色、センサー位置、前後マーカー寸法を実測・確認した値で埋める。

```powershell
python analysis/script/camera_xy_reference.py build `
  F:\path\run.mp4 F:\path\12656.csv `
  analysis/camera_xy/12656/calibration_points.csv `
  analysis/camera_xy/12656/camera_setup.json `
  analysis/camera_xy/12656/sync_events.json `
  --output-dir analysis/camera_xy/12656
```

出力は次のとおり。

- `tracking.csv`: フレームごとの画素位置、カメラ座標の機体原点、追跡状態、固定点から見たカメラ移動量。
- `reference.csv`: `time_ms`（展開済みログ時刻）、`x_mm`、`y_mm`、`heading_cw_deg`と`track_valid`。有効位置はログ原点と初期方位に合わせる。
- `diagnostics.json`: 校正点誤差、映像条件、同期残差、ログ番号・電圧・commit、使用した機体寸法、欠測区間。

無効な校正・同期・固定点移動があれば`reference.csv`を作らず終了する。追跡欠測だけの場合は有効行を残し、欠測区間を空欄のまま出す。旋回区間の欠測がある走行は係数探索や検証へ使わない。

## 初回走行の現行誤差

まず1走の`reference.csv`と診断JSONが品質ゲートを通ったら、横ICR候補を使わず現行`x/y`とジャイロ方位の誤差を出す。

```powershell
python analysis/script/analyze_camera_xy.py baseline `
  F:\path\12656.csv `
  analysis/camera_xy/12656/reference.csv `
  analysis/camera_xy/12656/diagnostics.json `
  --output analysis/camera_xy/12656/baseline.json
```

校正最大誤差、位置p95・終端誤差、ジャイロ方位p95・終端誤差を確認する。静止ジッター、同期残差、旋回中の欠測は`diagnostics.json`と`reference.csv`に残る。位置検査誤差5 mm超、同期残差が半フレーム超、対象旋回の追跡欠測があれば撮影をやり直す。

## 現行オドメトリと`K_lat`評価

10本の同条件・有効一次走行を集め、manifestテンプレートで最初の5本を`train`、残り5本を`validation`へ固定する。学習側と検証側のログ番号を重複させない。5本揃わない場合、係数採用判断をしない。

横移動窓は100 ms単位で作る。窓の方位差が5度未満、追跡の連続性が切れている、または横移動量が独立校正誤差の`3√2`倍未満の区間は係数へ使わない。速度帯・CW/CCWごとに最低10窓必要とし、`Δ横 ≈ K_lat·Δθ`を原点通過で当てはめる。測定ノイズより小さい横信号は`K_lat`を決める根拠にしない。

学習5走のみで係数を作る。

```powershell
python analysis/script/analyze_camera_xy.py fit `
  analysis/camera_xy/run_manifest.json `
  --output analysis/camera_xy/k_lat_training_model.json
```

学習に使っていない検証5走で、現行schema 10の`x/y`と候補`K_lat`を使った中点積分を比べる。

```powershell
python analysis/script/analyze_camera_xy.py evaluate `
  analysis/camera_xy/run_manifest.json `
  analysis/camera_xy/k_lat_training_model.json `
  --output analysis/camera_xy/k_lat_validation.json
```

結果には検証走ごとの位置誤差p95・終端誤差と、5走間の平均・標準偏差を出す。カメラ方位とジャイロ積分の方位誤差も別に出すため、方位誤差を横移動誤差と取り違えない。横ICR候補はオフライン比較値で、検証5走で現行軌跡より再現性よく改善しない限り採用しない。

## 合成検証と実測の状態

回帰テストは既知の射影変換、前後マーカー順の反転、時計回り符号、ゴールパルス補間、16 bit `cntlog`折り返し、追跡欠測を含む合成60 fps動画を確認する。学習用の合成軌跡では既知`K_lat`の復元と固定検証走での候補誤差を確認する。

ログ12678については実カメラの動画、12点の床座標、機体寸法を取得済みである。独立XYの校正検査点は最大3.62 mmで通過した。動画は1フレームの孤立欠落があり、手ブレ補正後の床点検査残差は全フレームで最大2.96 mmである。スタートとゴールは右センサーが短冊の進入端を通る映像時刻から同期する。実機ログ10本は未取得である。独立XYで改善を確認するまで`pathFollowerUpdatePose1ms()`、EKF、SD設定は変更しない。

追加動画・ログ12679〜12685の結果は`analysis/camera_xy/batch_12679_12685/icr_decision.md`に記録した。両方向の有効トレッドは条件付きで算出できたが、横ICR位置を含む定数5パラメータは採用できず、ファームウェア・SD設定は未変更である。
