# Yaw EKF 13059–13073 — Phase 1 CAD幾何報告

2026-10-09。**CAD/基板/ファームウェアは変更しない**。学習13059–13068、未学習13069–13073の区別は維持し、今回ログ同定は未実施。

## 原本と抽出法

- `Machine/robotrace_v3.STEP` （STEP AP214; Git blob `a94ccd02d5f4be9ca2228588136bf78a7d22ba2a`、STEP作成2026-10-03 02:58:02）
- `Circuit/robotrace_v2_Linesensor/robotrace_v2_Linesensor.kicad_pcb` (blob `576ad0c79fabc284c8c5c97c628ea8bd67415846`)
- `Circuit/robotrace_v2_main_v2/robotrace_v2_main_v2.kicad_pcb` (blob `40b84b5c7f060fc3e5f28a5fb8ddcb68880e1d82`)、両基板STEPと回路図
- `analysis/script/analyze_yi_step.py`、`analysis/yi_12838_12848/{report.md,sensor_centers.csv,axle_geometry.json,step_solids.csv}`
- `AGENTS.md`、robotrace-log-analysis / robotrace-hardware-review のSkill、`IMU.c/h`、`BMI088.c`、`encoder.c`、`lineSensor.c`、`PIDcontrol.c`、`motor.c`、`control.c`、`timer.c`、`log_schema.h`、`pathFollower.c`、ADC設定`main.c`

`AGENTS.md`の旧 `Machine/robotrace_v2 v86.step` は現行Machineフォルダに存在せず、採用していない。現行STEPのAP214アセンブリ姿勢を読み、KiCad座標から投影した。生STEPアセンブリ変換でライン基板 `R=[[-1,0,0],[0,0,1],[0,1,0]], t=[0,2.09,48.151700]`、メイン基板 `R`同じ、`t=[164.198334,0.975,62.596938]` （CADの変換前にKiCad v座標を反転）。

## 座標系・変換

前後車軸のSTEP Z座標 = -4.845232532, +20.845232625 mm。車軸中点のSTEP原点 = (X,Y,Z)=(0,8.25,8.000000047) mm。

機体座標(Xb,Yb,Zb)は車軸中点原点、前方・右方・上方正、Yaw CW正。CAD(Xc,Yc,Zc)から:

```text
 Xb =  Zc - 8.000000047
 Yb = -Xc
 Zb =  Yc - 8.25
```

KiCadライン基板の(u,v,w)から:

```text
 Xb = 40.151700071 - v
 Yb = u
 Zb = -6.160000 + w
```

よってKiCad→機体の同次変換行列:

```text
[ 0 -1  0  40.151700071 ]
[ 1  0  0   0           ]
[ 0  0  1  -6.160000    ]
[ 0  0  0   1           ]
```

メイン基板のKiCad→機体:

```text
[ 0 -1  0   54.596938071 ]
[ 1  0  0 -164.198334    ]
[ 0  0  1   -7.275000    ]
[ 0  0  0    1           ]
```

KiCad裏面実装のパッケージ実面・光学面は `w=0` のfootprint面とは異なる。

## 10chライン受光フットプリントとLED

`sensor_geometry.csv` を参照。ADC1 rank = [1,0,13,12,11,6,5,4,3,2]、main J4、反転FFC、line J1のnetを照合し、ch0=Q10（左端）、ch9=Q9（右端）と確認。Q位置の前方Xは外端74.222 mmから中央93.152 mmまで18.930 mmの差。**旧単一L=30 mm/等間隔3 mmは棄却する。**

Q–D中心距離約4.93 mm。中央Q X=93.152 mm、対応LED X=98.012 mm、幾何中点X=95.582 mm。過去の「中央95 mm実測」は床上の実効光学検出中心を測った可能性があるが、**幾何中点=実効中心は未証明**。TEMT7100X01の半感度角±60°、HIR19-21C/L11/TR8の全視野角145°、高さ、反射、正規化、白線幅と閾値に影響される。CROSSはEKFライン観測に使用しない。

## 車輪、BMI088、高さ

- 前後車軸間隔 25.690465 mm。4輪中心は機体前方X=±12.845 mm、右方Y=±54.55 mm。左右幾何間隔 109.10 mm。**25.69 mmはエンコーダYawのトレッドではない**。有効トレッドは別途同定が必要。
- タイヤCAD直径約23.0 mm、コード仕様23.5 mm。平坦な床面の想定Z=-11.50 または -11.75 mm。Q footprint面Z=-6.16 mm→床上約5.34 または5.59 mm。荷重時と受光面高さは未確定。
- 主基板U8のfootprintおよびSTEP形状(3.0×4.5×0.95 mm)からBMI088パッケージ中心 X=-31.675 mm、Y=-0.099 mm、Z=-5.205 mm。コード `IMU_OFFSET_Y_M=-0.03168` mにほぼ一致。IMUダイ内部中心とパッケージ中心は区別する。
- `IMU.c` は加速度に旋回中心オフセット補正を既に適用。記録 `acceleVal_X` にその補正を二重適用しない。Yaw角速度は搭載位置によらない。
- STEP基板の公称pitch/roll/yaw取り付け角は0°。実機のねじれは別。pitch 1°で前方距離93.152 mmの点の高さは車軸支点仮定で約1.626 mm変化。roll 1°で左右±41.7 mmの端の高さは約±0.728 mm変化。

## 感度、採用方針

L=95とCAD受光中央93.1517の差1.8483 mmはYaw角δ=10°, 30°, 45°で横方向予測差約0.321, 0.924, 1.307 mm。ただし95mmの**測定原点未判明**。ライン面内の取り付けyaw1°も中央付近で約1.63mmの横ずれとなり得る。右方位置を使用する非線形10ch観測をPhase 3で検討し、CAD値・中点値・±1～3mmの感度を比較する。

独立真値のない段階でYaw誤差改善率を確定しない。B/C/DのEKF結果、未学習ログ比較、STM32処理時間は今後のフェーズ。再計算スクリプトは `analysis/script/recompute_yaw_13059_13073_phase1.py`。測定手順は `measurement_request.md`。
