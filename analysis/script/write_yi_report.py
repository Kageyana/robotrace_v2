"""Write an inspectable analysis memo from the saved source-backed result tables."""
from pathlib import Path
import csv,json,numpy as np
ROOT=Path(__file__).resolve().parents[2];P=ROOT/'analysis/yi_12838_12848'
def read(name):
 with (P/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def link(name,label=None):return f'[{label or name}]({(P/name).as_posix()})'
def image(name):return f'![{name}]({(P/(name+".png")).as_posix()})'
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(str(x) for x in r)+' |' for r in rows])
health=read('health.csv');ind=read('individual_optima.csv');sum0=read('anchor_summary.csv');sume=read('extended_summary.csv');ms=read('model_sensitivity.csv');v=json.loads((P/'validation.json').read_text())
text='''# IMU前後オフセット yI の独立検証 — 12838～12848

検証日: 2026-10-03（日本時間）。対象: CW 12838～12842、CCW 12844～12848。ファームウェアの変更・書き込みは行っていない。

**推奨は自己位置推定の `yI=0 mm` 維持。`-63 mm` と `-20.5 mm` の採用は見送る。**

従来の `-20.5 mm` 最小点は再現したが、CROSS確定位置を基準にした対応点の前後ずれを吸収する値だった。ラインセンサーの独立検出とSTEPの素子位置から車軸中点の通過時刻へ補正すると、右端gyro積分の最小点は `+3.5 mm`、台形積分では `+1.0 mm` になった。これはIMU物理位置の同定ではない。採用値を+1 mmへ変更する根拠もまだ弱い。

## 1. 入力と10ログの健全性

既存CSVの `x`, `y`, `ROC`, `encTotalOptimal`、経路生成値は解析入力にしていない。読んだ値は `cntlog`, `encTotalL/R`, `gyroVal_Z`, `courseMarker`, `slipFlag/Lat`, `lSensorCari0..9` のみ。ヘッダの閉路判定は状態の説明に引用するが、座標の正解や最適化制約には使っていない。入力ファイルのSHA-256は health.csv に保存した。

'''
text+=table(['ログ','方向','行数','cntlog最大間隔 [ms]','CROSS/独立ライン検出','電圧 [V]','閉路判定'],[[r['log'],'CW' if int(r['log'])<12843 else 'CCW',r['rows'],r['max_gap_ms'],r['CROSS']+'/'+r['line_events'],r['battery'],r['closureValid']+' (reason='+r['closureReason']+')'] for r in health])
text+='''

全10本で `emcStop=0`、cntlog単調増加、行数とlogExpectedRows一致、最大間隔6 ms、CROSS6回、ライン検出6回、SLIPフラグ両種0、ヘッダsgMarkerAtLogEnd=7。ユーザーの完走申告とも整合する。距離増分と1 m/s付近の走行に対して大欠落を認めなかった。SLIPフラグ0は物理的スリップ不存在の証明ではない。

CW5本は `closureValid=0, closureReason=8`、CCW5本は閉路有効。**走行ログとして解析可能であることと、現行PATH経路元の受入条件を満たすことは別であり、CW5本は経路元として使えない。** この検証で閉路判定を書き換えてはいない。

CW電圧8.35→8.23 V、CCW8.17→8.06 V。走行順・電圧の差があるため方向だけを無条件に同一条件と扱わない。`6697266` の単体コミットはSTEP更新のみ。c727dfbと6697266間でIMU、マーカー、ライン、制御の対象処理に差はなく、途中のログスキーマ更新は存在する。GPIO左右反転はユーザー申告を解析条件として優先した。

## 2. 車軸・サイドマーカーの幾何

参照したのは現行 `Machine/robotrace_v3.STEP`、サイドマーカーのKiCad PCB/回路図、ラインPCB、メインPCB、`.ioc`、関連Coreソース。ユーザーは走行時から取り付け位置を変更していないと確認した。STEPはSolidWorks AP214、2026-10-03出力。OpenCascade等はこのPython環境にないため、AP214のNAUO、SHAPE_DEFINITION_REPRESENTATION、ITEM_DEFINED_TRANSFORMATIONを読み、部品から機体座標への配置変換を追跡した。部品境界はトポロジー頂点を使用し、面の任意原点やスプライン制御点を寸法として扱わない。

STEPの前後方向はZ、鉛直はY、左右はX。前後車軸中心のZは `-4.845233`, `20.845233 mm`、その中点は `8.000000 mm`。車軸間隔は `25.690465 mm`。ファームのIMU_OFFSET_Y_Mのコメントが示す基準点（前後車軸中点）に合わせた。

| センサー | 車軸中点から前方 | 前車軸から前方 | 後車軸から前方 |
| --- | --- | --- | --- |
| U1 パッケージ中心（左右同じ前後位置） | 38.326 mm | 25.480 mm | 51.171 mm |
| U2 パッケージ中心（左右同じ前後位置） | 48.326 mm | 35.480 mm | 61.171 mm |

PCBのU1 `(149.7,98.25,90°)`、U2 `(169.7,98.25,90°)` をSTEPの基板配置へ変換し、パッケージ頂点座標とも一致することを確認した。基板は平面内で30°傾いているため、PCB上の20 mm差は機体前後方向では10 mm差になる。左右U1同士・U2同士の前後差は計算精度内で0。

ユーザー確認: **実機右はU2のみ、左はU1・U2のOR**。CROSSへの先行進入は左右とも前方のU2が支配する。ただし左側ORは検出終了を伸ばすので、マーカー確定時刻がパッケージ中心通過そのものになるとは限らない。CCWのGPIO左右反転も反映し、1/2マーカーの方向比較は行わなかった。

**確定した数値はパッケージ・フットプリント中心の幾何座標。床上の実効反射検出点は推定であり、48.326 mmを実測検出中心と断定しない。** LBR-123Fには発光部・受光部が並列にあり、光学距離と閾値で検出点が変わる。メーカー図面では小型3.4×2.7 mmパッケージであり、数十mmの差をパッケージ内位置だけで説明することはできない。[Letex LBR-123Fデータシート](https://www.brightekeurope.com/productcart/pc/catalog/LBR-123F.pdf)。後車軸基準ならU2は約61.2 mmとなるが、その値を車軸中点基準のyIへ流用してはいけない。

'''+image('sensor_geometry')+'''

## 3. 独立したラインCROSSアンカー

全ログを走査し、**courseMarkerの値・近傍窓を使わずに**10chのうち8ch以上が1000を超える連続区間を検出した。8番目に高い値の閾値交差を距離方向に線形補間し、進入・退出位置の中点を用いた。区間幅3～80 mmという形態条件を設けた。全10ログでそれぞれ6イベントを検出し、各イベントでは10ch同時反応も存在する。通常の1～2chのライン追従とは分離できる。

全幅反応の中央はマーカー確定位置よりCWで平均56.748 mm前、CCWで平均61.738 mm前。ただしライン配列が円弧状なので、**全幅反応中央も車軸が交差線を通過した位置ではない**。総和ピークは中央chの通常ライン反応や飽和に影響されるため採用しなかった。

次に各イベントの外側8ch（0～3,6～9）について同一閾値の進入・退出中点を個別に求めた。中央4/5は通常ライン反応が高く、独立な退出が不明瞭になるので除いた。STEPとTEMT7100X01のフットプリント中心から各chの前方距離を求めた。左右の順序に対して前後距離は鏡対称なので左右符号の違いは傾きの符号だけに作用する。

前方距離は外端から `74.222,81.032,86.572,90.652,93.152 mm`（左右鏡対称）。ch毎の中点sに前方距離Fを加え、`s_center(ch)+F(ch)=s_axle + b*x(ch)` を最小二乗で当てはめた。切片s_axleを車軸中点の交差線通過アンカーとし、傾きで交差線に対する進入角の影響を吸収した。局所的に直線走行し、交差線が直線という仮定を使う。全60イベントのch整合RMSは平均0.661 mm、最大1.079 mm。左右鏡像でも切片は同じになる。

全幅中央→車軸中点通過の補正は平均83.472 mm（82.823～84.278 mm）。マーカー確定時点は推定車軸中点通過よりCWで平均26.338 mm前、CCWで平均22.120 mm前だった。この前後差が、従来の約-20 mmという最小点と整合する。

閾値800/1000/1200/1500/2000を変えても、幾何補正後の右端積分の最適yIは+3.5～+4.0 mm、最小RMS7.660～7.663 mm。8ch整合が良いので同じ物理交差線を拾っている説明を支持するが、光学応答・センサー時刻・CAD原点の精度は残る。

'''+image('line_cross_profiles')+'''

## 4. 積分とCW/CCW対応

距離換算は58.019 pulse/mm、kL=0.99675455、kR=1.00324545を固定した。先頭ログ行の累積値を引き、先頭をs=0とした。右端積分は依頼文どおり `dtheta=gyroVal_Z[i]*pi/180*dt`, `mid=theta+dtheta/2`, `dlat=-yI*dtheta`。dsは左右補正距離の平均。座標はこの値だけから全行再積分した。セグメント終端を強制的に閉じる補正も行っていない。

同じ物理CROSSの隣接検出間を1周とした。CW/CCWのスタート→第1CROSSまでの距離は方向で違うので対応に使わない。各ログの1～6番目CROSSから5区間を作り、指定ペアの同じ区間番号を比較した。CCWの点順を逆転し、エンコーダ距離の弧長比で1001点へリサンプル。鏡映と拡大縮小を禁止した最適2D回転・並進で位置合わせした。RMSは各区間の二乗位置誤差を等点数でプールして平方根を取る。セグメント1～5はCROSS基準の周回番号であり、スタートマーカー基準の絶対ラップ境界ではない。

車軸アンカーの周長はCW平均4726.564 mm、CCW平均4748.227 mm（差21.663 mm、約0.46%）。方向で走行線や滑りが同一とは限らない。全CW×全CCW25組（125セグメント）でも、マーカー最適-20.5 mm/RMS7.773 mm、車軸最適+3.5 mm/RMS7.670 mm。共有ログを再利用しており125独立試験とは解釈しない。

## 5. 0 / -63 / 最適値の比較

| アンカーと積分 | yI=0 RMS | yI=-63 RMS | yI=-20.5 RMS | 探索最小点 / RMS |
| --- | --- | --- | --- | --- |
| マーカー確定・右端 | 32.613 mm | 65.504 mm | 7.758 mm | -20.5 mm / 7.758 mm |
| ライン全幅中央・右端、幾何未補正 | 120.675 mm | 30.466 mm | 91.091 mm | -82.5 mm / 10.489 mm |
| ライン→車軸中点補正・右端 | 9.550 mm | 102.800 mm | 38.055 mm | +3.5 mm / 7.663 mm |
| ライン→車軸中点補正・台形 | 7.891 mm | 99.063 mm | 34.327 mm | +1.0 mm / 7.674 mm |

基準アンカーによって違う点対応を比較しているため、行間のRMSだけでどのアンカーが正しいかは選ばない。幾何で同じ基準点へそろえた結果を採用判断に使う。グリッドはマーカー-40～0 mm、0.5 mm刻み、範囲外だったラインおよび車軸は-120～+20 mmへ拡大した。台形の探索は-30～+15 mm、0.5 mm刻み。

'''+image('rms_vs_yi')+'''

## 6. CROSS端部除外

端部を除外した後、残った点に再度剛体位置合わせした。値は再計算結果なので、過去の点数・集計差による0.01～0.03 mm程度の差がある。

'''
rows=[]
for kind,src in [('marker',sum0),('axle',sume)]:
 for r in src:
  if r['anchor']==kind:rows.append([kind,f"{float(r['trim'])*100:g}%",f"{float(r['rms_0']):.3f}",f"{float(r['rms_neg63']):.3f}",r['yi_best'],f"{float(r['rms_best']):.3f}"])
text+=table(['アンカー','両端除外','0 RMS [mm]','-63 RMS [mm]','最適yI [mm]','最小RMS [mm]'],rows)
text+='''

端部除外で従来最小点が残った事実は再現した。しかし弧長位相のずれは全周の点対応に作用するため、端部除外だけでは交絡を排除できない。

## 7. 指定5ペアと周回別

'''
rows=[]
for i in range(1,6):
 a=next(r for r in ind if r['anchor']=='marker' and r['unit']==f'pair{i}');b=next(r for r in ind if r['anchor']=='axle_extended' and r['unit']==f'pair{i}');rows.append([a['cw']+' / '+a['ccw'],a['yi_best'],f"{float(a['rms_best']):.3f}",b['yi_best'],f"{float(b['rms_best']):.3f}"])
text+=table(['CW/CCWペア','マーカー最適yI','マーカーRMS [mm]','車軸最適yI','車軸RMS [mm]'],rows)+'\n\n'
rows=[]
for i in range(1,6):
 a=next(r for r in ind if r['anchor']=='marker' and r['unit']==f'lap{i}');b=next(r for r in ind if r['anchor']=='axle_extended' and r['unit']==f'lap{i}');rows.append([i,a['yi_best'],f"{float(a['rms_best']):.3f}",b['yi_best'],f"{float(b['rms_best']):.3f}"])
text+=table(['周回','マーカー最適yI','マーカーRMS [mm]','車軸最適yI','車軸RMS [mm]'],rows)
text+='''

## 8. 個別25区間のばらつき

0.5 mmグリッド。以下の範囲は観測された最小点の散らばりであり、信頼区間ではない。

'''
for kind,label in [('marker','マーカー確定: -19～-22 mm'),('axle_extended','車軸補正: +3.0～+4.5 mm')]:
 rows=[]
 for i in range(1,6):rows.append([f'Pair{i}']+[next(r['yi_best'] for r in ind if r['anchor']==kind and r['unit']==f'pair{i}_lap{j}') for j in range(1,6)])
 text+=label+'\n\n'+table(['ペア','Lap1','Lap2','Lap3','Lap4','Lap5'],rows)+'\n\n'
text+='''## 9. アンカーとyIの交絡を直接確認

`f(theta)=(sin(theta),cos(theta))` とすると、連続時間で提示式は

`p_yI = p_0 - yI * (f(theta)-f(theta_start))`

となる。つまり `yI=-20 mm` は、追跡点を車軸基準から前方20 mmへ移した座標を作る。剛体上のgyroの角速度は測定位置に依存しないため、gyroとエンコーダだけの積分へIMUの物理前後距離を入れる必然性はない。加速度には旋回中心ずれが作用するので別問題である。

マーカーのアンカー距離を前後へずらして同じ探索を行うと:

'''
text+=table(['アンカー移動 [mm]','最適yI [mm]','最小RMS [mm]'],[[r['anchor_shift_mm'],r['yi_best'],f"{float(r['rms_best']):.3f}"] for r in read('phase_sensitivity.csv')])
text+='''

20 mm前へ移すだけで-20.5→-0.5 mmとなりRMSは7.758→7.773 mm。このほぼ1対1の移動が、最適値の安定性を物理オフセットの証拠にできない直接の理由。アンカーをラインへ変えたことで最小点が約-83 mmへ変わり、そのライン素子前方距離約83.5 mmを引き戻すと0付近へ戻った。

'''+image('phase_confounding')+'''

## 10. 軌跡と残差分布

代表ペア12838/12844のLap3を、マーカーと車軸アンカーでそれぞれ比較した。全走行軌跡も保存した。全走行図の各開始原点は別であり、そのまま絶対位置を比較する図ではない。

'''+image('trajectory_comparison')+'\n\n'+image('full_trajectories')+'\n\n'+image('residual_distribution')+'\n\n'+image('residual_map')+'''

マーカー最適-20.5と車軸最適+3.5では、弧長約74.9%にそれぞれ約20.0/20.3 mmの最大残差が残る。約58～60%、68～69%、87～89%にも局所的な山がある。CROSS端部は支配的ではなく、単純な周回後半への単調増大でもない。

| 区間分類 | マーカー・-20.5 | 車軸・+3.5 | 車軸・0 |
| --- | --- | --- | --- |
| 直線: abs(gyro)<30 deg/s | 6.783 mm | 6.478 mm | 6.497 mm |
| 遷移: 30～150 deg/s | 8.749 mm | 9.319 mm | 12.530 mm |
| 旋回: abs(gyro)>=150 deg/s | 8.074 mm | 7.756 mm | 10.191 mm |
| 両端100 mm | 5.870 mm | 5.608 mm | 7.233 mm |
| 接線方向成分 | 4.218 mm | 4.011 mm | 6.593 mm |
| 法線方向成分 | 6.511 mm | 6.529 mm | 6.909 mm |

分類はCWの角速度による診断で、端部は他分類と重なる。法線方向が残差の大きな成分。CW/CCW走行線の違い、カーブとその出口での滑りや追従差、局所gyro誤差が候補。RMSとSLIPフラグだけでは因果を分離できない。周長差0.46%、電圧差もあるため、単一の定数を増やしてこの残差を無理に消さない。

## 11. 左右エンコーダスケールの交絡

依頼式の追加srelを-1～+1%、0.1%刻みで走査し、各srelでyIを再最適化した。

'''
text+=table(['アンカー','srel','最適yI [mm]','最小RMS [mm]'],[[r['anchor'],f"{float(r['value'])*100:g}%",r['yi_best'],f"{float(r['rms_best']):.3f}"] for r in ms if r['test']=='encoder_relative' and abs(float(r['value'])) in [0,.01]])
text+='''

全範囲でマーカー最適は-20.5 mm、車軸は+3.5 mm。RMSは負側端で少し低いが、探索端なので左右スケールの最適値を決定していない。既存kL/kRを変更する根拠とはしない。

## 12. gyro処理・scale / bias・積分則

`log_schema.h` の gyroVal_Z は imuVal.gyro.z。IMU.cのapplyOffsetIMUで温度補正・angleOffset除去・COEFF_DPD=-0.9924を適用済み。c727dfb時点も同じ。係数を二重に掛けていない。

追加scale kgを±0.2%、0.02%刻みで走査し、各点のyIを-5～+10 mmでprofileした。最良はkg=-0.2%の探索端、yI=+3.5、RMS7.645 mm。元の7.663から改善約0.018 mmしかなく、scaleの最適値は未同定。追加biasを±0.3 deg/s、0.025刻みで別に走査すると+0.075 deg/s、yI=+3.5、RMS7.532 mm（改善約0.131 mm）。scaleとbiasを同時に増やしてはいない。この微小改善からgyro定数変更を採用しない。

一方、区間中のgyroを右端値だけでなく前後ログ行の平均で台形積分すると、最適yIはマーカー-20.5→-23.0、車軸+3.5→+1.0 mmへ2.5 mm動く。距離ログ間隔約5 mmに対してこの大きさは重要。ログには間引かれたgyroしか残らず、実際の1 ms周期のgyro積分値を完全再現できない。0.5 mm刻みの数値最小点を物理精度0.5 mmと解釈しない。車軸・台形でyI=0のRMS7.891は最小7.674との差0.217 mmにとどまる。

車軸右端の周回角変化はCW平均359.591°、CCW平均-359.892°。区間ごとの閉路残差平均CW11.174 mm、CCW9.077 mm。閉路を強制するscale補正は行っていない。

'''+image('gyro_sensitivity')+'''

## 13. ファーム採用判断

- エンコーダ＋gyroの自己位置推定: **追加yIは0 mmを維持**。既存courseAnalysis.cのcalcXYcieとpathFollower.cのpathFollowerUpdatePose1msはこの形で、物理IMU前後位置による横移動項を使っていない。
- 加速度補正: IMU_OFFSET_Y_M=-0.03168 mを別パラメータとして維持。この検証は加速度のオフセットを同定していない。
- -63 mmは採用しない。-20.5 mmはマーカー基準の対応点調整値としては有効だが、普遍的な自己位置推定パラメータではない。
- +1～+3.5 mmの残差最適値も採用しない。積分則・センサー時刻・基準点精度に依存し、0との差が小さい。

実機の制御性能はこのオフライン解析だけでは保証できない。今回はファーム・ログ形式・ハード構成を変更しておらず、CMakeビルド、割り込み変更、実機走行確認の対象となる変更はない。SHORTCUT走行性能の正式採用は既存方針どおり各条件10本以上の実機検証が別途必要。

## 14. 残る確認と最小試験

今回の0 / -63採用判断のために追加ログを必須とはしない。微小有効オフセットや残差原因まで確定したい場合は、まず次を最小確認とする。

1. 固定した直線CROSSで、前後車軸中点の床投影とU2反射実効中心の距離を実測する。左OR/右U2の初反応とCROSS確定を確認し、20 mm幅確認＋10～20 mm判定がどの再検出履歴になるかを分離する。GPIOを戻した状態も含める。床上検出中心は今は未実測。
2. 同じ充電状態、同じ一次速度、CW1本＋CCW1本。ライン配列の前後位置は固定し、各走行には今回同様の5周を含める。ログに1 msごとの累積gyro角または区間積分角があれば、5 mm間引きによる積分則の交絡を分離できる。ログ変更を行うならスキーマ・AGENTS・最初のログ検証を同時に行う。今回は追加実装していない。
3. 残差の主山である周回75%付近を走行動画または外部位置測定でCW/CCW照合する。現在ログだけからgyro biasか滑りかを一意に決めない。

これは診断の最小試験であり、制御性能の採用確認10本の代わりではない。

## 15. 検証・成果物・再実行

数値の独立確認: 代表ログ全5590行をスカラー式で再積分し、ベクトル式との差は最大4.95e-12 mm。既知の回転・並進に対する剛体整列も回復した。10本の必須列、有限値、累積距離・時刻単調、行数、アンカー数、補間範囲を検証した。連続時間のレバーアーム恒等式と離散中点式の差は最大0.0661 mm、導出した切り捨て誤差上界0.2463 mm以下。根拠のない0.02 mm閾値は使用しない。

図はPNG/SVGを保存し、RMS、軌跡、CROSSプロファイル、残差、配置図の描画を確認した。RMS曲線はyIを重複排除して昇順に描き、軌跡比較は共通表示範囲・等倍アスペクトにした。

再実行（リポジトリ直下、現在のpythonはnumpy/matplotlib利用可）:

```powershell
python analysis/script/analyze_yi_step.py
python analysis/script/analyze_yi_12838_12848.py
python analysis/script/plot_yi_12838_12848.py
python analysis/script/validate_yi_12838_12848.py
python analysis/script/write_yi_report.py
```

依存: Python、NumPy、Matplotlib。CADカーネル、pandas、scipyは不要。MPLCONFIGDIRは書込可能な解析出力内へ設定。元CSV、基板、機体ファイルは変更しない。

'''
for name in ['source_manifest.json','axle_geometry.json','health.csv','sensor_centers.csv','line_events.csv','axle_anchors.csv','anchor_summary.csv','extended_summary.csv','individual_optima.csv','model_sensitivity.csv','threshold_sensitivity.csv','gyro_sensitivity.csv','residual_regions.csv','lap_closure.csv','phase_sensitivity.csv','validation.json']:text+='- '+link(name)+'\n'
text+='\n失敗・再発防止記録: [CROSS基準とyIの交絡]('+str((ROOT/'doc/失敗事例/2026-10-03-yi-cross-anchor-phase-confounding.md').as_posix())+')。数値・描画の手順修正も解析スクリプトへ反映した。\n'
(P/'report.md').write_text(text,encoding='utf-8')
print(P/'report.md')
