---
name: robotrace-log-analysis
description: ロボトレース走行ログを解析、比較、可視化するときに使う。CSVログ、XY軌跡、速度追従、角速度、スリップ、ラップタイム比較、autoStart 5走比較、emcStopやcntlog欠落の判定を扱う。
---

# Robotrace Log Analysis

## 2026-10-25：現行ログとSLIPフラグ集計

有効性・無効理由（電流有効性を含む）と記録間隔内統計16列はbin/CSVから削除済み。経路診断6列追加後の現行は通常profile=1・56列/154 B、詳細profile=0・62列/163 B。50列/135 B・56列/144 BはV2整理だけの時点のサイズ。`slipV2Flags`はbit0=縦、bit1=採用横、bit2=Yaw不整合。SLIP計画は版3・`slipPlannerPolicy=flag_sample_fraction`で、optimalIndex別の縦/横ON記録数/全記録数を使用する。時間率ではなく、最小ON時間3msの追加選別は廃止した。検出器版4・3/5ms継続判定と内部計算は維持する。

未観測区間は増速しない。有効性が未記録のため、無効時のフラグ0と正常OFF、記録間に終了した短いONは区別できない。更新後は新しい一次とDISTANCEを取得する。機体は旧スキーマを拒否し、PC自己記述変換・全CRC検証は維持する。現行の確認はリポジトリルートから必要権限で`python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`。以下の旧列数・区間時間率の記述は履歴とし、現行は[正式採用とSLIP走行](../../../doc/slip-v2-active.md)と`log_schema.h`を優先する。

経路診断6列は末尾の`pathErrorY_mm`（F32・mm）、`pathErrorHeading_cdeg`（S16・0.01deg）、`pathState`（U8）、`pathYawFF_dps/pathYawLateralFB_dps/pathYawHeadingFB_dps`（各F32・deg/s）。1ms更新の最新値を距離間隔で保存し、PATH/SHORTCUT/PATH_CURVE以外は0。方位偏差は最近傍接線−機体方位、操舵3成分は合計の±1800deg/s制限前。操舵式は維持する。経路更新版18は1msの自己位置積分→経路更新→PIDで実行し、SDの位置/方位補正係数は5ms基準の1/5を毎回適用する。退避100ms・復帰200ms・混合率切替100msは維持する。詳細は[自己記述ログ](../../../doc/self-describing-log.md)。

## 実行前の再発防止

最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。CSVの探索・読込・集計・比較を始めるときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。

ペリフェラル初期化の調査では、CubeMXの分割生成を仮定してspi.c/tim.cを検索パスへ指定しない。最初に `rg -n "MX_SPI2_Init|BaudRatePrescaler" robotrace_v2/Core/Src -g "*.c"` のように実在ディレクトリを検索し、結果に出たファイルだけを読む。このプロジェクトのSPI2初期化はmain.cに存在する。


RLOGヘッダ検査を組む前に `log_schema.h` の `LOG_SCHEMA_PROFILE_LIGHT` と実ヘッダのprofile・recordBytes・columnsを照合する。現行は軽量profile=1・66列/170B、詳細profile=0・72列/179B。名称から0/1を推測しない。旧ログは保存列定義と実ヘッダで世代を区別し、数値不一致を即座に破損と断定しない。[profile取り違えの記録](../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)参照。

動画処理のランタイムはフォルダ名で推測しない。最初にGet-Command python.exeとTest-Path -LiteralPath <実体> -PathType Leafで実行ファイルを確定し、その実体で必要モジュールのspec・import・版を確認する。追加導入前に既存解析のsys.path・局所deps・必要版を読む。今回検証した実体はC:/Users/Natsuki/AppData/Local/Programs/Python/Python310/python.exe、局所依存先はbuild/video-analysis-runtime。新規環境で存在を再確認する。[実体と依存先の再発](../../../doc/失敗事例/2026-10-04-noise-analysis-input-preflight.md)参照。

床校正のコードを書く前に全対象動画の初期画像から可視点ID・点数を棚卸しする。可視点だけをfitへ渡し、固定8点assert・先頭8個切出し・追跡の配列境界が残っていないか確認する。機体の赤黄を観測できない場合、外形中心の床投影を車軸位置や真の方位・滑り速度へ読み替えない。同じ方向・固定床位置の相対変位に限定し、全採用通過の原フレーム指定画像と測定線の感度を確認する。[2026-10-26の点数固定再発](../../../doc/失敗事例/2026-10-17-video-color-marker-identity.md)参照。

## 2026-10-21：V2正式採用後の現行仕様

ユーザー指示によりV1互換・SHADOW切替を削除した。縦はFast単独10/8m/s²・3/5ms、横はLatCandidate版5の6.7/4.5m/s²・3/5ms・LPF0.8を使用する（初回ON6.5、2026-10-24の既存変更でON6.7）。保存検出器版4、`slipDetectorMode=ACTIVE_V2`、`slipLatPolicy=matched_candidate_v5`。通常66列/170B、詳細72列/179B（重複整理前の正式採用ログ13571～13577は70列/181B）。旧slipFlag/slipFlagLatと旧詳細診断9列は存在しない。slipV2Flagsの横bitとslipV2Lat/PeakLat/OnLatMsも採用横と同じ意味へ変更した。以下のSHADOW・旧サイズ・旧制御維持の記述は導入時の履歴として扱い、現行は[正式採用とSLIP走行](../../../doc/slip-v2-active.md)を優先する。

横ログは`slipV2Lat/Flags横bit/PeakLat/OnLatMs`へ統一し、`slipV2LatCandidate*`列を廃止した。横独自の有効性は`slipV2LatValid/Reason/ValidMs/IntervalReason`へ改名し、横ON率は`slipV2OnLatMs / slipV2LatValidMs`を使う。横の理由256・再充填で共通有効性と異なるため共通ValidMsへ代用しない。SLIP読込も新列名だけを使い、更新後は新しい一次とDISTANCEを取得する。判定式・閾値・検出器版4・計画版2は維持し、列定義CRCで保存形式を識別する。

SLIPは同じ一次参照・正常ゴールのDISTANCEログから軸別の有効ms/ON msを集計する。不正・飽和・未観測を滑りなしへ置換しない。毎回一次計画から再生成し、DISTANCE→SLIPのオートスタート接続を確認する。未検証の滑り距離推定は制御へ入れない。最初の新形式実機ログと10本以上の実走行再現性は別に確認する。

## Overview

Use this skill when analyzing logs for the robotrace_v2 robot. Treat `AGENTS.md` as the source for project-wide units, log schema, paths, and safety policy.

スリップ評価では[XY基準・動画の優先順](../../../doc/slip-evaluation-policy.md)を使う。最初に一次基準のXY差で滑り有無を判定し、その後動画を確認する。現行は`Dx<=Dx_primary AND abs(Dy-Dy_primary)<=20mm`（Dx/Dyは絶対値、両端を含む）。旧Y条件の結果を現行条件として流用せず、既存動画ラベルを再利用した再判定では旧新ラベルと時系列を保存する。動画が明確なら動画、不確定ならXY判定を閾値調整の正として採用し、XY判定・動画の確実性・最終ラベル・採用根拠を別に保存する。解析だけの回はファームウェア・設定を変更しない。

動画の実測寸法は[マーカー・レンズ高さ](../../../doc/video-slip-marker-geometry.md)を参照する。2026-10-18のレンズ高は680mmへ訂正済み。新規解析で共通関数の旧デフォルト710mmを暗黙に使わず、校正と追跡へ同じ高さを明示する。過去の別系列の入力条件を変更するときは旧結果を保全して再解析する。


2026-10-18のユーザー指定：モータ指令電圧上限が同じ走行は電圧差のみで除外せず、速度等は別に記録する。現行横候補版5は、接線補正を除いた同窓残差Rnへ角加速度と同じ一次LPF（前回0.8・今回0.2）を適用し、同窓の適用済み接線項Tbarを引く。ON6.5/OFF4.5m/s²、3/5ms、5/30ms窓と43サンプル充填。ON6.5はユーザー指定で今回テスト・ビルド未実施。最初の有効Rnで初期化し、無効でLPFを消去、区間統計リセットでは保持する。版5の保存候補残差から(R0±Rc)/2でRn/Tbarを逆算しない（この分離は追加LPF前の版1〜4に限る）。版・式・追加LPF係数/初期化方式のメタデータを先に確認する。旧ON5.2での既知16陽性窓保持を現行ON6.5へ流用せず、一次の不要ON・短い滑りの減衰・遅延の実機確認は残る。検出器版3/fast_only・初回SHADOW横4.6/3.8・従来制御は維持。詳細はdoc/slip-lateral-candidate.md。

2026-10-18の対象訂正：新判定の二次実走は13505以降、一次13504の横判定はユーザー確認の誤検出として陰性対照へ保存する。13500～13503を初回新判定とした以前の結論は撤回。保存版・閾値・ビルドが同一でも対象所属を勝手に復元しない。詳細はdoc/slip-lateral-candidate.md。

## PATH_CURVEの物理ずれと内部XY（2026-10-26）

2026-10-26夜の完全逸脱停止追加後は、全モードで全10センサーの正規化値100未満が20ms連続するとemcStop=5で停止する。1個でも100以上で継続時間を解除し、PATH系のTRACKING中も適用する。以前の13810～13816は追加前のログ。以下の「PATH系の通常明暗停止無効」は当該旧ビルドの実装調査として保持する。解析用の明部閾値800以上なしを、今回の全暗部100未満の停止条件と同一視しない。詳細は[完全逸脱停止](../../../doc/path-line-loss-stop.md)。

`BOOST_PATH_CURVE=5`はPATHと同じ仮想ラインをDISTANCE共通の曲率別速度で走る。PATH/SHORTCUTと同じ経路診断6列を使用する。制御のpathPose、CSVのx/y、pathErrorY_mmは車輪＋ジャイロに依存する推定値で、物理位置の独立な検証ではない。pathErrorY_mmは世界座標Yの実偏差ではない。

XY重ね描きの良否を解釈する前に、ユーザーの物理観測を保持し、同じ記録のlSensorCari0～9（現行明部閾値800）・pathState・位置/方位補正係数と照合する。物理逸脱が報告された走行は、emcStop=0や小さい内部偏差を理由に正常へ置き換えない。開始時の原点リセットを、1走行内の周回ドリフトの説明に使わない。直線接線方向の位置は白線だけでは一意に求まらない。

TRACKING中の白線消失だけでは退避せず、白線消失による停止はLINE_FALLBACK後という現行実装を確認する。実位置がずれても仮想位置が追従できるため、CSVの再積分一致を実機追従成功と呼ばない。XY変換そのものは対応binの全CRC・生左右積算平均距離・保存積算角度で別に検証する。CROSSが方位位相と不整合な場合は無検証で周回対応や位置補正へ使わない。

検証済みの解析はリポジトリルート`D:/robotrace/robotrace_v2`で`python analysis/script/check_path_curve_y_drift_13742_13808.py`、`python analysis/script/validate_path_curve_drift_bins.py`、`python analysis/script/analyze_path_curve_y_drift_detail.py`、`python analysis/script/report_path_curve_y_drift.py`。Pythonの実体・NumPy/Matplotlibを確認し、原本を読み取り、出力はanalysisだけへ保存する。物理原因・実Y移動量・安全条件の修正は未確認。詳細は[観測限界の事例](../../../doc/失敗事例/2026-10-26-path-curve-odometry-hides-physical-drift.md)。

2026-10-26の動画002～008／ログ13810～13816はユーザー確認のレンズ高700 mm。10/18の680 mmという履歴と混ぜない。床fit7～8点・留保4点、外形中心の同一方向通過31枚を確認し、13811/13812/13814の初回～5回目は外側約57/54/85 mm、内部XYは逆向き約25/27/27 mm。物理ずれを動画でも確認したが車軸位置や原因分離は未確定。検証と制約はanalysis/video_path_curve_13810_13816/report.md。再監査はanalysis/script/finalize_video_path_curve_13810_13816.py。原本・全CRCと物理計測品質を別に検査し、物理原因と制御修正はopenを維持する。

## Inputs

- Logs live under `F:\Dropbox\Document\robotrace\Log\v2` when accessible.
- Logs are CSV, UTF-8, comma-separated. New logs have `key=value` metadata on line 1, column names on line 2, and data from line 3. Older logs may combine column names and metadata on line 1; resolve columns by name.
- The robot retains numbered `.bin` logs after every run, series completion/abort and emergency stop. It does not automatically convert CSV at shutdown or startup. `C:\tool\AutoCopyCsvFromSd` copies bin into its temp folder, validates all CRCs, converts on the PC, and publishes CSV to the existing log folder. Preserve temp bin originals; see `doc/pc-binary-log-import.md`.
- CSV metadata adds `binaryLogNumber`, `binaryDataCrc`, and `binarySchema` as conversion identity. CRC text is signed 32-bit; compare bit patterns when needed. Data columns and units are unchanged.
- Current binary logs use self-describing RLOG v2: column names, storage types, order, numeric formats and derived calculation version are embedded once per file. The PC converter accepts stored-column additions/removals/reordering without rebuilding. Legacy RLOG v1 six-format support is retired. Robot analysis requires its current build's layout. Header/I/O failures preserve originals and block starts; PC conversion validates all CRCs and preserves originals on failure. See `doc/self-describing-log.md`. Do not parse bin/tmp/part as CSV.
- The schema source is `robotrace_v2/Core/Inc/log_schema.h`.
- The log header contains data names and `parameter=value` entries.
- `logDistanceTargetMm` records the target row spacing; use `encTotalOptimal` deltas to evaluate actual spacing because sensor values update at 1 ms and may exceed the target at high speed.
- New primary logs record `pathSourceFormatVersion=1`, `closureValid`, `closureReason`, `logExpectedRows`, IMU calibration validity/sample/error counts, and distance-scale verification fields in the metadata row. These fields do not change CSV columns or binary records.
- A PATH route source must be a normally completed primary log with source format version 1, valid closure, exact expected row count, successful IMU calibration, and verified distance conversion. Saved legacy-format and invalid primary logs must be rejected as route sources; if the primary validation fails, autorun must stop before the next run.
- Firmware-side secondary-log parsing resolves required fields by header name, not fixed column number. Required fields are `courseMarker`, `encTotalOptimal`, `ROC`, `targetSpeed`, `optimalIndex`, `slipFlag`, and `slipFlagLat`.
- Distinguish run mode by `optimalTrace`.
- 手動解析後のDISTANCE/SLIP/PATH/SHORTCUTも、変更後は`primaryLogNumber`へ走行開始時の一次参照を保存する。`autoRunNumber=0,requestedMode=MANUAL`は維持するため、一次番号だけで自動系列へ所属させない。過去の0は未記録として扱い、直前番号で補完しない。手動DISTANCEとSLIPの区別を一次番号から推測しない。詳細は[手動参照番号](../../../doc/manual-run-primary-log.md)。
- Exclude failed runs when `emcStop != 0`.
- Invalidate logs with `cntlog` gaps or processing drops; identify and report the cause.
- If battery voltage differs significantly, recommend charging and retrying instead of comparing as equal conditions.

## Workflow

1. Inspect available logs and identify target log numbers.
2. Confirm the run mode from `optimalTrace` and compare only compatible run types.
3. Check validity:
   - `emcStop == 0`
   - `cntlog` is monotonic and plausible for distance-based logging
   - required columns from `log_schema.h` exist
   - `batteryVoltage_V` is suitable for comparison
4. Analyze required plots and tables:
   - XY trajectory
   - speed tracking
   - angular velocity
   - slip
   - lap time comparison table
5. Save generated graphs and tables under `analysis/`.
6. Report comparison target logs, changed condition, adoption decision, and remaining issues.

## Light and debug profiles

- On 2026-10-14 the user requested restoration of the light profile to 72 CSV columns / 183 bytes, including the 19 SHADOW and 10 lateral-candidate diagnostics. Detailed remains 87 columns / 220 bytes; detector calculations and legacy control/plans are unchanged. The temporary light43 / 108-byte period from 2026-10-13, including 13212–13227, does not contain these diagnostics. Other historical generations have 62 columns / 157 bytes (initial SHADOW) and 43 columns / 108 bytes (pre-SHADOW). Individual encoders, line sensors, acceleration, six IMU diagnostic columns and both motor currents remain in their existing positions. The trailing empty CSV cell is excluded from this count.
- Logs 12872–12878 used the temporary minimal 13-column/22-byte profile. Raw-wheel, independent line-CROSS, temperature-series and XYZ-angle analyses cannot use those logs; check required column names before analysis.
- The debug profile has 87 columns and 220 bytes. The unchanged 87-column/220-byte lateral-candidate schema is frozen in `log_schema_candidate_v1.h`; other frozen predecessors have 77 columns / 194 bytes (initial SHADOW) and 58 columns / 145 bytes (pre-SHADOW). All six supported binary formats can be recovered by either current build profile using strict profile/schema CRC/record-length identification; predecessor CSV keeps its original columns. Metadata validation and firmware secondary-run required fields remain available in both profiles.

## Column Meanings

- `cntlog`: time after run start, based on `cntRun`, `[ms]`.
- `encCurrentN`: average left/right encoder pulse count per 1 ms.
- `encCurrentL`, `encCurrentR`: signed left/right pulse counts per 1 ms `[pulse/ms]`; divide by `PULSE_MILLIMETER` for `[m/s]`.
- `encTotalL`, `encTotalR`: signed left/right accumulated pulses `[pulse]`, reset only at the start-marker transition to the timed run. New logs use the start marker as the origin; logs through 12878 accumulated since power-on. The first logged row is after the marker, so it may already be nonzero. Subtract the initial logged value and divide by `PULSE_MILLIMETER` for relative distance from the first row `[mm]`. Individual wheel conversion is diagnostic; the measured scale was verified for the left/right average.
- `lSensorCari0` through `lSensorCari9`: calibrated, normalized line sensor values, dimensionless `0..4095`; index 0 is the leftmost sensor. These and the four individual encoder columns are included in both light and debug profiles.
- `gyroVal_Z`: IMU Z angular velocity, `[deg/s]`.
- `gyroVal_X`, `gyroVal_Y`: offset/direction-corrected X/Y angular velocity, `[deg/s]`. `imuTemp_C`: latest IMU temperature, `[°C]`, acquired every 5 ms; `-999` means invalid.
- `imuAngle_X`, `imuAngle_Y`, `imuAngle_Z`: existing `imuVal.angle.x/y/z`, `[deg]`, reset to zero at the start marker. X/Y are accelerometer-fused attitude; Z is the unwrapped integral of corrected gyro values at 1 ms, including the existing temperature correction. Use Z for yaw drift analysis instead of reintegrating sparsely logged gyro values. These six diagnostic columns are appended in both light and debug profiles; temporary minimal logs lack them.
- `acceleVal_X`, `acceleVal_Y`, `acceleVal_Z`: IMU acceleration after offset calibration, `[g]`, including gravity; X/Y also include the existing rotation-center correction. Multiply by `GRAVITY_MPS2` for `[m/s^2]`. All three axes are included in both light and debug profiles.
- `courseMarker`: confirmed marker state while running: 1=RIGHTMARKER, 2=LEFTMARKER, 3=CROSSLINE. A right-marker row is not equivalent to an SG increment; `checkStartGoalMarker()` also requires distance/qualification conditions. Do not infer missing SG counts from right-marker row counts. Match verified cross events for closure alignment and use saved `goalMarkerOnset_p` for the final onset.
- `encTotalOptimal`: corrected distance count for secondary runs.
- `ROC`: curvature radius, `[mm]`.
- `targetSpeed`: target speed in encoder converted units, `[pulse/ms]`.
- `optimalIndex`: index into `PPAD[]` or `shortCutxycie[]`.
- `slipFlag`: longitudinal slip flag.
- `slipFlagLat`: lateral slip flag.
- `lineTraceCtrl`: current log column name; value is `lineTraceOmegaFBCtrl.pwm`.
- `motorCurrentL`, `motorCurrentR`: latest left/right motor current `[A]`, updated every 10 ms after 32-sample ADC averaging and zero-offset calibration. Included in both profiles; appended after IMU fields in the light profile since 2026-10-07. Earlier light logs omit them.
- `targetAngularvelo`: log target angular velocity, `[deg/s]`.
- 2026-09-30以降の通常・詳細ログは `motorpwmL`, `motorpwmR` と経路追従専用7列（`linePointX_mm`, `linePointY_mm`, `lineValid`, `pathErrorY_mm`, `pathErrorHeading_cdeg`, `pathState`, `pathLegalMargin_mm`）を出力しない。以下の経路追従専用列の意味は過去ログ向け。専用列必須の解析スクリプトは新ログに使用しない。
- `x`, `y`: スタートマーカー原点の推定位置[mm]。計算版2（logDerivedVersion=2、logXYDistance=raw_wheel_average）は、マーカー補正前の左右生エンコーダ累積値encTotalL/Rの平均距離差と、保存済み1ms積算角度imuAngle_Zの両端平均で計算する。int64の左右和を差分化して0.5倍し、半パルス・後退を保つ。encTotalOptimalのカーブ/CROSS位置補正をXYへ使わない。閉路Xも同じ計算とし、制御中自己位置・速度計画は変更しない。旧計算版1ではencTotalOptimalを使用していたが、ユーザー指定により新変換器の互換読込対象外。さらに古い疎なgyro再積分のログは別の過去形式であり、その形状を実滑りと即断しない。
- `linePointX_mm`, `linePointY_mm`: corresponding first-run line point, `[mm]`.
- `pathErrorY_mm`: signed lateral path error, `[mm]`.
- `pathErrorHeading_cdeg`: heading error, `[0.01 deg]`.
- `pathState`: 1 tracking, 2 line fallback, 3 rejoin blend, 4 localization lost.
- `pathLegalMargin_mm`: remaining line-overlap margin after the tracking-error budget, `[mm]`.

## SHADOW slip diagnostics (2026-10-12)

These 29 diagnostics are restored to both the 72-column light and 87-column detailed profiles on 2026-10-14. Logs from the temporary 43-column period contain no `slipV2*`, `imuYawAlphaFiltered` or `imuLatTangentialApplied` samples; report them as unrecorded, never healthy zero. Detector metadata alone does not establish recorded diagnostics. Check actual columns, schema and build information before running candidate-analysis scripts.

- `ROBOTRACE_SLIP_DETECTOR=SHADOW` is the default. `LEGACY` disables the new detector. Existing `slipFlag` / `slipFlagLat`, next-run plans and distance correction still use the legacy detector.
- New residuals: `slipV2LongFast`, `slipV2LongSlow`, `slipV2Lat` [m/s²], `slipV2YawDps` [deg/s]. Flags are a bit mask: longitudinal=1, lateral=2, yaw mismatch=4. These are model inconsistencies, not measured physical slip amounts.
- From 2026-10-14 detector version 3 (`slipLongPolicy=fast_only`), only LongFast drives the longitudinal flag: abs >10 m/s² for 3ms ON, abs <8 m/s² for 5ms OFF. Version 2 requires both Fast and Slow for ON and OFF. Windows, thresholds, validity and residuals remain fixed; LongSlow stays diagnostic. `slipV2OnLongMs` follows the selected rule, while `slipV2PeakLong` still takes the maximum absolute value of both windows. Identical schemas do not identify the decision rule: check metadata and leave unknown policy unresolved. See `doc/slip-long-fast-only.md`.
- Only interpret new residuals/flags when `slipV2Valid=1`. `slipV2Reason` records current invalidity; `slipV2IntervalReason` ORs invalid reasons across the whole recording interval. See AGENTS.md and doc/slip-detector-v2.md for bit values.
- `slipV2PeakLong/PeakLat/PeakYawDps` preserve interval absolute maxima. `slipV2OnLongMs/OnLatMs/OnYawMs/ValidMs/TotalMs` accumulate observed 1ms calls, saturating at 65535. Use valid observed time as the denominator; exclude saturated intervals and report missing/invalid time. The unrecorded tail after the final row is not included.
- `slipV2CurrentValid` requires calibration and a fresh finite read; `slipV2CurrentHighSamples` counts distinct 10ms acquisitions, not held 1ms samples. `slipV2PwmSum` is the saturated absolute PWM sum (0..2000). Current/PWM do not gate new slip flags.
- Missing new columns in old CSV mean unrecorded, never healthy zero. Do not compare old/new flags as ground truth. Existing 13074..13088 interpolation replay lacks the stored static-gravity reference and acquisition-success history and is only a diagnostic approximation.

## Lateral tangential candidate diagnostics

- See `doc/slip-lateral-candidate.md`. The candidate is diagnostic only; common IMU values and initial SHADOW residuals are unchanged. Versions 1–4 use `currentLinearX - 2*appliedTangentialX` with common windows. Version 5 additionally filters the neutral residual with the matched 0.8 LPF before subtracting the windowed tangential term. Do not treat the saved v5 residual as the old unfiltered residual or derive the tangential window by `(R0-Rcandidate)/2`. The right-X assumption remains physically unverified. Do not fit the observed regression coefficient into the correction.
- `slipV2LatCandidate` and `slipV2LatCandidatePeak` are the signed residual and interval absolute peak [m/s²]; `slipV2LatCandidateFlag` is ON=1, not a bitmask. `slipV2LatCandidateValid/Reason` are independent of the initial SHADOW validity. Reason 256 means tangential sample unavailable and applies only to the candidate; other reason bits match v2.
- `slipV2LatCandidateOnMs/ValidMs` and `slipV2LatCandidateIntervalReason` preserve interval ON/valid time and invalidity OR. Counters saturate at 65535. Use candidate ValidMs for candidate rates and initial SHADOW ValidMs for baseline rates. Exclude saturated intervals and report unmatched validity. Never compare intervals with candidate-only missing data as equal conditions.
- `imuYawAlphaFiltered` [rad/s²] and `imuLatTangentialApplied` [m/s²] are instantaneous snapshots from the existing correction. They are not 30ms-averaged residuals. Align smoothing and intervals before studying angular-acceleration dependence; sparse CSV cannot reproduce the actual 1ms filter history.
- Check candidate version/formula/axes/offset/LPF metadata and 72/87 columns in the first real log after restoration. The temporary 43-column light logs cannot support candidate diagnostics. Initial SHADOW logs 13094..13124 lack these 10 columns: report candidate diagnostics as unrecorded, not zero. Their sparse reconstruction cannot prove candidate performance.

## Straight-road wobble and acceleration residuals

- For straight-road braking wobble, define road geometry from the validated primary route/plan and match secondary `optimalIndex` to that plan. Do not select straight road by small measured gyro/alpha: that excludes the wobble being investigated. Keep geometry, commanded acceleration/braking, and measured yaw activity as separate labels. Check source plan parameters and source identity before reusing a host planner. Inspect a straight-interior boundary sensitivity and label the primary geometry as estimated, not physical ground truth.
- Evaluate the user's two hypotheses separately: braking-time Yaw inconsistency with actual gyro/heading/steering reversals, and acceleration-time positive LongFast with wheel-speed growth exceeding IMU acceleration. Negative LongFast is not the same acceleration-spin evidence. Fast/Slow have different windows; check whether the version-2 both-window condition suppresses a short candidate and distinguish version-3 Fast-only flags by metadata. Do not adopt new thresholds from these descriptive statistics. See `doc/slip-turn-residuals.md` and `analysis/script/analyze_straight_accel_brake_13228_13247.py`.
- Windows Illegal System DLL Relocation / WinError 623 may affect existing GCC-built host executables as well as compilers. Use the same authorized read-only host analysis in the permitted execution environment; do not change firmware to work around that launch failure. See `doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md`.

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_PATH_REPLAY`: verify XY trajectory and completion against the validated primary route. For legacy logs with path diagnostics, also verify lateral/heading error, fallback count, and line-position/heading correction. Goal is the halfway extension from the primary endpoint toward the origin; marker count does not terminate PATH mode.
- `BOOST_SHORTCUT`: verify the Level 1 straight corridor trajectory and completion. For legacy logs with path diagnostics, also verify non-negative `pathLegalMargin_mm` and absence of localization fallback. New logs cannot establish these diagnostic conditions directly. Goal uses the same extended-route arc length as PATH REPLAY.
- Route generation failure, extension failure, or point-capacity overflow must block route start. For automatic runs, invalid primary-source metadata blocks progression to the next run.
- PATH/SHORTCUT tuning remains unadopted until same-condition real-world runs provide at least 10 logs per level; assess completion rate and stop-position repeatability before lap time.
- Compare only logs with the same run mode; distance, path replay, and shortcut modes are not equivalent.

## Output Rules

- Put analysis scripts in `analysis/script/`.
- Use Python when useful.
- Save results in `analysis/`.
- Name outputs so the target log number and analysis type are clear.
- Examples: `log_00012_summary.csv`, `log_00012_xy.png`.
- If the input contract is not yet implemented, decide it before adding a script: single log, multi-log comparison, and autoStart 5-run comparison are separate modes.

## Known malformed candidate CSV headers (13130–13144)

These 15 originals have 72 numeric values but only 71 header names: the 30-byte temporary column buffer truncated `slipV2LatCandidateIntervalReason` and lost its comma, joining the next name. Never pass the original directly to a name-based numeric analysis. Use the derived copies under `analysis/slip_candidate_13125_13144/header_repaired/`, created by `analysis/script/check_slip_candidate_13125_13144.py`. Restoration requires the known schema CRC -1299980178, exact expected malformed header reproduced from the source, all 72 data values per row and expected row count. Keep and hash originals; reject unknown corruption. The source fix removes the short temporary buffer and has host/build validation. All 11 post-fix physical CSVs 13145–13155 (22:28:09 build) have the complete 72-column header and matching data rows without repair. Logs 13145–13150 are user-confirmed hand-held/free-spin checks; exclude them from normal-course completion and lap comparisons. The detailed 87-column physical CSV was not captured in this batch. See `doc/失敗事例/2026-10-12-csv-long-column-name-truncation.md`.
