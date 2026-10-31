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

版20/21/22等の新規比較では、CSVメタデータへ裸のintを使わず、compare_controller_20_21_22.metadata_integerの有限・整数相当検査を通す。全ログを読む前に5/5.00受入、5.1/nan/±inf拒否の6境界を確認する。13901～13910/13913～13922/13928の21原本で所属とSHA一致を確認済み。[整数表記の10/31再発](../../../doc/失敗事例/2026-10-14-log-metadata-integer-decimal-format.md)。

2026-10-31：動画の同期前に録画先頭で機体が既に移動しているか確認し、正のoffsetだけを仮定しない。負offsetも含む探索結果・全ログに対する被覆率・相関を保存する。ログ先頭が写っていなければSG開始位置を補完せず、最初の実観測基準の相対比較と未観測時間を明記する。保持RAM境界を映像へ写す際は同時刻の有効点の間隔を先に検査し、25msを超えたら未測定とする。他時刻や制動後位置へ置換しない。原解像度の色閾値感度が1°ゲートを超える点は不合格を保持し、全点方位精度合格と呼ばない。[動画の開始/終了境界](../../../doc/失敗事例/2026-10-30-video-post-run-coverage.md)。

新走行のRAMイベント比較前は、実events.csv・enum・生成条件を読む。イベントは理由だけでなく電流有効等の上位flags変化も記録されるため、旧走のイベント時刻一覧を期待値へ流用しない。各イベントの下位4bitと全周期履歴の一致・時刻順序・理由遷移を検証する。未知の準備スクリプト名もrg --filesで実在名を取得してから読む。PowerShellの依存した読取りはErrorActionPreference=Stopを設定する。入口はanalyze_curve22_070745_inputs.py、実14257周期/203レコードを検証済み。[派生補助の境界再発](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)。

派生スクリプトを複数生成する前に、全対象のAST ImportFrom/公開名を読み、importする名前の順序やROOT併記まで同一と仮定しない。全対象の置換計画を検査してから最初の生成を始める。完了済み段階は実ELF/配置SHA・生成ソース・成功証跡を照合し、消費側の生成失敗で配置照会やC試験を繰り返さない。版22統合のvalidate_path_trial_capture_decoder.pyは52配置と8異常試験を保全してCLI5件だけ再開した。[依存生成の再発](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。


binをRAMと直接照合する前に実serializerを読む。現行RLOGヘッダlittle/レコードbig/nativeRAM littleを分離し、先頭時刻の実バイトと保存列154Bを検証する。派生再生はfscanfのheader/設定/行幅を確認する（基準19、慣性27）。差替え外の浮動小数点テキストは保持し、差替え前の全出力不変を確認する。今回の入口はanalyze_observed_220124.py、run_observed_220124_validity.py、probe_observed_220124_tail.py。[バイト順](../../../doc/失敗事例/2026-10-30-native-ram-and-log-record-byteorder.md)、[入力幅/精度](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。

兄弟解析モジュールをimportする試験は、起動方法とsys.pathを確認する。repo rootのstdinから裸の兄弟名をimportせず、analysis/scriptに実在するスクリプト入口を絶対Python・明示workdirで実行する。validate_path_input_axis_pose.pyは7既知ケースを確認する。[起動契約の事例](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)。

新モジュールの検索では機能名からCore/Inc/slip等の下位フォルダを構成しない。直前に存在確認したCore/Inc等の親へrg -gを使い、実ファイル名を得てから読む。計測理由幅の検証はrecord_path_input_capture_search_preflight.pyが既知親・実宣言uint16_t・終了0を入口で確認する。[下位パス推測の再発](../../../doc/失敗事例/2026-08-11-windows-rg-glob-not-expanded.md)参照。

配置JSONを復号する前に各valueの型と特殊キーを読む。end:0等を[offset,size]と一括分解せず、名前で除外した後に形と範囲を確認する。完了Cの後の消費側失敗ではexe/ソース/生出力/配置SHAを照合し、未実行の復号から再開する。RAM復号は実ELF/Flash一致・凍結・全52フィールド配置を確認し、合成fixtureを実機データと呼ばない。今回の入口はdecode_path_input_capture.py/validate_path_input_capture_decoder.py。[異種配置の事例](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。


走行評価集合の「ログ正常終了」と「ユーザー/映像で板内完走確認済み」を分け、未確認を別区分で保持する。emcStop0だけを物理的なはみ出しなしの根拠にしない。現在の23走比較ではemcStop0は21本だが、明示確認済みは13901～13910/13913～13922の20本で13899は未確認。採否・10本の物理確認へ未確認ログを加算しない。[確認範囲の訂正](../../../doc/失敗事例/2026-10-30-normal-log-physical-scope.md)参照。



派生再生の付加CSVはproducerのsnprintf/fputsまたはmanifestで命名・使用列を確定する。現在の追従診断は出力全名へ.tracking.csvを追加し、例は13898.csv.tracking.csv。with_suffixで置換しない。yaw命令と分解成分も実ヘッダを読み、推測した列名を使用しない。集計のみの失敗はSHA/参照一致で完了Cを保全し未実行段階から再開する。[出力契約](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。



派生CSV解析で既存load/load_csvを流用する前に、関数実装のreturnと型を読む。check_path_curve_y_drift_13742_13808.loadは(summary, columns)で、生ヘッダはsummary['metadata']。生メタデータを直接返す別補助と混同せず、入口で型・原CSV番号・列数・SHAを照合する。重力を保存slipV2Latから除くときはlog_schema→保存元まで追跡し、matched_candidate_v5の平均と0.8 LPFを含む実C応答を使う。[補助の返り値](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。


新しい速度解析の前に、SPEED_PARAM_FIELDSの保存19項目/メタデータ15項目を確認する。search/stop/shortCut/pathReplayはログ非出力。searchが欠けても過去の実SD値を推測せず、凍結C再生の既定を使うなら出典・適用範囲を明記する。保存RAMの停止時値も全過去走へ転用しない。NumPy集計はJSON保存前にint/float/boolへ明示変換し、集計のみ失敗した場合は完成済みC出力を検証して未実行部分から再開する。[非出力項目](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)、[JSON集計型](../../../doc/失敗事例/2026-10-30-tracking-screen-json-scalar.md)参照。


白線から横速度を学ぶ状態を追加するときは、横移動ゼロの対照・方位/gyroずれへの誤配分・独立映像の留保誤差を比較する。合成位置誤差や白線照合回数だけを根拠に採用しない。今回の5状態9組は23走×O0/O2の460再生で全9組の13924留保位置誤差が悪化し未採用。加速度の物理符号はこの白線のみの試験から確定しない。[白線だけの横速度学習](../../../doc/失敗事例/2026-10-30-white-only-lateral-state.md)参照。


13924のような色点付き新規比較は、既存カメラ補助のsys.path設定を先に読む。この環境のOpenCV/SciPyはbuild/video-analysis-runtimeにあり、通常Python310のsite-packagesだけを仮定しない。依存先を明示後find_spec/importを検査し、CSVのgenfromtxtもencoding=utf-8-sigを指定する。時刻同期の粗密グリッドはNumPyだけで実行可能。[ランタイム再発](../../../doc/失敗事例/2026-09-02-path-gain-analysis-matplotlib-missing.md)参照。補助モジュールがsysをexportすると推測せず、argvは呼出し側で管理する。

床点の採用凸包を機体追跡の板ROIへ流用しない。欠測床点があっても現画像の板外周を別に確認し、inventoryをassert前に保存する。低被覆率を宣言する前に欠測の代表原フレームを確認する。[床点とROIの契約](../../../doc/失敗事例/2026-10-17-video-color-marker-identity.md)参照。方位はunwrap後に時間補間し、差だけをwrapする。179→-179度の中点自己試験を先に通す。[循環角の補間](../../../doc/失敗事例/2026-10-30-circular-heading-interpolation.md)参照。


過去系列を使う新規再生でも、全CSV/一次binの存在と原本SHAをコンパイルより前に解決する。<系列>/original/<番号>.csvという保存先を仮定しない。今回のbuild/path-yaw-ff-validation/replay.pyのcsv_sourceはbinary_validation.jsonのcsv_path/csv_sha256を優先し、旧系列はvalidation.jsonのoriginalsから番号と拡張子で一意に解決する。15走×O0/O2で両世代を確認した。[再生入口の世代差](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。

周回の相対横変位が小さくても、板内走行の成功とは扱わない。全通過・全周の代表画像に加え、追跡bboxの板端への最接近フレームを原動画の元解像度で確認する。明部が残る過渡横偏差では、全暗部停止5や欠測停止7と別に、白線の端素子・経路偏差・操舵3成分・取得周期を照合する。今回の確認はanalysis/script/finalize_heading_002102.py。新撮影の高さは今回の明示740mmを校正/追跡の両方へ採用し、以前の700mmを流用しない。[過渡偏差と部分張出し](../../../doc/失敗事例/2026-10-29-path-transient-board-protrusion.md)参照。

新しい監査で過去系列をまとめる前に、実在するbinary_validation.jsonとvalidation.jsonのキーを読む。CSV原本SHAは新しい系列ではcsv_path/csv_sha256、旧系列13848/13849ではvalidation.jsonのoriginalsにある。片側を推測せず、同じログ番号・拡張子で保存済みハッシュを解決して原本と照合する。今回の確認はanalysis/script/finalize_heading_probe_220117.py。[証跡契約の記録](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。

終端で明部が見えるのにnoMatchが増える場合、白線更新ageだけでなく延長経路のwhiteRoute.validと探索窓を確認する。今回の実行はanalysis/script/check_path_white_tail.py。両端が有効な観測区間がない地図を、未観測点の有効化や停止緩和で隠さない。[終端地図不足](../../../doc/失敗事例/2026-10-28-path-white-map-goal-extension.md)参照。補助関数の宣言先をpathPolicy.h等と推測せず、実在Core/Src・Core/Incディレクトリへシンボル検索する。派生解析器のpreflightはモジュールfor/withの束縛もsymtableで取得する現行版を使い、未定義名拒否とfor内包表記受入の自己試験を先に実行する。集計primaryは文字列の場合があるため、監査の新規比較前に実型を読み、finite・整数相当を確認してからmode/primary/emcを比較する。

PATHのIRQ時間が1ms未満でも、白線の取得窓・更新間隔・ageと回復条件を別に確認する。13877/13878はIRQ最大502µsでも鮮度切れ23.94%で長時間減速した。100ms連続回復が実際の更新周期で成立するか実関数試験を行い、集計ヒストグラムを1msの原因履歴へ読み替えない。ADC実装検索はsensor.c等を推測せず、リポジトリルートから`powershell.exe -NoProfile -ExecutionPolicy Bypass -File analysis/script/search_sensor_acquisition.ps1`で実在Core/Srcだけを検索する。[取得周期と回復条件](../../../doc/失敗事例/2026-10-28-path-sensor-cadence-prevents-recovery.md)参照。

既存ラッパーの基本ユーティリティ名は接頭辞から推測せず、実importを読んで実体の存在を確認してから参照する。解析を切り出し生成するときは依存関数も移し、生成元と生成物を両方修正する。保存前に構文検査と直接グローバル参照検査を行い、実入力の小さい再生で確認する。今回の単純な構造では`python analysis/script/preflight_generated_path_analysis.py`を使う（動的importや入れ子は別途確認）。[パス推測の再発](../../../doc/失敗事例/2026-08-11-windows-rg-glob-not-expanded.md)、[依存関数の欠落](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。

RAMのcolumnTitle由来メタデータをbin/CSVと照合するときは、assert前に実バイトの終端CR/LFを両側で確認する。片側だけrstripしない。終端改行だけを正規化し、項目名・順序・値・数値書式・他の文字は維持して全内容を照合する。さらに辞書として全項目一致を確認し、原本のSHA/CRCは無変換バイトで保持する。13870ではRAM末尾LF1文字だけで新規assertが停止した。既知の正規パス・明示workdir・必要権限でanalysis/script/analyze_path_timing_13869_13870.pyを実行して全CRC/全列/RAM照合まで確認する。[RAMヘッダ比較の再発](../../../doc/失敗事例/2026-10-17-shared-header-line-ending-check.md)参照。



2026-10-27の新撮影/未確定tmpの追加確認：同じ高さ・床配置でも画素位置の据置を仮定しない。校正起動前に全初期画像のIDラベル・可視面積・旧中心との差を確認し、今回観測した探索中心を使う。品質ゲートは維持する。未確定tmpの連続性検査後は、採用末尾の512B/実書込単位境界をまたぐ行を保守的に除外して感度と除外数を記録する。古い全CRC合格コンテナを対象tmpのCRC・メタデータへ転用しない。[床点位置](../../../doc/失敗事例/2026-10-17-video-color-marker-identity.md)、[tmp境界](../../../doc/失敗事例/2026-10-18-unsealed-tmp-partial-recovery.md)参照。

動画追跡と原フレーム画像生成は別ジョブとして、対応する各session_idの終了コード0を得てからmanifestを読む。履歴検証の必須番号を最新N本の表示窓から探さず、指定原本とSHAで検証する。ログバッファ関数もファイル名を推測せず、実在Core/Srcへrgで定義を探す（現行logBufferWriterはSDcard.c内）。


メタデータの新規assertを追加する前に、数値項目を型付き変換して判定する。整数を意味する値にも小数表記があり、alpha==100・emc==7などの数値判定を文字列100/7との一致で代用しない。保存書式や原本バイトの一致は別の検査にする。[数値表記の再発](../../../doc/失敗事例/2026-10-14-log-metadata-integer-decimal-format.md)参照。

2026-10-27の校正再発防止：下端の広い画像X区間で最大Yの色成分を選ぶ方式は機体反射を床点へ誤対応させた。校正前に全初期画像の床点IDラベルを目視し、IDごとの局所探索で実測重心・可視面積・位置差を検証する。欠測点は除外し、予測画素を観測へ置換しない。fitと独立留保の既存ゲートに合格するまで追跡しない。周回変位の比較切抜きは最初の通過の画像X/Yを両方固定し、床背景が動かないことを全採用図で確認する。画像名は生成manifestか実在一覧から取得する。詳細は[床点誤対応の10/27節](../../../doc/失敗事例/2026-10-17-video-color-marker-identity.md)。


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

## 2026-10-27：白線融合の観測点と実制御再生

融合比較の候補を作る前に、伝播する車体基準点・地図の基準点・センサー観測点を区別する。前方白線を車軸XYへ直接照合せず、一次の実センサー観測で小R区間の残差を確認する。全列棚卸しと有限の全候補を明記し、欠測列・未記録鮮度・重力基準を推定済み事実にしない。選定用で候補固定後に確認用を評価する。方法変更後に使い直す系列は新しい未見評価ではない。実制御の最近傍・ゲート・1ms補正上限・停止を含む原本bin再生を実施し、直線評価点の改善だけで採用しない。詳細は[白線観測点の混同](../../../doc/失敗事例/2026-10-27-path-white-observation-reference.md)と[版19](../../../doc/path-confidence-fusion.md)。初回実ログと同条件10走の採否確認は別途必要。

1ms化する前に観測雑音・伝播雑音の時間単位を照合し、疎なログ値を独立した1ms観測として重複融合しない。保存optimalIndexに依存するPC診断と、推定結果で最近傍を選ぶ実制御を分けて検証する。単一線の接線位置や真biasの精度を数値最適化だけで保証しない。既知bias合成試験が期待精度を満たさない場合も、診断を保全し、位置改善・有限性の成功でその失敗を消さない。
