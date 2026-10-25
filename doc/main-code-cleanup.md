# mainのログ処理整理（2026-10-24）

未使用関数と旧テスト、機体側CSV変換、旧CSV読込の順に整理した。走行モード番号、制御式、PID、スリップ閾値、SD設定の形式は変更していない。ログ列・保存型・列順・レコード長とRLOG v2、CRC32/IEEE、派生計算版も維持する。

## 残す処理と削除する処理

- `getAngleSensor()`は残す。全入力範囲から最小値を確実に選び、両端と隣接値がともに0の場合は角度0として返す。内側の有効入力の計算式は維持する。走行制御への新規接続は行わない。
- 呼び出しのない`readLogTest()`、`cmpfloat()`、`SDtest()`、`sd_flush_log()`、`getNextLogNumber()`、空の`Interrupt300ns()`、到達不能のヨーレート調整画面を削除した。未定義の`writeMarkerPos()`宣言、未使用のSDベンチとサンプルファイル操作APIも除去した。SDのmount/remountと既存初期化動作は維持する。
- 機体のCSV変換・進捗通知・CSV照合・出力バッファ・書式生成を削除した。記録キュー、最終書込、停止時の閉路検証、bin確定、起動時検証とtmp保持は維持する。
- 機体の解析元と選択一覧はbinだけとし、CSVフォールバック・旧列名・混在ヘッダ・CSV数値解析を削除した。採番は既存csv/bin/tmp/partすべての予約番号を保護する。
- CSV行用6 KiBの共有バッファと512 Bの書式バッファを削除した。ログI/O領域は走行中の2 KiB×3本と停止中の4 KiB先読みで引き続き共有する。スタック予約8 KiBは維持する。

## 型付き読込

`logSource.h`を読込契約の入口とする。停止中・ログ非記録中・FatFsロック下で1ファイルずつ使用する。

1. `logSourceOpen(file, number, purpose)`で現行スキーマのbinを開き、ヘッダ・実長・スキーマ・メタデータ・全記録CRCを検証する。不正binはCSVへ切り替えない。
2. `logSourceInfo()`と`logSourceMetadata()`で検証済み情報を取得する。メタデータの文字列表現は保存形式の一部として維持し、開いている間は変更しない。
3. `logSourceNext()`で`LogSourceRow`を取得する。速度外れ値・曲率・クロス区間処理を維持し、PATH用途では生左右距離と保存角度からXYを復元する。擬似CSVヘッダや空のデータ行は生成しない。
4. falseの戻りを`logSourceError()`/`logSourceEof()`で判別し、`logSourceClose()`の失敗も呼出側へ伝える。読込終端でも記録CRCを再照合する。

SLIP元の方式・一次番号・停止状態・ゴール回数・行数・軸別有効時間とON時間、PATH元の閉路・IMU校正・距離検証の条件は維持する。経路キャッシュを使う場合も元bin全体の検証を省略しない。

## PCと実機の確認

PCの`C:/tool/AutoCopyCsvFromSd`は変更していない。既存の自己記述変換器と旧CSV取込を維持し、SD原本削除はPC側での照合完了後に限る。

現在の回帰は[validation/log_storage](../validation/log_storage/README.md)と`validation/line_sensor_adc/run.py`。旧V1/SHADOW切替、機体CSV変換、変更前スナップショット依存の一時テストを廃止し、現在のbin保存・解析を検証する追跡対象のハーネスへ集約した。作業前後の通常/詳細×直線/閉路カーブで、記録と列定義の全バイト、および4方式の計画が一致した。

2026-10-25に、ユーザーが取得した13614〜13623（2026-10-24 23:51:34ビルド）を確認した。一次→PATH→SHORTCUT要求→DISTANCE→SLIPの5走×2系列が正常終了し、bin原本の全CRC・実長・CSV全保存値/メタデータ・既存66列/170B、校正値、DISTANCE→SLIPの元番号対応を検証した。解析前後の原本SHA256は一致。詳細は`analysis/main_cleanup_13614_13623/report.md`。

SHORTCUT要求2本は実際にはLevel 0のPATH相当であり、Level 1確認には数えない。DISTANCE2本とSLIP1本は既存XY基準外で、走行性能・物理滑りの課題は残る。再起動時の操作確認、ISR最大時間、各方式10本以上の同条件再現性は今回のログ確認だけでは完了しない。解析時にファームウェア・SD設定は変更していない。

## 制御API・画面・設定・メタデータの整理（2026-10-25）

ユーザー指定の順に、以下の4点を修正した。

1. `motorCommandOutSynth()`から全呼出しで0だった`dCmd`を削除した。3引数へ揃え、飽和処理・トレース優先処理・電圧補償を維持した。
2. ライントレース・角速度フィードバック・速度のPID画面を`setup_pid()`へ統合した。表示位置とフォント、選択・編集範囲、速度FF、開始停止の遷移、モーター出力の呼出し順を維持した。既存速度試験の`veloCtrl.kp`出力も変更していない。
3. `speedParameters.h`へ19項目の構造体・初期値・項目番号・UI・上下限・保存順を集約した。保存は100倍整数の`%04d,`×19・95B・改行なし。旧18項目や途中欠落の修復、有効値の保持も維持した。既存の`bst500`初期値が`PARAM_BOOST_600`を参照する点は今回の整理では維持した。
4. SLIP/PATHの整数メタデータ取得を`logMetadataReadInt()`へ統合した。`0`/`0.00`など整数相当の表記を共通で扱う。完全一致キー・重複なし・有限値・int32範囲を検証する。PATHでも偽の接頭キー、重複、行数の端数や範囲外を拒否する。保存メタデータと各方式の正常な解析元条件は維持した。

検証は追跡対象の`validation/control_refactor/run.py`と`validation/log_storage/run.py`を使用した。

- 実関数をO0/O2・Wall/Wextra/Werrorで検証し、変更前とモーター729ケース・PID画面2,592フレーム・速度設定178ケースが一致した。メタデータ35ケースと実PATH読込も成功した。
- 通常・詳細ログの保存、CRC/I/O異常、DISTANCE/SLIP/PATH/SHORTCUT、現行V2、PC変換が成功した。CRC正常なbinのメタデータを置換し、同値表記の受理と重複・偽キー・範囲外など11ケースの拒否を確認した。
- 作業直前の`build/main-refactor-20261025/before/`と比較し、通常/詳細×直線/閉路カーブの記録・列定義の全バイト、型付き読込結果、4方式の計画が一致した。比較結果は`build/main-refactor-20261025/comparison/validation.json`。
- Debug、DebugMarker、Release、ReleaseDetail、CleanupDiagnosticsが成功した。既存診断との照合で新規warningなし。結果は`build/main-refactor-20261025/builds/validation.json`。RAM予約量は同じで、直前からFLASH使用量はDebugで848B、Releaseで832B減少した。

1ms制御・PID計算・スリップ判定・緊急停止・ログスキーマは今回変更していない。既存のセンサー角度計算も両端対策を含めて残した。前述の実機ログ13614〜13623は今回の4項目修正前のビルドであり、今回の実機検証には数えない。書き込みは未実施。実機で各PID画面の表示・選択・保存・停止を確認し、新ビルドで一次とDISTANCE→SLIPおよびPATH/SHORTCUTのログを取得して確認する。

ホスト検証で判明した符号比較と旧参照コードの警告について、[整数比較の再発記録](失敗事例/2026-10-21-slip-v2-host-signed-bounds.md)と[参照ハーネスの追記](失敗事例/2026-10-24-cleanup-host-fixture-contract.md)を更新し、書き込み・デバッグSkillの実行前手順へ反映した。

## 補助API・左右速度PID・横統計と旧モードの整理（2026-10-25）

ユーザー指定の順に未使用補助API・引数、左右速度PID、横スリップ統計を整理し、旧`BOOST_MARKER`を廃止した。

- 呼出しのない`getTimeUs()`、`IMU_GetLinearAccelerationZMps2()`、`isLineSensorSettingCorrupt()`、`SD_WaitReadyMs()`と宣言を削除した。校正破損フラグやSD内部の待機・同期処理は維持する。前後加速度の互換ラッパーを削除し、同じY軸線形加速度取得を直接呼ぶ。直線判定の未使用引数`encNow`も削除した。指定された`getAngleSensor()`は両端対策を含めて維持する。
- 左右速度PIDを`updateWheelSpeedPID()`へ共通化した。左右の積分値、前回目標、偏差履歴、FF保持値は独立する。目標変更時の積分リセットとFF更新、未飽和時のみの積分、積分±10000、出力±1000と演算順を維持する。平均速度PIDの式と条件は変更しない。
- 横ピークとON時間は既存の`SlipDetectionInterval.peakLat/onLatMs`へ集約し、`lateralCandidate.interval.peak/onMs`の二重保持・更新だけを削除した。横独自の`validMs/invalidReason`、窓、LPF、継続判定、無効復帰、保存成功時の区間リセットを維持する。ログ列、保存型、列順、スキーマCRC、検出器版4、閾値、SLIP計画式は変更しない。
- `BOOST_MARKER`の定義と表示を削除し、モード番号1を廃止・再利用禁止とした。現行番号は0/2/3/4を維持し、廃止・未定義値では開始要求を既存の開始禁止処理で解除する。ISR診断配列は添字1を空けて他の番号を維持し、廃止値の診断を記録しない。

追跡対象の`validation/control_cleanup/run.py`を追加した。O0/O2・Wall/Wextra/Werrorで、変更前の実左右PIDと64,000周期、実スリップ検出器と104,000周期を比較し、出力・保存されたfloatビット列・フラグ・区間統計が一致した。左右独立性、FF保持、積分制限、欠測・非有限値・再充填、時刻折返し、区間リセットと16bit飽和も確認した。全256モード値、既存開始禁止条件、ISR診断の添字維持が成功した。

Debug、DebugMarker、Release、ReleaseDetail、CleanupDiagnosticsの5構成が成功した。過去の全体ビルドも含めて照合し、新規warningなし。FLASH使用量は直前からDebugで272B、Releaseで192B減少し、リンカ報告のRAM予約量は同じ。結果は`build/remaining-cleanup-20261025/builds/validation.json`。周期処理へブロッキングI/O、割込み禁止区間、表示処理は追加していない。FF計算の更新条件と呼出し回数も維持した。

センサー角度を含むADC、PID画面・速度設定・メタデータ、通常/詳細の保存とCRC/I/O異常、PC変換も成功した。変更前と通常/詳細×直線/閉路カーブを比較し、記録・列定義の全バイト、型付き読込結果、DISTANCE/SLIP/PATH/SHORTCUTの計画が一致した。比較結果は`build/remaining-cleanup-20261025/comparison/validation.json`、制御式・割込み経路・未使用APIの監査は`verification.json`へ保存した。変更前スナップショットなしの通常回帰実行も成功した。

今回も実機書き込み・走行・ISR最大時間の測定は未実施。前述の13614〜13623は今回の実装より前のログであり、今回の実機確認には数えない。新ビルドでは左右速度追従と横フラグ・ON時間・有効時間、DISTANCE→SLIPの計画と停止保存を確認し、ISR最大時間を別途測定する。

## 校正基準・速度メタデータ・未使用定数の整理（2026-10-25）

- `calibrationConstants.h`へ検証済みの距離換算58019 pulse/mとIMU校正100サンプルを集約した。`PULSE_METER`と`IMU_CALIBRATION_SAMPLE_COUNT`は実機側の別名とし、スリップ換算・一次ログ検証・PATH受入条件も共通基準を使う。実機設定と検証済み基準の照合は維持し、実機側だけを変更した未検証設定では経路元を無効にする。
- `SPEED_PARAM_FIELDS`の末尾へメタデータ出力フラグ0/1を追加し、SDcard.cの手作業の速度項目列挙を共通定義からの展開へ変更した。既存15項目の名前・順序・数値書式だけを出力し、SD保存は従来どおり全19項目・95Bとする。構造体の並び、UI調整幅、上下限、旧18項目や途中欠落の修復も維持する。
- 未使用の`PARAM_BOOST_500`を削除した。`bst500`の既定値は`PARAM_BOOST_600`による1.6 m/sを維持し、SD実機値を変更していない。

ローカルの`validation/constants_metadata/run.py`で20,001通りの距離換算をO0/O2の保存ビット列で確認し、通常/詳細各128ケースで実メタデータ全体が変更前とバイト一致した。実機側だけを58020 pulse/mまたは101サンプルにした隔離コピーでは、一次ログが理由10/9で拒否された。ビルド日時だけを固定し、負値と丸め境界も確認した。結果は`build/constants-metadata-validation/validation.json`。

設定178ケース、PID画面、左右PID64,000周期、スリップ104,000周期、全256モード、ADC・センサー角度、保存CRC/I/O異常、PC変換も成功した。通常/詳細×直線/閉路カーブの記録・列定義の全バイト、型付き行、DISTANCE/SLIP/PATH/SHORTCUT計画も変更前と一致した。`build/constants-metadata-20261025/comparison/validation.json`と`verification.json`へ保存した。validationはユーザー指定どおりローカル専用でGit管理対象外。

Debug、DebugMarker、Release、ReleaseDetail、CleanupDiagnosticsの5構成が成功し、新規warningなし。FLASH・RAM使用量は直前の整理後と同じ。制御式、割込み経路、モード番号、ログスキーマを変更していない。実機書き込み・走行・新ビルドの最初のログ確認は未実施。今回の結果は新しい物理校正値を実測したものではない。

試験側で発生した拒否理由番号の誤認と、比較元構造体の版の取り違えは、[既存の失敗記録](失敗事例/2026-10-24-cleanup-host-fixture-contract.md)へ追記した。修正後に通常/詳細と横統計統合前/後の両比較元で検証し、sd-settingsとflash-debugの実行前手順も更新した。

### bst500初期値の追加指示

ユーザーの追加指示で`PARAM_BOOST_500=1.5F`を復活させ、`SPEED_PARAM_FIELDS`の`bst500`初期値に指定した。既定値は1.6から1.5 m/sへ変わる。SD上の有効な既存値は引き続き優先し、保存済みの1.6等を自動で1.5へ置換しない。設定ファイル欠落時は19項目・95Bで作成し、10項目目を`0150,`として保存する。

O0/O2の速度設定180ケースで初期値・欠落作成・既存SD値優先を確認し、PID画面とメタデータも成功した。変更前比較はbst500初期値1.6→1.5だけを意図した差とした。Debug/Releaseビルドも成功し、実機・実SDは操作していない。試験側の小数定数比較は[保存型比較の再発記録](失敗事例/2026-10-12-host-float-snapshot-comparison.md)とSD設定Skillの事前手順へ反映した。


## ログ読込とPATH補助処理の所属整理（2026-10-25）

読込APIの宣言と実装を`logSource.h/.c`へ揃えた。従来`SDcard.c`に置かれていた先読み・レコード復元・CRC検証・型付き解析行の生成を移し、`logMetadata.c/.h`の整数メタデータ解析も同じモジュールへ統合した。`SDcard.c`は保存・確定・復旧・設定I/Oを担当する。

保存時の全件検証と読込は同じスキャンを使用する。保存時に固定したIMU校正状態と書込成否を非公開の引数で渡す。共有I/O 6 KiB、メタデータ4 KiB、Flash上の列定義はそれぞれ一つのまま保持し、保存キュー排出・停止中・FatFsロック下という切替条件を維持する。二つの実装間だけの契約は、プロジェクトのヘッダ配置に合わせて`Core/Inc/logStorageInternal.h`へ置く。`SDcard.c`と`logSource.c`専用と明記し、他のヘッダから共有領域を参照させない。

`pathPolicy.h`のPATH専用判定5関数は`pathFollower.c`のstatic関数へ移した。PATH/SHORTCUTの生成レベルとゴール選択の単純なラッパーは`control.c`の分岐へ戻した。速度・SHORTCUT設定で共通使用する欠落／範囲外項目の修復は、`SDcard.c/.h`の`sdRepairSparseSettings()`へ移した。`pathPolicy.h`と`logMetadata.c/.h`は削除した。

確認結果:

- Debug、DebugMarker、Release、ReleaseDetail、CleanupDiagnosticsの5構成でビルド成功。各構成の既存warning 6件と照合し、新規warningなし。
- 通常／詳細ログの保存・読込、CRC・I/O異常、DISTANCE／SLIP／PATH／SHORTCUTの計画生成、PC変換値の検証が成功。
- 作業前スナップショットと通常／詳細×直線／閉路カーブを比較し、全記録・列定義バイト、型付き解析行、4方式の計画結果が一致。メタデータも各128ケースで全バイト一致。
- 設定180ケースとメタデータ35ケースをO0/O2で確認し、部分反映・旧形式修復・不正値拒否を維持。移動した実関数の本体も比較した。

結果はローカル専用の`build/log-path-ownership-20261025/`へ保存した。`validation/`は引き続きGit管理対象外。ログ形式・制御式・安全条件は変更していない。書き込み・実機確認は未実施であり、新ビルドで停止時保存と次走解析、再起動後の読込、PATH／SHORTCUT走行を確認する。

## ログ書込バッファの実装統合（2026-10-25）

`SDcard.c`だけが使っていた`logBufferWriter.h`の状態構造体と12関数を、同ファイルの内部処理へ移した。関数本体、static inline指定、volatile指定、バッファ切替順序と失敗時処理は維持する。保存・読込で共有する512 Bセクタ、2 KiBバッファ、3本という定数は`logStorageInternal.h`へ移し、共有I/O領域は6 KiBのままとした。`logBufferWriter.h`を削除し、ログ関係の対象6ヘッダを5ヘッダへ整理した。

残るヘッダは、列定義の`log_schema.h`、PC変換器とも共有する形式定義の`logBinary.h`、複数モジュールが使う診断の`logPerformance.h`、読込APIの`logSource.h`、保存・読込間の内部定義の`logStorageInternal.h`とする。書込キューの実装を公開ヘッダから取り込む必要はなくなった。

Debug、DebugMarker、Release、ReleaseDetail、CleanupDiagnosticsの5構成でビルド成功。統合前と各構成の既存warningおよびRAM・FLASH使用量が一致した。通常／詳細の保存・CRC・I/O故障・PC変換検証も成功し、統合前の実ソースと直線／閉路カーブの全記録・列定義バイト、型付き解析行、DISTANCE／SLIP／PATH／SHORTCUTの計画が一致した。移動した全12関数・状態構造体・定数の一致と、それ以外の`SDcard.c`処理が変わっていないことも確認した。

確認結果は`build/log-buffer-integration-20261025/`へ保存した。ログ形式・制御経路・安全条件の変更はない。実機への書き込みと実SDでの停止時保存・次走読込は未実施。
