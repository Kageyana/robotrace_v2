---
name: robotrace-flash-debug
description: robotrace_v2をSTM32CubeCLT、STM32_Programmer_CLI、ST-Link/GDBでビルド・書き込み・デバッグするときに使う。Codexからの接続確認、停止・復旧と失敗手順の回避も扱う。
---

# robotrace_v2 書き込み・ST-Linkデバッグ

## 2026-10-25：現行ログとSLIPフラグ集計

有効性・無効理由（電流有効性を含む）と記録間隔内統計16列はbin/CSVから削除済み。経路診断6列追加後の現行は通常profile=1・56列/154 B、詳細profile=0・62列/163 B。50列/135 B・56列/144 BはV2整理だけの時点のサイズ。`slipV2Flags`はbit0=縦、bit1=採用横、bit2=Yaw不整合。SLIP計画は版3・`slipPlannerPolicy=flag_sample_fraction`で、optimalIndex別の縦/横ON記録数/全記録数を使用する。時間率ではなく、最小ON時間3msの追加選別は廃止した。検出器版4・3/5ms継続判定と内部計算は維持する。

未観測区間は増速しない。有効性が未記録のため、無効時のフラグ0と正常OFF、記録間に終了した短いONは区別できない。更新後は新しい一次とDISTANCEを取得する。機体は旧スキーマを拒否し、PC自己記述変換・全CRC検証は維持する。現行の確認はリポジトリルートから必要権限で`python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`。以下の旧列数・区間時間率の記述は履歴とし、現行は[正式採用とSLIP走行](../../../doc/slip-v2-active.md)と`log_schema.h`を優先する。

経路診断6列は末尾の`pathErrorY_mm`（F32・mm）、`pathErrorHeading_cdeg`（S16・0.01deg）、`pathState`（U8）、`pathYawFF_dps/pathYawLateralFB_dps/pathYawHeadingFB_dps`（各F32・deg/s）。1ms更新の最新値を距離間隔で保存し、PATH/SHORTCUT以外は0。方位偏差は最近傍接線−機体方位、操舵3成分は合計の±1800deg/s制限前。操舵式は維持する。経路更新版18は1msの自己位置積分→経路更新→PIDで実行し、SDの位置/方位補正係数は5ms基準の1/5を毎回適用する。退避100ms・復帰200ms・混合率切替100msは維持する。詳細は[自己記述ログ](../../../doc/self-describing-log.md)。

## 実行前の再発防止

版21へ戻す場合はprepare_controller21_restore.pyで現807dfea…/復元bc7a3fca…の実像・実265配置を別領域へ確定し、analysis/controller21_restore_20261031/hardware.py capture --image installed→flash --before→boot --beforeを使う。旧ベンチのinstalled/diagnosticの役割を流用しない。ST-Link enumeration failedでも所有サーバー終了を確認して接続復旧を依頼し、GDB読取りやダウンロードが始まっていないことを原ログで保存する。再接続後は5分以内の新しい実停止captureを必須とする。[版21復元時の未列挙](../../../doc/失敗事例/2026-10-27-stlink-usb-during-flash.md)。

Releaseを型付きで読戻す前は、CMakeCacheだけでなくcompile_commands.jsonの全対象Cで最後の-g/-Oを確認する。このツールチェーンの実効Releaseは-Os -g0で、全体CFLAGSの-g3は上書きされる。型情報は独立コピーの設定で-Osを維持して有効化し、実ELFのsizeofと元Releaseの実行命令列を照合する。前段失敗後は配置準備を呼ばず、run_path_release_types_pipeline.pyで完了証跡を確認する。[実効フラグと依存の事例](../../../doc/失敗事例/2026-10-31-release-dwarf-effective-flags.md)参照。


モーター/HALのモックを作る前に、実マクロ値だけでなく使用式（&handle、handle.Init、handle.Instance等）を読み、アドレス取得可能な変数と実参照形を用意する。MOTOR_TIM_HANDLERを定数0で代用しない。停止演算のvalidate_path_stopped_bench_host.pyは実setter抽出・全int16/電圧/合成指令・OFF時通常出力を検査し、完成O0演算を保全してモーター試験だけ再開する。[ハンドルの再発](../../../doc/失敗事例/2026-10-13-adc-host-fixture-contract.md)参照。


Programmer CLI等の.logを読む前に、実保存元がwrite_bytesか復号済みwrite_textか確認する。生ログの成功句・IDはread_bytesとASCIIバイト句で照合し、.log拡張子からUTF-8と仮定しない。証跡消費だけの失敗で実Flash書込みを反復しない。record_path_input_capture_deployment.pyの実原ログ検証を参照。[生CLIログの再発](../../../doc/失敗事例/2026-10-04-fpu-audit-log-encoding.md)。

入力RAM計測の移行前はpath_input_capture_hardware.py preflight→capture --image installed→flash --before <実回収先>の順にする。現在/計測の265配置と全Flash一致を分離し、書込み直前は5分以内の実RAMから停止を再復号する。合成fixtureを渡さず、CPUのIPSRがIRQを示す場合はNVIC activeとの整合を確認する。Core未検出は同条件を反復せずユーザーの再接続を待つ。ダウンロード開始後の異常はunknownとして保持し、Flash読戻しで確定する。[10/30入力計測の未検出](../../../doc/失敗事例/2026-10-27-stlink-usb-during-flash.md)参照。

制御フィクスチャの識別子抽出前に、実宣言が#define/enum/型のどれかと一致数を確認する。BOOST_PATH_CURVEは#define 5で、enumの代入式を仮定しない。validate_path_input_capture_host.pyは実宣言をcaller_preflight.jsonへ保存し、実timerフック4608条件×O0/O2を照合する。[宣言形式の再発](../../../doc/失敗事例/2026-10-13-adc-host-fixture-contract.md)参照。


外部CLIのnm/size出力を行正規表現へ渡す前に、bytes.decode後のCRLF残存と絶対シンボルの列数を確認する。生ログを保持し解析文字列をsplitlines/改行正規化で扱う。集計のみ失敗した完了ELFは入力・CMakeCache・全体ログ・SHAを保全し、リンクをやり直さず未実行の集計から再開する。[nm行末と再開](../../../doc/失敗事例/2026-10-30-arm-symbol-newline.md)参照。


新しい制御フィクスチャの生成前は、対象Core/Inc/courseAnalysis.hに実宣言があることを入口で確認する。PATHの現行識別子はBOOST_PATH_REPLAYで、BOOST_PATHと推測しない。check_tracking_guard_zero_decel.pyは実ヘッダのモード宣言を検査してからCを生成し、減速度0/無効/PATHをO0/O2で確認する。[試験契約の再発](../../../doc/失敗事例/2026-10-13-adc-host-fixture-contract.md)参照。


小数パラメータの上下限を含むPC候補は、引数と境界値の保存精度を揃え、最小/最大の実型入力を候補評価前に試す。5状態試作では0.8秒が直接の0.8f比較で拒否され、境界をvolatile floatへ保存して範囲を変えず修正した。30既知状態×O0/O2と460再生を確認。[設定境界の再発](../../../doc/失敗事例/2026-10-12-host-float-snapshot-comparison.md)参照。


保存RAMを追加復号するときはenumも固定32bitと仮定せず、対象ELFのDWARFからアドレスとsizeofを取得する。1/2/4byteの復号型と列挙値範囲を確認してから配列へ進む。13924のrunMemoryOwner/followerStateは1byte。確認はanalysis/script/read_marked_071234_state.py。[enum幅の不一致](../../../doc/失敗事例/2026-10-30-typed-ram-enum-size.md)参照。


静止ADC比較でもGDBのcallを使わない。現CubeCLT/GDBは直接関数呼出し後にsetup/mainのIPSR/VECTACTIVE32・NVIC active0となった。異常なCPU状態では負荷や走行へ進めず、通常起動へ戻す。ON/OFFは通常UIのdisplay4/sensors7の経路を使い、非走行/出力0/IPSR0を各段で保存する。サーバーは比較全体で1つを所有し、終了時に負荷0・優先度3・元の凍結・センサーOFFを確認してdetachする。公開未完validSamples0の場合はstale0/gap0を正常と解釈しない。[GDB直接呼出し後の例外状態](../../../doc/失敗事例/2026-10-28-gdb-call-exception-state.md)参照。

計測/校正のホスト化前に、DWT.hのマクロ形・共有値の型修飾子・motor.hの現行引数・courseAnalysis.hの現行モードを照合する。getCycleCounter()は末尾セミコロン付きマクロなので、式内呼出しを関数モックだけで合格させない。実マクロを再利用し計測OFF/ON両方をビルドする。現行の確認はvalidation/isr_diagnostics/run.py、validation/line_sensor_adc/run.py、robotrace_v2/tests/run_autostart_calibration_tests.pyと保存両プロファイル。派生読出し器は生成成功→DWARF配置→self-testを順に確認し、いずれか非0なら後続を起動しない。flash入口で対象ELF/expected.bin SHA・234項目・decoder試験成功を確認する。[ADCホスト契約の10/28節](../../../doc/失敗事例/2026-10-13-adc-host-fixture-contract.md)、[派生解析の依存ゲート](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)参照。

最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。ビルド・PCハーネス・ST-Linkを操作するときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。

binメタデータとPC変換CSVの1行目を比較する前に、両形式の改行規約と実バイトを確認する。binはLF、PC変換CSVはCRLFのため、内容照合では行末だけを正規化し、項目名・順序・値を含む他の全バイトは維持する。無変換コピーのバイト一致契約と混同しない。実行はリポジトリルートの明示workdir・必要権限で `python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe` とし、両プロファイルの全メタデータ内容・CRC・CSV値一致が通るまで完了としない。[bin/CSV改行差の事例](../../../doc/失敗事例/2026-10-17-shared-header-line-ending-check.md)を参照する。

ログ形式変更のPC変換器検証では、`C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`もGCC生成ホストexeとして最初の起動からrequire_escalatedを使用する。今回の解析フォルダ内の合成bin/出力CSVに限定し、Python subprocessのstdout=PIPE/stderr=STDOUT等で出力を取得、終了コード0・出力実在を確認してから全CRC/列/数値を照合する。PowerShellのStandardOutputEncodingエラーだけで原因を決めず、直接取得した起動エラーを確認する。詳細は[PC変換器起動の追記](../../../doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md)。

新規warningの比較前に、変更前ログが対象翻訳単位と構成を含むか確認する。最終の増分ビルドログだけを基準にせず、初回全体ビルド記録も照合し、既存診断と追加分を区別する。[増分ログの誤判定](../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)参照。

マージ後に補助ホストテストを追加実行する前に、そのテストの固定recordBytes・必須列・参照ログ番号の期待を現行のlog_schema.h・SDcard.c・courseAnalysis.cと照合する。旧形式の固定値を検出したら製品を旧契約へ戻さず、試験の責務を分けて入力・期待を更新する。手動参照メタデータ試験はレコード長をLOG_RECORD_SIZE_BYTESで照合し、66列/170 B・72列/179 Bの固定仕様は独立スキーマ試験で検証する。SLIP参照番号は実際の元DISTANCE番号を検証し、通常/詳細の両方を必要権限・ファームウェアルートの明示workdirで再実行する。[補助テストの旧契約](../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)参照。

### GDBスクリプト生成時の必須検査（2026-10-28）

このCubeCLT 1.19.0のARM GDBはPython非対応。実機接続前に生成済み.gdbを検査し、コマンド先頭のpython/pi/python-interactiveを拒否する。照合・集計は外部Python/PowerShell、GDB内は標準コマンドだけを使用する。非対応ブロック内のfinallyは接続解除を保証しない。エラー時は所有プロセス終了を確認し、リセットなしでCCR/PWM・開始要求・PRIMASK/Faultを回収してから進む。[再発と復旧](../../../doc/失敗事例/2026-10-04-cubeclt-gdb-python-unavailable.md)参照。

## 基本方針

- 接続・安全条件はリポジトリの `AGENTS.md` を正とする。書き込みは機体とST-Linkが接続されているときだけ実行する。
- ST-Link接続中に実走行しない。空転確認では機体を固定して車輪を浮かせる。緊急停止条件の一時無効化は、静止状態で確認できる対象条件に限る。
- 操作を始める前に `doc/失敗事例/` から関連事例を検索する。失敗した接続・停止操作を繰り返さず、対象MCUと安全状態を確認してから進む。
- CodexからCLI/GDBで操作する場合は、先に [CodexからのST-Link操作手順](references/codex-stlink-cli-debug.md) を読む。

### ビルド集計器の実行前（2026-10-28）

新設・変更した集計器では、read_text/write_textの全呼出しへencodingを明示したか、AST等で確認する。.ld、CMakeCache等も対象で、標準出力のUTF-8指定だけで済ませない。また既存warningがあるプロジェクトへassert not warningsを置かず、コンパイル単位の被覆を確認した変更前全体ログとの差分で新規診断を判定する。初回ログと再確認の増分ログは別保存。今回の検査はリポジトリルートから必要権限でbuild/isr-interval-validation/preflight_helpers.py、finalize.pyの順に実行する。[リンカ文字コード](../../../doc/失敗事例/2026-10-04-fpu-audit-log-encoding.md)、[警告ゼロ仮定の再発](../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)参照。

## ビルド

`robotrace_v2/` で対象構成をビルドする。書き込むELFとデバッグに読み込むELFを同じ構成にする。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Releaseの場合:

```powershell
cmake --preset Release
cmake --build --preset Release
```

ツール不足やコマンド失敗をビルド成功として扱わない。

ARM/ホストGCC起動前に`doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md`のDLL再配置エラー事例を確認する。この環境で同じIllegal System DLL Relocationが出たらソース不良と判断せず、同一コマンドを承認された制限外環境で検証する。コンパイラ起動失敗とコンパイル診断を区別し、根本原因は未解決として扱う。

## 書き込み

1. 機体とST-Linkの接続、対象MCUのDevice ID、ビルド成果物を確認する。ST-Linkの列挙とターゲット電圧だけではMCU接続成功と判定しない。
2. VS Codeの `CubeProg: Flash project (SWD)` タスク、または `STM32_Programmer_CLI` で意図したELFを書き込む。通常SWDで接続できない場合は、参照手順にある100 kHz・Under Resetを使う。
3. 書き込み・照合・起動の成否をコマンド出力で確認する。Device IDが取得できない状態で書き込みを続けない。

## デバッグ

VS Codeからは `Build & Debug Microcontroller - ST-Link` または `Attach to Microcontroller - ST-Link` を使う。Codexからの直接操作は参照手順に従い、GDBサーバーとProgrammer CLIでST-Linkを同時に保持しない。モーター回転中は不要なブレークポイントでコアを停止しない。

## 実行後

起動後のSD0を同条件の走行準備完了としない。IMU/SD・有効電圧・出力0・CPU状態に加え、比較元ログのメタデータと実際のSD速度/制御設定を照合する。電源/SD復帰後は停止状態で起動確認を取り直し、同一像の再書込みを前提にしない。FF2はリポジトリルートから`python build/path-yaw-ff-validation/audit_hardware.py`で全Flash/234項目配置/22設定を照合し、`same_condition_trial_ready`と実走未検証を分ける。[FF2起動時の電源/SD復帰](../../../doc/失敗事例/2026-10-27-stlink-usb-during-flash.md)参照。


- 書き込み成功と `initSystem()` のセンサー初期化結果を確認する。通常の書き込みだけなら空転確認は必須ではない。
- 高速走行前はセンサー初期化成功とバッテリー電圧7.5 V以上を確認する。
- 緊急停止条件を一時変更した場合は、変更条件、理由、残る実機リスクを報告する。
