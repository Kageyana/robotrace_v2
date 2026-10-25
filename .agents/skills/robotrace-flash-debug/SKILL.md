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

最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。ビルド・PCハーネス・ST-Linkを操作するときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。

binメタデータとPC変換CSVの1行目を比較する前に、両形式の改行規約と実バイトを確認する。binはLF、PC変換CSVはCRLFのため、内容照合では行末だけを正規化し、項目名・順序・値を含む他の全バイトは維持する。無変換コピーのバイト一致契約と混同しない。実行はリポジトリルートの明示workdir・必要権限で `python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe` とし、両プロファイルの全メタデータ内容・CRC・CSV値一致が通るまで完了としない。[bin/CSV改行差の事例](../../../doc/失敗事例/2026-10-17-shared-header-line-ending-check.md)を参照する。

ログ形式変更のPC変換器検証では、`C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`もGCC生成ホストexeとして最初の起動からrequire_escalatedを使用する。今回の解析フォルダ内の合成bin/出力CSVに限定し、Python subprocessのstdout=PIPE/stderr=STDOUT等で出力を取得、終了コード0・出力実在を確認してから全CRC/列/数値を照合する。PowerShellのStandardOutputEncodingエラーだけで原因を決めず、直接取得した起動エラーを確認する。詳細は[PC変換器起動の追記](../../../doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md)。

新規warningの比較前に、変更前ログが対象翻訳単位と構成を含むか確認する。最終の増分ビルドログだけを基準にせず、初回全体ビルド記録も照合し、既存診断と追加分を区別する。[増分ログの誤判定](../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)参照。

マージ後に補助ホストテストを追加実行する前に、そのテストの固定recordBytes・必須列・参照ログ番号の期待を現行のlog_schema.h・SDcard.c・courseAnalysis.cと照合する。旧形式の固定値を検出したら製品を旧契約へ戻さず、試験の責務を分けて入力・期待を更新する。手動参照メタデータ試験はレコード長をLOG_RECORD_SIZE_BYTESで照合し、66列/170 B・72列/179 Bの固定仕様は独立スキーマ試験で検証する。SLIP参照番号は実際の元DISTANCE番号を検証し、通常/詳細の両方を必要権限・ファームウェアルートの明示workdirで再実行する。[補助テストの旧契約](../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)参照。

## 基本方針

- 接続・安全条件はリポジトリの `AGENTS.md` を正とする。書き込みは機体とST-Linkが接続されているときだけ実行する。
- ST-Link接続中に実走行しない。空転確認では機体を固定して車輪を浮かせる。緊急停止条件の一時無効化は、静止状態で確認できる対象条件に限る。
- 操作を始める前に `doc/失敗事例/` から関連事例を検索する。失敗した接続・停止操作を繰り返さず、対象MCUと安全状態を確認してから進む。
- CodexからCLI/GDBで操作する場合は、先に [CodexからのST-Link操作手順](references/codex-stlink-cli-debug.md) を読む。

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

- 書き込み成功と `initSystem()` のセンサー初期化結果を確認する。通常の書き込みだけなら空転確認は必須ではない。
- 高速走行前はセンサー初期化成功とバッテリー電圧7.5 V以上を確認する。
- 緊急停止条件を一時変更した場合は、変更条件、理由、残る実機リスクを報告する。
