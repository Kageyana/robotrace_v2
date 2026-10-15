---
type: codex-failure
date: 2026-08-11
task: "Skill frontmatter の確認"
status: resolved
severity: low
tags:
  - codex/failure
  - windows
  - shell
---

# PowerShellでUnix風globを前提にしたrg確認が失敗した

## 2026-10-15 CSV変換中のモーター指令調査での再発

`Core/Src/interrupt*`をrgのパスに渡してos error 123、存在未確認の`buzzer.c/.h`を指定してos error 2が発生した。既存の事前列挙対策を適用できていなかった。実在する`Core/Src`全体を検索し、1ms処理が`timer.c`、ホイールクリック出力が`setup.c`にあることを確認した。追加の予防手順として、モジュール名が不明な初回検索は必ず`rg -n '検索語' robotrace_v2/Core/Src -g '*.c'`とし、個別ファイルへ絞るのはその結果に実在パスが出た後だけとする。同形式で`void wheelClick`の検索と終了成功を確認したためresolvedを維持する。ファームウェアの変更・実機操作は行っていない。

同日のSD高速化案調査でも、存在を確認せず`FATFS/App/ffconf.h`、`Core/Src/log.c`、`Core/Inc/log_buffer_writer.h`、直下`.vscode/tasks.json`を指定して読込・検索が失敗した。初回の事前列挙が徹底できていなかった。`rg --files`で実在する`FATFS/Target/ffconf.h`、`Core/Inc/logBufferWriter.h`を確認し、ログ実装は実在ディレクトリの検索から`SDcard.c`と特定、タスクは存在するファームウェア側`.vscode/tasks.json`を確認した。次回はツール呼出しを「一覧取得」「取得結果の実在パスによる読込」の2段階に分け、同一呼出し内で一覧を待たず推測パスを追加しない。実在パスでの設定・ログ実装取得に成功し、resolvedを維持する。

## 2026-10-15 Fast単独版の実走確認での再発

13248～13256の事例検索で`doc/失敗事例/2026-10-14-*.md`をパス引数へ渡し、os error 123が再発した。前回の注意事項だけでは初回呼出しへの反映が不十分だった。実在ルートと`-g '2026-10-14-*.md'`へ分離して関連記録を取得した。走行CSVの解析・原本には影響していない。

追加対策として`analysis/script/search_failure_notes.ps1`を作成した。検索ルートを実在する失敗事例フォルダへ固定し、絞込み文字列を常に`-g`へ渡す。次回の事例検索は`powershell.exe -NoProfile -ExecutionPolicy Bypass -File analysis/script/search_failure_notes.ps1 -Pattern 'metadata|cntlog|直線' -FileGlob '2026-10-14-*.md'`を使う。上記コマンドで関連記録の取得と終了成功を確認し、resolvedを維持する。

同補助スクリプトの初回実行は、日本語文字列を含むUTF-8 BOMなしの`.ps1`をWindows PowerShell 5.1が既定コードページで解釈し、引用符終端のParserErrorになった。UTF-8 BOM付きへ保存し直すと同じコマンドが成功した。このWindows PowerShell用補助はBOM付き保存を維持し、変更後は実際の`powershell.exe -File`で日本語パターンとワイルドカード絞込みの組合せを検証する。ファームウェアのCソースの文字コード規約は変更しない。

## 2026-10-14 追加ログ解析での再発

過去事例検索へ`doc/失敗事例/2026-10-14*`をパスとして渡し、os error 123が再発した。既存対策を初回呼出しへ適用できていなかった。`rg -n -g '2026-10-14*.md' 'メタデータ|小数|時刻|区間|閉路' doc/失敗事例`へ修正し、整数メタデータ・補正距離・マーカーの関連事例を取得できた。対処検証済みでresolvedを維持する。検索コマンドのパス引数は実在ルートだけとし、`*`を含む場合は実行前に必ず`-g`へ移す。同時に走行CSV探索は数値ファイル名だけを列挙する`check_additional_logs_13238_13247.py`の`discover()`へまとめ、型変換前に対象を選別する。

## 2026-10-12 次作業提案時の再発

SLIP関連の過去事例検索で、`doc/失敗事例/2026-10-*-*log*`等をrgのパス引数へ渡し、os error 123が再発した。既存対策を検索呼出しへ適用できていなかった。検索ルートを`doc/失敗事例`、絞込みを`-g '*log*.md' -g '*slip*.md'`へ変更し、関連事例を取得できた。対処検証済み、resolvedを維持する。次回はrg呼出し前にパス引数の`*`有無を確認し、存在するディレクトリと`-g`へ分離する。

## 2026-10-04 自動開始前校正調査での再発

同日の復帰再調査でCore/Inc/BMI088*を検索パスへ渡してos error 123が再発した。再発防止策が適用されていなかった。Core/Incをパスとし-g 'BMI088*.h'へ切り替えてジャイロ定義の検索成功を確認した。今後、検索パスは実在ディレクトリまたは実在ファイルのみとし、パターンは必ず-gへ渡す。

`Src/{setup,control,lineSensor,interrupt}.c` のBash形式の波括弧展開をPowerShellへ渡してParserErrorとなった。また、推測したinterrupt.cは存在しなかった。Srcディレクトリを検索してInterrupt1msの実装がtimer.cにあることを確認した。既存対策の適用漏れであり、今後はrgの検索パスを実在ディレクトリ＋-gへ限定し、対象ファイル名を推測しない。ディレクトリ検索で対処確認済み、resolvedを維持する。

## 要約

Skill frontmatter を確認するために `rg -n "^name:|^description:" .agents/skills/*/SKILL.md` のような Unix シェル前提の glob 指定を使い、Windows PowerShell では期待通り展開されず確認コマンドが失敗した。後続で検索対象をディレクトリにする方法へ切り替えて確認した。

## 発生した状況

- タスク: `.agents/skills/*/SKILL.md` の frontmatter 検証
- 実行環境: Windows PowerShell
- 前提条件: Unix 系シェルのように `*` がファイルパスへ展開される想定だった

## 何を試したか

1. `rg` に `.agents/skills/*/SKILL.md` を渡して frontmatter を検索した。
2. パス glob が期待通り処理されず、確認に失敗した。
3. `rg ... .agents/skills` のようにディレクトリを対象にする形へ変更した。

## 結果・エラー

具体的なログは保存していないが、Windows のパス glob 前提違いにより確認コマンドを再実行する必要があった。

## 原因

PowerShell と Unix シェルでワイルドカード展開の挙動が異なる。特に外部コマンドへ渡すパス glob は、期待通りに展開されない場合がある。

## 解決・回避策

`rg` には glob 付きファイルパスではなく、検索ルートディレクトリ `.agents/skills` を渡して確認した。必要な場合は `Get-ChildItem -Recurse -Filter SKILL.md` で対象ファイルを列挙する。

## 今後の予防策

- Windows PowerShell では Unix 風のパス glob を前提にしない。
- `rg` は検索対象ディレクトリを渡し、必要なら `-g` オプションを使う。
- ファイル一覧を厳密に作る場合は `Get-ChildItem` を使う。

## 関連

- 関連ノート:
- 参考リンク:

## 2026-10-13 ADC位相調査での再発

KiCad検索で`Circuit/.../*.kicad_sch`をパス引数へ渡し、os error 123が再発した。ディレクトリを対象に`-g '*.kicad_sch'`へ変更し、部品値・TIM3_CH3・左右マーカーと電源ラベルを取得できた。既存対策を初回コマンドへ適用できなかったため、次回は検索実行前にパス引数に`*`や`{}`がないことを確認し、パターンを必ず`-g`に分離する。対処検証済み、resolvedを維持する。

## 2026-10-03 再発と追加確認

yI検証で `rg ... robotrace_v2/Core/Inc/*h` を渡してos error 123が再発した。既存の対策がそのコマンドで適用されなかった。検索を `rg ... robotrace_v2/Core/Inc -g '*.h'` へ変更し、main.hとmarkerSensor.hのGPIO定義を取得できた。今後の検索はファイル列挙後に実在パスを使うか、ディレクトリ＋-gに限定する。存在を推測したlocalization.cも使わず、rg --filesで実装の所在を確認する。対処を検証したのでresolvedを維持する。

## 2026-10-03 周回ドリフト調査での再発

周回ドリフト調査でもSrc/*.cを検索パスに渡してos error 123が再発し、推測したsystem.cも存在しなかった。ディレクトリと-g '*.c'による検索でcalcXYcieの呼び出しがSDcard.cにあることを確認した。検索コマンドを組み立てる際は実在ファイルかディレクトリだけをパスとして渡す。対処を確認済み、resolved。

## 2026-10-03 IMU・距離・時刻切り分けでの再発

Src/log*をパスに渡した検索でos error 123、推測したtim.cで存在しないエラーが再発した。既存対策が適用されていなかった。ディレクトリ＋-gによる検索でwriteLogBufferPutsがSDcard.c、割り込み優先度がstm32f4xx_hal_msp.c、TIM6呼出しがmain.cにあることを確認。ファイル名を推測する前に`rg --files robotrace_v2/Core/Src`で一覧を取得し、検索呼出しのパス引数にワイルドカードを含めないことを再確認した。対処検証済み。

## 2026-10-13 ログ項目整理での推測ファイル名の再発

ログ読込処理を調べる際、存在を確認せずlogReader.c/.hを検索対象へ指定し、os error 2となった。既存の事前列挙を適用できていなかった。`rg --files robotrace_v2/Core`で実在するcourseLogCsv.c/.h、logSource.h、logBinary.hを確認し、実読込がSDcard.cにあることも検索で確認した。次回は新しいモジュール名を検索パスへ追加する前に、必ずrg --filesの結果と照合する。対処検証済み、resolvedを維持する。
