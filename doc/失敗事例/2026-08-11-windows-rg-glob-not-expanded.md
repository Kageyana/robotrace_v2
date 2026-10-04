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

## 2026-10-03 再発と追加確認

yI検証で `rg ... robotrace_v2/Core/Inc/*h` を渡してos error 123が再発した。既存の対策がそのコマンドで適用されなかった。検索を `rg ... robotrace_v2/Core/Inc -g '*.h'` へ変更し、main.hとmarkerSensor.hのGPIO定義を取得できた。今後の検索はファイル列挙後に実在パスを使うか、ディレクトリ＋-gに限定する。存在を推測したlocalization.cも使わず、rg --filesで実装の所在を確認する。対処を検証したのでresolvedを維持する。

## 2026-10-03 周回ドリフト調査での再発

周回ドリフト調査でもSrc/*.cを検索パスに渡してos error 123が再発し、推測したsystem.cも存在しなかった。ディレクトリと-g '*.c'による検索でcalcXYcieの呼び出しがSDcard.cにあることを確認した。検索コマンドを組み立てる際は実在ファイルかディレクトリだけをパスとして渡す。対処を確認済み、resolved。

## 2026-10-03 IMU・距離・時刻切り分けでの再発

Src/log*をパスに渡した検索でos error 123、推測したtim.cで存在しないエラーが再発した。既存対策が適用されていなかった。ディレクトリ＋-gによる検索でwriteLogBufferPutsがSDcard.c、割り込み優先度がstm32f4xx_hal_msp.c、TIM6呼出しがmain.cにあることを確認。ファイル名を推測する前に`rg --files robotrace_v2/Core/Src`で一覧を取得し、検索呼出しのパス引数にワイルドカードを含めないことを再確認した。対処検証済み。
