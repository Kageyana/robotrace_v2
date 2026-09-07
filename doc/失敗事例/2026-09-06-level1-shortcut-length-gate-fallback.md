---
type: codex-failure
date: 2026-09-06
task: "Level 1ショートカット走行ログ確認"
status: open
severity: medium
tags:
  - codex/failure
  - control
  - logging
  - shortcut
---

# Level 1経路が短縮率0.5%未満でLevel 0へフォールバックした

## 要約

一次走行12345に対するautoStart 3〜5の12347〜12349が、すべて`optimalTrace=4`、`shortcutLevel=0`となりLevel 1を実行しなかった。現行アルゴリズムをPC上で再現すると短縮率は約0.296%で、採用条件0.5%に届かない。

## 発生した状況

- タスク: Level 1ショートカット走行の最新ログ確認
- 実行環境: STM32F446、routeControllerVersion=10、gitCommit=9795729
- 一次走行: 12345
- 二次走行: 12346〜12349

## 何を試したか

1. 12345〜12349のログヘッダと経路追従指標を確認した。
2. 12345の`x`、`y`、`courseMarker`を現行の40 mm再標本化と制約付きElastic Bandへ入力した。
3. 元経路長、生成経路長、オフセット、自己交差、固定点数を集計した。

## 結果・エラー

```text
12347〜12349: optimalTrace=4, shortcutLevel=0, emcStop=0
route points=903
expanded anchors=590
source length=36065.158 mm
shortcut length=35958.466 mm
reduction=0.2958%
required reduction=0.5%
new self intersections=0
```

## 原因

確認できた直接原因は、`routeGenerateShortcut()`が生成経路長を元経路の99.5%以下にできない場合に生成失敗とすること。この一次経路の短縮率は約0.296%のため、`maxLevel=1`で要求してもLevel 0へフォールバックする。

短縮率が小さい一因として、マーカー検出点の前後100 mm展開後に903点中590点が固定点となり、変形可能区間が限られている。

なお、ログに`shortcutSettings.maxLevel`が記録されていないため、実機で読み込まれた`maxLevel`が0か1かはこのログだけでは確定できない。

## 解決・回避策

経路制御バージョン11で、全コースを平滑化するElastic Bandを直線回廊方式へ置換した。マーカー固定を廃止し、安全な回廊が1区間以上あり、生成後の全長が5 mm以上短い場合にLevel 1を採用する。12345のPC再現では2回廊、約29.7 mmの短縮を確認対象とする。

実機10本の完走率、合法余裕、indexジャンプをまだ確認していないため、ステータスは`open`を維持する。

PC再現では、12345が2回廊・29.669 mm短縮、2025全日本7180が3回廊・44.788 mm短縮となった。最大移動量はそれぞれ32.000 mm、31.780 mmで、新規自己交差はいずれも0だった。

## 今後の予防策

- `shortcutSettings.maxLevel`と経路生成失敗理由をログまたは起動表示で確認できるようにする。
- Version 11以降はログヘッダの`shortcutBuildStatus`、`shortcutCorridorCount`、`shortcutReduction_mm`を確認する。
- Level 1走行前に一次ログをPCで生成処理し、短縮率、固定点率、最大オフセット、自己交差を確認する。
- 生成条件変更後は低速で最低10本の実機ログを取得し、合法余裕、indexジャンプ、フォールバック、完走率を確認する。

## 関連

- 関連ノート: [[2026-08-23-shortcut-gate-requires-explicit-approval]]
- 関連ノート: [[2026-09-06-path-replay-nearest-index-jump-hairpin]]
- 解析対象: 12345〜12349
