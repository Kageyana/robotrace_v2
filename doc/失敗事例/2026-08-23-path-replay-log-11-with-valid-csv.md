---
type: codex-failure
date: 2026-08-23
task: PATH REPLAY 2走目の自動解析
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
---

# PATH REPLAY 2走目でError -11（PC側CSVは正常）

## 要約

実機の2走目自動解析で `Error -11` が発生した。PC側で確認できる12153～12159.csvは座標データを含む正常な一次走行ログであり、実機が読んだファイルまたは再読込処理に差異がある可能性がある。

## 発生した状況

- タスク: 一次走行ログからPATH REPLAY経路を生成
- 実行環境: STM32F446、FatFs、SDカード、`routeBuildFromLog()`
- 前提条件: 12153～12159.csvを一次走行ログとして使用

## 何を試したか

1. 12153～12159.csvのヘッダ、座標行数、走行距離、`cntlog`、`emcStop`を読み取った。
2. `pathFollower.c`の`routeBuildFromLog()`の`-11`判定とCSV列解析を確認した。

## 結果・エラー

```
実機: Error -11
PC側ログ: 各ファイル936～939行、x/y有効行936～939、座標走行距離9658～9689 mm、emcStop=0、cntlog欠落なし
```

## 原因

`-11`は有効座標点なし／走行距離40 mm未満、または再サンプリング後の経路点数不足で返る。PC側ログの数値からはこの条件に該当しないため、実機SD上の同名ファイルとの差異、解析対象ログ番号の差異、またはFatFs再読込（`f_lseek`後）の失敗が未確定原因。

## 解決・回避策

未解決。実機画面の`log N`と、実機SDカード上の`N.csv`のサイズ・先頭データを照合する。必要に応じて`routeBuildFromLog()`で一次パスの有効点数、再読込結果、再サンプリング点数を別エラーコードで表示する。

## 今後の予防策

- 自動解析前に、解析対象`N.csv`がSDルートにあり、ヘッダと座標行が存在することを確認する。
- `routeBuildFromLog()`の`-11`を一次パス失敗と再サンプリング失敗に分離する。
- PC側ログだけでなく、実機SD上の同一ファイルを取得して解析する。

## 関連

- 関連ノート: なし
- 参考リンク: `robotrace_v2/Core/Src/pathFollower.c`
