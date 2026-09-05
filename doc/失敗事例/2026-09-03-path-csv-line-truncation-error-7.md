---
type: codex-failure
date: 2026-09-03
task: "ロボトレース PATH REPLAY の一次走行ログ解析"
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
---

# 詳細CSVの行切り捨てで経路解析が-7になる

## 要約

詳細ログプロファイル（`LOG_SCHEMA_PROFILE_LIGHT=0`）で走行した一次ログの解析が`Error -7`で失敗した。

## 発生した状況

- タスク: 一次走行ログからPATH REPLAY/ショートカット経路を生成する。
- 実行環境: STM32F446、FatFs、DebugMarkerビルド。
- 対象ログ: 12235～12237。
- 前提条件: 詳細デバッグ列を追加したCSVスキーマ。

## 何を試したか

1. 新規ログのヘッダ、列数、行長、x/y軌跡を確認した。
2. ファームウェアの`routeBuildFromLog()`で`-7`を返す条件を確認した。
3. CSV出力バッファを拡張してDebugMarkerをビルドし、ST-Linkで書き込んだ。

## 結果・エラー

```
Error -7
12235.csv: 35行として認識、最大行長108630文字
12236.csv: 31行として認識、最大行長112709文字
12237.csv: 41行として認識、最大行長80579文字
```

## 原因

`SDcard.c`の`endLog()`が`logStr[256]`へ詳細列を含む1レコードを`snprintf()`していた。1行が256バイトを超えると末尾の改行が切り捨てられ、複数レコードが連結した。`pathFollower.c`は連結行を2048バイト単位で分割して誤ったx/y点列として読み、`PATH_ROUTE_MAX_POINTS`に達して`-7`（`routeOverflow`）を返した。

## 解決・回避策

`SDcard.c`のCSV行バッファを1024バイトへ拡張し、`snprintf()`の戻り値で切り捨てを検出するよう修正した。`formatLog`も512バイトへ拡張した。DebugMarkerビルドと書き込み検証は成功済み。12235～12237は破損済みのため再利用しない。

## 今後の予防策

詳細ログプロファイルを使う場合は、走行後にCSVの行数・列数・最大行長を自動検査する。最大行長が1023バイト以上、またはヘッダ列数とデータ列数が一致しないログはPATH解析へ渡さない。

## 関連

- 関連ノート: なし
- 参考リンク: `robotrace_v2/Core/Src/SDcard.c`, `robotrace_v2/Core/Src/pathFollower.c`
