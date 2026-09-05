---
type: codex-failure
date: 2026-09-03
task: "ロボトレース PATH REPLAY 走行ログ確認"
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
  - marker
---

# PATH REPLAYの早期ゴール判定と自己位置喪失

## 要約

詳細CSV修正版でPATH REPLAY解析は成功したが、複数系列で3～5走目が約4.1 mで正常終了扱いとなり、別系列では`emcStop=7`で停止した。

## 発生した状況

- 対象ログ: 12238～12256。
- 一次走行（`optimalTrace=0`）は約9.36～9.39 s、約9.65 mで完走。
- PATH REPLAYは`shortcutLevel=0`で検証。

## 何を確認したか

1. CSV列数・改行・`cntlog`連続性を確認した。
2. `sgMarkerAtLogEnd`、`encRightMarkerAtLogEnd_p`、`markerSensor`、`sgMarkerCount`を確認した。
3. `emcStop`と`pathState`、経路誤差を同一系列で比較した。

## 結果・エラー

```
12240/12241/12242, 12247/12248/12249, 12254/12255/12256:
  emcStop=0, sgMarkerAtLogEnd=3, 約4.07～4.58 sで終了
12244:
  emcStop=7, sgMarkerAtLogEnd=2, 約8.91 sで停止
12251:
  emcStop=7, sgMarkerAtLogEnd=2, 約5.21 sで停止
```

## 原因または有力な仮説

早期終了系列では、約1.6 m地点の`RIGHTMARKER`が`sgMarkerCount=2`となり、その後約4.0 m地点の右マーカーで3へ到達している。一次走行では同地点の右マーカーが記録されていないため、PATH REPLAY時の右マーカー誤検出または通常マーカーをゴール計数している可能性が高い。

12251では`pathState`が約4.82 mでラインフォールバックへ遷移し、終了時の`pathErrorY`は約43 mm、方位誤差は約-21 deg。`shortcut.txt`の`lineAlpha_x1000=000`のため、ラインセンサー有効時も自己位置補正は行われず、自己位置喪失に至った可能性が高い。12244は最終ログ後に同じ状態へ遷移した可能性がある。

## 次の検証

- ゴール計数を一次走行のマーカー位置または経路終端と照合し、PATH REPLAY中の通常右マーカーをゴール扱いしない。
- `lineAlpha_x1000`を小さな値から有効化して、自己位置ドリフトを抑制する比較走行を行う。
- 変更は1回の走行で最大2パラメータに限定し、10本以上のログで再現性を確認する。

## 今後の予防策

PATH REPLAY解析後は、`sgMarkerAtLogEnd`、`routeSourceLog`、最終距離、`emcStop`を自動表示し、4.1 m付近で`sgMarkerAtLogEnd=3`になったログを早期ゴールとして警告する。

## 関連

- 関連ノート: 2026-09-03-path-csv-line-truncation-error-7.md
- 参考リンク: `robotrace_v2/Core/Src/markerSensor.c`, `robotrace_v2/Core/Src/control.c`, `robotrace_v2/Core/Src/pathFollower.c`
