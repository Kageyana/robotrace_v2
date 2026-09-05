---
type: codex-failure
date: 2026-08-23
task: PATH REPLAYのkHeading調整
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
  - tuning
---

# PATH REPLAYでkHeadingを400へ下げるとヘアピン追従が悪化

## 要約

`shortcut.txt`のkHeadingを600から400へ下げたところ、前回のkHeading=600より横誤差・方位誤差が増え、同じヘアピン区間でLOCALIZATION_LOSTが再発した。

## 発生した状況

- 一次走行: 12174、12177
- PATH REPLAY: 12175、12176、12178
- 設定: `1,080,040,1500,0400,000`
- バッテリー: 7.76〜8.08 V

## 結果・エラー

```
12175: emcStop=0, lat_p95=65.0 mm, heading_p95=91.5 deg, max lat=85.7 mm
12176: emcStop=7, lat_p95=60.2 mm, heading_p95=87.5 deg
12178: emcStop=7, lat_p95=60.7 mm, heading_p95=85.7 deg
```

kHeading=600時の12170〜12172はlat_p95=34.6〜37.2 mm、heading_p95=53.5〜63.0 degだった。動画ではヘアピン進入後に旋回が遅れ、内側へ外れて停止した。

## 原因

kHeadingを下げたことで方位誤差への復元トルクが不足し、kLateral=1500だけではヘアピンの姿勢誤差を十分に抑えられなかった可能性が高い。7.76 Vの走行もあるが、8.07〜8.08 VでもemcStop=7が発生しており、電圧だけでは説明できない。

## 解決・回避策

未解決。kHeading=600へ戻し、次はlineAlphaまたはyawRate PIDを1項目だけ評価する。速度とkLateralは固定する。

## 今後の予防策

- ゲイン変更は同一電圧条件で比較する。
- p95横誤差・p95方位誤差・emcStopを変更前後で比較する。
- 1回の試行で複数のゲインを同時変更しない。

## 関連

- 関連ノート: 2026-08-23-path-replay-yaw-oscillation-hairpin.md
- 参考リンク:
