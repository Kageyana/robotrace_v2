---
type: codex-failure
date: 2026-09-03
task: "マーカー誤検出調査用ファームウェア書き込み"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace
  - st-link
  - flashing
---

# ST-Linkは検出されるがSTM32ターゲットを取得できない

## 要約

ST-Linkプローブは列挙されたが、STM32のコアIDを取得できず、デバッグログ追加ファームウェアを書き込めなかった。

## 発生した状況

- タスク: `LOG_SCHEMA_PROFILE_LIGHT=0` のDebug ELFを書き込み
- 実行環境: STM32CubeProgrammer v2.20.0、ST-LINK SN 066EFF574881774867025549
- 前提条件: ST-Linkプローブは検出、ターゲット電圧表示は3.24 V

## 何を試したか

1. `port=SWD mode=UR` で書き込みを試した。
2. `port=SWD mode=HOTPLUG` で再試行した。

## 結果・エラー

```text
Error: Unable to get core ID
Error: No STM32 target found!
```

## 原因

ST-Link本体は認識されたが、STM32ターゲットとのSWD通信が成立していない。電源、GND、SWDIO、SWCLK、NRST、またはターゲット側リセット状態の確認が必要。

## 解決・回避策

ターゲット基板の接続確認後、`port=SWD mode=UR` でSTM32F446のコアID取得とDebugMarker ELFの書き込み・ベリファイに成功した。

## 今後の予防策

書き込み前に `STM32_Programmer_CLI -l` でプローブ列挙を確認し、続けて `-c port=SWD mode=UR -l` 相当のターゲット接続確認を行う。コアIDが取得できない場合は書き込みを開始しない。

## 関連

- 関連ノート:
- 参考リンク:
