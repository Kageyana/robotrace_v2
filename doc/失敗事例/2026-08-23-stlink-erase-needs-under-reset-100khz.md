---
type: codex-failure
date: 2026-08-23
task: "robotrace_v2 Level 1ショートカットファームウェア書き込み"
status: resolved
severity: medium
tags:
  - codex/failure
  - stlink
  - stm32
---

# ST-Link書き込みは100 kHz Under Resetで成功した

## 要約

ST-Linkとターゲット電圧は認識したが、通常4 MHz接続ではCore IDを取得できず、100 kHz Hot Plugでは接続後のFlash消去に失敗した。100 kHz・Hardware Reset・Under Resetへ切り替えると、消去、書き込み、リセット、起動まで成功した。

## 発生した状況

- タスク: STM32F446へRelease ELFを書き込む
- 実行環境: STM32CubeProgrammer 2.20.0 / ST-Link V2 / SWD
- 前提条件: ターゲット電圧3.24 V、ST-Linkプローブと仮想COMは認識済み

## 何を試したか

1. 通常4 MHz SWD接続を試した。
2. 1 MHz Under Reset接続を試した。
3. 100 kHz Hot Plugで接続し、書き込みを試した。
4. 100 kHz、Hardware Reset、Under Resetで書き込んだ。

## 結果・エラー

```
Unable to get core ID
failed to erase memory
```

最終手順ではFile download completeとStart operation achieved successfullyを確認した。

## 原因

通常速度ではSWD通信が安定せず、Hot Plugでは実行中MCUのFlash消去が通らなかった可能性がある。配線品質やリセット状態のどちらが主因かは未確定。

## 解決・回避策

通常接続が失敗する場合は、次の接続条件を使用する。

```
--connect port=swd mode=UR reset=HWrst freq=100
```

## 今後の予防策

- 書き込み前にターゲット電圧とDevice IDを確認する。
- 通常接続でCore IDを取れない場合は、Hot Plugで書き込まず100 kHz Under Resetを優先する。
- 再発時はSWDIO、SWCLK、NRST、GNDの接触とケーブル長を確認する。

## 関連

- 関連ノート: [[2026-08-23-shortcut-gate-requires-explicit-approval]]
- 参考リンク: