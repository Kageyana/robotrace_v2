---
type: codex-failure
date: 2026-09-28
task: "ST-Linkと機体の接続状態確認"
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - st-link
  - flashing
---

# ST-Linkは検出されたがSTM32ターゲットへ接続できない

## 要約

STM32CubeProgrammerはST-Linkを列挙したが、SWD接続でCore IDを取得できなかった。プローブのPC接続は確認できた一方、機体MCUとの通信は確認できていない。

## 発生した状況

- タスク: ST-Linkと機体の接続状態確認
- 実行環境: STM32CubeProgrammer v2.20.0
- 前提条件: `-l` でST-Linkプローブを検出し、接続確認ではターゲット電圧3.24 Vを表示

## 何を試したか

1. `STM32_Programmer_CLI -l` でプローブを列挙した。
2. `STM32_Programmer_CLI -c port=SWD` でSWD接続を試した。
3. 100 kHz、Hardware Reset、Under Resetへ下げて接続確認した。

## 結果・エラー

```
Error: Unable to get core ID
Error: No STM32 target found!
```

## 原因

- 確認できた事実: ST-Link本体はPCに検出され、ターゲット電圧は3.24 Vと表示されたが、SWD経由でSTM32のCore IDを取得できなかった。
- 未確定事項: SWDIO、SWCLK、GND、NRSTの接触、ターゲット側リセット状態、接続速度のどれが原因かは未確認。
- 2026-09-03にも同種のCore ID取得失敗があり、基板接続後に復旧した記録がある。今回再発したため、以前の解決確認だけでは接続状態を保証できない。

## 解決・回避策

初回発生時は100 kHz、Hardware Reset、Under ResetでSTM32F446のDevice ID `0x421`を取得できた。2026-09-29の再発では同じ手順を含む復旧を試したが成功していない。

### 2026-09-29 再発

- モーター電源接続後、`-l st-link`ではプローブを列挙し、ターゲット電圧3.24 Vを表示した。
- `mode=HOTPLUG freq=1000`、`mode=UR reset=HWrst freq=100`、`mode=HWRSTPULSE freq=100`はいずれもCore ID取得に失敗した。
- この再発時点では、GDBとGDBサーバーは終了済み。Flash書き込みや測定開始は行っていない。
- 原因は未確定。SWDIO、SWCLK、GND、NRST、電源接続状態を再確認してから再試行する。

### 2026-09-29 再発後の復旧と再々発

- 後続の接続では、100 kHz・Under ResetでSTM32F446のDevice ID `0x421`を取得し、Release詳細ログ版の書込みとベリファイに成功した。
- Release軽量版へ切り替える今回の再接続では、同じ条件の `-c port=SWD mode=UR reset=HWrst freq=100 -l` でST-Link SN `066EFF574881774867025549` とターゲット電圧3.24 Vは認識されたが、Core ID取得に失敗した。
- 今回は書込み・測定を行っていない。接続状態が再び変化した原因は未確定。
- 次回はSWDIO、SWCLK、GND、NRST、ターゲット電源の接続を確認し、Core ID取得が成功するまで書き込みを開始しない。

## 今後の予防策

- 「ST-Linkが検出されたこと」と「STM32ターゲットに接続できたこと」を別々に確認して報告する。
- 書き込み前は `-l` に加え、ターゲットのCore ID取得も成功するまで書き込みを開始しない。
- 通常速度でCore ID取得に失敗した場合は、書き込み前に100 kHz・Hardware Reset・Under ResetでDevice ID取得を確認する。
- 上記Under Reset接続も失敗した場合は、書き込みや測定へ進まず、SWD配線とターゲット電源を確認してから再試行する。

## 関連

- 関連ノート: [[2026-09-03-stlink-target-not-found-marker-debug]], [[2026-08-23-stlink-erase-needs-under-reset-100khz]]
- 参考リンク:
