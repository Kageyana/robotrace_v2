---
type: codex-failure
date: 2026-09-08
task: "Version 12引き継ぎ後のST-Link接続確認"
status: resolved
severity: medium
tags:
  - codex/failure
  - stlink
  - flash
---

# ST-Link V2は認識するがMCUのcore IDを取得できない

## 確認できた事実

- STM32CubeProgrammer v2.20.0でST-Link V2（FW V2J46M33）を認識した。
- ターゲット電圧表示は3.24 V。バッテリー電圧ではない。
- `--connect port=SWD`と`--connect port=SWD freq=100 mode=UR reset=HWrst`の双方で`Unable to get core ID`、`No STM32 target found`となった。
- 書き込みは実施していない。Debug・Release ELFのビルドは成功済みで、埋め込み識別はcodex/path-association-guard / cf86014。
- サンドボックス外でも100 kHz Under Reset接続は同じエラーになった。既存の2026-08-23の回避手順では今回復旧しなかった。
- ユーザーが機体電源とSWDIO・SWCLK・GND・NRSTを確認し再接続した後も、サンドボックス外の100 kHz Under Resetと100 kHz Hot Plugは同じcore ID取得エラーとなった。再接続だけでは復旧しなかったため、openを維持する。

## 原因または未確定事項

MCU電源、SWDIO/SWCLK/GND接続、NRST接続、実機状態のいずれが原因かは未確定。ST-Link認識と電圧表示だけではMCU接続成功と判断できない。

## 次の調査と再発防止

- 実機の電源とSWD配線の接触を確認し、再接続する。
- リセット接続を使う場合はNRSTの接続を確認する。
- 再接続後も失敗する場合は、実機の起動表示の有無を確認して、ファームウェアが起動しているかを切り分ける。ST-Linkの3.24 V表示だけでMCU電源・起動正常と判断しない。
- 低速SWDでMCU名・device IDを取得できることを確認してからELFを書き込み、verify結果を確認する。
- センサー初期化と実機のファームウェア表示を確認する。ST-Link接続中は実走行しない。

## 確認方法・ステータス

ユーザーから実機の起動表示（IMU success、ブランチcodex/path-association-guard、コミット5838f7d）の報告後、100 kHz Under Resetで再接続するとSTM32F446xx（Device ID 0x421）を取得できた。復旧の直接原因は未確定。

同じ接続条件でcf86014のRelease ELFを`--download ... --verify --start`により書き込み、`Download verified successfully`、`Start operation achieved successfully`、終了コード0を確認した。接続・書き込み障害はresolvedとする。書き込み後の実機起動表示、SD初期化、実走行検証は別途確認が必要で、経路制御の採用判断を意味しない。
