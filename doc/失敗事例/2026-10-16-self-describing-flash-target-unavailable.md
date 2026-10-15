---
status: open
task: 自己記述RLOG v2の実機書込み
---

# ST-Linkは列挙できるがSTM32のCore IDを取得できない

確認結果: 既知の接続条件`port=SWD mode=UR reset=HWrst freq=100`で初回接続した。ST-Link SN=066EFF574881774867025549、FW=V2J48M35、電圧3.24 Vは取得できたが、`Unable to get core ID`／`No STM32 target found`、終了コード1だった。書込み・Flash読出し・GDB接続は実行していない。

原因: STM32ターゲットの認識失敗は確認済み。機体電源・SWD接続・リセット信号等のどれが原因かは未確定。プローブ電圧の取得だけでは機体起動やSWD通信を保証しない。

対処または次の調査: ユーザー側で機体電源とST-Linkの接続状態を確認後、同じ既知条件でDevice ID 0x421とCore IDを確認する。成功後に現在のFlashを保全し、ビルド済みv2 Debug ELFを書込み・全照合・起動確認する。PCはv2専用へ更新済みなので、書込み完了前の旧v1ログは変換せず保持される。

再発防止: 既存のflash-debug Skillの「プローブ列挙・電圧だけで成功としない。Device ID 0x421/Core IDが取れなければ書込みへ進まない」を適用した。同じ状態で接続方法を変えた連続再試行をしない。書込み済みとPC更新済みを分けて報告し、異なる形式のまま実走行試験へ進めない。

確認方法: 接続確認exit_code=0とDevice ID/Core ID、Download verified successfully、Start operation achieved successfullyを取得し、新形式初回ログのヘッダ・列定義・CRC・PC取込を検証する。それまではopenとする。

2026-10-16追記: ユーザーが書込み・実走行を行い、13288～13297の10本がRLOG v2として保存・自動取込された。全CRC・列定義・期待行数・正式CSV一致を確認し、新形式の利用は可能になった。CodexによるSWD再接続やFlash照合は今回実施しておらず、初回接続失敗の物理原因は未確定のため、この接続事例はopenを維持する。機能確認結果は`doc/self-describing-log.md`へ記録した。
