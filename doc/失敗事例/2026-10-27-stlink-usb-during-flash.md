---
type: codex-failure
date: 2026-10-27
task: PATH_CURVE版19の書込みと静止検証
status: open
severity: medium
---

# 書込み時間切れ後にST-LinkのUSB通信エラー

Debug ELF（ロード領域248792B）を100kHz・Under Resetで書込み・照合・起動する処理が、PC側の60秒制限で終了した。子プロセスの部分出力は保存する例外処理がなく失われた。この時点では書込完了と扱っていない。再書込みせず読出しで状態確認を試みたところDEV_USB_COMM_ERR。電圧だけで接続成功とせず作業を止めた。

ユーザーがUSBを挿し直した後、Device ID0x421を再確認してロード領域の読出しを開始した。原因がUSB接続なのか他の停止要因なのかは未確定。単にPC時間制限を伸ばしたことをUSB不良の修復とみなさない。

次回は書込みCLIを非同期の進捗付きセッションで実行し、1回の待機を60秒以内に保ちながら終了を回収する。Pythonで包む場合は十分な全体制限を置き、TimeoutExpiredのstdout/stderr原バイトも保存してから異常終了する。時間切れ・USBエラー後は二重接続や盲目的な再書込みを避け、接続復旧・Flash読出し・意図したロード領域とのバイト照合で状態を確定する。起動・PWMゼロ・detachとサーバー終了まで確認する。

復旧と比較の結果はbuild/path-fusion-stlink/に保存する。接続の根本原因は未確定でopen。

## 復旧・最終検証

USB挿し直し後、初回の意図したFlash全248792 Bがバイト一致した。その後、開始時設定固定を接続した最終ELFで再書込み・照合・リセット・起動が成功。正常時のダウンロード40.209秒・照合31.200秒だけでも60秒を超えるため、当初のPC全体制限は不足していた。Pythonの制限を180秒にし、ツールの待機は短く分けて完了を回収、TimeoutExpiredの部分出力も保存するよう変更した。USBエラーの根本原因は断定せずopenを維持する。書込内容と静止復帰は検証済み。

GDBでヘッダに宣言されたinitCurrentとpathFollowerCaptureRunStartSettingsがELFに残っていると仮定してエラーになった。前者は未使用で未リンク、後者は実制御からの呼出し自体が欠落していた。後者の試験では注入後の割込みマスク1で中断したため、まずリセットなしでPWM/CCR0を確認し、元のマスク0へ戻した。CaptureはSDなしでも必要な開始時設定固定としてcontrol.cへ実接続した。製品の未使用関数をテスト都合で保持する対処ではない。最終ELFで通常UI経路生成・全経路更新・注入復元・実ADCを確認した。

接続前にnmで全呼出し関数を照合する検査を実行スクリプトへ追加した。宣言があることを実ELFの存在確認で代用しない。これらのGDBスクリプト前提と設定固定接続は修復・検証済み。

## 2026-10-29：融合版3の書込み前接続で再発

正しい旧像を再構築して停止状態を確認した後、Programmer CLIの100kHz・Under Reset接続がDEV_USB_COMM_ERRで失敗した。ダウンロード命令へは到達しておらず、新版e98397b5…はまだ書き込んでいない。接続ログ原バイトをbuild/heading-fusion-validation/connect.raw.logへ保存した。直前GDBの出力はCCR2/3=0、PRIMASK0、CFSR/HFSR0で、サーバー終了済み。

同じ接続を反復せず、ユーザーへ停止状態でST-Link USBの挿し直しと機体電源の確認を依頼した。バッテリー読出しが0/無効だった事実だけでUSB原因を断定していない。復旧後はFlash/RAMと出力0を再確認し、Device ID0x421取得後にだけ書込みへ進む。status: open（接続復旧待ち、根本原因未特定）。

## 2026-10-29の復旧確認

USB再接続後は通常attachとUnder ResetでもCore IDを取得できなかった。SWD・機体電源の再接続後に通常attachが成功し、開始なし・出力0・既存像全バイト一致を再確認した。その後、Device ID0x421を取得してe98397b5…の書込み/照合/起動と全253404B読戻し一致が成功した。最初のバッテリーAD181はユーザーのUSB表示と一致し、電池再接続後は7.703778Vへ復帰した。USB/SWD接続障害の根本原因と同一とは断定しない。

IMU/SD成功、出力0、Fault0、IPSR0、実IRQ優先度を確認しdetach/サーバー終了まで完了した。status: resolved（今回の書込み・停止起動）、open（接続障害の根本原因）。各接続/Flash/GDBの原バイトと段階別captureはbuild/heading-fusion-validationに保持した。

## 2026-10-29：FF2書込み前の接続未検出

制御版20/FF2は4構成ビルド・厳密曲率単独試験・両ログ保存/CRC・30保存走行再生・モデル比較を完了した。書込み前の既存制御版19のリセットなしRAM/Flash確認で、GDBサーバーがTarget no device found/No device found on targetで終了コード4となり、接続できなかった。所有サーバーPID30552は終了確認済み。ダウンロード命令は実行していない。

ユーザーへ停止状態で電源/SWD/USBの再接続を依頼し、同条件の接続を反復していない。13894の走行RAMは別途保存・Flash全バイト一致・SDメタデータ全項目一致が確認済みである。新しい接続と停止を確認してからFF2へ書き込む。status: open（今回の接続復旧待ち）。USB単独を原因と断定しない。

## FF2の接続復旧・書込みと電源/SD復帰

ユーザーの停止・電源/SWD/USB再接続後、リセットなしattachで制御版19の全Flash一致、開始要求0、モーター出力0、PRIMASK/Fault0を確認した。Device ID0x421/Core Cortex-M4を取得し、制御版20/FF2のダウンロード41.372秒・照合31.876秒・リセット/起動が終了コード0で完了した。新ELF 4c9a376c…の全254564B読み戻しも一致した。status: resolved（今回の接続・書込み）、open（接続障害の根本原因）。

最初の停止起動はIMU1だがSD0・バッテリーAD180/電圧無効だったため、実走確認へ進めなかった。SD設定ではなく既定値の起動で、保存済み13894の速度・制御設定と同条件とはいえなかった。原因をファーム不良やUSB障害と断定せず、ユーザーへSD挿入・バッテリー電源確認・再起動を依頼した。

SD挿入・電池表示・停止の確認後は再書込みせず、通常setupで停止して再度全Flash/RAMを回収した。IMU1/SD1、AD1944/7.620595V、開始要求/auto/log/PWM/負荷/IPSR0、CCR/PRIMASK/CFSR/HFSR0、実IRQ優先度一致を確認した。SDから読んだ22項目の速度/制御値は13894の記録設定と一致した。GDB detachと所有サーバー終了も確認済み。旧無効電源のboot-20261028T220150Zと復帰後boot-20261028T220420Zは別フォルダで保持し、前者を成功で上書きしない。status: resolved（停止起動とSD設定/電源復帰）、open（FF2の実走・IRQ時間・コースアウト防止）。

再発防止は書込み成功と走行準備完了を分け、起動後にSD/IMU・有効電圧・出力0・CPU状態・SD設定の実値を記録すること。同条件比較は前ログのメタデータと照合し、SD0の既定値起動を採用しない。停止状態の電源/SD復帰後は同一像を再書込みせず起動確認をやり直す。build/path-yaw-ff-validation/audit_hardware.pyで、対象像SHA・全Flash・RAM配置234項目・停止・22設定の照合を実行し、復帰後のsame_condition_trial_ready=trueを確認した。Skillの実行後確認にも反映する。

## 2026-10-30：入力RAM計測の保全前にMCU未検出

- 対象: 追加計測版への移行前に現在のRAMと校正値をリセットなしで保全する処理。
- 事実: 検証済み旧hardware.py capture-runのサーバーPID72172がTarget no device found/No device found on targetで終了4。所有サーバーの終了を確認し、GDBの接続/読出しは開始されず、リセット・Flash書込みも行っていない。原ログはbuild/path-unmatched-speed-validation/after-run-20261030T030341Z-server.log、解析コピーはpath_input_capture_hardware_20261030/failed_target_probe.log。
- 未確定: 機体電源・SWD・USBのどこが原因かはこの結果から特定できない。ターゲット電圧や物理姿勢も今回未取得。
- 対処: 同条件を反復せず、ユーザーへ停止状態で電源/SWD/USBの再接続を依頼。今回は水平のままを依頼し、先の右下げ測定は接続復帰後の確認へ回す。
- 再発防止: 今回の書込み前はpath_input_capture_hardware.py capture --image installedで生RAM/Flash保全と全像一致を確認する。現在像/計測像の実ELF配置265項目を分け、型付き解釈はFlash一致後に限る。flash --beforeは5分以内の実機由来の停止・IRQ整合・生RAM再復号を必須とし、合成fixtureや古い状態JSONだけでは通さない。ダウンロード開始後の失敗は未書込みと断定せずunknown_until_verifiedを保持する。
- 確認: file-only GDBで2像の265配置を取得。保存済み13924を新しい現在像配置で再復号し、emc7/search1/F,D20を確認。合成通信12条件でFlash不一致の解釈拒否、通常IRQと例外不整合、非有限値、出力/開始状態、古い/合成書込み前データを確認。実機の再接続・計測版書込み・走行は未実施。
- ステータス: open（実機再接続/新計測）。resolved（回収手順のPC検証）。

## 2026-10-31：版21復元前にST-Linkを列挙できなかった

- 対象: ユーザー指定による実験版22から実績のある版21への復元。
- 準備済み: 版21 ELF bc7a3fca…・254652Bの期待バイナリ・過去試験を照合。file-only GDBで実265項目の配置/sizeofを取得し、従来234項目とも完全一致した。実行中想定807dfea…と復元先bc7a3fca…をinstalled/diagnosticへ分け、元の証跡と通常ソースは未変更。
- 事実: 新しいhardware.py capture --image installedの所有サーバーPID71532が終了2。原ログはST-Link enumeration failed / ST-LINK DLL error。GDB接続・RAM/Flash読取り・リセット・ダウンロードはいずれも開始されていない。サーバー終了を確認し、同じ条件を反復しなかった。
- 未確定: USB未接続か通信障害かは未特定。機体電圧やCore IDも今回は取得できていない。以前の版22確認を現在像の再確認と呼ばない。
- 対処: 機体停止・電源ON・ST-Link USB/SWD接続をユーザーへ依頼した。復旧後は新しいリセットなしcaptureで停止・全Flash一致を確認し、その5分以内の実データでだけflashへ進む。版21の書込み済みとは報告しない。
- 再発防止/実行点: prepare_controller21_restore.pyで対象2像と型配置を先に確定し、controller21_restore_20261031/hardware.pyを使用する。旧Release計測器のinstalled/diagnosticの役割を新しい復元へ流用しない。列挙失敗はMCU未検出と同様に所有サーバー終了・再接続依頼とし、Under Resetを含む書込みへ飛ばない。flash-debug Skill入口へ追加した。
- 検証: analysis/controller21_restore_20261031/preparation.json、controller21-layout.raw.log、status.json、capture-installed-20261030T234211027839Z/capture-server.log。準備と非書込み/所有プロセス終了は確認済み。
- status: resolved（復元準備・接続失敗時の停止手順）。open（接続復旧・実機への版21復元）。
