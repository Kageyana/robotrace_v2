---
type: codex-failure
date: 2026-10-13
task: "ラインセンサADCのST-Link静止検証"
status: open
severity: low
tags: [codex/failure, stlink, adc, debug]
---

# ST-Link照合時の接続失敗と停止中タイマーによる観測影響

## 確認した事実

- Under Reset/100kHzでDevice ID 0x421を取得し、Debug ELFの書込みを開始した。最初の長いコマンドは進捗までの出力しか保持しておらず、最終成功/照合/起動メッセージを回収できていなかった。
- 後続の単独`-v ELF -rst -s`は`ST-LINK error (DEV_CONNECT_ERR)`、終了コード1で失敗。他のGDB/Programmer/サーバープロセスはなく、ST-LinkのUSB列挙ではSN/FWが正常だった。
- 同じ100kHzでリセットしないGDB attachは成功。実Flash269568 Bを読み出し、Debug ELFのbinaryと全バイト一致を確認した。再消去・再書込み・USB再接続は行っていない。
- 通常GDB停止中にTIM3が進み、取得スキップと最大更新間隔が増加した。最初の観測区間のスキップ増分5/最大3msを、通常動作の実測値としては扱えなかった。

## 原因・未確定事項

CLI単独verifyのDEV_CONNECT_ERRの根本原因は未確定。最初の書込み完了メッセージがないことを、書込み不良と推測しない。今回の読出し比較ではFlashの一致を確認できている。

一方、デバッグ停止中もTIM3が進む設定は確認済み。CPU停止によってソフト位相更新・CC1要求処理が遅れるため、意図どおり窓失効を数える。この件数を通常動作時の競合として評価すると誤る。

## 対処と検証

CLIの失敗後は書込みを繰り返さず、USB列挙・保持プロセスを確認し、GDBのリセットなしattachへ切替。Flashの直接バイト比較とPWMゼロを確認した。CLIの根本原因は残るため本記録はopenとする。

DBGMCU_APB1_FZの元値0を保存したうえで、停止中だけTIM3/TIM6/TIM7を凍結する0x32を設定。競合注入の停止間隔を含めないよう最大間隔を区間開始時にリセットし、GDBを切り離して連続観測した。51465制御msで51465更新、スキップ増分0、最大1ms、停止検出増分0を確認。終了時に凍結設定を0へ復元、センサーOFF・PWMゼロ・ロック解除・Faultなしを確認してデタッチした。観測影響の回避と復元は検証済み。

## 再発防止

1. 長い`exec_command`は戻り値全体を保持する。`session_id`が返った場合は`write_stdin`で終了コードと最終出力を必ず回収し、`.output`だけを取り出してセッション情報を失わない。
2. verify/接続失敗をFlash不一致と同一視せず、保持プロセス・USB列挙・Device ID/接続方式を分けて確認する。実Flashのバイト比較でELFを確定する。
3. 取得窓・更新停止を計測する前にDBGMCUの停止中タイマー設定を確認し、GDB停止を含む区間と連続実行区間を分離する。元設定を保存・復元し、故障注入の累積件数も通常計測から分離する。
4. CLI再発時は実コマンド・版・接続状態・最終出力を保存し、GDB接続とは別に原因を調べる。今回成功したGDB attachだけでCLI問題が解決したとはしない。

確認資料: `analysis/line-sensor-adc-stlink/`、`doc/line-sensor-adc-stlink-validation.md`。

## 周期的欠測修正版の書込み時の再発

書込みセッション58065を`write_stdin`で待ったが、戻り値はまだ実行中（VerifyのRead progress）だった。その結果を確認せず次のGDBサーバー起動へ進めたため、Programmerと競合しDevice connect errorでサーバーが終了した。これは今回の実行順序の不備として確認できる。Programmerのexit_code=0、Download verified successfully、Start operation achieved successfullyを回収した後に再起動するとGDBサーバーは待受できた。

再発防止の追加: 待機ツールの呼出しを実行しただけで終了したと扱わない。`session_id`が残っている間は全てのST-Link接続・サーバー起動を禁止し、`exit_code`を確認してから次段へ進む。関数オーケストレーションでは依存操作を同一セルで無条件に続けず、戻り値を判定する。非昇格シェルのGet-Processだけでは昇格したProgrammerの生存を否定しない。初回単独verify失敗の原因は未確定だが、同じプローブを実行中プロセスが保持していた可能性を再調査対象とする。
