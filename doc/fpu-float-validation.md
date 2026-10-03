# FPU単精度化の実機比較

検証ブランチ: `codex/fpu-float-validation`。計測付き変更前とfloat化版を別コミットで管理する。PID・速度・加速度・安全条件・ログ形式は変更しない。

## 比較用成果物

リポジトリ直下で以下を各コミットから実行する。`--variant`は変更前で`baseline`、変更後で`float`を指定する。

```powershell
python analysis/script/fpu_float_validation.py --variant baseline --primary F:/Dropbox/Document/robotrace/Log/v2/12905.csv --primary F:/Dropbox/Document/robotrace/Log/v2/12879.csv
```

`analysis/fpu-float-audit/<variant>/`へDebug/Release ELF・SHA256・Gitハッシュ・設定・通常warning・追加診断・逆アセンブル・PCテスト結果を保存する。生成物はコミットしない。12905は正常な一次ログ、12879は閉路不成立の拒否確認用であり、比較用にログを書き換えない。経路再現には既存PCテストの固定速度設定とFatFsスタブを使用するため、実機速度計画の完全再現とは区別する。

比較はDebug同士で行う。通常ビルドの`ROBOTRACE_ISR_TIMING`はOFF、比較スクリプトは専用ビルドディレクトリでONを指定する。`main`の通常ビルド成果物を比較版と取り違えない。

**通常の`build/Debug`を指すFlashタスクでは計測OFF版が書き込まれることがある。** 書き込み前に選択したELFに`isrTimingStats`が存在することを確認し、比較用ELFをパスで直接指定する。リポジトリ直下からfloat化版を書き込む例:

```powershell
arm-none-eabi-nm.exe analysis/fpu-float-audit/float/Debug/robotrace_v2.elf | Select-String 'isrTimingStats'
STM32_Programmer_CLI.exe -c port=SWD mode=UR reset=HWrst freq=100 -d analysis/fpu-float-audit/float/Debug/robotrace_v2.elf -v -rst -s
```

変更前は上記パスの`float`を`baseline`へ替える。書き込みは実機接続時だけ実施する。走行後の回収ではこのリセット付きコマンドを実行しない。

実機とのELF照合でGDBの`compare-sections`だけに依存しない。このST-Link環境では、実FlashとELFのバイナリが一致するのに同コマンドが不一致を報告した。必要な場合はリセットなしでFlashを読み出し、`objcopy -O binary`で生成した比較対象とバイト比較する。変数は一致が確認できたELFで解釈する。GDB内PythonはこのCubeCLT版で使用できないため、コマンド列の生成・集計は外部Python/PowerShellで行う。

## DWT計測の読み出し

- 対象は`Interrupt1ms()`の既存DWTリセット直後から集計直前まで。入口前の処理、集計自体、呼出し元の処理、割り込みの入退場コストは含まない。計測区間に割り込む他の割り込みの時間は含む。厳密なTIM6全体の最悪実行時間を保証する値ではない。
- `isrTimingStats[0..4]`は順にNONE、MARKER、DISTANCE、SHORTCUT、PATH_REPLAY。走行状態`12 <= patternTrace < 100`を入口で満たした周期を集計する。
- `samples`は周期数、`minCycles`・`maxCycles`は最小・最大、`totalCycles`は64bit合計、`over1ms`は計測区間が1ms以上の周期数。サンプル数が0のモードは未計測。`samples`上限に達した場合は追加集計しない。
- 値は起動から累積する。**1比較セットにつき再起動してからオートスタートする。セット終了後の読み出し前にリセット・電源断・再書き込みしない。**
- 走行中はST-Linkを外す。停止・ログ保存完了後、車輪が静止し出力がゼロであることを確認し、同じDebug ELFでリセットなしのAttachを行う。初期化やFatFs関数をGDBから呼ばない。

GDBで保存する項目:

```text
p SystemCoreClock
p isrTimingStats
```

平均[us] = `totalCycles / samples * 1000000 / SystemCoreClock`、最大[us] = `maxCycles * 1000000 / SystemCoreClock`。演算はPC側で行う。読み出し後は接続を解除し、次セット開始前に再起動する。

## 走行手順と記録

1. 同じ機体・室内コース・SD設定を使用する。SD設定は比較開始前にコピーしSHA256を保存する。ログヘッダだけで設定一致を判断しない。
2. 実機とST-Link接続時のみ対象Debug ELFを書き込み、Device ID・照合・起動・センサー初期化を確認する。高速走行前の電圧は7.5V以上とする。緊急停止は無効化しない。
3. 変更前とfloat化版を交互に、各版10セット以上実施する。失敗も試行数に含める。各版で5走正常完了セットが10未満なら追加し、各モードで10本以上の正常ログを確保する。
4. セットごとに集計を読み出す。失敗した場合も保存済みログと停止要因を回収する。失敗版の設定を変更して比較を続けない。
5. ログは同じモード・条件で比較する。オートスタートのタイム比較は5走完了セットに限定する。cntlog折り返しは解析側で補正し、距離基準間隔で欠落を判断する。

`analysis/fpu-float-audit/run-register.csv`には以下を記録する:

```csv
set_id,variant,firmware_commit,elf_sha256,config,settings_sha256,battery_start_V,primary_log,run2_log,run3_log,run4_log,run5_log,completed_5_runs,stop_reason,timing_file,notes
```

処理時間のCSVは以下の形で保存する（modeはBOOSTの整数値）:

```csv
set_id,variant,mode,clock_hz,samples,min_cycles,max_cycles,total_cycles,over_1ms
```

## 判定

- ローカル: 両版のDebug/Releaseビルド・PC経路/XY/ログスキーマテストが成功し、新規の通常warningがない。制御演算の不要なdouble昇格と逆アセンブル上の倍精度呼出しが減っている。
- 同じ一次ログに対し、経路生成成功/拒否、短縮可否、整数座標・方位・速度、閉路X判定を比較する。判定差の原因を説明できない場合は実機比較へ進まない。
- 実機: モード別の平均/最大処理時間と1ms以上件数、完走率、タイム中央値・ばらつき、速度追従、スリップ、XY、停止位置、emcStop、cntlog欠落を比較する。現在の軽量ログに存在しないPATH専用列を比較の必須項目にしない。
- 単精度化は積分の丸め順序を変える。処理時間の改善と完走率・再現性の維持を確認する。採用はゴールタイムも安定して改善した場合のみとし、未確認・改善なしは検証ブランチに留める。
- 問題が出たら保存したbaseline ELFへ戻す。再利用可能な失敗は`doc/失敗事例/`へ原因・対処・具体的な再発防止策と検証状態を記録する。
