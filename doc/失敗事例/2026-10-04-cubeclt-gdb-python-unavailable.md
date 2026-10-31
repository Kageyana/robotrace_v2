---
status: resolved
---

# CubeCLTのARM GDBにPythonスクリプト機能がない

対象: FPU検証走行後の候補ELF照合。

確認結果: GDBコマンドファイルのpythonブロックが「Scripting in the Python language is not supported in this copy of GDB」で停止した。使用したのはSTM32CubeCLT_1.19.0のarm-none-eabi-gdb.exe。

原因: 当該GDBビルドにはPythonスクリプト機能が含まれない。

対処: 外部PowerShellでfile/compare-sections/detachのコマンド列を生成し、GDB標準コマンドだけで実行した。接続はリセットせず維持・解除した。

再発防止: GDB内Pythonを使う前にshow configurationで対応を確認する。この環境の照合・集計は外部Python/PowerShellで行い、GDBは標準コマンドを使用する。doc/fpu-float-validation.mdに制約を追記した。

検証: identify-candidates.gdbの標準コマンド列は最後のdetachまで成功した。

## 2026-10-28：停止後RAM採取での再発

対象: PATH_CURVE処理時間計測版を書き込む前のFlash/RAM保全。

確認結果: CubeCLT 1.19.0の既知の制約がSkillに記載されていたが、生成したbefore.gdbにPythonブロックを含めたため、attach直後に同じエラーで中断した。ブロック内のPWM確認・RAM採取・finallyのdetachは実行されなかった。書込み・リセットはまだ行っていなかった。

原因: 既知の非対応機能を生成段階で排除していなかった。GDB内finallyを接続解除の保証として扱ったが、その言語自体が非対応なら保証にならない。

対処: 所有サーバー終了とポート解放を確認し、標準GDBだけでリセットなし再接続した。左右CCR=0、PRIMASK=0、CFSR/HFSR=0を確認し、RAM 131072 BとFlash 249304 Bを保存した。外部Pythonで保存済みELF由来のbinaryと全バイト一致を確認し、一致ELFで状態を読み出した。開始要求0、左右PWM0を確認してdetach・所有サーバー終了まで完了した。

再発防止: 実機接続前に生成GDBの各コマンド先頭を検査し、python/pi/python-interactiveを拒否する。照合・集計・例外処理は外部Python、GDBは標準コマンドだけとする。Flash保全→外部照合→型付き読出しの順序を守る。Skillの操作入口にもこの検査を追加した。

検証: build/path-timing-validation/hardware.pyのattach生成ゲート、recovered.log、before_capture.json、prior-state.logで確認。初回中断を成功証跡に含めず、復旧時のdetachと所有サーバー終了を確認した。

ステータス: resolved（今回の採取手順の復旧と非対応コマンド拒否）。元GDBのPython非対応は環境制約として維持する。
