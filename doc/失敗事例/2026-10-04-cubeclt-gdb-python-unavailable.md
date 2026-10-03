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
