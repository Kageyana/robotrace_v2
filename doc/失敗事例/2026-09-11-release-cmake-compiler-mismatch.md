# Release CMakeのARMコンパイラ切り替え失敗

## 対象タスク

ログスキーマ変更後のSTM32 Debug/Releaseビルド確認。

## 事実として確認できた結果

- `cmake --build --preset Debug` は成功した。
- `cmake --preset Release; cmake --build --preset Release` は構成段階で失敗した。
- Release構成では、`C:/Users/Natsuki/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin/arm-none-eabi-gcc.exe` が選択された。
- CMakeのコンパイラ検査でARMリンカーへ `--major-image-version` などのWindowsホスト用オプションが渡され、`unrecognized option '--major-image-version'` となった。
- ソースのコンパイルエラーではない。Releaseのビルド成果物はこの試行では生成完了していない。

## 原因または未確定事項

Releaseの既存キャッシュまたはプリセットのツールチェーン選択が、Debugで使用できたSTM32CubeCLT側のARM GCCと異なる。CMakeキャッシュ、プリセットの環境変数、Ninjaのリンク設定のどれが切り替えを起こしたかは未確定。

## 実施した対処または次の調査

- Debugビルドでソース変更のコンパイルとリンクを確認した。
- Releaseは、既存のビルドディレクトリを壊さない範囲で、`CMakePresets.json`、ツールチェーンファイル、コンパイラ環境を比較して再構成する。

## 再発防止策

- Releaseを実行する前に、CMakeが選択したC/C++/ASMコンパイラの絶対パスを確認する。
- DebugとReleaseでARM GCCの提供元が一致していることを確認してから構成する。
- 構成が失敗した場合は、ソースのビルド失敗と報告せず、コンパイラ検査の失敗として記録する。

## 確認方法

Releaseの構成ログでコンパイラパスを確認し、ARMリンカーがホスト用オプションなしでコンパイラ検査を通過した後に `cmake --build --preset Release` を再実行する。

## 追加確認

STM32CubeCLT 1.19.0のGNU toolsとNinjaをPATHの先頭へ置き、CMake 3.28の`--fresh`でReleaseを再構成した。同じ環境で`cmake --build --preset Release`を再実行し、リンクまで成功した。構成ログで`C:/ST/STM32CubeCLT_1.19.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe`が選ばれたことも確認した。

## ステータス

resolved
