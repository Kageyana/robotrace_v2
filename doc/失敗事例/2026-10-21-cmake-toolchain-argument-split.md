---
status: resolved
task: SDログ保存の一時診断構成を作成
---

# PowerShellでCMakeの-D引数末尾が分割された

2026-10-21、明示workdirでcmake -S . -B build/SDWriteDiagnosis -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmakeを実行すると、引数がcmake/gcc-arm-none-eabiと.cmakeへ分割され、toolchain file not foundでconfigureが失敗した。CMakeは余分なパス.cmakeを無視したとの診断を出した。ソースの欠陥ではない。

対処: -D引数全体を単一引用符で囲み、存在確認したツールチェーンとNinjaの絶対パスを使用。configureとbuild成功を確認。

再発防止: PowerShellからCMakeへファイルパスを含む-D引数を渡す場合、'-DCMAKE_TOOLCHAIN_FILE=D:/robotrace/robotrace_v2/robotrace_v2/cmake/gcc-arm-none-eabi.cmake'のように引数全体を引用する。通常構成は既存presetを優先。追加再発防止参照へ反映。

確認: build/SDWriteDiagnosisのconfigure成功、ARM GCC 13.3.1でリンク成功。元ファームの実行状態・保存失敗原因とは別問題。
