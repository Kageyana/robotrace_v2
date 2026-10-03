---
status: open
---

# FPU比較スクリプトの保存ログ読取文字コード

対象: 計測付き変更前/float化版のビルドと成果物保存。

確認結果: 変更前Debugビルドは成功したが、UTF-8で保存したbuild.txtをPath.read_text()で読み直す際にWindows既定cp932が選ばれ、UnicodeDecodeErrorで検証が停止した。

原因: 出力の文字コードをUTF-8に指定し、読み戻し側の指定を省略していた。

対処: 保存ログの読取にもencoding='utf-8'を指定する。比較時の通常warning欠落を防ぐため、専用ビルドでは--clean-firstを使う。

再発防止: 検証スクリプトでテキストの読み書きに文字コードを明示する。変更前/変更後とも最後まで再実行し、manifest.json生成、ELFハッシュ、通常warning一覧、全PC検証の成功を確認する。

確認方法: analysis/script/fpu_float_validation.pyを両版で実行する。対処後の全工程確認待ち。
