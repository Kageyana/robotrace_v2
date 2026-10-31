---
status: resolved
---

2026-10-04 再発（Coreコメント調査）: ファイル読取にはUTF-8を指定したが、Python標準出力が既定CP932のままで、BOM文字の表示時にUnicodeEncodeErrorが発生し、一覧出力が途中で停止した。標準出力の文字コードも別途指定する必要がある。PowerShellで `$env:PYTHONIOENCODING='utf-8'` を設定して再実行し、Core以下67ファイルのUTF-8読取と調査完了を確認した。今後の日本語ソース調査では、読取のencoding指定に加え、Python起動前にPYTHONIOENCODINGを設定し、終了コード0と対象ファイル総数を確認する。ソース変更はなし。resolved。

# FPU比較スクリプトの保存ログ読取文字コード

対象: 計測付き変更前/float化版のビルドと成果物保存。

確認結果: 変更前Debugビルドは成功したが、UTF-8で保存したbuild.txtをPath.read_text()で読み直す際にWindows既定cp932が選ばれ、UnicodeDecodeErrorで検証が停止した。

原因: 出力の文字コードをUTF-8に指定し、読み戻し側の指定を省略していた。

対処: 保存ログの読取にもencoding='utf-8'を指定する。比較時の通常warning欠落を防ぐため、専用ビルドでは--clean-firstを使う。

再発防止: 検証スクリプトでテキストの読み書きに文字コードを明示する。変更前/変更後とも最後まで再実行し、manifest.json生成、ELFハッシュ、通常warning一覧、全PC検証の成功を確認する。

確認方法: 対処後、変更前版のDebug/Release、追加診断、PC経路/XY/スキーマテスト、計測集計テスト、一次ログ2本のリプレイが成功し、manifest.jsonとELFハッシュを生成できた。UTF-8読取の修正を確認済み。float化版でも同じ手順を使用する。

## 2026-10-28：リンカスクリプトの読取で再発

追加計測版のビルド集計で、STM32F446XX_FLASH.ldをPath.read_text()の既定CP932で読み、UnicodeDecodeErrorとなった。4構成ビルドと通常4モジュールの命令比較までは成功しており、製品のビルドエラーではない。

リンカファイルも日本語コメントを含むUTF-8入力としてencoding='utf-8-sig'を明示し、同じ集計を再実行した。ELF SHA、計測シンボル、RAM80376B/Flash250588B、8KiBスタック、ホストO0/O2、通常コード一致の検証JSON生成が成功した。

既存対策をログやCソースだけに適用し、.ldへ拡張できていなかった再発。補助検証器を実行する前に、read_text/write_textの全呼出しへ明示encodingがあることをASTで確認する。CMakeCache等の今はASCIIだけの入力にも適用する。flash-debug Skillの集計前手順へ追加し、build/isr-interval-validation/preflight_helpers.pyで対象スクリプトを検査する。status: resolved（今回の読み方と検証器事前検査）。MCU未接続・走行中性能は本件の検証対象外。

2026-10-04 再発: 残りのfloat化の一時集計コマンドでcompiler-diagnostics.txtを文字コード指定なしで読み、cp932のUnicodeDecodeErrorが発生した。ファームウェア、ビルド、検証結果への影響はない。encoding='utf-8'を明示して同じ診断を読み直し、残る昇格箇所を確認した。追加対策として一時コマンドにも保存時と同じ文字コード指定を必須とし、永続スクリプトvalidate_fpu_float_completion.pyの全テキスト読書きを明示指定した。再読取と検証スクリプトの成功を確認済み、resolved。

## 2026-10-26：GDB子プロセスの標準出力をCP932で自動復号した

全ライン暗部停止の静止実機試験で、Python subprocessのtext=TrueがGDBのUTF-8日本語ソース表示を既定CP932で復号し、reader threadのUnicodeDecodeErrorが発生した。取得stdoutがNoneとなり保存も失敗したため、その実行を成功証跡にしなかった。GDB側でどこまで処理したかを失われた出力から推定してはいけない。ファイル読書きのUTF-8指定だけでは子プロセスの復号を管理できない再発。

リセットなしの読取り専用再接続でpatternTrace0・開始0・auto0・emc0・カウンタ0・PRIMASK0・左右PWM/CCR0を確認してから再試験した。subprocessはバイナリstdoutで取得し最初にwrite_bytesで生ログを保存し、表示用だけUTF-8/backslashreplaceで復号、Python標準出力もUTF-8とした。再試験は終了コード0、19/20回の境界・全10個の解除・全状態復元・faultレジスタ0・detachを保存できた。

次回はGDBの初回呼出しからバイナリPIPEと原バイト保存を使用する。ログ処理失敗時は試験注入を繰り返さず、まずリセットなしで機体と出力・割込みマスクを確認する。SkillのGDB起動前へ反映した。今回の取得・復旧手順はresolved。証跡はbuild/emergency-line-loss/read-safe.raw.logとtarget.raw.log。

## 2026-10-30：Programmer CLIの生.logをUTF-8テキストと仮定

対象は計測版の実機導入証跡集計。新しいflash helperは.logへCLIの原バイトを書いていたが、消費側record_path_input_capture_deployment.pyがread_text(utf-8)で読み、0xb1でUnicodeDecodeErrorとなった。書込み・CLI照合・Flash読戻しは既に成功しており、MCUの失敗ではない。保存元のwrite_bytesと消費側の契約不一致。read_bytesとASCII成功句の照合へ修正し、同じ原ログ・保存RAM/Flashだけで集計を再開した。書込み/回収は繰り返していない。再発防止は拡張子.logで文字形式を推測せず、実producerの保存法を確認し、生ログの検証は原バイトで行う。表示用だけ明示的に復号する。実原ログのUTF-8不成立とASCII成功句、全256496B一致、IMU/SD/有効電圧/速度19項目一致を確認した。flash-debug Skillへ実行点を追加。status: resolved（証跡消費）。実走行入力と動的スタックはopen。
