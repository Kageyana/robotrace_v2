---
status: resolved
task: ログ保存関連コミットだけをmainへ統合
---

# 選択統合時のレコード定義・テスト依存・CMake引数

確認結果: SHADOWブランチとmainでは内部のlogReadRecord引数・レコード生成・任意のslipDetection.cの有無が異なる。最初の競合解消スクリプトはmainも手動の複数マクロ展開だと仮定し、assertで書込み前に停止した。mainは既にLOG_FIELD_LISTで生成していた。ホストランナーもSHADOW専用ソースを無条件でリンクしていたためmissing file、参考実装の旧3引数呼出しと校正スナップショットの前提がmainと合わずコンパイルが停止した。

対処: 実際のmainの関数定義を読み、1引数APIと通常43列/詳細58列を保った。スリップ診断を追加せず、ログ保存に必要なライン校正メタデータと長い列名の処理だけを6e76ed9から補完した。ローカルテストランナーは任意ソースの存在を確認し、比較用参考実装は明示したmain用マクロで実APIを呼ぶ。移行前mainにはない校正テストを比較emitに混ぜず、校正保持は統合後の実テストで検証した。MinGWが未使用の公開テスト関数を必ず破棄するとは仮定しない。

また、PowerShellから引用符なしの`-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`を渡すと末尾`.cmake`が別引数となり、CMakeがextra path警告を出した。既存キャッシュでARMビルド自体は成功したが、引数を一単位で引用して再configureし、キャッシュのパスとprofile=0を確認した。

再発防止: 選択統合前にmain側の実関数定義・型・生成マクロ・任意ソースを読む。旧ブランチのハーネスがそのまま動くとは仮定せず、必要なスナップショットと参考計算のAPIを先に分類する。CMakeの`-D`指定は引数全体を引用し、ツールチェーンとログprofileをキャッシュで照合する。取り込まない制御ファイルの差分が空であることを検証する。

検証: 通常/詳細の保存故障注入・校正保持・再起動・DISTANCE/SLIP/PATH回帰とauto-run/CSV/PATH統合試験に成功。main前後の計画・CSV比較と配備済みPC変換器への43/58列入力は`build/log-main-integration/validation.json`へ記録する。実機試験は別途とする。
