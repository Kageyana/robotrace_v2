# ファーム直下からのビルド出力先を1階層誤った

- 対象タスク: 走行モード別PID・FFゲインの実装前ビルド。
- ステータス: resolved。
- 確認した事実: workdirをrobotrace_v2/へ指定した後、ログ保存先を../../build/とし、存在しないリポジトリ外の親ディレクトリを参照した。configureは成功したが、リダイレクトが失敗してbuildは実行されなかった。ソース変更なし。
- 原因: リポジトリルート基準とファーム直下基準の相対パスを混同した。
- 対処: 保存先を実在する../build/へ訂正し、ErrorActionPreference=Stopでリダイレクト失敗後の後続操作を止める。Debug/Releaseともビルド終了コード0と保存ログを確認した。
- 再発防止: ビルドのworkdirと出力先を同時に解決し、初回から親ディレクトリの実在とリポジトリ内であることを確認する。configureとbuildを別の結果として検査する。
- 確認方法: build/mode-gains-baseline-Debug.logと同Release.logにリンク完了・RAM/FLASH集計があること。
- Skill: record-failuresは利用可能一覧・ローカルSkill検索で見つからなかったため、本プロジェクトの必須記載項目を使って記録した。Skill実行自体ではなくビルド呼出しの出力先指定の失敗として扱う。
