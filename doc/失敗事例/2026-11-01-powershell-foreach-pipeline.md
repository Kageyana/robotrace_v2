# PowerShellのforeach文を直接パイプへ接続した

- 対象タスク: 13993・13994の速度ゲイン変更ログの比較。
- 事実: 初回の読取コマンドで`foreach (...) { ... } | Format-List`と記述し、`An empty pipe element is not allowed`で構文解析が失敗した。コマンド全体が実行前に拒否され、原CSV・ファームウェア・設定の変更はない。
- 原因: 共通事前確認の「foreach文の結果を配列へ格納してからパイプへ渡す」を、複数処理をまとめたメタデータ読取コマンドで適用しなかった。
- 対処: `$items = foreach (...) { ... }; $items | Format-List`へ変更して読取成功。Pythonの数値解析は別ファイルに分離した。
- 再発防止: `analysis/script/read_braking_metadata.ps1`へ配列受け渡しを固定した。実行前にPowerShell Parserで不正な直接接続を拒否し、配列経由を受理する2境界を検査する。同種のメタデータ棚卸しでは即席foreachパイプを組まず、この入口を使用する。ログ解析Skillの実行前参照にも入口と検証方法を追記する。
- 確認方法: `powershell.exe -NoProfile -ExecutionPolicy Bypass -File analysis/script/read_braking_metadata.ps1`で2境界と実CSV5本の読取を確認する。
- record-failures Skillは同会話の個人Skill検索で未発見のため、AGENTS.mdの指定保存先へ手動で記録した。
- ステータス: resolved。Windows PowerShellで不正/正規構文の2境界と実CSV5本の読込が終了0、ログ解析Skillの実行前参照への追記・読戻しを確認した。

## 同作業のNode CLI引用符

可視化の検証をPowerShellの`node -e`へまとめ、JavaScript内のHTML属性引用符を`\"`で保護しようとした。PowerShellでこの保護が効かず、Nodeへ届いたコードが途中で切れてSyntaxErrorになった。完了済みPython集計とHTML生成は終了0であり、チャート自体の構文エラーではない。

対処として`analysis/script/validate_braking_13993_13994.cjs`へ検証コードを分離した。以後はNode実体とこのファイルパスだけをPowerShellへ渡し、HTMLを再生成せず、完了した入力SHA・8走の埋込み・全script構文から検証を再開する。Skillの入口にもNode検証を追加した。ファイル入口は終了0で、10原本SHA・8走の埋込み・全script構文が合格。結果は`analysis/braking_13993_13994/validation.json`。ステータス: resolved（CLI引用符の失敗のみ。ブラウザ描画は未確認）。
