# ゴール後停止調査での検索パス・パッチ事前確認の再発

## 同日の通常ライン停止変更での再発

- 事実: 制御の呼出元探索で未確認のCore/Src/system.cを複数パスへ混ぜ、os error 2となった。終了コード2で後続を止め、既存ソースや未実行処理の変更はなかった。既存の親ディレクトリ検索対策を手組み呼出しへ適用できなかった再発。
- 対処: 実在Core/Srcのみへ検索を限定し、loopSystemはcontrol.c内と確認。今後は検索対象を既知の親ディレクトリへ固定した入口から組み立てる。ビルドSkillの再発防止参照へ追記する。
- 追加事実: 変更後Debugのconfigure/buildとELF保存は成功したが、警告を行正規表現で抽出したためCRの有無を別warningとみなし集計だけが停止した。Releaseは未実行だった。
- 対処・予防: validation/goal_marker_stop/build.pyをsplitlinesとstripによる両側の行末正規化へ変更。実Debug ELF・保存コピーのSHAと実CのSHA一致を確認して--resume-debugで集計を再開し、未実行Releaseのみビルドした。基準・新規とも5種類の既存warningで、新規診断なし。集計失敗で完了ビルドを反復しない。
- 確認方法: 実在親への修正検索終了0。build.py after --resume-debugはDebug集計・Releaseビルドとも終了0。before/after原ログ・実ELF・SHAはbuild/goal-marker-stop-20261101へ保全。
- status: resolved（検索・警告集計手順）。実機制動確認は別途open。

- 対象: 13989～13991・13993の完走後異常終了の調査。
- 事実: 存在未確認のCore/Src/logBinary.cを複数パスのrgへ含め、os error 2。続くレポート追加と既存記録更新のパッチへ未読の「#」だけの行をアンカーとして含め、検証拒否。既存の存在確認・完全行アンカー対策を実行点へ適用できなかった。
- 影響: 原CSVとファームウェアは未変更。拒否後にreport.md不在、既存終端地図記録の未追記を確認した。完了済みCSV解析とdiagnosis.jsonは保全した。
- 原因: モジュール名からソース分割を推測したことと、独立した文書追加へ無関係な未確認更新を混ぜたこと。
- 対処: 実在Core/Srcと-g '*.c'のみの検索へ戻り、終了0でSDcard.c/logSource.c/pathFollower.cの実体を確認。レポートと既存記録を独立パッチで再開し成功。
- 再発防止: ログ関数検索も親ディレクトリを入口とし、native終了2以上で停止する。文書追加は独立パッチで先に確定し、既存更新はその呼出し前に読んだ完全行だけをアンカーにする。Skillの実行前に同条件を追記する。
- 確認方法: 修正検索終了0、report.mdの新規存在と本文、解析6原本SHAを確認。record-failures Skillは個人Skill一覧に存在せず、指定保存先へ手動記録。
- status: resolved（検索と文書保存の手順）。機体の終端停止問題は別記録でopen。
