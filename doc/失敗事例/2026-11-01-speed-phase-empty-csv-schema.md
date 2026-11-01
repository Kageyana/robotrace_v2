# 空の速度フェーズを含むCSVで列スキーマを先頭行から決めた

- 対象タスク: 2026-11-01の36走の速度追従解析。
- 事実: 全原CSVの解析とanalysis.json保存後、phases.csv出力時にDictWriterが追加キーを拒否して終了1。先頭の空フェーズはrows/secondsだけを持ち、後の有効フェーズにはRMSE等が存在した。原本・ファームウェア・SD設定の変更はない。
- 原因: 出力列をrows[0].keys()で決定した。空区間と有効区間のキー集合の差を検査していなかった。
- 対処: 全行キーの順序付き和集合で出力スキーマを確定した。空区間はrows=0/seconds=0を保持し、指標は空欄にした。標準出力も欠測をNoneで表示する。
- 再発防止: CSV保存前に全行キーを調べる。空→有効の最小例と実CSVの全行列幅を確認する。欠測指標をゼロへ置換しない。ログ解析Skillのfailure-prevention.mdにも入口の確認を追加する。
- 確認方法: analyze_speed_tracking_20261101.py再実行終了0、36原本のSHA256一致、全108フェーズの保存を確認。追加検証はanalysis/speed_tracking_20261101/validation.jsonに保存する。
- ステータス: resolved。全36本・108フェーズと30空フェーズを読み戻し、全CSV列幅一致、空指標が空欄であること、空→有効fixture、13999のRMS独立再計算0.22853738376270938m/sを確認した。validation.jsonへ保存済み。
- record-failures Skill: 個人Skill/プラグインのSKILL.md実在一覧を検索したが存在せず、AGENTS.mdの指定先へ手動記録した。
