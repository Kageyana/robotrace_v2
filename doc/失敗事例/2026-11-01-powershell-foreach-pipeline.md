# PowerShellのforeach文を直接パイプへ接続した

## 2026-11-02再発：角速度正規化の集計読取

- 対象タスク: 10/31以降のDISTANCE14本による角速度正規化の検証。
- 事実: 完了済みsummary.jsonの確認を即席PowerShellへ追加し、`foreach (...) { ... } | Out-String`を再度記述した。`An empty pipe element is not allowed`で構文解析段階に停止した。元CSV・ファームウェア・SD設定への変更はない。Pythonによる23原本の解析結果も保持された。
- 原因: 共通事前確認で既存対策を読んだ後も、解析結果の一時的な整形へ新しいforeach直結を追加した。既存対策の対象を原本の棚卸しだけに限定して解釈したことが誤りだった。
- 対処: 結果を配列へ格納してからFormat-Tableへ渡し、実診断付き4本の使用ゲイン・計画一致率・境界感度を読み取った。後続の候補集計・報告・再計算確認は`analysis/script/report_normalized_yaw_20261031.py`へ固定し、PowerShellの即席整形へ戻さない。
- 追加再発防止: summary.json等の派生結果にも同じ制約を適用する。今回の解析と再確認は`analyze_normalized_yaw_20261031.py`→`report_normalized_yaw_20261031.py`を使い、文書・Skillへ実入口を追記する。
- 確認: Pythonの2入口は終了0。14本・一次9本のSHA照合、4本の実記録P項/出力・記録間隔1msのD項、図の基準指令と実記録2155点の一致、候補RMS・制限率の再計算が成功。共通前提と集計を混同せず、物理的な発振解消は未確認のままとした。
- record-failures Skillは個人Skill・プラグインキャッシュのSKILL.md一覧で未発見のため、AGENTS.md指定の既存事例へ手動追記した。
- ステータス: resolved（解析入口、図の8通りの描画/操作検証、Skill実行前参照の追記・読み戻しを確認。Skillは挿入した1段落を除いた元バイトの一致も確認）。発振対策の実機採否とは別。

## 同日再発：14026以降の一次ログ棚卸し

- 事実: 初回の即席メタデータ棚卸しでもforeach文末を直接パイプへ接続し、同じ構文解析エラーで実行前に停止した。原本・ファーム・設定への変更は無い。
- 原因: 共通事前確認は読んだが、既存の配列経由の入口を使わず即席コマンドを作り、既存対策を再適用できなかった。
- 対処: 結果をinventory変数へ格納してからFormat-Tableへ渡し、14026～14044の19本・一次9本を確認。その後はanalysis/script/compare_shortcut_primaries_14026.py prepareのPython入口へ棚卸しを固定した。
- 追加予防: 14026以降の再棚卸しはこのPythonのprepareを使い、即席のPowerShell foreach文を作らない。別のPowerShell入口が必要な場合も実行前にParser検査を行う。ログ解析Skillの実行前へ具体的な入口を追加した。
- 確認: Pythonの棚卸し結果19本・一次9本が修正版PowerShellと一致。9本のSHAと全CSV検査、全一次O0/O2の経路・診断一致を確認した。
- ステータス: resolved（今回の棚卸し入口）。再発した事実は残す。

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
