# 14025/14027解析：補助関数の型・複数workdir・ブラウザ環境

- 対象タスク: DISTANCE14025の振動原因とD変更後14027の比較。
- ステータス: resolved（解析実行・表示検証の再発防止）。制御の実機原因分離とは別。

## 確認した結果

1. Node依存の解決をランタイムworkdirで行った呼出しへ、ファーム側の相対パスGet-Contentとgit statusを続けて記載した。Get-Contentが不存在となり、ErrorActionPreference=Stopで停止。元CSV・ファームへの変更なし。
2. NumPy配列から作った行辞書を既存distance_binsへ渡し、内部statistics.meanへnumpy.boolが到達してTypeError。比較表のテキスト出力まで完了していたがcomparison.jsonの確定前に停止。
3. Playwright同梱ブラウザの不存在を事前検査で検出。実在Edgeへ切り替えたが、sandbox一時プロファイルのDualEngine JSON置換がerror=-5となりブラウザ終了。通常権限の同条件では反復しなかった。

## 対処と再発防止

- 他リポジトリやランタイムworkdirを使う呼出しは依存調査だけへ分離する。ファーム読取は明示ROOTの絶対パス、Gitはgit -C ROOTとする。以後の解析はファームROOTを明示し、既存ファイルの読取に成功。
- 既存補助へ渡すNumPy値は境界でPython標準float/int/boolへ変換する。compare_distance_wobble_14025_14027.pyの行生成をmap(float, values)とし、実一次14023/14026の全区間を通過、comparison.json保存まで終了0を確認。
- 描画検証はPlaywrightライブラリの存在とブラウザ実行体の存在を別に確認する。verify_distance_wobble_chart.cjsに同梱→Edge→Chromeの実在選択を実装。同環境でheadless Edgeを使う場合は初回から必要な実行権限を選ぶ。許可された制限外実行で736px/360pxの2幅、SVG各2個、横はみ出しなし、文字の領域外はみ出しなし、pageerrorなし、画像生成を確認した。
- 生成済みの集計と原本SHAを保持し、失敗した段階だけ再開する。ブラウザ失敗で数値解析をやり直さない。

## 確認証拠

analysis/distance_wobble_14025/comparison.json、chart-check.json、chart-736.png、chart-360.png。原CSVの再読込前後SHA一致と、8f24c66の関連ソースSHAを保存した。

record-failures Skillは利用可能カタログ、個人skillsおよびプラグインcache内のSKILL.md一覧で見つからず、AGENTS.md指定先へ手動記録した。ログ解析Skillの実行前参照へ上記3点を追記する。
