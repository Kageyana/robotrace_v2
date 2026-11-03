# 進捗診断列の検証で、列名の前方一致を使った

- 対象：PATH系のSD時系列診断12列追加。status: resolved（検証入口・保存処理）、open（初回実ログ・実PC変換・実機処理時間）。
- 事実：fixture.pathProgressReasonを文字列の出現数1で探したところ、fixture.pathProgressReasonMaskにも一致して件数2となり、現在側のコンパイル前に検証器が停止した。生成途中のスキーマはメモリ内のみで、製品ファーム・原ログへ誤置換はしていない。旧側の保存検証は完了していた。
- 原因：C識別子の一致を文字列の部分一致として扱った。Reason/ReasonMaskのような接頭辞を共有する列名を境界として検査していなかった。
- 対処：fixture参照は単語境界付きの完全な識別子パターンで照合し、各列の件数が1であることを検査してから置換する。mainや宣言全体に使う既存のreplace_onceは変更せず、用途を分けた。
- 再発防止：列の定義・式の置換は列名をre.escapeし、前後に識別子境界を置く。通常/詳細の展開後に列名の一意性を確認する。保存試験ではヘッダ数値だけでなく、実writer/reader・全CRC・XY補完後の値・保存失敗時の区間保持を確認する。ログ解析Skillへ今回の入口と診断列の読み方を反映する。
- 確認：修正後のvalidation/path_progress_log/run.pyと最終validation/path_progress/finalize.pyが終了0。新旧×通常/詳細×O0/O2の56ログ4480行、slip値保持、一次/DISTANCE/SLIPの全バイナリ一致、PATH診断値、全CRC、XY補完後の保持、旧一次受入と旧PATH拒否を確認した。Debug/Releaseビルド・ARM厳密警告も通過。補正状態804B、ログ集計36B、最大単一フレームDebug240B/Release264Bを確認した。
- record-failures Skillはこの会話の利用可能一覧・既存検索で見つからないため、指定されたリポジトリ内へ直接記録した。
