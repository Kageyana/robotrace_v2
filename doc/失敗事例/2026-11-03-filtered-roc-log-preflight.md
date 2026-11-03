# フィルタ済みROCログの検証入口と編集アンカー

- 対象: `filteredROC_mm`の保存列追加。
- status: resolved（手順・fixture）。初回実機ログと実PC変換器の動作は未確認。
- record-failures Skillは利用可能一覧と前段の実在検索で見つからなかったため、指定されたリポジトリ内へ手動記録した。

## 事実と対処

1. 過去資料にある`validation/log_storage/run.py`をLeaf確認せず読もうとして不在で停止した。入口の事前確認ルールの再発である。実在一覧を取得し、現在ある`validation/mode_log_diagnostics/run.py`のwriter/reader生成を利用した。製品・原ログは変更していない。
2. ホスト生成で`static inline bool logBufferWriterRotateFullBuffer`を目印にし、宣言と定義の2箇所に一致して生成前assertが停止した。引数と末尾セミコロンを含む宣言全体へ限定した。
3. fixtureのmainを置き換えた結果、元のlegacySchemaTextが未使用になりWerrorで停止した。警告を無効化せず、既存スキーマ先頭一致のassertを追加して元の確認責務を維持した。
4. AGENTS.mdの長い1行の一部を完全行としてパッチへ渡し、照合失敗した。diffと新規文書の存在確認で全対象が未適用と確認し、新規文書作成と既存文書更新を分割した。最後に存在数1の限定置換を使った。
5. Skillの元バイトを直接decodeしてLFだけの見出しを探し、CRLFに一致せず準備assertが停止した。AGENTS.mdの完了済み更新は保全し、Skill準備だけを再開した。照合前に改行正規化し、保存時は元のCRLFへ戻した。

## 次回の事前手順

- 資料にある入口も`Test-Path -PathType Leaf`または`is_file()`を通す。新しい検証器は全依存を実行冒頭で照合し、不在なら生成・コンパイルへ進まない。
- 挿入目印は宣言の引数・セミコロンまで含め、宣言/定義の件数を別々に確認する。複数一致なら書く前に停止する。
- main等を置換する前に、元のassertが参照するconst・補助関数を一覧にし、必要な検査を新しいmainへ移す。
- 長いMarkdownの1行を部分文字列で置き換えるときは、出現数1を事前検査した範囲の小さい置換にする。パッチ拒否後は適用状態を確認し、未適用分だけを処理する。
- バイト列から読むSkillも、見出し検査前にCRLFを正規化し、元の改行形式・BOMを保持して保存する。前段完了後の準備失敗では完了済み変更を再適用しない。

## 確認

`validation/filtered_roc_log/run.py`の全依存確認、新旧×通常/詳細×O0/O2、56ファイル4480行の実writer/reader・getter・全CRCが成功した。実XY補完後のROC保持、旧形式一次拒否、既存列の全値一致も確認した。Debug/Releaseのビルド成功。実行前の再発防止をログ解析Skillへ追記する。
