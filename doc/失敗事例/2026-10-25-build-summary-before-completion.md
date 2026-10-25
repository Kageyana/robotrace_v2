---
type: codex-failure
date: 2026-10-25
task: ログ読込とPATH補助処理の所属整理
status: resolved
severity: low
tags: [codex/failure, build, asynchronous-output]
---

# ビルド完了前に集計ファイルを読んだ

## 確認した結果と原因

最終5構成のclean buildを実行し、`write_stdin`がまだ`session_id=87828`を返していた段階で、後続の警告照合を開始した。全構成終了後に生成する`final-builds/validation.json`が存在せず、照合がFileNotFoundErrorで停止した。ReleaseDetailまでの成功表示を全処理の完了と扱い、共通事前確認の「終了コード確認」を適用できなかった。

## 対処と再発防止

同じセッションを待ち、CleanupDiagnosticsを含む終了コード0を取得してから照合を実行した。ビルドの再起動やソースの変更で回避していない。

ビルドSkillの実行前手順へ次を追加する。`session_id`が残る戻り値では、同じ呼出し内の後続コマンドも集計ファイルを読まない。`write_stdin`を続け、終了コード0とセッション終了を確認した後、対象構成の全件・新しい集計ファイルの存在を検査する。並列化するのは依存しない処理に限定する。初回全体ビルドと後続の増分ログは別の出力先へ保存し、警告比較用の全体記録を残す。

## 確認方法とステータス

終了コード0の取得後、5構成の集計を読み、変更前の全体ビルドと警告を位置番号だけ正規化して照合した。全構成で既存6件が一致し、`build/log-path-ownership-20261025/final-builds/warning-comparison.json`へ成功結果を保存した。この実行順序の修正は検証済みのためresolvedとする。実機確認を示すものではない。
