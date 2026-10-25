---
type: codex-failure
date: 2026-10-25
task: ショートカット走行用の経路診断6列追加
status: resolved
severity: low
---

# PC変換用の検証binを連番以外の名前で保存した

## 確認した結果と原因

実経路制御・ログ保存・通常プロファイルの既存PC変換は成功したが、追加した`path-diagnostics.bin`の変換で`Filename/header number mismatch`となり停止した。内部ヘッダのログ番号は1であり、変換器が要求する連番ファイル名と一致していなかった。製品のログや変換器の不良ではなく、テスト用出力名の契約確認不足が原因。

## 対処と再発防止

検証種別はディレクトリ名へ分け、実保存binを`profile1/path-diagnostics/1.bin`と`profile0/path-diagnostics/1.bin`へ出力するようrunnerとハーネスを修正した。生成前に親フォルダを作り、既存出力CSVを削除してからPC変換する。実保存の番号・CRCをそのまま維持し、変換器の照合条件は緩めない。

再発防止は、PC変換を追加する初回から入力名がASCII連番であり、ヘッダのログ番号と一致することを確認すること。検証ケースの名前は親ディレクトリへ付ける。ログ保存・ビルドSkillの実行前参照へこの契約を追記する。

## 確認方法とステータス

リポジトリルート・必要権限で`python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`を再実行してexit=0。通常56列/154 B・詳細62列/163 B、各4行について全6列の正負・倍率・経路モード以外の0保存がPC変換後も一致した。既存50行の値、CRC・故障注入・DISTANCE/SLIP/PATH計画も成功。テスト側の出力契約修正はresolved。実機でのログ取得は別途未確認。
