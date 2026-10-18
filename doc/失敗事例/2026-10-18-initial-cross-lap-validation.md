---
type: codex-failure
date: 2026-10-18
task: 13505以降のログXY照合
status: open
severity: low
tags: [codex/failure, analysis, marker]
---

# 正常終了ログ13514の先頭CROSSで既存周回検証が停止した

13514はemcStop=0・5周相当の終了メタデータを持つが、courseMarkerのCROSS遷移は開始4ms/方位0.05度の先頭行と、約180/540/900/1260/1620度の計6件。全遷移を各周180度位相のCROSSとして照合する既存features関数がassertで停止した。他の正常ログでは同じ検査に成功した。

今回の集計では全体を停止せず13514のXYを未判定とし、原本・ON時間・横判定位置は保持する。先頭CROSSが残留状態か検出イベントかの原因は未確認。データを0へ置換したりCROSSを削除して検査を通したりせず、追加照合が済むまでopen。正常ログの数値・CRC検証とXY判定可能性を分離した。

再発防止：周回検査前にCROSS全件の時刻・方位を保存し、開始直後も含めて位相の対応を検査する。周回照合できないログだけXY未判定とし、他ログ・ログ自身の診断集計を継続する。証跡は`analysis/slip_new_13504_13516/validation.json`のper_log_checks/13514、labels.json、report.md。原因や正しい周回対応の追加検証は未実施。
