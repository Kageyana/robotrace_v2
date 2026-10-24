---
type: codex-failure
date: 2026-10-12
task: スリップ新旧並行評価の電流品質判定
status: resolved
severity: low
tags: [codex/failure, firmware, calibration]
---

# 未更新のinitCurrentを新診断の有効条件にしていた

## 確認した事実

初期実装では新診断のcurrentOkをinitCurrentでゲートした。レビュー時に全代入を検索すると、control.cのfalse初期値以外に更新がなく、このままでは実機の電流補助診断が常に無効になると分かった。ホスト検出器単体のcurrentOk=true入力では接続側の問題を検出できない。実機へは書き込んでいない。

## 原因と対処

変数名の「初期化状態」を実装済みの状態更新と取り違えた。既存の電流処理・制御は変更せず、timer.cで実際の校正処理の終了と、その後の新取得を独立に追跡する。校正直後は取得更新番号を0とし、新しい取得まで無効とする。

## 再発防止と確認方法

品質・許可フラグを新しい利用先へ接続する前に、宣言だけでなく全代入と状態遷移をrgで確認する。検出器単体と取得アダプターのテストを分け、実際のtimer.cヘルパーで校正前、校正中、校正終了、新取得を検証する。

`python robotrace_v2/tests/check_slip_legacy_parity.py`で実ヘルパーの遷移検証と、8000周期の従来出力ビット一致がPASS。実機のADC・配線の正常性まで保証したものではない。初期実装の接続問題とその再発防止検証を完了したためresolved。
