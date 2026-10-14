---
type: codex-failure
date: 2026-10-12
task: "IMU接線項スナップショットのホスト比較"
status: resolved
severity: low
tags: [codex/failure, test, float]
---

# float保存値とホストの拡張精度式を比較してテストが失敗した

## 確認した結果

MinGWホストで、floatへ保存した適用済み接線項と`alpha*IMU_OFFSET_Y_M`を直接完全一致比較したところ、最初の非ゼロ入力で失敗した。表示値は約0.592607439と0.592607433。実装変更前後のIMU出力比較では差を確認していない。

## 原因と対処

ホストの式評価にfloat保存値より余分な精度が残り、比較側の丸め契約が異なった。期待値を`volatile float expectedTerm=alpha*IMU_OFFSET_Y_M`として保存型へ実体化してから比較した。許容誤差を拡大して回避せず、ファームウェアの演算も変更していない。

## 再発防止と確認

組み込みfloatスナップショットの厳密ホスト比較では、期待値も同じ保存型へ書き出してから比較する。演算式同士の比較と保存ビット列比較を分ける。`robotrace_v2/tests/check_slip_candidate_parity.py`へ反映し、改修前後の実IMUと既存SHADOWをそれぞれ8000入力でバイト比較して一致、スナップショット・校正/リセット・物理合成入力の検証も成功。結果は`analysis/slip-lateral-candidate/parity.json`。対処・検証済みのためresolved。
