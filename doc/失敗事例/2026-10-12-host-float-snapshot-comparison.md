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

## 2026-10-18：横候補版5の期待残差で再発

横候補の時間応答整合を実装し、変更前後を比較したホスト検査で`c->result.residual==expected-tangent`が停止した。期待値を`volatile float expectedResidual`へ保存してからmemcmpで比較すると、16,000周期すべてが一致した。製品コードや許容誤差を変更せずに検査が通過し、角加速度から作った両方向・5/20/50Hzの合成入力、継続・無効復帰・区間保持、従来判定・計画回帰も成功した。

既存の事前手順は読んでいたが、新設テストの最終減算式に適用できていなかった。期待LPF状態だけをfloat変数へ置いても、最後の減算の保存契約までは保証しない。実Cの保存値と式を完全一致させる新設検査では、コンパイル前に各CHECKの比較右辺を確認し、**最終演算結果**をvolatile floatへ実体化してmemcmpする。中間値の宣言だけで済ませない。書込み・ビルドSkillのホスト検証入口にも追記する。

証跡は`analysis/slip_lateral_v5/host-validation.json`と`matched-paired-and-response.log`。期待値保存の修正と実C回帰は検証済みのためresolvedを維持する。実機の検出精度を解決済みとする意味ではない。
