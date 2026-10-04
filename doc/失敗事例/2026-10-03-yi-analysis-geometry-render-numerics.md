---
type: codex-failure
date: 2026-10-03
task: "yI解析の数値・CAD・図表生成"
status: resolved
severity: low
tags:
  - codex/failure
  - python
  - visualization
---

# STEP点・曲線順序・数値誤差上界を検証してから使う

## 確認した失敗

- STEPのMANIFOLD_SOLID配下の全CARTESIAN_POINTから範囲を求めると、PCBが約199 mm幅という誤った範囲になった。面の原点・曲線制御点が混入していた。
- 初回RMS図で旧-40～0曲線と拡張-120～20曲線をそのまま連結し、0から-120へ不要な直線が描かれた。
- レバーアーム連続式と離散中点式の差を根拠なく0.02 mm未満と仮定した検証が失敗した。実際の最大差は0.066110 mm。
- Matplotlibの既定ユーザーキャッシュが書込権限不足を警告した。

## 原因と対処

- STEPはトポロジーVERTEX_POINTの参照先を使用し、軸中心はCIRCLEの配置中心から求める。PCBの既知幅29 mm、部品中心とフットプリント中心の一致、左右配置を照合した。
- 曲線はアンカー/trimごとにyIの重複を排除し、昇順へ並べてから描画。修正図を再確認した。
- 中点のレバー項はdelta thetaだが厳密な点変換は2*sin(delta theta/2)。誤差上界63*sum(abs(delta-2*sin(delta/2)))を導出し、全10ログの実差を確認した。最大上界0.246311 mm以内だった。閾値を根拠なく緩める手順は使わない。
- MPLCONFIGDIRを解析出力内へ指定し、以後の描画で警告は出なかった。

## 再発防止と確認方法

1. CAD点群の種類を明記し、面の任意原点を寸法に使わない。円筒軸は円配置中心の同軸性を検査する。
2. 同じ系列のx座標は昇順・重複なしを保証し、保存後の画像を確認する。
3. 数値検証閾値は離散式から導く。スカラー積分で独立照合する。
4. Matplotlibキャッシュは書込可能なworkspace内へ明示する。

analyze_yi_step.py、plot_yi_12838_12848.py、validate_yi_12838_12848.pyへ対策を実装し、再実行・描画確認済み。status: resolved。


## 2026-10-04 再発: 校正ログ解析のフォントキャッシュ権限

check_line_calibration_12987.pyの描画は成功したが、既定のユーザー.matplotlibキャッシュ書込でPermission denied警告が発生。既存対策MPLCONFIGDIRを新スクリプトへ適用していなかった。matplotlibのimport前に解析出力内のmplconfigを指定し、再実行で警告なし・図2枚の生成と表示を確認した。今後の新しい描画スクリプトにもimport前の出力先指定を適用する。この再発の対処は検証済み。
