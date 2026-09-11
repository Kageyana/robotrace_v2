---
type: codex-failure
date: 2026-09-11
task: "旧PATHログを新形式へ仮想変換してVersion 12経路を復元する試験"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
  - log-recovery
---

# 旧ログでは要求ショートカットLevelを適用Levelから推定できない

## 要約

旧ログ12370を仮想的に新形式へ変換する際、保存されていた適用`shortcutLevel=0`を要求Levelとして使ったところ、再生成結果の`shortcutBuildStatus`が旧ヘッダと一致しなかった。旧ログの要求Levelを適用Levelから推定してはならないことを確認した。

## 発生した状況

- タスク: 旧Level 0/Level 1ログの3列削除・復元照合。
- 実行環境: bundled Python、実ログ12360～12362、12369～12372。
- 前提条件: 旧ログには`shortcutRequestedLevel`、`routePointCount`、`routeGeometryCrc32`、生成時設定がない。

## 何を試したか

1. 旧ヘッダの`routeSourceLog`と適用`shortcutLevel`を使って仮想新形式を生成した。
2. Version 12経路を再生成し、生成結果ヘッダとCRCを設定して復元した。
3. 12370で生成結果ヘッダの照合が停止した。

## 結果・エラー

```text
12370: shortcutBuildStatus header=4, regenerated=0
```

## 原因

旧ログは要求Levelと適用Levelを区別して記録していない。12370では要求Level 1で回廊生成を試したが回廊なしで適用Level 0になった可能性があり、適用Level 0を要求Level 0と置くと生成結果が変わる。過去実機設定が実際に何だったかはこのログだけでは証明できない。

## 解決・回避策

旧ログの仮想復元試験では、指定した要求Levelと経路生成設定を明記し、過去実機設定の証明とは分ける。新形式では`shortcutRequestedLevel`、`routeShortcutSettings.*`、点数、CRC、生成結果ヘッダを保存し、推定を不要にする。照合不一致は欠測扱いにする。

## 今後の予防策

- 旧ログの適用Levelから要求Levelを逆算しない。
- 旧ログ試験は指定設定による復元試験として結果へ明記する。
- 新形式の復元は要求Levelと生成時設定が欠けている場合に合格させない。

## 関連

- 参考リンク: `analysis/script/path_log_recovery.py`
