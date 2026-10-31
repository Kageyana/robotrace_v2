# PATH_CURVEの最終採用版21

2026-10-31、ユーザーの「版21を最終版にします」という指定により、PATH_CURVE制御の最終採用を版21とする。

| 項目 | 最終採用値 |
|---|---|
| 制御版 `routeControllerVersion` | 21 |
| 自己位置推定器 `pathFusionEstimator` | 3（位置・方位・gyroずれの4状態） |
| 旋回FF `pathYawFeedForwardVersion` | 2（実1ms移動速度を使用） |
| 未照合時の加速保留 `pathUnmatchedAccelHoldVersion` | 1 |

版21は有効な白線照合ができない間の再加速を保留する。曲率別速度表、PID、白線補正、緊急停止と速度ゼロ後2秒の停止モーター出力保持を維持する。SDの有効な設定値を優先する。

版22で試した横速度状態の追加と、横偏差20mm/方位偏差30°による最大1m/sへの追加減速は採用しない。実験ソース・解析結果は履歴として残し、通常ビルドへ組み込まない。追加9走の依頼はユーザー指示で取り消した。新たな指示があるまで、版22の調整・追加試走依頼は再開しない。

通常のファームウェアソースは `robotrace_v2/`、Debug/DebugMarker/Releaseの各presetを使用する。ルートのVS Code設定は `${workspaceFolder}/robotrace_v2` を参照する。今回、制御CソースとSD設定は変更していない。

## 実機への復元

復元対象は、13913～13922の確認済み走行で使用した保存済みの版21計測構成。

- ELF: `build/path-unmatched-speed-validation/MatchedLineSpeedTiming-bc7a3fca376fe10b9e2e42612812e0eccbf97eedacd9363269056c3a160eef87.elf`
- ELF SHA256: `bc7a3fca376fe10b9e2e42612812e0eccbf97eedacd9363269056c3a160eef87`
- プログラム領域: 254652 B。
- 準備済み手順: `analysis/controller21_restore_20261031/hardware.py`。実ELFの265項目の配置を確認済みで、従来234項目とも一致。

本指定時点ではユーザーが接続できないため、実機への復元は未実施。新しい接続確認・現在像とRAMの保全後に書込み、全Flash一致と停止・出力0・初期化・設定の確認を行う。最終採用の決定を、実機の書込み完了として扱わない。

版21の既存10走の確認結果は[実機検証の記録](path-unmatched-speed-hardware.md)に保持する。数mmの残留ずれがある例も記録されており、今回の採用判断で新しい物理検証の結果を追加したものではない。

機械可読の採用情報は[path-controller21-final.json](path-controller21-final.json)を参照する。
