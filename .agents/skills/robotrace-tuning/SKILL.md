---
name: robotrace-tuning
description: PID、速度フィードフォワード、速度/加速度、スリップ・経路追従パラメータを実機ログで比較し採否を決めるときに使う。
---

# 走行チューニング

1. 対象モードと変更前の基準ログを固定する。一次走行・`BOOST_DISTANCE`・`BOOST_PATH_REPLAY`・`BOOST_SHORTCUT` は別々に比較し、autoStartは5走完了した系列どうしで比較する。`robotrace-log-analysis` の手順で `emcStop`、`cntlog`、電圧、経路復元の有効性を先に検査する。
2. 1走で変えるパラメータは最大2個。SD上の設定値がコード既定値に優先するので、編集前に実機の設定を確認する。速度/加速度は `Core/Src/control.c`・`setup.c`・`setting/targetSpeeds.txt`、PID/速度FFは `PIDcontrol.c` と対応する `setting/*.txt`、経路追従は `pathFollower.c` と `setting/shortcut.txt` を確認する。形式は `robotrace-sd-settings` を参照する。
3. 制御変更は1 ms割り込み処理時間、モーター出力飽和、センサー更新間隔、緊急停止条件、ログサイズへの影響を確認し、ARMビルドを実行する。温度/電圧や速度設定が異なるログをそのまま効果比較しない。
4. 採用前に同条件の実機走行ログを最低10本収集し、完走率・ばらつき・ラップタイムを変更前と比較する。PATH系は `STOP_LOCALIZATION`、ラインフォールバック、5点以上の `optimalIndex` ジャンプ、Level 1 の合法余裕も確認する。再現性→完走率→ラップタイムの順で判断する。
5. 変更値・比較ログ番号・採否・残る実機確認を `analysis/` の結果とともに報告する。パラメータ変更内容をログヘッダだけから推定しない。
