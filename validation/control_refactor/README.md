# 制御API・PID画面・速度設定・メタデータ回帰

リポジトリルートで実行する。このWindows環境ではGCCと生成exeに必要な実行権限を初回から使用する。

```powershell
python validation/control_refactor/run.py
```

実`motor.c`・`setup.c`・`control.c`・`pathFollower.c`の関数を抽出し、`logMetadata.c`を直接リンクする。O0/O2・`-Wall -Wextra -Werror`で以下を検証する。

- モーター飽和729ケースとPID画面2,592フレームの表示・編集・出力・状態遷移。
- 速度設定178ケースの初期値・19項目順・丸め・途中欠落・旧18項目・範囲外の修復。
- メタデータ35ケースの整数表記・int32境界・完全一致キー・重複・不正値・失敗時の出力保持、および実PATH読込。

任意の変更前Coreを持つスナップショットがある場合は、変更前の実関数と各フレーム・保存値・状態も比較できる。

```powershell
python validation/control_refactor/run.py --baseline build/main-refactor-20261025/before
```

スナップショットは通常実行に不要。参照コード内の既存signed/unsigned比較だけ局所的に警告を許容し、現行コードのWerrorは維持する。出力は`build/main-refactor-20261025/validation/`。実SDと機体を操作せず、表示・操作の実機確認は別途行う。
