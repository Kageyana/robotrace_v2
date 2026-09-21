---
type: codex-failure
date: 2026-09-20
task: 左右エンコーダによる一次経路方位推定
status: resolved
severity: low
tags:
  - codex/failure
---

# 新しいホスト用CテストがGitの無視対象になった

## 要約

`analysis/script/test_heading_estimator.c`を追加して実行できたが、`git status`には現れなかった。

## 発生した状況

- タスク: 方位カルマンとXY積分の合成テストを追加する。
- 実行環境: WindowsのPowerShellとMinGW GCC。
- 前提条件: `analysis/script/*`は既定で無視し、Pythonスクリプトなど一部だけ例外登録している。

## 何を試したか

1. `git status --short`でCテストが表示されないことを確認した。
2. `git check-ignore -v analysis/script/test_heading_estimator.c`で`.gitignore:33`を特定した。
3. `.gitignore`にCテストの明示的な例外を追加した。

## 結果・エラー

テスト自体はホストGCCで実行できたが、例外追加前は成果物として追跡されなかった。例外追加後は`git status`で未追跡ファイルとして表示される。

## 原因

`analysis/script/*`の包括的な無視規則に、新規Cテスト用の例外がなかった。

## 解決・回避策

`.gitignore`へ`!analysis/script/test_heading_estimator.c`を追加した。

## 今後の予防策

- `analysis/script/`へ新しい非Pythonテストを追加したら、実行後に`git check-ignore -v <ファイル>`と`git status --short`で追跡可否を確認する。
- C単体テストは`-Wall -Wextra -Werror`でコンパイルし、同一関数内の変数名重複もビルド前に検出する。

## 関連

- 関連テスト: `analysis/script/test_heading_estimator.c`
