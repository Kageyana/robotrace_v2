---
type: codex-failure
date: 2026-09-29
task: "ST-Linkデバッグ手順のSkill化"
status: open
severity: low
tags:
  - codex/failure
  - robotrace
  - skill
---

# Skill検証器がPyYAML不足で起動しなかった

## 要約

`skill-creator`の`quick_validate.py`は、この環境の通常PythonとCodex同梱Pythonのどちらでも`yaml`をimportできず、Skill内容の検査へ進めなかった。

## 発生した状況

- タスク: `.agents/skills/robotrace-flash-debug/`の更新検証
- 実行環境: Windows、Python 3.10およびCodex同梱Python
- 前提条件: 検証器が`import yaml`を実行する。

## 何を試したか

1. 通常PythonとCodex同梱Pythonで`quick_validate.py`を実行した。

## 結果・エラー

```
ModuleNotFoundError: No module named 'yaml'
```

## 原因

両Python環境に検証器の依存パッケージ`PyYAML`がない。Skill本文の良否はこのエラーからは判断できない。

## 解決・回避策

検証器の実行は保留し、frontmatterの必須キーと書式、参照ファイル、コードフェンス、差分の空白エラーを別途確認する。

## 今後の予防策

- `quick_validate.py`実行前に、そのPythonで`python -c "import yaml"`を確認する。
- 不足時は依存パッケージを備えた環境で検証する。利用できない場合は手動の構造検査を実施し、`quick_validate.py`未実行を報告する。
- PyYAMLを用意して検証器の成功を確認した後、この記録を`resolved`にする。

## 関連

- 関連ノート: なし
- 参考リンク: なし
