---
type: codex-failure
date: 2026-09-22
task: "IMU温度ログのマーカーイベント確認"
status: resolved
severity: low
tags:
  - codex/failure
  - python
---

# importlibで未登録のdataclassモジュールを実行して失敗

## 要約

`spec_from_file_location()` で解析スクリプトを読み込む際、生成したモジュールを `sys.modules` に登録せず `exec_module()` を実行し、`@dataclass` の処理で例外になった。

## 発生した状況

- タスク: 解析スクリプトの関数を再利用して右マーカーイベントを表示
- 実行環境: Python 3.10
- 前提条件: 対象スクリプトに `@dataclass` がある

## 何を試したか

1. `module_from_spec()` の戻り値をそのまま `exec_module()` へ渡した。

## 結果・エラー

```text
AttributeError: 'NoneType' object has no attribute '__dict__'
```

## 原因

`dataclass` が型の所属モジュールを `sys.modules` から解決する時点で、動的モジュールが未登録だった。

## 解決・回避策

単純な調査用途では `runpy.run_path()` を使う。`importlib` が必要なら `exec_module()` 前に `sys.modules[spec.name] = module` を行う。

## 今後の予防策

`@dataclass` を含むローカルスクリプトの関数再利用には、通常インポートまたは `runpy.run_path()` を優先する。

## 関連

- 関連ノート:
- 参考リンク:
