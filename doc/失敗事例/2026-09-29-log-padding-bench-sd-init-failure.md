---
type: codex-failure
date: 2026-09-29
task: "Debugログへダミー42 Bを追加した空転ベンチ"
status: open
severity: low
tags:
  - codex/failure
  - robotrace
  - sd-card
  - stm32
---

# Debugベンチ初回起動でSD SPI初期化に失敗した

## 要約

Debugを書き込んだ直後の起動でカード検出は挿入状態だったが、SD SPI初期化に失敗し、ベンチはログ開始前に`SD_OPEN`で終了した。

## 発生した状況

- タスク: ダミー42 B追加後の通常・詳細PRIMARY空転ベンチ
- 実行環境: STM32F446、ST-Link、STM32CubeProgrammer 2.20.0、SDカード
- 前提条件: Debugを書き込み・ベリファイし、通常起動後にGDBからベンチ要求を行う。

## 何を試したか

1. 書込み後の通常起動でPRIMARYを開始した。
2. `SD_OPEN`停止後にカード検出、`initMSD`、FatFsマウント状態、`card_initialized`を読み取った。
3. Programmer CLIでUnder Resetハードウェアリセットを行い、起動時のSD初期化結果を再確認した。

## 結果・エラー

初回はカード検出がtrue、`initMSD=false`、`card_initialized=0`、`fs.fs_type=0`で、記録行・SD書込計測は0件だった。ハードウェアリセット後は起動時`SD_SPI_Init()`が`SD_OK`を返し、`initMSD=true`、マウント成功となった。原因となるカード応答や電源状態は初回に記録できていない。

## 原因

不明。初回のカード検出信号は正常だったが、カードのSPI初期化が失敗していた。リセット後の成功だけでは、カード起動タイミング、接触、電源、初期化処理のどれが原因か判定できない。

## 解決・回避策

Under Resetハードウェアリセット後に通常のファームウェア起動を通し、SD SPI初期化とマウントが成功した状態で空転ベンチを再実行した。軽量98 B・詳細151 Bの両PRIMARYケースが完走した。

## 今後の予防策

- ベンチ開始前に`initMSD=true`、カード初期化済み、FatFsマウント済みを確認する。
- `SD_OPEN`時はダミーバイトやログ書込み量の問題と決めつけず、カード検出、SPI初期化結果、mount結果を分けて記録する。
- 起動時SD初期化失敗が再発したら、リセットだけで終了扱いにせず、CMD0/CMD8/ACMD41応答とカード電源・接触状態を確認する。

## 関連

- 関連ノート: `analysis/debug-bench-results-2026-09-29.md`
- 参考リンク: なし
