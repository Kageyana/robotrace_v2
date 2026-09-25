---
type: codex-failure
date: 2026-09-25
task: 上方撮影による独立XY基準ツール
status: resolved
severity: low
tags:
  - codex/failure
---

# Matroskaの粗いPTS刻みを実fps判定に使いかけた

## 要約

60 fpsの合成動画でもMatroskaのPTSが1 ms刻みに量子化され、PTS差分だけから計算した平均fpsが60 fps未満に見える場合があった。PTS量子化を撮影fps低下と誤認すると、正常な映像を拒否する。

## 発生した状況

- タスク: FFmpeg/ffprobeを使う固定カメラ映像の撮影条件検査。
- 実行環境: Windows、ffmpeg/ffprobe、PNGフレーム列から作ったMatroskaテスト動画。
- 前提条件: コンテナ時刻基準は映像の実fpsより粗い場合がある。

## 何を試したか

1. ffprobeの各フレームPTSから平均fpsを求めた。
2. その値を60 fps以上の撮影条件ゲートに使った。

## 結果・エラー

```text
60 fps素材のPTS平均が60 fps未満となり、同期半フレーム幅もコンテナ時刻刻みに縮んだ。
```

## 原因

PTSはコンテナの時刻基準で丸められる。PTS差分由来の短期平均は実撮影rateと一致せず、同じPTSを使ったhalf-frame許容幅も過小評価し得る。

## 解決・回避策

ffprobeの動画ストリーム`avg_frame_rate`を基準に公称フレーム周期と半フレーム幅を求め、PTS全区間から算出した平均fpsも60 fps以上の確認条件へ加えた。PTS差分はフレーム時刻、中央値間隔、長い欠落の診断に使う。

## 今後の予防策

- 同期許容幅はコンテナPTSの隣接差分ではなく、動画ストリームの公称周期から求める。
- 異なるコンテナ・可変fpsでも、ストリームrateとPTS全区間rateを合成動画で照合する。

## 関連

- 関連ノート: `doc/camera-xy-reference.md`
- 参考リンク: なし
