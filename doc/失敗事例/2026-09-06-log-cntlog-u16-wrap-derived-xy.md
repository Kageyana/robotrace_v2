---
type: codex-failure
date: 2026-09-06
task: "Version 11 Level 1走行ログ12350～12357の確認"
status: open
severity: medium
tags:
  - codex/failure
  - logging
  - analysis
  - overflow
---

# cntlogの16 bit折り返しで派生XY座標が破損した

## 要約

68秒走行した12353で、`cntlog`が65531 msから14 msへ折り返した直後に、CSVの派生列`x`,`y`が約35 mジャンプした。実機の逸走や経路自己位置のジャンプではなく、`endLog()`の派生値計算で負の約65.5秒を`dt`として使用したログ生成不具合である。

## 確認結果

- 12353の実時間は16 bitカウンタを展開すると68632 ms。
- CSV行3464から3465で`cntlog: 65531 -> 14`、`x,y: (971,-2140) -> (29255,18434)`となった。
- 同じ箇所でも`optimalIndex=873`は連続し、`encCurrentN=29`、`targetSpeed=27`、`pathState=1`である。
- `pathErrorY_mm`、`pathErrorHeading_cdeg`、`linePointX_mm`、`linePointY_mm`などの保存列は継続している。
- 12356、12357は約36.7秒で折り返しがなく、同じ破損はない。

## 原因

`log_schema.h`は`cntRun`を`uint16_t`へ変換して`cntlog`へ保存する。`SDcard.c`の`endLog()`は`uint16_t time, beforeTime`に対して`dt = (float)(time - beforeTime) / 1000.0f`を計算するため、整数昇格後の差が`14 - 65531 = -65517 ms`となる。この`dt`を`calcROC()`、`calcXYcie()`、距離積算へ渡すことで派生列が破損する。

## 対処と再発防止

- ファームウェア側は差分を`uint16_t`へ戻してから秒へ変換し、16 bitの剰余差分として扱う。
- PC解析側の`analyze_path_following.py`は`cntlog`の折り返しを展開し、ラップ時間と単調性を判定する。
- 既存ログは`analysis/script/repair_cntlog_wrap.py`で`cntlog`を展開し、`encCurrentCorr_p`と`gyroVal_Z`から`x`,`y`を再計算する。元の実ログは上書きしない。
- 修正後は65.536秒を超える合成ログと実ログ12353で、`dt`が正、XYの不連続なし、実時間68632 msになることを確認する。
- 修正と実機ログ確認が完了するまでは`open`を維持する。
