---
type: codex-failure
date: 2026-10-18
task: 動画基準の横補正候補閾値変更の4構成ビルド
status: resolved
severity: low
tags: [codex/failure, build, analysis]
---

# リンカの使用メモリ表示をB固定で解析した

## 確認した結果と原因

Debug通常ビルド自体は成功したが、検証スクリプトの `RAM:\s+(\d+) B` がリンカ出力 `RAM: 86 KB` に一致せず、Noneの添字参照で結果保存が停止した。ファームウェアのビルド失敗やRAM不足ではなく、表示単位を固定した集計の誤り。

## 対処と再発防止

RAM使用端は `arm-none-eabi-size -A -d` で得たELF実RAMセクションの配置から正確に集計する。KB等の丸められた表示値をバイト精度として使わない。FLASH表示もB/KB/MB/GBを識別する。変更構成をclean buildして、増分ビルドでメモリ表示がない場合と混同しない。ソース変更やRAMしきい値の緩和は行わない。

ビルドSkillの成果物確認手順に、表示単位の検査・ELF由来の正確な集計を追記する。

## 確認方法・状態

`analysis/script/build_slip_lat_threshold_validation.py` を既知のGCC実行制限を避ける環境で再実行。Debug通常88064B、詳細88104B、Release通常/詳細88288Bで4構成が成功し、state=1640B・stack=8192Bと各ELF SHA256を保存した。結果は `analysis/slip_lat_threshold_20261018/build-validation.json` と各sections.txt。今回の集計不具合はresolved。
