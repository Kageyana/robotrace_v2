---
name: robotrace-hardware-review
description: ロボトレース機体のSTEP、KiCad回路、CubeMX iocを照合し、配線・寸法・極性・ファームウェアへの反映を判断するときに使う。
---

# 機体・回路確認

1. 対象に応じて `Machine/robotrace_v2 v86.step`、`Circuit/robotrace_v2_main_v2/`・`robotrace_v2_Linesensor/`・`robotrace_v2_Sidemarker/`・`robotrace_v2_Extended_UI/` 内の `.kicad_sch` / `.kicad_pcb`（PDFは補助）、`robotrace_v2/robotrace_v2.ioc`、関連する `Core/Src` と `Core/Inc` を照合する。全基板を毎回調べず、変更の影響範囲から辿る。
2. ピンや電源経路を変数名だけで推測しない。ADCのライン/マーカー入力とLED位相、左右モーターとPWM/エンコーダ、IMU取付方向と符号、バッテリー分圧、SD SPI/CS等のうち関係する接続を回路図と `.ioc` で突き合わせる。
3. 寸法変更時はSTEP・実測・コード定数を区別する。距離換算の正は `robotrace_v2/Core/Inc/encoder.h` の `PULSE_METER`、センサーの個数/順序と位置は `lineSensor` / `pathFollower` の定義を確認する。実機はライン `sensor[0]` が左端、`sensor[9]` が右端。BMI088生Yはソフトウェアで前進正へ反転する。
4. コードへ反映する場合は `.ioc` 再生成時の `USER CODE`、GPIO/タイマ/DMA競合、校正値とSD設定形式、ログスキーマ/単位、安全条件の影響を確認し、ビルドする。運用制約が変わったら `AGENTS.md` も更新する。
5. 報告には参照したファイル、確認できた事実と推測を分けて記し、クリアランス・極性・実機配線・センサー順など現物でしか確認できない事項を挙げる。
