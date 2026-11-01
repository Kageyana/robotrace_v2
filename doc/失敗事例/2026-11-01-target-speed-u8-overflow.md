---
type: codex-failure
date: 2026-11-01
task: 一次14007・14009・14012を元にした二次走行の直線減速調査
status: open
severity: high
---

# 4.41m/sを超える速度指令が8bit目標速度で折り返す

- 確認した事実: PIDcontrol.cのtargetSpeedはuint8_t。setTargetSpeed(4.6F)は266pulse/msを10へ折り返す。PIDと速度超過停止判定は折返し後の値、速度FFは元のfloat指令を使う。log_schema.hもU8。実関数のホスト再現で確認した。
- 実ログ: 14008・14010・14011・14013は約4.66mで目標が251～255から1～2pulse/msへ落ち、その後10pulse/msとなりemcStop=6。対応indexの半径は26425～32767mmで、曲率減速ではない。直線内部4.5～7.5mは実C生成でも全点4.6m/sを選択した。
- 原因: 10m/sまで設定できる速度指令と、255pulse/msまでの実制御・保存型の範囲不一致。エンコーダ換算58.019pulse/ms per m/sでは、折返し開始指令は約4.41235m/s。
- 調査時の対処: 原本・ファーム・SD・停止条件は変更せず、証拠とスクリプトをanalysis/primary_roc_14012/report.mdとanalysis/scriptの同番号4入口へ保存した。
- 次の修正: targetSpeedの定義・extern・使用側・ログ型を16bitへ整合。有限・0～10m/sの制限を設け、ログスキーマ変更をAGENTS.mdと変換/読込試験へ反映する。
- 再発防止: 速度上限・換算値変更と速度異常調査では、最大速度×PULSE_MILLIMETERを算出し、制御変数と保存型の両方の範囲を先に検査する。ROC表示だけから曲率誤判定と断定せず、生成時半径・計画速度・実制御速度・停止理由を分ける。4.4/4.42/4.6/10m/sを境界試験へ含める。
- 確認方法: verify_speed_wrap_14012.pyで実setterとjudgeOverSpeedを再現し、4本の折返し値と一致。pathFollower実Cの一次3本再生はO0/O2全出力一致。原binの全CRCと実機再走行は未確認。
- ステータス: open（修正・ホスト検証済み、初回実機ログと実走行は未確認）。record-failures Skillは個人Skill・プラグイン配下の実在一覧に無く、プロジェクト指定先へ手動記録した。

## 同日のuint16_t変更

ユーザー指示でtargetSpeedの定義・extern・保存スキーマをuint16_t/U16へ変更し、SLIP行受入の上限もUINT16_MAXへ変更した。setTargetSpeedには有限・非負・10m/s上限を適用し、FF指令にも同じ制限済み値を渡す。AGENTS.mdとログ文書を更新した。

validation/target_speed_u16/run.pyで4.4/4.42/4.6/10m/s、負値、非有限、上限超過と実ログwriter/readerの255/256/266/580を確認。通常/詳細×O0/O2で成功し、全CRC・列順・バッファ境界も一致。Debug/Releaseビルド成功、targetSpeedは両構成2byte・整列済みのSTRH。既存ビルド記録と警告比較し、新規warningなし。割込み呼出元・緊急停止・SDwriterのソースは変更していない。証跡はbuild/target-speed-u16-20261101。

書込み・実走行は行っていない。新しい一次ログを取得し、初回実機bin/CSVでU16の目標値・全CRCと直線加速を確認してから実機側をresolvedとする。
