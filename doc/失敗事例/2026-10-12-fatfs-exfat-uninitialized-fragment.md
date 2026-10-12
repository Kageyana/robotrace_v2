---
type: codex-failure
date: 2026-10-12
task: "空き容量確保後も失敗するbinの起動時CSV復旧を修正"
status: resolved
severity: high
tags:
  - codex/failure
---

# exFATファイルの断片カウンタ未初期化でseekが失敗

## 状況と確認した事実

- 前回のカードはFAT16・33,435,648 Bで容量不足だった。空き容量確保後の今回のカードはexFAT、fs.n_fatent=488146、csize=256、free_clst=487890。今回のエラーは容量不足ではない。
- 起動時に未変換4本を認識するが、13100.binのlogSourceOpenでFR_INVALID_OBJECTとなり、処理行数0で復旧が失敗。
- 13100.binは738081 B、157 B/記録、4684記録、メタデータ2181 B。ヘッダCRC・スキーマ・ファイルサイズ検証は成功した。
- メタデータ位置（735900 B）へのf_lseekがFR_INT_ERR=2を返す。input.fptrは512のまま、SPIエラー0。
- f_open後のfp->obj.n_fragが537000160などの未初期化値を持っていた。f_lseekはexFATでfill_last_fragを呼び、この値を使った不正なクラスタ番号をput_fatへ渡して失敗する。
- 使用しているFatFs R0.12cのf_openはexFATの切詰め経路ではn_fragを0にするが、通常のオープン成功経路では初期化していない。

## 修正

Middlewares/Third_Party/FatFs/src/ff.cのf_openで、exFATファイルの割当情報を設定する通常成功経路にfp->obj.n_frag=0を追加した。全FIL呼出箇所のゼロ初期化へ依存させず、未確定の末尾断片がないオープン直後の状態をFatFs内で設定する。ファイル形式・ログ列・割り込み・制御処理は変更しない。

## 確認と残る作業

- DebugとReleaseをビルド成功。変更箇所の新規warningなし。
- 修正版Debugの書込・照合・起動成功。Device ID 0x421を確認。
- 実機で13100～13103の4本がCSVへ確定し、conversionProgressは4/4、DONE、4738/4738行、100%。logRecoveryReady=true、conversionBatchComplete=true、SDロックと解析中フラグはfalse、共通入力ファイルは解放済み。
- 起動時CSV一覧に13100、13101、13102、13103を認識。通常経路は各CSVのsync/close/renameと元bin削除を成功した後にだけDONEへ進む。
- 続けて再起動し、fileCount=0、logRecoveryReady=true、SD/IMU初期化成功、同じ4本のCSV認識を確認。左右PWM=0、CFSR/HFSR=0、デバッグ接続を解除した。
- 既存run_log_deferred_tests.ps1が通常・詳細プロファイルで成功。異常注入のf_write errorは期待出力であり、実機エラーとは区別する。このホストテストのFatFsは模擬実装なので、実FatFs修正の検証根拠は上記の実機復旧・再起動結果とする。
- 未確定tmpの検出フラグlogTempWarning=trueは再起動後も残る。仕様に従い未確定tmpは自動昇格・削除していない。4本の確定済みbin復旧とは別の警告であり、復旧開始禁止にはなっていない。
- 今回の4本のCSVをPCへコピーして全行を独立検証する確認や、修正後の実走行は未実施。今回の変更はf_open初期化のみで、割り込み経路への処理追加はない。

## 再発防止

1. CubeMX/FatFs再生成・更新後は、exFATの通常f_openでn_fragを0にする修正が保持されているか確認する。
2. スタック上のFILを用いる実機起動経路で、binのヘッダ読取後にメタデータへseekし、CSV確定まで通過することを確認する。RAM FatFsを使う既存ホストテストだけでは実FatFsのこの欠陥を検出できない。
3. 容量不足を解消しても失敗する場合は、前回の原因を継続して仮定せず、ファイルシステム種別・空き容量・失敗段階・FatFs戻り値を取り直す。
4. 本調査開始時はST-Link USB通信エラーで接続不能だった。Programmer CLI一覧でもDEV_USB_COMM_ERR、SN/FWは「-」。ユーザーのUSB抜き差し後にDevice ID 0x421でattach成功。サーバーの待受を確認してからGDBを接続する。
5. 制限環境のARM GCC実行でDLL再配置エラーが再発した。通常環境の承認付きビルドで成功。ソース原因と混同しない。

## 証拠

- analysis/sd-debug-20261012/recheck/attach.txt、header.txt、seek.txt、repro.txt、flash.txt、completed.txt、reboot.txt。
- 前回の容量不足: 2026-10-12-sd-csv-recovery-disk-full.md。
