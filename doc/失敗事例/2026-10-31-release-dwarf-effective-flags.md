---
type: codex-failure
date: 2026-10-31
task: "停止専用演算像のRelease型付き読戻し"
status: resolved
severity: medium
---

# Releaseの実効-g0を確認せずDWARF配置を照会した

## 確認した事実と原因

通常ReleaseのpathStoppedBenchDataへsizeof/メンバー照会を行い、unknown typeで停止した。全体CFLAGSへ-g3を加えても、ツールチェーンのRelease設定-Os -g0が後に付くため、PathFusionFilter型が見つからなかった。CMakeCacheのCMAKE_C_FLAGS_RELEASEは-O3 -DNDEBUGと表示されるが、実コンパイル命令はツールチェーンのplain setにより-Os -g0だった。cache値だけでは実効設定を判定できない。

アプリtargetへ最後の-g3を追加した案はCoreには効いたが、別targetのHAL/FatFsには効かず、全Cの実効-g検査がビルド前に停止した。型情報付きビルドの失敗後に配置準備を呼んだ箇所もあり、validation.json不在で停止した。実機はモーター0固定Debug計測像を維持し、これらの失敗で書込みを反復していない。

## 対処・具体的再発防止

失敗したソース/ログ/ELFはpath_stopped_bench_release_hardware_20261030、path_release_with_types_20261030、path_release_dwarf_20261031のbefore/failed記録へ保全した。独立コピーのツールチェーンだけを-Os -g3へ変更し、Coreは全バイト不変とした。configure後・最初のコンパイル前にcompile_commands.jsonの全Cについて、最後の-Oがs、最後の-gが3であることを検査する。Releaseの命令列は元の-g0構成と照合し、融合/追従/モーター/timer/main、計測コピー/スケジューラで一致した。実ELFでsizeof(PathFusionFilter)=108と265状態＋62計測配置の照会が成功した。

run_path_release_types_pipeline.pyは、前段非0なら後段を呼ばず、完成済みビルドのSHAと型情報証跡を検証してから配置準備へ進む。呼出しはrepo root workdir・絶対Python・必要なGNU権限を初回から選ぶ。sourceProjectのコピーと元のReleaseコードを混同しない。

## 確認と限界

build_path_release_with_types.pyの2構成、prepare_path_stopped_bench_release_hardware.pyの327配置、run_path_release_types_pipeline.pyの前段失敗注入と完成済み証跡照合が成功。型/実効フラグ/生成依存はresolved。Releaseの実機計測と実走効果はこの修復だけでは証明していない。
