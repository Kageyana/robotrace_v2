# GDBの関数直接呼出し後に通常と異なる例外状態が残った

- 対象: 白線取得優先度の静止比較、計測版SHA886669cc…。
- 確認結果: 書込み/Flash全照合と通常起動ではsetupのIPSR0。GDBから`call powerLineSensors(1)`を行った後、setup→loopSystem→mainという停止点でIPSRとSCB ICSRのVECTACTIVEが32、NVIC activeレジスタ0/1は両方0となった。開始要求/autoStart/ログ記録/モーター出力0、Fault0だった。厳格な安全ガードで450µs負荷の投入前に中止し、この段階の値を静止比較の結果へ採用していない。
- 原因/未確定: 関数呼出し前後のCPU状態差は確認したが、GDB/サーバーの内部原因は未調査。通常のNVIC割込み発生と同一視せず、例外ガードを無効にして継続しなかった。別途USB通信エラー・Core ID取得失敗も発生したが、このCPU状態との因果は未確定。
- 対処: 所有プロセスを終了し、ユーザーの完全電源切替後に負荷0・優先度3・凍結0・開始/出力0・IPSR0を確認した。直接呼出しを全廃し、通常のセンサーテスト画面（display4/sensors7）で取得をON/OFFする。共有のmotor_testフラグは当該画面ではLED用で、モーターテスト画面を選ばず、各段で出力0と非走行を確認する。
- 再発防止: `bench_v3.py`の接続前にGDBのcallコマンドを拒否する。単一の所有サーバーを維持し、各停止点のIPSR/開始要求/出力/負荷/凍結/優先度を保存する。異常時はガードを緩めず、正常な起動へ戻してから進む。デバッガ停止中はTIM3/6/7を凍結し、終了時に元値を復元する。flash-debug Skillへ反映する。
- 検証: 関数を呼ばない通常UI経路で、旧0→新3→旧0→新3の4区間全てIPSR0、PWM0を維持した。新優先度は最大更新間隔1ms・age3ms以上0、旧は更新0。終了時に負荷0・優先度3・凍結0・センサーOFF・PWM0・IPSR0へ復元し、detachとサーバー終了が成功した。
- status: resolved（比較手順と設定復元）。GDB内部原因とUSB不安定の原因はopen。実走の改善確認とは区別する。

証跡: `build/adc-priority-validation/bench-v2/`の安全中止、`bench-active-state.log`のNVIC/ICSR読取り、`bench-v3/bench_validation.json`と各段のログ。最初の失敗原本は上書きしていない。
