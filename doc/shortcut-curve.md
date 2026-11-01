# SHORTCUT_CURVE（制御版29）

SHORTCUTと同じ経路を、一次走行の曲率に応じた可変速度で走る新モード。モード番号は6、ログのrouteControllerVersionは29。

LOG画面で一次ログを選び、左右で5番目のSHORTCUT_CURVEへ切り替え、中央押下で解析する。従来と同じくSHORTCUT設定のmaxLevelを使用する。開始画面はSHORTCUT CURVEと表示する。

形状はSHORTCUTと同じ。P1/P3を除外する版28の候補規則、変位設定50mmと合法余裕49.5mmの小さい方、補間、交差検査、短縮区間前後の保護域を引き継ぐ。短縮候補が無い場合、設定で短縮を無効にした場合、レベル0を指定した場合も、モード6のまま未短縮経路を曲率別速度で走る。無効な一次や延長失敗は走行を開始しない。

速度は短縮区間でも一次の丸め前曲率半径を使う。短縮後の経路が直線になったことを理由に、その区間の速度を直線上限へ置き換えない。DISTANCE/PATH_CURVEと同じSD設定bstStraight、bst1500～bst100をasignVelocity()へ渡す。SHORTCUT用のshortCut、PATH用のpathReplayは新モードの速度上限として使用しない。上限10m/s、先頭search速度、実際の短縮経路距離に沿ったacceleF/acceleDの加減速制約と減速先行decelLeadMmを適用する。加減速の速度量子化は上限を超えない方向へ丸める。端点・クロス直線・一次曲率の無い延長部分はPATH_CURVEの既存規則を使用する。

一次14045の保存速度表では計画最大3.5m/s、一次14038では4.2m/sとなる。これは生成した速度計画の上限であり、実機の到達速度や完走を確認した値ではない。search・shortCut・pathReplayは過去ログ非出力のため、ホスト検証では明示したfixture値を使い、過去SD値とは扱わない。

自動走行ではsetting/auto_run.txtの2～5走目にSHORTCUT_CURVEを指定できる。例は以下。

```text
2,DISTANCE
3,SLIP
4,PATH_CURVE
5,SHORTCUT_CURVE
```

設定後は再起動する。既定のDISTANCE/SLIP/PATH/SHORTCUTの順序は変更しない。SLIPの直前はDISTANCEという制約、欠落・破損時の既定修復、I/Oエラー時の上書き禁止を維持する。SHORTCUT_CURVEは常に1走目の正常な一次ログを参照する。

自己位置・操舵・FF・未照合中の加速保留・注意減速・喪失停止・短縮保護域は既存の経路制御を使用する。ゴール規定回数後の通常ライン追従による減速・停止、全10センサー暗部20msの完全逸脱停止も共通とする。1ms内に新しいSD読込や経路生成は追加しない。

ログはoptimalTrace=6で識別し、既存PATHの通常61列/169B、詳細67列/178Bを使う。列順・型・サイズ・計算版・CRC形式は変更しない。requestedModeは自動走行時SHORTCUT_CURVE、手動時MANUALを維持し、primaryLogNumber/routeSourceLogには解析元一次を記録する。ISR診断配列は既存添字を維持して添字6を追加する。

確認入口はvalidation/shortcut_curve/run.py（実生成・既存計画比較・一次曲率・キャッシュ・停止）、entries.py（画面・自動設定・保存）、build.py（Debug/ReleaseとRAM・IRQ責務）。一次14038/14045、減速先行0/61/99mmでSHORTCUTと形状が同じこと、既存3経路モードの全点・速度が変更前と一致すること、O0/O2一致を検査する。通常/詳細ログの実writer/reader・全CRCと方式/スキーマ不一致拒否も確認する。成果物はbuild/shortcut-curve-20261101へ保存する。検証コードはリポジトリ既定に従いGit管理対象外。

検証結果は全件成功。一次2本×先行距離3条件×既存3モード×O0/O2で変更前後の全計画が一致し、新モードはSHORTCUTと同じ形状・異なる曲率別速度となった。新方式の全計画もO0/O2一致、先頭・速度・加減速制約を満たした。欠測からの停止・回復、レベル0/短縮候補なし、モード切替とキャッシュ、一次半径の使用、画面5方式・表示幅・無変更時描画抑止・長押し1回・DMAガード、自動設定の全スロット・修復・I/O拒否・一次参照を確認した。通常/詳細×O0/O2の実writer/reader、全CRC、方式とスキーマ不一致拒否は成功。Debug/Releaseビルド成功、新規warningなし、通常構成のBSSは不変。DebugのDATAは16B減少、Releaseは不変。割込み診断ONでも実timer.cを両構成の実フラグで構文検査し、モード6・7要素配列の境界を確認した。

2026-11-01の初回実ログ14050は版29・方式6・参照一次14048。emcStop=0・SG=2で終了し、保存6793行はすべてTRACKING。一次からの実C再生成は6短縮区間・短縮53.28mm・最大変位46.10mmで、P1/P3を除外したSHORTCUTと速度以外の全計画列が一致した。短縮区間でも目標速度が変化し、全体最大目標は約3.50m/s。短縮区間の内部横偏差は平均絶対値1.00mm・最大5.48mmで、P5の保存注意状態はなかった。

通常の小R区間に内部横偏差最大16.83mm、加減速時に輪速と目標の差最大約1.02m/sが残る。内部偏差や補正前の輪速・ジャイロXYは床上の実位置の証明ではない。今回の原本はCSVのみで、実機RAMの計画CRC・原本bin全CRCは未照合。同条件10本以上での再現性・実位置・ラップの確認は別途必要。解析入口はanalysis/script/analyze_shortcut_curve_14048_14050.py、結果はanalysis/shortcut_curve_14048_14050/report.md。
