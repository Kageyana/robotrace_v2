---
type: codex-failure
date: 2026-10-11
task: "BMI088前後加速度の符号修正"
status: resolved
severity: medium
tags:
  - codex/failure
---

# BMI088の前方正Yをコードで反転していた

## 確認した事実

ユーザーがBMI088の生+Yは機体前方と確認した。BMI088getAccele()は生Yを−1倍しており、前後加速度を前方正として扱うIMU_GetForwardAccelerationMps2()、速度積分、縦スリップ残差と整合しなかった。校正はオフセットを引くだけなので符号反転を修復しない。実走行への影響の大きさは未測定。

## 原因・未確定事項

ソース上のY反転が取付方向と不一致だった。反転を導入した経緯は未確定。前の図示でも設計座標コメントを実際の物理方向と混同した。X・Zの実機取付方向は今回のY確認だけでは確定しない。

## 対処

BMI088.cのY反転だけを削除し、前方正のコメントを追加。Y位置補正は維持した。後方31.68mmのIMUの向心成分は前方正なので、既存補正−0.03168ω²がこれを除去する。AGENTS.mdへ符号と旧ログの扱いを追記し、図を修正後のコードに更新。X・Z・ジャイロ、割り込み順序・共有状態・SPI取引は変更しない。

## 確認方法・結果

- Debug / Releaseのconfigureとbuild成功。BMI088.cに警告なし。
- ローカル検証`tests/accel_y_sign/check.c`は実際のBMI088.c・IMU.cへ模擬SPIを接続。静止校正成功、前方/後方の1g入力の符号、CW/CCW一定角速度で位置ずれY成分の除去を確認しPASS。testsはリポジトリ既定で管理対象外。
- TIM6の通常取得・校正ともBMI088getAccele()を使い、修正後の符号で校正されることを確認。処理・共有状態の追加がないため新たな割り込み競合なし。実測割り込み時間は未確認。
- 通常権限のコンパイラ起動は`Illegal System DLL Relocation`で失敗した。ソースを変えず制限外の実行でARM GCCとPC用GCCの両方が成功。根本のDLL問題は未調査。今後同じエラーが出たらコードエラーと判断せず、同一コマンドの実行環境を分けて確認する。

PC検証コマンド（リポジトリ直下）:

```powershell
gcc -std=c11 -Wall -Wextra -I tests/accel_y_sign -I robotrace_v2/Core/Inc tests/accel_y_sign/check.c robotrace_v2/Core/Src/BMI088.c robotrace_v2/Core/Src/IMU.c -lm -o tests/accel_y_sign/check.exe
./tests/accel_y_sign/check.exe
```

## 再発防止・実機での次の確認

軸変更時は生軸の取付方向、変換式、校正、位置補正、利用側の符号を個別に照合する。図生成時は変換式とオフセットをassertしソースSHA256を保存する。変更後初回ログで直進加速時Y正・減速時Y負、エンコーダ速度差分との符号整合、縦スリップ判定の挙動を確認する。旧ログ13059～13073のY列には反転後の位置補正も含まれるため、列全体を単純に反転して新ログ相当と扱わない。

## 変更後実機ログでの確認（2026-10-11追記）

ユーザーが変更後走行と報告した13074～13088（build Oct 11 2026 19:08:09）を確認。15本とも正常終了、校正100サンプル・読出しエラー0、期待行数一致、43列、数値有限、cntlog単調。5mm距離基準の刻みに欠落を示す兆候なし。3系列すべてで縦スリップフラグは0。

直進・加減速299標本（81ms平滑化、20ms間隔、|Yaw|<60deg/s、|aEnc|>=2m/s²）でYとエンコーダ速度微分の相関+0.986、符号一致99.0%。旧13059～13073は372標本で相関−0.991、一致0%。独立に左右累積パルスの2階差分で検査しても、新規149標本で相関+0.987、一致100%。加速時Y正、減速時Y負を実機ログで確認したため、このY符号不一致の修正はresolvedとする。

13077の横フラグON率9.80%は別の残課題。X/Z方向、真のYaw精度、旋回中Y補正量の独立検証、実測割り込み時間、コンパイラ起動時のDLL問題の根本原因は解決した扱いにしない。解析原本・ファームウェアに今回の追加変更は行っていない。

詳細：analysis/accel_y_13074_13088/report.md、metrics.csv、validation.json、independent_validation.json。再実行はanalysis/script/analyze_accel_y_13074_13088.pyとvalidate_accel_y_13074_13088.py。30本の原本SHA256を解析後と独立検査で再照合済み。

## 2026-10-12のコンパイラ起動制限の再発

スリップ並行評価版のDebugビルドで、通常権限のNinjaによるARM GCC起動が同じIllegal System DLL Relocationで失敗した。同一構成をrequire_escalatedでビルドすると成功し、その後のDebug/Release・通常/詳細とホストテストも同じ環境で実行できた。コードエラーとしてソースを変更しない回避策を再確認した。サンドボックス内のDLL配置問題の根本原因は未調査のままであり、この記録のY符号修正resolvedと混同しない。

2026-10-13、ADC位相修正の逆アセンブル確認で`arm-none-eabi-objdump`にも同じDLL再配置エラーが発生した。コンパイラ以外のGNUツールにも同じ実行環境制約がある。制限外の同一コマンドでDebug/ReleaseのPRIMASK・固定サイズ公開処理を確認できた。今後はobjdump/nm/sizeも起動環境の事前確認対象へ含める。環境問題の根本原因はopen。

校正値メタデータ追加時にも、制限環境の`gcc.exe`による保存テストとNinjaのARM GCC起動で同じエラーが再発した。制限外の同一テストは通常・詳細とも成功し、Debug/Release ELFに校正値保持領域が含まれることをnmで確認、DebugMarkerビルドも成功した。対策はコンパイラ起動環境を分けることで引き続き有効。次回は関連ビルドの前にこの事例を確認し、同じDLLエラーならソース変更で回避せず、許可された制限外環境で同一コマンドを検証する。DLL配置の根本原因は未確定のため、この環境問題はopenを維持する。

2026-10-13、通常43列化のschema/互換ホストテストで通常権限のMinGW起動が同じDLL再配置エラーとなった。制限外の同一テストで43/87列、3世代6形式の読込・保存復旧・次走解析一致が成功し、4構成ARMビルドも成功した。実行環境を分ける既存対策は有効だが、最初のホストコンパイル前にこの事例を確認できていなかった。書込み・デバッグSkillへ、ホスト/ARMコンパイル前の起動環境確認を追加する。根本原因の未確定は継続する。

2026-10-14、直線加減速の残差解析では既に生成済みのGCCホスト`planner.exe`を通常環境から起動してもWinError 623（Illegal System DLL Relocation）が発生した。コンパイラやGNUツールだけでなく、それらで生成したホストexeも事前確認対象となる。同じ読取専用解析を許可された制限外環境で実行し、一次基準4計画・DISTANCE/SLIP各4本の集計と図生成に成功した。一次4＋二次8本の原本ハッシュは前回検証とも一致。ログ解析Skillにもホスト実行環境確認を追記し、ファームウェアをこの起動失敗の回避目的で変更しない手順を残した。回避策の検証は成功、DLL配置の根本原因はopenのまま。

2026-10-21追記: ST-LINK_gdbserver.exe --helpにも、通常制限環境でシステムDLLの無効な再配置が再発した。承認付き制限外環境の同一コマンドは成功し、attach/Flash読出し/GDB/書込みも実行可能だった。GNUツール以外のCubeCLT同梱exeも初回から制限外の実行対象へ含める。根本原因は未確定でopenのまま。書込み・デバッグSkillの実行前参照へ対象を追加した。

## 2026-10-21 横ログ重複整理のPC変換器起動

RlogToCsv.exeを通常制限環境からPowerShellの出力リダイレクト付きで起動すると、StandardOutputEncoding is only supported when standard output is redirectedで起動失敗した。出力取得をPython subprocessのPIPEへ明示すると、実際のCreateProcess失敗はWinError 623（システムDLLの無効な再配置）だった。既知の「GCC生成ホストexeも初回から制限外」の対象へPC変換器を含める確認が不足していた。

同一のPython subprocessコマンドを承認された制限外環境で実行すると、既存のRlogToCsv.exeのまま通常66列・詳細72列の各50行を変換できた。全CRC・機体側CSVとの全保存値/派生値一致はanalysis/slip-lat-log-dedup/validation.jsonで確認した。変換器コード・原本ログ・DLL/ACLは変更していない。起動エラーを製品ログ破損と混同しない。

再発防止: ログ形式変更時のPC変換器検証も、RlogToCsv.exeの最初の起動からrequire_escalatedを使い、入力bin/出力CSVを今回解析フォルダへ限定する。出力はPython subprocessのstdout=PIPE,stderr=STDOUT等で明示的に取得し、終了コード0と新規出力の存在を確認してから数値照合する。SD設定・書込みデバッグSkillの実行前へ反映した。回避手順は今回の2プロファイルで検証済み。DLL配置の根本原因は未確定のためopenを維持する。
