"""複数速度・曲率の段階分離解析を元CSVから独立照合し、日本語レポートを作成する。"""
from pathlib import Path
import csv,json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[2];O=ROOT/'analysis/multispeed_width_13006_13044';LOG=Path('F:/Dropbox/Document/robotrace/Log/v2');RAD=np.pi/180;PPM=58.019
TRAIN=[13006,13007,13012,13017];TEST=[13022,13030,13035,13040];MODES=['PRIMARY','PATH','SHORTCUT','DISTANCE','SLIP'];MODELS=['constant','speed','curvature','speed_curvature']
NAMES=dict(PRIMARY='PRIMARY',PATH='PATH',SHORTCUT='SHORTCUT要求・実PATH',DISTANCE='DISTANCE',SLIP='SLIP')
def read(name):
 with (O/name).open(encoding='utf-8-sig') as fp:return list(csv.DictReader(fp))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rms(x):return float(np.sqrt(np.mean(np.square(x))))
def values(rows,key):return np.array([float(r[key]) for r in rows])
def keys(model):
 k={'B0_mm'}
 if model in ['speed','speed_curvature']:k.add('Bv_mm_per_mps')
 if model in ['curvature','speed_curvature']:k.add('Bk_mm_per_5radpm')
 return k

def main():
 frozen=json.loads((O/'frozen_parameters.json').read_text());v=json.loads((O/'validation.json').read_text());dv=json.loads((O/'diagnostic_validation.json').read_text())
 assert sha(O/'frozen_parameters.json')==v['frozen_sha256']==(O/'pre_diagnostic_frozen.sha256').read_text()
 assert sha(ROOT/'analysis/script/analyze_multispeed_width_13006_13044.py')==v['script_sha256']
 assert sha(ROOT/'analysis/script/diagnose_multispeed_width_13006_13044.py')==dv['script_sha256']
 assert not frozen['CROSS_pose_used'] and set(TRAIN).isdisjoint(TEST) and frozen['encoder_mean_uses_raw_LR']
 health=read('health.csv');windows=read('windows.csv');metrics=read('per_log_metrics.csv');pred=read('predictions.csv');summary=read('summary.csv')
 assert len(health)==31 and len(metrics)==31*4 and len(pred)==len(windows)*4 and len(summary)==40
 assert {int(r['group']) for r in health if r['split']=='train'}==set(TRAIN)
 assert {int(r['group']) for r in health if r['split']=='test'}==set(TEST)
 fdict={(f['mode'],f['model']):f for f in frozen['width_models']}
 for f in fdict.values():assert set(f['coefficients'])==keys(f['model'])
 for f in json.loads((O/'window_sensitivity_models.json').read_text()):assert set(f['coefficients'])==keys(f['model'])
 delay=frozen['response']['delay_ms']/1000;tc=frozen['response']['tc_ms'];assert tc==0 # 今回選定された純遅れは元累積値から直接独立照合できる。
 errors=[];metric_errors=[];source_rows=0
 for hh in health:
  n=int(hh['log']);p=LOG/f'{n}.csv';assert sha(p)==hh['sha256'];source_rows+=int(hh['raw_rows'])
  with p.open(encoding='utf-8-sig') as fp:
   meta=dict(z.split('=',1) for z in next(fp).strip().split(',') if '=' in z);raw=list(csv.DictReader(fp))
  assert len(raw)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0 and meta['imuCalibrationValid']=='1'
  assert meta['gitCommit']=='8be46b1' and meta['buildDate']=='Oct  4 2026' and meta['buildTime']=='23:27:00'
  t=np.array([float(r['cntlog'])/1000 for r in raw]);left=np.array([float(r['encTotalL'])/PPM for r in raw]);right=np.array([float(r['encTotalR'])/PPM for r in raw]);angle=np.deg2rad([float(r['imuAngle_Z']) for r in raw])
  assert np.all(np.diff(t)>0)
  rr=[r for r in windows if int(r['log'])==n];lo=values(rr,'start_s');hi=values(rr,'end_s');dt=hi-lo
  assert np.allclose(dt,.1,atol=1e-12) and np.all(lo[1:]>=hi[:-1]-1e-12)
  # 左右それぞれを補間してから差・平均を作る。解析コードのresponse_rowsは呼ばない。
  dl=np.interp(hi-delay,t,left)-np.interp(lo-delay,t,left);dr=np.interp(hi-delay,t,right)-np.interp(lo-delay,t,right)
  qd=(dl-dr)/dt;qv=(dl+dr)/(2*1000*dt)
  vl=np.interp(hi,t,left)-np.interp(lo,t,left);vr=np.interp(hi,t,right)-np.interp(lo,t,right);speed=(vl+vr)/(2*1000*dt)
  om=(np.interp(hi,t,angle)-np.interp(lo,t,angle))/dt
  errors.append(max(np.max(abs(qd-values(rr,'response_diff_mmps'))),np.max(abs(qv-values(rr,'response_speed_mps'))),np.max(abs(speed-values(rr,'speed_mps'))),np.max(abs(om-values(rr,'omega_rads')))))
  assert errors[-1]<1e-6,(n,errors[-1])
  mode=hh['mode'];asym=frozen['asymmetry'];corrected=qd-asym['c_velocity_mm_per_m']*qv-asym['c0_mmps'];curvature=abs(om/speed)
  for model in MODELS:
   c=fdict[mode,model]['coefficients'];b=c['B0_mm']+c.get('Bv_mm_per_mps',0)*(speed-1.5)+c.get('Bk_mm_per_5radpm',0)*curvature/5
   err=(corrected/b-om)/RAD;saved=next(z for z in metrics if int(z['log'])==n and z['model']==model)
   metric_errors.append(abs(rms(err)-float(saved['angular_rms_dps'])))
   assert metric_errors[-1]<1e-6
   assert abs(np.sum(err*dt)-float(saved['covered_angle_error_deg']))<1e-5
   assert float(saved['B_min_mm'])>=70 and float(saved['B_max_mm'])<=200
   pp=[z for z in pred if int(z['log'])==n and z['model']==model]
   assert np.allclose(b,values(pp,'predicted_B_mm'),rtol=0,atol=1e-6)
   assert np.allclose(err,values(pp,'angular_error_dps'),rtol=0,atol=1e-6)
 for row in summary:
  rr=[r for r in metrics if r['split']==row['split'] and r['mode']==row['mode'] and r['model']==row['model']]
  assert abs(rms(values(rr,'angular_rms_dps'))-float(row['angular_rms_dps']))<1e-9
  assert abs(rms(values(rr,'covered_angle_error_deg'))-float(row['covered_angle_error_rms_deg']))<1e-9
 # 一次遅れカーネルをIIRの逐次計算と合成入力で照合。窓長可変の誤用もチェック。
 tt=np.arange(1000)*.001;raw=np.c_[tt*100+10*np.sin(tt*8),tt*1000];alpha=np.exp(-1/4);kernel=(1-alpha)*alpha**np.arange(128)
 conv=np.column_stack([np.convolve(raw[:,j]-raw[0,j],kernel,mode='full')[:len(tt)]+raw[0,j] for j in [0,1]])
 recursive=np.empty_like(raw);recursive[0]=raw[0]
 for i in range(1,len(tt)):recursive[i]=alpha*recursive[i-1]+(1-alpha)*raw[i]
 assert np.max(abs(conv-recursive))<1e-8
 residual=read('residual_quantiles.csv');coverage=read('coverage.csv');jack=read('asymmetry_group_jackknife.csv');gates=read('asymmetry_gate_sensitivity.csv');abl=read('component_ablation.csv');sensitivity=read('window_sensitivity.csv')
 for split in ['train','test']:
  for mode in MODES:
   counted=sum(int(z['windows']) for z in coverage if z['split']==split and z['mode']==mode)
   expected=sum(1 for z in windows if z['split']==split and z['mode']==mode)
   assert counted==expected
 def get(mode,model,key='angular_rms_dps',split='test'):
  return float(next(r[key] for r in summary if r['mode']==mode and r['model']==model and r['split']==split))
 report=['# 複数速度・曲率ログでの時間応答／左右換算差／実効幅の分離検証','',
  '結論: 31ログに拡張し、時間応答を先に選定、左右換算差を直線と定常旋回から推定して固定、その後にモード別の幅モデルを比較した。時間応答と左右差の扱いによる改善が大きい。速度・曲率依存の追加効果はDISTANCEでは小さく、SLIPではtest3本すべてで小幅な改善がある。全体へ共通の可変幅を採用する根拠は得られていない。','',
  '## 対象・系列分離','',
  '- 同一gitCommit=8be46b1、buildDate=Oct 4 2026、buildTime=23:27:00の正常終了31本。train14本、test17本。期待行数・必要列・IMU校正・原データSHA256を確認した。',
  '- trainの一次ログ系列=13006/13007/13012/13017、test系列=13022/13030/13035/13040。同じ一次ログに続くPATH、SHORTCUT要求、DISTANCE、SLIPを同じ側へ割り当て、系列を跨ぐ情報漏れを防ぐ。',
  '- PRIMARY 8本、PATH 7本、SHORTCUT要求6本、DISTANCE 5本、SLIP 5本。SHORTCUT要求ログの実optimalTraceは4（PATH）であり、実SHORTCUTとしては扱わない。DISTANCE/SLIPもrequestedModeで分けて評価する。',
  '- 緊急停止ログは除外。13006はclosureValid=0だが正常終了・距離検証成功なので相対センサー解析にのみ含む。二次ログのdistanceScaleVerified=0は一次ログ限定の検証項目のため、その値だけで除外せず、紐づく有効一次ログの距離検証成功を確認した。',
  '- testの一次ログは以前から確認済み。今回のモデルはtrainだけで選定・固定したが、既存ログの探索的検証であり、新規収集した独立実機testではない。電圧は約7.4～8.3V、trainとtestで走行順・電圧が交絡する。',
  f'- 非重複100ms窓はtrain {sum(1 for r in windows if r["split"]=="train")}、test {sum(1 for r in windows if r["split"]=="test")}、計{len(windows)}。開始0.25秒と終了0.15秒以上を除外。13031の最大15msの行間隔を含む4窓は時間補間精度不足として除外し、残る同ログは評価に含めた。行間隔は距離基準ログなので、15msだけを根拠に行落ちとは断定しない。',
  '- 速度は左右生累積Encoderの平均から求める。SLIPのencTotalOptimalに含まれる距離補正を速度へ混ぜない。CROSS姿勢・CROSS位置とも今回の解析に使わない。疎なgyroVal_Zは再積分しない。','',
  '## 分離するモデル','',
  '```text',
  'Cdiff = (encTotalL - encTotalR)/58.019       [mm]',
  'Cavg  = (encTotalL + encTotalR)/(2*58.019)   [mm]',
  'Ctilde(t) = first_order_filter_T(C(t - delay))',
  'dtilde = ΔCtilde_diff/Δt                   [mm/s]',
  'vtilde = ΔCtilde_avg/(1000*Δt)             [m/s]',
  'ω_IMU  = Δ(saved imuAngle_Z)/Δt            [rad/s]',
  'B(v,κ) = B0 + Bv*(v - 1.5)/1.0 + Bκ*|κ|/5 [mm]',
  'dtilde = B(v,κ)*ω_IMU + c*vtilde + c0',
  'κ = ω_IMU/v                               [rad/m]',
  'ω_encoder = (dtilde - c*vtilde - c0)/B(v,κ)',
  '```','',
  '- ステップ1: trainで遅れ0～12ms・一次遅れ時定数0～12msを各2ms刻みで探索。各モード固有の定数Bと共通のc/c0を補助変数として推定し、一次ログ系列単位LOOの角速度残差を、モード・ログ等重みで比較した。',
  '- ステップ2: 応答を固定し、直線相当（|ω|<10deg/s、|dω/dt|<100deg/s²、滑りフラグなし）でc/c0を、定常旋回（|ω|≥30deg/s、|dω/dt|<300deg/s²、滑りフラグなし）でモード別Bを交互に5回推定。幅の可変係数へ左右差を吸収させないよう、以降の全幅モデルでc/c0を共通固定した。',
  '- ステップ3: モード別に固定幅・速度依存・曲率依存・両方を比較。正則化はtrainの系列LOOで0/0.01/0.1/1から選定。このLOOは共通校正を全trainから固定した条件付き評価で、全校正手順を再学習する完全な外側CVではない。最終testは系列全体を保持して評価する。',
  '- Encoder累積値を1msの数値格子へ線形補間して応答モデルを計算する。元の1msサンプルを復元した主張ではない。IMU角度の差分は保存値から直接計算する。',
  '- 観測モデル上では時間応答・左右差・幅の項を分けたが、独立Yaw基準がないため、物理的な車輪径差・接地滑り・gyro誤差を一意に分離した校正ではない。','',
  '## 時間応答・左右差の結果','',
  f'- 選定された共通応答はEncoder側の純遅れ{frozen["response"]["delay_ms"]}ms、一次遅れ時定数{tc}ms。系列LOO RMSは{frozen["response"]["series_LOO_rms_dps"]:.4f}deg/s。gyroのハードウェア遅延が8msと確定した意味ではなく、センサー・機械・補間を含む相対応答近似。',
  '- 前解析の「未来IMUへ+10ms移動」と異なり、今回の純遅れは過去のEncoder区間を現在IMU区間へ対応させる。構造としては過去データで計算できるが、実時間EKFへの適用・遅延観測処理は未実装。',
  '- trainモード別の応答感度では、PRIMARY/PATHは遅れ6ms＋時定数2ms、SHORTCUT要求/DISTANCE/SLIPは純遅れ8msが最小。共通の8msは完全なセンサー伝達関数ではない。',
  f'- 分離推定した c={frozen["asymmetry"]["c_velocity_mm_per_m"]:.6f}mm/m（左右差に約{frozen["asymmetry"]["c_velocity_mm_per_m"]/10:.4f}%相当）、c0={frozen["asymmetry"]["c0_mmps"]:.6f}mm/s。直線370窓、定常旋回288窓から推定。高速の直線はDISTANCE 4窓・SLIP 12窓と少ない。',
  f'- 平均換算を保つ左右スケール補正に形式的に換算すると左{1-frozen["asymmetry"]["c_velocity_mm_per_m"]/2000:.8f}、右{1+frozen["asymmetry"]["c_velocity_mm_per_m"]/2000:.8f}に相当する。ただし実機の校正値として採用せず、コードへ反映しない。',
  f'- 系列を1つずつ外したcは{min(float(r["c_velocity_mm_per_m"]) for r in jack):.3f}～{max(float(r["c_velocity_mm_per_m"]) for r in jack):.3f}mm/m。直線判定条件を変えると{min(float(r["c_velocity_mm_per_m"]) for r in gates):.3f}～{max(float(r["c_velocity_mm_per_m"]) for r in gates):.3f}mm/mとなり、精密な物理校正値とは扱えない。',
  '- c0は残る定数不整合を吸収する項で、gyro biasや車輪センサーoffsetそのものとは同定できない。固定係数のままc0だけを外す診断ではtestが改善するモードがあり、この項の汎化も不十分。診断を根拠にtestへ合わせた係数の再選定はしていない。','',
  '## 別系列test・100ms窓の結果','',
  '保存IMUに対する角速度残差RMS [deg/s]。固定幅も今回のtrainでモード別に再推定した値であり、現行EKFの117.8375mmの直接評価ではない。','',
  '| モード | 固定幅 | 速度依存 | 曲率依存 | 速度＋曲率 |','|---|---:|---:|---:|---:|']
 for mode in MODES:report.append('| '+NAMES[mode]+' | '+' | '.join(f'{get(mode,model):.4f}' for model in MODELS)+' |')
 dist_gain=100*(1-get('DISTANCE','speed')/get('DISTANCE','constant'));slip_gain=100*(1-get('SLIP','speed_curvature')/get('SLIP','constant'))
 report+=['',
  f'- DISTANCEの速度依存は固定幅から{dist_gain:.2f}%低下。曲率を加えても改善しない。SLIPの速度＋曲率は{slip_gain:.2f}%低下し、13034/13039/13044の3本すべてで改善。',
  '- 幅の候補はB0=PRIMARY 112.5236、PATH 112.6613、SHORTCUT要求112.8153、DISTANCE 112.0732、SLIP 112.0993mm。すべて保存IMUとこの時間応答・左右差モデルに対する相対値で、物理トレッドや実機採用値ではない。',
  '- SLIPの速度＋曲率モデルはB0=111.6110mm、Bv=+3.4628mm/(m/s)、Bκ=+0.9220mm/(5rad/m)。testでの予測幅は110.2588～115.7071mm。DISTANCEの速度依存はBv=+1.4982mm/(m/s)。低速PRIMARYの速度係数は負であり、モード共通の単純な速度依存とは結論できない。','',
  '### 高速モードのログ別再現性','',
  '| ログ | モード | 固定幅 RMS [deg/s] | 速度＋曲率 RMS [deg/s] |','|---|---|---:|---:|']
 for n in [13033,13038,13043,13034,13039,13044]:
  a=next(r for r in metrics if int(r['log'])==n and r['model']=='constant');b=next(r for r in metrics if int(r['log'])==n and r['model']=='speed_curvature')
  report.append(f'| {n} | {a["mode"]} | {float(a["angular_rms_dps"]):.4f} | {float(b["angular_rms_dps"]):.4f} |')
 report+=['',
  '## 大きな不一致と原因の未確定事項','',
  '- 13031(PATH)は固定幅残差146.7582deg/s。20.055～20.155秒では平均前進速度約0.977m/s、左右差速度約4453mm/s、IMU角速度約550deg/sに対し、Encoder換算角速度は約2260deg/sとなる。幅を数mm変えるモデルでは説明できない。',
  '- 13031は後半の局所的な不一致、13033(DISTANCE)は固定幅残差49.5431deg/sで角速度ピーク等の不一致がある。いずれも正常終了なので主評価に含める。滑りフラグなしでも大きい残差があり、フラグだけでは全不一致を検出できない。',
  '- IMU.cのYawはgyro積分のみで、参照した現行コードにPATHからimuVal.angle.zを書き換える処理は見つからなかった。元ログの保存Yaw区間差と瞬時gyroも調べたが、疎な瞬時gyroとの局所差だけでYaw破損とは断定しない。接地滑り、車輪回転と機体旋回の乖離、Encoder系、IMU系のどれが原因かは実機・独立基準での確認が必要。',
  '- 13030系列のPATH/DISTANCEの大きな残差は可変幅へ吸収させて解決できない。系列を除外して改善した数値を主結果としては使わない。','',
  '## 窓長・構成要素の感度','',
  '- 40/200ms窓でも時間応答・左右差と正則化強度を固定し、幅係数だけtrainで再推定した。SLIPの速度＋曲率は40msで8.292→8.032deg/s、200msで5.003→4.671deg/sと小幅改善。DISTANCEの大きな不一致は窓長を変えても残る。',
  '- 同じ幅係数・c/c0のまま時間応答だけをOFFにすると、testのPRIMARYは2.542→12.123deg/s、SLIPは6.061→23.334deg/sへ悪化。今回の相対角速度モデルでは時間応答の寄与が可変幅の寄与より大きい。',
  '- 同じ幅係数のままcとc0を両方OFFにすると、PRIMARYは5.205deg/s、SLIPは10.830deg/s。c0だけOFFではそれぞれ2.215/5.490deg/sと改善する。校正の定数項を実機へ持ち込む根拠は得られていない。',
  '- 覆い窓の積算角度差もCSVへ保存。例えばPRIMARYの固定幅はtest全窓でRMS31.36°、SLIPは50.90°、速度＋曲率SLIPは47.43°で、長時間積算の整合性も十分とはいえない。この指標は正確な周回境界の閉路や独立Yaw精度ではない。','',
  '## 何が確認でき、何が残るか','',
  '- 確認できた: モード別の複数速度・複数曲率ログで、相対時間応答・共通左右差項を固定した後にもSLIPで可変幅の小幅な改善が再現した。',
  '- 未確認: 物理的な幅の速度依存／曲率依存の一意な分離、独立Yaw精度、全モード共通係数、可変幅を入れたEKF全体のXY・閉路改善、実機の時間同期や滑り原因。',
  '- 高速ほど直線、低速ほど急旋回という速度計画の相関が強い。複数速度を含むが、速度×曲率を独立に変えた実験ではない。DISTANCE/SLIPのtrainは各2本・test各3本で、電圧条件も異なる。',
  '- 結論: 現行EKFとファームウェアは変更しない。次の解析候補は、時間応答を合わせたEncoder角速度観測と左右差の速度比例項。固定定数offsetと可変幅は診断候補に留める。実機の物理校正には追加の独立基準を用意する。','',
  '## 図','',
  '![速度と曲率のカバレッジ](speed_curvature_coverage.png)','![時間応答選定](response_selection.png)','![モード別検証](mode_validation.png)','![大きな残差の元波形](large_residual_logs.png)','',
  '## 再現・検証','',
  '```powershell','python analysis/script/analyze_multispeed_width_13006_13044.py','python analysis/script/diagnose_multispeed_width_13006_13044.py','python analysis/script/validate_multispeed_width_13006_13044.py','```','',
  f'- {source_rows}元行、31ログ、{len(windows)}窓、124モデル別ログ指標、{len(pred)}予測行を照合。元CSVから左右を別々に補間して純遅れ区間・速度・IMU角速度を独立再計算。入力差分の最大差{max(errors):.3g}、角速度RMS差{max(metric_errors):.3g}deg/s。',
  '- 元CSV、固定パラメータ、解析ソースのSHA256不変、系列分離、非重複窓、係数名集合、幅範囲70～200mm、速度×曲率カバレッジ集計、一次遅れ畳込みと逐次IIRの一致を確認。',
  '- 4図を目視確認。モード内比較図は全データを収容し、外れログを表示した。元CSV・既存EKF・ファームウェア・SD設定を変更していない。ビルド・書込み・走行は実施していない。','']
 (O/'report.md').write_text('\n'.join(report),encoding='utf-8')
 plan=['# 独立した速度×曲率での実機確認案','',
  '本解析では新規実機走行を行っていない。既存ログでは速度計画と曲率が強く結び付いているため、下記は物理同定を確認するための収集条件案。既存の安全制約・室内環境・IMU校正・バッテリー運用を維持する。','',
  '- 外部動画または別の独立角度計測で機体Yawを記録する。CROSS姿勢は使わない。時計同期用の機体LED等のイベントを併記する。',
  '- 直線区間で複数速度を個別に保持し、左右Encoder平均の距離換算、左右差の速度比例成分、gyro静止バイアスを別々に確認する。',
  '- 同じ曲率を複数速度で、同じ速度を複数曲率で繰り返す。CW/CCWの両方向を含める。既存ログの速度上限約3.1m/sをそのまま全曲率へ適用せず、必要な旋回角速度・接地状態と既存安全制約から実行可能条件を決める。',
  '- 同一条件で複数回繰り返し、電圧・温度・タイヤ・床・走行方式を記録する。係数選定用系列と確認用系列を別にする。',
  '- 可能ならタイムスタンプ付き1msの左右累積Encoder、gyro Z、保存Yaw、加速度XYZ、IMU温度を記録し、全信号の読取時刻とログ蓄積時刻を確認する。現行の距離基準ログだけで1ms波形を復元できたとは扱わない。',
  '- 13031の後半、13033の旋回ピークのような不一致を動画・車輪回転・機体旋回で照合し、滑りか計測側の問題かを判断する。',
  '- ログ形式変更や実機パラメータ変更はこの案では行っていない。実施する場合は対応するAGENTS/Skillのビルド・ログ確認・実機制約に従う。','']
 (O/'実機確認条件.md').write_text('\n'.join(plan),encoding='utf-8')
 sources=['analysis/script/analyze_multispeed_width_13006_13044.py','analysis/script/diagnose_multispeed_width_13006_13044.py','analysis/script/validate_multispeed_width_13006_13044.py','robotrace_v2/Core/Src/IMU.c','robotrace_v2/Core/Src/timer.c','robotrace_v2/Core/Src/encoder.c','robotrace_v2/Core/Src/BMI088.c']
 result=dict(status='passed',logs=31,raw_rows=source_rows,windows=len(windows),metric_rows=len(metrics),prediction_rows=len(pred),input_difference_max_error=max(errors),angular_metric_max_error_dps=max(metric_errors),
  source_hashes_unchanged=True,frozen_parameters_unchanged=True,train_test_series_disjoint=True,CROSS_pose_used=False,raw_LR_mean=True,nonoverlap_checked=True,synthetic_IIR_check=True,
  firmware_modified=False,sources=[dict(path=str(ROOT/p),sha256=sha(ROOT/p)) for p in sources])
 (O/'independent_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
