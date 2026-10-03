"""追加float版のautoStart 5走セットを、直前の完了セットと比較する。"""
from pathlib import Path
import csv, json, hashlib, os
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'analysis/fpu-float-completion/runs-12928-12932'; OUT.mkdir(parents=True,exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
LOG=Path('F:/Dropbox/Document/robotrace/Log/v2')
NUMBERS=list(range(12922,12927))+list(range(12928,12933))
D={};summaries=[]
for n in NUMBERS:
 p=LOG/f'{n}.csv'
 with p.open(encoding='utf-8-sig',newline='') as f:
  reader=csv.reader(f);meta=dict(z.split('=',1) for z in next(reader) if '=' in z);header=next(reader);raw=[r for r in reader if r and any(r)]
 names=[x for x in header if x]
 bad=sum(len(r)!=len(header) for r in raw)
 if bad:raise ValueError(f'{n}: width mismatch {bad}')
 v={key:np.array([float(row[i]) for row in raw]) for i,key in enumerate(header) if key}
 required=['cntlog','encCurrentN','encTotalL','encTotalR','encTotalOptimal','targetSpeed','gyroVal_Z','slipFlag','slipFlagLat','x','y']
 assert all(c in v for c in required)
 nonfinite=sum(np.count_nonzero(~np.isfinite(a)) for a in v.values())
 ppm=float(meta['encoderPulsePerMeter']);scale=ppm/1000
 dt=np.diff(v['cntlog']);elapsed=v['cntlog'][-1]/1000
 # 補正距離でなく、生の左右累積エンコーダ平均で距離基準の欠落を判定する。
 ds=np.diff((v['encTotalL']+v['encTotalR'])/2)/scale
 bound=float(meta['logDistanceTargetMm'])+max(v['encCurrentN'])/scale+2/scale
 gap=np.count_nonzero(ds>bound)
 speed=v['encCurrentN']/scale;target=v['targetSpeed']/scale
 # 全走行の時間重み付き誤差。距離基準の行数平均で高速区間を過大評価しない。
 weights=np.r_[v['cntlog'][0],dt];error=abs(speed-target)
 summary=dict(log=n,commit=meta['gitCommit'],requested=meta['requestedMode'],actual_mode=int(float(meta['optimalTrace'])),level=int(meta['shortcutLevel']),auto_run=int(float(meta['autoStart'])),rows=len(raw),expected_rows=int(meta['logExpectedRows']),emcStop=int(float(meta['emcStop'])),bad_width=bad,nonfinite=int(nonfinite),time_nonpositive=int(np.count_nonzero(dt<=0)),max_gap_ms=int(max(dt)),raw_distance_max_mm=round(float(max(ds)),3),distance_gap_bound_mm=round(bound,3),suspicious_distance_gaps=int(gap),record_end_s=round(elapsed,3),speed_mae_mps=round(float(np.average(error,weights=weights)),4),speed_error_p95_mps=round(float(np.percentile(error,95)),4),peak_speed_mps=round(float(max(speed)),3),slip_time_pct=round(float(np.average(v['slipFlag']!=0,weights=weights)*100),3),lateral_slip_time_pct=round(float(np.average(v['slipFlagLat']!=0,weights=weights)*100),3),battery_header_v=float(meta['batteryVoltage_V']),battery_min_v=round(float(min(v['batteryVoltage_mV']))/1000,3),closure_valid=int(meta['closureValid']),closure_reason=int(meta['closureReason']),goal_x_mm=float(meta['goalMarkerX_mm']),goal_source_valid=int(meta['goalMarkerOnsetValid']),imu_valid=int(meta['imuCalibrationValid']),distance_valid=int(meta['distanceScaleVerified']),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
 summary['valid_run']=summary['emcStop']==0 and summary['rows']==summary['expected_rows'] and not any(summary[k] for k in ['bad_width','nonfinite','time_nonpositive','suspicious_distance_gaps'])
 D[n]=(meta,v,summary);summaries.append(summary)
with (OUT/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.DictWriter(f,summaries[0].keys());w.writeheader();w.writerows(summaries)
comparisons=[]
for a,b in zip(range(12922,12927),range(12928,12933)):
 ma,va,sa=D[a];mb,vb,sb=D[b]
 keys=[k for k in ma if k.startswith('tgtParam.') or k.startswith(('lineTraceCtrl.','lineTraceOmegaFBCtrl.','veloCtrl.','yawRateCtrl.','yawCtrl.','distCtrl.')) or k in ['speedFeedForwardGain','gyroScaleCoeff','imuTempCoeff_dpsPerC','encoderPulsePerMeter','routeControllerVersion','logDistanceTargetMm']]
 differences={k:[ma[k],mb.get(k)] for k in keys if ma[k]!=mb.get(k)}
 same_mode=(sa['requested']==sb['requested'] and sa['actual_mode']==sb['actual_mode'] and sa['level']==sb['level'])
 comparisons.append(dict(baseline=a,candidate=b,mode=sb['requested'],same_mode=same_mode,settings_differences=differences,record_end_difference_s=round(sb['record_end_s']-sa['record_end_s'],3),speed_mae_difference_mps=round(sb['speed_mae_mps']-sa['speed_mae_mps'],4),battery_header_difference_v=round(sb['battery_header_v']-sa['battery_header_v'],2)))
assert all(D[n][2]['valid_run'] for n in range(12928,12933))
primary=D[12928][2];assert primary['closure_valid'] and primary['imu_valid'] and primary['distance_valid']
assert [D[n][2]['auto_run'] for n in range(12928,12933)]==list(range(1,6))
assert all(D[n][0]['primaryLogNumber']=='12928' for n in range(12928,12933))
assert D[12932][0]['slipSourceLogNumber']=='12931'
for chart in ['xy_speed','gyro_slip']:
 fig,axes=plt.subplots(5,2,figsize=(12,15),layout='constrained')
 for i,(a,b) in enumerate(zip(range(12922,12927),range(12928,12933))):
  for n,color in [(a,'#999999'),(b,'#0066bb')]:
   meta,v,s=D[n];t=v['cntlog']/1000;scale=float(meta['encoderPulsePerMeter'])/1000
   if chart=='xy_speed':
    axes[i,0].plot(v['x'],v['y'],color=color,lw=.6,label=str(n));axes[i,0].set_aspect('equal',adjustable='datalim')
    axes[i,1].plot(t,v['encCurrentN']/scale,color=color,lw=.6,label=f'{n} actual')
    if n==b:axes[i,1].plot(t,v['targetSpeed']/scale,color='#dd8833',lw=.6,label='target')
   else:
    axes[i,0].plot(t,v['gyroVal_Z'],color=color,lw=.5,label=str(n))
    if n==b:
     axes[i,1].plot(t,v['slipFlag'],label='longitudinal',lw=.6)
     axes[i,1].plot(t,v['slipFlagLat']+1.5,label='lateral +1.5',lw=.6)
  axes[i,0].set_title(f'{b} {D[b][2]["requested"]} (actual={D[b][2]["actual_mode"]}, level={D[b][2]["level"]})')
  for j in range(2):axes[i,j].grid(alpha=.25);axes[i,j].legend(fontsize=7)
  axes[i,0].set(xlabel='X [mm]' if chart=='xy_speed' else 'time [s]',ylabel='Y [mm]' if chart=='xy_speed' else 'gyro Z [deg/s]')
  axes[i,1].set(xlabel='time [s]',ylabel='speed [m/s]' if chart=='xy_speed' else 'slip flags')
 fig.savefig(OUT/f'{chart}.png',dpi=130);plt.close(fig)
(OUT/'validation.json').write_text(json.dumps(dict(summaries=summaries,comparisons=comparisons,complete_set=True,notes=['Record-end times approximate goal times; goalTime itself is not stored.','No ISR timing available in logs.','SHORTCUT requested but actual PATH level 0 in both sets.','Only one set per firmware; lower battery and different primary source prevent causal attribution.']),ensure_ascii=False,indent=2),encoding='utf-8')
report=['# 追加float化版12928～12932の実走行確認','', '5走セット完了。全ログf3c2b85、emcStop=0、行数一致、非有限値・時刻逆行なし。一次12928の閉路X=42.43mm（±60mm内）、IMU校正・距離換算検証成功。','', '|ログ|要求 / 実モード|最終記録時刻[s]|速度MAE[m/s]|スリップ時間[%] 縦/横|電圧ヘッダ[V]|','|---|---|---:|---:|---:|---:|']
for n in range(12928,12933):
 s=D[n][2];report.append(f'|{n}|{s["requested"]} / {s["actual_mode"]} L{s["level"]}|{s["record_end_s"]:.3f}|{s["speed_mae_mps"]:.4f}|{s["slip_time_pct"]:.3f}/{s["lateral_slip_time_pct"]:.3f}|{s["battery_header_v"]:.2f}|')
report+=['','## 比較','', '比較元12922～12926は第一段階float版a993603。要求方式・実モード・ヘッダ収録ヘッダ収録設定値を比較し、異なるモード間の優劣を判定しない。','', '|比較|最終記録時刻差[s] 追加版－前版|速度MAE差[m/s]|電圧差[V]|設定差|','|---|---:|---:|---:|---|']
for c in comparisons:report.append(f'|{c["baseline"]}→{c["candidate"]}|{c["record_end_difference_s"]:+.3f}|{c["speed_mae_difference_mps"]:+.4f}|{c["battery_header_difference_v"]:+.2f}|{c["settings_differences"] or "なし"}|')
report+=['','最終記録時刻はゴール付近の記録であり、画面goalTimeそのものではない。速度MAEとスリップ割合は時間重み付き、誤差p95はサンプル基準。距離欠落検査は左右累積エンコーダの平均差分が「5mm＋全走行最大1ms移動量＋2pulse」以内か確認した。境界内の欠落をすべて否定するものではない。','', 'SHORTCUT要求12930は実モードPATH（optimalTrace=4）・Level 0。前版12924も同じであり、Level 1短縮経路の確認には使わない。二次走行のclosureValid=0 / closureReason=1は一次経路元の検証対象外であり、緊急停止とは区別する。','', '両版1セットずつで、追加版の電圧は約0.11～0.15V低い。一次経路元と校正結果も異なるため、タイム変化をfloat化の効果と断定できない。新ログにはPATH追従専用列とISR計測値がないので、横偏差・ロスト状態・処理時間は直接評価できない。','', 'UI長押し・描画・設定再読込の目視確認は走行ログだけでは判定できない。追加版の採用・mainへの統合はまだ行わない。','', '成果物: summary.csv、validation.json、xy_speed.png、gyro_slip.png。']
(OUT/'report.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
print(json.dumps(dict(candidate=[D[n][2] for n in range(12928,12933)],comparisons=comparisons),ensure_ascii=False,indent=2))
