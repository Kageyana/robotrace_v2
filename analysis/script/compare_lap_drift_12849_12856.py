"""Compare low-speed laps with preceding runs using independently detected line CROSS.

Missing CSV 12853 is reported. 12849 is measured/commanded 1 m/s, not 0.5.
Incomplete courseMarker CROSS counts are diagnostic, never used as lap bounds.
"""
from pathlib import Path
import csv,json,os,hashlib
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'analysis/lap_drift_12849_12856';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import numpy as np
import analyze_yi_12838_12848 as m
m.OUT=OUT
plt=m.plt
LOG=Path('F:/Dropbox/Document/robotrace/Log/v2')
NUMS=list(range(12838,12843))+list(range(12844,12857))
Q=np.linspace(0,1,1001)
health=[];intervals=[];profile=[];P={};missing=[]

def write(name,rows):
 with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
  w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)

def sample(s,z,q):
 return np.interp(q,s,z) if z.ndim==1 else np.column_stack([np.interp(q,s,z[:,j]) for j in range(z.shape[1])])

def rotate(z,h):
 c,s=np.cos(h),np.sin(h);return z@np.array([[c,-s],[s,c]])

for n in NUMS:
 path=LOG/f'{n}.csv'
 if not path.exists():missing.append(n);continue
 with path.open(encoding='utf-8-sig') as f:
  meta=dict(x.split('=',1) for x in next(f).strip().split(',') if '=' in x);rows=list(csv.DictReader(f))
 a=np.array([[float(r[c]) for c in m.COLS] for r in rows])
 speed=np.array([float(r['encCurrentN'])/58.019 for r in rows]);target=np.array([float(r['targetSpeed'])/58.019 for r in rows])
 xy=np.array([[float(r[c]) for c in ['x','y']] for r in rows])
 ds=np.r_[0,(np.diff(a[:,1])+np.diff(a[:,2]))/116.038];s=np.cumsum(ds)
 d=dict(a=a,s=s,meta=meta);m.D[n]=d;d['line'],detail=m.anchor(d)
 assert len(d['line'])==6,(n,len(d['line']))
 bounds=m.axleanchors(d)
 assert bounds[0]>0 and bounds[-1]<s[-1] and np.all(np.diff(bounds)>0)
 assert np.all(np.isfinite(a)) and np.all(np.diff(a[:,0])>0) and np.all(np.diff(s)>0)
 assert len(a)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0 and float(meta['optimalTrace'])==0
 direction='CW' if n<=12842 or n>=12853 else 'CCW'
 assert (np.sign(np.mean(a[:,3]))==1)==(direction=='CW')
 group=('CW1.0' if direction=='CW' else 'CCW1.0') if np.median(target)>.75 else ('CW0.5' if direction=='CW' else 'CCW0.5')
 marker=a[:,4];ci=np.flatnonzero((marker==3)&(np.r_[0,marker[:-1]]!=3))
 dt=np.r_[0,np.diff(a[:,0])]/1000;whole_dt=np.diff(a[:,0]);spacing=np.diff(s)
 assert np.max(whole_dt)<=np.ceil(5/np.median(speed))+2,(n,np.max(whole_dt))
 active=(s>bounds[0])&(s<bounds[-1])
 health.append(dict(log=n,group=group,rows=len(a),expected_rows=meta['logExpectedRows'],emcStop=meta['emcStop'],
  battery_V=meta['batteryVoltage_V'],git=meta['gitCommit'],target_mps=np.median(target),speed_median_mps=np.median(speed[active]),
  speed_time_mean_mps=np.sum(ds[active])/np.sum(dt[active])/1000,speed_error_rms_mps=np.sqrt(np.mean((speed[active]-target[active])**2)),
  max_gap_ms=whole_dt.max(),mean_row_distance_mm=spacing.mean(),max_row_distance_mm=spacing.max(),
  logged_cross=len(ci),independent_line_cross=6,slip_rows=np.count_nonzero(a[:,5]),lateral_slip_rows=np.count_nonzero(a[:,6]),
  calibration=meta['imuCalibrationValid'],distance_verified=meta['distanceScaleVerified'],gyro_temp_start_C=meta['imuTempCalibration_C'],
  gyro_temp_end_C=meta['imuTempEnd_C'],sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
 for method in ['logged','right','trapezoid']:
  w=a[:,3].copy()
  if method=='trapezoid':w[1:]=.5*(w[1:]+w[:-1])
  dh=np.deg2rad(w)*dt;h=np.cumsum(dh);mid=h-.5*dh
  p=xy if method=='logged' else np.c_[np.cumsum(ds*np.sin(mid)),np.cumsum(ds*np.cos(mid))]
  pa=sample(s,p,bounds);ha=sample(s,h,bounds)
  for k in range(5):
   query=bounds[k]+Q*(bounds[k+1]-bounds[k]);pp=sample(s,p,query);hh=sample(s,h,query)
   P[n,method,k]=dict(p=pp,h=hh,local=rotate(pp-pp[0],-hh[0]),omega=sample(s,a[:,3],query),length=bounds[k+1]-bounds[k])
   delta=pa[k+1]-pa[k];he=np.rad2deg(ha[k+1]-ha[k])-(360 if direction=='CW' else -360)
   intervals.append(dict(log=n,group=group,method=method,interval=k+1,delta_X_mm=delta[0],delta_Y_mm=delta[1],heading_error_deg=he,
                         lap_distance_mm=bounds[k+1]-bounds[k],lap_time_s=(np.interp(bounds[k+1],s,a[:,0])-np.interp(bounds[k],s,a[:,0]))/1000))
 for k in range(1,5):
  local=P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local']
  for i,q in enumerate(Q):profile.append(dict(log=n,group=group,interval=k+1,q=q,shape_difference_mm=np.linalg.norm(local[i]),
   local_dX_mm=local[i,0],local_dY_mm=local[i,1]))
write('health.csv',health);write('lap_intervals.csv',intervals);write('shape_profiles.csv',profile)
summary=[];bylog=[];onsets=[]
for r in health:
 n=r['log']
 for method in ['logged','right','trapezoid']:
  zz=[z for z in intervals if z['log']==n and z['method']==method]
  bylog.append(dict(log=n,group=r['group'],method=method,mean_dX_mm=np.mean([z['delta_X_mm'] for z in zz]),
                   mean_dY_mm=np.mean([z['delta_Y_mm'] for z in zz]),mean_heading_error_deg=np.mean([z['heading_error_deg'] for z in zz])))
for group in ['CW1.0','CCW1.0','CW0.5','CCW0.5']:
 ns=[r['log'] for r in health if r['group']==group]
 for method in ['logged','right','trapezoid']:
  zz=[z for z in bylog if z['group']==group and z['method']==method]
  summary.append(dict(group=group,method=method,logs=len(ns),mean_dX_mm=np.mean([z['mean_dX_mm'] for z in zz]),
   min_log_dX_mm=min(z['mean_dX_mm'] for z in zz),max_log_dX_mm=max(z['mean_dX_mm'] for z in zz),
   mean_dY_mm=np.mean([z['mean_dY_mm'] for z in zz]),mean_heading_error_deg=np.mean([z['mean_heading_error_deg'] for z in zz])))
 mean=np.mean([np.linalg.norm(P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'],axis=1) for n in ns for k in range(1,5)],axis=0)
 for threshold in [2,5]:
  rr=[(l,r) for l,r in m.runs(mean>threshold) if r-l>=20]
  onsets.append(dict(group=group,threshold_mm=threshold,onset_pct=Q[rr[0][0]]*100 if rr else '',end_shape_mm=mean[-1],max_shape_mm=mean.max()))
write('summary.csv',summary);write('per_log.csv',bylog);write('onsets.csv',onsets)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.grid':True,'grid.alpha':.2})
def save(fig,name):fig.savefig(OUT/f'{name}.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained')
for ax,(group,n) in zip(axs.flat,[('CW1.0',12838),('CCW1.0',12849),('CW0.5',12854),('CCW0.5',12850)]):
 for k in range(5):
  p=P[n,'trapezoid',k]['p'];ax.plot(p[:,0],p[:,1],label=f'interval {k+1}')
 ax.set(title=f'{group}: {n}, raw encoder + trapezoid gyro',xlabel='X [mm]',ylabel='Y [mm]');ax.set_aspect('equal');ax.legend(fontsize=8)
save(fig,'low_speed_trajectories')
fig,axs=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for ax,direction in zip(axs,['CW','CCW']):
 for group,color in [(direction+'1.0','#245b87'),(direction+'0.5','#be681e')]:
  ns=[r['log'] for r in health if r['group']==group]
  dd=np.array([np.linalg.norm(P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'],axis=1) for n in ns for k in range(1,5)])
  ax.plot(Q*100,dd.mean(0),c=color,label=group);ax.fill_between(Q*100,np.percentile(dd,10,axis=0),np.percentile(dd,90,axis=0),color=color,alpha=.15)
 ax.axhline(2,c='.5',ls=':');ax.set(title=direction,xlabel='Phase since independently detected CROSS [%]',ylabel='Shape difference [mm]');ax.legend()
save(fig,'speed_shape_comparison')
fig,axs=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for ax,direction in zip(axs,['CW','CCW']):
 for method,color,offset in [('logged','#245b87',-.12),('trapezoid','#be681e',.12)]:
  for r in bylog:
   if r['group'].startswith(direction) and r['method']==method:
    ax.scatter((0 if r['group'].endswith('1.0') else 1)+offset,r['mean_dX_mm'],c=color,s=30)
  ax.plot([],[],marker='o',c=color,ls='',label=method)
 ax.axhline(0,c='.5',ls=':');ax.set(xticks=[0,1],xticklabels=['1.0 m/s','0.5 m/s'],ylabel='Mean X shift per lap [mm]',title=direction);ax.legend()
save(fig,'speed_drift_comparison')
validation=dict(logs=len(health),missing=missing,independent_crosses='six per log; five complete intervals',
                health='PASS: finite raw values, exact row counts, monotonic time/distance, normal end',
                time_gaps='6 ms at 1 m/s and 11 ms at 0.5 m/s are consistent with distance-based 5 mm logging',
                conditions='12849 targetSpeed=58, 12850-52/54-56 targetSpeed=29; target = pulse/ms /58.019',
                no_slip_proof='No external lateral velocity/ground truth. Shape onset is a repeatability threshold, not absolute error onset.')
(OUT/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
report_rows=[]
for group in ['CW1.0','CW0.5','CCW1.0','CCW0.5']:
 logged=next(r for r in summary if r['group']==group and r['method']=='logged')
 raw=next(r for r in summary if r['group']==group and r['method']=='trapezoid')
 onset=next(r for r in onsets if r['group']==group and r['threshold_mm']==2)
 report_rows.append(f"| {group} | {logged['logs']} | {logged['mean_dX_mm']:+.1f} | {raw['mean_dX_mm']:+.1f} | {onset['onset_pct']:.1f}% |")
new_rows=[]
for r in health:
 if r['log']<12849:continue
 logged=next(z for z in bylog if z['log']==r['log'] and z['method']=='logged')
 raw=next(z for z in bylog if z['log']==r['log'] and z['method']=='trapezoid')
 new_rows.append(f"| {r['log']} | {r['group']} | {r['speed_time_mean_mps']:.3f} | {r['battery_V']} | {logged['mean_dX_mm']:+.1f} | {raw['mean_dX_mm']:+.1f} | {raw['mean_heading_error_deg']:+.3f} | {r['logged_cross']}/6 |")
report='''# 0.5 m/s走行と周回ドリフトの比較 — 12849～12856

2026-10-03。新規ログ7本を確認し、前回ログ10本と比較。ファームウェア変更・書き込みは行っていない。

## 条件の確認

- 12849～12852はCCW。12854～12856はCWで、gyro積算角度の符号とも一致。
- **12849はtargetSpeed=58 pulse/ms（約1.0 m/s）のまま**。0.5 m/s群には入れない。12850～12852、12854～12856はtargetSpeed=29（約0.5 m/s）。
- 12853.csvはF:/Dropbox/Document/robotrace/Log内に見つからないため未解析。欠番理由は不明。
- 実際の生エンコーダ距離/時間による平均速度は、0.5設定で0.533～0.541 m/s、1.0設定で約1.02～1.03 m/s。距離換算は58.019 pulse/mm。
- 新規の電圧は7.97→7.73 V。比較基準のCWは8.35～8.23 V、CCWは8.17～8.06 V（追加12849は7.97 V）。速度だけが異なる対照実験ではない。速度による因果や補正値採用の確定には、充電して同じ電圧帯での再走行が必要。

## 結果

**0.5 m/sへ下げてもX+方向のずれは解消せず、CWはほぼ同程度、CCWは増加した。** CSV x/yの表示上の問題だけではなく、生左右エンコーダ積算値とgyroから再積分しても残る。

| 方向・設定速度 | ログ数 | CSV座標のX移動 [mm/周] | 生データ台形積分 [mm/周] | 周回間形状差2 mmの検出位置 |
| --- | ---: | ---: | ---: | ---: |
'''+ '\n'.join(report_rows)+'''

1.0群: CW12838～12842、CCW12844～12849。前回CCW12844～12848だけの平均はCSV+7.8、生積分+5.8 mm/周で、12849を追加すると+8.0、+6.0。

「周」は独立に検出した同じCROSSから次のCROSSまで。各ログ5つの完全周回区間を使用。形状差の検出位置は各周の開始位置と開始推定角度をそろえ、最初の完全区間と後続4区間のXY差の大きさを平均したもの。平均差が2 mmを超えて2%以上の距離にわたり続く最初の点。CROSS通過を0%、次のCROSSを100%とする。**絶対位置誤差や横滑りがそこから始まったという意味ではない。** 毎周同じ誤差はこの形状比較だけでは見えない。

![速度別のXドリフト](speed_drift_comparison.png)

点は1ログ内5区間の平均。各周と同じ基準区間を共有する比較は独立試行ではない。

## 新規ログの個別値

| ログ | 方向・設定 | 生距離/時間の速度 [m/s] | 電圧 [V] | CSV X [mm/周] | 生積分 X [mm/周] | 一周±360°との差 [deg/周] | courseMarker/独立検出CROSS数 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
'''+ '\n'.join(new_rows)+'''

12850/12851では一周角度差が約+1°だが、12852では約-0.115°でもX+17.3 mm/周のずれが残る。CCW低速のずれを一周総角度の差だけで説明できたとは言えない。途中の角度誤差分布も必要。

## 差が増える区間

CWの2 mm検出位置は1.0設定23.5%、0.5設定22.9%とほぼ同じ。0.5群の平均一周距離を基準にするとCROSS後約1.08 mで、最初の広いカーブ区間。5 mm検出は59.4%→72.1%となり、中盤の形状差は多少小さいが、最終的なXドリフトは減っていない。

CCWの2 mm検出位置は25.2%→36.7%へ後ろに移った（0.5群の距離で約1.74 m）。前回の急旋回付近での形状差は低速で小さくなるが、その後に増加し、5 mm検出位置は49.8%→49.7%とほぼ同じ。60～85%では低速でも差が増える。この直線側での拡大は前の区間の角度誤差の伝播でも起こるので、その場所で滑ったとの断定はしない。

![速度別の周回間形状差](speed_shape_comparison.png)

帯は周回比較の10～90%分布で、信頼区間ではない。

![代表ログの5完全周回区間](low_speed_trajectories.png)

## CROSS不足とログ健全性

12854/12855/12856のcourseMarker=3立ち上がりは3/5/4回。一方、ラインセンサー8ch以上の全幅反応は全ログ6回あり、通常の1～2chのライン追従と分離できた。GPIOやCROSS確定の挙動差か、短いイベントのログ標本化かは未確定。記録されていないcourseMarkerイベントをあったことにして補完していない。

今回はcourseMarkerを周回の境界に使わず、独立したライン全幅反応と外側8chの個別進入/退出から車軸中点の通過位置を推定した。前回のsensor_centers.csvと同じ幾何補正。低速時の光学応答が同一という仮定は残る。

全17ログで期待行数一致、emcStop=0、IMU校正有効、距離換算検証有効、時刻/生距離単調、値有限。行間距離平均約5 mm。1.0設定最大6 ms、0.5設定最大11 msは速度と距離基準ログに整合し、時間差だけで欠落とは判定しない。縦横slipFlagは全行0で、物理的な滑り不存在の証明ではない。

## 解釈と次の調査

速度を下げたとき、CROSS直後からの周回間形状差が一部小さくなる効果はあるが、Xドリフトは残り、CCWでは増えた。したがって「速度依存の横滑りが主因で、低速化すれば解消する」という説明は今回の結果では支持されない。固定車輪の旋回に伴う横方向変位の欠落、gyro残留オフセット/温度、時間に依存する積分誤差などを分けて調べる必要がある。これらは仮説であり、原因を確定してはいない。

今回のraw台形積分では、時間増加に伴う一周角度の偏りもあるが、12852の例のように一周総角度だけでは説明できない。次は同一コース区間のgyro角度差、左右エンコーダ差、補正済み加速度を速度別に比較する。補正値やgyro係数は採用していない。

解析スクリプト: analysis/script/compare_lap_drift_12849_12856.py。入力CSVのSHA-256はhealth.csvに保存。前回のanalyze_yi_12838_12848.pyとsensor_centers.csvを再利用する。生積分ではCSV x/y、ROC、encTotalOptimalを使わず、x/yは別の診断指標として扱う。gyroScaleCoeffを二重適用しない。yI=0で比較。右端積分でも低速CCW+19.4、CW+10.3 mm/周で結論は同じ。
'''
(OUT/'report.md').write_text(report,encoding='utf-8')
print(json.dumps(dict(summary=summary,bylog=bylog,onsets=onsets),indent=2))
