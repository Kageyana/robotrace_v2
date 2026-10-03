"""Check 2-second settling + 5-second calibration against preceding 0.5 m/s logs."""
from pathlib import Path
import csv,json,os,hashlib
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/calibration_12857_12862';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import numpy as np
import analyze_yi_12838_12848 as m
m.OUT=OUT
plt=m.plt;LOG=Path('F:/Dropbox/Document/robotrace/Log/v2');Q=np.linspace(0,1,1001)
GROUPS={'CW_before':[12854,12855,12856],'CCW_before':[12850,12851,12852],
        'CW_after':[12857,12858,12859],'CCW_after':[12860,12861,12862]}
health=[];intervals=[];profiles=[];P={}
def write(name,rows):
 with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
  w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
def sample(s,z,q):
 return np.interp(q,s,z) if z.ndim==1 else np.column_stack([np.interp(q,s,z[:,j]) for j in range(z.shape[1])])
def rotate(z,h):
 c,s=np.cos(h),np.sin(h);return z@np.array([[c,-s],[s,c]])
for group,nums in GROUPS.items():
 for n in nums:
  path=LOG/f'{n}.csv'
  with path.open(encoding='utf-8-sig') as f:
   meta=dict(x.split('=',1) for x in next(f).strip().split(',') if '=' in x);rows=list(csv.DictReader(f))
  a=np.array([[float(r[c]) for c in m.COLS] for r in rows]);xy=np.array([[float(r[c]) for c in ['x','y']] for r in rows])
  ds=np.r_[0,(np.diff(a[:,1])+np.diff(a[:,2]))/116.038];s=np.cumsum(ds);dt=np.r_[0,np.diff(a[:,0])]/1000
  d=dict(a=a,s=s,meta=meta);m.D[n]=d;d['line'],_=m.anchor(d);assert len(d['line'])==6,n
  bounds=m.axleanchors(d);assert 0<bounds[0]<bounds[-1]<s[-1]
  assert len(a)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0 and float(meta['optimalTrace'])==0
  assert np.all(np.isfinite(a)) and np.all(np.diff(a[:,0])>0) and np.all(np.diff(s)>0)
  assert meta['imuCalibrationValid']=='1' and meta['imuCalibrationSamples']=='100' and meta['distanceScaleVerified']=='1'
  target=np.array([float(r['targetSpeed'])/58.019 for r in rows]);assert np.all(np.abs(target-.5)<.001),n
  assert (np.sign(np.mean(a[:,3]))==1)==group.startswith('CW_')
  active=(s>bounds[0])&(s<bounds[-1]);mark=a[:,4];ci=np.flatnonzero((mark==3)&(np.r_[0,mark[:-1]]!=3))
  assert np.diff(a[:,0]).max()<=11,n
  health.append(dict(log=n,group=group,rows=len(a),max_gap_ms=np.diff(a[:,0]).max(),target_mps=np.median(target),
   actual_mean_mps=ds[active].sum()/dt[active].sum()/1000,battery_V=meta['batteryVoltage_V'],buildDate=meta['buildDate'],buildTime=meta['buildTime'],
   gyro_offset_dps=meta['imuGyroOffsetZ_dps'],temp_start_C=meta['imuTempCalibrationStart_C'],temp_mean_C=meta['imuTempCalibration_C'],
   temp_cal_end_C=meta['imuTempCalibrationEnd_C'],temp_run_end_C=meta['imuTempEnd_C'],cal_samples=meta['imuCalibrationSamples'],
   cal_errors=meta['imuCalibrationReadErrors'],temp_samples=meta['imuTempCalibrationSamples'],temp_errors=meta['imuTempCalibrationReadErrors'],
   courseMarker_cross=len(ci),independent_cross=6,slip_rows=np.count_nonzero(a[:,5]),slip_lat_rows=np.count_nonzero(a[:,6]),
   sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
  for method in ['logged','right','trapezoid']:
   w=a[:,3].copy()
   if method=='trapezoid':w[1:]=.5*(w[1:]+w[:-1])
   dh=np.deg2rad(w)*dt;h=np.cumsum(dh);mid=h-.5*dh
   p=xy if method=='logged' else np.c_[np.cumsum(ds*np.sin(mid)),np.cumsum(ds*np.cos(mid))]
   pa=sample(s,p,bounds);ha=sample(s,h,bounds)
   for k in range(5):
    query=bounds[k]+Q*(bounds[k+1]-bounds[k]);pp=sample(s,p,query);hh=sample(s,h,query)
    P[n,method,k]=dict(p=pp,h=hh,local=rotate(pp-pp[0],-hh[0]),omega=sample(s,a[:,3],query))
    delta=pa[k+1]-pa[k]
    intervals.append(dict(log=n,group=group,method=method,interval=k+1,delta_X_mm=delta[0],delta_Y_mm=delta[1],
     heading_error_deg=np.rad2deg(ha[k+1]-ha[k])-(360 if group.startswith('CW_') else -360),
     lap_time_s=(np.interp(bounds[k+1],s,a[:,0])-np.interp(bounds[k],s,a[:,0]))/1000,lap_distance_mm=bounds[k+1]-bounds[k]))
  for k in range(1,5):
   dif=P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local']
   for i,q in enumerate(Q):profiles.append(dict(log=n,group=group,interval=k+1,q=q,shape_difference_mm=np.linalg.norm(dif[i])))
write('health.csv',health);write('lap_intervals.csv',intervals);write('shape_profiles.csv',profiles)
summary=[];per_log=[];onsets=[]
for group,nums in GROUPS.items():
 for method in ['logged','right','trapezoid']:
  rr=[r for r in intervals if r['group']==group and r['method']==method]
  summary.append(dict(group=group,method=method,logs=3,mean_X_mm=np.mean([r['delta_X_mm'] for r in rr]),
   mean_Y_mm=np.mean([r['delta_Y_mm'] for r in rr]),mean_heading_error_deg=np.mean([r['heading_error_deg'] for r in rr])))
  for n in nums:
   zz=[r for r in rr if r['log']==n]
   per_log.append(dict(log=n,group=group,method=method,mean_X_mm=np.mean([r['delta_X_mm'] for r in zz]),mean_Y_mm=np.mean([r['delta_Y_mm'] for r in zz]),
    mean_heading_error_deg=np.mean([r['heading_error_deg'] for r in zz])))
 norm=np.array([np.linalg.norm(P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'],axis=1) for n in nums for k in range(1,5)])
 mean=norm.mean(0)
 for threshold in [2,5]:
  rr=[(l,r) for l,r in m.runs(mean>threshold) if r-l>=20]
  onsets.append(dict(group=group,threshold_mm=threshold,onset_pct=Q[rr[0][0]]*100 if rr else '',end_shape_mm=mean[-1],max_shape_mm=mean.max()))
write('summary.csv',summary);write('per_log.csv',per_log);write('onsets.csv',onsets)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.grid':True,'grid.alpha':.2})
def save(fig,name):fig.savefig(OUT/f'{name}.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for ax,direction in zip(axs,['CW','CCW']):
 for suffix,color in [('before','#245b87'),('after','#be681e')]:
  group=direction+'_'+suffix;nums=GROUPS[group]
  norm=np.array([np.linalg.norm(P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'],axis=1) for n in nums for k in range(1,5)])
  ax.plot(Q*100,norm.mean(0),c=color,label=suffix);ax.fill_between(Q*100,np.percentile(norm,10,axis=0),np.percentile(norm,90,axis=0),color=color,alpha=.15)
 ax.axhline(2,c='.5',ls=':');ax.set(title=direction,xlabel='Phase since independent CROSS [%]',ylabel='Shape difference [mm]');ax.legend()
save(fig,'calibration_shape_comparison')
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,group in zip(axs.flat,GROUPS):
 for n in GROUPS[group]:
  for k in range(5):
   p=P[n,'trapezoid',k]['p'];ax.plot(p[:,0],p[:,1],lw=.7,alpha=.75)
 ax.set_aspect('equal');ax.set(title=f'{group}: all 3 logs, 5 intervals each',xlabel='X [mm]',ylabel='Y [mm]')
save(fig,'calibration_trajectories')
(OUT/'validation.json').write_text(json.dumps(dict(logs=12,health='PASS: normal end, expected row counts, finite values, monotonic time/distance, <=11 ms gaps',
 target='29 pulse/ms in every log',calibration='100 valid gyro/accelerometer samples; 20 temperature samples',
 external_ground_truth=False),indent=2),encoding='utf-8')
tbl=[]
for direction in ['CW','CCW']:
 for suffix in ['before','after']:
  group=direction+'_'+suffix
  log=next(r for r in summary if r['group']==group and r['method']=='logged')
  raw=next(r for r in summary if r['group']==group and r['method']=='trapezoid')
  tbl.append(f"| {direction} | {'変更前' if suffix=='before' else '変更後'} | {log['mean_X_mm']:+.1f} | {raw['mean_X_mm']:+.1f} | {raw['mean_heading_error_deg']:+.3f} |")
individual=[]
for n in range(12857,12863):
 log=next(r for r in per_log if r['log']==n and r['method']=='logged')
 raw=next(r for r in per_log if r['log']==n and r['method']=='trapezoid');he=next(r for r in health if r['log']==n)
 individual.append(f"| {n} | {'CW' if n<12860 else 'CCW'} | {log['mean_X_mm']:+.1f} | {raw['mean_X_mm']:+.1f} | {raw['mean_heading_error_deg']:+.3f} | {he['battery_V']} | {he['temp_mean_C']} → {he['temp_run_end_C']} |")
report='''# 走行前IMU校正の待機・取得時間変更 — 12857～12862

2026-10-03。CW12857～12859、CCW12860～12862。比較元は同じ0.5 m/s設定のCW12854～12856、CCW12850～12852。ファームウェアの変更・書き込みはこの解析では行っていない。

## 校正条件

ユーザー申告: カウントダウン7秒、残り5秒で校正開始、5秒間で100サンプル。押下後の約2秒を振動収束の待機に充てる。

現行コードもcontrol.cのcountdown=7000、countdown==5000でIMU_StartCalibration、IMU.hの100サンプル/50 msに一致する。温度は5有効IMUサンプルごとに取得するので、正常時は250 ms間隔で20回。校正終了とcountdown=0等の条件を満たすまで走行を開始しない。コメントには旧3秒/20 msの記載が残るが、今回ユーザーのコードは変更していない。

新規6ログは全てIMU校正100回・エラー0、温度20回・エラー0、校正有効。実際の取得開始時刻・間隔・押下時の静止振動は走行ログに記録されていないため、ログだけから5秒の取得時間や振動収束を実測確認したわけではない。コード設定・申告・ログのサンプル数の整合を確認した。

## Xドリフトの結果

**待機・校正時間の変更後もX+方向のドリフトは残った。CWはばらつきが大きく、CCWは3本とも同程度のずれを残す。押下振動が主因だったとは確認できない。**

| 方向 | 条件 | CSV X [mm/周] | 生エンコーダ＋台形gyro X [mm/周] | 一周±360°との差 [deg/周] |
| --- | --- | ---: | ---: | ---: |
'''+ '\n'.join(tbl)+'''

各群3ログ。1周は独立に検出したCROSSから次のCROSSまでで、各ログ5完全区間を平均。CW平均は12859の小さなずれを含むので個別値も必要。実際の同じコース地点を基準とし、推定座標の平行移動を残して計算する。

| ログ | 方向 | CSV X [mm/周] | 生積分 X [mm/周] | 一周角度差 [deg/周] | 電圧 [V] | 校正平均温度 → 終了温度 [°C] |
| --- | --- | ---: | ---: | ---: | ---: | --- |
'''+ '\n'.join(individual)+'''

12859の生積分Xは約+0.66 mm/周と小さい一方、12857/12858では+15.9/+20.1 mm/周。12859だけを校正変更の効果と扱わない。12858は一周総角度差が約-0.029°でもX+20.1 mm/周残り、一周総角度差だけではXドリフトを説明できない。途中の角度誤差分布や横方向の変位欠落も調べる必要がある。

## 周回途中の形状差

各周の開始位置と開始推定角度をそろえ、最初の完全CROSS間区間と後続4区間を正規化距離1001点で比較。XY差の大きさの平均が2 mmを超え、周回の2%以上続く最初の位置を検出点とする。これは絶対誤差の発生開始点・横滑り開始点ではない。初周から毎周同じ誤差は完全には捉えられない。

- CW: 2 mm検出22.9%→37.6%、5 mm検出72.1%→72.4%。前半の周回間差は小さくなるが、終盤の差は残る。
- CCW: 2 mm検出36.7%→29.0%、5 mm検出49.7%→53.6%。60～85%付近の最大形状差は約13.9→10.2 mmとなったが、Xの平行移動ドリフトは残る。

割合は車軸中点のCROSS通過を0%、次のCROSSを100%とする。帯は比較区間の10～90%分布で、独立試行の信頼区間ではない。

![校正変更前後の周回間形状差](calibration_shape_comparison.png)

![校正変更前後の生データ再積分軌跡](calibration_trajectories.png)

## 条件差と確認

全12ログで目標29 pulse/ms（約0.5 m/s）。新規の生距離/時間平均速度はCW約0.540 m/s、CCW約0.537 m/sで、前回とほぼ同程度。校正変更後の電圧はCW8.31→8.22 V、CCW8.18→8.08 V。前回CW7.79→7.73 V、CCW7.92→7.83 Vから充電されているため、速度だけでなく電圧条件も違う。温度変化量も異なる。

12857は校正平均25.375°C→走行終了31.125°Cで約+5.75°Cの昇温。12860も約+5.09°C。12859は約+0.41°Cだが12858も約+1.37°Cでドリフトが大きいため、温度安定だけを原因と断定しない。温度補正は有効だが、それで残留オフセットがゼロになることはログから保証できない。静止校正の標準偏差や各サンプルは保存されていないので、校正ノイズを直接比較することもできない。

現行のmarkerSensor.hは左右GPIOを反転したCCW設定、control.hはCOUNT_GOAL=7。過去の各ビルド時の作業ツリー全体はログに残っていない。方向はユーザー申告を優先し、gyro積算符号で一致確認。前回と同じcommit表示でもbuildDate/Timeは異なるため、同一バイナリとは扱わない。

生積算距離/gyro再積分にCSV x/y・ROC・encTotalOptimalは使わず、CSV座標は独立した診断指標として比較する。gyroスケール係数の二重適用なし、追加yI補正0。

全12ログで正常終了、期待行数一致、時刻/生距離単調、値有限、最大ログ間隔11 ms（5 mm基準・0.5 m/sと整合）。独立ラインCROSSは各6回。12857のcourseMarker CROSSは5回のため、前回同様にライン全幅反応と素子幾何から周回境界を求めた。縦横slipFlagは全行0であり、物理的滑り不存在の証明ではない。

## 現時点の判断

静止待機を設ける変更の効果を、この各方向3ログだけで採用・不採用とは判定しない。Xドリフトは校正待機の延長だけで解消していない。次の重点は12858と12859の同一コース区間で、gyro角度推移・左右エンコーダ差・補正済み加速度を比較し、Xの平行移動差がどの区間で生まれるかを調べること。新しい補正値は採用していない。

解析: analysis/script/compare_calibration_12857_12862.py。前回のanalyze_yi_12838_12848.pyとsensor_centers.csvを再利用。health.csvに入力SHA-256、ビルド日時、速度、電圧、温度、校正サンプル数を保存。コードを変更していないためビルド・実機書き込みは実施していない。
'''
(OUT/'report.md').write_text(report,encoding='utf-8')
print(json.dumps(dict(summary=summary,new_logs=[r for r in per_log if r['log']>=12857],onsets=onsets),indent=2))
