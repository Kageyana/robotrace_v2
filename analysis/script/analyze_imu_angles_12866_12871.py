"""Validate added IMU columns and use firmware 1-ms yaw for lap drift diagnostics."""
from pathlib import Path
import csv,json,os,hashlib
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/imu_angles_12866_12871';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import numpy as np
import analyze_yi_12838_12848 as m
m.OUT=OUT;plt=m.plt;LOG=Path('F:/Dropbox/Document/robotrace/Log/v2');Q=np.linspace(0,1,1001)
NEW=['imuTemp_C','gyroVal_X','gyroVal_Y','imuAngle_X','imuAngle_Y','imuAngle_Z']
health=[];intervals=[];onsets=[];profiles=[];P={};D={};sector=[]
def sample(s,z,q):
 return np.interp(q,s,z) if z.ndim==1 else np.column_stack([np.interp(q,s,z[:,j]) for j in range(z.shape[1])])
def rotate(z,h):
 c,s=np.cos(h),np.sin(h);return z@np.array([[c,-s],[s,c]])
def write(name,rr):
 with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
  w=csv.DictWriter(f,rr[0].keys());w.writeheader();w.writerows(rr)
for n in range(12866,12872):
 path=LOG/f'{n}.csv';lines=path.read_text(encoding='utf-8-sig').splitlines();meta=dict(z.split('=',1) for z in lines[0].split(',') if '=' in z)
 header=next(csv.reader([lines[1]]));raw=list(csv.reader(lines[2:]));assert len(header)==42 and header[-1]=='' and header[-7:-1]==NEW
 assert all(len(r)==len(header) and r[-1]=='' for r in raw)
 rr=list(csv.DictReader(lines[1:]));a=np.array([[float(r[c]) for c in m.COLS] for r in rr]);diag=np.array([[float(r[c]) for c in NEW] for r in rr]);xy=np.array([[float(r[c]) for c in ['x','y']] for r in rr])
 assert len(a)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0 and float(meta['optimalTrace'])==0
 assert np.all(np.isfinite(a)) and np.all(np.isfinite(diag));assert np.all(np.diff(a[:,0])>0) and np.max(np.diff(a[:,0]))<=11
 ds=np.r_[0,(np.diff(a[:,1])+np.diff(a[:,2]))/116.038];s=np.cumsum(ds);assert np.all(np.diff(s)>0)
 d=dict(a=a,s=s,meta=meta);m.D[n]=d;d['line'],_=m.anchor(d);assert len(d['line'])==6
 bounds=m.axleanchors(d);assert 0<bounds[0]<bounds[-1]<s[-1]
 direction='CW' if n<=12868 else 'CCW';assert (np.sign(diag[-1,5]-diag[0,5])==1)==(direction=='CW')
 dt=np.r_[0,np.diff(a[:,0])]/1000;h_true=np.deg2rad(diag[:,5]);h_trap=np.cumsum(np.deg2rad(np.r_[a[0,3],.5*(a[1:,3]+a[:-1,3])])*dt)
 # Sparse integration is anchored to the observed first firmware yaw, not forced to zero.
 h_trap+=h_true[0];h_right=np.cumsum(np.deg2rad(a[:,3])*dt)+h_true[0]
 active=(s>bounds[0])&(s<bounds[-1]);target=np.array([float(r['targetSpeed'])/58.019 for r in rr]);assert np.all(np.abs(target-.5)<.001)
 assert meta['imuCalibrationValid']=='1' and meta['imuCalibrationReadErrors']=='0' and meta['imuCalibrationSamples']=='100'
 temps=diag[:,0];assert np.all(temps!=-999)
 health.append(dict(log=n,direction=direction,rows=len(a),columns=41,max_line_bytes=max(len(l.encode('utf-8'))+2 for l in lines[2:]),max_gap_ms=np.diff(a[:,0]).max(),
  actual_mean_mps=ds[active].sum()/dt[active].sum()/1000,battery_V=meta['batteryVoltage_V'],temp_cal_C=meta['imuTempCalibration_C'],
  temp_min_C=temps.min(),temp_max_C=temps.max(),temp_end_C=temps[-1],temp_changes=np.count_nonzero(np.diff(temps)),
  temp_max_lattice_error=np.max(np.abs(temps*8-np.round(temps*8))),
  gyro_X_rms_dps=np.sqrt(np.mean(diag[active,1]**2)),gyro_Y_rms_dps=np.sqrt(np.mean(diag[active,2]**2)),
  angle_X_min_deg=diag[:,3].min(),angle_X_max_deg=diag[:,3].max(),angle_Y_min_deg=diag[:,4].min(),angle_Y_max_deg=diag[:,4].max(),
  yaw_end_deg=diag[-1,5],sparse_trapezoid_yaw_end_error_deg=np.rad2deg(h_trap[-1]-h_true[-1]),
  sparse_right_yaw_end_error_deg=np.rad2deg(h_right[-1]-h_true[-1]),
  slip_rows=np.count_nonzero(a[:,5]),slip_lat_rows=np.count_nonzero(a[:,6]),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
 D[n]=dict(s=s,a=a,diag=diag,bounds=bounds,h=h_true,p=None)
 for method,h in [('logged',h_true),('yaw_1ms',h_true),('sparse_trapezoid',h_trap),('sparse_right',h_right)]:
  mid=np.r_[h[0],.5*(h[1:]+h[:-1])];p=xy if method=='logged' else np.c_[np.cumsum(ds*np.sin(mid)),np.cumsum(ds*np.cos(mid))]
  if method=='yaw_1ms':D[n]['p']=p
  pa=sample(s,p,bounds);ha=sample(s,h,bounds)
  for k in range(5):
   query=bounds[k]+Q*(bounds[k+1]-bounds[k]);pp=sample(s,p,query);hh=sample(s,h,query)
   P[n,method,k]=dict(p=pp,h=hh,local=rotate(pp-pp[0],-hh[0]),omega=sample(s,a[:,3],query),length=bounds[k+1]-bounds[k])
   delta=pa[k+1]-pa[k]
   intervals.append(dict(log=n,direction=direction,method=method,interval=k+1,X_mm=delta[0],Y_mm=delta[1],
    heading_error_deg=np.rad2deg(ha[k+1]-ha[k])-(360 if direction=='CW' else -360),
    lap_time_s=(np.interp(bounds[k+1],s,a[:,0])-np.interp(bounds[k],s,a[:,0]))/1000))
 for k in range(1,5):
  dif=P[n,'yaw_1ms',k]['local']-P[n,'yaw_1ms',0]['local']
  for i,q in enumerate(Q):profiles.append(dict(log=n,direction=direction,interval=k+1,q=q,shape_mm=np.linalg.norm(dif[i])))
 for b in range(20):
  diffs=[(P[n,'yaw_1ms',k]['local']-P[n,'yaw_1ms',0]['local']) for k in range(1,5)]
  increments=[np.linalg.norm(z[(b+1)*50]-z[b*50]) for z in diffs]
  sector.append(dict(log=n,direction=direction,begin_pct=b*5,end_pct=(b+1)*5,mean_difference_change_mm=np.mean(increments)))
summary=[];per_log=[]
for direction in ['CW','CCW']:
 nums=list(range(12866,12869)) if direction=='CW' else list(range(12869,12872))
 for method in ['logged','yaw_1ms','sparse_trapezoid','sparse_right']:
  zz=[r for r in intervals if r['direction']==direction and r['method']==method]
  summary.append(dict(direction=direction,method=method,X_mm=np.mean([r['X_mm'] for r in zz]),Y_mm=np.mean([r['Y_mm'] for r in zz]),heading_error_deg=np.mean([r['heading_error_deg'] for r in zz])))
  for n in nums:
   zz2=[r for r in zz if r['log']==n];per_log.append(dict(log=n,direction=direction,method=method,X_mm=np.mean([r['X_mm'] for r in zz2]),Y_mm=np.mean([r['Y_mm'] for r in zz2]),heading_error_deg=np.mean([r['heading_error_deg'] for r in zz2])))
 for method in ['yaw_1ms','sparse_trapezoid']:
  mean=np.mean([np.linalg.norm(P[n,method,k]['local']-P[n,method,0]['local'],axis=1) for n in nums for k in range(1,5)],axis=0)
  for threshold in [2,5]:
   rr=[(l,r) for l,r in m.runs(mean>threshold) if r-l>=20]
   onsets.append(dict(direction=direction,method=method,threshold_mm=threshold,onset_pct=Q[rr[0][0]]*100 if rr else '',max_shape_mm=mean.max()))
write('health.csv',health);write('lap_intervals.csv',intervals);write('summary.csv',summary);write('per_log.csv',per_log);write('onsets.csv',onsets);write('phase_profiles.csv',profiles);write('phase_sectors.csv',sector)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.grid':True,'grid.alpha':.2})
def save(fig,name):fig.savefig(OUT/f'{name}.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(13,10),layout='constrained')
for ax,n in zip(axs.flat,range(12866,12872)):
 for k in range(5):
  z=P[n,'yaw_1ms',k]['p'];ax.plot(z[:,0],z[:,1],label=f'interval {k+1}')
 ax.set(title=f'{n}: raw encoder + firmware 1-ms yaw',xlabel='X [mm]',ylabel='Y [mm]');ax.set_aspect('equal');ax.legend(fontsize=7,loc='center')
save(fig,'yaw_1ms_trajectories')
fig,axs=plt.subplots(3,2,figsize=(13,9),layout='constrained')
for col,(direction,nums) in enumerate([('CW',range(12866,12869)),('CCW',range(12869,12872))]):
 for n in nums:
  d=D[n];t=d['a'][:,0]/1000;axs[0,col].plot(t,d['diag'][:,0],label=str(n));axs[1,col].plot(t,d['diag'][:,3],label=f'{n} X');axs[1,col].plot(t,d['diag'][:,4],ls='--',label=f'{n} Y')
  dtheta=d['h']-np.r_[d['h'][0],d['h'][:-1]];avg=np.rad2deg(dtheta)/np.r_[1,np.diff(d['a'][:,0])]/.001
  axs[2,col].plot(t,avg-d['a'][:,3],alpha=.5,label=str(n))
 axs[0,col].set(title=direction,ylabel='IMU temperature [C]');axs[1,col].set(ylabel='Fused X/Y angle [deg]');axs[2,col].set(xlabel='Run time [s]',ylabel='Interval mean yaw rate - sampled gyro [deg/s]')
 for ax in axs[:,col]:ax.legend(fontsize=7,ncol=2)
save(fig,'imu_diagnostics')
fig,axs=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for ax,(direction,nums) in zip(axs,[('CW',range(12866,12869)),('CCW',range(12869,12872))]):
 for method in ['yaw_1ms','sparse_trapezoid']:
  norms=np.array([np.linalg.norm(P[n,method,k]['local']-P[n,method,0]['local'],axis=1) for n in nums for k in range(1,5)])
  ax.plot(Q*100,norms.mean(0),label=method)
 ax.axhline(2,c='.5',ls=':');ax.set(title=direction,xlabel='Phase since independent CROSS [%]',ylabel='Shape difference [mm]');ax.legend()
save(fig,'shape_comparison')
validation=dict(logs=6,columns=41,trailing_empty_column=True,expected_rows='PASS',normal_end='PASS',finite_values='PASS',
 max_csv_row_bytes=max(r['max_line_bytes'] for r in health),max_gap_ms=max(r['max_gap_ms'] for r in health),
 imu_temperature='PASS: no -999, changing values on exact 0.125 C lattice',
 yaw='PASS: unwrapped +/-2160 degrees over six course laps, direction matches user',
 note='Firmware yaw itself contains gyro errors. Endpoint yaw removes sparse gyro integration, not all localization error.')
(OUT/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
tbl=[]
for direction in ['CW','CCW']:
 log=next(r for r in summary if r['direction']==direction and r['method']=='logged')
 true=next(r for r in summary if r['direction']==direction and r['method']=='yaw_1ms')
 sparse=next(r for r in summary if r['direction']==direction and r['method']=='sparse_trapezoid')
 tbl.append(f"| {direction} | {log['X_mm']:+.1f} | {true['X_mm']:+.1f} | {sparse['X_mm']:+.1f} | {true['heading_error_deg']:+.3f} |")
individual=[]
for n in range(12866,12872):
 he=next(r for r in health if r['log']==n);r=next(r for r in per_log if r['log']==n and r['method']=='yaw_1ms')
 individual.append(f"| {n} | {r['direction']} | {r['X_mm']:+.1f} | {r['Y_mm']:+.1f} | {r['heading_error_deg']:+.3f} | {he['battery_V']} | {he['temp_cal_C']} → {he['temp_end_C']:.3f} |")
report='''# IMU温度・XYZ角度を追加したログの検証 — 12866～12871

2026-10-03。CW12866～12868、CCW12869～12871。解析でファームウェアの変更・書き込みは行っていない。

## 新形式の実機ログ確認

追加6列 `imuTemp_C`, `gyroVal_X`, `gyroVal_Y`, `imuAngle_X`, `imuAngle_Y`, `imuAngle_Z` が全6ログで存在する。既存35列の後ろに追加され、通常41列（既存仕様の末尾空列は除く）。全行で見出しと列数が一致。最大データ行264バイト（CRLF含む）で1024バイトバッファに収まり、改行切り捨てや連結行なし。

期待行数一致、emcStop=0、optimalTrace=0、校正100回・エラー0、時刻と生距離単調、全使用列が有限値。目標29 pulse/ms（約0.5 m/s）、生距離/時間の平均実速度は約0.539～0.541 m/s。最大11 msのログ間隔は5 mmごとの記録に整合する。独立ラインCROSSは各6回。縦横slipFlagは全行0で、実スリップ不存在の証明ではない。

IMU温度は全行有効で-999なし。実測値は0.125°C格子に一致し、走行中に13～30回値が変化した。XYZ角度、XY角速度も有限値。Z角度はCWで約+2160°、CCWで約-2160°まで連続積算され、360°で折り返されていない。

## 1 ms積算Z角度で確認したXドリフト

**疎なログ角速度の再積分を避けても、X+方向のドリフトは残る。** CSV x/yだけの表示異常や、疎な角速度積分の誤差だけでは説明できない。

| 方向 | CSV X [mm/周] | 生距離＋1 ms Z角度 X [mm/周] | 生距離＋疎な台形gyro X [mm/周] | 1 ms Zの一周±360°との差 [deg/周] |
| --- | ---: | ---: | ---: | ---: |
'''+ '\n'.join(tbl)+'''

各方向3ログ、各ログCROSSから次のCROSSまでの5完全区間を平均。1 ms Z角度はCSVの `imuAngle_Z` の取得時点での値そのもの。ジャイロの測定誤差まで除いた正解角度ではない。

| ログ | 方向 | 生距離＋1 ms ZのX [mm/周] | Y [mm/周] | 一周角度差 [deg/周] | 電圧 [V] | 校正平均温度 → 記録終了温度 [°C] |
| --- | --- | ---: | ---: | ---: | ---: | --- |
'''+ '\n'.join(individual)+'''

CCW12869/12871は一周角度差が約+0.036/+0.022°でも、Xは約+13.9/+13.7 mm/周移動している。一周総角度のずれが小さくても閉路位置のずれは残る。区間内の角度誤差分布・横速度欠落・距離の偏りなどは別途検証が必要。

前回12857～12862の生距離＋疎な台形gyroではCW+12.2、CCW+16.5 mm/周。今回同じ疎な方法ではCW+16.2、CCW+14.0。ログのばらつき、電圧、温度、方向反転時の条件差があるので、ログ列の追加による改善・悪化とは扱わない。

![1 ms Z角度による全6ログの軌跡](yaw_1ms_trajectories.png)

## 差が増える区間の再評価

各周の開始位置と開始推定向きをそろえ、最初の完全CROSS間区間と後続4区間を正規化距離1001点で比較。平均XY形状差が2 mmを超えて周回の2%以上続く最初の地点を検出点とする。CROSSを0%、次のCROSSを100%とする。この地点は絶対位置誤差や横滑りの発生開始点ではない。初周から毎周同じ誤差は形状比較だけでは見えない。

- CW: 1 ms Z角度では**52.7%**、同じログの疎な台形gyroでは46.6%。1 ms Zでの最大平均形状差は約3.63 mm、5 mmの持続検出なし。
- CCW: 1 ms Z角度では**40.6%**、疎な台形gyroでは34.1%。5 mm検出は60.2%、最大平均形状差は約7.95 mm。

同じデータでも積分方法だけで2 mm検出位置が約6ポイント動く。従来の疎なgyroによる検出位置は解析モデルに依存しており、今後の同一区間比較では記録済み1 ms Z角度を優先する。今回の検出位置も外部の絶対誤差開始点ではない。

![1 ms積算と疎な角速度積分の形状差比較](shape_comparison.png)

一周あたりX+14～17 mmの平行移動があるのに形状差は3～8 mm程度という結果は矛盾ではない。形状差のグラフでは各周の開始位置・向きを取り除いており、毎周繰り返される平行移動の成分は取り除かれる。形状の再現性と絶対推定位置のドリフトを分けて扱う。

## 温度とXY姿勢角

12866の記録温度は28.75→32.625°C、12869は29.125→33.375°Cへ上昇。12868/12871では温度変化が小さいが、Xドリフトはそれぞれ+17.8/+13.7 mm/周残る。温度変化がある走行だけに発生する現象ではない。温度補正後の残留誤差がないことを示すものではない。

X融合姿勢角の最小値は-35.5～-51.2°、Yはおおむね-32.4～+31.1°の範囲まで変化し、コースに同期したピークが繰り返される。これは現行calcDegreesの加速度融合値であり、走行中の並進・旋回加速度を受ける。**そのまま実際の機体傾斜角とは見なさない。** 現行コードのX/Yは加速度由来角度をコンプリメンタリ係数で更新した値。これを用いた3D座標回転やgyro Zの傾斜補正を直ちに採用しない。

XY角速度のRMSはX約7.9～13.1 deg/s、Y約2.5～2.7 deg/s。瞬間的な角速度・振動も含む指標で、姿勢角ピークの原因をこのRMSだけでは同定できない。

![温度・融合XY角・角速度標本化の診断](imu_diagnostics.png)

下段はZ積算角度差から求めたログ間隔内平均角速度と、行に記録された終端gyroVal_Zの差。曲率変化・振動時に瞬間標本と間隔平均が異なるのは自然で、差だけをセンサー異常と判定しない。疎な台形gyroと記録済みZの走行終端角度差は-0.540～+0.658°で、ログ後の再積分精度にも限界がある。

## 再計算方法と限界

入力は左右生積算エンコーダ、cntlog、校正済み角速度、ライン値、追加IMU列。CSV x/yは診断比較としてのみ使い、生軌跡計算には使用しない。ROC・encTotalOptimal・経路生成値も使用しない。

ds=(ΔencTotalL+ΔencTotalR)/2/58.019 [mm]。方位h=imuAngle_Z [deg]をそのままラジアンへ変換し、隣接ログ間の平均方位で各dsをXYへ投影する。1 ms角度の終端値を使うためログgyroの疎な積分は不要。ただしログ間隔内の距離/角度の細かな時間関係までは記録されておらず、厳密な1 ms XY再現ではない。既存温度・方向補正を再適用しない。追加yI=0。

独立CROSSは全幅ライン反応と前回の素子幾何から車軸中点通過へ補正する。外部の正しいコース座標、車体横速度は記録されていないため、原因を確定してはいない。次の重点は各周で繰り返す平行移動成分であり、旋回前後の距離・方向・横加速度の整合を調べる。新しい補正値の採用やファームウェア変更は行っていない。

スクリプト: analysis/script/analyze_imu_angles_12866_12871.py。health.csvにSHA-256、列数・行数・最大行長、温度・角度範囲を保存。前回のanalyze_yi_12838_12848.pyとsensor_centers.csvを再利用。validation.jsonが初回実機ログの新列・保存形式の確認結果。
'''
(OUT/'report.md').write_text(report,encoding='utf-8')
print(json.dumps(dict(summary=summary,onsets=onsets,per_log=[r for r in per_log if r['method']=='yaw_1ms']),indent=2))
