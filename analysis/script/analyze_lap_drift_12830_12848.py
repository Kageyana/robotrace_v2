"""Locate repeatability loss without treating a reconstructed lap as ground truth.

Raw encoder mean is the baseline. Logged XY is an independent firmware diagnostic.
Six CROSS events delimit five complete laps plus two partial intervals.
"""
from pathlib import Path
import os, csv, json, hashlib
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'analysis/lap_drift_12830_12848'
OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(OUT / '.mplcache')
import numpy as np
import analyze_yi_12838_12848 as m
m.OUT = OUT
plt = m.plt
LOG = Path('F:/Dropbox/Document/robotrace/Log/v2')
NUMS = list(range(12830, 12843)) + list(range(12844, 12849))
Q = np.linspace(0, 1, 1001)
health, laps, profiles, bins = [], [], [], []
P = {}

def write(name, rows):
    with (OUT / name).open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, rows[0].keys()); w.writeheader(); w.writerows(rows)

def integrate(a, method='trapezoid', corrected=False):
    dt = np.r_[0, np.diff(a[:, 0])] / 1000
    w = a[:, 3].copy()
    if method == 'trapezoid': w[1:] = .5 * (w[1:] + w[:-1])
    dh = np.deg2rad(w) * dt; h = np.cumsum(dh); mid = h - .5 * dh
    kl, kr = (.99675455, 1.00324545) if corrected else (1, 1)
    ds = np.r_[0, (kl * np.diff(a[:, 1]) + kr * np.diff(a[:, 2])) / 116.038]
    return np.c_[np.cumsum(ds * np.sin(mid)), np.cumsum(ds * np.cos(mid))], h

def rotate(v, angle):
    # CW-positive heading: row-vector transform rotates forward toward +X.
    c, s = np.cos(angle), np.sin(angle)
    return v @ np.array([[c, -s], [s, c]])

def sample(s, values, query):
    if values.ndim == 1: return np.interp(query, s, values)
    return np.c_[[np.interp(query, s, values[:, j]) for j in range(values.shape[1])]].T

for n in NUMS:
    path = LOG / f'{n}.csv'
    with path.open(encoding='utf-8-sig') as f:
        meta = dict(v.split('=', 1) for v in next(f).strip().split(',') if '=' in v)
        rows = list(csv.DictReader(f))
    a = np.array([[float(r[c]) for c in m.COLS] for r in rows])
    xy = np.array([[float(r[c]) for c in ['x', 'y']] for r in rows])
    s = ((a[:, 1] + a[:, 2]) - (a[0, 1] + a[0, 2])) / 116.038
    d = dict(a=a, s=s, meta=meta)
    m.D[n] = d
    d['line'], _ = m.anchor(d)
    mark = a[:, 4]; ci = np.flatnonzero((mark == 3) & (np.r_[0, mark[:-1]] != 3))
    assert len(ci) == len(d['line']) == 6, n
    assert np.all(np.diff(a[:, 0]) > 0) and np.all(np.diff(s) > 0), n
    assert len(a) == int(meta['logExpectedRows']) and float(meta['emcStop']) == 0, n
    assert float(meta['optimalTrace']) == 0, n
    anchors = m.axleanchors(d)
    assert anchors[-1] < s[-1] and anchors[0] > s[0], n
    direction = 'CW' if n < 12844 else 'CCW'
    health.append(dict(log=n, direction=direction, rows=len(a), gap_ms=np.diff(a[:, 0]).max(), distance_mm=s[-1],
                       battery_V=meta['batteryVoltage_V'], git=meta['gitCommit'], emcStop=meta['emcStop'],
                       calibration=meta['imuCalibrationValid'], slip_rows=np.count_nonzero(a[:, 5]),
                       lateral_slip_rows=np.count_nonzero(a[:, 6]), cross_count=len(ci),
                       prefix_mm=anchors[0], suffix_mm=s[-1]-anchors[-1], sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for method in ['logged', 'right', 'trapezoid', 'trapezoid_calibrated']:
        p, h = integrate(a, 'right' if method == 'right' else 'trapezoid', method.endswith('calibrated'))
        if method == 'logged': p = xy
        pa = sample(s, p, anchors); ha = sample(s, h, anchors)
        for k in range(5):
            pp = sample(s, p, anchors[k] + Q * (anchors[k+1]-anchors[k]))
            hh = sample(s, h, anchors[k] + Q * (anchors[k+1]-anchors[k]))
            omega = sample(s, a[:, 3], anchors[k] + Q * (anchors[k+1]-anchors[k]))
            expected = 2 * np.pi if direction == 'CW' else -2 * np.pi
            closure = pa[k+1]-pa[k]
            laps.append(dict(log=n, direction=direction, method=method, interval=k+1, length_mm=anchors[k+1]-anchors[k],
                             duration_ms=np.interp(anchors[k+1],s,a[:,0])-np.interp(anchors[k],s,a[:,0]),
                             start_X_mm=pa[k,0], start_Y_mm=pa[k,1], delta_X_mm=closure[0], delta_Y_mm=closure[1],
                             heading_error_deg=np.rad2deg(ha[k+1]-ha[k]-expected)))
            P[n,method,k] = dict(p=pp, h=hh, local=rotate(pp-pp[0], -hh[0]), omega=omega,
                                  length=anchors[k+1]-anchors[k], start_h=hh[0])
    # Relative to first complete CROSS-to-CROSS lap; no arbitrary rigid best fit.
    for method in ['logged', 'right', 'trapezoid', 'trapezoid_calibrated']:
        ref = P[n,method,0]
        for k in range(1,5):
            cur = P[n,method,k]
            global_diff = cur['p'] - ref['p']
            translated = (cur['p']-cur['p'][0])-(ref['p']-ref['p'][0])
            local_diff = cur['local']-ref['local']
            dh = np.rad2deg((cur['h']-cur['h'][0])-(ref['h']-ref['h'][0]))
            for i,q in enumerate(Q):
                profiles.append(dict(log=n,direction=direction,method=method,interval=k+1,q=q,s_mm=q*ref['length'],
                                     global_dX_mm=global_diff[i,0],global_dY_mm=global_diff[i,1],
                                     translated_dX_mm=translated[i,0],translated_dY_mm=translated[i,1],
                                     local_dX_mm=local_diff[i,0],local_dY_mm=local_diff[i,1],heading_change_deg=dh[i]))
    for b in range(20):
        ix=slice(b*50,(b+1)*50+1)
        ds = [P[n,'trapezoid',k]['local'][ix]-P[n,'trapezoid',0]['local'][ix] for k in range(1,5)]
        x0 = b*50; x1 = (b+1)*50
        dd = np.array([v[-1]-v[0] for v in ds])
        bins.append(dict(log=n,direction=direction,bin=b+1,begin_pct=b*5,end_pct=(b+1)*5,
                         begin_mm=b*.05*P[n,'trapezoid',0]['length'],end_mm=(b+1)*.05*P[n,'trapezoid',0]['length'],
                         local_increment_mm=np.mean(np.linalg.norm(dd,axis=1)),
                         local_increment_X_mm=dd[:,0].mean(),local_increment_Y_mm=dd[:,1].mean(),
                         mean_abs_gyro_dps=np.mean(np.abs(P[n,'trapezoid',0]['omega'][ix]))))
write('health.csv',health); write('lap_closure.csv',laps); write('phase_profiles.csv',profiles); write('phase_bins.csv',bins)

# Aggregate actual same-location lap translations and orientation-corrected shape error.
summary=[]
for direction,nums in [('CW',NUMS[:13]),('CCW',NUMS[13:])]:
    for method in ['logged','right','trapezoid','trapezoid_calibrated']:
        rr=[r for r in laps if r['direction']==direction and r['method']==method]
        summary.append(dict(direction=direction,method=method,intervals=len(rr),mean_dX_mm=np.mean([r['delta_X_mm'] for r in rr]),
                            min_dX_mm=min(r['delta_X_mm'] for r in rr),max_dX_mm=max(r['delta_X_mm'] for r in rr),
                            mean_dY_mm=np.mean([r['delta_Y_mm'] for r in rr]),
                            mean_heading_error_deg=np.mean([r['heading_error_deg'] for r in rr])))
write('summary.csv',summary)

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.grid':True,'grid.alpha':.2})
def save(fig,name):
    fig.savefig(OUT/f'{name}.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax,n in zip(axs,[12838,12844]):
    d=m.D[n];p,h=integrate(d['a']);ax.plot(p[:,0],p[:,1],color='.8',lw=1)
    for k in range(5):
        z=P[n,'trapezoid',k]['p'];ax.plot(z[:,0],z[:,1],label=f'CROSS interval {k+1}');ax.scatter(z[0,0],z[0,1],s=20)
    ax.set(title=f'{n}: raw encoder + trapezoid gyro',xlabel='X [mm]',ylabel='Y [mm]');ax.set_aspect('equal');ax.legend(fontsize=8)
save(fig,'reintegrated_laps')
fig,axs=plt.subplots(2,3,figsize=(15,8),layout='constrained')
for row,(direction,nums) in enumerate([('CW',NUMS[:13]),('CCW',NUMS[13:])]):
    for n in nums:
        r=[z for z in laps if z['log']==n and z['method']=='logged']
        anchors_x=[z['start_X_mm'] for z in r]+[r[-1]['start_X_mm']+r[-1]['delta_X_mm']]
        axs[row,0].plot(range(1,7),anchors_x,np.random.default_rng(n).choice(['-','--',':']),label=str(n),alpha=.8)
    for k in range(1,5):
        tt=np.array([(P[n,'trapezoid',k]['p']-P[n,'trapezoid',k]['p'][0])-(P[n,'trapezoid',0]['p']-P[n,'trapezoid',0]['p'][0]) for n in nums])
        ll=np.array([P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'] for n in nums])
        axs[row,1].plot(Q*100,tt[:,:,0].mean(0),label=f'interval {k+1} minus 1')
        axs[row,2].plot(Q*100,np.linalg.norm(ll,axis=2).mean(0),label=f'interval {k+1} minus 1')
    axs[row,0].set(title=f'{direction}: firmware X at identical CROSS',xlabel='CROSS occurrence',ylabel='X [mm]')
    axs[row,1].set(title=f'{direction}: start translation removed',xlabel='CROSS-to-CROSS phase [%]',ylabel='X difference [mm]')
    axs[row,2].set(title=f'{direction}: start translation + heading removed',xlabel='CROSS-to-CROSS phase [%]',ylabel='Shape difference [mm]')
    for ax in axs[row]:ax.legend(fontsize=7,ncol=2)
save(fig,'drift_decomposition')
fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained')
phase_summary=[]
for row,(direction,nums) in enumerate([('CW',NUMS[:13]),('CCW',NUMS[13:])]):
    local=np.array([P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'] for n in nums for k in range(1,5)])
    norms=np.linalg.norm(local,axis=2);mean=norms.mean(0)
    w=np.array([P[n,'trapezoid',0]['omega'] for n in nums]).mean(0)
    axs[row,0].plot(Q*100,mean,label='Mean shape difference');axs[row,0].fill_between(Q*100,np.percentile(norms,10,axis=0),np.percentile(norms,90,axis=0),alpha=.2,label='10-90% across intervals')
    axs[row,0].set(title=direction,xlabel='CROSS-to-CROSS phase [%]',ylabel='Shape difference [mm]');axs[row,0].legend()
    axs[row,1].plot(Q*100,w);axs[row,1].set(title=direction,xlabel='CROSS-to-CROSS phase [%]',ylabel='Mean gyro [deg/s]')
    for threshold in [2,5,10]:
        mask=mean>threshold;run=[(l,r) for l,r in m.runs(mask) if r-l>=20]
        if run:
            i=run[0][0];phase_summary.append(dict(direction=direction,threshold_mm=threshold,first_sustained_pct=Q[i]*100,first_sustained_mm=Q[i]*np.mean([P[n,'trapezoid',0]['length'] for n in nums]),duration_requirement_pct=2))
save(fig,'shape_onset')
write('onset_thresholds.csv',phase_summary)
checks=[]
for group,nums in [('CW30-37',NUMS[:8]),('CW38-42',NUMS[8:13]),('CCW44-48',NUMS[13:])]:
    for method in ['right','trapezoid','trapezoid_calibrated']:
        for registration in ['normalized_distance','absolute_distance']:
            diff=[]
            for n in nums:
                ref=P[n,method,0]
                for k in range(1,5):
                    cur=P[n,method,k]
                    if registration=='absolute_distance':
                        limit=min(ref['length'],cur['length'])
                        refp=sample(Q*ref['length'],ref['local'],Q*limit)
                        curp=sample(Q*cur['length'],cur['local'],Q*limit)
                    else:refp,curp=ref['local'],cur['local']
                    diff.append(np.linalg.norm(curp-refp,axis=1))
            mean=np.mean(diff,axis=0)
            for threshold in [2,5]:
                rr=[(l,r) for l,r in m.runs(mean>threshold) if r-l>=20]
                checks.append(dict(group=group,method=method,registration=registration,threshold_mm=threshold,
                                   first_sustained_pct=Q[rr[0][0]]*100 if rr else '',end_shape_mm=mean[-1]))
write('onset_sensitivity.csv',checks)

# Plot phase locations on representative contours, not as surveyed coordinates.
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax,n,thresholds in zip(axs,[12838,12844],[[23.5,59.4,72.5],[25.6,35,50.1]]):
    z=P[n,'trapezoid',0]['p'];ax.plot(z[:,0],z[:,1],color='.35')
    for q in range(0,100,10):
        i=q*10;ax.scatter(z[i,0],z[i,1],s=12,c='.5');ax.annotate(f'{q}%',z[i],xytext=(4,4),textcoords='offset points',fontsize=8)
    for q in thresholds:
        i=round(q*10);ax.scatter(z[i,0],z[i,1],s=55,facecolors='none',edgecolors='crimson');ax.annotate(f'{q:g}%',z[i],xytext=(10,-18),textcoords='offset points',color='crimson',fontsize=9)
    ax.set(title=f'{n}: phase since axle CROSS (red: investigation points)',xlabel='Reintegrated X [mm]',ylabel='Reintegrated Y [mm]');ax.set_aspect('equal')
save(fig,'investigation_locations')

# Reconstruct logged XY exactly: firmware samples endpoint gyro and integer corrected speed.
firmware_checks=[]
for n in NUMS:
    with (LOG/f'{n}.csv').open(encoding='utf-8-sig') as f:
        next(f);rr=list(csv.DictReader(f))
    t=np.array([float(r['cntlog']) for r in rr]);dt=np.diff(np.r_[0,t])/1000
    gyro=np.array([float(r['gyroVal_Z']) for r in rr]);vel=np.array([float(r['encCurrentCorr_p']) for r in rr])*1000/58.019
    h=np.cumsum(np.deg2rad(gyro)*dt)
    xy_est=np.c_[np.cumsum(vel*dt*np.sin(h)),np.cumsum(vel*dt*np.cos(h))]
    xy_logged=np.array([[float(r['x']),float(r['y'])] for r in rr])
    firmware_checks.append(dict(log=n,max_error_mm=np.max(np.linalg.norm(xy_est-xy_logged,axis=1)),end_error_mm=np.linalg.norm(xy_est[-1]-xy_logged[-1])))
write('firmware_xy_check.csv',firmware_checks)
assert max(r['max_error_mm'] for r in firmware_checks)<.1, 'Float32 firmware reconstruction should agree within 0.1 mm over 28 m'
assert all(r['gap_ms']<=6 for r in health)
assert len(laps)==18*5*4 and len(profiles)==18*4*4*len(Q)
assert np.max(np.abs(rotate(np.eye(2),.7) @ rotate(np.eye(2),-.7)-np.eye(2)))<1e-12
group_summary=[]
fig,axs=plt.subplots(2,3,figsize=(15,8),layout='constrained')
for col,(group,nums) in enumerate([('CW30-37',NUMS[:8]),('CW38-42',NUMS[8:13]),('CCW44-48',NUMS[13:])]):
    local=np.array([P[n,'trapezoid',k]['local']-P[n,'trapezoid',0]['local'] for n in nums for k in range(1,5)])
    norm=np.linalg.norm(local,axis=2)
    mean=norm.mean(0)
    axs[0,col].plot(Q*100,mean);axs[0,col].fill_between(Q*100,np.percentile(norm,10,axis=0),np.percentile(norm,90,axis=0),alpha=.2)
    axs[0,col].axhline(2,color='.5',ls=':');axs[0,col].axhline(5,color='.5',ls='--')
    axs[0,col].set(title=group,xlabel='Phase since CROSS [%]',ylabel='Shape difference [mm]')
    axs[1,col].plot(Q*100,np.array([P[n,'trapezoid',0]['omega'] for n in nums]).mean(0))
    axs[1,col].set(xlabel='Phase since CROSS [%]',ylabel='Gyro [deg/s]')
    length=np.mean([P[n,'trapezoid',0]['length'] for n in nums]);prefix=np.mean([m.axleanchors(m.D[n])[0] for n in nums])
    cs=[r for r in checks if r['group']==group and r['method']=='trapezoid' and r['registration']=='normalized_distance']
    closure=[r for r in laps if r['log'] in nums and r['method']=='logged']
    raw=[r for r in laps if r['log'] in nums and r['method']=='trapezoid']
    group_summary.append(dict(group=group,logs=len(nums),length_mm=length,first_cross_mm=prefix,
                              onset_2mm_pct=cs[0]['first_sustained_pct'],onset_2mm_from_cross_mm=cs[0]['first_sustained_pct']/100*length,
                              onset_5mm_pct=cs[1]['first_sustained_pct'],onset_5mm_from_cross_mm=cs[1]['first_sustained_pct']/100*length,
                              firmware_dx_per_lap_mm=np.mean([r['delta_X_mm'] for r in closure]),
                              raw_dx_per_lap_mm=np.mean([r['delta_X_mm'] for r in raw]),
                              heading_error_deg=np.mean([r['heading_error_deg'] for r in raw])))
save(fig,'group_onsets');write('group_summary.csv',group_summary)
validation=dict(health='PASS: 18 logs, exact rows, monotonic timestamps and distance, <=6 ms gaps, emcStop=0, six CROSS events',
                firmware_xy_max_difference_mm=max(r['max_error_mm'] for r in firmware_checks),
                rotation_round_trip='PASS',intervals_per_method=90,
                warning='A closed-loop trajectory and external orientation truth are unavailable; shape difference onset is not absolute error onset.')
(OUT/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')

table='\n'.join(f"| {r['group']} | {r['firmware_dx_per_lap_mm']:+.1f} | {r['raw_dx_per_lap_mm']:+.1f} | {r['onset_2mm_pct']:.1f}% / {r['onset_2mm_from_cross_mm']:.0f} mm | {r['onset_5mm_pct']:.1f}% / {r['onset_5mm_from_cross_mm']:.0f} mm |" for r in group_summary)
report=f'''# 周回によるXドリフトの発生区間調査 — 12830～12848

2026-10-03。対象18ログ。12843.csvは存在しない。ファームウェア変更・書き込みは行っていない。

## 結果

同じCROSS地点の推定XはCW平均+11.0 mm/周、CCW平均+7.8 mm/周増加した。生の左右エンコーダ積算値と台形積分gyroによる再計算でもCW+9.1、CCW+5.8 mm/周となり、表示座標だけの異常ではない。ここでの「周」は同じCROSSから次のCROSSまで。

開始位置・開始推定角度を一致させた後、最初の完全CROSS間区間と後続4区間の軌跡形状差を比較した。平均差が2 mmを超え、その状態が周回の2%以上（約95 mm）続く最初の地点を検出点とした。**この2 mmは解析上のしきい値であり、実際の誤差がそこから発生したとの断定ではない。** 微小な差はそれ以前から存在する。

| ログ群 | CSVのX移動 [mm/周] | 生データ台形積分 [mm/周] | 形状差2 mm検出（CROSSから） | 形状差5 mm検出（CROSSから） |
| --- | ---: | ---: | --- | --- |
{table}

CW12830～12837と12838～12842では検出位置が異なるため、CW全体を平均した37.5%を共通の開始位置として採用しない。前者の電圧8.13→7.90 V、後者8.35→8.23 V。電圧の影響を同定したわけではない。CCWは8.17→8.06 V。

![ログ群別の検出位置](group_onsets.png)

## 調査を優先する区間

12838～12842の形状差はCROSS後約1.1 m（23.5%）、最初の広いカーブ区間内ですでに2 mmを超える。CCWはCROSS後約1.2 m（25.6%）、最初の大きな正旋回から負の急旋回へ切り替わる区間で2 mmを超える。

変化の大きい区間として、CW70～75%（CROSS後約3.31～3.55 m）とCCW25～40%（約1.19～1.90 m）を優先する。CROSSを共通基準にCCWの進行順を反転すると、CW70～75%はCCW25～30%に概ね対応し、同じ急旋回付近になる。ただし周回距離・進行方向による検出点の差があるので、割合の反転だけで厳密な同一点とは扱わない。

5%区間ごとに、開始位置・向きをそろえた差ベクトルの変化量を計算した（phase_bins.csv）。CW70～75%は平均2.47 mmで最大、CCW35～40%は2.05 mm、30～35%は1.95 mm。CCWでは急旋回の後にも差が増え、60～85%の直線付近でも拡大する。この直線上の拡大は手前で生じた角度誤差の伝播でも起こるため、その場所で横滑りしたという証拠にはならない。

![代表ログのコース上の調査位置](investigation_locations.png)

灰色の線と座標は生ログから再積分した推定値。実測コース図ではない。割合はCROSS通過を0%、次のCROSSを100%とする。赤丸は2/5 mm検出点と大きく変化する区間の代表点。CWの2/5 mm点は12838～12842群、CCWは12844～12848群の平均。

## 比較方法と意味

- ユーザーの6周走行のうち、CROSS6回で直接切り出せる完全な区間は5周分。開始前側約2.54 m（CW）/1.80 m（CCW）と終了側約1.77 m/2.52 mは全軌跡図に表示するが、完全周回との形状差計算に混ぜない。
- CROSSはcourseMarker=3の立ち上がりを確認し、独立に検出したライン全幅反応と前回の素子位置から車軸中点通過へ補正。前回の解析スクリプトのanchor/axleanchorsとsensor_centers.csvを再利用した。
- 基準の距離は `(ΔencTotalL+ΔencTotalR)/2/58.019` [mm]。x/yは生データ再計算に使用せず、CSV座標の診断として別に比較した。外部座標の正解としては使っていない。
- gyroは校正済みgyroVal_Z。スケール係数-0.9924を再適用しない。追加yI補正0。台形積分角度と中点方位で変位を積分した。
- 同一ログ内で最初の完全CROSS間区間を基準に後続4区間を比較。1001点の正規化距離で対応付け、開始位置を引き、開始角度だけを取り除く。軌跡全体の最小二乗位置合わせは行わない。基準区間自体の誤差は未知であり、毎周同じ誤差を完全に同定することはできない。
- グラフの形状差はX差だけではなくXY差の大きさ。開始角度をそろえた座標系なので、形状差のXをそのままスタート原点のX方向とは扱わない。
- 同じログの周回や同じ基準区間を共有する比較は独立試行ではない。帯は分布の10～90%であり、信頼区間ではない。

![周回ドリフトの分解](drift_decomposition.png)

左は座標の平行移動を残した同一CROSS地点のX。中は各区間の開始位置だけをそろえたX差。右はさらに開始推定角度をそろえたXY形状差。左に残る平行移動と右の形状差は別の量。

![全走行の再積分と完全区間](reintegrated_laps.png)

## 感度と確認

右端gyro積分/台形積分、既存の左右エンコーダ相対補正あり/なし、正規化距離/等距離対応で再確認（onset_sensitivity.csv）。2 mm検出位置はCW30～37で41.1～43.4%、CW38～42で22.4～24.0%、CCW44～48で25.5～25.6%。5 mm検出はそれぞれ79.9～82.9%、56.3～59.5%、49.8～50.1%。群間差のほうが手法による差より大きい。

全18ログでemcStop=0、期待行数一致、時刻/生距離単調、最大ログ間隔6 ms、CROSS6回、IMU校正有効。縦横slipFlagはすべて0で、実スリップ不存在の証明ではない。ログSHA-256をhealth.csvに保存。

CSV x/yは現行SDcard.c/endLog→courseAnalysis.c/calcXYcieの式（整数encCurrentCorr_p×dt、右端gyro×dt、更新後角度）を再現し、最大差{max(r['max_error_mm'] for r in firmware_checks):.3f} mmだった。float32の逐次丸めとCSV丸めを含むため厳密な同一ビット比較ではない。生積算エンコーダ＋台形gyroとの周回X差は平均約2～3 mm/周。ログ標本からの再計算なので、その差をそのまま実機の誤差修正量と断定しない。

積算角度の一周±360°からの平均差はCW-0.174°/周、CCW-0.030°/周。方位誤差だけで平行移動がどの区間に発生したかは一意に決まらない。加速度・エンコーダ左右差・角速度の詳細因果診断は次段階とする。

## 現時点の限界と次の調査

今回特定したのは**周回ごとの推定形状の差が検出可能になる区間と、差が大きく変化する区間**。外部の正しいコース座標・横速度は記録されていないため、初周から毎周同じだけ生じる絶対位置誤差の開始点や、横滑りの発生点を一意に確定することはできない。実走行は同じコースをたどっているというユーザー確認を前提にした推定不整合の調査である。

次は最初の広いカーブと上記急旋回区間について、左右エンコーダ差・gyro積算角度・補正済み加速度・推定横加速度の整合を比較し、並進横速度の欠落、角度誤差、ログ再積分の差を区別する。現在は補正値の採用もファームウェア変更も行っていない。

実行: `python analysis/script/analyze_lap_drift_12830_12848.py`。前回コミット内のanalyze_yi_12838_12848.pyとanalysis/yi_12838_12848/sensor_centers.csvが必要。追加のビルドはファームウェア未変更のため実施していない。
'''
(OUT/'report.md').write_text(report,encoding='utf-8')
print(json.dumps(dict(summary=summary,onsets=phase_summary,top_bins={direction:sorted([dict(bin=b,mean_increment_mm=np.mean([r['local_increment_mm'] for r in bins if r['direction']==direction and r['bin']==b])) for b in range(1,21)],key=lambda x:x['mean_increment_mm'],reverse=True)[:5] for direction in ['CW','CCW']}),indent=2))
