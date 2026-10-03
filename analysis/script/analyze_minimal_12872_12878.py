"""Compare minimal logs against diagnostic logs at confirmed CROSS onsets."""
from pathlib import Path
import csv, json, os, hashlib
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'analysis/minimal_12872_12878'
OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(OUT / '.mplcache')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
LOG = Path('F:/Dropbox/Document/robotrace/Log/v2')
FIELDS = 'cntlog encCurrentN gyroVal_Z courseMarker encTotalOptimal ROC targetSpeed optimalIndex slipFlag slipFlagLat encCurrentCorr_p x y'.split()
health, laps, summary = [], [], []
DATA = {}
for n in range(12866, 12879):
    p = LOG / f'{n}.csv'
    lines = p.read_text(encoding='utf-8-sig').splitlines()
    meta = dict(v.split('=', 1) for v in lines[0].split(',') if '=' in v)
    header = next(csv.reader([lines[1]]))
    raw = list(csv.reader(lines[2:]))
    assert all(len(r) == len(header) and r[-1] == '' for r in raw), n
    if n >= 12872:
        assert header == FIELDS + [''], n
    rows = list(csv.DictReader(lines[1:]))
    a = {k: np.array([float(r[k]) for r in rows]) for k in FIELDS}
    assert all(np.all(np.isfinite(v)) for v in a.values()), n
    assert len(rows) == int(meta['logExpectedRows']), n
    assert float(meta['emcStop']) == 0 and float(meta['optimalTrace']) == 0, n
    assert meta['imuCalibrationValid'] == '1' and meta['imuCalibrationReadErrors'] == '0', n
    assert meta['distanceScaleVerified'] == '1', n
    t, s = a['cntlog'], a['encTotalOptimal'] / 58.019
    assert np.all(np.diff(t) > 0) and np.all(np.diff(s) > 0), n
    assert np.max(np.diff(t)) <= 11, n
    ii = np.flatnonzero((a['courseMarker'] == 3) & (np.r_[0, a['courseMarker'][:-1]] != 3))
    assert len(ii) == 6, n
    direction = 'CW' if n <= 12868 or n >= 12876 else 'CCW'
    assert (a['gyroVal_Z'].mean() > 0) == (direction == 'CW'), n
    group = direction + ('_minimal' if n >= 12872 else '_diagnostic')
    h = np.cumsum(np.r_[0, .5*(a['gyroVal_Z'][1:]+a['gyroVal_Z'][:-1])*np.diff(t)/1000])
    active = slice(ii[0], ii[-1]+1)
    health.append(dict(log=n, group=group, rows=len(rows), columns=len(header)-1,
        max_row_bytes=max(len(v.encode('utf-8'))+2 for v in lines[2:]), max_gap_ms=np.diff(t).max(),
        target_mps=np.median(a['targetSpeed'])/58.019,
        actual_distance_mps=(s[ii[-1]]-s[ii[0]])/(t[ii[-1]]-t[ii[0]]),
        speed_tracking_rmse_mps=np.sqrt(np.mean((a['encCurrentN'][active]-a['targetSpeed'][active])**2))/58.019,
        battery_V=float(meta['batteryVoltage_V']), cal_samples=int(meta['imuCalibrationSamples']),
        temp_cal_C=float(meta['imuTempCalibration_C']), temp_end_C=float(meta['imuTempEnd_C']),
        closure_valid=int(meta['closureValid']), closure_reason=int(meta['closureReason']),
        goal_marker_X_mm=float(meta['goalMarkerX_mm']),
        slip_rows=int(np.count_nonzero(a['slipFlag'])), slip_lat_rows=int(np.count_nonzero(a['slipFlagLat'])),
        sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    # mm/ms has the same numerical value as m/s; detect conversion mistakes.
    assert .45 < health[-1]['actual_distance_mps'] < .6, n
    for k, (i, j) in enumerate(zip(ii[:-1], ii[1:])):
        laps.append(dict(log=n, group=group, interval=k+1,
            delta_X_mm=a['x'][j]-a['x'][i], delta_Y_mm=a['y'][j]-a['y'][i],
            lap_time_s=(t[j]-t[i])/1000, lap_distance_mm=s[j]-s[i],
            sparse_heading_error_deg=h[j]-h[i]-(360 if direction == 'CW' else -360)))
    DATA[n] = a, ii
for n in range(12866, 12879):
    rr = [r for r in laps if r['log'] == n]
    summary.append(dict(log=n, group=rr[0]['group'],
        **{k:float(np.mean([r[k] for r in rr])) for k in ['delta_X_mm','delta_Y_mm','lap_time_s','lap_distance_mm','sparse_heading_error_deg']}))
def write(name, rr):
    with (OUT/name).open('w', encoding='utf-8-sig', newline='') as f:
        w=csv.DictWriter(f,rr[0].keys()); w.writeheader(); w.writerows(rr)
write('health.csv',health); write('lap_intervals.csv',laps); write('per_log.csv',summary)
groups=[]
for group in ['CW_diagnostic','CW_minimal','CCW_diagnostic','CCW_minimal']:
    rr=[r for r in summary if r['group']==group]
    groups.append(dict(group=group, logs=len(rr), **{k:float(np.mean([r[k] for r in rr])) for k in ['delta_X_mm','delta_Y_mm','lap_time_s','sparse_heading_error_deg']}))
write('group_comparison.csv',groups)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.grid':True,'grid.alpha':.2})
fig,axs=plt.subplots(2,4,figsize=(13,9),layout='constrained')
for ax,n in zip(axs.flat,range(12872,12879)):
    a,ii=DATA[n]
    for k,(i,j) in enumerate(zip(ii[:-1],ii[1:])):
        ax.plot(a['x'][i:j+1],a['y'][i:j+1],label=f'lap {k+1}',lw=1)
    ax.set_aspect('equal'); ax.set(title=f'{n}: {summary[n-12866]["group"].split("_")[0]}',xlabel='X [mm]',ylabel='Y [mm]')
axs.flat[-1].axis('off')
axs.flat[-1].legend(*axs.flat[0].get_legend_handles_labels(),loc='center')
fig.savefig(OUT/'trajectories.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for ax,direction in zip(axs,['CW','CCW']):
    for suffix,color in [('diagnostic','#888888'),('minimal','#2166ac')]:
        for r in summary:
            if r['group']==direction+'_'+suffix:
                rr=[v for v in laps if v['log']==r['log']]
                ax.plot(range(6),np.r_[0,np.cumsum([v['delta_X_mm'] for v in rr])],label=str(r['log']),color=color,alpha=.7)
    ax.set(title=direction,xlabel='CROSS interval count',ylabel='X shift since first confirmed CROSS [mm]');ax.legend()
fig.savefig(OUT/'x_drift_comparison.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(11,6),layout='constrained')
for n in range(12872,12879):
    a,_=DATA[n]
    axs[0].plot(a['cntlog']/1000,a['encCurrentN']/58.019,lw=.5,alpha=.5,label=str(n))
    axs[1].plot(a['cntlog']/1000,a['gyroVal_Z'],lw=.5,alpha=.5)
axs[0].axhline(29/58.019,color='black',ls='--',label='target')
axs[0].set(xlabel='Time [s]',ylabel='Encoder speed [m/s]');axs[0].legend(ncol=4)
axs[1].set(xlabel='Time [s]',ylabel='Gyro Z [deg/s]')
fig.savefig(OUT/'speed_gyro.png',dpi=160);plt.close(fig)
table='\n'.join(f"| {r['log']} | {r['group'].split('_')[0]} | {r['delta_X_mm']:+.1f} | {r['delta_Y_mm']:+.1f} | {r['lap_time_s']:.3f} |" for r in summary if r['log']>=12872)
report=f'''# 最小ログ12872～12878の検証

12872～12875 CCW、12876～12878 CW。ファームウェア変更・書き込みは実施していない。

## 記録形式と健全性

全7本が通常13列（末尾空列を除く）。最大データ行80バイト。期待行数一致、正常終了emcStop=0、一次走行optimalTrace=0、校正100サンプル・読取エラー0、距離換算検証成功。時刻・距離は単調、最大ログ間隔11 msで5 mm距離記録に整合。全使用値は有限値。CROSS確定は各6回、slipFlag/slipFlagLatは全行0。フラグ0は実スリップがない証拠ではない。

目標29 pulse/ms = 0.4998 m/s、CROSS区間の距離/時間で約0.538～0.540 m/s。速度追従RMSEは約0.043～0.044 m/s。

## 周回ドリフト

各ログの6回のCROSS確定先頭行を境界に、5個の完全CROSS間区間を比較。CSV x/yを評価する。独立ラインCROSSや実位置を使った解析ではない。前回12866～12871も同じCROSS確定・CSV x/y方式で再計算した。

| ログ | 方向 | X [mm/周] | Y [mm/周] | 区間時間 [s/周] |
| --- | --- | ---: | ---: | ---: |
{table}

同じ比較方法で、CWは前回+16.6 → 今回+9.7 mm/周（約41%減）、CCWは+15.5 → +17.0 mm/周。CWは全3本で+8.2～+10.4 mm/周へ減った。CCWは+14.9～+19.5 mm/周で残る。前回の生距離＋1 ms積算Z角度の値とは解析方法が異なるため直接混ぜない。

CW区間時間は8.677 → 8.710 s、CCWは8.773 → 8.773 s。ログ縮小でラップタイムが改善した結果ではない。

![コースプロット](trajectories.png)

![前回とのXドリフト比較](x_drift_comparison.png)

## 結論と限界

X+ドリフトはログを22バイトへ減らしても残る。CWでは今回小さいが、CCWは減っていないため、ログ負荷だけを原因とすることはできない。方向切替、温度、電圧、走行間ばらつきは交絡する。CCWの前回電圧8.15～8.06 Vに対し今回8.03～7.88 V、CWは前回8.30～8.19 Vに対し今回8.26～8.15 V。温度はメタデータの校正・終了値だけで、走行中の温度系列はない。

新ログは左右累積エンコーダ、ラインセンサー、imuAngle_Zを含まない。独立全幅ラインによるアンカー補正、生距離での独立再積分、記録済み1 ms角度での比較を実施できない。CROSS確定時刻の揺れも含まれるため、今回の数値から横滑りの開始地点や物理原因を断定しない。

閉路検証は12876のみ有効（goalMarkerX=42.56 mm）。他6本はclosureReason=8、ゴールマーカー位置Xが67.13～80.33 mm（CCW）または63.50～65.86 mm（CW）で±60 mm条件を超える。正常終了していても、これら6本は現行PATH/SHORTCUTの経路元として無効。ゴールマーカーXと停止時の最終Xは異なる。

![速度・角速度](speed_gyro.png)

再現: `python analysis/script/analyze_minimal_12872_12878.py`。health.csvにSHA256、列数、行数、電圧・温度、校正、閉路検証を保存。lap_intervals.csvに全区間、per_log.csvとgroup_comparison.csvに集計を保存。
'''
(OUT/'report.md').write_text(report,encoding='utf-8')
print(json.dumps(dict(groups=groups,new_logs=[r for r in summary if r['log']>=12872],health=[r for r in health if r['log']>=12872]),ensure_ascii=False,indent=2))
