"""Compare original XY with cumulative distance + recorded 1-ms yaw. Preserve inputs."""
from pathlib import Path
import csv,json,os,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/xy_recovery_12879_12884';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
summ=[]
fig,axes=plt.subplots(2,3,figsize=(12,9),layout='constrained')
for n,ax in zip(range(12879,12885),axes.flat):
    path=Path(f'F:/Dropbox/Document/robotrace/Log/v2/{n}.csv')
    lines=path.read_text(encoding='utf-8-sig').splitlines();meta=dict(v.split('=',1) for v in lines[0].split(',') if '=' in v)
    rr=list(csv.DictReader(lines[1:]));a={k:np.array([float(r[k]) for r in rr]) for k in ['cntlog','encTotalL','encTotalR','encTotalOptimal','encCurrentCorr_p','imuAngle_Z','gyroVal_Z','x','y','courseMarker']}
    assert len(rr)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0
    assert all(np.all(np.isfinite(z)) for z in a.values())
    assert np.all(np.diff(a['cntlog'])>0) and np.all(np.diff(a['encTotalOptimal'])>=0)
    s=a['encTotalOptimal']/58.019;ds=np.diff(np.r_[0,s]);h=np.deg2rad(a['imuAngle_Z']);hm=(np.r_[0,h[:-1]]+h)/2
    x=np.cumsum(ds*np.sin(hm));y=np.cumsum(ds*np.cos(hm))
    yaw=np.cumsum(a['gyroVal_Z']*np.diff(np.r_[0,a['cntlog']])/1000)
    oldDistance=a['encCurrentCorr_p']/58.019*np.diff(np.r_[0,a['cntlog']])
    oldX=np.cumsum(oldDistance*np.sin(np.deg2rad(yaw)));oldY=np.cumsum(oldDistance*np.cos(np.deg2rad(yaw)))
    oldError=float(np.max(np.hypot(oldX-a['x'],oldY-a['y'])))
    assert oldError<2, (n,oldError)
    ii=np.flatnonzero((a['courseMarker']==3)&(np.r_[0,a['courseMarker'][:-1]]!=3))
    assert len(ii)>=5
    row=dict(log=n,rows=len(rr),cross_count=len(ii),csv_X_per_lap_mm=float(np.diff(a['x'][ii]).mean()),
        recovered_X_per_lap_mm=float(np.diff(x[ii]).mean()),recovered_Y_per_lap_mm=float(np.diff(y[ii]).mean()),
        sparse_yaw_end_error_deg=float(yaw[-1]-a['imuAngle_Z'][-1]),original_xy_reproduction_max_error_mm=oldError,
        csv_goal_marker_X_mm=float(meta['goalMarkerX_mm']),
        recovered_goal_marker_X_mm=float(np.interp(float(meta['goalMarkerOnset_p'])/58.019,s,x)),
        expected_six_laps=(len(ii)==6),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    summ.append(row)
    with (OUT/f'{n}_recovered_xy.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['cntlog','x_original_mm','y_original_mm','x_recovered_mm','y_recovered_mm'])
        w.writerows(zip(a['cntlog'],a['x'],a['y'],x,y))
    ax.plot(a['x'],a['y'],lw=.7,c='#b74444',label='Original CSV')
    ax.plot(x,y,lw=.7,c='#2475ad',label='Cumulative distance + 1-ms yaw')
    ax.set_aspect('equal');ax.set(title=str(n),xlabel='X [mm]',ylabel='Y [mm]');ax.grid(alpha=.2)
axes.flat[0].legend(fontsize=7)
fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
with (OUT/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,summ[0].keys());w.writeheader();w.writerows(summ)
print(json.dumps(summ,ensure_ascii=False,indent=2))
