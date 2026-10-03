"""Check second-corner wheel motion without equating wheel odometry with ground truth."""
from pathlib import Path
import csv,os,json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/video_12888';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
path=Path('F:/Dropbox/Document/robotrace/Log/v2/12888.csv')
lines=path.read_text(encoding='utf-8-sig').splitlines();meta=dict(z.split('=',1) for z in lines[0].split(',') if '=' in z)
rows=list(csv.DictReader(lines[1:]));cols='cntlog encCurrentL encCurrentR encTotalL encTotalR gyroVal_Z imuAngle_Z encTotalOptimal x y'.split()
a={k:np.array([float(r[k]) for r in rows]) for k in cols}
assert len(rows)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0
assert all(np.all(np.isfinite(v)) for v in a.values()) and np.all(np.diff(a['cntlog'])>0)
assert np.max(np.diff(a['cntlog']))<=11
t=a['cntlog']/1000;pulse=float(meta['encoderPulsePerMeter'])/1000
mask=a['gyroVal_Z']>100;ed=np.diff(np.r_[False,mask,False].astype(int))
intervals=[(i,j) for i,j in zip(np.flatnonzero(ed==1),np.flatnonzero(ed==-1)) if j-i>=10 and a['gyroVal_Z'][i:j].mean()>250]
assert len(intervals)==6
summary=[]
for lap,(i,j) in enumerate(intervals,1):
    duration=t[j-1]-t[i]
    dl=(a['encTotalL'][j-1]-a['encTotalL'][i])/pulse
    dr=(a['encTotalR'][j-1]-a['encTotalR'][i])/pulse
    yaw=np.deg2rad(a['imuAngle_Z'][j-1]-a['imuAngle_Z'][i])
    k=i+np.argmin(a['encCurrentR'][i:j])
    summary.append(dict(lap=lap,start_s=t[i],end_s=t[j-1],outer_mean_mps=dl/duration/1000,
        inner_mean_mps=dr/duration/1000,average_forward_mps=(dl+dr)/2/duration/1000,
        effective_track_mm=(dl-dr)/yaw,inner_min_pulse_ms=a['encCurrentR'][k],min_time_s=t[k],
        outer_at_min_mps=a['encCurrentL'][k]/pulse,average_at_min_mps=(a['encCurrentL'][k]+a['encCurrentR'][k])/2/pulse))
with (OUT/'second_corner.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,summary[0].keys());w.writeheader();w.writerows(summary)
fig,axs=plt.subplots(2,1,figsize=(10,6),layout='constrained')
for lap,(i,j) in enumerate(intervals):
    q=(t>=t[i]-.15)&(t<=t[j-1]+.15)
    axs[0].plot(t[q]-t[i],a['encCurrentR'][q]/pulse,alpha=.6,label=f'lap {lap+1}')
i,j=intervals[0];q=(t>=t[i]-.15)&(t<=t[j-1]+.15)
for name,color in [('encCurrentL','#d65f31'),('encCurrentR','#2471a3')]:
    axs[1].plot(t[q]-t[i],a[name][q]/pulse,label='outer (left)' if name.endswith('L') else 'inner (right)',color=color)
axs[1].plot(t[q]-t[i],(a['encCurrentL'][q]+a['encCurrentR'][q])/2/pulse,color='black',label='left/right mean')
for ax in axs:ax.grid(alpha=.25);ax.set(xlabel='Time since second-corner threshold [s]',ylabel='Wheel speed [m/s]');ax.legend()
axs[0].set_title('12888: inner-wheel speeds during second corner, all 6 laps')
axs[1].set_title('First lap: wheel speeds and their mean (not measured ground speed)')
fig.savefig(OUT/'wheel_speeds.png',dpi=160);plt.close(fig)
(OUT/'validation.json').write_text(json.dumps(dict(rows=len(rows),second_corners=6,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),external_ground_speed=False,columns=cols),indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
