"""Tire-center speeds under a prescribed midpoint ICR, not ground truth.

Two independent estimates: observed encoder / geometric cosine, and IMU
angular rate times distance to prescribed ICR. Their disagreement tests the
assumed geometry/rolling constraints. Instantaneous inputs match calcROC.
"""
from pathlib import Path
import csv,json,os,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'analysis/tire_ground_speed_12888';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
path=Path('F:/Dropbox/Document/robotrace/Log/v2/12888.csv')
lines=path.read_text(encoding='utf-8-sig').splitlines()
meta=dict(z.split('=',1) for z in lines[0].split(',') if '=' in z)
rr=list(csv.DictReader(lines[1:]))
cols='cntlog encCurrentN encCurrentL encCurrentR gyroVal_Z'.split()
d={c:np.array([float(r[c]) for r in rr]) for c in cols}
assert len(rr)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0
assert all(np.all(np.isfinite(v)) for v in d.values()) and np.all(np.diff(d['cntlog'])>0)
geom=json.loads((ROOT/'analysis/yi_12838_12848/axle_geometry.json').read_text())
a=geom['wheelbase_mm']/2;b=109.1/2;scale=float(meta['encoderPulsePerMeter'])/1000
t=d['cntlog']/1000
omega=np.deg2rad(d['gyroVal_Z'])
center=d['encCurrentN']/scale*1000 # mm/s, same sampled input as calcROC
mask=d['gyroVal_Z']>100;ed=np.diff(np.r_[False,mask,False].astype(int))
windows=[(i,j) for i,j in zip(np.flatnonzero(ed==1),np.flatnonzero(ed==-1)) if j-i>=10 and d['gyroVal_Z'][i:j].mean()>250]
assert len(windows)==6
data=[];summary=[]
for lap,(i,j) in enumerate(windows,1):
    R=center[i:j]/omega[i:j]
    for side,q,enc in [('outer_left',R+b,d['encCurrentL'][i:j]/scale),('inner_right',R-b,d['encCurrentR'][i:j]/scale)]:
        radius=np.hypot(q,a)
        angle=np.arctan2(a,abs(q))
        cosine=abs(q)/radius
        valid=cosine>=.05 # numerical guard, not a correction or model validity claim
        inverse=np.divide(abs(enc),cosine,out=np.full(len(q),np.nan),where=valid)
        rigid=abs(omega[i:j])*radius/1000
        expected_forward=omega[i:j]*q/1000
        for k in range(len(q)):
            data.append(dict(lap=lap,time_s=t[i+k],side=side,center_radius_mm=R[k],
                tire_radius_mm=radius[k],slip_angle_acute_deg=np.rad2deg(angle[k]),
                encoder_forward_mps=enc[k],cosine=cosine[k],
                encoder_div_cos_mps=inverse[k] if valid[k] else '',
                gyro_times_radius_mps=rigid[k],expected_forward_mps=expected_forward[k],
                forward_residual_mps=enc[k]-expected_forward[k],inverse_valid=bool(valid[k])))
        summary.append(dict(lap=lap,side=side,samples=len(q),invalid_inverse_samples=int(np.count_nonzero(~valid)),
            angle_median_deg=np.median(np.rad2deg(angle)),encoder_mean_mps=np.mean(enc),
            encoder_div_cos_mean_valid_mps=np.mean(inverse[valid]),gyro_times_radius_mean_mps=np.mean(rigid),
            forward_residual_rms_mps=np.sqrt(np.mean((enc-expected_forward)**2))))
def write(name,rows):
    with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
write('components.csv',data);write('second_corner_summary.csv',summary)
fig,axs=plt.subplots(2,1,figsize=(10,7),layout='constrained')
for ax,side in zip(axs,['outer_left','inner_right']):
    rs=[r for r in data if r['lap']==1 and r['side']==side]
    time=[r['time_s'] for r in rs]
    ax.plot(time,[r['encoder_forward_mps'] for r in rs],label='encoder forward')
    ax.plot(time,[r['encoder_div_cos_mps'] if r['inverse_valid'] else np.nan for r in rs],label='encoder / cosine')
    ax.plot(time,[r['gyro_times_radius_mps'] for r in rs],label='IMU rate * assumed radius',ls='--')
    ax.set(title=side,xlabel='Time [s]',ylabel='Speed [m/s]');ax.grid(alpha=.2);ax.legend()
fig.savefig(OUT/'first_corner_speeds.png',dpi=160);plt.close(fig)
validation=dict(input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),normal_end=True,six_corners=True,
    wheelbase_mm=2*a,nominal_tire_center_track_mm=2*b,track_source='existing STEP analysis documented in analysis/video_12888/report.md',
    midpoint_lateral_speed_assumed_zero=True,ICR_fore_aft_at_midpoint=True,no_longitudinal_slip_required_for_encoder_inversion=True,
    inverse_cosine_guard=.05,firmware_changed=False,measured_ground_speed=False,
    note='front/rear speed magnitudes equal under assumption; lateral directions opposite. Radius straight clamp not applied within selected corners.')
(OUT/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
print('Representative stopped inner-wheel row:')
print(json.dumps([r for r in data if r['lap']==1 and abs(r['time_s']-2.321)<1e-8],indent=2))
