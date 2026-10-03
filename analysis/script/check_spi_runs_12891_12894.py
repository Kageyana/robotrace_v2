"""Validate newly found runs; changed-SPI run membership requires user evidence.

buildTime is compiled in SDcard.c and does not reliably identify a main.c-only
rebuild. Do not treat header timestamps as proof of SPI clock.
"""
from pathlib import Path
import csv,json,hashlib,os
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/spi_runs_12891_12894';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import analyze_yi_12838_12848 as m
plt=m.plt;LOG=Path('F:/Dropbox/Document/robotrace/Log/v2')
nums=[12885,12886,12887,12888,12890,12869,12870,12871,12891,12892,12893,12894]
summary=[];laps=[];P={}
for n in nums:
    path=LOG/f'{n}.csv';lines=path.read_text(encoding='utf-8-sig').splitlines()
    meta=dict(z.split('=',1) for z in lines[0].split(',') if '=' in z);rr=list(csv.DictReader(lines[1:]))
    cols=m.COLS+['imuAngle_Z','imuTemp_C','gyroVal_X','gyroVal_Y','acceleVal_X','acceleVal_Y','acceleVal_Z','encTotalOptimal','targetSpeed']
    v={c:np.array([float(r[c]) for r in rr]) for c in cols}
    assert len(rr)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0
    assert float(meta['optimalTrace'])==0 and meta['imuCalibrationValid']=='1' and meta['distanceScaleVerified']=='1'
    assert all(np.all(np.isfinite(z)) for z in v.values())
    t=v['cntlog']/1000;assert np.all(np.diff(t)>0), n
    dl=np.r_[0,np.diff(v['encTotalL'])]/58.019;dr=np.r_[0,np.diff(v['encTotalR'])]/58.019
    ds=(dl+dr)/2;s=np.cumsum(ds);assert np.all(ds[1:]>0)
    # Distance-triggered logging: speed changes time gaps, not target distance.
    assert np.max(ds[1:])<7, (n,'unexpected distance gap',np.max(ds[1:]))
    d=dict(a=np.column_stack([v[c] for c in m.COLS]),s=s,meta=meta);m.D[n]=d
    d['line'],_=m.anchor(d);assert len(d['line'])==6
    bounds=m.axleanchors(d);cross=np.array([r['cross_angle_deg'] for r in m.axlerows[-6:]])
    h=np.deg2rad(v['imuAngle_Z']);sgn=1 if h[-1]>h[0] else -1;direction='CW' if sgn>0 else 'CCW'
    mid=np.r_[h[0],(h[:-1]+h[1:])/2];p=np.c_[np.cumsum(ds*np.sin(mid)),np.cumsum(ds*np.cos(mid))]
    ah=np.interp(bounds,s,np.rad2deg(h));xy=np.column_stack([np.interp(bounds,s,p[:,j]) for j in [0,1]])
    delta=np.diff(xy,axis=0);error=np.diff(ah)-sgn*360-np.diff(cross)
    a0=np.interp(bounds,s,t);active=(s>=bounds[0])&(s<=bounds[-1]);dt=np.r_[0,np.diff(t)]
    sparse=np.cumsum(np.deg2rad(v['gyroVal_Z'])*dt)+h[0]
    for k in range(5):
        laps.append(dict(log=n,direction=direction,interval=k+1,X_mm=delta[k,0],Y_mm=delta[k,1],
            closure_mm=np.linalg.norm(delta[k]),imu_minus_line_turn_error_deg=error[k],lap_time_s=a0[k+1]-a0[k]))
    sample=dict(log=n,direction=direction,rows=len(rr),build_time=meta['buildTime'],battery_V=meta['batteryVoltage_V'],
        mean_X_mm=delta[:,0].mean(),mean_Y_mm=delta[:,1].mean(),mean_closure_mm=np.linalg.norm(delta,axis=1).mean(),
        mean_turn_residual_deg=error.mean(),mean_speed_mps=ds[active].sum()/dt[active].sum()/1000,
        target_speed_min_mps=np.min(v['targetSpeed']/58.019),target_speed_max_mps=np.max(v['targetSpeed']/58.019),
        distance_step_min_mm=np.min(ds[1:]),distance_step_max_mm=np.max(ds[1:]),last_time_s=t[-1],
        temp_start_C=v['imuTemp_C'][0],temp_end_C=v['imuTemp_C'][-1],
        max_gap_ms=np.max(np.diff(t))*1000,sparse_yaw_end_error_deg=np.rad2deg(sparse[-1]-h[-1]),
        gyro_peak_abs_dps=np.max(abs(v['gyroVal_Z'])),gyro_X_rms_dps=np.sqrt(np.mean(v['gyroVal_X'][active]**2)),
        gyro_Y_rms_dps=np.sqrt(np.mean(v['gyroVal_Y'][active]**2)),
        accel_Z_mean_g=np.mean(v['acceleVal_Z'][active]),accel_Z_std_g=np.std(v['acceleVal_Z'][active]),
        temperature_invalid_rows=int(np.count_nonzero(v['imuTemp_C']==-999)),
        imu_cal_errors=meta['imuCalibrationReadErrors'],sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    summary.append(sample);P[n]=(p,s,bounds)
def write(name,rows):
    with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
write('per_log.csv',summary);write('lap_intervals.csv',laps)
fig,axs=plt.subplots(2,2,figsize=(10,9),layout='constrained')
for ax,n in zip(axs.flat,range(12891,12895)):
    p,s,bounds=P[n]
    for k in range(5):
        q=(s>=bounds[k])&(s<=bounds[k+1]);ax.plot(p[q,0],p[q,1],label=f'lap interval {k+1}')
    ax.set(title=str(n),xlabel='X [mm]',ylabel='Y [mm]');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7)
fig.savefig(OUT/'courses.png',dpi=150);plt.close(fig)
for row in summary:assert hashlib.sha256((LOG/f"{row['log']}.csv").read_bytes()).hexdigest()==row['sha256']
(OUT/'validation.json').write_text(json.dumps(dict(normal_runs=nums,new_logs=list(range(12891,12895)),
    exact_rows=True,monotonic_time=True,finite_values=True,independent_cross_events=6,input_hashes_unchanged=True,
    spi_changed_run_membership='12891-12894 all, confirmed by user',runtime_SPI_read_errors_available=False,
    ISR_execution_time_available=False,register_readback_available=False),indent=2),encoding='utf-8')
print(json.dumps([r for r in summary if r['log']>=12891],indent=2))
