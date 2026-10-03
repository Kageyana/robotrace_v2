"""Compare initialization-fixed runs; explicitly unwrap U16 log time for diagnostics.
Raw logs remain invalid PATH sources; no source data is changed.
"""
from pathlib import Path
import csv,json,hashlib,os,argparse
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--new-logs',type=int,nargs='+',default=[12895,12896,12897])
parser.add_argument('--output',default='analysis/imu_init_runs_12895_12897')
args=parser.parse_args();new_logs=args.new_logs
OUT=ROOT/args.output;OUT.mkdir(parents=True,exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import analyze_yi_12838_12848 as m
plt=m.plt;LOG=Path('F:/Dropbox/Document/robotrace/Log/v2')
nums=[12885,12886,12887,12888,12890,12869,12870,12871,12891,12892,12893,12894]+list(dict.fromkeys([12895,12896,12897]+new_logs))
summary=[];laps=[];P={};anchor_rejections=[]
for n in nums:
    path=LOG/f'{n}.csv';lines=path.read_text(encoding='utf-8-sig').splitlines()
    meta=dict(z.split('=',1) for z in lines[0].split(',') if '=' in z);rr=list(csv.DictReader(lines[1:]))
    cols=m.COLS+['imuAngle_Z','imuTemp_C','gyroVal_X','gyroVal_Y','acceleVal_X','acceleVal_Y','acceleVal_Z','encTotalOptimal','targetSpeed']
    v={c:np.array([float(r[c]) for r in rr]) for c in cols}
    assert len(rr)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0
    assert meta['imuCalibrationValid']=='1'
    mode=int(float(meta['optimalTrace']))
    assert all(np.all(np.isfinite(z)) for z in v.values())
    raw_t=v['cntlog']; wraps=np.diff(raw_t)<-65000; t=(raw_t+np.r_[0,np.cumsum(wraps)]*65536)/1000; assert np.all(np.diff(t)>0), n
    dl=np.r_[0,np.diff(v['encTotalL'])]/58.019;dr=np.r_[0,np.diff(v['encTotalR'])]/58.019
    ds=(dl+dr)/2;s=np.cumsum(ds);assert np.all(ds[1:]>0)
    # Distance-triggered logging: speed changes time gaps, not target distance.
    # Allow speed-dependent 1-ms sampling overshoot; corrected-mode raw spacing is diagnostic.
    if mode==0: assert np.max(ds[1:])<float(meta['logDistanceTargetMm'])+2, (n,'unexpected distance gap',np.max(ds[1:]))
    d=dict(a=np.column_stack([v[c] for c in m.COLS]),s=s,meta=meta);m.D[n]=d
    d['line'],_=m.anchor(d);assert len(d['line'])>=2, (n,'insufficient independent CROSS events')
    cross_count=len(d['line'])
    events=d['line'].copy();bounds=[];cross=[];event_indices=[]
    for event_index,event in enumerate(events):
        d['line']=np.array([event])
        try:
            bound=m.axleanchors(d)[0]
        except AssertionError as exc:
            anchor_rejections.append(dict(log=n,event=event_index+1,reason=str(exc)));continue
        bounds.append(bound);cross.append(m.axlerows[-1]['cross_angle_deg']);event_indices.append(event_index)
    d['line']=events;bounds=np.array(bounds);cross=np.array(cross);spans=np.diff(event_indices)
    assert len(bounds)>=2, (n,'insufficient valid CROSS fits')
    h=np.deg2rad(v['imuAngle_Z']);sgn=1 if h[-1]>h[0] else -1;direction='CW' if sgn>0 else 'CCW'
    mid=np.r_[h[0],(h[:-1]+h[1:])/2];p=np.c_[np.cumsum(ds*np.sin(mid)),np.cumsum(ds*np.cos(mid))]
    ah=np.interp(bounds,s,np.rad2deg(h));xy=np.column_stack([np.interp(bounds,s,p[:,j]) for j in [0,1]])
    delta=np.diff(xy,axis=0);error=(np.diff(ah)-sgn*360*spans-np.diff(cross))/spans
    a0=np.interp(bounds,s,t);active=(s>=bounds[0])&(s<=bounds[-1]);dt=np.r_[0,np.diff(t)]
    sparse=np.cumsum(np.deg2rad(v['gyroVal_Z'])*dt)+h[0]
    for k in range(len(bounds)-1):
        laps.append(dict(log=n,mode=mode,direction=direction,interval=k+1,lap_span=int(spans[k]),X_mm=delta[k,0]/spans[k],Y_mm=delta[k,1]/spans[k],
            closure_mm=np.linalg.norm(delta[k])/spans[k],imu_minus_line_turn_error_deg=error[k],lap_time_s=(a0[k+1]-a0[k])/spans[k]))
    sample=dict(log=n,mode=mode,direction=direction,rows=len(rr),build_time=meta['buildTime'],battery_V=meta['batteryVoltage_V'],
        mean_X_mm=delta[:,0].sum()/spans.sum(),mean_Y_mm=delta[:,1].sum()/spans.sum(),mean_closure_mm=np.linalg.norm(delta,axis=1).sum()/spans.sum(),
        mean_turn_residual_deg=np.average(error,weights=spans),mean_speed_mps=ds[active].sum()/dt[active].sum()/1000,
        target_speed_min_mps=np.min(v['targetSpeed']/58.019),target_speed_max_mps=np.max(v['targetSpeed']/58.019),
        distance_step_min_mm=np.min(ds[1:]),distance_step_max_mm=np.max(ds[1:]),last_time_s=t[-1],
        temp_start_C=v['imuTemp_C'][0],temp_end_C=v['imuTemp_C'][-1],
        max_gap_ms=np.max(np.diff(t))*1000,sparse_yaw_end_error_deg=np.rad2deg(sparse[-1]-h[-1]),
        gyro_peak_abs_dps=np.max(abs(v['gyroVal_Z'])),gyro_X_rms_dps=np.sqrt(np.mean(v['gyroVal_X'][active]**2)),
        gyro_Y_rms_dps=np.sqrt(np.mean(v['gyroVal_Y'][active]**2)),
        accel_Z_mean_g=np.mean(v['acceleVal_Z'][active]),accel_Z_std_g=np.std(v['acceleVal_Z'][active]),
        temperature_invalid_rows=int(np.count_nonzero(v['imuTemp_C']==-999)),
        independent_cross_events=cross_count,valid_cross_fits=len(bounds),lap_span_total=int(spans.sum()),time_wraps=int(sum(wraps)),distance_scale_verified=meta['distanceScaleVerified'],distance_scale_error_p=meta['distanceScaleError_p'],closure_reason=meta['closureReason'],imu_cal_errors=meta['imuCalibrationReadErrors'],sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    summary.append(sample);P[n]=(p,s,bounds)
def write(name,rows):
    with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
write('per_log.csv',summary);write('lap_intervals.csv',laps)
if anchor_rejections:write('anchor_rejections.csv',anchor_rejections)
plot_rows=(len(new_logs)+2)//3
fig,axs=plt.subplots(plot_rows,3,figsize=(15,4.5*plot_rows),layout='constrained',squeeze=False)
for ax,n in zip(axs.flat,new_logs):
    p,s,bounds=P[n]
    for k in range(len(bounds)-1):
        q=(s>=bounds[k])&(s<=bounds[k+1]);ax.plot(p[q,0],p[q,1],label=f'lap interval {k+1}')
    ax.set(title=str(n),xlabel='X [mm]',ylabel='Y [mm]');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7)
for ax in list(axs.flat)[len(new_logs):]: ax.set_visible(False)
fig.savefig(OUT/'courses.png',dpi=150);plt.close(fig)
for row in summary:assert hashlib.sha256((LOG/f"{row['log']}.csv").read_bytes()).hexdigest()==row['sha256']
(OUT/'validation.json').write_text(json.dumps(dict(normal_runs=nums,new_logs=new_logs,
    raw_new_log_time_monotonic=all(r['time_wraps']==0 for r in summary if r['log'] in new_logs),raw_new_logs_valid_path_sources=all(r['closure_reason']=='0' for r in summary if r['log'] in new_logs),exact_rows=True,monotonic_time_after_explicit_U16_unwrap=True,finite_values=True,independent_cross_events={r['log']:r['independent_cross_events'] for r in summary},input_hashes_unchanged=True,
    anchor_rejections=anchor_rejections,run_membership='Newly saved logs after user run notification; see per_log.csv for build times',runtime_SPI_read_errors_available=False,
    ISR_execution_time_available=False,register_readback_available=False),indent=2),encoding='utf-8')
print(json.dumps([r for r in summary if r['log'] in new_logs],indent=2))
