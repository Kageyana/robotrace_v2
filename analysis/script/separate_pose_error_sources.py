"""Separate yaw, relative encoder scale and timing sensitivity, offline only.

Closed laps constrain errors but do not identify physical sensor parameters.
Independent line-CROSS headings provide a second constraint with threshold
sensitivity. Keep source bytes unchanged and do not optimize firmware values.
"""
from pathlib import Path
import csv,json,hashlib,os
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'analysis/pose_error_sources_12866_12890';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import analyze_yi_12838_12848 as m
plt=m.plt
NUMS=list(range(12866,12872))+list(range(12885,12891))
LOG=Path('F:/Dropbox/Document/robotrace/Log/v2')
health=[];events=[];laps=[];curves=[];fits=[];excluded=[];D={};constrained=[]
def write(name,rows):
    with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
def evaluate(d,mode='baseline',value=0):
    h=d['h'].copy(); ds=d['ds'].copy()
    if mode=='gyro_bias_dps':h+=np.deg2rad(value)*(d['t']-d['t'][0])
    elif mode=='gyro_gain_pct':h=d['h'][0]+(h-d['h'][0])*(1+value/100)
    elif mode=='wheel_relative_pct':
        ds=((1-value/200)*d['dl']+(1+value/200)*d['dr'])/2
    elif mode=='yaw_time_shift_ms':
        h=np.interp(d['t']+value/1000,d['t'],h)
    elif mode=='distance_common_pct':ds*=1+value/100
    elif mode=='heading_right':pass
    elif mode=='heading_left':h=np.r_[h[0],h[:-1]]
    if mode not in ['heading_right','heading_left']:h=np.r_[h[0],(h[:-1]+h[1:])/2]
    p=np.c_[np.cumsum(ds*np.sin(h)),np.cumsum(ds*np.cos(h))]
    pa=np.column_stack([np.interp(d['bounds'],d['s'],p[:,j]) for j in [0,1]])
    return np.diff(pa,axis=0),p
for n in NUMS:
    path=LOG/f'{n}.csv'; lines=path.read_text(encoding='utf-8-sig').splitlines()
    meta=dict(z.split('=',1) for z in lines[0].split(',') if '=' in z)
    rr=list(csv.DictReader(lines[1:]));assert len(rr)==int(meta['logExpectedRows'])
    if float(meta['emcStop'])!=0:
        excluded.append(dict(log=n,reason='emergency stop',emcStop=meta['emcStop']));continue
    assert float(meta['optimalTrace'])==0 and meta['imuCalibrationValid']=='1' and meta['distanceScaleVerified']=='1'
    cols=m.COLS+['imuAngle_Z','encTotalOptimal','imuTemp_C','targetSpeed']
    v={c:np.array([float(r[c]) for r in rr]) for c in cols}
    assert all(np.all(np.isfinite(z)) for z in v.values())
    t=v['cntlog']/1000;assert np.all(np.diff(t)>0) and np.max(np.diff(t))<=.011001
    ppm=float(meta['encoderPulsePerMeter'])/1000
    dl=np.r_[0,np.diff(v['encTotalL'])]/ppm;dr=np.r_[0,np.diff(v['encTotalR'])]/ppm
    ds=(dl+dr)/2;s=np.cumsum(ds);assert np.all(ds[1:]>0)
    md=dict(a=np.column_stack([v[c] for c in m.COLS]),s=s,meta=meta)
    m.D[n]=md;md['line'],_=m.anchor(md);assert len(md['line'])==6
    bounds=m.axleanchors(md);ar=m.axlerows[-6:];cross=np.array([r['cross_angle_deg'] for r in ar])
    h=np.deg2rad(v['imuAngle_Z']);sgn=1 if h[-1]>h[0] else -1
    ah=np.interp(bounds,s,np.rad2deg(h));at=np.interp(bounds,s,t)
    # s_cross(x) = intercept + x*tan(heading relative to CROSS normal).
    yawerr=np.diff(ah)-sgn*360
    independent_yawerr=yawerr-np.diff(cross)
    d=dict(t=t,h=h,s=s,ds=ds,dl=dl,dr=dr,bounds=bounds,direction='CW' if sgn>0 else 'CCW',meta=meta)
    D[n]=d;delta,p=evaluate(d);d['baseline']=delta;d['p']=p
    encopt_rel=v['encTotalOptimal']-v['encTotalOptimal'][0]
    rawavg=(v['encTotalL']-v['encTotalL'][0]+v['encTotalR']-v['encTotalR'][0])/2
    health.append(dict(log=n,direction=d['direction'],rows=len(rr),battery_V=meta['batteryVoltage_V'],
        temp_start_C=v['imuTemp_C'][0],temp_end_C=v['imuTemp_C'][-1],
        max_gap_ms=np.max(np.diff(t))*1000,mean_target_mps=np.mean(v['targetSpeed']/ppm),
        encopt_minus_rawavg_min_p=np.min(encopt_rel-rawavg),encopt_minus_rawavg_max_p=np.max(encopt_rel-rawavg),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for k in range(6):
        events.append(dict(log=n,direction=d['direction'],event=k+1,time_s=at[k],
            imu_heading_deg=ah[k],line_heading_deg=cross[k],line_fit_rms_mm=ar[k]['channel_fit_rms_mm'],
            channels=ar[k]['channels']))
    for k in range(5):
        sl=(s>=bounds[k])&(s<=bounds[k+1])
        # Effective track from integrated wheel difference; not physical track truth.
        ldist=np.interp(bounds[k+1],s,np.cumsum(dl))-np.interp(bounds[k],s,np.cumsum(dl))
        rdist=np.interp(bounds[k+1],s,np.cumsum(dr))-np.interp(bounds[k],s,np.cumsum(dr))
        laps.append(dict(log=n,direction=d['direction'],interval=k+1,X_mm=delta[k,0],Y_mm=delta[k,1],
            closure_mm=np.linalg.norm(delta[k]),duration_s=at[k+1]-at[k],
            imu_turn_error_deg=yawerr[k],line_heading_change_deg=np.diff(cross)[k],
            imu_minus_line_turn_error_deg=independent_yawerr[k],
            effective_track_mm=(ldist-rdist)/np.deg2rad(ah[k+1]-ah[k]),
            temperature_mean_C=np.mean(v['imuTemp_C'][sl])))
    # Calibration hypotheses constrained by independent CROSS headings,
    # not by choosing a parameter that merely makes the XY loop close.
    independent_bias=-np.sum(independent_yawerr)/(at[-1]-at[0])
    independent_gain=-np.sum(independent_yawerr)/(ah[-1]-ah[0])*100
    for mode,value in [('gyro_bias_dps',independent_bias),('gyro_gain_pct',independent_gain)]:
        z,_=evaluate(d,mode,value)
        constrained.append(dict(log=n,direction=d['direction'],parameter=mode,value=value,
            baseline_X_mm=delta[:,0].mean(),corrected_X_mm=z[:,0].mean(),
            baseline_closure_rms_mm=np.sqrt(np.mean(np.sum(delta**2,axis=1))),
            corrected_closure_rms_mm=np.sqrt(np.mean(np.sum(z**2,axis=1)))))
    for mode,grid in [('gyro_bias_dps',np.linspace(-.3,.3,121)),('gyro_gain_pct',np.linspace(-.5,.5,101)),
                      ('wheel_relative_pct',np.linspace(-2,2,81)),('yaw_time_shift_ms',np.linspace(-20,20,81))]:
        costs=[]
        for value in grid:
            z,_=evaluate(d,mode,value);cost=np.mean(np.sum(z*z,axis=1));costs.append(cost)
            curves.append(dict(log=n,direction=d['direction'],parameter=mode,value=value,
                mean_X_mm=np.mean(z[:,0]),mean_Y_mm=np.mean(z[:,1]),rms_closure_mm=np.sqrt(cost),
                rms_change_from_baseline_mm=np.sqrt(np.mean(np.sum((z-delta)**2,axis=1)))))
        j=int(np.argmin(costs));value=grid[j]
        fits.append(dict(log=n,direction=d['direction'],parameter=mode,best_value=value,
            baseline_rms_mm=np.sqrt(np.mean(np.sum(delta*delta,axis=1))),best_rms_mm=np.sqrt(costs[j]),
            optimum_at_scan_boundary=bool(j in [0,len(grid)-1])))
    for threshold in [800,1200,1500]:
        m.axleanchors(md,threshold);newar=m.axlerows[-6:]
        cb=np.array([r['axle_cross_mm'] for r in newar]);ca=np.array([r['cross_angle_deg'] for r in newar])
        ch=np.interp(cb,s,np.rad2deg(h))
        for k in range(5):
            events.append(dict(log=n,direction=d['direction'],event=f'interval{k+1}_threshold{threshold}',time_s='',
                imu_heading_deg=np.diff(ch)[k]-sgn*360,line_heading_deg=np.diff(ca)[k],
                line_fit_rms_mm='',channels=''))
write('health.csv',health);write('cross_events.csv',events);write('lap_diagnostics.csv',laps)
write('sensitivity.csv',curves);write('closure_only_optima.csv',fits)
write('cross_constrained_yaw.csv',constrained)
# Summaries keep old and new batteries/builds separate.
group=[];sensitivity=[]
for batch,ns in [('12866-12871',range(12866,12872)),('12885-12890',range(12885,12891))]:
    for direction in ['CW','CCW']:
        rows=[r for r in laps if r['log'] in ns and r['direction']==direction]
        if not rows:continue
        group.append(dict(batch=batch,direction=direction,intervals=len(rows),
            X_mm=np.mean([r['X_mm'] for r in rows]),Y_mm=np.mean([r['Y_mm'] for r in rows]),
            imu_turn_error_mean_deg=np.mean([r['imu_turn_error_deg'] for r in rows]),
            imu_minus_line_turn_error_mean_deg=np.mean([r['imu_minus_line_turn_error_deg'] for r in rows]),
            imu_minus_line_turn_error_std_deg=np.std([r['imu_minus_line_turn_error_deg'] for r in rows]),
            effective_track_mean_mm=np.mean([r['effective_track_mm'] for r in rows])))
        for mode,values in [('gyro_bias_dps',[-.1,.1]),('gyro_gain_pct',[-.1,.1]),
                            ('wheel_relative_pct',[-1,1]),('yaw_time_shift_ms',[-1,1,-5,5,-10,10]),
                            ('distance_common_pct',[-1,1]),('heading_left',[0]),('heading_right',[0])]:
            for value in values:
                zz=[];changes=[]
                for n,d in D.items():
                    if n not in ns or d['direction']!=direction:continue
                    z,_=evaluate(d,mode,value);zz.extend(z);changes.extend(z-d['baseline'])
                zz=np.array(zz);changes=np.array(changes)
                sensitivity.append(dict(batch=batch,direction=direction,parameter=mode,value=value,
                    mean_X_mm=zz[:,0].mean(),mean_Y_mm=zz[:,1].mean(),
                    mean_closure_mm=np.linalg.norm(zz,axis=1).mean(),
                    rms_change_from_baseline_mm=np.sqrt(np.mean(np.sum(changes**2,axis=1)))))
write('summary.csv',group);write('sensitivity_summary.csv',sensitivity)
threshold_summary=[]
for n,d in D.items():
    for threshold in [800,1000,1200,1500]:
        if threshold==1000:
            rs=[r for r in laps if r['log']==n]
            error=[r['imu_minus_line_turn_error_deg'] for r in rs]
        else:
            rs=[r for r in events if r['log']==n and str(r['event']).endswith(f'_threshold{threshold}')]
            error=[r['imu_heading_deg']-r['line_heading_deg'] for r in rs]
        threshold_summary.append(dict(log=n,threshold=threshold,mean_turn_residual_deg=np.mean(error),std_turn_residual_deg=np.std(error)))
write('line_threshold_sensitivity.csv',threshold_summary)
for row in health:
    assert hashlib.sha256((LOG/f"{row['log']}.csv").read_bytes()).hexdigest()==row['sha256']
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,mode in zip(axs.flat,['gyro_bias_dps','gyro_gain_pct','wheel_relative_pct','yaw_time_shift_ms']):
    for n in [12866,12869,12888]:
        rows=[r for r in curves if r['log']==n and r['parameter']==mode]
        ax.plot([r['value'] for r in rows],[r['rms_closure_mm'] for r in rows],label=str(n))
    ax.set(xlabel=mode,ylabel='RMS closure [mm]');ax.grid(alpha=.2);ax.legend()
fig.savefig(OUT/'sensitivity.png',dpi=150);plt.close(fig)
(OUT/'validation.json').write_text(json.dumps(dict(logs=list(D),excluded=excluded,
    original_csv_unchanged=True,firmware_changed=False,ground_truth_xy_available=False,
    heading_reference='line-CROSS 8-channel geometry fit, threshold sensitivity included',
    time_shift_definition='positive = use IMU heading from later logged time',
    relative_scale_definition='value pct: left multiplier 1-value/200; right multiplier 1+value/200',
    warning='closure-only optima are not calibration values; sources may be confounded'),indent=2),encoding='utf-8')
print(json.dumps(group,indent=2))
print('Independent CROSS constrained yaw hypotheses:');print(json.dumps(constrained,indent=2))
