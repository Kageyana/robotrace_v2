"""Offline wheel-vector reconstruction and deliberately extra cosine projection.

Encoder measures the longitudinal component. Lateral wheel motion below assumes
zero chassis lateral velocity and a rotation center at the axle midpoint.
The extra projection is a sensitivity experiment, not an identified slip model.
"""
from pathlib import Path
import csv, json, hashlib, os
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'analysis/wheel_projection_12866_12890'
OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(OUT / '.mplcache')
import analyze_yi_12838_12848 as m
plt = m.plt
geometry = json.loads((ROOT/'analysis/yi_12838_12848/axle_geometry.json').read_text())
A = geometry['wheelbase_mm']/2
LOG = Path('F:/Dropbox/Document/robotrace/Log/v2')
NUMS = list(range(12866,12872)) + list(range(12885,12891))
health, laps, summary, trajectories, excluded = [], [], [], {}, []

def write(name, rows):
    with (OUT/name).open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, rows[0].keys()); w.writeheader(); w.writerows(rows)

def project_again(longitudinal, lateral):
    magnitude = np.hypot(longitudinal, lateral)
    # cos of the acute slip angle; preserve reverse-wheel signs.
    cosine = np.divide(abs(longitudinal), magnitude,
                       out=np.ones_like(magnitude), where=magnitude>0)
    return longitudinal*cosine

# Checks at rest, straight travel, inner-wheel stop, reverse, and pure pivot.
assert np.allclose(project_again(np.array([0.,2.,-2.]), np.zeros(3)), [0,2,-2])
assert project_again(np.array([-2.]), np.array([1.]))[0] < 0
assert sum(project_again(np.array([2.,-2.]), np.array([1.,1.]))) == 0
for n in NUMS:
    path = LOG/f'{n}.csv'
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    meta = dict(z.split('=',1) for z in lines[0].split(',') if '=' in z)
    rows = list(csv.DictReader(lines[1:]))
    cols = m.COLS + ['imuAngle_Z', 'encTotalOptimal']
    values = {c:np.array([float(r[c]) for r in rows]) for c in cols}
    assert len(rows)==int(meta['logExpectedRows']), n
    if float(meta['emcStop'])!=0:
        excluded.append(dict(log=n,reason='emergency stop',emcStop=meta['emcStop'],rows=len(rows)))
        continue
    assert float(meta['optimalTrace'])==0 and meta['imuCalibrationValid']=='1'
    assert meta['imuCalibrationReadErrors']=='0' and meta['distanceScaleVerified']=='1'
    assert all(np.all(np.isfinite(v)) for v in values.values())
    assert np.all(np.diff(values['cntlog'])>0) and np.diff(values['cntlog']).max()<=11
    ppm = float(meta['encoderPulsePerMeter'])/1000
    dl = np.r_[0,np.diff(values['encTotalL'])]/ppm
    dr = np.r_[0,np.diff(values['encTotalR'])]/ppm
    ds = (dl+dr)/2
    assert np.all(ds[1:]>0)
    s = np.cumsum(ds)
    h = np.deg2rad(values['imuAngle_Z'])
    dh = np.r_[0,np.diff(h)]
    mid = np.r_[h[0],(h[:-1]+h[1:])/2]
    # Front/rear contact lateral motion has opposite signs, equal magnitude.
    lateral = A*dh
    # Reconstruct wheel ground vectors and take their longitudinal components.
    recovered = []
    for wheel in [dl,dr]:
        magnitude = np.hypot(wheel,lateral)
        recovered.append(magnitude*np.cos(np.arctan2(lateral,wheel)))
    exact = (recovered[0]+recovered[1])/2
    assert np.allclose(exact, ds, atol=1e-12, rtol=0)
    extra = (project_again(dl,lateral)+project_again(dr,lateral))/2
    d = dict(a=np.column_stack([values[c] for c in m.COLS]),s=s,meta=meta)
    m.D[n] = d
    d['line'], _ = m.anchor(d)
    assert len(d['line'])==6, (n,len(d['line']))
    bounds = m.axleanchors(d)
    assert 0<bounds[0]<bounds[-1]<s[-1]
    direction = 'CW' if h[-1]>h[0] else 'CCW'
    health.append(dict(log=n,direction=direction,rows=len(rows),normal_end=True,
        max_gap_ms=np.diff(values['cntlog']).max(),line_events=len(bounds),
        battery_V=meta['batteryVoltage_V'],sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for method, step in [('encoder_mean',ds),('vector_forward',exact),('extra_cosine',extra)]:
        p = np.c_[np.cumsum(step*np.sin(mid)), np.cumsum(step*np.cos(mid))]
        trajectories[n,method] = p
        anchor_xy = np.column_stack([np.interp(bounds,s,p[:,k]) for k in [0,1]])
        cumulative = np.cumsum(step)
        anchor_distance = np.interp(bounds,s,cumulative)
        for k in range(5):
            delta = anchor_xy[k+1]-anchor_xy[k]
            laps.append(dict(log=n,direction=direction,method=method,interval=k+1,
                X_mm=delta[0],Y_mm=delta[1],closure_mm=np.linalg.norm(delta),
                distance_mm=anchor_distance[k+1]-anchor_distance[k]))
        rs = [r for r in laps if r['log']==n and r['method']==method]
        summary.append(dict(log=n,direction=direction,method=method,
            X_mm=np.mean([r['X_mm'] for r in rs]),Y_mm=np.mean([r['Y_mm'] for r in rs]),
            mean_closure_mm=np.mean([r['closure_mm'] for r in rs]),
            mean_lap_distance_mm=np.mean([r['distance_mm'] for r in rs]),
            max_plot_change_mm=np.linalg.norm(p-trajectories[n,'encoder_mean'],axis=1).max()))
    if n==12888:
        write('12888_components.csv',[dict(time_ms=values['cntlog'][i],delta_yaw_deg=np.rad2deg(dh[i]),
            left_forward_mm=dl[i],right_forward_mm=dr[i],
            front_lateral_assumed_mm=lateral[i],rear_lateral_assumed_mm=-lateral[i],
            center_lateral_assumed_mm=0,forward_mean_mm=ds[i],extra_cosine_mean_mm=extra[i]) for i in range(len(ds))])

write('health.csv',health);write('lap_intervals.csv',laps);write('per_log.csv',summary)
group = []
for batch, nums in [('12866-12871',range(12866,12872)),('12885-12890',range(12885,12891))]:
    for direction in ['CW','CCW']:
        for method in ['encoder_mean','vector_forward','extra_cosine']:
            rs=[r for r in laps if r['log'] in nums and r['direction']==direction and r['method']==method]
            if not rs:continue
            group.append(dict(batch=batch,direction=direction,method=method,intervals=len(rs),
                X_mm=np.mean([r['X_mm'] for r in rs]),Y_mm=np.mean([r['Y_mm'] for r in rs]),
                mean_closure_mm=np.mean([r['closure_mm'] for r in rs]),
                mean_lap_distance_mm=np.mean([r['distance_mm'] for r in rs])))
write('summary.csv',group)
fig,axs=plt.subplots(1,3,figsize=(15,5),layout='constrained')
for ax,n in zip(axs,[12866,12869,12888]):
    for method,color in [('encoder_mean','#2471a3'),('extra_cosine','#d65f31')]:
        p=trajectories[n,method];ax.plot(p[:,0],p[:,1],color=color,label=method)
    ax.set(title=str(n),xlabel='X [mm]',ylabel='Y [mm]');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(OUT/'course_comparison.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for ax,key,label in zip(axs,['X_mm','mean_closure_mm'],['Mean X drift per lap [mm]','Mean closure magnitude per lap [mm]']):
    for method,color in [('encoder_mean','#2471a3'),('extra_cosine','#d65f31')]:
        rs=[r for r in summary if r['method']==method]
        ax.plot([r['log'] for r in rs],[r[key] for r in rs],'o-',color=color,label=method)
    ax.set(xlabel='Log number',ylabel=label);ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(OUT/'drift_comparison.png',dpi=160);plt.close(fig)
validation=dict(logs=[r['log'] for r in health],excluded=excluded,wheelbase_mm=2*A,forward_reconstruction='PASS: equals signed encoder mean within 1e-12 mm per interval',
    negative_encoder_sign='PASS',raw_inputs_unchanged=True,firmware_changed=False,
    measured_chassis_lateral_velocity=False,extra_cosine_physical_basis=False,
    reference='same physical CROSS inferred from eight line channels and CAD sensor offsets',
    integration='stored 1-ms yaw, midpoint heading, cumulative individual wheel differences')
(OUT/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
print(json.dumps(group,indent=2))
