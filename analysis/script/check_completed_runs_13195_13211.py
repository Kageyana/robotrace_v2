"""ゲイン調整後ログの正常終了・設定・センサー変化を記録する。"""
from pathlib import Path
import csv
import hashlib
import json
import os
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'analysis/completed_runs_13195_13211'
OUT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault('MPLCONFIGDIR', str(OUT/'mpl-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SOURCE = Path(r'F:\Dropbox\Document\robotrace\Log\v2')
records, data, hashes = [], {}, {}
for n in [13194, *range(13195, 13212)]:
    path = SOURCE/f'{n}.csv'
    hashes[n] = hashlib.sha256(path.read_bytes()).hexdigest()
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    meta = dict(x.split('=', 1) for x in lines[0].split(',') if '=' in x)
    header = lines[1].rstrip(',').split(',')
    rows = [r[:-1] if r[-1] == '' else r for r in csv.reader(lines[2:]) if r]
    assert len(header) == len(set(header)) == 43
    assert all(len(r) == len(header) for r in rows)
    a = np.asarray(rows, dtype=float)
    assert np.isfinite(a).all()
    assert len(rows) == int(meta['logExpectedRows'])
    c = {key: a[:, i] for i, key in enumerate(header)}
    assert np.all(np.diff(c['cntlog']) > 0)
    distance = c['encTotalOptimal']/58.019
    ds = np.diff(distance)
    # 5mm刻みの残余持越しに対して、現在速度による1ms量子化を許容する。
    quantization_mm = np.max(np.abs(c['encCurrentN']))/58.019 + .5
    backwards = np.flatnonzero(ds <= 0)
    if backwards.size:
        # DISTANCE/SLIPはマーカーで補正済み距離が戻る。ログ欠落と区別する。
        assert float(meta['optimalTrace']) == 2
        assert np.all(c['courseMarker'][backwards] == 0)
        assert np.all(c['courseMarker'][backwards+1] > 0)
        assert ds.min() > -200  # CORR_STEP_MAX_MM
    assert np.max(np.abs(ds[ds>0]-5)) < quantization_mm, (n, ds.min(), ds.max(), quantization_mm)
    ls = a[:, [header.index(f'lSensorCari{i}') for i in range(10)]]
    mask = (distance >= 1050) & (distance < 1170)
    record = dict(log=n, mode=meta['optimalTrace'], requested_mode=meta['requestedMode'],
        auto_run=int(meta['autoRunNumber']), stop=int(float(meta['emcStop'])), rows=len(rows),
        recorded_end_s=float(c['cntlog'][-1]/1000), battery_V=float(meta['batteryVoltage_V']),
        kp=float(meta['lineTraceOmegaFBCtrl.kp']), ki=float(meta['lineTraceOmegaFBCtrl.ki']),
        kd=float(meta['lineTraceOmegaFBCtrl.kd']), closure_valid=int(meta['closureValid']),
        closure_reason=int(meta['closureReason']), calibration_valid=int(meta['lineSensorCalibrationValid']),
        imu_valid=int(meta['imuCalibrationValid']), unique_vectors=len(np.unique(ls,axis=0)),
        distance_step_min_mm=float(ds.min()), distance_step_max_mm=float(ds.max()),
        marker_distance_backward_events=int(backwards.size),
        curve_target_yaw_median=float(np.median(c['targetAngularvelo'][mask])),
        curve_actual_yaw_median=float(np.median(c['gyroVal_Z'][mask])),
        curve_steering_median=float(np.median(c['lineTraceCtrl'][mask])))
    records.append(record)
    data[n] = (c, ls, distance, meta)
with (OUT/'summary.csv').open('w', encoding='utf-8-sig', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(records[0])); w.writeheader(); w.writerows(records)

for n in [13195, 13207, 13210]:
    c, ls, distance, meta = data[n]
    t = c['cntlog']/1000
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), constrained_layout=True)
    axes[0,0].plot(c['x'], c['y']); axes[0,0].axis('equal')
    axes[0,0].set(title=f'{n}: {meta["requestedMode"]}, stop={meta["emcStop"]}', xlabel='x (mm)', ylabel='y (mm)')
    for key in ['targetSpeed', 'encCurrentN']:
        axes[0,1].plot(t, c[key]/58.019, label=key)
    axes[0,1].set(title='Speed tracking', xlabel='time (s)', ylabel='m/s')
    for key in ['targetAngularvelo', 'gyroVal_Z']:
        axes[1,0].plot(t,c[key],label=key)
    axes[1,0].set(title='Yaw tracking',xlabel='time (s)',ylabel='deg/s')
    axes[1,1].pcolormesh(t,np.arange(10),ls.T,shading='nearest',vmin=0,vmax=4095)
    axes[1,1].set(title='Line sensors: 0=left, 9=right',xlabel='time (s)',ylabel='index')
    for key in ['slipFlag','slipFlagLat']:
        axes[2,0].step(t,c[key],where='post',label=key)
    axes[2,0].set(title='Legacy slip (SHADOW unrecorded)',xlabel='time (s)',ylabel='flag')
    for ref in [13194,13195,13207]:
        rc, rls, rd, rm = data[ref]; b=rd<1400
        axes[2,1].plot(rd[b],rc['gyroVal_Z'][b],label=f'{ref}: P{rm["lineTraceOmegaFBCtrl.kp"]}/D{rm["lineTraceOmegaFBCtrl.kd"]}')
    axes[2,1].set(title='Primary first curves (conditions differ)',xlabel='distance (mm)',ylabel='actual yaw (deg/s)')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=7)
    fig.savefig(OUT/f'{n}-verification.png',dpi=140); plt.close(fig)

assert all(r['stop']==0 for r in records if r['log']>=13195)
for start in [13195,13207]:
    group=[r for r in records if start<=r['log']<start+5]
    assert [r['auto_run'] for r in group]==[1,2,3,4,5]
for n in hashes:
    assert hashes[n]==hashlib.sha256((SOURCE/f'{n}.csv').read_bytes()).hexdigest()
(OUT/'validation.json').write_text(json.dumps(dict(originals_unchanged=True,hashes=hashes,records=records),indent=2),encoding='utf-8')
print(json.dumps(records,indent=2))
