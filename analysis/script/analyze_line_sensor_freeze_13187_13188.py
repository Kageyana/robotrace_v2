from pathlib import Path
import csv
import hashlib
import json
import os
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'analysis/line_sensor_freeze_13187_13188/mpl-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SOURCE = Path(r'F:\Dropbox\Document\robotrace\Log\v2')
OUT = ROOT / 'analysis/line_sensor_freeze_13187_13188'
OUT.mkdir(parents=True, exist_ok=True)
LOGS = [13172, 13185, 13186, 13187, 13188]
summaries, datasets, hashes = [], {}, {}
for number in LOGS:
    path = SOURCE / f'{number}.csv'
    hashes[number] = hashlib.sha256(path.read_bytes()).hexdigest()
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    meta = dict(cell.split('=',1) for cell in lines[0].split(',') if '=' in cell)
    header = next(csv.reader([lines[1]]))
    if header[-1] == '': header.pop()
    rows = list(csv.reader(lines[2:]))
    rows = [r[:-1] if r[-1] == '' else r for r in rows if r]
    assert all(len(r) == len(header) for r in rows)
    data = np.asarray(rows, dtype=float)
    assert np.isfinite(data).all()
    col = {name: data[:,i] for i,name in enumerate(header)}
    assert np.all(np.diff(col['cntlog']) > 0)
    assert len(rows) == int(meta['logExpectedRows'])
    ls = np.array([col[f'lSensorCari{i}'] for i in range(10)]).T
    changed = np.flatnonzero(np.any(np.diff(ls,axis=0) != 0,axis=1)) + 1
    tail_start = int(changed[-1]) if len(changed) else 0
    summary = dict(log=number, mode=float(meta['optimalTrace']), stop=int(float(meta['emcStop'])),
                   rows=len(rows), columns=len(header), elapsed_ms=col['cntlog'][-1],
                   expected_rows=int(meta['logExpectedRows']),
                   unique_sensor_vectors=len(np.unique(ls,axis=0)),
                   frozen_tail_start_ms=col['cntlog'][tail_start],
                   frozen_tail_ms=col['cntlog'][-1]-col['cntlog'][tail_start],
                   max_record_interval_ms=np.diff(col['cntlog']).max(),
                   battery_V=float(meta['batteryVoltage_V']),
                   build=meta['buildDate']+' '+meta['buildTime'],branch=meta['branch'],
                   calibration_valid=int(meta['lineSensorCalibrationValid']),
                   line_kp=float(meta['lineTraceOmegaFBCtrl.kp']),line_kd=float(meta['lineTraceOmegaFBCtrl.kd']))
    summaries.append(summary)
    datasets[number] = (col,ls,meta)
with (OUT/'log-summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(summaries[0]));writer.writeheader();writer.writerows(summaries)

fig,axes=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
for row,number in enumerate([13187,13188]):
    col,ls,meta=datasets[number];t=col['cntlog']/1000
    for i in range(10): axes[row,0].plot(t,ls[:,i],label=str(i))
    axes[row,0].set(title=f'{number}: all 10 channels frozen',ylabel='line sensor (0..4095)',xlabel='time (s)',ylim=(0,4095))
    axes[row,0].legend(ncol=5,fontsize=8)
    axes[row,1].plot(t,col['targetAngularvelo'],label='target')
    axes[row,1].plot(t,col['gyroVal_Z'],label='actual')
    axes[row,1].set(title='Yaw response',ylabel='deg/s',xlabel='time (s)');axes[row,1].legend()
    axes[row,2].plot(t,col['targetSpeed']/58.019,label='target')
    axes[row,2].plot(t,col['encCurrentL']/58.019,label='left')
    axes[row,2].plot(t,col['encCurrentR']/58.019,label='right')
    axes[row,2].set(title='Speed response',ylabel='m/s',xlabel='time (s)');axes[row,2].legend()
for ax in axes.flat: ax.grid(alpha=.25)
fig.savefig(OUT/'sensor-and-control.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
for number in [13187,13188]:
    col,ls,meta=datasets[number];t=col['cntlog']/1000
    axes[0,0].plot(col['x'],col['y'],label=str(number))
    axes[0,1].plot(t,col['motorVoltageCmdL_mV']/1000,label=f'{number} left')
    axes[0,1].plot(t,col['motorVoltageCmdR_mV']/1000,label=f'{number} right')
    axes[1,0].step(t,col['slipFlag'],where='post',label=f'{number} long')
    axes[1,0].step(t,col['slipFlagLat'],where='post',label=f'{number} lateral')
    axes[1,1].plot(t,col['encTotalOptimal']/58.019,label=str(number))
axes[0,0].set(title='Estimated XY (diagnostic only)',xlabel='x (mm)',ylabel='y (mm)');axes[0,0].axis('equal')
axes[0,1].set(title='Motor voltage commands',xlabel='time (s)',ylabel='V')
axes[1,0].set(title='Legacy slip flags',xlabel='time (s)',ylabel='flag')
axes[1,1].set(title='Recorded distance',xlabel='time (s)',ylabel='mm')
for ax in axes.flat: ax.legend();ax.grid(alpha=.25)
fig.savefig(OUT/'trajectory-and-output.png',dpi=160);plt.close(fig)
for number in LOGS: assert hashes[number] == hashlib.sha256((SOURCE/f'{number}.csv').read_bytes()).hexdigest()
(OUT/'input-validation.json').write_text(json.dumps(dict(hashes=hashes,summaries=summaries,originals_unchanged=True),indent=2),encoding='utf-8')
print(json.dumps(summaries,indent=2))
