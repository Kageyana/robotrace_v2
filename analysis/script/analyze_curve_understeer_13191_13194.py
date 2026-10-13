"""一次走行の旋回不足を診断する。原本は読み取り専用、PID再生は行わない。"""
from pathlib import Path
import csv
import hashlib
import json
import os
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'analysis/curve_understeer_13191_13194'
OUT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault('MPLCONFIGDIR', str(OUT / 'mpl-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SOURCE = Path(r'F:\Dropbox\Document\robotrace\Log\v2')
NUMBERS = [13172, 13191, 13192, 13193, 13194]
datasets, summaries, intervals, hashes = {}, [], [], {}
for number in NUMBERS:
    path = SOURCE / f'{number}.csv'
    hashes[number] = hashlib.sha256(path.read_bytes()).hexdigest()
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    meta = dict(cell.split('=', 1) for cell in lines[0].split(',') if '=' in cell)
    header = next(csv.reader([lines[1]]))
    if header[-1] == '':
        header.pop()
    rows = [r[:-1] if r[-1] == '' else r for r in csv.reader(lines[2:]) if r]
    assert len(set(header)) == len(header)
    assert all(len(r) == len(header) for r in rows)
    a = np.asarray(rows, dtype=float)
    assert np.isfinite(a).all()
    c = {name: a[:, i] for i, name in enumerate(header)}
    assert len(rows) == int(meta['logExpectedRows'])
    assert float(meta['optimalTrace']) == 0  # 一次走行だけを比較
    assert np.all(np.diff(c['cntlog']) > 0)
    distance = c['encTotalOptimal'] / 58.019
    assert np.all(np.diff(distance) > 0)
    ls = np.array([c[f'lSensorCari{i}'] for i in range(10)]).T
    starts = np.r_[0, np.flatnonzero(np.any(np.diff(ls, axis=0) != 0, axis=1)) + 1]
    ends = np.r_[starts[1:] - 1, len(ls) - 1]
    longest = float(np.max(c['cntlog'][ends] - c['cntlog'][starts]))
    # 5mm距離ログでは停止後の時間間隔増大は欠落を意味しない。
    ds = np.diff(distance)
    # 残余パルスを次回へ持ち越すため、1ms進行量ぶん上下に量子化される。
    assert np.all((ds > 3.0) & (ds < 7.0)), (number, ds.min(), ds.max())
    dark = (distance > 1000) & (ls.max(axis=1) < 100)
    dark_index = np.flatnonzero(dark)
    summary = dict(log=number, rows=len(rows), columns=len(header),
                   stop=int(float(meta['emcStop'])), elapsed_ms=float(c['cntlog'][-1]),
                   battery_V=float(meta['batteryVoltage_V']),
                   build=meta['buildDate']+' '+meta['buildTime'],
                   kp=float(meta['lineTraceOmegaFBCtrl.kp']),
                   ki=float(meta['lineTraceOmegaFBCtrl.ki']),
                   kd=float(meta['lineTraceOmegaFBCtrl.kd']),
                   unique_sensor_vectors=len(np.unique(ls, axis=0)),
                   max_observed_same_vector_interval_ms=longest,
                   max_record_interval_ms=float(np.diff(c['cntlog']).max()),
                   distance_step_min_mm=float(ds.min()), distance_step_max_mm=float(ds.max()),
                   first_dark_after_1m_mm=float(distance[dark_index[0]]) if dark_index.size else None,
                   first_dark_after_1m_ms=float(c['cntlog'][dark_index[0]]) if dark_index.size else None,
                   line_calibration_valid=int(meta['lineSensorCalibrationValid']),
                   imu_calibration_valid=int(meta['imuCalibrationValid']))
    summaries.append(summary)
    datasets[number] = (c, ls, meta, distance)
    for lo, hi in [(400, 600), (1050, 1170), (1170, 1220)]:
        mask = (distance >= lo) & (distance < hi)
        entry = dict(log=number, start_mm=lo, end_mm=hi, rows=int(mask.sum()))
        for key in ['targetAngularvelo', 'gyroVal_Z', 'lineTraceCtrl', 'encCurrentN',
                    'encCurrentL', 'encCurrentR', 'targetSpeed',
                    'motorVoltageCmdL_mV', 'motorVoltageCmdR_mV']:
            entry[key+'_median'] = float(np.median(c[key][mask]))
        entry['yaw_error_median_dps'] = float(np.median((c['targetAngularvelo']-c['gyroVal_Z'])[mask]))
        entry['motor_voltage_abs_max_V'] = float(max(np.abs(c[k][mask]).max() for k in
                                                      ['motorVoltageCmdL_mV','motorVoltageCmdR_mV'])/1000)
        entry['right_edge_peak_fraction'] = float(np.mean(ls[mask].argmax(axis=1) == 9))
        intervals.append(entry)

for name, records in [('summary', summaries), ('curve-intervals', intervals)]:
    with (OUT/f'{name}.csv').open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(records[0])); w.writeheader(); w.writerows(records)

for number in [13193, 13194]:
    c, ls, meta, distance = datasets[number]
    t = c['cntlog']/1000
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    axes[0, 0].plot(c['x'], c['y'])
    axes[0, 0].set(title=f'{number}: estimated XY, stop={meta["emcStop"]}', xlabel='x (mm)', ylabel='y (mm)')
    axes[0, 0].axis('equal')
    for key, label in [('targetSpeed','target'), ('encCurrentL','left'), ('encCurrentR','right')]:
        axes[0, 1].plot(t, c[key]/58.019, label=label)
    axes[0, 1].set(title='Wheel speed', xlabel='time (s)', ylabel='m/s')
    axes[1, 0].plot(t, c['targetAngularvelo'], label='target')
    axes[1, 0].plot(t, c['gyroVal_Z'], label='actual')
    axes[1, 0].set(title='Yaw response', xlabel='time (s)', ylabel='deg/s')
    # pcolormesh preserves nonuniform distance-based time spacing.
    axes[1, 1].pcolormesh(t, np.arange(10), ls.T, shading='nearest', vmin=0, vmax=4095)
    axes[1, 1].set(title='Normalized line sensors (0=left, 9=right)', xlabel='time (s)', ylabel='sensor index')
    for key, label in [('motorVoltageCmdL_mV','left'), ('motorVoltageCmdR_mV','right')]:
        axes[2, 0].plot(t, c[key]/1000, label=label)
    axes[2, 0].axhline(float(meta['batteryVoltage_V']), color='gray', linestyle=':', label='header battery')
    axes[2, 0].set(title='Motor voltage commands', xlabel='time (s)', ylabel='V')
    for key, label in [('slipFlag','longitudinal'), ('slipFlagLat','lateral')]:
        axes[2, 1].step(t, c[key], where='post', label=label)
    axes[2, 1].set(title='Legacy slip flags (SHADOW diagnostics unrecorded)', xlabel='time (s)', ylabel='flag')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=8)
    fig.savefig(OUT/f'{number}-diagnosis.png', dpi=150)
    plt.close(fig)

fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
for number in [13172, 13193, 13194]:
    c, ls, meta, distance = datasets[number]
    mask = distance <= 1350
    label = f'{number} P={meta["lineTraceOmegaFBCtrl.kp"]} D={meta["lineTraceOmegaFBCtrl.kd"]}'
    axes[0].plot(distance[mask], c['gyroVal_Z'][mask], label=label)
    axes[1].plot(distance[mask], c['lineTraceCtrl'][mask], label=label)
    axes[2].plot(distance[mask], ls[mask].argmax(axis=1), label=label)
axes[0].set(title='Same primary mode, different gains/calibration/firmware; diagnostic comparison', ylabel='actual yaw (deg/s)')
axes[1].set(ylabel='wheel target differential (pulse/ms)')
axes[2].set(ylabel='brightest sensor index', xlabel='distance (mm)', yticks=range(10))
for ax in axes:
    ax.axvspan(1050, 1220, color='orange', alpha=.1); ax.grid(alpha=.25); ax.legend(fontsize=8)
fig.savefig(OUT/'primary-comparison.png', dpi=150); plt.close(fig)
for number in NUMBERS:
    assert hashes[number] == hashlib.sha256((SOURCE/f'{number}.csv').read_bytes()).hexdigest()
(OUT/'validation.json').write_text(json.dumps(dict(hashes=hashes, originals_unchanged=True,
    summaries=summaries, intervals=intervals), ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(dict(summaries=summaries, intervals=[e for e in intervals if e['start_mm']==1050]), indent=2))
