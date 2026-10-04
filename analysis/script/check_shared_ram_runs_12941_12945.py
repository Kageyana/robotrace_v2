"""RAM共通化後のautoStart 5走を検証する。元ログは変更しない。"""
from pathlib import Path
import csv
import hashlib
import json
import os
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'analysis/shared_ram_runs_12941_12945'
OUT.mkdir(parents=True, exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(OUT / '.mplcache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOG = Path('F:/Dropbox/Document/robotrace/Log/v2')
results, data = [], {}
for number in range(12941, 12946):
    path = LOG / f'{number}.csv'
    with path.open(encoding='utf-8-sig', newline='') as file:
        reader = csv.reader(file)
        meta = dict(cell.split('=', 1) for cell in next(reader) if '=' in cell)
        header = next(reader)
        raw = [row for row in reader if row and any(row)]
    bad_width = sum(len(row) != len(header) for row in raw)
    if bad_width:
        raise ValueError(f'{number}: malformed rows: {bad_width}')
    values = {name: np.array([float(row[i]) for row in raw])
              for i, name in enumerate(header) if name}
    scale = float(meta['encoderPulsePerMeter']) / 1000
    time = values['cntlog'].copy()
    wraps = np.diff(time) < -32768
    time[1:] += np.cumsum(wraps) * 65536
    dt = np.diff(time)
    weights = np.r_[time[0], dt]
    distance = np.diff((values['encTotalL'] + values['encTotalR']) / 2) / scale
    # 1ms取得の距離ログは目標間隔を最大1ms分超える。瞬時速度の量子化余裕を含む。
    bound = float(meta['logDistanceTargetMm']) + max(values['encCurrentN']) / scale + 2 / scale
    speed, target = values['encCurrentN'] / scale, values['targetSpeed'] / scale
    nonfinite = sum(np.count_nonzero(~np.isfinite(array)) for array in values.values())
    summary = dict(log=number, auto_run=int(float(meta['autoStart'])),
                   requested=meta['requestedMode'], actual_mode=int(float(meta['optimalTrace'])),
                   shortcut_level=int(meta['shortcutLevel']), rows=len(raw),
                   expected_rows=int(meta['logExpectedRows']), columns=len(values),
                   emc_stop=int(float(meta['emcStop'])), time_wraps=int(sum(wraps)),
                   nonpositive_dt=int(sum(dt <= 0)), max_gap_ms=float(max(dt)),
                   raw_distance_max_mm=round(float(max(distance)), 3),
                   distance_gap_bound_mm=round(bound, 3), suspicious_distance_gaps=int(sum(distance > bound)),
                   negative_raw_distance_steps=int(sum(distance < 0)), bad_width=bad_width,
                   nonfinite_values=int(nonfinite), recorded_end_s=round(float(time[-1])/1000, 3),
                   battery_header_v=float(meta['batteryVoltage_V']),
                   battery_min_v=round(float(min(values['batteryVoltage_mV']))/1000, 3),
                   speed_mae_mps=round(float(np.average(abs(speed-target), weights=weights)), 4),
                   peak_speed_mps=round(float(max(speed)), 3),
                   slip_time_pct=round(float(np.average(values['slipFlag'] != 0, weights=weights))*100, 3),
                   lateral_slip_time_pct=round(float(np.average(values['slipFlagLat'] != 0, weights=weights))*100, 3),
                   primary_log=int(meta['primaryLogNumber']), route_source=int(meta['routeSourceLog']),
                   closure_valid=int(meta['closureValid']), closure_reason=int(meta['closureReason']),
                   goal_x_mm=float(meta['goalMarkerX_mm']), imu_valid=int(meta['imuCalibrationValid']),
                   distance_valid=int(meta['distanceScaleVerified']),
                   binary_number=int(meta['binaryLogNumber']), binary_crc=meta['binaryDataCrc'],
                   binary_schema=meta['binarySchema'], metadata=meta,
                   sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    summary['valid_csv'] = (summary['rows'] == summary['expected_rows'] and summary['binary_number'] == number
                            and not any(summary[key] for key in ['emc_stop', 'bad_width', 'nonfinite_values',
                                        'nonpositive_dt', 'suspicious_distance_gaps', 'negative_raw_distance_steps']))
    results.append(summary)
    data[number] = values, time, scale

checks = dict(all_csv_valid=all(row['valid_csv'] for row in results),
              auto_sequence=[row['auto_run'] for row in results] == list(range(1, 6)),
              shared_primary=all(row['primary_log'] == 12941 for row in results),
              primary_valid=all(results[0][key] == 1 for key in ['closure_valid', 'imu_valid', 'distance_valid']),
              unique_binary_crc=len({row['binary_crc'] for row in results}) == 5,
              same_schema=len({row['binary_schema'] for row in results}) == 1,
              candidate_branch=all(row['metadata']['branch'] == 'codex/shared-run-ram' for row in results),
              requested_modes=[row['requested'] for row in results] == ['PRIMARY','PATH','SHORTCUT','DISTANCE','SLIP'],
              slip_source=results[-1]['metadata']['slipSourceLogNumber'] == '12944',
              path_sources=all(row['route_source'] == 12941 for row in results[1:3]),
              old_path_invalidated=all(row['route_source'] == 0 for row in results[3:]))
(OUT / 'validation.json').write_text(json.dumps(dict(checks=checks, logs=results), ensure_ascii=False, indent=2), encoding='utf-8')
fields = [key for key in results[0] if key != 'metadata']
with (OUT / 'summary.csv').open('w', encoding='utf-8-sig', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=fields)
    writer.writeheader()
    writer.writerows({key: row[key] for key in fields} for row in results)

fig, axes = plt.subplots(5, 4, figsize=(17, 15), layout='constrained')
for i, row in enumerate(results):
    values, time, scale = data[row['log']]
    t = time / 1000
    axes[i, 0].plot(values['x'], values['y'], lw=.7)
    axes[i, 0].set_aspect('equal', adjustable='datalim')
    axes[i, 0].set_title(f"{row['log']} {row['requested']} / actual {row['actual_mode']}")
    axes[i, 0].set_xlabel('X [mm]'); axes[i, 0].set_ylabel('Y [mm]')
    axes[i, 1].plot(t, values['encCurrentN']/scale, lw=.7, label='actual')
    axes[i, 1].plot(t, values['targetSpeed']/scale, lw=.7, label='target')
    axes[i, 1].set_ylabel('Speed [m/s]')
    axes[i, 2].plot(t, values['gyroVal_Z'], lw=.6)
    axes[i, 2].set_ylabel('Angular velocity [deg/s]')
    axes[i, 3].plot(t, values['slipFlag'], lw=.6, label='longitudinal')
    axes[i, 3].plot(t, values['slipFlagLat']+1.5, lw=.6, label='lateral +1.5')
    for ax in axes[i, 1:]:
        ax.set_xlabel('Recorded time [s]'); ax.grid(alpha=.2)
    axes[i, 1].legend(); axes[i, 3].legend()
fig.savefig(OUT / 'run_checks.png', dpi=140)
plt.close(fig)

lines = ['# autoStart 12941～12945：RAM共通化後の確認', '',
         '対象は2026-10-04 14:12:21ビルド。元CSVは変更していない。', '',
         '| ログ | 走行 | 要求方式 | 実方式 | 行数/期待 | 記録終了[s] | 停止理由 | 電圧[V] |',
         '|---|---:|---|---:|---|---:|---:|---:|']
for row in results:
    lines.append(f"| {row['log']} | {row['auto_run']} | {row['requested']} | {row['actual_mode']} | {row['rows']}/{row['expected_rows']} | {row['recorded_end_s']} | {row['emc_stop']} | {row['battery_header_v']} |")
lines += ['', '検証結果：' + json.dumps(checks, ensure_ascii=False), '',
          '記録終了時刻は停止までのログ時間であり、厳密なゴールラップタイムとは区別する。',
          '欠落判定は左右累積距離の差分を使用し、5mm目標間隔と1ms取得による超過を考慮した。',
          'closure/distanceScale検証は一次ログが対象。二次ログの0を一次ログ無効と混同しない。',
          '一次ログ12941の閉路X=' + str(results[0]['goal_x_mm']) + 'mmは±60mm内で有効。5走とも行数一致・41列・有限値・停止理由0で、距離基準の欠落疑いは0だった。',
          'SHORTCUT要求の12943は実方式PATH（optimalTrace=4）、shortcutLevel=0。短縮は適用されていない。',
          '走行順は一次→PATH→SHORTCUT要求（実際PATH）→DISTANCE→SLIP。PATHからDISTANCEへの共有RAM切替と、その後のSLIP開始・正常終了・CSV保存を確認できた。DISTANCE/SLIPのrouteSourceLog=0は旧PATH経路を無効化した実装と一致する。',
          'RAM使用量とスタック最大使用量はCSVに含まれないため、実測値としては確認できない。SHORTCUT Level 1とDISTANCE→PATH方向の実機切替は今回未確認。',
          '直前セット12936～12940は7.59～7.53V、今回は8.35～8.25Vで約0.72～0.76V高い。タイム差や速度追従の差をRAM共通化の効果として判断しない。',
          '今回1セットの完了で共通化に起因する明らかな異常は見られない。再現性やPATH/SHORTCUT性能の採用判断には追加走行が必要。',
          'CRC識別値の保存は確認できるが、削除済みバイナリの実データCRCはCSVから再検証できない。',
          'PC側フォルダの残存ファイル確認は実機SDカード全体の確認を保証しない。',
          '走行間待ち時間、CSV変換時間、画面表示、電源断復旧、最大スタック使用量はCSVだけでは確認できない。', '',
          '![走行確認](run_checks.png)']
(OUT / 'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
print(json.dumps(dict(checks=checks, logs=[{key: row[key] for key in ['log','requested','actual_mode','shortcut_level','rows','recorded_end_s','valid_csv','closure_valid','distance_valid','speed_mae_mps','suspicious_distance_gaps']} for row in results]), ensure_ascii=False, indent=2))
if not all(checks.values()):
    raise SystemExit('validation failed; inspect validation.json')
