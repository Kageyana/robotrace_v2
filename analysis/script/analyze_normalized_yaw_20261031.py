"""10/31以降のDISTANCEログで角速度正規化を診断する。実機応答は変更しない。"""
import csv
import hashlib
import io
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT.parent / 'Log' / 'v2'
OUT = ROOT / 'analysis' / 'normalized_yaw_20261031'
JST = timezone(timedelta(hours=9))
SINCE = datetime(2026, 10, 31, tzinfo=JST)


def integer(value):
    x = float(value)
    if not math.isfinite(x) or not x.is_integer():
        raise ValueError(f'Not a finite integer: {value}')
    return int(x)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def save_csv(path, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        writer.writerows(rows)


def read_meta(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return dict(x.split('=', 1) for x in next(csv.reader(stream)) if '=' in x)


def load(number):
    path = LOG / f'{number}.csv'
    raw = path.read_bytes()
    if b'\0' in raw:
        raise ValueError('NUL byte')
    reader = csv.reader(io.StringIO(raw.decode('utf-8-sig')))
    meta = dict(x.split('=', 1) for x in next(reader) if '=' in x)
    header = next(reader)
    if header and header[-1] == '':
        header.pop()
    if not header or len(set(header)) != len(header):
        raise ValueError('Empty or duplicate columns')
    rows = []
    for line, row in enumerate(reader, 3):
        if row and row[-1] == '':
            row.pop()
        if len(row) != len(header):
            raise ValueError(f'Width mismatch at line {line}')
        rows.append([float(x) for x in row])
    if len(rows) != integer(meta['logExpectedRows']):
        raise ValueError('Expected row count mismatch')
    ident = dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), rows=len(rows), columns=len(header))
    if not rows:
        return meta, {}, dict(**ident, status='EMPTY')
    a = np.asarray(rows, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite sample')
    d = dict(zip(header, a.T))
    dt = np.diff(d['cntlog'])
    if not np.all(dt > 0):
        raise ValueError('Nonmonotonic time')
    ppm = float(meta['encoderPulsePerMeter']) / 1000
    if ppm != 58.019:
        raise ValueError('Uncalibrated encoder scale')
    d['speed'] = d['encCurrentN'] / ppm
    d['target'] = d['targetSpeed'] / ppm
    d['distance'] = (d['encTotalL'] + d['encTotalR']) / (2 * ppm)
    sensors = np.column_stack([d[f'lSensorCari{i}'] for i in range(10)]).astype(np.float32)
    # Cのfloat逐次加算→int32格納順序を再現。未記録のセンサー取得鮮度は補完しない。
    left = np.zeros(len(a), dtype=np.float32)
    right = left.copy()
    for i, weight in enumerate([1, 1.01, 1.02, 1.03, 1.04]):
        left = np.float32(left + sensors[:, 4-i] * np.float32(weight))
        right = np.float32(right + sensors[:, 5+i] * np.float32(weight))
    d['sensorDiff'] = right.astype(np.int64) - left.astype(np.int64)
    d['sensorTarget'] = (d['sensorDiff'] * d['encCurrentN'].astype(np.int64)) >> 9
    d['dev'] = d['targetAngularvelo'] - np.trunc(d['gyroVal_Z'])
    d['centroid'] = (sensors * np.arange(10)).sum(axis=1) / np.maximum(sensors.sum(axis=1), 1)
    d['brightMax'] = sensors.max(axis=1)
    d['rail'] = (abs(d['motorVoltageCmdL_mV']) >= 7090) | (abs(d['motorVoltageCmdR_mV']) >= 7090)
    d['p2Prediction'] = np.clip((2 * d['dev'].astype(np.int64)) >> 5, -1000, 1000)
    ds = np.diff(d['distance'])
    return meta, d, dict(**ident, status='VALID', dt_min_ms=float(dt.min()), dt_max_ms=float(dt.max()),
        distance_step_min_mm=float(ds.min()), distance_step_max_mm=float(ds.max()),
        large_distance_steps=int(np.count_nonzero(ds > 5 + np.max(d['speed']) + .15)),
        binary_crc_revalidated=False)


def plan_radii(d, ppm=58.019):
    """50mm中央値の参照形状。CSV丸め入力の診断であり、bin実C再生ではない。"""
    base = int(d['encTotalOptimal'][0])
    step = int(np.float32(ppm) * 50)
    bucket, result = [], []
    for distance, radius in zip(d['encTotalOptimal'], d['ROC']):
        if distance - base >= step:
            if not bucket:
                raise ValueError('Empty 50mm bucket')
            result.append(int(np.median(bucket)))
            base += step
            bucket = []
        # ROCは非負の距離計画半径。Cのlroundf相当。
        bucket.append(math.floor(radius + .5) if radius >= 0 else math.ceil(radius - .5))
        bucket = bucket[:12]
    return np.asarray(result)


def straight_mask(d, radii, margin=2, threshold=2800):
    valid = np.array([i >= margin and i+margin < len(radii) and
        np.all(radii[i-margin:i+margin+1] >= threshold) for i in d['optimalIndex'].astype(int)])
    return valid


def weights(d, mask):
    # 次の保存点までの保持近似。境界を跨ぐ区間と20ms超の欠測を集計しない。
    dt = np.diff(d['cntlog'])
    return np.r_[dt * mask[:-1] * mask[1:] * (dt <= 20), 0.]


def stats(d, mask):
    result = dict(rows=int(mask.sum()))
    w = weights(d, mask)
    total = float(w.sum())
    if total <= 0:
        return result
    mean = lambda a: float(np.sum(w * a) / total)
    rms = lambda a: math.sqrt(mean(a*a))
    result.update(seconds=total/1000, speed_mean=mean(d['speed']), target_speed_mean=mean(d['target']),
        speed_error_rms=rms(d['speed']-d['target']), target_yaw_rms=rms(d['targetAngularvelo']),
        gyro_rms=rms(d['gyroVal_Z']), yaw_error_rms=rms(d['dev']), delta_rms=rms(d['lineTraceCtrl']),
        target_yaw_abs_max=float(np.max(abs(d['targetAngularvelo'][mask]))),
        gyro_abs_max=float(np.max(abs(d['gyroVal_Z'][mask]))), sensor_diff_rms=rms(d['sensorDiff']),
        centroid_offset_rms=rms(d['centroid']-4.5), rail_fraction=mean(d['rail']),
        battery_mean_V=mean(d['batteryVoltage_mV'])/1000,
        sensor_target_within_1dps_fraction=mean(abs(d['targetAngularvelo']-d['sensorTarget']) <= 1),
        p2_output_match_fraction=mean(d['lineTraceCtrl'] == d['p2Prediction']),
        target_gyro_correlation=None,
        yaw_flag_fraction=mean((d['slipV2Flags'].astype(int)&4) != 0),
        long_flag_fraction=mean((d['slipV2Flags'].astype(int)&1) != 0),
        lat_flag_fraction=mean((d['slipV2Flags'].astype(int)&2) != 0))
    x, y = d['targetAngularvelo'][mask], d['gyroVal_Z'][mask]
    if len(x)>2 and np.std(x)>0 and np.std(y)>0:
        result['target_gyro_correlation'] = float(np.corrcoef(x, y)[0,1])
    if 'lineKpUsed' in d:
        result.update(straight_gain_fraction=mean(d['lineGainState']==2),
            curve_gain_fraction=mean(d['lineGainState']==3),
            wheel_clip_fraction=mean(d['wheelTargetLimitFlags']!=0),
            wheel_target_error_rms_p=rms((d['wheelTargetL_p']-d['wheelTargetR_p'])-
                (d['encCurrentL']-d['encCurrentR'])))
    return result


def diagnostics(d):
    if 'lineKpUsed' not in d:
        return dict(used_gains='UNRECORDED', wheel_targets='UNRECORDED')
    active = d['lineGainState'] > 0
    p = d['lineKpUsed'] * d['dev']
    if not np.all(p[active] == d['linePTerm_x32'][active]):
        raise ValueError('Recorded P term mismatch')
    out = np.clip((d['linePTerm_x32'].astype(np.int64)+d['lineDTerm_x32'].astype(np.int64)) >> 5, -1000,1000)
    if not np.all(out[active] == d['lineTraceCtrl'][active]):
        raise ValueError('Recorded controller output mismatch')
    adjacent = active & np.r_[False, (np.diff(d['cntlog'])==1)&active[:-1]]
    dpred = d['lineKdUsed'] * np.r_[0,np.diff(d['dev'])]
    if not np.all(dpred[adjacent] == d['lineDTerm_x32'][adjacent]):
        raise ValueError('Adjacent 1ms D term mismatch')
    combinations = []
    for kp,kd,state in sorted(set(zip(d['lineKpUsed'].astype(int),d['lineKdUsed'].astype(int),d['lineGainState'].astype(int)))):
        mask = (d['lineKpUsed']==kp)&(d['lineKdUsed']==kd)&(d['lineGainState']==state)
        combinations.append(dict(kp=int(kp),kd=int(kd),state=int(state),rows=int(mask.sum())))
    return dict(gains=combinations, active_rows=int(active.sum()), p_matches=int(active.sum()),
        output_matches=int(active.sum()), d_1ms_matches=int(adjacent.sum()))


def segments(d, mask):
    ii = np.flatnonzero(mask)
    if not len(ii):
        return []
    return np.split(ii, np.flatnonzero(np.diff(ii)>1)+1)


def response(d, ii):
    t = d['cntlog'][ii]
    if len(t)<30 or t[-1]-t[0]<200 or np.max(np.diff(t))>10:
        return {}
    grid = np.arange(t[0], t[-1]+.1, 1.)
    keys = ['targetAngularvelo','gyroVal_Z','lineTraceCtrl','sensorDiff']
    arrays = {k:np.interp(grid,t,d[k][ii]) for k in keys}
    if 'wheelTargetL_p' in d:
        arrays['wheel_target_diff'] = np.interp(grid,t,(d['wheelTargetL_p']-d['wheelTargetR_p'])[ii])
        arrays['wheel_actual_diff'] = np.interp(grid,t,(d['encCurrentL']-d['encCurrentR'])[ii])
    arrays = {k:v-v.mean() for k,v in arrays.items()}
    freq = np.fft.rfftfreq(len(grid),.001)
    band = (freq>=5)&(freq<=min(60,500/np.max(np.diff(t))))
    peaks = {}
    for k,a in arrays.items():
        power = abs(np.fft.rfft(a*np.hanning(len(a))))**2
        if power[band].sum()>0:
            peaks[k] = float(freq[band][np.argmax(power[band])])
    correlations = {}
    for a,b in [('targetAngularvelo','gyroVal_Z'),('lineTraceCtrl','gyroVal_Z'),
                ('wheel_target_diff','wheel_actual_diff')]:
        if a not in arrays or b not in arrays:
            continue
        vals = []
        for lag in range(0,41):
            x,y = arrays[a],arrays[b]
            if lag:
                x,y = x[:-lag],y[lag:]
            if np.std(x)>0 and np.std(y)>0:
                vals.append(dict(lag_ms=lag,correlation=float(np.corrcoef(x,y)[0,1])))
        if vals:
            correlations[a+'->'+b] = dict(best=max(vals,key=lambda r:r['correlation']), samples=vals)
    return dict(duration_ms=float(t[-1]-t[0]),saved_dt_max_ms=float(np.max(np.diff(t))),
        resolution_hz=float(freq[1]),peaks_hz=peaks,correlations=correlations)


def snapshot_candidates(d, mask):
    """P2/D0記録点だけの指令感度。閉ループ走行性能を予測しない。"""
    mask = mask & (d['lineKpUsed']==2)&(d['lineKdUsed']==0)&(d['courseMarker']==0)
    if np.count_nonzero(mask)<2:
        return []
    w = weights(d,mask)
    total = w.sum()
    if total<=0:
        return []
    mean = lambda a: float(np.sum(w*a)/total)
    rms = lambda a: math.sqrt(mean(a*a))
    gain = np.minimum(1.,2./np.maximum(d['speed'],.5))
    target, gyro = d['targetAngularvelo'], np.trunc(d['gyroVal_Z'])
    mean_target = (d['wheelTargetL_p']+d['wheelTargetR_p'])/2
    baseline_delta = d['lineTraceCtrl']
    # 両輪非飽和点なら平均は厳密。片輪飽和点では自由輪から使用平均を復元。
    lf,rf = abs(d['wheelTargetL_p'])<290,abs(d['wheelTargetR_p'])<290
    mean_target[lf] = d['wheelTargetL_p'][lf]-baseline_delta[lf]
    mean_target[~lf&rf] = d['wheelTargetR_p'][~lf&rf]+baseline_delta[~lf&rf]
    recoverable = lf|rf
    results = []
    for name, error in [('baseline',target-gyro),('both_speed_scaled',gain*(target-gyro)),
                        ('target_speed_scaled',gain*target-gyro)]:
        delta = np.clip(np.floor(2.*error/32),-1000,1000)
        validw = w*recoverable
        clip = ((abs(mean_target+delta)>290)|(abs(mean_target-delta)>290))
        results.append(dict(candidate=name,rows=int(mask.sum()),seconds=float(total/1000),
            delta_rms_p=rms(delta),delta_abs_max_p=float(np.max(abs(delta[mask]))),
            mean_speed_scale=mean(gain),p_target_component_rms_p=rms(2*gain*target/32 if name!='baseline' else 2*target/32),
            p_gyro_component_rms_p=rms(2*gain*gyro/32 if name=='both_speed_scaled' else 2*gyro/32),
            recoverable_wheel_time_fraction=mean(recoverable),
            wheel_clip_fraction=float(np.sum(validw*clip)/np.sum(validw)) if np.sum(validw)>0 else None,
            predicted_gyro_rms=None,closed_loop_validated=False))
    return results


def self_checks():
    for value,expected in [('2',2),('2.00',2)]:
        assert integer(value)==expected
    for value in ['2.1','nan','inf','-inf']:
        try:
            integer(value)
        except ValueError:
            pass
        else:
            raise AssertionError(value)
    toy = dict(optimalIndex=np.arange(9))
    assert straight_mask(toy,np.full(9,3000)).tolist()==[False,False,True,True,True,True,True,False,False]
    rr=np.full(9,3000);rr[4]=100
    assert not straight_mask(toy,rr).any()
    assert np.allclose((np.array([1.,-3.,10.])/100)*100,np.array([1.,-3.,10.]))


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    self_checks()
    inventory, targets = [], []
    for path in sorted(LOG.glob('*.csv')):
        if not path.stem.isdigit():
            continue
        saved = datetime.fromtimestamp(path.stat().st_mtime, JST)
        if saved<SINCE:
            continue
        try:
            meta = read_meta(path)
            mode = integer(meta.get('optimalTrace',-1))
            slip_source = integer(meta.get('slipSourceLogNumber',0))
            classification = ('SLIP' if slip_source else 'DISTANCE') if mode==2 else 'OTHER'
            row = dict(log=int(path.stem),saved_jst=saved.isoformat(),mode=mode,classification=classification,
                primary=meta.get('primaryLogNumber'),slip_source=slip_source,emc=meta.get('emcStop'),
                expected_rows=meta.get('logExpectedRows'),sha256=sha(path))
            inventory.append(row)
            if classification=='DISTANCE':
                targets.append(int(path.stem))
        except (ValueError,KeyError,StopIteration) as exc:
            inventory.append(dict(log=int(path.stem),saved_jst=saved.isoformat(),classification='REJECTED',reason=str(exc),sha256=sha(path)))
    save_csv(OUT/'inventory.csv',inventory)
    result = dict(selection=dict(since_jst=SINCE.isoformat(),date_field='filesystem LastWriteTime JST, not buildDate or proven run time',
        selected_distance=sorted(targets),slip_logs=[r['log'] for r in inventory if r['classification']=='SLIP']),
        logs={},primaries={},rejected=[],windows=[],bands=[],segments=[],candidates=[],sensitivity=[])
    cache = {}
    chart_rows = []
    for number in sorted(targets):
        try:
            meta,d,identity = load(number)
            if not d:
                result['logs'][number]=dict(identity=identity,metadata=meta)
                continue
            primary = integer(meta['primaryLogNumber'])
            if primary not in cache:
                pm,pd,pi=load(primary)
                if integer(pm['optimalTrace'])!=0 or integer(pm['emcStop'])!=0 or not pd:
                    raise ValueError(f'Invalid reference primary: {primary}')
                radii = plan_radii(pd)
                cache[primary]=(pm,pd,pi,radii)
                result['primaries'][primary]=dict(identity=pi,metadata=pm,plan_bins=len(radii))
                save_csv(OUT/f'{primary}_reference_radii.csv',[dict(index=i,radius_mm=int(r)) for i,r in enumerate(radii)])
            pm,pd,pi,radii=cache[primary]
            diag = diagnostics(d)
            straight = straight_mask(d,radii)
            normal = integer(meta['emcStop'])==0 and integer(meta.get('sgMarkerAtLogEnd',0))==2
            fast = straight&(d['speed']>=3)
            item=dict(identity=identity,metadata=meta,primary=primary,normal_log_end=normal,
                physical_course_completion='UNCONFIRMED',diagnostics=diag,
                whole=stats(d,np.ones(len(d['cntlog']),dtype=bool)),straight=stats(d,straight),fast_straight=stats(d,fast),
                max_index=int(d['optimalIndex'].max()),unmapped_index_rows=int(np.count_nonzero(d['optimalIndex']>=len(radii))))
            if 'lineGainPlanEnterRoc_mm' in d:
                idx=d['optimalIndex'].astype(int)
                valid=idx+4<len(radii)
                item['reference_plan_check']=dict(rows=int(valid.sum()),
                    enter_same_index_fraction=float(np.mean(d['lineGainPlanEnterRoc_mm'][valid]==radii[idx[valid]+4])),
                    exit_same_index_fraction=float(np.mean(d['lineGainPlanExitRoc_mm'][valid]==radii[idx[valid]+2])))
            result['logs'][number]=item
            for lo,hi in [(1,2),(2,3),(3,3.5),(3.5,4),(4,5.5)]:
                result['bands'].append(dict(log=number,normal_log_end=normal,lo_mps=lo,hi_mps=hi,
                    **stats(d,straight&(d['speed']>=lo)&(d['speed']<hi))))
            for margin in [1,2,4]:
                result['sensitivity'].append(dict(log=number,margin_bins=margin,threshold_mm=2800,
                    **stats(d,straight_mask(d,radii,margin)&(d['speed']>=3))))
            for lo,hi in [(4700,8000),(5000,6500),(6500,7500)]:
                mask=(d['distance']>=lo)&(d['distance']<hi)
                result['windows'].append(dict(log=number,lo_mm=lo,hi_mm=hi,normal_log_end=normal,
                    reference_straight_fraction=float(np.mean(straight[mask])) if mask.any() else None,
                    **stats(d,mask)))
            for sid,ii in enumerate(segments(d,straight)):
                mask=np.zeros(len(straight),dtype=bool);mask[ii]=True
                s=stats(d,mask)
                if s.get('seconds',0)<.15:
                    continue
                result['segments'].append(dict(log=number,segment=sid,start_mm=float(d['distance'][ii[0]]),
                    end_mm=float(d['distance'][ii[-1]]),start_ms=float(d['cntlog'][ii[0]]),
                    end_ms=float(d['cntlog'][ii[-1]]),**s,response=response(d,ii)))
            if 'lineKpUsed' in d:
                for scope,mask in [('straight',straight),('fast_straight',fast)]:
                    for cand in snapshot_candidates(d,mask):
                        result['candidates'].append(dict(log=number,scope=scope,**cand))
            # 代表波形候補は全14本で同じ固定窓を保存。出力で恣意的に対象を除外しない。
            mask=(d['distance']>=4700)&(d['distance']<8000)
            for i in np.flatnonzero(mask):
                chart_rows.append(dict(log=number,reference_straight=bool(straight[i]),
                    **{k:float(d[k][i]) for k in ['cntlog','distance','speed','target','sensorDiff',
                    'targetAngularvelo','gyroVal_Z','lineTraceCtrl','centroid','motorVoltageCmdL_mV','motorVoltageCmdR_mV']},
                    **{k:float(d[k][i]) for k in ['lineKpUsed','lineKdUsed','lineGainState','wheelTargetL_p','wheelTargetR_p','wheelTargetLimitFlags'] if k in d}))
        except (ValueError,KeyError,AssertionError,FileNotFoundError) as exc:
            result['rejected'].append(dict(log=number,reason=str(exc)))
    result['source_hashes']={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),
        ROOT/'robotrace_v2/Core/Src/PIDcontrol.c',ROOT/'robotrace_v2/Core/Src/control.c',
        ROOT/'robotrace_v2/Core/Src/timer.c',ROOT/'robotrace_v2/Core/Src/courseAnalysis.c']}
    result['limitations']=['走行日はLastWriteTime JSTで選定。buildDateを走行日へ使用しない。',
        '原binがなく全CRCは再検証していない。CSV全行幅・有限性・期待行数・時刻と原本SHAを確認。',
        '一次CSVの50mm中央値半径と二次optimalIndexで直線内部を定義。実測gyroで二次の区間を除外しない。',
        '一次CSVの形状も自己推定。実際の床上の直線・位置や板内完走を保証しない。',
        '時間率は隣接保存点が両方対象で20ms以下の場合の保持近似。1ms全履歴ではない。',
        '候補は実記録P2/D0・非マーカー点だけの同一入力指令感度。ジャイロ低下や閉ループ安定性は予測しない。',
        '速度正規化例はmin(1,2/max(v,0.5))。2m/sは比較用の任意基準で採用値ではない。',
        'D!=0周期の速度正規化後D項は未記録1ms履歴が必要なため評価対象外。',
        'FFTと相互相関は疎な保存点の線形補間による診断。閉ループの相関を単体の遅延・因果としない。']
    for key in ['windows','bands','segments','candidates','sensitivity']:
        save_csv(OUT/f'{key}.csv',[{k:v for k,v in row.items() if k!='response'} for row in result[key]])
    save_csv(OUT/'straight_traces.csv',chart_rows)
    # 同じプロセスの最後に全入力の不変性を再確認する。
    identities=[v['identity'] for v in result['logs'].values()]+[v['identity'] for v in result['primaries'].values()]
    if not all(sha(Path(i['path']))==i['sha256'] for i in identities):
        raise ValueError('Original changed during analysis')
    result['original_hashes_rechecked']=len(identities)
    save_json(OUT/'summary.json',result)
    print('SELECTED',result['selection'])
    print('REJECTED',result['rejected'])
    for n,item in result['logs'].items():
        a=item.get('fast_straight',{})
        print('LOG',n,'normal',item.get('normal_log_end'),'fast', {k:round(a[k],3) for k in
            ['seconds','speed_mean','target_yaw_rms','gyro_rms','yaw_error_rms','rail_fraction','wheel_clip_fraction'] if k in a})
    print('PASS original SHA recheck',len(identities))


if __name__=='__main__':
    main()
