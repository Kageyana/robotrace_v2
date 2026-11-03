"""初回の前後補正診断を全行検査する。CSV原本・ファーム・設定は変更しない。"""
from pathlib import Path
import collections
import csv
import hashlib
import json
import math

ROOT=Path(__file__).resolve().parents[2]
LOG=ROOT.parent/'Log/v2'
OUT=ROOT/'analysis/path_progress_14120_14128'
NUMBERS=[14120,14121,14122,14123,14124,14125,14126,14128]
SLIP={'slipV2LongFast','slipV2LongSlow','slipV2Lat','slipV2YawDps','slipV2Flags',
      'slipV2CurrentHighSamples','slipV2PwmSum','imuYawAlphaFiltered','imuLatTangentialApplied'}
REASONS={0:'disabled',1:'collecting',2:'matching',3:'unobservable',4:'map_gap',5:'residual',
         6:'ambiguous',7:'confirming',8:'accepted',9:'no_line'}


def integer(value):
    v=float(value)
    if not math.isfinite(v) or not v.is_integer():raise ValueError(value)
    return int(v)


def load(n):
    p=LOG/f'{n}.csv'
    if not p.is_file():raise FileNotFoundError(p)
    sha=hashlib.sha256(p.read_bytes()).hexdigest()
    with p.open(encoding='utf-8-sig',newline='') as f:
        entries=[s.split('=',1) for s in f.readline().strip().rstrip(',').split(',')]
        if len({v[0] for v in entries})!=len(entries):raise ValueError(n,'duplicate metadata')
        meta=dict(entries);reader=csv.reader(f);names=next(reader);raw=list(reader)
    if not names[-1]=='' or len(set(names))!=len(names):raise ValueError(n,'column names')
    if not raw or any(len(r)!=len(names) or r[-1]!='' for r in raw):raise ValueError(n,'row width')
    rows=[dict(zip(names[:-1],map(float,r[:-1]))) for r in raw]
    if any(not math.isfinite(v) for r in rows for v in r.values()):raise ValueError(n,'nonfinite')
    if len(rows)!=integer(meta['logExpectedRows']):raise ValueError(n,'row count')
    if integer(meta['binaryLogNumber'])!=n:raise ValueError(n,'conversion identity')
    if any(b['cntlog']<=a['cntlog'] for a,b in zip(rows,rows[1:])):raise ValueError(n,'time order')
    ppm=integer(meta['encoderPulsePerMeter'])/1000
    for r in rows:
        r['distance_mm']=(r['encTotalL']+r['encTotalR'])/2/ppm
        r['speed_mps']=r['encCurrentN']/ppm;r['target_mps']=r['targetSpeed']/ppm
        sensors=[r[f'lSensorCari{i}'] for i in range(10)]
        r['sensor_max']=max(sensors);r['sensor_peak']=sensors.index(max(sensors))
        r['bright800']=sum(v>=800 for v in sensors)
    return p,sha,meta,names[:-1],rows


def sample(r):
    keys=['cntlog','distance_mm','speed_mps','target_mps','optimalIndex','pathState','pathErrorY_mm',
          'pathErrorHeading_cdeg','gyroVal_Z','targetAngularvelo','sensor_max','sensor_peak','bright800',
          'pathProgressReason','pathProgressReasonMask','pathProgressAlong_mm','pathProgressRms_mm',
          'pathProgressInformation','pathProgressMatchValid','pathProgressMatchAge_ms',
          'pathProgressAcceptCount','pathProgressRejectCount','pathProgressDelta_mm']
    return {k:round(r[k],6) for k in keys if k in r}


def main():
    OUT.mkdir(parents=True,exist_ok=True);summary={}
    for n in NUMBERS:
        p,sha,m,names,rows=load(n)
        q=dict(metadata=m,sha256=sha,columns=len(names),rows=len(rows),time_end_ms=rows[-1]['cntlog'],
               distance_end_mm=rows[-1]['distance_mm'],max_dt_ms=max(b['cntlog']-a['cntlog'] for a,b in zip(rows,rows[1:])),
               max_spacing_mm=max(b['distance_mm']-a['distance_mm'] for a,b in zip(rows,rows[1:])),
               slip_columns=sorted(SLIP&set(names)))
        if 'pathProgressReason' in names:
            if len(names)!=(67 if n==14128 else 76):raise ValueError(n,'unexpected diagnostic schema')
            if len(q['slip_columns'])!=(0 if n==14128 else 9):raise ValueError(n,'slip schema')
            counts=collections.Counter(integer(r['pathProgressReason']) for r in rows)
            if set(counts)-set(REASONS):raise ValueError(n,'reason out of range')
            q['reason_rows']={REASONS[k]:v for k,v in sorted(counts.items())}
            q['reason_mask_rows']={REASONS[k]:sum(bool(integer(r['pathProgressReasonMask'])&(1<<k)) for r in rows) for k in REASONS}
            q['accepted_records_sum']=sum(integer(r['pathProgressAcceptCount']) for r in rows)
            q['rejected_records_sum']=sum(integer(r['pathProgressRejectCount']) for r in rows)
            q['applied_signed_sum_mm']=sum(r['pathProgressDelta_mm'] for r in rows)
            q['applied_abs_sum_mm']=sum(r['pathProgressAbsDelta_mm'] for r in rows)
            q['valid_rows']=sum(integer(r['pathProgressMatchValid']) for r in rows)
            q['numerical_rows']=sum(r['pathProgressMatchAge_ms']<65535 for r in rows)
            q['max_information']=max(r['pathProgressInformation'] for r in rows)
            q['state_rows']=dict(collections.Counter(integer(r['pathState']) for r in rows))
            q['state_changes']=[sample(r) for i,r in enumerate(rows) if i==0 or r['pathState']!=rows[i-1]['pathState']]
            q['numerical_updates']=[sample(r) for i,r in enumerate(rows)
                if r['pathProgressMatchAge_ms']<65535 and (i==0 or rows[i-1]['pathProgressMatchAge_ms']==65535 or
                    rows[i-1]['pathProgressMatchAge_ms']+r['cntlog']-rows[i-1]['cntlog']-r['pathProgressMatchAge_ms']>0.5)]
            q['end']=sample(rows[-1])
            q['peak_y']=sample(max(rows,key=lambda r:abs(r['pathErrorY_mm'])))
            if abs(q['applied_signed_sum_mm']-float(m['pathProgressApplied_mm']))>0.002:raise ValueError(n,'header delta mismatch')
            if q['accepted_records_sum']!=integer(m['pathProgressAccepted']):raise ValueError(n,'header count mismatch')
            for i,r in enumerate(rows):
                if r['pathProgressAbsDelta_mm']+1e-5<abs(r['pathProgressDelta_mm']):raise ValueError(n,'absolute delta')
                if integer(r['pathProgressReasonMask'])&~1023:raise ValueError(n,'reason mask')
                previous=rows[i-1]['pathProgressTotal_mm'] if i else 0
                if abs(r['pathProgressTotal_mm']-previous-r['pathProgressDelta_mm'])>0.002:raise ValueError(n,'cumulative delta')
            if n==14128:
                with (OUT/'14128_tail.csv').open('w',encoding='utf-8-sig',newline='') as f:
                    tail=[r for r in rows if r['cntlog']>=rows[-1]['cntlog']-850]
                    w=csv.DictWriter(f,list(sample(tail[0])));w.writeheader();w.writerows(sample(r) for r in tail)
        summary[str(n)]=q
        print('LOG',n,json.dumps({k:v for k,v in q.items() if k not in ['metadata','numerical_updates','state_changes','end','peak_y','sha256']},ensure_ascii=False))
        if n==14128:
            print('STATES',json.dumps(q['state_changes']))
            print('LATEST_EVALUATIONS',json.dumps(q['numerical_updates'][-10:]))
            print('END',q['end'])
            print('PEAK_Y',q['peak_y'])
    for n,q in summary.items():
        if hashlib.sha256((LOG/f'{n}.csv').read_bytes()).hexdigest()!=q['sha256']:raise ValueError(n,'original changed')
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    print('PASS: all rows validated; delta/header/cumulative/interval checks; 8 original hashes unchanged')


def details():
    for n in [14121,14122,14123,14124,14125,14128]:
        _,_,m,names,rows=load(n)
        path=[r for r in rows if 'pathProgressReason' in r]
        disabled=[r for r in path if r['pathProgressReason']==0]
        q=dict(log=n,stopped=m['emcStop'],end_age=m.get('pathObservationAgeAtEnd_ms'),
               end_confidence=m.get('pathConfidenceAtEnd'),line_alpha=m.get('shortcutSettings.lineAlpha_x1000'),
               max_y=max(abs(r['pathErrorY_mm']) for r in rows),
               disabled_rows=len(disabled),disabled_tracking_rows=sum(r['pathState'] in [1,5] for r in disabled))
        if 'slipV2Flags' in names:
            q['disabled_with_slip_rows']=sum(r['slipV2Flags']!=0 for r in disabled)
            q['disabled_tracking_with_slip_rows']=sum(r['slipV2Flags']!=0 and r['pathState'] in [1,5] for r in disabled)
        print('DISABLE',json.dumps(q))
        if n==14128:
            print('TYPICAL_TAIL')
            for t in range(2650,3110,25):
                r=min(rows,key=lambda r:abs(r['cntlog']-t));print(json.dumps(sample(r)))
            for threshold in [800,100]:
                last=max((r for r in rows if r['sensor_max']>=threshold),key=lambda r:r['cntlog'])
                print('LAST_BRIGHT',threshold,sample(last))
            print('HEAD',json.dumps({k:v for k,v in m.items() if k.startswith(('path','shortcut','tgtParam.'))}))
    _,_,m,_,source=load(14126)
    starts=[r for i,r in enumerate(source) if r['courseMarker']==3 and (i==0 or source[i-1]['courseMarker']!=3)]
    print('PRIMARY_CROSS',[(r['cntlog'],round(r['distance_mm'],2),r['imuAngle_Z']) for r in starts])


if __name__=='__main__':
    import sys
    details() if len(sys.argv)>1 and sys.argv[1]=='details' else main()
