"""複数速度ログの時間応答・左右換算差・実効幅を段階的に分けて解析する。"""
from pathlib import Path
import csv,json,hashlib,os
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/multispeed_width_13006_13044';OUT.mkdir(exist_ok=True)
os.environ['MPLCONFIGDIR']=str(OUT/'.mplcache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
LOG=Path('F:/Dropbox/Document/robotrace/Log/v2');RAD=np.pi/180;PPM=58.019
TRAIN_GROUPS=[13006,13007,13012,13017];TEST_GROUPS=[13022,13030,13035,13040]
MODES=['PRIMARY','PATH','SHORTCUT','DISTANCE','SLIP'];WIDTH_MODELS=['constant','speed','curvature','speed_curvature']
RIDGES=[0.,.01,.1,1.];V0=1.5;VS=1.;KS=5.;OLD_B=117.8374851692594
RESPONSE_GRID=[(delay,tc) for delay in [0,2,4,6,8,10,12] for tc in [0,2,4,6,8,10,12]]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(name,rows):
 with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:
  w=csv.DictWriter(f,list(dict.fromkeys(k for r in rows for k in r)));w.writeheader();w.writerows(rows)
def col(rows,key):return np.array([r[key] for r in rows],float)
def metadata(path):
 with path.open(encoding='utf-8-sig') as f:
  head=next(f);m=dict(z.split('=',1) for z in head.strip().split(',') if '=' in z);names=next(f).strip().split(',')
 return m,names

def load_all(split):
 groups=TRAIN_GROUPS if split=='train' else TEST_GROUPS;data=[];inventory=[]
 for n in range(13006,13045):
  path=LOG/f'{n}.csv'
  if not path.exists():continue
  meta,names=metadata(path);mode=meta.get('requestedMode');group=int(meta.get('primaryLogNumber',0));status='outside_split'
  if group not in groups:continue
  if float(meta.get('emcStop',1))!=0:status='excluded_emergency_stop'
  elif mode not in MODES:status='excluded_mode'
  elif meta.get('gitCommit')!='8be46b1' or meta.get('buildDate')!='Oct  4 2026' or meta.get('buildTime')!='23:27:00':status='excluded_build'
  else:status='included'
  inventory.append(dict(log=n,group=group,split=split,mode=mode,actual_mode=meta.get('optimalTrace'),status=status,sha256=sha(path)))
  if status!='included':continue
  with path.open(encoding='utf-8-sig') as f:next(f);rows=list(csv.DictReader(f))
  required=['cntlog','encTotalL','encTotalR','imuAngle_Z','slipFlag','slipFlagLat','imuTemp_C','targetSpeed','encTotalOptimal','x','y']
  assert set(required)<=set(names) and len(rows)==int(meta['logExpectedRows'])
  a={k:np.array([float(r[k]) for r in rows]) for k in required};assert all(np.isfinite(v).all() for v in a.values())
  assert meta['imuCalibrationValid']=='1' and float(meta['encoderPulsePerMeter'])==58019
  if mode=='PRIMARY':assert meta['distanceScaleVerified']=='1'
  else:
   primary,_=metadata(LOG/f'{group}.csv');assert primary['distanceScaleVerified']=='1' and primary['closureValid']=='1'
  t=a['cntlog']/1000;s=(a['encTotalL']+a['encTotalR'])/(2*PPM);w=(a['encTotalL']-a['encTotalR'])/PPM;h=a['imuAngle_Z']*RAD
  assert np.all(np.diff(t)>0) and np.all(np.diff(s)>0)
  tt=np.arange(t[0],t[-1]+.0001,.001);cum=np.column_stack([np.interp(tt,t,w),np.interp(tt,t,s)])
  starts=t[0]+.25+np.arange(int(np.floor((t[-1]-.15-(t[0]+.25))/.1)))*.1;ends=starts+.1
  assert np.all(ends<=t[-1]-.15+1e-9)
  base=[];dropped=0
  for lo,hi in zip(starts,ends):
   idx=np.searchsorted(t,[lo,hi]);l=max(0,idx[0]-1);r=min(len(t),idx[1]+1)
   maxgap=float(np.diff(t[l:r]).max())
   if maxgap>.0100001:dropped+=1;continue
   v=float((np.interp(hi,t,s)-np.interp(lo,t,s))/.1/1000);om=float((np.interp(hi,t,h)-np.interp(lo,t,h))/.1)
   rawdiff=float((np.interp(hi,t,w)-np.interp(lo,t,w))/.1)
   slip=bool(np.any(a['slipFlag'][l:r]>.5) or np.any(a['slipFlagLat'][l:r]>.5))
   base.append(dict(log=n,group=group,split=split,mode=mode,start_s=float(lo),end_s=float(hi),dt_s=.1,speed_mps=v,omega_rads=om,omega_dps=om/RAD,
      curvature_radpm=om/v,raw_diff_mmps=rawdiff,slip_any=int(slip),max_gap_ms=maxgap*1000,temperature_C=float(np.mean(a['imuTemp_C'][l:r])),
      target_speed_mps=float(np.mean(a['targetSpeed'][l:r]))/PPM))
  for i,z in enumerate(base):
   if 0<i<len(base)-1 and base[i+1]['start_s']-base[i-1]['start_s']<.21:
    alpha=(base[i+1]['omega_rads']-base[i-1]['omega_rads'])/(base[i+1]['start_s']-base[i-1]['start_s'])/RAD
   else:alpha=float('inf')
   z['alpha_dps2']=alpha;z['straight']=int(abs(om:=z['omega_dps'])<10 and abs(alpha)<100 and not z['slip_any'])
   z['steady_turn']=int(abs(om)>=30 and abs(alpha)<300 and not z['slip_any'])
  data.append(dict(n=n,group=group,split=split,mode=mode,meta=meta,a=a,t=t,s=s,w=w,h=h,tt=tt,cum=cum,base=base,dropped=dropped,sha256=sha(path),filter_cache={0:cum}))
 return data,inventory

def filtered_cumulative(d,tc_ms):
 if tc_ms not in d['filter_cache']:
  alpha=np.exp(-1/tc_ms);length=int(np.ceil(32*tc_ms));kernel=(1-alpha)*alpha**np.arange(length)
  raw=d['cum'];out=np.column_stack([np.convolve(raw[:,j]-raw[0,j],kernel,mode='full')[:len(raw)]+raw[0,j] for j in [0,1]])
  d['filter_cache'][tc_ms]=out
 return d['filter_cache'][tc_ms]
def response_rows(data,delay_ms,tc_ms):
 rows=[]
 for d in data:
  filtered=filtered_cumulative(d,tc_ms);tt=d['tt'];b=d['base'];lo=col(b,'start_s')-delay_ms/1000;hi=col(b,'end_s')-delay_ms/1000
  diff=(np.interp(hi,tt,filtered[:,0])-np.interp(lo,tt,filtered[:,0]))/col(b,'dt_s')
  vf=(np.interp(hi,tt,filtered[:,1])-np.interp(lo,tt,filtered[:,1]))/col(b,'dt_s')/1000
  rows.extend([{**z,'response_diff_mmps':float(dw),'response_speed_mps':float(v)} for z,dw,v in zip(b,diff,vf)])
 return rows

def weights(rows,balance_mode=True):
 logs=[r['log'] for r in rows];modes=[r['mode'] for r in rows];counts={n:logs.count(n) for n in set(logs)}
 mode_logs={mode:len({r['log'] for r in rows if r['mode']==mode}) for mode in set(modes)}
 w=np.array([1/counts[r['log']]/(mode_logs[r['mode']] if balance_mode else 1) for r in rows]);return w*len(w)/w.sum()
def robust_fit(a,y,w,ridge=0.,penalty=None):
 if penalty is None:penalty=np.zeros(a.shape[1])
 scale=np.maximum(np.sqrt(np.mean(a*a,axis=0)),1e-9);az=a/scale;ww=w.copy()
 for _ in range(10):
  gram=az.T@(ww[:,None]*az)+ridge*len(y)*np.diag(penalty)
  c=np.linalg.lstsq(gram,az.T@(ww*y),rcond=1e-12)[0];res=y-az@c
  sd=max(1.,1.4826*np.median(np.abs(res-np.median(res))));ww=w*np.minimum(1,1.5*sd/np.maximum(np.abs(res),1e-12))
 return c/scale

def fit_instrument(rows):
 om=col(rows,'omega_rads');a=np.column_stack([om*np.array([r['mode']==m for r in rows]) for m in MODES]+[col(rows,'response_speed_mps'),np.ones(len(rows))])
 c=robust_fit(a,col(rows,'response_diff_mmps'),weights(rows));return dict(B_mode=dict(zip(MODES,c[:5].tolist())),c_velocity_mm_per_m=float(c[5]),c0_mmps=float(c[6]))
def instrument_errors(rows,f):
 b=np.array([f['B_mode'][r['mode']] for r in rows]);return ((col(rows,'response_diff_mmps')-f['c_velocity_mm_per_m']*col(rows,'response_speed_mps')-f['c0_mmps'])/b-col(rows,'omega_rads'))/RAD

def balanced_rms(rows,error):
 vals=[]
 for mode in sorted({r['mode'] for r in rows}):
  ms=[]
  for n in sorted({r['log'] for r in rows if r['mode']==mode}):
   mask=np.array([r['log']==n for r in rows]);ms.append(float(np.mean(error[mask]**2)))
  vals.append(np.mean(ms))
 return float(np.sqrt(np.mean(vals)))

def select_response(data):
 search=[];foldrows=[]
 for delay,tc in RESPONSE_GRID:
  rows=response_rows(data,delay_ms=delay,tc_ms=tc);pred=np.zeros(len(rows))
  for group in TRAIN_GROUPS:
   learn=[r for r in rows if r['group']!=group];ix=np.array([r['group']==group for r in rows]);val=[r for r in rows if r['group']==group]
   fitted=fit_instrument(learn);e=instrument_errors(val,fitted);pred[ix]=e
   foldrows.append(dict(delay_ms=delay,tc_ms=tc,group=group,rms_dps=balanced_rms(val,e)))
  score=balanced_rms(rows,pred);full=fit_instrument(rows)
  search.append(dict(delay_ms=delay,tc_ms=tc,series_LOO_rms_dps=score,**{f'B_{k}_mm':v for k,v in full['B_mode'].items()},c_velocity_mm_per_m=full['c_velocity_mm_per_m'],c0_mmps=full['c0_mmps']))
 best=min(search,key=lambda z:z['series_LOO_rms_dps']);near=[r for r in search if r['series_LOO_rms_dps']<=best['series_LOO_rms_dps']*1.01]
 # 同程度なら自由パラメータが少ない応答モデルを選ぶ。
 selected=min(near,key=lambda z:(int(z['delay_ms']>0)+int(z['tc_ms']>0),z['series_LOO_rms_dps']))
 write('response_train_search.csv',search);write('response_series_LOO.csv',foldrows)
 print('selected response',selected,flush=True);return selected

def separate_asymmetry(rows,response_fit,write_history=True):
 straight=[r for r in rows if r['straight']];turns=[r for r in rows if r['steady_turn']]
 assert len(straight)>=30 and len(turns)>=30
 bmode=dict(response_fit['B_mode']);history=[];c=response_fit['c_velocity_mm_per_m'];c0=response_fit['c0_mmps']
 for iteration in range(5):
  b=np.array([bmode[r['mode']] for r in straight]);y=col(straight,'response_diff_mmps')-b*col(straight,'omega_rads')
  a=np.c_[col(straight,'response_speed_mps'),np.ones(len(straight))];c,c0=robust_fit(a,y,weights(straight))
  for mode in MODES:
   rr=[r for r in turns if r['mode']==mode]
   if len(rr)<5:continue
   om=col(rr,'omega_rads');y=col(rr,'response_diff_mmps')-c*col(rr,'response_speed_mps')-c0
   bmode[mode]=float(robust_fit(om[:,None],y,weights(rr,False))[0])
  history.append(dict(iteration=iteration,c_velocity_mm_per_m=float(c),c0_mmps=float(c0),straight_count=len(straight),steady_turn_count=len(turns),**{f'B_{m}_mm':v for m,v in bmode.items()}))
 if write_history:
  write('asymmetry_stage_history.csv',history)
  print('separated asymmetry',history[-1],flush=True)
 return dict(c_velocity_mm_per_m=float(c),c0_mmps=float(c0),B_steady_mode=bmode,straight_windows=len(straight),steady_turn_windows=len(turns),
  straight_speed_min=float(col(straight,'speed_mps').min()),straight_speed_max=float(col(straight,'speed_mps').max()))

def width_design(rows,model):
 om=col(rows,'omega_rads');v=col(rows,'speed_mps');kk=np.abs(col(rows,'curvature_radpm'));cols=[om];names=['B0_mm']
 if model in ['speed','speed_curvature']:cols.append(om*(v-V0)/VS);names.append('Bv_mm_per_mps')
 if model in ['curvature','speed_curvature']:cols.append(om*kk/KS);names.append('Bk_mm_per_5radpm')
 return np.column_stack(cols),names

def fit_width(rows,model,ridge,asym):
 a,names=width_design(rows,model);y=col(rows,'response_diff_mmps')-asym['c_velocity_mm_per_m']*col(rows,'response_speed_mps')-asym['c0_mmps']
 c=robust_fit(a,y,weights(rows,False),ridge,[0]+[1]*(len(names)-1))
 return dict(model=model,ridge=ridge,coefficients=dict(zip(names,c.tolist())))
def width_predict(rows,f,asym):
 c=f['coefficients'];b=c['B0_mm']+c.get('Bv_mm_per_mps',0)*(col(rows,'speed_mps')-V0)/VS+c.get('Bk_mm_per_5radpm',0)*np.abs(col(rows,'curvature_radpm'))/KS
 y=col(rows,'response_diff_mmps')-asym['c_velocity_mm_per_m']*col(rows,'response_speed_mps')-asym['c0_mmps'];err=(y/b-col(rows,'omega_rads'))/RAD
 return b,err

def select_width(rows,asym):
 chosen={};search=[]
 for mode in MODES:
  rr=[r for r in rows if r['mode']==mode];groups=sorted({r['group'] for r in rr})
  for model in WIDTH_MODELS:
   for ridge in ([0.] if model=='constant' else RIDGES):
    preds=np.zeros(len(rr));valid=True
    for group in groups:
     learn=[r for r in rr if r['group']!=group];ix=np.array([r['group']==group for r in rr]);val=[r for r in rr if r['group']==group]
     f=fit_width(learn,model=model,ridge=ridge,asym=asym);b,e=width_predict(val,f,asym);preds[ix]=e;valid &= bool(np.all((b>=70)&(b<=200)))
    search.append(dict(mode=mode,model=model,ridge=ridge,conditional_series_LOO_rms_dps=balanced_rms(rr,preds),width_range_valid=valid))
   eligible=[r for r in search if r['mode']==mode and r['model']==model and r['width_range_valid']]
   best=min(eligible,key=lambda z:z['conditional_series_LOO_rms_dps']);f=fit_width(rr,model=model,ridge=best['ridge'],asym=asym);f['mode']=mode;chosen[mode,model]=f
   print('width',mode,model,best['conditional_series_LOO_rms_dps'],f['coefficients'],flush=True)
 write('width_train_search.csv',search);return chosen

def evaluate(rows,models,asym,label='separated'):
 metrics=[];pred=[]
 for (mode,model),f in models.items():
  rr=[r for r in rows if r['mode']==mode];b,e=width_predict(rr,f,asym)
  for n in sorted({r['log'] for r in rr}):
   sel=np.array([r['log']==n for r in rr]);turn=sel&(np.abs(col(rr,'omega_dps'))>=30);ns=sel&(col(rr,'slip_any')==0);r0=next(r for r in rr if r['log']==n)
   metrics.append(dict(log=n,group=r0['group'],split=r0['split'],mode=mode,model=model,dataset=label,windows=int(sel.sum()),
      angular_rms_dps=float(np.sqrt(np.mean(e[sel]**2))),turn_rms_dps=float(np.sqrt(np.mean(e[turn]**2))),no_slip_rms_dps=float(np.sqrt(np.mean(e[ns]**2))) if ns.any() else float('nan'),
      mean_error_dps=float(e[sel].mean()),covered_angle_error_deg=float(np.sum(e[sel]*col(rr,'dt_s')[sel])),B_min_mm=float(b[sel].min()),B_max_mm=float(b[sel].max()),width_range_valid=bool(np.all((b[sel]>=70)&(b[sel]<=200)))))
  pred.extend([dict(log=r['log'],mode=mode,model=model,start_s=r['start_s'],predicted_B_mm=float(bb),angular_error_dps=float(ee)) for r,bb,ee in zip(rr,b,e)])
 return metrics,pred

def summarize(metrics):
 rows=[]
 for split in ['train','test']:
  for mode in MODES:
   for model in WIDTH_MODELS:
    rr=[r for r in metrics if r['split']==split and r['mode']==mode and r['model']==model]
    rows.append(dict(split=split,mode=mode,model=model,logs=len(rr),angular_rms_dps=float(np.sqrt(np.mean([r['angular_rms_dps']**2 for r in rr]))),
       covered_angle_error_rms_deg=float(np.sqrt(np.mean([r['covered_angle_error_deg']**2 for r in rr]))),B_min_mm=min(r['B_min_mm'] for r in rr),B_max_mm=max(r['B_max_mm'] for r in rr)))
 return rows

def coverage(rows):
 result=[]
 for split in ['train','test']:
  for mode in MODES:
   rr=[r for r in rows if r['mode']==mode and r['split']==split];v=col(rr,'speed_mps');k=np.abs(col(rr,'curvature_radpm'))
   for vl,vh in zip([.5,1.1,1.5,2.,2.5],[1.1,1.5,2.,2.5,3.5]):
    for kl,kh in zip([0,.5,2.,5.,8.],[.5,2.,5.,8.,15.]):
     mask=(v>=vl)&(v<vh)&(k>=kl)&(k<kh);result.append(dict(split=split,mode=mode,speed_low=vl,speed_high=vh,curvature_low=kl,curvature_high=kh,windows=int(mask.sum()),logs=len({r['log'] for r,m in zip(rr,mask) if m})))
 return result

if __name__=='__main__':
 train,inventory=load_all('train');assert len(train)==14
 print('train windows',sum(len(d['base']) for d in train),flush=True)
 response=select_response(train);tr=response_rows(train,delay_ms=response['delay_ms'],tc_ms=response['tc_ms'])
 instrument=fit_instrument(tr);asym=separate_asymmetry(tr,instrument);models=select_width(tr,asym)
 frozen=dict(train_groups=TRAIN_GROUPS,test_groups=TEST_GROUPS,train_logs=[d['n'] for d in train],response=response,asymmetry=asym,width_models=list(models.values()),
  V0_mps=V0,VS_mps=VS,KS_radpm=KS,window_s=.1,encoder_mean_uses_raw_LR=True,CROSS_pose_used=False,positive_delay_filters_past_encoder=True,
  input_train_hashes={str(d['n']):d['sha256'] for d in train},common_calibration_fixed_during_width_LOO=True,test_previously_seen=True)
 (OUT/'frozen_parameters.json').write_text(json.dumps(frozen,indent=2),encoding='utf-8');frozen_sha=sha(OUT/'frozen_parameters.json')
 print('FROZEN before test evaluation',flush=True)
 test,inv2=load_all('test');assert len(test)==17;data=train+test;inventory+=inv2
 te=response_rows(test,delay_ms=response['delay_ms'],tc_ms=response['tc_ms']);allrows=tr+te;write('windows.csv',allrows)
 metrics,pred=evaluate(allrows,models,asym);write('per_log_metrics.csv',metrics);write('predictions.csv',pred);summary=summarize(metrics);write('summary.csv',summary)
 write('coverage.csv',coverage(allrows));write('inventory.csv',inventory)
 health=[]
 for d in data:
  v=col(d['base'],'speed_mps');kk=np.abs(col(d['base'],'curvature_radpm'))
  health.append(dict(log=d['n'],group=d['group'],split=d['split'],mode=d['mode'],actual_mode=d['meta']['optimalTrace'],raw_rows=len(d['t']),expected_rows=d['meta']['logExpectedRows'],
    windows=len(d['base']),dropped_low_time_resolution_windows=d['dropped'],v_min=v.min(),v_p10=np.percentile(v,10),v_median=np.median(v),v_p90=np.percentile(v,90),v_max=v.max(),
    curvature_max=kk.max(),v_abs_curvature_correlation=float(np.corrcoef(v,kk)[0,1]),straight_windows=sum(r['straight'] for r in d['base']),steady_turn_windows=sum(r['steady_turn'] for r in d['base']),
    slip_windows=sum(r['slip_any'] for r in d['base']),battery_V=d['meta']['batteryVoltage_V'],max_time_gap_ms=np.diff(d['t']).max()*1000,sha256=d['sha256'],cross_pose_updates=0))
 write('health.csv',health)
 assert sha(OUT/'frozen_parameters.json')==frozen_sha and all(sha(LOG/f'{d["n"]}.csv')==d['sha256'] for d in data)
 (OUT/'validation.json').write_text(json.dumps(dict(status='passed',log_count=len(data),windows=len(allrows),frozen_sha256=frozen_sha,input_hashes_unchanged=True,firmware_modified=False,
   cross_pose_used=False,sparse_gyro_reintegrated=False,raw_encoder_mean=True,train_test_series_disjoint=True,script_sha256=sha(Path(__file__))),indent=2),encoding='utf-8')
 print(json.dumps(summary,indent=2),flush=True)
