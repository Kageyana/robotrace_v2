"""Independent encoder/gyro reintegration. CSV x,y and derived path fields are never read."""
from pathlib import Path
import csv,json,hashlib,os,numpy as np
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).resolve().parents[2]/'analysis/yi_12838_12848/.mplcache'))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'analysis/yi_12838_12848';OUT.mkdir(exist_ok=True)
LOG=Path('F:/Dropbox/Document/robotrace/Log/v2');CW=list(range(12838,12843));CCW=list(range(12844,12849));Y=np.arange(-40,0.01,.5)
COLS=['cntlog','encTotalL','encTotalR','gyroVal_Z','courseMarker','slipFlag','slipFlagLat']+[f'lSensorCari{i}' for i in range(10)]
D={}
def writecsv(name,rows):
 with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
def runs(mask):
 edges=np.diff(np.r_[False,mask,False].astype(int));return list(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)-1))
def anchor(d,threshold=1000,count=8):
 # Continuous count>=8 runs, distances at interpolated order-statistic threshold crossings.
 val=np.sort(d['a'][:,7:],axis=1)[:,10-count];s=d['s'];out=[];details=[]
 for l,r in runs(val>=threshold):
  if l==0 or r==len(s)-1:continue
  begin=np.interp(threshold,[val[l-1],val[l]],[s[l-1],s[l]])
  end=np.interp(threshold,[val[r+1],val[r]],[s[r+1],s[r]])
  if end-begin<3 or end-begin>80:continue
  mid=(begin+end)/2;out.append(mid);details.append((begin,end,mid,l,r))
 return np.array(out),details
def load_logs():
    health=[];events=[]
    for n in CW+CCW:
     path=LOG/f'{n}.csv'
     with path.open(encoding='utf-8-sig') as f:
      meta=dict(x.split('=',1) for x in next(f).strip().split(',') if '=' in x)
      a=np.array([[float(row[c]) for c in COLS] for row in csv.DictReader(f)])
     ds=np.r_[0,(.99675455*np.diff(a[:,1])+1.00324545*np.diff(a[:,2]))/2/58.019];s=np.cumsum(ds);m=a[:,4];ci=np.flatnonzero((m==3)&(np.r_[0,m[:-1]]!=3));d={'a':a,'s':s,'ds':ds,'ci':ci,'meta':meta};D[n]=d
     assert np.all(np.isfinite(a)) and np.all(np.diff(a[:,0])>0) and np.all(np.diff(s)>0), n
     assert len(a)==int(meta['logExpectedRows']) and float(meta['emcStop'])==0, n
     assert float(meta['optimalTrace'])==0 and int(meta['encoderPulsePerMeter'])==58019, n
     assert meta['imuCalibrationValid']=='1' and meta['distanceScaleVerified']=='1', n
     ac,detail=anchor(d);d['line']=ac;d['marker']=s[ci]
     assert len(ac)==len(ci)==6 and np.all(np.abs(s[ci]-ac)<100), n
     health.append(dict(log=n,rows=len(a),emcStop=meta['emcStop'],monotonic=bool(np.all(np.diff(a[:,0])>0)),max_gap_ms=np.diff(a[:,0]).max(),CROSS=len(ci),line_events=len(ac),optimalTrace=meta['optimalTrace'],git=meta['gitCommit'],battery=meta['batteryVoltage_V'],sgMarkerAtLogEnd=meta['sgMarkerAtLogEnd'],closureValid=meta['closureValid'],closureReason=meta['closureReason'],slip_rows=int(np.count_nonzero(a[:,5])),slip_lat_rows=int(np.count_nonzero(a[:,6])),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
     # Keep source provenance and health in CSV; avoid dumping rows on every import helper run.
     for k,(b,e,mid,l,r) in enumerate(detail):events.append(dict(log=n,event=k+1,start_mm=b,end_mm=e,center_mm=mid,width_mm=e-b,marker_minus_line_mm=(s[ci[k]]-mid if k<len(ci) else np.nan),peak_channels_1000=int(np.max((a[l:r+1,7:]>=1000).sum(1)))))
    writecsv('health.csv',health);writecsv('line_events.csv',events)

def trajectory(d,yi=0,kg=0,bias=0,srel=0,method='right'):
 a=d['a'];dt=np.r_[0,np.diff(a[:,0])]/1000;omega=a[:,3].copy()
 if method=='trapezoid':omega[1:]=(omega[1:]+omega[:-1])/2
 thd=np.deg2rad(omega*(1+kg)+bias)*dt;th=np.cumsum(thd);mid=th-.5*thd
 ds=np.r_[0,(.99675455*(1-srel/2)*np.diff(a[:,1])+1.00324545*(1+srel/2)*np.diff(a[:,2]))/2/58.019]
 p=np.column_stack([np.cumsum(ds*np.sin(mid)-yi*thd*np.cos(mid)),np.cumsum(ds*np.cos(mid)+yi*thd*np.sin(mid))])
 return p,th
Q=np.linspace(0,1,1001)
def laps(n,kind='marker',yi=0,kg=0,bias=0,srel=0,method='right',shift=0):
 d=D[n];p,_=trajectory(d,yi,kg,bias,srel,method);bounds=d[kind]+shift
 assert np.all(np.diff(bounds)>0) and bounds[0]>=d['s'][0] and bounds[-1]<=d['s'][-1], (n,kind,shift)
 return [np.column_stack([np.interp(b+(e-b)*Q,d['s'],p[:,j]) for j in range(2)]) for b,e in zip(bounds[:-1],bounds[1:])]
def align(a,b):
 aa=a-a.mean(0);bb=b-b.mean(0);u,_,vt=np.linalg.svd(bb.T@aa);r=u@np.diag([1,np.linalg.det(u@vt)])@vt;aligned=bb@r+a.mean(0);res=aligned-a
 return np.mean(np.sum(res**2,axis=1)),aligned,res
PAIRS=list(zip(CW,CCW))
def scores(yi,kind='marker',trim=0,kg=0,bias=0,srel=0,method='right',pairs=PAIRS,shift=0):
 ll={n:laps(n,kind,yi,kg,bias,srel,method,shift) for n in CW+CCW};ix=(Q>=trim)&(Q<=1-trim);out=[]
 for c,w in pairs:
  for k,(a,b) in enumerate(zip(ll[c],ll[w])):out.append(align(a[ix],b[::-1][ix])[0])
 return np.array(out)
# Per-channel enter/exit midpoints converted to a common axle-center crossing.
with (OUT/'sensor_centers.csv').open(encoding='utf-8') as f:geom=sorted([r for r in csv.DictReader(f) if r['board']=='line'],key=lambda r:float(r['X_mm']))
F=np.array([float(r['forward_mm']) for r in geom]);LX=np.array([float(r['X_mm']) for r in geom]);axlerows=[]
assert len(F)==10 and np.max(np.abs(F-F[::-1]))<.01, 'Line-sensor geometry must be mirror-symmetric in forward position'
def axleanchors(d,threshold=1000):
 s=d['s'];centers=[]
 for event in d['line']:
  mids=[];xx=[];wid=[]
  for ch in [0,1,2,3,6,7,8,9]:
   val=d['a'][:,7+ch];mask=(val>=threshold)&(abs(s-event)<65);candidates=[]
   for l,r in runs(mask):
    if l==0 or r==len(s)-1 or val[l-1]>=threshold or val[r+1]>=threshold:continue
    begin=np.interp(threshold,[val[l-1],val[l]],[s[l-1],s[l]]);end=np.interp(threshold,[val[r+1],val[r]],[s[r+1],s[r]])
    if 8<end-begin<50:candidates.append((abs((begin+end)/2-event),(begin+end)/2,end-begin))
   if candidates:
    _,mid,width=min(candidates);mids.append(mid+F[ch]);xx.append(LX[ch]);wid.append(width)
  assert len(mids)>=6,(event,len(mids))
  intercept,slope=np.linalg.lstsq(np.column_stack([np.ones(len(xx)),xx]),mids,rcond=None)[0];fit=intercept+slope*np.array(xx);centers.append(intercept)
  axlerows.append(dict(log=next(n for n,v in D.items() if v is d),threshold=threshold,event=len(centers),axle_cross_mm=intercept,line_to_axle_mm=intercept-event,channel_fit_rms_mm=np.sqrt(np.mean((np.array(mids)-fit)**2)),cross_angle_deg=np.rad2deg(np.arctan(slope)),width_mm=np.mean(wid),channels=len(mids)))
 return np.array(centers)

def main():
    load_logs()
    curves=[];summary=[];individual=[]
    for kind in ['marker','line']:
     for trim in [0,.03,.05,.10]:
      ss=np.array([scores(y,kind,trim) for y in Y]);rms=np.sqrt(ss.mean(1));best=int(np.argmin(rms));summary.append(dict(anchor=kind,trim=trim,yi_best=Y[best],rms_best=rms[best],rms_0=np.sqrt(scores(0,kind,trim).mean()),rms_neg63=np.sqrt(scores(-63,kind,trim).mean())))
      curves.extend(dict(anchor=kind,trim=trim,yi=y,rms=r) for y,r in zip(Y,rms))
      if trim==0:
       for j,(c,w) in enumerate(PAIRS):
        ix=slice(j*5,(j+1)*5);b=np.argmin(ss[:,ix].mean(1));individual.append(dict(anchor=kind,unit=f'pair{j+1}',cw=c,ccw=w,yi_best=Y[b],rms_best=np.sqrt(ss[b,ix].mean())))
       for k in range(5):
        b=np.argmin(ss[:,k::5].mean(1));individual.append(dict(anchor=kind,unit=f'lap{k+1}',cw='all',ccw='all',yi_best=Y[b],rms_best=np.sqrt(ss[b,k::5].mean())))
       for j,(c,w) in enumerate(PAIRS):
        for k in range(5):
         b=np.argmin(ss[:,j*5+k]);individual.append(dict(anchor=kind,unit=f'pair{j+1}_lap{k+1}',cw=c,ccw=w,yi_best=Y[b],rms_best=np.sqrt(ss[b,j*5+k])))
     print('summary',summary[-4:],flush=True)
    writecsv('anchor_summary.csv',summary);writecsv('rms_vs_yi.csv',curves);writecsv('individual_optima.csv',individual)
    for d in D.values():d['axle']=axleanchors(d)
    EXT=np.arange(-120,20.01,.5);extended=[];newsum=[]
    for kind in ['line','axle']:
     for trim in [0,.03,.05,.10]:
      ss=np.array([scores(y,kind,trim) for y in EXT]);rr=np.sqrt(ss.mean(1));b=np.argmin(rr)
      newsum.append(dict(anchor=kind,trim=trim,yi_best=EXT[b],rms_best=rr[b],rms_0=np.sqrt(scores(0,kind,trim).mean()),rms_neg63=np.sqrt(scores(-63,kind,trim).mean()),rms_neg20_5=np.sqrt(scores(-20.5,kind,trim).mean())))
      extended.extend(dict(anchor=kind,trim=trim,yi=y,rms=r) for y,r in zip(EXT,rr))
      if trim==0:
       for j,(c,w) in enumerate(PAIRS):
        ix=slice(j*5,(j+1)*5);b=np.argmin(ss[:,ix].mean(1));individual.append(dict(anchor=kind+'_extended',unit=f'pair{j+1}',cw=c,ccw=w,yi_best=EXT[b],rms_best=np.sqrt(ss[b,ix].mean())))
       for k in range(5):
        b=np.argmin(ss[:,k::5].mean(1));individual.append(dict(anchor=kind+'_extended',unit=f'lap{k+1}',cw='all',ccw='all',yi_best=EXT[b],rms_best=np.sqrt(ss[b,k::5].mean())))
       for j,(c,w) in enumerate(PAIRS):
        for k in range(5):
         b=np.argmin(ss[:,j*5+k]);individual.append(dict(anchor=kind+'_extended',unit=f'pair{j+1}_lap{k+1}',cw=c,ccw=w,yi_best=EXT[b],rms_best=np.sqrt(ss[b,j*5+k])))
     print('extended',newsum[-4:],flush=True)
    writecsv('extended_summary.csv',newsum);writecsv('extended_curves.csv',extended);writecsv('individual_optima.csv',individual);writecsv('axle_anchors.csv',axlerows)
    # Global anchor displacement sensitivity: this reveals phase/lever-arm confounding.
    phase=[]
    for shift in [-40,-20,0,20,40,60]:
     ys=np.arange(shift-30,shift-10.01,.5);rr=np.array([np.sqrt(scores(y,shift=shift).mean()) for y in ys]);b=np.argmin(rr);phase.append(dict(anchor_shift_mm=shift,yi_best=ys[b],rms_best=rr[b]))
    writecsv('phase_sensitivity.csv',phase);print('phase',phase,flush=True)
    # Threshold sensitivity with geometric correction retained.
    sensitivity=[]
    for threshold in [800,1000,1200,1500,2000]:
     for d in D.values():d['axle_thr']=axleanchors(d,threshold)
     ys=np.arange(-10,15.01,.5);rr=np.array([np.sqrt(scores(y,'axle_thr').mean()) for y in ys]);b=np.argmin(rr);sensitivity.append(dict(threshold=threshold,yi_best=ys[b],rms_best=rr[b],rms_0=np.sqrt(scores(0,'axle_thr').mean())))
    writecsv('threshold_sensitivity.csv',sensitivity);print('threshold',sensitivity,flush=True)
    # Numerical integration model and encoder scale sensitivity.
    checks=[]
    for kind in ['marker','axle']:
     ys=np.arange(-30,15.01,.5)
     for method in ['right','trapezoid']:
      rr=np.array([np.sqrt(scores(y,kind,method=method).mean()) for y in ys]);b=np.argmin(rr);checks.append(dict(anchor=kind,test='integration_'+method,value=0,yi_best=ys[b],rms_best=rr[b]))
     for sr in np.linspace(-.01,.01,21):
      rr=np.array([np.sqrt(scores(y,kind,srel=sr).mean()) for y in ys]);b=np.argmin(rr);checks.append(dict(anchor=kind,test='encoder_relative',value=sr,yi_best=ys[b],rms_best=rr[b]))
    writecsv('model_sensitivity.csv',checks)
    # Additional gyro scale is applied to already corrected gyro values; no second -0.9924.
    gyro=[];ys=np.arange(-5,10.01,.5)
    for kg in np.linspace(-.002,.002,21):
     rr=np.array([np.sqrt(scores(y,'axle',kg=kg).mean()) for y in ys]);b=np.argmin(rr);gyro.append(dict(test='scale',value=kg,yi_best=ys[b],rms_best=rr[b],rms_at0=np.sqrt(scores(0,'axle',kg=kg).mean())))
    for bias in np.linspace(-.3,.3,25):
     rr=np.array([np.sqrt(scores(y,'axle',bias=bias).mean()) for y in ys]);b=np.argmin(rr);gyro.append(dict(test='bias_dps',value=bias,yi_best=ys[b],rms_best=rr[b],rms_at0=np.sqrt(scores(0,'axle',bias=bias).mean())))
    writecsv('gyro_sensitivity.csv',gyro);print('gyro best',min(gyro[:21],key=lambda r:r['rms_best']),min(gyro[21:],key=lambda r:r['rms_best']),flush=True)

if __name__=='__main__':
    main()
