"""Static research figures and residual diagnostics for the independent yI analysis."""
from pathlib import Path
import csv,json,numpy as np
import analyze_yi_12838_12848 as m
m.load_logs()
for d in m.D.values():d['axle']=m.axleanchors(d)
plt=m.plt;OUT=m.OUT
BLUE='#245b87';ORANGE='#be681e';GREY='#5c6269'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'axes.axisbelow':True,'lines.linewidth':1.6})
def read(name):
 with (OUT/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def save(fig,name):fig.savefig(OUT/(name+'.png'),dpi=170,bbox_inches='tight');fig.savefig(OUT/(name+'.svg'),bbox_inches='tight');plt.close(fig)
curves=read('rms_vs_yi.csv')+read('extended_curves.csv');fig,axs=plt.subplots(1,3,figsize=(15.5,4.7),layout='constrained')
for ax,kind,title,lim in zip(axs,['marker','line','axle'],['Confirmed marker (delayed)','Line full-width midpoint','Line midpoint + sensor geometry'],[(-40,0),(-120,20),(-80,15)]):
 by_y={float(r['yi']):r for r in curves if r['anchor']==kind and float(r['trim'])==0};rows=[by_y[y] for y in sorted(by_y)];xs=np.array([float(r['yi']) for r in rows]);yy=np.array([float(r['rms']) for r in rows]);ax.plot(xs,yy,color=BLUE);b=np.argmin(yy);ax.scatter(xs[b],yy[b],c=BLUE);ax.annotate(f'{xs[b]:g} mm / {yy[b]:.2f} mm',(xs[b],yy[b]),xytext=(0,14),textcoords='offset points',ha='center',fontsize=10);ax.axvline(0,color=GREY,ls=':');ax.axvline(-20.5,color=ORANGE,ls='--');ax.set(xlim=lim,title=title,xlabel='yI [mm]',ylabel='CW/CCW RMS [mm]')
fig.suptitle('Anchor choice moves the fitted yI (25 paired lap segments, 1001 points each)',fontsize=14);save(fig,'rms_vs_yi')
fig,axs=plt.subplots(2,3,figsize=(15,9.4),layout='constrained');residualrows=[]
for row,kind in enumerate(['marker','axle']):
 for col,yi in enumerate([0,-63,-20.5]):
  a=m.laps(12838,kind,yi)[2];b=m.laps(12844,kind,yi)[2][::-1];rms,b,res=m.align(a,b);ax=axs[row,col];ax.plot(a[:,0],a[:,1],color=BLUE,label='CW 12838');ax.plot(b[:,0],b[:,1],color=ORANGE,ls='--',label='CCW 12844 reversed');ax.scatter(a[0,0],a[0,1],marker='o',s=25,c=GREY);ax.set_aspect('equal');ax.set_xlim(-200,900);ax.set_ylim(-1200,700);ax.set(title=f'{kind} anchor | yI={yi:g} mm | RMS={np.sqrt(rms):.2f} mm',xlabel='Aligned X [mm]',ylabel='Aligned Y [mm]');ax.title.set_fontsize(10)
fig.legend(*axs[0,0].get_legend_handles_labels(),loc='outside lower center',ncol=2)
fig.suptitle('Representative Lap 3: encoder + gyro reintegration; rigid rotation/translation only',fontsize=13);save(fig,'trajectory_comparison')
fig,axs=plt.subplots(1,3,figsize=(15,4.8),layout='constrained')
for ax,yi in zip(axs,[0,-63,-20.5]):
 for logs,color,label in [(m.CW,BLUE,'CW 12838-12842'),(m.CCW,ORANGE,'CCW 12844-12848')]:
  for idx,n in enumerate(logs):
   p,_=m.trajectory(m.D[n],yi);ax.plot(p[:,0],p[:,1],color=color,label=label if idx==0 else None,alpha=.4,lw=1)
 ax.set_xlim(-900,900);ax.set_ylim(-1200,800)
 ax.set_aspect('equal');ax.set(title=f'yI={yi:g} mm',xlabel='Start-relative X [mm]',ylabel='Start-relative Y [mm]');ax.legend(fontsize=9)
fig.suptitle('10 logs: full recorded trajectories (no alignment; separate start origins)',fontsize=13);save(fig,'full_trajectories')
fig,axs=plt.subplots(2,1,figsize=(12,8),layout='constrained')
classes=[]
for kind,ax in zip(['marker','axle'],axs):
 for yi,color,style in [(0,BLUE,'-'),(-20.5,ORANGE,'--'),(3.5,GREY,':')]:
  errors=[];omegas=[];signed=[]
  for c,w in m.PAIRS:
   la=m.laps(c,kind,yi);lb=m.laps(w,kind,yi);bounds=m.D[c][kind]
   for k,(a,b) in enumerate(zip(la,lb)):
    mse,_,res=m.align(a,b[::-1]);errors.append(np.sum(res**2,axis=1));s=bounds[k]+(bounds[k+1]-bounds[k])*m.Q;om=np.interp(s,m.D[c]['s'],m.D[c]['a'][:,3]);omegas.append(om)
    dp=np.gradient(a,axis=0);t=dp/np.linalg.norm(dp,axis=1)[:,None];signed.append(np.column_stack([np.sum(res*t,axis=1),res[:,0]*t[:,1]-res[:,1]*t[:,0]]))
  errors=np.array(errors);om=np.array(omegas);rr=np.sqrt(errors.mean(0));ax.plot(m.Q*100,rr,color=color,ls=style,label=f'yI={yi:g} mm')
  for j,(q,r) in enumerate(zip(m.Q,rr)):residualrows.append(dict(anchor=kind,yi=yi,progress=q,rms_mm=r))
  for label,mask in [('straight_absomega_lt30',abs(om)<30),('transition_30to150',(abs(om)>=30)&(abs(om)<150)),('turn_absomega_ge150',abs(om)>=150),('cross_end_100mm',(m.Q<100/4735)|(m.Q>1-100/4735))]:
   if mask.ndim==1:mask=np.broadcast_to(mask,errors.shape)
   classes.append(dict(anchor=kind,yi=yi,region=label,points=int(mask.sum()),rms_mm=np.sqrt(errors[mask].mean())))
  signed=np.array(signed);classes.append(dict(anchor=kind,yi=yi,region='tangential_component',points=signed.shape[0]*signed.shape[1],rms_mm=np.sqrt(np.mean(signed[:,:,0]**2))))
  classes.append(dict(anchor=kind,yi=yi,region='normal_component',points=signed.shape[0]*signed.shape[1],rms_mm=np.sqrt(np.mean(signed[:,:,1]**2))))
 ax.set(title=f'{kind} anchor: pointwise RMS across 25 lap pairs',xlabel='CW normalized lap distance [%]',ylabel='Position residual [mm]');ax.legend();ax.set_ylim(bottom=0)
save(fig,'residual_distribution');m.writecsv('residual_distribution.csv',residualrows);m.writecsv('residual_regions.csv',classes)
# Residual map for a geometric anchor, representative Lap 3.
fig,axs=plt.subplots(1,2,figsize=(11,5.4),layout='constrained')
for ax,yi in zip(axs,[0,3.5]):
 a=m.laps(12838,'axle',yi)[2];b=m.laps(12844,'axle',yi)[2][::-1];mse,_,res=m.align(a,b);err=np.linalg.norm(res,axis=1);sc=ax.scatter(a[:,0],a[:,1],c=err,s=8,vmin=0,vmax=25,cmap='cividis');ax.set_aspect('equal');ax.set(title=f'yI={yi:g} mm | RMS={np.sqrt(mse):.2f} mm',xlabel='X [mm]',ylabel='Y [mm]');fig.colorbar(sc,ax=ax,label='Residual [mm]')
fig.suptitle('Residual location: axle-corrected anchor, 12838/12844 Lap 3');save(fig,'residual_map')
# Full-width event profiles show the enter-exit midpoint, without courseMarker gating.
fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained')
for row,n in enumerate([12838,12844]):
 d=m.D[n];line=d['line'][0];s=d['s']-line;sel=abs(s)<65;ax=axs[row,0]
 for ch in range(10):ax.plot(s[sel],d['a'][sel,7+ch],alpha=.55,lw=1,label=str(ch))
 ax.axvline(0,color=GREY,ls=':');ax.axvline(d['marker'][0]-line,color=ORANGE,ls='--',label='Marker confirmed');ax.set(title=f'{n}: 10 channel values',xlabel='Distance from full-width midpoint [mm]',ylabel='Calibrated sensor value');ax.legend(ncol=6,fontsize=8,loc='lower center',bbox_to_anchor=(.5,1.01));ax.set_title(f'{n}: 10 channel values',pad=42)
 ax=axs[row,1];val=np.sort(d['a'][:,7:],axis=1)[:,2];ax.plot(s[sel],val[sel],color=BLUE,label='8th-highest channel');ax.axhline(1000,color=GREY,ls=':');_,det=m.anchor(d);begin,end,mid,*_=det[0];ax.axvspan(begin-line,end-line,color=BLUE,alpha=.12,label='Full-width interval');ax.axvline(0,color=GREY,ls='--');ax.set(title=f'{n}: independently selected full-width interval',xlabel='Distance from midpoint [mm]',ylabel='Order statistic');ax.legend(fontsize=9)
save(fig,'line_cross_profiles')
# CAD top projection. Plot explicit topological vertices and footprint centers.
ag=json.loads((OUT/'axle_geometry.json').read_text());mid=ag['midpoint_Z_mm'];half=ag['wheelbase_mm']/2
pts=json.loads((OUT/'step_points.json').read_text());fig,ax=plt.subplots(figsize=(9,7),layout='constrained')
for key,p in pts.items():
 if any(name in key for name in ['tire_miniZ','robotrace_v2_Sidemarker','Linesensor_PCB.step']):
  p=np.array(p);ax.scatter(p[:,0],p[:,2]-mid,s=2,color='#9fa6ac',alpha=.35)
with (OUT/'sensor_centers.csv').open() as f:sr=list(csv.DictReader(f))
for r in sr:
 x=float(r['X_mm']);f=float(r['forward_mm']);side=r['board'].startswith('side');ax.scatter(x,f,color=ORANGE if side else BLUE,s=42 if side else 18)
 if side:ax.annotate(f"{r['ref']}  +{f:.2f} mm",(x,f),xytext=(4,7),textcoords='offset points',fontsize=9)
for z,name in [(-half,'Rear axle'),(0,'Axle midpoint'),(half,'Front axle')]:ax.axhline(z,color=GREY,ls='--' if z else '-');ax.text(-62,z+1,name,fontsize=9)
ax.set_aspect('equal');ax.set(xlim=(-90,90),ylim=(-30,105),xlabel='STEP X [mm] (lateral)',ylabel='Forward from axle midpoint [mm]',title='STEP assembly + KiCad footprint centers (top projection)');save(fig,'sensor_geometry')
phase=read('phase_sensitivity.csv');fig,ax=plt.subplots(figsize=(8.5,5),layout='constrained');ax.plot([float(r['anchor_shift_mm']) for r in phase],[float(r['yi_best']) for r in phase],color=BLUE,marker='o');ax.set(xlabel='Added forward shift of confirmed-marker anchor [mm]',ylabel='Fitted yI [mm]',title='Changing anchor phase changes fitted yI almost one-for-one');save(fig,'phase_confounding')
# Additional scale/bias sensitivities, profile over yI in [-5,10].
g=read('gyro_sensitivity.csv');fig,axs=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
for ax,test,label,factor in zip(axs,['scale','bias_dps'],['Additional gyro scale [%]','Additional gyro bias [deg/s]'],[100,1]):
 rows=[r for r in g if r['test']==test];ax.plot([float(r['value'])*factor for r in rows],[float(r['rms_best']) for r in rows],color=BLUE);ax.set(xlabel=label,ylabel='Profiled RMS [mm]',title='Axle geometry anchor: optimize yI for each value')
save(fig,'gyro_sensitivity')
# Useful validation and all cross combinations, with fixed geometry.
extra=[]
for kind in ['marker','axle']:
 ys=np.arange(-25,-15.01,.5) if kind=='marker' else np.arange(-2,8.01,.5)
 allpairs=[(c,w) for c in m.CW for w in m.CCW];rr=np.array([np.sqrt(m.scores(y,kind,pairs=allpairs).mean()) for y in ys]);b=np.argmin(rr);extra.append(dict(anchor=kind,pairs=25,segments=125,yi_best=ys[b],rms_best=rr[b]))
extra.append(dict(anchor='axle_trapezoid_yi0',pairs=5,segments=25,yi_best=0,rms_best=np.sqrt(m.scores(0,'axle',method='trapezoid').mean())))
m.writecsv('extra_checks.csv',extra)
# Per-lap closure and angle: these do not constrain closure to zero.
closure=[]
for n,d in m.D.items():
 p,th=m.trajectory(d);bounds=d['axle']
 for k,(b,e) in enumerate(zip(bounds[:-1],bounds[1:])):
  coords=np.column_stack([np.interp([b,e],d['s'],p[:,j]) for j in range(2)]);ang=np.interp([b,e],d['s'],th)
  closure.append(dict(log=n,lap=k+1,length_mm=e-b,heading_change_deg=np.rad2deg(ang[1]-ang[0]),closure_mm=np.linalg.norm(coords[1]-coords[0])))
m.writecsv('lap_closure.csv',closure)
print('EXTRA CHECKS',extra)
print('REGIONS',classes)
print('FIGURES COMPLETE')
