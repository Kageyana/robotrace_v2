import csv,json,sys,os,tempfile
from pathlib import Path
sys.path.insert(0,str(Path('analysis/script').resolve()))
from analyze_path_following import percentile
from path_log_recovery import read_csv_log
os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'robotrace-matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path('analysis/12363_12367')
out.mkdir(parents=True,exist_ok=True)
fig,axs=plt.subplots(5,1,figsize=(11,15),constrained_layout=True)
xy,axxy=plt.subplots(figsize=(7,7),layout='constrained')
result=[]
for n in range(12363,12368):
 log=read_csv_log(Path('F:/Dropbox/Document/robotrace/Log/v2')/f'{n}.csv')
 rows=[{name:float(value) for name,value in row.items()} for row in log.rows]
 raw=log.rows; cols=list(enumerate(log.fields))
 t=[r['cntlog']/1000 for r in rows]; dt=[b['cntlog']-a['cntlog'] for a,b in zip(rows,rows[1:])]
 result.append(dict(log=n,rows=len(rows),malformed_rows=sum(len(r)<len(cols) for r in raw),time_s=t[-1],distance_mm=rows[-1]['encTotalOptimal']/54.324,dt_min=min(dt),dt_max=max(dt),speed_error_p95_mps=percentile([abs(r['encCurrentN']-r['targetSpeed'])/53.424 for r in rows],.95),slip_long_samples=sum(r['slipFlag']!=0 for r in rows),slip_lat_samples=sum(r['slipFlagLat']!=0 for r in rows),index_last=rows[-1]['optimalIndex'],battery_min_V=min(r['batteryVoltage_mV'] for r in rows)/1000))
 axxy.plot([r['x'] for r in rows],[r['y'] for r in rows],label=str(n))
 if n==12363: continue
 for ax,key,scale in zip(axs,['encCurrentN','targetAngularvelo','pathErrorY_mm','pathErrorHeading_cdeg','slipFlagLat'],[1/53.424,1,1,.01,1]):
  ax.plot(t,[r[key]*scale for r in rows],label=str(n),color=f'C{n-12364}')
 axs[1].plot(t,[r['gyroVal_Z'] for r in rows],alpha=.5,lw=.8,ls='--',color=f'C{n-12364}')
 axs[4].plot(t,[r['slipFlag'] for r in rows],ls='--',alpha=.5,color=f'C{n-12364}')
axs[0].axhline(54/53.424,color='black',ls='--',label='target')
for ax,label in zip(axs,['Speed [m/s]','Yaw [deg/s]\nsolid: target; dashed: gyro','Lateral error [mm]','Heading error [deg]','Slip\nsolid: lateral; dashed: long.']):
 ax.set_ylabel(label); ax.set_xlabel('Time [s]'); ax.grid(True); ax.legend(fontsize=8,ncol=5)
fig.suptitle('12364-12367: Level 0 replay')
fig.savefig(out/'log_12364_12367_tracking.png',dpi=140)
axxy.set(xlabel='Derived x [mm]',ylabel='Derived y [mm]',title='12363 primary / 12364-12367 replay: derived XY')
axxy.axis('equal'); axxy.grid(); axxy.legend(); xy.savefig(out/'log_12363_12367_xy.png',dpi=140)
(out/'log_12363_12367_checks.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
for r in result: print(json.dumps(r))
