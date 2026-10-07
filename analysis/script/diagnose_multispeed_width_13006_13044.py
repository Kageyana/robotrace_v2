"""複数速度幅解析の校正感度・窓長・成分除去を診断し、図を保存する。"""
from pathlib import Path
import csv,json,copy
import numpy as np
import analyze_multispeed_width_13006_13044 as m
import matplotlib.pyplot as plt
O=m.OUT

def read(name):
 with (O/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def resized_data(data,dt):
 out=[]
 for d in data:
  z={**d};t=d['t'];a=d['a'];starts=t[0]+.25+np.arange(int(np.floor((t[-1]-.15-(t[0]+.25))/dt)))*dt;b=[]
  for l in starts:
   e=l+dt;j=np.searchsorted(t,[l,e]);lo=max(0,j[0]-1);hi=min(len(t),j[1]+1)
   if np.diff(t[lo:hi]).max()>.0100001:continue
   v=(np.interp(e,t,d['s'])-np.interp(l,t,d['s']))/dt/1000;om=(np.interp(e,t,d['h'])-np.interp(l,t,d['h']))/dt
   b.append(dict(log=d['n'],group=d['group'],split=d['split'],mode=d['mode'],start_s=float(l),end_s=float(e),dt_s=dt,speed_mps=float(v),omega_rads=float(om),omega_dps=float(om/m.RAD),curvature_radpm=float(om/v),
     raw_diff_mmps=float((np.interp(e,t,d['w'])-np.interp(l,t,d['w']))/dt),slip_any=int(np.any(a['slipFlag'][lo:hi]>.5) or np.any(a['slipFlagLat'][lo:hi]>.5))))
  z['base']=b;out.append(z)
 return out

if __name__=='__main__':
 frozen=json.loads((O/'frozen_parameters.json').read_text());original=m.sha(O/'frozen_parameters.json');response=frozen['response'];asym=frozen['asymmetry'];models={(f['mode'],f['model']):f for f in frozen['width_models']}
 train,_=m.load_all('train');test,_=m.load_all('test');data=train+test
 tr=m.response_rows(train,delay_ms=response['delay_ms'],tc_ms=response['tc_ms']);rows=tr+m.response_rows(test,delay_ms=response['delay_ms'],tc_ms=response['tc_ms'])
 jack=[]
 for group in m.TRAIN_GROUPS:
  rr=[r for r in tr if r['group']!=group];cal=m.separate_asymmetry(rr,m.fit_instrument(rr),write_history=False)
  jack.append(dict(excluded_group=group,**{k:v for k,v in cal.items() if k!='B_steady_mode'}))
 m.write('asymmetry_group_jackknife.csv',jack)
 gates=[]
 for maxomega in [5,10,20]:
  for maxalpha in [100,300]:
   rr=[{**r,'straight':int(abs(r['omega_dps'])<maxomega and abs(r['alpha_dps2'])<maxalpha and not r['slip_any'])} for r in tr]
   if sum(r['straight'] for r in rr)<30:continue
   cal=m.separate_asymmetry(rr,m.fit_instrument(rr),write_history=False)
   gates.append(dict(straight_omega_max_dps=maxomega,straight_alpha_max_dps2=maxalpha,**{k:v for k,v in cal.items() if k!='B_steady_mode'}))
 m.write('asymmetry_gate_sensitivity.csv',gates)
 # 共通校正を固定したまま各モードの時間応答プロファイルを確認する。
 prof=[]
 for delay,tc in m.RESPONSE_GRID:
  rr=m.response_rows(train,delay_ms=delay,tc_ms=tc)
  for mode in m.MODES:
   z=[r for r in rr if r['mode']==mode];groups=sorted({r['group'] for r in z});err=np.zeros(len(z))
   for group in groups:
    learn=[r for r in z if r['group']!=group];ix=np.array([r['group']==group for r in z]);val=[r for r in z if r['group']==group]
    f=m.fit_width(learn,model='constant',ridge=0,asym=asym);b,e=m.width_predict(val,f,asym);err[ix]=e
   prof.append(dict(mode=mode,delay_ms=delay,tc_ms=tc,conditional_series_LOO_rms_dps=m.balanced_rms(z,err)))
 m.write('response_mode_profiles.csv',prof)
 # 固定係数で構成要素だけをOFFにする。testを用いた再推定は行わない。
 ablation=[]
 for mode in m.MODES:
  rr=[r for r in rows if r['mode']==mode];f=models[mode,'constant'];b,e=m.width_predict(rr,f,asym)
  raw=m.col(rr,'raw_diff_mmps');rd=m.col(rr,'response_diff_mmps');rv=m.col(rr,'speed_mps');fv=m.col(rr,'response_speed_mps');om=m.col(rr,'omega_rads')
  cv=asym['c_velocity_mm_per_m'];c0=asym['c0_mmps'];variants=dict(separated_fixed=e,
   no_response_same_coeff=((raw-cv*rv-c0)/b-om)/m.RAD,no_asymmetry_same_B=(rd/b-om)/m.RAD,
   no_constant_offset_same_coeff=((rd-cv*fv)/b-om)/m.RAD,old_width_response_asymmetry=((rd-cv*fv-c0)/m.OLD_B-om)/m.RAD,
   old_width_raw_no_asymmetry=(raw/m.OLD_B-om)/m.RAD)
  for name,err in variants.items():
   for n in sorted({r['log'] for r in rr}):
    mask=np.array([r['log']==n for r in rr]);r0=next(r for r in rr if r['log']==n)
    ablation.append(dict(mode=mode,log=n,group=r0['group'],split=r0['split'],variant=name,rms_dps=float(np.sqrt(np.mean(err[mask]**2))),covered_angle_error_deg=float(np.sum(err[mask]*m.col(rr,'dt_s')[mask]))))
 m.write('component_ablation.csv',ablation)
 # 40/200ms窓では時間応答と左右差を固定し、幅係数をtrainで再推定する。
 sensitivity=[];sensitivity_models=[]
 for ms in [40,200]:
  resized=resized_data(data,ms/1000);rr=m.response_rows(resized,delay_ms=response['delay_ms'],tc_ms=response['tc_ms']);fits={}
  for key,f in models.items():
   learn=[r for r in rr if r['mode']==key[0] and r['split']=='train'];ff=m.fit_width(learn,model=key[1],ridge=f['ridge'],asym=asym);ff['mode']=key[0];fits[key]=ff;sensitivity_models.append(dict(window_ms=ms,**ff))
  met,_=m.evaluate(rr,fits,asym,label=f'{ms}ms');sensitivity.extend([dict(window_ms=ms,**z) for z in met])
 m.write('window_sensitivity.csv',sensitivity);(O/'window_sensitivity_models.json').write_text(json.dumps(sensitivity_models,indent=2),encoding='utf-8')
 # 固定モデルとの差をログ単位で保存。外れログも主評価に残す。
 metrics=read('per_log_metrics.csv');robust=[]
 for mode in m.MODES:
  rr=[r for r in rows if r['mode']==mode]
  for model in m.WIDTH_MODELS:
   b,e=m.width_predict(rr,models[mode,model],asym)
   for n in sorted({r['log'] for r in rr}):
    mask=np.array([r['log']==n for r in rr]);r0=next(r for r in rr if r['log']==n);q=np.percentile(abs(e[mask]),[50,90,95,100])
    robust.append(dict(log=n,group=r0['group'],split=r0['split'],mode=mode,model=model,abs_error_p50_dps=q[0],abs_error_p90_dps=q[1],abs_error_p95_dps=q[2],abs_error_max_dps=q[3]))
 m.write('residual_quantiles.csv',robust)
 # 速度×曲率のカバレッジ。
 fig,axs=plt.subplots(1,3,figsize=(14,4.4),layout='constrained')
 for ax,mode in zip(axs,['PRIMARY','DISTANCE','SLIP']):
  for split,c,mark in [('train','#263746','o'),('test','#C58A12','x')]:
   z=[r for r in rows if r['mode']==mode and r['split']==split];ax.scatter(m.col(z,'speed_mps'),abs(m.col(z,'curvature_radpm')),s=11,alpha=.35,c=c,marker=mark,label=split)
  ax.set_title(mode);ax.set_xlabel('Raw mean encoder speed [m/s]');ax.set_ylabel('Absolute IMU curvature [rad/m]');ax.set_xlim(.5,3.3);ax.set_ylim(0,15);ax.grid(alpha=.2);ax.legend()
 fig.savefig(O/'speed_curvature_coverage.png',dpi=170);plt.close(fig)
 # 応答パラメータ候補の全範囲を表示。
 search=read('response_train_search.csv');grid=np.zeros((7,7))
 for z in search:grid[int(float(z['tc_ms']))//2,int(float(z['delay_ms']))//2]=float(z['series_LOO_rms_dps'])
 fig,ax=plt.subplots(figsize=(7.5,5),layout='constrained');im=ax.imshow(grid,origin='lower',extent=(-1,13,-1,13),aspect='auto',cmap='viridis_r')
 ax.set_xticks(range(0,13,2));ax.set_yticks(range(0,13,2));ax.set_xlabel('Encoder pure delay [ms]');ax.set_ylabel('Encoder first-order time constant [ms]')
 ax.plot(response['delay_ms'],response['tc_ms'],'rx',ms=13,mew=2);fig.colorbar(im,ax=ax,label='Train series-LOO angular RMS [deg/s]')
 fig.savefig(O/'response_selection.png',dpi=170);plt.close(fig)
 # モード内で全モデル・全ログを比較し、testの大きな残差も表示。
 fig,axs=plt.subplots(2,3,figsize=(16,8.5),layout='constrained')
 colors=['#263746','#C58A12','#566E40','#CB617C'];labels=['Fixed','Speed','Curvature','Speed + curvature']
 for ax,mode in zip(axs.flat,m.MODES):
  nums=sorted({int(r['log']) for r in metrics if r['mode']==mode});ntrain=sum(1 for n in nums if next(r for r in metrics if int(r['log'])==n)['split']=='train');limit=max(float(r['angular_rms_dps']) for r in metrics if r['mode']==mode)*1.1
  for model,c,label in zip(m.WIDTH_MODELS,colors,labels):
   yy=[float(next(r['angular_rms_dps'] for r in metrics if int(r['log'])==n and r['model']==model)) for n in nums]
   assert max(yy)<limit;ax.plot(range(len(nums)),yy,color=c,label=label,marker='o',ms=3,lw=1.3)
  ax.axvline(ntrain-.5,color='#888',ls=':');ax.set_ylim(0,limit);ax.set_xticks(range(len(nums)),nums,rotation=40);ax.set_title(mode+(' requested; actual PATH' if mode=='SHORTCUT' else ''));ax.set_ylabel('Encoder minus saved IMU RMS [deg/s]');ax.grid(alpha=.2)
 axs.flat[-1].axis('off');fig.legend(*axs.flat[0].get_legend_handles_labels(),loc='outside upper center',ncol=4)
 fig.savefig(O/'mode_validation.png',dpi=170);plt.close(fig)
 # 外れた検証ログの原信号を確認できる図。
 fig,axs=plt.subplots(2,2,figsize=(13,7),layout='constrained')
 for j,n in enumerate([13031,13033]):
  rr=[r for r in rows if r['log']==n];mode=rr[0]['mode'];t=m.col(rr,'start_s')+.05;b,e=m.width_predict(rr,models[mode,'constant'],asym)
  axs[0,j].plot(t,m.col(rr,'omega_dps'),color='#263746',label='Saved IMU window yaw rate');axs[0,j].plot(t,m.col(rr,'omega_dps')+e,color='#C58A12',lw=1,label='Separated fixed-width encoder')
  axs[0,j].set_title(f'{n} {mode}');axs[0,j].set_ylabel('Angular rate [deg/s]');axs[0,j].legend(fontsize=8)
  axs[1,j].plot(t,m.col(rr,'speed_mps'),color='#607C93',label='Raw mean encoder speed');axs[1,j].fill_between(t,0,m.col(rr,'slip_any')*.5,color='#CB617C',alpha=.25,label='Any slip flag (0 / 0.5)')
  axs[1,j].set_ylabel('Speed [m/s]');axs[1,j].legend(fontsize=8)
 for ax in axs.flat:ax.grid(alpha=.2);ax.set_xlabel('Time [s]')
 fig.savefig(O/'large_residual_logs.png',dpi=170);plt.close(fig)
 assert m.sha(O/'frozen_parameters.json')==original and all(m.sha(m.LOG/f'{d["n"]}.csv')==d['sha256'] for d in data)
 (O/'diagnostic_validation.json').write_text(json.dumps(dict(status='passed',frozen_parameters_unchanged=True,source_hashes_unchanged=True,diagnostics_do_not_retune_test=True,
  jackknife_groups=len(jack),gate_variants=len(gates),window_sizes_ms=[40,100,200],script_sha256=m.sha(Path(__file__))),indent=2),encoding='utf-8')
 print('asymmetry jackknife',jack);print('asymmetry gate sensitivity',gates)
 print('best response per mode',[min([z for z in prof if z['mode']==mode],key=lambda z:z['conditional_series_LOO_rms_dps']) for mode in m.MODES])
