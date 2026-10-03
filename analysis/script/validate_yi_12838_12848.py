"""Targeted independent numerical and input-contract checks; no firmware tests."""
import json,numpy as np
import analyze_yi_12838_12848 as m
m.load_logs()
for d in m.D.values():d['axle']=m.axleanchors(d)
assert not set(m.COLS)&{'x','y','ROC','encTotalOptimal','linePointX_mm','linePointY_mm'}
expected=[5590,5590,5590,5591,5591,5614,5613,5613,5614,5614]
for n,rows in zip(m.CW+m.CCW,expected):
 d=m.D[n];assert len(d['a'])==rows==int(d['meta']['logExpectedRows']);assert np.all(np.isfinite(d['a']));assert len(d['line'])==len(d['marker'])==len(d['axle'])==6;assert np.all(np.diff(d['s'])>0);assert np.all(np.diff(d['a'][:,0])>0);assert np.max(np.diff(d['a'][:,0]))<=6
 assert d['axle'][0]>d['s'][0] and d['axle'][-1]<d['s'][-1]
# Scalar reintegration of a whole representative log, independent of vector construction.
d=m.D[12838];a=d['a'];x=y=theta=0;scalar=[[0,0]]
for i in range(1,len(a)):
 dt=(a[i,0]-a[i-1,0])/1000;dthed=a[i,3]*np.pi/180*dt;ds=(.99675455*(a[i,1]-a[i-1,1])+1.00324545*(a[i,2]-a[i-1,2]))/116.038;mid=theta+dthed/2;lat=63*dthed;x+=ds*np.sin(mid)+lat*np.cos(mid);y+=ds*np.cos(mid)-lat*np.sin(mid);theta+=dthed;scalar.append([x,y])
p,th=m.trajectory(d,-63);scalar_error=np.max(np.abs(p-np.array(scalar)));assert scalar_error<1e-8
# Exact rigid recovery with a known rotation; no reflection and no scale fit.
t=np.linspace(0,2*np.pi,200);a=np.column_stack([np.cos(t)*20,np.sin(t)*8]);angle=.7;r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);b=a@r+[13,-8];mse,_,_=m.align(a,b);assert mse<1e-20
# Continuous lever-arm identity compared with midpoint integration error.
errors=[];bounds=[]
for d in m.D.values():
 p0,th=m.trajectory(d,0);p63,_=m.trajectory(d,-63);analytic=p0+63*np.column_stack([np.sin(th)-np.sin(th[0]),np.cos(th)-np.cos(th[0])]);errors.append(np.max(np.linalg.norm(p63-analytic,axis=1)));delta=np.diff(th);bound=63*np.sum(np.abs(delta-2*np.sin(delta/2)));bounds.append(bound);assert errors[-1]<=bound+1e-8
# The midpoint lever term uses dtheta instead of the exact 2*sin(dtheta/2).
# Check its derived truncation bound, rather than assuming a sub-0.02 mm error.
result={'input_contract':'PASS (raw encoders, gyro, sensor values only)','health':'PASS (10 logs, exact expected rows, 6 events per anchor, monotonic time and raw-derived distance)','scalar_vs_vector_max_mm':float(scalar_error),'known_rigid_transform_mse_mm2':float(mse),'lever_arm_identity_max_midpoint_discretization_error_mm':float(max(errors)),'lever_arm_identity_derived_error_bound_max_mm':float(max(bounds)),'axle_trapezoid_comparison':{str(y):float(np.sqrt(m.scores(y,'axle',method='trapezoid').mean())) for y in [0,-63,-20.5,1]},'all_25_pairs_are_not_independent_trials':True}
(m.OUT/'validation.json').write_text(json.dumps(result,indent=2));print(result)
