"""Inspect STEP AP214 assembly transforms and per-solid geometric points, without CAD runtime."""
import re,json,csv,pathlib,numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2]
OUT=ROOT/'analysis/yi_12838_12848';OUT.mkdir(exist_ok=True)
s=(ROOT/'Machine/robotrace_v3.STEP').read_text()
e={int(i):v.strip() for i,v in re.findall(r'#(\d+)\s*=\s*(.*?);',s,re.S)}
def refs(i):return list(map(int,re.findall(r'#(\d+)',e[i])))
def vector(i):return np.array(list(map(float,re.findall(r'\(\s*([^()]+)\s*\)',e[i])[-1].split(','))))
def axis(i):
 p,z,x=refs(i);z=vector(z);x=vector(x);a=np.eye(4);a[:3,:3]=np.column_stack([x,np.cross(z,x),z]);a[:3,3]=vector(p);return a
names={};reps={};geometry={};children={}
for i,v in e.items():
 if v.startswith('PRODUCT_DEFINITION ('):names[i]=re.findall("'([^']*)'",e[refs(refs(i)[0])[0]])[0]
 if v.startswith('SHAPE_DEFINITION_REPRESENTATION'):
  pds,rep=refs(i);pd=refs(pds)[-1]
  if e[pd].startswith('PRODUCT_DEFINITION ('):reps[pd]=rep
 if v.startswith('SHAPE_REPRESENTATION_RELATIONSHIP'):
  a,b=refs(i);geometry.setdefault(a,[]).append(b)
for i,v in e.items():
 if v.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
  rel,pds=refs(i);nauo=refs(pds)[-1];parent,child=refs(nauo);a,b,tr=refs(rel);ap,ac=refs(tr)
  # AP214 item-defined transformation maps child's ac to parent's ap.
  t=axis(ap)@np.linalg.inv(axis(ac));children.setdefault(parent,[]).append((child,t,nauo))
def collect(i,kind):
 seen=set();todo=[i];ret=[]
 while todo:
  j=todo.pop()
  if j in seen:continue
  seen.add(j)
  if e[j].startswith(kind):ret.append(j);continue
  todo.extend(refs(j))
 return ret
rows=[];pts={};axle_centers=[]
def walk(pd,t,path):
 name=names[pd];rep=reps.get(pd);sol=[]
 if rep:
  for gr in geometry.get(rep,[]):sol.extend(collect(gr,'MANIFOLD_SOLID_BREP'))
 for j in sol:
  if name=='axial_wheelLR':
   circle_ids=collect(j,'CIRCLE');centers=np.array([axis(refs(c)[0])[:3,3] for c in circle_ids]);centers=centers@t[:3,:3].T+t[:3,3]
   assert np.ptp(centers[:,2])<1e-6, 'Axle circle centers must share a forward coordinate'
   axle_centers.append(float(np.median(centers[:,2])))
  ids=collect(j,'VERTEX_POINT');p=np.array([vector(refs(k)[0]) for k in ids]);p=p@t[:3,:3].T+t[:3,3]
  row=dict(path=path,name=name,solid=j,npoints=len(p),xmin=p[:,0].min(),xmax=p[:,0].max(),ymin=p[:,1].min(),ymax=p[:,1].max(),zmin=p[:,2].min(),zmax=p[:,2].max())
  rows.append(row);pts[f'{path}/{j}']=p.tolist()
 for child,ct,n in children.get(pd,[]):walk(child,t@ct,f'{path}/{n}:{names[child]}')
ROOT_PD=next(pd for pd,name in names.items() if name=='robotrace_v3')
walk(ROOT_PD,np.eye(4),'root')
assert len(axle_centers)==2
AXLE_MID=float(np.mean(axle_centers))
(OUT/'axle_geometry.json').write_text(json.dumps({'axis_forward_Z_mm':sorted(axle_centers),'midpoint_Z_mm':AXLE_MID,'wheelbase_mm':abs(axle_centers[1]-axle_centers[0])},indent=2))
with (OUT/'step_solids.csv').open('w',newline='',encoding='utf-8') as f:w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
# Bounds use topological vertices, not free surface origins or spline control points.
(OUT/'step_points.json').write_text(json.dumps(pts))
for r in rows:
 if r['name'] in ['robotrace_v2_Sidemarker','tire_miniZ','axial_wheelLR','wheel_gear','robotrace_v2_Linesensor_PCB.step']:print(r)
# Export footprint centers transformed through the STEP assembly.
import re
sensor_rows=[]
for child,t,n in children[ROOT_PD]:
 if names[child]=='robotrace_v2_Sidemarker':
  for ref,x in [('U1',149.7),('U2',169.7)]:
   p=t@np.array([x,-98.25,2.495,1]);sensor_rows.append(dict(board=f'side_{n}',ref=ref,X_mm=p[0],height_mm=p[1],forward_mm=p[2]-AXLE_MID))
 if names[child]=='robotrace_v2_Linesensor':
  c2,t2,_=children[child][0]
  for c3,t3,n3 in children[c2]:
   if names[c3]=='robotrace_v2_Linesensor_PCB.step':
    line_t=t@t2@t3
pcb=(ROOT/'Circuit/robotrace_v2_Linesensor/robotrace_v2_Linesensor.kicad_pcb').read_text(encoding='utf-8')
for block in pcb.split('(footprint ')[1:]:
 ref=re.search(r'\(fp_text reference "(Q\d+)"',block)
 if ref:
  x,y,*_=map(float,re.search(r'\(at ([^()]+)\)',block).group(1).split());p=line_t@np.array([x,-y,0,1]);sensor_rows.append(dict(board='line',ref=ref.group(1),X_mm=p[0],height_mm=p[1],forward_mm=p[2]-AXLE_MID))
with (OUT/'sensor_centers.csv').open('w',newline='',encoding='utf-8') as f:w=csv.DictWriter(f,sensor_rows[0].keys());w.writeheader();w.writerows(sensor_rows)
print('sensor centers',sensor_rows)
