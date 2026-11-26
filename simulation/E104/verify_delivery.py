import json
from pathlib import Path
import numpy as np
import open3d as o
from verify_contact import check
root=Path(__file__).resolve().parents[2]
reg=root/'algorithm/scene-fusion'
sim=root/'simulation/E104'
def load(p):
 return json.loads(p.read_text())
def read_points(p):
 dt=np.dtype([('point','<f4',(3,)),('rgb','u1',(3,)),('view','u1')])
 with p.open('rb') as f:
  for line in f:
   if line==b'end_header\n':break
  return np.fromfile(f,dt)
checks=[]
for rg in sorted((reg/'E104').glob('*/*/鲁棒性.json')):
 d=load(rg); t=load(rg.with_name('位姿.json')); path=rg.parent
 b=path.parent.name
 source=reg/'E103'/b/path.name
 assert len(d)==5 and set(d)=={'clean','noise_outliers','half_views','bad_loop','marker_occlusion'}
 assert not next(row for row in d['bad_loop']['edges'] if row['source']==0)['accepted']
 assert d['marker_occlusion']['axis']['marker_stations'][0]!=0
 parts=[read_points(p) for p in sorted((source/'单视角').glob('*.ply'))]
 for method,fname in [('poses','推荐'),('R02','R02'),('R06c','R06c')]:
  q=read_points(path/f'{fname}.ply')
  assert np.bincount(q['view']).tolist()==[len(x) for x in parts]
  T=np.array(t[method])
  assert np.allclose(T[:,:3,:3]@T[:,:3,:3].transpose(0,2,1),np.eye(3),atol=1e-8)
  assert np.allclose(T[0],np.eye(4))
  l=0
  for i,x in enumerate(parts):
   step=max(1,len(x)//170)
   p=x['point'][::step]@T[i,:3,:3].T+T[i,:3,3]
   assert np.allclose(p,q['point'][l:l+len(x):step],atol=1e-4)
   l+=len(x)
 checks.append(f'{b}/{path.name}')
assert len(checks)==10
deliver=load(sim/'推荐.json')
measured=[]
count=0
for j in sorted((sim/'场景').glob('*/结果.json')):
 folder=j.parent
 r=load(j)
 assert set(r['tests'])=={'T0','T1','T2'}
 for trial,s in r['tests'].items():
  assert s['split']['measured_training_points']>0
  assert s['split']['visible_holdout_points']>0
  for method,v in s['methods'].items():
   q=folder/f'{trial}-{method}.ply'
   if v['status']=='scored':
    with q.open('rb') as f:
     h={}
     for line in f:
      line=line.strip()
      if line==b'end_header':break
      if line.startswith(b'element '):
       _,key,n=line.split()
       h[key]=int(n)
    assert h[b'vertex']==v['vertices'] and h[b'face']==v['triangles']
   count+=1
 if folder.name in deliver and deliver[folder.name]['watertight']:
  g=deliver[folder.name]['watertight']
  f=folder/'封闭候选.ply'
  assert f.read_bytes()==(folder/f'T0-{g}.ply').read_bytes()
  m=o.io.read_triangle_mesh(str(f))
  v=r['tests']['T0']['methods'][g]['volume_mm3']
  contact, volume=check(m)
  assert np.isclose(v,volume) and np.isclose(r['tests']['T0']['methods'][g]['floor_cap_area_mm2'],contact['floor_cap_area_mm2'])
  measured.append(folder.name+'/'+g)
results=dict(rigid_fusions_verified=30,registration_groups=checks,perturbations=50,
             mesh_trials_count=count,verified_watertight_recommendations=measured)
(sim/'交付核验.json').write_text(json.dumps(results,indent=2,ensure_ascii=False)+'\n')
print(results)
