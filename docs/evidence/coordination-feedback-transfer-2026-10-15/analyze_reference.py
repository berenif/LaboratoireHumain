"""Read archived states only: loaded convex hull, CoM velocity and joint motion."""
import json,math
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
model=json.loads((ROOT/'rust/crates/model/data/canonical-v1.json').read_text())['segments']; ids={s['id']:i for i,s in enumerate(model)}
def quat(d):return [d[k] for k in 'xyzw']
def mul(a,b):
 x,y,z,w=a;X,Y,Z,W=b
 return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]
def angles(poses,i):
 j=model[i]['jointProfile'];a=mul(quat(poses[ids[model[i]['parent']]]['rotation']),quat(j['parentFrame']['rotation']));b=mul(quat(poses[i]['rotation']),quat(j['childFrame']['rotation']));r=mul([-a[0],-a[1],-a[2],a[3]],b)
 if r[3]<0:r=[-v for v in r]
 return [(2*math.atan2(v,r[3])+math.pi)%(2*math.pi)-math.pi for v in r[:3]]
def hull(points):
 p=sorted(set(tuple(x) for x in points));cross=lambda a,b,c:(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);lower=[];upper=[]
 for part,seq in ((lower,p),(upper,reversed(p))):
  for v in seq:
   while len(part)>=2 and cross(part[-2],part[-1],v)<=0:part.pop()
   part.append(v)
 return lower[:-1]+upper[:-1] if len(p)>1 else p
rows=[]
for line in (ROOT/'docs/evidence/coordination-exploration-execution-2026-10-15/native-trace.jsonl').open():
 r=json.loads(line);step=r.get('nativeSubstep');snap=r.get('snapshot')
 if step not in (36,40,44,80,120,168,240,300,360,384) or not snap or r.get('kind') not in ('actual-history','exploration-step'):continue
 poses=snap['segments'];support=r.get('support',{});contacts=r.get('segmentContacts',support.get('segments',[]));com=r.get('com',support.get('center_of_mass_m'));mass=sum(s['massKg'] for s in model)
 velocity=[sum(s['massKg']*pose['linearVelocity'][axis] for s,pose in zip(model,poses))/mass for axis in 'xyz'];cp=[com[i]+velocity[i]*math.sqrt(max(.2,com[1])/9.81) for i in range(3)]
 poly=hull([[c['world_point_m'][0],c['world_point_m'][2]] for s in contacts if s['role'] in ('Hindfoot','Forefoot') for c in s['contacts'] if c['current_upward_load_n']>=3])
 cmd=r.get('commands',r.get('owner',{}).get('last'))
 rows.append({'nativeSubstep':step,'comM':com,'comVelocityMpsFromArchivedBodyVelocities':velocity,'capturePointM':cp,'loadedFootHullXZ':poly,'pelvisVelocityMps':poses[0]['linearVelocity'],'torsoAngularVelocityRadps':poses[2]['angularVelocity'],'joints':[{'id':model[i]['id'],'measuredCoordinatesRad':angles(poses,i),'commandRad':cmd['positions'][i]} for i in (1,2,15,16,17,18,20,21,22,23)]})
out={'scope':'Archived measurements, no native steps/FK/world; f64 quaternion coordinate reconstruction and mass-weighted archived native velocities','rows':rows,'hypothesis':'Forward CoM velocity and trunk rotation grow while the hip reference flexes further, knees stay almost fixed and measured ankles are near their 20-degree upper limit. Capture exit precedes retained-point loss. Missing state-dependent braking/recentering is supported as a candidate explanation, not established as the sole dynamic cause.','additionalNativeSteps':0}
(HERE/'causal-baseline.json').write_text(json.dumps(out,indent=2)+'\n')
for r in rows[:5]: print(r['nativeSubstep'],r['comM'],r['comVelocityMpsFromArchivedBodyVelocities'],r['capturePointM'])
