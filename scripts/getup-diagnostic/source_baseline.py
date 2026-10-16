#!/usr/bin/env python3
"""Pinned original-conditions positive control; never runs without a reserved ledger."""
import argparse,ast,ctypes as ct,datetime,hashlib,json,math,pathlib,signal,time
PROCESS_STARTED=time.monotonic()
import numpy as np
import scipy
from scipy import interpolate
from scipy.spatial import ConvexHull,QhullError
import torch
from compare_mujoco import World
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'outputs/getup-source-baseline-2026-10-16';LEDGER=ROOT/'docs/evidence/getup-source-baseline-2026-10-16.json'
SOURCE=ROOT/'outputs/getup-mujoco-comparison-2026-10-16/source-model'
CLASSIC=OUT/'dm-control-model/humanoid.xml';ORDER=np.array([1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20]);SPEED=.5;POWER=.66342
STANDING=np.array([0,-.11665,0,-.03601,-.03219,.05025,-.39803,-.30198,.13615,-.03601,-.03219,.05025,-.39803,-.30198,-.13615,.40763,-.41967,-1.5795,-.40763,.41967,-1.5795])
sha=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
def source_spline():
 p=OUT/'source/utils.py';assert sha(p)=='6aab3093ad6f4ddfcb7a8c42e7b57c20a0c9fa879f21c48dc68735453c6ec850'
 fn=next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='interpolate_motion');scope={'np':np,'math':math,'interpolate':interpolate};exec(compile(ast.Module(body=[fn],type_ignores=[]),'pinned-interpolate-motion','exec'),scope);return scope['interpolate_motion']
def load_actor(name,width):
 p=ROOT/f'outputs/getup-policy-2026-10-16/source/{name}.pt';expected={'teacher':'9935efee1b1d46e796bb241dc1240f1905ad40e0d20284909cc063575606b244','student':'8b6be27ad1395cee3ddcd953907dad151cdfe1bd75b98a6bf02bfed30623fa0d'}
 assert sha(p)==expected[name] and tuple(map(int,torch.__version__.split('+')[0].split('.')[:2]))>=(2,6)
 net=torch.nn.Sequential(torch.nn.Linear(width,1024),torch.nn.ReLU(),torch.nn.Linear(1024,1024),torch.nn.ReLU(),torch.nn.Linear(1024,42));weights=torch.load(p,map_location='cpu',weights_only=True);assert all(type(v) is torch.Tensor and v.dtype==torch.float32 and torch.isfinite(v).all() for v in weights.values());net.load_state_dict({k.removeprefix('trunk.'):v for k,v in weights.items()},strict=True);net.eval()
 def predict(x):
  x=np.asarray(x,dtype=np.float32);assert x.shape==(width,) and np.isfinite(x).all()
  with torch.inference_mode():return (torch.tanh(net(torch.from_numpy(x[None]))[0,:21])*float(x[-1])).numpy().copy()
 return predict
class SourceWorld(World):
 forwards=0
 def __init__(self,xml):
  super().__init__(xml,OUT/'mujoco_bridge.so');l=self.lib
  for name,args,ret in [('lh_support_count',[ct.c_void_p],ct.c_int),('lh_body_velocity',[ct.c_void_p,ct.c_int,ct.POINTER(ct.c_double)],None),('lh_reset_data',[ct.c_void_p],None),('lh_assign_state',[ct.c_void_p,ct.POINTER(ct.c_double),ct.POINTER(ct.c_double)],None)]:
   f=getattr(l,name);f.argtypes=args;f.restype=ret
 def install(self,q,dq,reset=True):
  q=np.asarray(q,dtype=np.float64);dq=np.asarray(dq,dtype=np.float64)
  fn=self.lib.lh_set_state if reset else self.lib.lh_assign_state;fn(self.h,q.ctypes.data_as(ct.POINTER(ct.c_double)),dq.ctypes.data_as(ct.POINTER(ct.c_double)));SourceWorld.forwards+=1
 def random_reset(self,rng,label):
  self.lib.lh_reset_data(self.h);SourceWorld.forwards+=1;base=self.arr(0,(28,)).copy();ranges=self.arr(16,(22,2)).copy()
  for k in range(1000):
   q=random_q(base,ranges,rng);self.install(q,np.zeros(27),False)
   if self.dim(7)==0:
    return {'label':label,'samples':k+1,'qpos':q.tolist(),'qvel':[0.]*27,'timeS':self.lib.lh_time(self.h),'nativeSteps':0}
  raise RuntimeError('Source collision-free reset sample bound reached')
 def extra(self):
  loads=self.arr(20,(17,)).copy();n=self.lib.lh_support_count(self.h);points=self.arr(21,(128,))[:2*n].reshape(-1,2).copy();velocities=[]
  for i in range(1,17):
   v=np.zeros(6);self.lib.lh_body_velocity(self.h,i,v.ctypes.data_as(ct.POINTER(ct.c_double)));velocities.append(v)
  return loads,points,np.array(velocities)
def random_q(base,ranges,rng):
 q=base.copy();quat=rng.rand(4);quat/=np.linalg.norm(quat);q[3:7]=quat
 for i in range(21):q[7+i]=rng.uniform(*ranges[1+i])
 return q
def controls(q,dq,target,residual,holding):return ((target+residual-q)-(.2 if holding else .1)*dq)[ORDER]
class Stable:
 def __init__(self,dt):self.dt=dt;self.steps=0;self.maximum=0.;self.first=None;self.anchors=None
 def update(self,w,row,phase_step):
  loads,points,vel=w.extra();feet=[w.ids[n] for n in ['left_foot','right_foot']];mass=w.arr(10,(17,)).sum();com=np.array(row['comSource']);cv=np.array(row['observation'][37:40]);capture=com[:2]+cv[:2]*math.sqrt(max(0.,com[2])/9.81);margin=None
  if len(points)>=3:
   try:
    hull=ConvexHull(np.unique(points,axis=0));eq=hull.equations;margin=float(min(-np.max(eq[:,:2]@com[:2]+eq[:,2]),-np.max(eq[:,:2]@capture+eq[:,2])))
   except QhullError:pass
  positions=w.arr(3,(17,3))[feet].copy();lin=float(np.linalg.norm(vel[:,3:],axis=1).max());ang=float(np.linalg.norm(vel[:,:3],axis=1).max());other=max(loads[i] for i in range(1,17) if i not in feet)
  conditions={'head':row['headHeightM']>=1.4,'upright':row['observation'][36]>=.97,'bothFeet':bool(np.all(loads[feet]>=3)),'footWeight':float(loads[feet].sum())>=.8*mass*9.81,'onlyFeet':other<3,'linearSpeed':lin<=.22,'angularSpeed':ang<=.65,'balance':margin is not None and margin>=-1e-6}
  stable=all(conditions.values());drift=None
  if stable:
   if self.anchors is None:self.anchors=positions
   drift=float(np.linalg.norm(positions-self.anchors,axis=1).max());stable=drift<=.01
  if not stable:self.steps=0;self.anchors=None
  else:self.steps+=1
  duration=self.steps*self.dt;self.maximum=max(self.maximum,duration)
  if duration>=1.-1e-9 and self.first is None:self.first=phase_step*self.dt
  return {'stableSeconds':duration,'maximumStableSeconds':self.maximum,'firstStableStandingTimeS':self.first,'conditions':conditions,'footLoadsN':loads[feet].tolist(),'maximumOtherFloorLoadN':float(other),'maximumBodyLinearMps':lin,'maximumBodyAngularRadps':ang,'footDriftM':drift,'minimumComCaptureHullMarginM':margin,'solvedFootContactPoints':points.tolist()}
def prepare(teacher,student,spline):
 w=SourceWorld(CLASSIC)
 try:
  assert abs(w.lib.lh_dt(w.h)-.005)<1e-12
  # Compare random samples with the actual inspected dm_control1.0.1 function.
  path=ROOT/'outputs/getup-causal-review-2026-10-16/dm-control-1.0.1-randomizers.py';assert sha(path)=='a6261f7c185accba5cb19daf62b9012fb297d8375c46520fa70d0005a30a4160'
  tree=ast.parse(path.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='randomize_limited_and_rotational_joints')
  from types import SimpleNamespace as NS
  scope={'np':np,'mjbindings':NS(enums=NS(mjtJoint=NS(mjJNT_HINGE=3,mjJNT_SLIDE=2,mjJNT_BALL=1,mjJNT_FREE=0)))};exec(compile(ast.Module(body=[fn],type_ignores=[]),'pinned-dm-control-randomizer','exec'),scope)
  names=['root']+[w.lib.lh_name(w.h,3,i).decode() for i in range(1,22)];ranges=w.arr(16,(22,2)).copy();base=w.arr(0,(28,)).copy()
  class Named:
   def __init__(self,q):self.q=q
   def __getitem__(self,k):return self.q[:7] if k=='root' else self.q[7+names.index(k)-1:8+names.index(k)-1]
   def __setitem__(self,k,v):self.__getitem__(k)[:]=v
  for seed in range(32):
   expected=base.copy();fake=NS(named=NS(data=NS(qpos=Named(expected))),model=NS(njnt=22,id2name=lambda i,t:names[i],jnt_type=np.r_[0,np.full(21,3)],jnt_limited=np.r_[False,np.ones(21,dtype=bool)],jnt_range=ranges))
   scope[fn.name](fake,np.random.RandomState(seed));assert np.array_equal(random_q(base,ranges,np.random.RandomState(seed)),expected)
  fixtures=json.loads((ROOT/'rust/crates/sim/data/getup-policy/parity.json').read_text());maxerror=0.
  for f in fixtures['actors']:maxerror=max(maxerror,float(np.max(np.abs((teacher if f['actor']=='teacher' else student)(f['input'])-np.array(f['action'])))))
  assert maxerror<5e-5
  for f in fixtures['pd']:
   assert np.max(np.abs(np.clip(controls(np.array(f['q']),np.array(f['dq']),np.array(f['reference']),np.array(f['residual']),f['holding']),-1,1)-np.array(f['controls'])))<1e-6
  sample={k:np.random.RandomState(i).normal(size=(5,n)) for i,(k,n) in enumerate([('qpos',28),('qvel',27),('com',3),('com_ori',3)])};result=spline(sample,.5,False);assert all(v.shape[0]==10 and np.allclose(v[0],sample[k][0]*(.5 if k=='qvel' else 1)) for k,v in result.items())
  w.install(base,np.zeros(27));assert w.lib.lh_step(w.h,0)==-1 and w.dim(6)==0
  row=w.snapshot(0,np.zeros(21));observed=Stable(.005).update(w,row,0);assert observed['stableSeconds']==0 and sum(observed['footLoadsN'])==0
  result={'passed':True,'physicsIntegrations':0,'randomizerFixtures':32,'actorFixtures':32,'pdFixtures':32,'maximumActorError':maxerror,'sourceInterpolationEndpointsAndSpeedPassed':True,'zeroBudgetStepRejected':True,'standingCannotPassBeforeSolvedLoads':True,'staticForwards':SourceWorld.forwards,'versions':{'mujoco':213,'torch':torch.__version__,'numpy':np.__version__,'scipy':scipy.__version__}}
  write(OUT/'preflight.json',result);print(json.dumps(result,indent=2))
 finally:w.close()
def run(teacher,student,spline):
 ledger=json.loads(LEDGER.read_text());assert ledger['status']=='reserved' and ledger['seed']==0 and ledger['speed']==.5
 assert ledger['nativeChargedAfter']==ledger['nativeChargedBefore']+9760<=300000 and ledger['wallChargedAfterSeconds']==ledger['wallChargedBeforeSeconds']+180<=7200
 assert sha(ROOT/ledger['priorClosedRecord'])==ledger['priorClosedRecordSha256']
 for p,h in ledger['inputs'].items():assert sha(ROOT/p)==h,p
 assert json.loads((OUT/'preflight.json').read_text())['passed'];(OUT/'started-once').open('x').close()
 signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(TimeoutError('180s baseline deadline')));signal.alarm(max(1,int(175-(time.monotonic()-PROCESS_STARTED))))
 result={'seed':0,'speed':.5,'teacherPower':POWER,'nativeSteps':0,'phaseSteps':{},'attempts':[],'studentStarted':False,'completeSlowEpisode':False,'actualStableStanding':False,'projectQualified':False,'maximumSelfPenetrationM':0.,'maximumFloorPenetrationM':0.,'startedUtc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
 stream=(OUT/'states.ndjson').open('x');decisions=(OUT/'decisions.ndjson').open('x');rng=np.random.RandomState(0)
 def advance(w,phase,index,u,observer=None,save_frame=False):
  assert result['nativeSteps']<9760 and time.monotonic()-PROCESS_STARTED<175
  result['nativeSteps']+=1;result['phaseSteps'][phase]=result['phaseSteps'].get(phase,0)+1;write(OUT/'result.json',result)
  w.arr(2,(21,))[:]=u;assert w.lib.lh_step(w.h,9760)>0
  row=w.snapshot(index,np.clip(u,-1,1));row['phase']=phase;row['phaseNativeStep']=index;row['rawControlMaximumAbs']=float(np.max(np.abs(u)))
  if observer:row['standing']=observer.update(w,row,index)
  floor=max([0.]+[-c['distanceM'] for c in row['contacts'] if 'world' in [c['body1'],c['body2']]])
  result['maximumSelfPenetrationM']=max(result['maximumSelfPenetrationM'],row['maximumSelfPenetrationM']);result['maximumFloorPenetrationM']=max(result['maximumFloorPenetrationM'],floor)
  if save_frame:
   with (OUT/'frames.ndjson').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
  else:row.pop('bodyPosesSource')
  stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();return row
 def zeros(w,phase):
  for i in range(400):advance(w,phase,i+1,np.zeros(21),save_frame=(i%20==0 or i==399))
 try:
  classic=SourceWorld(CLASSIC)
  try:
   reset=classic.random_reset(rng,'teacher-constructor-classic');write(OUT/'constructor-reset.json',reset);zeros(classic,'constructor-preparation-discarded')
  finally:classic.close()
  # Source student constructor/reset randomizations use their own independent stream;
  # they are discarded without integrating before the reference state is installed.
  srng=np.random.RandomState(0);discard=SourceWorld(CLASSIC)
  try:write(OUT/'student-constructor-reset.json',discard.random_reset(srng,'student-constructor-classic-discarded'))
  finally:discard.close()
  actual=SourceWorld(SOURCE/'humanoid_high_freq.xml');write(OUT/'student-discarded-reset.json',actual.random_reset(srng,'student-pre-reference-discarded'))
  weak=SourceWorld(SOURCE/'humanoid_static.xml');reference=None;accepted=None
  try:
   for attempt in range(1,3):
    reset=weak.random_reset(rng,f'teacher-attempt-{attempt}');write(OUT/f'teacher-reset-{attempt}.json',reset);zeros(weak,f'teacher-{attempt}-preparation');end_next=False;knots={k:[] for k in ['qpos','qvel','com','com_ori']};summary={'attempt':attempt,'decisions':0,'terminalReached':False,'maximumHeadHeightM':0.}
    for tick in range(250):
     obs=weak.observation();u=teacher(np.r_[obs,POWER]);decisions.write(json.dumps({'phase':f'teacher-{attempt}','decision':tick,'observation':np.r_[obs,POWER].tolist(),'action':u.tolist()})+'\n')
     for j in range(5):row=advance(weak,f'teacher-{attempt}',tick*5+j+1,u,save_frame=j==4)
     summary['decisions']=tick+1;summary['maximumHeadHeightM']=max(summary['maximumHeadHeightM'],row['headHeightM'])
     knots['qpos'].append(weak.arr(0,(28,)).copy());knots['qvel'].append(weak.arr(1,(27,)).copy());knots['com'].append(weak.arr(6,(17,3))[weak.ids['torso']].copy());knots['com_ori'].append(weak.observation()[34:37])
     if end_next:summary['terminalReached']=True;break
     if row['headHeightM']>1.4:end_next=True
     if tick==249:summary['terminalReached']=end_next
    result['attempts'].append(summary);write(OUT/'result.json',result);print(json.dumps({'teacherMilestone':summary,'nativeSteps':result['nativeSteps']}),flush=True)
    if summary['terminalReached']:
     accepted=summary;reference=spline({k:np.array(v) for k,v in knots.items()},.5,False);break
   if reference is None:result['outcome']='source-teacher-failed-within-two-original-loop-attempts';result['completedDiagnostic']=True;return
   n=len(reference['qpos']);required=20*(n+150);result['referenceFrames']=n;result['requiredCompleteStudentSteps']=required
   np.savez_compressed(OUT/'source-reference.npz',**reference)
   if result['nativeSteps']+required>9760:
    result['outcome']='teacher-reference-only-complete-SLOW-cannot-fit';result['slowBudgetShortfallSteps']=result['nativeSteps']+required-9760
    if result['nativeSteps']+200<=9760:
     stable=Stable(.005)
     for i in range(200):advance(weak,'teacher-only-continuation',i+1,teacher(np.r_[weak.observation(),POWER]),stable,save_frame=i%5==4)
     result['teacherOnlyContinuationMaximumStableSeconds']=stable.maximum
    result['completedDiagnostic']=True
    return
   reference['qpos']=np.concatenate([reference['qpos'],np.r_[[0,0,1.5,0,0,0,0],STANDING][None]])
   reference['com']=np.concatenate([reference['com'],[[0,0,1.4]]]);reference['com_ori']=np.concatenate([reference['com_ori'],[[0,0,1.]]])
   actual.install(reference['qpos'][0],reference['qvel'][0]);result['studentStarted']=True;stable=Stable(.00125);result['studentInitialHeadHeightM']=float(actual.observation()[21]);write(OUT/'student-initial.json',actual.snapshot(0,np.zeros(21)))
   for tick in range(n+150):
    def target(i):return np.r_[reference['qpos'][i][7:],reference['com'][i,2],reference['com_ori'][i]]
    x=np.r_[actual.observation(),target(min(tick,n)),target(min(tick+5,n)),1.];residual=student(x);holding=tick>=n;decisions.write(json.dumps({'phase':'student','decision':tick,'observation':x.tolist(),'residual':residual.tolist(),'holding':holding})+'\n')
    for j in range(20):
     u=controls(actual.arr(0,(28,))[7:],actual.arr(1,(27,))[6:],STANDING if holding else reference['qpos'][tick][7:],residual,holding)
     row=advance(actual,'student',tick*20+j+1,u,stable,save_frame=j==19)
    step=tick+1;com=row['comSource'][2];reason=None
    if step>=n and com<.5:reason='source-done-low-CoM-holding'
    elif step<n and abs(com-reference['com'][step-1,2])>.5:reason='source-done-CoM-tracking-error'
    elif step>=n+150:reason='source-reference-plus150-hold-complete';result['completeSlowEpisode']=True
    result['studentControlDecisions']=step;result['studentTimeS']=step*.025;result['maximumStableStandingSeconds']=stable.maximum;result['firstStableStandingTimeS']=stable.first;result['actualStableStanding']=stable.first is not None;result['studentFinal']=row
    if reason:result['sourceDoneReason']=reason;break
   result['outcome']='complete-SLOW-with-verified-stable-standing' if result['completeSlowEpisode'] and result['actualStableStanding'] else 'SLOW-ended-without-verified-complete-stable-standing'
  finally:weak.close();actual.close()
  result['completedDiagnostic']=True
 except BaseException as error:
  result['completedDiagnostic']=False;result['error']=repr(error);raise
 finally:
  signal.alarm(0);stream.close();decisions.close();result['finishedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat();result['wallSeconds']=time.monotonic()-PROCESS_STARTED;result['staticForwards']=SourceWorld.forwards;write(OUT/'result.json',result);print(json.dumps({k:v for k,v in result.items() if k!='studentFinal'},indent=2),flush=True)
if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['prepare','run']);args=parser.parse_args();torch.set_num_threads(1);teacher=load_actor('teacher',68);student=load_actor('student',118);spline=source_spline()
 if args.mode=='prepare':prepare(teacher,student,spline)
 else:run(teacher,student,spline)
