#!/usr/bin/env python3
"""One source SLOW continuation after a serialization failure; never refunds steps.
Reuses the completed archived teacher and exact archived initial student state.
The interrupted first student step remains counted; this full student repeats it.
"""
import ctypes as ct,datetime,json,math,pathlib,signal,time
START=time.monotonic()
import numpy as np
import torch
from source_baseline import OUT,ROOT,LEDGER,SOURCE,STANDING,SourceWorld,Stable,controls,load_actor,sha,write

def main():
 r=json.loads(LEDGER.read_text());assert r['status']=='reserved';c=r['harnessContinuation'];assert c['maximumContinuations']==1
 old=json.loads((OUT/'interrupted-segment/result.json').read_text());assert old['nativeSteps']==966 and old['phaseSteps']['student']==1 and 'not JSON serializable' in old['error']
 assert r['nativeChargedAfter']==299998 and r['nativeReservation']==9760 and r['wallReservationSeconds']==180
 for p,h in r['continuationInputs'].items():assert sha(ROOT/p)==h,p
 (OUT/'continuation-started-once').open('x').close()
 remaining=180-old['wallSeconds'];signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(TimeoutError('Aggregate180s source bound')));signal.alarm(max(1,int(remaining-(time.monotonic()-START)-3)))
 result=dict(old);result.pop('error',None);result['completedDiagnostic']=False;result['harnessContinuation']={'reason':old['error'],'previousCountedSteps':966,'previousWallSeconds':old['wallSeconds'],'unarchivedInterruptedStudentSteps':1,'referenceRegenerated':False,'preparationOrTeacherReplayed':False,'initialStudentStateReinstalled':True,'sourceStudentRestartCount':1}
 ref={k:v.copy() for k,v in np.load(OUT/'source-reference.npz',allow_pickle=False).items()};n=len(ref['qpos']);required=20*(n+150);assert required==4320 and old['nativeSteps']+required<=9760
 ref['qpos']=np.concatenate([ref['qpos'],np.r_[[0,0,1.5,0,0,0,0],STANDING][None]]);ref['com']=np.concatenate([ref['com'],[[0,0,1.4]]]);ref['com_ori']=np.concatenate([ref['com_ori'],[[0,0,1.]]])
 torch.set_num_threads(1);predict=load_actor('student',118);world=SourceWorld(SOURCE/'humanoid_high_freq.xml');stable=Stable(.00125)
 stream=(OUT/'student-continuation-states.ndjson').open('x');decisions=(OUT/'student-continuation-decisions.ndjson').open('x')
 try:
  initial=json.loads((OUT/'student-initial.json').read_text());assert np.max(np.abs(np.array(initial['qpos'])-ref['qpos'][0]))<1e-12 and np.max(np.abs(np.array(initial['qvel'])-ref['qvel'][0]))<1e-12
  world.install(initial['qpos'],initial['qvel']);assert np.max(np.abs(world.observation()-np.array(initial['observation'])))<1e-12
  original=next(json.loads(s) for s in (OUT/'decisions.ndjson').read_text().splitlines() if json.loads(s)['phase']=='student')
  def target(i):return np.r_[ref['qpos'][i][7:],ref['com'][i,2],ref['com_ori'][i]]
  x=np.r_[world.observation(),target(0),target(min(5,n)),1.];assert np.array_equal(predict(x),np.array(original['residual'],dtype=np.float32))
  result['harnessContinuation']['initialObservationAndResidualExactlyMatched']=True
  for tick in range(n+150):
   x=np.r_[world.observation(),target(min(tick,n)),target(min(tick+5,n)),1.];residual=predict(x);holding=tick>=n;decisions.write(json.dumps({'phase':'student-continuation','decision':tick,'observation':x.tolist(),'residual':residual.tolist(),'holding':holding})+'\n')
   for j in range(20):
    assert result['nativeSteps']<9760 and time.monotonic()-START+old['wallSeconds']<177
    u=controls(world.arr(0,(28,))[7:],world.arr(1,(27,))[6:],STANDING if holding else ref['qpos'][tick][7:],residual,holding)
    result['nativeSteps']+=1;result['phaseSteps']['student-continuation']=result['phaseSteps'].get('student-continuation',0)+1;write(OUT/'result.json',result)
    world.arr(2,(21,))[:]=u;local=tick*20+j+1;assert world.lib.lh_step(world.h,4320)==local
    row=world.snapshot(local,np.clip(u,-1,1));row['phase']='student-continuation';row['phaseNativeStep']=local;row['rawControlMaximumAbs']=float(np.max(np.abs(u)));row['standing']=stable.update(world,row,local)
    result['maximumSelfPenetrationM']=max(result['maximumSelfPenetrationM'],row['maximumSelfPenetrationM']);result['maximumFloorPenetrationM']=max(result['maximumFloorPenetrationM'],max([0.]+[-p['distanceM'] for p in row['contacts'] if 'world' in [p['body1'],p['body2']]]))
    if j==19:
     with (OUT/'frames.ndjson').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
    else:row.pop('bodyPosesSource')
    stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
   step=tick+1;com=row['comSource'][2];reason=None
   if step>=n and com<.5:reason='source-done-low-CoM-holding'
   elif step<n and abs(com-ref['com'][step-1,2])>.5:reason='source-done-CoM-tracking-error'
   elif step>=n+150:reason='source-reference-plus150-hold-complete';result['completeSlowEpisode']=True
   result.update(studentControlDecisions=step,studentTimeS=step*.025,maximumStableStandingSeconds=stable.maximum,firstStableStandingTimeS=stable.first,actualStableStanding=stable.first is not None,studentFinal=row)
   if reason:result['sourceDoneReason']=reason;break
  result['outcome']='complete-SLOW-with-verified-stable-standing' if result['completeSlowEpisode'] and result['actualStableStanding'] else 'SLOW-ended-without-verified-complete-stable-standing';result['completedDiagnostic']=True;assert world.dim(6)==result['phaseSteps']['student-continuation']
 except BaseException as error:result['completedDiagnostic']=False;result['error']=repr(error);raise
 finally:
  signal.alarm(0);stream.close();decisions.close();world.close();result['finishedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat();result['continuationWallSeconds']=time.monotonic()-START;result['wallSeconds']=old['wallSeconds']+result['continuationWallSeconds'];result['staticForwards']=old['staticForwards']+SourceWorld.forwards;write(OUT/'result.json',result);print(json.dumps({k:v for k,v in result.items() if k!='studentFinal'},indent=2))
if __name__=='__main__':main()
