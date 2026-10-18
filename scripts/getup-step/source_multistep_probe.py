#!/usr/bin/env python3
"""Alternating feedback-controlled source steps, one uninterrupted engine world."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from source_step_probe import World,Controller,ROOT,sha
from source_step_preview import LiveStep
from scipy.spatial.transform import Rotation
def posture_ready(row,c):
 com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
 goal=c.transfer_ready_goal()if hasattr(c,'transfer_ready_goal')else c.com1
 return (not hasattr(c,'robust_support_ready')or c.robust_support_ready(row))and np.linalg.norm((com-goal)[:2])<.008 and np.linalg.norm(capture-goal[:2])<.01 and row['jointLimitMinimumMarginRad']>=.005 and min(row['jointLimitMarginsRad'][i]for i in (7,13))>=getattr(c,'minimum_ankle_ready_rad',0.) and row['maximumBodyLinearMps']<=.05 and row['maximumBodyAngularRadps']<=.2
def loaded_settle_ready(row,c,mass):
 # Loaded double-support admission, not an intermediate quiet hold.
 com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
 return (all(f['loadN']>=3 and f['clearance']<=.003 for f in row['feet'])
  and sum(f['loadN']for f in row['feet'])>=.8*mass*9.81
  and row['minimumComCaptureMarginM']is not None and row['minimumComCaptureMarginM']>=.01
  and np.linalg.norm((com-c.finalCom)[:2])<.008 and np.linalg.norm(capture-c.finalCom[:2])<.01
  and row['jointLimitMinimumMarginRad']>=.005
  and row['maximumBodyLinearMps']<=.05 and row['maximumBodyAngularRadps']<=.2)
def unloaded_support_ready(row,c,mass):
 return posture_ready(row,c)and row['feet'][c.swing_side]['loadN']<.5 and row['feet'][c.stance_side]['loadN']>=.8*mass*9.81
class MultiStep(LiveStep):
 def __init__(self,base,helper,output,distances=(.03,.06,.09,.045),planned_support=False,time_scale=1.):
  assert .75<=time_scale<=2. and(planned_support or time_scale==1.),'time scaling requires the planned-support controller';self.time_scale=time_scale;self.planned_support=planned_support;self.distances=tuple(distances);self.step_index=0;self.step_records=[];self.ready_run=0;super().__init__(base,helper,output,maximum_integrations=getattr(self,'sequence_integration_budget',64000 if time_scale>1 else 48000));self.begin_step()
 def begin_step(self):
  w=self.world;side=self.step_index%2;requested=self.distances[self.step_index];distance_limit=getattr(self,'experimental_distance_limit',.12);candidate=Controller(w,side,requested+.002,self_avoidance=True,maximum_distance=distance_limit);separation=float((candidate.targets[side][0]-candidate.targets[1-side][0])@candidate.heading);maximum=max(.025,getattr(self,'support_relative_reach',.18)-separation);chosen=min(requested+.002,maximum);assert chosen>=requested,'requested placement outside the bounded support-relative reach';self.controller=Controller(w,side,chosen,self_avoidance=True,maximum_distance=distance_limit);self.unload_run=0;self.settle_ready_run=0
  if self.planned_support:
   from plan_source_stance import plan
   reference=plan(w,1-side,True,.9);assert reference['staticMotorContactFeasible']and reference['minimumRequiredMotorFraction']<.925 and reference['minimumSelfSeparationM']>.01,'no motor-reserved stance reference';self.controller.com1=np.array(reference['comReferenceSource']);self.controller.lean_axis=Rotation.from_matrix(np.array(reference['rootReferenceSource'])@self.controller.rootR.T).as_rotvec();self.controller.q0=np.array(reference['qReference']);self.controller.joint_reserve=.01;self.controller.trajectory_time_scale=self.time_scale;self.controller.posture_start=w.arr(0,(28,))[7:].copy();(self.folder/f'stance-reference-{self.step_index+1:02d}.json').write_text(json.dumps(reference,indent=2)+'\n')
  self.step_start=self.steps;self.step_slip=self.slip.copy();self.phase='transfer';self.phase_start=self.steps*.00125;self.clear_run=self.land_run=self.quiet_run=0;self.anchors=None;self.ready_run=0;self.controller.solve(0.,'transfer');w.checkpoint(self.folder/f'step-{self.step_index+1:02d}-transfer.bin');reference={k:(v.tolist()if isinstance(v,np.ndarray)else[[a.tolist(),b.tolist()]for a,b in v]if k=='targets'else v)for k,v in vars(self.controller).items()if k!='w'};(self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(json.dumps(reference,default=lambda x:x.tolist())+'\n')
  self.step_records.append(dict(index=self.step_index+1,swingSide='left'if side==0 else'right',requestedDistanceM=requested,commandedDistanceM=chosen,startNativeStep=self.steps,startSourceTimeS=w.l.lh_time(w.h),entryCOM=w.arr(6,(17,3))[1].copy().tolist(),initialFeet=[f['position']for f in w.packet()['feet']],bodyAdaptsThrough='current-state COM/capture feedback, measured contact load, whole-body inverse dynamics, joint ground barriers; no inter-step state installation'))
 def decorate(self,row):
  row=super().decorate(row)
  if hasattr(self,'step_start'):
   c=self.controller;row['stepIndex']=self.step_index+1;row['swingSide']=c.swing_side;row['requestedForwardM']=self.distances[self.step_index];row['swingForwardDisplacementM']=float((np.array(row['feet'][c.swing_side]['position'])-c.targets[c.swing_side][0])@c.heading)
  return row
 def advance(self,ticks):
  if type(ticks)is not int or not 0<=ticks<=4:raise ValueError('invalid tick count')
  before=self.steps
  if self.paused or self.finished:return 0
  w=self.world
  for _ in range(ticks*20):
   if self.total>=self.maximum:self.failure='bounded-multistep-budget';self.paused=True;break
   c=self.controller;side=c.swing_side;stance=1-side;requested=self.distances[self.step_index];t=self.steps*.00125;elapsed=t-self.phase_start
   if self.steps%4==0:
    self.u,info=c.solve(elapsed,self.phase);self.notes.append(dict(t=t,phase=self.phase,stepIndex=self.step_index+1,**info));self.max_motor=max(self.max_motor,float(max(abs(self.u))))
   w.arr(2,(21,))[:]=self.u;self.total+=1;actual=w.l.lh_step_audited(w.h,self.maximum);assert actual==self.steps+1;self.steps+=1;row=w.packet();self.slip+=np.array([f['loadedSlipSpeedMps']for f in row['feet']])*.00125;self.loaded=np.where(np.array([f['loadN']for f in row['feet']])>=3,self.loaded+1,0);self.row=self.decorate(row);self.max_clear=max(self.max_clear,row['feet'][side]['clearance']);self.journal.write(json.dumps(row,allow_nan=False)+'\n');forward=row['swingForwardDisplacementM'];swing=row['feet'][side];support=row['feet'][stance]
   if row['warnings']or row['nonfootLoadN']>=3 or all(f['loadN']<3 for f in row['feet']):self.failure='support-or-warning-guard'
   elif max(self.slip)>.01:self.failure='global-loaded-slip-10mm'
   elif row['maximumSelfPenetrationM']>.005:self.failure='self-penetration-5mm'
   elif row['com'][2]<.65:self.failure='height-guard'
   if self.failure:self.paused=True;break
   nextphase=None
   if self.phase=='transfer'and elapsed>=c.phase_duration(self.phase):
    com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
    if self.planned_support:
     ready_posture=posture_ready(row,c)
     if ready_posture:c.unload_admitted=True;c.floorTargets[side]=w.ground_height(c.targets[side][0])+c.radii[side]+.0001
     self.unload_run=self.unload_run+1 if unloaded_support_ready(row,c,w.mass)else 0
     if self.unload_run>=80:nextphase='lift'
    elif np.linalg.norm((com-c.com1)[:2])<.012 and np.linalg.norm(capture-c.com1[:2])<.015:nextphase='lift'
   elif self.phase=='lift':
    self.clear_run=self.clear_run+1 if swing['clearance']>=.025 and swing['loadN']<3 and support['loadN']>=.8*w.mass*9.81 else 0
    if elapsed>=c.phase_duration(self.phase) and self.clear_run>=80:nextphase='translate';self.step_records[-1]['lift25mmConsecutiveS']=self.clear_run*.00125
   elif self.phase=='translate'and elapsed>=c.phase_duration(self.phase):
    if forward>=requested and swing['clearance']>=.025 and swing['loadN']<3:nextphase='land'
   elif self.phase=='land':
    self.land_run=self.land_run+1 if swing['loadN']>=3 and support['loadN']>=3 and swing['clearance']<=.003 and forward>=requested else 0
    if elapsed>=c.phase_duration(self.phase) and self.land_run>=80:
     nextphase='settle';self.step_records[-1].update(landingNativeStep=self.steps,landingTimeS=self.steps*.00125,achievedAtLoadAcquisitionM=forward,measuredSupportAnchors=[f['position']for f in row['feet']],swingLoadedN=swing['loadN'],stanceLoadedN=support['loadN'])
     if getattr(self,'continuous_loaded_handoff',False)and self.step_index<len(self.distances)-1:
      self.step_records[-1].update(completedNativeStep=self.steps,completedTimeS=self.steps*.00125,achievedForwardM=forward,loadedSlipThisStepM=(self.slip-self.step_slip).tolist(),bodyLinearMps=row['maximumBodyLinearMps'],bodyAngularRadps=row['maximumBodyAngularRadps'],comCaptureMarginM=row['minimumComCaptureMarginM'],quietConsecutiveS=0.,completedAt='measured-loaded-landing')
      self.step_index+=1;self.begin_step();self.events.append(dict(event='next-actual-support-step',timeS=self.steps*.00125,stepIndex=self.step_index+1));print('NEXT STEP',self.step_index+1,'requested',self.distances[self.step_index],flush=True);continue

   elif self.phase=='settle':
    if getattr(self,'loaded_settle_handoff',False)and self.step_index<len(self.distances)-1:
     self.settle_ready_run=self.settle_ready_run+1 if loaded_settle_ready(row,c,w.mass)else 0
     if elapsed>=c.phase_duration(self.phase)and self.settle_ready_run>=80 and forward>=requested:
      self.step_records[-1].update(completedNativeStep=self.steps,completedTimeS=self.steps*.00125,achievedForwardM=forward,loadedSlipThisStepM=(self.slip-self.step_slip).tolist(),bodyLinearMps=row['maximumBodyLinearMps'],bodyAngularRadps=row['maximumBodyAngularRadps'],comCaptureMarginM=row['minimumComCaptureMarginM'],quietConsecutiveS=0.,loadedSettleReadyConsecutiveS=self.settle_ready_run*.00125,completedAt='measured-loaded-settle')
      self.step_index+=1;self.begin_step();self.events.append(dict(event='next-actual-support-step',timeS=self.steps*.00125,stepIndex=self.step_index+1));print('NEXT STEP',self.step_index+1,'requested',self.distances[self.step_index],flush=True);continue
    elif elapsed>=c.phase_duration(self.phase):nextphase='stand'
   elif self.phase=='stand':
    stable=row['maximumBodyLinearMps']<=.05 and row['maximumBodyAngularRadps']<=.15 and row['minimumComCaptureMarginM']is not None and row['minimumComCaptureMarginM']>=.01 and all(f['loadN']>=3 for f in row['feet'])and self.quiet_run>0
    self.ready_run=self.ready_run+1 if stable else 0
    final=self.step_index==len(self.distances)-1
    if (final and self.quiet_run>=4000 or not final and self.ready_run>=200)and forward>=requested:
     self.step_records[-1].update(completedNativeStep=self.steps,completedTimeS=self.steps*.00125,achievedForwardM=forward,loadedSlipThisStepM=(self.slip-self.step_slip).tolist(),bodyLinearMps=row['maximumBodyLinearMps'],bodyAngularRadps=row['maximumBodyAngularRadps'],comCaptureMarginM=row['minimumComCaptureMarginM'],quietConsecutiveS=self.quiet_run*.00125)
     if final:self.finished=True;self.paused=True;break
     self.step_index+=1;self.begin_step();self.events.append(dict(event='next-actual-support-step',timeS=self.steps*.00125,stepIndex=self.step_index+1));print('NEXT STEP',self.step_index+1,'requested',self.distances[self.step_index],flush=True);continue
   if nextphase:
    if nextphase=='settle'and getattr(self,'loaded_settle_handoff',False):
     c.finalCom[:2]=np.mean(np.array([f['position']for f in row['feet']]),axis=0)[:2]+.035*c.heading[:2]
     self.step_records[-1].update(settleMeasuredFeet=[f['position']for f in row['feet']],settleComTargetSource=c.finalCom.tolist())
     path=self.folder/f'controller-step-{self.step_index+1:02d}.json';record=json.loads(path.read_text());record['finalCom']=c.finalCom.tolist();path.write_text(json.dumps(record)+'\n')
    self.events.append(dict(event=nextphase,timeS=self.steps*.00125,stepIndex=self.step_index+1));self.phase=nextphase;self.phase_start=self.steps*.00125;w.checkpoint(self.folder/f'step-{self.step_index+1:02d}-{nextphase}.bin');
    if not self.planned_support or nextphase in ['settle','stand']:c.q0=np.clip(w.arr(0,(28,))[7:].copy(),w.limits[:,0]+.002,w.limits[:,1]-.002)
   elif self.phase!='stand'and elapsed>c.phase_duration(self.phase)+2.:self.failure='phase-timeout-'+self.phase;self.paused=True;break
  self.journal.flush();return (self.steps-before+19)//20
 def close(self):
  if self.world:
   self.save('measured-alternating-multistep-plus5squiet'if self.finished else self.failure or'preview-closed');self.journal.close();self.world.checkpoint(self.folder/'final.bin');self.world.close();self.world=None
 def close(self):
  if self.world:
   self.save('measured-alternating-multistep-plus5squiet'if self.finished else self.failure or'preview-closed');self.journal.close();self.world.checkpoint(self.folder/'final.bin');self.world.close();self.world=None
 def save(self,reason):
  super().save(reason)
  if hasattr(self,'folder')and hasattr(self,'step_records'):
   (self.folder/'multistep-summary.json').write_text(json.dumps(dict(stopped=reason,passed=self.finished,steps=self.step_records,actualIntegrations=self.steps,totalActualIntegrations=self.total,physicsSimSeconds=self.steps*.00125,finalQuietS=self.quiet_run*.00125,globalLoadedSlipM=self.slip.tolist(),final=self.row,physicalStateInstallationsBetweenSteps=0,controllerSHA256=sha(ROOT/'scripts/getup-step/source_step_probe.py'),runnerSHA256=sha(__file__)),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--helper',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--time-scale',type=float,default=1.);p.add_argument('--planned-support',action='store_true');p.add_argument('--distances',type=float,nargs='+',default=[.03,.06,.09,.045]);a=p.parse_args();assert 2<=len(a.distances)<=6 and all(.025<=d<=.09 for d in a.distances);live=MultiStep(a.base,a.helper,a.output,tuple(a.distances),a.planned_support,a.time_scale);live.paused=False;started=time.monotonic()
 try:
  while not live.finished and not live.failure:
   assert time.monotonic()-started<300.,'300s process cap';live.advance(4)
   if live.steps%800==0:print('progress',live.steps*.00125,'step',live.step_index+1,live.phase,'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
  reason='measured-alternating-multistep-plus5squiet'if live.finished else live.failure;live.save(reason);print(reason,[(s['requestedDistanceM'],s.get('achievedForwardM'))for s in live.step_records],flush=True)
 except Exception as error:
  live.failure=repr(error);live.save(live.failure);raise
 finally:live.close()
