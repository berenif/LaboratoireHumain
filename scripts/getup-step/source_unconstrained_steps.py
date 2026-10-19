#!/usr/bin/env python3
"""Explicit diagnostic: measured quality gates report-only; true native physics.

The stop-only comparison preserves the best controller's optimization and
physical phase admissions. The scheduled comparison also removes admission
requirements, while still recording whether each commanded phase achieved
its physical predicates. Neither comparison qualifies the production gait.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from source_step_probe import sha
from source_walk_limits import loaded_slip_exceeded,loaded_slip_stop_reason
from source_contact_impedance_probe import prepare_impedance
from source_robust_support_steps import RobustSupportSteps
from source_multistep_probe import posture_ready,loaded_settle_ready,unloaded_support_ready


class UnconstrainedSteps(RobustSupportSteps):
    def __init__(self,base,helper,output,count=10,scheduled=False):
        self.scheduled=scheduled
        self.quality_events={}
        self.quality_counts={}
        self.minimum_capture=1.
        self.minimum_joint=1.
        self.minimum_com_height=10.
        self.maximum_penetration=0.
        self.maximum_tilt=0.
        self.maximum_nonfoot_load=0.
        self.recorded_integrations=0
        super().__init__(base,helper,output,count)
        self.code_at_start[Path(__file__).name]=sha(__file__)

    def record_quality(self,name,row):
        self.quality_counts[name]=self.quality_counts.get(name,0)+1
        if name not in self.quality_events:
            self.quality_events[name]=dict(timeS=row['elapsedS'],nativeStep=row['sourceStep'],
                stepIndex=row.get('stepIndex',1),phase=row['phase'])
            print('REPORT ONLY',name,self.quality_events[name],flush=True)

    def decorate(self,row):
        row=super().decorate(row)
        if self.failure:
            self.record_quality(self.failure,row)
            self.failure=None
        return row

    def observe_quality(self,row):
        if row['warnings']:self.record_quality('native-warning',row)
        if row['nonfootLoadN']>=3:self.record_quality('nonfoot-contact',row)
        if all(f['loadN']<3 for f in row['feet']):self.record_quality('both-feet-unloaded',row)
        if loaded_slip_exceeded(self.slip,self.loaded_slip_limit_m):self.record_quality(loaded_slip_stop_reason(self.loaded_slip_limit_m),row)
        if row['maximumSelfPenetrationM']>.005:self.record_quality('self-penetration-5mm',row)
        if row['com'][2]<.65:self.record_quality('height-below-65cm',row)
        self.minimum_com_height=min(self.minimum_com_height,row['com'][2])
        self.minimum_joint=min(self.minimum_joint,row['jointLimitMinimumMarginRad'])
        self.maximum_penetration=max(self.maximum_penetration,row['maximumSelfPenetrationM'])
        self.maximum_tilt=max(self.maximum_tilt,row['torsoTiltDeg'])
        self.maximum_nonfoot_load=max(self.maximum_nonfoot_load,row['nonfootLoadN'])
        if row['minimumComCaptureMarginM'] is not None:self.minimum_capture=min(self.minimum_capture,row['minimumComCaptureMarginM'])

    def save(self,reason):
        if self.finished:reason='diagnostic-measured-steps-plus-five-seconds-quiet'
        super().save(reason)
        if hasattr(self,'folder'):
            diagnostic=dict(schema=1,diagnosticOnly=True,productionDefaultsChanged=False,
                mode='scheduled-admission'if self.scheduled else'report-only-stops',
                measuredQualityStopsFatal=False,optimizationConstraintsRetained=True,
                globalLoadedSlipLimitM=self.loaded_slip_limit_m,globalLoadedSlipStopEnabled=False,
                measuredPhaseAdmissionsRetained=not self.scheduled,
                gravityAndCollisionRetained=True,physicalMotorAndJointLimitsRetained=True,
                rootAssistance=False,physicalStateWritesAfterInitialization=0,
                playbackSpeed=1.,finiteNumericalGuardsRetained=True,
                requestedSteps=len(self.distances),actualIntegrations=self.steps,
                recordedIntegrations=self.recorded_integrations,
                unjournaledTerminalIntegrations=self.steps-self.recorded_integrations,
                physicsSimSeconds=self.steps*.00125,stopReason=reason,
                completedMeasuredSteps=sum('completedNativeStep'in r for r in self.step_records),
                commandCyclesStarted=len(self.step_records),finalQuietS=self.quiet_run*.00125,
                firstQualityViolations=self.quality_events,qualityViolationFrameCounts=self.quality_counts,
                globalLoadedSlipM=self.slip.tolist(),minimumCaptureMarginM=self.minimum_capture,
                minimumJointLimitMarginRad=self.minimum_joint,minimumCOMHeightM=self.minimum_com_height,
                maximumSelfPenetrationM=self.maximum_penetration,maximumTorsoTiltDeg=self.maximum_tilt,
                maximumNonfootLoadN=self.maximum_nonfoot_load)
            (self.folder/'unconstrained-diagnostic.json').write_text(json.dumps(diagnostic,indent=2)+'\n')
            path=self.folder/'summary.json';summary=json.loads(path.read_text())
            summary.update(diagnosticOnly=True,recordedIntegrations=self.recorded_integrations,
                unjournaledTerminalIntegrations=self.steps-self.recorded_integrations,
                originalQualityQualificationClaim=False,globalLoadedSlipStopEnabled=False)
            path.write_text(json.dumps(summary,indent=2)+'\n')
            path=self.folder/'multistep-summary.json';multi=json.loads(path.read_text())
            multi.update(passed=False,diagnosticOnly=True,sequenceAndQuietCompleted=self.finished,
                measuredQualityGatesPassed=self.finished and not bool(self.quality_counts),
                originalModelQualificationClaim=False,globalLoadedSlipStopEnabled=False)
            path.write_text(json.dumps(multi,indent=2)+'\n')
            for name in ('cadenced-step-config.json','large-step-config.json'):
                path=self.folder/name;config=json.loads(path.read_text())
                config.update(measuredQualityStopsFatal=False,globalLoadedSlipStopEnabled=False,
                    measuredPhaseAdmissionsRetained=not self.scheduled,
                    originalContactJointMotorCollisionCaptureGates=False,
                    originalMotorAndContactSafetyGates=False,runtimeMeasuredCaptureAndJointGuards=False,
                    physicalModelGravityCollisionsJointLimitsAndActuatorsPreserved=True,
                    planningBarriersEnabled=not self.scheduled)
                path.write_text(json.dumps(config,indent=2)+'\n')

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
      self.recorded_integrations=self.steps
      self.observe_quality(row)
      if not np.isfinite(np.r_[row['qpos'],row['qvel'],self.u]).all():
       self.failure='nonfinite-native-state-or-command';self.paused=True;break
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
       if hasattr(c,'prepare_phase_handoff'):c.prepare_phase_handoff(self.phase,nextphase,row,landing_consecutive_samples=self.land_run)
       if nextphase=='settle'and getattr(self,'loaded_settle_handoff',False):
        c.finalCom[:2]=np.mean(np.array([f['position']for f in row['feet']]),axis=0)[:2]+.035*c.heading[:2]
        self.step_records[-1].update(settleMeasuredFeet=[f['position']for f in row['feet']],settleComTargetSource=c.finalCom.tolist())
        path=self.folder/f'controller-step-{self.step_index+1:02d}.json';record=json.loads(path.read_text());record['finalCom']=c.finalCom.tolist();path.write_text(json.dumps(record)+'\n')
       self.events.append(dict(event=nextphase,timeS=self.steps*.00125,stepIndex=self.step_index+1));self.phase=nextphase;self.phase_start=self.steps*.00125;w.checkpoint(self.folder/f'step-{self.step_index+1:02d}-{nextphase}.bin');
       if not self.planned_support or nextphase in ['settle','stand']:c.q0=np.clip(w.arr(0,(28,))[7:].copy(),w.limits[:,0]+.002,w.limits[:,1]-.002)
      elif self.phase!='stand'and elapsed>c.phase_duration(self.phase)+2.:self.failure='admission-observation-ended-'+self.phase;self.paused=True;break
     self.journal.flush();return (self.steps-before+19)//20


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--inspector',type=Path,required=True)
    p.add_argument('--count',type=int,choices=(1,10),default=10)
    p.add_argument('--wall-cap-s',type=float,default=1800.)
    a=p.parse_args()
    assert 120.<=a.wall_cap_s<=3600.
    session,helper,_=prepare_impedance(a.output,a.inspector,8.)
    live=UnconstrainedSteps(session,helper,a.output/'run',a.count)
    live.paused=False;started=time.monotonic()
    print('DYNAMIC RUN START',str(live.folder),flush=True)
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started<a.wall_cap_s,'diagnostic wall process cap'
            live.advance(4)
            if live.steps%800==0:
                print('progress',live.steps*.00125,'step',live.step_index+1,live.phase,
                    'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
        print(live.failure or'diagnostic-ten-steps-and-five-seconds-quiet',flush=True)
    except Exception as error:
        live.failure=repr(error)
        print('NUMERICAL OR EXECUTION TERMINAL',repr(error),flush=True)
    finally:live.close()
    return 0 if live.finished else 1


if __name__=='__main__':raise SystemExit(main())
