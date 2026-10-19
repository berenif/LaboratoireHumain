#!/usr/bin/env python3
"""Diagnostic ten-cycle schedule with planning and admission gates removed.

Exact native dynamics, gravity, collisions, actual joint limits and original
actuator gears/strength remain. Only finite numeric execution is required.
A command cycle is never reported as an achieved physical step.
"""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import clarabel
import numpy as np
from scipy import sparse
from scipy.spatial.transform import Rotation

from source_step_probe import Controller,PTR,ptr,sha,clock
from source_opposed_arm_swing import OpposedArmController,limb_telemetry
from source_cadenced_ten_steps import PHASE_DURATIONS
from source_contact_impedance_probe import prepare_impedance
from source_unconstrained_steps import UnconstrainedSteps


class ExtrapolatedArmMap:
    def __init__(self,original):self.__dict__.update(original.__dict__)
    def evaluate(self,delta):
        return tuple(np.concatenate([curve(delta[i],order)for i,curve in enumerate(self.curves)])for order in (0,1,2))


class RelaxedController(OpposedArmController):
    # No planner geometry barriers. Desired trajectories remain torque tasks.
    def add_capture_constraints(self,*args):pass
    def solve(self, t, phase='transfer'):
        if hasattr(self, 'prepared_arm_measurement'):
            legs, leg_rate, leg_acc = self.prepared_arm_measurement()
        else:
            legs, leg_rate, _ = limb_telemetry(self.w, 'leg')
            now = self.w.l.lh_time(self.w.h)
            dt = None if self.previous_time is None else now-self.previous_time
            leg_acc = ((leg_rate-self.previous_leg_rate)/dt
                       if dt is not None and dt > 1e-8 else np.zeros(2))
            self.previous_time, self.previous_leg_rate = now, leg_rate.copy()
        delta = -(legs-self.leg_origin)
        mapped, jac, hessian = self._mapping.evaluate(delta)
        dq = jac*np.repeat(-leg_rate, 3)
        ddq = hessian*np.repeat(leg_rate**2, 3)-jac*np.repeat(leg_acc, 3)
        s, ds, dds = clock(t, getattr(self,'arm_placement_duration',4.)) if self.placement and phase == 'transfer' else (1., 0., 0.)
        target = self.arm_start+s*(mapped-self.arm_start)
        velocity = ds*(mapped-self.arm_start)+s*dq
        acceleration = dds*(mapped-self.arm_start)+2*ds*dq+s*ddq
        self.q0[[6, 12]] = self.knee_goal
        self.q0[15:] = target
        self.arm_posture = target
        self.arm_velocity = velocity
        self.arm_acceleration = acceleration
        if hasattr(self, 'prepare_reference_context'):
            self.prepare_reference_context(t, phase)
        u, core = self.relaxed_core(t, phase)
        actual, _, vectors = limb_telemetry(self.w, 'arm')
        core['opposedArmSwing'] = dict(measuredLegPitchRad=legs.tolist(), measuredLegPitchRateRadps=leg_rate.tolist(),
            legPitchOriginRad=self.leg_origin.tolist(), targetArmPitchChangeRad=delta.tolist(),
            targetArmPitchRad=(self._mapping.neutral_pitch+delta).tolist(),
            measuredArmPitchRad=actual.tolist(), measuredArmVectorsM=vectors.tolist(),
            targetArmJointRad=target.tolist(), targetArmVelocityRadps=velocity.tolist(),
            targetArmAccelerationRadps2=acceleration.tolist(), placementFraction=s,
            actualSagittalErrorRad=(actual-self._mapping.neutral_pitch-delta).tolist(),
            mappingClipped=False, amplitudeGain=-1., feedbackUsesMeasuredLeg=True)
        return u, core

    def relaxed_core(self,t,phase='transfer',reference_context=None):
     if reference_context is None:reference_context=getattr(self,'_prepared_reference_context',None)
     w=self.w;v=w.arr(1,(27,)).copy();q=w.arr(0,(28,))[7:].copy();M=w.read('lh_mass_matrix',(27,27));bias=w.read('lh_bias',(27,));Jcom=w.read('lh_com_jac',(3,27));com=w.arr(6,(17,3))[1].copy();feet=w.feet();single=phase in ['lift','translate']or(phase=='land'and w.arr(20,(17,))[self.swing_body]<3.);ncontact=4 if single else 8;N=48+ncontact*3
     Aeq=[];beq=[];costA=[];costb=[];ineq=[];ub=[]
     def task(J,target,weight):
      row=np.zeros((len(target),N));row[:,:27]=J;costA.extend(row*weight);costb.extend(np.asarray(target)*weight)
     motion_scale=getattr(self,'trajectory_time_scale',1.);s,ds,dds=clock(t,self.phase_duration('transfer'))
     target=self.com0+(self.com1-self.com0)*s if phase=='transfer'else self.com1
     vel=(self.com1-self.com0)*ds if phase=='transfer'else np.zeros(3);acc=(self.com1-self.com0)*dds if phase=='transfer'else np.zeros(3)
     if phase in ['settle','stand']:
      s,ds,dds=clock(t,self.phase_duration('settle'))if phase=='settle'else(1.,0.,0.);target=self.com1+(self.finalCom-self.com1)*s;vel=(self.finalCom-self.com1)*ds;acc=(self.finalCom-self.com1)*dds
     if reference_context is not None:
      reference_context.validate(self,t,phase);target,vel,acc=reference_context.com.arrays()
     elif hasattr(self,'adjust_com_reference'):target,vel,acc=self.adjust_com_reference(t,phase,target,vel,acc)
     task(Jcom,acc+45*(target-com)+13*(vel-Jcom@v)-w.read('lh_jdot',(3,),-1),12.)
     if reference_context is not None:
      for row,bound in reference_context.capture_constraints:ineq.append(np.array(row));ub.append(bound)
     elif hasattr(self,'add_capture_constraints'):self.add_capture_constraints(phase,N,Jcom,com,v,ineq,ub)
     if reference_context is not None:
      for orientation in reference_context.orientations:
       body=orientation.body;Jr=w.read('lh_body_jac',(6,27),body)[3:];R=w.arr(5,(17,3,3))[body];root_target,root_velocity,root_acceleration=orientation.arrays();rerr=Rotation.from_matrix(root_target@R.T).as_rotvec();task(Jr,root_acceleration+orientation.kp*rerr+orientation.kd*(root_velocity-Jr@v)-w.read('lh_jdot',(6,),body)[3:],orientation.weight)
     elif hasattr(self,'add_upright_tasks'):
      self.add_upright_tasks(t,phase,N,task,ineq,ub)
     else:
      Jr=w.read('lh_body_jac',(6,27),1)[3:];R=w.arr(5,(17,3,3))[1];root_target=self.rootR;root_velocity=np.zeros(3);root_acceleration=np.zeros(3)
      if self.self_avoidance:
       lean,lean_v,lean_a=clock(t,self.phase_duration('transfer'))if phase=='transfer'else(1.,0.,0.)
       if phase=='settle':lean,lean_v,lean_a=clock(t,self.phase_duration('settle'));lean,lean_v,lean_a=1.-lean,-lean_v,-lean_a
       elif phase=='stand':lean,lean_v,lean_a=0.,0.,0.
       root_target=Rotation.from_rotvec(self.lean_axis*lean).as_matrix()@self.rootR;root_velocity=self.lean_axis*lean_v;root_acceleration=self.lean_axis*lean_a
      rerr=Rotation.from_matrix(root_target@R.T).as_rotvec();task(Jr,root_acceleration+40*rerr+12*(root_velocity-Jr@v)-w.read('lh_jdot',(6,),1)[3:],4.)
     if reference_context is not None:
      joint_target,joint_velocity,joint_acceleration=reference_context.joints.arrays();task(np.eye(27)[6:],joint_acceleration+20*(joint_target-q)+8*(joint_velocity-v[6:]),.2)
     elif phase=='transfer'and hasattr(self,'posture_start'):
      ps,pv,pa=clock(t,self.phase_duration('transfer'));delta=self.q0-self.posture_start;task(np.eye(27)[6:],delta*pa+20*(self.posture_start+delta*ps-q)+8*(delta*pv-v[6:]),.2)
     else:task(np.eye(27)[6:],20*(self.q0-q)-8*v[6:],.2)
     if self.self_avoidance:
      if reference_context is not None:
       arm_target,arm_velocity,arm_acceleration=reference_context.arms.arrays();task(np.eye(27)[21:],arm_acceleration+reference_context.arm_gains[0]*(arm_target-q[15:])+reference_context.arm_gains[1]*(arm_velocity-v[21:]),reference_context.arm_gains[2])
      elif hasattr(self,'arm_acceleration_task'):
       arm=self.arm_acceleration_task(t,phase);task(arm.jacobian,arm.target,arm.weight)
      elif hasattr(self,'arm_kp'):task(np.eye(27)[21:],self.arm_acceleration+self.arm_kp*(self.arm_posture-q[15:])+self.arm_kd*(self.arm_velocity-v[21:]),self.arm_weight)
      else:task(np.eye(27)[21:],20*(self.arm_posture-q[15:])-10*v[21:],1.)
     contacts=[]
     landing_adjustment=self.landing_adjustment(t,phase)if reference_context is None and hasattr(self,'landing_adjustment')else None
     for side,foot in enumerate(feet):
      body=foot['body'];J=w.read('lh_body_jac',(6,27),body);b=w.read('lh_jdot',(6,),body);pos=w.arr(3,(17,3))[body];R=w.arr(5,(17,3,3))[body];pos0,Rinitial=self.targets[side];Rflat=self.flatRotations[side];axis=Rotation.from_matrix(Rflat@Rinitial.T).as_rotvec();pt=pos0.copy();pvel=np.zeros(3);pacc=np.zeros(3);wvel=np.zeros(3);wacc=np.zeros(3);R0=Rflat
      if phase=='transfer':
       sf,dsf,ddsf=clock(t,self.phase_duration('transfer'));R0=Rotation.from_rotvec(axis*sf).as_matrix()@Rinitial;wvel=axis*dsf;wacc=axis*ddsf;pt[2]+=(self.floorTargets[side]-pos0[2])*sf;pvel[2]=(self.floorTargets[side]-pos0[2])*dsf;pacc[2]=(self.floorTargets[side]-pos0[2])*ddsf
      else:pt[2]=self.floorTargets[side]
      if side==self.swing_side and phase!='transfer':
       s,ds,dds=clock(t,self.phase_duration(phase))if phase in ['lift','translate','land']else(1.,0.,0.)
       if phase=='lift':pt[2]+=.04*s;pvel[2]=.04*ds;pacc[2]=.04*dds
       elif phase=='translate':pt[2]+=.04;pt+=self.step_distance*self.heading*s;pvel=self.step_distance*self.heading*ds;pacc=self.step_distance*self.heading*dds
       elif phase=='land':pt=self.landing.copy();pt[2]+=.04*(1-s)-.0005*s;pvel[2]=-.0405*ds;pacc[2]=-.0405*dds
       else:pt=self.landing.copy();pt[2]-=.0005
       if landing_adjustment is not None:
        offset,rate,acceleration=landing_adjustment;pt[2]-=offset;pvel[2]-=rate;pacc[2]-=acceleration
      if reference_context is not None:pt,pvel,pacc,R0,wvel,wacc=reference_context.feet[side].arrays()
      desired=np.r_[pacc+100*(pt-pos)+20*(pvel-J[:3]@v),wacc+100*Rotation.from_matrix(R0@R.T).as_rotvec()+20*(wvel-J[3:]@v)]-b
      task(J,desired,20. if side==self.swing_side and phase in ['lift','translate','land']else 25.)
      if not single or side==self.stance_side:
       for point in foot['points']:
        p=point.copy();p[2]-=foot['radius'];contacts.append((body,p))
     dynamics=np.zeros((27,N));dynamics[:,:27]=M;dynamics[:,27:48]=-self.B
     for i,(body,p)in enumerate(contacts):
      J=w.read('lh_point_jac',(3,27),body,ptr(p));dynamics[:,48+3*i:51+3*i]=-J.T
      for direction in [np.array([1,0,-.6]),np.array([-1,0,-.6]),np.array([0,1,-.6]),np.array([0,-1,-.6]),np.array([0,0,-1.])]:
       row=np.zeros(N);row[48+3*i:51+3*i]=direction;ineq.append(row);ub.append(0.)
      row=np.zeros(N);row[50+3*i]=1.;costA.append(row*.001);costb.append(w.mass*9.81/ncontact*.001)
      for horizontal in [0,1]:
       shear=np.zeros(N);shear[48+3*i+horizontal]=1.;costA.append(shear*getattr(self,'shear_weight',.1));costb.append(0.)
     if phase=='transfer'and (reference_context.unload_admitted if reference_context is not None else getattr(self,'unload_admitted',False)):
      unload=np.zeros(N)
      for i,(body,point)in enumerate(contacts):
       if body==self.swing_body:unload[50+3*i]=1.
      ineq.append(unload);ub.append(0.)
     Aeq.extend(dynamics);beq.extend(-bias)
     for i in range(21):
      row=np.zeros(N);row[27+i]=1;ineq.extend([row,-row]);ub.extend([.95,.95])
     ground_adjustments=[];self_adjustments=[]
     ca=np.array(costA);cb=np.array(costb);P=ca.T@ca+np.diag(np.r_[np.full(27,1e-5),np.full(21,1e-3),np.full(ncontact*3,1e-8)]);linear=-ca.T@cb
     # Diagnostic planning has no capture, joint stopping, contact-cone,
     # unload, clearance, or self-avoidance inequalities. Applied controls retain
     # the actual actuator's [-1,1] range. Dynamics equality is not an assist.
     ineq=[];ub=[]
     for i in range(21):
      row=np.zeros(N);row[27+i]=1.;ineq.extend([row,-row]);ub.extend([1.,1.])
     AA=np.vstack([Aeq,ineq]);bb=np.r_[beq,ub]
     settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=100
     sol=clarabel.DefaultSolver(sparse.csc_matrix(P),linear,sparse.csc_matrix(AA),bb,
      [clarabel.ZeroConeT(len(beq)),clarabel.NonnegativeConeT(len(ub))],settings).solve()
     if str(sol.status)not in ['Solved','AlmostSolved']:raise RuntimeError('relaxed QP '+str(sol.status))
     preference=None
     x=np.array(sol.x);info=dict(comTarget=target.tolist(),comErrorM=float(np.linalg.norm(target-com)),predictedForcesN=x[48:N].reshape(-1,3).tolist(),maximumMotorFraction=float(np.max(abs(x[27:48]))),dynamicsResidual=float(np.max(abs(dynamics@x[:N]+bias))),jointGroundAdjustments=ground_adjustments,activeJointGroundBarriers=sum(row['active']for row in ground_adjustments),selfAvoidance=self_adjustments,activeSelfAvoidanceBarriers=sum(row['active']for row in self_adjustments))
     if preference is not None:info['softLoadHinge']=dict(policy='retained-acquired-load-hinge-v1',baseQPWidth=N,augmentedQPWidth=N+1,slackN=float(x[N]),targetNormalN=preference.floor_jet[0],predictedNewFootNormalN=float(sum(x[i]for i in normal_indices)),weightPerNewton=preference.weight,hardForceFloor=False)
     if reference_context is not None:
      info['referenceContext']=reference_context.receipt();info['qpAssemblySHA256']={name:hashlib.sha256(np.ascontiguousarray(value,dtype='<f8').tobytes()).hexdigest()for name,value in [('P',P),('linear',linear),('A',AA),('b',bb)]}
     return x[27:48],info
class ScheduledSteps(UnconstrainedSteps):
    def __init__(self,*args,**kwargs):
        self.recorded_integrations=0
        super().__init__(*args,**kwargs)

    def begin_step(self):
        if self.step_index==0:
            super().begin_step()
            self.arm_map=ExtrapolatedArmMap(self.arm_map)
        else:
            previous=self.controller
            # Use current measured feet and COM for a fresh reference; no
            # static feasibility optimization or inter-step physical reset.
            w=self.world;side=self.step_index%2
            self.controller=Controller(w,side,self.distances[self.step_index]+.002,
                self_avoidance=True,maximum_distance=.302)
            reference=dict(qReference=previous.q0.tolist(),comReferenceSource=self.controller.com1.tolist(),
                rootReferenceSource=previous.rootR.tolist())
            self.controller=OpposedArmController.from_controller(self.controller,reference,self.arm_map,
                self.leg_origin,False)
            self.controller.phase_durations=PHASE_DURATIONS.copy()
            self.step_start=self.steps;self.step_slip=self.slip.copy()
            self.phase='transfer';self.phase_start=self.steps*.00125
            self.clear_run=self.land_run=self.quiet_run=0
            self.unload_run=self.settle_ready_run=self.ready_run=0
            self.anchors=None
            self.step_records.append(dict(index=self.step_index+1,
                swingSide='left'if side==0 else'right',requestedDistanceM=self.distances[self.step_index],
                commandedDistanceM=self.controller.step_distance,startNativeStep=self.steps,
                startSourceTimeS=w.l.lh_time(w.h),entryCOM=w.arr(6,(17,3))[1].tolist(),
                referenceConstruction='current measured support; no feasibility solve or reach admission'))
            w.checkpoint(self.folder/f'step-{self.step_index+1:02d}-transfer.bin')
        self.controller.__class__=RelaxedController
        self.controller._mapping=self.arm_map
        self.controller.phase_durations=PHASE_DURATIONS.copy()
        if self.step_index==0:self.controller.phase_durations['transfer']=4.
        self.controller.arm_placement_duration=4.
        record={k:v for k,v in vars(self.controller).items()if k!='w'and not k.startswith('_')}
        (self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(json.dumps(record,default=lambda x:x.tolist())+'\n')

    def advance(self,ticks):
        if type(ticks)is not int or not 0<=ticks<=4:raise ValueError('invalid ticks')
        before=self.steps
        if self.paused or self.finished:return 0
        w=self.world
        for _ in range(ticks*20):
            if self.total>=self.maximum:
                self.failure='bounded-integration-budget';self.paused=True;break
            c=self.controller;t=self.steps*.00125;elapsed=t-self.phase_start
            if self.steps%4==0:
                self.u,info=c.solve(elapsed,self.phase)
                if not np.isfinite(self.u).all():raise ValueError('nonfinite-control')
                self.u=np.clip(self.u,-1.,1.)
                self.notes.append(dict(t=t,phase=self.phase,stepIndex=self.step_index+1,**info))
                self.max_motor=max(self.max_motor,float(max(abs(self.u))))
            w.arr(2,(21,))[:]=self.u;self.total+=1
            actual=w.l.lh_step_audited(w.h,self.maximum)
            assert actual==self.steps+1
            self.steps+=1;row=w.packet()
            if not np.isfinite(np.r_[row['qpos'],row['qvel']]).all():raise ValueError('nonfinite-state')
            self.slip+=np.array([f['loadedSlipSpeedMps']for f in row['feet']])*.00125
            self.loaded=np.where(np.array([f['loadN']for f in row['feet']])>=3,self.loaded+1,0)
            self.row=self.decorate(row);self.observe_quality(row)
            self.max_clear=max(self.max_clear,row['feet'][c.swing_side]['clearance'])
            self.journal.write(json.dumps(row,allow_nan=False)+'\n')
            self.recorded_integrations=self.steps
            phase=self.phase;nextphase=None
            swing=row['feet'][c.swing_side];support=row['feet'][c.stance_side]
            forward=row['swingForwardDisplacementM'];requested=self.distances[self.step_index]
            self.clear_run=self.clear_run+1 if phase=='lift'and swing['clearance']>=.025 and swing['loadN']<3 and support['loadN']>=.8*w.mass*9.81 else 0
            self.land_run=self.land_run+1 if phase=='land'and swing['loadN']>=3 and support['loadN']>=3 and swing['clearance']<=.003 and forward>=requested else 0
            if phase!='stand'and elapsed>=c.phase_duration(phase):
                nextphase={'transfer':'lift','lift':'translate','translate':'land','land':'settle','settle':'stand'}[phase]
                if phase=='lift':self.step_records[-1]['lift25mmConsecutiveS']=self.clear_run*.00125
                if phase=='land':self.step_records[-1].update(landingObserved=bool(self.land_run>=80),
                    landingConsecutiveS=self.land_run*.00125,achievedAtScheduledLandingM=forward)
                if phase=='settle'and self.step_index<len(self.distances)-1:
                    self.step_records[-1].update(commandCycleEndS=self.steps*.00125,
                        actualForwardM=forward,physicalLandingObserved=self.step_records[-1].get('landingObserved',False))
                    self.step_index+=1;self.begin_step()
                    self.events.append(dict(event='next-scheduled-cycle',timeS=self.steps*.00125,stepIndex=self.step_index+1))
                    print('NEXT SCHEDULED CYCLE',self.step_index+1,flush=True);continue
            elif phase=='stand'and elapsed>=5.:
                self.step_records[-1].update(commandCycleEndS=self.steps*.00125,actualForwardM=forward,
                    physicalLandingObserved=self.step_records[-1].get('landingObserved',False))
                self.failure='scheduled-ten-cycles-and-five-second-observation-ended'
                self.paused=True;break
            if nextphase:
                if nextphase=='settle':c.finalCom[:2]=np.mean(np.array([f['position']for f in row['feet']]),axis=0)[:2]+.035*c.heading[:2]
                self.events.append(dict(event=nextphase,timeS=self.steps*.00125,stepIndex=self.step_index+1,scheduled=True))
                self.phase=nextphase;self.phase_start=self.steps*.00125
                w.checkpoint(self.folder/f'step-{self.step_index+1:02d}-{nextphase}.bin')
        self.journal.flush();return(self.steps-before+19)//20

    def save(self,reason):
        super().save(reason)
        if hasattr(self,'folder'):
            p=self.folder/'unconstrained-diagnostic.json';r=json.loads(p.read_text())
            r.update(mode='relaxed-planning-and-scheduled-admission',optimizationConstraintsRetained=False,
                retainedOptimizationConstraints=['exact-mass-matrix-dynamics-equality','original-actuator-command-range'],
                removedPlanningConstraints=['capture','arm-map-range','joint-stopping-reserve','self-avoidance',
                    'collider-ground-clearance','virtual-contact-friction-cones','unload-force'],
                measuredPhaseAdmissionsRetained=False,staticReferenceAdmissionAfterFirstStep=False,
                initialReferenceUsesExistingStaticSetup=True,
                recordedIntegrations=self.recorded_integrations,
                unjournaledTerminalIntegrations=self.steps-self.recorded_integrations,
                physicalLandingsObserved=sum(bool(s.get('landingObserved'))for s in self.step_records),
                scheduledCyclesFinished=sum('commandCycleEndS'in s for s in self.step_records),
                noStepSuccessClaimFromSchedule=True)
            p.write_text(json.dumps(r,indent=2)+'\n')
            for name in ('cadenced-step-config.json','large-step-config.json'):
                p=self.folder/name;config=json.loads(p.read_text())
                config.update(measuredQualityStopsFatal=False,measuredPhaseAdmissionsRetained=False,
                    originalContactJointMotorCollisionCaptureGates=False,
                    originalMotorAndContactSafetyGates=False,runtimeMeasuredCaptureAndJointGuards=False,
                    physicalModelGravityCollisionsJointLimitsAndActuatorsPreserved=True,
                    planningBarriersEnabled=False)
                if 'robustSupportAdmission'in config:config['robustSupportAdmission']['enabled']=False
                p.write_text(json.dumps(config,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--inspector',type=Path,required=True)
    a=p.parse_args()
    session,helper,_=prepare_impedance(a.output,a.inspector,8.)
    live=ScheduledSteps(session,helper,a.output/'run',10,True)
    live.code_at_start[Path(__file__).name]=sha(__file__)
    live.paused=False;started=time.monotonic()
    print('RELAXED DYNAMIC START',str(live.folder),flush=True)
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started<1200.,'1200s process cap'
            live.advance(4)
            if live.steps%800==0:print('progress',live.steps*.00125,'cycle',live.step_index+1,live.phase,'slip',live.slip,flush=True)
    except KeyboardInterrupt:
        live.failure='operator-interrupted-diagnostic'
        print(live.failure,flush=True)
    except Exception as error:
        live.failure=repr(error)
        print('EXECUTION TERMINAL',repr(error),flush=True)
    finally:live.close()
    return 0 if live.failure=='scheduled-ten-cycles-and-five-second-observation-ended'else 1


if __name__=='__main__':raise SystemExit(main())
