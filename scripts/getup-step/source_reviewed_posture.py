#!/usr/bin/env python3
"""Isolated, explicitly unqualified coupled-posture native pilot.

One current-state static family is frozen after the baseline correction finishes.
Its geometry stays provisional until fresh 100 ms posture risk history qualifies.
No live pose write, root force, or physical gate change.
"""
import argparse
from collections import deque
from dataclasses import asdict, replace
from scipy.spatial.transform import Rotation
import json
from pathlib import Path
import shutil
import time
import numpy as np

from source_step_probe import clock, sha, ROOT, ptr
from source_reviewed_walk import ReviewedController, ReviewedSteps
from source_robust_support_steps import actual_stance_patch
from source_contact_impedance_probe import prepare_impedance
from source_reference_context import Jet as ContextJet, Orientation, canonical
from source_causal_com_overlap import certify_overlap
from source_continuous_com import validate_reference
from coupled_posture_reference import (Jet, OrientationJet, CoupledSample, CoupledPlan,
    C2Segment, HeadingPath, CurrentSupport, orientation_residual, chart)
from source_posture_retarget import retarget_current_reference, initial_coupled_reference
from coupled_posture_kinematics import ModelFK, pose_jet_from_source, pose_jet_to_source
from source_bounded_com_settle import build_bounded_settle
from source_continuous_com import QuinticSegment

PROFILE = 'reviewed-coupled-posture-pilot-v3'
DURATION = 1.52
# Joint-only initializer of the independently checked active-foot static family.
# It supplies neither world position nor an applied pose; one bounded optimizer
# reconstructs current foot/COM/orientation targets and a separate scratch recheck.
SEED = (0.18050181924398612, -0.2972829659936228, -0.19732176749503477, 0.14638199997990162, -0.07062275347295917, 0.123107691197508, -0.4568222193594367, -0.2890658503988637, -0.15162675091264105, -0.19939141296204496, 0.11589658398035425, 0.017912581470861395, -0.4782739862277805, -0.22022151461437406, -0.2220632498590764)
RUNTIME = ('source_reviewed_posture.py','source_posture_retarget.py',
 'coupled_posture_reference.py','coupled_posture_kinematics.py','coupled_posture_admission.py',
 'source_bounded_com_settle.py')


def inherited_plan(start, initial, candidate, limits):
    """Explicit transition from the inherited reference, strict endpoint reserve.

    Generic CoupledPlan.between still requires .06 throughout both ankles. This
    isolated handoff preserves the inherited .02 ankle-reference floor while
    monotonically acquiring .06 at its endpoint. It does not alter native guards.
    """
    limits=np.asarray(limits); q=np.array(initial.lower_joints.position)
    existing=np.minimum(q-limits[:15,0],limits[:15,1]-q)
    if np.any(existing < .02-1e-10):
        raise ValueError('inherited posture reference lacks original .02 interior')
    endpoint=np.array(candidate.target.lower_joints.position)
    required=np.full(15,.02);required[[7,13]]=.06
    if np.any(np.minimum(endpoint-limits[:15,0],limits[:15,1]-endpoint)<required-1e-10):
        raise ValueError('posture endpoint lacks reserved joints/ankles')
    if any(abs(x)>1e-10 for jet in (initial.com,initial.lower_joints)
            for row in (jet.velocity,jet.acceleration) for x in row):
        raise ValueError('minimal isolated posture pilot requires stationary incoming COM/joint jets')
    plan=CoupledPlan(candidate,C2Segment.between(start,DURATION,initial.com,candidate.target.com),
        C2Segment.between(start,DURATION,initial.lower_joints,candidate.target.lower_joints),
        HeadingPath.between(start,DURATION,initial.thorax,candidate.target.thorax),
        HeadingPath.between(start,DURATION,initial.pelvis,candidate.target.pelvis),
        tuple(q[:3]),tuple(map(tuple,limits)))
    floor=np.minimum(existing,required);bounds=plan.lower_joints.component_bounds()
    if np.any(bounds[:,0]<limits[:15,0]+floor-1e-10) or np.any(bounds[:,1]>limits[:15,1]-floor+1e-10):
        raise ValueError('inherited posture transition decreases its declared reserve floor')
    if np.any(np.maximum(abs(bounds[:3,0]-q[:3]),abs(bounds[:3,1]-q[:3]))>np.deg2rad(5)+1e-10):
        raise ValueError('posture transition exceeds anchored lumbar change')
    if plan.com.maximum_norm(1)>.19260552331163003+1e-10 or plan.com.maximum_norm(2)>.3901818436690813+1e-10:
        raise ValueError('posture transition exceeds frozen COM envelope')
    return plan,dict(policy='explicit-inherited-reference-to-reserved-endpoint-v1',
        initialReserveRad=existing.tolist(),wholePathReferenceReserveFloorRad=floor.tolist(),
        endpointRequiredReserveRad=required.tolist(),genericWholePath06GuardUnchanged=True,
        nativePhysicalGatesUnchanged=True)


class PostureController(ReviewedController):
    def install_reviewed(self, step_index, mode):
        super().install_reviewed(step_index,mode)
        self._posture_plan=None;self._posture_attempted=False;self._posture_receipt=None
        self._posture_overlap=None;self._posture_provisional=None
        self._posture_return=None;self._posture_history=deque();self._posture_admitted=False
        self.minimum_ankle_ready_rad=.035

    def _command_identity(self,t,phase):
        base=super()._command_identity(t,phase)
        return base+(None if self._posture_plan is None else self._posture_plan.candidate.key,)

    def _observe_support(self,t,phase):
        if self._posture_plan is None:
            return super()._observe_support(t,phase)
        if phase!='transfer' or self._posture_admitted:return
        now=self.w.l.lh_time(self.w.h)
        corners,present,clearance=actual_stance_patch(self.w,self.stance_side)
        stance=self.w.feet()[self.stance_side]['body']
        loaded=bool(self.w.arr(20,(17,))[stance]>=.8*self.w.mass*9.81)
        if not loaded:
            self._posture_history.clear();return
        risk,forecast=self._forecast_unload_risk(t,corners,clearance)
        self.robust_support['currentPostureUnloadForecast']=forecast
        history=self._posture_history
        if history and now-history[-1][0]>.0050001:history.clear()
        history.append((now,risk,present.copy(),loaded))
        while len(history)>1 and history[1][0]<now-.1-1e-10:history.popleft()
        if len(history)<21 or now-history[0][0]<.1-1e-9:return
        votes=np.bincount([h[1] for h in history if h[1]>=0],minlength=4)
        weak=int(np.argmax(votes));survivors=tuple(i for i in range(4) if i!=weak)
        if survivors!=self._posture_provisional or votes[weak]/len(history)<.8:return
        if not all(h[3] and all(h[2][list(survivors)]) for h in history):return
        # Original correction is already complete. Its geometry/barriers remain
        # provisional while new tasks acquire their own causal lift admission.
        witnesses=[];folder=self._posture_evidence_path.parent/f'posture-risk-step-{self._reviewed_step_index:02d}'
        folder.mkdir()
        for record in self._robust_payloads:
            if history[0][0]-1e-10<=record['sourceTimeS']<=now+1e-10:
                path=folder/f"source-{record['sourceStep']:06d}.native-payload.zlib"
                path.write_bytes(record['compressed'])
                witnesses.append(dict(sourceStep=record['sourceStep'],sourceTimeS=record['sourceTimeS'],
                    payloadSHA256=record['payloadSHA256'],payloadBytes=record['payloadBytes'],
                    compressedSHA256=sha(path),filename=path.name))
        if len(witnesses)!=len(history) or len({w['sourceStep'] for w in witnesses})!=len(history):
            raise ValueError('posture fresh history lacks unique exact native witnesses')
        admission=dict(schema=1,profile=PROFILE,historySourceStartS=history[0][0],historySourceEndS=now,
            historySamples=len(history),historyPredictedRiskCorners=[h[1] for h in history],
            riskAgreementFrequency=float(votes[weak]/len(history)),survivingCorners=list(survivors),
            allSurvivorsActuallySupportedThroughoutHistory=True,candidateKey=self._posture_plan.candidate.key,
            originalAdmissionInherited=False,originalCorrectionCompleteBeforeActivation=True,
            selectedAtTransferS=t,modelSHA256=sha(self.w.xml),helperSHA256=sha(self.w.helper),
            bridgeSHA256=sha(self.w.base/'source_bridge_v2.so'),
            engineSHA256=sha(self.w.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'),
            physicsIntegrations=0,samples=witnesses)
        (folder/'manifest.json').write_text(json.dumps(admission,indent=2)+'\n')
        self._posture_admitted=True
        self.robust_support['postureFreshAdmission']=admission

    def admit_robust_reference(self,t,delta,duration):
        # This is solely the unchanged, independently qualified early baseline
        # correction. New posture admission has its own complete fresh window.
        if self._posture_plan is not None:
            raise ValueError('frozen posture candidate cannot be retargeted by a new correction')
        return super().admit_robust_reference(t,delta,duration)

    def corrected_com_reference(self,t,phase):
        if self._posture_plan is not None and phase=='settle' and hasattr(self,'_com_path'):
            now=self.w.l.lh_time(self.w.h)
            if not hasattr(self,'_com_settle'):
                start=now-t;p,v,a=self._com_path.evaluate(start)
                initial=(self.com1+self._robust_delta+p,v,a)
                final=(self.finalCom.copy(),np.zeros(3),np.zeros(3))
                legacy=QuinticSegment.between(start,self.phase_duration('settle'),initial,final)
                n=self.w.l.lh_support_count(self.w.h)
                points=self.w.arr(21,(128,))[:2*n].reshape(-1,2).copy()
                self._com_settle,self._bounded_settle_receipt=build_bounded_settle(start,
                    self.phase_duration('settle'),initial,final,points,legacy=legacy)
                self._bounded_settle_receipt.update(plannedAtNativeStep=self.w.l.lh_dim(self.w.h,6),
                    plannedAtSourceTimeS=now,modelSHA256=sha(self.w.xml),helperSHA256=sha(self.w.helper))
                path=self._posture_evidence_path.with_name(f'posture-settle-step-{self._reviewed_step_index:02d}.json')
                path.write_text(json.dumps(self._bounded_settle_receipt,indent=2)+'\n')
                self._bounded_settle_receipt_sha=sha(path)
            jet=self._com_settle.evaluate(now)
            self._com_reference_diagnostic=validate_reference(self,phase,jet)
            return jet
        if self._posture_plan is None or phase!='transfer':
            return super().corrected_com_reference(t,phase)
        p,v,a=self._posture_plan.com.at(self.w.l.lh_time(self.w.h)).arrays()
        # The completed old correction is embedded in the initial jet. It is
        # not added again; its unchanged barriers retain their complete clock.
        self._robust_clock=(1.,0.,0.)
        return p,v,a

    def robust_support_ready(self,row):
        return (self._posture_plan is not None and self._posture_admitted and
            self.w.l.lh_time(self.w.h)>=self._posture_plan.com.start+DURATION-1e-10
            and super().robust_support_ready(row))

    def transform_reviewed_reference_context(self,context):
        if (self._posture_plan is None and context.phase=='transfer'
                and context.elapsed>=self.phase_duration('transfer')-1e-10):
            if self._posture_attempted:raise ValueError('posture candidate rejected; no automatic replanning')
            if not self.robust_support.get('selected') or self._robust_clock[0]<1.-1e-12:
                return context
            corners,present,clearance=actual_stance_patch(self.w,self.stance_side)
            survivors=tuple(self._robust_survivors)
            if not all(present[list(survivors)]):return context
            self._posture_attempted=True
            result=retarget_current_reference(self,context,SEED,provisional_survivors=survivors)
            self._posture_receipt=result.receipt()
            self._posture_receipt['originalAdmissionInheritedForPosture']=False
            self._posture_evidence_path.write_text(json.dumps(self._posture_receipt,indent=2)+'\n')
            if not result.accepted:
                raise ValueError('single posture family rejected: '+str(self._posture_receipt['rejectionReasons']))
            initial=initial_coupled_reference(result)
            self._posture_plan,floor=inherited_plan(context.source_time,initial,result.candidate,self.w.limits)
            self._posture_initial=initial
            self._posture_receipt['inheritedTransition']=floor
            self._posture_receipt['frozenPlan']=self._posture_plan.receipt()
            self._posture_evidence_path.write_text(json.dumps(self._posture_receipt,indent=2)+'\n')
            self._posture_provisional=survivors;self._posture_transfer_start=context.elapsed
            self._posture_baseline_delta=self._robust_delta.copy()
            self.com1=np.array(result.candidate.target.com.position)-self._posture_baseline_delta
            self._posture_overlap=certify_overlap(np.array(result.candidate.target.com.position)-
                np.array(initial.com.position),DURATION,np.zeros(3),0.,1.)
            if not self._posture_overlap['passed']:raise ValueError('posture COM envelope rejected')
            if self._posture_history or self._posture_admitted:
                raise ValueError('posture activation inherited old-reference risk history')
        if self._posture_plan is None:return context
        sample=self._posture_plan.at(context.source_time)
        if context.phase=='settle' and self._posture_return is None:
            start=context.source_time-context.elapsed
            T=self.phase_duration('settle')
            lower=C2Segment.between(start,T,sample.lower_joints,Jet.rest(self.q0[:15]))
            # The inherited handoff has just updated q0 from measured joints.
            # Freeze that SAME return goal for both joints and pelvis FK.
            # Fixed-world-axis SO(3) C2 arcs carry zero endpoint angular jets.
            model=ModelFK.from_model(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))
            root=pose_jet_from_source(Jet.rest(np.zeros(3)),self.rootR,np.zeros(3),np.zeros(3))
            pelvis=pose_jet_to_source(model.forward(Jet.rest(self.q0),root)['pelvis'])
            targets=(self.rootR,np.array(pelvis.rotation).reshape(3,3))
            arcs=[]
            for row,target in zip((sample.thorax,sample.pelvis),targets):
                R=row.arrays()[0]
                axis=Rotation.from_matrix(target@R.T).as_rotvec()
                arcs.append(dict(body=row.body,initialRotation=R.tolist(),axis=axis.tolist()))
            self._posture_return=dict(start=start,duration=T,lower=lower,arcs=arcs)
        if context.phase in ('settle','stand') and self._posture_return is not None:
            ret=self._posture_return; r,rd,rdd=clock(context.source_time-ret['start'],ret['duration'])
            rows=[]
            for arc in ret['arcs']:
                axis=np.array(arc['axis']); R=Rotation.from_rotvec(axis*r).as_matrix()@np.array(arc['initialRotation'])
                rows.append(OrientationJet(arc['body'],R.reshape(-1),axis*rd,axis*rdd))
            sample=CoupledSample(sample.com,ret['lower'].at(context.source_time),*rows)
        # Complete COM already includes the admitted robust correction and later
        # continuous/settle handoffs. Never overwrite it with the posture-only jet.
        complete=self.corrected_com_reference(context.elapsed,context.phase)
        diagnostic=validate_reference(self,context.phase,complete)
        corners,present,_=actual_stance_patch(self.w,self.stance_side)
        n=self.w.l.lh_support_count(self.w.h)
        support=CurrentSupport(context.source_step,context.source_time,context.step_index,context.phase,
            tuple(map(tuple,self.w.arr(21,(128,))[:2*n].reshape(-1,2))),tuple(map(tuple,corners)),
            tuple(bool(x) for x in present),self._posture_provisional,
            bool(self.robust_support.get('selected') and context.phase in ('transfer','lift','translate','land')))
        diagnostic.update(support.validate_com(Jet(*complete)))
        lower=sample.lower_joints.arrays();old=context.joints.arrays()
        joints=ContextJet.make(*(np.r_[a,b[15:]] for a,b in zip(lower,old)))
        s,_,_=clock(context.source_time-self._posture_plan.com.start,DURATION)
        orientations=(tuple(o for o in context.orientations if o.body not in (1,4))+
            tuple(Orientation.make(o.body,*o.arrays(),weight=weight) for o,weight in
                ((sample.thorax,4.),(sample.pelvis,4.*s))))
        config=json.loads(context.config_json)
        # Activation changes the ready/projection endpoint before observation;
        # bind that live field even in the first already-built draft context.
        config['com1']=self.com1.tolist()
        config['coupledPosture']=dict(profile=PROFILE,plan=self._posture_plan.receipt(),
            frozenCandidateInputSHA256=self._posture_receipt['inputSHA256'],
            provisionalSurvivors=list(self._posture_provisional),pelvisWeight=4.*s,
            completedBaselineCorrectionM=self._posture_baseline_delta.tolist(),
            originalAdmissionInheritedForPosture=False,
            inheritedTransition=self._posture_receipt['inheritedTransition'],
            originalRestReturn=None if self._posture_return is None else
                dict(start=self._posture_return['start'],duration=self._posture_return['duration'],
                    lower=asdict(self._posture_return['lower']),arcs=self._posture_return['arcs']))
        causal=json.loads(context.causal_json);causal['coupledPostureOverlap']=self._posture_overlap
        if hasattr(self,'_bounded_settle_receipt'):
            config['boundedSettle']=dict(version=self._bounded_settle_receipt['version'],
                profile=self._bounded_settle_receipt['profile'],
                fullCertificateSHA256=self._bounded_settle_receipt_sha,
                currentActualHullOnly=True,liveValidationEveryCommand=True,
                durationUnchanged=True,measuredTargetUnchanged=True,frozenCapsUnchanged=True)
        diagnostics=json.loads(context.diagnostics_json);diagnostics['coupledPosture']=dict(
            **diagnostic,sampleSHA256=sample.fingerprint(),softTasks=True,delayedArmsUnchanged=True,
            lowerJointReferenceOnly=True)
        return replace(context,com=ContextJet.make(*complete),joints=joints,orientations=orientations,
            config_json=canonical(config),causal_json=canonical(causal),diagnostics_json=canonical(diagnostics))

    def solve(self,t,phase='transfer'):
        u,info=super().solve(t,phase)
        info['reviewedWalk']['featureProfile']=PROFILE
        info['coupledPosture']=dict(enabled=self._posture_plan is not None,
            candidateKey=None if self._posture_plan is None else self._posture_plan.candidate.key,
            provisionalSurvivors=self._posture_provisional,overlap=self._posture_overlap,
            freshPostureAdmission=self._posture_admitted)
        if self._posture_plan is not None:
            response=self.w.read('lh_motor_response',(33,),ptr(np.ascontiguousarray(u)))
            native_v=self.w.arr(1,(27,));objectives=[]
            for row in self._prepared_reference_context.orientations:
                if row.body not in (1,4):continue
                R=self.w.arr(5,(17,3,3))[row.body]
                J=self.w.read('lh_body_jac',(6,27),row.body)[3:]
                bias=self.w.read('lh_jdot',(6,),row.body)[3:]
                reference=OrientationJet(row.body,row.rotation,row.velocity,row.acceleration)
                objectives.append(dict(body=row.body,weight=row.weight,
                    measuredHeadingPitchRollRad=chart(R).tolist(),
                    referenceHeadingPitchRollRad=chart(reference.arrays()[0]).tolist(),
                    **orientation_residual(reference,R,J@native_v,kp=row.kp,kd=row.kd,
                        achieved_angular_acceleration=J@response[:27]+bias)))
            info['coupledPosture']['softOrientationObjectives']=objectives
        return u,info


class PostureSteps(ReviewedSteps):
    reviewed_controller_class=PostureController
    def __init__(self,base,helper,output,count=1):
        super().__init__(base,helper,output,count,'continuous-com-causal')
        self.code_at_start.update({name:sha(Path(__file__).with_name(name)) for name in RUNTIME})
    def begin_step(self):
        super().begin_step()
        self.controller._posture_evidence_path=self.folder/f'posture-static-step-{self.step_index+1:02d}.json'
    def save(self,reason):
        super().save(reason)
        if not hasattr(self,'folder'):return
        for name in ('reviewed-walk-run.json','reviewed-walk-config.json'):
            path=self.folder/name
            if not path.exists():continue
            value=json.loads(path.read_text());value.update(profile=PROFILE,
                coupledPostureEnabled=True,finalPlanQualificationClaim=False,
                oldProfileQualificationClaim=False,
                minimumPostureAnkleReadyRad=.035,
                inheritedPhysicalGatesUnchanged=True)
            path.write_text(json.dumps(value,indent=2)+'\n')
        path=self.folder/'cadenced-step-config.json'
        if path.exists():
            value=json.loads(path.read_text())
            value['reviewedReferenceProfile']['featureProfile']=PROFILE
            path.write_text(json.dumps(value,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--inspector',type=Path,required=True)
    p.add_argument('--count',type=int,choices=(1,2),default=1);a=p.parse_args()
    session,helper,preparation=prepare_impedance(a.output,a.inspector,8.)
    shutil.copyfile(a.inspector,a.output/'contact-inspector.so')
    live=PostureSteps(session,helper,a.output/'run',a.count);live.paused=False
    started=time.monotonic()
    try:
        while not live.finished and not live.failure:
            if time.monotonic()-started>=1200.:raise RuntimeError('1200s process cap')
            live.advance(4)
            if live.steps%800==0:print('progress',live.steps*.00125,'step',live.step_index+1,
                live.phase,'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
        print(live.failure or 'posture-physical-pilot-plus5squiet',flush=True)
    except Exception as error:
        live.failure=repr(error);raise
    finally:live.close()
    raise SystemExit(0 if live.finished else 1)

if __name__=='__main__':main()
