#!/usr/bin/env python3
"""Explicit impratio8 gait with causal weak-corner support admission.

The corroborated prospective-unload pressure/clearance corner is the risk case.
All three survivors must belong to actual measured support. A bounded C2 COM
reference shift and predictive triangle barriers precede lift admission. The
every-native-frame capture/joint/motor guards remain active; cumulative slip
uses the explicit current walking-slip policy.
This is not robustness to arbitrary corner loss: that has no positive-margin
solution for a rectangular footprint.
"""
import argparse
import copy
import ctypes as C
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import zlib

import numpy as np
from scipy.spatial import ConvexHull,QhullError

from source_step_probe import Controller,PTR,clock,ptr,sha
from source_cadenced_ten_steps import CadencedController,CadencedSteps
from source_contact_impedance_probe import prepare_impedance
from source_arm_knee_mobility import native_bytes

RESERVE=.003
MATCH_TOLERANCE=.001
MAX_SHIFT=.012


def project_inset(point,equations,reserve):
    """Exact nearest point in a bounded 2D half-plane intersection."""
    point=np.asarray(point,dtype=float);A=equations[:,:2];b=-reserve-equations[:,2]
    feasible=lambda x:np.all(A@x<=b+1e-12)
    candidates=[point]if feasible(point)else[]
    for a,limit in zip(A,b):
        x=point-a*((a@point-limit)/(a@a))
        if feasible(x):candidates.append(x)
    for i in range(len(A)):
        for j in range(i):
            pair=A[[i,j]]
            if abs(np.linalg.det(pair))>1e-12:
                x=np.linalg.solve(pair,b[[i,j]])
                if feasible(x):candidates.append(x)
    if not candidates:raise ValueError('empty robust support inset')
    return min(candidates,key=lambda x:float(np.sum((x-point)**2))).copy()


def actual_stance_patch(w,side):
    foot=w.feet()[side]
    corners=np.array(foot['points']).copy();corners[:,2]-=foot['radius']
    count=w.l.lh_support_count(w.h)
    supports=w.arr(21,(128,))[:2*count].reshape(-1,2)
    present=np.array([bool(len(supports)and np.min(np.linalg.norm(supports-p[:2],axis=1))<=MATCH_TOLERANCE)for p in corners])
    clearances=np.array([p[2]-w.ground_height(p)for p in corners])
    # Keep geometric corner IDs for history, but use the distinct ACTUAL
    # measured material support coordinates for all conservative polygons.
    matched=corners.copy();indices=[]
    for i,p in enumerate(corners):
        if present[i]:
            index=int(np.argmin(np.linalg.norm(supports-p[:2],axis=1)))
            matched[i,:2]=supports[index];indices.append(index)
        else:indices.append(-1)
    for i in range(len(indices)):
        if indices[i]>=0 and indices.count(indices[i])>1:present[i]=False
    return matched,present,clearances


class RobustSupportController(CadencedController):
    def _forecast_unload_risk(self,t,corners,clearance):
        """Causal proposed-unload solve, never an applied control or rollout.

        Virtual normal-pressure ranking is admitted only when a pure native
        acceleration response independently predicts the same corner's largest
        100ms upward clearance. This horizon is the original unload admission
        window. Only CURRENT actual points enter the planning polygon.
        """
        before=native_bytes(self.w);counter=self.w.l.lh_dim(self.w.h,6)
        probe=copy.copy(self);probe.__class__=CadencedController
        probe.unload_admitted=True;probe.floorTargets=list(self.floorTargets)
        probe.floorTargets[self.swing_side]=self.w.ground_height(self.targets[self.swing_side][0])+self.radii[self.swing_side]+.0001
        fields=('q0','com0','com1','targets','rootR','flatRotations','radii','floorTargets','swing_side',
            'stance_side','swing_body','self_avoidance','B','heading','landing','step_distance','finalCom',
            'lean_axis','phase_durations','joint_reserve','posture_start','shear_weight','trajectory_time_scale',
            'arm_posture','arm_velocity','arm_acceleration','arm_kp','arm_kd','arm_weight','leg_origin',
            'unload_admitted','_seek_time','_seek_depth','_seek_rate','_seek_jet')
        probe_input=json.loads(json.dumps(dict(fields={name:getattr(probe,name)for name in fields if hasattr(probe,name)},
            armMappingRanges=probe._mapping.ranges,elapsedTransferS=t,phase='transfer',
            coreEntryPoint='Controller.solve on copied CadencedController; existing arm targets already computed'),
            default=lambda value:value.tolist()))
        # Arm targets were computed once by this actual outer solve. Bypass
        # OpposedArmController.solve here so its derivative history is untouched.
        u,info=Controller.solve(probe,t,'transfer')
        forces=np.array(info['predictedForcesN']).reshape(2,4,3)[self.stance_side,:,2]
        w=self.w;w.l.lh_motor_response.argtypes=[C.c_void_p,PTR,PTR];w.l.lh_motor_response.restype=None
        response=w.read('lh_motor_response',(33,),ptr(np.ascontiguousarray(u)))
        foot=w.feet()[self.stance_side];body=foot['body'];v=w.arr(1,(27,))
        Jbody=w.read('lh_body_jac',(6,27),body);jet=w.read('lh_jdot',(6,),body)
        omega=Jbody[3:]@v;origin=w.arr(3,(17,3))[body]
        rates=[];accelerations=[]
        for point in corners:
            J=w.read('lh_point_jac',(3,27),body,ptr(np.ascontiguousarray(point)))
            r=point-origin;bias=jet[:3]+np.cross(jet[3:],r)+np.cross(omega,np.cross(omega,r))
            rates.append(float(J[2]@v));accelerations.append(float(J[2]@response[:27]+bias[2]))
        horizon=.1;forecast=clearance+horizon*np.array(rates)+.5*horizon**2*np.array(accelerations)
        pressure_risk=int(np.argmin(forces));clearance_risk=int(np.argmax(forecast))
        assert native_bytes(w)==before and w.l.lh_dim(w.h,6)==counter,'unload forecast changed native state'
        digest=hashlib.sha256(before).hexdigest()
        if not hasattr(self,'_robust_payloads'):self._robust_payloads=deque(maxlen=22)
        self._robust_payloads.append(dict(sourceStep=counter,sourceTimeS=w.l.lh_time(w.h),
            payloadSHA256=digest,payloadBytes=len(before),compressed=zlib.compress(before,1)))
        telemetry=dict(virtualUnloadNormalForcesN=forces.tolist(),nativeUnloadEndpointVerticalAccelerationMps2=accelerations,
            nativeUnloadEndpointVerticalVelocityMps=rates,localClearanceForecastM=forecast.tolist(),predictionHorizonS=horizon,
            pressureRiskCorner=pressure_risk,clearanceRiskCorner=clearance_risk,
            proposedMotorControl=u.tolist(),nativeProposedResponse=response.tolist(),
            probeControllerInput=probe_input,
            nativePayloadSHA256=digest,nativePayloadSourceStep=counter,
            currentMatchedSupportCoordinatesSource=corners.tolist(),currentEndpointClearanceM=clearance.tolist(),
            proposedMotorMaximumFraction=float(max(abs(u))),proposedControlApplied=False,physicsRolloutIntegrations=0,
            nativeBytesAndCounterUnchanged=True)
        return pressure_risk if pressure_risk==clearance_risk else -1,telemetry

    def _observe_support(self,t,phase):
        if not hasattr(self,'_robust_history'):
            self._robust_history=deque();self.robust_support=dict(selected=False,causalHistoryOnly=True)
        if phase!='transfer' or self.robust_support['selected']:return
        now=self.w.l.lh_time(self.w.h)
        corners,present,clearance=actual_stance_patch(self.w,self.stance_side)
        loads=self.w.arr(20,(17,));stance_body=self.w.feet()[self.stance_side]['body']
        loaded=bool(loads[stance_body]>=.8*self.w.mass*9.81)
        if not loaded:
            self._robust_history.clear()
            return
        state_only=bool(getattr(self,'_state_only_support_selection',False))
        if not state_only and t<self.phase_duration('transfer')-.1-1e-10:return
        risk,forecast=self._forecast_unload_risk(t,corners,clearance)
        self.robust_support['currentCausalUnloadForecast']=forecast
        if self._robust_history and now-self._robust_history[-1][0]>.0050001:self._robust_history.clear()
        self._robust_history.append((now,risk,present.copy(),loaded))
        while len(self._robust_history)>1 and self._robust_history[1][0]<now-.1-1e-10:self._robust_history.popleft()
        # Wait for nominal transfer, then use only the preceding measured
        # window. The extra movement is admitted by state, not a fixed lift cut.
        if (not state_only and t<self.phase_duration('transfer'))or len(self._robust_history)<21 or now-self._robust_history[0][0]<.1-1e-9:return
        history=list(self._robust_history);votes=np.bincount([h[1]for h in history if h[1]>=0],minlength=4)
        weak=int(np.argmax(votes));survivors=[i for i in range(4)if i!=weak]
        if votes[weak]/len(history)<.8 or not all(h[3]and all(h[2][survivors])for h in history):return
        equations=ConvexHull(corners[survivors,:2]).equations
        # The 1mm endpoint association tolerance is also the reference cushion
        # above the separately checked 3mm measured admission reserve.
        goal=project_inset(self.com1[:2],equations,RESERVE+MATCH_TOLERANCE)
        delta=np.r_[goal-self.com1[:2],0.];distance=float(np.linalg.norm(delta))
        if distance>MAX_SHIFT:
            self.robust_support.update(lastRejectedWeakCorner=weak,lastRejectedCorrectionM=distance,
                lastAdmissionRejection='bounded correction unavailable; continue observing current support')
            return
        duration=max(1.,1.875*distance/.025,np.sqrt(5.78*distance/.15))
        # An opt-in controller may reject a proposed overlap before any support
        # selection/correction is committed. Legacy observers have no hook.
        if hasattr(self,'admit_robust_reference') and not self.admit_robust_reference(t,delta,duration):return
        com=self.w.arr(6,(17,3))[1];J=self.w.read('lh_com_jac',(3,27));v=self.w.arr(1,(27,));cp=com[:2]+np.sqrt(com[2]/9.81)*(J@v)[:2]
        initial_margin=float(min(np.min(-(equations[:,:2]@com[:2]+equations[:,2])),np.min(-(equations[:,:2]@cp+equations[:,2]))))
        self._robust_delta=delta;self._robust_start=t;self._robust_duration=duration
        self._robust_weak=weak;self._robust_survivors=survivors;self._robust_initial_reserve=min(initial_margin-.001,RESERVE)
        witnesses=[]
        if hasattr(self,'_robust_evidence_folder'):
            folder=self._robust_evidence_folder;folder.mkdir()
            for record in self._robust_payloads:
                if history[0][0]-1e-10<=record['sourceTimeS']<=now+1e-10:
                    path=folder/f"source-{record['sourceStep']:06d}.native-payload.zlib"
                    path.write_bytes(record['compressed'])
                    witnesses.append(dict(sourceStep=record['sourceStep'],sourceTimeS=record['sourceTimeS'],
                        payloadSHA256=record['payloadSHA256'],payloadBytes=record['payloadBytes'],
                        compressedSHA256=sha(path),filename=path.name))
            (folder/'manifest.json').write_text(json.dumps(dict(schema=1,modelSHA256=sha(self.w.xml),
                helperSHA256=sha(self.w.helper),bridgeSHA256=sha(self.w.base/'source_bridge_v2.so'),
                engineSHA256=sha(self.w.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'),
                stepIndex=self._robust_step_index,phase='transfer',stanceSide=self.stance_side,
                selectedAtTransferS=t,payloadEncoding='zlib-compressed exact lh_checkpoint_export payload',
                originalNativeWorldUnchanged=True,physicsIntegrations=0,samples=witnesses),indent=2)+'\n')
        self.robust_support=dict(selected=True,causalHistoryOnly=True,weakCorner=weak,survivingCorners=survivors,
            supportPatchConstruction='three-unique-nearest-actual-support-xy',
            riskSignal='planned-unload-QP-minimum-normal-pressure-and-native-100ms-clearance-agreement',
            selectedCausalUnloadForecast=forecast,
            retainedExactNativeForecastWitnesses=len(witnesses),
            forecastHistoryNativeSteps=[record['sourceStep']for record in witnesses],
            historyPredictedRiskCorners=[h[1]for h in history],
            historySourceStartS=history[0][0],historySourceEndS=now,historySamples=len(history),
            riskAgreementFrequency=float(votes[weak]/len(history)),allSurvivorsActuallySupportedThroughoutHistory=True,
            selectedAtTransferS=t,comCorrectionM=delta.tolist(),correctionMagnitudeM=distance,maximumCorrectionM=MAX_SHIFT,
            correctionDurationS=duration,maximumReferenceSpeedMps=.025,maximumReferenceAccelerationMps2=.15,
            admissionReserveM=RESERVE,referenceReserveM=RESERVE+MATCH_TOLERANCE,
            endpointSupportAssociationToleranceM=MATCH_TOLERANCE,arbitraryCornerLossRobustnessClaim=False)

    def adjust_com_reference(self,t,phase,target,velocity,acceleration):
        self._observe_support(t,phase)
        if not self.robust_support['selected']:return target,velocity,acceleration
        if phase=='transfer':s,ds,dds=clock(t-self._robust_start,self._robust_duration)
        elif phase=='settle':
            s,ds,dds=clock(t,self.phase_duration('settle'));s,ds,dds=1.-s,-ds,-dds
        elif phase=='stand':s,ds,dds=0.,0.,0.
        else:s,ds,dds=1.,0.,0.
        self._robust_clock=(s,ds,dds)
        return target+s*self._robust_delta,velocity+ds*self._robust_delta,acceleration+dds*self._robust_delta

    def transfer_ready_goal(self):
        return self.com1+getattr(self,'_robust_delta',np.zeros(3))

    def robust_support_ready(self,row):
        if not getattr(self,'robust_support',{}).get('selected',False):return False
        if self._robust_clock[0]<1.:return False
        try:
            corners,present,_=actual_stance_patch(self.w,self.stance_side)
            if not all(present[self._robust_survivors]):return False
            eq=ConvexHull(corners[self._robust_survivors,:2]).equations
            com=np.array(row['com']);cp=com[:2]+np.sqrt(com[2]/9.81)*np.array(row['comVelocity'])[:2]
            return bool(min(np.min(-(eq[:,:2]@com[:2]+eq[:,2])),np.min(-(eq[:,:2]@cp+eq[:,2])))>=RESERVE)
        except (QhullError,ValueError):return False

    def add_capture_constraints(self,phase,width,Jcom,com,v,inequalities,bounds):
        super().add_capture_constraints(phase,width,Jcom,com,v,inequalities,bounds)
        self.robust_support_barrier=[]
        if not getattr(self,'robust_support',{}).get('selected',False)or phase not in ('transfer','lift','translate','land'):return
        corners,present,_=actual_stance_patch(self.w,self.stance_side)
        # No unseen or disappeared corner is installed as virtual support.
        if not all(present[self._robust_survivors]):return
        equations=ConvexHull(corners[self._robust_survivors,:2]).equations
        velocity=Jcom@v;tau=np.sqrt(com[2]/9.81);tau_rate=velocity[2]/(2.*9.81*tau)
        cp=com[:2]+tau*velocity[:2];bias=self.w.read('lh_jdot',(3,),-1);gain=8.
        s,ds,dds=self._robust_clock if phase=='transfer'else(1.,0.,0.)
        delta_reserve=RESERVE-self._robust_initial_reserve
        reserve=self._robust_initial_reserve+s*delta_reserve;rdot=ds*delta_reserve;rddot=dds*delta_reserve
        for face in equations:
            normal,offset=face[:2],face[2]
            cp_margin=-float(normal@cp+offset);com_margin=-float(normal@com[:2]+offset)
            row=np.zeros(width);row[:27]=tau*normal@Jcom[:2];inequalities.append(row)
            bounds.append(gain*(cp_margin-reserve)-rdot-normal@((1.+tau_rate)*velocity[:2]+tau*bias[:2]))
            row=np.zeros(width);row[:27]=normal@Jcom[:2];inequalities.append(row)
            bounds.append(gain**2*(com_margin-reserve)-2.*gain*normal@velocity[:2]-normal@bias[:2]-2.*gain*rdot-rddot)
            self.robust_support_barrier.append(dict(captureMarginM=cp_margin,comMarginM=com_margin,reserveM=reserve,
                reserveRateMps=rdot,reserveAccelerationMps2=rddot,actualSurvivingSupportOnly=True,gainPerS=gain))

    def solve(self,t,phase='transfer'):
        u,info=super().solve(t,phase)
        info['robustSupportAdmission']=dict(self.robust_support,barriers=self.robust_support_barrier,
            correctionClock=getattr(self,'_robust_clock',(0.,0.,0.)))
        return u,info


class RobustSupportSteps(CadencedSteps):
    def __init__(self,base,helper,output,count=10):
        self.contact_impratio=8.
        super().__init__(base,helper,output,count,.18,'loaded-settle',False)
        self.code_at_start[Path(__file__).name]=sha(__file__)
        self.code_at_start['source_contact_impedance_probe.py']=sha(Path(__file__).with_name('source_contact_impedance_probe.py'))

    def begin_step(self):
        super().begin_step();self.controller.__class__=RobustSupportController
        self.controller._robust_evidence_folder=self.folder/f'robust-risk-step-{self.step_index+1:02d}'
        self.controller._robust_step_index=self.step_index+1

    def save(self,reason):
        super().save(reason)
        if hasattr(self,'folder'):
            path=self.folder/'cadenced-step-config.json';config=json.loads(path.read_text())
            config['contactImpedanceExperiment']=dict(impratio=8.,baselineImpratio=1.,muChanged=False,originalModelQualificationClaim=False,separatelyModelBoundColdCache=True)
            config['robustSupportAdmission']=dict(enabled=True,causalWeakCornerHistoryS=.1,minimumFrequency=.8,
                supportPatchConstruction='three-unique-nearest-actual-support-xy',
                riskSignal='planned-unload-QP-minimum-normal-pressure-and-native-100ms-clearance-agreement',
                forecastHorizonS=.1,forecastControlApplied=False,forecastRolloutIntegrations=0,
                actualSurvivorsRequired=True,maximumComCorrectionM=MAX_SHIFT,measuredAdmissionReserveM=RESERVE,
                allFrameOriginalCaptureGuardUnchanged=True,arbitraryCornerLossRobustnessClaim=False)
            path.write_text(json.dumps(config,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--inspector',type=Path,required=True);p.add_argument('--count',type=int,choices=(1,10),default=10)
    a=p.parse_args();session,helper,_=prepare_impedance(a.output,a.inspector,8.)
    live=RobustSupportSteps(session,helper,a.output/'run',a.count);live.paused=False;started=time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started<1200.,'1200s process cap';live.advance(4)
            if live.steps%800==0:print('progress',live.steps*.00125,'step',live.step_index+1,live.phase,'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
        print(live.failure or 'robust-support-steps-plus5squiet',flush=True)
    except Exception as error:live.failure=repr(error);raise
    finally:live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__=='__main__':main()
