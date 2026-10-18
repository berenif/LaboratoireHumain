#!/usr/bin/env python3
"""Opt-in faster physical alternating steps; original measured safety gates.

Durations are native source seconds, not playback scaling. Ten steps share one
world and one loaded-slip accumulator, followed by the original final 5s quiet.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull,QhullError
from scipy.spatial.transform import Rotation

from source_step_probe import sha
from source_selected_step import prepare
from source_upright_large_steps import LargeStep,ContactSeekingController

PHASE_DURATIONS = dict(transfer=3., lift=.8, translate=1.2, land=1., settle=1.8)


def thigh_pitch_jet(w,side):
    """Read-only torso-relative thigh angle, generalized Jacobian and Jdot·v."""
    positions=w.arr(3,(17,3));R=w.arr(5,(17,3,3))[1]
    velocity=w.arr(1,(27,))
    a,b=[w.l.lh_id(w.h,1,name.encode())for name in (side+'_thigh',side+'_shin')]
    Ja=w.read('lh_body_jac',(6,27),a)[:3];Jb=w.read('lh_body_jac',(6,27),b)[:3]
    Jr=w.read('lh_body_jac',(6,27),1)[3:]
    ba=w.read('lh_jdot',(6,),a)[:3];bb=w.read('lh_jdot',(6,),b)[:3]
    br=w.read('lh_jdot',(6,),1)[3:]
    d=R.T@(positions[b]-positions[a]);omega=R.T@(Jr@velocity)
    relative_world=Jb-Ja
    D=R.T@relative_world+np.cross(d[:,None],R.T@Jr,axis=0)
    rate=D@velocity
    bias=R.T@(bb-ba)-2.*np.cross(omega,R.T@(relative_world@velocity))+np.cross(d,R.T@br)+np.cross(omega,np.cross(omega,d))
    x,z=d[0],d[2];r2=x*x+z*z
    gradient=np.array([-z,0.,x])/r2
    H=np.array([[2.*x*z,0.,z*z-x*x],[0.,0.,0.],[z*z-x*x,0.,-2.*x*z]])/r2**2
    return float(np.arctan2(x,-z)),gradient@D,float(gradient@bias+rate@H@rate)


class CadencedController(ContactSeekingController):
    def add_capture_constraints(self,phase,width,Jcom,com,v,inequalities,bounds):
        self.arm_range_barrier=[]
        for index,side in enumerate(('right','left')):
            angle,J,bias=thigh_pitch_jet(self.w,side)
            rate=float(J@v);horizon=.2;reserve=.02
            lower=self.leg_origin[index]-self._mapping.ranges[index,1]+reserve
            upper=self.leg_origin[index]-self._mapping.ranges[index,0]-reserve
            row=np.zeros(width);row[:27]=J
            inequalities.extend([row,-row])
            bounds.extend([2.*(upper-angle-horizon*rate)/horizon**2-bias,
                           2.*(angle-lower+horizon*rate)/horizon**2+bias])
            self.arm_range_barrier.append(dict(side=side,measuredThighPitchRad=angle,
                measuredPitchRateRadps=rate,minimumReservedMarginRad=min(angle-lower,upper-angle),
                predictionHorizonS=horizon,verifiedArmRangeReserveRad=reserve,clippingEnabled=False))
        self.capture_barrier=[]
        if phase not in ('lift','translate','land'):
            return
        count=self.w.l.lh_support_count(self.w.h)
        points=self.w.arr(21,(128,))[:2*count].reshape(-1,2)
        try:
            equations=ConvexHull(np.unique(points,axis=0)).equations
        except (QhullError,ValueError):
            return  # The unchanged measured runtime guard rejects no support.
        velocity=Jcom@v
        tau=np.sqrt(com[2]/9.81)
        tau_rate=velocity[2]/(2.*9.81*tau)
        capture=com[:2]+tau*velocity[:2]
        bias=self.w.read('lh_jdot',(3,),-1)
        gain=8.;reserve=.003
        for face in equations:
            normal,offset=face[:2],face[2]
            cp_margin=-float(normal@capture+offset)
            row=np.zeros(width);row[:27]=tau*normal@Jcom[:2]
            inequalities.append(row)
            bounds.append(gain*(cp_margin-reserve)-normal@((1.+tau_rate)*velocity[:2]+tau*bias[:2]))
            position_margin=-float(normal@com[:2]+offset)
            row=np.zeros(width);row[:27]=normal@Jcom[:2]
            inequalities.append(row)
            bounds.append(gain**2*(position_margin-reserve)-2.*gain*normal@velocity[:2]-normal@bias[:2])
            self.capture_barrier.append(dict(captureMarginM=cp_margin,comMarginM=position_margin,
                reserveM=reserve,gainPerS=gain,measuredSupportOnly=True))

    def solve(self,t,phase='transfer'):
        u,info=super().solve(t,phase)
        info['captureBarrier']=self.capture_barrier
        info['opposedArmRangeBarrier']=self.arm_range_barrier
        return u,info


class CadencedSteps(LargeStep):
    def __init__(self, base, helper, output, count=10, distance=.18):
        self.sequence_integration_budget = 160000
        self.continuous_loaded_handoff = True
        super().__init__(base, helper, output, (distance,)*count, False)
        self.code_at_start[Path(__file__).name] = sha(__file__)

    def begin_step(self):
        previous=getattr(self,'controller',None)if self.step_index>0 else None
        super().begin_step()
        if previous is not None:
            # Preserve target C0/C1 and the measured-thigh differentiation
            # history at a loaded handoff. Native pose/velocity are untouched.
            self.controller.com0=previous.com1.copy()
            reference_R=Rotation.from_rotvec(self.controller.lean_axis).as_matrix()@self.controller.rootR
            self.controller.rootR=Rotation.from_rotvec(previous.lean_axis).as_matrix()@previous.rootR
            self.controller.lean_axis=Rotation.from_matrix(reference_R@self.controller.rootR.T).as_rotvec()
            self.controller.posture_start=previous.q0.copy()
            self.controller.previous_time=previous.previous_time
            self.controller.previous_leg_rate=previous.previous_leg_rate.copy()

        self.controller.phase_durations = PHASE_DURATIONS.copy()
        if self.step_index == 0:
            self.controller.phase_durations['transfer'] = 4.
        self.controller.__class__=CadencedController
        self.controller.arm_placement_duration = self.controller.phase_duration('transfer')
        record = {k:v for k,v in vars(self.controller).items() if k!='w' and not k.startswith('_')}
        (self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(
            json.dumps(record,default=lambda x:x.tolist())+'\n')
        self.step_records[-1]['phaseDurationsS'] = self.controller.phase_durations.copy()

    def save(self, reason):
        super().save(reason)
        if hasattr(self, 'folder'):
            (self.folder/'cadenced-step-config.json').write_text(json.dumps(dict(
                physicalTimeOnly=True, playbackSpeed=1.,
                requestedSteps=len(self.distances), requestedDistanceM=self.distances[0],
                phaseDurationsS=PHASE_DURATIONS, initialTransferS=4.,
                finalQuietRequiredS=5., intermediateStableRequiredS=0.,continuousLoadedHandoff=True,
                measuredSupportCaptureBarrier=dict(reserveM=.003,gainPerS=8.),
                opposedArmRangeBarrier=dict(verifiedRangeReserveRad=.02,predictionHorizonS=.2,clippingEnabled=False),
                globalLoadedSlipLimitM=.01, integrationBudget=self.sequence_integration_budget,
                originalContactJointMotorCollisionCaptureGates=True,
                noInterstepStateInstallation=True),indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--count', type=int, default=10)
    p.add_argument('--distance', type=float, default=.18)
    a=p.parse_args()
    assert 1<=a.count<=10 and .12<=a.distance<=.24
    session,helper,_=prepare(a.output)
    live=CadencedSteps(session,helper,a.output/'run',a.count,a.distance)
    live.paused=False; started=time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started<1200., '1200s process cap'
            live.advance(4)
            if live.steps%800==0:
                print('progress',live.steps*.00125,'step',live.step_index+1,live.phase,
                      'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
        print(live.failure or 'measured-cadenced-steps-plus5squiet',flush=True)
    except Exception as error:
        live.failure=repr(error)
        raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__=='__main__': main()
