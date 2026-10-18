#!/usr/bin/env python3
"""Opt-in large-step test with compliant upright torso/pelvis torque tasks.

The upright preference has a 1.5 degree deadband and a 12 degree tilt cone.
Only joint motor torques reach the physical world. All original contact, ray,
motor, joint, slip, capture and final five-second quiet checks are retained.
The larger geometric request/reach envelope is explicit and experimental.
"""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from source_step_probe import ROOT, sha, clock
from source_selected_step import prepare
from source_opposed_arm_swing import OpposedArmController, OpposedArmMultiStep, SagittalArmMap
from source_opposed_arm_swing import limb_telemetry
from source_arm_knee_mobility import KNEE_GOAL_RAD, native_bytes
from plan_source_arm_knee import plan_posture

DEADBAND_DEG = 1.5
TILT_LIMIT_DEG = 12.
INHERITED_ENTRY_LIMIT_END_S = .06375


class ExtendedArmMap(SagittalArmMap):
    def __init__(self,q,limits,goal):
        # Verified exact FK inversion with the original .02rad joint reserve.
        # Rearward limits differ because the shoulder axes are mirrored.
        super().__init__(q,limits,goal,ranges=[[-.6203251020812989,.80],[-.5292790834808349,.80]])


def body_orientation(rotation):
    """World-referenced pitch/roll, with yaw factored out (source z is up)."""
    yaw, pitch, roll = Rotation.from_matrix(rotation).as_euler('ZYX')
    return dict(pitchDeg=float(np.rad2deg(pitch)), rollDeg=float(np.rad2deg(roll)),
                yawDeg=float(np.rad2deg(yaw)),
                tiltDeg=float(np.rad2deg(np.arccos(np.clip(rotation[2,2], -1, 1)))))


def cone_jet(up, angular_jacobian, omega, angular_bias):
    """h=up.z-cos(limit); exact hdot and acceleration coefficients."""
    row = np.cross(up, np.array([0.,0.,1.])) @ angular_jacobian
    derivative = float(np.cross(omega, up)[2])
    bias = float((np.cross(angular_bias, up)+np.cross(omega,np.cross(omega,up)))[2])
    return row, derivative, bias


class ContactSeekingController(OpposedArmController):
    """Small bounded downward target correction, governed by measured contact.

    A geometrically near-ground foot can remain unloaded because the torque QP
    trades a millimetre of foot tracking against other tasks. This reference
    search starts only after the nominal descent, never creates virtual load,
    and retains the original landing deadline and all physical gates.
    """
    SEEK_MAX_M = .003
    SEEK_SPEED_MPS = .003
    SEEK_ACCEL_MPS2 = .03
    SEEK_LOAD_N = 12.

    def landing_adjustment(self, t, phase):
        now = self.w.l.lh_time(self.w.h)
        if hasattr(self, '_seek_time') and now == self._seek_time:
            return self._seek_jet
        dt = 0. if not hasattr(self, '_seek_time') else now-self._seek_time
        if dt < 0.:
            raise ValueError('contact seek requires monotonic native source time')
        depth = getattr(self, '_seek_depth', 0.)
        previous = getattr(self, '_seek_rate', 0.)
        remaining = max(0., self.SEEK_MAX_M-depth)
        # Discrete stopping-distance bound includes this update's travel.
        # Braking begins before the positional cap without a velocity reset.
        half_accel_dt = .5*self.SEEK_ACCEL_MPS2*dt
        safe_rate = max(0., math.sqrt(half_accel_dt**2 +
            2*self.SEEK_ACCEL_MPS2*max(0., remaining-.5*previous*dt))-half_accel_dt)
        desired = 0.
        if phase == 'land':
            feet = self.w.feet()
            loads = self.w.arr(20, (17,))
            loaded = float(loads[self.swing_body]) >= self.SEEK_LOAD_N
            nominal_done = t >= self.phase_duration('land')
            near_ground = feet[self.swing_side]['clearance'] <= .003
            stance_loaded = loads[feet[self.stance_side]['body']] >= .8*self.w.mass*9.81
            if nominal_done and near_ground and stance_loaded and not loaded:
                desired = min(self.SEEK_SPEED_MPS, safe_rate)
        # An acquired contact or phase change requests braking. Continue its
        # tiny remaining target travel through settle, rather than dropping
        # target speed discontinuously at the original 3N landing admission.
        rate = previous + float(np.clip(desired-previous,
            -self.SEEK_ACCEL_MPS2*dt, self.SEEK_ACCEL_MPS2*dt))
        depth += min(remaining, .5*(previous+rate)*dt)
        acceleration = (rate-previous)/dt if dt > 0. else 0.
        self._seek_time, self._seek_depth, self._seek_rate = now, depth, rate
        self._seek_jet = (depth, rate, acceleration)
        self.contact_seek = dict(depthM=depth, rateMps=rate,
            accelerationMps2=acceleration, limitM=self.SEEK_MAX_M,
            measuredLoadThresholdN=self.SEEK_LOAD_N,
            originalLandingDeadlinePreserved=True)
        return self._seek_jet

    def solve(self, t, phase='transfer'):
        u, info = super().solve(t, phase)
        info['contactSeek'] = self.contact_seek
        return u, info


class UprightController(ContactSeekingController):
    def add_upright_tasks(self, t, phase, width, task, inequalities, bounds):
        w = self.w
        v = w.arr(1, (27,))
        rotations = w.arr(5, (17,3,3))
        self.upright_control = []
        for body, weight in ((1, 4.), (4, 3.)):
            R = rotations[body]
            J = w.read('lh_body_jac', (6,27), body)[3:]
            bias = w.read('lh_jdot', (6,), body)[3:]
            omega = J@v
            up = R[:,2]
            # Use the motor-reserved upright reference as bounded balance
            # slack. It minimizes vertical error in the static planner;
            # commanding zero lean despite that witness can unload unsafely.
            start=np.array(self.upright_start[str(body)])
            target=np.array(self.upright_references[str(body)])
            s,ds,dds=clock(t,4.) if phase=='transfer' else (1.,0.,0.)
            if phase in ('settle','stand'):
                yaw=math.atan2(target[1,0],target[0,0])
                upright=Rotation.from_euler('Z',yaw).as_matrix()
                start=target;target=upright
                s,ds,dds=clock(t,4.) if phase=='settle' else (1.,0.,0.)
            axis=Rotation.from_matrix(target@start.T).as_rotvec()
            target_R=Rotation.from_rotvec(axis*s).as_matrix()@start
            desired_up=target_R[:,2]
            correction = np.cross(up, desired_up)
            angle = math.acos(float(np.clip(up@desired_up,-1.,1.)))
            correction *= max(angle-math.radians(DEADBAND_DEG),0.)/max(np.linalg.norm(correction),1e-12)
            horizontal = np.eye(3)[:2]
            task(horizontal@J, horizontal@(axis*dds+40.*correction+12.*(axis*ds-omega)-bias), weight)
            if body==4:
                current_yaw=math.atan2(R[1,0],R[0,0])
                error=math.atan2(math.sin(self.pelvis_yaw-current_yaw),math.cos(self.pelvis_yaw-current_yaw))
                task(J[2:3],np.array([40.*error-12.*omega[2]-bias[2]]),4.)
            row, derivative, second_bias = cone_jet(up, J, omega, bias)
            h = float(up[2]-math.cos(math.radians(TILT_LIMIT_DEG)))
            # Exponential cone barrier; it bounds inclination while allowing
            # balance motion inside it. This adds no physical constraint.
            inequality = np.zeros(width); inequality[:27] = -row
            inequalities.append(inequality)
            bounds.append(second_bias+16.*derivative+64.*h)
            self.upright_control.append(dict(body='torso' if body==1 else 'pelvis',
                **body_orientation(R), softDeadbandDeg=DEADBAND_DEG,
                tiltLimitDeg=TILT_LIMIT_DEG, coneMargin=h, coneVelocity=derivative,
                taskWeight=weight,target=body_orientation(target_R)))
        # Retain the baseline's soft axial heading preference. Uprightness only
        # replaces pitch/roll lean; dropping yaw tracking can twist the spine.
        R=rotations[1];J=w.read('lh_body_jac',(6,27),1)[3:]
        bias=w.read('lh_jdot',(6,),1)[3:]
        error=Rotation.from_matrix(self.rootR@R.T).as_rotvec()
        task(J[2:3],np.array([40.*error[2]-12.*(J@v)[2]-bias[2]]),4.)
        if phase == 'transfer' and hasattr(self, 'minimum_ankle_ready_rad'):
            # The static reserved posture was underweighted by the global .2
            # posture task. Reach its ankle reference through joint torques
            # before the unchanged measured-reserve unload admission.
            indices = np.array([7, 13])
            q = w.arr(0, (28,))[7:]
            task(np.eye(27)[6+indices],
                 20.*(self.q0[indices]-q[indices])-8.*v[6+indices], 3.)

    def solve(self, t, phase='transfer'):
        u, info = super().solve(t, phase)
        info['uprightControl'] = self.upright_control
        return u, info


class LargeStep(OpposedArmMultiStep):
    def __init__(self, base, helper, output, distances, upright):
        self.experimental_distance_limit = .302
        self.support_relative_reach = .30
        self.upright_enabled = upright
        self.arm_mapping_type=ExtendedArmMap
        super().__init__(base, helper, output, distances)
        self.code_at_start.update({name:sha(Path(__file__).with_name(name)) for name in
            ('source_upright_large_steps.py',)})

    def begin_step(self):
        super().begin_step()
        if not self.upright_enabled:
            self.controller.__class__ = ContactSeekingController
        if self.upright_enabled:
            w=self.world;before=native_bytes(w)
            arms=self.arm_map.evaluate(-(limb_telemetry(w,'leg')[0]-self.leg_origin))[0]
            reference=plan_posture(w,self.controller.stance_side,True,.9,arms,KNEE_GOAL_RAD,upright=1.,ankle_reserve=.06)
            assert native_bytes(w)==before
            (self.folder/f'upright-reference-{self.step_index+1:02d}.json').write_text(json.dumps(reference,indent=2)+'\n')
            assert reference['staticMotorContactFeasible'] and reference['minimumRequiredMotorFraction']<.925, 'no reserved upright static posture reference'
            assert reference['minimumSelfSeparationM']>.025, 'upright posture too close to self collision'
            (self.folder/f'stance-reference-{self.step_index+1:02d}.json').write_text(json.dumps(reference,indent=2)+'\n')
            self.controller=UprightController.from_controller(self.controller,reference,self.arm_map,self.leg_origin,self.step_index==0)
            self.controller.upright_references=reference['uprightBodyReferencesSource']
            self.controller.upright_start={str(body):w.arr(5,(17,3,3))[body].copy().tolist()for body in (1,4)}
            self.controller.minimum_ankle_ready_rad=.035
            R=w.arr(5,(17,3,3))[4];self.controller.pelvis_yaw=math.atan2(R[1,0],R[0,0])
            self.controller.solve(0., 'transfer')
        # The parent writes this before the subclass is installed. Replace only
        # its descriptive record; no state is installed or integrated here.
        record = {k:v for k,v in vars(self.controller).items() if k!='w' and not k.startswith('_')}
        (self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(
            json.dumps(record,default=lambda x:x.tolist())+'\n')

    def decorate(self, row):
        row = super().decorate(row)
        rotations = self.world.arr(5, (17,3,3))
        row['uprightMeasured'] = dict(torso=body_orientation(rotations[1]),
            pelvis=body_orientation(rotations[4]),
            torsoRelativePelvis=body_orientation(rotations[4].T@rotations[1]))
        if row['elapsedS']>INHERITED_ENTRY_LIMIT_END_S:
            if row['jointLimitMinimumMarginRad']<0:
                self.failure='measured-new-joint-limit-overrun'
        if row['elapsedS']>0:
            if row['minimumComCaptureMarginM']is None or row['minimumComCaptureMarginM']<0:
                self.failure='measured-current-support-capture-outside'
        if self.upright_enabled:
            if max(row['uprightMeasured'][body]['tiltDeg'] for body in ('torso','pelvis'))>TILT_LIMIT_DEG+1e-8:
                self.failure='measured-upright-tilt-12deg'
            if row['phase']=='stand' and (row['uprightMeasured']['pelvis']['tiltDeg']>5. or
                    row['uprightMeasured']['torsoRelativePelvis']['tiltDeg']>5.):
                self.quiet_run=0;row['quietConsecutiveS']=0.
        return row

    def save(self, reason):
        super().save(reason)
        if hasattr(self,'folder'):
            (self.folder/'large-step-config.json').write_text(json.dumps(dict(
                uprightEnabled=self.upright_enabled,softDeadbandDeg=DEADBAND_DEG,
                uprightStaticPreferenceWeight=1.,
                uprightAnkleReferenceReserveRad=.06,uprightAnkleReadyReserveRad=.035,
                uprightTransferAnkleTaskWeight=3.,uprightTransferAnkleTaskKp=20.,
                uprightTransferAnkleTaskKd=8.,
                runtimeMeasuredCaptureAndJointGuards=True,inheritedJointEntryGraceS=INHERITED_ENTRY_LIMIT_END_S,
                tiltLimitDeg=TILT_LIMIT_DEG,requestedDistancesM=self.distances,
                experimentalMaximumDistanceM=self.experimental_distance_limit,
                experimentalMaximumSupportRelativeReachM=self.support_relative_reach,
                originalMaximumSupportRelativeReachM=.18,
                originalMotorAndContactSafetyGates=True,
                boundedContactSeek=dict(maximumDepthM=ContactSeekingController.SEEK_MAX_M,
                    maximumSpeedMps=ContactSeekingController.SEEK_SPEED_MPS,
                    maximumAccelerationMps2=ContactSeekingController.SEEK_ACCEL_MPS2,
                    measuredLoadThresholdN=ContactSeekingController.SEEK_LOAD_N,
                    originalLandingDeadlinePreserved=True)),indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--distances', type=float, nargs='+', default=[.12,.18,.24])
    p.add_argument('--reference', action='store_true', help='unchanged opposed-arm orientation task')
    args = p.parse_args()
    assert 1<=len(args.distances)<=4 and all(.03<=d<=.30 for d in args.distances)
    session, helper, selected = prepare(args.output)
    live = LargeStep(session,helper,args.output/'run',tuple(args.distances),not args.reference)
    live.paused=False; started=time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started<300., '300s process cap'
            live.advance(4)
            if live.steps%800==0:
                print('progress',live.steps*.00125,'step',live.step_index+1,live.phase,
                      'slip',live.slip,'quiet',live.quiet_run*.00125,flush=True)
        print(live.failure or 'measured-alternating-multistep-plus5squiet',flush=True)
    except Exception as error:
        live.failure=repr(error)
        raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__=='__main__': main()
