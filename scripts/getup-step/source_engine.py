#!/usr/bin/env python3
"""Pinned original200Hz MuJoCo frozen-WEAK diagnostic, separate phase ledger.

This never changes the native application. Transferred q/qvel are installed only
in a new diagnostic source world; source default reset is logged separately.
"""
import argparse,ctypes as ct,datetime as dt,fcntl,hashlib,importlib.util,json,math,signal,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import torch
from actors import DenseActor
from source_math import teacher_observation,P
from native_env import atomic_json,sha256,ROOT

POWER=.663420431
OLD=ROOT/'scripts/getup-diagnostic/compare_mujoco.py'
spec=importlib.util.spec_from_file_location('counted_source_world',OLD);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)


def torch_actor(dense):
    torch.set_num_threads(1)
    width=dense.dimensions[0]
    net=torch.nn.Sequential(torch.nn.Linear(width,1024),torch.nn.ReLU(),torch.nn.Linear(1024,1024),torch.nn.ReLU(),torch.nn.Linear(1024,42));net.eval()
    with torch.no_grad():
        for i,(w,b)in zip([0,2,4],dense.layers):net[i].weight.copy_(torch.from_numpy(w));net[i].bias.copy_(torch.from_numpy(b))
    def infer(observation):
        obs=np.asarray(observation,dtype=np.float32)
        if obs.shape!=(width,)or not np.isfinite(obs).all()or not 0<=obs[-1]<=1:raise ValueError('invalid source actor observation')
        with torch.inference_mode():raw=net(torch.from_numpy(obs)[None,:])[0];action=torch.tanh(raw[:21])*obs[-1]
        if not torch.isfinite(raw).all()or not torch.isfinite(action).all():raise ValueError('nonfinite frozen actor')
        return raw.numpy().copy(),action.numpy().copy()
    return infer


def frozen_actor():
    dense=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin');predict=torch_actor(dense)
    return lambda physical:predict(teacher_observation(physical,POWER)),dense


class World(old.World):
    def __init__(self,base):
        super().__init__(base/'source-model/humanoid_static.xml',base/'source_bridge.so')
        self.lib.lh_forward.argtypes=[ct.c_void_p];self.lib.lh_forward.restype=None
        self.lib.lh_step_audited.argtypes=[ct.c_void_p,ct.c_int];self.lib.lh_step_audited.restype=ct.c_int
        self.lib.lh_foot_slip_speed.argtypes=[ct.c_void_p,ct.c_int];self.lib.lh_foot_slip_speed.restype=ct.c_double
        self.lib.lh_reset_data.argtypes=[ct.c_void_p];self.lib.lh_reset_data.restype=None
        if abs(self.lib.lh_dt(self.h)-.005)>1e-12:raise ValueError('not original200Hz XML')
    def metrics(self):
        physical=self.observation().copy();com=self.arr(6,(17,3))[self.ids['torso']].copy();loads=self.arr(20,(17,)).copy();feet=[self.ids[x]for x in ['left_foot','right_foot']];contacts=self.contacts()
        result=dict(sourceStep=self.dim(6),timeS=self.lib.lh_time(self.h),physicalObservation=physical.tolist(),comSource=com.tolist(),loadsN=loads[feet].tolist(),allBodyFloorLoadsN=loads.tolist(),nonfootSupport=bool(any(v>=3 for i,v in enumerate(loads)if i not in [0,*feet])),loadedSlipSpeedMps=[self.lib.lh_foot_slip_speed(self.h,i)for i in range(2)],maximumSelfPenetrationM=max([0.]+[-p['distanceM']for p in contacts if 'world'not in[p['body1'],p['body2']]]),contacts=contacts,qpos=self.arr(0,(28,)).copy().tolist(),qvel=self.arr(1,(27,)).copy().tolist(),qfrcActuator=self.arr(8,(27,)).copy().tolist(),qfrcPassive=self.arr(9,(27,)).copy().tolist(),sourceWarnings=self.lib.lh_warnings(self.h))
        json.dumps(result,allow_nan=False)
        return result


def fingerprints(base):
    files=[base/'source_bridge.so',ROOT/'scripts/getup-step/source_bridge.c',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',Path(__file__),ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/teacher.bin',base/'native-quiet-entry.json',*sorted((base/'source-model').glob('*.xml'))]
    return {str(p.resolve()):sha256(p)for p in files}


def preflight(base):
    record=json.loads((base/'native-quiet-entry.json').read_text());physical=np.array(record['nativePhysicalObservation']);snapshot=record['nativeSnapshot'];infer,dense=frozen_actor();world=World(base)
    try:
        world.lib.lh_forward(world.h);default=dict(qpos=world.arr(0,(28,)).copy().tolist(),headHeightSourceM=float(world.observation()[21]),physicsIntegrations=world.dim(6),label='original XML default reset, NOT a source quiet stance or a teacher training reset')
        qpos,qvel=world.state(snapshot['segments'],physical);obs=world.observation();model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());parts={p['id']:p for p in model['segments']};poses={p['id']:p for p in snapshot['segments']};positions=world.arr(3,(17,3));quats=world.arr(4,(17,4));max_position=max_angle=0.
        for name in model['referenceMetadata']['sourceBodyOrder']:
            i=world.ids[name];max_position=max(max_position,float(np.linalg.norm(P.T@positions[i]-old.vec(poses[name]['position']))));native_r=Rotation.from_quat([poses[name]['rotation'][k]for k in'xyzw']).as_matrix();source_r=P.T@Rotation.from_quat(quats[i][[1,2,3,0]]).as_matrix()@P;max_angle=max(max_angle,float(Rotation.from_matrix(native_r.T@source_r).magnitude()))
        block_errors=[float(np.max(abs(obs[a:b]-physical[a:b])))for a,b in [(0,21),(21,37),(37,40),(40,67)]]
        raw_same,action_same=infer(physical);raw_source,action_source=infer(obs);native_raw=np.array(record['nativeTeacherRaw']);native_action=np.array(record['nativeTeacherAction']);mass_error=inertia_error=0.
        for name in model['referenceMetadata']['sourceBodyOrder']:
            i=world.ids[name];part=parts[name];mass_error=max(mass_error,abs(world.arr(10,(17,))[i]-part['massKg']));r=Rotation.from_quat(world.arr(13,(17,4))[i][[1,2,3,0]]).as_matrix();tensor=P.T@r@np.diag(world.arr(12,(17,3))[i])@r.T@P;inertia_error=max(inertia_error,float(np.max(abs(tensor-np.array(part['goldenMass']['inertia'])))))
        gears=world.arr(14,(21,6))[:,0];gear_error=0.
        for part in parts.values():
            h=part['reference'].get('hinge')
            if h:gear_error=max(gear_error,abs(gears[h['actuatorIndex']]-h['gear']))
        q_before=world.arr(0,(28,)).copy();v_before=world.arr(1,(27,)).copy();t_before=world.lib.lh_time(world.h);rejected=world.lib.lh_step_audited(world.h,0)==-1
        if world.dim(6)!=0 or world.lib.lh_time(world.h)!=t_before or not np.array_equal(world.arr(0,(28,)),q_before)or not np.array_equal(world.arr(1,(27,)),v_before):raise ValueError('zero-step guard changed state')
        atomic_json(base/'preflight-measurements.json',dict(physicsIntegrations=world.dim(6),maximumBodyPositionErrorM=max_position,maximumBodyRotationErrorRad=max_angle,observationBlockErrors=block_errors,massError=mass_error,inertiaError=inertia_error,gearError=gear_error,rawSameError=float(np.max(abs(raw_same-native_raw))),actionSameError=float(np.max(abs(action_same-native_action))),actionSourceDifference=float(np.max(abs(action_source-action_same))),warnings=world.lib.lh_warnings(world.h)))
        if mass_error>1e-6 or inertia_error>1e-6 or gear_error!=0 or max_position>2e-4 or max_angle>2e-6 or block_errors[0]>1e-12 or block_errors[1]>2e-4 or block_errors[2]>2e-4 or block_errors[3]>1e-12:raise ValueError('matched source configuration/parameter parity rejected')
        if np.max(abs(raw_same-native_raw))>2e-4 or np.max(abs(action_same-native_action))>5e-5 or np.max(abs(action_source-action_same))>1e-3:raise ValueError('frozen teacher inference/configuration parity rejected')
        report=dict(passed=True,engineVersion=world.lib.lh_version(),physicsIntegrations=0,explicitStaticForwardCalls=2,zeroStepGuardRejectedWithoutMutation=rejected,sourceDtSeconds=.005,sourcePolicyHz=40,teacherPower=POWER,sourceDefaultReset=default,matching='transferred native quiet q/qvel, normalized root; source contact/solver warm starts cold and not transferable; no settling integrations',maximumBodyPositionErrorM=max_position,maximumBodyRotationErrorRad=max_angle,observationBlockErrors=block_errors,massMaximumErrorKg=mass_error,inertiaMaximumErrorKgM2=inertia_error,gearMaximumError=gear_error,raw42ErrorSameObservation=float(np.max(abs(raw_same-native_raw))),action21ErrorSameObservation=float(np.max(abs(action_same-native_action))),action21DifferenceFromSourceFKObservation=float(np.max(abs(action_source-action_same))),nativeSourceCOM=record['nativeSourceCOM'],matchedSourceCOM=world.arr(6,(17,3))[world.ids['torso']].copy().tolist(),qpos=qpos.tolist(),qvel=qvel.tolist(),sourceInitialMetrics=world.metrics(),fingerprints=fingerprints(base),qualification='configuration/actor parity only, not identical engine state or stance success')
        atomic_json(base/'preflight.json',report);print(json.dumps({k:v for k,v in report.items()if k not in['sourceInitialMetrics','sourceDefaultReset','qpos','qvel','fingerprints']},indent=2))
    finally:world.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['preflight']);p.add_argument('--base',type=Path,required=True);a=p.parse_args();preflight(a.base)
if __name__=='__main__':main()
