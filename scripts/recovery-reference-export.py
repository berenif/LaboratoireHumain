#!/usr/bin/env python3
"""Bounded, headless inference of the pinned official Tao teacher/student actors.
No training, optimizer/replay buffer, GUI, or changes to source model/dynamics.
Use only with the separately authorized source checkout and isolated environment.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--budget-receipt', type=Path, required=True)
args = parser.parse_args()
receipt_path=args.budget_receipt.resolve()
receipt=json.loads(receipt_path.read_text())
if not all(receipt.get(k) is True for k in ('thirdPartyInstallationAuthorized','thirdPartyExecutionAuthorized','checkpointLoadingAuthorized')):
    raise RuntimeError('Explicit extraction authorization missing')
started=int(receipt.get('actualExportsStarted',receipt.get('actualExportsCompleted',0)))
remaining=float(receipt['exportBudgetSeconds'])-(time.monotonic()-float(receipt['exportBudgetStartedMonotonic']))
if started>=2 or remaining<=0:
    raise RuntimeError('Extraction manifest exhausted: at most two exports / one hour')
source = args.source.resolve()
out = args.out.resolve()
if out.exists():
    raise RuntimeError('Export destination must be fresh; existing evidence is immutable')
receipt['actualExportsStarted']=started+1
receipt_path.write_text(json.dumps(receipt,indent=2)+'\n')
out.mkdir(parents=True)

sys.path.insert(0, str(source))
os.chdir(source)
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import torch
import mujoco
import env as official
from SAC import Actor

torch.set_num_threads(1)
torch.set_num_interop_threads(1)
torch.manual_seed(args.seed)
np.random.seed(args.seed)
start = time.monotonic()
signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(TimeoutError('600 s export limit')))
signal.alarm(max(1,min(600,int(remaining))))

# Rendering only: suppress constructor/reset images, leaving all physics intact.
official.HumanoidStandupEnv.render = lambda self, mode=None: np.zeros((1, 1, 3), dtype=np.uint8)
official.HumanoidStandupVelocityEnv.render = official.HumanoidStandupEnv.render

class ActorOnly:
    def __init__(self, directory):
        file = source / directory / 'model/best_model_actor.pt'
        # Tensor-only deserialization; no critic, optimizer or executable object.
        weights = torch.load(file, map_location='cpu', weights_only=True)
        state_dim = weights['trunk.0.weight'].shape[1]
        action_dim = weights['trunk.4.bias'].shape[0] // 2
        self.actor = Actor(state_dim, action_dim, -5, 2).eval()
        self.actor.load_state_dict(weights)
        self.sha256 = hashlib.sha256(file.read_bytes()).hexdigest()
        self.state_dim = state_dim
    def select_action(self, state):
        if len(state) != self.state_dim:
            raise ValueError(f'Observation mismatch {len(state)} != {self.state_dim}')
        with torch.no_grad():
            return self.actor(torch.as_tensor(state, dtype=torch.float32)[None],
                              compute_pi=False, compute_log_pi=False)[0][0].numpy()

teacher = ActorOnly('experiment/pretrained/teacher')
student = ActorOnly('experiment/pretrained/student')
config = json.loads((source/'experiment/pretrained/student/args.json').read_text())
config.update(to_file=False, teacher_student=True, fast_speed=0.5, slow_speed=0.5,
              teacher_power=json.loads((source/'experiment/pretrained/teacher/args.json').read_text())['curr_power'])
from types import SimpleNamespace
settings = SimpleNamespace(**config)

teacher_attempts = []
def bounded_teacher_episode(self, test_time=False):
    """Official run_one_episode with a maximum of two teacher attempts, no images."""
    while len(teacher_attempts) < 2:
        self.teacher_env.power = self.args.teacher_power
        self.reset_teacher_trajectory()
        state = self.teacher_env.reset(test_time=True)
        self.teacher_initial_state = {'qpos': self.teacher_env.physics.data.qpos.copy(),
                                      'qvel': self.teacher_env.physics.data.qvel.copy()}
        self.teacher_episode_length = 0
        end_next = False
        done = False
        while not done:
            state, _, done, _ = self.teacher_env.step(self.teacher_policy.select_action(state), test_time=True)
            self.teacher_episode_length += 1
            for key, value in [('qpos',self.teacher_env.physics.data.qpos),
                               ('qvel',self.teacher_env.physics.data.qvel),
                               ('com',self.teacher_env.physics.center_of_mass_position()),
                               ('com_ori',self.teacher_env.physics.torso_vertical_orientation())]:
                self.teacher_traj[key].append(value.copy())
            if end_next:
                break
            if self.teacher_env.physics.head_height() > 1.4:
                end_next = True
        teacher_attempts.append({'steps':self.teacher_episode_length,
                                 'headHeightM':float(self.teacher_env.physics.head_height()),
                                 'sourceHeightTriggerReached':end_next})
        print(json.dumps({'teacherAttempt':teacher_attempts[-1]}), flush=True)
        if end_next:
            self.teacher_traj = {key:np.asarray(value) for key,value in self.teacher_traj.items()}
            self.modify_teacher_traj()
            return
    raise RuntimeError('No complete teacher reference within two attempts')
official.HumanoidStandupVelocityEnv.run_one_episode = bounded_teacher_episode

world = official.HumanoidStandupVelocityEnv(settings, args.seed)
world.set_teacher_policy(teacher)
state = world.reset(test_time=True, speed=0.5)
physics = world.physics
model = physics.model.ptr
data = physics.data.ptr
body_names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,i) or f'body{i}' for i in range(model.nbody)]
geom_names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or f'geom{i}' for i in range(model.ngeom)]
records = {k:[] for k in ('time_s','qpos','qvel','body_pos','body_quat_xyzw',
                         'com_pos','com_vel','actuator_force','qfrc_actuator','floor_load_n',
                         'contact_count','head_height_m','torso_upright')}
contact_rows=[]
contact_offsets=[0]
def solved_contacts(solve_start):
    loads=np.zeros(model.nbody)
    for i in range(data.ncon):
        c=data.contact[i]
        force=np.zeros(6)
        mujoco.mj_contactForce(model,data,i,force)
        b1=int(model.geom_bodyid[c.geom1]); b2=int(model.geom_bodyid[c.geom2])
        if (b1==0) != (b2==0):
            loads[b2 if b1==0 else b1] += max(0.0,float(force[0]))
        contact_rows.append([solve_start,c.geom1,c.geom2,*c.pos, c.dist,*force])
    return loads,int(data.ncon)

def record(loads,contact_count):
    contact_offsets.append(len(contact_rows))
    vals={'time_s':float(data.time),'qpos':data.qpos.copy(),'qvel':data.qvel.copy(),
          'body_pos':data.xpos.copy(),'body_quat_xyzw':data.xquat[:,[1,2,3,0]].copy(),
          'com_pos':physics.center_of_mass_position().copy(),
          'com_vel':physics.center_of_mass_velocity().copy(),
          'actuator_force':data.actuator_force.copy(),'qfrc_actuator':data.qfrc_actuator.copy(),
          'floor_load_n':loads,'contact_count':contact_count,
          'head_height_m':float(physics.head_height()),'torso_upright':float(physics.torso_upright())}
    for key,value in vals.items(): records[key].append(value)

# Match dm_control's legacy step operations exactly. Capture forces with the
# solver's contact list BEFORE mj_step1 rebuilds it for the new pose.
if not physics.legacy_step or model.opt.integrator == mujoco.mjtIntegrator.mjINT_RK4:
    raise RuntimeError('Exporter requires the inspected legacy non-RK4 one-substep path')
def observed_step(nstep=1):
    if nstep != 1: raise RuntimeError('Unexpected source step batch')
    solve_start=float(data.time)
    with physics.check_invalid_state():
        mujoco.mj_step2(model,data)
        loads,count=solved_contacts(solve_start)
        mujoco.mj_step1(model,data)
    record(loads,count)
physics.step=observed_step
# Reset-forward provides no new solved force; initial sample is explicitly empty.
record(np.zeros(model.nbody),0)
control_steps=0
reason='30 s source bound'
while float(data.time)-records['time_s'][0] < 30.0:
    state,_,done,_=world.step(student.select_action(state),test_time=True)
    control_steps+=1
    if done:
        reason='source done'
        break
    if control_steps >= world.teacher_episode_length + 80:
        reason='reference plus 2 s source hold'
        break
arrays={key:np.asarray(value) for key,value in records.items()}
arrays['contact_rows']=np.asarray(contact_rows,dtype=np.float64).reshape(-1,13)
arrays['contact_offsets']=np.asarray(contact_offsets,dtype=np.int64)
arrays['body_mass_kg']=model.body_mass.copy()
arrays['body_parent_id']=model.body_parentid.copy()
arrays['geom_body_id']=model.geom_bodyid.copy()
arrays['geom_pos_local']=model.geom_pos.copy()
arrays['geom_quat_local_wxyz']=model.geom_quat.copy()
arrays['geom_type']=model.geom_type.copy()
arrays['geom_size']=model.geom_size.copy()
arrays['actuator_gear']=model.actuator_gear.copy()
file=out/'tao-slow-seed0.npz'
np.savez_compressed(file,**arrays)
metadata={'sourceRepository':'https://github.com/tianxintao/get_up_control',
          'sourceCommit':'5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e',
          'seed':args.seed,'targetSpeed':0.5,'teacherAttempts':teacher_attempts,
          'teacherActorSha256':teacher.sha256,'studentActorSha256':student.sha256,
          'sourceCoordinates':{'up':'+Z','quaternion':'xyzw','length':'m','angle':'rad'},
          'bodyNames':body_names,'geomNames':geom_names,
          'timeStepS':float(model.opt.timestep),'sourceControlSubsteps':4*world.env._n_sub_steps,
          'sourceReferenceSteps':world.teacher_episode_length,'controlSteps':control_steps,
          'physicsDurationS':float(arrays['time_s'][-1]-arrays['time_s'][0]),
          'samples':len(arrays['time_s']),'endReason':reason,
          'initialHeadHeightM':float(arrays['head_height_m'][0]),
          'finalHeadHeightM':float(arrays['head_height_m'][-1]),
          'finalTorsoUpright':float(arrays['torso_upright'][-1]),
          'finalComSpeedMps':float(np.linalg.norm(arrays['com_vel'][-1])),
          'finalFloorLoadsN':dict(zip(body_names,arrays['floor_load_n'][-1].tolist())),
          'wallS':time.monotonic()-start,'numericFileSha256':hashlib.sha256(file.read_bytes()).hexdigest(),
          'nativeRapierSuccess':False,
          'exportAttempt':started+1,
          'contactObservation':'after mj_step2 with solved contact list, before mj_step1 rebuild',
          'contactRowTime':'solve start; state sample is solve end',
          'limitations':['Modern dependency port; not an exact historical engine reproduction',
                         'Source initialization reset is not permitted in a native candidate',
                         'Source anatomy, contacts, torque caps and standing criteria differ',
                         'Starts at first teacher controlled sample after source drop settling',
                         'No native adaptation/geometry/dynamic qualification yet']}
(out/'tao-slow-seed0.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(metadata),flush=True)
signal.alarm(0)
receipt['actualExportsCompleted']=int(receipt.get('actualExportsCompleted',0))+1
receipt_path.write_text(json.dumps(receipt,indent=2)+'\n')
