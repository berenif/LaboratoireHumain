"""Original800Hz SLOW residual interface and mean initialization. No learner loop.

Archived proposal flags stay false. Measurement-only intervals require the caller's
counted integration callback; collection/optimizer release needs separate admission.
"""
import copy,json
from pathlib import Path
import numpy as np
from actors import DenseActor
from native_env import ROOT
from source_math import slow_controls,student_observation
from source_slow_control import SlowWorld
from source_task import SourceStabilization,decode_source_frame
from source_task_env import SourceTaskEnv,SourceTaskWorld
from stabilization_init import logstd_raw_bias
from task import load_config
from training import source_module
CONFIG_PATH=Path(__file__).with_name('task-config-mujoco-slow-stabilization-v1.json')
CONFIG_SHA='c8ac2dba22ac8c1f1e386575624f45f7b1ee5855b7340fba285b984e233faebf'

class SourceSLOWWorld(SlowWorld):
    task_packet=SourceTaskWorld.task_packet

class SourceSLOWCore(SourceStabilization):
    def __init__(self,entry,model,*,config_path=CONFIG_PATH):
        super().__init__(entry,model,config_path=config_path)
        if self.config['name']!='source_slow_residual_stabilization_v1'or self.sample_seconds!=.00125:raise ValueError('requires reviewed SLOW800Hz preset')
        self.source_standing=np.asarray(model['referenceMetadata']['standingPose'],dtype=float)
        self.source_actor_target=np.r_[self.source_standing,1.4,0.,0.,1.]
    def observation(self,physical,power=1.):
        if power!=1.:raise ValueError('SLOW power is1')
        return student_observation(physical,self.source_actor_target,self.source_actor_target)

class SourceSLOWEnv(SourceTaskEnv):
    def __init__(self,world,model,*,bindings,rng,original_entry_packet=None):
        super().__init__(world,model,config_path=CONFIG_PATH,expected_config_sha256=CONFIG_SHA,bindings=bindings,rng=rng,original_entry_packet=original_entry_packet,core_factory=SourceSLOWCore)
        self.held_command=None
    def measured_interval(self,residual,counted_integrate,*,extrinsic_cutoff=False):
        """No replay/optimizer: source residual held, PD refreshed at each substep."""
        if self.resume_required or self.core.terminal:raise ValueError('restore needed or task terminal')
        a=np.asarray(residual,dtype=float)
        if a.shape!=(21,)or not np.isfinite(a).all()or max(abs(a))>1:raise ValueError('qpos-order residual21 radians required')
        if not callable(counted_integrate):raise ValueError('counted integration callback required')
        before=self.core.observation(self.world.observation());rows=[];expected=[];preview=copy.deepcopy(self.core)
        self.held_command=dict(qref=self.core.source_standing.tolist(),residual=a.tolist(),kd=.2,nativeSamples=20,completedSamples=0)
        for sample in range(20):
            q=self.world.arr(0,(28,))[7:].copy();dq=self.world.arr(1,(27,))[6:].copy();u=slow_controls(self.core.source_standing,a,q,dq,holding=True)
            self.held_command.update(lastPreQ=q.tolist(),lastPreDQ=dq.tolist());self.world.arr(2,(21,))[:]=u;prior=self.world.dim(6);counted_integrate(self.world)
            if self.world.dim(6)!=prior+1:raise ValueError('exactly one counted800Hz integration required')
            row=self.world.task_packet();actual=np.asarray(row['qfrcActuator'])[6:]
            gear=np.zeros(21)
            for p in self.core.model['segments']:
                h=p['reference'].get('hinge')
                if h:gear[h['qposIndex']]=h['gear']
            from source_math import ORDER
            error=float(max(abs(actual[ORDER]-gear[ORDER]*u)))
            if error>1e-5:raise ValueError('source per-native PD torque parity failed')
            row['slowCommandAudit']=dict(expectedControls=u.tolist(),maximumTorqueErrorNm=error)
            # Audit is stripped from engine packets before controller hashing.
            expected.append(row.pop('slowCommandAudit'));rows.append(row);self.held_command['completedSamples']=sample+1
            x=decode_source_frame(row,self.core.model)
            if x['nativeId']!=self.core.native_id+sample+1 or abs(row['timeS']-(self.core.start_time+(x['nativeId']-self.core.start_id)*.00125))>1e-9:raise ValueError('invalid per-native SLOW clock')
            preview.slip+=x['slipSpeed']*.00125
            failure,_,_=preview._failure_and_quiet(x)
            if failure:break
        info=self.core.consume(rows,extrinsic_cutoff=extrinsic_cutoff)
        info.update(sourceResidualCommandVerified=True,maximumPerNativeTorqueErrorNm=max(x['maximumTorqueErrorNm']for x in expected),sourceInnerPDHz=800,sourceKDSeconds=.2)
        return dict(state=before.tolist(),action=a.tolist(),next_state=info['actualFinalObservation118'],reward=info['reward'],not_done=info['not_done'],info=info,actualFrames=rows,perNativeControls=expected)
    def checkpoint(self):
        blob,side=super().checkpoint()
        # Core bindings include only fixed identities. Last held command belongs
        # in the envelope, whose own checksum binds it together with core state.
        envelope=dict(schema=1,core=side,heldCommand=copy.deepcopy(self.held_command))
        import hashlib
        envelope['sha256']=hashlib.sha256(json.dumps(envelope,sort_keys=True,allow_nan=False).encode()).hexdigest()
        return blob,envelope
    def restore(self,blob,side):
        import hashlib
        unsigned={k:v for k,v in side.items()if k!='sha256'}
        if side.get('schema')!=1 or side.get('sha256')!=hashlib.sha256(json.dumps(unsigned,sort_keys=True,allow_nan=False).encode()).hexdigest():raise ValueError('SLOW envelope checksum mismatch')
        command=side['heldCommand']
        if command is not None:
            if command.get('kd')!=.2 or command.get('nativeSamples')!=20 or type(command.get('completedSamples'))is not int or not 1<=command['completedSamples']<=20:raise ValueError('invalid restored held source command')
            for key in ['qref','residual','lastPreQ','lastPreDQ']:
                v=np.asarray(command[key],dtype=float)
                if v.shape!=(21,)or not np.isfinite(v).all():raise ValueError('invalid restored source command')
            if max(abs(np.array(command['residual'])))>1 or not np.array_equal(command['qref'],self.core.source_standing):raise ValueError('source holding command changed')
        if command is None and side['core']['state']['native_id']!=side['core']['state']['start_id']:raise ValueError('missing restored SLOW command')
        if command is not None:
            elapsed=side['core']['state']['native_id']-side['core']['state']['start_id']
            if elapsed<1 or command['completedSamples']!=(elapsed%20 or 20):raise ValueError('restored SLOW interval boundary mismatch')
            expected=slow_controls(command['qref'],command['residual'],command['lastPreQ'],command['lastPreDQ'],True)
            if max(abs(expected-np.array(side['core']['state']['previous_controls'])))>1e-6:raise ValueError('restored SLOW command/control mismatch')
        super().restore(blob,side['core']);self.held_command=copy.deepcopy(command)

def original_slow_actor():
    source=source_module();source.torch.set_num_threads(1);actor=source.Actor(118,21,-5,2)
    dense=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/student.bin')
    with source.torch.no_grad():
        for index,(w,b)in zip([0,2,4],dense.layers):actor.trunk[index].weight.copy_(source.torch.from_numpy(w));actor.trunk[index].bias.copy_(source.torch.from_numpy(b))
    return actor

def initialize_slow_agent(seed=0):
    config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA);p=config['pilot'];source=source_module();source.torch.set_num_threads(1);source.torch.manual_seed(seed)
    agent=source.SAC(118,21,discount=p['discount'],actor_lr=p['actorLearningRate'],critic_lr=p['criticLearningRate'],alpha_lr=p['temperatureLearningRate'],init_temperature=p['initialAlpha'],policy_freq=1,tau=p['tau'])
    original=original_slow_actor();agent.actor.load_state_dict(original.state_dict())
    with source.torch.no_grad():agent.actor.trunk[4].weight[21:].zero_();agent.actor.trunk[4].bias[21:].fill_(logstd_raw_bias())
    return agent,original
