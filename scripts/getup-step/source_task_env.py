"""Counted original-source torque-teacher environment boundary, disabled preset.

No optimizer, no executable collection CLI, no permission/budget bypass. The
integration callback MUST own the separately reviewed write-ahead resource ledger
and return after exactly one charged mj_step2->mj_step1 call. All five actual
frames are decoded and checked before another action. Stage0 uses combined real
support; later stepping requires per-foot point telemetry/reference admission.
"""
import copy
import ctypes as ct
import hashlib,json
import numpy as np
from native_env import sha256
from source_engine import old
from source_slow_control import SlowWorld
from source_task import SourceStabilization


class SourceTaskWorld(SlowWorld):
    """200Hz static source model with the audited v2 checkpoint/support API."""
    def __init__(self,base):
        old.World.__init__(self,base/'source-model/humanoid_static.xml',base/'source_bridge_v2.so')
        for name,args,ret in [('lh_step_audited',[ct.c_void_p,ct.c_int],ct.c_int),('lh_foot_slip_speed',[ct.c_void_p,ct.c_int],ct.c_double),('lh_support_count',[ct.c_void_p],ct.c_int),('lh_body_velocity',[ct.c_void_p,ct.c_int,ct.POINTER(ct.c_double)],None),('lh_checkpoint_size',[ct.c_void_p],ct.c_int),('lh_checkpoint_export',[ct.c_void_p,ct.c_void_p,ct.c_int],ct.c_int),('lh_checkpoint_import',[ct.c_void_p,ct.c_void_p,ct.c_int],ct.c_int)]:
            f=getattr(self.lib,name);f.argtypes=args;f.restype=ret
        if abs(self.lib.lh_dt(self.h)-.005)>1e-12:raise ValueError('source teacher model is not200Hz')
        self.checkpoint_binding=bytes.fromhex(sha256(base/'source-model/humanoid_static.xml'))+bytes.fromhex(sha256(base/'source_bridge_v2.so'))+bytes.fromhex(sha256(base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    def task_packet(self):
        row=self.metrics();count=self.lib.lh_support_count(self.h)
        if not 0<=count<=64:raise ValueError('invalid source support count')
        points=self.arr(21,(128,))[:2*count].reshape(-1,2).copy();velocities=[]
        for body in range(1,17):
            v=np.zeros(6);self.lib.lh_body_velocity(self.h,body,v.ctypes.data_as(ct.POINTER(ct.c_double)));velocities.append(v)
        v=np.array(velocities)
        row['sourceQuiet']=dict(solvedPositiveForcePointsCurrentXY=points.tolist(),maximumBodyLinearMps=float(np.max(np.linalg.norm(v[:,3:],axis=1))),maximumBodyAngularRadps=float(np.max(np.linalg.norm(v[:,:3],axis=1))))
        row['measurementStage']='current post-step configuration and velocity; prior solved positive-force material anchors transported to current pose'
        json.dumps(row,allow_nan=False);return row


class SourceTaskEnv:
    def __init__(self,world,model,*,config_path,expected_config_sha256,bindings,rng,original_entry_packet=None,core_factory=SourceStabilization):
        self.world=world;current=world.task_packet();entry=current if original_entry_packet is None else original_entry_packet;self.core=core_factory(entry,model,config_path=config_path)
        self.resume_required=self.core.packet_hash(current)!=self.core.packet_hash(entry)
        if self.core.config_sha!=expected_config_sha256:raise ValueError('externally pinned source task config mismatch')
        if not isinstance(bindings,dict)or not bindings:raise ValueError('source/model/code bindings required')
        self.bindings=bindings.copy();self.rng=rng
    def step(self,normalized_actuator_torque,counted_integrate):
        # This cannot run against the reviewed disabled draft. Admission requires
        # a new reviewed config/loader change and budget, never a caller boolean.
        if not self.core.config['pilot']['collectionEnabled']:raise ValueError('source task collection is disabled')
        if self.resume_required:raise ValueError('original entry supplied: native/controller checkpoint restore required')
        if self.core.terminal:raise ValueError('source task ended')
        u=np.asarray(normalized_actuator_torque,dtype=float)
        if u.shape!=(21,)or not np.isfinite(u).all()or np.max(abs(u))>1:raise ValueError('original normalized actuator torque21 required')
        before=self.core.observation(self.world.observation());rows=[]
        self.world.arr(2,(21,))[:]=u
        for _ in range(5):
            prior=self.world.dim(6);counted_integrate(self.world)
            if self.world.dim(6)!=prior+1:raise ValueError('counted source callback must execute exactly one integration')
            row=self.world.task_packet();rows.append(row)
            if self.core.terminal_in_prefix(rows):break
        result=self.core.consume(rows,expected_teacher_controls=u)
        # No reset observation is ever substituted. Physical partial terminals
        # retain finite final state/mask0; infrastructure exceptions abort replay.
        return dict(state=before.tolist(),action=u.tolist(),next_state=result['actualFinalObservation118'],reward=result['reward'],not_done=result['not_done'],info=result,actualFrames=rows)
    def checkpoint(self):
        if self.resume_required or self.core.packet_hash(self.world.task_packet())!=self.core.last_packet_hash:raise ValueError('native/controller checkpoint boundary mismatch')
        blob=self.world.checkpoint();side=self.core.checkpoint(blob,self.bindings,self.rng.bit_generator.state)
        return blob,side
    def restore(self,blob,side):
        # Caller must preserve the global charge ledger across environment rewind.
        before=self.world.checkpoint();old_core=copy.deepcopy(self.core);old_rng=copy.deepcopy(self.rng.bit_generator.state)
        try:
            self.world.restore(blob);state=self.core.restore(side,blob,self.world.task_packet(),expected_bindings=self.bindings);self.rng.bit_generator.state=state;self.resume_required=False
        except BaseException:
            self.world.restore(before);self.core=old_core;self.rng.bit_generator.state=old_rng;raise
