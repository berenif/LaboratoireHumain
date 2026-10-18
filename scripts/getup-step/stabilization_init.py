"""Causal source SLOW torque labels for a NEW direct-torque initialization.

No fitting, engine calls or model export occurs on import. This is not a SLOW
residual head re-labeling and not teacher replay. First-substep controls from an
800Hz policy are only proposed labels for a future held200Hz torque teacher.
"""
import hashlib,json,math
from pathlib import Path
import numpy as np
from source_math import ORDER,conditioned_teacher_observation
from source_task import decode_source_frame,fixed_stabilization_target


def causal_pair(previous,current,fixed_target,gear_qpos):
    if type(previous.get('sourceStep'))is not int or type(current.get('sourceStep'))is not int or current['sourceStep']!=previous['sourceStep']+1:raise ValueError('causal label requires consecutive PRE/action packets')
    if not all(type(r.get('timeS'))in(int,float)and math.isfinite(r['timeS'])for r in [previous,current])or abs(current['timeS']-previous['timeS']-.00125)>1e-9:raise ValueError('label is not one source800Hz integration')
    u=np.asarray(current['controlsActuatorOrder'],dtype=float);force=np.asarray(current['qfrcActuator'],dtype=float);gear=np.asarray(gear_qpos,dtype=float)
    if u.shape!=(21,)or force.shape!=(27,)or gear.shape!=(21,)or not np.isfinite([*u,*force,*gear]).all()or min(gear)<=0:raise ValueError('invalid measured motor labels')
    derived=force[6:][ORDER]/gear[ORDER]
    if np.max(abs(u-derived))>1e-6 or np.max(abs(u))>=1-1e-6:raise ValueError('inconsistent or saturated actual torque label')
    t=np.r_[fixed_target['q'],fixed_target['com'][2],fixed_target['com_ori']]
    obs=conditioned_teacher_observation(previous['physicalObservation'],t,t,1.)
    return obs.astype(np.float32),u.astype(np.float32),np.arctanh(u).astype(np.float32)


def build_causal_dataset(journal,model):
    raw=Path(journal).read_bytes();rows=[json.loads(line)for line in raw.splitlines()]
    if len(rows)!=6401 or any(type(r.get('sourceStep'))is not int for r in rows)or [r.get('sourceStep')for r in rows]!=list(range(6401)):raise ValueError('expected exact admitted source SLOW journal IDs0..6400')
    target=fixed_stabilization_target(decode_source_frame(rows[-1],model));gear=np.zeros(21)
    for p in model['segments']:
        h=p['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    xs=[];us=[];mus=[];ids=[]
    for native_id in range(2401,6401,20):
        x,u,mu=causal_pair(rows[native_id-1],rows[native_id],target,gear);xs.append(x);us.append(u);mus.append(mu);ids.append(native_id)
    result=dict(observation=np.array(xs),control=np.array(us),raw_mean_target=np.array(mus),label_source_ids=np.array(ids,dtype=np.int64))
    receipt=dict(sourceJournalSHA256=hashlib.sha256(raw).hexdigest(),sourceInitialId=0,sourceFinalId=6400,pairCount=200,fitPairCount=120,heldoutPairCount=80,firstLabelId=2401,lastLabelId=6381,preInputIdShift=-1,labelStride=20,power=1.,fixedEntryTargetSourceId=6400,sourceHz=800,futureTeacherHz=200,policyHz=40,saturatedLabels=0,maximumActualControl=float(np.max(abs(result['control']))),scope='actual PRE physical→first-substep normalized actuator torque; approximate initialization labels, not teacher replay or dynamics equivalence')
    return result,receipt


def logstd_raw_bias(std=.02):
    if not math.isfinite(std)or not math.exp(-5)<std<math.exp(2):raise ValueError('std outside source map interior')
    return math.atanh((math.log(std)+1.5)/3.5)


def initialize_torque_actor(config):
    from training import source_module,ROOT
    from actors import DenseActor
    source=source_module();source.torch.set_num_threads(1);source.torch.manual_seed(config['pilot']['initializer']['fixedSeed'])
    actor=source.Actor(118,21,-5,2).to(source.device)
    dense=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin').conditioned_teacher()
    with source.torch.no_grad():
        for i,(w,b)in zip([0,2,4],dense.layers):actor.trunk[i].weight.copy_(source.torch.from_numpy(w));actor.trunk[i].bias.copy_(source.torch.from_numpy(b))
        actor.trunk[4].weight[21:].zero_();actor.trunk[4].bias[21:].fill_(logstd_raw_bias())
    return actor


def control_error(actor,observations,controls):
    import torch
    x=torch.as_tensor(observations,dtype=torch.float32)
    with torch.inference_mode():
        predicted,_,_,logstd=actor(x,compute_pi=False,compute_log_pi=False)
        error=predicted.numpy()-np.asarray(controls)
    if not np.isfinite(error).all():raise ValueError('nonfinite fitted actor')
    return dict(rms=float(np.sqrt(np.mean(error**2))),maximumAbsolute=float(np.max(abs(error))),perActuatorRMS=np.sqrt(np.mean(error**2,axis=0)).tolist(),perActuatorMaximumAbsolute=np.max(abs(error),axis=0).tolist(),maximumStdError=float(np.max(abs(np.exp(logstd.numpy())-.02))))


def fit_mean_only(actor,dataset,config,*,before_optimizer_step):
    """Bounded new supervised adaptation, not source SAC or a training resume.

    before_optimizer_step must charge/admit every update in the NEW stage ledger.
    Mean-only loss gives exactly-zero gradients in the logstd output rows; their
    values are checked after every step. No actor weights are exported here.
    """
    import torch
    cfg=config['pilot']['initializer'];rng=np.random.default_rng(cfg['fixedSeed']);torch.set_num_threads(1)
    x=torch.as_tensor(dataset['observation'][:120],dtype=torch.float32);y=torch.as_tensor(dataset['raw_mean_target'][:120],dtype=torch.float32)
    if x.shape!=(120,118)or y.shape!=(120,21)or not torch.isfinite(x).all()or not torch.isfinite(y).all():raise ValueError('wrong admitted supervised dataset')
    optimizer=torch.optim.Adam(actor.parameters(),lr=cfg['learningRate'],betas=(.9,.999))
    last=actor.trunk[4];fixed_w=last.weight[21:].detach().clone();fixed_b=last.bias[21:].detach().clone();losses=[]
    for update in range(cfg['maximumSupervisedUpdates']):
        indexes=torch.as_tensor(rng.integers(0,120,cfg['batchSize']),dtype=torch.long)
        raw=actor.trunk(x[indexes]);loss=torch.mean((raw[:,:21]-y[indexes])**2)
        if not torch.isfinite(loss):raise ValueError('nonfinite initialization loss')
        optimizer.zero_grad(set_to_none=True);loss.backward()
        if torch.count_nonzero(last.weight.grad[21:])or torch.count_nonzero(last.bias.grad[21:]):raise ValueError('mean fit modified logstd gradient')
        if any(p.grad is not None and not torch.isfinite(p.grad).all()for p in actor.parameters()):raise ValueError('nonfinite initialization gradient')
        before_optimizer_step(update);optimizer.step()
        if any(not torch.isfinite(p).all()for p in actor.parameters()):raise ValueError('nonfinite fitted parameters')
        if not torch.equal(last.weight[21:],fixed_w)or not torch.equal(last.bias[21:],fixed_b):raise ValueError('mean fit changed frozen logstd')
        losses.append(float(loss.detach()))
    report=dict(updates=len(losses),firstLoss=losses[0],finalLoss=losses[-1],fit=control_error(actor,dataset['observation'][:120],dataset['control'][:120]),heldout=control_error(actor,dataset['observation'][120:],dataset['control'][120:]),logstdWeightsUnchanged=True,logstdBiasUnchanged=True)
    report['admitted']=bool(report['heldout']['rms']<=cfg['validationRMSControlMaximum']and report['heldout']['maximumAbsolute']<=cfg['validationMaximumAbsoluteControlError']and report['heldout']['maximumStdError']<1e-7)
    return report
