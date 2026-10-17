#!/usr/bin/env python3
"""Bounded CPU SAC pilot over the native JSONL experiment; no app replacement."""
import argparse, copy, datetime, hashlib, importlib.util, json, math, os
from pathlib import Path
import random, signal, struct, subprocess, sys, time
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'scripts/getup-training/SAC.py'
SOURCE_SHA = '64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420'
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == SOURCE_SHA
spec = importlib.util.spec_from_file_location('pinned_getup_sac', SOURCE)
upstream = importlib.util.module_from_spec(spec); spec.loader.exec_module(upstream)
SEED = 20261017
# Frozen before any training or baseline evaluation. Same state, pulses, duration.
HELDOUT = [{'name':name,'force_n':force,'pulse_start':20,'pulse_ticks':8,'ticks':160}
    for name,force in [('forward',[20.,0.]),('backward',[-20.,0.]),
                       ('left',[0.,20.]),('right',[0.,-20.])]]


def actor_tensors(blob):
    assert blob[:8] == b'LHGPW001'
    dims = struct.unpack('<4I', blob[8:24]); assert dims in [(118,1024,1024,42),(120,1024,1024,42)]
    values=np.frombuffer(blob[24:],dtype='<f4').copy(); offset=0; tensors={}
    for idx,(a,b) in zip([0,2,4],zip(dims,dims[1:])):
        w=values[offset:offset+a*b].reshape(b,a).copy(); offset+=a*b
        bias=values[offset:offset+b].copy();offset+=b
        if idx==0 and a==118:
            expanded=np.zeros((b,120),np.float32);expanded[:,:117]=w[:,:117];expanded[:,119]=w[:,117];w=expanded
        tensors[f'trunk.{idx}.weight']=torch.from_numpy(w)
        tensors[f'trunk.{idx}.bias']=torch.from_numpy(bias)
    assert offset==len(values) and all(torch.isfinite(t).all() for t in tensors.values())
    return tensors


def export_actor(actor,path):
    blob=b'LHGPW001'+struct.pack('<4I',120,1024,1024,42)
    for idx in [0,2,4]:
        for kind in ['weight','bias']:
            blob+=actor.state_dict()[f'trunk.{idx}.{kind}'].detach().cpu().numpy().astype('<f4').tobytes()
    path.write_bytes(blob)
    restored=copy.deepcopy(actor);restored.load_state_dict(actor_tensors(blob))
    probe=torch.linspace(-.2,.2,120).repeat(8,1);probe[:,-1]=1
    with torch.no_grad():
        assert torch.equal(actor(probe,compute_pi=False,compute_log_pi=False)[0],
                           restored(probe,compute_pi=False,compute_log_pi=False)[0])
    return hashlib.sha256(blob).hexdigest()


class Replay:
    def __init__(self,rng):
        self.rng=rng;self.size=0
        self.s=np.zeros((3000,120),np.float32);self.a=np.zeros((3000,21),np.float32)
        self.n=np.zeros_like(self.s);self.r=np.zeros((3000,1),np.float32);self.nd=np.zeros_like(self.r)
    def add(self,s,a,n,r,not_done):
        i=self.size;assert i<3000
        self.s[i]=s;self.a[i]=a;self.n[i]=n;self.r[i]=r;self.nd[i]=not_done;self.size+=1
    def sample(self,n):
        i=self.rng.integers(self.size,size=n)
        return tuple(torch.from_numpy(x[i]) for x in [self.s,self.a,self.n,self.r,self.nd])
    def state(self):
        return {'size':self.size,**{k:torch.from_numpy(getattr(self,k)[:self.size].copy()) for k in ['s','a','n','r','nd']}}


class Native:
    def __init__(self,binary,checkpoint,out,deadline):
        self.out=out;self.deadline=deadline;self.last_wall=0.;self.last_native=0
        self.trace=(out/'native-protocol.ndjson').open('w');self.uncertain=[]
        self.proc=subprocess.Popen([str(binary),str(checkpoint),'360'],cwd=ROOT,
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(out/'native-stderr.log').open('w'),text=True)
    def command(self,command):
        if time.time()>=self.deadline:raise TimeoutError('total-compute deadline')
        timeout=min(5.,self.deadline-time.time())
        if command.get('op')=='step':timeout=min(timeout,360.-self.last_wall-.25)
        if timeout<=0:raise TimeoutError('No request budget remaining')
        began=time.perf_counter()
        def alarm(_signum,_frame):raise TimeoutError('native full-response timeout')
        old_handler=signal.signal(signal.SIGALRM,alarm)
        signal.setitimer(signal.ITIMER_REAL,timeout)
        try:
            self.proc.stdin.write(json.dumps(command)+'\n');self.proc.stdin.flush()
            line=self.proc.stdout.readline()
            if not line:raise RuntimeError('native process closed stdout')
            row=json.loads(line)
        except Exception:
            if command.get('op')=='step':
                elapsed=time.perf_counter()-began
                self.last_wall+=elapsed;self.uncertain.append({'chargedWallS':elapsed,'nativeIntegrationsUpperBound':20})
            self.proc.kill();self.ledger();raise
        finally:
            signal.setitimer(signal.ITIMER_REAL,0);signal.signal(signal.SIGALRM,old_handler)
        # Counters are charged on all responses, including errors and failures.
        assert row['processPhysicsWallS']>=self.last_wall and row['processNativeSubsteps']>=self.last_native
        self.last_wall=row['processPhysicsWallS'];self.last_native=row['processNativeSubsteps']
        self.trace.write(json.dumps({'request':command,'response':row})+'\n');self.trace.flush()
        self.ledger()
        if not row['ok']:raise RuntimeError(row['error'])
        return row['result']
    def ledger(self):
        (self.out/'budget-ledger.json').write_text(json.dumps({'physicsWallSeconds':self.last_wall,
            'nativeIntegrations':self.last_native,'uncertainOutstandingRequests':self.uncertain,'physicsLimitSeconds':360,
            'utcDeadline':datetime.datetime.fromtimestamp(self.deadline,datetime.timezone.utc).isoformat()},indent=2)+'\n')
    def close(self):
        try:self.proc.stdin.close()
        except BrokenPipeError:pass
        self.proc.wait(timeout=20);self.trace.close();self.ledger()


def scalar_observation(state):return np.asarray(state['observation'],dtype=np.float32)


def evaluate(native,actor,label,out):
    results=[]
    for case in HELDOUT:
        state=native.command({'op':'reset'});frames=[];rewards=[];states=[];stopped=None
        for tick in range(case['ticks']):
            action=actor.select_action(scalar_observation(state))
            force=case['force_n'] if case['pulse_start']<=tick<case['pulse_start']+case['pulse_ticks'] else [0.,0.]
            transition=native.command({'op':'step','action':action.tolist(),'torso_force_n':force})
            state=transition['state'];rewards.append(transition['reward']['total']);states.append(state)
            if tick%2==0:frames.append(native.command({'op':'snapshot'}))
            if transition['terminated'] or transition['truncated']:
                stopped=transition.get('failure') or ('terminated' if transition['terminated'] else 'truncated');break
        metrics={'case':case,'label':label,'decisions':len(rewards),'return':float(sum(rewards)),
            'stopped':stopped,'final':state,'maxAnchorDriftM':max(s['maximum_anchor_drift_m'] for s in states),
            'minimumTorsoUp':min(s['metrics']['torso_up'] for s in states),
            'finalContinuousQuietS':state['continuous_quiet_s']}
        (out/f'{label}-{case["name"]}.json').write_text(json.dumps({'metrics':metrics,'frames':frames},indent=2)+'\n')
        results.append(metrics)
    return results


def save_checkpoint(agent,replay,rng,native,out,step,updates,report):
    native_path=out/'learner-native-checkpoint.bin'
    checkpoint_error=None
    try:native.command({'op':'checkpoint','path':str(native_path)})
    except Exception as e:checkpoint_error=repr(e)
    payload={'schema':1,'model':agent.state_dict(),'actorOptimizer':agent.actor_optimizer.state_dict(),
        'criticOptimizer':agent.critic_optimizer.state_dict(),'alphaOptimizer':agent.log_alpha_optimizer.state_dict(),
        'logAlpha':agent.log_alpha.detach().clone(),'totalUpdates':agent.total_it,'actorLogStdMin':agent.actor.log_std_min,
        'actorLogStdMax':agent.actor.log_std_max,'replay':replay.state(),'numpyRng':rng.bit_generator.state,
        'torchRng':torch.get_rng_state(),'pythonRng':random.getstate(),'trainingTransitions':step,
        'updateCount':updates,'budgetWallSeconds':native.last_wall,'budgetNativeIntegrations':native.last_native,
        'nativeCheckpointSha256':hashlib.sha256(native_path.read_bytes()).hexdigest() if native_path.exists() else None,
        'nativeCheckpointSynchronized':checkpoint_error is None,'nativeCheckpointError':checkpoint_error,
        'deadlineUnix':native.deadline,'newPhysicsAllowanceSeconds':360,
        'resumeRequiresBudgetCarryforward':True,'report':report}
    torch.save(payload,out/'learner.pt')
    restored=torch.load(out/'learner.pt',map_location='cpu',weights_only=True)
    assert restored['trainingTransitions']==step and restored['replay']['size']==replay.size
    assert all(torch.equal(v,restored['model'][k]) for k,v in agent.state_dict().items())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--deadline',type=str,required=True)
    p.add_argument('--validate-only',action='store_true');a=p.parse_args()
    deadline=datetime.datetime.fromisoformat(a.deadline.replace('Z','+00:00')).timestamp()
    assert deadline<=datetime.datetime(2026,10,17,10,44,20,tzinfo=datetime.timezone.utc).timestamp()
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'started-once').open('x').close()
    torch.set_num_threads(2);torch.set_num_interop_threads(1);torch.manual_seed(SEED);random.seed(SEED)
    rng=np.random.default_rng(SEED)
    agent=upstream.SAC(120,21,init_temperature=.001,actor_lr=1e-6,critic_lr=1e-4,discount=.99)
    agent.actor.load_state_dict(actor_tensors((ROOT/'rust/crates/sim/data/getup-policy/student.bin').read_bytes()))
    # Preserve deterministic mean; cap exploration std before both rollout/TD target sampling.
    agent.actor.log_std_min=-5.;agent.actor.log_std_max=math.log(.05)
    parity=json.loads((ROOT/'rust/crates/sim/data/getup-policy/parity.json').read_text())
    max_error=0.
    for row in parity['actors']:
        if row['actor']!='student':continue
        old=np.asarray(row['input'],np.float32);x=np.r_[old[:117],np.float32(.1),np.float32(-.2),old[117]].astype(np.float32)
        actual=agent.select_action(x);max_error=max(max_error,float(np.max(np.abs(actual-np.asarray(row['action'])))))
    assert max_error<5e-5
    baseline=copy.deepcopy(agent)
    initsha=export_actor(agent.actor,a.output/'initial-actor.bin')
    config={'seed':SEED,'heldout':HELDOUT,'maxTrainingTransitions':3000,'maxUpdates':1000,
        'criticWarmupUpdates':200,'batchSize':128,'actorLR':1e-6,'criticLR':1e-4,'explorationStdMax':.05,
        'physicsLimitSeconds':360,'totalDeadlineUTC':a.deadline,'evaluationReserveSeconds':100,
        'inputCheckpointSha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        'nativeBinarySha256':hashlib.sha256(a.binary.read_bytes()).hexdigest(),
        'trainerSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'sourceSACSha256':SOURCE_SHA,'initialActorSha256':initsha,'initialParityMaxError':max_error,
        'torchVersion':torch.__version__,'numpyVersion':np.__version__,'threads':2,'gpu':False,
        'timeoutBootstrap':'yes for finite horizon; no for physical failure/termination; zero-integration excluded'}
    (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    if a.validate_only:print(json.dumps(config,indent=2));return
    native=Native(a.binary.resolve(),a.checkpoint.resolve(),a.output.resolve(),deadline)
    replay=Replay(rng);updates=0;steps=0;gradient_steps=0;gradient_norm_max=0.;benchmark={};termination_counts={}
    training_log=(a.output/'training.ndjson').open('w')
    def before_optimizer(opt,args,kwargs):
        nonlocal gradient_steps,gradient_norm_max
        params=[p for group in opt.param_groups for p in group['params'] if p.grad is not None]
        assert params and all(torch.isfinite(p.grad).all() for p in params),'nonfinite gradients'
        norm=torch.nn.utils.clip_grad_norm_(params,1.0,error_if_nonfinite=True)
        gradient_steps+=1;gradient_norm_max=max(gradient_norm_max,float(norm))
    for opt in [agent.actor_optimizer,agent.critic_optimizer,agent.log_alpha_optimizer]:opt.register_step_pre_hook(before_optimizer)
    result={'status':'running'}
    try:
        initial_state=native.command({'op':'observe'})
        native_action=native.command({'op':'initializedAction'})['action']
        native_torch_error=float(np.max(np.abs(np.asarray(native_action)-baseline.select_action(scalar_observation(initial_state)))))
        assert native_torch_error<5e-5
        config['nativeTorchInitializationActionError']=native_torch_error
        config['trainingDirections']='diagonals only; heldout cardinal directions'
        (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
        before=evaluate(native,baseline,'baseline',a.output)
        (a.output/'baseline-metrics.json').write_text(json.dumps(before,indent=2)+'\n')
        state=native.command({'op':'reset'});episode_tick=0;force=[0.,0.]
        while steps<3000:
            # Preserve candidate paired evaluation time and native budget upfront.
            if native.last_wall>=260 or time.time()>=deadline-150:break
            obs=scalar_observation(state)
            action=agent.select_action(obs)
            action=np.clip(action+rng.normal(0,.02,21),-1,1).astype(np.float32)
            if episode_tick==20:
                theta=float(rng.choice([math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4]));magnitude=rng.uniform(5,15)
                force=[float(magnitude*math.cos(theta)),float(magnitude*math.sin(theta))]
            pulse=force if 20<=episode_tick<28 else [0.,0.]
            native_before=native.last_native
            tr=native.command({'op':'step','action':action.tolist(),'torso_force_n':pulse})
            integrated=native.last_native-native_before
            if integrated==0:break
            nxt=tr['state'];failure=tr.get('failure')
            fatal=tr['terminated'] or bool(failure)
            replay.add(obs,action,scalar_observation(nxt),tr['reward']['total'],0. if fatal else 1.)
            steps+=1;episode_tick+=1
            if replay.size>=512:
                target_updates=200 if updates<200 else updates+(1 if steps%3==0 and updates<1000 else 0)
                begin=time.perf_counter();n_before=updates
                while updates<target_updates:
                    warm=updates<200;agent.policy_freq=10**9 if warm else 1
                    agent.train(replay,128)
                    if warm:
                        with torch.no_grad():
                            for x,y in zip(agent.critic.parameters(),agent.critic_target.parameters()):y.lerp_(x,agent.tau)
                    updates+=1
                    assert all(math.isfinite(v[-1]) for v in agent.loss_dict.values() if v),'nonfinite loss'
                    if updates==100:
                        benchmark={'first100CriticUpdatesWallSeconds':time.perf_counter()-begin,
                            'threads':2,'batch':128};(a.output/'benchmark.json').write_text(json.dumps(benchmark,indent=2)+'\n')
                    if time.time()>=deadline-150:break
                agent.reset_record()
            training_log.write(json.dumps({'step':steps,'updates':updates,'reward':tr['reward'],
                'anchorDriftM':nxt['metrics']['anchor_drift_m'],'physicsWallSeconds':native.last_wall,
                'terminated':tr['terminated'],'truncated':tr['truncated'],'failure':failure})+'\n');training_log.flush()
            if fatal or tr['truncated'] or episode_tick>=200:
                reason=failure or ('terminated' if tr['terminated'] else 'time-limit-bootstrap')
                termination_counts[reason]=termination_counts.get(reason,0)+1
                state=native.command({'op':'reset'});episode_tick=0
            else:state=nxt
            if steps%500==0:
                save_checkpoint(agent,replay,rng,native,a.output,steps,updates,{'phase':'training','episodeTick':episode_tick,'currentForce':force})
        save_checkpoint(agent,replay,rng,native,a.output,steps,updates,{'phase':'trained-before-eval','episodeTick':episode_tick,'currentForce':force})
        candidatesha=export_actor(agent.actor,a.output/'candidate-actor.bin')
        probe_state=native.command({'op':'observe'})
        native_export=native.command({'op':'actorAction','path':str((a.output/'candidate-actor.bin').resolve())})['action']
        native_export_error=float(np.max(np.abs(np.asarray(native_export)-agent.select_action(scalar_observation(probe_state)))))
        assert native_export_error<5e-5
        anchor_norm=float(agent.actor.trunk[0].weight[:,117:119].detach().norm())
        after=evaluate(native,agent,'candidate',a.output)
        (a.output/'candidate-metrics.json').write_text(json.dumps(after,indent=2)+'\n')
        eval_complete=all(row['decisions']==row['case']['ticks'] or row['stopped']=='balance-episode-ended: No further integration admitted' for row in before+after)
        result={'status':'completed' if eval_complete else 'incomplete-evaluation','trainingTransitions':steps,'updates':updates,'criticOnlyUpdates':min(updates,200),
            'actorUpdates':max(0,updates-200),'finiteGradientOptimizerSteps':gradient_steps,'maximumPreclipGradientNorm':gradient_norm_max,
            'anchorColumnWeightNorm':anchor_norm,'candidateActorSha256':candidatesha,'initialActorSha256':initsha,
            'terminationCounts':termination_counts,'baseline':before,'candidate':after,'benchmark':benchmark,
            'nativeCandidateExportActionError':native_export_error,'initialMeanActionParityMaxError':max_error,'candidateExportRoundtripExact':True,
            'pairedEvaluation':'all four predeclared cases, same initial checkpoint/force schedule/deterministic actor',
            'trainedPolicyIsExperimental':True,'defaultApplicationControllerChanged':False}
    except Exception as e:
        result={'status':'failed','error':repr(e),'trainingTransitions':steps,'updates':updates}
        try:save_checkpoint(agent,replay,rng,native,a.output,steps,updates,result)
        except Exception as save_error:result['checkpointError']=repr(save_error)
        raise
    finally:
        native.close();training_log.close()
        result['physicsWallSeconds']=native.last_wall;result['nativeIntegrations']=native.last_native
        result['newPhysicsAllowanceSeconds']=360;result['remainingPhysicsSeconds']=360-native.last_wall
        result['utcFinished']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ['baseline','candidate']},indent=2))

if __name__=='__main__':main()
