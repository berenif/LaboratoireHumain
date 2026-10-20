#!/usr/bin/env python3
"""Corrected interleaved SAC runner. Missing retained inputs block before startup.

The original rejected pilot and all its receipts remain unchanged. Synthetic unit
fixtures are never admitted by this command's retained-input preflight.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import select
import subprocess
import sys
import time
from dataclasses import asdict
from getup_learning_loop import (CAPS, CATEGORIES, ROLES, ContractError, EpisodeReplay,
    FixedWitness, Gaussian, LoopPhase, PauseReceipt, Record, UpdateGuard, preflight,
    require_execution_ready, remaining_seconds)

ROOT = Path(__file__).resolve().parents[1]
GAMMA = math.exp(-.025 / 25)

class AbsoluteWatchdog:
    """A separate CPU process can stop an in-flight synchronous WASM call.

    Closing its pipe disarms it. It is armed before importing/constructing models;
    the 100ms safety margin is inside, never beyond, the approved deadline.
    """
    PROGRAM = '''import json,os,select,signal,sys,time
deadline=float(sys.argv[1])-.1; parent=int(sys.argv[2]); children=[]
while True:
 remaining=deadline-time.time()
 if remaining<=0:break
 if select.select([sys.stdin],[],[],remaining)[0]:
  line=sys.stdin.readline()
  if not line:
   for pid in children:
    try:os.kill(pid,signal.SIGKILL)
    except ProcessLookupError:pass
   sys.exit(0)
  children.append(int(line))
for pid in children+[parent]:
 try:os.kill(pid,signal.SIGKILL)
 except ProcessLookupError:pass
'''
    def __init__(self, deadline):
        import os
        self.proc = subprocess.Popen([sys.executable,'-c',self.PROGRAM,str(deadline),str(os.getpid())],
            stdin=subprocess.PIPE,text=True)
    def register(self, pid):
        self.proc.stdin.write(str(pid)+'\n');self.proc.stdin.flush()
        self.assert_alive()
    def assert_alive(self):
        if self.proc.poll() is not None:raise ContractError('Absolute-deadline watchdog exited')
    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=2)

class BoundedWasm:
    """Reuse the existing bridge/process, with a deadline-aware JSONL wait."""
    def __init__(self, legacy, output, worker, deadline, watchdog):
        self.inner=legacy.Wasm(output,worker);self.proc=self.inner.proc;self.deadline=deadline
        self.watchdog=watchdog
        watchdog.register(self.proc.pid)
    def call(self, request):
        self.watchdog.assert_alive()
        timeout=min(120.,remaining_seconds(self.deadline,time.time()))
        self.proc.stdin.write(json.dumps(request)+'\n');self.proc.stdin.flush()
        if not select.select([self.proc.stdout],[],[],timeout)[0]:
            self.proc.kill()
            raise ContractError('Bounded WASM response timeout; no retry or restart')
        line=self.proc.stdout.readline()
        if not line:raise ContractError('Bounded WASM bridge closed')
        remaining_seconds(self.deadline,time.time())
        value=json.loads(line)
        if not value['ok']:raise ContractError(json.dumps(value))
        return value['result']
    def close(self):
        try:
            self.proc.stdin.close();self.proc.wait(timeout=min(2.,max(.001,self.deadline-time.time())))
        except (BrokenPipeError,subprocess.TimeoutExpired):
            self.proc.kill();self.proc.wait(timeout=2)

class RecoveryHooks:
    """Use the shared durable-backup API; a local archive is insufficient."""
    def __init__(self, manifest, paths, manifest_path):
        from experiment_backup import LibraryBackend, RecoveryGuard, require_recovery_point
        settings=manifest.get('durable_recovery',{})
        required=('root','run_id','receipt','library_helpers','proof_directory','source')
        if any(not settings.get(key) for key in required):
            raise ContractError('Verified private durable recovery configuration is required')
        if settings['source'].get('dirty') is not False:
            raise ContractError('Commit the reviewed runner before a costly run; no unbacked dirty source')
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        if commit!=settings['source'].get('commit') or subprocess.check_output(
                ['git','status','--porcelain','--untracked-files=normal'],cwd=ROOT,text=True).strip():
            raise ContractError('Durable source identity differs from the clean executable checkout')
        backend=LibraryBackend(settings['library_helpers'],settings['proof_directory'])
        proof=require_recovery_point(settings['receipt'],manifest_path,ROLES,backend,run_id=settings['run_id'])
        self.guard=RecoveryGuard(settings['root'],settings['run_id'],backend,settings['source'],synthetic=False)
        self.guard.previous=proof['manifest_sha256']
        self.dependencies=settings.get('excluded_dependencies',[])
        self.artifacts=[{'role':'input_manifest','path':str(manifest_path),'schema':'retained-input/v1',
            'provenance':'Exact reviewed input mapping, no synthetic substitution','retention_class':'user_generated'}]
        for role,path in paths.items():
            item=manifest['files'][role]
            if item.get('retention_class') not in ('user_generated','project_source'):
                raise ContractError(f'Retention rights/allowlist is unresolved for {role}')
            self.artifacts.append({'role':role,'path':path,'schema':item.get('schema'),
                'provenance':item.get('provenance'),'retention_class':item['retention_class']})

    def checkpoint(self, output, config, provenance, deadline):
        local=('learner-state.pt','replay-local.npz','replay-metadata-local.json','replay-rng-local.json',
               'loop-state-local.json','learning-loop-report.json','frozen-config.json','budget-ledger.json',
               'current-world-checkpoint.bin')
        artifacts=self.artifacts+[{'role':'live_'+name.replace('.','_'), 'path':str(output/name),
            'schema':'corrected-learning-loop/v1','provenance':provenance,'retention_class':'user_generated'} for name in local]
        if (output/'transitions.ndjson').exists():
            artifacts.append({'role':'fresh_transitions','path':str(output/'transitions.ndjson'),
                'schema':'actual-WASM-transitions/v1','provenance':provenance,'retention_class':'user_generated'})
        return self.guard.checkpoint(artifacts,config,20261020,provenance,
            required_roles=[item['role'] for item in artifacts],excluded_dependencies=self.dependencies,deadline=deadline)

def load_legacy():
    path = ROOT / 'scripts/train-getup-wasm-adaptation.py'
    spec = importlib.util.spec_from_file_location('closed_pilot_interfaces', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def validate_episode(episode, lines, policies):
    """Canonical mapping contract; real lost audit-schema adaptation is gated."""
    first, last = episode['first_line'], episode['last_line']
    if type(first) is not int or type(last) is not int or not 0 <= first <= last < len(lines):
        raise ContractError('Episode raw line boundaries are invalid')
    rows = lines[first:last+1]
    if episode['behavior_policy_id'] not in policies:
        raise ContractError('Episode behavior policy has no verified identity')
    if episode['case'] not in ('original','belly','heldout'):
        raise ContractError('Unknown retained physical case')
    if episode['case']=='original' and episode['seed_sha256'] != 'c49445348d9e96543f283c8d9a318241c24ffbb0a03f938e2490bd2167ec6ed6':
        raise ContractError('BACK replay must use the frozen original-trigger seed')
    if rows[0]['preNative'] != episode['source_start_native']+20:
        raise ContractError('Episode omits or relabels its first controlled decision')
    for index, row in enumerate(rows):
        if row['label'] != episode['label'] or row['postNative'] != row['preNative']+20:
            raise ContractError('Raw segment changes episode or native publication step')
        if index and rows[index-1]['postNative'] != row['preNative']:
            raise ContractError('Raw episode is not exactly native-contiguous')
        if type(row['terminal']) is not bool or type(row['success']) is not bool:
            raise ContractError('Raw terminal receipt is not Boolean')
        if row['terminal'] != (row['success'] or bool(row['failure'])) or row['success'] and row['failure']:
            raise ContractError('Raw terminal outcome is inconsistent')
        if index<len(rows)-1 and row['terminal']:
            raise ContractError('Episode continues past its training terminal')
        if row['preNative'] < episode['impact_native'] or not math.isclose(
            row['status']['since_impact_s'],(row['postNative']-episode['impact_native'])/800,abs_tol=1e-10):
            raise ContractError('Episode impact/recovery clock differs from raw status')
    endpoint=rows[-1]
    outcome='success' if endpoint['success'] else 'failure' if endpoint['failure'] else 'truncation'
    if episode['outcome'] != outcome or episode['complete'] is not bool(endpoint['terminal']):
        raise ContractError('Episode outcome/completeness differs from its real endpoint')
    if endpoint['success'] and not (endpoint['status']['prototype_standing_passed']
            and endpoint['status']['stable_standing_s']>=1.
            and episode['impact_native']+800<=endpoint['postNative']<=episode['impact_native']+20000):
        raise ContractError('Successful episode lacks the original800 predicate receipt')
    if endpoint['failure'] and endpoint['failure'].get('check')=='original25s-deadline' and endpoint['postNative']!=episode['impact_native']+20000:
        raise ContractError('Deadline failure was relabeled from an administrative truncation')
    return rows

def load_seed_records(manifest, paths):
    """Use the verified old replay and line/episode mapping; never invent labels."""
    import numpy as np
    arrays = np.load(paths['replay'], allow_pickle=False)
    metadata = json.loads(Path(paths['replay_metadata']).read_text())
    lines = [json.loads(line) for line in Path(paths['transitions']).open()]
    if metadata.get('schema') != 1 or metadata.get('source_replay_sha256') != manifest['files']['replay']['sha256']:
        raise ContractError('Replay metadata does not name the verified replay')
    count = len(arrays['0'])
    covered=set()
    for episode in manifest['episodes'].values():
        validate_episode(episode,lines,manifest['behavior_policies'])
        span=set(range(episode['first_line'],episode['last_line']+1))
        if covered.intersection(span):raise ContractError('Retained episode raw ranges overlap')
        covered.update(span)
    mapping = metadata.get('records', [])
    if sorted(row.get('replay_index', -1) for row in mapping) != list(range(count)):
        raise ContractError('Every retained row must be accounted for exactly once')
    if arrays['0'].shape != (count, 1024) or arrays['2'].shape != (count, 1024) or arrays['1'].shape != (count, 21):
        raise ContractError('Old frozen-feature replay shape mismatch')
    if any(arrays[str(i)].dtype != np.dtype('float32') for i in range(5)) or any(arrays[str(i)].shape!=(count,1) for i in (3,4)):
        raise ContractError('Retained replay dtype or reward/bootstrap shape mismatch')
    if not all(np.isfinite(arrays[str(i)]).all() for i in range(5)):
        raise ContractError('Nonfinite retained replay')
    result, excluded = [], []
    for info in sorted(mapping, key=lambda item: item['replay_index']):
        index = info['replay_index']
        if info.get('verified') is not True:
            if not info.get('exclusion_reason'):
                raise ContractError('Unverified replay requires an explicit exclusion reason')
            excluded.append({'index': index, 'reason': info['exclusion_reason']})
            continue
        episode = manifest['episodes'].get(info['episode_id'])
        behavior = manifest['behavior_policies'].get(info['behavior_policy_id'])
        if not episode or not behavior or info.get('uncertain_execution') or info['behavior_policy_id'] != episode['behavior_policy_id']:
            raise ContractError('Missing episode/behavior identity or uncertain execution')
        start, end = info['transition_line_start'], info['transition_line_end']
        if not episode['first_line'] <= start <= end <= episode['last_line']:
            raise ContractError('Retained transition range is invalid')
        segment = lines[start:end+1]
        if any(row['label'] != episode['label'] for row in segment):
            raise ContractError('Transition segment crosses episode/source identity')
        first, last = segment[0], segment[-1]
        if first['preObservation'] is None or len(first['preObservation']) != 118:
            raise ContractError('Retained transition has no current118-input source state')
        phase = 'holding' if first['referenceDecision'] >= first['status']['reference_frames'] else 'pre_holding'
        if info['phase'] != phase:
            raise ContractError('Retained phase label differs from reference clock')
        total = sum(GAMMA**step * row['reward'] for step, row in enumerate(segment))
        if arrays['3'][index, 0].tobytes() != np.float32(total).tobytes():
            raise ContractError('Retained discounted reward chronology mismatch')
        terminal = bool(last['terminal'])
        if float(arrays['4'][index, 0]) != float(not terminal):
            raise ContractError('Retained terminal/bootstrap mask mismatch')
        if not np.array_equal(arrays['1'][index], np.asarray(first['correction'], np.float32)):
            raise ContractError('Retained action is not the applied correction c')
        if max(abs(a-b) for a,b in zip(first['appliedResidual'], first['originalResidual'])) > .050001:
            raise ContractError('Retained applied residual violates original correction bound')
        category = info['category']
        baseline_success = episode.get('baseline_success') is True
        if category == 'baseline_preservation' and not baseline_success:
            raise ContractError('Preservation must name a verified baseline-success episode')
        if category != 'baseline_preservation' and episode['case'] != 'original':
            raise ContractError('BACK replay categories exclude nearby/BELLY cases')
        if baseline_success and (not episode['complete'] or episode['outcome']!='success'
                or behavior.get('kind')!='baseline_deterministic'):
            raise ContractError('Baseline preservation identity/outcome is unverified')
        if category=='baseline_preservation' and any(any(c!=0 for c in row['correction']) for row in segment):
            raise ContractError('Baseline preservation contains a learned/exploratory correction')
        if any(row['preObservation'] is not None for row in segment[1:]):
            raise ContractError('Replay segment swallows a later source decision')
        if len(segment)>1 and not terminal:
            raise ContractError('A folded standing tail must retain its actual terminal')
        result.append(Record(info['episode_id'], category, phase, info['behavior_policy_id'],
            tuple(float(v)*32 for v in arrays['0'][index]), tuple(map(float,arrays['1'][index])),
            tuple(float(v)*32 for v in arrays['2'][index]), total,
            0. if terminal else GAMMA**len(segment), terminal,
            bool(info.get('administrative_truncation')), first['preNative'], last['postNative'],
            episode['impact_native'], episode['complete'], 'retained_verified',
            episode['outcome'], baseline_success).validate())
    return result, excluded

class SACLearner:
    """Reuse the closed pilot's actor/critics; change cadence/sampling/guards only."""
    def __init__(self, legacy):
        import numpy as np
        import torch
        self.np, self.torch = np, torch
        torch.set_num_threads(2)
        random.seed(20261020); np.random.seed(20261020); torch.manual_seed(20261020)
        self.actor, self.critic = legacy.CorrectionActor(), legacy.Critic()
        self.target = copy.deepcopy(self.critic)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=1e-4)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=1e-4)
        self.log_alpha = torch.tensor(math.log(.01), requires_grad=True)
        self.alpha_optimizer = torch.optim.Adam([self.log_alpha], lr=1e-4)
        self.learning_updates = 0

    def policy_id(self):
        h = hashlib.sha256(b'frozen118-correction-head-v1')
        for key, value in sorted(self.actor.state_dict().items()):
            h.update(key.encode()); h.update(value.detach().cpu().numpy().tobytes())
        return h.hexdigest()

    def gaussians(self, features):
        t = self.torch
        with t.no_grad():
            x = t.tensor(features, dtype=t.float32)/32
            mean = self.actor.mean(x).double().tolist()
            logstd = self.actor.log_std.clamp(-5., -2.).double().tolist()
        return [Gaussian(tuple(row), tuple(logstd)) for row in mean]

    def action(self, features, deterministic=False):
        t = self.torch
        with t.no_grad():
            output = self.actor(t.tensor(features, dtype=t.float32).reshape(1,-1)/32,
                                sample=not deterministic)
        return output[0 if deterministic else 1][0].numpy().tolist()

    def batch(self, replay):
        ids = replay.batch_indices(128)
        rows = [replay.rows[i] for i in ids]
        t = self.torch
        values = ([(x.features) for x in rows], [x.correction for x in rows],
                  [x.next_features for x in rows], [[x.reward] for x in rows],
                  [[x.discount] for x in rows])
        tensors = [t.tensor(value, dtype=t.float32) for value in values]
        tensors[0] /= 32; tensors[2] /= 32
        preservation = t.tensor([x.category == 'baseline_preservation' for x in rows])
        return (*tensors, preservation)

    def critic_attempt(self, batch):
        t = self.torch
        s,c,n,r,discount,preservation = batch
        with t.no_grad():
            _, nc, lp = self.actor(n)
            a,b = self.target(n,nc)
            target = r + discount*(t.minimum(a,b)-self.log_alpha.exp().detach()*lp)
        a,b = self.critic(s,c)
        loss = t.nn.functional.mse_loss(a,target)+t.nn.functional.mse_loss(b,target)
        self.critic_optimizer.zero_grad(); loss.backward()
        t.nn.utils.clip_grad_norm_(self.critic.parameters(),10.); self.critic_optimizer.step()
        return {'critic':float(loss.detach()), 'q_mean':float(t.minimum(a,b).mean().detach()),
                'target_mean':float(target.mean())}

    def joint_attempt(self, batch):
        t = self.torch
        metrics = self.critic_attempt(batch)
        s,c,n,r,discount,preservation = batch
        mu,pc,lp = self.actor(s); a,b = self.critic(s,pc)
        preserve = mu[preservation].square().mean()
        loss = (self.log_alpha.exp().detach()*lp-t.minimum(a,b)).mean()+10*preserve
        self.actor_optimizer.zero_grad(); loss.backward()
        t.nn.utils.clip_grad_norm_(self.actor.parameters(),1.); self.actor_optimizer.step()
        alpha_loss = (self.log_alpha.exp()*(-lp+21).detach()).mean()
        self.alpha_optimizer.zero_grad(); alpha_loss.backward(); self.alpha_optimizer.step()
        self.polyak(); self.learning_updates += 1
        if not self.finite():
            raise ContractError('Nonfinite learning state')
        metrics.update(actor=float(loss.detach()), preservation=float(preserve.detach()),
                       alpha=float(self.log_alpha.exp().detach()))
        return metrics

    def polyak(self):
        with self.torch.no_grad():
            for source, target in zip(self.critic.parameters(),self.target.parameters()):
                target.lerp_(source,.005)

    def finite(self):
        return (self.torch.isfinite(self.log_alpha).all().item()
            and all(self.torch.isfinite(p).all().item() for model in (self.actor,self.critic,self.target)
                    for p in model.parameters()))

    def capture_learning_state(self):
        models = {'actor':self.actor, 'critic':self.critic, 'target':self.target}
        return copy.deepcopy({'models':{k:v.state_dict() for k,v in models.items()},
            'gradients':{k:[None if p.grad is None else p.grad.detach().clone() for p in v.parameters()]
                         for k,v in models.items()}, 'modes':{k:v.training for k,v in models.items()},
            'optimizers':{k:getattr(self,k).state_dict() for k in
                          ('actor_optimizer','critic_optimizer','alpha_optimizer')},
            'log_alpha':self.log_alpha.detach().clone(),
            'alpha_gradient':None if self.log_alpha.grad is None else self.log_alpha.grad.detach().clone(),
            'learning_updates':self.learning_updates, 'torch_rng':self.torch.get_rng_state(),
            'numpy_rng':self.np.random.get_state(), 'python_rng':random.getstate()})

    def restore_learning_state(self, state):
        for key in ('actor','critic','target'):
            model = getattr(self,key); model.load_state_dict(state['models'][key]);model.train(state['modes'][key])
            for parameter, gradient in zip(model.parameters(),state['gradients'][key]):
                parameter.grad = None if gradient is None else gradient.clone()
        for key,value in state['optimizers'].items():getattr(self,key).load_state_dict(value)
        with self.torch.no_grad():self.log_alpha.copy_(state['log_alpha'])
        self.log_alpha.grad = None if state['alpha_gradient'] is None else state['alpha_gradient'].clone()
        self.learning_updates = state['learning_updates']
        self.torch.set_rng_state(state['torch_rng']);self.np.random.set_state(state['numpy_rng']);random.setstate(state['python_rng'])

class ExistingWasmEnvironment:
    """The existing JSONL bridge remains the sole physical runtime owner."""
    def __init__(self, legacy, output, worker, deadline, watchdog):
        self.wasm = BoundedWasm(legacy, output, worker, deadline, watchdog)
        self.output = output
        self.state = None
        self.episode = 0
        self.builder = None
        self.pending_origin = None
        self.complete_episodes = 0
        self.legacy = legacy

    def reset_original(self):
        self.episode += 1
        self.episode_id = f'fresh-original-{self.episode:03d}'
        self.state = self.wasm.call({'op':'reset','case':'original','external':True,
                                    'label':self.episode_id})
        self.builder = self.legacy.SourceTransition(GAMMA)

    def pause(self, name):
        state = self.wasm.call({'op':'state'})
        receipt = self.wasm.call({'op':'checkpoint','name':name})
        return PauseReceipt(self.episode_id,state['native'],state['impactEpoch'],
            state['sourceEpoch'],state['status']['reference_index'],state['status']['phase'],receipt['sha256'])

    def collect_block(self, learner, replay, phase, wall_check):
        policy = learner.policy_id()
        phase.begin_collection(policy)
        recent = []
        while phase.phase == 'collection' or self.builder is not None and self.builder.pending is not None:
            wall_check()
            if self.state is None:self.reset_original()
            preview = self.state.get('preview')
            source = self.state
            action = learner.action(preview['features']) if preview else None
            if learner.policy_id() != policy:raise ContractError('Weights changed within collection block')
            result = self.wasm.call({'op':'step','c':action,'training':bool(preview),
                'episodeId':self.episode_id,'policyId':policy})
            transition = self.builder.push(source,action,result)
            if preview:
                phase.collected(policy)
                self.pending_origin = (source, policy, action)
            if transition:
                origin, behavior, correction = self.pending_origin
                holding = origin['preview']['referenceDecision'] >= origin['status']['reference_frames']
                current = Record(self.episode_id, CATEGORIES[1] if holding else CATEGORIES[0],
                    'holding' if holding else 'pre_holding', behavior, tuple(transition[0]),
                    tuple(correction),tuple(transition[2]),transition[3],
                    0. if transition[4] else GAMMA**((result['native']-origin['native'])//20), bool(transition[4]),False,
                    origin['native'],result['native'],origin['impactEpoch'],bool(result['terminal']),
                    'fresh_wasm','success' if result['success'] else 'failure' if result['terminal'] else 'pending',False)
                recent.append(replay.add(current))
            self.state = result['state']
            if result['terminal']:
                self.complete_episodes += 1
                replay.finish_episode(self.episode_id,'success' if result['success'] else 'failure')
                self.state = None
            if result['failure'] and result['failure'].get('check') != 'original25s-deadline':
                raise ContractError('Physical integrity failure during collection')
            # A handoff tail has no new source decisions. Finish it to keep the
            # last action's terminal reward, without renewing its recovery clock.
            if phase.phase == 'updates' and self.builder.pending is None:break
        if len(recent) != 256:
            raise ContractError('Collection block must retain all256 fresh decision transitions')
        return recent, policy

    def close(self):self.wasm.close()

def run(manifest, paths, output, manifest_path=None):
    require_execution_ready()
    records, excluded = load_seed_records(manifest, paths)
    replay = EpisodeReplay(records)
    if output.exists() and any(output.iterdir()):raise ContractError('No experiment restart or output reuse')
    if manifest_path is None:raise ContractError('Exact retained input manifest path is required')
    recovery=RecoveryHooks(manifest,paths,manifest_path)
    # All required input/provenance checks precede runtime/model/deadline startup.
    output.mkdir(parents=True,exist_ok=True)
    started = time.time(); deadline = started+3600
    config = {'wallSeconds':3600,'nativeIntegrations':300000,'gamma':GAMMA,
        'inputContract':{'checkpoints':{'original':paths['original_seed'],'heldout':paths['nearby_seed'],'belly':paths['belly_seed']},
         'baselineArtifactDirectory':str(Path(paths['baseline_wasm']).parent),
         'profile':paths['profile'],'files':{role:{'path':path,'sha256':manifest['files'][role]['sha256']} for role,path in paths.items()},
         'startedAtUnixMs':round(started*1000),'deadlineUnixMs':round(deadline*1000)}}
    (output/'frozen-config.json').write_text(json.dumps(config,indent=2)+'\n')
    report={'schema':1,'startedAtUnix':started,'deadlineUnix':deadline,'caps':CAPS,
            'trainingExecuted':False,'physicalExperimentExecuted':False,'excludedRetained':excluded,
            'accepted':0,'rejected':0,'blocks':[],'candidateAccepted':False,'releaseAccepted':False}
    reserve = manifest['evaluation_reserve_integrations']
    reserve_wall = reserve/manifest['measured_native_per_wall_second']+180
    protected_wall=reserve_wall
    def wall_check():
        watchdog.assert_alive()
        remaining_seconds(deadline,time.time(),protected_wall)
    learner=environment=None;phase=LoopPhase();watchdog=AbsoluteWatchdog(deadline);retention_failed=False
    def save_local(boundary):
        report['phase']=vars(phase);report['completeFreshEpisodes']=environment.complete_episodes
        report['ledger']=environment.wasm.call({'op':'ledger'})
        learner.torch.save(learner.capture_learning_state(),output/'learner-state.pt')
        learner.np.savez_compressed(output/'replay-local.npz',
            features=learner.np.asarray([row.features for row in replay.rows],dtype='float32'),
            correction=learner.np.asarray([row.correction for row in replay.rows],dtype='float32'),
            next_features=learner.np.asarray([row.next_features for row in replay.rows],dtype='float32'))
        metadata=[{k:v for k,v in asdict(row).items() if k not in ('features','correction','next_features')} for row in replay.rows]
        (output/'replay-metadata-local.json').write_text(json.dumps(metadata)+'\n')
        (output/'replay-rng-local.json').write_text(json.dumps(replay.rng.getstate())+'\n')
        if environment.episode:
            environment.wasm.call({'op':'checkpoint','name':'current-world-checkpoint.bin'})
            authoritative=environment.wasm.call({'op':'state'})
        else:(output/'current-world-checkpoint.bin').write_bytes(Path(paths['original_seed']).read_bytes())
        if not environment.episode:authoritative=None
        cursor_mode=('evaluation' if getattr(environment,'episode_id','').startswith('final-') else
                     'collection_active' if environment.state is not None else 'collection_awaiting_reset')
        state={'schema':1,'boundary':boundary,'phase':vars(phase),'absoluteStartedUnix':started,'absoluteDeadlineUnix':deadline,
            'episodeCounter':environment.episode,'episodeId':getattr(environment,'episode_id',None),
            'completeEpisodes':environment.complete_episodes,'cachedPhysicalState':environment.state,
            'authoritativePhysicalState':authoritative,'cursorMode':cursor_mode,
            'pendingTransition':environment.builder.pending if environment.builder else None,
            'pendingOrigin':environment.pending_origin,'bridge':environment.wasm.call({'op':'bridgeState'}),
            'replayCapacity':replay.capacity,'restartAuthorized':False}
        if environment.state is not None and environment.state!=authoritative:
            raise ContractError('Python cached state differs from the exact paused bridge state')
        (output/'loop-state-local.json').write_text(json.dumps(state,default=lambda x:x.tolist())+'\n')
        report['elapsedWallSeconds']=time.time()-started
        (output/'learning-loop-report.json').write_text(json.dumps(report,indent=2)+'\n')
    def retain_boundary(boundary):
        nonlocal retention_failed
        wall_check();save_local(boundary)
        try:
            proof=recovery.checkpoint(output,config,f'Coherent paused {boundary}; world/controller clocks unchanged; real WASM',deadline-protected_wall)
        except BaseException:
            retention_failed=True
            raise
        report.setdefault('recoveryPoints',[]).append({'boundary':boundary,'proof':str(proof)})
        wall_check()
    try:
        wall_check();legacy=load_legacy();wall_check();learner=SACLearner(legacy)
        environment=ExistingWasmEnvironment(legacy,output,Path(paths['worker_wasm']).parent,deadline,watchdog)
        environment.wasm.call({'op':'reserve','integrations':reserve,'wallSeconds':reserve_wall})
        while phase.phase == 'warmup':
            wall_check();learner.critic_attempt(learner.batch(replay));learner.polyak()
            report['trainingExecuted']=True
            if not learner.finite():raise ContractError('Nonfinite critic warmup')
            phase.warmup_done()
        retain_boundary('critic-warmup-complete')
        while phase.phase == 'ready_collection':
            report['physicalExperimentExecuted']=True
            recent, policy = environment.collect_block(learner,replay,phase,wall_check)
            before = environment.pause(f'block-{phase.cycle:02d}-before.bin')
            retain_boundary(f'collection-{phase.cycle:02d}-complete')
            ids = replay.witness_indices(recent);features=[replay.rows[i].features for i in ids]
            witness=FixedWitness(features,learner.gaussians(features),policy)
            block={'cycle':phase.cycle,'collectingPolicy':policy,'witness':witness.sha256,
                   'witnessIndices':ids,'attempts':[],'pause':asdict(before)}
            guard=UpdateGuard(learner,replay,phase,witness)
            for _ in range(64):
                wall_check()
                result,metrics,accepted=guard.attempt(lambda:learner.batch(replay))
                report['accepted' if accepted else 'rejected']+=1
                block['attempts'].append({'accepted':accepted,**asdict(result),'metrics':metrics})
            before.unchanged(environment.pause(f'block-{phase.cycle:02d}-after.bin'))
            report['blocks'].append(block);phase.finish_block()
            retain_boundary(f'updates-{phase.cycle:02d}-complete')
            report['phase']=vars(phase);(output/'learning-loop-report.json').write_text(json.dumps(report,indent=2)+'\n')
        if environment.state is not None:
            replay.finish_episode(environment.episode_id,'truncation',truncated=True)
            report['administrativelyTruncatedEpisode']=environment.episode_id
        if phase.phase=='stopped_no_actor_acceptance':
            report['stopped']='block-accepted-no-actor-update'
            return
        # Final candidate is singular and deterministic. Original BACK is always
        # evaluated first; zero diagnostic interval stops without wider search.
        protected_wall=0.;wall_check()
        environment.wasm.call({'op':'reserve','integrations':0,'wallSeconds':0})
        head=learner.actor.exported();(output/'candidate-head.json').write_text(json.dumps(head)+'\n')
        parity=0.
        for index in ids:
            features=replay.rows[index].features
            actual=environment.wasm.call({'op':'headparity','head':head,'features':features})
            if actual['physicsIntegrations']!=0:raise ContractError('Pure export check advanced physics')
            expected=learner.action(features,deterministic=True)
            parity=max(parity,max(abs(a-b) for a,b in zip(actual['correction'],expected)))
        if parity>2e-6:raise ContractError('Pure WASM correction-head export parity failed')
        report['pureExportMaximumCorrectionError']=parity
        report['final']=[]
        for case in ('original','heldout','belly'):
            state=environment.wasm.call({'op':'reset','case':case,'label':'final-'+case,'head':head,'prepared':True})
            environment.state=state;environment.episode_id='final-'+case;environment.builder=None;environment.pending_origin=None
            first_success=None;first_success_receipt=None;rollout_error=0.;hold=None
            while True:
                wall_check();result=environment.wasm.call({'op':'step'})
                if state.get('preview'):
                    receipt=result['state']['lastCorrection']
                    if receipt['native_substep']!=state['native']:raise ContractError('Missing applied correction receipt')
                    expected=learner.action(state['preview']['features'],deterministic=True)
                    rollout_error=max(rollout_error,max(abs(a-b) for a,b in zip(receipt['correction'],expected)))
                    if rollout_error>2e-6:raise ContractError('Applied WASM export parity failed')
                state=result['state']
                environment.state=state
                if result['success'] and first_success is None:
                    first_success=result['native'];first_success_receipt=environment.wasm.call({'op':'standingReceipt'})
                    hold=environment.wasm.call({'op':'hold','firstSuccessReceipt':first_success_receipt})
                    if not hold['passed']:raise ContractError('Original anchored hold failed')
                    state=environment.wasm.call({'op':'state'})
                    environment.state=state
                    result['longestMeasuredInterval']=hold['longestMeasuredInterval']
                    break
                if result['failure'] or state['status']['since_impact_s']>=25:break
            row={k:result[k] for k in ('success','failure','longestMeasuredInterval','native')};row['case']=case
            row.update(first800GateNative=first_success,diagnosticEndpointNative=state['impactEpoch']+20000,
                       finalPhysicalNative=state['native'],maximumAppliedCorrectionError=rollout_error,hold=hold)
            report['final'].append(row)
            if result['failure'] and result['failure'].get('check')!='original25s-deadline':raise ContractError('Final physical integrity failure')
            if not result['success'] or (case=='original' and result['longestMeasuredInterval']==0):
                report['stopped']='final-original-or-preservation-failure';break
        report['stopped']=report.get('stopped','single-final-candidate-evaluated-no-promotion')
        report['physicalQualificationPassed']=(len(report['final'])==3
            and all(row['success'] and row['hold'] and row['hold']['passed'] for row in report['final']))
    except BaseException as error:
        report['stopped']='integrity-resource-or-contract-stop';report['error']=str(error)
        raise
    finally:
        report['phase']=vars(phase);report['completeFreshEpisodes']=environment.complete_episodes if environment else 0
        if environment:
            try:report['ledger']=environment.wasm.call({'op':'ledger'})
            except BaseException:
                ledger_path=output/'budget-ledger.json'
                report['ledger']=json.loads(ledger_path.read_text()) if ledger_path.exists() else {'unavailable':True}
        if learner and environment:
            try:
                save_local('terminal-stop')
                if not retention_failed:
                    recovery.checkpoint(output,config,'Terminal experiment state; no automatic restart',deadline=deadline)
                else:report['terminalRemoteRetentionSuppressed']='Prior failed/uncertain durable operation; no automatic retry'
            except BaseException as error:report['terminalRecoveryError']=str(error)
        report['elapsedWallSeconds']=time.time()-started
        (output/'learning-loop-report.json').write_text(json.dumps(report,indent=2)+'\n')
        if environment:environment.close()
        watchdog.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    try:
        manifest,paths=preflight(args.manifest)
        if not args.run:
            print(json.dumps({'hashPreflightPassed':True,'auditAdapterVerified':False,
                'experimentRunnable':False,'experimentStarted':False,'trainingExecuted':False}));return
        if args.output is None:raise ContractError('--output is required for an explicitly requested run')
        run(manifest,paths,args.output.resolve(),args.manifest.resolve())
    except (ContractError,FileNotFoundError) as error:
        print(json.dumps({'blocked':str(error),'experimentStarted':False,'trainingExecuted':False}),file=sys.stderr)
        raise SystemExit(2)
if __name__=='__main__':main()
