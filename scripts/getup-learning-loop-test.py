#!/usr/bin/env python3
"""Pure synthetic contract tests. No model, training, WASM or physical calls."""
import copy
import ast
from dataclasses import replace
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

spec=importlib.util.spec_from_file_location('getup_learning_loop',Path(__file__).with_name('getup_learning_loop.py'))
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
spec=importlib.util.spec_from_file_location('loop_runner',Path(__file__).with_name('train-getup-learning-loop.py'))
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)

def row(episode='one',category='back_pre_holding',**kwargs):
    values=dict(episode=episode,category=category,phase='holding' if category=='back_holding' else 'pre_holding',behavior='synthetic-only',features=(0.,)*1024,correction=(0.,)*21,next_features=(1.,)*1024,reward=0.,discount=.999000499833375,terminal=False,truncated=False,pre_native=500,post_native=520,impact_native=480,complete_episode=True,source='retained_verified',episode_outcome='success',baseline_success=category=='baseline_preservation')
    values.update(kwargs);return m.Record(**values)

def replay():return m.EpisodeReplay([row(),row('hold','back_holding'),row('belly','baseline_preservation')])
def gaussian(mean=0.,logstd=-3.):return m.Gaussian((mean,)*21,(logstd,)*21)
def phase_updates():
    p=m.LoopPhase()
    for _ in range(256):p.warmup_done()
    p.begin_collection('collecting')
    for _ in range(256):p.collected('collecting')
    return p

class FakeLearner:
    def __init__(self):
        self.state={'actor':0.,'critic':1,'target':2,'actor_optimizer':{'step':7,'momentum':[3]},'critic_optimizer':{'step':9},'alpha_optimizer':{'step':5},'alpha':.01,'gradients':[4,5],'cpu_rng':(2,3),'numpy_rng':(8,9),'python_rng':(1,2),'learning_counter':12}
    def capture_learning_state(self):return copy.deepcopy(self.state)
    def restore_learning_state(self,value):self.state=copy.deepcopy(value)
    def joint_attempt(self,batch):
        for key in ('critic','target','learning_counter'):self.state[key]+=100
        self.state['actor_optimizer']['step']+=1;self.state['actor_optimizer']['momentum'][0]=999
        self.state['actor']=batch;self.state['cpu_rng']=(99,99);self.state['gradients']=[99]
        return {'critic':1.,'actor':2.}
    def gaussians(self,features):return [gaussian(self.state['actor']) for _ in features]

class Contracts(unittest.TestCase):
    def test_missing_manifest_fails_before_runtime_and_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            with self.assertRaisesRegex(m.ContractError,'Missing retained-input manifest'):m.preflight(root/'absent.json')
            self.assertEqual(list(root.iterdir()),[])

    def test_every_missing_role_or_hash_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);asset=root/'one';asset.write_text('synthetic input, never replay')
            manifest={'schema':1,'synthetic':False,'caps':m.CAPS,'files':{role:{'path':'one','sha256':m.digest(asset)} for role in m.ROLES},'episodes':{'one':{}},'behavior_policies':{'one':{}},'baseline_identity':{'one':{}},'measured_native_per_wall_second':800,'evaluation_reserve_integrations':147000}
            target=root/'manifest.json'
            pins={role:m.digest(asset) for role in m.PINNED_RETAINED}
            with mock.patch.object(m,'PINNED_RETAINED',pins):
             for role in m.ROLES:
                broken=copy.deepcopy(manifest);del broken['files'][role];target.write_text(json.dumps(broken))
                with self.assertRaises(m.ContractError):m.preflight(target)
                broken=copy.deepcopy(manifest);broken['files'][role]['sha256']='0'*64;target.write_text(json.dumps(broken))
                with self.assertRaisesRegex(m.ContractError,'hash mismatch'):m.preflight(target)

    def test_synthetic_inputs_never_pass_real_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'manifest.json';path.write_text(json.dumps({'schema':1,'synthetic':True,'caps':m.CAPS}))
            with self.assertRaisesRegex(m.ContractError,'non-synthetic'):m.preflight(path)

    def test_replay_exact_strata_and_episode_uniform_sampling(self):
        records=[row('short')]+[row('long') for _ in range(100)]+[row('hold','back_holding'),row('belly','baseline_preservation')]
        r=m.EpisodeReplay(records);batch=r.batch_indices();categories=[r.rows[i].category for i in batch]
        self.assertEqual([categories.count(c) for c in m.CATEGORIES],[64,32,32])
        sampled=r.category_indices('back_pre_holding',2000);short=sum(r.rows[i].episode=='short' for i in sampled)
        self.assertTrue(850<short<1150)

    def test_replay_preserves_all_retained_rows_without_eviction(self):
        r=replay();original=tuple(r.rows)
        for i in range(4096):r.add(row('fresh',source='fresh_wasm',complete_episode=False,episode_outcome='pending'))
        self.assertEqual(tuple(r.rows[:3]),original)
        with self.assertRaisesRegex(m.ContractError,'capacity'):r.add(row())

    def test_preservation_is_verified_episode_success_not_reward(self):
        with self.assertRaises(m.ContractError):row(category='baseline_preservation',reward=100,episode_outcome='failure').validate()
        with self.assertRaises(m.ContractError):row(category='baseline_preservation',complete_episode=False).validate()
        with self.assertRaises(m.ContractError):row(terminal=True,discount=.9).validate()
        self.assertFalse(row(truncated=True,complete_episode=False,episode_outcome='truncation').terminal)

    def test_summed_21d_kl_not_dimension_mean(self):
        witness=m.FixedWitness([(0.,)*1024],[gaussian(logstd=-1.)],'collecting')
        result=witness.evaluate([gaussian(.04,-1.)])
        self.assertGreater(result.mean_summed_kl,.1);self.assertLess(result.maximum_deterministic_change,.05);self.assertFalse(result.accepted)

    def test_kl_is_mean_over_states_and_fixed_direction(self):
        witness=m.FixedWitness([(0.,)*1024]*2,[gaussian(logstd=-1.)]*2,'collecting')
        result=witness.evaluate([gaussian(.04,-1.),gaussian(0.,-1.)])
        self.assertAlmostEqual(result.mean_summed_kl,21*.04**2/(4*math.exp(-2)));self.assertTrue(result.accepted)

    def test_collecting_reference_stays_fixed_after_accepted_step(self):
        witness=m.FixedWitness([(0.,)*1024],[gaussian(logstd=0.)],'collecting');before=witness.sha256
        self.assertTrue(witness.evaluate([gaussian(.03,0.)]).accepted)
        self.assertFalse(witness.evaluate([gaussian(.06,0.)]).accepted);self.assertEqual(before,witness.sha256)

    def test_nonfinite_policy_is_rejected(self):
        witness=m.FixedWitness([(0.,)*1024],[gaussian()],'collecting')
        self.assertFalse(witness.evaluate([gaussian(math.nan)]).accepted)

    def test_entire_learning_state_rollback_attempt_count_monotonic(self):
        learner=FakeLearner();before=learner.capture_learning_state();r=replay();phase=phase_updates();witness=m.FixedWitness([(0.,)*1024],[gaussian()],'collecting')
        result,metrics,accepted=m.UpdateGuard(learner,r,phase,witness).attempt(.5)
        self.assertFalse(accepted);self.assertEqual(learner.state,before);self.assertEqual(phase.attempts,1);self.assertEqual(phase.block_accepted,0)
        phase.finish_block();self.assertEqual(phase.phase,'stopped_no_actor_acceptance')
        with self.assertRaises(m.ContractError):phase.begin_collection('next')

    def test_replay_rng_rollback_includes_batch_sampling(self):
        learner=FakeLearner();r=replay();before=r.rng.getstate();phase=phase_updates();witness=m.FixedWitness([(0.,)*1024],[gaussian()],'collecting')
        def batch():r.batch_indices();return .5
        m.UpdateGuard(learner,r,phase,witness).attempt(batch)
        self.assertEqual(before,r.rng.getstate())

    def test_collection_policy_and_full_cadence_caps(self):
        phase=m.LoopPhase()
        with self.assertRaises(m.ContractError):phase.begin_collection('early')
        for _ in range(256):phase.warmup_done()
        for cycle in range(16):
            phase.begin_collection(str(cycle))
            with self.assertRaises(m.ContractError):phase.collected('different')
            for _ in range(256):phase.collected(str(cycle))
            with self.assertRaises(m.ContractError):phase.collected(str(cycle))
            for _ in range(64):phase.attempt();phase.accepted()
            with self.assertRaises(m.ContractError):phase.attempt()
            phase.finish_block()
        self.assertEqual((phase.fresh,phase.attempts,phase.phase),(4096,1024,'finished'))

    def test_pause_preserves_full_world_and_recovery_clock(self):
        pause=m.PauseReceipt('episode',2000,480,480,76,'slow-following','abc')
        pause.unchanged(copy.deepcopy(pause))
        for field,value in [('native',2001),('impact',2000),('checkpoint_sha256','different'),('reference_index',0)]:
            with self.assertRaises(m.ContractError):pause.unchanged(replace(pause,**{field:value}))

    def test_source_only_execution_gate_precedes_models_outputs_and_runtime(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(runner,'load_legacy') as legacy, mock.patch.object(runner,'load_seed_records') as seed, mock.patch.object(runner,'RecoveryHooks') as backup:
            output=Path(directory)/'never-created'
            with self.assertRaisesRegex(m.ContractError,'Source-only'):runner.run({}, {}, output)
            legacy.assert_not_called();seed.assert_not_called();backup.assert_not_called();self.assertFalse(output.exists())
        self.assertNotIn('torch',sys.modules)

    def test_learning_and_transport_deadlines_protect_reserved_wall(self):
        self.assertEqual(m.remaining_seconds(3600,1000,500),2100)
        with self.assertRaises(m.ContractError):m.remaining_seconds(3600,3100,500)
        with self.assertRaises(m.ContractError):m.remaining_seconds(3600,3600)
        with self.assertRaises(m.ContractError):m.remaining_seconds(math.inf,3600)

    def test_existing_bridge_wait_uses_absolute_remaining_time_and_never_retries(self):
        bridge=runner.BoundedWasm.__new__(runner.BoundedWasm);bridge.deadline=100.
        bridge.watchdog=type('Watchdog',(),{'assert_alive':lambda self:None})()
        bridge.proc=type('Process',(),{'stdin':io.StringIO(),'stdout':io.StringIO('{"ok":true,"result":7}\n'),'kill':mock.Mock()})()
        with mock.patch.object(runner.time,'time',side_effect=[99.,99.1]), mock.patch.object(runner.select,'select',return_value=([bridge.proc.stdout],[],[])) as select_call:
            self.assertEqual(bridge.call({'op':'state'}),7)
            self.assertEqual(select_call.call_args.args[-1],1.)
        with mock.patch.object(runner.time,'time',return_value=99.), mock.patch.object(runner.select,'select',return_value=([],[],[])):
            with self.assertRaisesRegex(m.ContractError,'no retry'):bridge.call({'op':'state'})
        bridge.proc.kill.assert_called_once()
        ast.parse(runner.AbsoluteWatchdog.PROGRAM)

    def test_episode_completion_updates_prior_block_rows_without_relabeling_actions(self):
        r=replay();start=len(r.rows)
        for i in range(2):r.add(row('new',source='fresh_wasm',behavior=f'block-{i}',complete_episode=False,episode_outcome='pending'))
        r.add(row('new',source='fresh_wasm',behavior='block-2',terminal=True,discount=0.))
        r.finish_episode('new','success')
        self.assertEqual([x.behavior for x in r.rows[start:]],['block-0','block-1','block-2'])
        self.assertTrue(all(x.complete_episode and x.episode_outcome=='success' for x in r.rows[start:]))
        with self.assertRaises(m.ContractError):r.finish_episode('new','truncation',truncated=True)
        r.add(row('unfinished',source='fresh_wasm',complete_episode=False,episode_outcome='pending'))
        r.finish_episode('unfinished','truncation',truncated=True)
        self.assertFalse(r.rows[-1].terminal);self.assertTrue(r.rows[-1].truncated)

    def test_exact_raw_episode_contiguity_outcome_and_clock(self):
        rows=[dict(label='original',preNative=500+i*20,postNative=520+i*20,terminal=i==39,success=i==39,failure=None,
            status={'since_impact_s':(40+i*20)/800,'prototype_standing_passed':i==39,'stable_standing_s':1. if i==39 else 0.}) for i in range(40)]
        episode=dict(first_line=0,last_line=39,label='original',behavior_policy_id='policy',case='original',seed_sha256=m.PINNED_RETAINED['original_seed'],source_start_native=480,impact_native=480,outcome='success',complete=True)
        runner.validate_episode(episode,rows,{'policy':{}})
        for broken in (dict(episode,outcome='failure'),dict(episode,impact_native=400),dict(episode,seed_sha256='bad')):
            with self.assertRaises(m.ContractError):runner.validate_episode(broken,rows,{'policy':{}})
        broken=copy.deepcopy(rows);broken[-1]['preNative']=521
        with self.assertRaises(m.ContractError):runner.validate_episode(episode,broken,{'policy':{}})

    def test_existing_source_tail_is_closed_after_256th_decision_before_updates(self):
        # Extract the existing source-transition class without importing torch or
        # constructing even a synthetic neural network.
        tree=ast.parse(Path(__file__).with_name('train-getup-wasm-adaptation.py').read_text())
        definition=next(item for item in tree.body if isinstance(item,ast.ClassDef) and item.name=='SourceTransition')
        namespace={'np':type('NP',(),{'float32':float,'zeros':staticmethod(lambda n,dtype:[0.]*n)})}
        exec(compile(ast.Module(body=[definition],type_ignores=[]),'<existing SourceTransition>','exec'),namespace)
        def state(native,decision=None):return {'native':native,'impactEpoch':480,'sourceEpoch':480,
            'preview':None if decision is None else {'features':[0.]*1024,'referenceDecision':decision},
            'status':{'reference_frames':100,'reference_index':decision or 255,'phase':'source' if decision else 'balance'}}
        class Bridge:
            def __init__(self):self.calls=[]
            def call(self,q):
                self.calls.append(q);n=len(self.calls);terminal=n==258
                return {'state':state(500+n*20,n+1 if n<256 else None),'native':500+n*20,
                    'terminal':terminal,'success':terminal,'failure':None,'reward':100. if terminal else 1.}
        environment=runner.ExistingWasmEnvironment.__new__(runner.ExistingWasmEnvironment)
        environment.wasm=Bridge();environment.state=state(500,1);environment.episode_id='synthetic-tail';environment.episode=1
        environment.builder=namespace['SourceTransition'](runner.GAMMA);environment.pending_origin=None;environment.complete_episodes=0
        learner=type('Learner',(),{'policy_id':lambda self:'fixed','action':lambda self,f:[0.]*21})()
        phase=m.LoopPhase()
        for _ in range(256):phase.warmup_done()
        r=replay();ids,policy=environment.collect_block(learner,r,phase,lambda:None)
        self.assertEqual((len(ids),phase.fresh,phase.phase),(256,256,'updates'))
        self.assertEqual(len(environment.wasm.calls),258);self.assertFalse(environment.wasm.calls[-1]['training'])
        self.assertTrue(r.rows[-1].terminal);self.assertEqual(r.rows[-1].discount,0.)
        self.assertAlmostEqual(r.rows[-1].reward,1+runner.GAMMA+100*runner.GAMMA**2)
        self.assertTrue(all(r.rows[i].complete_episode for i in ids));self.assertIsNone(environment.state)

    def test_collection_learning_collection_preserves_ongoing_episode_and_epoch(self):
        def state(n):return {'native':500+n*20,'impactEpoch':480,'sourceEpoch':480,
            'preview':{'features':[0.]*1024,'referenceDecision':n+1},'status':{'reference_frames':100}}
        class Bridge:
            def __init__(self):self.steps=0
            def call(self,q):
                self.steps+=1
                return {'state':state(self.steps),'native':500+self.steps*20,'reward':0.,'terminal':False,'success':False,'failure':None}
        class Builder:
            pending=None
            def push(self,s,c,result):return (s['preview']['features'],c,result['state']['preview']['features'],0.,False)
        environment=runner.ExistingWasmEnvironment.__new__(runner.ExistingWasmEnvironment)
        environment.wasm=Bridge();environment.state=state(0);environment.episode_id='same-original';environment.episode=1
        environment.builder=Builder();environment.pending_origin=None;environment.complete_episodes=0
        learner=type('Learner',(),{'identity':'before','policy_id':lambda self:self.identity,'action':lambda self,f:[0.]*21})()
        phase=m.LoopPhase();r=replay()
        for _ in range(256):phase.warmup_done()
        first,_=environment.collect_block(learner,r,phase,lambda:None)
        receipt=(environment.state['native'],environment.state['impactEpoch'],environment.episode_id)
        phase.attempt();phase.accepted();phase.finish_block();learner.identity='after'
        self.assertEqual(receipt,(environment.state['native'],480,'same-original'))
        second,_=environment.collect_block(learner,r,phase,lambda:None)
        self.assertEqual(environment.state['native'],10740);self.assertEqual(environment.episode,1)
        self.assertTrue(all(r.rows[i].behavior=='before' for i in first))
        self.assertTrue(all(r.rows[i].behavior=='after' for i in second))
        self.assertTrue(all(r.rows[i].impact_native==480 for i in first+second))

    def test_durable_hook_delegates_all_live_state_roles_and_original_deadline(self):
        calls=[]
        guard=type('Guard',(),{'checkpoint':lambda self,*args,**kw:calls.append((args,kw)) or Path('synthetic-proof-only')})()
        hooks=runner.RecoveryHooks.__new__(runner.RecoveryHooks);hooks.guard=guard;hooks.artifacts=[];hooks.dependencies=[]
        with tempfile.TemporaryDirectory() as directory:
            hooks.checkpoint(Path(directory),{'clock':'fixed'},'synthetic unit delegation',3600.)
        artifacts=calls[0][0][0];roles={item['role'] for item in artifacts}
        self.assertIn('live_learner-state_pt',roles);self.assertIn('live_replay-local_npz',roles)
        self.assertIn('live_loop-state-local_json',roles);self.assertIn('live_current-world-checkpoint_bin',roles)
        self.assertIn('live_budget-ledger_json',roles);self.assertIn('live_replay-rng-local_json',roles)
        self.assertEqual(set(calls[0][1]['required_roles']),roles);self.assertEqual(calls[0][1]['deadline'],3600.)

    def test_durable_configuration_is_required_before_backend_or_models(self):
        backend=mock.Mock()
        fake=type('Backup',(),{'LibraryBackend':backend,'RecoveryGuard':mock.Mock(),'require_recovery_point':mock.Mock()})
        with mock.patch.dict(sys.modules,{'experiment_backup':fake}):
            with self.assertRaisesRegex(m.ContractError,'durable recovery configuration'):runner.RecoveryHooks({}, {}, Path('absent'))
        backend.assert_not_called()

if __name__=='__main__':unittest.main()
