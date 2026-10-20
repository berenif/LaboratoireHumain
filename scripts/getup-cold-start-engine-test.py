#!/usr/bin/env python3
"""Production phases, scripted I/O, REAL disposable synthetic learner/replay."""
import copy
from dataclasses import asdict,replace
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from experiment_backup import restore_bundle,sha256,verify_bundle
from getup_cold_start_engine import *
from getup_cold_start_rehearsal import *

spec=importlib.util.spec_from_file_location('interfaces',Path(__file__).with_name('train-getup-learning-loop.py'))
interfaces=importlib.util.module_from_spec(spec);spec.loader.exec_module(interfaces)
IDENTITY={'sourceCommit':'a'*40,'workerSha256':'b'*64,'protocolSha256':'c'*64,'runId':'disposable-software-only'}


def factories():
    legacy=interfaces.load_legacy()
    return lambda:interfaces.SACLearner(legacy),lambda:legacy.SourceTransition(interfaces.GAMMA)


def build(root,**bridge_options):
    clock=ScriptClock();plan=make_plan(clock);output=Path(root)/'pilot'
    bridge=ScriptedPhysics(output,clock,plan,**bridge_options)
    transport=LocalRehearsalTransport(Path(root)/'recovery',clock)
    recovery=CoherentRecovery(transport,bridge.independent_controller,synthetic=True)
    learner_factory,builder_factory=factories()
    pilot=ColdPilot(output=output,learner=learner_factory(),learner_factory=learner_factory,
        builder_factory=builder_factory,bridge=bridge,recovery=recovery,clock=clock,
        plan=plan,identity=IDENTITY,synthetic=True)
    return pilot,transport


def fingerprint(value):
    import numpy as np
    import torch
    h=hashlib.sha256()
    def add(x):
        if isinstance(x,torch.Tensor):h.update(b't'+str(x.dtype).encode()+str(tuple(x.shape)).encode()+x.detach().numpy().tobytes())
        elif isinstance(x,np.ndarray):h.update(b'n'+str(x.dtype).encode()+str(x.shape).encode()+x.tobytes())
        elif isinstance(x,dict):
            for k in sorted(x,key=str):add(str(k));add(x[k])
        elif isinstance(x,(list,tuple)):
            h.update(str(type(x)).encode())
            for y in x:add(y)
        else:h.update(repr(x).encode()+b'\0')
    add(value);return h.hexdigest()


def restore_from_archive(archive,root):
    manifest=restore_bundle(archive,Path(root)/'restore',sha256(archive),64*1024**2)
    files={x['role']:Path(root)/'restore'/x['path'] for x in manifest['files']}
    cursor=json.loads(files['cursor'].read_text());plan=PhasePlan(**cursor['plan'])
    clock=ScriptClock(plan.started);controller=ScriptedPhysics(Path(root)/'controller',clock,plan)
    learner_factory,builder_factory=factories()
    return restore_coherent(Path(root)/'restore',manifest,expected_identity=IDENTITY,
        expected_plan=plan,expected_ledger=cursor['ledger'],learner_factory=learner_factory,
        builder_factory=builder_factory,controller_factory=controller.independent_controller,synthetic=True)


def next_update_probe(state):
    sampled=state.replay.batch_indices(128)
    guard=UpdateGuard(state.learner,state.replay,state.phase,state.witness)
    result=guard.attempt(lambda:state.learner.batch(state.replay))
    return {'sampled':sampled,'result':encoded([asdict(result[0]),result[1],result[2]]),'state':fingerprint(state.learner.capture_learning_state()),
        'replay_rng':encoded(state.replay.rng.getstate()),'phase':encoded(vars(state.phase))}


class ProductionRehearsal(unittest.TestCase):
    def test_complete_production_path_and_fresh_process_update(self):
        with tempfile.TemporaryDirectory(prefix='lh-software-test-') as tmp:
            pilot,transport=build(tmp)
            probes=[]
            def check(archive,manifest,boundary):
                if boundary!='collection-00':return
                # Expected side is the uninterrupted LIVE paused objects, not
                # another reconstruction that could share the same restore bug.
                before=pilot.learner.capture_learning_state()
                replay_rng=pilot.replay.rng.getstate();phase=copy.deepcopy(vars(pilot.phase))
                try:expected=next_update_probe(pilot)
                finally:
                    pilot.learner.restore_learning_state(before);pilot.replay.rng.setstate(replay_rng)
                    pilot.phase.__dict__.clear();pilot.phase.__dict__.update(phase)
                completed=subprocess.run([sys.executable,__file__,'--probe',str(archive)],
                    text=True,capture_output=True,check=True,timeout=60)
                actual=json.loads(completed.stdout)
                for key in expected:self.assertEqual(expected[key],actual[key],key)
                probes.append(True)
            transport.after_verified=check
            report=pilot.run()
            self.assertEqual(report['stopped'],'single-candidate-complete-no-promotion',report)
            self.assertEqual(len(probes),1);self.assertGreater(report['accepted'],0);self.assertGreater(report['rejected'],0)
            self.assertNotEqual(*report['policyHistory']);self.assertEqual(pilot.phase.cycle,2)
            self.assertEqual(report['ledger']['iterativeDecisions'],512)
            self.assertEqual(len(transport.sizes),7);self.assertEqual(len(transport.retained),2)
            self.assertEqual(len(list((transport.root/'bundles').iterdir())),2)
            self.assertLess(max(transport.sizes),32*1024**2)
            self.assertLess(transport.cumulative,160*1024**2)
            for flag in ('trainingExecuted','physicalQualificationPassed','actualTransitionParityPassed',
                         'actualCoherentRecoveryPassed','candidateAccepted','releaseAccepted'):
                self.assertIs(report[flag],False)
            self.assertTrue(report['syntheticEvaluationPassed'])
            self.assertEqual(len(pilot.replay.rows),522) # duplicate parity decision is charged, not inserted twice
            self.assertTrue(all(r.source=='synthetic_rehearsal' for r in pilot.replay.rows))
            with __import__('zipfile').ZipFile(transport.retained[-1]['archive']) as zipped:
                final_payload={x.filename:{'payload_bytes':x.file_size,'compressed_bytes':x.compress_size}
                    for x in zipped.infolist() if x.filename.startswith('payload/')}
            print(json.dumps({'softwareRehearsalArchiveBytes':transport.sizes,
                'projectedCumulativeBytes':transport.cumulative,'actualRemoteBytesAdded':0,
                'finalSyntheticPayloadFiles':final_payload,
                'localRetainedBytes':sum(p.stat().st_size for p in Path(tmp).rglob('*') if p.is_file())}))

    def test_partial_bootstrap_malformed_and_uncertain_keep_nonresumable_evidence(self):
        for defect in ('malformed_call','fail_call'):
            with self.subTest(defect=defect),tempfile.TemporaryDirectory(prefix='lh-failure-') as tmp:
                pilot,transport=build(tmp,**{defect:5})
                report=pilot.run();self.assertEqual(report['stopped'],'safe-stop')
                self.assertNotIn('terminalRetentionError',report,report)
                self.assertEqual(pilot.bridge.physical_calls,5)
                self.assertEqual(transport.requests,['terminal-stop'])
                manifest=verify_bundle(transport.retained[0]['archive'])
                self.assertFalse(manifest['full_resume_ready']);self.assertFalse(manifest['application_resume_eligible'])
                self.assertFalse(report['syntheticOptimizerMechanicsExercised'])
                if defect=='fail_call':
                    self.assertEqual(report['ledger']['uncertainIntegrations'],5020)
                    self.assertEqual(report['ledger']['nativeIntegrations'],5100)
                    self.assertTrue(report['ledger']['pendingCall'])

    def test_empty_stratum_stops_before_optimizer(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp,empty_holding=True);report=pilot.run()
            self.assertEqual(report['stopped'],'safe-stop');self.assertIn('category',report['error'].lower())
            self.assertFalse(report['syntheticOptimizerMechanicsExercised'])
            self.assertNotIn('terminalRetentionError',report)

    def test_failed_or_delayed_backup_never_retries_or_updates(self):
        for defect in ('failure','delay'):
            with self.subTest(defect=defect),tempfile.TemporaryDirectory() as tmp:
                pilot,transport=build(tmp)
                if defect=='failure':transport.fail_boundary='bootstrap-before-any-optimizer-update'
                else:transport.delay=181
                report=pilot.run()
                self.assertEqual(transport.requests,['bootstrap-before-any-optimizer-update'])
                self.assertFalse(report['syntheticOptimizerMechanicsExercised'])
                self.assertIn('terminalRetentionError',report)

    def test_deadline_reserve_and_rapid_reset_forecasts(self):
        clock=ScriptClock();plan=make_plan(clock)
        ledger={'nativeIntegrations':0}
        self.assertEqual(plan.admit_sequence(clock.now(),ledger),2)
        self.assertEqual(plan.admit_sequence(clock.now()+150,ledger),2)
        with self.assertRaises(ContractError):plan.admit_sequence(clock.now()+150,ledger,
            prefix_seconds=60,prefix_integrations=1000)
        for bad in (replace(plan,deadline=plan.deadline+1),replace(plan,evaluation_end=plan.deadline-1)):
            with self.assertRaises(ContractError):bad.admit_sequence(clock.now(),ledger)
        with self.assertRaises(ContractError):plan.admit_sequence(clock.now()+1000,ledger)
        with self.assertRaises(ContractError):plan.admit_sequence(clock.now(),{'nativeIntegrations':100000})
        with self.assertRaises(ContractError):plan.admit_sequence(clock.now(),ledger,cumulative_uploaded_bytes=40*1024**2)
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp,episode_decisions=1,forecast_per_reset=5000)
            report=pilot.run();self.assertEqual(report['stopped'],'safe-stop')
            self.assertIn('phase envelope',report['error']);self.assertLessEqual(report['ledger']['nativeIntegrations'],153000)
            self.assertEqual(report['ledger']['uncertainIntegrations'],0)

    def test_restore_failure_does_not_mutate_live_rng(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp);pilot.cursor.planned_blocks=2;pilot.parity();pilot.bootstrap()
            before=pilot.learner.capture_learning_state()
            def reject(*args):raise ContractError('controller rejected after fresh learner construction')
            pilot.recovery.controller_factory=reject
            with self.assertRaises(ContractError):pilot.backup('bootstrap-before-any-optimizer-update')
            assert_state_equal(before,pilot.learner.capture_learning_state())

    def test_restore_rejects_mixed_identity_ledger_deadline_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp);pilot.cursor.planned_blocks=2;pilot.parity();pilot.bootstrap()
            pilot.backup('bootstrap-before-any-optimizer-update')
            archive=transport.retained[-1]['archive'];restored=Path(tmp)/'corruption'
            manifest=restore_bundle(archive,restored,sha256(archive),64*1024**2)
            files={x['role']:restored/x['path'] for x in manifest['files']}
            original=files['cursor'].read_text();good=json.loads(original)
            for defect in ('run','deadline','reduced','pending','mixed'):
                bad=copy.deepcopy(good)
                if defect=='run':bad['identity']['runId']='different'
                if defect=='deadline':bad['plan']['deadline']+=1
                if defect=='reduced':bad['ledger']['nativeIntegrations']-=20
                if defect=='pending':bad['ledger']['pendingCall']={'chargedUpperBound':5020}
                if defect=='mixed':bad['snapshot_id']='different'
                files['cursor'].write_text(json.dumps(bad))
                with self.subTest(defect=defect),self.assertRaises(ContractError):
                    restore_coherent(restored,manifest,expected_identity=IDENTITY,expected_plan=pilot.plan,
                        expected_ledger=good['ledger'],learner_factory=pilot.learner_factory,
                        builder_factory=pilot.builder_factory,controller_factory=pilot.bridge.independent_controller,synthetic=True)
            files['cursor'].write_text(original)

    def test_pending_tail_restore_continues_exact_folded_transition(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp,pending_tail=True)
            pilot.cursor.planned_blocks=2;pilot.parity();pilot.bootstrap()
            pilot.backup('bootstrap-before-any-optimizer-update')
            pilot.enter('collection',120,20140)
            pilot.reset('belly','baseline-deterministic-zero')
            for _ in range(3):
                row,result=pilot.step([0.]*21,'bootstrap')
                if row:
                    index=pilot.replay.add(row);pilot.mapping.append(dict(pilot.mapper.mapping[-1],replay_index=index))
            self.assertIsNotNone(pilot.mapper.builder.pending)
            pilot.backup('terminal-stop',retention=True)
            restored=pilot.recovery.last_restored
            query={'op':'step','c':None,'training':False,'collectionPhase':'bootstrap',
                   'episodeId':pilot.cursor.episode_id,'policyId':pilot.mapper.behavior}
            original_result=pilot.bridge.call(query);restored_result=restored.controller.call(query)
            self.assertEqual(original_result,restored_result)
            a=pilot.mapper.push(pilot.state,None,original_result)
            b=restored.mapper.push(restored.cursor['state'],None,restored_result)
            self.assertEqual(a,b);self.assertEqual(pilot.mapper.mapping,restored.mapper.mapping)
            self.assertEqual(pilot.mapper.mapping[-1]['transition_line_end']-pilot.mapper.mapping[-1]['transition_line_start']+1,2)

    def test_zero_step_physical_refusal_preserves_cause_without_replay_or_retry(self):
        from types import SimpleNamespace
        pilot=ColdPilot.__new__(ColdPilot)
        source={'native':4560,'impactEpoch':560,'sourceEpoch':None,'preview':None}
        observed=dict(source,status={'phase':'physical-roll-Rejected'})
        failure={'check':'getup-policy','offending':'fixed5-second quiet-prone refusal'}
        result={'state':observed,'native':4560,'reward':-100.,'terminal':True,
                'success':False,'failure':failure,'nativeTraceSha256':hashlib.sha256(b'[]').hexdigest()}
        pilot.state=source;pilot.cursor=SimpleNamespace(active=True,episode_id='refused')
        pilot.mapper=SimpleNamespace(behavior='baseline-deterministic-zero',push=mock.Mock(),
            builder=SimpleNamespace(pending=None),pending_origin=None,pending_line=None)
        pilot.call=mock.Mock(return_value=result)
        with self.assertRaisesRegex(ContractError,'Physical integrity failure:.*getup-policy'):
            pilot.step(None,'bootstrap')
        self.assertIs(pilot.state,observed)
        pilot.mapper.push.assert_not_called()
        pilot.call.assert_called_once()
        self.assertFalse(pilot.call.call_args.args[0]['training'])

    def test_absolute_stage_cutoffs_and_exactly_two_blocks(self):
        clock=ScriptClock();plan=make_plan(clock)
        expected={'immutable-input-restore':120,'zero-correction-qualification':180,
            'transition-parity':240,'bootstrap':600,'backup:bootstrap-before-any-optimizer-update':600,
            'warmup':900,'backup:critic-warmup-complete':900,'backup:collection-00':1500,
            'backup:updates-01':1500,'backup:collection-01':2100,'backup:updates-02':2100,
            'evaluation':3300,'backup:terminal-stop':3600}
        for name,offset in expected.items():self.assertEqual(plan.stage_cutoff(name),plan.started+offset)
        self.assertEqual(plan.stage_cutoff('collection',0),plan.started+1500)
        self.assertEqual(plan.stage_cutoff('updates',1),plan.started+2100)
        with self.assertRaises(ContractError):plan.stage_cutoff('collection',2)
        with self.assertRaisesRegex(ContractError,'two-minute'):plan.assert_inputs_ready(plan.started+120)
        fast=replace(plan,parity_seconds=1,bootstrap_seconds=1,backup_seconds=1,
            warmup_seconds=1,collection_seconds=1,updates_seconds=1,
            bootstrap_integrations=1,block_integrations=1,parity_integrations=1,
            backup_archive_envelope_bytes=1,backup_payload_envelope_bytes=1)
        self.assertEqual(fast.admit_sequence(clock.now(),{'nativeIntegrations':0},maximum_blocks=12),2)
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp)
            for name in ('zero-correction-qualification','transition-parity','bootstrap',
                         'backup:bootstrap-before-any-optimizer-update','warmup','backup:updates-01'):
                pilot.clock.value=pilot.plan.stage_cutoff(name)-59.99
                before=pilot.bridge.physical_calls
                with self.assertRaisesRegex(ContractError,'absolute cutoff'):pilot.enter(name,60)
                self.assertEqual(pilot.bridge.physical_calls,before)

    def test_missing_resource_admission_and_closed_output_do_not_start_clock(self):
        spec=importlib.util.spec_from_file_location('cold_entry',Path(__file__).with_name('train-getup-cold-start.py'))
        entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
        self.assertEqual(entry.admitted_start({'started_at_unix':1000.},1005.),1000.)
        for value in (None,float('nan'),1006.,800.):
            with self.subTest(start=value),self.assertRaisesRegex(ContractError,'no clock renewal'):
                entry.admitted_start({'started_at_unix':value},1005.)
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'not-created'
            with mock.patch.object(entry.time,'time',side_effect=AssertionError('clock started')):
                with self.assertRaisesRegex(ContractError,'resource decision'):
                    entry.run(output,Path(tmp),Path(tmp),'unadmitted')
                self.assertFalse(output.exists())
                output.mkdir()
                with self.assertRaisesRegex(ContractError,'cannot be reused'):
                    entry.run(output,Path(tmp),Path(tmp),'old')

    def test_dead_bridge_uncertain_command_keeps_durable_charge_without_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp,fail_call=5)
            original=pilot.bridge.call
            def dead_after_failure(query):
                if pilot.bridge.physical_calls>=5:raise BrokenPipeError('scripted dead bridge')
                return original(query)
            pilot.bridge.call=dead_after_failure
            report=pilot.run()
            self.assertNotIn('terminalRetentionError',report,report)
            self.assertEqual(report['ledger']['nativeIntegrations'],5100)
            self.assertEqual(report['ledger']['uncertainIntegrations'],5020)
            self.assertEqual(pilot.bridge.physical_calls,5)

    def test_rejected_production_update_block_restores_full_state_and_sampling(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp);pilot.cursor.planned_blocks=2;pilot.parity();pilot.bootstrap()
            pilot.cursor.coherent_before_updates=True;pilot.phase.warmup=256;pilot.phase.phase='ready_collection'
            pilot.collect();before=pilot.learner.capture_learning_state();rng=pilot.replay.rng.getstate()
            original=pilot.learner.joint_attempt
            def rejected(batch):
                metrics=original(batch);metrics['forced_test_nonfinite']=float('nan');return metrics
            pilot.learner.joint_attempt=rejected
            pilot.updates()
            assert_state_equal(before,pilot.learner.capture_learning_state())
            self.assertEqual(rng,pilot.replay.rng.getstate())
            self.assertEqual(pilot.phase.attempts,64);self.assertEqual(pilot.report['rejected'],64)
            self.assertEqual(pilot.phase.phase,'stopped_no_actor_acceptance')

    def test_last_optimizer_iteration_overrun_is_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            pilot,transport=build(tmp);pilot.cursor.planned_blocks=2;pilot.parity();pilot.bootstrap()
            pilot.cursor.coherent_before_updates=True
            count=0
            def work(kind):
                nonlocal count
                count+=1
                if count==256:pilot.clock.advance(121)
            pilot.clock.work=work
            with self.assertRaisesRegex(ContractError,'hard wall'):pilot.warmup()


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--probe':
        with tempfile.TemporaryDirectory(prefix='lh-fresh-child-') as tmp:
            print(json.dumps(next_update_probe(restore_from_archive(Path(sys.argv[2]),tmp)),sort_keys=True))
    else:unittest.main()
