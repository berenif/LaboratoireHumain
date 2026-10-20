#!/usr/bin/env python3
"""Cold contracts + real CPU Torch optimizer rollback; no physics/training run.

Tiny synthetic tensors exercise the real learner. They never become pilot data.
"""
import copy
from dataclasses import replace
import importlib.util
import json
import hashlib
from pathlib import Path
import tempfile
import time
import unittest
import subprocess
from unittest import mock

import getup_cold_start as cold
from getup_learning_loop import (ContractError, EpisodeReplay, FixedWitness, Record,
                                 UpdateGuard)
from experiment_backup import sha256

spec = importlib.util.spec_from_file_location('corrected_runner',
    Path(__file__).with_name('train-getup-learning-loop.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
spec = importlib.util.spec_from_file_location('cold_runner',
    Path(__file__).with_name('train-getup-cold-start.py'))
cold_runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cold_runner)


def record(category, episode):
    return Record(episode, category, 'holding' if category == 'back_holding' else 'pre_holding',
        'synthetic-test-only', (0.125,) * 1024, (0.,) * 21, (.25,) * 1024,
        .1, runner.GAMMA, False, False, 500, 520, 480, True, 'retained_verified',
        'success', category == 'baseline_preservation')


def recursive_equal(test, left, right):
    import numpy as np
    import torch
    test.assertEqual(type(left), type(right))
    if isinstance(left, torch.Tensor):
        test.assertTrue(torch.equal(left, right))
    elif isinstance(left, np.ndarray):
        test.assertTrue(np.array_equal(left, right))
    elif isinstance(left, dict):
        test.assertEqual(left.keys(), right.keys())
        for key in left: recursive_equal(test, left[key], right[key])
    elif isinstance(left, (tuple, list)):
        test.assertEqual(len(left), len(right))
        for a, b in zip(left, right): recursive_equal(test, a, b)
    else:
        test.assertEqual(left, right)


class ColdContracts(unittest.TestCase):
    def boundary(self):
        started=time.time()-300;deadline=started+3600
        ledger={'nativeIntegrations':2380,'realIntegrations':2380,'forecastIntegrations':0,
            'trainingDecisions':0,'completedCalls':122,'uncertainIntegrations':0,
            'uncertainCalls':[],'limit':300000,'evaluationReserveIntegrations':147000,
            'deadlineUnixMs':round(deadline*1000)}
        config={'inputContract':{'startedAtUnixMs':round(started*1000),'deadlineUnixMs':round(deadline*1000)}}
        report={'runId':'test-one-attempt','source':{'commit':'a'*40},'startedAtUnix':started,
            'deadlineUnix':deadline,'ledger':ledger,'trainingExecuted':False,'accepted':0,'rejected':0,
            'stopped':'cold-start-safe-stop','backupFailureLatched':False,'candidateAccepted':False,
            'error':'Original absolute deadline or protected evaluation wall reached',
            'adapterParity':{'passed':True,'physicsIntegrations':2380}}
        amendment={'schema':'lh-cold-qualification-amendment/v1','runId':report['runId'],
            'boundary':'after-coldQualify-before-first-bootstrap-reset','amendedSourceCommit':'b'*40,
            'originalSourceCommit':'a'*40,'startedAtUnix':started,'deadlineUnix':deadline,
            'learnerSeed':20261020,'genericResumeAuthorized':False,'limitedUsefulOperationOnly':True}
        return report,ledger,config,amendment

    def test_exact_reentry_rejects_identity_deadline_counter_and_uncertain_changes(self):
        original=self.boundary()
        result=cold.validate_qualification_boundary(*original,'test-one-attempt','b'*40,time.time())
        self.assertEqual(result['before_evaluation_integrations'],150620)
        for defect in ('run','deadline','counter','boundary','uncertain','pending','decisions','updates','scope'):
            report,ledger,config,amendment=copy.deepcopy(original)
            if defect=='run':amendment['runId']='different'
            if defect=='deadline':amendment['deadlineUnix']+=1
            if defect=='counter':ledger['nativeIntegrations']-=1
            if defect=='boundary':amendment['boundary']='any-checkpoint'
            if defect=='uncertain':ledger['uncertainCalls']=[{'chargedUpperBound':5020}]
            if defect=='pending':ledger['pendingCall']={'chargedUpperBound':5020}
            if defect=='decisions':ledger['trainingDecisions']=1
            if defect=='updates':report['accepted']=1
            if defect=='scope':amendment['limitedUsefulOperationOnly']=False
            with self.subTest(defect=defect),self.assertRaises(ContractError):
                cold.validate_qualification_boundary(report,ledger,config,amendment,
                    'test-one-attempt','b'*40,time.time())

    def test_limited_reentry_dispatches_one_reset_no_prepare_qualify_or_optimizer(self):
        report,ledger,config,amendment=self.boundary()
        original_report=copy.deepcopy(report)
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'existing';output.mkdir()
            for name,value in [('cold-start-report.json',report),('budget-ledger.json',ledger)]:
                cold.atomic_json(output/name,value)
            path=Path(directory)/'amendment.json';cold.atomic_json(path,amendment)
            exchange=mock.Mock(failed=True);calls=[]
            bridge=mock.Mock();bridge.deadline=report['deadlineUnix']
            def command(query):
                calls.append(query['op'])
                if query['op']=='ledger':return copy.deepcopy(ledger)
                if query['op']=='reset':
                    ledger['nativeIntegrations']+=20;ledger['realIntegrations']+=20;ledger['completedCalls']+=1
                    return {'native':360}
                return {}
            bridge.call.side_effect=command
            learner=mock.Mock()
            with mock.patch.object(cold_runner,'verify_qualification_amendment',
                    return_value=(amendment,copy.deepcopy(report),config,exchange)), \
                 mock.patch.object(cold_runner.subprocess,'check_output',side_effect=lambda cmd,**k: '' if 'diff' in cmd else 'b'*40), \
                 mock.patch.object(cold_runner.subprocess,'run'), \
                 mock.patch.object(cold_runner.interfaces,'AbsoluteWatchdog',
                    return_value=mock.Mock(spec=['assert_alive','close'])), \
                 mock.patch.object(cold_runner.interfaces,'load_legacy'), \
                 mock.patch.object(cold_runner.interfaces,'SACLearner',return_value=learner), \
                 mock.patch.object(cold_runner.interfaces,'BoundedWasm',return_value=bridge):
                with self.assertRaisesRegex(ContractError,'Limited useful initialization completed'):
                    cold_runner.run(output,Path(directory),Path(directory),'test-one-attempt',path)
            self.assertEqual([x for x in calls if x in ('reset','step','coldPrepare','coldQualify','hold')],['reset'])
            self.assertEqual(ledger['nativeIntegrations'],2400)
            learner.joint_attempt.assert_not_called();learner.critic_attempt.assert_not_called()
            retained=json.loads((output/'amendments/original-stop-report.json').read_text())
            self.assertEqual(retained,original_report)
            result=json.loads((output/'limited-bootstrap-initialization.json').read_text())
            self.assertFalse(result['nextOperationAuthorized'])
            self.assertEqual(result['originalDeadlineUnix'],report['deadlineUnix'])

    def test_javascript_limited_claim_rejects_reduced_ledger_and_renewal(self):
        report,ledger,config,amendment=self.boundary()
        claim={'schema':'lh-cold-qualification-claim/v1','runId':report['runId'],
            'maximumNewIntegrations':5020,'maximumPhysicalCalls':1,
            'operation':'bootstrap-belly-initialization','originalDeadlineUnixMs':ledger['deadlineUnixMs']}
        script="""import assert from 'node:assert/strict';
import {validateColdQualificationClaim as check} from './scripts/getup-cold-start-protocol-contract.mjs';
const [c,a,l,f]=JSON.parse(process.argv[1]);check(c,a,l,f,Date.now());
for(const mutate of [
 x=>x[0].maximumPhysicalCalls=2,x=>x[2].nativeIntegrations=0,
 x=>x[2].deadlineUnixMs+=1,x=>x[2].pendingCall={},
 x=>x[1].boundary='generic',x=>x[2].uncertainCalls=[{}]]){
 const x=structuredClone([c,a,l,f]);mutate(x);assert.throws(()=>check(...x,Date.now()));
}"""
        subprocess.run(['node','--input-type=module','-e',script,json.dumps([claim,amendment,ledger,config])],
            cwd=cold_runner.ROOT,check=True,capture_output=True)

    def test_reentry_requires_exact_private_application_receipts(self):
        request={'run_id':'one','kind':'shared','archive_sha256':'a'*64,
                 'archive_bytes':3,'manifest_sha256':'b'*64}
        receipt=dict(request,schema='lh-private-drive-receipt/v1',owner_only=True,
            shared=False,download_completed=True,drive_file_id='test-not-remote')
        application={'passed':True,'physics_integrations':0,'archive_sha256':'a'*64}
        cold.validate_backup_identity(request,receipt,application,'one')
        for r,a in ((None,application),(receipt,None),({},application),
                    (dict(receipt,owner_only=False),application),
                    (receipt,dict(application,passed=False)),
                    (dict(receipt,manifest_sha256='c'*64),application)):
            with self.assertRaisesRegex(ContractError,'receipt'):
                cold.validate_backup_identity(request,r,a,'one')
        with self.assertRaisesRegex(ContractError,'unadmitted'):
            cold.require_full_phase_capacity()

    def test_terminal_evidence_without_active_runtime_never_calls_physics(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            ledger={'nativeIntegrations':2380,'trainingDecisions':0}
            report={'stopped':'cold-start-safe-stop','ledger':ledger,'trainingExecuted':False}
            cold.atomic_json(root/'budget-ledger.json',ledger)
            wasm=mock.Mock();learner=mock.Mock()
            roles=cold_runner.collect_stop_evidence(root,report,time.time()+60,
                wasm=wasm,learner=learner,state=None)
            wasm.call.assert_not_called()
            learner.capture_learning_state.assert_not_called()
            self.assertNotIn('stopped_runtime',roles)
            cursor=json.loads((root/'stopped-cursor.json').read_text())
            self.assertFalse(cursor['runtimeCaptured'])
            self.assertFalse(cursor['bootstrapReplayConstructed'])
            self.assertFalse(cursor['coherentBootstrapBackupVerified'])
            self.assertEqual(cursor['runtimeOmission'],'no-active-runtime')
            exchange=cold.DriveExchange(root/'exchange','synthetic-terminal-test',
                {'commit':'a'*40,'dirty':False,'description':'unit test'},time.time()+60,'a'*64)
            request=exchange.snapshot([cold.artifact(role,root/name,'test/v1')
                for role,name in roles.items()],{'resumeAuthorized':False},'partial-safe-stop-evidence')
            value=json.loads(request.read_text())
            manifest=cold.restore_bundle(value['archive_path'],root/'restored',value['archive_sha256'])
            proof=cold_runner.verify_stop_documents(root/'restored',manifest,root,roles)
            self.assertTrue(proof['passed'])
            self.assertEqual(proof['physics_integrations'],0)
            self.assertFalse(manifest['full_resume_ready'])
            for key in ('application_resume_eligible','full_resume_ready'):
                changed=dict(manifest);changed[key]=True
                with self.assertRaisesRegex(ContractError,'nonresumable'):
                    cold_runner.verify_stop_documents(root/'restored',changed,root,roles)

    def test_terminal_live_capture_failure_still_retains_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);wasm=mock.Mock()
            wasm.call.side_effect=RuntimeError('no active runtime')
            report={'stopped':'cold-start-safe-stop','ledger':None}
            roles=cold_runner.collect_stop_evidence(root,report,time.time()+60,
                wasm=wasm,learner=mock.Mock(),state={'native':500})
            self.assertEqual(set(roles),{'report','stopped_cursor'})
            cursor=json.loads((root/'stopped-cursor.json').read_text())
            self.assertEqual(cursor['runtimeOmission'],'live-state-capture-failed')
            self.assertFalse(cursor['runtimeCaptured'])
            self.assertFalse(cursor['restartAuthorized'])

    def test_historical_runner_still_unconditionally_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'absent'
            with self.assertRaisesRegex(ContractError, 'Source-only'):
                runner.run({}, {}, out)
            self.assertFalse(out.exists())

    def test_bootstrap_and_iterative_caps_cannot_be_borrowed(self):
        for invalid in (0, -1, 1025, True):
            with self.assertRaises(ContractError): cold.ColdPhase(invalid)
        phase = cold.ColdPhase(1024)
        for _ in range(256): phase.warmup_done()
        for block in range(12):
            phase.begin_collection(str(block))
            for _ in range(256): phase.collected(str(block))
            for _ in range(64): phase.attempt(); phase.accepted()
            phase.finish_block()
        self.assertEqual((phase.fresh, phase.attempts, phase.phase), (3072, 768, 'finished'))
        with self.assertRaises(ContractError): phase.begin_collection('extra')

    def test_real_torch_rejected_attempt_restores_complete_learning_state(self):
        legacy = runner.load_legacy()
        learner = runner.SACLearner(legacy)
        replay = EpisodeReplay([record(c, c) for c in
                                ('back_pre_holding', 'back_holding', 'baseline_preservation')])
        learner.joint_attempt(learner.batch(replay))  # Populate all Adam moments + gradients.
        learner.actor.eval()
        before = learner.capture_learning_state()
        self.assertTrue(all(before['optimizers'][key]['state'] for key in before['optimizers']))
        replay_rng = replay.rng.getstate()
        features = [replay.rows[0].features]
        witness = FixedWitness(features, learner.gaussians(features), learner.policy_id())
        phase = cold.ColdPhase(1024)
        for _ in range(256): phase.warmup_done()
        phase.begin_collection(learner.policy_id())
        for _ in range(256): phase.collected(learner.policy_id())
        original_attempt = learner.joint_attempt
        def violating_attempt(batch):
            metrics = original_attempt(batch)
            with learner.torch.no_grad(): learner.actor.mean.bias.add_(1.)
            return metrics
        with mock.patch.object(learner, 'joint_attempt', side_effect=violating_attempt):
            decision, _, accepted = UpdateGuard(learner, replay, phase, witness).attempt(
                lambda: learner.batch(replay))
        self.assertFalse(accepted)
        self.assertGreater(decision.maximum_deterministic_change, .05)
        recursive_equal(self, before, learner.capture_learning_state())
        self.assertEqual(replay_rng, replay.rng.getstate())
        self.assertEqual((phase.attempts, phase.block_accepted), (1, 0))
        # Serialized checkpoint rehydrates exact optimizer/gradient/RNG state too.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'real-state.pt'
            learner.torch.save(before, path)
            state = learner.torch.load(path, weights_only=False, map_location='cpu')
            learner.joint_attempt(learner.batch(replay))
            learner.restore_learning_state(state)
            recursive_equal(self, before, learner.capture_learning_state())

    def test_mapper_retains_exact_source_tail_and_terminal_discount(self):
        legacy = runner.load_legacy()
        mapper = cold.FreshMapper(legacy.SourceTransition(runner.GAMMA), 'fresh-1', 'zero', 'original')
        def state(native, preview=True):
            return {'native':native, 'impactEpoch':480, 'sourceEpoch':480,
                'status':{'reference_frames':10},
                'preview':{'features':[1.]*1024,'referenceDecision':10} if preview else None}
        first = state(500)
        second = state(520, False)
        second['lastCorrection'] = {'native_substep':500,'correction':[0.]*21}
        self.assertIsNone(mapper.push(first, [0.]*21,
            {'state':second,'native':520,'reward':1.,'terminal':False,'success':False}))
        self.assertIsNotNone(mapper.builder.pending)
        third = state(540, False)
        row = mapper.push(second, None, {'state':third,'native':540,'reward':100.,
                                       'terminal':True,'success':True})
        self.assertEqual((row.pre_native,row.post_native,row.discount,row.category),
                         (500,540,0.,'back_holding'))
        self.assertAlmostEqual(row.reward, 1. + runner.GAMMA * 100.)
        self.assertEqual(len(mapper.completed_rows()), 1)
        with self.assertRaises(ContractError):
            mapper.push(third, None, {'state':third,'native':540,'terminal':False})

    def test_mapper_rejects_action_clock_and_preservation_relabeling(self):
        legacy = runner.load_legacy()
        for defect in ('clock', 'correction'):
            mapper = cold.FreshMapper(legacy.SourceTransition(runner.GAMMA), 'new', 'zero', 'belly')
            source = {'native':500,'impactEpoch':480,'sourceEpoch':480,
                'status':{'reference_frames':100},'preview':{'features':[0.]*1024,'referenceDecision':1}}
            post = copy.deepcopy(source);post['native']=520
            post['lastCorrection']={'native_substep':500,'correction':[0.]*21}
            if defect == 'clock':post['impactEpoch']=500
            else:post['lastCorrection']['correction'][0]=.5
            with self.assertRaises(ContractError):
                mapper.push(source,[0.]*21,dict(state=post,native=520,reward=1.,terminal=True,success=True))

    def test_drive_exchange_exact_bytes_privacy_and_no_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);payload=root/'payload';payload.write_text('synthetic storage test only')
            exchange=cold.DriveExchange(root/'exchange','synthetic-test',
                {'commit':'a'*40,'dirty':False,'description':'synthetic test'},time.time()+60)
            request=exchange.snapshot([cold.artifact('test',payload,'test/v1')],{},
                                      'synthetic-only test','shared')
            value=json.loads(request.read_text())
            receipt=dict(schema='lh-private-drive-receipt/v1',
                **{k:value[k] for k in ('run_id','kind','archive_sha256','archive_bytes','manifest_sha256')},
                owner_only=True,shared=False,drive_file_id='synthetic-not-remote',
                download_completed=True,downloaded_path=value['archive_path'])
            receipt_path=root/'receipt.json';receipt_path.write_text(json.dumps(receipt))
            # This local fake is deliberately only a unit test, no durable claim.
            validator=lambda path,manifest: {'passed':True,'physics_integrations':0}
            exchange.verify_receipt(request,receipt_path,validator)
            self.assertEqual(exchange.shared_sha256,value['manifest_sha256'])
            with self.assertRaises(ContractError):
                exchange.snapshot([cold.artifact('test',payload,'test/v1')],{},'duplicate','shared')
            other=cold.DriveExchange(root/'other','synthetic-test',
                {'commit':'a'*40,'dirty':False,'description':'synthetic test'},time.time()+60)
            receipt['owner_only']=False;receipt_path.write_text(json.dumps(receipt))
            with self.assertRaises(ContractError):other.verify_receipt(request,receipt_path,validator)
            with self.assertRaisesRegex(ContractError,'no retry'):
                other.verify_receipt(request,receipt_path,validator)

    def test_oversized_input_rejected_before_copy_or_transfer_request(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);payload=root/'large'
            with payload.open('wb') as stream:stream.truncate(cold.PROTOCOL['shared_payload_bytes']+1)
            exchange=cold.DriveExchange(root/'exchange','synthetic-test',
                {'commit':'a'*40,'dirty':False,'description':'synthetic test'},time.time()+60)
            with self.assertRaisesRegex(ContractError,'before copying'):
                exchange.snapshot([cold.artifact('test',payload,'test/v1')],{},'oversized','shared')
            self.assertFalse(list((root/'exchange').glob('*.request.json')))

    def test_peak_cap_includes_pending_third_generation_before_upload(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);payload=root/'payload';payload.write_bytes(b'abc')
            exchange=cold.DriveExchange(root/'exchange','synthetic-test',
                {'commit':'a'*40,'dirty':False,'description':'test'},time.time()+60,'a'*64)
            exchange.shared_bytes=32*1024**2
            exchange.verified=[{'payload_bytes':64*1024**2}]*2
            with self.assertRaisesRegex(ContractError,'pending exceeds'):
                exchange.snapshot([cold.artifact('live',payload,'test/v1')],{},'oversized')
            self.assertFalse(list((root/'exchange').glob('*.request.json')))

    def test_replay_rejects_arbitrary_or_changed_runtime_features(self):
        import numpy as np
        from getup_cold_start_verify import verify_replay_mapping
        row=record('back_pre_holding','actual-episode')
        arrays={'features':np.zeros((1,1024),np.float32),
                'correction':np.zeros((1,21),np.float32),
                'next_features':np.ones((1,1024),np.float32)}
        class Arrays(dict):
            @property
            def files(self):return list(self)
        arrays=Arrays(arrays)
        metadata=[{key:value for key,value in __import__('dataclasses').asdict(row).items()
            if key not in ('features','correction','next_features')}]
        mapping=[{'replay_index':0,'transition_line_start':0,'transition_line_end':0}]
        raw=[{'episodeId':row.episode,'behaviorPolicyId':row.behavior,
            'preNative':500,'postNative':520,'impactEpoch':480,'reward':.1,'terminal':False,
            'correction':[0.]*21,'preObservation':[0.]*118,
            'sourceFeaturesSha256':hashlib.sha256(arrays['features'][0].tobytes()).hexdigest(),
            'nextFeaturesSha256':hashlib.sha256(arrays['next_features'][0].tobytes()).hexdigest()}]
        self.assertTrue(verify_replay_mapping(arrays,metadata,mapping,raw,runner.GAMMA)['passed'])
        for key in ('features','next_features'):
            changed=Arrays({name:value.copy() for name,value in arrays.items()})
            changed[key].fill(777)
            with self.assertRaisesRegex(ContractError,'actual runtime'):
                verify_replay_mapping(changed,metadata,mapping,raw,runner.GAMMA)

    def test_application_restore_precedes_old_generation_prune_request(self):
        with tempfile.TemporaryDirectory() as directory:
            import threading
            root=Path(directory);payload=root/'payload';payload.write_bytes(b'abc')
            exchange=cold.DriveExchange(root/'exchange','synthetic-test',
                {'commit':'a'*40,'dirty':False,'description':'test'},time.time()+60,'a'*64)
            old=[]
            for i in range(2):
                owned=exchange.directory/'bundles'/str(i);owned.mkdir(parents=True)
                archive=owned/'old.zip';archive.write_bytes(b'old')
                old.append({'manifest_sha256':str(i)*64,'drive_file_id':'old-'+str(i),
                            'payload_bytes':3,'archive_path':str(archive)})
            exchange.verified=old
            request=exchange.snapshot([cold.artifact('live',payload,'test/v1')],{},'replacement')
            value=json.loads(request.read_text())
            receipt=dict(schema='lh-private-drive-receipt/v1',
                **{k:value[k] for k in ('run_id','kind','archive_sha256','archive_bytes','manifest_sha256')},
                owner_only=True,shared=False,drive_file_id='synthetic-new',
                download_completed=True,downloaded_path=value['archive_path'])
            receipt_path=root/'receipt.json';receipt_path.write_text(json.dumps(receipt))
            def validator(path,manifest):
                self.assertFalse(list(exchange.directory.glob('*.prune-request.json')))
                self.assertTrue(Path(old[0]['archive_path']).exists())
                return {'passed':True,'physics_integrations':0}
            def acknowledge():
                while not list(exchange.directory.glob('*.prune-request.json')):time.sleep(.005)
                p=next(exchange.directory.glob('*.prune-request.json'));data=json.loads(p.read_text())
                self.assertTrue(Path(data['application_verified']).exists())
                cold.atomic_json(data['receipt'],{
                    'replacement_manifest_sha256':data['replacement_manifest_sha256'],
                    'deleted_drive_file_ids':data['drive_file_ids']})
            thread=threading.Thread(target=acknowledge);thread.start()
            exchange.verify_receipt(request,receipt_path,validator);thread.join()
            self.assertFalse(Path(old[0]['archive_path']).exists())
            self.assertTrue(Path(old[1]['archive_path']).exists())
            self.assertEqual(len(exchange.verified),2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
