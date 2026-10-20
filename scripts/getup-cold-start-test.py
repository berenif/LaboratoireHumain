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
from unittest import mock

import getup_cold_start as cold
from getup_learning_loop import (ContractError, EpisodeReplay, FixedWitness, Record,
                                 UpdateGuard)
from experiment_backup import sha256

spec = importlib.util.spec_from_file_location('corrected_runner',
    Path(__file__).with_name('train-getup-learning-loop.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


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
