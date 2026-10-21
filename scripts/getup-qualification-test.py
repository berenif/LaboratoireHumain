#!/usr/bin/env python3
"""Qualification production path: scripted physics/transfer, real state/archive restore.
No WASM runtime, Drive write, experimental data or optimizer operation is used.
"""
import copy,importlib.util,json,tempfile,unittest
from pathlib import Path
from unittest import mock
from getup_cold_start_engine import CoherentRecovery
from getup_cold_start_rehearsal import ScriptClock,ScriptedPhysics,LocalRehearsalTransport
from getup_qualification import *
from getup_learning_loop import ContractError
from experiment_backup import verify_bundle,sha256
spec=importlib.util.spec_from_file_location('engine_tests',Path(__file__).with_name('getup-cold-start-engine-test.py'))
engine=importlib.util.module_from_spec(spec);spec.loader.exec_module(engine)

class QualificationPhysics(ScriptedPhysics):
    def __init__(self,*args,byte_stop_after=None,fail_after_operation=None,**kwargs):
        super().__init__(*args,**kwargs);self.byte_stop_after=byte_stop_after
        self.fail_after_operation=fail_after_operation
    def charge(self,real=20,forecast=0):
        before=copy.deepcopy(self.ledger)
        super().charge(real,forecast)
        row={'synthetic':True,'preNative':before['realIntegrations'],'postNative':self.ledger['realIntegrations'],
            'before':{'forecast':before['forecastIntegrations']},'after':{'forecast':self.ledger['forecastIntegrations']},
            'callSequence':self.ledger['completedCalls'],'ledgerNativeEnd':self.ledger['nativeIntegrations'],'failure':None}
        with (self.output/'qualification-native.ndjson').open('a') as stream:stream.write(json.dumps(row)+'\n')
    def call(self,q):
        if q['op']=='qualificationStorage':
            if self.byte_stop_after is not None and self.physical_calls>=self.byte_stop_after:
                raise ContractError('Qualification byte stop BEFORE dispatch')
            return {'synthetic':True,'terminalBaseBytes':q['terminalBaseBytes']}
        if q['op']=='qualificationRelease':
            self.state=None;return {'physicsIntegrations':0}
        if q['op']=='qualificationCapture':
            return {'physicsIntegrations':0,'files':[],'activeKnown':self.state is not None,
                'state':copy.deepcopy(self.state)}
        if q['op']=='qualificationStage':return {'synthetic':True,'name':q['name']}
        if q['op']=='coldQualify':
            self.charge(real=1000,forecast=10000)
            self.ledger['trainingDecisions']+=50;self.ledger['bootstrapDecisions']+=50
            path=self.output/'native-parity-resume.bin';path.write_text(json.dumps(self.checkpoint()))
            value={'passed':True,'synthetic':True,'physicsIntegrations':11000,
                'zeroCorrectionSamples':400,'resumedSamples':200,'resumeCheckpointSha256':sha256(path)}
            atomic_json(self.output/'cold-adapter-parity.json',value)
            atomic_json(self.output/'cold-fixtures.json',{'synthetic':True,'nativePreparation':3040})
            return value
        result=super().call(q)
        if q['op']=='reset': # first source decision produced inside initialization is counted
            self.ledger['trainingDecisions']+=1;self.ledger['bootstrapDecisions']+=1
        if q['op']=='hold':
            for suffix in ('first800','30s-hold'):
                path=self.output/(self.label+'-'+suffix+'-checkpoint.bin')
                path.write_text(json.dumps(self.checkpoint()))
            result.update(samples=24000,physicalSeconds=30,finalCheckpointSha256=sha256(path))
            atomic_json(self.output/(self.label+'-30s-hold.json'),result)
        if q['op']==self.fail_after_operation:
            if q['op']=='hold':
                self.index+=1;self.state=self.current(preview=False)
            raise ContractError('scripted known failure after '+q['op'])
        return result

def build(root,**options):
    clock=ScriptClock();plan=QualificationPlan(clock.now(),clock.now()+3600,clock.now()+2100,clock.now()+3300)
    output=Path(root)/'pilot';bridge=QualificationPhysics(output,clock,plan,**options)
    transport=LocalRehearsalTransport(Path(root)/'recovery',clock)
    transport.exchange.run_id=engine.IDENTITY['runId']
    recovery=CoherentRecovery(transport,bridge.independent_controller,synthetic=True)
    factory,builder=engine.factories();learner=factory()
    pilot=QualificationPilot(output=output,learner=learner,learner_factory=factory,
        builder_factory=builder,bridge=bridge,recovery=recovery,clock=clock,plan=plan,
        native_validator=lambda paths:{'passed':all(isinstance(json.loads(p.read_text()),dict) for p in paths),'physics_integrations':0},
        identity=dict(engine.IDENTITY,sharedManifestSha256='b'*64,seedManifestSha256='c'*64),synthetic=True,byte_guard=QualificationByteGuard(untouched_learner_bytes(learner)))
    atomic_json(output/'phase-inputs.json',{'identity':pilot.identity,**{name:{'drive_file_id':'synthetic-'+name,
        'archive_sha256':'a'*64,'archive_bytes':1,'manifest_sha256':digest*64,'members':[{'synthetic':True}]}
        for name,digest in [('shared','b'),('seeds','c')]}})
    return pilot,transport

class QualificationTests(unittest.TestCase):
    def test_complete_partial_strata_fresh_restore_one_slot_no_optimizer(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp)
            with mock.patch.object(p.learner,'joint_attempt',side_effect=AssertionError('NO OPTIMIZER')):
                report=p.run_qualification()
            self.assertEqual(report['stopped'],'qualification-only-complete-no-training',report)
            self.assertTrue(report['syntheticQualificationRestorePassed'],report)
            self.assertFalse(report['actualQualificationRestorePassed'])
            self.assertEqual(t.requests,['qualification-parity','qualification-belly','qualification-terminal']);self.assertEqual(len(t.sizes),3)
            restored=p.recovery.last_restored
            self.assertIsNot(restored.learner,p.learner);self.assertIsNot(restored.replay,p.replay)
            self.assertTrue(restored.replay.diagnostic_only)
            self.assertIn('back_pre_holding',report['diagnosticReplay']['missingStrata'])
            with self.assertRaises(ContractError):restored.replay.batch_indices(128)
            with self.assertRaises(ContractError):p.optimizer_gate()
            self.assertEqual(report['ledger']['evaluationReserveIntegrations'],147000)
            self.assertEqual(p.bridge.calls.count('hold'),2)
            manifest=verify_bundle(t.retained[-1]['archive'],sha256(t.retained[-1]['archive']))
            self.assertFalse(manifest['application_resume_eligible'])
            print('QUALIFICATION_SYNTHETIC_BYTES',t.sizes, 'TERMINAL_PAYLOAD',sum(x['bytes'] for x in manifest['files']))
    def test_optional_original_is_permanently_rejected_before_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);r=p.run_qualification(include_original=True)
            self.assertEqual(r['stopped'],'qualification-safe-stop')
            self.assertIn('permanently omitted',r['error'])
            self.assertEqual(p.bridge.physical_calls,0)
            self.assertEqual(t.requests,['qualification-terminal'])
    def test_byte_stop_after_parity_retains_partial_populated_restore(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp,byte_stop_after=4);r=p.run_qualification()
            self.assertEqual(r['stopped'],'qualification-safe-stop')
            self.assertIn('byte stop',r['error'])
            self.assertEqual(p.bridge.physical_calls,4)
            self.assertTrue(r['syntheticQualificationRestorePassed'],r)
            self.assertIn('back_pre_holding',r['diagnosticReplay']['missingStrata'])
            self.assertIn('back_holding',r['diagnosticReplay']['missingStrata'])
            self.assertEqual(r['diagnosticReplay']['approvedStratumRows']['baseline_preservation'],0)
            self.assertEqual(len(t.requests),2)
    def test_uncertain_execution_retained_once_never_retried_or_coherent(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp,fail_call=5);r=p.run_qualification()
            self.assertEqual(r['stopped'],'qualification-safe-stop');self.assertEqual(p.bridge.physical_calls,5)
            self.assertEqual(r['ledger']['uncertainIntegrations'],5020)
            self.assertFalse(r.get('syntheticQualificationRestorePassed',False))
            self.assertEqual(len(t.requests),2)
    def test_phase_cutoff_and_growth_source_identity_without_physics(self):
        validate_growth_sources(Path(__file__).resolve().parents[1])
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);p.clock.advance(2050)
            with self.assertRaisesRegex(ContractError,'cutoff'):
                p.enter('transition-parity',60,5060)
            self.assertEqual(p.bridge.physical_calls,0)
            with self.assertRaises(ContractError):p.plan.admit_sequence(1,{})
    def test_known_failure_inside_reset_or_hold_keeps_actual_active_runtime(self):
        from experiment_backup import restore_bundle
        for op in ('reset','hold'):
            with self.subTest(op=op),tempfile.TemporaryDirectory() as tmp:
                p,t=build(tmp,fail_after_operation=op);r=p.run_qualification()
                self.assertIn('known failure',r['error']);self.assertNotIn('terminalRetentionError',r)
                archive=t.retained[-1]['archive']
                target=Path(tmp)/'inspect';manifest=restore_bundle(archive,target,sha256(archive))
                files={x['role']:target/x['path'] for x in manifest['files']}
                self.assertIn('runtime',files)
                cursor=json.loads(files['cursor'].read_text())
                self.assertEqual(cursor['state'],p.bridge.state)
                self.assertTrue(cursor['runtime_active'])
                self.assertFalse(manifest['application_resume_eligible'])
    def test_early_preparation_seed_is_retained_once_until_immutable_verified(self):
        from experiment_backup import restore_bundle
        for verified in (False,True):
            with self.subTest(verified=verified),tempfile.TemporaryDirectory() as tmp:
                p,t=build(tmp)
                p.identity=dict(p.identity,seedManifestSha256='c'*64 if verified else None)
                exact=b'synthetic exact first prepared checkpoint; no physics'
                (p.output/'original-seed.bin').write_bytes(exact)
                # Later failed tracked runtime exists without a completed
                # Python reset/mapper; terminal capture must preserve both.
                p.bridge.case='belly';p.bridge.label='failed-preparation'
                p.bridge.state=p.bridge.current()
                p.failure_latched=True;p.report['error']='scripted later preparation failure'
                r=p.finalize();self.assertNotIn('terminalRetentionError',r)
                archive=t.retained[-1]['archive'];target=Path(tmp)/'inspect'
                manifest=restore_bundle(archive,target,sha256(archive))
                files={x['role']:target/x['path'] for x in manifest['files']}
                role='unverified_preparation_original_seed'
                self.assertEqual(role in files,not verified)
                if not verified:self.assertEqual(files[role].read_bytes(),exact)
                self.assertIn('runtime',files)
                self.assertFalse(manifest['application_resume_eligible'])
    def test_oversized_unverified_seed_is_diagnostic_under_terminal_caps(self):
        from experiment_backup import restore_bundle
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);p.identity=dict(p.identity,seedManifestSha256=None)
            path=p.output/'original-seed.bin';path.write_bytes(b's'*(2*1024**2+1))
            p.failure_latched=True;p.report['error']='immutable seed allowance exceeded'
            r=p.finalize();self.assertNotIn('terminalRetentionError',r)
            self.assertTrue(r['immutableSeedAdmissionFailed'])
            self.assertFalse(r.get('syntheticQualificationRestorePassed',False))
            archive=t.retained[-1]['archive'];target=Path(tmp)/'inspect'
            manifest=restore_bundle(archive,target,sha256(archive))
            files={x['role']:target/x['path'] for x in manifest['files']}
            self.assertEqual(sha256(files['unverified_preparation_original_seed']),sha256(path))
            self.assertFalse(manifest['application_resume_eligible'])
            self.assertLess(sum(x['bytes'] for x in manifest['files']),32*1024**2)
            self.assertLess(archive.stat().st_size,18*1024**2)

    def test_phase_cache_eviction_occurs_only_after_exact_restore_and_durable_reference(self):
        from getup_phase_proofs import reference_hash
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);seen=[]
            old=Path(tmp)/'old-closed-proof';old.write_bytes(b'never evict preexisting evidence')
            def verified(archive,manifest,boundary):
                journal=json.loads(p.proofs.path.read_text())
                seen.append((boundary,len(journal['references'])))
                # Adapter has restored, but commit/eviction has not happened.
                self.assertTrue(archive.is_file())
                if boundary=='qualification-parity':
                    self.assertTrue((p.output/'native-parity-resume.bin').is_file())
                    self.assertNotIn('learner',[x['role'] for x in manifest['files']])
                if boundary=='qualification-belly':
                    self.assertTrue((p.output/'parity-runtime.bin').is_file())
                    self.assertNotIn('learner',[x['role'] for x in manifest['files']])
            t.after_verified=verified;r=p.run_qualification()
            self.assertNotIn('terminalRetentionError',r)
            self.assertEqual(seen,[('qualification-parity',0),('qualification-belly',1),('qualification-terminal',2)])
            self.assertEqual(old.read_bytes(),b'never evict preexisting evidence')
            self.assertEqual(len(p.proofs.references),2)
            for ref in p.proofs.references:
                copyref=dict(ref);digest=copyref.pop('reference_sha256')
                self.assertEqual(reference_hash(copyref),digest)
                self.assertTrue((t.remote/ref['archive_sha256']).is_file())
            self.assertFalse((p.output/'native-parity-resume.bin').exists())
            self.assertFalse((p.output/'parity-runtime.bin').exists())
            self.assertFalse(list(t.root.glob('dependency-cache/*.zip')))
            self.assertEqual(t.cumulative,t.exchange.remote_uploaded_bytes)
            self.assertEqual(len(p.recovery.last_restored.rows),6)
            self.assertEqual({x['case'] for x in p.recovery.last_restored.mapping},{'belly','heldout'})
            self.assertEqual(p.recovery.last_restored.mapping,p.mapping)
            self.assertGreater(len(p.prior_raw_bytes),0)

    def test_missing_or_corrupt_dependency_stops_before_nearby_no_evidence_eviction(self):
        for corruption in ('missing','corrupt'):
            with self.subTest(corruption=corruption),tempfile.TemporaryDirectory() as tmp:
                p,t=build(tmp);original=p.preservation
                def preservation(case):
                    original(case)
                    if case=='belly':
                        path=t.remote/p.proofs.references[0]['archive_sha256']
                        path.unlink() if corruption=='missing' else path.write_bytes(b'corrupt')
                p.preservation=preservation;r=p.run_qualification()
                self.assertEqual(r['stopped'],'qualification-safe-stop')
                self.assertIn('terminalRetentionError',r)
                self.assertEqual(len(p.proofs.references),1)
                self.assertEqual(p.bridge.calls.count('hold'),1)
                self.assertTrue((p.output/'parity-runtime.bin').is_file())
                self.assertTrue((p.output/'transitions.ndjson').is_file())
                self.assertEqual(t.requests,['qualification-parity','qualification-belly'])

    def test_failed_or_delayed_immutable_write_is_charged_without_retry_or_eviction(self):
        for mode in ('failed','delayed','application'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as tmp:
                p,t=build(tmp);before=t.exchange.remote_uploaded_bytes
                if mode=='failed':t.fail_boundary='qualification-parity'
                elif mode=='delayed':t.delay=181
                else:p.proofs.native_validator=lambda paths:{'passed':False,'physics_integrations':0}
                r=p.run_qualification()
                self.assertEqual(r['stopped'],'qualification-safe-stop')
                self.assertEqual(t.requests,['qualification-parity'])
                self.assertEqual(p.bridge.physical_calls,1)
                self.assertTrue((p.output/'native-parity-resume.bin').is_file())
                self.assertEqual(p.proofs.references,[])
                self.assertGreater(t.exchange.remote_uploaded_bytes,before)
                self.assertTrue(t.exchange.pending_immutable)
                self.assertFalse(list(t.root.glob('*.cache-evicted.json')))

    def test_dependency_identity_ledger_and_membership_cannot_be_mixed(self):
        from getup_phase_proofs import ledger_interval,reference_hash
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);p.run_qualification()
            before=p.proofs.references[0]['ledger_end']
            for change in ({'nativeIntegrations':0},{'deadlineUnixMs':before['deadlineUnixMs']+1},
                           {'uncertainIntegrations':5020}):
                with self.assertRaises(ContractError):ledger_interval(before,dict(before,**change))
            restored=p.recovery.last_restored;prior=copy.deepcopy(restored.rows[:3])
            p.verify_dependency_union(restored,{'prior_rows':prior})
            altered=copy.copy(restored);altered.rows=list(reversed(restored.rows))
            with self.assertRaises(ContractError):p.verify_dependency_union(altered,{'prior_rows':prior})
            altered=copy.copy(restored);altered.mapping=list(reversed(restored.mapping))
            with self.assertRaises(ContractError):p.verify_dependency_union(altered,{'prior_rows':prior})
            ref=dict(p.proofs.references[0]);digest=ref.pop('reference_sha256')
            ref['drive_file_id']='another-file-with-same-name'
            self.assertNotEqual(reference_hash(ref),digest)

    def test_interrupted_eviction_keeps_reference_and_never_duplicates_raw_or_replays(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);original=Path.unlink;failed=[False]
            def interrupted(path,*args,**kwargs):
                if path==p.output/'phase-mapping.json' and len(p.proofs.references)==2 and not failed[0]:
                    failed[0]=True;raise OSError('scripted interruption after journal commit')
                return original(path,*args,**kwargs)
            with mock.patch.object(Path,'unlink',interrupted):r=p.run_qualification()
            self.assertTrue(failed[0]);self.assertEqual(r['stopped'],'qualification-safe-stop')
            self.assertIn('interruption after journal',r['error'])
            self.assertNotIn('terminalRetentionError',r)
            self.assertEqual(len(p.proofs.references),2)
            self.assertEqual(p.bridge.calls.count('hold'),1)
            self.assertTrue(r['syntheticQualificationRestorePassed'])
            self.assertEqual(len(p.recovery.last_restored.rows),3)
            self.assertEqual(p.current_raw_bytes(),b'')
            self.assertEqual(len(p.raw_lines()),4) # executed parity duplicate remains exactly once
            self.assertTrue((p.output/'transitions.ndjson').exists())
            self.assertEqual(t.requests,['qualification-parity','qualification-belly','qualification-terminal'])

    def test_oversize_archive_is_retained_locally_before_any_transfer_or_eviction(self):
        import os,types
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);stat=Path.stat
            def oversized(path,*args,**kwargs):
                value=stat(path,*args,**kwargs)
                if path.suffix=='.zip' and 'bundles' in path.parts:
                    values=list(value);values[6]=18*1024**2+1;return os.stat_result(values)
                return value
            with mock.patch.object(Path,'stat',oversized):r=p.run_qualification()
            self.assertIn('compressed bundle exceeds',r['error'])
            self.assertEqual(p.bridge.physical_calls,1)
            self.assertTrue(list(t.root.glob('bundles/*/*.zip')))
            self.assertFalse(list(t.root.glob('*.receipt.json')))
            self.assertFalse(list(t.root.glob('*.cache-evicted.json')))
            self.assertTrue((p.output/'native-parity-resume.bin').is_file())
            self.assertEqual(p.proofs.references,[])

    def test_dependency_readback_peak_is_checked_before_fetch(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);p.run_qualification();ref=p.proofs.references[0]
            t.exchange.external_local_bytes=192*1024**2
            with mock.patch.object(t.port,'dependency_fetch',side_effect=AssertionError('NO DOWNLOAD')):
                with self.assertRaisesRegex(ContractError,'staging'):
                    t.dependency(ref,p.plan.deadline)

if __name__=='__main__':unittest.main()
