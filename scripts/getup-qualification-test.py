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
    def call(self,q):
        if q['op']=='qualificationStorage':
            if self.byte_stop_after is not None and self.physical_calls>=self.byte_stop_after:
                raise ContractError('Qualification byte stop BEFORE dispatch')
            return {'synthetic':True,'terminalBaseBytes':q['terminalBaseBytes']}
        if q['op']=='qualificationCapture':
            return {'physicsIntegrations':0,'files':[],'activeKnown':self.state is not None,
                'state':copy.deepcopy(self.state)}
        if q['op']=='qualificationStage':return {'synthetic':True,'name':q['name']}
        if q['op']=='coldQualify':
            self.charge(real=1000,forecast=10000)
            self.ledger['trainingDecisions']+=50;self.ledger['bootstrapDecisions']+=50
            return {'passed':True,'synthetic':True,'physicsIntegrations':11000}
        result=super().call(q)
        if q['op']=='reset': # first source decision produced inside initialization is counted
            self.ledger['trainingDecisions']+=1;self.ledger['bootstrapDecisions']+=1
        if q['op']==self.fail_after_operation:
            if q['op']=='hold':
                self.index+=1;self.state=self.current(preview=False)
            raise ContractError('scripted known failure after '+q['op'])
        return result

def build(root,**options):
    clock=ScriptClock();plan=QualificationPlan(clock.now(),clock.now()+3600,clock.now()+2100,clock.now()+3300)
    output=Path(root)/'pilot';bridge=QualificationPhysics(output,clock,plan,**options)
    transport=LocalRehearsalTransport(Path(root)/'recovery',clock)
    recovery=CoherentRecovery(transport,bridge.independent_controller,synthetic=True)
    factory,builder=engine.factories();learner=factory()
    pilot=QualificationPilot(output=output,learner=learner,learner_factory=factory,
        builder_factory=builder,bridge=bridge,recovery=recovery,clock=clock,plan=plan,
        identity=engine.IDENTITY,synthetic=True,byte_guard=QualificationByteGuard(untouched_learner_bytes(learner)))
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
            self.assertEqual(t.requests,['qualification-terminal']);self.assertEqual(len(t.sizes),1)
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
            print('QUALIFICATION_SYNTHETIC_BYTES',t.sizes[0], 'PAYLOAD',sum(x['bytes'] for x in manifest['files']))
    def test_optional_original_is_recovery_only_no_extra_hold(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp);r=p.run_qualification(include_original=True)
            self.assertEqual(r['stopped'],'qualification-only-complete-no-training',r)
            self.assertFalse(r['originalDiagnostic']['extra30sHold'])
            self.assertEqual(p.bridge.calls.count('hold'),2)
            self.assertEqual(len(t.requests),1)
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
            self.assertEqual(len(t.requests),1)
    def test_uncertain_execution_retained_once_never_retried_or_coherent(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=build(tmp,fail_call=5);r=p.run_qualification()
            self.assertEqual(r['stopped'],'qualification-safe-stop');self.assertEqual(p.bridge.physical_calls,5)
            self.assertEqual(r['ledger']['uncertainIntegrations'],5020)
            self.assertFalse(r.get('syntheticQualificationRestorePassed',False))
            self.assertEqual(len(t.requests),1)
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

if __name__=='__main__':unittest.main()
