#!/usr/bin/env python3
"""Preservation production path; scripted physics/private transport ONLY.
Real mapper/replay/learner/archive mechanics. No native worker or network calls.
"""
import copy,hashlib,importlib.util,json,random,shutil,tempfile,unittest,zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
from experiment_backup import create_snapshot,restore_bundle,verify_bundle,sha256,atomic_json
from getup_cold_start import artifact
from getup_external_parity import validate_external_parity,restore_external_parity
from getup_qualification import QualificationPilot,QualificationPlan,QualificationByteGuard,untouched_learner_bytes,PLAN
from getup_cold_start_engine import CoherentRecovery,restore_coherent
from dataclasses import asdict
from getup_cold_start_rehearsal import ScriptClock,LocalRehearsalTransport
from getup_learning_loop import ContractError
spec=importlib.util.spec_from_file_location('qualification_tests',Path(__file__).with_name('getup-qualification-test.py'))
qtests=importlib.util.module_from_spec(spec);spec.loader.exec_module(qtests)
engine=qtests.engine


def doc(path):return {'path':str(path),'sha256':sha256(path)}

def input_bundle(root,roles,kind):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    paths={role:root/(role+'.bin') for role in roles}
    for role,path in paths.items():path.write_text(json.dumps({'synthetic':True,'role':role}))
    archive=create_snapshot(root/'bundles',{'run_id':'foreign-native-parity','source':{'commit':'a'*40,'dirty':False,'description':'Disposable software-only fixture'},
        'synthetic':True,'provenance':'DISPOSABLE SOFTWARE REHEARSAL','application_resume_eligible':False,
        'config':{},'seed':20261020,'artifacts':[artifact(k,v,'synthetic-input') for k,v in paths.items()],
        'required_roles':list(paths),'excluded_dependencies':[]})
    m=verify_bundle(archive,sha256(archive))
    with zipfile.ZipFile(archive) as z:mh=hashlib.sha256(z.read('manifest.json')).hexdigest()
    request={'run_id':'foreign-native-parity','kind':kind,'archive_path':str(archive),'archive_sha256':sha256(archive),
        'archive_bytes':archive.stat().st_size,'manifest_sha256':mh,'synthetic':True,'payload_bytes':sum(x['bytes'] for x in m['files'])}
    receipt={**request,'schema':'lh-private-drive-receipt/v1','drive_file_id':'synthetic-'+kind,'owner_only':True,'shared':False,'download_completed':True}
    application={'passed':True,'physics_integrations':0,'archive_sha256':sha256(archive),'synthetic':True}
    for name,value in [('request',request),('receipt',receipt),('application',application)]:atomic_json(root/(name+'.json'),value)
    proof={k:doc(root/(k+'.json')) for k in ('request','receipt','application')}
    proof['archive']={'path':str(archive),'sha256':sha256(archive),'bytes':archive.stat().st_size}
    return proof,m,paths,request


def prior_fixture(root):
    root=Path(root);p,t=qtests.build(root/'old')
    shared,sm,sp,sq=input_bundle(root/'shared',('worker_wasm','worker_glue','profile','model','weak','slow','source_identity','back_unstepped','belly_unstepped'),'shared')
    seeds,sem,sep,seq=input_bundle(root/'seeds',('original_seed','heldout_seed','belly_seed'),'seeds')
    p.identity.update(runId='foreign-native-parity',sourceCommit='a'*40,
        workerSha256=next(x['sha256'] for x in sm['files'] if x['role']=='worker_wasm'),
        protocolSha256='c'*64,sharedManifestSha256=sq['manifest_sha256'],seedManifestSha256=seq['manifest_sha256'])
    t.exchange.run_id=p.identity['runId'];t.exchange.shared_sha256=sq['manifest_sha256'];t.exchange.seed_sha256=seq['manifest_sha256']
    inputs={'identity':p.identity,'shared':{'drive_file_id':'synthetic-shared','archive_sha256':sq['archive_sha256'],'archive_bytes':sq['archive_bytes'],'manifest_sha256':sq['manifest_sha256'],'members':sm['files']},
        'seeds':{'drive_file_id':'synthetic-seeds','archive_sha256':seq['archive_sha256'],'archive_bytes':seq['archive_bytes'],'manifest_sha256':seq['manifest_sha256'],'members':sem['files']}}
    atomic_json(p.output/'phase-inputs.json',inputs)
    for i in range(122):p.bridge.charge(real=20 if i<119 else 0,forecast=1020 if i<2 else 0)
    p.bridge.ledger.update(trainingDecisions=50,bootstrapDecisions=50,qualificationWeakDecisions=102)
    atomic_json(p.output/'budget-ledger.json',p.bridge.ledger)
    cp=p.output/'native-parity-resume.bin';cp.write_text(json.dumps(p.bridge.checkpoint()))
    parity={'passed':True,'synthetic':True,'physicsIntegrations':4420,'zeroCorrectionSamples':400,'resumedSamples':200,'resumeCheckpointSha256':sha256(cp)}
    atomic_json(p.output/'cold-adapter-parity.json',parity);p.report['nativeQualification']=parity
    atomic_json(p.output/'cold-fixtures.json',{'cases':{case:{'checkpointSha256':sha256(sep[case+'_seed'])} for case in ('original','heldout','belly')}})
    p.proofs.persist();p.physical_proof_mode='PARITY'
    proof=p.recovery.checkpoint(p,'qualification-parity',p.clock.now()+180);p.physical_proof_mode=None
    request=Path(proof['request_path']);rq=json.loads(request.read_text());archive=Path(rq['archive_path'])
    pp={k:doc(request.with_name(request.name.replace('.request.json',suffix))) for k,suffix in
        [('request','.request.json'),('receipt','.receipt.json'),('application','.application-verified.json')]}
    pp['archive']={'path':str(archive),'sha256':sha256(archive),'bytes':archive.stat().st_size}
    prerequisite={'schema':'lh-preservation-external-prerequisite/v1','parity':pp,'seeds':seeds,
        'original_identity':p.identity,'original_ledger':doc(p.output/'budget-ledger.json')}
    result=validate_external_parity(prerequisite,sm,'new-preservation','c'*64,synthetic=True)
    return SimpleNamespace(prerequisite=prerequisite,validated=result,shared=shared,shared_manifest=sm,
        shared_paths=sp,old=p,transport=t,archive=archive,ledger_sha=sha256(p.output/'budget-ledger.json'))


def new_pilot(root,prior,**bridge_options):
    root=Path(root);clock=ScriptClock(prior.old.plan.started+10000)
    plan=QualificationPlan(clock.now(),clock.now()+3600,clock.now()+2100,clock.now()+3300,preservation_only=True)
    output=root/'pilot';bridge=qtests.QualificationPhysics(output,clock,plan,**bridge_options)
    transport=LocalRehearsalTransport(root/'recovery',clock)
    transport.exchange.run_id='new-preservation'
    shutil.copyfile(prior.archive,transport.remote/sha256(prior.archive))
    factory,builder=engine.factories();learner=factory();identity=dict(prior.old.identity,runId='new-preservation',sourceCommit='d'*40)
    recovery=CoherentRecovery(transport,bridge.independent_controller,synthetic=True)
    pilot=QualificationPilot(output=output,learner=learner,learner_factory=factory,builder_factory=builder,
        bridge=bridge,recovery=recovery,clock=clock,plan=plan,identity=identity,synthetic=True,
        byte_guard=QualificationByteGuard(untouched_learner_bytes(learner)),
        external_qualification=prior.validated['reference'],
        native_validator=lambda paths:{'passed':all(isinstance(json.loads(p.read_text()),dict) for p in paths),'physics_integrations':0})
    inputs=json.loads((prior.old.output/'phase-inputs.json').read_text());inputs['identity']=identity
    atomic_json(output/'phase-inputs.json',inputs)
    return pilot,transport


def rng_fingerprint():
    import numpy as np,torch
    return engine.fingerprint((random.getstate(),np.random.get_state(),torch.get_rng_state()))


class PreservationTests(unittest.TestCase):
    def test_complete_new_chain_omits_history_and_two_archive_cadence(self):
        with tempfile.TemporaryDirectory() as tmp:
            prior=prior_fixture(Path(tmp)/'prior');p,t=new_pilot(Path(tmp)/'new',prior)
            with mock.patch.object(p.learner,'joint_attempt',side_effect=AssertionError('NO OPTIMIZER')):
                result=p.run_qualification()
            self.assertNotIn('terminalRetentionError',result);self.assertTrue(result['syntheticQualificationRestorePassed'],result)
            self.assertEqual(t.requests,['qualification-belly','qualification-terminal'])
            self.assertEqual(p.proof_order,['BELLY']);self.assertEqual(len(p.proofs.references),1)
            self.assertFalse({'coldPrepare','coldQualify'}&set(p.bridge.calls))
            self.assertEqual(sha256(prior.old.output/'budget-ledger.json'),prior.ledger_sha)
            self.assertLess(result['ledger']['bootstrapDecisions'],50)
            self.assertEqual(result['sourceDecisionCap'],974);self.assertEqual(result['maximumQualificationIntegrations'],103040)
            self.assertEqual(len(p.raw_lines()),len(p.mapping)+1) # retain both parity calls; exclude only duplicate
            self.assertEqual(len(p.excluded_lines),1)
            self.assertEqual({x['case'] for x in p.mapping},{'belly','heldout'})
            journal=json.loads(p.proofs.path.read_text());self.assertEqual(journal['external_qualification']['original_ledger']['nativeIntegrations'],4420)
            self.assertEqual(journal['references'][0]['ledger_begin']['nativeIntegrations'],0)
            self.assertIsNot(p.recovery.last_restored.learner,p.learner)
            for request in t.exchange.directory.glob('*.request.json'):
                archive=Path(json.loads(request.read_text())['archive_path'])
                if archive.exists():
                    manifest=verify_bundle(archive,sha256(archive))
                    config=manifest['config']['cold_start']
                    self.assertGreaterEqual(config['dependency_peak_bytes'],prior.validated['reference']['archive_bytes']+prior.validated['reference']['payload_bytes'])
            print('PRESERVATION_SYNTHETIC_ARCHIVE_BYTES',t.sizes)

    def test_full_production_cli_seed_reuse_and_no_historical_physical_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);prior=prior_fixture(root/'prior');clock=ScriptClock(prior.old.plan.started+10000)
            spec=importlib.util.spec_from_file_location('preservation_cli',Path(__file__).with_name('train-getup-cold-start.py'))
            entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
            output=root/'new-output';shared=root/'current-inputs';shared.mkdir()
            atomic_json(shared/'input-manifest.json',{'software_rehearsal':True})
            worker=shared/'worker';worker.mkdir();(worker/'lh_worker.js').write_text('SCRIPTED ONLY');(worker/'lh_worker_bg.wasm').write_bytes(b'SCRIPTED ONLY')
            transport=LocalRehearsalTransport(root/'new-recovery',clock)
            shutil.copyfile(prior.archive,transport.remote/sha256(prior.archive))
            transport.exchange.run_id='new-preservation'
            source={'commit':'d'*40,'dirty':False,'description':'DISPOSABLE SOFTWARE REHEARSAL'}
            transport.exchange.source=source
            shared_docs={key:json.loads(Path(prior.shared[key]['path']).read_text()) for key in ('request','receipt','application')}
            shared_payload=shared_docs['request']['payload_bytes'];seed_payload=sum(x['bytes'] for x in prior.validated['seed_manifest']['files'])
            admission={'mode':'preservation-only','started_at_unix':clock.now(),'software_readiness':{'sha256':'e'*64},
                'include_original_diagnostic':False,'corrected_shared_archive':prior.shared['archive'],
                'verified_shared':prior.shared,'_verified_shared':shared_docs,
                '_external_qualification':prior.validated,'_external_prerequisite':prior.prerequisite,
                '_verified_prior_generations':copy.deepcopy(transport.exchange.verified),'_qualification_storage':{},
                'storage':{'cumulative_uploaded_bytes':44717304,'staging_bytes':192*1024**2,
                    'remote_payload_bytes':shared_payload+seed_payload+10,'protected_local_backup_bytes':0}}
            inputs={'files':{k:{'path':str(v)} for k,v in prior.shared_paths.items()}}
            immutable={'shared':(prior.shared['archive'],prior.shared_manifest),
                'seeds':(prior.prerequisite['seeds']['archive'],prior.validated['seed_manifest'])}
            admission_path=root/'scripted-admission.json';atomic_json(admission_path,{'synthetic':True})
            captured={};actual_recovery=CoherentRecovery;actual_pilot=QualificationPilot
            def make_bridge(legacy,out,worker,deadline,watchdog):
                plan=QualificationPlan(clock.now(),clock.now()+3600,clock.now()+2100,clock.now()+3300,preservation_only=True)
                bridge=qtests.QualificationPhysics(out,clock,plan);bridge.close=lambda:None;captured['bridge']=bridge;return bridge
            def make_pilot(**kwargs):
                kwargs['synthetic']=True;kwargs['bridge'].synthetic=True;p=actual_pilot(**kwargs);captured['pilot']=p;return p
            def restore_command(command,**kwargs):
                self.assertEqual(Path(command[1]).name,'getup-cold-start-restore.mjs')
                return SimpleNamespace(stdout=json.dumps({'passed':True,'physics_integrations':0,'cases':[{'native':0,'passed':True}]}))
            with mock.patch.object(entry,'future_admission',return_value=(admission,{},inputs,immutable,'d'*40)), \
                 mock.patch.object(entry,'TwoSlotDriveExchange',return_value=transport.exchange), \
                 mock.patch.object(entry,'SystemClock',return_value=clock), \
                 mock.patch.object(entry.time,'time',side_effect=clock.now), \
                 mock.patch.object(entry.subprocess,'run',side_effect=restore_command), \
                 mock.patch.object(entry.interfaces,'AbsoluteWatchdog',return_value=SimpleNamespace(assert_alive=lambda:None,close=lambda:None)), \
                 mock.patch.object(entry.interfaces,'BoundedWasm',side_effect=make_bridge), \
                 mock.patch('getup_cold_start_engine.DrivePhaseTransport',return_value=transport), \
                 mock.patch('getup_cold_start_engine.CoherentRecovery',side_effect=lambda t,c,synthetic:actual_recovery(t,captured['bridge'].independent_controller,synthetic=True)), \
                 mock.patch('getup_qualification.QualificationPilot',side_effect=make_pilot):
                report=entry.run(output,worker,shared,'new-preservation',admission_path=admission_path)
            self.assertNotIn('terminalRetentionError',report,report)
            self.assertIn('phaseResult',report,report)
            self.assertTrue(report['phaseResult']['syntheticQualificationRestorePassed'],report)
            self.assertEqual(transport.requests,['qualification-belly','qualification-terminal'])
            self.assertFalse({'coldPrepare','coldQualify'}&set(captured['bridge'].calls))
            self.assertEqual(sha256(prior.old.output/'budget-ledger.json'),prior.ledger_sha)
            self.assertEqual(report['externalQualificationRestore']['foreign_ledger_imported'],False)
            self.assertFalse(report['externalQualificationRestore']['learner_restored'])
            self.assertEqual(report['source']['commit'],'d'*40)

    def test_foreign_restore_never_constructs_learner_and_preserves_rng_on_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            prior=prior_fixture(Path(tmp)/'prior');target=Path(tmp)/'restore'
            manifest=restore_bundle(prior.archive,target,sha256(prior.archive));before=rng_fingerprint()
            with mock.patch.object(engine.interfaces,'SACLearner',side_effect=AssertionError('NO LEARNER')):
                result=restore_external_parity(target,manifest,prior.validated['reference'],lambda p:{'passed':True,'physics_integrations':0},synthetic=True)
                self.assertFalse(result['learner_restored']);self.assertEqual(before,rng_fingerprint())
                with self.assertRaises(ContractError):restore_external_parity(target,manifest,prior.validated['reference'],lambda p:{'passed':False,'physics_integrations':0},synthetic=True)
                self.assertEqual(before,rng_fingerprint())

    def test_nondurable_mixed_inputs_and_foreign_ledger_reject_before_new_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            prior=prior_fixture(Path(tmp)/'prior');before=rng_fingerprint()
            variants=[]
            changed=copy.deepcopy(prior.prerequisite);changed['original_identity']['workerSha256']='0'*64;variants.append(changed)
            changed=copy.deepcopy(prior.prerequisite);changed['parity']['receipt']['sha256']='0'*64;variants.append(changed)
            changed=copy.deepcopy(prior.prerequisite);changed['original_ledger']['sha256']='0'*64;variants.append(changed)
            for bad in variants:
                with self.assertRaises(ContractError):validate_external_parity(bad,prior.shared_manifest,'new-preservation','c'*64,synthetic=True)
            with self.assertRaises(ContractError):validate_external_parity(prior.prerequisite,{'files':[]},'new-preservation','c'*64,synthetic=True)
            self.assertEqual(before,rng_fingerprint());self.assertEqual(sha256(prior.old.output/'budget-ledger.json'),prior.ledger_sha)

    def test_new_source_cap_and_forbidden_preparation_before_dispatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            prior=prior_fixture(Path(tmp)/'prior');p,t=new_pilot(Path(tmp)/'new',prior)
            for name,seconds,steps in [('fixture-preparation',120,3040),('zero-correction-qualification',120,11000)]:
                with self.assertRaises(ContractError):p.enter(name,seconds,steps)
            p.bridge.ledger['bootstrapDecisions']=974
            with self.assertRaisesRegex(ContractError,'source cap'):p.call({'op':'reset'},physical=True)
            self.assertEqual(p.bridge.physical_calls,0)

    def test_physical_pending_tail_restore_uses_no_learner(self):
        with tempfile.TemporaryDirectory() as tmp:
            p,t=engine.build(tmp,pending_tail=True)
            p.cursor.planned_blocks=2;p.parity();p.bootstrap()
            p.enter('collection',120,20140);p.reset('belly','baseline-deterministic-zero')
            for _ in range(3):
                row,result=p.step([0.]*21,'bootstrap')
                if row:
                    index=p.replay.add(row);p.mapping.append(dict(p.mapper.mapping[-1],replay_index=index))
            self.assertIsNotNone(p.mapper.builder.pending)
            roles,cursor,_=p.write_snapshot('physical-pending',include_learner=False)
            manifest={'files':[{'role':role,'path':name,'sha256':sha256(p.output/name)} for role,name in roles.items()],
                'synthetic':True,'config':{'identity':p.identity,'snapshot_id':cursor['snapshot_id']}}
            before=rng_fingerprint()
            restored=restore_coherent(p.output,manifest,expected_identity=p.identity,expected_plan=p.plan,
                expected_ledger=cursor['ledger'],learner_factory=lambda: self.fail('No learner in physical proof'),
                builder_factory=p.builder_factory,controller_factory=p.bridge.independent_controller,
                synthetic=True,physical_only=True)
            self.assertIsNone(restored.learner);self.assertIsNotNone(restored.mapper.builder.pending)
            self.assertEqual(before,rng_fingerprint())

    def test_production_preflight_foreign_rejections_before_clock_or_runtime(self):
        # Mock earlier source/build checks only; exercise the actual CLI foreign
        # prerequisite branch and real validator, never turn scripted bytes into
        # an accepted actual proof or create an output/runtime.
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);prior=prior_fixture(root/'prior')
            spec=importlib.util.spec_from_file_location('preservation_preflight',Path(__file__).with_name('train-getup-cold-start.py'))
            entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
            worker=root/'worker';worker.mkdir()
            (worker/'lh_worker.js').write_bytes(b'scripted');(worker/'lh_worker_bg.wasm').write_bytes(b'scripted')
            shared=root/'inputs';shared.mkdir();source=shared/'source.json';source.write_text('{}')
            inputs={'files':{'source_identity':{'path':str(source)},'profile':{'path':str(source)}}}
            atomic_json(shared/'input-manifest.json',inputs)
            ready=root/'ready.json';atomic_json(ready,{'software_passed':True,'actual_trial_authorized':False,
                'source_files':{},'dependencies':{},'worker_glue_sha256':sha256(worker/'lh_worker.js'),
                'worker_sha256':sha256(worker/'lh_worker_bg.wasm'),'protocol_sha256':sha256(entry.PROTOCOL_PATH)})
            prerequisite=root/'external.json';atomic_json(prerequisite,prior.prerequisite)
            output=root/'never-created'
            record={'schema':'lh-cold-future-admission/v1','actual_run_authorized':True,
                'output_path':str(output.resolve()),'mode':'preservation-only','exact_blocks':0,'run_id':'new',
                'source_commit':'d'*40,'resource_decision_reference':'DISPOSABLE TEST ONLY',
                'software_readiness':doc(ready),'independent_review_reference':'synthetic',
                'corrected_build_receipt_sha256':sha256(source),'qualification_plan_sha256':sha256(entry.ROOT/'scripts/getup-qualification-plan.json'),
                'include_original_diagnostic':False,'corrected_shared_archive':prior.shared['archive']}
            bundle={'synthetic':False,'application_resume_eligible':False,'files':[
                {'role':role,'sha256':sha256(path)} for role,path in
                [('source_identity',source),('profile',source),('input_manifest',shared/'input-manifest.json')]]}
            path=root/'admission.json'
            for defect in ('missing','hash','nondurable'):
                current=copy.deepcopy(record)
                if defect!='missing':current['external_qualification']=doc(prerequisite)
                if defect=='hash':current['external_qualification']['sha256']='0'*64
                atomic_json(path,current)
                with mock.patch.object(entry,'SOFTWARE_SOURCE_FILES',[]), \
                     mock.patch.object(entry.subprocess,'check_output',side_effect=['d'*40+'\n','']), \
                     mock.patch.object(entry,'input_preflight'),mock.patch.object(entry,'validate_build_receipt'), \
                     mock.patch('experiment_backup.verify_bundle',return_value=bundle), \
                     mock.patch('getup_qualification.validate_growth_sources'), \
                     mock.patch('getup_qualification.validate_shared_reuse',side_effect=AssertionError('Must reject before later gates')), \
                     mock.patch.object(entry.interfaces,'BoundedWasm',side_effect=AssertionError('NO RUNTIME')):
                    with self.subTest(defect=defect),self.assertRaises(ContractError):
                        entry.future_admission(path,output,worker,shared,'new')
                self.assertFalse(output.exists())

    def test_external_download_corruption_retains_state_without_nearby_or_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            prior=prior_fixture(Path(tmp)/'prior');p,t=new_pilot(Path(tmp)/'new',prior)
            (t.remote/sha256(prior.archive)).write_bytes(b'corrupt')
            result=p.run_qualification()
            self.assertEqual(result['stopped'],'qualification-safe-stop');self.assertIn('terminalRetentionError',result)
            self.assertEqual(t.requests,['qualification-belly']);self.assertEqual(p.bridge.calls.count('hold'),1)
            self.assertTrue((p.output/'parity-runtime.bin').exists());self.assertTrue(prior.archive.exists())

if __name__=='__main__':unittest.main()
