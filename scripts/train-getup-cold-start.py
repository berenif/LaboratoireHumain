#!/usr/bin/env python3
"""Separate, single-use cold-start pilot; all uploads use an operator Drive gate.

This never calls or disables the historical runner's unconditional closed gate.
Run once, then service *.request.json using the authorized private Drive worker.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

from experiment_backup import atomic_json, sha256
from getup_cold_start import PROTOCOL,PROTOCOL_PATH,DriveExchange,input_preflight
from getup_learning_loop import ContractError,remaining_seconds
from getup_worker_build import validate_build_receipt
from getup_drive_slots import TwoSlotDriveExchange

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('historical_interfaces',
    Path(__file__).with_name('train-getup-learning-loop.py'))
interfaces = importlib.util.module_from_spec(spec)
spec.loader.exec_module(interfaces)


def jsonable(value):
    if hasattr(value, 'tolist'): return value.tolist()
    raise TypeError(type(value).__name__)


def collect_stop_evidence(output, report, deadline, *, wasm=None, learner=None,
                          state=None, mapper=None, mapping=(), replay=None):
    """Retain diagnostics even when qualification leaves no active runtime.

    This does not create/reset a runtime or advance physics. Optional live-state
    capture failure must not discard the report and authoritative disk ledger.
    Every result remains explicitly ineligible for application resume.
    """
    cursor={'restartAuthorized':False,'applicationResumeEligible':False,
        'bootstrapReplayConstructed':replay is not None,
        'coherentBootstrapBackupVerified':any(
            x.get('boundary')=='bootstrap-before-any-optimizer-update'
            for x in report.get('recoveryPoints',[])),
        'absoluteDeadlineUnix':deadline,'ledger':report.get('ledger'),
        'runtimeCaptured':False,'runtimeOmission':'no-active-runtime',
        'pendingTransition':mapper.builder.pending if mapper else None,
        'pendingOrigin':mapper.pending_origin if mapper else None,
        'mapping':list(mapping)}
    roles={'stopped_cursor':'stopped-cursor.json','report':'cold-start-report.json'}
    if state is not None and wasm is not None and learner is not None:
        try:
            live=wasm.call({'op':'state'})
            bridge=wasm.call({'op':'bridgeState'})
            wasm.call({'op':'checkpoint','name':'stopped-runtime.bin'})
            learner.torch.save(learner.capture_learning_state(),output/'stopped-learner.pt')
            cursor.update(state=live,bridge=bridge,runtimeCaptured=True,runtimeOmission=None)
            roles.update(stopped_runtime='stopped-runtime.bin',stopped_learner='stopped-learner.pt')
        except Exception as error:
            cursor['runtimeOmission']='live-state-capture-failed'
            cursor['runtimeCaptureError']=str(error)
    (output/'stopped-cursor.json').write_text(json.dumps(cursor,default=jsonable,allow_nan=False)+'\n')
    for role,name in [('ledger','budget-ledger.json'),('config','frozen-config.json'),
                      ('transitions','transitions.ndjson'),('fixtures','cold-fixtures.json'),
                      ('parity','cold-adapter-parity.json'),
                      ('limited_initialization','limited-bootstrap-initialization.json'),
                      ('amendment','amendments/qualification-amendment.json'),
                      ('continuation_claim','continuation-claim.json'),
                      ('bridge_claim','continuation-bridge-consumed.json'),
                      ('original_stop_report','amendments/original-stop-report.json')]:
        if (output/name).is_file():roles[role]=name
    atomic_json(output/'cold-start-report.json',report)
    return roles


def verify_stop_documents(restored, manifest, output, roles):
    """Operationally decode terminal diagnostics; never assert runtime readiness."""
    if manifest.get('application_resume_eligible') is not False or manifest.get('full_resume_ready') is not False:
        raise ContractError('Partial stop evidence must be explicitly nonresumable')
    files={item['role']:restored/item['path'] for item in manifest['files']}
    if set(files)!=set(roles):
        raise ContractError('Terminal evidence roles differ')
    for role,name in roles.items():
        if name.endswith('.json'):
            if json.loads(files[role].read_text())!=json.loads((output/name).read_text()):
                raise ContractError('Restored terminal diagnostic differs: '+role)
    cursor=json.loads(files['stopped_cursor'].read_text())
    if cursor.get('restartAuthorized') is not False or cursor.get('applicationResumeEligible') is not False:
        raise ContractError('Terminal cursor cannot authorize resume')
    if 'ledger' in files and cursor['ledger']!=json.loads(files['ledger'].read_text()):
        raise ContractError('Terminal cursor/ledger disagreement')
    return {'passed':True,'physics_integrations':0,'application_resume_eligible':False,
            'scope':'terminal-documents-not-runtime-resume'}


class SystemClock:
    def now(self):return time.time()
    def work(self,kind):pass


SOFTWARE_SOURCE_FILES=(
    'scripts/getup_cold_start_engine.py','scripts/getup_cold_start_rehearsal.py',
    'scripts/getup-cold-start-engine-test.py','scripts/train-getup-cold-start.py',
    'scripts/getup_cold_start.py','scripts/getup_cold_start_verify.py',
    'scripts/getup_learning_loop.py','scripts/train-getup-learning-loop.py',
    'scripts/train-getup-wasm-adaptation.py','scripts/getup-wasm-adaptation-protocol.mjs',
    'scripts/getup-cold-start-restore.mjs','scripts/getup-cold-start-protocol-contract.mjs',
    'scripts/getup-learning-loop-protocol-contract.mjs','scripts/experiment_backup.py',
    'scripts/getup-cold-start-protocol.json','scripts/getup_drive_slots.py',
    'scripts/getup_worker_build.py','scripts/getup-cold-worker-contract.json',
    'scripts/getup-cold-worker-contract.mjs','scripts/getup-cold-worker-capabilities.mjs',
    'scripts/build-getup-cold-worker.py','scripts/getup-cold-start-unstepped.mjs',
    'scripts/getup_qualification.py','scripts/getup-qualification-plan.json',
    'scripts/getup-qualification-budget.mjs','scripts/getup-qualification-budget.test.mjs',
    'scripts/getup-qualification-test.py','scripts/getup-corrective-readiness-test.py')


def future_admission(path,output,worker,shared,run_id):
    """Pure pre-clock checks. Software success never grants trial authority."""
    from experiment_backup import verify_bundle
    from getup_cold_start import validate_backup_identity
    if output.exists():raise ContractError('Closed/output attempt cannot be reused or restarted')
    if path is None:raise ContractError('Future root resource decision/admission required; software tests authorize no trial')
    admission=json.loads(Path(path).read_text())
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','diff','HEAD','--name-only'],cwd=ROOT,text=True).strip():
        raise ContractError('Reviewed committed source is required')
    if (admission.get('schema')!='lh-cold-future-admission/v1'
            or admission.get('actual_run_authorized') is not True
            or admission.get('output_path')!=str(output.resolve())
            or admission.get('mode')!='qualification-only' or admission.get('exact_blocks')!=0
            or admission.get('run_id')!=run_id or admission.get('source_commit')!=commit
            or not admission.get('resource_decision_reference')):
        raise ContractError('Explicit future run/source/resource decision is absent')
    ready=admission['software_readiness'];ready_path=Path(ready['path'])
    if sha256(ready_path)!=ready['sha256']:raise ContractError('Software readiness receipt changed')
    readiness=json.loads(ready_path.read_text())
    if (readiness.get('software_passed') is not True or readiness.get('actual_trial_authorized') is not False
            or not admission.get('independent_review_reference')):
        raise ContractError('Software/independent-review gate absent; never a physical acceptance')
    if set(readiness['source_files'])!=set(SOFTWARE_SOURCE_FILES):
        raise ContractError('Incomplete software-tested source inventory')
    import importlib.metadata
    if any(importlib.metadata.version(name)!=version for name,version in readiness['dependencies'].items()):
        raise ContractError('Software-tested dependencies changed')
    for name,digest in readiness['source_files'].items():
        if sha256(ROOT/name)!=digest:raise ContractError('Software-tested source differs: '+name)
    if (sha256(worker/'lh_worker.js')!=readiness['worker_glue_sha256']
            or sha256(worker/'lh_worker_bg.wasm')!=readiness['worker_sha256']
            or sha256(PROTOCOL_PATH)!=readiness['protocol_sha256']):
        raise ContractError('Worker/protocol differs from software readiness identity')
    manifest_path=shared/'input-manifest.json';input_preflight(manifest_path)
    manifest=json.loads(manifest_path.read_text())
    build=validate_build_receipt(manifest['files']['source_identity']['path'],worker,
        manifest['files']['profile']['path'])
    if admission.get('corrected_build_receipt_sha256')!=sha256(manifest['files']['source_identity']['path']):
        raise ContractError('Corrected semantic build is not admitted')
    from getup_qualification import (PLAN_PATH,PLAN,validate_growth_sources,
        validate_shared_reuse,qualification_staging_scope,validate_owner_inventory)
    validate_growth_sources(ROOT)
    if admission.get('qualification_plan_sha256')!=sha256(PLAN_PATH):
        raise ContractError('Exact reviewed qualification-only operation/byte plan required')
    if type(admission.get('include_original_diagnostic')) is not bool:
        raise ContractError('Optional original recovery-only workload must be frozen explicitly')
    item=admission['corrected_shared_archive'];archive=Path(item['path'])
    if sha256(archive)!=item['sha256'] or archive.stat().st_size!=item['bytes']:
        raise ContractError('Corrected shared archive differs')
    bundle=verify_bundle(archive,item['sha256'],32*1024**2)
    if bundle['synthetic'] or bundle['application_resume_eligible']:
        raise ContractError('Corrected shared archive must be actual unstepped inputs only')
    entries={x['role']:x for x in bundle['files']}
    paths={role:Path(info['path']) for role,info in manifest['files'].items()}
    paths['input_manifest']=manifest_path
    if set(entries)!=set(paths) or any(sha256(p)!=entries[role]['sha256'] for role,p in paths.items()):
        raise ContractError('Corrected archive does not contain exact bound inputs')
    if item['bytes']+PLAN['seed_archive_bytes']>12*1024**2:
        raise ContractError('Corrected shared+futureseed archive allowance exceeded')
    receipts={'shared':(item,bundle)}
    reused_shared=validate_shared_reuse(admission['verified_shared'],item,bundle,time.time())
    admission['_verified_shared']=reused_shared
    storage=admission['storage']
    staging_limit=qualification_staging_scope(admission)
    if (storage['cumulative_uploaded_bytes']<37690936 or storage['cumulative_uploaded_bytes']>160*1024**2
            or not storage.get('owner_inventory_reference')):
        raise ContractError('Complete prior cumulative/owner storage admission required')
    measured=0
    for directory in storage['protected_local_backup_roots']:
        path=Path(directory).resolve()
        if not path.is_relative_to(ROOT/'outputs'/'cold-start'):
            raise ContractError('Protected evidence root must remain in the owned cold-start area')
        measured+=sum(p.stat().st_size for p in path.rglob('*') if p.is_file())
    if measured!=storage['protected_local_backup_bytes'] or measured>160*1024**2:
        raise ContractError('Protected local evidence byte inventory differs')
    prior=[]
    for item in storage['prior_generations']:
        documents={key:json.loads(Path(item[key]['path']).read_text()) for key in ('request','receipt','application')}
        for key in documents:
            if sha256(Path(item[key]['path']))!=item[key]['sha256']:raise ContractError('Prior generation proof changed')
        request,receipt,application=(documents[key] for key in ('request','receipt','application'))
        validate_backup_identity(request,receipt,application,request['run_id'])
        if request['kind']!='generation':raise ContractError('Prior generation role differs')
        verify_bundle(request['archive_path'],request['archive_sha256'],64*1024**2)
        prior.append(dict(manifest_sha256=request['manifest_sha256'],archive_sha256=request['archive_sha256'],
            application_verified=True,owned=True,drive_file_id=receipt['drive_file_id'],
            payload_bytes=request['payload_bytes'],archive_path=request['archive_path'],archive_bytes=request['archive_bytes'],
            generation_nonce=request.get('generation_nonce')))
    if len(prior)!=2 or len({x['drive_file_id'] for x in prior})!=len(prior):
        raise ContractError('Exact current/previous remote generation inventory required')
    if storage['remote_payload_bytes']<sum(x['payload_bytes'] for x in prior):
        raise ContractError('Remote payload inventory omits retained generations')
    validate_owner_inventory(admission,prior)
    from getup_drive_slots import qualification_storage_envelope
    shared_local=sum(p.stat().st_size for p in shared.rglob('*') if p.is_file())
    archive_metadata=sum(p.stat().st_size for p in archive.parent.rglob('*') if p.is_file())-archive.stat().st_size
    admission['_qualification_storage']=qualification_storage_envelope(
        protected_local_bytes=measured+archive_metadata,
        corrected_shared_payload_bytes=shared_local,corrected_shared_archive_bytes=admission['corrected_shared_archive']['bytes'],
        seed_payload_bytes=2*1024**2,seed_archive_bytes=PLAN['seed_archive_bytes'],
        prior_cumulative_before_corrected_inputs=storage['cumulative_uploaded_bytes'],
        slot_archive_bytes=[x['archive_bytes'] for x in prior],active_working_bytes=0,
        staging_limit_bytes=staging_limit,shared_already_verified=True)
    admission['_verified_prior_generations']=prior
    return admission,readiness,manifest,receipts,commit


def admitted_start(admission,now):
    # Frozen before launch and announced to the owner. Validation/notification
    # time consumes this same hour; the runner never renews the timestamp.
    import math
    started=admission.get('started_at_unix')
    if not isinstance(started,(int,float)) or not math.isfinite(started) or not 0<=now-started<120:
        raise ContractError('Fixed admission start is future/stale/missing; no clock renewal')
    return started


def run(output,worker,shared,run_id,continuation_path=None,admission_path=None):
    """One future authorized process; no generic resume of the closed attempt."""
    from dataclasses import asdict
    import shutil
    import tempfile
    from experiment_backup import restore_bundle
    from getup_cold_start_engine import CoherentRecovery,DrivePhaseTransport
    from getup_qualification import (QualificationPilot,QualificationPlan,QualificationByteGuard,
        untouched_learner_bytes,restore_verified_shared,PLAN)
    if continuation_path is not None:
        raise ContractError('Qualification amendment is consumed/closed; no continuation or new clock')
    admission,readiness,inputs,immutable,commit=future_admission(admission_path,output,worker,shared,run_id)
    # All new runtime loading, operational restoration and qualification are
    # INSIDE this one absolute clock, not relabeled software preparation.
    started=admitted_start(admission,time.time());deadline=started+3600;clock=SystemClock()
    plan=QualificationPlan(started,deadline,started+2100,started+3300)
    output.mkdir(parents=True)
    source={'commit':commit,'dirty':False,'description':'Reviewed source; no exact historical recovery claim'}
    exchange=TwoSlotDriveExchange(output/'recovery',run_id,source,deadline,
        slots=admission['_verified_prior_generations'])
    exchange.remote_uploaded_bytes=admission['storage']['cumulative_uploaded_bytes']
    exchange.local_staging_limit_bytes=admission['storage']['staging_bytes']
    exchange.verified=admission['_verified_prior_generations']
    exchange.retired_payload_bytes=admission['storage']['remote_payload_bytes']-sum(x['payload_bytes'] for x in exchange.verified)
    # Closed local evidence stays untouched; remote current/previous retention
    # only overwrites the older owned slot; current stays intact until restore.
    # All prior/Trash bytes remain in cumulative storage; no quota refund.
    exchange.external_local_bytes=admission['storage']['protected_local_backup_bytes']+sum(p.stat().st_size for p in shared.rglob('*') if p.is_file())+admission['corrected_shared_archive']['bytes']
    watchdog=interfaces.AbsoluteWatchdog(deadline);wasm=None;pilot=None
    report={'schema':'lh-cold-runtime-admission/v1','runId':run_id,'startedAtUnix':started,
        'deadlineUnix':deadline,'source':source,'admissionSha256':sha256(admission_path),
        'softwareReadinessSha256':admission['software_readiness']['sha256'],
        'actualTrialAuthorized':True,'releaseAccepted':False,'candidateAccepted':False,
        'historicalRecoveryClaimed':False,'reusedImmutableArchives':True,
        'mode':'qualification-only','optimizerAuthorized':False,'candidateEvaluationAuthorized':False,
        'storageEnvelope':admission['_qualification_storage']}
    atomic_json(output/'cold-start-report.json',report)
    print(json.dumps({'event':'START','run_id':run_id,'started':started,'deadline':deadline,
        'protocol_sha256':sha256(PROTOCOL_PATH),'absolutePhaseCutoffs':asdict(plan)}),flush=True)
    def run_restore(glue,wasm_path,checkpoints,state_file=None,operation_deadline=None):
        end=min(operation_deadline or plan.stage_cutoff('immutable-input-restore'),deadline)
        remaining_seconds(end,time.time())
        command=['node',str(ROOT/'scripts/getup-cold-start-restore.mjs'),str(glue),str(wasm_path),
                 *[str(x) for x in checkpoints]]
        if state_file:command+=['--controller',str(state_file)]
        result=subprocess.run(command,cwd=ROOT,text=True,capture_output=True,check=True,
            timeout=min(120.,remaining_seconds(end,time.time())))
        remaining_seconds(end,time.time());return json.loads(result.stdout)
    def transfer(request,validator):
        details=json.loads(request.read_text());end=details['deadline_unix']
        receipt=request.with_name(request.name.replace('.request.json','.receipt.json'))
        print(json.dumps({'event':'DRIVE_REQUEST','request':str(request),'receipt':str(receipt),'details':details}),flush=True)
        while not receipt.exists():
            watchdog.assert_alive();remaining_seconds(end,time.time());time.sleep(.5)
        result=exchange.verify_receipt(request,receipt,validator)
        remaining_seconds(end,time.time());return result
    def controller_factory(path,bridge,state,ledger):
        # Independent process, fresh native runtime. Runtime-free snapshots
        # intentionally do not launch a worker or call state/checkpoint.
        if path is None:return {'runtime_active':False,'ledger':ledger}
        with tempfile.TemporaryDirectory(prefix='controller-restore-',dir=output) as tmp:
            expected=Path(tmp)/'controller.json'
            atomic_json(expected,{'state':state,'bridge':bridge})
            result=run_restore(worker/'lh_worker.js',worker/'lh_worker_bg.wasm',[path],expected,
                pilot.phase_deadline if pilot else plan.work_end)
        return {'runtime_active':True,'ledger':ledger,'receipt':result,'bridge':bridge,'state':state}
    try:
        plan.assert_inputs_ready(clock.now())
        def restore_shared(restored,manifest):
            files={x['role']:restored/x['path'] for x in manifest['files']}
            result=run_restore(files['worker_glue'],files['worker_wasm'],
                [files['back_unstepped'],files['belly_unstepped']])
            if any(x['native']!=0 for x in result['cases']):raise ContractError('Corrected inputs are not unstepped')
            report['sharedOperationalRestore']=result
            return result
        # Prior private application proof + fresh exact private readback were
        # preflighted. Restore the SAME locally hash-verified archive inside the
        # new clock; no upload/request or cumulative upload quota charge.
        shared_item,_=immutable['shared']
        prior_shared=admission['_verified_shared']['request']
        restore_verified_shared(exchange,shared_item,prior_shared,restore_shared,started+300)
        plan.assert_inputs_ready(clock.now())
        proof=output/'operational-inputs.json'
        atomic_json(proof,dict(schema='lh-cold-start-operational-inputs/v1',passed=True,
            physicsIntegrations=0,privateDriveRoundtrip=True,previousPrivateRoundtripReused=True,
            newRemoteTransfer=False,manifestSha256=sha256(shared/'input-manifest.json'),
            sharedManifestSha256=exchange.shared_sha256,
            freshReadOnlyReadbackVerifiedBeforeClock=True,
            previousPrivateProof=admission['verified_shared']))
        legacy=interfaces.load_legacy();factory=lambda:interfaces.SACLearner(legacy)
        learner=factory();learner_bytes=untouched_learner_bytes(learner)
        seed_paths={case:str(output/(case+'-seed.bin')) for case in ('original','heldout','belly')}
        config={'wallSeconds':3600,'nativeIntegrations':300000,'gamma':interfaces.GAMMA,
            'qualificationOnly':True,'qualificationPlan':PLAN,
            'qualificationStorage':{'untouchedLearnerBytes':learner_bytes},
            'coldStart':{'schema':PROTOCOL['schema'],'manifest':str(shared/'input-manifest.json'),'operationalProof':str(proof)},
            'inputContract':{'startedAtUnixMs':round(started*1000),'deadlineUnixMs':round(started*1000)+3600000,
                'files':inputs['files'],'profile':inputs['files']['profile']['path'],
                'checkpoints':seed_paths}}
        atomic_json(output/'frozen-config.json',config)
        plan.assert_inputs_ready(clock.now())
        wasm=interfaces.BoundedWasm(legacy,output,worker,deadline,watchdog);wasm.synthetic=False
        recovery=CoherentRecovery(DrivePhaseTransport(exchange,transfer,clock),controller_factory,synthetic=False)
        identity={'runId':run_id,'sourceCommit':commit,'workerSha256':sha256(worker/'lh_worker_bg.wasm'),
            'protocolSha256':sha256(PROTOCOL_PATH),'sharedManifestSha256':exchange.shared_sha256,
            'seedManifestSha256':exchange.seed_sha256,'admissionSha256':sha256(admission_path)}
        pilot=QualificationPilot(output=output,learner=learner,learner_factory=factory,
            byte_guard=QualificationByteGuard(learner_bytes),
            builder_factory=lambda:legacy.SourceTransition(interfaces.GAMMA),bridge=wasm,
            recovery=recovery,clock=clock,plan=plan,identity=identity,synthetic=False)
        plan.assert_inputs_ready(clock.now())
        report['inputsCompletedAtUnix']=clock.now()
        pilot.enter('fixture-preparation',120,3040)
        report['freshFixtures']=pilot.call({'op':'coldPrepare'},physical=True,upper_bound=8060)
        if sum(Path(path).stat().st_size for path in seed_paths.values())>2*1024**2:
            raise ContractError('Measured fresh seed payload exceeds frozen2MiB allowance')
        pilot.enter('seed-backup',180,0)
        from getup_cold_start import artifact
        seed_request=exchange.snapshot([artifact(case+'_seed',path,'getup-native-checkpoint') for case,path in seed_paths.items()],
            {'corrected_worker_sha256':identity['workerSha256'],
             'qualification_immutable':True,
             'metered_preparation_ledger_sha256':sha256(output/'budget-ledger.json'),'resumeAuthorized':False},
            'metered-corrected-triggered-seeds','seeds',deadline-pilot.phase_deadline)
        details=json.loads(seed_request.read_text())
        if details['archive_bytes']>PLAN['seed_archive_bytes'] or details['archive_bytes']+admission['corrected_shared_archive']['bytes']>12*1024**2:
            raise ContractError('Measured corrected immutable archive allowance exceeded before upload')
        def restore_seeds(restored,manifest):
            files={x['role']:restored/x['path'] for x in manifest['files']}
            return run_restore(worker/'lh_worker.js',worker/'lh_worker_bg.wasm',
                [files[case+'_seed'] for case in ('original','heldout','belly')],operation_deadline=pilot.phase_deadline)
        transfer(seed_request,restore_seeds)
        pilot.identity['seedManifestSha256']=exchange.seed_sha256
        report['phaseResult']=pilot.run_qualification(include_original=admission['include_original_diagnostic'])
    except BaseException as error:
        report.update(stopped='runtime-admission-safe-stop',error=str(error))
        if pilot is not None:
            pilot.failure_latched=True
            pilot.report.update(stopped='qualification-safe-stop',error=str(error))
            try:report['phaseResult']=pilot.finalize()
            except BaseException as stop_error:report['terminalRetentionError']=str(stop_error)
    finally:
        report['elapsedWallSeconds']=time.time()-started
        report['cumulativeRemoteExposureBytes']=exchange.remote_uploaded_bytes
        report['pendingImmutableExposure']=getattr(exchange,'pending_immutable',{})
        atomic_json(output/'cold-start-report.json',report)
        if wasm:wasm.close()
        watchdog.close()
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--worker',type=Path,required=True)
    parser.add_argument('--shared',type=Path,required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--admission',type=Path,required=True,
        help='Future root resource decision + reviewed software/immutable/storage identities; never generated by tests')
    args=parser.parse_args()
    run(args.output.resolve(),args.worker.resolve(),args.shared.resolve(),args.run_id,
        admission_path=args.admission.resolve())


if __name__=='__main__':main()
