#!/usr/bin/env python3
"""Separate, single-use cold-start pilot; all uploads use an operator Drive gate.

This never calls or disables the historical runner's unconditional closed gate.
Run once, then service *.request.json using the authorized private Drive worker.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

from experiment_backup import atomic_json, sha256
from getup_cold_start import (PROTOCOL, PROTOCOL_PATH, SHARED_ROLES, ColdPhase,
    DriveExchange, FreshMapper, artifact, input_preflight, require_full_phase_capacity,
    verify_qualification_amendment)
from getup_learning_loop import (ContractError, EpisodeReplay, FixedWitness,
    UpdateGuard, remaining_seconds)

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


def run(output, worker, shared, run_id, continuation_path=None):
    if output.exists() and continuation_path is None:
        raise ContractError('No cold-start restart/output reuse or deadline renewal')
    if subprocess.check_output(['git','diff','HEAD','--name-only'],cwd=ROOT,text=True).strip():
        raise ContractError('Commit reviewed tracked source before the single pilot')
    for source in ('scripts/train-getup-cold-start.py','scripts/getup_cold_start.py',
                   'scripts/getup-cold-start-protocol.json'):
        subprocess.run(['git','ls-files','--error-unmatch',source],cwd=ROOT,
                       check=True,stdout=subprocess.DEVNULL)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    amendment=None
    if continuation_path:
        amendment,report,config,exchange=verify_qualification_amendment(
            output,continuation_path,run_id,commit,time.time())
        started=report['startedAtUnix'];deadline=report['deadlineUnix']
        # This amendment admits one bounded initialization, not final evaluation.
        # The stopped report and its durable receipt remain immutable evidence.
        history=output/'amendments';history.mkdir(exist_ok=True)
        original_report=history/'original-stop-report.json'
        if original_report.exists():raise ContractError('Continuation was already initiated')
        original_report.write_bytes((output/'cold-start-report.json').read_bytes())
        (history/'qualification-amendment.json').write_bytes(Path(continuation_path).read_bytes())
        report=dict(report,continuation={'amendmentSha256':sha256(continuation_path),
            'originalStopReportSha256':sha256(original_report),'amendedSourceCommit':commit,
            'originalDeadlineUnchanged':True,'genericResume':False})
        report.pop('stopped',None);report.pop('error',None)
    else:
        output.mkdir(parents=True)
        started=time.time();deadline=started+PROTOCOL['wall_seconds']
    source={'commit':commit,'dirty':False,'description':'Reviewed cold-start source; prior lost experience not recovered'}
    if not amendment:
        report={'schema':PROTOCOL['schema'],'runId':run_id,'source':source,
            'startedAtUnix':started,'deadlineUnix':deadline,'protocol':PROTOCOL,
            'trainingExecuted':False,'physicalExperimentExecuted':False,'releaseAccepted':False,
            'candidateAccepted':False,'historicalRecoveryClaimed':False,'accepted':0,'rejected':0}
    atomic_json(output/'cold-start-report.json',report)
    print(json.dumps({'event':'START','run_id':run_id,'started':started,'deadline':deadline,
                      'protocol_sha256':sha256(PROTOCOL_PATH)}),flush=True)
    watchdog=interfaces.AbsoluteWatchdog(deadline)
    if not amendment:exchange=DriveExchange(output/'recovery',run_id,source,deadline)
    wasm=learner=replay=None
    mapper=None;state=None;episode_number=0;episode_id=None
    phase=None;mapping=[];bootstrap_indices=[];bootstrap_outcomes=[]
    if not amendment:config={}
    protected_wall=180.;pending_indices=[];backup_failed=False
    def wall():
        watchdog.assert_alive()
        remaining_seconds(deadline,time.time(),protected_wall)
    def save_report():
        report['elapsedWallSeconds']=time.time()-started
        atomic_json(output/'cold-start-report.json',report)
    def run_restore(glue, wasm_path, checkpoints):
        wall()
        result=subprocess.run(['node',str(ROOT/'scripts/getup-cold-start-restore.mjs'),
            str(glue),str(wasm_path),*[str(x) for x in checkpoints]],cwd=ROOT,text=True,
            capture_output=True,check=True,timeout=min(120.,remaining_seconds(deadline,time.time(),protected_wall)))
        return json.loads(result.stdout)
    def verify_shared(restored, manifest):
        files={item['role']:restored/item['path'] for item in manifest['files']}
        result=run_restore(files['worker_glue'],files['worker_wasm'],
                           [files['back_unstepped'],files['belly_unstepped']])
        if any(case['native']!=0 for case in result['cases']):
            raise ContractError('Shared runtime restore is not unstepped')
        return result
    def transfer(request, validator):
        nonlocal backup_failed
        receipt=request.with_name(request.name.replace('.request.json','.receipt.json'))
        print(json.dumps({'event':'DRIVE_REQUEST','request':str(request),'receipt':str(receipt),
                          'details':json.loads(request.read_text())}),flush=True)
        try:
            while not receipt.exists():
                wall();time.sleep(.5)
            result=exchange.verify_receipt(request,receipt,validator)
            wall()
            return result
        except BaseException:
            backup_failed=True
            raise
    def call(request):
        wall()
        return wasm.call(request)
    def raw_lines():
        path=output/'transitions.ndjson'
        return sum(1 for _ in path.open()) if path.exists() else 0
    def ledger():
        return call({'op':'ledger'})
    def create_mapper(case, policy):
        nonlocal state,mapper,episode_number,episode_id,pending_indices
        episode_number+=1;episode_id=f'cold-{case}-{episode_number:03d}'
        state=call({'op':'reset','case':case,'label':episode_id,'external':True,'prepared':True})
        mapper=FreshMapper(legacy.SourceTransition(interfaces.GAMMA),episode_id,policy,case,raw_lines())
        pending_indices=[]
    def one_step(action, collection_phase):
        nonlocal state
        result=call({'op':'step','c':action,'training':bool(state.get('preview')),
            'collectionPhase':collection_phase,'episodeId':episode_id,'policyId':mapper.behavior})
        row=mapper.push(state,action,result);state=result['state']
        if result['failure'] and result['failure'].get('check')!='original25s-deadline':
            raise ContractError('Actual physical integrity failure: '+json.dumps(result['failure']))
        return row,result
    def verify_live(restored, manifest):
        files={item['role']:restored/item['path'] for item in manifest['files']}
        result=run_restore(worker/'lh_worker.js',worker/'lh_worker_bg.wasm',[files['runtime_checkpoint']])
        # Exact bytes were verified first; loading this self-produced trusted
        # Torch artifact is not remote arbitrary-pickle execution.
        stored=learner.torch.load(files['learner'],weights_only=False,map_location='cpu')
        before=learner.capture_learning_state()
        learner.restore_learning_state(stored)
        from getup_cold_start_verify import assert_state_equal, verify_replay_mapping
        assert_state_equal(before,learner.capture_learning_state())
        arrays=learner.np.load(files['replay'],allow_pickle=False)
        metadata=json.loads(files['replay_metadata'].read_text())
        cursor=json.loads(files['cursor'].read_text())
        restored_mapping=json.loads(files['mapping'].read_text())
        if restored_mapping!=mapping or cursor['mapping']!=mapping:
            raise ContractError('Downloaded mapper lineage differs from paused live mapping')
        raw=[json.loads(line) for line in files['transitions'].open()]
        verified_mapping=verify_replay_mapping(arrays,metadata,restored_mapping,raw,interfaces.GAMMA)
        parity=json.loads(files['resume_parity'].read_text())
        if any(raw[i].get('training') is True and i!=parity['firstExcludedRawLine']
               for i in verified_mapping['unmappedRawLines']):
            raise ContractError('Executed fresh training transition has no mapping/exclusion')
        for key in ('features','correction','next_features'):
            expected=learner.np.asarray([getattr(row,key) for row in replay.rows],dtype='float32')
            if not learner.np.array_equal(arrays[key],expected):
                raise ContractError('Downloaded replay tensors differ from paused live replay')
        expected_metadata=[{k:v for k,v in asdict(row).items()
            if k not in ('features','correction','next_features')} for row in replay.rows]
        if metadata!=expected_metadata:
            raise ContractError('Downloaded replay episode metadata differs')
        def encoded(value):
            return json.loads(json.dumps(value,default=jsonable))
        if (cursor['replayRng']!=encoded(replay.rng.getstate())
                or cursor['pendingOrigin']!=encoded(mapper.pending_origin if mapper else None)
                or cursor['pendingTransition']!=encoded(mapper.builder.pending if mapper else None)
                or cursor['cachedState']!=encoded(state)
                or cursor['phase']!=encoded(vars(phase) if phase else None)):
            raise ContractError('Downloaded pending mapper/phase/replay RNG differs')
        # Rehydrate independent pending transition and RNG, without consuming
        # the live learner/replay streams or advancing the physical runtime.
        if mapper:
            restored_builder=legacy.SourceTransition(interfaces.GAMMA)
            restored_builder.pending=cursor['pendingTransition']
            if encoded(restored_builder.pending)!=encoded(mapper.builder.pending):
                raise ContractError('Pending SourceTransition rehydration differs')
        def tuples(value):
            return tuple(map(tuples,value)) if isinstance(value,list) else value
        import random
        restored_rng=random.Random();restored_rng.setstate(tuples(cursor['replayRng']))
        if restored_rng.getstate()!=replay.rng.getstate():
            raise ContractError('Replay RNG rehydration differs')
        if cursor['absoluteDeadlineUnix']!=deadline or cursor['ledger']!=ledger():
            raise ContractError('Restored deadline/cumulative ledger differs')
        if cursor['bridge']!=call({'op':'bridgeState'}):
            raise ContractError('Restored bridge reward/metric/snapshot state differs')
        saved_state=call({'op':'state'})
        restored_state=call({'op':'coldRestore','path':str(files['runtime_checkpoint']),
            'sha256':sha256(files['runtime_checkpoint']),'bridge':cursor['bridge']})
        if restored_state['physicsIntegrations']!=0 or restored_state['state']!=saved_state:
            raise ContractError('Operational runtime restore changed exact paused state')
        return result
    def checkpoint(boundary):
        nonlocal backup_failed
        wall()
        current=call({'op':'state'})
        call({'op':'checkpoint','name':'runtime-checkpoint.bin'})
        learner.torch.save(learner.capture_learning_state(),output/'learner.pt')
        learner.np.savez_compressed(output/'replay.npz',
            features=learner.np.asarray([r.features for r in replay.rows],dtype='float32'),
            correction=learner.np.asarray([r.correction for r in replay.rows],dtype='float32'),
            next_features=learner.np.asarray([r.next_features for r in replay.rows],dtype='float32'))
        metadata=[{k:v for k,v in asdict(row).items() if k not in ('features','correction','next_features')}
                  for row in replay.rows]
        atomic_json(output/'replay-metadata.json',metadata)
        atomic_json(output/'mapping.json',mapping)
        cursor={'schema':1,'boundary':boundary,'absoluteStartedUnix':started,'absoluteDeadlineUnix':deadline,
            'phase':vars(phase) if phase else None,'episodeId':episode_id,'episodeCounter':episode_number,
            'cachedState':state,'authoritativeState':current,'ledger':ledger(),
            'bridge':call({'op':'bridgeState'}),'replayRng':replay.rng.getstate(),
            'pendingOrigin':mapper.pending_origin if mapper else None,
            'pendingTransition':mapper.builder.pending if mapper else None,
            'mapperRawCalls':mapper.raw_calls if mapper else None,'mapping':mapping,
            'bootstrapIndices':bootstrap_indices,'restartAuthorized':False}
        (output/'cursor.json').write_text(json.dumps(cursor,default=jsonable,allow_nan=False)+'\n')
        save_report()
        roles={'learner':'learner.pt','replay':'replay.npz','replay_metadata':'replay-metadata.json',
            'mapping':'mapping.json','cursor':'cursor.json','runtime_checkpoint':'runtime-checkpoint.bin',
            'transitions':'transitions.ndjson','ledger':'budget-ledger.json',
            'config':'frozen-config.json','report':'cold-start-report.json',
            'fixtures':'cold-fixtures.json','parity':'cold-adapter-parity.json',
            'resume_parity':'cold-transition-resume-parity.json'}
        # Triggered seeds live once in the separate shared-seed generation.
        artifacts=[artifact(role,output/name,'cold-start/v1') for role,name in roles.items()]
        try:
            request=exchange.snapshot(artifacts,config,boundary,wall_reserve=protected_wall)
            transfer(request,verify_live)
        except BaseException:
            backup_failed=True
            raise
        report.setdefault('recoveryPoints',[]).append({'boundary':boundary,
            'manifestSha256':exchange.verified[-1]['manifest_sha256']})
        save_report()
    try:
        if amendment:
            # Exact one-time post-qualification reentry. No unstepped creation,
            # preparation, qualification or immutable re-upload is repeated.
            legacy=interfaces.load_legacy();learner=interfaces.SACLearner(legacy)
            claim={'schema':'lh-cold-qualification-claim/v1','runId':run_id,
                'amendmentPath':str(Path(continuation_path).resolve()),
                'amendmentSha256':sha256(continuation_path),
                'ledgerSha256':sha256(output/'budget-ledger.json'),
                'originalDeadlineUnixMs':round(deadline*1000),
                'maximumNewIntegrations':5020,'maximumPhysicalCalls':1,
                'operation':'bootstrap-belly-initialization'}
            with (output/'continuation-claim.json').open('x') as stream:
                json.dump(claim,stream)
            startup=time.time()
            wasm=interfaces.BoundedWasm(legacy,output,worker,deadline,watchdog)
            before=wasm.call({'op':'ledger'})
            startup_seconds=time.time()-startup
            call({'op':'reserve','integrations':147000,'wallSeconds':180.})
            dispatch=time.time();operation_deadline=min(deadline-180.,dispatch+120.)
            if operation_deadline<=dispatch:
                raise ContractError('No original wall remaining for limited initialization')
            wasm.call({'op':'admitUseful','maximumIntegrations':5020,
                'deadlineUnixMs':int(operation_deadline*1000)})
            wasm.deadline=operation_deadline
            try:
                state=wasm.call({'op':'reset','case':'belly','label':'cold-belly-001',
                    'external':True,'prepared':True})
            finally:wasm.deadline=deadline
            after=wasm.call({'op':'ledger'})
            result={'schema':'lh-cold-limited-initialization/v1',
                'scope':'one-required-bootstrap-initialization-not-replay-or-evaluation',
                'dispatchUnix':dispatch,'operationDeadlineUnix':operation_deadline,
                'originalDeadlineUnix':deadline,'startupWallSeconds':startup_seconds,
                'operationWallSeconds':time.time()-dispatch,'beforeLedger':before,
                'afterLedger':after,'state':state,'nextOperationAuthorized':False,
                'trainingExecuted':False,'maximumChargedIntegrations':5020,
                'nativeCallTiming':after.get('limitedCallTiming'),
                'forecastObserved':after['forecastIntegrations']>before['forecastIntegrations'],
                'fullEvaluationTimingEstablished':False}
            atomic_json(output/'limited-bootstrap-initialization.json',result)
            report['limitedInitialization']=result
            raise ContractError('Limited useful initialization completed; further phase admission required')
        else:
            # Build products and pure tests predate the pilot. Input/runtime backup,
            # all qualification, preparation and any optimizer work do not.
            shared.mkdir(parents=True,exist_ok=True)
            subprocess.run(['node',str(ROOT/'scripts/getup-cold-start-unstepped.mjs'),str(worker),
                str(ROOT/'docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json'),str(shared)],
                cwd=ROOT,check=True,capture_output=True,timeout=min(120.,remaining_seconds(deadline,time.time())))
            atomic_json(shared/'source-identity.json',dict(source,repository='https://github.com/berenif/LaboratoireHumain',
                model_bins={name:sha256(ROOT/'rust/crates/sim/data/getup-policy'/name)
                            for name in ('teacher.bin','student.bin')},
                worker_features=['getup-policy-adaptation-experiment','root-translation-carry-experiment',
                    'getup-terminal-tail-observer-experiment','getup-physical-roll-experiment',
                    'lh-sim/unit-root-rotation-experiment']))
            files=dict(worker_wasm=worker/'lh_worker_bg.wasm',worker_glue=worker/'lh_worker.js',
                profile=ROOT/'docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json',
                back_unstepped=shared/'back-unstepped.bin',belly_unstepped=shared/'belly-unstepped.bin',
                runtime_unstepped_receipt=shared/'unstepped-runtime.json',cold_start_protocol=PROTOCOL_PATH,
                source_identity=shared/'source-identity.json')
            manifest={'schema':PROTOCOL['schema'],'synthetic':False,'protocol':PROTOCOL,
                'files':{role:{'path':str(path.resolve()),'sha256':sha256(path),'bytes':path.stat().st_size}
                         for role,path in files.items()}}
            manifest_path=shared/'input-manifest.json';atomic_json(manifest_path,manifest)
            input_preflight(manifest_path)
            artifacts=[artifact(role,path,'cold-input/v1') for role,path in files.items()]
            artifacts.append(artifact('input_manifest',manifest_path,'cold-input-manifest/v1'))
            request=exchange.snapshot(artifacts,PROTOCOL,'actual-unstepped-required-inputs','shared',wall_reserve=protected_wall)
            transfer(request,verify_shared)
            proof=output/'operational-inputs.json'
            atomic_json(proof,dict(schema='lh-cold-start-operational-inputs/v1',passed=True,
                physicsIntegrations=0,privateDriveRoundtrip=True,manifestSha256=sha256(manifest_path),
                sharedManifestSha256=exchange.shared_sha256))
            config={'wallSeconds':3600,'nativeIntegrations':300000,'gamma':interfaces.GAMMA,
                'coldStart':{'schema':PROTOCOL['schema'],'manifest':str(manifest_path),'operationalProof':str(proof)},
                'inputContract':{'startedAtUnixMs':round(started*1000),'deadlineUnixMs':round(started*1000)+3600000,
                    'files':manifest['files'],'profile':str(files['profile']),
                    'checkpoints':{case:str(output/(case+'-seed.bin')) for case in ('original','heldout','belly')}}}
            atomic_json(output/'frozen-config.json',config)
            legacy=interfaces.load_legacy();learner=interfaces.SACLearner(legacy)
            wasm=interfaces.BoundedWasm(legacy,output,worker,deadline,watchdog)
            call({'op':'reserve','integrations':147000,'wallSeconds':protected_wall})
            report['physicalExperimentExecuted']=True;save_report()
            report['fixtures']=call({'op':'coldPrepare'})
            def verify_seeds(restored, manifest):
                seed_files={item['role']:restored/item['path'] for item in manifest['files']}
                return run_restore(worker/'lh_worker.js',worker/'lh_worker_bg.wasm',
                    [seed_files[case+'_seed'] for case in ('original','heldout','belly')])
            seeds=[artifact(case+'_seed',output/(case+'-seed.bin'),'fresh-triggered-seed/v1')
                   for case in ('original','heldout','belly')]
            transfer(exchange.snapshot(seeds,config,'fresh-triggered-immutable-seeds','seeds',wall_reserve=protected_wall),verify_seeds)
            report['adapterParity']=call({'op':'coldQualify'})
        # Fixed connector overhead cannot be extrapolated per integration. The
        # approved amendment is one initialization, not a full phase capacity gate.
        require_full_phase_capacity()
        wall();call({'op':'reserve','integrations':147000,'wallSeconds':protected_wall})
        bootstrap=[]
        for case in ('belly','heldout','original'):
            # Always leave a full future block and final evaluation untouched;
            # bootstrap cannot knowingly consume its own learning admission.
            if ledger()['nativeIntegrations']+20000+10000+15120+147000+5020>300000:
                raise ContractError('No room for next bootstrap episode plus a block and final reserve')
            create_mapper(case,'baseline-deterministic-zero')
            if case=='belly':
                # Real same-runtime pause/restore + next trace/reward/mapping
                # parity. Both actual calls are charged, neither silently retried.
                paused=call({'op':'checkpoint','name':'resume-probe.bin'})
                bridge=call({'op':'bridgeState'});before=state
                action=[0.]*21 if before.get('preview') else None
                query={'op':'step','c':action,'training':bool(action),'collectionPhase':'bootstrap',
                    'episodeId':episode_id,'policyId':mapper.behavior}
                first=call(query)
                call({'op':'coldRestore','path':str(output/'resume-probe.bin'),'sha256':paused['sha256'],'bridge':bridge})
                second=call(query)
                if first!=second:raise ContractError('Actual next transition/reward/terminal restore parity failed')
                mapper.first_raw_line=raw_lines()-1
                row=mapper.push(before,action,second);state=second['state']
                atomic_json(output/'cold-transition-resume-parity.json',
                    {'passed':True,'firstExcludedRawLine':raw_lines()-2,'retainedRawLine':raw_lines()-1,
                     'reason':'duplicate physical restore validation, both calls charged',
                     'sameNextStateRewardTerminal':True,'originalEpochsPreserved':True})
            while not mapper.finished:
                used=ledger().get('bootstrapDecisions',0)
                if used>=1024 and mapper.builder.pending is None:
                    break
                action=[0.]*21 if state.get('preview') else None
                if action is not None and used>=1024:
                    raise ContractError('Bootstrap decision cap reached before pending tail closure')
                one_step(action,'bootstrap')
            if case!='original':
                rows=mapper.completed_rows(preservation=True)
            elif mapper.finished:
                rows=mapper.completed_rows()
            else:
                if not mapper.rows or mapper.builder.pending is not None:
                    raise ContractError('Incomplete bootstrap source transition')
                rows=[replace(r,episode_outcome='truncation',complete_episode=False,
                    truncated=i==len(mapper.rows)-1).validate() for i,r in enumerate(mapper.rows)]
            offset=len(bootstrap);bootstrap.extend(rows)
            for entry in mapper.mapping:
                mapping.append(dict(entry,replay_index=entry['replay_index']+offset))
            bootstrap_outcomes.append({'case':case,'rows':len(rows),'outcome':rows[-1].episode_outcome})
        replay=EpisodeReplay(bootstrap,fresh_allowance=3072)
        bootstrap_indices=list(range(len(bootstrap)))
        used=ledger()['nativeIntegrations']
        per_block_estimate=256*20+2*5000
        planned_blocks=min(12,(300000-used-147000-5020)//per_block_estimate)
        if planned_blocks<1:raise ContractError('Bootstrap leaves no full iterative block plus final reserve')
        phase=ColdPhase(ledger()['bootstrapDecisions'],planned_blocks)
        report['bootstrap']=bootstrap_outcomes
        report['bootstrapDecisions']=phase.bootstrap_decisions
        report['predeclaredIterativeBlocks']=phase.planned_blocks
        checkpoint('bootstrap-before-any-optimizer-update')
        # Re-admit the complete remaining worst-case collection + evaluation,
        # with separately measured phase costs, future backup/restore and margin.
        used=ledger()['nativeIntegrations']
        projected_collection=phase.planned_blocks*per_block_estimate+5020
        if used+projected_collection+147000>300000:
            raise ContractError('Bootstrap leaves insufficient integration budget for fixed iterative pilot')
        require_full_phase_capacity()
        call({'op':'reserve','integrations':147000,'wallSeconds':protected_wall})
        while phase.phase=='warmup':
            wall();learner.critic_attempt(learner.batch(replay));learner.polyak()
            report['trainingExecuted']=True
            if not learner.finite():raise ContractError('Nonfinite cold critic warmup')
            phase.warmup_done()
        checkpoint('critic-warmup-complete')
        state=None;mapper=None
        while phase.phase=='ready_collection':
            policy=learner.policy_id();phase.begin_collection(policy);recent=[]
            while phase.phase=='collection' or (mapper and mapper.builder.pending is not None):
                wall()
                if state is None:
                    create_mapper('original',policy)
                mapper.behavior=policy
                if learner.policy_id()!=policy:raise ContractError('Collection policy changed mid-block')
                action=learner.action(state['preview']['features']) if state.get('preview') else None
                row,result=one_step(action,'iterative')
                if action is not None:phase.collected(policy)
                if row:
                    index=replay.add(row);recent.append(index);pending_indices.append(index)
                    mapping.append(dict(mapper.mapping[-1],replay_index=index))
                if result['terminal']:
                    replay.finish_episode(episode_id,'success' if result['success'] else 'failure')
                    state=None;mapper=None
                if phase.phase=='updates' and (mapper is None or mapper.builder.pending is None):break
            if len(recent)!=256:raise ContractError('Full256-transition collection required')
            checkpoint(f'collection-{phase.cycle:02d}')
            # Witness historical portion is strictly bootstrap, never this block.
            ids=list(recent)
            for category in ('back_pre_holding','baseline_preservation'):
                groups={}
                for i in bootstrap_indices:
                    r=replay.rows[i]
                    if r.category==category:groups.setdefault(r.episode,[]).append(i)
                episodes=sorted(groups)
                for _ in range(64):
                    candidates=groups[episodes[replay.rng.randrange(len(episodes))]]
                    ids.append(candidates[replay.rng.randrange(len(candidates))])
            features=[replay.rows[i].features for i in ids]
            witness=FixedWitness(features,learner.gaussians(features),policy)
            guard=UpdateGuard(learner,replay,phase,witness)
            before=call({'op':'checkpoint','name':'before-updates.bin'})
            before_state=call({'op':'state'})
            for _ in range(64):
                wall();decision,metrics,accepted=guard.attempt(lambda:learner.batch(replay))
                report['accepted' if accepted else 'rejected']+=1
            after=call({'op':'checkpoint','name':'after-updates.bin'})
            if before!=after or call({'op':'state'})!=before_state:
                raise ContractError('Learning pause advanced physical/controller state')
            phase.finish_block();checkpoint(f'updates-{phase.cycle:02d}')
        if state is not None and mapper and not mapper.finished:
            replay.finish_episode(episode_id,'truncation',truncated=True)
        if phase.phase!='finished':
            raise ContractError('No accepted actor update in block; no candidate evaluation')
        report['evaluationStarted']=True
        protected_wall=0.;call({'op':'reserve','integrations':0,'wallSeconds':0})
        head=learner.actor.exported();atomic_json(output/'candidate-head.json',head)
        for i in ids:
            features=replay.rows[i].features
            actual=call({'op':'headparity','head':head,'features':features})
            if actual['physicsIntegrations']!=0 or max(abs(a-b) for a,b in
                zip(actual['correction'],learner.action(features,deterministic=True)))>2e-6:
                raise ContractError('Final deterministic export parity failed')
        report['final']=[]
        for case in ('original','heldout','belly'):
            state=call({'op':'reset','case':case,'label':'final-'+case,'head':head,'prepared':True})
            mapper=None;episode_id='final-'+case
            while True:
                result=call({'op':'step'})
                if state.get('preview'):
                    applied=result['state']['lastCorrection']
                    if applied['native_substep']!=state['native'] or max(abs(a-b) for a,b in
                        zip(applied['correction'],learner.action(state['preview']['features'],deterministic=True)))>2e-6:
                        raise ContractError('Final applied correction parity failed')
                state=result['state']
                if result['terminal']:break
            row={'case':case,'success':result['success'],'failure':result['failure'],
                 'longestMeasuredInterval':result['longestMeasuredInterval']}
            if result['success']:
                first=call({'op':'standingReceipt'})
                row['hold']=call({'op':'hold','firstSuccessReceipt':first})
                state=call({'op':'state'})
                row['longestMeasuredInterval']=row['hold']['longestMeasuredInterval']
            report['final'].append(row)
            if not row['success'] or (case=='original' and row['longestMeasuredInterval']==0):
                break
        report['physicalQualificationPassed']=len(report['final'])==3 and all(
            r['success'] and r.get('hold',{}).get('passed') for r in report['final'])
        report['stopped']='single-cold-candidate-finished-no-promotion'
        checkpoint('terminal-stop')
    except BaseException as error:
        report['stopped']='cold-start-safe-stop';report['error']=str(error)
        if wasm:
            try:report['ledger']=wasm.call({'op':'ledger'})
            except BaseException:
                path=output/'budget-ledger.json'
                if path.exists():report['ledger']=json.loads(path.read_text())
        report['backupFailureLatched']=backup_failed
        save_report()
        # Safe-stop retention is not an experiment restart or another candidate.
        # It preserves partial evidence when bootstrap fails an honest gate,
        # without presenting incomplete strata as trainable replay.
        if not backup_failed and not exchange.failed and exchange.shared_sha256 and time.time()<deadline-30:
            try:
                protected_wall=0.
                if replay:
                    # Previously persisted replay remains coherent with its
                    # prior paused checkpoint, not this partial cursor.
                    report['latestCoherentRecoveryRetainedSeparately']=bool(exchange.verified)
                roles=collect_stop_evidence(output,report,deadline,wasm=wasm,learner=learner,
                    state=state,mapper=mapper,mapping=mapping,replay=replay)
                def verify_stop(restored, manifest):
                    result=verify_stop_documents(restored,manifest,output,roles)
                    files={item['role']:restored/item['path'] for item in manifest['files']}
                    if 'stopped_runtime' in files:
                        run_restore(worker/'lh_worker.js',worker/'lh_worker_bg.wasm',[files['stopped_runtime']])
                        from getup_cold_start_verify import assert_state_equal
                        stored=learner.torch.load(files['stopped_learner'],weights_only=False,map_location='cpu')
                        assert_state_equal(stored,learner.capture_learning_state())
                    return result
                request=exchange.snapshot([artifact(role,output/name,'cold-stop-not-resumable/v1')
                    for role,name in roles.items()],dict(config,resumeAuthorized=False,
                        incompleteBootstrap=replay is None),'partial-safe-stop-evidence')
                transfer(request,verify_stop)
                report['partialStopEvidenceDurable']=True
            except BaseException as retention_error:
                report['partialStopRetentionError']=str(retention_error)
            save_report()
        print(json.dumps({'event':'STOP','report':report}),flush=True)
        raise
    finally:
        save_report()
        if wasm:wasm.close()
        watchdog.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--worker',type=Path,required=True)
    parser.add_argument('--shared',type=Path,required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--continue-after-qualification',type=Path)
    args=parser.parse_args()
    run(args.output.resolve(),args.worker.resolve(),args.shared.resolve(),args.run_id,
        args.continue_after_qualification.resolve() if args.continue_after_qualification else None)


if __name__=='__main__':
    main()
