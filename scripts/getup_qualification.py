"""Narrow corrected-worker qualification; no learning or candidate methods.

Uses the existing mapper, diagnostic replay, learner serialization, independent
restore and Drive adapter. Only clock/bridge/transport are scripted in tests.
"""
from dataclasses import dataclass,asdict,replace
import copy,json
from pathlib import Path
from getup_cold_start_engine import ColdPilot,PhasePlan
from getup_learning_loop import ContractError,EpisodeReplay,CATEGORIES,diagnostic_membership
from experiment_backup import atomic_json

PLAN_PATH=Path(__file__).with_name('getup-qualification-plan.json')
PLAN=json.loads(PLAN_PATH.read_text())

def validate_growth_sources(root):
    from experiment_backup import sha256
    for name,digest in PLAN['growth_bound_source_sha256'].items():
        if sha256(Path(root)/name)!=digest:
            raise ContractError('Frozen runtime growth source changed: '+name)
    if PLAN['schema']!='lh-corrected-qualification-phases/v2' or PLAN['next_call_growth_bytes'] is not None:
        raise ContractError('Reviewed replacement-aware serialized bound changed')
    model=json.loads((Path(root)/'rust/crates/model/data/getup-v1.json').read_text())
    primitives=[p for segment in model['segments'] for p in segment['reference']['primitives']]
    hinges=[s['reference']['hinge'] for s in model['segments'] if s['reference']['hinge']]
    if (len(model['segments'])!=25 or len(primitives)!=19 or len(hinges)+6!=27
            or any(p['kind'] not in ('sphere','capsule') for p in primitives)
            or PLAN['frozen_geometry']['maximum_primitive_pairs']!=(19+7)*(19+6)//2):
        raise ContractError('Frozen geometry/cardinality growth bound changed')

def request_existing_shared(exchange,item,deadline):
    """Reuse the one measured local archive, not another immutable ZIP copy."""
    import hashlib,zipfile
    from experiment_backup import verify_bundle
    archive=Path(item['path']);manifest=verify_bundle(archive,item['sha256'],32*1024**2)
    exchange.operation_deadline=deadline
    request={'schema':'lh-private-drive-request/v1','run_id':exchange.run_id,
        'synthetic':False,'kind':'shared','boundary':'corrected-unstepped-inputs',
        'archive_path':str(archive),'archive_sha256':item['sha256'],'archive_bytes':item['bytes'],
        'payload_bytes':sum(x['bytes'] for x in manifest['files']),
        'manifest_sha256':hashlib.sha256(zipfile.ZipFile(archive).read('manifest.json')).hexdigest(),
        'required_roles':manifest['required_roles'],'deadline_unix':deadline,
        'absolute_deadline_unix':exchange.deadline,
        'remote_cumulative_archive_bytes_before':exchange.remote_uploaded_bytes,
        'verified_generations':exchange.verified,'max_retained_payload_bytes':160*1024**2,
        'privacy':'owner-only; no shared link; exact corrected archive only',
        'retention':'immutable once; no old evidence deletion',
        'download_cleanup':'exact duplicate only after application-verified marker'}
    exchange.precharge_immutable(request)
    path=exchange.directory/'corrected-shared.request.json';atomic_json(path,request);return path


def validate_shared_reuse(proof,item,manifest,now):
    """No upload: prior application proof plus fresh exact private readback."""
    from experiment_backup import sha256
    from getup_cold_start import validate_backup_identity
    documents={}
    for role in ('request','receipt','application','private_identity'):
        bound=proof[role];path=Path(bound['path'])
        if sha256(path)!=bound['sha256']:raise ContractError('Reused shared proof changed: '+role)
        documents[role]=json.loads(path.read_text())
    request,receipt,application,private=(documents[x] for x in
        ('request','receipt','application','private_identity'))
    validate_backup_identity(request,receipt,application,request['run_id'])
    if (request['kind']!='shared' or any(request[k]!=item[k.replace('archive_','')] for k in
            ('archive_sha256','archive_bytes'))
            or request['required_roles']!=manifest['required_roles']
            or request['payload_bytes']!=sum(x['bytes'] for x in manifest['files'])):
        raise ContractError('Reused shared archive/roles differ')
    import hashlib,zipfile
    with zipfile.ZipFile(item['path']) as bundle:
        if hashlib.sha256(bundle.read('manifest.json')).hexdigest()!=request['manifest_sha256']:
            raise ContractError('Reused shared manifest differs')
    if (private.get('schema')!='lh-reused-shared-private-identity/v1'
            or private.get('drive_file_id')!=receipt['drive_file_id']
            or private.get('archive_sha256')!=item['sha256'] or private.get('archive_bytes')!=item['bytes']
            or private.get('owner_only') is not True or private.get('shared') is not False
            or private.get('new_upload') is not False or private.get('new_download') is not True
            or private.get('download_completed') is not True
            or private.get('downloaded_archive_sha256')!=item['sha256']
            or private.get('downloaded_archive_bytes')!=item['bytes']
            or private.get('previous_application_sha256')!=proof['application']['sha256']
            or not 0<=now-private.get('verified_at_unix',0)<=1800
            or not 0<=now-private.get('readback_verified_at_unix',0)<=1800):
        raise ContractError('Fresh compatible private shared identity absent')
    metadata=private.get('current_metadata',{})
    if metadata.get('id')!=receipt['drive_file_id'] or int(metadata.get('size',-1))!=item['bytes']:
        raise ContractError('Current private shared metadata differs')
    return documents


def restore_verified_shared(exchange,item,request,validator,deadline):
    """Production local rehydrate of already-backed-up bytes, never a transfer."""
    import tempfile
    from experiment_backup import restore_bundle
    if exchange.now()>=deadline:raise ContractError('Reused input restore deadline')
    if exchange.retired_payload_bytes<request['payload_bytes']:
        raise ContractError('Retained shared payload inventory incomplete')
    with tempfile.TemporaryDirectory(prefix='shared-reuse-',dir=exchange.directory) as tmp:
        restored=Path(tmp)/'payload'
        manifest=restore_bundle(item['path'],restored,item['sha256'],32*1024**2)
        result=validator(restored,manifest)
        if result.get('passed') is not True or result.get('physics_integrations')!=0:
            raise ContractError('Reused inputs require actual unstepped application restore')
    if exchange.now()>=deadline:raise ContractError('Reused input restore deadline')
    exchange.shared_sha256=request['manifest_sha256']
    exchange.shared_bytes=request['payload_bytes']
    exchange.shared_archive=Path(item['path'])
    exchange.retired_payload_bytes-=exchange.shared_bytes
    return result


def qualification_staging_scope(admission):
    storage=admission['storage'];limit=storage['staging_bytes']
    if limit==160*1024**2:return limit
    if (limit!=192*1024**2 or storage.get('staging_scope_run_id')!=admission['run_id']
            or not storage.get('staging_amendment_reference')
            or storage.get('active_output_bytes')!=32*1024**2
            or storage.get('archive_bytes')!=18*1024**2
            or storage.get('cumulative_archive_bytes')!=160*1024**2
            or storage.get('mutable_uploads')!=1):
        raise ContractError('192MiB is a scoped local amendment, not remote/output/cadence permission')
    return limit


def validate_owner_inventory(admission,prior):
    from experiment_backup import sha256
    storage=admission['storage'];bound=storage['owner_inventory'];path=Path(bound['path'])
    if sha256(path)!=bound['sha256']:raise ContractError('Owner inventory changed')
    inventory=json.loads(path.read_text());ledger_path=Path(inventory['ledger_path'])
    if sha256(ledger_path)!=inventory['ledger_sha256']:raise ContractError('Closed transfer ledger changed')
    ledger=json.loads(ledger_path.read_text())
    if (inventory.get('schema')!='lh-drive-owner-qualification-inventory/v1'
            or inventory.get('planned_run_id')!=admission['run_id']
            or inventory.get('unknown_transfers') is not False
            or inventory.get('cumulative_uploaded_bytes')!=ledger.get('cumulative_uploaded_bytes')
            or storage['cumulative_uploaded_bytes']!=ledger.get('cumulative_uploaded_bytes')
            or storage['cumulative_uploaded_bytes']<43566444
            or inventory.get('live_archive_bytes')!=ledger.get('live_bytes')
            or storage.get('live_archive_bytes')!=ledger.get('live_bytes')
            or inventory.get('trashed_archive_bytes')!=ledger.get('trashed_bytes')
            or storage.get('trashed_archive_bytes')!=ledger.get('trashed_bytes')):
        raise ContractError('Owner counters differ or refund closed exposure')
    pending=ledger.get('pending_replacement')
    if pending and pending.get('status')!='application-verified-pointer-committed':
        raise ContractError('Unresolved prior replacement')
    keys=('drive_file_id','archive_sha256','manifest_sha256','archive_bytes','payload_bytes')
    expected=[ledger['older_slot'],ledger['current_slot']]
    if len(prior)!=2 or len(inventory['prior_generations'])!=2:
        raise ContractError('Owner requires two ordered slots')
    for proposed,recorded,owned in zip(prior,expected,inventory['prior_generations']):
        if any(proposed[k]!=recorded[k] or proposed[k]!=owned[k] for k in keys):
            raise ContractError('Owner current/older generation identity differs')
        if recorded.get('generation_nonce') and (
                proposed.get('generation_nonce')!=recorded['generation_nonce']
                or owned.get('generation_nonce')!=recorded['generation_nonce']):
            raise ContractError('Owner generation nonce differs')
    if storage['remote_payload_bytes']<inventory['all_project_retained_payload_bytes']+inventory['synthetic_test_archive_bytes']:
        raise ContractError('Owner retained payload omitted')
    return inventory


def untouched_learner_bytes(learner):
    import io
    stream=io.BytesIO()
    learner.torch.save({'snapshot_id':'0'*64,'identity':{},'state':learner.capture_learning_state()},stream)
    return stream.tell()+65536  # exact tensor archive plus bounded identity/nonce overhead


class QualificationByteGuard:
    def __init__(self,learner_bytes):
        self.learner_bytes=learner_bytes
    def __call__(self,pilot,query,upper_bound):
        rows=len(pilot.bootstrap_rows)
        if pilot.mapper:
            known={(r.episode,r.pre_native,r.post_native) for r in pilot.bootstrap_rows}
            rows+=sum((r.episode,r.pre_native,r.post_native) not in known for r in pilot.mapper.rows)
        rows+=int(query['op']=='step')
        # 2x1024 +21 float32 values=8276B/row.11000B also covers each
        # metadata/mapping record and ZIP overhead;2MiB bounds cursor/pending,
        # state/report/ledger and final small JSON evidence.
        terminal=self.learner_bytes+2*1024**2+11000*rows
        exchange=pilot.recovery.transport.exchange
        staging=exchange.external_local_bytes+sum(p.stat().st_size for p in exchange.directory.rglob('*') if p.is_file())
        dependencies=max((r['archive_bytes']+r['payload_bytes'] for r in pilot.proofs.references),default=0)
        proof=pilot.bridge.call({'op':'qualificationStorage','terminalBaseBytes':terminal,
            'stagingBaseBytes':staging,'dependencyRestoreBytes':dependencies,
            'stagingLimitBytes':exchange.local_staging_limit_bytes})
        pilot.report['byteGuard']=proof
        return proof


@dataclass(frozen=True)
class QualificationPlan(PhasePlan):
    parity_seconds:float=60.
    parity_integrations:int=5060
    backup_payload_envelope_bytes:int=32*1024**2
    backup_archive_envelope_bytes:int=18*1024**2
    qualification_only:bool=True

    def stage_cutoff(self,name,cycle=0):
        if name=='immutable-input-restore':return self.started+300
        if name.startswith('backup:') or name in ('coherent-terminal-restore','terminal-retention'):
            return self.deadline
        if name not in PLAN['phases']:raise ContractError('Qualification phase not admitted')
        return self.work_end

    def assert_inputs_ready(self,now):
        if now>=self.started+300:raise ContractError('Qualification input300s cutoff')

    def admit_sequence(self,*args,**kwargs):
        raise ContractError('Qualification cannot admit a training cadence')


class QualificationPilot(ColdPilot):
    """One continuous qualification, one terminal application restore, no retry."""
    def __init__(self,*args,byte_guard,native_validator=None,**kwargs):
        super().__init__(*args,**kwargs)
        self.byte_guard=byte_guard;self.terminal_attempted=False
        self.physical_proof_mode=None;self.prior_raw_bytes=b'';self.last_snapshot_roles={}
        self.committed_raw_duplicate=None
        from getup_phase_proofs import PhaseProofs
        def missing_validator(paths):raise ContractError('Native phase restore validator missing')
        self.proofs=PhaseProofs(self,native_validator or missing_validator)
        self.diagnostic_restore_required=False
        self.untouched_learning_state=self.learner.capture_learning_state()
        self.report.update(mode='qualification-only',optimizerAuthorized=False,
            fullTrainingReplayReady=False,physicalQualificationPassed=False,
            holdEvidenceGranularity=PLAN['hold_evidence'],normalIntegrationEnvelope=112060,
            sharedReserveIntegrations=5020,maximumQualificationIntegrations=117080)

    def enter(self,name,seconds,integrations=0,*,evaluation=False,retention=False):
        if evaluation:raise ContractError('No candidate evaluation in qualification')
        if name.startswith('backup:'):
            if self.terminal_attempted:raise ContractError('One terminal generation only; no retry')
            self.terminal_attempted=True
            return super().enter(name,seconds,0,retention=True)
        phase=PLAN['phases'].get(name)
        if not phase or seconds!=phase['seconds'] or integrations!=phase['normal']:
            raise ContractError('Exact named qualification phase allowance required')
        if self.ledger()['nativeIntegrations']+integrations+5020>117080:
            raise ContractError('One shared reserve cannot be repeatedly allocated')
        super().enter(name,seconds,integrations+5020)
        if integrations:
            self.bridge.call({'op':'qualificationStage','name':name})

    def call(self,query,*,physical=False,upper_bound=5020):
        if physical:
            self.byte_guard(self,query,upper_bound)
            ledger=self.ledger()
            if ledger['nativeIntegrations']+upper_bound>117080:
                raise ContractError('Qualification aggregate including single reserve exhausted')
            if ledger.get('bootstrapDecisions',0)>=1024:
                raise ContractError('Inclusive qualification source cap reached')
        return super().call(query,physical=physical,upper_bound=upper_bound)

    def optimizer_gate(self):raise ContractError('Qualification optimizer is unconditionally closed')
    def warmup(self):self.optimizer_gate()
    def collect(self):self.optimizer_gate()
    def updates(self):self.optimizer_gate()
    def evaluate(self):raise ContractError('Qualification exports no candidate and performs no candidate evaluation')

    def finish_episode_rows(self,preservation):
        rows=self.mapper.completed_rows(preservation=preservation)
        offset=len(self.bootstrap_rows)
        self.bootstrap_rows.extend(rows)
        self.mapping.extend(dict(x,replay_index=x['replay_index']+offset) for x in self.mapper.mapping)
        # Keep the active runtime for full restoration, but its completed mapper
        # is no longer pending. Snapshot indices still point into diagnostic rows.
        self.mapper.rows=rows
        return rows

    def preservation(self,case):
        phase=PLAN['phases'][case+'-preservation']
        self.enter(case+'-preservation',phase['seconds'],phase['normal'])
        if case=='heldout':self.reset(case,'baseline-deterministic-zero')
        while not self.mapper.finished:
            action=[0.]*21 if self.state.get('preview') else None
            self.step(action,'bootstrap')
        rows=self.finish_episode_rows(True)
        first=self.call({'op':'standingReceipt'})
        hold=self.call({'op':'hold','firstSuccessReceipt':first},physical=True,upper_bound=29020)
        if not hold.get('passed'):raise ContractError('Preservation30s hold failed')
        self.state=self.call({'op':'state'})
        self.report.setdefault('preservation',{})[case]={'passed':True,'rows':len(rows),'hold':hold}

    def current_raw_bytes(self):
        import hashlib
        path=self.output/'transitions.ndjson'
        current=path.read_bytes() if path.exists() else b''
        if self.committed_raw_duplicate is not None:
            if hashlib.sha256(current).hexdigest()!=self.committed_raw_duplicate:
                raise ContractError('Committed raw cache changed during interrupted eviction')
            return b'' # exact durable dependency already owns these rows
        return current

    def raw_lines(self):
        raw=self.prior_raw_bytes+self.current_raw_bytes()
        return [json.loads(line) for line in raw.splitlines()]

    def validate_dependencies(self,restored,manifest):
        return self.proofs.resolve(restored,manifest)

    def verify_dependency_union(self,state,closure):
        from getup_cold_start_engine import encoded
        previous=closure.get('prior_rows',[])
        if encoded([asdict(x) for x in state.rows[:len(previous)]])!=encoded([asdict(x) for x in previous]):
            raise ContractError('Diagnostic replay union changed prior phase membership')
        if state.mapping!=self.mapping:
            raise ContractError('Diagnostic replay union changed global raw-span mapping')

    def prove_phase(self,phase):
        name=phase.lower()+'-proof';self.enter(name,180,0)
        if phase=='BELLY':self.diagnostic_replay()
        self.proofs.persist() # bind the now-verified seed identity, without changing old evidence
        self.physical_proof_mode=phase
        try:proof=self.recovery.checkpoint(self,'qualification-'+phase.lower(),self.phase_deadline)
        finally:self.physical_proof_mode=None
        ref=self.proofs.commit(phase,proof)
        # Only after durable reference + exact remote/application proof. This
        # releases a transient WASM object, never a required original checkpoint.
        self.call({'op':'qualificationRelease'})
        self.cursor.active=False;self.state=None;self.mapper=None;self.replay=None
        self.report.setdefault('immutablePhaseProofs',[]).append(ref)
        return ref

    def diagnostic_replay(self):
        rows=list(self.bootstrap_rows)
        if self.mapper:
            known={(r.episode,r.pre_native,r.post_native) for r in rows}
            for row,entry in zip(self.mapper.rows,self.mapper.mapping):
                if (row.episode,row.pre_native,row.post_native) not in known:
                    self.mapping.append(dict(entry,replay_index=len(rows)));rows.append(row)
        if not rows:raise ContractError('No populated diagnostic replay')
        self.replay=EpisodeReplay(rows,fresh_allowance=0,diagnostic_only=True)
        self.report['diagnosticReplay']={'rows':len(rows),**diagnostic_membership(rows,self.mapping),
            'trainingBatchAuthorized':False,'provisionalRecordCategoriesAreNotTrainingCoverage':True}
        # These are checks of diagnostic membership/RNG, not training batches.
        before=self.replay.rng.getstate()
        self.report['diagnosticReplay']['sampleIndices']=self.replay.diagnostic_sample_indices(16)
        self.replay.rng.setstate(before)

    def write_snapshot(self,boundary):
        unverified_seeds={}
        if not self.identity.get('seedManifestSha256'):
            # Preparation can stop after an earlier fixture was checkpointed
            # and freed, but before the once-only immutable seed transaction.
            # Its sole checkpoint is terminal evidence, not a verified seed.
            for case in ('original','heldout','belly'):
                name=case+'-seed.bin'
                if (self.output/name).is_file():
                    unverified_seeds['unverified_preparation_'+case+'_seed']=name
        self.report['unverifiedPreparationSeedEvidence']=list(unverified_seeds.values())
        seed_bytes=sum((self.output/name).stat().st_size for name in unverified_seeds.values())
        self.report['unverifiedPreparationSeedBytes']=seed_bytes
        # The immutable allowance can fail without discarding already-created
        # diagnostic state. JS writeCheckpoint reserved those exact bytes plus
        # complete terminal capture BEFORE writing; normal32MiB/18MiB terminal
        # limits still apply, and this never admits a seed upload or reuse.
        self.report['immutableSeedAdmissionFailed']=seed_bytes>2*1024**2
        try:
            capture=self.bridge.call({'op':'qualificationCapture'})
            self.report['terminalRuntimeCapture']=capture
            if capture.get('activeKnown'):
                # A failed reset/hold may have advanced before Python received
                # its result. Capture known paused native state, never replay
                # that command or pretend the old cached state is current.
                self.state=capture['state'];self.cursor.active=True
        except BaseException as error:self.report['terminalRuntimeCaptureError']=str(error)
        roles,cursor,_=super().write_snapshot('terminal-stop' if self.failure_latched else boundary,
            include_learner=not self.physical_proof_mode)
        import hashlib
        raw=self.output/'transitions.ndjson'
        current=self.current_raw_bytes()
        if self.committed_raw_duplicate is not None:
            # Interrupted cache eviction is not replay. Keep its exact original
            # local bytes, but archive an empty NEW segment and reference them.
            (self.output/'phase-current-transitions.ndjson').write_bytes(current)
            roles['transitions']='phase-current-transitions.ndjson'
        cursor['raw_sha256']=hashlib.sha256(self.prior_raw_bytes+current).hexdigest()
        cursor['raw_current_start_line']=len(self.prior_raw_bytes.splitlines())
        atomic_json(self.output/'phase-cursor.json',cursor)
        self.proofs.persist();roles['phase_dependencies']='phase-dependencies.json'
        roles.update(unverified_seeds)
        if self.replay is not None:
            cursor['diagnostic_membership']=diagnostic_membership(self.replay.rows,self.mapping)
            atomic_json(self.output/'phase-cursor.json',cursor)
        if cursor['runtime_capture_error'] or cursor['runtime_omitted_unresolved']:
            self.diagnostic_restore_required=False
        for role,name in [('native_call_evidence','qualification-native.ndjson'),
            ('fixtures','cold-fixtures.json'),('native_parity','cold-adapter-parity.json'),
            ('operational_inputs','operational-inputs.json'),('config','frozen-config.json'),
            ('transition_parity_runtime','parity-runtime.bin'),
            ('native_parity_runtime','native-parity-resume.bin'),
            ('immutable_input_references','phase-inputs.json')]:
            if (self.output/name).is_file():roles[role]=name
        for path in sorted(self.output.glob('cold-*-*.json'))+sorted(self.output.glob('cold-*-checkpoint.bin')):
            if path.name in roles.values() or path.name=='cold-start-report.json':continue
            roles['evidence_'+path.stem.replace('-','_')]=path.name
        for path in sorted(self.output.glob('qualification-runtime-*.bin')):
            roles[path.stem.replace('-','_')]=path.name
        working=sum(p.stat().st_size for p in self.output.rglob('*') if p.is_file()
            and not p.is_relative_to(self.output/'recovery'))
        if working>PLAN['active_output_bytes']:
            raise ContractError('Measured active qualification output exceeds32MiB')
        self.report['activeOutputBytes']=working
        # This is independently restorable diagnostic evidence, never an
        # automatic runtime resume or a training-readiness claim.
        self.last_snapshot_roles=roles.copy()
        return roles,cursor,False

    def run_qualification(self,*,include_original=False):
        try:
            if include_original:raise ContractError('Optional original workload permanently omitted')
            self.enter('zero-correction-qualification',120,11000)
            self.report['nativeQualification']=self.call({'op':'coldQualify'},physical=True,upper_bound=16020)
            self.prove_phase('PARITY')
            self.parity()
            self.preservation('belly');self.prove_phase('BELLY')
            self.preservation('heldout')
            self.report['qualificationPhysicalGatesPassed']=True
            self.report['stopped']='qualification-only-complete-no-training'
        except BaseException as error:
            self.failure_latched=True
            self.report.update(stopped='qualification-safe-stop',error=str(error))
        return self.finalize()

    def finalize(self):
        try:
            self.diagnostic_replay()
        except BaseException as error:self.report['diagnosticReplayError']=str(error)
        from getup_cold_start_verify import assert_state_equal
        assert_state_equal(self.untouched_learning_state,self.learner.capture_learning_state())
        ledger=self.durable_ledger()
        self.diagnostic_restore_required=(self.replay is not None and self.cursor.transition_parity
            and not ledger.get('pendingCall') and not ledger.get('uncertainCalls')
            and not ledger.get('uncertainIntegrations'))
        try:
            self.backup('qualification-terminal',retention=True)
            self.report['actualQualificationRestorePassed']=self.diagnostic_restore_required and not self.synthetic
            self.report['syntheticQualificationRestorePassed']=self.diagnostic_restore_required and self.synthetic
        except BaseException as error:self.report['terminalRetentionError']=str(error)
        self.report['ledger']=self.durable_ledger()
        atomic_json(self.output/'phase-report.json',self.report)
        return self.report

