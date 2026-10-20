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
    if PLAN['next_call_growth_bytes']!=20*1024**2:
        raise ContractError('Reviewed20MiB per-call growth bound changed')
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
        proof=pilot.bridge.call({'op':'qualificationStorage','terminalBaseBytes':terminal})
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
    def __init__(self,*args,byte_guard,**kwargs):
        super().__init__(*args,**kwargs)
        self.byte_guard=byte_guard;self.terminal_attempted=False
        self.diagnostic_restore_required=False
        self.untouched_learning_state=self.learner.capture_learning_state()
        self.report.update(mode='qualification-only',optimizerAuthorized=False,
            fullTrainingReplayReady=False,physicalQualificationPassed=False,
            holdEvidenceGranularity=PLAN['hold_evidence'],normalIntegrationEnvelope=137060,
            sharedReserveIntegrations=5020,maximumQualificationIntegrations=142080)

    def enter(self,name,seconds,integrations=0,*,evaluation=False,retention=False):
        if evaluation:raise ContractError('No candidate evaluation in qualification')
        if name.startswith('backup:'):
            if self.terminal_attempted:raise ContractError('One terminal generation only; no retry')
            self.terminal_attempted=True
            return super().enter(name,seconds,0,retention=True)
        phase=PLAN['phases'].get(name)
        if not phase or seconds!=phase['seconds'] or integrations!=phase['normal']:
            raise ContractError('Exact named qualification phase allowance required')
        if self.ledger()['nativeIntegrations']+integrations+5020>142080:
            raise ContractError('One shared reserve cannot be repeatedly allocated')
        super().enter(name,seconds,integrations+5020)
        if integrations:
            self.bridge.call({'op':'qualificationStage','name':name})

    def call(self,query,*,physical=False,upper_bound=5020):
        if physical:
            self.byte_guard(self,query,upper_bound)
            ledger=self.ledger()
            if ledger['nativeIntegrations']+upper_bound>142080:
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

    def original_diagnostic(self):
        phase=PLAN['phases']['original-diagnostic']
        self.enter('original-diagnostic',phase['seconds'],phase['normal'])
        self.reset('original','baseline-deterministic-zero')
        while not self.mapper.finished:
            action=[0.]*21 if self.state.get('preview') else None
            self.step(action,'bootstrap')
        rows=self.finish_episode_rows(False)
        self.report['originalDiagnostic']={'rows':len(rows),'outcome':rows[-1].episode_outcome,
            'extra30sHold':False,'learningTargetMayFail':True}

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
        try:
            capture=self.bridge.call({'op':'qualificationCapture'})
            self.report['terminalRuntimeCapture']=capture
            if capture.get('activeKnown'):
                # A failed reset/hold may have advanced before Python received
                # its result. Capture known paused native state, never replay
                # that command or pretend the old cached state is current.
                self.state=capture['state'];self.cursor.active=True
        except BaseException as error:self.report['terminalRuntimeCaptureError']=str(error)
        roles,cursor,_=super().write_snapshot('terminal-stop' if self.failure_latched else boundary)
        if self.replay is not None:
            cursor['diagnostic_membership']=diagnostic_membership(self.replay.rows,self.mapping)
            atomic_json(self.output/'phase-cursor.json',cursor)
        if cursor['runtime_capture_error'] or cursor['runtime_omitted_unresolved']:
            self.diagnostic_restore_required=False
        for role,name in [('native_call_evidence','qualification-native.ndjson'),
            ('fixtures','cold-fixtures.json'),('native_parity','cold-adapter-parity.json'),
            ('operational_inputs','operational-inputs.json'),('config','frozen-config.json'),
            ('transition_parity_runtime','parity-runtime.bin')]:
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
        return roles,cursor,False

    def run_qualification(self,*,include_original=False):
        try:
            self.enter('zero-correction-qualification',120,11000)
            self.report['nativeQualification']=self.call({'op':'coldQualify'},physical=True,upper_bound=16020)
            self.parity()
            self.preservation('belly');self.preservation('heldout')
            if include_original:self.original_diagnostic()
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

