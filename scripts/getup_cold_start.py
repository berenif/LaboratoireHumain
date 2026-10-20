"""Cold-start contracts. Never reinterpret the lost pilot's data or readiness.

This is a different experiment identity, not a bypass of require_execution_ready.
No physics or Torch import occurs here.
"""
from __future__ import annotations
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time

from experiment_backup import (RecoveryError, atomic_json, create_snapshot,
    sha256, verify_bundle, restore_bundle)
from getup_learning_loop import ContractError, LoopPhase, Record

PROTOCOL_PATH = Path(__file__).with_name('getup-cold-start-protocol.json')
PROTOCOL = json.loads(PROTOCOL_PATH.read_text())
SHARED_ROLES = ('worker_wasm', 'worker_glue', 'profile',
                'back_unstepped', 'belly_unstepped', 'runtime_unstepped_receipt',
                'cold_start_protocol', 'source_identity')
FORBIDDEN_PARTS = {'.git', '.toolchain', '.sites-runtime', 'node_modules',
                   '__pycache__', 'target', 'target-wasm', 'venv'}


def validate_qualification_boundary(report, ledger, config, amendment, run_id, commit, now):
    """One exact stopped boundary, not generic checkpoint resume or a new run."""
    if (amendment.get('schema')!='lh-cold-qualification-amendment/v1'
            or amendment.get('boundary')!='after-coldQualify-before-first-bootstrap-reset'
            or amendment.get('runId')!=run_id or report.get('runId')!=run_id
            or amendment.get('amendedSourceCommit')!=commit
            or amendment.get('originalSourceCommit')!=report.get('source',{}).get('commit')):
        raise ContractError('Wrong same-attempt identity/source/boundary')
    started,deadline=report['startedAtUnix'],report['deadlineUnix']
    if (amendment.get('startedAtUnix')!=started or amendment.get('deadlineUnix')!=deadline
            or deadline-started!=3600 or now>=deadline
            or config['inputContract']['startedAtUnixMs']!=round(started*1000)
            or config['inputContract']['deadlineUnixMs']!=round(deadline*1000)
            or ledger.get('deadlineUnixMs')!=round(deadline*1000)):
        raise ContractError('Original start/deadline cannot be renewed')
    exact={'nativeIntegrations':2380,'realIntegrations':2380,'forecastIntegrations':0,
           'trainingDecisions':0,'completedCalls':122,'uncertainIntegrations':0,
           'limit':300000,'evaluationReserveIntegrations':147000}
    if any(ledger.get(key)!=value for key,value in exact.items()):
        raise ContractError('Exact consumed qualification counters required; no refund')
    if (ledger.get('pendingCall') or ledger.get('uncertainCalls')
            or ledger.get('bootstrapDecisions',0) or ledger.get('iterativeDecisions',0)
            or report.get('ledger')!=ledger or report.get('trainingExecuted') is not False
            or report.get('accepted')!=0 or report.get('rejected')!=0
            or report.get('stopped')!='cold-start-safe-stop'
            or report.get('backupFailureLatched') is not False
            or report.get('error')!='Original absolute deadline or protected evaluation wall reached'
            or report.get('adapterParity',{}).get('passed') is not True
            or report.get('adapterParity',{}).get('physicsIntegrations')!=2380
            or 'bootstrap' in report or report.get('candidateAccepted') is not False):
        raise ContractError('Not the untouched, certain post-qualification stop boundary')
    if (amendment.get('learnerSeed')!=20261020 or amendment.get('genericResumeAuthorized') is not False
            or amendment.get('limitedUsefulOperationOnly') is not True):
        raise ContractError('Only untouched deterministic constructor reconstruction is allowed')
    return {'remaining_integrations':297620,'before_evaluation_integrations':150620,
            'deadline':deadline,'started':started}


def validate_backup_identity(request, receipt, application, run_id):
    if (not all(isinstance(x,dict) for x in (request,receipt,application))
            or any(x.get('synthetic',False) is not False for x in (request,receipt,application))
            or request.get('run_id')!=run_id
            or receipt.get('schema')!='lh-private-drive-receipt/v1'
            or receipt.get('owner_only') is not True or receipt.get('shared') is not False
            or receipt.get('download_completed') is not True
            or not receipt.get('drive_file_id') or application.get('passed') is not True
            or application.get('physics_integrations')!=0
            or application.get('archive_sha256')!=request.get('archive_sha256')
            or any(receipt.get(k)!=request.get(k) for k in
                   ('run_id','kind','archive_sha256','archive_bytes','manifest_sha256'))):
        raise ContractError('Missing exact private operational backup receipt')


def verify_qualification_amendment(output, amendment_path, run_id, commit, now):
    """Bind original artifacts/receipts and restore cumulative storage accounting."""
    output=Path(output);amendment=json.loads(Path(amendment_path).read_text())
    report=json.loads((output/'cold-start-report.json').read_text())
    ledger=json.loads((output/'budget-ledger.json').read_text())
    config=json.loads((output/'frozen-config.json').read_text())
    validate_qualification_boundary(report,ledger,config,amendment,run_id,commit,now)
    import importlib.metadata,platform,subprocess
    dependencies={'python':platform.python_version(),
                  'torch':importlib.metadata.version('torch'),'numpy':importlib.metadata.version('numpy')}
    if amendment.get('dependencies')!=dependencies:
        raise ContractError('Untouched learner dependency identity differs')
    root=Path(__file__).resolve().parents[1]
    constructors=('scripts/train-getup-learning-loop.py','scripts/train-getup-wasm-adaptation.py',
                  'scripts/getup-training/SAC.py')
    if set(amendment.get('constructorSources',{}))!=set(constructors):
        raise ContractError('Exact untouched learner constructor source bindings required')
    for name in constructors:
        digest=sha256(root/name)
        original=subprocess.check_output(['git','show',report['source']['commit']+':'+name],cwd=root)
        if digest!=amendment['constructorSources'][name] or hashlib.sha256(original).hexdigest()!=digest:
            raise ContractError('Untouched seeded constructor changed')
    required={'cold-start-report.json','budget-ledger.json','frozen-config.json',
              'cold-fixtures.json','cold-adapter-parity.json','operational-inputs.json',
              'original-seed.bin','heldout-seed.bin','belly-seed.bin'}
    if set(amendment.get('bindings',{}))!=required:
        raise ContractError('Exact boundary artifact bindings required')
    for name,digest in amendment['bindings'].items():
        if sha256(output/name)!=digest:raise ContractError('Boundary artifact changed: '+name)
    for name in ('transitions.ndjson','replay.npz','learner.pt','runtime-checkpoint.bin'):
        if (output/name).exists():raise ContractError('Bootstrap/learning already began')
    if (output/'continuation-claim.json').exists():
        raise ContractError('Same-attempt amendment already consumed; no repeat')
    manifest,_=input_preflight(config['coldStart']['manifest'])
    fixtures=json.loads((output/'cold-fixtures.json').read_text())
    for case in ('original','heldout','belly'):
        if sha256(output/(case+'-seed.bin'))!=fixtures['cases'][case]['checkpointSha256']:
            raise ContractError('Triggered immutable seed changed')
    exchange=DriveExchange(output/'recovery',run_id,
        {'commit':commit,'dirty':False,'description':'Audited same-attempt boundary amendment'},
        report['deadlineUnix'])
    seen=set()
    for item in amendment.get('backups',[]):
        paths={kind:output/item[kind] for kind in ('request','receipt','application')}
        for kind,path in paths.items():
            if sha256(path)!=item[kind+'_sha256']:raise ContractError('Backup evidence changed')
        request,receipt,application=[json.loads(paths[k].read_text()) for k in ('request','receipt','application')]
        validate_backup_identity(request,receipt,application,run_id)
        if sha256(request['archive_path'])!=request['archive_sha256']:
            raise ContractError('Immutable original archive changed')
        archived=verify_bundle(request['archive_path'],request['archive_sha256'])
        archived_hashes={entry['role']:entry['sha256'] for entry in archived['files']}
        kind=request['kind']
        if kind in seen:raise ContractError('Duplicate boundary backup kind')
        seen.add(kind);exchange.remote_uploaded_bytes+=request['archive_bytes']
        if kind=='shared':
            if archived_hashes.get('input_manifest')!=sha256(config['coldStart']['manifest']):
                raise ContractError('Original operational inputs differ from retained archive')
            exchange.shared_sha256=request['manifest_sha256'];exchange.shared_bytes=request['payload_bytes']
            exchange.shared_archive=request['archive_path']
        elif kind=='seeds':
            if any(archived_hashes.get(case+'_seed')!=sha256(output/(case+'-seed.bin'))
                   for case in ('original','heldout','belly')):
                raise ContractError('Triggered seeds differ from retained archive')
            exchange.seed_sha256=request['manifest_sha256'];exchange.seed_bytes=request['payload_bytes']
        elif kind=='generation':
            if request['boundary']!='partial-safe-stop-evidence':raise ContractError('Wrong stop evidence')
            if any(archived_hashes.get(role)!=sha256(output/name) for role,name in
                   [('report','cold-start-report.json'),('ledger','budget-ledger.json'),
                    ('config','frozen-config.json')]):
                raise ContractError('Original terminal report/ledger differs from retained archive')
            exchange.verified.append(dict(manifest_sha256=request['manifest_sha256'],
                drive_file_id=receipt['drive_file_id'],payload_bytes=request['payload_bytes'],
                archive_path=request['archive_path'],archive_bytes=request['archive_bytes']))
        else:raise ContractError('Unexpected backup kind')
    owner=json.loads((output/'recovery/drive-transfer-owner-ledger.json').read_text())
    if (seen!={'shared','seeds','generation'} or exchange.remote_uploaded_bytes!=owner['cumulative_uploaded_bytes']
            or owner['trashed_bytes']!=0
            or exchange.remote_uploaded_bytes>PROTOCOL['retained_payload_bytes']):
        raise ContractError('Original cumulative Drive accounting/receipts incomplete')
    return amendment,report,config,exchange


def require_full_phase_capacity():
    raise ContractError('Full phase capacity remains unadmitted; one initialization is not evaluation/learning evidence')


def checked_files(files):
    """Hash exact allowlisted files; reject build caches and unbounded payloads."""
    result = {}
    for role, item in files.items():
        path = Path(item['path']).resolve()
        if FORBIDDEN_PARTS.intersection(path.parts):
            raise ContractError('Backup cannot contain repository history/caches/toolchains')
        if not path.is_file() or path.is_symlink():
            raise ContractError(f'Missing regular input: {role}')
        if item.get('sha256') != sha256(path):
            raise ContractError(f'Cold-start input hash mismatch: {role}')
        if item.get('bytes') != path.stat().st_size:
            raise ContractError(f'Cold-start input size mismatch: {role}')
        result[role] = str(path)
    return result


def input_preflight(manifest_path):
    path = Path(manifest_path)
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != PROTOCOL['schema'] or manifest.get('synthetic') is not False:
        raise ContractError('Distinct real cold-start manifest is required')
    if manifest.get('protocol') != PROTOCOL:
        raise ContractError('Cold-start caps or physical acceptance changed')
    if set(manifest.get('files', {})) != set(SHARED_ROLES):
        raise ContractError('Exact cold-start shared input roles are required')
    files = checked_files(manifest['files'])
    if sum(item['bytes'] for item in manifest['files'].values()) > PROTOCOL['shared_payload_bytes']:
        raise ContractError('Shared immutable payload byte cap')
    runtime = json.loads(Path(files['runtime_unstepped_receipt']).read_text())
    if runtime.get('physicsIntegrations') != 0 or runtime.get('passed') is not True:
        raise ContractError('Actual unstepped runtime verification is required')
    for case, role in [('back', 'back_unstepped'), ('belly', 'belly_unstepped')]:
        receipt = runtime.get('cases', {}).get(case, {})
        if (receipt.get('nativeBefore') != 0 or receipt.get('nativeAfter') != 0
                or receipt.get('checkpointSha256') != manifest['files'][role]['sha256']
                or receipt.get('roundtripStateExact') is not True):
            raise ContractError('Unstepped runtime receipt/hash differs')
    return manifest, files


class ColdPhase(LoopPhase):
    """Bootstrap never borrows decisions from the 12 iterative blocks."""
    def __init__(self, bootstrap_decisions, planned_blocks=12):
        if type(bootstrap_decisions) is not int or not 0 < bootstrap_decisions <= 1024:
            raise ContractError('Bootstrap decision cap')
        super().__init__()
        self.bootstrap_decisions = bootstrap_decisions
        if type(planned_blocks) is not int or not 1<=planned_blocks<=12:
            raise ContractError('Predeclared iterative block count must fit original cap')
        self.planned_blocks=planned_blocks

    def begin_collection(self, policy_id):
        if self.cycle >= self.planned_blocks:
            raise ContractError('Cold-start iterative block cap')
        super().begin_collection(policy_id)

    def collected(self, policy_id):
        if self.fresh >= 3072 or self.bootstrap_decisions + self.fresh >= 4096:
            raise ContractError('Combined cold-start decision cap')
        super().collected(policy_id)

    def finish_block(self):
        super().finish_block()
        if self.cycle == self.planned_blocks and self.phase == 'ready_collection':
            self.phase = 'finished'


class FreshMapper:
    """Map actual bridge calls, including pending standing tails, to replay.

    The mapper consumes executed source/result pairs, not reconstructed audit
    labels. Pending origin survives a pause and its final reward remains folded.
    """
    def __init__(self, builder, episode, behavior, case, first_raw_line=0, *, synthetic=False):
        if case not in ('original', 'heldout', 'belly'):
            raise ContractError('Unknown fresh physical case')
        self.builder, self.episode, self.behavior, self.case = builder, episode, behavior, case
        self.pending_origin = None
        self.rows = []
        self.raw_calls = 0
        self.first_raw_line = first_raw_line
        self.pending_line = None
        self.mapping = []
        self.finished = False
        self.synthetic = synthetic

    def push(self, source, correction, result):
        if self.finished:
            raise ContractError('Fresh episode continued past its terminal')
        post = result['state']
        if (result['native'] != post['native'] or post['native'] != source['native'] + 20
                or post['impactEpoch'] != source['impactEpoch']
                or post['sourceEpoch'] != source['sourceEpoch']):
            raise ContractError('Fresh native/impact/source chronology changed')
        if source.get('preview'):
            if correction is None or len(correction) != 21:
                raise ContractError('Source decision has no exact applied correction')
            receipt = post.get('lastCorrection')
            if (not receipt or receipt['native_substep'] != source['native']
                    or max(abs(a-b) for a,b in zip(receipt['correction'], correction)) > 2e-6):
                raise ContractError('Applied correction differs from source decision')
            self.pending_origin = (source, tuple(correction))
            self.pending_line = self.first_raw_line + self.raw_calls
        elif correction is not None:
            raise ContractError('Standing tail cannot introduce a new source action')
        transition = self.builder.push(source, correction, result)
        self.raw_calls += 1
        if transition:
            if self.pending_origin is None:
                raise ContractError('Missing pending source origin')
            origin, action = self.pending_origin
            holding = origin['preview']['referenceDecision'] >= origin['status']['reference_frames']
            duration = (post['native'] - origin['native']) // 20
            import struct
            def f32(values):
                return struct.unpack('<'+str(len(values))+'f',struct.pack('<'+str(len(values))+'f',*values))
            row = Record(self.episode, 'back_holding' if holding else 'back_pre_holding',
                'holding' if holding else 'pre_holding', self.behavior,
                f32(transition[0]), f32(action), f32(transition[2]), float(transition[3]),
                0. if result['terminal'] else self.builder.gamma ** duration,
                bool(result['terminal']), False, origin['native'], post['native'],
                origin['impactEpoch'], bool(result['terminal']),
                'synthetic_rehearsal' if self.synthetic else 'fresh_wasm',
                'success' if result['success'] else 'failure' if result['terminal'] else 'pending',
                False).validate()
            self.rows.append(row)
            self.mapping.append({'replay_index': len(self.rows)-1, 'episode_id': self.episode,
                'behavior_policy_id': self.behavior, 'case': self.case,
                'transition_line_start': self.pending_line,
                'transition_line_end': self.first_raw_line + self.raw_calls-1,
                'pre_native': row.pre_native, 'post_native': row.post_native,
                'discount': row.discount, 'reward': row.reward})
            self.pending_origin = None
            self.pending_line = None
        if result['terminal']:
            self.finished = True
        return self.rows[-1] if transition else None

    def completed_rows(self, preservation=False):
        from dataclasses import replace
        if not self.finished or not self.rows or self.builder.pending is not None:
            raise ContractError('A completed physical episode is required')
        outcome = self.rows[-1].episode_outcome
        if preservation and (self.case == 'original' or outcome != 'success'
                or any(any(c != 0 for c in row.correction) for row in self.rows)):
            raise ContractError('Preservation requires complete zero-correction nearby/BELLY success')
        if self.case != 'original' and not preservation:
            raise ContractError('Nearby/BELLY cannot be relabeled as BACK learning data')
        return [replace(row, complete_episode=True, episode_outcome=outcome,
            category='baseline_preservation' if preservation else row.category,
            baseline_success=preservation).validate() for row in self.rows]


class DriveExchange:
    """Operator-mediated official Drive transfer, with a fail-closed handshake.

    A trusted transfer owner uploads exactly the request bundle, confirms private
    ownership, downloads it through the connector and writes a receipt. Bytes
    are independently checked here. A receipt is NOT a restart authorization.
    No connector credentials, download URL, or generic Library workaround exist.
    """
    def __init__(self, directory, run_id, source, deadline, shared_sha256=None, *, clock=None, synthetic=False):
        self.clock=clock;self.synthetic=synthetic
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.run_id, self.source, self.deadline = run_id, source, deadline
        self.shared_sha256 = shared_sha256
        self.shared_bytes = 0
        self.shared_archive = None
        self.seed_sha256 = None
        self.seed_bytes = 0
        self.retired_payload_bytes = 0  # Recoverable Drive Trash can still consume quota.
        self.remote_uploaded_bytes = 394  # Existing explicitly scoped tiny Drive drill.
        self.external_local_bytes = 0
        self.operation_deadline = deadline
        self.verified = []
        self.failed = False

    def now(self):return self.clock.now() if self.clock else time.time()

    def sleep(self,seconds):
        if self.clock:self.clock.sleep(seconds)
        else:time.sleep(seconds)

    def snapshot(self, artifacts, config, boundary, kind='generation', wall_reserve=0.):
        if self.failed:
            raise ContractError('Prior failed/uncertain Drive operation; no automatic retry')
        self.operation_deadline=self.deadline-wall_reserve
        if not 0<=wall_reserve<3600 or self.now() >= self.operation_deadline:
            raise ContractError('Original cold-start deadline reached')
        if kind not in ('shared', 'seeds', 'generation') or (kind == 'shared' and self.shared_sha256):
            raise ContractError('Immutable shared inputs are retained once')
        if kind in ('seeds','generation') and not self.shared_sha256:
            raise ContractError('Shared inputs must be remotely verified first')
        if kind=='seeds' and self.seed_sha256:
            raise ContractError('Triggered immutable seeds are retained once')
        if kind in getattr(self,'pending_immutable',{}):
            raise ContractError('Unresolved immutable upload; no retry')
        limit = PROTOCOL['shared_payload_bytes' if kind in ('shared','seeds') else 'generation_payload_bytes']
        qualification=kind=='generation' and config.get('plan',{}).get('qualification_only') is True
        if qualification:limit=min(limit,32*1024**2)
        total = 0
        for item in artifacts:
            path = Path(item['path']).resolve()
            if FORBIDDEN_PARTS.intersection(path.parts):
                raise ContractError('Caches/history excluded from backup')
            total += path.stat().st_size
        if total > limit:
            raise ContractError('Measured payload exceeds hard byte cap before copying/upload')
        if kind=='seeds' and self.shared_bytes+total>PROTOCOL['shared_payload_bytes']:
            raise ContractError('Combined immutable input+seed byte cap')
        peak = self.shared_bytes + self.seed_bytes + self.retired_payload_bytes + sum(x['payload_bytes'] for x in self.verified) + total
        if peak > PROTOCOL['retained_payload_bytes']:
            raise ContractError('Peak shared+current+previous+pending exceeds byte cap before upload')
        # Backup-root storage only; the original working inputs are not copied
        # here until admission. Reserve payload + archive simultaneously.
        retained_local = self.external_local_bytes + sum(p.stat().st_size for p in self.directory.rglob('*') if p.is_file())
        staging=(max(2*total,total+2*18*1024**2,
            18*1024**2+config.get('slot_prewrite_download_bytes',0)) if qualification else 3*total)
        if retained_local + staging + 4*1024**2 > PROTOCOL['retained_payload_bytes']:
            raise ContractError('Local archive/staging peak exceeds backup byte cap before copying')
        if kind == 'generation' and any(item['role'] in SHARED_ROLES for item in artifacts):
            raise ContractError('Generation must reference immutable inputs, not copy them')
        archive = create_snapshot(self.directory / 'bundles', dict(run_id=self.run_id,
            synthetic=self.synthetic, source=self.source, provenance=boundary,
            config={'cold_start': config, 'shared_manifest_sha256': self.shared_sha256,
                    'seed_manifest_sha256':self.seed_sha256},
            seed=20261020, artifacts=artifacts, required_roles=[x['role'] for x in artifacts],
            excluded_dependencies=[], max_payload_bytes=limit,
            application_resume_eligible=(kind=='generation' and config.get('resumeAuthorized') is not False),
            previous_manifest_sha256=self.verified[-1]['manifest_sha256'] if self.verified else None))
        if archive.stat().st_size > min(18*1024**2 if qualification else 32*1024**2,limit + 1024**2):
            raise ContractError('Measured compressed bundle exceeds byte cap')
        if self.remote_uploaded_bytes+archive.stat().st_size>PROTOCOL['retained_payload_bytes']:
            raise ContractError('Cumulative remote archive bytes including Trash exceed cap before upload')
        manifest = verify_bundle(archive, max_payload_bytes=limit)
        request = {'schema': 'lh-private-drive-request/v1', 'run_id': self.run_id,
            'synthetic':self.synthetic,'kind': kind, 'boundary': boundary, 'archive_path': str(archive),
            'archive_sha256': sha256(archive), 'archive_bytes': archive.stat().st_size,
            'payload_bytes': total, 'manifest_sha256': hashlib.sha256(
                __import__('zipfile').ZipFile(archive).read('manifest.json')).hexdigest(),
            'required_roles': manifest['required_roles'], 'deadline_unix': self.operation_deadline,
            'absolute_deadline_unix': self.deadline,'retired_payload_bytes':self.retired_payload_bytes,
            'remote_cumulative_archive_bytes_before':self.remote_uploaded_bytes,
            'shared_manifest_sha256': self.shared_sha256,'seed_manifest_sha256':self.seed_sha256,
            'verified_generations': self.verified,
            'max_verified_generations': 2, 'max_retained_payload_bytes': 167772160,
            'privacy': 'owner-only; no shared link; exact archive only',
            'retention': 'current+previous only; NEVER prune before application-verified prune request',
            'download_cleanup': 'transfer owner removes its exact temporary download only after application-verified marker'}
        suffix='.slot-draft.json' if getattr(self,'_unpublished_slot_request',False) else '.request.json'
        request_path = self.directory / (manifest['generation'] + suffix)
        if config.get('qualification_immutable') is True:
            self.precharge_immutable(request)
        atomic_json(request_path, request)
        return request_path

    def precharge_immutable(self,request):
        kind=request['kind']
        if kind not in ('shared','seeds') or kind in getattr(self,'pending_immutable',{}):
            raise ContractError('Immutable upload already pending; no retry')
        after=self.remote_uploaded_bytes+request['archive_bytes']
        if after>PROTOCOL['retained_payload_bytes']:
            raise ContractError('Pending immutable exposure exceeds cumulative cap')
        self.pending_immutable=getattr(self,'pending_immutable',{})
        self.pending_immutable[kind]=request['archive_sha256']
        request.update(cumulative_precharged=True,remote_cumulative_exposure_after=after,
            uncertain_write_policy='charged before request; no refund or automatic retry')
        self.remote_uploaded_bytes=after

    def verify_receipt(self, request_path, receipt_path, restore_validator):
        if self.failed:
            raise ContractError('Prior failed/uncertain Drive operation; no retry')
        try:
            if self.now() >= self.operation_deadline:
                raise ContractError('Original cold-start deadline expired during backup')
            request, receipt = [json.loads(Path(p).read_text()) for p in (request_path, receipt_path)]
            if (receipt.get('synthetic',False) is not self.synthetic
                    or request.get('synthetic',False) is not self.synthetic):
                raise ContractError('Scripted and actual Drive receipts cannot be mixed')
            expected = ('run_id', 'kind', 'archive_sha256', 'archive_bytes', 'manifest_sha256')
            if receipt.get('schema') != 'lh-private-drive-receipt/v1' or any(receipt.get(k) != request[k] for k in expected):
                raise ContractError('Drive receipt does not bind the exact requested bundle')
            if (receipt.get('owner_only') is not True or receipt.get('shared') is not False
                    or not receipt.get('drive_file_id') or receipt.get('download_completed') is not True):
                raise ContractError('Private remote upload/download has not been verified')
            downloaded = Path(receipt['downloaded_path'])
            if downloaded.stat().st_size != request['archive_bytes']:
                raise ContractError('Remote downloaded byte count differs')
            manifest = verify_bundle(downloaded, request['archive_sha256'],
                PROTOCOL['shared_payload_bytes' if request['kind'] in ('shared','seeds') else 'generation_payload_bytes'])
            if manifest['synthetic'] is not self.synthetic:
                raise ContractError('Scripted archive cannot open an actual recovery gate')
            if set(manifest['required_roles']) != set(request['required_roles']):
                raise ContractError('Downloaded required roles differ')
            if hashlib.sha256(__import__('zipfile').ZipFile(downloaded).read('manifest.json')).hexdigest() != request['manifest_sha256']:
                raise ContractError('Downloaded manifest identity differs')
            # Bytes alone do not establish an operational application restore.
            # Never load an arbitrary pickle received from a remote stranger:
            # downloaded bytes must match the locally produced, pinned archive.
            restored = self.directory / ('restore-' + manifest['generation'])
            restore_bundle(downloaded, restored, request['archive_sha256'],
                PROTOCOL['shared_payload_bytes' if request['kind'] in ('shared','seeds') else 'generation_payload_bytes'])
            try:
                operational = restore_validator(restored, manifest)
                if (not isinstance(operational, dict) or operational.get('passed') is not True
                        or operational.get('physics_integrations') != 0):
                    raise ContractError('Application restore validator did not establish an exact unstepped restore')
            finally:
                shutil.rmtree(restored)
            verified_path=Path(request_path).with_name(Path(request_path).name.replace('.request.json','.application-verified.json'))
            atomic_json(verified_path,{'passed':True,'synthetic':self.synthetic,'archive_sha256':request['archive_sha256'],
                'physics_integrations':0,'downloaded_path':str(downloaded)})
            self.commit_verified(request, receipt, request_path, verified_path)
            return manifest
        except BaseException:
            self.failed = True
            raise


    def commit_verified(self, request, receipt, request_path, verified_path):
        generations = self.verified + [dict(manifest_sha256=request['manifest_sha256'],
            drive_file_id=receipt['drive_file_id'], payload_bytes=request['payload_bytes'],
            archive_path=request['archive_path'],archive_bytes=request['archive_bytes'])]
        if request.get('cumulative_precharged'):
            if getattr(self,'pending_immutable',{}).get(request['kind'])!=request['archive_sha256']:
                raise ContractError('Immutable cumulative reservation identity differs')
            del self.pending_immutable[request['kind']]
        else:self.remote_uploaded_bytes+=request['archive_bytes']
        if request['kind'] == 'shared':
            self.shared_sha256 = request['manifest_sha256']
            self.shared_bytes = request['payload_bytes']
            self.shared_archive = request['archive_path']
        elif request['kind']=='seeds':
            self.seed_sha256=request['manifest_sha256']
            self.seed_bytes=request['payload_bytes']
        else:
            stale = generations[:-2]
            if receipt.get('pruned_drive_file_ids'):
                raise ContractError('Prior generation was pruned before application restore validation')
            if stale:
                prune_request=Path(request_path).with_name(Path(request_path).name.replace('.request.json','.prune-request.json'))
                prune_receipt=prune_request.with_name(prune_request.name.replace('.prune-request.json','.pruned.json'))
                expected_ids={x['drive_file_id'] for x in stale}
                atomic_json(prune_request,{'schema':'lh-private-drive-prune/v1','run_id':self.run_id,
                    'replacement_manifest_sha256':request['manifest_sha256'],
                    'application_verified':str(verified_path),'drive_file_ids':sorted(expected_ids),
                    'deadline_unix':self.operation_deadline,'receipt':str(prune_receipt)})
                print(json.dumps({'event':'DRIVE_PRUNE_REQUEST','request':str(prune_request)}),flush=True)
                while not prune_receipt.exists():
                    if self.now()>=self.operation_deadline:
                        raise ContractError('Deadline expired awaiting exact owned-generation cleanup')
                    self.sleep(.5)
                pruned=json.loads(prune_receipt.read_text())
                if (pruned.get('replacement_manifest_sha256')!=request['manifest_sha256']
                        or set(pruned.get('deleted_drive_file_ids',[]))!=expected_ids):
                    raise ContractError('Current+previous retention cleanup is not acknowledged')
            generations = generations[-2:]
            self.retired_payload_bytes+=sum(x['payload_bytes'] for x in stale)
            if self.shared_bytes+self.seed_bytes+self.retired_payload_bytes + sum(x['payload_bytes'] for x in generations) > 167772160:
                raise ContractError('Total retained payload cap')
            self.verified = generations
            for old in stale:
                old_archive = Path(old['archive_path']).resolve()
                # Delete only exact generations created by this instance.
                if old_archive.parent.parent == (self.directory / 'bundles').resolve():
                    shutil.rmtree(old_archive.parent)
                # Prior closed-run evidence outside this instance is NEVER
                # deleted; its bytes remain in external_local_bytes.


def artifact(role, path, schema):
    return {'role': role, 'path': str(Path(path).resolve()), 'schema': schema,
            'provenance': 'Fresh cold-start; not historical reconstruction',
            'retention_class': 'user_generated'}
