"""Two immutable physical phase proofs plus one terminal slot, no new runner.

Only exact new-run cache duplicates may be evicted. Every reference is durable
before eviction and can be resolved by ordinary private download. No physics,
learner restoration, restart permission or quota credit is implied by a proof.
"""
import hashlib,json,shutil,tempfile
from pathlib import Path
from dataclasses import asdict
from experiment_backup import atomic_json,sha256,verify_bundle,restore_bundle
from getup_learning_loop import ContractError

def reference_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def ledger_interval(begin,end,*,allow_uncertain=False):
    if (begin.get('deadlineUnixMs')!=end.get('deadlineUnixMs')
            or not allow_uncertain and (end.get('pendingCall') or end.get('uncertainCalls') or end.get('uncertainIntegrations'))):
        raise ContractError('Phase proof cannot certify renewed/uncertain execution')
    for key in ('nativeIntegrations','realIntegrations','forecastIntegrations','completedCalls',
                'trainingDecisions','bootstrapDecisions'):
        if end.get(key,0)<begin.get(key,0):raise ContractError('Phase proof refunded '+key)

class PhaseProofs:
    def __init__(self,pilot,native_validator):
        self.pilot=pilot;self.native_validator=native_validator;self.references=[]
        self.path=pilot.output/'phase-dependencies.json'
        if self.path.exists():raise ContractError('No phase-proof restart or existing journal reuse')
        self.start=pilot.durable_ledger();self.persist()

    def persist(self):
        atomic_json(self.path,{'schema':'lh-qualification-phase-dependencies/v1',
            'identity':self.pilot.identity,'references':self.references,
            'automatic_restart_authorized':False})

    def verify_native(self,restored,manifest):
        checkpoints=[Path(restored)/x['path'] for x in manifest['files']
            if x['role'] in ('runtime','transition_parity_runtime','native_parity_runtime')
            or x['path'].endswith('-checkpoint.bin')]
        if checkpoints:
            value=self.native_validator(checkpoints)
            if value.get('passed') is not True or value.get('physics_integrations')!=0:
                raise ContractError('Physical phase checkpoint application restore failed')

    def verify_native_interval(self,files,begin,end):
        if 'native_call_evidence' not in files:
            if end['completedCalls']!=begin['completedCalls']:raise ContractError('Native call interval evidence missing')
            return
        rows=[json.loads(x) for x in files['native_call_evidence'].read_text().splitlines()]
        native=begin['nativeIntegrations'];real=forecast=0
        for index,row in enumerate(rows,begin['completedCalls']+1):
            r=row['postNative']-row['preNative'];f=row['after']['forecast']-row['before']['forecast']
            if r<0 or f<0 or row['callSequence']!=index or row['failure'] is not None:
                raise ContractError('Native interval reset/failure/call gap')
            real+=r;forecast+=f;native+=r+f
            if row['ledgerNativeEnd']!=native:raise ContractError('Native interval charge mismatch')
        if (len(rows)!=end['completedCalls']-begin['completedCalls']
                or native!=end['nativeIntegrations']
                or real!=end['realIntegrations']-begin['realIntegrations']
                or forecast!=end['forecastIntegrations']-begin['forecastIntegrations']):
            raise ContractError('Native interval does not conserve full phase charges')

    def verify_phase_evidence(self,restored,manifest,phase):
        files={x['role']:Path(restored)/x['path'] for x in manifest['files']}
        if 'learner' in files:raise ContractError('Physical phase duplicated learner')
        if 'immutable_input_references' not in files:raise ContractError('Artifact/profile/seed dependencies absent')
        inputs=json.loads(files['immutable_input_references'].read_text())
        if inputs.get('identity')!=self.pilot.identity:raise ContractError('Phase input identity differs')
        for name,key in [('shared','sharedManifestSha256'),('seeds','seedManifestSha256')]:
            ref=inputs[name]
            if (ref['manifest_sha256']!=self.pilot.identity[key] or not ref['drive_file_id']
                    or len(ref['archive_sha256'])!=64 or ref['archive_bytes']<=0
                    or not ref['members']):raise ContractError('Immutable input closure incomplete')
        report=json.loads(files['report'].read_text())['report']
        if phase=='PARITY':
            if not {'native_parity','native_parity_runtime','native_call_evidence','fixtures'}<=files.keys():
                raise ContractError('Complete native parity evidence absent')
            parity=json.loads(files['native_parity'].read_text())
            if (parity.get('passed') is not True or parity.get('zeroCorrectionSamples')!=400
                    or parity.get('resumedSamples')!=200
                    or parity.get('resumeCheckpointSha256')!=sha256(files['native_parity_runtime'])
                    or report.get('nativeQualification')!=parity):
                raise ContractError('Exact400/200 native parity proof differs')
        elif phase=='BELLY':
            result=report.get('preservation',{}).get('belly',{})
            checkpoints=[p for p in files.values() if p.name.endswith('-checkpoint.bin')]
            holds=[p for p in files.values() if p.name.endswith('-30s-hold.json')]
            if (result.get('passed') is not True or not json.loads(files['cursor'].read_text())['phase_cursor']['transition_parity']
                    or not any('first800' in p.name for p in checkpoints) or not holds):
                raise ContractError('BELLY parity/first800/hold proof absent')
            # The hold receipt binds exact final checkpoint bytes. Maxima are
            # computed over every native sample by the existing frozen worker.
            hold=json.loads(holds[0].read_text())
            if (hold.get('passed') is not True or hold.get('samples')!=24000
                    or hold.get('physicalSeconds')!=30
                    or hold.get('finalCheckpointSha256') not in [sha256(p) for p in checkpoints]):
                raise ContractError('BELLY exact final30s checkpoint/receipt differs')
        else:raise ContractError('Unreviewed immutable physical phase')

    def commit(self,phase,proof):
        expected=['PARITY','BELLY']
        if len(self.references)>=2 or phase!=expected[len(self.references)]:
            raise ContractError('Exact ordered immutable phase proofs required')
        request_path=Path(proof['request_path']);request=json.loads(request_path.read_text())
        receipt_path=request_path.with_name(request_path.name.replace('.request.json','.receipt.json'))
        marker_path=request_path.with_name(request_path.name.replace('.request.json','.application-verified.json'))
        receipt=json.loads(receipt_path.read_text());marker=json.loads(marker_path.read_text())
        manifest=verify_bundle(request['archive_path'],request['archive_sha256'],32*1024**2)
        config=manifest['config'].get('cold_start',manifest['config'])
        if (request['kind']!='phase-proof' or config.get('physical_phase_proof')!=phase
                or config.get('learner_restore_claimed') is not False
                or marker.get('passed') is not True or marker.get('physics_integrations')!=0
                or marker.get('archive_sha256')!=request['archive_sha256']
                or receipt.get('archive_sha256')!=request['archive_sha256']
                or receipt.get('owner_only') is not True or receipt.get('shared') is not False):
            raise ContractError('Phase proof lacks exact private application verification')
        if request['run_id']!=self.pilot.identity['runId']:
            raise ContractError('Phase proof run identity differs')
        end=self.pilot.durable_ledger();ledger_interval(self.start,end)
        ref={'schema':'lh-immutable-physical-phase/v1','phase':phase,'status':'application-verified',
            'run_id':request['run_id'],'identity':self.pilot.identity.copy(),
            'drive_file_id':receipt['drive_file_id'],'archive_sha256':request['archive_sha256'],
            'archive_bytes':request['archive_bytes'],'manifest_sha256':request['manifest_sha256'],
            'payload_bytes':request['payload_bytes'],'members':manifest['files'],
            'ledger_begin':self.start,'ledger_end':end,'snapshot_id':config['snapshot_id'],
            'learner_restore_claimed':False,'dependencies':[r['reference_sha256'] for r in self.references]}
        ref['reference_sha256']=reference_hash(ref)
        self.references.append(ref);self.persist() # COMMIT POINT before any cache eviction.
        self.start=end
        # Retain exact prior raw chronology in memory for continued mapper line
        # indices. Durable reconstruction uses the phase reference, not memory.
        raw=self.pilot.output/'transitions.ndjson'
        chunk=raw.read_bytes() if raw.exists() else b''
        self.pilot.prior_raw_bytes+=chunk
        self.pilot.committed_raw_duplicate=hashlib.sha256(chunk).hexdigest() if raw.exists() else None
        keep={'budget-ledger.json','frozen-config.json','operational-inputs.json',
              'original-seed.bin','heldout-seed.bin','belly-seed.bin','phase-dependencies.json','phase-inputs.json'}
        for entry in manifest['files']:
            # Artifact paths inside the bundle may contain role prefixes; map
            # exact original new-run files via snapshot roles, never basename
            # guessing or deletion of a prior closed-run/archive input.
            name=self.pilot.last_snapshot_roles.get(entry['role'])
            if not name or name in keep:continue
            path=self.pilot.output/name
            if path.parent!=self.pilot.output or not path.is_file() or sha256(path)!=entry['sha256']:
                raise ContractError('Refuse eviction of changed/unowned phase working file')
            path.unlink()
            if path==raw:self.pilot.committed_raw_duplicate=None
        archive=Path(request['archive_path']).resolve()
        owned=(self.pilot.recovery.transport.exchange.directory/'bundles').resolve()
        if archive.parent.parent!=owned or sha256(archive)!=ref['archive_sha256']:
            raise ContractError('Refuse eviction of unowned/unverified phase archive cache')
        shutil.rmtree(archive.parent)
        atomic_json(request_path.with_name(request_path.name.replace('.request.json','.cache-evicted.json')),
            {'reference_sha256':ref['reference_sha256'],'new_local_duplicates_only':True,
             'remote_evidence_retained':True,'cumulative_quota_credit':0})
        return ref

    def resolve(self,restored,manifest):
        from getup_cold_start_engine import restore_coherent
        files={x['role']:Path(restored)/x['path'] for x in manifest['files']}
        if 'phase_dependencies' not in files:raise ContractError('Phase dependency manifest missing')
        journal=json.loads(files['phase_dependencies'].read_text())
        if journal.get('identity')!=self.pilot.identity or journal.get('references')!=self.references:
            raise ContractError('Phase dependency closure differs from committed journal')
        raw_prefix=b'';rows=[];last=None
        for index,ref in enumerate(journal['references']):
            value=dict(ref);digest=value.pop('reference_sha256')
            if (reference_hash(value)!=digest or ref['phase']!=['PARITY','BELLY'][index]
                    or ref['identity']!=self.pilot.identity or ref['status']!='application-verified'
                    or ref['dependencies']!=[x['reference_sha256'] for x in journal['references'][:index]]):
                raise ContractError('Mixed/corrupt phase dependency')
            ledger_interval(ref['ledger_begin'],ref['ledger_end'])
            if last is not None and ref['ledger_begin']!=last:raise ContractError('Phase ledger interval gap')
            last=ref['ledger_end']
            downloaded=Path(self.pilot.recovery.transport.dependency(ref,self.pilot.phase_deadline))
            validated=False
            try:
                m=verify_bundle(downloaded,ref['archive_sha256'],32*1024**2)
                import zipfile
                with zipfile.ZipFile(downloaded) as z:
                    if hashlib.sha256(z.read('manifest.json')).hexdigest()!=ref['manifest_sha256']:
                        raise ContractError('Dependency manifest hash differs')
                if downloaded.stat().st_size!=ref['archive_bytes'] or m['files']!=ref['members']:
                    raise ContractError('Dependency member/size closure differs')
                with tempfile.TemporaryDirectory(prefix='phase-dependency-',dir=self.pilot.recovery.transport.exchange.directory) as tmp:
                    payload=Path(tmp)/'payload';restore_bundle(downloaded,payload,ref['archive_sha256'],32*1024**2)
                    c=m['config'].get('cold_start',m['config'])
                    if (c.get('physical_phase_proof')!=ref['phase'] or c['snapshot_id']!=ref['snapshot_id']
                            or m['run_id']!=ref['run_id'] or ref['run_id']!=self.pilot.identity['runId']):
                        raise ContractError('Dependency phase/generation differs')
                    state=restore_coherent(payload,m,expected_identity=self.pilot.identity,
                        expected_plan=self.pilot.plan,expected_ledger=ref['ledger_end'],
                        learner_factory=self.pilot.learner_factory,builder_factory=self.pilot.builder_factory,
                        controller_factory=self.pilot.recovery.controller_factory,synthetic=self.pilot.synthetic,
                        physical_only=True,raw_prefix=raw_prefix)
                    phase_files={x['role']:payload/x['path'] for x in m['files']}
                    self.verify_native_interval(phase_files,ref['ledger_begin'],ref['ledger_end'])
                    self.verify_phase_evidence(payload,m,ref['phase'])
                    self.verify_native(payload,m)
                    f={x['role']:payload/x['path'] for x in m['files']}
                    raw_prefix+=f['transitions'].read_bytes();rows=state.rows;validated=True
            finally:
                # Only the new expected dependency cache, never an immutable
                # original/closed proof. A failed readback remains for diagnosis.
                if validated and downloaded.parent==(self.pilot.recovery.transport.exchange.directory/'dependency-cache'):
                    downloaded.unlink(missing_ok=True)
        if last is not None:ledger_interval(last,self.pilot.durable_ledger(),allow_uncertain=True)
        phase=manifest['config'].get('cold_start',manifest['config']).get('physical_phase_proof')
        if phase:
            self.verify_native_interval(files,self.start,self.pilot.durable_ledger())
            self.verify_phase_evidence(restored,manifest,phase)
        self.verify_native(restored,manifest)
        return {'raw_prefix':raw_prefix,'prior_rows':rows}
