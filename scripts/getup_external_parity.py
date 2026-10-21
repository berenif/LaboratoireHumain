"""An immutable FOREIGN native proof, never a continuation/new-run ledger.

Pure validation does not construct a learner or runtime. Operational validation
loads only the already-bound checkpoint without stepping; no learner/RNG restore.
"""
import hashlib,json,zipfile
from dataclasses import make_dataclass
from pathlib import Path
from types import SimpleNamespace
from experiment_backup import sha256,verify_bundle
from getup_learning_loop import ContractError
from getup_phase_proofs import PhaseProofs,reference_hash


def bound_document(item):
    path=Path(item['path'])
    if sha256(path)!=item['sha256']:raise ContractError('External prerequisite document changed')
    return json.loads(path.read_text())


def private_proof(proof,*,synthetic=False):
    documents={k:bound_document(proof[k]) for k in ('request','receipt','application')}
    q,r,a=(documents[k] for k in ('request','receipt','application'))
    if (any(bool(x.get('synthetic',False)) is not synthetic for x in (q,r,a))
            or r.get('schema')!='lh-private-drive-receipt/v1' or r.get('success',True) is not True
            or not r.get('drive_file_id') or r.get('owner_only') is not True or r.get('shared') is not False
            or r.get('download_completed') is not True or a.get('passed') is not True
            or a.get('physics_integrations')!=0
            or a.get('archive_sha256')!=q.get('archive_sha256')
            or any(r.get(k)!=q.get(k) for k in ('run_id','kind','archive_sha256','archive_bytes','manifest_sha256'))):
        raise ContractError('External prerequisite is not durably privately application-verified')
    item=proof['archive'];path=Path(item['path'])
    if (sha256(path)!=item['sha256'] or path.stat().st_size!=item['bytes']
            or item['sha256']!=q['archive_sha256'] or item['bytes']!=q['archive_bytes']):
        raise ContractError('External archive identity differs')
    manifest=verify_bundle(path,item['sha256'],18*1024**2)
    with zipfile.ZipFile(path) as z:
        if hashlib.sha256(z.read('manifest.json')).hexdigest()!=q['manifest_sha256']:
            raise ContractError('External manifest identity differs')
    if bool(manifest['synthetic']) is not synthetic:raise ContractError('External mode differs')
    return documents,manifest


def validate_external_parity(prerequisite,shared_manifest,new_run_id,protocol_sha256,*,synthetic=False):
    if prerequisite.get('schema')!='lh-preservation-external-prerequisite/v1':
        raise ContractError('Explicit preservation-only prerequisite required')
    docs,manifest=private_proof(prerequisite['parity'],synthetic=synthetic)
    seeds,seed_manifest=private_proof(prerequisite['seeds'],synthetic=synthetic)
    c=manifest['config'].get('cold_start',manifest['config']);identity=c['identity']
    operation=docs['application'].get('operational',docs['application'])
    if (c.get('physical_phase_proof')!='PARITY' or c.get('learner_restore_claimed') is not False
            or c.get('resumeAuthorized') is not False or identity['runId']==new_run_id
            or identity!=prerequisite['original_identity'] or manifest['source']['commit']!=identity['sourceCommit']
            or operation.get('physical_proof') is not True
            or operation.get('learner_restored') is not False
            or identity['protocolSha256']!=protocol_sha256):
        raise ContractError('External proof semantics/identity or no-learner scope differs')
    path=prerequisite['parity']['archive']['path']
    with zipfile.ZipFile(path) as z:
        files={x['role']:x for x in manifest['files']}
        read=lambda role:json.loads(z.read(files[role]['path']))
        ledger=read('ledger');inputs=read('immutable_input_references');cursor=read('cursor');parity=read('native_parity')
        old=bound_document(prerequisite['original_ledger'])
        if ledger!=old or ledger!=cursor['ledger'] or inputs['identity']!=identity:
            raise ContractError('External original ledger/input identity differs')
        if (ledger['nativeIntegrations']!=4420 or ledger['realIntegrations']!=2380
                or ledger['forecastIntegrations']!=2040 or ledger['completedCalls']!=122
                or ledger['bootstrapDecisions']!=50 or ledger['trainingDecisions']!=50
                or ledger.get('pendingCall') or ledger.get('uncertainCalls') or ledger.get('uncertainIntegrations')
                or cursor['runtime_active'] or cursor['mapper'] is not None or cursor['replay_exists']
                or cursor['automatic_restart_authorized'] is not False
                or parity.get('passed') is not True or parity.get('zeroCorrectionSamples')!=400
                or parity.get('resumedSamples')!=200
                or files['native_parity_runtime']['sha256']!=parity.get('resumeCheckpointSha256')):
            raise ContractError('Exact closed03 native parity boundary required')
        for name,actual in [('shared',shared_manifest),('seeds',seed_manifest)]:
            ref=inputs[name]
            if (ref['members']!=actual['files'] or ref['manifest_sha256']!=identity['sharedManifestSha256' if name=='shared' else 'seedManifestSha256']):
                raise ContractError('Worker/glue/model/actors/profile/triggered seed membership changed')
        if (inputs['seeds']['archive_sha256']!=seeds['request']['archive_sha256']
                or inputs['seeds']['drive_file_id']!=seeds['receipt']['drive_file_id']
                or identity['seedManifestSha256']!=seeds['request']['manifest_sha256']):
            raise ContractError('Verified triggered seed identity differs')
        expected_worker=next(x['sha256'] for x in shared_manifest['files'] if x['role']=='worker_wasm')
        if identity['workerSha256']!=expected_worker:raise ContractError('Corrected worker differs')
        fixtures=read('fixtures')['cases'];members={x['role']:x for x in seed_manifest['files']}
        if set(members)!={'original_seed','belly_seed','heldout_seed'}:raise ContractError('Exact triggered seeds required')
        for case in ('original','belly','heldout'):
            if fixtures[case]['checkpointSha256']!=members[case+'_seed']['sha256']:
                raise ContractError('Prepared seed changed or impact lineage unbound')
    ref={'schema':'lh-external-native-parity/v1','phase':'PARITY','status':'application-verified',
        'identity':identity,'run_id':identity['runId'],'original_plan':c['plan'],'original_ledger':ledger,
        'drive_file_id':docs['receipt']['drive_file_id'],'archive_sha256':docs['request']['archive_sha256'],
        'archive_bytes':docs['request']['archive_bytes'],'manifest_sha256':docs['request']['manifest_sha256'],
        'payload_bytes':sum(x['bytes'] for x in manifest['files']),'members':manifest['files'],
        'historical_source_decisions':50,'new_source_decisions':974,'learner_restore_claimed':False,
        'automatic_restart_authorized':False,
        'retention_application_sha256':prerequisite['parity']['application']['sha256']}
    ref['reference_sha256']=reference_hash(ref)
    return {'reference':ref,'seed_documents':seeds,'seed_manifest':seed_manifest,'manifest':manifest}


def restore_external_parity(restored,manifest,reference,native_validator,*,synthetic=False):
    """Fresh physical-only restoration under ORIGINAL identity/plan/ledger.

    No original cursor/raw/replay/RNG is returned into the new run. Its ledger
    remains separate; learner_factory is a tripwire, not a seeded constructor.
    """
    from getup_cold_start_engine import restore_coherent
    value=dict(reference);digest=value.pop('reference_sha256')
    if reference_hash(value)!=digest:raise ContractError('External reference changed')
    c=manifest['config'].get('cold_start',manifest['config'])
    if (c['identity']!=reference['identity'] or c['plan']!=reference['original_plan']
            or manifest['files']!=reference['members']):raise ContractError('Mixed external generation')
    def forbidden(*args):raise ContractError('Foreign proof must not construct learner/mapper/active runtime')
    plan=make_dataclass('OriginalQualificationPlan',[(k,type(v)) for k,v in c['plan'].items()])(**c['plan'])
    state=restore_coherent(restored,manifest,expected_identity=reference['identity'],expected_plan=plan,
        expected_ledger=reference['original_ledger'],learner_factory=forbidden,builder_factory=forbidden,
        controller_factory=lambda path,bridge,state,ledger:forbidden() if path is not None else {'runtime_active':False},
        synthetic=synthetic,physical_only=True)
    if state.learner is not None or state.rows or state.mapping or state.replay or state.mapper:
        raise ContractError('Foreign native proof cannot import learning state')
    verifier=PhaseProofs.__new__(PhaseProofs)
    verifier.pilot=SimpleNamespace(identity=reference['identity']);verifier.native_validator=native_validator
    files={x['role']:Path(restored)/x['path'] for x in manifest['files']}
    begin=dict(reference['original_ledger'])
    for key in ('nativeIntegrations','realIntegrations','forecastIntegrations','completedCalls','trainingDecisions','bootstrapDecisions'):begin[key]=0
    verifier.verify_native_interval(files,begin,reference['original_ledger'])
    verifier.verify_phase_evidence(restored,manifest,'PARITY');verifier.verify_native(restored,manifest)
    return {'passed':True,'physics_integrations':0,'learner_restored':False,'foreign_ledger_imported':False,
        'reference_sha256':reference['reference_sha256'],'native_samples_compared_historically':400,'restored_samples_compared_historically':200}
