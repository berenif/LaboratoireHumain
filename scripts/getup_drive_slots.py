"""Two owned Drive slots, not Trash rotation. No connector calls in this module.

Overwrite only older, retain current until verified application restore, charge
bytes before external write. Drive offers no conditional ETag/CAS; a single owner
must validate the expected old bytes. Unknown completion never authorizes retry.
"""
from pathlib import Path
import json,shutil
from experiment_backup import atomic_json,sha256
from getup_cold_start import DriveExchange
from getup_learning_loop import ContractError


def qualification_storage_envelope(*,protected_local_bytes,corrected_shared_payload_bytes,
        corrected_shared_archive_bytes,seed_payload_bytes,seed_archive_bytes,
        prior_cumulative_before_corrected_inputs,slot_archive_bytes,active_working_bytes,
        terminal_payload_bytes=32*1024**2,terminal_archive_bytes=18*1024**2,
        staging_limit_bytes=160*1024**2,shared_already_verified=False):
    """One terminal generation only. Old slot archives are already protected.

    This is sizing/admission arithmetic, not permission to upload or run. The
    staging cap excludes the live original output, whose measured disk footprint
    is nevertheless reported separately. No quota credit for overwritten bytes.
    """
    values=locals().copy()
    if any(type(x) is not int or x<0 for name,x in values.items()
           if name not in ('slot_archive_bytes','shared_already_verified')):
        raise ContractError('Measured nonnegative integer storage inventory required')
    if staging_limit_bytes not in (160*1024**2,192*1024**2) or type(shared_already_verified) is not bool:
        raise ContractError('Explicit bounded local staging scope required')
    if len(slot_archive_bytes)!=2 or any(type(x) is not int or x<=0 for x in slot_archive_bytes):
        raise ContractError('Exactly two measured owned slot archive sizes required')
    if (terminal_payload_bytes>32*1024**2 or terminal_archive_bytes>18*1024**2
            or corrected_shared_archive_bytes+seed_archive_bytes>12*1024**2):
        raise ContractError('Qualification-only input/payload/archive envelope exceeded')
    # Corrected inputs are retained once outside the new exchange directory.
    base=protected_local_bytes+corrected_shared_payload_bytes+corrected_shared_archive_bytes+seed_payload_bytes+seed_archive_bytes
    margin=4*1024**2
    stages={
        'shared_transfer_restore':base+corrected_shared_archive_bytes+corrected_shared_payload_bytes+margin,
        'seed_create_transfer_restore':base+max(2*seed_payload_bytes,seed_archive_bytes+seed_payload_bytes)+margin,
        # Even an incompressible rejected ZIP may temporarily approach payload
        # size during creation, before the strict archive-size check.
        'terminal_create':base+max(2*terminal_payload_bytes,terminal_payload_bytes+terminal_archive_bytes)+margin,
        'older_slot_prewrite_readback':base+terminal_archive_bytes+max(slot_archive_bytes)+margin,
        'terminal_transfer_restore':base+2*terminal_archive_bytes+terminal_payload_bytes+margin}
    cumulative=prior_cumulative_before_corrected_inputs+(0 if shared_already_verified else corrected_shared_archive_bytes)+seed_archive_bytes+terminal_archive_bytes
    if max(stages.values())>staging_limit_bytes or cumulative>160*1024**2:
        raise ContractError('One-generation qualification staging/cumulative envelope exceeds scoped cap')
    return {'stage_peaks_bytes':stages,'staging_peak_bytes':max(stages.values()),
        'active_original_working_bytes':active_working_bytes,
        'total_recovery_artifact_peak_bytes':max(stages.values())+active_working_bytes,
        'active_output_bound_bytes':32*1024**2,
        'total_recovery_artifact_bound_bytes':max(stages.values())+32*1024**2,
        'cumulative_archive_bytes':cumulative,'mutable_generations':1,
        'local_staging_limit_bytes':staging_limit_bytes,'remote_cumulative_limit_bytes':160*1024**2,
        'shared_transfer_reused':shared_already_verified,
        'active_working_copy_in_staging_cap':False,'physics_authorized':False}


def validate_slots(slots):
    if len(slots)!=2 or len({x.get('drive_file_id') for x in slots})!=2:
        raise ContractError('Two distinct explicitly owned older/current slots required')
    for slot in slots:
        if (not slot.get('drive_file_id') or len(slot.get('manifest_sha256',''))!=64
                or len(slot.get('archive_sha256',''))!=64 or slot.get('archive_bytes',0)<=0
                or not slot.get('application_verified') or slot.get('owned') is not True):
            raise ContractError('Slot identity/ownership/application proof incomplete')
    return slots


def classify_slot_readback(intent,slot_id,archive_path):
    """Read-only reconciliation. Even old bytes do NOT grant an automatic retry."""
    if slot_id!=intent['target']['drive_file_id']:raise ContractError('Wrong reconciliation slot')
    digest=sha256(archive_path);size=Path(archive_path).stat().st_size
    if digest==intent['archive_sha256'] and size==intent['archive_bytes']:
        return 'exact-new-awaiting-application-verification'
    if digest==intent['target']['archive_sha256'] and size==intent['target']['archive_bytes']:
        return 'exact-old-no-automatic-retry'
    return 'unrecognized-or-corrupt-stop'


def classify_slot_pointer(directory,intent):
    """Read-only commit-point reconciliation; no new upload or pointer mutation."""
    directory=Path(directory)
    try:current=json.loads((directory/'slot-current.json').read_text())['current']
    except (OSError,ValueError,KeyError):return 'unrecognized-pointer-stop'
    if current==intent['protected_current']:return 'old-current-intact'
    marker=directory/(intent['generation']+'.application-verified.json')
    if (current.get('drive_file_id')==intent['target']['drive_file_id']
            and current.get('manifest_sha256')==intent['manifest_sha256']
            and current.get('archive_sha256')==intent['archive_sha256'] and marker.exists()):
        proof=json.loads(marker.read_text())
        if proof.get('passed') is True and proof.get('archive_sha256')==intent['archive_sha256']:
            return 'new-current-application-verified'
    return 'unrecognized-pointer-stop'


class TwoSlotDriveExchange(DriveExchange):
    def __init__(self,*args,slots,**kwargs):
        super().__init__(*args,**kwargs)
        self.verified=list(validate_slots(slots))
        self.pointer=self.directory/'slot-current.json';self.intent_path=self.directory/'slot-pending.json'
        if self.pointer.exists() or self.intent_path.exists():
            raise ContractError('Existing slot transaction requires explicit reconciliation, never blind reuse')
        self.persist_pointer()

    def persist_pointer(self):
        atomic_json(self.pointer,{'schema':'lh-drive-two-slot-pointer/v1','run_id':self.run_id,
            'older':self.verified[0],'current':self.verified[1],
            'current_guaranteed_during_replacement':True,'revisions_relied_upon':False})

    def snapshot(self,artifacts,config,boundary,kind='generation',wall_reserve=0.):
        if kind!='generation':return super().snapshot(artifacts,config,boundary,kind,wall_reserve)
        if self.intent_path.exists() or self.failed:
            raise ContractError('Unresolved slot write; no retry or new generation')
        # The pre-write old-byte download is also bounded staging, even if the
        # pending terminal generation is tiny. It is removed before new download.
        old_staging=max(x['archive_bytes'] for x in self.verified)
        qualification=config.get('plan',{}).get('qualification_only') is True
        self._unpublished_slot_request=True
        if qualification:
            if getattr(self,'qualification_generation_requested',False):
                raise ContractError('Exactly one qualification terminal generation; no retry')
            self.qualification_generation_requested=True
            config=dict(config,slot_prewrite_download_bytes=old_staging)
            try:request_path=super().snapshot(artifacts,config,boundary,kind,wall_reserve)
            finally:self._unpublished_slot_request=False
        else:
            self.external_local_bytes+=old_staging
            try:request_path=super().snapshot(artifacts,config,boundary,kind,wall_reserve)
            finally:
                self.external_local_bytes-=old_staging
                self._unpublished_slot_request=False
        draft_path=request_path
        request_path=draft_path.with_name(draft_path.name.replace('.slot-draft.json','.request.json'))
        request=json.loads(draft_path.read_text());target,current=self.verified
        intent={'schema':'lh-drive-slot-intent/v1','run_id':self.run_id,
            'generation':request_path.name.removesuffix('.request.json'),
            'archive_sha256':request['archive_sha256'],'manifest_sha256':request['manifest_sha256'],
            'archive_bytes':request['archive_bytes'],'target':target,'protected_current':current,
            'cumulative_before':self.remote_uploaded_bytes,
            'cumulative_after':self.remote_uploaded_bytes+request['archive_bytes'],
            'status':'reserved-possibly-submitted','automatic_retry_authorized':False,
            'no_server_side_cas':True,'deadline_unix':request['deadline_unix']}
        if intent['cumulative_after']>160*1024**2:raise ContractError('Slot replacement cumulative cap')
        # Persist before surfacing request to transfer owner. Never refund this
        # exposure even when interruption makes write completion unknown.
        atomic_json(self.intent_path,intent);self.remote_uploaded_bytes=intent['cumulative_after']
        request.update(operation='replace-owned-slot',target_drive_file_id=target['drive_file_id'],
            protected_current_drive_file_id=current['drive_file_id'],
            expected_target_archive_sha256=target['archive_sha256'],
            expected_target_manifest_sha256=target['manifest_sha256'],
            generation_nonce=intent['generation'],slot_intent_path=str(self.intent_path),
            slot_intent_sha256=sha256(self.intent_path),remote_cumulative_archive_bytes_reserved=self.remote_uploaded_bytes,
            no_server_side_cas=True,prewrite_download_staging_bytes=old_staging,
            retention='two owned slots; overwrite older only; current pointer swaps AFTER exact application restore; no Trash/revisions')
        atomic_json(request_path,request);draft_path.unlink();return request_path

    def verify_receipt(self,request_path,receipt_path,restore_validator):
        request=json.loads(Path(request_path).read_text())
        if request.get('kind')!='generation':return super().verify_receipt(request_path,receipt_path,restore_validator)
        try:
            intent=json.loads(self.intent_path.read_text());receipt=json.loads(Path(receipt_path).read_text())
            if (sha256(self.intent_path)!=request.get('slot_intent_sha256')
                    or receipt.get('drive_file_id')!=request.get('target_drive_file_id')
                    or receipt.get('generation_nonce')!=request.get('generation_nonce')
                    or receipt.get('prewrite_target_archive_sha256')!=intent['target']['archive_sha256']
                    or receipt.get('protected_current_untouched') is not True
                    or receipt.get('write_count')!=1 or receipt.get('operation')!='replace-owned-slot'
                    or self.remote_uploaded_bytes!=intent['cumulative_after']):
                raise ContractError('Slot replacement receipt/nonce/expected-old/current binding differs')
            return super().verify_receipt(request_path,receipt_path,restore_validator)
        except BaseException:
            self.failed=True
            # Leave exact original durable intent/hash intact for reconciliation.
            pointer_state=classify_slot_pointer(self.directory,json.loads(self.intent_path.read_text()))
            atomic_json(self.directory/'slot-stop.json',{'intent_sha256':sha256(self.intent_path),
                'status':'failed-or-uncertain-no-retry','charged_cumulative_bytes':self.remote_uploaded_bytes,
                'pointer_state':pointer_state,'current_pointer_unchanged':pointer_state=='old-current-intact',
                'automatic_retry_authorized':False})
            raise

    def commit_verified(self,request,receipt,request_path,verified_path):
        if request['kind']!='generation':return super().commit_verified(request,receipt,request_path,verified_path)
        old,current=self.verified
        new={'manifest_sha256':request['manifest_sha256'],'archive_sha256':request['archive_sha256'],
            'drive_file_id':receipt['drive_file_id'],'payload_bytes':request['payload_bytes'],
            'archive_path':request['archive_path'],'archive_bytes':request['archive_bytes'],
            'generation_nonce':request['generation_nonce'],'application_verified':True,'owned':True}
        # Cumulative bytes were precharged; overwriting is NOT a quota credit.
        # Live payload slot replaced; ALL uploaded archive bytes remain charged,
        # including unknown revisions. No cumulative quota credit is granted.
        self.verified=[current,new];self.persist_pointer()
        completed=self.directory/(request['generation_nonce']+'.slot-committed.json')
        atomic_json(completed,{'intent_sha256':sha256(self.intent_path),
            'current_manifest_sha256':new['manifest_sha256'],'cumulative_uploaded_bytes':self.remote_uploaded_bytes,
            'application_verified':True,'atomic_pointer_committed':True})
        old_archive=Path(old['archive_path']).resolve()
        if old_archive.parent.parent==(self.directory/'bundles').resolve():shutil.rmtree(old_archive.parent)
        self.intent_path.unlink()
