#!/usr/bin/env python3
"""Nonphysics semantic gates and actual two-slot archive/receipt fault paths."""
import copy,json,subprocess,tempfile,time,unittest
from unittest import mock
from pathlib import Path
from experiment_backup import sha256,atomic_json,verify_bundle
from getup_learning_loop import ContractError
from getup_cold_start import artifact
from getup_drive_slots import TwoSlotDriveExchange,classify_slot_readback,classify_slot_pointer
from getup_worker_build import CONTRACT,ROOT,check_capabilities

class CorrectiveReadiness(unittest.TestCase):
    def test_effective_semantics_reject_roll_missing_standing_or_unknown(self):
        good={'schema':'lh-getup-compiled-capabilities/v1','effectiveSimFeatures':CONTRACT['effective_sim_features'],
              'physicsIntegrations':0,'constructsRuntime':False}
        check_capabilities(good)
        for features in [good['effectiveSimFeatures']+['getup-physical-roll-experiment'],
            [x for x in good['effectiveSimFeatures'] if x!='standing-dynamics-experiment'],
            good['effectiveSimFeatures']+['getup-terminal-bumpless-experiment']]:
            with self.assertRaises(ContractError):check_capabilities(dict(good,effectiveSimFeatures=features))
        script="""import assert from 'node:assert/strict';
import {checkColdWorkerCapabilities as check} from './scripts/getup-cold-worker-contract.mjs';
const x=JSON.parse(process.argv[1]);check(x);
assert.throws(()=>check({...x,effectiveSimFeatures:[...x.effectiveSimFeatures,'getup-physical-roll-experiment']}));
assert.throws(()=>check({...x,effectiveSimFeatures:x.effectiveSimFeatures.filter(v=>v!=='standing-dynamics-experiment')}));"""
        subprocess.run(['node','--input-type=module','-e',script,json.dumps(good)],cwd=ROOT,check=True)

    def test_zero_step_lookalikes_never_enter_mapper_or_replay(self):
        from types import SimpleNamespace
        from getup_cold_start_engine import ColdPilot
        import hashlib
        source={'native':4560,'impactEpoch':560,'sourceEpoch':None,'preview':None}
        result={'native':4560,'state':dict(source),'terminal':True,'success':False,
            'failure':{'check':'getup-policy'},'reward':-100.,
            'nativeTraceSha256':hashlib.sha256(b'[]').hexdigest()}
        for defect in ['explicit-refusal','nonterminal','epoch-change','source-epoch','preview','action','pending','trace']:
            with self.subTest(defect=defect):
                pilot=ColdPilot.__new__(ColdPilot)
                pilot.state=copy.deepcopy(source);pilot.cursor=SimpleNamespace(active=True,episode_id='refused')
                pilot.mapper=SimpleNamespace(behavior='baseline-deterministic-zero',push=mock.Mock(),
                    builder=SimpleNamespace(pending=('pending',) if defect=='pending' else None),
                    pending_origin=None,pending_line=None)
                observed=copy.deepcopy(result);action=None
                if defect=='nonterminal':observed['terminal']=False
                if defect=='epoch-change':observed['state']['impactEpoch']=999
                if defect=='source-epoch':observed['state']['sourceEpoch']=123
                if defect=='preview':observed['state']['preview']={'features':[]}
                if defect=='action':action=[0.]*21
                if defect=='trace':observed['nativeTraceSha256']='f'*64
                pilot.call=mock.Mock(return_value=observed)
                expected='Physical integrity failure' if defect=='explicit-refusal' else 'Malformed zero-step refusal'
                with self.assertRaisesRegex(ContractError,expected):pilot.step(action,'bootstrap')
                pilot.mapper.push.assert_not_called();pilot.call.assert_called_once()
                self.assertIs(pilot.state,observed['state'])

    def test_old_manifest_cannot_be_a_corrected_build_receipt(self):
        from getup_worker_build import validate_build_receipt
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'old-source.json'
            atomic_json(path,{'worker_features':['getup-physical-roll-experiment'],
                'commit':'a'*40,'dirty':False})
            with self.assertRaisesRegex(ContractError,'semantic worker recipe'):
                validate_build_receipt(path,Path(temp)/'worker',Path(temp)/'profile')

    def test_failed_immutable_upload_exposure_remains_precharged(self):
        from getup_cold_start import DriveExchange
        for kind in ('shared','seeds'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);payload=root/'input';payload.write_text('synthetic immutable')
                x=DriveExchange(root/'recovery','synthetic',{'commit':'a'*40,'dirty':False,'description':'software-only'},
                    time.time()+3600,synthetic=True)
                x.remote_uploaded_bytes=22056886
                if kind=='seeds':x.shared_sha256='b'*64
                path=x.snapshot([artifact('diagnostic',payload,'test/v1')],
                    {'qualification_immutable':True},'synthetic-input',kind)
                request=json.loads(path.read_text());charged=22056886+request['archive_bytes']
                self.assertEqual(x.remote_uploaded_bytes,charged)
                self.assertTrue(request['cumulative_precharged'])
                receipt=root/'bad-receipt.json';atomic_json(receipt,{})
                with self.assertRaises(ContractError):x.verify_receipt(path,receipt,lambda *_:None)
                self.assertEqual(x.remote_uploaded_bytes,charged)
                self.assertIn(kind,x.pending_immutable)
                with self.assertRaises(ContractError):
                    x.snapshot([artifact('diagnostic',payload,'test/v1')],{},'retry',kind)

    def setup_exchange(self,root):
        slots=[]
        for i in range(2):
            p=root/f'old-{i}';p.write_bytes(b'previous-'+str(i).encode())
            slots.append({'drive_file_id':f'synthetic-owned-{i}','archive_path':str(p),'archive_sha256':sha256(p),
                'archive_bytes':p.stat().st_size,'payload_bytes':p.stat().st_size,'manifest_sha256':str(i+1)*64,
                'application_verified':True,'owned':True})
        exchange=TwoSlotDriveExchange(root/'recovery','synthetic-unit',{'commit':'a'*40,'dirty':False,'description':'software-only'},
            time.time()+3600,'b'*64,synthetic=True,slots=slots)
        exchange.seed_sha256='c'*64;exchange.remote_uploaded_bytes=22056886
        return exchange

    def request(self,exchange,root):
        payload=root/'payload';payload.write_text('synthetic diagnostic')
        path=exchange.snapshot([artifact('diagnostic',payload,'test/v1')],{'resumeAuthorized':False},'terminal-stop')
        return path,json.loads(path.read_text())

    def receipt(self,root,path,q,*,corrupt=False):
        downloaded=root/'download.zip';downloaded.write_bytes(Path(q['archive_path']).read_bytes())
        if corrupt:downloaded.write_bytes(b'corrupt')
        receipt=root/'receipt.json';atomic_json(receipt,{k:q[k] for k in ('run_id','kind','archive_sha256','archive_bytes','manifest_sha256')}|
            {'schema':'lh-private-drive-receipt/v1','synthetic':True,'owner_only':True,'shared':False,
            'download_completed':True,'downloaded_path':str(downloaded),'drive_file_id':q['target_drive_file_id'],
            'generation_nonce':q['generation_nonce'],'prewrite_target_archive_sha256':q['expected_target_archive_sha256'],
            'protected_current_untouched':True,'operation':'replace-owned-slot','write_count':1})
        return receipt

    def test_two_slot_success_swaps_only_after_application_and_rotates_ids(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);x=self.setup_exchange(root);targets=[];total=22056886
            for _ in range(3):
                current=json.loads(x.pointer.read_text())['current'];p,q=self.request(x,root);targets.append(q['target_drive_file_id'])
                total+=q['archive_bytes'];self.assertEqual(x.remote_uploaded_bytes,total)
                self.assertNotEqual(q['target_drive_file_id'],current['drive_file_id'])
                self.assertEqual(json.loads(x.pointer.read_text())['current'],current)
                def validate(restored,manifest):
                    self.assertEqual(json.loads(x.pointer.read_text())['current'],current)
                    self.assertFalse(manifest['full_resume_ready'])
                    return {'passed':True,'physics_integrations':0}
                x.verify_receipt(p,self.receipt(root,p,q),validate)
                self.assertEqual(x.remote_uploaded_bytes,total)
                pointer=json.loads(x.pointer.read_text());self.assertEqual(pointer['current']['manifest_sha256'],q['manifest_sha256'])
                self.assertEqual(pointer['older'],current);self.assertFalse(x.intent_path.exists())
            self.assertEqual(targets,['synthetic-owned-0','synthetic-owned-1','synthetic-owned-0'])
            self.assertFalse(list(x.directory.glob('*.prune-request.json')))
            self.assertEqual(len(list((x.directory/'bundles').iterdir())),2)

    def test_interruption_corrupt_readback_and_restore_failure_never_refund_or_retry(self):
        for failure in ['interrupted','corrupt','application']:
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as temp:
                root=Path(temp);x=self.setup_exchange(root);pointer=x.pointer.read_bytes();p,q=self.request(x,root)
                intent=json.loads(x.intent_path.read_text());charged=x.remote_uploaded_bytes
                self.assertEqual(classify_slot_readback(intent,q['target_drive_file_id'],intent['target']['archive_path']),
                    'exact-old-no-automatic-retry')
                self.assertEqual(classify_slot_readback(intent,q['target_drive_file_id'],q['archive_path']),
                    'exact-new-awaiting-application-verification')
                if failure!='interrupted':
                    receipt=self.receipt(root,p,q,corrupt=failure=='corrupt')
                    def validate(*args):raise ContractError('scripted independent application restore failure')
                    with self.assertRaises(Exception):x.verify_receipt(p,receipt,validate)
                self.assertEqual(x.pointer.read_bytes(),pointer);self.assertEqual(x.remote_uploaded_bytes,charged)
                with self.assertRaises(ContractError):self.request(x,root)
                self.assertTrue(x.intent_path.exists())
                self.assertFalse(list(x.directory.glob('*.application-verified.json')))

    def test_interruption_at_application_and_pointer_commit_boundaries(self):
        import getup_drive_slots as module
        for after_pointer in [False,True]:
            with self.subTest(after_pointer=after_pointer),tempfile.TemporaryDirectory() as temp:
                root=Path(temp);x=self.setup_exchange(root);p,q=self.request(x,root)
                receipt=self.receipt(root,p,q);intent=json.loads(x.intent_path.read_text())
                original=module.atomic_json
                def interrupted(path,value):
                    if (after_pointer and str(path).endswith('.slot-committed.json')) or (
                            not after_pointer and Path(path).name=='slot-current.json'):
                        raise OSError('scripted disk interruption')
                    return original(path,value)
                with mock.patch.object(module,'atomic_json',side_effect=interrupted):
                    with self.assertRaises(OSError):
                        x.verify_receipt(p,receipt,lambda *_:{'passed':True,'physics_integrations':0})
                expected='new-current-application-verified' if after_pointer else 'old-current-intact'
                self.assertEqual(classify_slot_pointer(x.directory,intent),expected)
                stop=json.loads((x.directory/'slot-stop.json').read_text())
                self.assertEqual(stop['current_pointer_unchanged'],not after_pointer)
                with self.assertRaises(ContractError):self.request(x,root)
                self.assertEqual(x.remote_uploaded_bytes,intent['cumulative_after'])

    def test_whole_cadence_reserves_growth_of_both_slots(self):
        from getup_cold_start_engine import DrivePhaseTransport,PhasePlan
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);x=self.setup_exchange(root)
            x.shared_bytes=112*1024**2
            usage=DrivePhaseTransport(x,None,None).storage_usage()
            self.assertEqual(usage['retained_payload_bytes'],(112+34)*1024**2)
            p=PhasePlan(1,3601,2101,3301)
            with self.assertRaisesRegex(ContractError,'cadence'):
                p.admit_sequence(1,{'nativeIntegrations':0},**usage)

    def test_storage_envelope_charges_overwrites_and_corrected_immutables(self):
        self.assertEqual(22056886+7*20*1024**2-160*1024**2,1085366)
        reserved=22056886+12*1024**2+7*18*1024**2
        self.assertEqual(reserved,166760374);self.assertLess(reserved,160*1024**2)

    def test_one_generation_qualification_storage_keeps_live_disk_visible(self):
        from getup_drive_slots import qualification_storage_envelope
        args=dict(protected_local_bytes=57041539,corrected_shared_payload_bytes=16218147,
            corrected_shared_archive_bytes=10620053,seed_payload_bytes=2*1024**2,
            seed_archive_bytes=1962859,prior_cumulative_before_corrected_inputs=22056886,
            slot_archive_bytes=[5198266,5889728],active_working_bytes=12*1024**2)
        result=qualification_storage_envelope(**args)
        self.assertEqual(result['mutable_generations'],1)
        self.assertLessEqual(result['staging_peak_bytes'],160*1024**2)
        self.assertEqual(result['total_recovery_artifact_peak_bytes'],
            result['staging_peak_bytes']+args['active_working_bytes'])
        self.assertFalse(result['physics_authorized'])
        with self.assertRaisesRegex(ContractError,'staging'):
            qualification_storage_envelope(**dict(args,protected_local_bytes=args['protected_local_bytes']+8*1024**2))
        with self.assertRaisesRegex(ContractError,'envelope'):
            qualification_storage_envelope(**dict(args,terminal_payload_bytes=33*1024**2))

    def test_scoped_local_amendment_never_extends_remote_output_or_cadence(self):
        from getup_qualification import qualification_staging_scope
        from getup_drive_slots import qualification_storage_envelope
        admission={'run_id':'future-software-only','storage':dict(staging_bytes=192*1024**2,
            staging_scope_run_id='future-software-only',staging_amendment_reference='explicit root scoped amendment',
            active_output_bytes=32*1024**2,archive_bytes=18*1024**2,
            cumulative_archive_bytes=160*1024**2,mutable_uploads=1)}
        self.assertEqual(qualification_staging_scope(admission),192*1024**2)
        for key,value in [('staging_bytes',193*1024**2),('staging_scope_run_id','other'),
            ('staging_amendment_reference',''),('active_output_bytes',33*1024**2),
            ('cumulative_archive_bytes',192*1024**2),('archive_bytes',19*1024**2),('mutable_uploads',2)]:
            changed=copy.deepcopy(admission);changed['storage'][key]=value
            with self.assertRaises(ContractError):qualification_staging_scope(changed)
        args=dict(protected_local_bytes=68184679,corrected_shared_payload_bytes=16218147,
            corrected_shared_archive_bytes=10620053,seed_payload_bytes=2097152,
            seed_archive_bytes=1962859,prior_cumulative_before_corrected_inputs=37690936,
            slot_archive_bytes=[5889728,5013997],active_working_bytes=32*1024**2,
            shared_already_verified=True)
        with self.assertRaises(ContractError):qualification_storage_envelope(**args)
        result=qualification_storage_envelope(**args,staging_limit_bytes=192*1024**2)
        self.assertEqual(result['staging_peak_bytes'],174580362)
        self.assertEqual(result['total_recovery_artifact_peak_bytes'],208134794)
        self.assertEqual(result['cumulative_archive_bytes'],58528163)
        with self.assertRaises(ContractError):
            qualification_storage_envelope(**dict(args,prior_cumulative_before_corrected_inputs=159*1024**2),
                staging_limit_bytes=192*1024**2)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);x=self.setup_exchange(root);x.external_local_bytes=170*1024**2
            with self.assertRaisesRegex(ContractError,'Local archive'):self.request(x,root)
            x.local_staging_limit_bytes=192*1024**2
            path,q=self.request(x,root)
            self.assertEqual(q['max_retained_payload_bytes'],160*1024**2)

    def test_shared_reuse_binds_exact_private_proofs_without_new_transfer(self):
        from getup_qualification import validate_shared_reuse
        import hashlib,zipfile
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);now=time.time()
            manifest={'required_roles':['software-fixture'],'files':[{'bytes':3}]}
            raw=json.dumps(manifest).encode();archive=root/'fixture.zip'
            with zipfile.ZipFile(archive,'w') as z:z.writestr('manifest.json',raw)
            item={'path':str(archive),'bytes':archive.stat().st_size,'sha256':sha256(archive)}
            request={'run_id':'prior-unit-fixture','kind':'shared','synthetic':False,
                'archive_sha256':item['sha256'],'archive_bytes':item['bytes'],'payload_bytes':3,
                'manifest_sha256':hashlib.sha256(raw).hexdigest(),'required_roles':['software-fixture']}
            receipt=dict(request,schema='lh-private-drive-receipt/v1',owner_only=True,shared=False,
                download_completed=True,drive_file_id='not-a-real-remote-id')
            application={'passed':True,'physics_integrations':0,'archive_sha256':item['sha256']}
            proof={}
            for role,value in [('request',request),('receipt',receipt),('application',application)]:
                path=root/(role+'.json');atomic_json(path,value);proof[role]={'path':str(path),'sha256':sha256(path)}
            private={'schema':'lh-reused-shared-private-identity/v1','drive_file_id':receipt['drive_file_id'],
                'archive_sha256':item['sha256'],'archive_bytes':item['bytes'],'owner_only':True,'shared':False,
                'new_upload':False,'new_download':True,'download_completed':True,
                'downloaded_archive_sha256':item['sha256'],'downloaded_archive_bytes':item['bytes'],
                'readback_verified_at_unix':now,'immutable_identity_unchanged':False,
                'previous_application_sha256':proof['application']['sha256'],'verified_at_unix':now,
                'current_metadata':{'id':receipt['drive_file_id'],'size':str(item['bytes'])}}
            path=root/'private.json'
            def bind(value):
                atomic_json(path,value);proof['private_identity']={'path':str(path),'sha256':sha256(path)}
            bind(private)
            result=validate_shared_reuse(proof,item,manifest,now)
            self.assertFalse(result['private_identity']['new_upload'])
            for key,value in [('shared',True),('owner_only',False),('download_completed',False),
                ('archive_sha256','f'*64),('new_upload',True),('verified_at_unix',now-1801),
                ('readback_verified_at_unix',now-1801),('downloaded_archive_sha256','d'*64),
                ('drive_file_id','another-id'),('previous_application_sha256','e'*64)]:
                bind(dict(private,**{key:value}))
                with self.assertRaises(ContractError):validate_shared_reuse(proof,item,manifest,now)
            bind(private)
            with self.assertRaises(ContractError):
                validate_shared_reuse(proof,item,dict(manifest,required_roles=['wrong']),now)
            archive.write_bytes(b'corrupt')
            with self.assertRaises(Exception):validate_shared_reuse(proof,item,manifest,now)

    def test_owner_inventory_rejects_refund_stale_slots_and_unresolved_replacement(self):
        from getup_qualification import validate_owner_inventory
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);x=self.setup_exchange(root);prior=copy.deepcopy(x.verified)
            ledger={'cumulative_uploaded_bytes':37690936,'live_bytes':32492670,'trashed_bytes':0,
                'older_slot':prior[0],'current_slot':prior[1]}
            ledger_path=root/'ledger.json';inventory_path=root/'owner.json'
            inventory={'schema':'lh-drive-owner-qualification-inventory/v1','planned_run_id':'future',
                'unknown_transfers':False,'ledger_path':str(ledger_path),'prior_generations':prior,
                'cumulative_uploaded_bytes':37690936,'live_archive_bytes':32492670,'trashed_archive_bytes':0,
                'all_project_retained_payload_bytes':100,'synthetic_test_archive_bytes':394}
            admission={'run_id':'future','storage':{'cumulative_uploaded_bytes':37690936,
                'live_archive_bytes':32492670,'trashed_archive_bytes':0,'remote_payload_bytes':494}}
            def bind():
                atomic_json(ledger_path,ledger);inventory['ledger_sha256']=sha256(ledger_path)
                atomic_json(inventory_path,inventory)
                admission['storage']['owner_inventory']={'path':str(inventory_path),'sha256':sha256(inventory_path)}
            bind();validate_owner_inventory(admission,prior)
            for key,value in [('cumulative_uploaded_bytes',22056886),('live_archive_bytes',37690936),
                ('trashed_archive_bytes',1),('remote_payload_bytes',493)]:
                bad=copy.deepcopy(admission);bad['storage'][key]=value
                with self.assertRaises(ContractError):validate_owner_inventory(bad,prior)
            with self.assertRaises(ContractError):validate_owner_inventory(admission,list(reversed(prior)))
            inventory['unknown_transfers']=True;bind()
            with self.assertRaises(ContractError):validate_owner_inventory(admission,prior)
            inventory['unknown_transfers']=False;ledger['pending_replacement']={'status':'uncertain'};bind()
            with self.assertRaises(ContractError):validate_owner_inventory(admission,prior)

    def test_production_reuse_restore_adapter_has_no_upload_or_double_charge(self):
        from getup_qualification import restore_verified_shared
        from experiment_backup import create_snapshot
        from getup_cold_start import DriveExchange
        import zipfile,hashlib
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);payload=root/'payload';payload.write_bytes(b'explicit synthetic input')
            archive=create_snapshot(root/'archive',dict(run_id='synthetic-test',synthetic=True,
                source={'commit':'a'*40,'dirty':False,'description':'software fixture only'},
                provenance='software-test',config={},seed=1,
                artifacts=[artifact('fixture',payload,'synthetic-test/v1')],required_roles=['fixture'],
                excluded_dependencies=[],max_payload_bytes=1024,application_resume_eligible=False))
            item={'path':str(archive),'sha256':sha256(archive),'bytes':archive.stat().st_size}
            request={'manifest_sha256':hashlib.sha256(zipfile.ZipFile(archive).read('manifest.json')).hexdigest(),
                'payload_bytes':payload.stat().st_size}
            x=DriveExchange(root/'exchange','synthetic-next',{'commit':'b'*40},time.time()+3600,synthetic=True)
            x.remote_uploaded_bytes=37690936;x.retired_payload_bytes=1000
            def validate(restored,manifest):
                self.assertTrue(manifest['synthetic'])
                self.assertEqual((restored/manifest['files'][0]['path']).read_bytes(),payload.read_bytes())
                return {'passed':True,'physics_integrations':0,'synthetic':True}
            with mock.patch.object(x,'snapshot',side_effect=AssertionError('NO NEW REQUEST')):
                result=restore_verified_shared(x,item,request,validate,time.time()+60)
            self.assertTrue(result['synthetic']);self.assertEqual(x.remote_uploaded_bytes,37690936)
            self.assertEqual(x.retired_payload_bytes,1000-payload.stat().st_size)
            self.assertEqual(x.shared_sha256,request['manifest_sha256'])
            self.assertFalse(list(x.directory.glob('*.request.json')))
            self.assertFalse(list(x.directory.glob('shared-reuse-*')))
            with self.assertRaises(ContractError):
                restore_verified_shared(x,item,request,validate,time.time()-1)

if __name__=='__main__':unittest.main()
