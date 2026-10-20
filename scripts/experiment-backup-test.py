#!/usr/bin/env python3
"""Synthetic, stdlib-only backup contracts. No network, physics or training."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch
import zipfile

import experiment_backup as backup


class BackupTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.inputs = self.root / 'inputs'; self.inputs.mkdir()
        self.spec = {'run_id': 'synthetic-unit-only', 'synthetic': True,
            'source': {'commit': '0' * 40, 'dirty': True, 'description': 'Synthetic test source'},
            'config': {'physics': False}, 'seed': 17,
            'provenance': 'Synthetic fixture only, no physical experiment',
            'required_roles': ['replay', 'checkpoint', 'input_manifest'],
            'excluded_dependencies': [], 'artifacts': []}
        for role in self.spec['required_roles']:
            path = self.inputs / (role + '.json')
            path.write_text(json.dumps({'synthetic': True, 'role': role}) + '\n')
            self.spec['artifacts'].append({'role': role, 'path': str(path),
                'schema': 'synthetic-test/v1', 'provenance': 'Synthetic test only',
                'retention_class': 'user_generated'})
        (self.inputs / 'input_manifest.json').write_text(json.dumps({'synthetic': True,
            'files': {role: {'sha256': backup.sha256(self.inputs / (role + '.json'))}
                      for role in ('replay', 'checkpoint')}}) + '\n')

    def tearDown(self):
        self.temporary.cleanup()

    def snapshot(self, spec=None):
        return backup.create_snapshot(self.root / 'generations', spec or self.spec)

    def rewrite(self, bundle, mutate):
        with zipfile.ZipFile(bundle) as archive:
            members = {name: archive.read(name) for name in archive.namelist()}
        mutate(members)
        changed = self.root / ('changed-' + str(time.time_ns()) + '.zip')
        with zipfile.ZipFile(changed, 'w') as archive:
            for name, value in members.items():
                archive.writestr(name, value)
        return changed

    def test_two_immutable_generations_and_exact_restore(self):
        first = self.snapshot(); first_hash = backup.sha256(first)
        second_spec = copy.deepcopy(self.spec)
        second_spec['previous_manifest_sha256'] = backup.sha256(first.parent / 'manifest.json')
        second = self.snapshot(second_spec)
        self.assertNotEqual(first.parent, second.parent)
        self.assertEqual(backup.sha256(first), first_hash)
        manifest = backup.restore_bundle(first, self.root / 'restore', first_hash)
        self.assertTrue(manifest['synthetic']); self.assertFalse(manifest['full_resume_ready'])
        self.assertEqual(backup.verify_bundle(second)['previous_manifest_sha256'], second_spec['previous_manifest_sha256'])
        for item in manifest['files']:
            self.assertEqual(backup.sha256(self.root / 'restore' / item['path']), item['sha256'])

    def test_restore_will_not_overwrite(self):
        destination = self.root / 'existing'; destination.mkdir()
        with self.assertRaisesRegex(backup.RecoveryError, 'must not exist'):
            backup.restore_bundle(self.snapshot(), destination)

    def test_wrong_archive_hash_rejected_before_extraction(self):
        with self.assertRaisesRegex(backup.RecoveryError, 'Archive SHA256'):
            backup.restore_bundle(self.snapshot(), self.root / 'restore', 'f' * 64)
        self.assertFalse((self.root / 'restore').exists())

    def test_corrupt_payload_rejected(self):
        bundle = self.rewrite(self.snapshot(), lambda files: files.update({'payload/replay/replay.json': b'wrong'}))
        with self.assertRaisesRegex(backup.RecoveryError, 'hash/size mismatch'):
            backup.restore_bundle(bundle, self.root / 'restore')
        self.assertFalse((self.root / 'restore').exists())

    def test_missing_unlisted_and_traversal_members_rejected(self):
        original = self.snapshot()
        for member in ('../escape', '/absolute', 'payload/extra.json', 'a\\b'):
            changed = self.rewrite(original, lambda files: files.update({member: b'bad'}))
            with self.subTest(member=member), self.assertRaises(backup.RecoveryError):
                backup.verify_bundle(changed)
        changed = self.rewrite(original, lambda files: files.pop('payload/replay/replay.json'))
        with self.assertRaisesRegex(backup.RecoveryError, 'members differ'):
            backup.verify_bundle(changed)

    def test_duplicate_member_rejected(self):
        bundle = self.snapshot()
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            with zipfile.ZipFile(bundle, 'a') as archive:
                archive.writestr('manifest.json', b'{}')
        with self.assertRaisesRegex(backup.RecoveryError, 'Duplicate archive member'):
            backup.verify_bundle(bundle)

    def test_expansion_bound_rejected(self):
        with self.assertRaisesRegex(backup.RecoveryError, 'expansion'):
            backup.verify_bundle(self.snapshot(), max_payload_bytes=-4 * 1024**2)

    def test_snapshot_rejects_secrets_symlinks_and_upstream_assets(self):
        secret = self.inputs / '.env'; secret.write_text('SECRET=not-real')
        link = self.inputs / 'link'; link.symlink_to(self.inputs / 'replay.json')
        for path, retention in ((secret, 'user_generated'), (link, 'user_generated'),
                                (self.inputs / 'replay.json', 'third_party')):
            spec = copy.deepcopy(self.spec); spec['artifacts'][0].update(path=str(path), retention_class=retention)
            with self.subTest(path=path, retention=retention), self.assertRaises(backup.RecoveryError):
                self.snapshot(spec)
        self.assertFalse(list((self.root / 'generations').glob('.staging-*')))

    def test_snapshot_rejects_changed_source_and_cleans_staging(self):
        original = backup.shutil.copyfileobj
        def change_source(incoming, outgoing, length):
            original(incoming, outgoing, length)
            Path(incoming.name).write_text('changed during copy')
        with patch.object(backup.shutil, 'copyfileobj', side_effect=change_source):
            with self.assertRaisesRegex(backup.RecoveryError, 'changed while snapshotting'):
                self.snapshot()
        self.assertFalse(list((self.root / 'generations').glob('.staging-*')))

    def test_required_roles_source_config_seed_and_provenance(self):
        for key in ('source', 'config', 'seed', 'provenance', 'synthetic'):
            spec = copy.deepcopy(self.spec); spec.pop(key)
            with self.subTest(key=key), self.assertRaises(backup.RecoveryError):
                self.snapshot(spec)
        spec = copy.deepcopy(self.spec); spec['required_roles'].append('missing')
        with self.assertRaisesRegex(backup.RecoveryError, 'Missing required'):
            self.snapshot(spec)

    def test_unavailable_upstream_dependency_never_claims_full_resume(self):
        spec = copy.deepcopy(self.spec); spec['synthetic'] = False
        spec['excluded_dependencies'] = [{'role': 'upstream', 'sha256': '1' * 64,
            'reason': 'License unverified', 'reacquisition': 'Exact source unavailable', 'required_for_resume': True}]
        self.assertFalse(backup.verify_bundle(self.snapshot(spec))['full_resume_ready'])

    def test_false_full_resume_claim_rejected(self):
        def mutate(files):
            value = json.loads(files['manifest.json']); value['full_resume_ready'] = True
            files['manifest.json'] = backup.canonical(value)
        with self.assertRaisesRegex(backup.RecoveryError, 'full-resume claim'):
            backup.verify_bundle(self.rewrite(self.snapshot(), mutate))

    def test_local_receipt_cannot_substitute_for_remote_proof(self):
        receipt = self.root / 'receipt.json'
        backup.atomic_json(receipt, {'verified': True, 'local_path': str(self.snapshot())})
        with self.assertRaisesRegex(backup.RecoveryError, 'durable recovery'):
            backup.require_recovery_point(receipt, self.inputs / 'input_manifest.json', ['replay'], None)

    def proof_fixture(self, synthetic=False):
        spec = copy.deepcopy(self.spec); spec['synthetic'] = synthetic
        bundle = self.snapshot(spec); manifest = backup.verify_bundle(bundle)
        proof = {'schema': backup.RECEIPT_SCHEMA, 'store': 'private-chatgpt-library',
            'locator': {'library_file_id': 'synthetic-mocked-library-item'},
            'bundle_sha256': backup.sha256(bundle), 'run_id': spec['run_id'],
            'synthetic': synthetic, 'full_resume_ready': not synthetic, 'files': manifest['files']}
        receipt = self.root / 'receipt.json'; backup.atomic_json(receipt, proof)
        class FakeBackend:
            def __init__(self, fixture): self.fixture = fixture; self.calls = 0
            def prove(self, locator, digest, deadline):
                self.calls += 1
                return self.fixture
        return receipt, FakeBackend(proof)

    def test_preflight_fresh_remote_proof_binds_exact_inputs_and_roles(self):
        receipt, backend = self.proof_fixture()
        result = backup.require_recovery_point(receipt, self.inputs / 'input_manifest.json',
            self.spec['required_roles'], backend, run_id=self.spec['run_id'])
        self.assertEqual(result['run_id'], self.spec['run_id']); self.assertEqual(backend.calls, 1)
        for roles, run_id in ((['absent-role'], None), (['replay'], 'another-run')):
            with self.assertRaises(backup.RecoveryError):
                backup.require_recovery_point(receipt, self.inputs / 'input_manifest.json', roles, backend, run_id=run_id)
        (self.inputs / 'input_manifest.json').write_text('different exact input')
        with self.assertRaisesRegex(backup.RecoveryError, 'exact retained-input'):
            backup.require_recovery_point(receipt, self.inputs / 'input_manifest.json', ['replay'], backend)

    def test_preflight_checks_each_input_artifact_hash_not_just_role_presence(self):
        receipt, backend = self.proof_fixture()
        backend.fixture['files'][0]['sha256'] = 'f' * 64
        with self.assertRaisesRegex(backup.RecoveryError, 'input artifact hash differs'):
            backup.require_recovery_point(receipt, self.inputs / 'input_manifest.json', ['replay'], backend)

    def test_synthetic_remote_proof_cannot_admit_a_real_run(self):
        receipt, backend = self.proof_fixture(synthetic=True)
        with self.assertRaisesRegex(backup.RecoveryError, 'Synthetic or incomplete'):
            backup.require_recovery_point(receipt, self.inputs / 'input_manifest.json', ['replay'], backend)

    def test_upload_failure_leaves_local_generation_without_success_receipt(self):
        class FailedBackend:
            def retain_many(self, paths, deadline): raise backup.RecoveryError('Mocked upload failure')
        guard = backup.RecoveryGuard(self.root / 'generations', self.spec['run_id'], FailedBackend(),
                                    self.spec['source'], synthetic=True)
        with self.assertRaisesRegex(backup.RecoveryError, 'upload failure'):
            guard.checkpoint(self.spec['artifacts'], self.spec['config'], self.spec['seed'],
                self.spec['provenance'], required_roles=self.spec['required_roles'])
        self.assertEqual(len(list((self.root / 'generations').glob('*/*.zip'))), 1)
        self.assertFalse(list((self.root / 'generations').glob('*/recovery-proof.json')))
        self.assertIsNone(guard.previous)

    def test_expired_deadline_blocks_without_starting_any_external_work(self):
        guard = backup.RecoveryGuard(self.root / 'generations', self.spec['run_id'], None, self.spec['source'])
        with self.assertRaisesRegex(backup.RecoveryError, 'deadline'):
            guard.checkpoint(self.spec['artifacts'], {}, 0, 'test', required_roles=['replay'], deadline=time.time()-1)
        self.assertFalse((self.root / 'generations').exists())

    def test_uncertain_write_latches_guard_and_blocks_terminal_retry(self):
        class FailedBackend:
            calls = 0
            def retain_many(self, paths, deadline):
                self.calls += 1
                raise backup.RecoveryError('Uncertain remote write')
        backend = FailedBackend()
        guard = backup.RecoveryGuard(self.root / 'generations', self.spec['run_id'], backend,
                                    self.spec['source'], synthetic=True)
        for message in ('Uncertain', 'inspect Library'):
            with self.assertRaisesRegex(backup.RecoveryError, message):
                guard.checkpoint(self.spec['artifacts'], self.spec['config'], self.spec['seed'],
                    self.spec['provenance'], required_roles=self.spec['required_roles'])
        self.assertEqual(backend.calls, 1)
        self.assertEqual(len(list((self.root / 'generations').glob('*/*.zip'))), 1)

    def test_helper_failure_redacts_signed_urls_and_has_no_write_retry(self):
        failure = subprocess.CompletedProcess([], 1, '', 'secret https://example.invalid/?signature=hidden')
        with patch.object(backup.subprocess, 'run', return_value=failure) as run:
            with self.assertRaises(backup.RecoveryError) as error:
                backup.helper_call(['fake-official-helper'], {})
        self.assertNotIn('signature', str(error.exception)); self.assertEqual(run.call_count, 1)

    def test_missing_runtime_helpers_fails_closed(self):
        with self.assertRaisesRegex(backup.RecoveryError, 'official Library helpers'):
            backup.LibraryBackend(self.root / 'no-helpers', self.root / 'proofs')

    def mocked_library(self):
        helpers = self.root / 'mocked-helpers'; helpers.mkdir()
        for name in ('library_upload.py', 'library_file_transfer.py', 'library_hosted_apps.py'):
            (helpers / name).write_text('# Synthetic placeholder; never executed\n')
        return backup.LibraryBackend(helpers, self.root / 'proofs')

    def test_mocked_remote_materialization_restores_bytes_and_passes_complete_transfer(self):
        # Synthetic integration contract only: no claim of an actual upload.
        bundle = self.snapshot(); downloaded = self.root / 'mocked-download.zip'
        backup.shutil.copyfile(bundle, downloaded)
        transfer = {'workspace_path': str(downloaded), 'library_file_id': 'mocked-library-id',
                    'xattrs': [], 'completion_token': 'mocked-test-only'}
        prepared = {'structuredContent': {'result': {'transfers': [transfer]}}}
        with patch.object(backup, 'helper_call', return_value=prepared), patch.object(
                backup.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, str(downloaded), '')) as run:
            proof = self.mocked_library().prove({'library_file_id': 'mocked-library-id'}, backup.sha256(bundle))
        args = run.call_args.args[0]
        self.assertEqual(args[-2:], ['apply-xattrs', str(downloaded)])
        self.assertEqual(json.loads(run.call_args.kwargs['input']), transfer)
        restored = Path(proof['restored_directory'])
        self.assertNotEqual(restored.parent, bundle.parent)
        self.assertTrue(proof['synthetic']); self.assertFalse(proof['full_resume_ready'])
        for item in proof['files']:
            self.assertEqual(backup.sha256(restored / item['path']), item['sha256'])

    def test_mocked_remote_identity_mismatch_cannot_issue_recovery_proof(self):
        prepared = {'structuredContent': {'transfers': [{'library_file_id': 'wrong-id'}]}}
        with patch.object(backup, 'helper_call', return_value=prepared):
            with self.assertRaisesRegex(backup.RecoveryError, 'identity mismatch'):
                self.mocked_library().prove({'library_file_id': 'expected-id'}, 'a' * 64)
        self.assertFalse(list((self.root / 'proofs').glob('*/recovery-proof.json')))


if __name__ == '__main__':
    unittest.main()
