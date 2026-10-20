#!/usr/bin/env python3
"""Private, immutable experiment recovery points. Does not run physics/training.

Only explicitly listed project-source/user-generated artifacts are retained.
Library integration uses current official helpers supplied at runtime, never
credentials or a storage URL committed to this repository.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import time
import uuid
import zipfile

SCHEMA = 'lh-experiment-backup/v1'
RECEIPT_SCHEMA = 'lh-experiment-recovery-proof/v1'
ROLE = re.compile(r'^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,95}$')
SHA = re.compile(r'^[0-9a-f]{64}$')
DEFAULT_MAX_BYTES = 8 * 1024**3


class RecoveryError(RuntimeError):
    pass


def canonical(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def utc_now():
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def fsync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def fsync_tree_directories(path):
    for directory, _, _ in os.walk(path, topdown=False):
        fsync_directory(directory)


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('xb') as stream:
            stream.write(canonical(value)); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def checked_name(value, label):
    if not isinstance(value, str) or not ROLE.fullmatch(value):
        raise RecoveryError(f'Invalid {label}')
    return value


def safe_member(name):
    path = PurePosixPath(name)
    if not name or path.is_absolute() or '..' in path.parts or '\\' in name or ':' in name:
        raise RecoveryError('Unsafe archive member')
    if name != str(path) or any(part in ('', '.') for part in path.parts):
        raise RecoveryError('Noncanonical archive member')
    return path


def validate_source(source):
    if not isinstance(source, dict) or not re.fullmatch(r'[0-9a-f]{40}', source.get('commit', '')):
        raise RecoveryError('Exact source commit is required')
    if type(source.get('dirty')) is not bool or not source.get('description'):
        raise RecoveryError('Source tree state and provenance are required')


def checked_artifact(item):
    role = checked_name(item.get('role'), 'artifact role')
    if item.get('retention_class') not in ('user_generated', 'project_source'):
        raise RecoveryError(f'{role}: upstream/unknown assets must be references, not uploads')
    if not item.get('schema') or not item.get('provenance'):
        raise RecoveryError(f'{role}: artifact schema and provenance are required')
    source = Path(item['path']).absolute()
    if any(part in ('.git', '.aws', '.codex') for part in source.parts) or any(
            part.startswith('.env') for part in source.parts) or source.name in ('auth.json', 'credentials') or source.suffix in ('.pem', '.key'):
        raise RecoveryError(f'{role}: credential/configuration path is forbidden')
    if any(parent.is_symlink() for parent in (source, *source.parents)) or not source.is_file():
        raise RecoveryError(f'{role}: source must be a regular, nonsymlink file')
    return role, source


def create_snapshot(root, spec):
    """Copy a paused, coherent writer's explicit files to one atomic generation.

    The caller must pause producers first. Changes during copying are rejected;
    no collection-wide transaction can be inferred from arbitrary live files.
    """
    run_id = checked_name(spec.get('run_id'), 'stable run ID')
    validate_source(spec.get('source'))
    if type(spec.get('synthetic')) is not bool or not spec.get('provenance'):
        raise RecoveryError('Explicit synthetic flag and experiment provenance are required')
    eligible = spec.get('application_resume_eligible', True)
    if type(eligible) is not bool:
        raise RecoveryError('Application resume eligibility must be explicit Boolean')
    if 'config' not in spec or 'seed' not in spec:
        raise RecoveryError('Frozen config and seed are required')
    artifacts = spec.get('artifacts', [])
    if not artifacts:
        raise RecoveryError('No artifacts specified')
    required = list(spec.get('required_roles', []))
    if len(set(required)) != len(required):
        raise RecoveryError('Duplicate required role')
    for role in required:
        checked_name(role, 'required role')
    dependencies = spec.get('excluded_dependencies', [])
    for dependency in dependencies:
        if not dependency.get('role') or not SHA.fullmatch(dependency.get('sha256', '')) or not dependency.get('reacquisition') or not dependency.get('reason'):
            raise RecoveryError('Excluded dependency needs role, exact hash, reason and reacquisition status')
        if type(dependency.get('required_for_resume')) is not bool:
            raise RecoveryError('Excluded dependency must explicitly state whether required for resume')
    root = Path(root).absolute()
    root.mkdir(parents=True, exist_ok=True)
    generation = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '-' + uuid.uuid4().hex[:12]
    staging = root / ('.staging-' + generation)
    final = root / generation
    staging.mkdir(mode=0o700)
    files = []
    try:
        seen = set()
        total = 0
        for item in artifacts:
            role, source = checked_artifact(item)
            if role in seen:
                raise RecoveryError('Duplicate artifact role: ' + role)
            seen.add(role)
            before = source.stat()
            relative = 'payload/' + role + '/' + source.name
            safe_member(relative)
            destination = staging / relative
            destination.parent.mkdir(parents=True)
            with source.open('rb') as incoming, destination.open('xb') as outgoing:
                shutil.copyfileobj(incoming, outgoing, 1024 * 1024)
                outgoing.flush(); os.fsync(outgoing.fileno())
            after = source.stat()
            copied_hash = sha256(destination)
            if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (
                    after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns) or copied_hash != sha256(source):
                raise RecoveryError('Artifact changed while snapshotting: ' + role)
            total += destination.stat().st_size
            if total > spec.get('max_payload_bytes', DEFAULT_MAX_BYTES):
                raise RecoveryError('Explicit maximum snapshot payload exceeded')
            files.append({'role': role, 'path': relative, 'sha256': copied_hash,
                          'bytes': destination.stat().st_size, 'schema': item['schema'],
                          'provenance': item['provenance'], 'retention_class': item['retention_class']})
        if not set(required).issubset(seen):
            raise RecoveryError('Missing required artifacts: ' + ', '.join(sorted(set(required) - seen)))
        previous = spec.get('previous_manifest_sha256')
        if previous is not None and not SHA.fullmatch(previous):
            raise RecoveryError('Invalid previous generation manifest hash')
        incomplete = any(item['required_for_resume'] for item in dependencies)
        manifest = {'schema': SCHEMA, 'run_id': run_id, 'generation': generation,
                    'created_at': utc_now(), 'synthetic': spec['synthetic'],
                    'source': spec['source'], 'config': spec['config'], 'seed': spec['seed'],
                    'provenance': spec['provenance'], 'files': files, 'required_roles': required,
                    'excluded_dependencies': dependencies, 'previous_manifest_sha256': previous,
                    'application_resume_eligible': eligible,
                    'full_resume_ready': bool(required) and not spec['synthetic'] and not incomplete and eligible,
                    'producer_contract': 'writers-paused; explicit-files; no-hidden-automatic-globs'}
        atomic_json(staging / 'manifest.json', manifest)
        filename = 'LaboratoireHumain-backup-' + run_id + '-' + generation + '.zip'
        bundle = staging / filename
        with zipfile.ZipFile(bundle, 'x', compression=zipfile.ZIP_DEFLATED, allowZip64=True) as archive:
            for entry in ['manifest.json'] + [item['path'] for item in files]:
                archive.write(staging / entry, entry)
        with bundle.open('rb') as stream:
            os.fsync(stream.fileno())
        verify_bundle(bundle, max_payload_bytes=spec.get('max_payload_bytes', DEFAULT_MAX_BYTES))
        # Keep one archive + small manifest. Never silently prune prior generations.
        shutil.rmtree(staging / 'payload')
        fsync_tree_directories(staging)
        os.rename(staging, final)
        fsync_directory(root)
        return final / filename
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def verify_bundle(bundle, expected_sha256=None, max_payload_bytes=DEFAULT_MAX_BYTES):
    if expected_sha256 is not None and sha256(bundle) != expected_sha256:
        raise RecoveryError('Archive SHA256 mismatch')
    with zipfile.ZipFile(bundle) as archive:
        entries = archive.infolist()
        names = [item.filename for item in entries]
        if len(set(names)) != len(names):
            raise RecoveryError('Duplicate archive member')
        if sum(item.file_size for item in entries) > max_payload_bytes + 4 * 1024**2:
            raise RecoveryError('Archive expansion exceeds approved bound')
        for item in entries:
            safe_member(item.filename)
            if item.is_dir() or stat.S_ISLNK(item.external_attr >> 16) or item.flag_bits & 1:
                raise RecoveryError('Archive contains nonregular/encrypted member')
        if 'manifest.json' not in names or archive.getinfo('manifest.json').file_size > 4 * 1024**2:
            raise RecoveryError('Missing or oversized manifest')
        manifest = json.loads(archive.read('manifest.json'))
        if manifest.get('schema') != SCHEMA or type(manifest.get('synthetic')) is not bool:
            raise RecoveryError('Unknown backup schema or synthetic status')
        checked_name(manifest.get('run_id'), 'run ID'); checked_name(manifest.get('generation'), 'generation')
        validate_source(manifest.get('source'))
        files = manifest.get('files', [])
        if not files or set(names) != {'manifest.json', *(item['path'] for item in files)}:
            raise RecoveryError('Archive members differ from explicit manifest')
        roles = [item['role'] for item in files]
        if len(set(roles)) != len(roles) or len({item['path'] for item in files}) != len(files):
            raise RecoveryError('Duplicate manifest role/path')
        for item in files:
            checked_name(item['role'], 'role')
            if item.get('retention_class') not in ('user_generated', 'project_source') or not item.get('schema') or not item.get('provenance'):
                raise RecoveryError('Missing artifact classification/provenance')
            if not SHA.fullmatch(item.get('sha256', '')) or type(item.get('bytes')) is not int or item['bytes'] < 0:
                raise RecoveryError('Invalid artifact hash/size')
            if item['path'] != 'payload/' + item['role'] + '/' + PurePosixPath(item['path']).name:
                raise RecoveryError('Artifact path differs from role')
            info = archive.getinfo(item['path'])
            digest = hashlib.sha256()
            with archive.open(info) as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
            if info.file_size != item['bytes'] or digest.hexdigest() != item['sha256']:
                raise RecoveryError('Artifact hash/size mismatch: ' + item['role'])
        required = manifest.get('required_roles')
        if not isinstance(required, list) or not set(required).issubset(roles):
            raise RecoveryError('Manifest omits a required role')
        dependencies = manifest.get('excluded_dependencies')
        if not isinstance(dependencies, list):
            raise RecoveryError('Missing excluded-dependency declaration')
        for item in dependencies:
            if not SHA.fullmatch(item.get('sha256', '')) or not item.get('reason') or not item.get('reacquisition') or type(item.get('required_for_resume')) is not bool:
                raise RecoveryError('Invalid excluded dependency')
        eligible=manifest.get('application_resume_eligible',True)  # Older v1 manifests remain readable.
        if type(eligible) is not bool:raise RecoveryError('Invalid application resume eligibility')
        ready = bool(required) and not manifest['synthetic'] and not any(item['required_for_resume'] for item in dependencies) and eligible
        if manifest.get('full_resume_ready') is not ready:
            raise RecoveryError('Invalid full-resume claim')
        return manifest


def restore_bundle(bundle, destination, expected_sha256=None, max_payload_bytes=DEFAULT_MAX_BYTES):
    """Verify everything before atomic extraction into a new isolated directory."""
    manifest = verify_bundle(bundle, expected_sha256, max_payload_bytes)
    destination = Path(destination).absolute()
    if destination.exists():
        raise RecoveryError('Restore destination must not exist; never overwrite a run')
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = destination.with_name('.restore-' + uuid.uuid4().hex)
    staging.mkdir(mode=0o700)
    try:
        with zipfile.ZipFile(bundle) as archive:
            for item in archive.infolist():
                target = staging.joinpath(*safe_member(item.filename).parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(item) as incoming, target.open('xb') as outgoing:
                    shutil.copyfileobj(incoming, outgoing, 1024 * 1024)
                    outgoing.flush(); os.fsync(outgoing.fileno())
        for item in manifest['files']:
            if sha256(staging / item['path']) != item['sha256']:
                raise RecoveryError('Restored file mismatch: ' + item['role'])
        fsync_tree_directories(staging)
        os.rename(staging, destination); fsync_directory(destination.parent)
        return manifest
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def helper_call(command, payload, deadline=None):
    timeout = 180 if deadline is None else min(180, deadline - time.time())
    if timeout <= 0:
        raise RecoveryError('Original experiment deadline reached before durable operation')
    try:
        result = subprocess.run(command, input=json.dumps(payload), text=True,
                                capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as error:
        raise RecoveryError('Durable operation timed out; outcome uncertain, no automatic write retry') from error
    if result.returncode:
        # Helper stderr may include signed transfer URLs; retain it privately,
        # but never reflect it in logs, reports, or user-facing exceptions.
        raise RecoveryError('Official Library helper failed; run is paused, no automatic write retry')
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RecoveryError('Library helper returned an unrecognized result; no automatic write retry') from error


class LibraryBackend:
    """Use fresh official Library helpers outside Git; no tokens or URLs saved.

    Prepared upload handles large archives (including >=50 MiB) without loading
    them into a JSON/base64 request. Server limits still apply; failures pause.
    """
    def __init__(self, helper_directory, proof_directory, max_payload_bytes=DEFAULT_MAX_BYTES):
        self.helpers = Path(helper_directory).resolve()
        self.proofs = Path(proof_directory).absolute()
        self.max_payload_bytes = max_payload_bytes
        for name in ('library_upload.py', 'library_hosted_apps.py', 'library_file_transfer.py'):
            if not (self.helpers / name).is_file():
                raise RecoveryError('Current official Library helpers are required at runtime')
        self.proofs.mkdir(parents=True, exist_ok=True)

    def retain_many(self, bundles, deadline=None):
        bundles = [Path(path).resolve() for path in bundles]
        if not 1 <= len(bundles) <= 20:
            raise RecoveryError('Retain one to twenty immutable generations per batch')
        for path in bundles:
            verify_bundle(path, max_payload_bytes=self.max_payload_bytes)
        # The current Library skill owns this helper and its supported protocol.
        response = helper_call([sys.executable, str(self.helpers / 'library_upload.py')],
            {'uploads': [{'local_path': str(path), 'purpose': 'create_library_file',
                          'library_artifact_type': 'other'} for path in bundles]}, deadline)
        results = response.get('results', [])
        if len(results) != len(bundles):
            raise RecoveryError('Uncorrelated upload result; no retry until Library is inspected')
        receipts = []
        for bundle, result in zip(bundles, results):
            # Keep result parsing strict and ordered. An accepted upload alone
            # never creates a recovery proof; fresh bytes must come back.
            value = result.get('result', result)
            if result.get('error') or not value.get('library_file_id'):
                raise RecoveryError('A generation was not retained; earlier successful writes remain retained')
            locator = {'library_file_id': value['library_file_id'],
                       'file_id': value.get('file_id'),
                       'file_name': value.get('file_name', value.get('name', bundle.name)),
                       'version': value.get('version', value.get('version_id'))}
            receipts.append(self.prove(locator, sha256(bundle), deadline))
        return receipts

    def prove(self, locator, expected_sha256, deadline=None):
        """Download from private Library, extract separately, verify every byte."""
        if not locator.get('library_file_id') or not SHA.fullmatch(expected_sha256):
            raise RecoveryError('A confirmed remote locator and exact archive hash are required')
        # Use the official client only for the documented resolved-reference
        # materialization tool, then its transfer helper unchanged.
        program = ('import json,sys;sys.path.insert(0,sys.argv[1]);'
                   'from library_hosted_apps import HostedAppsClient;'
                   'print(json.dumps(HostedAppsClient().call_tool("connector_openai_library",'
                   '"prepare_materialize",json.load(sys.stdin))))')
        attempt = self.proofs / ('roundtrip-' + uuid.uuid4().hex)
        attempt.mkdir(mode=0o700)
        item = {'library_file_id': locator['library_file_id']}
        if locator.get('file_id') and locator.get('file_name'):
            item.update(file_id=locator['file_id'], file_name=locator['file_name'])
        prepared = helper_call([sys.executable, '-c', program, str(self.helpers)],
            {'items': [item], 'destination': {'directory': str(attempt / 'download')}}, deadline)
        payload = prepared.get('structuredContent')
        if payload is None:
            blocks = [item['text'] for item in prepared.get('content', []) if item.get('type') == 'text']
            if len(blocks) != 1:
                raise RecoveryError('Ambiguous Library materialization result')
            payload = json.loads(blocks[0])
        payload = payload.get('result', payload)
        transfers = payload.get('transfers', [])
        if len(transfers) != 1 or transfers[0].get('library_file_id') != locator['library_file_id']:
            raise RecoveryError('Remote materialization identity mismatch')
        transfer = transfers[0]
        downloaded = Path(transfer['workspace_path']) if transfer.get('workspace_path') else attempt / 'downloaded.zip'
        command = [sys.executable, str(self.helpers / 'library_file_transfer.py'),
                   'apply-xattrs' if transfer.get('workspace_path') else 'materialize', str(downloaded)]
        # Transfer helper prints a path, not JSON. All transfer details stay in
        # process stdin and are never copied into durable receipts or Git.
        timeout = 180 if deadline is None else min(180, deadline-time.time())
        if timeout <= 0:
            raise RecoveryError('Original deadline reached before restoring remote bytes')
        try:
            completed = subprocess.run(command, input=json.dumps(transfer), text=True,
                capture_output=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired as error:
            raise RecoveryError('Remote materialization timed out') from error
        if completed.returncode or not downloaded.is_file():
            raise RecoveryError('Remote bytes could not be materialized with Library metadata')
        manifest = restore_bundle(downloaded, attempt / 'restored', expected_sha256, self.max_payload_bytes)
        receipt = {'schema': RECEIPT_SCHEMA, 'store': 'private-chatgpt-library',
                   'locator': locator, 'bundle_sha256': expected_sha256,
                   'manifest_sha256': sha256(attempt / 'restored/manifest.json'),
                   'run_id': manifest['run_id'], 'generation': manifest['generation'],
                   'restored_at': utc_now(), 'synthetic': manifest['synthetic'],
                   'full_resume_ready': manifest['full_resume_ready'],
                   'restored_directory': str(attempt / 'restored'),
                   'files': manifest['files']}
        atomic_json(attempt / 'recovery-proof.json', receipt)
        return receipt


def require_recovery_point(receipt_path, input_manifest, required_roles, backend, deadline=None, run_id=None):
    """Fail closed before model/physics startup, using a NEW remote round trip."""
    receipt = json.loads(Path(receipt_path).read_text())
    if receipt.get('schema') != RECEIPT_SCHEMA or receipt.get('store') != 'private-chatgpt-library':
        raise RecoveryError('A restore-tested private durable recovery point is required')
    proof = backend.prove(receipt.get('locator', {}), receipt.get('bundle_sha256', ''), deadline)
    if proof['synthetic'] or not proof['full_resume_ready']:
        raise RecoveryError('Synthetic or incomplete recovery points cannot admit a real run')
    if run_id is not None and proof['run_id'] != run_id:
        raise RecoveryError('Recovery point belongs to another stable run ID')
    files = {item['role']: item for item in proof['files']}
    if not required_roles or not set(required_roles).issubset(files):
        raise RecoveryError('Recovery point omits required retained inputs')
    if files.get('input_manifest', {}).get('sha256') != sha256(input_manifest):
        raise RecoveryError('Recovery point is not bound to this exact retained-input manifest')
    inputs = json.loads(Path(input_manifest).read_text()).get('files', {})
    for role in required_roles:
        if role == 'input_manifest':
            continue
        expected = inputs.get(role, {}).get('sha256')
        if not SHA.fullmatch(expected or '') or files[role]['sha256'] != expected:
            raise RecoveryError('Recovery point input artifact hash differs from retained-input manifest: ' + role)
    return proof


class RecoveryGuard:
    def __init__(self, root, run_id, backend, source, synthetic=False):
        self.root = Path(root); self.run_id = run_id; self.backend = backend
        self.source = source; self.synthetic = synthetic; self.previous = None
        self.retention_blocked = False

    def checkpoint(self, artifacts, config, seed, provenance, *, required_roles,
                   excluded_dependencies=(), deadline=None):
        if self.retention_blocked:
            raise RecoveryError('A prior durable operation failed; inspect Library before any further write')
        if deadline is not None and time.time() >= deadline:
            raise RecoveryError('Original deadline reached before checkpoint; no restart')
        bundle = create_snapshot(self.root, {'run_id': self.run_id, 'source': self.source,
            'synthetic': self.synthetic, 'config': config, 'seed': seed, 'provenance': provenance,
            'artifacts': artifacts, 'required_roles': required_roles,
            'excluded_dependencies': list(excluded_dependencies), 'previous_manifest_sha256': self.previous})
        self.retention_blocked = True
        proof = self.backend.retain_many([bundle], deadline)[0]
        if proof['synthetic'] != self.synthetic:
            raise RecoveryError('Remote checkpoint changed experiment provenance')
        if not self.synthetic and not proof['full_resume_ready']:
            raise RecoveryError('Checkpoint retained but prerequisites are incomplete; real run must stay paused')
        self.previous = proof['manifest_sha256']
        proof_path = bundle.parent / 'recovery-proof.json'
        atomic_json(proof_path, proof)
        self.retention_blocked = False
        if deadline is not None and time.time() >= deadline:
            raise RecoveryError('Checkpoint consumed original remaining wall time; run must stop')
        return proof_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    create = sub.add_parser('snapshot'); create.add_argument('--spec', type=Path, required=True)
    create.add_argument('--root', type=Path, required=True)
    restore = sub.add_parser('restore'); restore.add_argument('bundle', type=Path)
    restore.add_argument('--destination', type=Path, required=True); restore.add_argument('--sha256')
    retain = sub.add_parser('retain'); retain.add_argument('bundles', type=Path, nargs='+')
    retain.add_argument('--library-helpers', type=Path, required=True)
    retain.add_argument('--proof-directory', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == 'snapshot':
            print(create_snapshot(args.root, json.loads(args.spec.read_text())))
        elif args.command == 'restore':
            value = restore_bundle(args.bundle, args.destination, args.sha256)
            print(json.dumps({'verified': True, 'run_id': value['run_id'], 'synthetic': value['synthetic'],
                              'full_resume_ready': value['full_resume_ready']}))
        else:
            backend = LibraryBackend(args.library_helpers, args.proof_directory)
            receipts = backend.retain_many(args.bundles)
            print(json.dumps(receipts, indent=2))
    except (RecoveryError, OSError, ValueError, zipfile.BadZipFile) as error:
        print(json.dumps({'blocked': str(error), 'experiment_started': False}), file=sys.stderr)
        raise SystemExit(2)


if __name__ == '__main__':
    main()
