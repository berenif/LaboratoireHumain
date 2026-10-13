#!/usr/bin/env python3
"""Prepare a NEW local-pair manifest from explicit, source-bound evidence.

Pure file/JSON/hash operations only: no worker, budget mutation, FK, or physics.
First create a pending receipt with --template. After the listed checks actually
pass, fill each check with hashed evidence and run --readiness. A receipt is an
explicit review attestation; this script verifies its completeness and binding,
not the scientific truth of arbitrary evidence. Output is exclusive/create-only.
Neither output constitutes authorization to execute the native pair.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
ORIGINAL = REPO / 'docs/evidence/coordination-prototype-2026-10-12'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def record(path):
    path = Path(path).resolve(strict=True)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return {'path': str(path), 'sha256': digest.hexdigest()}


def records(paths):
    return [record(path) for path in sorted(set(Path(p).resolve() for p in paths))]


def source_tree_files(root):
    """Include vendored build inputs and data; exclude generated/local caches."""
    root = Path(root)
    if not root.exists():
        return []
    excluded = {'target', '.git', '__pycache__'}
    return [path for path in root.rglob('*') if path.is_file()
            and not excluded.intersection(path.relative_to(root).parts)]


def verify_record(item):
    require(isinstance(item, dict) and set(item) == {'path', 'sha256'},
            'Evidence must be an explicit path/sha256 record')
    actual = record(item['path'])
    require(actual == item, 'Evidence is changed or its path is not absolute: ' + str(item['path']))
    return actual


def budget_module():
    spec = importlib.util.spec_from_file_location('local_pair_budget', REPO / 'scripts/coordination-prototype-budget.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_static_reference(original=ORIGINAL):
    program_path = original / 'continuation-candidate-program.json'
    program = read(program_path)
    tasks = read(original / 'continuation-tasks.json')
    require([key['substep'] for key in program['keyframes']] == [0, 168, 300, 432, 1680, 1920],
            'The frozen six-key reference changed')
    require(program['keyframes'][-1]['positions'] == program['keyframes'][-2]['positions'],
            'Missing final one-second reference hold')
    require(all(value == 0 for key in program['keyframes'][-2:] for row in key['velocities'] for value in row),
            'Final reference hold has nonzero endpoint velocity')
    require(tasks['programSha256'] == record(program_path)['sha256'], 'Task/program mismatch')
    require(tasks['nativeHz'] == 240 and tasks['controlHz'] == 60 and tasks['physicalIntegrations'] == 0,
            'Wrong static task scope or rates')
    require([row['substep'] for row in tasks['rows']] == list(range(0, 1921, 4)),
            'Incomplete archived reference tasks')
    for suffix in ('', '-60deg'):
        audit = read(original / ('continuation-native-audit' + suffix + '.json'))
        trace = original / ('continuation-native-geometry' + suffix + '.jsonl.gz')
        require(audit['programSha256'] == tasks['programSha256'], 'Native audit/program mismatch')
        require(audit['compressedTraceSha256'] == record(trace)['sha256'], 'Changed native geometry archive')
        require(audit['nativeCommandCompilerPassed'] is True and audit['nativeGeometryRefusals'] == []
                and audit['nativeGeometryQueries'] == 1921 and audit['physicalIntegrations'] == 0,
                'Incomplete or refused static continuation audit')


def inventory(binary):
    binary = Path(binary).resolve(strict=True)
    require(os.access(binary, os.X_OK), 'Transition worker must be an executable file')
    validate_static_reference()
    # Original physical inputs are compared byte-for-byte, not reinterpreted.
    old = read(ORIGINAL / 'frozen-manifest.json')
    old_repo = Path(old['files']['profile'][0]['path']).parents[4]
    def original_record(item):
        actual = record(REPO / Path(item['path']).relative_to(old_repo))
        require(actual['sha256'] == item['sha256'], 'Original frozen input changed: ' + actual['path'])
        return actual
    for role in ('model', 'profile', 'fixture', 'criteria'):
        for item in old['files'][role]:
            original_record(item)
    for item in old['files']['code']:
        if Path(item['path']).name in ('trajectory_program.rs', 'trajectory_geometry.rs', 'continuation.rs'):
            original_record(item)
    index_path = HERE / 'archive-index.json'
    compressed = HERE / 'native-run/native-result.json.gz'
    archived = [item for item in read(index_path)['files']
                if Path(item['gzipPath']).name == compressed.name]
    require(len(archived) == 1, 'Missing/ambiguous compressed parity receipt')
    archived = archived[0]
    require(record(compressed)['sha256'] == archived['gzipSha256'], 'Changed compressed parity receipt')
    raw = gzip.decompress(compressed.read_bytes())
    require(len(raw) == archived['rawBytes'] and hashlib.sha256(raw).hexdigest() == archived['rawSha256'],
            'Compressed parity receipt does not reproduce its archived bytes')
    native = json.loads(raw)
    require(native['actualNativeSteps'] == 44 and native['nativeAccounting']['exact'] is True
            and all(native['outcome'].get(name) is True for name in
                    ('integrity_ok', 'prefix_matches_archive', 'parity_verified', 'diagnostic_present')),
            'The archived 44-step parity receipt did not pass')
    crates = REPO / 'rust/crates'
    build_inputs = [*source_tree_files(REPO / 'rust/vendor'),
                    *source_tree_files(REPO / '.cargo'), *source_tree_files(REPO / 'rust/.cargo')]
    build_inputs.extend(path for base in (REPO, REPO / 'rust')
                        for name in ('rust-toolchain', 'rust-toolchain.toml', 'build.rs')
                        if (path := base / name).is_file())
    code = [*crates.rglob('*.rs'), *crates.rglob('Cargo.toml'), REPO / 'rust/Cargo.toml',
            *build_inputs,
            REPO / 'rust/Cargo.lock', REPO / 'scripts/coordination-prototype-budget.py',
            REPO / 'scripts/run-coordination-prototype.py',
            REPO / 'scripts/coordination-prototype-budget-test.py',
            REPO / 'scripts/run-coordination-prototype-test.py', Path(__file__),
            HERE / 'prepare_local_pair_manifest_test.py']
    files = {role: [original_record(item) for item in old['files'][role]] for role in ('model', 'profile', 'fixture')}
    files.update({
        'reference': records([ORIGINAL / 'continuation-candidate-program.json', ORIGINAL / 'continuation-tasks.json']),
        'code': records(code), 'binary': records([binary]),
        'criteria': records([HERE / 'local-pair-criteria.json', REPO / 'rust/crates/sim/data/recovery-contract-v1.json']),
        'continuation': records([ORIGINAL / name for name in (
            'continuation-route-audit.json', 'continuation-native-audit.json', 'continuation-native-audit-60deg.json',
            'continuation-native-geometry.jsonl.gz', 'continuation-native-geometry-60deg.jsonl.gz')]),
        'preflight_evidence': records([index_path, compressed, HERE / 'native-run/run-result.json']),
    })
    return files


def bindings(files):
    return sorted({(item['path'], item['sha256']) for group in files.values() for item in group})


def pending_receipt(files, prerequisites):
    return {'schema': 1, 'protocolMode': 'local-pair', 'physicalIntegrations': 0,
            'staticInspections': 0, 'nativeExecutionAuthorized': False,
            'checkedFiles': [{'path': path, 'sha256': digest} for path, digest in bindings(files)],
            'checks': {name: {'status': 'pending', 'evidence': []} for name in prerequisites}}


def validate_readiness(receipt, files, prerequisites):
    require(set(receipt) == {'schema', 'protocolMode', 'physicalIntegrations', 'staticInspections',
                             'nativeExecutionAuthorized', 'checkedFiles', 'checks'}, 'Unknown/missing receipt fields')
    require(receipt['schema'] == 1 and receipt['protocolMode'] == 'local-pair'
            and type(receipt['physicalIntegrations']) is int and receipt['physicalIntegrations'] == 0
            and type(receipt['staticInspections']) is int and receipt['staticInspections'] == 0
            and receipt['nativeExecutionAuthorized'] is False, 'This preparation receipt must remain pure and non-authorizing')
    checked = [verify_record(item) for item in receipt['checkedFiles']]
    require(len({item['path'] for item in checked}) == len(checked), 'Duplicate preparation binding')
    require(bindings({'checked': checked}) == bindings(files), 'Preparation evidence is stale or incompletely bound')
    require(set(receipt['checks']) == set(prerequisites), 'Missing/unknown local-pair prerequisites')
    evidence = []
    for name in prerequisites:
        check = receipt['checks'][name]
        require(set(check) == {'status', 'evidence'} and check['status'] == 'passed'
                and isinstance(check['evidence'], list) and len(check['evidence']) > 0,
                'Missing passed evidence for ' + name)
        evidence.extend(verify_record(item) for item in check['evidence'])
    return records(item['path'] for item in evidence)


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--template', type=Path, help='Write a pending, non-admitting preparation receipt')
    action.add_argument('--readiness', type=Path, help='Explicit reviewed preparation receipt')
    parser.add_argument('--output', type=Path, help='New manifest path, required with --readiness')
    args = parser.parse_args()
    try:
        require((args.readiness is not None) == (args.output is not None), '--output is required only with --readiness')
        module = budget_module()
        _, _, prerequisites = module.protocol_contract('local-pair')
        files = inventory(args.binary)
        if args.template:
            write_new(args.template, pending_receipt(files, prerequisites))
            print(json.dumps({'pendingReceipt': record(args.template), 'physicalIntegrations': 0, 'nativeExecutionAuthorized': False}))
            return
        evidence = validate_readiness(read(args.readiness), files, prerequisites)
        files['preflight_evidence'] = records([args.readiness, *(item['path'] for item in files['preflight_evidence'] + evidence)])
        manifest = {'schema': 1, 'protocolMode': 'local-pair', 'files': files,
                    'prerequisites': {name: True for name in prerequisites},
                    'workerInputs': {'programPath': str(ORIGINAL / 'continuation-candidate-program.json'),
                                     'tasksPath': str(ORIGINAL / 'continuation-tasks.json')}}
        write_new(args.output, manifest)
        print(json.dumps({'manifest': record(args.output), 'physicalIntegrations': 0, 'nativeExecutionAuthorized': False}))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, 'REFUSED: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
