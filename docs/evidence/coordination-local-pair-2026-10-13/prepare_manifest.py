#!/usr/bin/env python3
"""Pure wrapper for the local A/B pair. Never starts a worker or a ledger.

Reuses the prior reviewed builder unchanged, with the reduced 7100-second
criteria and explicit additional evidence. --template leaves every prerequisite
pending. --readiness requires exact current file bindings and reviewed evidence.
"""
import argparse
import copy
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE_PATH = HERE.parent / 'coordination-native-serialization-2026-10-13/prepare_local_pair_manifest.py'
SPEC = importlib.util.spec_from_file_location('previous_local_pair_builder', BASE_PATH)
base = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(base)


def validate_criteria(prior, proposed):
    expected = copy.deepcopy(prior)
    expected['physics']['maximumWallSeconds'] = 7100
    base.require(proposed == expected, 'Only the wall ceiling may change, from 7200 to 7100 seconds')


def inventory(binary, preflight_files):
    files = base.inventory(binary)
    criteria = HERE / 'local-pair-criteria.json'
    validate_criteria(base.read(base.HERE / 'local-pair-criteria.json'), base.read(criteria))
    limits, order, _ = base.budget_module().protocol_contract('local-pair')
    base.require(limits['physicsWallSeconds'] == 7100 and limits['totalNativeSteps'] == 78736
                 and order == ['A', 'B'], 'Ledger local-pair bounds do not match this preparation')
    files['criteria'] = base.records([criteria, base.REPO / 'rust/crates/sim/data/recovery-contract-v1.json'])
    files['code'] = base.records([*(item['path'] for item in files['code']),
                                  base.REPO / 'scripts/run-coordination-numeric-parity.py', Path(__file__),
                                  HERE / 'prepare_manifest_test.py', HERE / 'check_continuation_archive.py'])
    files['preflight_evidence'] = base.records([
        *(item['path'] for item in files['preflight_evidence']),
        HERE / 'continuation-preflight.md', HERE / 'continuation-archive-check.json',
        *preflight_files])
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--preflight-file', type=Path, action='append', default=[],
                        help='Reviewed local evidence or cumulative budget receipt; repeat as needed')
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--template', type=Path)
    action.add_argument('--readiness', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        base.require((args.readiness is not None) == (args.output is not None),
                     '--output is required only with --readiness')
        _, _, prerequisites = base.budget_module().protocol_contract('local-pair')
        files = inventory(args.binary, args.preflight_file)
        if args.template:
            base.write_new(args.template, base.pending_receipt(files, prerequisites))
            print(json.dumps({'pendingReceipt': base.record(args.template), 'physicalIntegrations': 0,
                              'staticInspections': 0, 'nativeExecutionAuthorized': False}))
            return
        evidence = base.validate_readiness(base.read(args.readiness), files, prerequisites)
        files['preflight_evidence'] = base.records([
            args.readiness, *(item['path'] for item in files['preflight_evidence'] + evidence)])
        manifest = {'schema': 1, 'protocolMode': 'local-pair', 'files': files,
                    'prerequisites': {name: True for name in prerequisites},
                    'workerInputs': {
                        'programPath': str(base.ORIGINAL / 'continuation-candidate-program.json'),
                        'tasksPath': str(base.ORIGINAL / 'continuation-tasks.json')}}
        base.write_new(args.output, manifest)
        print(json.dumps({'manifest': base.record(args.output), 'physicalIntegrations': 0,
                          'staticInspections': 0, 'nativeExecutionAuthorized': False}))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, 'REFUSED: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
