#!/usr/bin/env python3
"""Read archived JSON/gzip values and hashes only; never calculate geometry."""
import gzip
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('local_builder', HERE / 'prepare_manifest.py')
wrapper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(wrapper)
b = wrapper.base


def check():
    b.validate_static_reference()
    inputs = [Path(__file__), HERE / 'prepare_manifest.py', HERE / 'local-pair-criteria.json',
              b.HERE / 'local-pair-criteria.json', b.ORIGINAL / 'frozen-criteria.json',
              b.ORIGINAL / 'continuation-candidate-program.json', b.ORIGINAL / 'continuation-tasks.json',
              b.ORIGINAL / 'continuation-route-audit.json', b.ORIGINAL / 'continuation-seed.json',
              b.ORIGINAL / 'frozen-manifest.json']
    previous = b.read(b.HERE / 'local-pair-criteria.json')
    current = b.read(HERE / 'local-pair-criteria.json')
    wrapper.validate_criteria(previous, current)
    original_criteria = b.read(b.ORIGINAL / 'frozen-criteria.json')
    b.require(all(current['local'][key] == value for key, value in original_criteria['local'].items()),
              'An original local criterion changed')
    for key in ('fixture', 'unchanged', 'forbidden', 'finalQualification'):
        b.require(current[key] == original_criteria[key], 'Original physical contract changed: ' + key)
    archived = []
    for suffix, heading in (('', 0), ('-60deg', 60)):
        path = b.ORIGINAL / ('continuation-native-geometry' + suffix + '.jsonl.gz')
        audit = b.ORIGINAL / ('continuation-native-audit' + suffix + '.json')
        count = 0
        with gzip.open(path, 'rt') as stream:
            for step, raw in enumerate(stream):
                row = json.loads(raw)
                b.require(row['substep'] == step and row['query']['programSubstep'] == step,
                          'Non-consecutive archived geometry rows')
                b.require(row['result']['nativeRefusal'] is None
                          and row['result']['physicalIntegrations'] == 0
                          and len(row['result']['poses']) == 25, 'Refused/incomplete archived geometry row')
                count += 1
        b.require(count == 1921, 'Incomplete archived geometry grid')
        archived.append({'headingDegrees': heading, 'archivedRowsRead': count,
                         'firstSubstep': 0, 'lastSubstep': count - 1, 'refusals': 0,
                         'physicalIntegrationsOriginallyReported': 0})
        inputs.extend([path, audit])
    old = b.read(b.ORIGINAL / 'frozen-manifest.json')
    old_repo = Path(old['files']['profile'][0]['path']).parents[4]
    unchanged = []
    for role in ('model', 'profile', 'fixture'):
        for item in old['files'][role]:
            path = b.REPO / Path(item['path']).relative_to(old_repo)
            actual = b.record(path)
            b.require(actual['sha256'] == item['sha256'], 'Original model/profile/fixture changed')
            inputs.append(path)
            unchanged.append(actual)
    fixture_path = b.REPO / 'rust/crates/sim/data/recovery-v1.json'
    fixture = next(case for case in b.read(fixture_path)['cases'] if case['id'] == 'landed-crouch-left')
    b.require(len(fixture['poses']) == 25, 'Fixture does not have exactly 25 stored poses')
    for pose in fixture['poses']:
        b.require(all(v == 0 for key in ('linearVelocity', 'angularVelocity') for v in pose[key].values()),
                  'Fixture initial velocity changed')
    initial = b.read(b.ORIGINAL / 'continuation-seed.json')['initial']
    source_paths = [b.REPO / 'scripts/recovery-fixtures.ts',
                    b.REPO / 'rust/crates/sim/src/recovery/planner/whole_body_controller.rs',
                    b.REPO / 'rust/crates/sim/src/recovery/planner/continuation.rs',
                    b.REPO / 'rust/crates/sim/src/recovery/trajectory_program.rs',
                    b.REPO / 'rust/crates/sim/src/recovery/trajectory_geometry.rs']
    inputs.extend(source_paths)
    for item in old['files']['code']:
        if Path(item['path']).name in ('continuation.rs', 'trajectory_program.rs', 'trajectory_geometry.rs'):
            path = b.REPO / Path(item['path']).relative_to(old_repo)
            b.require(b.record(path)['sha256'] == item['sha256'], 'Static compiler/geometry source changed')
    return {'schema': 1, 'scope': 'archived-value and hash checks only', 'status': 'passed',
            'physicalIntegrations': 0, 'staticInspections': 0, 'newGeometryQueries': 0,
            'nativeWorldsConstructed': 0, 'nativeExecutionAuthorized': False,
            'criteriaChange': {'field': 'physics.maximumWallSeconds', 'before': 7200, 'after': 7100},
            'originalLocalCriteriaUnchanged': True, 'unchangedPhysicalInputs': unchanged,
            'geometryArchives': archived, 'fixture': {'id': fixture['id'], 'storedPoses': 25,
                'allStoredVelocitiesZero': True, 'rootPositionFromArchiveM': initial['rootPositionM'],
                'comFromArchiveM': initial['inspection']['centerOfMassM'],
                'impactOrSettleExecutedHere': False},
            'inputs': b.records(inputs)}


if __name__ == '__main__':
    print(json.dumps(check(), indent=2, allow_nan=False))
