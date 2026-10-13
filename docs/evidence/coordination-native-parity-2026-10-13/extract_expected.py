#!/usr/bin/env python3
"""Freeze existing JSON values for the approved parity protocol. No physics/FK."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import struct
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OLD = ROOT / 'docs/evidence/coordination-prototype-2026-10-12/native-run'
NUMERIC = ROOT / 'docs/evidence/coordination-numeric-correction-2026-10-12'
STATES = OLD / '002-B.states.jsonl.gz'
REPORT = OLD / '002-B.json'
INDEX = OLD / 'archive-index.json'
CERTIFIED = NUMERIC / 'certified-commands.json'
SCALES = [0.0, 0.5, 1.0, 1.5]
MOTOR_SCALARS = ['target_pos', 'target_vel', 'stiffness', 'damping', 'max_force',
                'impulse', 'delivered_abs_impulse', 'delivered_impulse']
MOTOR_VECTORS = ['delivered_angular_impulse_1', 'delivered_angular_impulse_2']


def sha(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def f32_bits(value):
    packed = struct.pack('<f', value)
    assert struct.unpack('<f', packed)[0] == value, 'Archive value not an exact widened f32'
    return struct.unpack('<I', packed)[0]


def body_motor_bits(frame):
    values = []
    assert [b['segment'] for b in frame['support']['bodies']] == list(range(25))
    for body in frame['support']['bodies']:
        for field in ['position_m', 'rotation_xyzw', 'linear_velocity_mps', 'angular_velocity_radps']:
            values.extend(body[field])
    assert [m['segment'] for m in frame['motors']] == list(range(1, 25))
    for joint in frame['motors']:
        assert len(joint['motors']) == 6
        for motor in joint['motors']:
            values.extend(motor[k] for k in MOTOR_SCALARS)
            for key in MOTOR_VECTORS:
                values.extend(motor[key])
    assert len(values) == 2341
    return [f32_bits(value) for value in values]


def prepare():
    compressed = STATES.read_bytes()
    raw = gzip.decompress(compressed) if compressed[:2] == b'\x1f\x8b' else compressed
    index = json.loads(INDEX.read_bytes())
    provenance = next(f for f in index['files'] if f['original'] == '002-B.states.jsonl')
    assert sha(compressed) == provenance['archiveSha256']
    assert sha(raw) == provenance['originalSha256']
    uncompressed_path = OLD / provenance['original']
    if uncompressed_path.exists():
        assert uncompressed_path.read_bytes() == raw
    rows = [json.loads(line) for line in raw.splitlines()]
    assert len(rows) == 48
    groups = [rows[12 * i:12 * (i + 1)] for i in range(4)]
    for index, group in enumerate(groups):
        assert [row['nativeSubstep'] for row in group] == list(range(1, 13))
        assert all(row['kind'] == f'preview-0-{index}' for row in group)
        assert all(row['owner']['feedbackScale'] == SCALES[index] for row in group)
    equivalence = []
    for native in range(1, 13):
        normalized = []
        for group in groups:
            frame = copy.deepcopy(group[native - 1])
            frame.pop('kind')
            frame['owner']['feedbackScale'] = 1.0
            normalized.append(canonical(frame))
        assert len(set(normalized)) == 1
        equivalence.append({'nativeSubstep': native, 'fourCasesEqual': True,
                            'normalizedFrameSha256': sha(normalized[0])})
    prefix = groups[2]  # Exact source rows, already feedbackScale=1; no owner mutation.
    certified = json.loads(CERTIFIED.read_bytes())
    assert certified['nativeIntegrations'] == 0
    cases = []
    for index, source in enumerate(certified['cases']):
        assert source['feedbackScale'] == SCALES[index]
        assert source['actualF32CommandsCertified'] == 4 and source['replayedCommandBitsEqual']
        audit = source['audit']
        assert audit['stage'] == 'certified guarded exact f32 commands'
        assert audit['previousQ'] == prefix[-1]['owner']['q']
        commands = audit['guardedAttempt']['attemptedCommands']
        assert [command['nativeSubstep'] for command in commands] == [12, 13, 14, 15]
        for command in commands:
            for key in ['positions', 'velocities']:
                assert len(command[key]) == 25 and all(len(row) == 3 for row in command[key])
                for value in (v for row in command[key] for v in row):
                    f32_bits(value)
        cases.append({'feedbackScale': source['feedbackScale'], 'iterations': source['iterations'],
                      'commands': commands})
    assert [case['iterations'] for case in cases] == [8, 8, 12, 12]
    report = json.loads(REPORT.read_bytes())
    assert report['actualNativeSteps'] == 0 and report['predictionNativeSteps'] == 48
    result = {
        'schema': 1,
        'scope': 'Archived expected prefix 1..12 and previously certified commands 12..15; no new native execution',
        'prefixFeedbackScale': 1.0,
        'prefixFrames': prefix,
        'prefixBodyAndMotorBits': [body_motor_bits(frame) for frame in prefix],
        'prefixBodyAndMotorBitLayout': {
            'representation': 'IEEE754 binary32 bits stored as unsigned integers',
            'bodyOrder': 'support.bodies, segment 0..24',
            'bodyFields': ['position_m xyz', 'rotation_xyzw', 'linear_velocity_mps xyz', 'angular_velocity_radps xyz'],
            'motorOrder': 'motors, segment 1..24, then each of six motors in archived array order',
            'motorScalarFields': MOTOR_SCALARS, 'motorVectorFieldsXyz': MOTOR_VECTORS,
            'valuesPerBody': 13, 'valuesPerMotor': 14, 'valuesPerFrame': 2341,
        },
        'cases': cases,
        'sourceSha256': {str(p.relative_to(ROOT)): sha(p.read_bytes()) for p in [STATES, REPORT, INDEX, CERTIFIED]},
        'decompressedStateSha256': sha(raw),
        'sourcePrefixJsonlLines': list(range(25, 37)),
        'allCasePrefixEquivalence': equivalence,
        'comparisonScope': {
            'prefixPhysicalNativeSubsteps': [1, 12],
            'certifiedCommandNativeSubsteps': [12, 15],
            'prefixAllArchivedFieldsAvailable': True,
            'ignoreTraceMetadataFields': ['kind'],
            'newCcdJournalFieldsCompareSeparately': ['ccd', 'ccdJournalCollected'],
            'newOwnerDiagnosticFieldsMustBeExplicitlyAbsentOrNullInPrefix': ['lastAttempt', 'numericAttempt'],
            'exactBitComparisons': 'Use f32 bits for body/motor values and positions/velocities of certified commands; no epsilon.',
            'otherNumericFields': 'Preserve full archived JSON values, including f64 observations and signed zero; no f32 cast for owner.rates or support arithmetic.',
            'physicalHistoryAfter12': 'Unavailable in old B; commands 12..15 are numeric certification only, not physical states or predicted success.',
            'prefixClock': 'Post-step12 has owner.previousStep11, owner.nextSubstep12, snapshot.tick2; configure12 begins tick3.',
        },
        'extractionCounters': {'nativeIntegrations': 0, 'worldConstructions': 0,
                               'fkInspections': 0, 'geometryQueries': 0},
    }
    return result


def main():
    result = prepare()
    encoded = json.dumps(result, separators=(',', ':'), allow_nan=False).encode() + b'\n'
    receipt = {'schema': 1, 'scope': result['scope'], 'expectedSha256': sha(encoded),
               'expectedBytes': len(encoded), 'scriptSha256': sha(Path(__file__).read_bytes()),
               'sourceSha256': result['sourceSha256'],
               'decompressedStateSha256': result['decompressedStateSha256'],
               'prefixFrames': 12, 'prefixPhysicalCasesVerified': 4,
               'certifiedCases': 4, 'commandsPerCase': 4,
               'allCasePrefixEquivalence': result['allCasePrefixEquivalence'],
               'extractionCounters': result['extractionCounters']}
    receipt_bytes = json.dumps(receipt, indent=2, allow_nan=False).encode() + b'\n'
    destination = HERE / 'expected.json'
    receipt_path = HERE / 'extraction-receipt.json'
    if '--check' in sys.argv:
        assert destination.read_bytes() == encoded, 'Expected fixture drift'
        assert receipt_path.read_bytes() == receipt_bytes, 'Receipt drift'
    else:
        destination.write_bytes(encoded)
        receipt_path.write_bytes(receipt_bytes)
    print(json.dumps({k: v for k, v in receipt.items()
                      if k not in ('sourceSha256', 'allCasePrefixEquivalence')}))


if __name__ == '__main__':
    main()
