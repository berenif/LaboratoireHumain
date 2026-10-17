#!/usr/bin/env python3
"""Extract archived values only; no simulation, FK, or geometric computation."""
import hashlib
import json
from pathlib import Path
import struct

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OLD = ROOT / 'docs/evidence/coordination-prototype-2026-10-12'
PLAN = OLD / 'native-run/002-B-planning-000.json'
STATES = OLD / 'native-run/002-B.states.jsonl'
REPORT = OLD / 'native-run/002-B.json'
MODEL = ROOT / 'rust/crates/model/data/canonical-v1.json'
PROGRAM = OLD / 'continuation-candidate-program.json'
TASKS = OLD / 'continuation-tasks.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def f32_bits(values):
    return [struct.unpack('<I', struct.pack('<f', x))[0] for x in values]


def pose_values(p):
    return ([p['position'][k] for k in 'xyz'] + [p['rotation'][k] for k in 'xyzw']
            + [p['linearVelocity'][k] for k in 'xyz']
            + [p['angularVelocity'][k] for k in 'xyz'])


def body_values(b):
    return (b['pos']['position']['translation'] + b['pos']['position']['rotation']
            + b['vels']['linvel'] + b['vels']['angvel'])


def main():
    plan = json.loads(PLAN.read_text())
    report = json.loads(REPORT.read_text())
    reference = json.loads(TASKS.read_text())
    model = json.loads(MODEL.read_text())
    all_rows = [json.loads(line) for line in STATES.read_text().splitlines()]
    rows = [row for row in all_rows if row['nativeSubstep'] == 12]
    assert len(rows) == len(plan['cases']) == 4
    assert report['headingDegrees'] == 0 and report['actualNativeSteps'] == 0
    base = rows[0]
    physical_keys = [k for k in base if k not in ('kind', 'owner')]
    owner_common = {k: v for k, v in base['owner'].items() if k != 'feedbackScale'}
    physical = {k: base[k] for k in physical_keys}
    for i, row in enumerate(rows):
        assert row['kind'] == f'preview-0-{i}'
        assert digest({k: row[k] for k in physical_keys}) == digest(physical)
        assert digest({k: v for k, v in row['owner'].items() if k != 'feedbackScale'}) == digest(owner_common)
        assert row['snapshot']['segments'] == plan['cases'][i]['terminalState']['segments']
    assert base['owner']['previousStep'] == 11 and base['owner']['nextSubstep'] == 12
    assert base['owner']['patches']['active'] == []
    assert len(base['support']['bodies']) == len(base['snapshot']['segments']) == 25

    # The archived parity worlds contain native local inertial properties.
    # Match bodies by their existing bit-exact recorded poses, never by FK.
    inertials = None
    inertial_sources = []
    for case_index, case in enumerate(plan['cases']):
        for frame in case['firstPredictedSteps']['frames']:
            world = json.loads(bytes(frame['world']))
            state = json.loads(bytes(frame['state']))
            native_bodies = [(i, item['Occupied']['value'])
                             for i, item in enumerate(world['bodies']['bodies']['items'])
                             if 'Occupied' in item]
            matched = []
            for segment, pose in enumerate(state['snapshot']['segments']):
                key = f32_bits(pose_values(pose))
                candidates = [(i, b) for i, b in native_bodies if f32_bits(body_values(b)) == key]
                assert len(candidates) == 1
                arena_index, body = candidates[0]
                matched.append({'segment': segment, 'id': pose['id'],
                                'bodyArenaIndex': arena_index,
                                'localMassProperties': body['mprops']['local_mprops']})
            if inertials is None:
                inertials = matched
                native_dt = world['integration_parameters']['dt']
            assert digest(matched) == digest(inertials)
            inertial_sources.append({'caseIndex': case_index, 'nativeSubstep': frame['native_substep'],
                                     'worldBytesSha256': hashlib.sha256(bytes(frame['world'])).hexdigest()})

    cases = []
    for index, (case, row) in enumerate(zip(plan['cases'], rows)):
        terminal = case['terminalOwner']
        assert terminal['q'] == row['owner']['q']
        assert terminal['nextSubstep'] == 12 and terminal['previousStep'] == 11
        rate_changed = terminal['rates'] != row['owner']['rates']
        assert rate_changed == (index == 0)
        cases.append({'caseIndex': index, 'previewKind': row['kind'],
                      'sourceJsonlLine': 12 * (index + 1),
                      'feedbackScale': case['feedbackScale'],
                      'actualNativeStepsInPreview': case['actualNativeSteps'],
                      'usable': case['usable'], 'firstFailure': case['firstFailure'],
                      'refusal': case['refusal'], 'score': case['score'],
                      'terminalDiagnostic': case['terminalDiagnostic'],
                      'terminalOwner': terminal,
                      'candidateF64RatesWrittenBeforeRefusal': rate_changed,
                      'rejectedF64Rates': terminal['rates'] if rate_changed else None,
                      'physicalStateSha256CanonicalJson': digest(physical),
                      'motorsSha256CanonicalJson': digest(row['motors'])})

    artifact = {
        'schemaVersion': 1,
        'scope': 'Archived B step 12 before configure(12), with post-refusal owner evidence; no actual B history',
        'nativeSubstep': 12, 'headingDegrees': report['headingDegrees'],
        'nativeDtSAsArchivedF32Json': native_dt, 'nativeDtF32Bits': f32_bits([native_dt])[0],
        'tickSubsteps': 4, 'actualBHistoryNativeSteps': 0,
        'sourceSha256': {str(p.relative_to(ROOT)): sha(p) for p in [PLAN, STATES, REPORT, MODEL, PROGRAM, TASKS]},
        'physical': physical,
        'ownerBeforeConfigure': base['owner'],
        'ownerBeforeConfigurePerCaseRule': 'Clone ownerBeforeConfigure and replace feedbackScale with cases[i].feedbackScale only.',
        'cases': cases,
        'archivedLocalMassProperties': inertials,
        'archivedLocalMassPropertiesSources': inertial_sources,
        'modelSegmentAttributes': [{k: v for k, v in segment.items() if k not in ('geometry', 'initial')}
                                   for segment in model['segments']],
        'commandProgram': json.loads(PROGRAM.read_text()),
        'referenceRow': next(row for row in reference['rows'] if row['substep'] == 12),
        'materialTargets': reference['materialTargets'],
        'extractionChecks': {
            'allFourCompletePhysicalRecordsIdentical': True,
            'allFourMotorRecordsIdentical': True,
            'allFourOwnerPrefixesIdenticalExceptFeedbackScale': True,
            'allFourTerminalPoseRecordsEqualTheirStep12Archive': True,
            'all16ArchivedLocalMassPropertyRecordsIdentical': True,
            'archivedInertialBodiesMatchedByExactF32PoseAndVelocityBits': True,
            'ownerCoordinatesUnchangedAtRefusal': True,
            'onlyCase0StoredNewF64CandidateRatesBeforeRefusal': True,
            'noNewSimulationSteps': 0, 'noNewFkInspections': 0, 'noNewGeometryQueries': 0,
        },
        'caveats': [
            'terminalOwner has patches/timers already mutated by the refused configure(12); replay must start from ownerBeforeConfigure.',
            'rejectedF64Rates is null for scales 0.5/1/1.5 because their solve returned before memory.rates assignment; no rejected solution vector was archived.',
            'Native local mass properties come from existing steps 1..4 and are identical across all 16 archived worlds; world CoM at step 12 is not reconstructed here.',
            'snapshot stamp tick is 2 at native step 12; the rejected next configure reports tick 3 substep 0. Do not confuse these clocks.',
            'All floats are copied from archives. Canonical JSON SHA checks distinguish signed zero; f32 bits are used only to match archived body identities.',
        ],
    }
    destination = HERE / 'archived-step12.json'
    destination.write_text(json.dumps(artifact, separators=(',', ':'), allow_nan=False) + '\n')
    receipt = {'artifact': destination.name, 'sha256': sha(destination),
               'bytes': destination.stat().st_size, 'scriptSha256': sha(Path(__file__)),
               'sourceSha256': artifact['sourceSha256'], 'checks': artifact['extractionChecks']}
    (HERE / 'archived-step12-extraction-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({k: v for k, v in receipt.items() if k != 'sourceSha256'}))


if __name__ == '__main__':
    main()
