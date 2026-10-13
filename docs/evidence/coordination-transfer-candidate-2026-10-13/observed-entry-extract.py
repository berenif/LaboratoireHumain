#!/usr/bin/env python3
"""Extract archived observations only. No model, FK, native world or simulation."""
from pathlib import Path
import csv
import gzip
import hashlib
import json

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
LOCAL = ROOT / 'docs/evidence/coordination-local-pair-2026-10-13'
PROTO = ROOT / 'docs/evidence/coordination-prototype-2026-10-12/native-run'
FEET = (18, 19, 23, 24)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def dump(name, value):
    path = OUT / name
    data = (json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n').encode()
    if path.exists():
        if path.read_bytes() != data:
            raise RuntimeError(f'Refusing to overwrite different extraction: {path}')
    else:
        path.write_bytes(data)
    return {'path': str(path.relative_to(ROOT)), 'bytes': len(data), 'sha256': sha(data)}


def local_rows(raw_path):
    index_bytes = (LOCAL / 'archive-index.json').read_bytes()
    entry = next(e for e in json.loads(index_bytes)['files'] if e['rawPath'] == raw_path)
    parts = []
    sources = []
    for p in sorted(entry['parts'], key=lambda x: x['order']):
        compressed = (LOCAL / p['gzipPath']).read_bytes()
        assert len(compressed) == p['gzipBytes'] and sha(compressed) == p['gzipSha256']
        raw = gzip.decompress(compressed)
        assert len(raw) == p['rawBytes'] and sha(raw) == p['rawSha256']
        parts.append(raw)
        sources.append({'path': str((LOCAL / p['gzipPath']).relative_to(ROOT)),
                        'sha256': sha(compressed), 'bytes': len(compressed)})
    raw = b''.join(parts)
    assert len(raw) == entry['rawBytes'] and sha(raw) == entry['rawSha256']
    return [json.loads(line) for line in raw.splitlines()], {
        'rawPath': str((LOCAL / raw_path).relative_to(ROOT)),
        'rawSha256': sha(raw), 'rawBytes': len(raw),
        'archiveIndexSha256': sha(index_bytes), 'parts': sources,
        'hashesVerified': True,
    }


def proto_rows(raw_path):
    index_bytes = (PROTO / 'archive-index.json').read_bytes()
    entry = next(e for e in json.loads(index_bytes)['files'] if e['original'] == raw_path)
    compressed = (PROTO / entry['archive']).read_bytes()
    assert len(compressed) == entry['archiveBytes'] and sha(compressed) == entry['archiveSha256']
    raw = gzip.decompress(compressed)
    assert len(raw) == entry['originalBytes'] and sha(raw) == entry['originalSha256']
    return [json.loads(line) for line in raw.splitlines()], {
        'rawPath': str((PROTO / raw_path).relative_to(ROOT)),
        'rawSha256': sha(raw), 'rawBytes': len(raw),
        'archiveIndexSha256': sha(index_bytes),
        'parts': [{'path': str((PROTO / entry['archive']).relative_to(ROOT)),
                   'sha256': sha(compressed), 'bytes': len(compressed)}],
        'hashesVerified': True,
    }


def eligible(row):
    diagnostic = row['localDiagnostic']
    support = row['support']
    return (row['nativeSubstep'] > 0 and row['nativeSubstep'] % 4 == 0
            and diagnostic['com_inside'] and diagnostic['capture_inside']
            and all(support['segments'][i]['loaded_substeps'] >= 12
                    and support['segments'][i]['persistent_load_n'] >= 3
                    and any(c['current_upward_load_n'] >= 3 and c['upward_normal'] >= .65
                            for c in support['segments'][i]['contacts']) for i in FEET))


def metrics(row):
    d, s = row['localDiagnostic'], row['support']
    b = s['bodies']
    return {'nativeSubstep': row['nativeSubstep'], 'timeS': s['time_s'],
            'pelvisHeightM': b[0]['position_m'][1],
            'pelvisVerticalVelocityMps': b[0]['linear_velocity_mps'][1],
            'comHeightM': s['center_of_mass_m'][1],
            'comVerticalVelocityMps': s['center_of_mass_velocity_mps'][1],
            'maximumLinearMps': d['maximum_linear_mps'],
            'maximumAngularRadps': d['maximum_angular_radps'],
            'footSupportFraction': d['support_fraction'],
            'comInside': d['com_inside'], 'captureInside': d['capture_inside'],
            'heelLoadsN': d['heel_loads_n'], 'localPassed': d['passed'],
            'fourFeetPersistentEntryPredicate': eligible(row),
            'persistentLoads': [{'segment': x['segment'], 'id': x['id'],
                                 'loadN': x['persistent_load_n'],
                                 'substeps': x['loaded_substeps']}
                                for x in s['segments'] if x['persistent_load_n'] >= 3]}


def main():
    selected_rows, selected_source = local_rows('native-run/001-A.states.jsonl')
    matches = [r for r in selected_rows if eligible(r)]
    assert matches and matches[0]['nativeSubstep'] == 36
    selected = matches[0]
    assert selected['kind'] == 'actual-history'
    owner = selected['owner']
    assert owner['nextSubstep'] == 36 and owner['previousStep'] == 35
    assert owner['last']['commandSubstep'] == 35 and owner['q'] == owner['last']['positions']
    assert len(selected['support']['bodies']) == len(selected['snapshot']['segments']) == 25
    assert len(owner['last']['positions']) == len(owner['last']['velocities']) == 25
    prefix = selected_rows[:36]
    assert [r['nativeSubstep'] for r in prefix] == list(range(1, 37))
    prior = ROOT / 'docs/evidence/coordination-numeric-correction-2026-10-12/archived-step12.json'
    prior_bytes = prior.read_bytes()
    mass_archive = json.loads(prior_bytes)
    program = ROOT / 'docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json'
    tasks = ROOT / 'docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json'
    fixture = {
        'schema': 1, 'scope': 'Observed native A state after 36 integrations; pure archive extraction',
        'status': 'observed-dynamic-entry-only-not-stable-not-native-resume-checkpoint',
        'selectionRule': 'First positive multiple-of-four native boundary of local-pair A with all four foot segments currently loaded >=3 N and persistent >=12 substeps, archived CoM and capture inside. No ranking on future success.',
        'source': selected_source, 'sourceRowOneBased': 36,
        'sourceCanonicalRowSha256': sha(json.dumps(selected, sort_keys=True, separators=(',', ':')).encode()),
        'nativeSubstep': 36, 'physicalPrefixStepsRequiredForExactReplay': 36,
        'physicalPrefixPurpose': 'Original A reference tracking from its analytic seed, not an externally delivered fall or a reset at step 36',
        'headingDegrees': 0, 'measurements': metrics(selected),
        'commandEntry': {'native_substep': 36, 'last_installed_command_substep': 35,
                         'previous_q': owner['last']['positions'],
                         'previous_dq': owner['last']['velocities']},
        'ownerBeforeConfigure36': owner,
        'archivedNativeRow': selected,
        'prefixCommands': [{'nativeSubstepAfterIntegration': r['nativeSubstep'],
                            'commandSubstep': r['owner']['last']['commandSubstep'],
                            'positions': r['owner']['last']['positions'],
                            'velocities': r['owner']['last']['velocities']}
                           for r in prefix],
        'prefixObservationWindow24Through36': [
            {'nativeSubstep': r['nativeSubstep'], 'support': r['support'],
             'localDiagnostic': r['localDiagnostic'], 'persistence': r['persistence'],
             'integratedSupportLoadsN': r['integratedSupportLoadsN'],
             'integratedSupportSubsteps': r['integratedSupportSubsteps']}
            for r in prefix[23:]],
        'relatedMassPropertyArchive': {
            'path': str(prior.relative_to(ROOT)), 'sha256': sha(prior_bytes),
            'localMassProperties': mass_archive['archivedLocalMassProperties'],
            'sourceWorlds': mass_archive['archivedLocalMassPropertiesSources'],
            'scope': 'Existing original-prototype native local mass properties only; not remeasured in A36',
            'bodyWorldCenterOfMassAtEntry': None,
            'bodyNativeMassKgAtEntry': None,
        },
        'referenceCandidate': {'programPath': str(program.relative_to(ROOT)),
                               'programSha256': sha(program.read_bytes()),
                               'tasksPath': str(tasks.relative_to(ROOT)),
                               'tasksSha256': sha(tasks.read_bytes()),
                               'referenceRecenterHoldCompletedSubstep': 432,
                               'referenceStandingSubstep': 1680,
                               'referenceEndSubstep': 1920,
                               'scope': 'Unchanged eight-second candidate; new raccord and physical admission pending'},
        'missingForExactWorldResume': ['Rapier world/cache snapshot at step36',
                                      'cached body world centers of mass at step36',
                                      'complete original recovery-controller private memory',
                                      'complete original stepping/allocator private memory'],
        'measuredJointCoordinates': None, 'measuredJointCoordinateRates': None,
        'noResetPermission': True, 'nativeExecutionAuthorized': False,
        'newNativeSteps': 0, 'newFkEvaluations': 0, 'newWorlds': 0,
        'caveats': [
            'previous_q/dq are installed target commands, not measured physical joint coordinates or velocities.',
            'The observation is after native step36 and before configure(36); snapshot.stamp.tick is still8 and contact_time_s is0.13333333333333333. Do not replace nativeSubstep36/time_s0.15 with the coarser snapshot stamp.',
            'owner.enabled=false and owner.patches are empty because this is A. Do not invent historical B patches from these contacts.',
            'All original poses, velocities and material contacts are copied. No angles, body CoM cache or FK is reconstructed.',
            'Exact future continuation requires physically replaying the fixed36-step prefix and comparing it, not restoring these poses into an empty world.',
            'No posterior heel load and no stable interval are present. The state is a dynamic shallow crouch, not an acquired standing support.',
        ],
    }
    fixture_info = dump('observed-entry-step36.json', fixture)
    scans = []
    for label, get, path, kind in [
        ('local-pair A', local_rows, 'native-run/001-A.states.jsonl', 'actual history'),
        ('prototype A', proto_rows, '001-A.states.jsonl', 'actual history'),
        ('local-pair B', local_rows, 'native-run/002-B.states.jsonl', 'prediction copies; B real zero'),
        ('prototype B', proto_rows, '002-B.states.jsonl', 'prediction copies; B real zero'),
        ('local-pair verification', local_rows, 'native-run/000-native-verification.states.jsonl', 'technical admission'),
        ('prototype verification', proto_rows, '000-native-verification.states.jsonl', 'technical admission'),
    ]:
        rows, source = (selected_rows, selected_source) if label == 'local-pair A' else get(path)
        matched = [r for r in rows if eligible(r)]
        milestones = [metrics(r) for r in rows if r['nativeSubstep'] in
                      {12,16,24,32,35,36,39,40,44,48,96,240,300,360,480,960}]
        scans.append({'label': label, 'kind': kind, 'source': source, 'rows': len(rows),
                      'maximumNativeSubstep': max(r['nativeSubstep'] for r in rows),
                      'entryPredicateMatches': [r['nativeSubstep'] for r in matched],
                      'minimumPelvisHeightM': min(r['support']['bodies'][0]['position_m'][1] for r in rows),
                      'localPassedRows': sum(r['localDiagnostic']['passed'] for r in rows),
                      'milestones': milestones})
    reports = []
    for rel in ['docs/evidence/foot-coordination-2026-10-12/candidate-report.json',
                'docs/evidence/foot-coordination-2026-10-12/witness-report.json',
                'docs/getup-ab-results-2026-10-12.json',
                'docs/recovery-foot-support-investigation-2026-10-12.json',
                'docs/recovery-physical-support-results-2026-10-11.json']:
        p = ROOT / rel
        data = p.read_bytes()
        d = json.loads(data)
        summaries = []
        for key in ('initial', 'final'):
            if key in d and 'segments' in d[key]:
                state = d[key]
                summaries.append({'name': key, 'simulationTimeS': state['simulation_time_s'],
                                  'pelvisHeightM': state['segments'][0]['position']['y'],
                                  'posesCount': len(state['segments']),
                                  'contacts': state['contacts']})
        reports.append({'path': rel, 'sha256': sha(data), 'bytes': len(data),
                        'snapshotSummaries': summaries,
                        'scope': 'Historical report only; no continuous rich native state trace available here'})
    scan = {'schema': 1, 'scope': 'Read-only archive inventory and scalar selection; zero FK/native/world',
            'selected': fixture_info, 'selectionFixedByParent': True,
            'selectionNativeSubstep': 36, 'scan': scans, 'historicalReports': reports,
            'missingHistoricalRawDirectory': 'evidence/',
            'excluded': ['Static continuation geometry is not physical evidence',
                         'Launcher-portability fake traces are OS tests, not physical evidence',
                         'Numeric parity traces cover only the initial16 native states, not a later crouch'],
            'newNativeSteps': 0, 'newFkEvaluations': 0, 'newWorlds': 0}
    scan_info = dump('observed-entry-scan.json', scan)
    receipt = {'schema': 1, 'extractorSha256': sha(Path(__file__).read_bytes()),
               'outputs': [fixture_info, scan_info], 'sourceHashChecks': 'passed',
               'selectionRuleCheck': 'first eligible tick boundary is36',
               'nativeSteps': 0, 'fkEvaluations': 0, 'worldConstructions': 0}
    dump('observed-entry-receipt.json', receipt)
    print(json.dumps({'selected': fixture_info, 'scan': scan_info,
                      'measurements': metrics(selected)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
