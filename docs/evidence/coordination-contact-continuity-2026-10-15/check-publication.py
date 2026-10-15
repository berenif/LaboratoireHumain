#!/usr/bin/env python3
"""Check static evidence hashes and recorded test outcomes; never run simulation."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(name):
    return json.loads((HERE / name).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


checks = []


def check_hash(path, expected, category):
    file = ROOT / path
    actual = digest(file)
    assert actual == expected, (str(path), expected, actual)
    checks.append({'path': str(path), 'sha256': actual, 'category': category})


audit = read('final-budget-and-integrity.json')
for row in audit['immutableEvidence']['checks']:
    check_hash(row['path'], row['expectedSha256'], 'closed-physical-evidence')
assert len(checks) == 42
review = read('implementation-review.json')
for row in review['sources']:
    check_hash(row['path'], row['sha256'], 'independently-reviewed-source')
assert len(review['sources']) == 18
figure = read('continuity-proof-receipt.json')
for group in ('sourcesSha256', 'outputsSha256'):
    for path, sha in figure[group].items():
        target = HERE / path if group == 'outputsSha256' else Path(path)
        check_hash(target, sha, 'static-figure')

outcomes = {}
passes = []
for name in ('first-pure-tests/receipt.json', 'fixture-fix-pure-tests/receipt.json'):
    receipt = read(name)
    assert receipt['newNativeSteps'] == 0
    assert receipt['newFkOrWorldInspections'] == 0
    phase = {}
    for run in receipt['runs']:
        check_hash(run['log'], run['logSha256'], 'recorded-pure-test-log')
        log = (ROOT / run['log']).read_text()
        found = re.findall(r'^test (\S+) \.\.\. (ok|FAILED)$', log, re.M)
        assert len(found) == run['expectedTests']
        for test, result in found:
            assert test not in phase
            phase[test] = result
    assert set(phase) == set(receipt['selectedTestNames'])
    passes.append({'receipt': name, 'passed': sum(v == 'ok' for v in phase.values()),
                   'failed': sum(v == 'FAILED' for v in phase.values())})
    outcomes.update(phase)
assert len(outcomes) == 82 and set(outcomes.values()) == {'ok'}
assert [(p['passed'], p['failed']) for p in passes] == [(79, 3), (3, 0)]

builds = ['test-compile-fixture-fix.log', 'production-check.log',
          'worker-debug-build-only.log']
for name in builds:
    assert re.search(r'Finished `(?:test|dev)` profile', (HERE / name).read_text()), name

report = ROOT / 'docs/coordination-contact-continuity-2026-10-15.md'
links = []
for target in re.findall(r'\]\(([^)]+)\)', report.read_text()):
    if '://' not in target:
        path = (report.parent / target).resolve()
        assert path.is_file(), target
        links.append(target)

diff_check = subprocess.run(['git', 'diff', '--cached', '--check'], cwd=ROOT,
                            text=True, capture_output=True)
assert not diff_check.stderr, diff_check.stderr
raw_log_whitespace = diff_check.stdout.splitlines()
assert all(re.fullmatch(r'docs/evidence/coordination-contact-continuity-2026-10-15/'
                       r'(?:first-pure-tests|fixture-fix-pure-tests)/[^:]+\.log:'
                       r'\d+: new blank line at EOF\.', line)
           for line in raw_log_whitespace), raw_log_whitespace
authored_check = subprocess.run(['git', 'diff', '--cached', '--check', '--', '.',
                                ':(exclude)**/*.log'], cwd=ROOT, text=True,
                               capture_output=True)
assert authored_check.returncode == 0, authored_check.stdout + authored_check.stderr

tracked = [p for p in HERE.rglob('*') if p.is_file()
           and p.name != 'publication-check.json']
tracked += [report, ROOT / 'TODO.md']
inventory = [{'path': str(p.relative_to(ROOT)), 'bytes': p.stat().st_size,
              'sha256': digest(p)} for p in sorted(tracked)]
result = {
    'schema': 'static-publication-check.v1',
    'checkedUtc': datetime.now(timezone.utc).isoformat(),
    'sourceBaseCommit': subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'scope': 'Read-only file/Git checks and parsing existing test logs; no test or native execution',
    'newNativeSteps': 0, 'newWorlds': 0, 'newFkInspections': 0,
    'checkedHashes': checks,
    'recordedPureTestPasses': passes,
    'distinctTestsUltimatelyPassed': len(outcomes),
    'successfulBuildLogs': builds,
    'validReportLinks': links,
    'authoredStagedWhitespaceCheck': 'PASS',
    'preservedRawLogWhitespace': raw_log_whitespace,
    'artifactInventory': inventory,
}
(HERE / 'publication-check.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'hashChecks': len(checks), 'distinctTestsUltimatelyPassed': 82,
                  'artifacts': len(inventory), 'newNativeSteps': 0}))
