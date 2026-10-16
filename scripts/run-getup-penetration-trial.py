#!/usr/bin/env python3
"""One separately authorized incremental reservation; never renews old ledgers."""
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    temp = path.with_suffix('.tmp')
    with temp.open('w') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


def admit(ledger):
    assert ledger['status'] == 'prepared', 'Admission already consumed or closed'
    assert ledger['incrementalNativeAllowance'] == 8200
    assert ledger['maximumMainNativeSteps'] == 3200
    assert ledger['maximumTeacherNativeSteps'] == 5000
    assert ledger['incrementalWallAllowanceSeconds'] == 180
    assert ledger['maximumPhysicalHistories'] == ledger['maximumForecastHistories'] == 1
    assert ledger['refund'] is False
    if ledger.get('amendment') == 'recoverable-launcher-counter-retry':
        first_path = REPO / ledger['firstAttemptRecord']
        assert sha(first_path) == ledger['firstAttemptRecordSha256']
        first = json.loads(first_path.read_text())
        assert first['status'] == 'closed'
        assert first['outcome'] == 'closed-harness-counter-failure-before-impact'
        assert first['actualConsumedNativeSteps'] == ledger['actualNativeConsumedBefore'] == 20
        assert first['actualWallSeconds'] == ledger['actualWallConsumedBeforeSeconds']
        assert ledger['remainingNativeSteps'] == 8200 - first['actualConsumedNativeSteps']
        assert ledger['remainingWallSeconds'] == 180 - first['actualWallSeconds']
        assert ledger['retry'] is True
    else:
        assert ledger['retry'] is False
    prior = REPO / ledger['priorClosedRecord']
    assert sha(prior) == ledger['priorClosedRecordSha256']
    previous = json.loads(prior.read_text())
    assert previous['status'] == 'closed'
    assert previous['nativeChargedAfter'] == ledger['historicalReservedNativeSteps'] == 299998
    assert previous['wallChargedAfterSeconds'] == ledger['historicalReservedWallSeconds']
    for path, digest in ledger['inputs'].items():
        assert sha(REPO / path) == digest, 'Frozen input changed: ' + path
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip() == ledger['sourceBase']


def run(path):
    path = path.resolve(strict=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger = json.loads(path.read_text())
        admit(ledger)
        output = REPO / ledger['outputDirectory']
        output.mkdir(exist_ok=True)
        (output / 'started-once').open('x').close()
        started = time.monotonic()
        now = dt.datetime.now(dt.timezone.utc)
        native_budget = ledger.get('remainingNativeSteps', 8200)
        wall_budget = ledger.get('remainingWallSeconds', 180)
        ledger.update(status='running-reservation-retained', startedUtc=now.isoformat(),
                      deadlineUtc=(now + dt.timedelta(seconds=wall_budget)).isoformat(),
                      nativeReservation=native_budget, wallReservationSeconds=wall_budget,
                      cumulativeReservedNativeSteps=299998 + 8200)
        save(path, ledger)
        command = ['node', 'scripts/validate-getup-bounded.mjs', str(path)]
        remaining = max(0, wall_budget - (time.monotonic() - started))
        with (output / 'physical-run.log').open('w') as log:
            child = subprocess.Popen(command, cwd=REPO, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            try:
                code = child.wait(timeout=remaining)
                ledger['processExitCode'] = code
            except subprocess.TimeoutExpired:
                ledger['timedOut'] = True
                code = None
            finally:
                # Kill any remaining browser/worker descendants in this launch.
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait()
        ledger['actualWallSeconds'] = time.monotonic() - started
        ledger['endedUtc'] = dt.datetime.now(dt.timezone.utc).isoformat()
        ledger['status'] = 'closed'
        result_path = output / 'result.json'
        if result_path.exists():
            result = json.loads(result_path.read_text())
            ledger['resultSha256'] = sha(result_path)
            ledger['lastReportedCounters'] = result.get('lastReportedCounters')
            counters = result.get('counters')
            if result.get('harnessCompleted') and code == 0:
                assert 0 <= counters['real'] <= 3200
                assert 0 <= counters['forecast'] <= 5000
                assert counters['total'] == counters['real'] + counters['forecast'] <= native_budget
                ledger['actualConsumedNativeSteps'] = counters['total']
                ledger['actualMainNativeSteps'] = counters['real']
                ledger['actualForecastNativeSteps'] = counters['forecast']
                ledger['outcome'] = result['outcome']
            else:
                ledger['outcome'] = 'closed-uncertain-harness-or-timeout'
                ledger['unobservedStepsChargedConservatively'] = native_budget
        else:
            ledger['outcome'] = 'closed-no-final-diagnostics'
            ledger['unobservedStepsChargedConservatively'] = native_budget
        ledger['cumulativeIncrementalActualWallSeconds'] = ledger.get('actualWallConsumedBeforeSeconds', 0) + ledger['actualWallSeconds']
        if 'actualConsumedNativeSteps' in ledger:
            ledger['cumulativeIncrementalActualNativeSteps'] = ledger.get('actualNativeConsumedBefore', 0) + ledger['actualConsumedNativeSteps']
            assert ledger['cumulativeIncrementalActualNativeSteps'] <= 8200
        assert sha(REPO / ledger['priorClosedRecord']) == ledger['priorClosedRecordSha256']
        save(path, ledger)
        print(json.dumps(ledger, indent=2))


if __name__ == '__main__':
    run(Path(sys.argv[1]))
