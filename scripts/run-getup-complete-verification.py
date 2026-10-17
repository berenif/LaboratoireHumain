#!/usr/bin/env python3
"""One deterministic replay; actual180s ceiling retained, planning-step cap amended."""
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(path, value):
    temp = path.with_suffix('.tmp')
    with temp.open('w') as file:
        json.dump(value, file, indent=2); file.write('\n'); file.flush(); os.fsync(file.fileno())
    os.replace(temp, path)


def admit(record):
    assert record['status'] == 'prepared'
    assert record['maximumAdditionalNativeSteps'] == 10000
    assert record['userCumulativeWallLimitSeconds'] == 180
    prior_path = REPO/record['priorAttemptRecord']
    assert sha(prior_path) == record['priorAttemptRecordSha256']
    prior = json.loads(prior_path.read_text())
    assert prior['status'] == 'closed'
    assert prior['cumulativeIncrementalActualNativeSteps'] == 3760
    if record.get('recoverableCheckpointRetry'):
        failed_path = REPO/record['failedReplayRecord']
        assert sha(failed_path) == record['failedReplayRecordSha256']
        failed = json.loads(failed_path.read_text())
        assert failed['status'] == 'closed' and failed['lastCounters']['total'] == 1600
        assert record['actualNativeConsumedBefore'] == 3760+1600
        assert record['actualWallConsumedBeforeSeconds'] == failed['cumulativeActualWallSeconds']
        assert record['remainingNativeForCorrectedLaunch'] == 10000-1600
    else:
        assert record['actualNativeConsumedBefore'] == 3760
        assert record['actualWallConsumedBeforeSeconds'] == prior['cumulativeIncrementalActualWallSeconds']
    assert record['remainingWallSeconds'] == 180-record['actualWallConsumedBeforeSeconds']
    assert record['amendedPlanningCumulativeNativeCeiling'] == 3760+10000
    assert record['maximumRealDurationS'] == 10 and record['maximumCandidateReplays'] == 1
    preflight_path = REPO/record['preflightFile']
    assert sha(preflight_path) == record['preflightSha256']
    preflight = json.loads(preflight_path.read_text())
    assert preflight['passed'] and preflight['physicsIntegrations'] == 0
    assert preflight['anticipatedWallFromPriorSeconds'] < record['remainingWallSeconds']
    for path, digest in record['immutablePriorRecords'].items(): assert sha(REPO/path)==digest
    for path, digest in record['inputs'].items(): assert sha(REPO/path)==digest, path
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip() == record['sourceBase']


def run(path):
    path = path.resolve(strict=True)
    record = json.loads(path.read_text()); admit(record)
    output = REPO/record['outputDirectory']
    (output/'launch-started-once').open('x').close()
    began = time.monotonic(); now = dt.datetime.now(dt.timezone.utc)
    limit = record['remainingWallSeconds']
    record.update(status='running', startedUtc=now.isoformat(), deadlineUtc=(now+dt.timedelta(seconds=limit)).isoformat())
    save(path, record)
    command = ['node','scripts/complete-getup-verification.mjs',str(path),str(output)]
    with (output/'physical-run.log').open('w') as log:
        child = subprocess.Popen(command,cwd=REPO,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            # Conservative100ms shutdown margin under the unchanged hard cap.
            record['processExitCode'] = child.wait(timeout=max(0,limit-(time.monotonic()-began)-.1))
        except subprocess.TimeoutExpired:
            record['timedOut'] = True
        finally:
            try: os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError: pass
            child.wait()
    record['actualWallSeconds'] = time.monotonic()-began
    record['cumulativeActualWallSeconds'] = record['actualWallConsumedBeforeSeconds']+record['actualWallSeconds']
    record.update(status='closed',endedUtc=dt.datetime.now(dt.timezone.utc).isoformat())
    result_path = output/'result.json'
    if result_path.exists():
        result=json.loads(result_path.read_text()); record['resultSha256']=sha(result_path)
        record['lastCounters']=result['counters']
        if result.get('harnessCompleted') and not record.get('timedOut'):
            count=result['counters']['total']; assert count<=record.get('remainingNativeForCorrectedLaunch',10000)
            record['actualAdditionalNativeSteps']=count
            record['cumulativeActualNativeSteps']=record['actualNativeConsumedBefore']+count
            record['outcome']=result['outcome']
        else:
            record['outcome']='closed-incomplete-diagnostics-retained'
    else: record['outcome']='closed-without-final-report'
    for p,h in record['immutablePriorRecords'].items(): assert sha(REPO/p)==h
    save(path,record)
    print(json.dumps(record,indent=2))


if __name__=='__main__': run(Path(sys.argv[1]))
