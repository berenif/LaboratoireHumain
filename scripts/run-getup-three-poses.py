#!/usr/bin/env python3
"""One three-case batch, actual elapsed computation charged, global180s ceiling."""
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def save(path,value):
    temp=path.with_suffix('.tmp')
    with temp.open('w') as file:
        json.dump(value,file,indent=2);file.write('\n');file.flush();os.fsync(file.fileno())
    os.replace(temp,path)

def admit(record):
    assert record['status']=='prepared'
    assert record['cases']==['back','belly','side']
    assert record['maximumActualPhysicalWallSeconds']==180
    assert record['maximumPreparationNativeStepsPerCase']==1600
    assert record['maximumRecoveryNativeStepsPerCase']==20000
    assert record['maximumTeacherNativeStepsPerCase']==5000
    assert record['maximumAttemptsPerCase']==1
    assert record['chargeUnusedReservationsAsActual'] is False
    assert json.loads((REPO/record['preflightFile']).read_text())['physicsIntegrations']==0
    for p,h in record['inputs'].items():assert sha(REPO/p)==h,p
    for p,h in record['immutablePriorRecords'].items():assert sha(REPO/p)==h,p
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()==record['sourceBase']

def run(path):
    path=path.resolve(strict=True);r=json.loads(path.read_text());admit(r)
    out=REPO/r['outputDirectory'];(out/'batch-started-once').open('x').close()
    began=time.monotonic();r.update(status='running',startedUtc=dt.datetime.now(dt.timezone.utc).isoformat(),entries=[])
    save(path,r)
    for index,kind in enumerate(r['cases']):
        remaining=180-(time.monotonic()-began)
        if remaining<=3:
            r['notStartedCases']=r['cases'][index:];break
        allocation=remaining/(len(r['cases'])-index)
        caseout=out/kind;caseout.mkdir();entry=dict(case=kind,status='running',maximumWallSeconds=allocation,
            startedUtc=dt.datetime.now(dt.timezone.utc).isoformat(),
            deadlineUtc=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=allocation)).isoformat())
        r['entries'].append(entry);save(path,r)
        for p,h in r['inputs'].items():assert sha(REPO/p)==h,p
        started=time.monotonic()
        with (caseout/'physical-run.log').open('w') as log:
            child=subprocess.Popen(['node','scripts/getup-three-poses.mjs',str(path),kind,str(caseout)],cwd=REPO,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:entry['processExitCode']=child.wait(timeout=max(0,min(allocation,180-(time.monotonic()-began))-.1))
            except subprocess.TimeoutExpired:entry['timedOut']=True
            finally:
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                child.wait()
        entry.update(status='closed',actualWallSeconds=time.monotonic()-started)
        resultpath=caseout/'result.json'
        if resultpath.exists():
            result=json.loads(resultpath.read_text());entry['resultSha256']=sha(resultpath)
            entry['lastObservedNativeCounters']=result['counters'];entry['outcome']=result.get('outcome','incomplete')
            entry['fixtureValid']=result['fixtureValid'];entry['standingPassed']=result['standingPassed']
            if result.get('harnessCompleted') and not entry.get('timedOut'):
                entry['actualNativeStepsKnown']=result['counters']['total']
            else:entry['actualNativeStepsKnown']=None
        save(path,r)
    r.update(status='closed',actualBatchWallSeconds=time.monotonic()-began,
        actualCaseWallSeconds=sum(e['actualWallSeconds'] for e in r['entries']),
        endedUtc=dt.datetime.now(dt.timezone.utc).isoformat())
    for p,h in r['immutablePriorRecords'].items():assert sha(REPO/p)==h,p
    save(path,r);print(json.dumps(r,indent=2))

if __name__=='__main__':run(Path(sys.argv[1]))
