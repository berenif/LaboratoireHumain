#!/usr/bin/env python3
"""One corrected side fixture, strictly inside the unused part of the180s batch."""
import datetime as dt
import hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
REPO=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(path,r):
    tmp=path.with_suffix('.tmp')
    with tmp.open('w') as f:json.dump(r,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)
def run(path):
    path=path.resolve(strict=True);r=json.loads(path.read_text())
    assert r['status']=='prepared' and r['maximumAttempts']==1 and r['case']=='side-supported'
    prior=REPO/r['priorLedger'];assert sha(prior)==r['priorLedgerSha256']
    previous=json.loads(prior.read_text());remaining=180-previous['actualBatchWallSeconds']
    assert r['maximumActualPhysicalWallSeconds']<=remaining
    assert r['priorActualPhysicalWallSeconds']==previous['actualBatchWallSeconds']
    assert json.loads((REPO/r['preflightFile']).read_text())['physicsIntegrations']==0
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()==r['sourceBase']
    for p,h in r['inputs'].items():assert sha(REPO/p)==h,p
    out=REPO/r['outputDirectory'];(out/'amendment-started-once').open('x').close();caseout=out/r['case'];caseout.mkdir()
    began=time.monotonic();cap=r['maximumActualPhysicalWallSeconds']
    r.update(status='running',startedUtc=dt.datetime.now(dt.timezone.utc).isoformat(),entries=[dict(case=r['case'],status='running',
        deadlineUtc=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=cap)).isoformat())]);save(path,r)
    with (caseout/'physical-run.log').open('w') as log:
        child=subprocess.Popen(['node','scripts/getup-three-poses.mjs',str(path),r['case'],str(caseout)],cwd=REPO,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:r['processExitCode']=child.wait(timeout=max(0,cap-(time.monotonic()-began)-.15))
        except subprocess.TimeoutExpired:r['timedOut']=True
        finally:
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            child.wait()
    actual=time.monotonic()-began;r.update(status='closed',actualPhysicalWallSeconds=actual,
        cumulativeBatchPhysicalWallSeconds=r['priorActualPhysicalWallSeconds']+actual,
        unusedBatchPhysicalWallSeconds=180-r['priorActualPhysicalWallSeconds']-actual,
        endedUtc=dt.datetime.now(dt.timezone.utc).isoformat());r['entries'][0]['status']='closed'
    resultpath=caseout/'result.json'
    if resultpath.exists():
        result=json.loads(resultpath.read_text());r['resultSha256']=sha(resultpath);r['outcome']=result.get('outcome','incomplete')
        r['lastObservedCounters']=result['counters'];r['fixtureValid']=result['fixtureValid'];r['standingPassed']=result['standingPassed']
        r['actualNativeStepsKnown']=result['counters']['total'] if result.get('harnessCompleted') and not r.get('timedOut') else None
    assert sha(prior)==r['priorLedgerSha256'];save(path,r)
    print(json.dumps({k:v for k,v in r.items() if k not in ['inputs','entries']},indent=2))
if __name__=='__main__':run(Path(sys.argv[1]))
