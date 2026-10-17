#!/usr/bin/env python3
"""Thin single-shot launcher for the existing worker and its integration ledger."""
import datetime, hashlib, json, os, selectors, subprocess, time, uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name, value):
    with (HERE/name).open('x') as f:
        json.dump(value,f,indent=2,ensure_ascii=False,allow_nan=False); f.write('\n'); f.flush(); os.fsync(f.fileno())
prior=ROOT/'docs/evidence/coordination-exploration-execution-2026-10-15/closed.json'
assert sha(prior)=='057f5de537cdd65a38489af8f9d390677af2998f583432b581761665b38f1f5e'
previous=json.loads(prior.read_text())
assert previous['status']=='closed' and previous['workerResult']['actualNativeSteps']==6240
assert previous['maximumNativeStepsCharged']==7000 and previous['wallSecondsCharged']==120
# There must be no native worker already alive. Read process executable metadata only.
for entry in Path('/proc').iterdir():
    if entry.name.isdigit():
        try: name=(entry/'exe').resolve().name
        except OSError: continue
        assert not name.startswith('coordination_transfer_worker'), f'Existing native worker {entry.name}'
binaries=list(Path('/workspace/lh-feedback-target/release/deps').glob('coordination_transfer_worker-*'))
binary=next(p for p in binaries if p.is_file() and os.access(p,os.X_OK) and p.suffix!='.d')
inputs=json.loads((ROOT/'docs/evidence/coordination-contact-diagnostic-2026-10-15/manifest.json').read_text())['inputs']
operation=str(uuid.uuid4())
start=time.monotonic(); utc=time.time()
claim={'operationId':operation,'status':'reserved-before-spawn','startedUtc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'maximumNativeSteps':7000,'maximumWallSeconds':120,'maximumHistorySteps':6240,'histories':1,'forecasts':0,'retries':0,
 'nativeChargedBefore':146508,'nativeChargedAfter':153508,'wallChargedBeforeSeconds':346.328294294003192,
 'wallChargedAfterSeconds':466.328294294003192,'noRefund':True,'binary':str(binary),'binarySha256':sha(binary),
 'inputHashes':{v:sha(v) for v in inputs.values()},'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()}
# Exclusive durable claim also prevents a duplicate launch after timeout or failure.
save('launch-claim.json',claim)
process=None; result={}; error=None
try:
    with (HERE/'worker.stderr.log').open('xb') as stderr:
        process=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr,cwd=ROOT)
        selector=selectors.DefaultSelector(); selector.register(process.stdout,selectors.EVENT_READ)
        if not selector.select(min(10,113-(time.monotonic()-start))): raise TimeoutError('Worker readiness timeout')
        ready=json.loads(process.stdout.readline()); assert ready['event']=='ready' and ready['pid']==process.pid
        message={'op':'feedback-run','operationId':operation,'inputs':inputs,
          'outputPath':str(HERE/'native-result.json'),'tracePath':str(HERE/'native-trace.jsonl'),
          'permit':{'reservationId':operation,'reservedSteps':7000,'workerPid':process.pid,
          'deadlineUnixS':utc+113,'remainingWallSeconds':113-(time.monotonic()-start)}}
        save('request.json',message)
        print('NATIVE HISTORY START '+datetime.datetime.now(datetime.timezone.utc).isoformat(),flush=True)
        output,_=process.communicate((json.dumps(message)+'\n').encode(),timeout=max(.01,113-(time.monotonic()-start)))
        (HERE/'worker.stdout.jsonl').write_bytes(output)
        if process.returncode!=0: raise RuntimeError(f'Worker exit {process.returncode}')
        result=json.loads(output); assert result['op']=='feedback-run' and result['operationId']==operation
except BaseException as exc:
    error=repr(exc)
finally:
    if process is not None and process.poll() is None:
        process.terminate()
        try: process.wait(timeout=min(2,max(.01,119-(time.monotonic()-start))))
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=1)
    elapsed=time.monotonic()-start
    accounting=HERE/'native-result.json.accounting.jsonl'
    last=json.loads(accounting.read_text().splitlines()[-1]) if accounting.exists() else None
    save('closed.json',{'operationId':operation,'status':'closed','elapsedSeconds':elapsed,'error':error,
      'workerResult':result,'lastAccounting':last,'maximumNativeStepsCharged':7000,'wallSecondsCharged':120,
      'retries':0,'pid':process.pid if process else None,'returncode':process.returncode if process else None,
      'priorLedgerUnchanged':sha(prior)==claim.get('priorLedgerSha256', '057f5de537cdd65a38489af8f9d390677af2998f583432b581761665b38f1f5e')})
    print(json.dumps({'elapsedSeconds':elapsed,'error':error,'result':result}),flush=True)
