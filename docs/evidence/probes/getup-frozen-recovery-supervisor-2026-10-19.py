import subprocess,pathlib,time,json,datetime,hashlib
D=pathlib.Path('outputs/getup-recovery-root-20261019');binary=D/'frozen-recovery-replay';results=[];began=time.monotonic()
for kind in ['back','belly','side','side-supported']:
 out=D/kind;out.mkdir(exist_ok=False);entry={'kind':kind,'startedUtc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'operationalHangTimeoutSeconds':300,'status':'running'};results.append(entry);(D/'process-ledger.json').write_text(json.dumps({'cases':results},indent=2)+'\n');t=time.monotonic()
 with (out/'stdout.log').open('w') as stdout,(out/'stderr.log').open('w') as stderr:
  try:
   p=subprocess.run([str(binary),kind,str(out)],stdout=stdout,stderr=stderr,timeout=300);entry['exitCode']=p.returncode;entry['status']='terminal'
  except subprocess.TimeoutExpired:entry['status']='interrupted-operational-timeout';entry['unreceiptedActualWork']='Unknown after last complete trace receipt, potentially including synchronous WEAK forecast; not success'
 entry['wholeProcessWallSeconds']=time.monotonic()-t;entry['finishedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
 p=out/'result.json'
 if p.exists():
  result=json.loads(p.read_text());entry['resultSha256']=hashlib.sha256(p.read_bytes()).hexdigest();entry['outcome']=result['outcome'];entry['realNativeIntegrations']=result['realNativeIntegrations'];entry['forecastNativeIntegrations']=result['forecastNativeIntegrations'];entry['standingPassed']=result['standingPassed']
 print(json.dumps(entry),flush=True);(D/'process-ledger.json').write_text(json.dumps({'cases':results,'totalWholeProcessWallSeconds':time.monotonic()-began},indent=2)+'\n')
