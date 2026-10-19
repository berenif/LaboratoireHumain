import subprocess,pathlib,time,json,datetime,hashlib,sys
D=pathlib.Path('outputs/getup-terminal-balance-20261019'); binary=pathlib.Path('rust/target/release/examples/getup_terminal_recovery');ledger=D/'process-ledger.json';results=json.loads(ledger.read_text())['cases'] if ledger.exists() else []
for kind in sys.argv[1:] or ['back','belly','side','side-supported']:
 out=D/kind;out.mkdir(exist_ok=False);entry={'kind':kind,'startedUtc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'operationalHangTimeoutSeconds':300,'status':'running'};results.append(entry);ledger.write_text(json.dumps({'cases':results},indent=2)+'\n');t=time.monotonic()
 with (out/'stdout.log').open('w') as stdout,(out/'stderr.log').open('w') as stderr:
  try:
   p=subprocess.run([str(binary),kind,str(out)],stdout=stdout,stderr=stderr,timeout=300);entry['exitCode']=p.returncode;entry['status']='terminal'
  except subprocess.TimeoutExpired:entry['status']='interrupted-operational-timeout';entry['unreceiptedActualWork']='Unknown after last complete trace receipt, potentially including synchronous WEAK forecast; not success'
 entry['wholeProcessWallSeconds']=time.monotonic()-t;entry['finishedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
 p=out/'result.json'
 if p.exists():
  result=json.loads(p.read_text());entry['resultSha256']=hashlib.sha256(p.read_bytes()).hexdigest();entry['outcome']=result['outcome'];entry['realNativeIntegrations']=result['realNativeIntegrations'];entry['forecastNativeIntegrations']=result['forecastNativeIntegrations'];entry['standingPassed']=result['standingPassed'];entry['terminalBalance']=result['terminalBalance']
 print(json.dumps(entry),flush=True);ledger.write_text(json.dumps({'cases':results},indent=2)+'\n')
 if entry.get('exitCode',0)!=0 or entry['status']!='terminal':sys.exit(1)
