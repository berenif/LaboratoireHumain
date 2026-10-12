#!/usr/bin/env python3
"""Hash compiled/static inputs only. Never starts a worker or changes a budget."""
import argparse, hashlib, importlib.util, json, pathlib, re
p=argparse.ArgumentParser();p.add_argument('--binary',type=pathlib.Path,required=True);a=p.parse_args()
evidence=pathlib.Path(__file__).resolve().parent;repo=evidence.parents[2]
spec=importlib.util.spec_from_file_location('budget',repo/'scripts/coordination-prototype-budget.py');budget=importlib.util.module_from_spec(spec);spec.loader.exec_module(budget)
def record(path):
 path=path.resolve(strict=True)
 return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
def records(paths):return [record(path) for path in sorted(set(paths))]
logs=list(evidence.glob('*-tests.log'))
for log in logs:
 text=log.read_text()
 if not (re.search(r'test result: ok\.',text) or re.search(r'\nOK\s*$',text)):raise RuntimeError('Missing successful test receipt: '+str(log))
if 'Finished' not in (evidence/'production-check.log').read_text():raise RuntimeError('Missing production compilation')
if 'Executable' not in (evidence/'native-worker-build.log').read_text():raise RuntimeError('Missing native worker compilation')
files={
 'model':records((repo/'rust/crates/model').rglob('*.json')),
 'profile':records([repo/'rust/crates/sim/data/recovery-profile-v24.json']),
 'fixture':records([repo/'rust/crates/sim/data/recovery-v1.json']),
 'reference':records([evidence/'continuation-candidate-program.json',evidence/'continuation-tasks.json']),
 'code':records([*list((repo/'rust/crates/sim/src').rglob('*.rs')),*list((repo/'rust/crates/model/src').rglob('*.rs')),*list((repo/'rust/crates/contracts/src').rglob('*.rs')),repo/'rust/Cargo.toml',repo/'rust/Cargo.lock',repo/'rust/crates/sim/Cargo.toml',repo/'scripts/coordination-prototype-budget.py',repo/'scripts/run-coordination-prototype.py']),
 'binary':records([a.binary]),
 'criteria':records([evidence/'frozen-criteria.json',repo/'rust/crates/sim/data/recovery-contract-v1.json']),
 'continuation':records([evidence/'continuation-route-audit.json',evidence/'continuation-native-audit.json',evidence/'continuation-native-audit-60deg.json']),
 'preflight_evidence':records([*logs,evidence/'production-check.log',evidence/'native-worker-build.log',evidence/'owner-final-review.md',evidence/'software-review.md']),
}
manifest={'schema':1,'prerequisites':{name:True for name in budget.PREREQUISITES},'files':files,'workerInputs':{'programPath':str(evidence/'continuation-candidate-program.json'),'tasksPath':str(evidence/'continuation-tasks.json')}}
output=evidence/'frozen-manifest.json'
with output.open('x') as f:json.dump(manifest,f,indent=2);f.write('\n')
print(json.dumps({'manifest':str(output),'sha256':record(output)['sha256'],'files':sum(map(len,files.values()))}))
