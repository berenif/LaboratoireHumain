#!/usr/bin/env python3
"""Compile the frozen diagnostic or its unit harness; never execute a worker."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('kind', choices=['release', 'pure-worker-test'])
parser.add_argument('--label', required=True, choices=['admitted'])
args = parser.parse_args()
initial = json.loads((HERE / (args.kind + '-build-receipt.json'
                    if args.kind == 'release' else 'pure-worker-test-compile-receipt.json')).read_text())
prefix = args.kind + '-' + args.label
paths = {name: HERE / (prefix + suffix) for name, suffix in
         [('stdout', '.jsonl'), ('stderr', '.stderr.log'), ('receipt', '-receipt.json')]}
assert not any(path.exists() for path in paths.values()), 'Do not overwrite a build receipt'
environment = os.environ.copy()
environment.update(CARGO_HOME='/workspace/lh-cargo',
                   RUSTC='/workspace/lh-rustup/toolchains/1.89.0-x86_64-unknown-linux-gnu/bin/rustc')
if args.kind == 'release':
    environment['CARGO_TARGET_DIR'] = '/workspace/lh-contact-diagnostic-target'
source = ROOT / 'rust/crates/sim/src/recovery/planner/transfer_worker.rs'
historical = ROOT / 'rust/target/release/deps/coordination_transfer_worker-8daed3b7e6b04b70'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


source_before = sha(source)
assert source_before == '811df09bae544476e03026a9d832f7ea306ff774fe85e42929f974efd2c3f89c'
historical_before = sha(historical)
assert historical_before == '63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5'
start = time.monotonic()
begun = datetime.now(timezone.utc).isoformat()
with paths['stdout'].open('x') as stdout, paths['stderr'].open('x') as stderr:
    completed = subprocess.run(initial['command'], cwd=ROOT, env=environment,
                               stdout=stdout, stderr=stderr)
executables = []
for line in paths['stdout'].read_text().splitlines():
    data = json.loads(line)
    if data.get('reason') == 'compiler-artifact' and data.get('executable'):
        executables.append({'path': data['executable'], 'sha256': sha(data['executable']),
                            'target': data['target']})
receipt = {'command': initial['command'], 'startedUtc': begun,
           'wallSeconds': time.monotonic() - start, 'exitCode': completed.returncode,
           'workerSourceBefore': source_before, 'workerSourceAfter': sha(source),
           'historicalReleaseBefore': historical_before, 'historicalReleaseAfter': sha(historical),
           'executables': executables, 'workerExecuted': False, 'nativeSteps': 0,
           'stdoutSha256': sha(paths['stdout']), 'stderrSha256': sha(paths['stderr'])}
receipt['candidateSourceStable'] = receipt['workerSourceAfter'] == source_before
receipt['historicalReleasePreserved'] = receipt['historicalReleaseAfter'] == historical_before
with paths['receipt'].open('x') as file:
    file.write(json.dumps(receipt, indent=2) + '\n')
assert receipt['workerSourceAfter'] == source_before
assert receipt['historicalReleaseAfter'] == historical_before
print(json.dumps(receipt))
raise SystemExit(completed.returncode)
