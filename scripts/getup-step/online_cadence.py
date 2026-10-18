"""Parent-authorized checkpoint cadence overlay and explicit benchmark migration.

Only offline I/O timing occurs here. No actor update, replay collection or engine
integration is performed. Existing charged physical proof is never rerun/refunded.
"""
import ast,copy,hashlib,json,os,time
from pathlib import Path
import torch
from native_env import sha256,atomic_json
CADENCE=Path(__file__).with_name('online-checkpoint-cadence-v1.json')
CADENCE_SHA='2b0379a2ffd59a7d5c55e6a3bdb12ee69970e828e2ce7dec8b67c22aacc65df6'
ORIGINAL_PREFLIGHT_SHA='55ffb3be17b5135c1ade743a96b8b91df54261c79ea58ff189cdf29ec43bc581'
ORIGINAL_SUMMARY_SHA='725cf5fc32ddf893f056842ffe41fd8e9b08e7b63cc415a15afbc15f8d3e9fae'
OLD_RUNNER_SHA='d22d3664abf2881afa288bafd0e90e493efff923056a19a0f4d5fba623605c09'
UNCHANGED=['benchmark','setup','audit','audit_ok','probes','transition_record','write_rows','interval','valid_physical_terminal','identity']
def function_hashes(path):
    tree=ast.parse(Path(path).read_text());return {node.name:hashlib.sha256(ast.dump(node,include_attributes=False).encode()).hexdigest()for node in tree.body if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef))}
def compatible(stage,source_receipts,current_runner):
    if sha256(CADENCE)!=CADENCE_SHA:raise ValueError('checkpoint overlay changed')
    if sha256(stage/'benchmark-preflight.json')!=ORIGINAL_PREFLIGHT_SHA or sha256(stage/'benchmark/summary.json')!=ORIGINAL_SUMMARY_SHA:raise ValueError('historical physical benchmark proof changed')
    old=stage/'reviewed-benchmark-runner.py.txt'
    if sha256(old)!=OLD_RUNNER_SHA:raise ValueError('original reviewed benchmark runner missing')
    a,b=function_hashes(old),function_hashes(current_runner)
    if any(a[k]!=b[k]for k in UNCHANGED):raise ValueError('physical/source benchmark function changed')
    count=0
    for receipt in source_receipts:
        for p,digest in receipt['inputHashes'].items():
            if Path(p).resolve()==Path(current_runner).resolve():
                if digest!=OLD_RUNNER_SHA:raise ValueError('unknown benchmark runner revision')
            elif sha256(p)!=digest:raise ValueError('benchmark interface input changed: '+p)
            count+=1
    report=dict(passed=True,scope='checkpoint cadence overlay; physical/source benchmark functions unchanged by AST; no new engine or optimizer',originalRunnerSHA256=OLD_RUNNER_SHA,currentRunnerSHA256=sha256(current_runner),cadenceOverlaySHA256=CADENCE_SHA,unchangedFunctions={k:a[k]for k in UNCHANGED},checkedPriorInputRecords=count,originalPreflightSHA256=ORIGINAL_PREFLIGHT_SHA,originalBenchmarkSummarySHA256=ORIGINAL_SUMMARY_SHA)
    atomic_json(stage/'cadence-benchmark-migration.json',report);return report

def io_timings(stage,guard,terminal_record):
    """Maximum-sized compact receipts + terminal fsync proof, NOT training."""
    guard.check();out=stage/'cadence-io-timing';out.mkdir(exist_ok=True);head=dict(weight=torch.zeros(42,1024),bias=torch.zeros(42));adam=dict(state={0:dict(step=torch.tensor(994.),exp_avg=torch.zeros(42,1024),exp_avg_sq=torch.zeros(42,1024)),1:dict(step=torch.tensor(994.),exp_avg=torch.zeros(42),exp_avg_sq=torch.zeros(42))},param_groups=[dict(params=[0,1])]);receipt=dict(TIMING_ONLY_NEVER_LEARNER=True,head=head,proposedHead=copy.deepcopy(head),beforeAdam=copy.deepcopy(adam),proposedAdam=copy.deepcopy(adam),retainedAdam=copy.deepcopy(adam),noise=torch.zeros(128,21),roles=torch.zeros(3,5056,dtype=torch.uint8),auditLatentStd=[[.1]*21 for _ in range(201)],elevenScaleRecords=[dict(scale=2.**(-i),loss=0.,kl=0.)for i in range(11)])
    compact=[];proof=[]
    for i in range(3):
        begin=time.monotonic();guard.torch_save(out/f'compact-{i}-TIMING-ONLY.pt',receipt);sha256(out/f'compact-{i}-TIMING-ONLY.pt');guard.check();compact.append(time.monotonic()-begin)
        begin=time.monotonic()
        with(out/'terminal-proof-TIMING-ONLY.ndjson').open('a')as stream:
            for _ in range(3):stream.write(json.dumps(dict(timingOnly=True,interval=terminal_record,resetCacheSHA256='0'*64,resetOrdinal=5000),allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
        guard.check();proof.append(time.monotonic()-begin)
    result=dict(engineIntegrations=0,optimizerCalls=0,timingOnlyNeverTraining=True,worstCompactReceiptWallSeconds=max(compact),worstThreeJournalTerminalFsyncWallSeconds=max(proof));atomic_json(stage/'cadence-io-timing.json',result);return result

def save_complete(out,guard,env,agent,replay,roles,label,*,immutable=None):
    from online_io import rolling_checkpoint
    entry=rolling_checkpoint(out,guard,env,agent,replay,roles,label,immutable=immutable)
    # The learner replay binds its last charged global policy ID as well. This
    # extra manifest metadata makes lag explicit without loading the .pt file.
    entry.update(actualCandidateTransitions=len(replay.items),chargedGlobalPolicyInterval=guard.budget.data['totalPolicyIntervals'],chargedGlobalSourceIntegrations=guard.budget.data['sourceIntegrations'],recoveryScope='complete state at these IDs; later charged work is not refunded or assumed current')
    guard.check();atomic_json(out/'latest-checkpoint.json',entry);return entry

def periodic_checkpoint_due(n):
    if type(n)is not int or not 1<=n<=5000:raise ValueError('verified candidate interval count required')
    return n%64==0

def candidate_identity(budget,pre,stage):
    from online_run import identity
    if budget.data.get('status','').startswith('closed'):raise ValueError('closed stage cannot collect or reopen')
    path=stage/'fresh-candidate-window.json'
    if not path.exists():identity(budget,pre);return
    x=json.loads(path.read_text());identity(budget,x)
    if x['schema']!=1 or x['cadenceOverlaySHA256']!=CADENCE_SHA or x['historicalBenchmarkPreflightSHA256']!=ORIGINAL_PREFLIGHT_SHA or x['historicalBenchmarkSummarySHA256']!=ORIGINAL_SUMMARY_SHA:raise ValueError('fresh candidate proof binding invalid')
    prior=Path(x['closedBenchmarkLedger']);d=json.loads(prior.read_text())
    if sha256(prior)!=x['closedBenchmarkLedgerSHA256']or not d['status'].startswith('closed')or d['candidateOptimizerCalls']!=0 or d['stagePolicyIntervals']['training']!=0 or d['optimizerCalls']!=18 or d['sourceIntegrations']!=1280:raise ValueError('prior physical costs not closed/preserved')
    if not x['noBudgetOrCounterTransfer']or x['newPhysicsPermissionSeconds']!=20:raise ValueError('fresh resource authority missing')
