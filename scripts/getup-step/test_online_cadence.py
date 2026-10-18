import ast,json,tempfile,unittest
from pathlib import Path
from online_cadence import function_hashes,UNCHANGED,periodic_checkpoint_due,OLD_RUNNER_SHA
from native_env import ROOT,sha256
class Tests(unittest.TestCase):
    def test_physical_benchmark_ast_is_exact_and_cadence_is_counted(self):
        old=ROOT/'outputs/getup-source-online/session-20261018T023139Z/reviewed-benchmark-runner.py.txt';new=Path(__file__).with_name('online_run.py');self.assertEqual(sha256(old),OLD_RUNNER_SHA);a,b=function_hashes(old),function_hashes(new)
        for k in UNCHANGED:self.assertEqual(a[k],b[k],k)
        self.assertFalse(periodic_checkpoint_due(63));self.assertTrue(periodic_checkpoint_due(64));self.assertTrue(periodic_checkpoint_due(1024));self.assertFalse(periodic_checkpoint_due(5000))
        for n in [0,True,64.,5001]:
            with self.assertRaises(ValueError):periodic_checkpoint_due(n)
    def test_fresh_window_binds_closed_prior_costs_and_does_not_reopen(self):
        import types
        from online_cadence import candidate_identity,CADENCE_SHA,ORIGINAL_PREFLIGHT_SHA,ORIGINAL_SUMMARY_SHA
        from online_run import PLAN_SHA,PROTOCOL_SHA
        with tempfile.TemporaryDirectory()as d:
            stage=Path(d);prior=stage/'closed.json';prior.write_text(json.dumps(dict(status='closed-benchmark',candidateOptimizerCalls=0,stagePolicyIntervals=dict(training=0),optimizerCalls=18,sourceIntegrations=1280)))
            fields=dict(stageStartedUtc='start',stageDeadlineUtc='deadline');x=dict(schema=1,**fields,cadenceOverlaySHA256=CADENCE_SHA,historicalBenchmarkPreflightSHA256=ORIGINAL_PREFLIGHT_SHA,historicalBenchmarkSummarySHA256=ORIGINAL_SUMMARY_SHA,closedBenchmarkLedger=str(prior),closedBenchmarkLedgerSHA256=sha256(prior),noBudgetOrCounterTransfer=True,newPhysicsPermissionSeconds=20);(stage/'fresh-candidate-window.json').write_text(json.dumps(x));b=types.SimpleNamespace(profile='slow-online',data=dict(startedUtc='start',deadlineUtc='deadline',planSHA256=PLAN_SHA,protocolSHA256=PROTOCOL_SHA,status='fresh',pendingOptimizer=None,requestOutstanding=False))
            candidate_identity(b,{},stage);b.data['status']='closed-old'
            with self.assertRaises(ValueError):candidate_identity(b,{},stage)
            b.data['status']='fresh';prior.write_text('{}')
            with self.assertRaises(ValueError):candidate_identity(b,{},stage)
    def test_recovery_policy_is_explicit_and_does_not_claim_current(self):
        x=json.loads(Path(__file__).with_name('online-checkpoint-cadence-v1.json').read_text());self.assertEqual(x['fullRollingEveryVerifiedTransitions'],64);self.assertEqual(x['maximumActualNewTransitionsUnchanged'],5000);self.assertIn('charged ledger',x['interruption']);self.assertIn('actual reset cache hash and ordinal',x['everyTerminalBeforeReset'])
if __name__=='__main__':unittest.main()
