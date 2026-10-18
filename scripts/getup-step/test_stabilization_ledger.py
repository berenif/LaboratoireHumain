import datetime,json,tempfile,unittest
from pathlib import Path
from stabilization_ledger import StageBudget
class StageBudgetTests(unittest.TestCase):
    def make(self,path):
        x=dict(deadlineUtc=(datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(minutes=30)).isoformat(),requestOutstanding=False,physicsSeconds=0.,sourceIntegrations=0,totalPolicyIntervals=0,stagePhysicsSeconds=dict(benchmark=0.,training=0.,validation=0.),stagePolicyIntervals=dict(benchmark=0,training=0,validation=0),validationWallReservedSeconds=300,aggregatePhysicsLimitSeconds=10.)
        x['benchmarkDeadlineUtc']=(datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(seconds=120)).isoformat();path.write_text(json.dumps(x));return x
    def test_one_call_and_interruption_are_charged(self):
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'ledger.json';self.make(p);n=[0]
            with StageBudget(p)as b:
                b.begin_interval('benchmark');b.integrate('benchmark',lambda maximum:n.__setitem__(0,n[0]+1),lambda:n[0]);self.assertEqual(b.data['sourceIntegrations'],1)
                def stop(_):n[0]+=1;raise KeyboardInterrupt()
                with self.assertRaises(KeyboardInterrupt):b.integrate('benchmark',stop,lambda:n[0])
                self.assertEqual(b.data['sourceIntegrations'],2);self.assertGreater(b.data['physicsSeconds'],0);self.assertFalse(b.data['requestOutstanding'])
    def test_all_three_validation_reserves(self):
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'ledger.json';x=self.make(p);x['stagePolicyIntervals']['training']=796;x['totalPolicyIntervals']=860;p.write_text(json.dumps(x))
            with StageBudget(p)as b:
                with self.assertRaises(TimeoutError):b.begin_interval('training')
                b.begin_interval('validation');self.assertEqual(b.data['totalPolicyIntervals'],861)
            x=self.make(p);x['physicsSeconds']=7.;x['stagePhysicsSeconds']['training']=6.;x['stagePhysicsSeconds']['benchmark']=1.;p.write_text(json.dumps(x))
            with StageBudget(p)as b:
                with self.assertRaises(TimeoutError):b.allowance('training')
                self.assertGreater(b.allowance('validation'),0.)
    def test_shared_stage_lock_and_local_benchmark_deadline(self):
        import fcntl
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'ledger.json';x=self.make(p);other=(p.parent/'stage.lock').open('a');fcntl.flock(other,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):StageBudget(p)
            other.close();x['benchmarkDeadlineUtc']=(datetime.datetime.now(datetime.timezone.utc)-datetime.timedelta(seconds=1)).isoformat();p.write_text(json.dumps(x))
            with StageBudget(p)as b:
                with self.assertRaises(TimeoutError):b.allowance('benchmark')
    def test_uncertain_counter_blocks_restart(self):
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'ledger.json';self.make(p);n=[0]
            with StageBudget(p)as b:
                def counter():return 0 if not n[0]else float('nan')
                with self.assertRaises(ValueError):b.integrate('benchmark',lambda _:n.__setitem__(0,1),counter)
                self.assertTrue(b.data['requestOutstanding']);self.assertEqual(b.data['sourceIntegrations'],1)
            with self.assertRaises(ValueError):StageBudget(p)
    def test_separate_analytic_diagnostic_limits(self):
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'ledger.json';x=self.make(p)
            x.update(profile='analytic-diagnostic',aggregatePhysicsLimitSeconds=1.,validationWallReservedSeconds=0,allIntervalLimit=64)
            p.write_text(json.dumps(x))
            with StageBudget(p)as b:
                self.assertGreater(b.allowance('benchmark'),0.)
                for stage in ['training','validation']:
                    with self.assertRaises(TimeoutError):b.begin_interval(stage)
                for _ in range(64):b.begin_interval('benchmark')
                with self.assertRaises(TimeoutError):b.begin_interval('benchmark')
                self.assertEqual(b.data['totalPolicyIntervals'],64)
            x['aggregatePhysicsLimitSeconds']=10.;p.write_text(json.dumps(x))
            with self.assertRaises(ValueError):StageBudget(p)
if __name__=='__main__':unittest.main()
