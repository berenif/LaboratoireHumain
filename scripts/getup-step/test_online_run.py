import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import torch
from online_learning import initialize,Roles,OnlineReplay,restore
from online_io import Guard,rolling_checkpoint
from online_run import projection,valid_physical_terminal
class Budget:
    def __init__(self,p):self.path=p;self.data=dict(optimizerCalls=0,pendingOptimizer=None)
    def allowance(self,phase):return 10.
    def remaining_wall(self):return 1000.
class Env:
    def checkpoint(self):return b'actual-native',dict(counter=10,packetSHA='actual',sourceCommand='held')
class Tests(unittest.TestCase):
    def test_alternating_complete_save_immutable_and_interruption(self):
        with tempfile.TemporaryDirectory()as d:
            out=Path(d);b=Budget(out/'ledger.json');g=Guard(b,out);a,o=initialize();r=OnlineReplay();roles=Roles('fixture');r.rng=roles.replay
            one=rolling_checkpoint(out,g,Env(),a,r,roles,'one',immutable='entry');immutable=json.loads((out/'milestone-entry-manifest.json').read_text());self.assertEqual(Path(immutable['learner']).name,'milestone-entry.pt')
            saved=torch.load(one['learner'],weights_only=True);aa,oo,rr,gg=restore(saved['learning']);self.assertEqual(len(rr.items),0)
            with patch.object(g,'torch_save',side_effect=OSError('disk interruption')):
                with self.assertRaises(OSError):rolling_checkpoint(out,g,Env(),a,r,roles,'interrupted')
            self.assertEqual(json.loads((out/'latest-checkpoint.json').read_text()),one)
            for marker in ['pendingOptimizer','requestOutstanding','proposalInProgress']:
                b.data[marker]='unresolved'
                with self.assertRaises(ValueError):rolling_checkpoint(out,g,Env(),a,r,roles,'invalid')
                self.assertEqual(json.loads((out/'latest-checkpoint.json').read_text()),one);b.data[marker]=None
            two=rolling_checkpoint(out,g,Env(),a,r,roles,'two');self.assertNotEqual(one['slot'],two['slot']);self.assertEqual((out/'milestone-entry.bin').read_bytes(),b'actual-native')
    def test_optimizer_precharge_pending_and_kind_limits(self):
        with tempfile.TemporaryDirectory()as d:
            g=Guard(Budget(Path(d)/'ledger.json'),Path(d));g.charge('benchmark-critic');self.assertEqual(g.budget.data['optimizerCalls'],1)
            with self.assertRaises(ValueError):g.charge('benchmark-head')
            g.complete('benchmark-critic')
            for _ in range(2):g.charge('benchmark-head');g.complete('benchmark-head');g.commit_proposal('benchmark-head')
            with self.assertRaises(ValueError):g.charge('benchmark-head')
    def test_complete_benchmark_orchestration_with_fake_world(self):
        import datetime,types,numpy as np
        from online_run import benchmark,PLAN_SHA,PROTOCOL_SHA
        from native_env import sha256
        class World:
            def __init__(self):self.n=0;self.lib=types.SimpleNamespace(lh_step_audited=self.step);self.h=1
            def step(self,h,maximum):self.n+=1;return 1
            def dim(self,key):return self.n
            def restore(self,blob):self.n=0
            def observation(self):return np.zeros(67)
            def task_packet(self):return dict(n=self.n)
            def checkpoint(self):return b'cache'
            def close(self):pass
        class FakeEnv:
            def __init__(self,world,*args,**kwargs):self.world=world;self.core=types.SimpleNamespace(observation=lambda physical:np.r_[np.zeros(117,dtype=np.float32),np.float32(1)])
            def checkpoint(self):return b'cache',dict(n=self.world.n)
            def restore(self,blob,side):self.world.n=side['n']
            def measured_interval(self,action,step):
                frames=[]
                for _ in range(20):step(self.world);frames.append(dict(sourceStep=self.world.n,sourceWarnings=0,maximumSelfPenetrationM=0.))
                x=self.core.observation(None).tolist();return dict(state=x,action=action.tolist(),next_state=x,reward=1.,not_done=1.,info=dict(nativeSampleCount=20,physicalFailure=None,success=False,terminated=False,truncated=False,maximumPerNativeTorqueErrorNm=0.),actualFrames=frames,perNativeControls=[dict(maximumTorqueErrorNm=0.)for _ in frames])
        with tempfile.TemporaryDirectory()as folder:
            stage=Path(folder);now=datetime.datetime.now(datetime.timezone.utc);ledger=dict(profile='slow-online',startedUtc=now.isoformat(),deadlineUtc=(now+datetime.timedelta(minutes=45)).isoformat(),planSHA256=PLAN_SHA,protocolSHA256=PROTOCOL_SHA,aggregatePhysicsLimitSeconds=20.,validationWallReservedSeconds=300,allIntervalLimit=5704,maximumSourceSteps=114080,physicsSeconds=0.,sourceIntegrations=0,totalPolicyIntervals=0,stagePhysicsSeconds=dict(benchmark=0.,training=0.,validation=0.),stagePolicyIntervals=dict(benchmark=0,training=0,validation=0),optimizerCalls=0,benchmarkOptimizerCalls=0,candidateOptimizerCalls=0,pendingOptimizer=None,requestOutstanding=False)
            (stage/'ledger.json').write_text(json.dumps(ledger));pre=dict(stageStartedUtc=ledger['startedUtc'],stageDeadlineUtc=ledger['deadlineUtc'],sourceInitialPacket=dict(n=0),inputHashes={});(stage/'benchmark-preflight.json').write_text(json.dumps(pre));w=World();env=FakeEnv(w)
            with patch('online_run.setup',return_value=(w,env,b'cache',{})),patch('online_run.SourceSLOWEnv',FakeEnv),patch('online_run.probes',return_value=torch.cat([torch.zeros(201,117),torch.ones(201,1)],1)),patch('online_run.rolling_checkpoint',return_value={}),patch('online_io.Guard.storage',return_value=0):
                benchmark(stage,stage,sha256(stage/'benchmark-preflight.json'))
            result=json.loads((stage/'benchmark/summary.json').read_text());self.assertTrue(result['passed']);self.assertEqual(result['allBranchNativeIntegrations'],1280);self.assertEqual(result['benchmarkOptimizerCalls'],18)
    def test_whole_path_projection_and_warning_abort(self):
        bench=dict(timings=dict(actual64IntervalLoopWallSeconds=6.4,actualCacheAndControllerResetWallSeconds=.01,worstFullCheckpointWallSeconds=.02,worstCriticUpdateWallSeconds=.01,worstFull2560Witness11ScaleProposalWallSeconds=.03),physicsSeconds=.01,allBranchNativeIntegrations=1280)
        p=projection(bench,5000,994,1000);self.assertGreater(p['trainingWall'],5000*.1*1.5);self.assertGreater(p['validationWall'],64.)
        t=dict(info=dict(maximumPerNativeTorqueErrorNm=0.,physicalFailure='native_integrity'),actualFrames=[dict(sourceWarnings=0,maximumSelfPenetrationM=.006)])
        valid_physical_terminal(t);t['actualFrames'][0]['sourceWarnings']=1
        with self.assertRaises(ValueError):valid_physical_terminal(t)
if __name__=='__main__':unittest.main()
