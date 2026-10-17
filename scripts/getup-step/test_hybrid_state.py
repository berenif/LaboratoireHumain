import copy,json,tempfile,unittest
from pathlib import Path
from hybrid_state import validate_controller,save_task_checkpoint,restore_task_checkpoint
from native_env import atomic_json,sha256

class HybridTests(unittest.TestCase):
    def state(self,ref='a',cfg='b'):
        return dict(schema=1,phase='prepare',referenceSHA256=ref,configSHA256=cfg,nativeId=400,phaseWallStartNativeId=0,
                    referenceClock=dict(tau=.5,rate=1.,last_native_substep=400,target_rate=1.,response_seconds=.1),
                    unloadRun=0,clearanceRun=0,touchdownRun=0,loadRun=0,finalQuietRun=0,awardedEvents=[],entryHeading=[1.,0.,0.],entryFootOrigin=[0.,0.,0.],physicalSlipAnchors={'original':True},postLandingAnalysisAnchors=None,lastAction=[0.]*21,RNG={'seed':20261017})
    def test_restore_scalars_clocks_hashes_and_event_prefix(self):
        state=self.state();self.assertEqual(validate_controller(state,reference_sha256='a',config_sha256='b'),state)
        for field,value in [('nativeId',400.),('referenceSHA256','wrong'),('finalQuietRun',-1),('awardedEvents',['clearance']),('entryHeading',[0.,0.,0.])]:
            bad=copy.deepcopy(state);bad[field]=value
            with self.assertRaises(ValueError):validate_controller(bad,reference_sha256='a',config_sha256='b')
        bad=copy.deepcopy(state);bad['referenceClock']['last_native_substep']=399
        with self.assertRaises(ValueError):validate_controller(bad,reference_sha256='a',config_sha256='b')
    def test_checked_save_restore_no_budget_rewind_and_hash_rejects_before_load(self):
        class FakeNative:
            def __init__(self,root):self.substeps=777;self.ledger=type('Ledger',(),{'path':root/'ledger.json'})();self.calls=[]
            def command(self,x):
                self.calls.append(x)
                if x['op']=='checkpoint':Path(x['path']).write_bytes(b'physical-caches');return {'roundtripPassed':True}
                return {'diagnostic':{'nativeSubsteps':400},'observe':{'q':[1,2]},'snapshot':{'segments':[3]},'load':{}}[x['op']]
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);ref=root/'ref';cfg=root/'cfg';ref.write_bytes(b'reference');cfg.write_bytes(b'config');state=self.state(sha256(ref),sha256(cfg));n=FakeNative(root);p=root/'native.bin'
            save_task_checkpoint(n,p,state,reference_path=ref,config_path=cfg);restored=restore_task_checkpoint(n,p,reference_path=ref,config_path=cfg)
            self.assertEqual(restored,state);self.assertEqual(n.substeps,777)
            p.write_bytes(b'corrupt');n.calls=[]
            with self.assertRaises(ValueError):restore_task_checkpoint(n,p,reference_path=ref,config_path=cfg)
            self.assertEqual(n.calls,[])
if __name__=='__main__':unittest.main()
