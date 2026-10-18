#!/usr/bin/env python3
"""Saved-packet observer regression/profile. ZERO engine/optimizer operations.

Legacy own source is read from the named Git commit; a fake world returns only
already recorded packets. Replayed IDs are not new integrations or physics cost.
"""
import argparse,copy,hashlib,importlib.util,json,subprocess,sys,time
from pathlib import Path
import numpy as np
from native_env import ROOT,atomic_json,sha256
from slow_task import SourceSLOWEnv

BASELINE='54957e7843f5aeab507d8bff314e670611ddf260'  # published exact tree
def legacy_modules(out):
    modules=[]
    for name,filename in [('observer_legacy_core','source_task.py'),('observer_legacy_slow','slow_task.py')]:
        source=subprocess.check_output(['git','show',BASELINE+':scripts/getup-step/'+filename],cwd=ROOT).decode()
        if filename=='slow_task.py':source=source.replace('from source_task import SourceStabilization,decode_source_frame','from observer_legacy_core import SourceStabilization,decode_source_frame')
        path=out/(name+'.py');path.write_text(source);spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
        module.CONFIG_PATH=Path(__file__).with_name('task-config-mujoco-slow-stabilization-v1.json'if filename=='slow_task.py'else'task-config-mujoco-stabilization-v1.json');modules.append(module)
    return modules

class SavedWorld:
    def __init__(self,entry,packets):self.row=copy.deepcopy(entry);self.packets=iter(packets);self.controls=np.zeros(21);self.replayed=0
    def task_packet(self):return copy.deepcopy(self.row)
    def observation(self):return np.array(self.row['physicalObservation'])
    def dim(self,i):
        if i!=6:raise ValueError('only saved source counter available')
        return self.row['sourceStep']
    def arr(self,i,shape):
        if i==0:return np.array(self.row['qpos'])
        if i==1:return np.array(self.row['qvel'])
        if i==2:return self.controls
        raise ValueError('no engine arrays available')
    def replay_one(self):self.row=next(self.packets);self.replayed+=1

def run(base,out):
    out.mkdir(parents=True,exist_ok=False);legacy=legacy_modules(out)[1];model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());oldstage=ROOT/'outputs/getup-source-slow-stabilization/session-20261018T010710Z';entry=json.loads((oldstage/'benchmark-preflight.json').read_text())['sourceInitialPacket'];results=[];hashes={}
    for name in ['source_task.py','source_observer.py','slow_task.py','source_task_env.py','replay_observer_parity.py']:
        p=Path(__file__).with_name(name);hashes[str(p.resolve())]=sha256(p)
    for p in [ROOT/'rust/crates/model/data/getup-v1.json',oldstage/'benchmark-preflight.json',Path(__file__).with_name('task-config-mujoco-slow-stabilization-v1.json')]:hashes[str(p.resolve())]=sha256(p)
    for branch in ['frozen-original','candidate']:
        packets_path=base/(branch+'-frames.ndjson');commands_path=base/(branch+'-commands.json');packets=[json.loads(line)['packet']for line in packets_path.read_text().splitlines()];commands=json.loads(commands_path.read_text());hashes[str(packets_path)]=sha256(packets_path);hashes[str(commands_path)]=sha256(commands_path)
        for label,klass in [('legacy',legacy.SourceSLOWEnv),('optimized',SourceSLOWEnv)]:
            world=SavedWorld(entry,packets);env=klass(world,model,bindings={'saved-packet-only':'no engine'},rng=np.random.default_rng(0));started=time.perf_counter()
            for index,wanted in enumerate(commands):
                got=env.measured_interval(wanted['action'],lambda w:w.replay_one())
                for key in ['state','action','next_state','reward','not_done','info']:
                    if got[key]!=wanted[key]:raise AssertionError(f'{label}/{branch}/{index}/{key} differs from historical receipt')
            wall=time.perf_counter()-started
            results.append(dict(branch=branch,observer=label,policyIntervals=len(commands),savedPacketsReplayed=world.replayed,wallSeconds=wall,policyIntervalsPerSecond=len(commands)/wall,allCommandAndNativeSafetyOutputsExactlyEqual=True,finalNativeId=env.core.native_id,finalSlipM=env.core.slip.tolist(),terminal=env.core.terminal))
    for path,digest in hashes.items():
        if sha256(path)!=digest:raise ValueError('saved physical input changed')
    report=dict(scope='pure saved-packet fake-world replay; no engine construction, integration, optimizer or policy inference',newNativeIntegrations=0,optimizerCalls=0,legacyCommit=BASELINE,results=results,aggregateLegacyWallSeconds=sum(x['wallSeconds']for x in results if x['observer']=='legacy'),aggregateOptimizedWallSeconds=sum(x['wallSeconds']for x in results if x['observer']=='optimized'),inputHashes=hashes)
    report['pureObserverSpeedup']=report['aggregateLegacyWallSeconds']/report['aggregateOptimizedWallSeconds'];atomic_json(out/'summary.json',report);print(json.dumps({k:v for k,v in report.items()if k!='inputHashes'},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--saved-result',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.saved_result,a.output)
