#!/usr/bin/env python3
"""Zero-integration physical/Python checkpoint restore smoke test."""
import argparse,json
from pathlib import Path
from native_env import PilotLedger,SourceNative,atomic_json,sha256
from hybrid_state import save_task_checkpoint,restore_task_checkpoint

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--seed',type=Path,required=True);p.add_argument('--entry',type=Path,required=True);p.add_argument('--reference',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--ledger',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    ledger=PilotLedger(a.ledger);n=SourceNative(a.binary,a.seed,a.output,ledger,stage='frozenReference',physics_cap=0.)
    try:
        before=n.command(dict(op='load',path=str(a.entry.resolve())));diag=n.command(dict(op='diagnostic'));native_id=diag['nativeSubsteps'];ref=json.loads(a.reference.read_text());heading=[.97177241,0.,.23592030];norm=sum(x*x for x in heading)**.5;heading=[x/norm for x in heading]
        state=dict(schema=1,phase='prepare',referenceSHA256=sha256(a.reference),configSHA256=sha256(a.config),nativeId=native_id,phaseWallStartNativeId=native_id,
            referenceClock=dict(tau=0.,rate=1.,last_native_substep=native_id,target_rate=1.,response_seconds=.1),unloadRun=0,clearanceRun=0,touchdownRun=0,loadRun=0,finalQuietRun=0,awardedEvents=[],entryHeading=heading,entryFootOrigin=[before['metrics']['foot_positions'][0][k]for k in 'xyz'],physicalSlipAnchors=dict(ownedByExactNativeCheckpoint=True,loadedSlipAtEntry=before['loaded_slip_distance_m']),postLandingAnalysisAnchors=None,lastAction=before['metrics']['controls'],RNG=dict(algorithm='numpy.PCG64',seed=20261017),scope='restore plumbing only; prior source command retained, no new task executed')
        path=a.output/'entry.bin';save_task_checkpoint(n,path,state,reference_path=a.reference,config_path=a.config)
        restored=restore_task_checkpoint(n,path,reference_path=a.reference,config_path=a.config)
        if restored!=state or n.substeps!=0 or n.physics!=0:raise ValueError('restore parity/zero physics failed')
        report=dict(passed=True,physicsIntegrations=0,physicsWallSeconds=0.,nativePhysicalId=native_id,controllerExact=True,physicalObservationAndSnapshotExact=True,budgetCountersNotRewound=True,entrySHA256=sha256(a.entry),referenceSHA256=sha256(a.reference),configSHA256=sha256(a.config),checkpointSHA256=sha256(path),scope='entry restore only; moving phase continuation remains separately gated')
        atomic_json(a.output/'summary.json',report);print(json.dumps(report,indent=2))
    finally:n.close();ledger.close()
if __name__=='__main__':main()
