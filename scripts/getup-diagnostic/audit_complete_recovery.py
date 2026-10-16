#!/usr/bin/env python3
"""Read-only audit from archived generalized motion, matching poses and loads."""
from pathlib import Path
import hashlib
import json
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

began=time.monotonic()
root=Path(__file__).resolve().parents[2]
out=root/(sys.argv[1] if len(sys.argv)>1 else 'outputs/getup-complete-replay-2026-10-16')
r=json.loads((out/'result.json').read_text())
ref=json.loads((out/'reference-report.json').read_text())
ledger=json.loads((root/'docs/evidence/getup-complete-replay-2026-10-16.json').read_text())
rows=[json.loads(x) for x in (out/'trace.ndjson').read_text().splitlines()]
model=json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
vec=lambda v:np.array([v[k] for k in 'xyz'])
rot=lambda q:Rotation.from_quat([q[k] for k in 'xyzw'])
world=lambda v:np.array(v)[[1,2,0]]
assert r['harnessCompleted'] and r.get('harnessError') is None and r['final']['failure'] is None
assert r['counters']==dict(real=7840,teacher=540,checkpointValidation=20,total=8400)
assert r['checkpointNextTick']==dict(nativeSteps=20,snapshotEqual=True,controllerEqual=True)
assert r['exactPrefixPacketsCompared']==41
assert ledger['cumulativeActualNativeSteps']==13760 and ledger['cumulativeActualWallSeconds']<=180
assert ref['stable_substeps']==2407 and ref['status']['prototype_standing_passed']
# Controller anchors use left/right order; source model topology is right/left.
footids=[next(i for i,p in enumerate(model['segments']) if p['id']==name)
         for name in ['left_foot','right_foot']]
weight=model['totalMassKg']*9.81
n=7840; stable=ref['stable_substeps']; proof_s=(n-stable+800)/800
impact_s=ref['impact_substep']/800
anchors=np.array([vec(v) for v in ref['stable_feet']])
samples=[]
for d in ref['student_decisions']:
    if d['native_substep']<n-stable+1: continue
    obs=np.array(d['observation']); poses={p['id']:p for p in d['before']}
    origin=[(world(obs[40:43]),rot(poses['torso']['rotation']).apply(world(obs[43:46])))]
    for p in model['segments'][1:]:
        parent=next(i for i,q in enumerate(model['segments']) if q['id']==p['parent'])
        pr=poses[p['parent']]; rb=poses[p['id']]; jp=p['jointProfile']; R=rot(pr['rotation'])
        anchor=vec(pr['position'])+R.apply(vec(jp['parentFrame']['anchor']))
        axis=(R*rot(jp['parentFrame']['rotation'])).apply([1,0,0])
        h=p['reference']['hinge']; rate=0 if h is None else obs[46+h['qposIndex']]
        v,w=origin[parent]; angular=w+axis*rate
        origin.append((v+np.cross(w,anchor-vec(pr['position']))+np.cross(angular,vec(rb['position'])-anchor),angular))
    twists=[]
    for i,p in enumerate(model['segments']):
        v,w=origin[i]
        twists.append((v+np.cross(w,rot(poses[p['id']]['rotation']).apply(vec(p['goldenMass']['centerOfMass']))),w))
    maxv=max(np.linalg.norm(v) for v,w in twists); maxw=max(np.linalg.norm(w) for v,w in twists)
    comv=sum(p['massKg']*twists[i][0] for i,p in enumerate(model['segments']))/model['totalMassKg']
    err=float(np.max(np.abs(comv-world(obs[37:40]))))
    assert err<5e-5 and maxv<=.22001 and maxw<=.65001
    feet=np.array([vec(poses[model['segments'][i]['id']]['position']) for i in footids])
    drift=float(np.max(np.linalg.norm(feet-anchors,axis=1))); assert drift<=.010001
    head=poses['head']['position']['y']; up=rot(poses['torso']['rotation']).apply([0,1,0])[1]
    assert head>1.4 and up>=.97
    samples.append(dict(timeS=d['native_substep']/800,headM=head,torsoUpDot=float(up),
                        maximumBodyComSpeedMps=float(maxv),maximumAngularRadps=float(maxw),
                        maximumFootDriftM=drift,comVelocityReconstructionErrorMps=err))
assert samples
loads=[]
for s in rows:
    assert s['diagnostics']['direct_pelvis_force_n']==s['diagnostics']['direct_pelvis_torque_nm']==0
    assert s['diagnostics']['requested_motor_effort_nm']<=920.001
    if s['diagnostics']['getup']['policy']['stable_standing_s']<=0: continue
    feet=[sum(c['normal_load_n'] for c in s['contacts'] if c['segment']==i) for i in footids]
    other=max([c['normal_load_n'] for c in s['contacts'] if c['segment'] not in footids] or [0])
    assert min(feet)>=3 and sum(feet)>=.8*weight and other<3
    loads.append(dict(timeS=s['simulation_time_s'],footLoadsN=feet,fractionWeight=sum(feet)/weight,
                      maximumOtherLoadN=other,stableSeconds=s['diagnostics']['getup']['policy']['stable_standing_s']))
for p,v in ledger['inputs'].items(): assert sha(root/p)==v,p
for p,v in ledger['immutablePriorRecords'].items(): assert sha(root/p)==v,p
for p,v in ledger['protectedUnrelatedFilesSha256'].items(): assert sha(root/p)==v,p
reload=json.loads((out/'final-checkpoint-reload.json').read_text()); assert reload['passed'] and reload['physicsIntegrations']==0
report=dict(physicsIntegrationsForAnalysis=0,outcome='single-case-prototype-standing-success',
 historyDurationS=9.8,slowElapsedS=9.8-ref['status']['started_at_s'],slowDecisions=ref['status']['policy_decisions'],
 standingStableContinuousFinalS=stable/800,oneSecondStandingProofAtPhysicalS=proof_s,
 impactAtPhysicalS=impact_s,standingProofAfterImpactS=proof_s-impact_s,deadline25sSatisfied=proof_s-impact_s<=25,
 firstQualifiedArchivedSnapshotS=next(s['simulation_time_s'] for s in rows if s['diagnostics']['getup']['policy']['prototype_standing_passed']),
 realNativeSteps=7840,teacherForecastSteps=540,checkpointValidationForecastSteps=20,thisReplayNativeSteps=8400,
 earlierCheckpointDefectNativeSteps=1600,beforeThisTaskNativeSteps=3760,cumulativeActualIncrementalNativeSteps=13760,
 thisReplayWallSeconds=ledger['actualWallSeconds'],cumulativeActualSimulationWallSeconds=ledger['cumulativeActualWallSeconds'],
 userActualWallCeilingSeconds=180,planningStepCapAmendmentOnly=True,prefixMatchesPreviousExactlyAt41Observations=True,
 checkpointNextTickExact=r['checkpointNextTick'],finalCheckpointReloadPassed=reload,checkpointCount=len(r['checkpoints']),
 stableDecisionSamplesAudited=len(samples),stableSampledSupportPackets=len(loads),stableDecisionMetrics=samples,stableSampledLoads=loads,
 velocityMethod='Current whole-body twists reconstructed from generalized root/joint velocities and matching before-decision poses; not stale RigidBody caches. Native checks run every substep; this independent audit samples40Hz decisions and coarse support packets.',
 finalHeadM=ref['status']['actual_head_height_m'],finalFootLoadsN=loads[-1]['footLoadsN'],finalFootWeightFraction=loads[-1]['fractionWeight'],
 maximumSelfPenetrationM=max(s['diagnostics']['max_self_penetration_m'] for s in rows),
 maximumFloorPenetrationM=max(s['diagnostics']['max_floor_penetration_m'] for s in rows),
 maximumAnchorErrorM=max(s['diagnostics']['max_anchor_separation_m'] for s in rows),
 maximumJointLimitErrorRad=max(s['diagnostics']['max_limit_error_rad'] for s in rows),
 maximumRequestedMotorEffortNm=max(s['diagnostics']['requested_motor_effort_nm'] for s in rows),
 physicalFailure=None,qualification24Cases=False,recoveryDefaultEnabled=False,
 immutableRecordsAndUnrelatedFilesUnchanged=True,noExtraPhysicsAfterHistory=True,auditWallSeconds=time.monotonic()-began)
(out/'postrun-audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['stableDecisionMetrics','stableSampledLoads','finalCheckpointReloadPassed']},indent=2))
