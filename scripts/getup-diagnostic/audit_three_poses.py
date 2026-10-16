#!/usr/bin/env python3
"""Audit the three fixed pose histories only; never imports a simulator."""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/getup-three-poses-2026-10-16'
ledger=json.loads((ROOT/'docs/evidence/getup-three-poses-2026-10-16.json').read_text())
reload=json.loads((OUT/'checkpoint-reload-audit.json').read_text())
model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
summary=[]
for kind in ledger['cases']:
    directory=OUT/kind
    result=json.loads((directory/'result.json').read_text())
    rows=[json.loads(line) for line in (directory/'trace.ndjson').read_text().splitlines()]
    controller=json.loads((directory/'reference-report.json').read_text())
    assert result['harnessCompleted'] and result.get('harnessError') is None
    assert result.get('failure') is None
    assert result['counters']['real']==round(rows[-1]['simulation_time_s']*800)
    assert result['counters']['forecast']==controller['status']['forecast_substeps']
    assert result['counters']['total']==result['counters']['real']+result['counters']['forecast']
    native=0
    for row in rows:
        current=round(row['simulation_time_s']*800)
        assert current>=native and row['simulation_time_s']==current/800
        native=current
        assert row['stamp']['generation']==1 and row['counters']['strikes']==0
        assert row['diagnostics']['direct_pelvis_force_n']==row['diagnostics']['direct_pelvis_torque_nm']==0
        assert row['diagnostics']['requested_motor_effort_nm']<=920.001
    record=dict(case=kind,fixtureValid=result['fixtureValid'],rawOutcome=result['outcome'],
        physicalDurationS=rows[-1]['simulation_time_s'],preparationDurationS=None if result['triggerNative'] is None else result['triggerNative']/800,
        fixtureAdmission=result.get('fixtureAtTrigger'),finalFixtureState=result['fixture'],
        realNativeSteps=result['counters']['real'],forecastNativeSteps=result['counters']['forecast'],
        totalNativeSteps=result['counters']['total'],teacherDecisions=controller['status']['teacher_decisions'],
        slowDecisions=controller['status']['policy_decisions'],
        slowElapsedS=0 if controller['status']['started_at_s'] is None else rows[-1]['simulation_time_s']-controller['status']['started_at_s'],
        nativeStandingCriterionReached=controller['status']['prototype_standing_passed'],
        finalStableStandingS=controller['status']['stable_standing_s'],
        maximumSampledStableStandingS=max(s['diagnostics']['getup']['policy']['stable_standing_s'] for s in rows),
        finalMotion=rows[-1]['motion'],finalHeadM=next(p['position']['y'] for p in rows[-1]['segments'] if p['id']=='head'),
        maximumSelfPenetrationM=max(s['diagnostics']['max_self_penetration_m'] for s in rows),
        maximumFloorPenetrationM=max(s['diagnostics']['max_floor_penetration_m'] for s in rows),
        maximumAnchorErrorM=max(s['diagnostics']['max_anchor_separation_m'] for s in rows),
        maximumJointLimitErrorRad=max(s['diagnostics']['max_limit_error_rad'] for s in rows),
        maximumRequestedMotorEffortNm=max(s['diagnostics']['requested_motor_effort_nm'] for s in rows),
        physicalFailure=None,checkpointReloadPassed=next(c['passed'] for c in reload['cases'] if c['case']==kind))
    if result['fixtureValid']:
        fixture=result['fixtureAtTrigger']
        assert fixture['ready'] and fixture['settled_substeps']>=120
        assert fixture['maximum_body_com_speed_mps']<=.22 and fixture['maximum_angular_radps']<=.65
        assert fixture['ground_load_n']>=.8*model['totalMassKg']*9.81
        assert controller['impact_substep']==result['triggerNative']
        assert controller['status']['reference_seed_matches_real'] and controller['status']['teacher_terminal_reached']
        assert controller['teacher_failure'] is None
        first=next(i for i,s in enumerate(rows) if s['diagnostics']['getup']['policy']['prototype_standing_passed'])
        before=rows[first-1];after=rows[first]
        old_n=round(before['simulation_time_s']*800)
        old_stable=round(before['diagnostics']['getup']['policy']['stable_standing_s']*800)
        new_n=round(after['simulation_time_s']*800)
        assert old_stable>0 and new_n-old_n<800
        proof=old_n-old_stable+800
        assert old_n<proof<=new_n
        # Counter increments once per native integration or resets to0. Within
        # <800 steps, a newly latched1s flag must complete this existing chain.
        record.update(oneSecondProofPhysicalS=proof/800,
            oneSecondProofAfterTriggerS=(proof-result['triggerNative'])/800,
            proofInference='Derived from the last unlatched stable counter and next latched packet; native gap<1s rules out reset and a new800-step chain before first latch.',
            firstFlagArchivedAtS=after['simulation_time_s'],
            currentStabilityLostByFinalSnapshot=controller['status']['stable_standing_s']<1,
            outcome='native1s-reached-but-current-stability-lost')
        assert record['oneSecondProofAfterTriggerS']<=25
    else:
        assert kind=='side' and controller['status']['forecast_substeps']==0
        assert not controller['student_decisions'] and not controller['teacher_frames']
        f=result['fixture']
        assert f['maximum_body_com_speed_mps']<=.22 and f['maximum_angular_radps']<=.65
        assert f['torso_face_up']<-.35 and f['torso_side_up']<.8
        record.update(outcome='invalid-lateral-fixture-settled-in-oblique-prone-orientation',
            controllerEvaluated=False,remainingWork='Define and mechanically validate a settled lateral fixture without changing controller parameters; then evaluate that same controller. No new fixture variant launched in this batch.')
    summary.append(record)
assert ledger['status']=='closed' and ledger['actualBatchWallSeconds']<=180
assert sum(c['totalNativeSteps'] for c in summary)==24640
for p,h in ledger['inputs'].items():assert sha(ROOT/p)==h,p
for p,h in ledger['immutablePriorRecords'].items():assert sha(ROOT/p)==h,p
for p,h in ledger['protectedUnrelatedFilesSha256'].items():assert sha(ROOT/p)==h,p
report=dict(physicsIntegrationsForAnalysis=0,cases=summary,
 actualBatchWallSeconds=ledger['actualBatchWallSeconds'],actualCaseWallSeconds=ledger['actualCaseWallSeconds'],
 maximumActualPhysicalWallSeconds=180,unusedActualWallAllowanceSeconds=180-ledger['actualBatchWallSeconds'],
 actualNativeSteps=24640,fixtureValidatedCases=2,controllerEvaluatedCases=2,native1sCriterionReachedCases=2,
 currentlyStableAtFinalSnapshotCases=0,invalidFixtures=1,qualification24Cases=False,
 sameModelWeightsAndParameters=True,oldRecordsAndUnrelatedFilesUnchanged=True,
 interpretation='Two valid fixtures reached the existing native1s standing criterion, but both lost current stability before their final sampled packets. The lateral definition rolled into an oblique prone equilibrium and was not admitted; this is not a controller failure. No robustness qualification is inferred.')
(OUT/'postrun-audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
