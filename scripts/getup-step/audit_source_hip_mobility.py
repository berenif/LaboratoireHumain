#!/usr/bin/env python3
"""Saved-data-only comparison of the paired hip-ROM experiment."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from source_step_probe import ROOT, sha
from audit_source_multistep import audit


def limits_from_session(session):
    root = ET.parse(session/'source-model/humanoid_high_freq.xml').getroot()
    joints = [j for j in root.findall('.//worldbody//joint') if 'name' in j.attrib]
    return [j.attrib['name'] for j in joints], np.deg2rad([list(map(float,j.attrib['range'].split())) for j in joints])


def measure(folder, session, original_limits):
    summary = json.loads((folder/'multistep-summary.json').read_text())
    entry = json.loads((folder/'summary.json').read_text())['entry']
    names, limits = limits_from_session(session)
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    gears = np.zeros(21)
    for p in model['segments']:
        hinge = p['reference'].get('hinge')
        if hinge:
            gears[hinge['qposIndex']] = hinge['gear']
    minimum = np.full(21,np.inf); maximum = -minimum.copy(); margins = minimum.copy(); original_margin = minimum.copy()
    torque_sq = np.zeros(21); peak_effort = np.zeros(21); abs_work = 0.; sum_effort = 0.
    oldq = np.array(entry['qpos']); qerror = 0.; n = 0; outside_original_hip = 0; last_bad = None
    hip = np.array(['_hip_' in name for name in names]); hipframe = 0
    for line in (folder/'frames.ndjson').open():
        row = json.loads(line); q = np.array(row['qpos']); v = np.array(row['qvel']); n += 1
        actual_margins = np.minimum(q[7:]-limits[:,0],limits[:,1]-q[7:])
        np.testing.assert_allclose(actual_margins,row['jointLimitMarginsRad'],rtol=0,atol=1e-14)
        old_margins = np.minimum(q[7:]-original_limits[:,0],original_limits[:,1]-q[7:])
        minimum = np.minimum(minimum,q[7:]); maximum = np.maximum(maximum,q[7:]); margins = np.minimum(margins,actual_margins); original_margin = np.minimum(original_margin,old_margins)
        outside_original_hip += int(np.min(old_margins[hip]) < 0.)
        if np.min(actual_margins) < 0:
            last_bad = row['elapsedS']
        effort = abs(np.array(row['actualMotorTorquesNm']))/gears
        torque_sq += effort**2; sum_effort += float(np.sum(effort)); peak_effort = np.maximum(peak_effort,effort)
        abs_work += float(np.sum(abs(np.array(row['actualMotorTorquesNm'])*v[6:])))*.00125
        qerror = max(qerror, float(max(abs(q[7:]-oldq[7:]-.00125*v[6:])))); oldq=q
    qualified = audit(folder) if summary['passed'] else None
    if qualified:
        (folder/'saved-data-audit.json').write_text(json.dumps(qualified,indent=2)+'\n')
    result = dict(stopped=summary['stopped'], passed=summary['passed'], actualIntegrations=n, physicsSimSeconds=n*.00125,
                  finalQuietS=summary['finalQuietS'], globalLoadedSlipM=summary['globalLoadedSlipM'],
                  achievedDistancesM=[s.get('achievedForwardM') for s in summary['steps']],
                  lastOutsideOwnJointLimitTimeS=last_bad, originalHipLimitExceededSamples=outside_original_hip,
                  hingePositionVelocityContinuityMaxErrorRad=qerror, absoluteJointMechanicalWorkJ=abs_work,
                  meanSumNormalizedMotorEffort=sum_effort/n,
                  hips=[dict(joint=name, observedMinDeg=float(np.rad2deg(minimum[i])), observedMaxDeg=float(np.rad2deg(maximum[i])),
                             observedSpanDeg=float(np.rad2deg(maximum[i]-minimum[i])), minimumOwnLimitReserveDeg=float(np.rad2deg(margins[i])),
                             minimumOriginalLimitReserveDeg=float(np.rad2deg(original_margin[i])),
                             rmsNormalizedMotorEffort=float(np.sqrt(torque_sq[i]/n)), maximumNormalizedMotorEffort=float(peak_effort[i])) for i,name in enumerate(names) if hip[i]],
                  allJoints=[dict(joint=name, minimumLimitReserveDeg=float(np.rad2deg(margins[i])), rmsNormalizedMotorEffort=float(np.sqrt(torque_sq[i]/n)),
                                  maximumNormalizedMotorEffort=float(peak_effort[i])) for i,name in enumerate(names)],
                  savedDataQualification=qualified, modelSHA256=sha(session/'source-model/humanoid_high_freq.xml'))
    return result


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('pair',type=Path); parser.add_argument('--historical-reference',type=Path)
    args=parser.parse_args(); _, original=limits_from_session(args.pair/'reference-session')
    result=dict(experiment='bilateral-hip-ROM-plus10deg-per-bound', savedDataOnly=True, auditPhysicsIntegrations=0,
                preflight=json.loads((args.pair/'preflight.json').read_text()), arms={})
    for name in ('reference','wider'):
        result['arms'][name]=measure(args.pair/name/'generation-001',args.pair/f'{name}-session',original)
    if args.historical_reference:
        historical=json.loads((args.historical_reference/'multistep-summary.json').read_text())
        fresh=json.loads((args.pair/'reference/generation-001/multistep-summary.json').read_text())
        result['referenceReproducesHistoricalMeasuredState']=historical['final']==fresh['final'] and historical['steps']==fresh['steps']
        assert result['referenceReproducesHistoricalMeasuredState']
    result['adaptation']=json.loads((args.pair/'wider-session/adaptation-receipt.json').read_text())
    result['scope']=dict(originalModelChanged=False, defaultPromoted=False, controllerChanged=False, motorPowerChanged=False,
                         frictionChanged=False, requestedDistancesChanged=False, phaseTimingChanged=False,
                         physicalRootAssistance=False, physicalStateInstallationsBetweenSteps=0,
                         initialState='identical measured qpos/qvel, fresh model-specific solver state', anatomicalRealismClaim=False)
    (args.pair/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({name:{k:value for k,value in arm.items() if k not in ('hips','allJoints','savedDataQualification')} for name,arm in result['arms'].items()},indent=2))

if __name__=='__main__':
    main()
