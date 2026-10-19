#!/usr/bin/env python3
"""Compact saved-frame arm/knee comparison. No physical integrations."""
import argparse
import json
from collections import OrderedDict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from audit_journal_io import digest as sha_file, json_array_items

ROOT = Path(__file__).resolve().parents[2]
DT = .00125


def audit_control_notes(notes):
    missing = object()
    minimum_gap = max_offset = missing
    for note in notes:
        for adjustment in note['selfAvoidance']:
            value = adjustment['separationM']
            minimum_gap = value if minimum_gap is missing else min(minimum_gap, value)
        offset = max(abs(np.array(note.get('postureExperiment', {}).get('armBalanceOffsetRad', [0.]))))
        max_offset = offset if max_offset is missing else max(max_offset, offset)
    return (None if minimum_gap is missing else minimum_gap,
            0. if max_offset is missing else max_offset)


def arm_geometry(qpos):
    # Source body's exact FK implementation, imported without any engine step.
    from source_step_preview import fk, P
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    kin = fk.Kinematics(model)
    q = np.array(qpos)
    root = np.eye(4)
    root[:3, :3] = P.T @ Rotation.from_quat(q[[4,5,6,3]]).as_matrix() @ P
    root[:3, 3] = P.T @ q[:3]
    poses = kin.forward(q[7:], root)
    result = {}
    for side in ('right', 'left'):
        shoulder = poses[side+'_upper_arm'][:3,3]
        hand = poses[side+'_hand'][:3,3]
        # Native model Y vertical. Horizontal displacement is geometric only;
        # it is not a collider clearance, which is separately reported.
        result[side] = dict(handHorizontalDistanceFromShoulderM=float(np.linalg.norm((hand-shoulder)[[0,2]])),
                            handBelowShoulderM=float(shoulder[1]-hand[1]))
    return result


def audit(folder):
    saved = json.loads((folder/'summary.json').read_text())
    multi = json.loads((folder/'multistep-summary.json').read_text())
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    names, gears = ['']*21, np.zeros(21)
    for part in model['segments']:
        hinge = part['reference'].get('hinge')
        if hinge:
            i = hinge['qposIndex']; names[i] = hinge['name']; gears[i] = hinge['gear']
    qmin, qmax = np.full(21, np.inf), np.full(21, -np.inf)
    vmax, taumax, work, u2 = [np.zeros(21) for _ in range(4)]
    previous_q = np.array(saved['entry']['qpos'])
    previous_slip = np.zeros(2)
    continuity = 0.
    warnings = flight = bad_rays = 0
    maxpen = nonfoot = 0.
    minjoint = np.inf; last_outside = None
    phases = OrderedDict(); count = 0
    for line in (folder/'frames.ndjson').open():
        row = json.loads(line)
        count += 1
        assert row['sourceStep'] == count
        assert abs(row['elapsedS']-DT*count) < 1e-11
        q = np.array(row['qpos'])[7:]; v = np.array(row['qvel'])[6:]
        tau = np.array(row['actualMotorTorquesNm']); u = tau/gears
        qmin, qmax = np.minimum(qmin,q), np.maximum(qmax,q)
        vmax, taumax = np.maximum(vmax,abs(v)), np.maximum(taumax,abs(tau))
        work += abs(tau*v)*DT; u2 += u*u
        continuity = max(continuity,float(max(abs(q-previous_q[7:]-DT*v))))
        previous_q = np.array(row['qpos'])
        warnings = max(warnings,row['warnings'])
        flight += all(f['loadN']<3 for f in row['feet'])
        nonfoot = max(nonfoot,row['nonfootLoadN'])
        maxpen = max(maxpen,row['maximumSelfPenetrationM'])
        minjoint = min(minjoint,row['jointLimitMinimumMarginRad'])
        if row['jointLimitMinimumMarginRad'] < 0: last_outside = row['elapsedS']
        assert len(row['jointGroundRays']) == 22
        assert all(len(f['groundRays']) == 4 for f in row['feet'])
        bad_rays += sum(not r['hit'] or not r['ownBodyExcluded'] for r in row['jointGroundRays'])
        bad_rays += sum(not r['hit'] or not r['ownBodyExcluded'] for f in row['feet'] for r in f['groundRays'])
        key = (row['stepIndex'],row['phase'])
        phase = phases.setdefault(key,dict(step=key[0],phase=key[1],samples=0,
                maximumArmJointSpeedRadps=0.,kneeMinDeg=np.full(2,np.inf),kneeMaxDeg=np.full(2,-np.inf),
                maximumMotorFraction=0.,minimumJointReserveRad=np.inf,minimumComCaptureMarginM=None,
                maximumSelfPenetrationM=0.,loadedSlipIncrementM=np.zeros(2)))
        phase['samples'] += 1
        phase['maximumArmJointSpeedRadps'] = max(phase['maximumArmJointSpeedRadps'],float(max(abs(v[15:]))))
        phase['kneeMinDeg'] = np.minimum(phase['kneeMinDeg'],np.rad2deg(q[[6,12]]))
        phase['kneeMaxDeg'] = np.maximum(phase['kneeMaxDeg'],np.rad2deg(q[[6,12]]))
        phase['maximumMotorFraction'] = max(phase['maximumMotorFraction'],float(max(abs(u))))
        phase['minimumJointReserveRad'] = min(phase['minimumJointReserveRad'],row['jointLimitMinimumMarginRad'])
        if row['minimumComCaptureMarginM'] is not None:
            value = row['minimumComCaptureMarginM']
            phase['minimumComCaptureMarginM'] = value if phase['minimumComCaptureMarginM'] is None else min(value,phase['minimumComCaptureMarginM'])
        phase['maximumSelfPenetrationM'] = max(phase['maximumSelfPenetrationM'],row['maximumSelfPenetrationM'])
        slip = np.array(row['loadedSlipDistanceM'])
        phase['loadedSlipIncrementM'] += slip-previous_slip
        previous_slip = slip
        phase['endpointArmGeometry'] = row['qpos']
    assert count == multi['actualIntegrations']
    assert max(abs(previous_slip-multi['globalLoadedSlipM'])) < 1e-11
    minimum_gap, max_offset = audit_control_notes(json_array_items(folder/'control.json'))
    phase_rows = []
    for phase in phases.values():
        phase['durationS'] = phase['samples']*DT
        phase['endpointArmGeometry'] = arm_geometry(phase['endpointArmGeometry'])
        phase_rows.append(phase)
    joint_rows = [dict(joint=name,minimumAngleDeg=float(np.rad2deg(qmin[i])),maximumAngleDeg=float(np.rad2deg(qmax[i])),
                       excursionDeg=float(np.rad2deg(qmax[i]-qmin[i])),maximumSpeedRadps=float(vmax[i]),
                       maximumTorqueNm=float(taumax[i]),rmsMotorFraction=float(np.sqrt(u2[i]/count)),
                       absoluteMechanicalWorkJ=float(work[i])) for i,name in enumerate(names)]
    result = dict(savedDataOnly=True,physicsIntegrationsForAudit=0,stopped=multi['stopped'],passed=multi['passed'],
        actualIntegrations=count,physicsSimSeconds=count*DT,finalQuietS=multi['finalQuietS'],
        steps=multi['steps'],globalLoadedSlipM=previous_slip.tolist(),
        finalComCaptureMarginM=multi['final']['minimumComCaptureMarginM'],
        maximumMotorFraction=float(max(taumax/gears)),maximumArmJointSpeedRadps=float(max(vmax[15:])),
        maximumSelfPenetrationM=maxpen,minimumMeasuredSelfSeparationM=minimum_gap,
        maximumNonfootLoadN=nonfoot,flightSamples=int(flight),warnings=warnings,missingOrSelfEligibleGroundRays=int(bad_rays),
        jointRaysPerSample=22,footRaysPerSample=8,hingeContinuityMaximumErrorRad=continuity,
        inheritedInitialJointMarginRad=saved['entry']['jointLimitMinimumMarginRad'],minimumJointReserveRad=minjoint,
        lastOutsideJointLimitTimeS=last_outside,maximumArmBalanceOffsetRad=float(max_offset),
        totalAbsoluteMechanicalWorkJ=float(sum(work)),armAbsoluteMechanicalWorkJ=float(sum(work[15:])),
        initialArmGeometry=arm_geometry(saved['entry']['qpos']),finalArmGeometry=arm_geometry(multi['final']['qpos']),
        finalKneeAnglesDeg=np.rad2deg(np.array(multi['final']['qpos'])[7:][[6,12]]).tolist(),
        phases=phase_rows,joints=joint_rows,
        hashes={n:sha_file(folder/n) for n in ('frames.ndjson','control.json','summary.json','multistep-summary.json','final.bin')},
        modelSHA256=saved['modelSHA256'],helperSHA256=saved['helperSHA256'],
        physicalStateWritesAfterInitialization=saved['physicalStateWritesAfterInitialization'],
        rootAssistance=saved['rootAssistance'])
    # Summaries have separate quiet/transition admission gates. A completed
    # trajectory with inherited entry limit overrun is not an all-sample pass.
    result['strictAllSampleJointLimitsPass'] = bool(minjoint>=0)
    return json.loads(json.dumps(result,default=lambda x:x.tolist()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path)
    args = p.parse_args()
    result = audit(args.folder)
    (args.output or args.folder/'arm-knee-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('stopped','passed','actualIntegrations','globalLoadedSlipM','maximumArmJointSpeedRadps','finalKneeAnglesDeg','maximumMotorFraction','minimumMeasuredSelfSeparationM')},indent=2))


if __name__ == '__main__':
    main()
