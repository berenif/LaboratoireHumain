#!/usr/bin/env python3
"""Read-only reduction of the two saved B15 native trials; never launches physics.
Run with python3 scripts/reduce-getup-native-force-audit.py RAW_AUDIT_FOLDER.
The residual is a continuous-model trapezoidal diagnostic, not an exact native
implicit/split-stage integration identity. Unknown native stop impulses are
excluded using the QP's declared active stop indices, without inferring values.
"""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
if len(sys.argv) != 2:
    raise SystemExit("Usage: reduce-getup-native-force-audit.py RAW_AUDIT_FOLDER")
BASE = Path(sys.argv[1])
sp = importlib.util.spec_from_file_location('contact_dynamics_audit', ROOT / 'scripts/getup-contact-dynamics.py')
d = importlib.util.module_from_spec(sp)
sp.loader.exec_module(d)
model = json.loads((ROOT / 'rust/crates/model/data/getup-v1.json').read_text())
controller = d.ContactDynamics(model)
actuator_indices = np.zeros(21, dtype=int)
for part in model['segments']:
    h = part['reference'].get('hinge')
    if h:
        actuator_indices[h['qposIndex']] = h['actuatorIndex']


def state(frame):
    native_com = np.zeros(3)
    for part, segment in zip(model['segments'], frame['snapshot']['segments']):
        pose = d.reference.transform(segment['position'], segment['rotation'])
        native_com += part['massKg'] * (pose[:3, 3] + pose[:3, :3] @ d.reference.xyz(part['goldenMass']['centerOfMass']))
    native_com /= model['totalMassKg']
    obs = frame['physicalObservation']
    diag = dict(contactMotion=frame['contactMotion'], com=dict(zip('xyz', native_com)),
                comVelocity=dict(zip('xyz', [obs[38], obs[39], obs[37]])))
    return controller.state(obs, frame['snapshot'], diag)


report = dict(
    interpretation='Two independent same-checkpoint25ms trials, all20 native1/800s impulse observations. No additional integrations. Not a step or long-hold qualification.',
    residualDefinition='Sum_k [0.5*(M_before+M_after)*(v_after-v_before) + 0.5*dt*(bias_before+bias_after) - dt*tau_actual - sum_contacts(J_after.T*reported_world_impulse)]. Divide by25ms for equivalent force/torque. This is approximate continuous-model quadrature, not a native integration identity.',
    contactJacobianDefinition='live uses current material point; frozen uses normalized FK body COM plus exported solverLeverFromCom. Every point impulse equals normal*normalImpulseNs+tangentImpulseWorldNs, directed onto the foot.',
    cases=[])
for mode in ['live-geometric', 'native-frozen']:
    x = json.loads((BASE / (mode + '.json')).read_text())
    frames = x['transition']['native_dynamics_audit']
    assert len(frames) == 21
    states = [state(f) for f in frames]
    first, final = states[0], states[-1]
    predicted = np.array(x['prediction']['qdd'])
    actual = (final['v'] - first['v']) / .025
    impulses = {key: np.zeros(27) for key in ['live', 'frozen']}
    residuals = {key: [] for key in impulses}
    inertia, bias, motor = np.zeros((3, 27))
    stop_coordinates = [6 + stop['qpos'] for stop in x['prediction']['activeStops']]
    keep = [i for i in range(27) if i not in stop_coordinates]
    for k in range(1, len(frames)):
        before, after = states[k - 1], states[k]
        inertial_impulse = .5 * (before['M'] + after['M']) @ (after['v'] - before['v'])
        bias_impulse = .000625 * (before['bias'] + after['bias'])
        motor_impulse = np.r_[np.zeros(6), np.array(frames[k]['motorTorquesNmActuatorOrder'])[actuator_indices]] * .00125
        inertia += inertial_impulse
        bias += bias_impulse
        motor += motor_impulse
        contact_impulses = {key: np.zeros(27) for key in impulses}
        for foot in frames[k]['contactMotion']['feet']:
            body = foot['body']
            pose = after['geometry'][0][body]
            com = pose[:3, 3] + d.normalized_rotation(pose[:3, :3]) @ d.reference.xyz(controller.by_id[body]['goldenMass']['centerOfMass'])
            for contact in foot['contacts']:
                impulse = d.reference.xyz(contact['normal']) * contact['normalImpulseNs'] + d.reference.xyz(contact['tangentImpulseWorldNs'])
                points = dict(live=d.reference.xyz(contact['point']), frozen=com + d.reference.xyz(contact['solverLeverFromCom']))
                for key, point in points.items():
                    jacobian, _ = controller.point_jacobian(after['geometry'], body, point)
                    contact_impulses[key] += jacobian.T @ impulse
        for key in impulses:
            residuals[key].append(inertial_impulse + bias_impulse - motor_impulse - contact_impulses[key])
            impulses[key] += contact_impulses[key]
    actual_com_acc = (final['Jcom'] @ final['v'] - first['Jcom'] @ first['v']) / .025
    feet = []
    for i, body in enumerate(['left_foot', 'right_foot']):
        pose, pose2 = first['geometry'][0][body], first['geometryNext'][0][body]
        _, jw = controller.point_jacobian(first['geometry'], body, pose[:3, 3])
        _, jw2 = controller.point_jacobian(first['geometryNext'], body, pose2[:3, 3])
        angular_bias = (jw2 - jw) @ first['v'] / first['directionalEpsilon']
        omega0 = d.reference.xyz(frames[0]['contactMotion']['feet'][i]['angularVelocityRadps'])
        omega1 = d.reference.xyz(frames[-1]['contactMotion']['feet'][i]['angularVelocityRadps'])
        feet.append(dict(body=body, predictedAngularAccelerationWorldRadps2=(jw @ predicted + angular_bias).tolist(), actualFinalVelocityAngularAccelerationWorldRadps2=((omega1 - omega0) / .025).tolist()))
    rotations, angular_velocities = [], []
    for frame in frames:
        segment = next(p for p in frame['snapshot']['segments'] if p['id'] == 'right_foot')
        rotations.append(Rotation.from_quat([segment['rotation'][k] for k in 'xyzw']).as_matrix())
        angular_velocities.append(d.reference.xyz(frame['contactMotion']['feet'][1]['angularVelocityRadps']))
    pose_rate = np.array([Rotation.from_matrix(b @ a.T).as_rotvec() / .00125 for a, b in zip(rotations, rotations[1:])])
    pose_final_velocity_difference = pose_rate - np.array(angular_velocities)[1:]
    case = dict(rightPoseTransport=dict(
                    meanPoseRateMinusFinalVelocityWorldRadps=pose_final_velocity_difference.mean(axis=0).tolist(),
                    minimumPoseRateMinusFinalVelocityWorldRadps=pose_final_velocity_difference.min(axis=0).tolist(),
                    maximumPoseRateMinusFinalVelocityWorldRadps=pose_final_velocity_difference.max(axis=0).tolist(),
                    totalPoseRotationWorldRad=Rotation.from_matrix(rotations[-1] @ rotations[0].T).as_rotvec().tolist()), mode=mode, rawArtifact=(mode + '.json'), qddPredictionErrorNorm=float(np.linalg.norm(actual - predicted)),
                predictedCOMAccelerationWorldMps2=x['prediction']['achievedComAcceleration'], actualCOMAccelerationWorldMps2=actual_com_acc.tolist(),
                comAccelerationPredictionErrorNormMps2=float(np.linalg.norm(actual_com_acc - x['prediction']['achievedComAcceleration'])),
                feet=feet, excludedGeneralizedStopCoordinates=stop_coordinates, impulseClosure={})
    for key in impulses:
        residual = (inertia + bias - motor - impulses[key]) / .025
        samples = np.array(residuals[key]) / .00125
        case['impulseClosure'][key] = dict(rootForceResidualEquivalentN=residual[:3].tolist(), rootMomentResidualEquivalentNm=residual[3:6].tolist(),
                nonStopGeneralizedResidualNormEquivalent=float(np.linalg.norm(residual[keep])), nonStopMaximumAbsoluteResidualEquivalent=float(max(abs(residual[keep]))),
                perNativeNonStopResidualRMSEEquivalent=float(np.sqrt(np.mean(samples[:, keep] ** 2))))
    report['cases'].append(case)
report['remainingMismatch'] = ['Native yaw acceleration remains near zero while the QP predicts positive yaw acceleration, consistent with missing sticking/friction-mode complementarity.',
    'Reported final generalized velocity is relaxed after biased-pass pose integration. Pose transport and final velocity remain distinct; separate pose-rate evidence must be retained.',
    'Nonzero impulse residual includes continuous-model quadrature, internal implicit dynamics, frozen-lever timing and f32 effects. No exact momentum certificate is claimed.']
output = BASE / 'impulse-model-reduction.json'
output.write_text(json.dumps(report, indent=2) + '\n')
print(output)
for case in report['cases']:
    print(case['mode'], 'qddError', case['qddPredictionErrorNorm'], 'comAccError', case['comAccelerationPredictionErrorNormMps2'], 'closure', {k: v['nonStopGeneralizedResidualNormEquivalent'] for k, v in case['impulseClosure'].items()})
