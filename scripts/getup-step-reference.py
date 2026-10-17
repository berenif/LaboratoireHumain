#!/usr/bin/env python3
"""Pure-kinematic GetUp leg reference probe. This script never integrates physics.

A reachable foot path is not a physically executable step. This diagnostic asks
the frozen SLOW student how its *effective* target responds to that path before
any separately authorized physical experiment.
"""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def xyz(v):
    return np.array([v[k] for k in 'xyz'], dtype=float)


def transform(position, rotation):
    x, y, z, w = [rotation[k] for k in 'xyzw']
    # Match native glam's actual q*v*q-conjugate operation, including the small
    # recorded floating-point quaternion norm drift. Never normalize or edit the
    # physical checkpoint to make the mathematical reconstruction agree.
    norm2 = x*x+y*y+z*z+w*w
    out = np.eye(4)
    out[:3, :3] = [[norm2-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), norm2-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), norm2-2*(x*x+y*y)]]
    out[:3, 3] = xyz(position)
    return out


class Kinematics:
    def __init__(self, model):
        self.model = model
        self.parts = model['segments']
        self.nominal = np.array(model['referenceMetadata']['standingPose'])
        self.parent_frames = {}
        self.child_inverse = {}
        self.limits = np.zeros((21, 2))
        self.gears = np.zeros(21)
        for p in self.parts[1:]:
            j = p['jointProfile']
            self.parent_frames[p['id']] = transform(j['parentFrame']['anchor'], j['parentFrame']['rotation'])
            self.child_inverse[p['id']] = np.linalg.inv(transform(j['childFrame']['anchor'], j['childFrame']['rotation']))
            h = p['reference'].get('hinge')
            if h:
                a = j['axes'][0]
                self.limits[h['qposIndex']] = a['minRadians'], a['maxRadians']
                self.gears[h['qposIndex']] = h['gear']

    def forward(self, q, root):
        out = {'torso': root.copy()}
        for p in self.parts[1:]:
            rot = np.eye(4)
            h = p['reference'].get('hinge')
            if h:
                angle = q[h['qposIndex']]
                c, s = np.cos(angle), np.sin(angle)
                rot[1:3, 1:3] = [[c, -s], [s, c]]
            out[p['id']] = out[p['parent']] @ self.parent_frames[p['id']] @ rot @ self.child_inverse[p['id']]
        return out

    def bottom(self, foot, pose):
        p = next(p for p in self.parts if p['id'] == foot)
        return min((pose @ np.r_[xyz(c[k]), 1])[1]-c['radius']
                   for c in p['reference']['primitives'] for k in ['a', 'b'])

    def leg_ik(self, q, root, foot, target, orientation, max_delta=.6, indices=None):
        """Damped least-squares reference only; never edits a physical state."""
        idx = np.array(indices) if indices is not None else (np.arange(9, 15) if foot == 'left_foot' else np.arange(3, 9))
        q = np.array(q).copy()
        original = q.copy()
        def residual(angles):
            pose = self.forward(angles, root)[foot]
            # 0.1 m angular scale lets the limited ankle retain feasible pitch.
            cross = sum(np.cross(pose[:3, i], orientation[:3, i]) for i in range(3)) * .5
            return np.r_[target-pose[:3, 3], cross*.1]
        for _ in range(80):
            error = residual(q)
            if np.linalg.norm(error) < 1e-5:
                break
            jac = np.column_stack([(residual(q + np.eye(21)[i]*1e-4)-error)/1e-4 for i in idx])
            delta = -np.linalg.solve(jac.T@jac + np.eye(len(idx))*1e-5, jac.T@error)
            q[idx] += np.clip(delta, -.08, .08)
            q[idx] = np.clip(q[idx], np.maximum(self.limits[idx, 0], original[idx]-max_delta),
                            np.minimum(self.limits[idx, 1], original[idx]+max_delta))
        return q, self.forward(q, root)[foot], residual(q)

    def sagittal_heading(self, q, root, foot):
        poses = self.forward(q,root)
        thigh = next(p for p in self.parts if p['id'] == ('left_thigh' if foot == 'left_foot' else 'right_thigh'))
        axis = poses[thigh['parent']][:3,:3] @ self.parent_frames[thigh['id']][:3,:3] @ np.array([1.,0.,0.])
        heading = np.cross(axis,np.array([0.,1.,0.]))
        return heading/np.linalg.norm(heading)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output/'started-once').open('x').close()
    proc = subprocess.Popen([str(a.binary.resolve()), str(a.checkpoint.resolve()), '0', '--upright-entry'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    trace = []
    def command(request):
        proc.stdin.write(json.dumps(request)+'\n');proc.stdin.flush()
        row = json.loads(proc.stdout.readline());trace.append({'request':request,'response':row})
        assert row['ok'], row
        assert row['processNativeSubsteps'] == 0 and row['processPhysicsWallS'] == 0
        return row['result']
    state = command({'op':'observe'})
    snapshot = command({'op':'snapshot'})
    diag = command({'op':'diagnostic'})
    q = np.array(state['observation'][:21])
    kin = Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))
    root = transform(snapshot['segments'][0]['position'], snapshot['segments'][0]['rotation'])
    poses = kin.forward(q, root)
    parity = max(np.max(np.abs(poses[p['id']]-transform(p['position'],p['rotation'])))
                 for p in snapshot['segments'])
    assert parity < 2e-5, parity
    nominal = np.r_[kin.nominal, 1.4, 0., 0., 1.]
    base = command({'op':'referenceAction','now':nominal.tolist(),'ahead':nominal.tolist()})
    rows = []
    for foot in ['left_foot','right_foot']:
        foot_pose = poses[foot]
        for lift in [.025, .06]:
            target = foot_pose[:3,3].copy();target[1] += lift
            candidate, result, error = kin.leg_ik(q, root, foot, target, foot_pose)
            ref = nominal.copy()
            idx = np.arange(9,15) if foot == 'left_foot' else np.arange(3,9)
            # Apply only the IK displacement to the source nominal reference.
            ref[idx] += candidate[idx]-q[idx]
            response = command({'op':'referenceAction','now':ref.tolist(),'ahead':ref.tolist()})
            effective_change = np.array(response['effectiveTarget'])-base['effectiveTarget']
            rows.append({'foot':foot,'requestedLiftM':lift,'referenceDeltaRad':(candidate-q).tolist(),
                         'kinematicPositionErrorM':float(np.linalg.norm(error[:3])),
                         'kinematicWholeFootClearanceM':kin.bottom(foot,result),
                         'effectiveTargetChangeRad':effective_change.tolist(),
                         'response':response})
    after = command({'op':'snapshot'});assert snapshot == after
    proc.stdin.close();assert proc.wait(timeout=10) == 0
    report = {'physicsIntegrations':0,'kinematicsSnapshotParityMaxError':float(parity),
              'initial':state,'initialDiagnostic':diag,'baselineResponse':base,'rows':rows,
              'interpretation':'Reachability and frozen-policy sensitivity only; no executed step or learning.'}
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (a.output/'protocol.ndjson').write_text(''.join(json.dumps(row)+'\n' for row in trace))
    print(json.dumps({k:report[k] for k in ['physicsIntegrations','kinematicsSnapshotParityMaxError']}))
    print(json.dumps([{k:row[k] for k in ['foot','requestedLiftM','kinematicPositionErrorM','kinematicWholeFootClearanceM']} for row in rows],indent=2))


if __name__ == '__main__':
    main()
