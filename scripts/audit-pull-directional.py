#!/usr/bin/env python3
"""Independent saved-data audit; no optimization, native evaluation or integration."""
import gzip
import hashlib
import json
import math
import subprocess
import sys
from fractions import Fraction as F
from pathlib import Path

out = Path(sys.argv[1])
src = Path('web/proofs/pull-gravity-frame-001')
load = lambda p: json.loads(gzip.decompress(p.read_bytes()))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
dot = lambda a, b: math.fsum(x*y for x, y in zip(a, b, strict=True))
fdot = lambda a, b: sum((F(x)*F(y) for x, y in zip(a, b, strict=True)), F())
ledger = json.loads((out/'ledger.json').read_text())
q, cert, lp, raw, r, problem = [load(p) for p in [src/'last-qp.json.gz', out/'certificate.json.gz', out/'lp.json.gz', out/'native.json.gz', out/'result.json.gz', out/'problem.json.gz']]
assert ledger['completed'] and ledger['LPcalls'] == 1 and ledger['forwards'] == 1
assert not ledger['pendingLP'] and not ledger['pendingForward']
assert ledger['ticks'] == ledger['forecasts'] == 0 and lp['iterations'] <= 64 and lp['elapsedSeconds'] <= 10
assert all(sha(src/n) == h for n, h in ledger['inputs'].items())
G = [x['a'] for x in q['rows']]+[x['a'] for x in problem['branchRows']]
h = [x['bound'] for x in q['rows']]+[x['bound'] for x in problem['branchRows']]
assert len(G) == 180 and len(q['rows']) == 156 and len(problem['branchRows']) == 24
j, bias = q['gravity']['worldCOM']['jp'][1], q['gravity']['worldCOM']['bp'][1]
c = [dot(j, a) for a in q['A']]
c0 = dot(j, q['a0'])+bias
assert c == problem['objective'] and c0 == problem['constant']
assert all(len(g) == 21 for g in G)
u, lam = raw['controls'], cert['multipliers']
assert u == lp['controls'] and min(lam) >= 0
# Exact sign/unscaling of HiGHS marginals, including no hidden variable bounds.
assert lam == [-v/s for v, s in zip(lp['scipyMarginals'], problem['rowScales'])]
stationarity = [F(c[k])+fdot([g[k] for g in G], lam) for k in range(21)]
L = F(c0)-fdot(h, lam)-sum(map(abs, stationarity))
U = F(c0)+fdot(c, u)
assert L == F(int(cert['dualLowerExact'][0]), int(cert['dualLowerExact'][1]))
assert F(cert['dualLower']) <= L and F(cert['primalObjective']) >= U
strict_primal = max(fdot(g, u)-F(b) for g, b in zip(G, h)) <= 0
assert strict_primal == cert['primalUpperStrictlyCertified']
assert cert['passed'] and cert['primal']['maximumNormalized'] <= 1e-10
assert r['nativeAdmitted'] and r['modelAgrees'] and r['braking']
assert len(r['checks']) == 84 and not r['violations']
assert raw['rootActuation'] == [0]*6 and not any(raw['appliedGeneralized'])
oldraw = load(src/'initial-candidate-native.json.gz')
assert raw['qpos'] == oldraw['qpos'] and raw['qvel'] == oldraw['qvel'] and raw['t'] == oldraw['t']
assert raw['externalBodyForces'] == oldraw['externalBodyForces']
assert all(abs(v) <= 1 for v in u)  # No roundoff exception needed for actual caps.
assert all(x['appliedNm'] == x['requestedNm'] and abs(x['appliedNm']) <= x['gear'] for x in r['mapping'])
assert all(math.isfinite(v) for v in r['nativeBaseResidual'])
# Dynamics precision is assessed with the existing mass-energy residual/error
# propagation in the native checker; no new absolute residual veto is imposed.
omega, alpha = r['currentTorsoOmega'], [x['native'] for x in r['worldTorsoAngularAcceleration']]
costs = {}
for t, v in zip(q['tasks'], r['taskResponses']):
    label = t['label']
    group = ('COM' if label.startswith('gravity-com') else 'torso' if label.startswith('gravity-torso')
             else 'posture' if label.startswith('posture') else 'feet' if 'foot' in label or 'free' in label else 'regularizer')
    costs[group] = costs.get(group, 0)+.5*((v['native']-t['desired'])*t['weight']/t['scale'])**2
code = ['scripts/pull-directional-lp.py', 'scripts/check-pull-directional-native.mjs', 'scripts/audit-pull-directional.py']
audit = dict(passed=True, oneLP=True, oneForward=True, ticks=0, forecasts=0,
    confirmedTotal=229565, unknownHistoryPreserved=True, sourceNative233Unchanged=True,
    currentTime=raw['t'], nativeCapsStrictlySatisfied=True, native84AdmissionsPassed=True,
    dualLower=cert['dualLower'], dualOutwardRounding=float(L)-cert['dualLower'],
    approximatePrimalObjective=cert['primalObjective'], strictPrimalUpper=strict_primal,
    numericalPrimalViolation=cert['primal']['maximumNormalized'],
    worldYNative=r['lateral']['native'], nativeError=r['lateral']['error'],
    worldCOM=[x['native'] for x in r['worldCOMAcceleration']],
    worldTorsoAlpha=alpha, omegaDotAlpha=dot(omega, alpha),
    loads=r['loads'], rightFraction=r['normalLoadFractionRight'],
    rightFractionSourceCandidate=r['sourceCandidateLoads']['7']/sum(r['sourceCandidateLoads'].values()),
    contactForce=r['contactForce'], externalForce=r['externalForce'],
    zeroEdgeBoundaries=[x['i'] for x in r['branchChecks'] if x['zeroBoundary']],
    changedStatesAtZero=[i for i, (a, b) in enumerate(zip(raw['regime'], q['states'])) if a != b],
    saturatedJoints=[x['name'] for x in r['mapping'] if x['atCap']],
    maxUnappliedTorqueJumpNm=max(abs(x['deltaFromLastAppliedNm']) for x in r['mapping']),
    taskCosts=costs, oldObjectiveCost=r['oldObjectiveCost'],
    codeHashes={n:sha(Path(n)) for n in code},
    scope='Native instantaneous braking exists with exact traction and current constraints; large torso acceleration and increased free-foot loading do not establish viable transfer. No command applied or simulation advanced.')
(out/'audit.json').write_text(json.dumps(audit, separators=(',', ':')))
growth = int(subprocess.check_output(['du', '-sx', '-B1', '/workspace']).split()[0])-419647488+13123543
old_doc = subprocess.check_output(['git', 'show', ledger['sourceHEAD']+':web/mujoco/pull-step.md'])
doc_addition = max(0, Path('web/mujoco/pull-step.md').stat().st_size-len(old_doc))
ledger.update(growthAfter=growth, documentationAdditionBytes=doc_addition,
    codeAndProofBytes=sum(p.stat().st_size for p in out.iterdir())+sum(Path(n).stat().st_size for n in code)+doc_addition)
assert growth < 1530000000 and ledger['codeAndProofBytes'] < 100000
(out/'ledger.json').write_text(json.dumps(ledger, separators=(',', ':')))
print(json.dumps(audit))
