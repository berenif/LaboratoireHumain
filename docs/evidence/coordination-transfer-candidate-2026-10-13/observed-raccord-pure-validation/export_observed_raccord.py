#!/usr/bin/env python3
"""Offline arithmetic reconstruction only; no Rust, FK, world or solver call."""
from pathlib import Path
import bisect
import hashlib
import json
import math
import struct

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
ENTRY = ROOT / 'docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json'
PROGRAM = ROOT / 'docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json'
MODEL = ROOT / 'rust/crates/model/data/canonical-v1.json'
HELPER = ROOT / 'rust/crates/sim/src/recovery/planner/observed_raccord.rs'
LOG = OUT / 'observed-raccord-tests.log'
f = lambda v: struct.unpack('f', struct.pack('f', v))[0]
add = lambda a, b: f(a + b)
sub = lambda a, b: f(a - b)
mul = lambda a, b: f(a * b)
div = lambda a, b: f(a / b)
bits = lambda v: struct.unpack('I', struct.pack('f', v))[0]
dt = f(1 / 240)


def coordinates(rows):
    return [[f(v) for v in row] for row in rows]


entry = json.loads(ENTRY.read_text())['commandEntry']
model = json.loads(MODEL.read_text())
program = json.loads(PROGRAM.read_text())
keys = program['keyframes']
for key in keys:
    key['positions'] = coordinates(key['positions'])
    key['velocities'] = coordinates(key['velocities'])
clocks = [key['substep'] for key in keys]
previous = coordinates(entry['previous_q'])
previous_dq = coordinates(entry['previous_dq'])
target = keys[clocks.index(432)]['positions']
assert all(v == 0 for row in keys[clocks.index(432)]['velocities'] for v in row)


def source_q(step):
    """Same f32 operation order as CompiledProgram::command; positions only."""
    if step >= clocks[-1]:
        return keys[-1]['positions']
    right = bisect.bisect_right(clocks, step)
    a, b = keys[right - 1], keys[right]
    if a['substep'] == step:
        return a['positions']
    span = mul(f(b['substep'] - a['substep']), dt)
    u = div(f(step - a['substep']), f(b['substep'] - a['substep']))
    u2, u3 = mul(u, u), mul(mul(u, u), u)
    h0 = add(sub(mul(2., u3), mul(3., u2)), 1.)
    h1 = add(sub(u3, mul(2., u2)), u)
    h2 = add(mul(-2., u3), mul(3., u2))
    h3 = sub(u3, u2)
    return [[add(add(add(mul(h0, a['positions'][i][k]),
        mul(mul(h1, span), a['velocities'][i][k])), mul(h2, b['positions'][i][k])),
        mul(mul(h3, span), b['velocities'][i][k])) for k in range(3)] for i in range(25)]


displacement = 0.
duration_floor = 0.
for i in range(25):
    for k in range(3):
        delta = target[i][k] - previous[i][k]
        displacement = max(displacement, abs(delta))
        if delta:
            denominator = 1. + math.copysign(1., delta) * previous_dq[i][k]
            assert denominator > 0
            duration_floor = max(duration_floor, 3. * abs(delta) / denominator)
steps = max(240, math.ceil(duration_floor / dt))
duration = steps * dt
coefficients = []
maximum_continuous_rate = 0.
for i in range(25):
    row = []
    for k in range(3):
        q0, v0 = previous[i][k], previous_dq[i][k]
        delta = target[i][k] - q0
        c = [q0, duration * v0, 3. * delta - 2. * duration * v0, -2. * delta + duration * v0]
        times = [0., 1.]
        if c[3]:
            times.append(-c[2] / (3. * c[3]))
        for u in times:
            if 0. <= u <= 1.:
                maximum_continuous_rate = max(maximum_continuous_rate,
                    abs(((3. * c[3] * u + 2. * c[2]) * u + c[1]) / duration))
        row.append(c)
    coefficients.append(row)

layout = [(i, 'xyz'.index(a['coordinate']), segment['id'], a)
    for i, segment in enumerate(model['segments'])
    for a in (segment.get('jointProfile') or {}).get('axes', [])]
modeled = {(i, k) for i, k, _, _ in layout}
commands = [{'nativeSubstep': entry['native_substep'] - 1,
    'historicalOnly': True, 'positions': previous, 'velocities': previous_dq}]


def append_command(q):
    old = commands[-1]['positions']
    dq = [[div(sub(q[i][k], old[i][k]), dt) for k in range(3)] for i in range(25)]
    for i in range(25):
        for k in range(3):
            assert math.isfinite(q[i][k]) and math.isfinite(dq[i][k])
            assert abs(sub(q[i][k], old[i][k])) <= dt and abs(dq[i][k]) <= 1.
            if (i, k) not in modeled:
                assert q[i][k] == 0. and dq[i][k] == 0.
    for i, k, _, axis in layout:
        assert f(axis['minRadians']) <= q[i][k] <= f(axis['maxRadians'])
    commands.append({'nativeSubstep': entry['native_substep'] + len(commands) - 1,
        'historicalOnly': False, 'positions': q, 'velocities': dq})


for index in range(1, steps + 1):
    u = index / steps
    q = target if index == steps else [[f(((c[3] * u + c[2]) * u + c[1]) * u + c[0])
        for c in row] for row in coefficients]
    append_command(q)
for source_step in range(433, 1921):
    append_command(source_q(source_step))

start = entry['native_substep']
standing_command = start + steps + 1680 - 432 - 1
end = start + len(commands) - 1
assert steps == 240 and end == 1764 and end <= 6000
assert all(v == 0. for command in commands if command['nativeSubstep'] > standing_command
    for row in command['velocities'] for v in row)
per_axis = []
for index, (i, k, name, _) in enumerate(layout):
    samples = commands[1:]
    per_axis.append({'axis': index, 'segment': i, 'id': name, 'coordinate': 'xyz'[k],
        'entryQ': previous[i][k], 'entryDq': previous_dq[i][k],
        'targetQ': target[i][k], 'minimumEmittedQ': min(c['positions'][i][k] for c in samples),
        'maximumEmittedQ': max(c['positions'][i][k] for c in samples),
        'maximumAbsoluteEmittedRate': max(abs(c['velocities'][i][k]) for c in samples)})
max_rate = lambda values: max(abs(v) for c in values for row in c['velocities'] for v in row)
records = [{'path': str(p.relative_to(ROOT)), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
    for p in (ENTRY, MODEL, PROGRAM, HELPER, LOG, Path(__file__))]
document = {
    'schema': 1,
    'scope': 'Independent offline arithmetic reconstruction, not a Rust runtime export or physical admission',
    'rustTests': {'passed': 3, 'failed': 0, 'filteredOut': 237, 'elapsedTestSeconds': 0.21,
        'logPath': str(LOG.relative_to(ROOT))},
    'nativeIntegrations': 0, 'fkEvaluations': 0,
    'readinessReceipt': {'kind': 'synthetic interface test only', 'native_substep': start,
        'first_stable_substep': 0, 'actualA36HeelReadinessClaimed': False},
    'sourceHashes': records,
    'entry': {'native_substep': start, 'last_installed_command_substep': start - 1,
        'previous_q': previous, 'previous_dq': previous_dq,
        'maximumAbsoluteDq': max(abs(v) for row in previous_dq for v in row)},
    'bridge': {'durationSubsteps': steps, 'nominalDurationSeconds': steps / 240,
        'durationUsingNativeF32DtSeconds': duration,
        'displacementTimeLowerBoundSeconds': displacement,
        'bernsteinCertifiedDurationFloorSeconds': duration_floor,
        'maximumContinuousRateComputedF64': maximum_continuous_rate,
        'maximumAbsoluteEmittedRate': max_rate(commands[1:steps + 1]),
        'polynomialCoefficientsBySegmentAndCoordinate': coefficients,
        'continuousExtremaQualification': 'f64 evaluation, not interval or rational certification'},
    'timing': {'start_native_substep': start, 'bridge_substeps': steps,
        'source_cut_substep': 432, 'bridge_last_command_native_substep': start + steps - 1,
        'first_tail_command_native_substep': start + steps, 'first_tail_source_substep': 433,
        'source_standing_reference': 1680, 'standing_command_substep': standing_command,
        'standing_observation_clock': standing_command + 1,
        'source_hold_completed': 1920, 'end_native_substep_exclusive': end,
        'absolute_origin_substep': 0, 'absolute_deadline_native_substep': 6000,
        'tailSubsteps': 1920 - 432, 'completeReferenceSubsteps': end - start,
        'completeReferenceNominalSeconds': (end - start) / 240,
        'absoluteEndNominalSeconds': end / 240, 'extraStandingHoldSubsteps': 240,
        'latestAdmissibleEntrySubstepForTheseCommands': 6000 - (end - start)},
    'maximumAbsoluteEmittedRateFullReference': max_rate(commands[1:]),
    'validation': {'finiteEmittedCommands': True, 'anatomicalCommands': True,
        'slewAtMostOneRadPerSecond': True, 'exactF32FiniteDifferenceDq': True,
        'historicalEntryNotCountedAsNewCommand': True, 'extraStandingHoldCommandsZeroDq': True,
        'physicalRecoveryQualified': False},
    'perAxis': per_axis,
    'commandsIncludingHistoricalEntry': commands,
}
destination = OUT / 'observed-raccord-export.json'
with destination.open('x') as stream:
    json.dump(document, stream, separators=(',', ':'), allow_nan=False)
    stream.write('\n')
print(json.dumps({key: document[key] for key in ('timing', 'maximumAbsoluteEmittedRateFullReference')}))
print(json.dumps({'bridge': {key: value for key, value in document['bridge'].items()
    if key != 'polynomialCoefficientsBySegmentAndCoordinate'}, 'exportBytes': destination.stat().st_size,
    'exportSha256': hashlib.sha256(destination.read_bytes()).hexdigest()}))
