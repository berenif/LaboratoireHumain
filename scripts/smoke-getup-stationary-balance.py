#!/usr/bin/env python3
"""One bounded native experiment/protocol smoke, never an RL training run."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--binary', type=Path, required=True)
p.add_argument('--checkpoint', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--physics-budget', type=float, default=0)
p.add_argument('--steps', type=int, default=0)
p.add_argument('--aggregate-remaining', type=float, default=46.914406751)
a = p.parse_args()
assert 0 <= a.steps <= 40 and 0 <= a.physics_budget <= min(5,a.aggregate_remaining)
assert a.steps == 0 or a.physics_budget > 0
out = a.output
out.mkdir(parents=True, exist_ok=True)
(out/'started-once').open('x').close()
proc = subprocess.Popen([str(a.binary.resolve()),str(a.checkpoint.resolve()),str(a.physics_budget)],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=(out/'stderr.log').open('w'), text=True)
trace = (out/'protocol.ndjson').open('w')

def send(command, ok=True):
    wire = command if isinstance(command,str) else json.dumps(command)
    proc.stdin.write(wire+'\n'); proc.stdin.flush()
    raw = proc.stdout.readline()
    assert raw, 'native process exited without response'
    result = json.loads(raw)
    trace.write(json.dumps({'request':command,'response':result})+'\n'); trace.flush()
    assert result['ok'] == ok, result
    return result

initial = send({'op':'observe'})
snapshot = send({'op':'snapshot'})
for bad in ['{', {'op':'unknown'}, {'op':'step','action':[0]},
            {'op':'step','action':[2]*21}, {'op':'step','action':[0]*21,'torso_force_n':[81,0]}, {'op':'observe','bogus':True}]:
    send(bad,False)
action = send({'op':'initializedAction'})
assert snapshot == send({'op':'snapshot'})
cp = str((out/'checkpoint-before.bin').resolve())
assert send({'op':'checkpoint','path':cp})['result']['roundtripPassed']
assert send({'op':'load','path':cp})['result'] == initial['result']
assert send({'op':'reset'})['result'] == initial['result']
if a.steps == 0:
    blocked = send({'op':'step','action':[0]*21})
    assert blocked['result']['truncated'] and blocked['processNativeSubsteps'] == 0
    assert snapshot == send({'op':'snapshot'})

transitions = []
checkpoint_next_transition_equal = None
regular_steps = a.steps - 2 if a.steps >= 15 else a.steps
for i in range(regular_steps):
    # Existing expanded SLOW initialization, not a trained balance policy.
    action = send({'op':'initializedAction'})['result']['action']
    result = send({'op':'step','action':action,'torso_force_n':[10,0] if i == 10 else [0,0]})
    transitions.append(result)
    send({'op':'snapshot'})
    if i == 12 and a.steps >= 15:
        branch_path = str((out/'checkpoint-mid.bin').resolve())
        assert send({'op':'checkpoint','path':branch_path})['result']['roundtripPassed']
        same_action = send({'op':'initializedAction'})['result']['action']
        uninterrupted = send({'op':'step','action':same_action})
        uninterrupted_snapshot = send({'op':'snapshot'})['result']
        used_before_load = uninterrupted['processPhysicsWallS']
        count_before_load = uninterrupted['processNativeSubsteps']
        loaded = send({'op':'load','path':branch_path})
        assert loaded['processPhysicsWallS'] == used_before_load
        assert loaded['processNativeSubsteps'] == count_before_load
        restored = send({'op':'step','action':same_action})
        restored_snapshot = send({'op':'snapshot'})['result']
        # Wall time is a process budget, never part of physical state equality.
        left = {k:v for k,v in uninterrupted['result'].items() if k != 'budget_physics_wall_s'}
        right = {k:v for k,v in restored['result'].items() if k != 'budget_physics_wall_s'}
        checkpoint_next_transition_equal = left == right and uninterrupted_snapshot == restored_snapshot
        assert checkpoint_next_transition_equal, 'checkpoint next transition differs'
        transitions.extend([uninterrupted,restored])
    if result['result']['terminated'] or result['result']['truncated']:
        break
final = send({'op':'observe'})
last_cp = str((out/'checkpoint-final.bin').resolve())
assert send({'op':'checkpoint','path':last_cp})['result']['roundtripPassed']
assert send({'op':'load','path':last_cp})['result'] == final['result']
reset = send({'op':'reset'})
assert reset['result'] == initial['result']
assert reset['processNativeSubsteps'] == final['processNativeSubsteps']
assert reset['processPhysicsWallS'] == final['processPhysicsWallS']
proc.stdin.close()
assert proc.wait(timeout=30) == 0
before = initial['result']; after = final['result']
assert before['retained_teacher_forecast_substeps'] == after['retained_teacher_forecast_substeps']
assert len(after['observation']) == 120 and after['observation'][-1] == 1
assert after['physics_integrations'] <= final['processNativeSubsteps']
summary = {
    'scope':'native experiment plumbing smoke; not training or naturalness validation',
    'inputCheckpointSha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
    'binarySha256':hashlib.sha256(a.binary.read_bytes()).hexdigest(),
    'requestedDecisionSteps':a.steps,'completedDecisionSteps':len(transitions),
    'nativeIntegrations':final['processNativeSubsteps'],
    'simulatedSeconds':final['processNativeSubsteps']*0.00125,'episodeElapsedSeconds':after['elapsed_s'],'physicsWallSeconds':final['processPhysicsWallS'],
    'physicsWallBudgetSeconds':a.physics_budget,
    'aggregateAllowanceBeforeRunSeconds':a.aggregate_remaining,
    'remainingAggregatePhysicsAllowanceSeconds':a.aggregate_remaining-final['processPhysicsWallS'],
    'invalidProtocolRequestsRejected':6,'readOnlyOperationsPreserveSnapshot':True,
    'checkpointRoundtripExact':True,'checkpointNextTransitionExact':checkpoint_next_transition_equal,'resetPreservesProcessBudget':True,
    'teacherForecastUnchanged':True,'stationaryPolicyTrained':False,
    'finalMetrics':after,'allTransitionRewardsFinite':all(
        math.isfinite(r['result']['reward']['total']) for r in transitions),
}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
