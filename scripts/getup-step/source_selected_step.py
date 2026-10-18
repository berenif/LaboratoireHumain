#!/usr/bin/env python3
"""Selected native source-step recipe: wider hips + flexed knees, original arms.

Rebuild the correct model-bound cold cache automatically. The original model,
baseline entrypoint and application/WASM defaults remain untouched.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from source_step_probe import ROOT, World, sha
from source_hip_mobility import prepare_session
from source_multistep_probe import MultiStep
from source_arm_knee_mobility import ArmKneeMultiStep

PRESET = Path(__file__).with_name('source-step-selected-v1.json')


def load_preset():
    preset = json.loads(PRESET.read_text())
    assert preset['schema'] == 1 and preset['selectedVariant'] == 'hip10-knee27_5'
    return preset


def resolve_variant(value, preset):
    variant = preset['selectedVariant'] if value == 'selected' else value
    assert variant in ('hip10-knee27_5', 'baseline'), 'unknown source-step variant'
    return variant


def state_sha(entry):
    return hashlib.sha256(np.r_[entry['qpos'], entry['qvel']].astype('<f8').tobytes()).hexdigest()


def prepare(output, requested='selected', base=None, helper=None):
    preset = load_preset()
    variant = resolve_variant(requested, preset)
    base = (base or ROOT/preset['sourceSession']).resolve()
    helper = (helper or ROOT/preset['helper']).resolve()
    assert sha(base/'source-model/humanoid_high_freq.xml') == preset['sourceModelSHA256'], 'unexpected source model'
    assert sha(helper) == preset['helperSHA256'], 'unexpected dynamics helper'
    seed = World(base, helper)
    try:
        seed.restore(base/'slow-continuation/final.bin')
        entry = seed.packet()
    finally:
        seed.close()
    assert state_sha(entry) == preset['initialQposQvelSHA256'], 'unexpected initial qpos/qvel'
    output.mkdir(parents=True, exist_ok=False)
    session = output/'model-session'
    receipt = prepare_session(base, helper, entry, session, 10. if variant == 'hip10-knee27_5' else 0.)
    expected = preset['selectedModelSHA256' if variant == 'hip10-knee27_5' else 'sourceModelSHA256']
    assert receipt['modelSHA256'] == expected
    check = World(session, helper)
    try:
        check.restore(session/'slow-continuation/final.bin')
        assert check.l.lh_dim(check.h, 6) == 0
        assert state_sha(check.packet()) == preset['initialQposQvelSHA256']
    finally:
        check.close()
    selected = dict(variant=variant, modelSHA256=expected, helperSHA256=sha(helper),
                    initialQposQvelSHA256=state_sha(entry), modelSpecificColdCache=True,
                    physicsIntegrations=0, armsExperimentEnabled=False,
                    kneeNeutralFlexionDeg=[27.5,27.5] if variant == 'hip10-knee27_5' else None)
    (output/'selection.json').write_text(json.dumps(selected, indent=2)+'\n')
    return session, helper, selected


def create_live(session, helper, output, variant):
    if variant == 'hip10-knee27_5':
        return ArmKneeMultiStep(session, helper, output, 'knees')
    assert variant == 'baseline'
    return MultiStep(session, helper, output, (.03,.06,.09,.045), True, 2.)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--variant', choices=('selected','baseline'), default='selected')
    p.add_argument('--base', type=Path, help='relocated retained original source assets')
    p.add_argument('--helper', type=Path, help='relocated retained v9 helper')
    p.add_argument('--prepare-only', action='store_true', help='create/verify model and cache; zero integrations')
    return p


def main():
    args = parser().parse_args()
    session, helper, selected = prepare(args.output, args.variant, args.base, args.helper)
    print(json.dumps(selected), flush=True)
    if args.prepare_only:
        return
    live = create_live(session, helper, args.output/'run', selected['variant'])
    live.paused = False
    started = time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started < 300., '300s process cap'
            live.advance(4)
        print(live.failure or 'measured-alternating-multistep-plus5squiet', flush=True)
    except Exception as error:
        live.failure = repr(error)
        raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__ == '__main__':
    main()
