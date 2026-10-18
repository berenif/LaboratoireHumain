#!/usr/bin/env python3
"""Explicit hip-ROM-only paired experiment. Original assets remain local.

Each model receives the same measured q/qvel with fresh native contacts and
solver state. No original cache is imported into an adapted model. Existing
multi-step controller, target timing, motor caps and friction are unmodified.
"""
import argparse
import ctypes as C
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from source_step_probe import ROOT, World, sha
from plan_source_stance import plan

HIP_RANGES_DEG = {"x": (-25., 5.), "z": (-30., 25.), "y": (-110., 20.)}
EXPANSION_DEG = 10.


def adapt_xml(text, expansion=EXPANSION_DEG):
    """Change six explicit range attributes only; reject an unexpected model."""
    assert expansion in (0., EXPANSION_DEG)
    original = ET.fromstring(text)
    adapted = ET.fromstring(text)
    receipt = []
    for side in ("right", "left"):
        for axis, before in HIP_RANGES_DEG.items():
            name = f"{side}_hip_{axis}"
            joint = adapted.find(f".//joint[@name='{name}']")
            assert joint is not None and tuple(map(float, joint.attrib['range'].split())) == before
            after = (before[0] - expansion, before[1] + expansion)
            old_tag = next(line for line in text.splitlines() if f'name="{name}"' in line and '<joint ' in line)
            old_range = joint.attrib['range']
            new_tag = old_tag.replace(f'range="{old_range}"', f'range="{after[0]:g} {after[1]:g}"')
            text = text.replace(old_tag, new_tag, 1)
            joint.set('range', f'{after[0]:g} {after[1]:g}')
            receipt.append(dict(joint=name, sourceAxis=joint.attrib['axis'], beforeDeg=list(before), afterDeg=list(after)))
    # Confirm no other XML property was changed, including all motor settings.
    checked = ET.fromstring(text)
    for item in receipt:
        checked.find(f".//joint[@name='{item['joint']}']").set('range', original.find(f".//joint[@name='{item['joint']}']").attrib['range'])
    assert ET.tostring(checked) == ET.tostring(original)
    return text, receipt


def prepare_session(base, helper, entry, output, expansion):
    """Isolated model package with its OWN freshly rebuilt, model-bound cache."""
    output.mkdir(parents=True, exist_ok=False)
    model = output / 'source-model'
    model.mkdir()
    for source in (base / 'source-model').iterdir():
        if source.is_file() and source.suffix == '.xml':
            shutil.copyfile(source, model / source.name)
    source = base / 'source-model/humanoid_high_freq.xml'
    text, ranges = adapt_xml(source.read_text(), expansion)
    # Preserve the byte-identical original model in the reference arm.
    if expansion:
        (model / source.name).write_text(text)
    for name in ('source_bridge_v2.so', 'dependency'):
        (output / name).symlink_to((base / name).resolve(), target_is_directory=(base / name).is_dir())
    (output / 'slow-continuation').mkdir()
    w = World(output, helper)
    try:
        w.install_cold(entry)
        assert w.l.lh_dim(w.h, 6) == 0
        np.testing.assert_array_equal(w.arr(0, (28,)), entry['qpos'])
        np.testing.assert_array_equal(w.arr(1, (27,)), entry['qvel'])
        w.checkpoint(output / 'slow-continuation/final.bin')
    finally:
        w.close()
    receipt = dict(experimental=bool(expansion), sourceModelSHA256=sha(source), modelSHA256=sha(model/source.name), ranges=ranges,
                   modelChanges='six hip joint ranges only' if expansion else 'none', originalWarmCacheImported=False,
                   cacheRebuiltFromIdenticalColdQposQvel=True, physicsIntegrations=0,
                   anatomicalRealismClaim=False, motorPowerChanged=False, defaultPromoted=False)
    (output / 'adaptation-receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


def native_bytes(w):
    size = w.l.lh_checkpoint_size(w.h)
    out = C.create_string_buffer(size)
    assert w.l.lh_checkpoint_export(w.h, out, size) == size
    return out.raw


def preflight(reference, wider, helper, entry):
    worlds = [World(base, helper) for base in (reference, wider)]
    try:
        for w in worlds:
            w.restore(w.base/'slow-continuation/final.bin')
        a, b = worlds
        before = [native_bytes(w) for w in worlds]
        for kind, shape in [(0, (28,)), (1, (27,)), (3, (17, 3)), (5, (17, 3, 3)), (10, (17,)), (14, (21, 6))]:
            np.testing.assert_array_equal(a.arr(kind, shape), b.arr(kind, shape))
        np.testing.assert_array_equal(a.read('lh_mass_matrix', (27,27)), b.read('lh_mass_matrix', (27,27)))
        assert a.friction_parameters() == b.friction_parameters()
        changed = []
        for i in range(21):
            name = a.l.lh_joint_name(a.h, i+1).decode()
            if '_hip_' in name:
                np.testing.assert_allclose(b.limits[i]-a.limits[i], np.deg2rad([-10,10]), atol=1e-14)
                changed.append(name)
            else:
                np.testing.assert_array_equal(a.limits[i], b.limits[i])
        assert len(changed) == 6
        # Model-bound cache rejection is tested before touching native state.
        rejected = False
        try:
            b.restore(reference/'slow-continuation/final.bin')
        except AssertionError:
            rejected = True
        assert rejected and native_bytes(b) == before[1]
        references = []
        for w in worlds:
            r = plan(w, 1, True, .9)
            assert r['staticMotorContactFeasible'] and r['minimumRequiredMotorFraction'] < .925
            assert r['minimumSelfSeparationM'] > .01
            q = np.array(r['qReference'])
            assert np.all(q >= w.limits[:,0]+.02-1e-10) and np.all(q <= w.limits[:,1]-.02+1e-10)
            references.append({k:r[k] for k in ('minimumRequiredMotorFraction','minimumSelfSeparationM','minimumJointReserveRad')})
        assert all(native_bytes(w) == blob and w.l.lh_dim(w.h,6) == 0 for w,blob in zip(worlds,before))
        return dict(passed=True, physicsIntegrations=0, identicalInitialQposQvel=True, identicalCompiledGeometryMassGearsFriction=True,
                    changedCompiledLimits=changed, otherJointLimitsUnchanged=True, foreignModelCacheRejected=True,
                    plannerUsesOwnModelAndLimits=True, plannerAndGeometryQueriesPreserveFullNativeState=True,
                    staticReferences=references, finalQuietAndWalkingPredicatesChanged=False)
    finally:
        for w in worlds:
            w.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--helper', type=Path, required=True)
    parser.add_argument('--reference-folder', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    previous = json.loads((args.reference_folder/'summary.json').read_text())
    assert sha(args.base/'source-model/humanoid_high_freq.xml') == previous['modelSHA256'], 'source model differs from measured reference'
    assert sha(args.helper) == previous['helperSHA256'], 'helper differs from measured reference'
    entry = previous['entry']
    inputs = {str(path.relative_to(ROOT)): sha(path) for path in [Path(__file__).resolve(), ROOT/'scripts/getup-step/source_multistep_probe.py', ROOT/'scripts/getup-step/source_step_probe.py', ROOT/'scripts/getup-step/source_step_preview.py', ROOT/'scripts/getup-step/plan_source_stance.py', ROOT/'scripts/getup-step-reference.py', ROOT/'rust/crates/model/data/getup-v1.json']}
    (args.output/'implementation-hashes.json').write_text(json.dumps(dict(files=inputs, helperSHA256=sha(args.helper)),indent=2)+'\n')
    receipts = {}
    for name, expansion in [('reference',0.), ('wider',EXPANSION_DEG)]:
        receipts[name] = prepare_session(args.base, args.helper, entry, args.output/f'{name}-session', expansion)
    checked = preflight(args.output/'reference-session', args.output/'wider-session', args.helper, entry)
    (args.output/'preflight.json').write_text(json.dumps(checked, indent=2)+'\n')
    print(json.dumps(checked), flush=True)
    if not args.execute:
        return
    for name in ('reference','wider'):
        command = [sys.executable, str(ROOT/'scripts/getup-step/source_multistep_probe.py'), '--base',str(args.output/f'{name}-session'),
                   '--helper',str(args.helper), '--output',str(args.output/name), '--planned-support','--time-scale','2','--distances','.03','.06','.09','.045']
        print('RUN', name, flush=True)
        with (args.output/f'{name}.log').open('x') as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=330, check=False)
        print(name, 'process return code', completed.returncode, flush=True)
        assert completed.returncode == 0, f'{name} process failed; evidence is retained'

if __name__ == '__main__':
    main()
