"""Checked Python/native task sidecars. Restoring never resets budget counters."""
import hashlib,json,math
from pathlib import Path
from reference import PhaseClock
from native_env import atomic_json,sha256

ROOT=Path(__file__).resolve().parents[2]
IMPLEMENTATION=sorted(p for p in Path(__file__).parent.glob('*.py')if not p.name.startswith('test_'))+[ROOT/'scripts/getup-step-reference.py',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/student.bin',ROOT/'rust/crates/sim/data/getup-policy/teacher.bin']

PHASES=['prepare','lift','advance','lower','acquire','settle','hold']
COUNTERS=['unloadRun','clearanceRun','touchdownRun','loadRun','finalQuietRun']


def validate_controller(state,*,reference_sha256,config_sha256):
    # Roundtrip validation also rejects arrays containing NaN/Infinity.
    json.dumps(state,allow_nan=False)
    if state.get('schema')!=1 or state.get('phase')not in PHASES:raise ValueError('invalid hybrid schema/phase')
    if state.get('referenceSHA256')!=reference_sha256 or state.get('configSHA256')!=config_sha256:raise ValueError('hybrid reference/config hash mismatch')
    for name in ['nativeId','phaseWallStartNativeId',*COUNTERS]:
        if type(state.get(name))is not int or state[name]<0:raise ValueError('invalid hybrid native counter '+name)
    if state['phaseWallStartNativeId']>state['nativeId']:raise ValueError('phase starts after physical state')
    clock=PhaseClock(**state['referenceClock'])
    if clock.last_native_substep!=state['nativeId']:raise ValueError('clock/native ID mismatch')
    for name,width in [('entryHeading',3),('entryFootOrigin',3),('lastAction',21)]:
        values=state.get(name)
        if not isinstance(values,list)or len(values)!=width or not all(type(v)in(int,float)and math.isfinite(v)for v in values):raise ValueError('invalid hybrid vector '+name)
    if abs(sum(x*x for x in state['entryHeading'])-1)>1e-8 or abs(state['entryHeading'][1])>1e-8:raise ValueError('heading must be fixed horizontal unit vector')
    if max(abs(x)for x in state['lastAction'])>1:raise ValueError('invalid last normalized action')
    order=['unloaded','clearance','forward','touchdown','loadAcquired','quietComplete']
    events=state.get('awardedEvents')
    if not isinstance(events,list)or events!=order[:len(events)]:raise ValueError('invalid event prefix')
    for name in ['physicalSlipAnchors','postLandingAnalysisAnchors','RNG']:
        if name not in state:raise ValueError('missing hybrid '+name)
    # Anchors may be null before measured landing, never silently regenerated.
    return state


def save_task_checkpoint(native,path,controller,*,reference_path,config_path):
    path=Path(path);reference_hash=sha256(reference_path);config_hash=sha256(config_path)
    validate_controller(controller,reference_sha256=reference_hash,config_sha256=config_hash)
    diagnostic=native.command(dict(op='diagnostic'))
    if diagnostic['nativeSubsteps']!=controller['nativeId']:raise ValueError('controller counter does not match current physical state')
    observation=native.command(dict(op='observe'));snapshot=native.command(dict(op='snapshot'))
    result=native.command(dict(op='checkpoint',path=str(path.resolve())))
    sidecar=dict(schema=1,nativeSHA256=sha256(path),referenceSHA256=reference_hash,configSHA256=config_hash,
                 controller=controller,physicalNativeId=diagnostic['nativeSubsteps'],observation=observation,snapshot=snapshot,
                 ledger=str(native.ledger.path.resolve()),processNativeSubstepsAtSave=native.substeps,implementationSHA256={str(p.relative_to(ROOT)):sha256(p)for p in IMPLEMENTATION},
                 nativeBinarySHA256=native.record['binarySHA256']if hasattr(native,'record')else None,
                 budgetRule='load preserves current process/global accounting; stored counters never replace ledger')
    atomic_json(path.with_suffix('.task.json'),sidecar)
    return result


def restore_task_checkpoint(native,path,*,reference_path,config_path):
    path=Path(path);sidecar=json.loads(path.with_suffix('.task.json').read_text())
    if sidecar.get('schema')!=1 or sha256(path)!=sidecar['nativeSHA256']:raise ValueError('native checkpoint hash mismatch')
    reference_hash=sha256(reference_path);config_hash=sha256(config_path)
    if sidecar['ledger']!=str(native.ledger.path.resolve()):raise ValueError('task restore cannot switch/refund pilot ledger')
    if sidecar.get('nativeBinarySHA256')is not None and native.record['binarySHA256']!=sidecar['nativeBinarySHA256']:raise ValueError('native binary changed across restore')
    if set(sidecar['implementationSHA256'])!={str(p.relative_to(ROOT))for p in IMPLEMENTATION}:raise ValueError('hybrid implementation inventory changed')
    for file,digest in sidecar['implementationSHA256'].items():
        if sha256(ROOT/file)!=digest:raise ValueError('hybrid implementation changed across restore')
    if sidecar['referenceSHA256']!=reference_hash or sidecar['configSHA256']!=config_hash:raise ValueError('task checkpoint inputs changed')
    state=validate_controller(sidecar['controller'],reference_sha256=reference_hash,config_sha256=config_hash)
    native.command(dict(op='load',path=str(path.resolve())))
    diagnostic=native.command(dict(op='diagnostic'))
    if diagnostic['nativeSubsteps']!=state['nativeId']or diagnostic['nativeSubsteps']!=sidecar['physicalNativeId']:raise ValueError('restored physical/controller native IDs differ')
    if native.command(dict(op='observe'))!=sidecar['observation']or native.command(dict(op='snapshot'))!=sidecar['snapshot']:raise ValueError('restored task state differs')
    return state
