"""Read the pinned helper's last audited solve without advancing its world.

Use synchronously after lh_step_audited on the live world. The helper's wrench
cache is process-global and owner-specific, is not checkpointed, and cannot be
recovered by importing a saved native state. Load agreement is necessary, not
an independent proof that such a cache was populated in this process.

The cached force/moment precedes integration; row geometry and the helper's
force-weighted material-point slip are transported to the current step1 stage.
No current contact frame is paired with a previous solved contact force here.
"""
import ctypes as C
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np

from source_step_probe import PTR

PHYSICS_STEP_S = .00125
MU = 1.
LOADED_THRESHOLD_N = 3.
LOAD_TOLERANCE_N = 1e-8
SIDES = ('left', 'right')
_PRESET = json.loads(Path(__file__).with_name('source-step-selected-v1.json').read_text())


@lru_cache(maxsize=32)
def _file_hash(path, size, mtime_ns):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _hash(path):
    path = Path(path).resolve(); stat = path.stat()
    return _file_hash(str(path), stat.st_size, stat.st_mtime_ns)


def _bindings(w):
    helper = _hash(w.helper)
    if helper != _PRESET['helperSHA256']:
        raise ValueError('last-solved wrench reader requires the pinned v9 helper')
    return dict(helperSHA256=helper, modelSHA256=_hash(w.xml),
                accessor='lh_last_ground_wrench', sideOrder=list(SIDES))


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('invalid '+name) from error
    if not np.isfinite(value):
        raise ValueError('nonfinite '+name)
    return value


def reader(w, row):
    """Return JSON-finite per-foot diagnostics; never apply or integrate control.

    The sole utilization value is ||sum Ft||/(mu*sum Fn). For common mu=1 and
    horizontal ground this is a lower bound on the largest individual-contact
    shear utilization. A small value cannot establish nonsaturation.
    """
    step = row.get('sourceStep')
    if type(step) is not int or step < 0:
        raise ValueError('invalid native sourceStep')
    now = _finite(row.get('sourceTimeS'), 'native sourceTimeS')
    if now < 0. or (step > 0 and now < PHYSICS_STEP_S-1e-10):
        raise ValueError('invalid native sample time')
    if not isinstance(row.get('feet'), (list, tuple)) or len(row['feet']) != 2:
        raise ValueError('two left/right foot rows required')
    before_step, before_time = w.l.lh_dim(w.h, 6), w.l.lh_time(w.h)
    if step != before_step or abs(now-before_time) > 1e-10:
        raise ValueError('row does not match current native sample')
    held = np.asarray(w.arr(2, (21,)), dtype=float).copy()
    if held.shape != (21,) or not np.isfinite(held).all():
        raise ValueError('invalid held native control')
    binding = _bindings(w)
    timestep = w.l.lh_dt
    timestep.argtypes, timestep.restype = [C.c_void_p], C.c_double
    if abs(_finite(timestep(w.h), 'native timestep')-PHYSICS_STEP_S) > 1e-15:
        raise ValueError('expected unchanged native timestep 0.00125s')
    friction = w.friction_parameters()['effectiveFootGroundContacts']
    for contact in friction:
        values = np.asarray(contact, dtype=float)
        if values.shape != (9,) or not np.isfinite(values).all():
            raise ValueError('invalid effective foot-ground friction record')
        if abs(values[4]-MU) > 1e-12 or abs(values[5]-MU) > 1e-12:
            raise ValueError('expected unchanged native foot-ground mu=1')
    function = w.l.lh_last_ground_wrench
    function.argtypes, function.restype = [C.c_void_p, PTR], None
    wrench = np.asarray(w.read('lh_last_ground_wrench', (2, 6)), dtype=float)
    if wrench.shape != (2, 6) or not np.isfinite(wrench).all():
        raise ValueError('invalid finite left/right solved wrench')
    available = step > 0
    if not available and np.any(wrench != 0.):
        raise ValueError('no prior audited integration for nonzero wrench')
    feet = []
    for index, side in enumerate(SIDES):
        foot = row['feet'][index]
        load = _finite(foot.get('loadN'), side+' load')
        slip = _finite(foot.get('loadedSlipSpeedMps'), side+' loaded slip speed')
        if load < 0. or slip < 0.:
            raise ValueError('negative load or loaded-slip speed')
        normal = float(wrench[index, 2])
        if abs(normal-load) > LOAD_TOLERANCE_N:
            raise ValueError(side+' solved wrench Fz disagrees with row load')
        loaded = bool(available and load >= LOADED_THRESHOLD_N)
        with np.errstate(over='ignore', invalid='ignore'):
            shear = _finite(np.hypot(wrench[index, 0], wrench[index, 1]), side+' net shear')
        utilization = _finite(shear/(MU*normal), side+' net utilization') if loaded else None
        feet.append(dict(side=side, sideIndex=index,
            forceWorldSourceN=wrench[index, :3].tolist(),
            momentWorldSourceNmAboutSolvedFootOrigin=wrench[index, 3:].tolist(),
            normalForceN=normal, rowLoadN=load, normalForceRowErrorN=normal-load,
            netShearForceN=shear, loadedForUtilization=loaded,
            netShearUtilizationLowerBound=utilization,
            utilizationUndefinedReason=None if loaded else ('no-prior-audited-sample' if not available else 'below-3N-load'),
            transportedLoadedSlipSpeedMps=slip,
            transportedLoadedSlipIncrementM=slip*PHYSICS_STEP_S if available else 0.))
    if (w.l.lh_dim(w.h, 6) != before_step or w.l.lh_time(w.h) != before_time
            or not np.array_equal(w.arr(2, (21,)), held)):
        raise ValueError('solved-wrench diagnostic mutated native counter/time/control')
    return dict(schema=1, sourceStep=step, sourceTimeS=now,
        phase=row.get('phase'), stepIndex=row.get('stepIndex'),
        physicsStepS=PHYSICS_STEP_S, nativeTimeStepSVerified=True, solvedWrenchAvailable=available,
        solvedStageSourceStepBeforeIntegration=step-1 if available else None,
        solvedStageSourceTimeS=now-PHYSICS_STEP_S if available else None,
        transportedGeometrySourceTimeS=now,
        forceMomentStage='cached mj_step2 solve before integration; pre-step1 geometry',
        momentDefinition='sum(contact lever cross world force) about solved-stage foot body origin; intrinsic contact couples excluded',
        slipStage='current mj_step1 material-point velocity weighted by just-solved normal forces',
        coordinateFrame='source world x-forward z-up', frictionCoefficient=MU,
        currentEffectiveFootGroundContactsMu1Verified=bool(len(friction)),
        loadedThresholdN=LOADED_THRESHOLD_N, normalLoadAgreementToleranceN=LOAD_TOLERANCE_N,
        utilizationMeaning='lower bound on maximum individual-contact shear utilization; cancellation can hide saturation',
        individualContactNonsaturationEstablished=False,
        cacheRequirement='live synchronous audited sample; not restored from checkpoint and not superseded by another world',
        heldMotorControlSHA256=hashlib.sha256(np.ascontiguousarray(held,dtype='<f8').tobytes()).hexdigest(),
        nativeCounterTimeHeldControlUnchanged=True, bindings=binding, feet=feet)


read_solved_wrench_diagnostics = reader
