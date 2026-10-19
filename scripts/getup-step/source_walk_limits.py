"""Current cumulative walking-slip stop; historical evidence keeps its own limit."""
import math

GLOBAL_LOADED_SLIP_LIMIT_M = .020
LEGACY_GLOBAL_LOADED_SLIP_LIMIT_M = .010
SUPPORTED_SAVED_SLIP_LIMITS_M = (.010, .020)


def validate_saved_slip_limit(value):
    if type(value)not in(int,float)or not math.isfinite(value)or value not in SUPPORTED_SAVED_SLIP_LIMITS_M:
        raise ValueError('unsupported saved walking-slip limit')
    return float(value)


def loaded_slip_exceeded(distances,limit=GLOBAL_LOADED_SLIP_LIMIT_M):
    """20 mm exactly is allowed; stop at the first value strictly above it."""
    return any(float(distance)>limit for distance in distances)


def loaded_slip_stop_reason(limit=GLOBAL_LOADED_SLIP_LIMIT_M):
    return f'global-loaded-slip-{limit*1000:g}mm'
