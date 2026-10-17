"""Pure adapted task reward/transition bookkeeping. No collection or optimizer.

This file intentionally does not reproduce source rewards; source_math.py does.
Full20-sample intervals and verified finite physical-terminal partials are valid.
"""
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
import math
import numpy as np

CONFIG_PATH=Path(__file__).with_name('task-config-v1.json')


def load_config(path=CONFIG_PATH,*,expected_sha256=None):
    raw=Path(path).read_bytes();digest=hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and digest!=expected_sha256:raise ValueError('task config hash mismatch')
    config=json.loads(raw)
    if config['schema']!=1:raise ValueError('unsupported task schema')
    if config['pilot']['learningEnabled'] or config['pilot']['collectionEnabled']:
        raise ValueError('this draft has no collection/learning admission')
    validate_config(config)
    terms=config['reward']['denseTerms']
    if not math.isclose(sum(t['weight']for t in terms.values()),1.,abs_tol=1e-12):raise ValueError('dense weights must sum to1')
    return config,digest


def validate_config(config):
    reward=config['reward']
    for name,term in reward['denseTerms'].items():
        if not math.isfinite(term['scale'])or term['scale']<=0 or not math.isfinite(term['weight'])or term['weight']<0:
            raise ValueError('invalid reward scale/weight')
        if type(reward['errorDimensions'][name])is not int or reward['errorDimensions'][name]<1:raise ValueError('invalid error dimension')
    if not math.isclose(sum(t['weight']for t in reward['denseTerms'].values()),1.,abs_tol=1e-12):raise ValueError('weights must sum to1')
    for name in ['normalizedControlMeanSquareCost','normalizedControlChangeMeanSquareCost','loadedSlipIncrementCostPerMeter','physicalFailurePenalty']:
        if not math.isfinite(reward[name])or reward[name]<0:raise ValueError('invalid reward cost')
    if any(not math.isfinite(x)or x<0 for x in reward['eventBonuses'].values()):raise ValueError('invalid event bonus')


def _finite_vector(value,name):
    x=np.asarray(value,dtype=float)
    if x.ndim>1 or x.size<1 or not np.isfinite(x).all():raise ValueError('invalid '+name)
    return x


@dataclass
class TaskHistory:
    """External hybrid state, intentionally not claimed present in actor118."""
    last_native_id:int
    awarded_events:list=field(default_factory=list)

    def __post_init__(self):
        if type(self.last_native_id)is not int or self.last_native_id<0:raise ValueError('invalid native ID')
        if not isinstance(self.awarded_events,list)or len(set(self.awarded_events))!=len(self.awarded_events):raise ValueError('invalid event history')

    def evaluate(self,native_ids,errors,controls,previous_controls,loaded_slip_increment_m,
                 *,confirmed_events=(),physical_failure=None,truncated=False,config=None):
        """Reward references/errors belong to PRE-action phase for this interval.

        Confirmed events are produced by the separately tested measured phase
        evaluator, not inferred from target time. No mutation on invalid input.
        """
        self.__post_init__()
        config=config or load_config()[0];validate_config(config);cfg=config['reward'];ids=list(native_ids)
        if any(type(i)is not int for i in ids)or not 1<=len(ids)<=20 or ids!=list(range(self.last_native_id+1,self.last_native_id+1+len(ids))):
            raise ValueError('task transition requires consecutive NEW native IDs')
        if len(ids)!=20 and not physical_failure:
            raise ValueError('only verified finite physical terminals may be partial')
        if set(errors)!=set(cfg['denseTerms']):raise ValueError('missing/extra task error terms')
        terms={}
        for name,term in cfg['denseTerms'].items():
            error=_finite_vector(errors[name],name).reshape(-1)
            if error.shape!=(cfg['errorDimensions'][name],):raise ValueError('wrong error dimension '+name)
            terms[name]=float(np.exp(-.5*np.mean((error/term['scale'])**2)))
        control=_finite_vector(controls,'controls');previous=_finite_vector(previous_controls,'previous controls')
        if control.shape!=(21,)or previous.shape!=(21,)or np.max(abs(control))>1 or np.max(abs(previous))>1:
            raise ValueError('normalized original controls21 required')
        if not math.isfinite(loaded_slip_increment_m)or loaded_slip_increment_m<0:raise ValueError('invalid slip increment')
        if not isinstance(truncated,bool)or physical_failure is not None and physical_failure not in config['termination']['failureCodes']:raise ValueError('invalid terminal signal/failure code')
        order=cfg['eventOrder'];awarded=self.awarded_events.copy();bonus=0.
        if awarded!=order[:len(awarded)]:raise ValueError('event history not an ordered prefix')
        for event in confirmed_events:
            if event not in order:raise ValueError('unknown task event')
            if event in awarded:continue
            if event!=order[len(awarded)]:raise ValueError('event out of physical sequence')
            awarded.append(event);bonus+=cfg['eventBonuses'][event]
        dense=sum(cfg['denseTerms'][name]['weight']*value for name,value in terms.items())
        duration_fraction=len(ids)/20.
        dense*=duration_fraction
        costs=duration_fraction*(cfg['normalizedControlMeanSquareCost']*float(np.mean(control**2))+cfg['normalizedControlChangeMeanSquareCost']*float(np.mean((control-previous)**2)))+cfg['loadedSlipIncrementCostPerMeter']*loaded_slip_increment_m
        if physical_failure:
            # Failed intervals cannot grant success events, even if a target was crossed.
            awarded=self.awarded_events.copy();bonus=0.
        reward=dense-costs+bonus-(cfg['physicalFailurePenalty']if physical_failure else 0.)
        if not all(math.isfinite(x)for x in [dense,costs,bonus,reward]):raise ValueError('nonfinite computed task reward')
        self.last_native_id=ids[-1];self.awarded_events=awarded
        success=bool(not physical_failure and awarded==order)
        return dict(reward=reward,dense=dense,terms=terms,cost=costs,eventBonus=bonus,
                    terminated=bool(physical_failure or success),truncated=bool(truncated and not physical_failure and not success),
                    not_done=0. if physical_failure or success else 1.,physicalFailure=physical_failure,
                    success=success,awardedEvents=awarded.copy(),nativeSampleCount=len(ids),durationSeconds=len(ids)/800.)
