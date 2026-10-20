"""Fail-closed contracts for the separately authorized corrected learning loop.

No runtime, model, deadline, or training is constructed by this module.
"""
from __future__ import annotations
import copy
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import random

CATEGORIES = ('back_pre_holding', 'back_holding', 'baseline_preservation')
ROLES = ('learning_audit', 'provenance_audit', 'replay', 'replay_metadata',
         'original_seed', 'nearby_seed', 'belly_seed', 'worker_wasm', 'worker_glue',
         'baseline_wasm', 'baseline_glue', 'profile', 'baseline_receipt', 'transitions')
PINNED_RETAINED = {
    'learning_audit': 'a7bf3bf79b616c9b8fc5775887ba94241bbb933fcf7ffbed290e3523833591ef',
    'provenance_audit': 'a2d6bc0dbe2e3b24732e532296d76c42ae95722b3f5dd64eba45b908e22d0690',
    'replay': '7a4218e4efcd16f3b5d7b7266a214bc09c4d0a679341b952b47ca76fcc6b38f9',
    'transitions': 'b9b44bf1e2a9d44df08e954c749123061baf521f10ae3640789d54560b15f305',
    'original_seed': 'c49445348d9e96543f283c8d9a318241c24ffbb0a03f938e2490bd2167ec6ed6',
    'nearby_seed': '579ef7a1fa2f93ebb7f3ffd14229e246c73f02c3829d93afac9817104bfae0b6',
    'worker_wasm': '921a373a0964db6ecaa86f3c1f10a91124f6f672b3dc123f0a0232721424cf56',
    'baseline_wasm': '53395c6d8b968774e07ee38eeb1931047acefb0df3cd090a59bc1943b9867e28',
    'baseline_receipt': 'c4a0da18bfe1f0e5c400b3fc31eaa39ba3425cf0b10d94f35e1cecb219328a53',
}
CAPS = {'wall_seconds': 3600, 'fresh_decisions': 4096, 'joint_attempts': 1024,
        'native_integrations': 300000, 'warmup_updates': 256, 'cycles': 16,
        'block_decisions': 256, 'block_attempts': 64, 'batch_size': 128}

# Raw inputs and the actual audit schemas were lost with the previous workspace.
# A synthetic fixture must never stand in for their reviewed real-data adapter.
EXECUTION_BLOCKER = ('Source-only implementation: retained raw inputs and their '
    'actual audit/metadata adapter must be recovered and independently verified '
    'before enabling an experiment. No replacement data is authorized here.')

def require_execution_ready():
    raise ContractError(EXECUTION_BLOCKER)

class ContractError(RuntimeError):
    pass

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def remaining_seconds(deadline, now, reserve=0.):
    remaining = deadline - now - reserve
    if not math.isfinite(remaining) or remaining <= 0:
        raise ContractError('Original absolute deadline or protected evaluation wall reached')
    return remaining

def preflight(manifest_path):
    """Verify every required exact retained input before importing training code."""
    path = Path(manifest_path)
    if not path.is_file():
        raise ContractError(f'Missing retained-input manifest: {path}')
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != 1 or manifest.get('synthetic') is not False:
        raise ContractError('Real, non-synthetic retained input schema1 is required')
    if manifest.get('caps') != CAPS:
        raise ContractError('The approved caps must match exactly')
    files = manifest.get('files', {})
    resolved = {}
    for role in ROLES:
        value = files.get(role, {})
        source = Path(value.get('path', '__missing__'))
        if not source.is_absolute():
            source = path.parent / source
        expected = value.get('sha256', '')
        if len(expected) != 64 or any(c not in '0123456789abcdef' for c in expected):
            raise ContractError(f'Missing exact SHA256 for retained input {role}')
        if role in PINNED_RETAINED and expected != PINNED_RETAINED[role]:
            raise ContractError(f'Retained input hash mismatch against closed-pilot receipt: {role}')
        if not source.is_file():
            raise ContractError(f'Missing retained input {role}: {source}')
        if digest(source) != expected:
            raise ContractError(f'Retained input hash mismatch: {role}')
        resolved[role] = str(source.resolve())
    for field in ('behavior_policies', 'episodes', 'baseline_identity'):
        if not manifest.get(field):
            raise ContractError(f'Missing retained provenance: {field}')
    throughput = manifest.get('measured_native_per_wall_second', 0)
    reserve = manifest.get('evaluation_reserve_integrations', 0)
    if not math.isfinite(throughput) or throughput <= 0:
        raise ContractError('Verified measured throughput is required')
    if not isinstance(reserve, int) or not 147000 <= reserve < CAPS['native_integrations']:
        raise ContractError('Reserve all three25s+30s holds and forecasts first')
    return manifest, resolved

@dataclass(frozen=True)
class Record:
    episode: str
    category: str
    phase: str
    behavior: str
    features: tuple
    correction: tuple
    next_features: tuple
    reward: float
    discount: float
    terminal: bool
    truncated: bool
    pre_native: int
    post_native: int
    impact_native: int
    complete_episode: bool
    source: str
    episode_outcome: str
    baseline_success: bool

    def validate(self):
        if self.category not in CATEGORIES or self.phase not in ('pre_holding', 'holding'):
            raise ContractError('Unknown replay category/phase')
        if not self.episode or not self.behavior or self.source not in ('retained_verified', 'fresh_wasm', 'synthetic_rehearsal'):
            raise ContractError('Replay requires episode, behavior and real provenance')
        if self.episode_outcome not in ('pending', 'success', 'failure', 'truncation'):
            raise ContractError('Unknown episode outcome')
        if self.complete_episode != (self.episode_outcome in ('success','failure')):
            raise ContractError('Episode completeness/outcome mismatch')
        if self.source == 'retained_verified' and self.episode_outcome == 'pending':
            raise ContractError('Retained episode outcome is unverified')
        if self.category == 'baseline_preservation' and not (self.baseline_success and self.complete_episode and self.episode_outcome == 'success'):
            raise ContractError('Preservation requires verified complete baseline success')
        if self.category == 'back_pre_holding' and self.phase != 'pre_holding':
            raise ContractError('Pre-holding replay phase mismatch')
        if self.category == 'back_holding' and self.phase != 'holding':
            raise ContractError('Holding replay phase mismatch')
        if len(self.features) != 1024 or len(self.next_features) != 1024 or len(self.correction) != 21:
            raise ContractError('Frozen-feature/correction dimensions differ')
        if not all(math.isfinite(x) for x in self.features + self.next_features + self.correction):
            raise ContractError('Nonfinite replay')
        if any(abs(x) > 1 for x in self.correction) or not math.isfinite(self.reward):
            raise ContractError('Invalid correction/reward')
        if self.terminal and self.truncated or not 0 <= self.discount <= 1:
            raise ContractError('Terminal and administrative truncation are distinct')
        if any(type(value) is not int for value in (self.pre_native,self.post_native,self.impact_native)) or self.impact_native<0:
            raise ContractError('Integer native chronology is required')
        if self.post_native <= self.pre_native or self.pre_native < self.impact_native or (self.post_native-self.pre_native)%20:
            raise ContractError('Invalid physical episode chronology')
        if self.terminal and self.discount != 0:
            raise ContractError('Real terminal transitions must not bootstrap')
        return self

def diagnostic_membership(rows,mapping):
    """Case-bound diagnostic inventory, never infer BACK from provisional rows."""
    if len(rows)!=len(mapping):raise ContractError('Diagnostic mapping membership incomplete')
    by_case={};coverage={category:0 for category in CATEGORIES}
    for index,(row,entry) in enumerate(zip(rows,mapping)):
        case=entry['case']
        if entry['replay_index']!=index or entry['episode_id']!=row.episode or case not in ('original','heldout','belly'):
            raise ContractError('Diagnostic case/episode/index differs')
        key=f'{case}:{row.episode_outcome}:{row.phase}'
        by_case[key]=by_case.get(key,0)+1
        if case=='original' and row.category in CATEGORIES[:2]:coverage[row.category]+=1
        if case!='original' and row.category=='baseline_preservation' and row.complete_episode and row.baseline_success:
            coverage['baseline_preservation']+=1
    return {'byCaseOutcomePhase':by_case,'approvedStratumRows':coverage,
        'missingStrata':[category for category,n in coverage.items() if n==0]}


class EpisodeReplay:
    """Keep all records. No eviction or category success-rate interpretation."""
    def __init__(self, retained, fresh_allowance=4096, seed=20261020, *, diagnostic_only=False):
        self.diagnostic_only=diagnostic_only
        self.rows = []
        self.groups = {category: {} for category in CATEGORIES}
        self.capacity = len(retained) + fresh_allowance
        self.rng = random.Random(seed)
        for row in retained:
            self.add(row)
        if not diagnostic_only and any(not self.groups[c] for c in CATEGORIES):
            raise ContractError('Every approved replay category needs verified episodes')

    def add(self, row):
        row.validate()
        if len(self.rows) >= self.capacity:
            raise ContractError('Replay capacity exhausted; no silent eviction')
        index = len(self.rows)
        self.rows.append(row)
        self.groups[row.category].setdefault(row.episode, []).append(index)
        return index

    def finish_episode(self, episode, outcome, truncated=False):
        """Finalize every row, including earlier blocks of an ongoing episode."""
        if outcome not in ('success', 'failure', 'truncation') or truncated != (outcome == 'truncation'):
            raise ContractError('Invalid episode completion receipt')
        indices = [i for i, row in enumerate(self.rows) if row.episode == episode]
        if not indices or any(self.rows[i].source not in ('fresh_wasm','synthetic_rehearsal') for i in indices):
            raise ContractError('Only the current fresh episode can be finalized')
        last=self.rows[indices[-1]]
        if truncated and any(self.rows[i].terminal or self.rows[i].complete_episode for i in indices):
            raise ContractError('A completed episode cannot be administratively truncated')
        if not truncated and (not last.terminal or last.episode_outcome!=outcome):
            raise ContractError('Episode completion requires its true terminal outcome')
        for index in indices:
            row = self.rows[index]
            self.rows[index] = replace(row, complete_episode=not truncated,
                episode_outcome=outcome, truncated=truncated and index == indices[-1]).validate()

    def category_indices(self, category, count):
        episodes = sorted(self.groups[category])
        if not episodes:
            raise ContractError(f'Empty replay category: {category}')
        result = []
        for _ in range(count):
            episode = episodes[self.rng.randrange(len(episodes))]
            candidates = self.groups[category][episode]
            result.append(candidates[self.rng.randrange(len(candidates))])
        return result

    def batch_indices(self, size=128):
        if self.diagnostic_only:
            raise ContractError('Diagnostic partial replay cannot supply a training batch')
        if size != 128:
            raise ContractError('Frozen batch size128 required')
        return (self.category_indices(CATEGORIES[0], 64)
                + self.category_indices(CATEGORIES[1], 32)
                + self.category_indices(CATEGORIES[2], 32))

    def diagnostic_sample_indices(self,count):
        if not self.diagnostic_only or not self.rows or type(count) is not int or count<0:
            raise ContractError('Populated diagnostic replay and explicit sample count required')
        return [self.rng.randrange(len(self.rows)) for _ in range(count)]

    def witness_indices(self, recent):
        if len(recent) != 256:
            raise ContractError('Witness requires the entire256-decision collection block')
        return (tuple(recent) + tuple(self.category_indices(CATEGORIES[0], 64))
                + tuple(self.category_indices(CATEGORIES[2], 64)))

@dataclass(frozen=True)
class Gaussian:
    mean: tuple
    log_std: tuple

    def validate(self):
        if len(self.mean) != 21 or len(self.log_std) != 21:
            raise ContractError('Gaussian action dimension must be21')
        if not all(math.isfinite(x) for x in self.mean + self.log_std):
            raise ContractError('Nonfinite Gaussian')
        return self

@dataclass(frozen=True)
class GuardResult:
    accepted: bool
    mean_summed_kl: float
    maximum_deterministic_change: float

class FixedWitness:
    """KL(collecting||candidate); the reference never advances after acceptance."""
    def __init__(self, features, reference, policy_id):
        self.features = tuple(tuple(row) for row in features)
        self.reference = tuple(row.validate() for row in reference)
        if not policy_id or len(self.features) != len(self.reference) or not self.reference:
            raise ContractError('Invalid fixed collecting-policy witness')
        if any(len(row) != 1024 or not all(math.isfinite(x) for x in row) for row in self.features):
            raise ContractError('Invalid frozen witness features')
        self.policy_id = policy_id
        self.sha256 = hashlib.sha256(repr((self.features, self.reference, policy_id)).encode()).hexdigest()

    def evaluate(self, candidate):
        if len(candidate) != len(self.reference):
            raise ContractError('Witness population changed')
        kl = 0.0
        maximum = 0.0
        try:
            for old, new in zip(self.reference, candidate):
                new.validate()
                state_kl = 0.0
                for mu0, ls0, mu1, ls1 in zip(old.mean, old.log_std, new.mean, new.log_std):
                    state_kl += ls1 - ls0 + .5 * (math.exp(2*(ls0-ls1))
                        + (mu0-mu1)**2 * math.exp(-2*ls1) - 1)
                    maximum = max(maximum, abs(math.tanh(mu1)-math.tanh(mu0)))
                kl += state_kl
            kl /= len(self.reference)
        except (OverflowError, ContractError):
            return GuardResult(False, math.inf, math.inf)
        return GuardResult(math.isfinite(kl) and kl <= .1 and maximum <= .05, kl, maximum)

class LoopPhase:
    """Caps/attempts are exposure counters, intentionally never rolled back."""
    def __init__(self):
        self.warmup = 0
        self.cycle = 0
        self.fresh = 0
        self.block_fresh = 0
        self.attempts = 0
        self.block_attempts = 0
        self.block_accepted = 0
        self.phase = 'warmup'
        self.collecting_policy = None
        self.pending_attempt = False

    def warmup_done(self):
        if self.phase != 'warmup' or self.warmup >= 256:
            raise ContractError('Critic-only warmup phase/cap violation')
        self.warmup += 1
        if self.warmup == 256:
            self.phase = 'ready_collection'

    def begin_collection(self, policy_id):
        if self.phase != 'ready_collection' or self.cycle >= 16 or not policy_id:
            raise ContractError('Collection cadence violation')
        self.phase = 'collection'
        self.collecting_policy = policy_id
        self.block_fresh = self.block_attempts = self.block_accepted = 0

    def collected(self, policy_id):
        if self.phase != 'collection' or policy_id != self.collecting_policy or self.fresh >= 4096:
            raise ContractError('Collection policy changed or decision cap exhausted')
        self.fresh += 1
        self.block_fresh += 1
        if self.block_fresh == 256:
            self.phase = 'updates'

    def attempt(self):
        if self.pending_attempt or self.phase != 'updates' or self.block_attempts >= 64 or self.attempts >= 1024:
            raise ContractError('Joint attempt cadence/cap violation')
        self.block_attempts += 1
        self.attempts += 1
        self.pending_attempt = True

    def rejected(self):
        if not self.pending_attempt:
            raise ContractError('No pending joint attempt')
        self.pending_attempt = False

    def accepted(self):
        if self.phase != 'updates' or not self.pending_attempt:
            raise ContractError('No pending joint attempt')
        self.block_accepted += 1
        self.pending_attempt = False

    def finish_block(self):
        if self.phase != 'updates' or self.pending_attempt:
            raise ContractError('Cannot end an incomplete collection block')
        if not self.block_accepted:
            self.phase = 'stopped_no_actor_acceptance'
            return
        self.cycle += 1
        self.phase = 'finished' if self.cycle == 16 else 'ready_collection'

@dataclass(frozen=True)
class PauseReceipt:
    episode: str
    native: int
    impact: int
    source_start: int
    reference_index: int
    phase: str
    checkpoint_sha256: str

    def unchanged(self, after):
        if self != after:
            raise ContractError('Physical/controller state changed during learning pause')

class UpdateGuard:
    def __init__(self, learner, replay, phase, witness):
        self.learner, self.replay, self.phase, self.witness = learner, replay, phase, witness

    def attempt(self, batch):
        self.phase.attempt()
        snapshot = copy.deepcopy(self.learner.capture_learning_state())
        replay_rng = self.replay.rng.getstate()
        try:
            chosen = batch() if callable(batch) else batch
            metrics = self.learner.joint_attempt(chosen)
            decision = self.witness.evaluate(self.learner.gaussians(self.witness.features))
            finite = all(math.isfinite(value) for value in metrics.values())
        except BaseException:
            self.learner.restore_learning_state(snapshot)
            self.replay.rng.setstate(replay_rng)
            self.phase.rejected()
            raise
        if not decision.accepted or not finite:
            self.learner.restore_learning_state(snapshot)
            self.replay.rng.setstate(replay_rng)
            self.phase.rejected()
        else:
            self.phase.accepted()
        return decision, metrics, decision.accepted and finite
