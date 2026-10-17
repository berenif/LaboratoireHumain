#!/usr/bin/env python3
"""Durable, conservative budget for the 2026-10-12 coordination prototype.

This module NEVER starts a worker or integrates physics. Python worker API::

    budget = Budget(new_run_directory)
    budget.init_preparation(initial_inspections=0)
    prep = budget.reserve('preparation', active_seconds=3600, inspections=100)
    # Do only the reserved preparation, then:
    budget.complete(prep['id'], actual_inspections=73)
    budget.freeze(manifest)
    # frozen-verification permits only preflight native verification, charged
    # to the SAME 21120-step allocation / 300000 total / two-hour deadline.
    check = budget.reserve('verification', native_steps=240, worker_pid=os.getpid())
    budget.complete(check['id'], actual_native_steps=240, outcome=check_outcome)
    budget.admit_native_evidence(native_evidence)
    history = budget.reserve('history', history='A', worker_pid=os.getpid())
    # Check budget.guard(history['id']) before each native integration batch.
    budget.complete(history['id'], actual_native_steps=960, outcome=outcome)

CLI: init-preparation, freeze --manifest FILE, reserve --request FILE,
extend-preparation --request FILE,
complete --request FILE, admit-native-evidence --evidence FILE,
close --reason TEXT, status (all require --root DIRECTORY).
`reserve` request is {"kind": ..., ...}; `complete` includes `reservation_id`.

Manifest schema 1: `files` maps every REQUIRED_ROLES member to a nonempty list
of {"path": absolute_or_relative_path, "sha256": hex_digest}; relative paths
are relative to the manifest file for CLI, cwd for Python. `prerequisites`
maps every PREREQUISITES member to true. These attestations require actual
evidence in the frozen files; this ledger cannot establish their truth.
`freeze` checks STATIC prerequisites and enters frozen-verification. It does
not assume the native parity/accounting tests have already integrated physics.
Its continuation gate means a defined and checked numerical/geometrical/static
route to standing; it is not a prerequisite of demonstrated physical recovery.
Optional protocolMode='local-pair' replaces four_histories_frozen with
local_pair_frozen and closes after A/B, including successful B. Its cap is
16 same-process verification + 2*960 histories + 40*4*480 previews = 78736;
short probes are forbidden. An absent mode preserves the original contract.
Native evidence schema 1: `gates` maps NATIVE_GATES to true; `verifications` is
a nonempty list of {"reservation_id": ..., "path": ..., "sha256": ...} matching
completed verification outcomes' evidence files. At least one charged native
step must actually have been reported for every referenced verification.
Admission rechecks these hashes and the frozen files, retains the existing
clock, worker and counters, then enters frozen-ready. Evidence is rechecked at
every later native admission. No history/planning is allowed before this gate.

Reservation schema: preparation(active_seconds, inspections), history(history,
worker_pid), planning(history_id, long_steps=[1..480]*0..4,
short_probes=0..93), verification(native_steps, worker_pid).
Planning is at most 10 Hz (history_step=0,24,...,936), 40 calls per B history.
All four history reservations are 960 steps, including any replayed prefix.
Complete native reservations with actual_native_steps and outcome. Outcome
requires boolean integrity_ok/support_feasible/predictions_usable/
diagnostic_present; histories additionally require local_success. Histories
and verifications require a hashed evidence file. A failed A diagnostic still
permits B; unsuccessful B stops.

Reservations are charged in full forever, including successful unused work.
record_native_failure archives a structurally valid failed worker receipt and
its bounded actual step count, then closes admission without refund. It can
append this evidence after a deadline or closure, but never grants permission,
changes the admission clock, or reopens a reservation/protocol.
An unfinished physical reservation blocks further physical reservations except
one planning call nested in its active history. Missing prerequisites, clock
regressions, changed hashes, expired caps, process failures, and unknown outcomes
close the version. Explicit close never reopens or refunds it. A new ledger
requires a brand-new directory, and is NOT authority to renew this allocation.
No previous ledger is opened, imported, changed, or credited.

Linux /proc enforces one live native worker identity, one OS thread, and the
frozen executable at admission/guard/completion. The worker MUST call guard
before each native batch and independently bound that batch by its reservation
and the returned deadlines. A metadata journal cannot police uninstrumented
integrations, transient threads, other executables, or concurrent compilation;
the frozen native-accounting/process-isolation evidence is a launch prerequisite.
All clocks/process inspectors are injectable only for synthetic unit tests.
"""

import argparse
from contextlib import contextmanager
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid


SCHEMA = 'laboratoirehumain.coordination-prototype-budget.v1'
LIMITS = {
    'preparationActiveSeconds': 40 * 3600,
    'preparationCalendarSeconds': 5 * 24 * 3600,
    'staticInspections': 20000,
    'histories': 4,
    'nativeHz': 240,
    'historyNativeSteps': 960,
    'totalNativeSteps': 300000,
    'predictionAndVerificationSteps': 296160,
    'verificationSteps': 21120,
    'physicsWallSeconds': 2 * 3600,
    'planningCallsPerBHistory': 40,
    'planningSpacingNativeSteps': 24,
    'longRolloutsPerCall': 4,
    'longStepsPerRollout': 480,
    'shortProbesPerCall': 93,
    'shortStepsPerProbe': 4,
    'processes': 1,
    'threads': 1,
}
HISTORY_ORDER = ['A', 'B', 'B-repeat', 'B60']
DEFAULT_PROTOCOL_MODE = 'four-histories'
LOCAL_PAIR_MODE = 'local-pair'
LOCAL_PAIR_LIMITS = dict(LIMITS, histories=2, totalNativeSteps=78736,
    predictionAndVerificationSteps=76816, verificationSteps=16, shortProbesPerCall=0,
    physicsWallSeconds=7100)
REQUIRED_ROLES = ('model', 'profile', 'fixture', 'reference', 'code', 'binary',
                  'criteria', 'continuation', 'preflight_evidence')
PREREQUISITES = ('unit_and_static_tests_passed', 'complete_numeric_reference',
                 'continuation_to_standing_defined_and_verified',
                 'unchanged_physical_contract', 'native_accounting_statically_tested',
                 'single_process_single_thread_no_concurrent_compilation',
                 'prediction_copy_and_motor_paths_statically_checked',
                 'local_diagnostic_implemented', 'four_histories_frozen')
NATIVE_GATES = ('native_accounting_verified', 'prediction_state_and_motor_semantics_verified')


class BudgetError(RuntimeError):
    """Admission refused; never interpret this as permission to integrate."""


def protocol_contract(mode=DEFAULT_PROTOCOL_MODE):
    """Closed choices only; mode selection never creates or renews a ledger."""
    if mode == DEFAULT_PROTOCOL_MODE:
        return LIMITS, HISTORY_ORDER, PREREQUISITES
    if mode == LOCAL_PAIR_MODE:
        return LOCAL_PAIR_LIMITS, ['A', 'B'], PREREQUISITES[:-1] + ('local_pair_frozen',)
    raise BudgetError('Unknown frozen protocolMode')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode()


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def utc(seconds):
    return dt.datetime.fromtimestamp(seconds, dt.timezone.utc).isoformat()


def parse_utc(value):
    parsed = dt.datetime.fromisoformat(value.replace('Z', '+00:00'))
    if parsed.tzinfo is None:
        raise BudgetError('UTC start must include a timezone')
    return parsed.timestamp()


def integer(value, name, minimum=0, maximum=None):
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        raise BudgetError('Invalid ' + name)
    return value


class Clock:
    def sample(self):
        return {'wall': time.time(), 'monotonic': time.monotonic(),
                'boot': Path('/proc/sys/kernel/random/boot_id').read_text().strip()}


def inspect_process(pid):
    """PID plus boot/start ticks prevents PID reuse from resuming a crashed worker."""
    integer(pid, 'worker_pid', 1)
    root = Path('/proc') / str(pid)
    stat = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    if stat[0] in ('Z', 'X', 'x'):
        raise BudgetError('Worker is not alive')
    threads = len(list((root / 'task').iterdir()))
    if threads != 1:
        raise BudgetError('Worker must have exactly one OS thread')
    return {'pid': pid, 'startTicks': stat[19], 'binarySha256': sha(root / 'exe'),
            'boot': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'threads': threads}


def durable_save(path, value):
    """Atomic snapshot; a leftover pending write makes later access fail closed."""
    envelope = {'payload': value, 'sha256': hashlib.sha256(canonical(value)).hexdigest()}
    temporary = path.with_name('.ledger.pending')
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(envelope, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


class Budget:
    def __init__(self, root, clock=None, process_inspector=None):
        self.root = Path(root).absolute()
        self.path = self.root / 'ledger.json'
        self.clock = clock or Clock()
        self.process_inspector = process_inspector or inspect_process

    def _read_verified(self):
        if not self.path.is_file() or (self.root / '.ledger.pending').exists():
            raise BudgetError('No complete current ledger or uncertain pending write')
        try:
            envelope = json.loads(self.path.read_text())
            state = envelope['payload']
            if envelope['sha256'] != hashlib.sha256(canonical(state)).hexdigest():
                raise BudgetError('Ledger integrity digest mismatch')
            expected_limits, _, _ = protocol_contract(state.get('protocolMode', DEFAULT_PROTOCOL_MODE))
            if state['schema'] != SCHEMA or state['limits'] != expected_limits:
                raise BudgetError('Ledger schema or budget contract changed')
            return state
        except (KeyError, ValueError, TypeError) as exc:
            raise BudgetError('Malformed ledger; no physical work allowed') from exc

    def status(self):
        """Read-only receipt, including closed ledgers; never grants permission."""
        with (self.root / 'coordinator.lock').open('r') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise BudgetError('Another coordinator owns the ledger') from exc
            state = self._read_verified()
            return {'schema': state['schema'], 'id': state['id'], 'status': state['status'],
                    'protocolMode': state.get('protocolMode', DEFAULT_PROTOCOL_MODE), 'limits': state['limits'],
                    'closureReason': state.get('closureReason'), 'counters': self.counters(state),
                    'preparation': state['preparation'], 'physics': state['physics'],
                    'lastEvent': state['events'][-1], 'authorizesIntegration': False}

    def init_preparation(self, *, started_at_utc=None, initial_active_seconds=0,
                         initial_inspections=0):
        now = self.clock.sample()
        start = now['wall'] if started_at_utc is None else parse_utc(started_at_utc)
        integer(initial_active_seconds, 'initial_active_seconds', 0, LIMITS['preparationActiveSeconds'])
        integer(initial_inspections, 'initial_inspections', 0, LIMITS['staticInspections'])
        if start > now['wall'] or now['wall'] - start >= LIMITS['preparationCalendarSeconds']:
            raise BudgetError('Preparation start outside the approved five-day window')
        if initial_active_seconds > math.ceil(now['wall'] - start):
            raise BudgetError('Initial active debit exceeds elapsed preparation time')
        # Existence is refusal, even if this directory looks empty or contains an old ledger.
        self.root.mkdir(mode=0o700, parents=False, exist_ok=False)
        (self.root / 'coordinator.lock').touch(mode=0o600, exist_ok=False)
        state = {
            'schema': SCHEMA, 'id': str(uuid.uuid4()), 'limits': dict(LIMITS),
            'status': 'preparing', 'createdUtc': utc(now['wall']), 'lastClock': now,
            'preparation': {'startedUtc': utc(start),
                            'deadlineUtc': utc(start + LIMITS['preparationCalendarSeconds']),
                            'deadlineMonotonic': now['monotonic'] + LIMITS['preparationCalendarSeconds'] - (now['wall'] - start),
                            'initialActiveSeconds': initial_active_seconds,
                            'initialInspections': initial_inspections},
            'reservations': [], 'events': [], 'worker': None, 'physics': None,
        }
        self._event(state, 'init-preparation', now)
        durable_save(self.path, state)
        return state

    @contextmanager
    def _transaction(self, audit_only=False):
        if not self.path.is_file() or (self.root / '.ledger.pending').exists():
            raise BudgetError('No complete current ledger or uncertain pending write')
        with (self.root / 'coordinator.lock').open('r+') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise BudgetError('Another coordinator owns the ledger') from exc
            state = self._read_verified()
            if state['status'].startswith('closed') and not audit_only:
                raise BudgetError('Protocol is closed: ' + state.get('closureReason', ''))
            try:
                now = self.clock.sample()
                if not audit_only:
                    self._check_clock(state, now)
                yield state, now
            except (BudgetError, OSError, KeyError, ValueError, TypeError) as exc:
                state['status'] = 'closed-refused'
                state['closureReason'] = str(exc)
                self._event(state, 'closed-refused', state['lastClock'], reason=str(exc))
                durable_save(self.path, state)
                if isinstance(exc, BudgetError):
                    raise
                raise BudgetError(str(exc)) from exc
            else:
                if not audit_only:
                    state['lastClock'] = now
                durable_save(self.path, state)

    @staticmethod
    def _event(state, action, now, **details):
        state['events'].append({'sequence': len(state['events']), 'action': action,
                                'utc': utc(now['wall']), **details})

    @staticmethod
    def _check_clock(state, now):
        previous = state['lastClock']
        if (now['boot'] != previous['boot'] or
                any(not isinstance(now[k], (int, float)) or not math.isfinite(now[k])
                    for k in ('wall', 'monotonic')) or
                now['wall'] < previous['wall'] or now['monotonic'] < previous['monotonic']):
            raise BudgetError('Clock regressed or machine rebooted; no budget renewal')
        if state['status'] == 'preparing' and (now['wall'] >= parse_utc(state['preparation']['deadlineUtc']) or
                                              now['monotonic'] >= state['preparation']['deadlineMonotonic']):
            raise BudgetError('Preparation five-day deadline reached')
        if state['physics'] and (now['wall'] >= state['physics']['deadlineWall'] or
                                 now['monotonic'] >= state['physics']['deadlineMonotonic']):
            raise BudgetError('Physics two-hour wall deadline reached')

    @staticmethod
    def counters(state):
        entries = state['reservations']
        known = sum(e.get('actualNativeSteps', 0) for e in entries)
        unreported = sum(e.get('nativeSteps', 0) for e in entries
                         if 'actualNativeSteps' not in e)
        return {
            'preparationActiveSeconds': state['preparation']['initialActiveSeconds'] +
                sum(e.get('activeSeconds', 0) for e in entries),
            'staticInspections': state['preparation']['initialInspections'] +
                sum(e.get('inspections', 0) for e in entries),
            'reservedNativeSteps': sum(e.get('nativeSteps', 0) for e in entries),
            'reservedPredictionAndVerificationSteps': sum(e.get('nativeSteps', 0) for e in entries
                                                         if e['kind'] != 'history'),
            'reservedVerificationSteps': sum(e.get('nativeSteps', 0) for e in entries
                                             if e['kind'] == 'verification'),
            'actualNativeStepsKnown': known,
            'actualNativeStepsExact': unreported == 0,
            'unreportedReservedNativeSteps': unreported,
            'actualNativeStepsLowerBound': known,
            'actualNativeStepsUpperBound': known + unreported,
        }

    @staticmethod
    def _manifest(manifest, base):
        if manifest.get('schema') != 1:
            raise BudgetError('Manifest schema must be 1')
        _, _, prerequisites = protocol_contract(manifest.get('protocolMode', DEFAULT_PROTOCOL_MODE))
        if any(manifest.get('prerequisites', {}).get(p) is not True for p in prerequisites):
            missing = [p for p in prerequisites if manifest.get('prerequisites', {}).get(p) is not True]
            raise BudgetError('Missing preparation prerequisites: ' + ', '.join(missing))
        files = {}
        for role in REQUIRED_ROLES:
            records = manifest.get('files', {}).get(role)
            if not isinstance(records, list) or not records:
                raise BudgetError('Missing frozen role: ' + role)
            files[role] = []
            for record in records:
                path = Path(record['path'])
                path = (base / path).resolve(strict=True) if not path.is_absolute() else path.resolve(strict=True)
                digest = record['sha256']
                if not isinstance(digest, str) or len(digest) != 64 or sha(path) != digest:
                    raise BudgetError('Frozen input hash mismatch: ' + str(path))
                files[role].append({'path': str(path), 'sha256': digest})
        normalized = {'schema': 1, 'files': files, 'prerequisites': dict(manifest['prerequisites'])}
        if 'protocolMode' in manifest:
            normalized['protocolMode'] = manifest['protocolMode']
        return normalized

    @staticmethod
    def _verify_frozen(state, allow_verification=False):
        allowed = ('frozen-verification', 'frozen-ready') if allow_verification else ('frozen-ready',)
        if state['status'] not in allowed:
            raise BudgetError('History/planning requires frozen-ready; verification requires frozen-verification')
        if state.get('protocolMode', DEFAULT_PROTOCOL_MODE) != state['manifest'].get('protocolMode', DEFAULT_PROTOCOL_MODE):
            raise BudgetError('Frozen protocolMode changed')
        if hashlib.sha256(canonical(state['manifest'])).hexdigest() != state['manifestSha256']:
            raise BudgetError('Frozen manifest changed')
        for records in state['manifest']['files'].values():
            for record in records:
                if sha(record['path']) != record['sha256']:
                    raise BudgetError('Frozen input changed: ' + record['path'])
        if state['status'] == 'frozen-ready':
            if hashlib.sha256(canonical(state['nativeEvidence'])).hexdigest() != state['nativeEvidenceSha256']:
                raise BudgetError('Native admission evidence changed')
            for record in state['nativeEvidence']['verifications']:
                if sha(record['path']) != record['sha256']:
                    raise BudgetError('Native verification evidence changed: ' + record['path'])

    def freeze(self, manifest, base=None):
        with self._transaction() as (state, now):
            if state['status'] != 'preparing':
                raise BudgetError('Only preparing protocol may be frozen')
            if any(e['status'] == 'reserved' for e in state['reservations']):
                raise BudgetError('Unfinished preparation reservation')
            state['manifest'] = self._manifest(manifest, Path(base or Path.cwd()))
            state['protocolMode'] = state['manifest'].get('protocolMode', DEFAULT_PROTOCOL_MODE)
            state['limits'] = dict(protocol_contract(state['protocolMode'])[0])
            state['manifestSha256'] = hashlib.sha256(canonical(state['manifest'])).hexdigest()
            state['status'] = 'frozen-verification'
            state['frozenUtc'] = utc(now['wall'])
            self._event(state, 'freeze', now, manifestSha256=state['manifestSha256'])
        return {'status': state['status'], 'manifestSha256': state['manifestSha256']}

    def admit_native_evidence(self, evidence, base=None):
        """Admit charged native preflight results without resetting any allocation."""
        with self._transaction() as (state, now):
            if state['status'] != 'frozen-verification':
                raise BudgetError('Native evidence admission requires frozen-verification')
            self._verify_frozen(state, allow_verification=True)
            if any(e['status'] == 'reserved' for e in state['reservations']):
                raise BudgetError('Unfinished reservation at native evidence admission')
            if evidence.get('schema') != 1 or any(evidence.get('gates', {}).get(k) is not True for k in NATIVE_GATES):
                raise BudgetError('Missing native accounting or prediction parity gate')
            records = evidence.get('verifications')
            if not isinstance(records, list) or not records:
                raise BudgetError('Native admission requires completed charged verification evidence')
            accepted = []
            seen = set()
            for record in records:
                reservation_id = record['reservation_id']
                entry = next((e for e in state['reservations'] if e['id'] == reservation_id), None)
                if (reservation_id in seen or entry is None or entry['kind'] != 'verification' or
                        entry['status'] != 'completed' or entry.get('actualNativeSteps', 0) <= 0):
                    raise BudgetError('Evidence is not a unique completed charged native verification')
                seen.add(reservation_id)
                path = Path(record['path'])
                path = (Path(base or Path.cwd()) / path).resolve(strict=True)
                normalized = {'path': str(path), 'sha256': record['sha256']}
                if normalized != entry['outcome']['evidence'] or sha(path) != record['sha256']:
                    raise BudgetError('Native evidence does not match completed verification receipt')
                accepted.append({'reservation_id': reservation_id, **normalized})
            self._worker(state)
            state['nativeEvidence'] = {'schema': 1, 'gates': {k: True for k in NATIVE_GATES},
                                       'verifications': accepted}
            state['nativeEvidenceSha256'] = hashlib.sha256(canonical(state['nativeEvidence'])).hexdigest()
            state['status'] = 'frozen-ready'
            self._event(state, 'admit-native-evidence', now,
                        nativeEvidenceSha256=state['nativeEvidenceSha256'], counters=self.counters(state))
        return {'status': state['status'], 'nativeEvidenceSha256': state['nativeEvidenceSha256']}

    def _worker(self, state, pid=None):
        pid = pid if pid is not None else (state['worker']['pid'] if state['worker'] else os.getpid())
        identity = self.process_inspector(pid)
        if identity.get('threads') != 1 or identity.get('pid') != pid:
            raise BudgetError('Expected one live single-threaded worker')
        hashes = {e['sha256'] for e in state['manifest']['files']['binary']}
        if identity['binarySha256'] not in hashes:
            raise BudgetError('Worker executable is not the frozen binary')
        if state['worker'] is not None and identity != state['worker']:
            raise BudgetError('Worker changed, restarted, or became multithreaded')
        state['worker'] = identity

    def extend_preparation(self, reservation_id, *, additional_inspections,
                           additional_active_seconds=0):
        """Irreversible top-up of an unexpired preparation lease, never a restart."""
        with self._transaction() as (state, now):
            entry = next((e for e in state['reservations'] if e['id'] == reservation_id), None)
            if (state['status'] != 'preparing' or entry is None or entry['kind'] != 'preparation' or
                    entry['status'] != 'reserved'):
                raise BudgetError('Only an active preparation reservation can be extended')
            elapsed = max(now['monotonic'] - entry['startedMonotonic'],
                          now['wall'] - parse_utc(entry['startedUtc']))
            if elapsed >= entry['activeSeconds']:
                raise BudgetError('Expired preparation reservation cannot be renewed')
            inspections = integer(additional_inspections, 'additional_inspections')
            seconds = integer(additional_active_seconds, 'additional_active_seconds')
            if not inspections and not seconds:
                raise BudgetError('Preparation extension must reserve additional work')
            counts = self.counters(state)
            if counts['staticInspections'] + inspections > LIMITS['staticInspections']:
                raise BudgetError('Static inspection cap reached')
            if counts['preparationActiveSeconds'] + seconds > LIMITS['preparationActiveSeconds']:
                raise BudgetError('Preparation active-time cap reached')
            entry['inspections'] += inspections
            entry['activeSeconds'] += seconds
            self._event(state, 'extend-preparation', now, reservationId=reservation_id,
                        additionalInspections=inspections, additionalActiveSeconds=seconds,
                        counters=self.counters(state))
        return entry

    def reserve(self, kind, **request):
        with self._transaction() as (state, now):
            limits, history_order, _ = protocol_contract(state.get('protocolMode', DEFAULT_PROTOCOL_MODE))
            entry = {'id': str(uuid.uuid4()), 'kind': kind, 'status': 'reserved',
                     'startedUtc': utc(now['wall']), 'startedMonotonic': now['monotonic']}
            pending = [e for e in state['reservations'] if e['status'] == 'reserved']
            counts = self.counters(state)
            if kind == 'preparation':
                if state['status'] != 'preparing' or pending:
                    raise BudgetError('Preparation not open or previous reservation unfinished')
                active = integer(request.pop('active_seconds'), 'active_seconds', 1)
                inspections = integer(request.pop('inspections', 0), 'inspections')
                if counts['preparationActiveSeconds'] + active > LIMITS['preparationActiveSeconds']:
                    raise BudgetError('Preparation active-time cap reached')
                if counts['staticInspections'] + inspections > LIMITS['staticInspections']:
                    raise BudgetError('Static inspection cap reached')
                entry.update(activeSeconds=active, inspections=inspections)
            else:
                self._verify_frozen(state, allow_verification=kind == 'verification')
                self._worker(state, request.pop('worker_pid', None))
                if kind == 'history':
                    if pending:
                        raise BudgetError('Uncertain previous native reservation; no new history')
                    histories = [e for e in state['reservations'] if e['kind'] == 'history']
                    index = len(histories)
                    if index >= limits['histories']:
                        raise BudgetError('Frozen history-count cap reached')
                    name = request.pop('history')
                    if name != history_order[index]:
                        raise BudgetError('Frozen history order is ' + ', '.join(history_order))
                    if index >= 2 and any(not e['outcome']['local_success'] for e in histories[1:]):
                        raise BudgetError('B repetitions require the preceding local diagnostic')
                    entry.update(history=name, headingDegrees=60 if name == 'B60' else 0,
                                 nativeSteps=limits['historyNativeSteps'])
                elif kind == 'planning':
                    history_id = request.pop('history_id')
                    history = next((e for e in pending if e['id'] == history_id and e['kind'] == 'history'), None)
                    if history is None or history['history'] == 'A' or len(pending) != 1:
                        raise BudgetError('Planning requires one active B history and no uncertain call')
                    calls = [e for e in state['reservations'] if e.get('historyId') == history_id]
                    if len(calls) >= limits['planningCallsPerBHistory']:
                        raise BudgetError('Forty planning-call cap reached')
                    step = integer(request.pop('history_step'), 'history_step', 0, 959)
                    if step % 24 or (calls and step < calls[-1]['historyStep'] + 24):
                        raise BudgetError('Planning cadence exceeds 10 Hz or repeats a call')
                    longs = request.pop('long_steps', [])
                    if not isinstance(longs, list) or len(longs) > limits['longRolloutsPerCall']:
                        raise BudgetError('At most four long rollouts per planning call')
                    for steps in longs:
                        integer(steps, 'long rollout steps', 1, limits['longStepsPerRollout'])
                    probes = integer(request.pop('short_probes', 0), 'short_probes', 0, limits['shortProbesPerCall'])
                    if state.get('protocolMode') == LOCAL_PAIR_MODE and longs != [480] * 4:
                        raise BudgetError('Local pair requires the frozen four 480-step previews')
                    entry.update(historyId=history_id, historyStep=step, longSteps=longs,
                                 shortProbes=probes, nativeSteps=sum(longs) + 4 * probes)
                elif kind == 'verification':
                    if pending:
                        raise BudgetError('Verification requires no outstanding native reservation')
                    steps = integer(request.pop('native_steps'), 'native_steps', 1)
                    if state.get('protocolMode') == LOCAL_PAIR_MODE and steps != 16:
                        raise BudgetError('Local pair requires exactly 16 same-process verification steps')
                    if counts['reservedVerificationSteps'] + steps > limits['verificationSteps']:
                        raise BudgetError('Technical-verification step cap reached')
                    entry['nativeSteps'] = steps
                else:
                    raise BudgetError('Unknown reservation kind')
                if counts['reservedNativeSteps'] + entry['nativeSteps'] > limits['totalNativeSteps']:
                    raise BudgetError('Total ' + str(limits['totalNativeSteps']) + ' native-step cap reached')
                if kind != 'history' and (counts['reservedPredictionAndVerificationSteps'] + entry['nativeSteps'] >
                                          limits['predictionAndVerificationSteps']):
                    raise BudgetError('Prediction/verification allocation exhausted')
                if state['physics'] is None:
                    state['physics'] = {'startedUtc': utc(now['wall']),
                                        'deadlineWall': now['wall'] + limits['physicsWallSeconds'],
                                        'deadlineMonotonic': now['monotonic'] + limits['physicsWallSeconds']}
                entry['deadlineUtc'] = utc(state['physics']['deadlineWall'])
                entry['deadlineMonotonic'] = state['physics']['deadlineMonotonic']
            if request:
                raise BudgetError('Unknown reservation parameters: ' + ', '.join(sorted(request)))
            state['reservations'].append(entry)
            self._event(state, 'reserve', now, reservationId=entry['id'],
                        counters=self.counters(state))
        return entry

    def guard(self, reservation_id):
        """Check before an integration batch; durable reservation already covers it.

        Does not spend extra credit or count an integration. The instrumented
        worker must reject actual step counts above the reservation on its own.
        """
        with self._transaction() as (state, now):
            entry = next((e for e in state['reservations'] if e['id'] == reservation_id), None)
            if entry is None or entry['status'] != 'reserved' or entry['kind'] == 'preparation':
                raise BudgetError('No active native reservation')
            self._verify_frozen(state, allow_verification=entry['kind'] == 'verification')
            self._worker(state)
            pending = [e for e in state['reservations'] if e['status'] == 'reserved']
            if entry['kind'] == 'history' and len(pending) != 1:
                raise BudgetError('Complete nested prediction before integrating the history')
            result = {'nativeSteps': entry['nativeSteps'],
                      'remainingWallSeconds': min(state['physics']['deadlineWall'] - now['wall'],
                                                 state['physics']['deadlineMonotonic'] - now['monotonic']),
                      'deadlineUtc': entry['deadlineUtc'], 'deadlineMonotonic': entry['deadlineMonotonic']}
        return result

    def complete(self, reservation_id, *, actual_native_steps=None, actual_inspections=None,
                 outcome=None, success=True):
        with self._transaction() as (state, now):
            entry = next((e for e in state['reservations'] if e['id'] == reservation_id), None)
            if entry is None or entry['status'] != 'reserved':
                raise BudgetError('Unknown or already completed reservation')
            if type(success) is not bool:
                raise BudgetError('Completion success must be a boolean')
            if not success:
                raise BudgetError('Worker failed or timed out; reservation retained')
            if entry['kind'] == 'preparation':
                if actual_native_steps is not None or outcome is not None:
                    raise BudgetError('Preparation never includes native integration')
                integer(actual_inspections, 'actual_inspections', 0, entry['inspections'])
                elapsed = max(now['monotonic'] - entry['startedMonotonic'],
                              now['wall'] - parse_utc(entry['startedUtc']))
                if elapsed > entry['activeSeconds']:
                    raise BudgetError('Reserved preparation time exceeded')
                entry.update(actualInspections=actual_inspections, elapsedSeconds=elapsed)
            else:
                self._verify_frozen(state, allow_verification=entry['kind'] == 'verification')
                self._worker(state)
                integer(actual_native_steps, 'actual_native_steps', 0, entry['nativeSteps'])
                if (state.get('protocolMode') == LOCAL_PAIR_MODE and entry['kind'] == 'verification'
                        and actual_native_steps != 16):
                    raise BudgetError('Local pair verification must complete exactly 16 native steps')
                if actual_inspections is not None:
                    raise BudgetError('Native completion cannot credit preparation inspections')
                if not isinstance(outcome, dict) or any(outcome.get(k) is not True for k in
                    ('integrity_ok', 'support_feasible', 'predictions_usable', 'diagnostic_present')):
                    raise BudgetError('Missing/failed integrity, support, prediction or expected diagnostic')
                if entry['kind'] == 'history':
                    if any(e['status'] == 'reserved' and e.get('historyId') == entry['id']
                           for e in state['reservations']):
                        raise BudgetError('Uncertain nested prediction at history completion')
                    if type(outcome.get('local_success')) is not bool:
                        raise BudgetError('History needs an explicit local-success diagnostic')
                if entry['kind'] in ('history', 'verification'):
                    evidence = outcome.get('evidence', {})
                    path = Path(evidence['path']).resolve(strict=True)
                    if sha(path) != evidence.get('sha256'):
                        raise BudgetError('Missing or changed native diagnostic evidence')
                    outcome = dict(outcome, evidence={'path': str(path), 'sha256': evidence['sha256']})
                entry.update(actualNativeSteps=actual_native_steps, outcome=outcome)
            entry.update(status='completed', endedUtc=utc(now['wall']))
            self._event(state, 'complete', now, reservationId=entry['id'], counters=self.counters(state))
            if entry['kind'] == 'history' and entry['history'] != 'A':
                local_pair = state.get('protocolMode') == LOCAL_PAIR_MODE
                if local_pair or not outcome['local_success'] or entry['history'] == 'B60':
                    state['status'] = 'closed-completed'
                    state['closureReason'] = ('local pair A/B completed; no replay or continuation authorization'
                        if local_pair else 'four frozen histories completed; no full-history authorization'
                        if outcome['local_success'] else 'B local diagnostic failed')
                    self._event(state, 'close', now, reason=state['closureReason'])
        return entry

    def record_native_failure(self, reservation_id, *, actual_native_steps, outcome, reason):
        """Evidence-only write, including after expiry; never admission to physics.

        The receipt must refer to this ledger's still-reserved native operation,
        report a bounded integer count, and supply a readable hashed file. Any
        unreported nested reservation stays fully charged and uncertain.
        """
        with self._transaction(audit_only=True) as (state, now):
            entry = next((e for e in state['reservations'] if e['id'] == reservation_id), None)
            if entry is None or entry['status'] != 'reserved' or entry['kind'] == 'preparation':
                raise BudgetError('Failure receipt requires an outstanding native reservation')
            integer(actual_native_steps, 'actual_native_steps', 0, entry['nativeSteps'])
            if not isinstance(reason, str) or not reason.strip() or not isinstance(outcome, dict):
                raise BudgetError('Failure receipt requires its outcome and a refusal reason')
            for key in ('integrity_ok', 'support_feasible', 'predictions_usable', 'diagnostic_present'):
                if type(outcome.get(key)) is not bool:
                    raise BudgetError('Failure outcome requires explicit boolean diagnostics')
            if entry['kind'] == 'history' and type(outcome.get('local_success')) is not bool:
                raise BudgetError('Failure history receipt requires explicit local_success')
            evidence = outcome.get('evidence', {})
            path = Path(evidence['path']).resolve(strict=True)
            if sha(path) != evidence.get('sha256'):
                raise BudgetError('Failure diagnostic evidence hash mismatch')
            canonical(outcome)  # Reject non-JSON/nonfinite metadata before changing the entry.
            clock_refusal = None
            try:
                self._check_clock(state, now)
            except BudgetError as exc:
                clock_refusal = str(exc)
            entry.update(status='failed-reported', actualNativeSteps=actual_native_steps,
                         outcome=dict(outcome, evidence={'path': str(path), 'sha256': evidence['sha256']}),
                         endedUtc=utc(now['wall']), failureReason=reason,
                         admissionRefusalAtReceipt=clock_refusal)
            if not state['status'].startswith('closed'):
                state['status'] = 'closed-native-failure'
                state['closureReason'] = reason
            self._event(state, 'record-native-failure', now, reservationId=reservation_id,
                        reason=reason, counters=self.counters(state), authorizesIntegration=False)
        return entry

    def close(self, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise BudgetError('Closure requires a reason')
        with self._transaction() as (state, now):
            state['status'] = 'closed-explicit'
            state['closureReason'] = reason
            self._event(state, 'close', now, reason=reason, counters=self.counters(state))
        return {'status': state['status'], 'reason': reason, 'counters': self.counters(state)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--root', type=Path, required=True)
    commands = parser.add_subparsers(dest='command', required=True)
    init = commands.add_parser('init-preparation')
    init.add_argument('--started-at-utc')
    init.add_argument('--initial-active-seconds', type=int, default=0)
    init.add_argument('--initial-inspections', type=int, default=0)
    freeze = commands.add_parser('freeze')
    freeze.add_argument('--manifest', type=Path, required=True)
    admission = commands.add_parser('admit-native-evidence')
    admission.add_argument('--evidence', type=Path, required=True)
    for name in ('reserve', 'complete', 'extend-preparation'):
        commands.add_parser(name).add_argument('--request', type=Path, required=True)
    commands.add_parser('close').add_argument('--reason', required=True)
    commands.add_parser('status')
    args = parser.parse_args()
    budget = Budget(args.root)
    try:
        if args.command == 'init-preparation':
            result = budget.init_preparation(started_at_utc=args.started_at_utc,
                                             initial_active_seconds=args.initial_active_seconds,
                                             initial_inspections=args.initial_inspections)
        elif args.command == 'freeze':
            result = budget.freeze(json.loads(args.manifest.read_text()), args.manifest.resolve().parent)
        elif args.command == 'admit-native-evidence':
            result = budget.admit_native_evidence(json.loads(args.evidence.read_text()), args.evidence.resolve().parent)
        elif args.command in ('reserve', 'complete', 'extend-preparation'):
            result = getattr(budget, args.command.replace('-', '_'))(**json.loads(args.request.read_text()))
        elif args.command == 'close':
            result = budget.close(args.reason)
        else:
            result = budget.status()
        print(json.dumps(result, indent=2, allow_nan=False))
    except (BudgetError, OSError, ValueError, TypeError) as exc:
        parser.exit(2, 'REFUSED: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
