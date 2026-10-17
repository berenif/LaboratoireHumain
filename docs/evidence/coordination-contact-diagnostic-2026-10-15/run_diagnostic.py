#!/usr/bin/env python3
"""Single contact diagnostic supervisor, with no real transfer. No execution on import or without --execute.

Inspect only: --manifest FILE (no approval needed and no worker is started).
Execute only after distinct approval: --manifest FILE --authorization FILE --execute
No approved authorization or frozen manifest is supplied by this implementation.
The schemas/constants below are the complete, strict external contract. Build the
dedicated worker separately, before the 120-second clock. Never reuse a batch.
Tests may inject inert workers; neither this file nor its tests run physics now.

The immutable claim burns 52 + 516 steps before spawn. Two UUID reservations
are bound to one PID before one contact-diagnostic request. Ready has at most 30 seconds;
native execution stops seven seconds before the overall 120-second deadline.
No refund, resume, extra operation, renewed horizon or second output is supported.
The old supervisor provides only read-only JSON/process/pipe helpers; its run,
budget and old authorization paths are never called.
"""

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import time
import uuid

REPO = Path(__file__).resolve().parents[3]
BATCH_ROOT = Path(__file__).resolve().parent
COMMON_PATH = REPO / 'scripts/run-coordination-numeric-parity.py'
_spec = importlib.util.spec_from_file_location('transfer_supervision_common', COMMON_PATH)
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)
Refusal = common.Refusal
Clock = common.Clock
NativeWorker = common.NativeWorker
canonical, exact, read_json, sha = common.canonical, common.exact, common.read_json, common.sha
write_new, utc = common.write_new, common.utc

PROTOCOL = 'contact-diagnostic-a36-568-120s-2026-10-15'
MANIFEST_SCHEMA = 'laboratoirehumain.contact-diagnostic-manifest.v1'
AUTHORIZATION_SCHEMA = 'laboratoirehumain.contact-diagnostic-authorization.v1'
TECHNICAL, HISTORY, TOTAL = 52, 516, 568
WALL_SECONDS, STARTUP_SECONDS, STOP_SECONDS = 120, 30, 7
REAL_STEPS, PREVIEWS, HORIZON = 36, 1, 480
PREPARATION = REPO / 'docs/evidence/coordination-transfer-candidate-2026-10-13/preparation-reservation.json'
PREPARATION_SHA256 = 'd7940da3ff7f3d29898eb25f1c09a2487158b6ff34820dad8ce3491766e33f0b'
INPUT_ROLES = {'programPath': 'program', 'tasksPath': 'tasks',
               'entryPath': 'entry', 'prefixFramesPath': 'prefix'}
ROLES = {'binary', 'code', 'program', 'tasks', 'entry', 'prefix', 'model',
         'profile', 'criteria', 'validation', 'protocol', 'preparation'}
GATES = ('integrity_ok', 'technical_parity_passed', 'diagnostic_completed', 'no_real_transfer')
RESULT_FIELDS = ('schema', 'event', 'op', 'pid', 'operationId',
    'technicalReservationId', 'historyReservationId', 'reservedNativeSteps',
    'technicalReservedNativeSteps', 'historyReservedNativeSteps',
    'technicalActualNativeSteps', 'historyActualNativeSteps', 'actualNativeSteps',
    'outcome', 'evidencePath', 'refusal', 'countsReconciled', 'technicalAccounting',
    'historyAccounting', 'prefixExclusions', 'noRefund', 'automaticRenewalAllowed')
PREFIX_EXCLUSIONS = ['kind label only', 'owner.transfer absent or null only',
    'support.native_contact_boundary additive telemetry only; live presence and clock checked']
PREREQUISITES = ('static_tests_passed', 'dedicated_release_built_without_execution',
    'unchanged_physical_contract', 'archived_a36_prefix_available',
    'complete_decision_state_parity_reviewed', 'single_process_single_thread',
    'diagnostic_and_stop_rules_frozen', 'distinct_native_authorization_required')
LIMITS = {'technicalNativeSteps': TECHNICAL, 'historyAndPredictionNativeSteps': HISTORY,
          'totalNativeSteps': TOTAL, 'maximumWallSeconds': WALL_SECONDS,
          'maximumRealHistorySteps': REAL_STEPS, 'maximumPreviewCalls': PREVIEWS,
          'maximumStepsPerPreview': HORIZON, 'maximumHistories': 0, 'maximumDiagnosticRuns': 1}


def bounded(value, maximum, *, nullable=False):
    if nullable and value is None:
        return None
    if type(value) is not int or not 0 <= value <= maximum:
        raise Refusal('Invalid bounded native counter')
    return value


def required_code_paths():
    """Freeze the whole Rust source inventory and both supervision sources."""
    paths = {str(Path(__file__).resolve()), str(COMMON_PATH)}
    for path in (REPO / 'rust').rglob('*'):
        if (path.is_file() and 'target' not in path.relative_to(REPO).parts and
                (path.suffix == '.rs' or path.name in ('Cargo.toml', 'Cargo.lock'))):
            paths.add(str(path.resolve()))
    return paths


def validate_authorization(mp, manifest, authorization_path):
    ap = common.absolute_file(str(Path(authorization_path).absolute()))
    authorization = read_json(ap)
    exact(authorization, ('schema', 'protocolId', 'authorizationId', 'approved',
        'approvalEvidence', 'approvedAtUtc', 'manifestSha256', 'singleUse',
        'reopenHistoricalFamily', 'limits'))
    if (manifest['schema'] != MANIFEST_SCHEMA or authorization['schema'] != AUTHORIZATION_SCHEMA
            or manifest['protocolId'] != PROTOCOL or authorization['protocolId'] != PROTOCOL):
        raise Refusal('Wrong protocol or schema')
    identifier = authorization['authorizationId']
    if (not isinstance(identifier, str) or not identifier.startswith('contact-diagnostic-distinct-')
            or len(identifier) < 30 or identifier != manifest['authorizationId']
            or authorization['approved'] is not True or authorization['singleUse'] is not True
            or authorization['reopenHistoricalFamily'] is not False
            or not isinstance(authorization['approvalEvidence'], str)
            or not authorization['approvalEvidence'].strip()
            or authorization['manifestSha256'] != sha(mp)):
        raise Refusal('A distinct explicit approval tied to this manifest is required')
    try:
        approval_time = datetime.fromisoformat(authorization['approvedAtUtc'])
        if approval_time.tzinfo is None or approval_time.timestamp() > time.time():
            raise ValueError('approval time')
    except (TypeError, ValueError, OverflowError) as exc:
        raise Refusal('Approval needs a valid nonfuture timezone-aware timestamp') from exc
    exact(authorization['limits'], LIMITS)
    if any(type(authorization['limits'][k]) is not int or authorization['limits'][k] != v
           for k, v in LIMITS.items()):
        raise Refusal('Authorization must match the exact distinct step/time limits')
    return authorization, ap


def preflight(manifest_path, authorization_path=None, *, batch_root=BATCH_ROOT,
              require_authorization=True):
    mp = common.absolute_file(str(Path(manifest_path).absolute()))
    manifest = read_json(mp)
    exact(manifest, ('schema', 'protocolId', 'authorizationId', 'binaryPath',
                     'outputPath', 'inputs', 'files', 'prerequisites'))
    identifier = manifest['authorizationId']
    if (manifest['schema'] != MANIFEST_SCHEMA or manifest['protocolId'] != PROTOCOL
            or not isinstance(identifier, str) or not identifier.startswith('contact-diagnostic-distinct-')
            or len(identifier) < 30):
        raise Refusal('Wrong manifest schema, protocol or distinct authorization identity')
    if require_authorization and authorization_path is None:
        raise Refusal('Execution requires a distinct explicit authorization JSON; inspection cannot authorize it')
    authorization, ap = ((None, None) if authorization_path is None
                         else validate_authorization(mp, manifest, authorization_path))
    exact(manifest['prerequisites'], PREREQUISITES)
    if any(manifest['prerequisites'][k] is not True for k in PREREQUISITES):
        raise Refusal('A frozen prerequisite is absent or false')
    batch = Path(batch_root).resolve(strict=True)
    output = Path(manifest['outputPath'])
    if (not output.is_absolute() or output.parent != batch or output.resolve() != output
            or output.exists() or output.is_symlink()):
        raise Refusal('Output must be a new direct child of the fixed batch directory')
    if (batch / 'authorization-claim.json').exists():
        raise Refusal('This batch was already claimed; no retry or different output')
    if not isinstance(manifest['files'], list) or not manifest['files']:
        raise Refusal('Frozen source/binary/input files are required')
    records, roles = {}, {}
    for item in manifest['files']:
        exact(item, ('path', 'sha256', 'role'))
        if item['role'] not in ROLES or item['path'] in records:
            raise Refusal('Unknown role or duplicate frozen path')
        record = common.checked_record({k: item[k] for k in ('path', 'sha256')})
        records[record['path']] = record
        roles.setdefault(item['role'], set()).add(record['path'])
    if set(roles) != ROLES or not required_code_paths().issubset(roles['code']):
        raise Refusal('Incomplete frozen roles or Rust/supervisor source inventory')
    binary = common.absolute_file(manifest['binaryPath'])
    if roles['binary'] != {str(binary)} or not os.access(binary, os.X_OK):
        raise Refusal('Exactly one dedicated frozen executable required')
    exact(manifest['inputs'], INPUT_ROLES)
    for key, role in INPUT_ROLES.items():
        path = common.absolute_file(manifest['inputs'][key])
        if str(path) not in roles[role] or not isinstance(read_json(path), dict):
            raise Refusal('Worker input is not frozen JSON in the required role')
    prefix = read_json(manifest['inputs']['prefixFramesPath'])
    if prefix.get('schema') != 1 or not isinstance(prefix.get('prefixFrames'), list) or len(prefix['prefixFrames']) != 36:
        raise Refusal('Exactly 36 archived prefix frames are required')
    entry = read_json(manifest['inputs']['entryPath'])
    if entry.get('nativeSubstep') != 36:
        raise Refusal('Only the frozen A36 entry is admitted')
    if str(PREPARATION) not in roles['preparation'] or sha(PREPARATION) != PREPARATION_SHA256:
        raise Refusal('Original open preparation and protected history must be preserved')
    preparation = read_json(PREPARATION)
    protected = []
    for key in ('protectedClosedLedgers', 'protectedBinaries', 'protectedManifests'):
        for item in preparation[key]:
            protected.append(common.checked_record({'path': str(REPO / item['path']), 'sha256': item['sha256']}))
    if len(protected) != 14 or str(binary) in {x['path'] for x in protected}:
        raise Refusal('Four historical binaries and all closed records must be preserved')
    common.environment_preflight()  # Before a claim, reservation or process.
    return {'manifest': manifest, 'manifestPath': str(mp), 'manifestSha256': sha(mp),
        'authorization': authorization, 'authorizationPath': None if ap is None else str(ap),
        'authorizationSha256': None if ap is None else sha(ap),
        'binary': binary, 'binarySha256': sha(binary), 'output': output, 'batchRoot': batch,
        'records': list(records.values()), 'protected': protected}


def verify_files(prepared):
    for record in prepared['records'] + prepared['protected'] + [
        {'path': prepared['manifestPath'], 'sha256': prepared['manifestSha256']},
        {'path': prepared['authorizationPath'], 'sha256': prepared['authorizationSha256']}]:
        common.checked_record(record)


class Journal:
    """Append-first hash-chain journal plus an atomic readable state snapshot."""
    def __init__(self, prepared, clock):
        self.clock, self.output = clock, prepared['output']
        self.start, self.start_mono = clock.wall(), clock.monotonic()
        self.deadline = self.start_mono + WALL_SECONDS
        self.native_deadline = self.deadline - STOP_SECONDS
        self.startup_deadline = self.start_mono + STARTUP_SECONDS
        self.state = {'schema': 'laboratoirehumain.contact-diagnostic-native-ledger.v1',
            'authorizationId': prepared['authorization']['authorizationId'],
            'operationId': str(uuid.uuid4()), 'status': 'reserved',
            'reservedNativeSteps': TOTAL, 'refundNativeSteps': 0,
            'actualNativeStepsKnown': 0, 'actualNativeStepsLowerBound': 0,
            'actualNativeStepsUpperBound': 0, 'operationMayHaveStarted': False, 'worker': None,
            'startedUtc': utc(self.start), 'startedMonotonic': self.start_mono,
            'deadlineUnixS': self.start + WALL_SECONDS, 'deadlineMonotonic': self.deadline,
            'nativeDeadlineUnixS': self.start + WALL_SECONDS - STOP_SECONDS,
            'nativeDeadlineMonotonic': self.native_deadline,
            'maximumWallSeconds': WALL_SECONDS, 'startupMaximumSeconds': STARTUP_SECONDS,
            'shutdownReserveSeconds': STOP_SECONDS, 'manifestSha256': prepared['manifestSha256'],
            'authorizationSha256': prepared['authorizationSha256'],
            'reservations': {name: {'reservationId': str(uuid.uuid4()),
                'reservedSteps': steps, 'actualNativeStepsKnown': 0,
                'actualNativeStepsLowerBound': 0, 'actualNativeStepsUpperBound': 0, 'workerPid': None}
                for name, steps in (('technical', TECHNICAL), ('history', HISTORY))}}
        # Claim is exclusive and durable before directory creation or spawn.
        write_new(prepared['batchRoot'] / 'authorization-claim.json',
            {'authorizationId': self.state['authorizationId'], 'operationId': self.state['operationId'],
             'outputPath': str(self.output), 'reservedNativeSteps': TOTAL,
             'reservations': self.state['reservations'], 'maximumWallSeconds': WALL_SECONDS,
             'startedUtc': self.state['startedUtc'], 'manifestSha256': prepared['manifestSha256']})
        self.output.mkdir()
        common.fsync_directory(self.output.parent)
        self.events = (self.output / 'ledger-events.ndjson').open('xb')
        common.fsync_directory(self.output)
        self.sequence, self.previous_hash = 0, None
        self.save('reserve-two-budgets-before-spawn')

    def save(self, event, **fields):
        entry = {'sequence': self.sequence, 'previousSha256': self.previous_hash,
                 'event': event, 'utc': utc(self.clock.wall()), 'state': self.state, **fields}
        digest = hashlib.sha256(canonical(entry)).hexdigest()
        self.events.write(canonical({'payload': entry, 'sha256': digest}) + b'\n')
        self.events.flush()
        os.fsync(self.events.fileno())
        self.sequence += 1
        self.previous_hash = digest
        temporary = self.output / 'physical-ledger.next.json'
        snapshot = {'payload': self.state, 'sha256': hashlib.sha256(canonical(self.state)).hexdigest(),
                    'lastEventSha256': digest, 'events': self.sequence}
        write_new(temporary, snapshot)
        os.replace(temporary, self.output / 'physical-ledger.json')
        common.fsync_directory(self.output)

    def guard(self, *, native=True):
        wall = self.clock.wall()
        deadline = self.native_deadline if native else self.deadline
        wall_deadline = self.state['nativeDeadlineUnixS'] if native else self.state['deadlineUnixS']
        if wall < self.start or wall >= wall_deadline or self.clock.monotonic() >= deadline:
            raise Refusal('Frozen deadline reached or clock reversed')

    def permit(self, name, pid):
        self.guard()
        r = self.state['reservations'][name]
        if r['workerPid'] != pid:
            raise Refusal('Reservation must bind the unique worker before operation')
        return {'reservationId': r['reservationId'], 'reservedSteps': r['reservedSteps'],
            'deadlineUnixS': self.state['nativeDeadlineUnixS'],
            'deadlineMonotonic': self.native_deadline,
            'remainingWallSeconds': max(0, min(self.native_deadline - self.clock.monotonic(),
                self.state['nativeDeadlineUnixS'] - self.clock.wall())), 'workerPid': pid}

    def mark_operation_in_flight(self):
        # A short write or a lost response cannot justify an exact zero count.
        # Burn both complete reserves before the first request byte can leave.
        if self.state['operationMayHaveStarted']:
            raise Refusal('The single operation cannot be sent twice')
        self.state.update(operationMayHaveStarted=True, actualNativeStepsKnown=None,
                          actualNativeStepsLowerBound=0, actualNativeStepsUpperBound=TOTAL)
        for reservation in self.state['reservations'].values():
            reservation.update(actualNativeStepsKnown=None, actualNativeStepsLowerBound=0,
                               actualNativeStepsUpperBound=reservation['reservedSteps'])

    def close(self, reason):
        self.state['status'] = ('closed-success' if reason is None else
            'closed-unknown' if self.state['actualNativeStepsKnown'] is None else 'closed-refused')
        self.state['reason'] = reason
        self.state['elapsedWallSeconds'] = self.clock.monotonic() - self.start_mono
        self.save('close-without-refund-or-renewal')


def accounting_bounds(accounting, actual, maximum):
    """A completed receipt bounds an interrupted call by completed..admitted."""
    if accounting is None:
        if actual != 0:
            raise Refusal('An inactive integration gate must report exactly zero')
        return (0, 0)
    exact(accounting, ('admittedNativeSteps', 'completedNativeSteps',
                      'actualNativeStepsExact', 'abandonedNativeSlots'))
    admitted = bounded(accounting['admittedNativeSteps'], maximum)
    completed = bounded(accounting['completedNativeSteps'], maximum)
    abandoned = bounded(accounting['abandonedNativeSlots'], maximum)
    is_exact = accounting['actualNativeStepsExact']
    if (type(is_exact) is not bool or not completed <= admitted <= completed + 1
            or abandoned + admitted > maximum
            or is_exact != (completed == admitted)
            or actual != (completed if is_exact else None)):
        raise Refusal('Invalid completed/admitted native accounting interval')
    return completed, admitted


def set_accounting(journal, name, actual, bounds):
    journal.state['reservations'][name].update(actualNativeStepsKnown=actual,
        actualNativeStepsLowerBound=bounds[0], actualNativeStepsUpperBound=bounds[1])


def update_total(journal):
    values = list(journal.state['reservations'].values())
    known = [v['actualNativeStepsKnown'] for v in values]
    journal.state.update(actualNativeStepsKnown=(sum(known) if None not in known else None),
        actualNativeStepsLowerBound=sum(v['actualNativeStepsLowerBound'] for v in values),
        actualNativeStepsUpperBound=sum(v['actualNativeStepsUpperBound'] for v in values))


def archive_receipt(response, message, prepared, journal):
    exact(response, RESULT_FIELDS)
    if (type(response['schema']) is not int or response['schema'] != 1
            or response['event'] != 'result' or response['op'] != 'contact-diagnostic'
            or type(response['pid']) is not int or response['pid'] != message['technicalPermit']['workerPid']
            or response['operationId'] != message['operationId']
            or response['evidencePath'] != message['outputPath']):
        raise Refusal('Unexpected operation, PID, operation identity or evidence destination')
    for name, amount in (('', TOTAL), ('technical', TECHNICAL), ('history', HISTORY)):
        key = name + 'ReservedNativeSteps' if name else 'reservedNativeSteps'
        if type(response[key]) is not int or response[key] != amount:
            raise Refusal('Worker changed the frozen reservation')
    for name in ('technical', 'history'):
        if response[name + 'ReservationId'] != message[name + 'Permit']['reservationId']:
            raise Refusal('Receipt is not tied to both durable reservations')
    if (response['prefixExclusions'] != PREFIX_EXCLUSIONS or response['noRefund'] is not True
            or response['automaticRenewalAllowed'] is not False
            or type(response['countsReconciled']) is not bool
            or (response['refusal'] is not None and not isinstance(response['refusal'], str))):
        raise Refusal('Worker changed parity exclusions, accounting or nonrenewal contract')
    technical = bounded(response['technicalActualNativeSteps'], TECHNICAL, nullable=True)
    history = bounded(response['historyActualNativeSteps'], HISTORY, nullable=True)
    actual = bounded(response['actualNativeSteps'], TOTAL, nullable=True)
    expected = None if technical is None or history is None else technical + history
    if actual != expected:
        raise Refusal('Total counter must equal both known counters, otherwise remain unknown')
    exact(response['outcome'], GATES)
    if any(type(response['outcome'][k]) is not bool for k in GATES):
        raise Refusal('Outcome gates must be explicit booleans')
    path = common.absolute_file(message['outputPath'])
    evidence = read_json(path)
    exact(evidence, (*RESULT_FIELDS, 'audit'))
    for key in RESULT_FIELDS:
        if canonical(evidence[key]) != canonical(response[key]):
            raise Refusal('Native evidence disagrees with the result receipt: ' + key)
    audit = evidence['audit']
    if not isinstance(audit, dict):
        raise Refusal('Missing native audit object')
    counters = {key: bounded(audit.get(key), maximum) for key, maximum in (
        ('realHistoryNativeSteps', REAL_STEPS), ('forecastCalls', PREVIEWS),
        ('forecastRequestedSteps', PREVIEWS * HORIZON), ('forecastActualSteps', PREVIEWS * HORIZON),
        ('technicalPrefixNativeSteps', 36), ('technicalParityNativeSteps', 16))}
    intervals = {}
    for name, count, maximum in (('technical', technical, TECHNICAL), ('history', history, HISTORY)):
        intervals[name] = accounting_bounds(response[name + 'Accounting'], count, maximum)
    reconciled = (intervals['technical'][0] == counters['technicalPrefixNativeSteps'] + counters['technicalParityNativeSteps']
                  and intervals['history'][0] == counters['realHistoryNativeSteps'] + counters['forecastActualSteps'])
    if response['countsReconciled'] != reconciled:
        raise Refusal('Native accounting reconciliation flag contradicts its breakdown')
    calls, requested, forecast = (counters[k] for k in ('forecastCalls', 'forecastRequestedSteps', 'forecastActualSteps'))
    if calls is not None and requested is not None and requested != calls * HORIZON:
        raise Refusal('Each forecast must reserve exactly 480 steps')
    if forecast is not None and requested is not None and forecast > requested:
        raise Refusal('Forecast steps exceeded the requested reserve')
    set_accounting(journal, 'technical', technical, intervals['technical'])
    set_accounting(journal, 'history', history, intervals['history'])
    update_total(journal)
    journal.state.update(outcome=response['outcome'], accounting=counters,
        evidence={'path': str(path), 'sha256': sha(path)})
    # A well-formed failed/partial receipt is preserved before any gate refuses it.
    journal.save('archive-bounded-receipt-before-outcome-gates')
    if (technical != TECHNICAL or not all(response['outcome'].values()) or actual is None
            or not reconciled or response['refusal'] is not None):
        raise Refusal('Partial, unknown or refused native outcome; no continuation or retry')
    if counters['realHistoryNativeSteps'] != 36 or counters['forecastCalls'] != 1:
        raise Refusal('Completed diagnostic must contain only original36 prefix and one forecast')
    if (any(audit.get(k) is not False for k in ('historyCompleted', 'localSuccess', 'originalTerminalPassed'))
            or audit.get('diagnosticCompleted') is not True):
        raise Refusal('Diagnostic cannot claim a real transfer, local success or recovery')
    trace = common.absolute_file(message['tracePath'])
    if not trace.stat().st_size:
        raise Refusal('Successful history needs a nonempty native trace')
    journal.state['trace'] = {'path': str(trace), 'sha256': sha(trace)}


def recover_native_checkpoints(journal):
    """After the child is killed/reaped, retain the last durable native bounds.

    Missing or malformed journals never turn uncertainty into zero. A partial
    final record conservatively leaves one potentially in-flight native call.
    This reads only this run's journals, never a historical ledger.
    """
    if not journal.state['operationMayHaveStarted']:
        return
    recovered, problems = {}, {}
    for name, maximum in (('technical', TECHNICAL), ('history', HISTORY)):
        reservation = journal.state['reservations'][name]
        if reservation['actualNativeStepsKnown'] is not None:
            continue
        path = journal.output / f'native-result.json.{name}.accounting.jsonl'
        try:
            path = common.absolute_file(str(path))
            if path.stat().st_size > 100 * 1024 * 1024:
                raise Refusal('Native checkpoint journal exceeds the frozen read cap')
            previous, partial, records = None, False, 0
            with path.open('rb') as stream:
                for raw in stream:
                    if not raw.endswith(b'\n'):
                        partial = True
                        break
                    row = common.decode_json(raw)
                    exact(row, ('event', 'reservationId', 'admittedNativeSteps',
                        'completedNativeSteps', 'pendingNativeSlots', 'abandonedNativeSlots',
                        'actualNativeStepsExact', 'nextNativeClock'))
                    if row['reservationId'] != reservation['reservationId']:
                        raise Refusal('Native checkpoint reservation mismatch')
                    admitted = bounded(row['admittedNativeSteps'], maximum)
                    completed = bounded(row['completedNativeSteps'], maximum)
                    abandoned = bounded(row['abandonedNativeSlots'], maximum)
                    bounded(row['pendingNativeSlots'], 4)
                    bounded(row['nextNativeClock'], REAL_STEPS + HORIZON)
                    if (type(row['actualNativeStepsExact']) is not bool
                            or not completed <= admitted <= completed + 1
                            or row['actualNativeStepsExact'] != (completed == admitted)
                            or admitted + abandoned > maximum):
                        raise Refusal('Native checkpoint counters are inconsistent')
                    if previous is None:
                        valid = row['event'] == 'permit-activated' and admitted == completed == abandoned == 0
                    elif row['event'] == 'before-integration':
                        valid = (admitted == previous['admittedNativeSteps'] + 1
                                 and completed == previous['completedNativeSteps']
                                 and admitted == completed + 1
                                 and abandoned == previous['abandonedNativeSlots'])
                    elif row['event'] == 'after-integration':
                        valid = (admitted == previous['admittedNativeSteps']
                                 and completed == previous['completedNativeSteps'] + 1
                                 and completed == admitted
                                 and row['nextNativeClock'] == previous['nextNativeClock'] + 1
                                 and abandoned == previous['abandonedNativeSlots'])
                    elif row['event'] == 'abandon-unused-tick-slots-without-refund':
                        valid = (admitted == previous['admittedNativeSteps'] == completed == previous['completedNativeSteps']
                                 and previous['abandonedNativeSlots'] <= abandoned <= previous['abandonedNativeSlots'] + 4
                                 and row['pendingNativeSlots'] == 0)
                    else:
                        valid = False
                    if not valid:
                        raise Refusal('Native checkpoint event sequence is inconsistent')
                    previous, records = row, records + 1
            if previous is None:
                raise Refusal('Native checkpoint journal has no complete activation')
            lower = max(reservation['actualNativeStepsLowerBound'], previous['completedNativeSteps'])
            upper = min(reservation['actualNativeStepsUpperBound'], maximum,
                        previous['admittedNativeSteps'] + int(partial))
            if lower > upper:
                raise Refusal('Native checkpoint bounds contradict the receipt')
            set_accounting(journal, name, lower if lower == upper else None, (lower, upper))
            recovered[name] = {'path': str(path), 'sha256': sha(path), 'completeRecords': records,
                               'partialFinalRecord': partial, 'lower': lower, 'upper': upper}
        except (OSError, ValueError, Refusal) as exc:
            problems[name] = str(exc)
    update_total(journal)
    journal.state['nativeCheckpointRecovery'] = {'recovered': recovered, 'unresolved': problems}
    journal.save('recover-durable-native-checkpoints-after-worker-stop')


@contextmanager
def hard_deadline(deadline, worker_getter):
    # Retain the separately reviewed signal/PID cleanup; correct its old label.
    try:
        with common.hard_deadline(deadline, worker_getter):
            yield
    except Refusal as exc:
        raise Refusal(str(exc).replace('Hard120s deadline', 'Frozen transfer deadline')) from exc


def arm(deadline):
    signal.setitimer(signal.ITIMER_REAL, max(0.000001, deadline - time.monotonic()))


def run(manifest_path, authorization_path=None, *, execute=False, batch_root=BATCH_ROOT,
        clock=None, worker_factory=NativeWorker, inspector=common.inspect_process,
        alarm_factory=hard_deadline, arm_deadline=arm):
    prepared = preflight(manifest_path, authorization_path, batch_root=batch_root,
                         require_authorization=execute)
    if not execute:
        return {'status': 'static-preflight-passed', 'executeRequested': False,
                'nativeSteps': 0, 'manifestSha256': prepared['manifestSha256'],
                'frozenFilesVerified': len(prepared['records']),
                'historicalProtectedFilesVerified': len(prepared['protected']),
                'authorizationVerified': prepared['authorization'] is not None,
                'distinctAuthorizationRequiredBeforeExecution': True,
                'inspectionGrantsNativePermission': False}
    clock = clock or Clock()
    journal = Journal(prepared, clock)
    worker, identity, error = None, None, None
    transcript = (prepared['output'] / 'protocol.ndjson').open('xb')
    def record(direction, value):
        transcript.write(canonical({'direction': direction, 'utc': utc(clock.wall()), 'message': value}) + b'\n')
        transcript.flush()
        os.fsync(transcript.fileno())
    def guard():
        journal.guard()
        if worker is not None:
            state = inspector(worker.pid)
            if (state['pid'] != worker.pid or state['threads'] != 1 or state['children']
                    or state['binaryPath'] != str(prepared['binary'])
                    or state['binarySha256'] != prepared['binarySha256']
                    or (identity is not None and state != identity)):
                raise Refusal('Single worker PID/thread/child/binary identity contract failed')
    try:
        with alarm_factory(journal.deadline, lambda: worker):
            arm_deadline(journal.startup_deadline)
            verify_files(prepared)
            journal.guard()
            worker = worker_factory(prepared['binary'], prepared['output'])
            ready = worker.receive(journal.startup_deadline, guard)
            record('worker-to-parent', ready)
            exact(ready, ('event', 'pid', 'protocol'))
            if (ready['event'] != 'ready' or ready['protocol'] != 'transfer-run-v1'
                    or type(ready['pid']) is not int or ready['pid'] != worker.pid):
                raise Refusal('Ready PID does not identify the one spawned worker')
            if (clock.monotonic() >= journal.startup_deadline
                    or clock.wall() >= journal.start + STARTUP_SECONDS):
                raise Refusal('Startup exceeded 30 seconds within the global clock')
            guard()
            identity = inspector(worker.pid)
            journal.state['worker'] = identity
            for reservation in journal.state['reservations'].values():
                reservation['workerPid'] = worker.pid
            journal.save('bind-both-reservations-to-single-worker-before-operation')
            arm_deadline(journal.native_deadline)
            verify_files(prepared)
            guard()
            message = {'op': 'contact-diagnostic', 'operationId': journal.state['operationId'],
                'technicalPermit': journal.permit('technical', worker.pid),
                'historyPermit': journal.permit('history', worker.pid),
                'inputs': prepared['manifest']['inputs'],
                'outputPath': str(prepared['output'] / 'native-result.json'),
                'tracePath': str(prepared['output'] / 'native-trace.jsonl')}
            record('parent-to-worker', message)
            journal.mark_operation_in_flight()
            journal.save('record-single-transfer-operation-durably-before-send', message=message)
            worker.send(message, journal.native_deadline, guard)
            response = worker.receive(journal.native_deadline, guard)
            record('worker-to-parent', response)
            archive_receipt(response, message, prepared, journal)
            verify_files(prepared)
            guard()
            # Frozen worker promises no integration after result and waits EOF.
            arm_deadline(journal.deadline)
            if worker.finish(journal.deadline) != 0:
                raise Refusal('Worker exit was not successful')
            journal.guard(native=False)
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        if worker is not None:
            try:
                worker.kill()
            except BaseException as cleanup:
                error += '; kill failed: ' + str(cleanup)
    finally:
        if worker is not None:
            try:
                worker.dispose()
            except BaseException as exc:
                error = (error + '; ' if error else '') + 'Cleanup failed: ' + str(exc)
        try:
            recover_native_checkpoints(journal)
        except BaseException as exc:
            error = (error + '; ' if error else '') + 'Checkpoint recovery failed: ' + str(exc)
        try:
            verify_files(prepared)
            journal.state['terminalFrozenFilesVerified'] = True
        except BaseException as exc:
            journal.state['terminalFrozenFilesVerified'] = False
            error = (error + '; ' if error else '') + 'Frozen-file audit failed: ' + str(exc)
        try:
            journal.guard(native=False)
        except Refusal as exc:
            error = (error + '; ' if error else '') + str(exc)
        journal.close(error)
        transcript.close()
        journal.events.close()
    result = {'status': journal.state['status'], 'error': error,
        'qualification24Cases': False, 'physicalFallToCrouchProved': False, 'realTransferAuthorized': False,
        'budget': journal.state}
    write_new(prepared['output'] / 'run-result.json', result)
    if error:
        raise Refusal(error)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--authorization', help='Required only with --execute; inspection needs no approval')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        print(json.dumps(run(args.manifest, args.authorization, execute=args.execute), allow_nan=False))
    except (Refusal, OSError, ValueError) as exc:
        parser.exit(1, f'Refused: {exc}\n')


if __name__ == '__main__':
    main()
