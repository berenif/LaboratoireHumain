#!/usr/bin/env python3
"""Single authorized 44-step native parity batch; default is read-only preflight.

CLI: --manifest FILE [--execute]. Build the separate numeric_parity_worker first.
The manifest must use SCHEMA, AUTHORIZATION_ID, and these exact fields:
  schema, authorizationId, outputPath, binaryPath, inputs, files, prerequisites.
inputs = {fixturePath, programPath, tasksPath}; all paths are absolute.
files = [{path, sha256, role}], with roles binary/code/fixture/program/tasks/model/
profile/criteria/validation/preparation. Required prerequisites are listed below.
The preparation role contains the parent's preparation.json, including six old
register hashes and the originalReleaseWorker hash. Neither is ever modified.
The runner itself must be a frozen code file. Unknown fields/roles are refused.

One immutable authorization claim at BATCH_ROOT prevents any second execution,
even with a different output directory. A new output/physical-ledger.json charges
all 44 before subprocess creation. Crash, refusal, timeout and partial results
never refund credit; no resume, reload, retry, planning, or budget extension API.
The 120 seconds include subprocess startup, IO, verification and normal exit.
SIGALRM kills the native process group at the same hard deadline; no grace for
physics after expiry. A separate native worker must enforce each step too.

NDJSON: ready {event:'ready',pid}; parent sends exactly one run:
{op:'run',permit:{reservedSteps:44,deadlineUnixS,deadlineMonotonic,
remainingWallSeconds,workerPid},inputs,outputPath,tracePath}.
Worker waits for stdin EOF after result, never integrates after the result:
{event:'result',op:'numeric-parity',actualNativeSteps,outcome,evidencePath}.
outcome has exactly integrity_ok,prefix_matches_archive,parity_verified,
diagnostic_present (booleans). Evidence JSON repeats actualNativeSteps/outcome.
Success requires 44 and every gate true. Prefix mismatch requires <=12 actual.
Valid partial receipts are preserved before failure; malformed receipts leave
actual unknown. Native 'real' and 'copy' each count toward the same 44.

Tests inject inert workers/clocks/inspectors; this module never runs on import.
"""

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import threading
import time

REPO = Path(__file__).resolve().parents[1]
BATCH_ROOT = REPO / 'docs/evidence/coordination-native-parity-2026-10-13'
SCHEMA = 'laboratoirehumain.coordination-numeric-parity-manifest.v1'
AUTHORIZATION_ID = 'coordination-numeric-parity-2026-10-13T06:44Z'
STEPS, WALL_SECONDS, MAX_LINE = 44, 120, 1024 * 1024
PROC = Path('/proc')
GATES = ('integrity_ok', 'prefix_matches_archive', 'parity_verified', 'diagnostic_present')
PREREQUISITES = ('static_tests_passed', 'compiled_release_before_run',
    'unchanged_physical_contract', 'archived_prefix_12_available',
    'certified_commands_all_scales_available', 'single_native_process_thread',
    'separate_authorization_44_steps_120_seconds')
ROLES = {'binary', 'code', 'fixture', 'program', 'tasks', 'model', 'profile',
         'criteria', 'validation', 'preparation'}


class Refusal(RuntimeError):
    pass


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def exact(value, keys):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise Refusal('Missing/unknown fields; expected ' + ','.join(keys))


def decode_json(encoded):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise Refusal('Duplicate JSON key: ' + key)
            result[key] = value
        return result
    def constant(value):
        raise Refusal('Nonfinite JSON number: ' + value)
    return json.loads(encoded, object_pairs_hook=pairs, parse_constant=constant)


def read_json(path):
    return decode_json(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def fsync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_new(path, value):
    with Path(path).open('xb') as stream:
        stream.write(canonical(value) + b'\n')
        stream.flush()
        os.fsync(stream.fileno())
    fsync_directory(Path(path).parent)


def utc(value):
    return datetime.fromtimestamp(value, timezone.utc).isoformat()


class Clock:
    wall = staticmethod(time.time)
    monotonic = staticmethod(time.monotonic)


def absolute_file(value):
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise Refusal('Frozen file path must be absolute')
    path = Path(value)
    if path.is_symlink() or not path.is_file() or path.resolve() != path:
        raise Refusal('Frozen file must be canonical, regular and not a symlink: ' + value)
    return path


def checked_record(record):
    exact(record, ('path', 'sha256'))
    path = absolute_file(record['path'])
    if (not isinstance(record['sha256'], str) or len(record['sha256']) != 64 or
            sha(path) != record['sha256']):
        raise Refusal('Frozen file hash mismatch: ' + str(path))
    return {'path': str(path), 'sha256': record['sha256']}


def preflight(manifest_path, *, batch_root=BATCH_ROOT):
    path = absolute_file(str(Path(manifest_path).absolute()))
    manifest = read_json(path)
    exact(manifest, ('schema', 'authorizationId', 'outputPath', 'binaryPath', 'inputs', 'files', 'prerequisites'))
    if manifest['schema'] != SCHEMA or manifest['authorizationId'] != AUTHORIZATION_ID:
        raise Refusal('Wrong schema or distinct authorization identifier')
    exact(manifest['prerequisites'], PREREQUISITES)
    if any(manifest['prerequisites'][key] is not True for key in PREREQUISITES):
        raise Refusal('Required frozen prerequisite is not true')
    batch_root = Path(batch_root).resolve(strict=True)
    output = Path(manifest['outputPath'])
    if (not output.is_absolute() or output.parent != batch_root or output.resolve() != output
            or output.exists() or output.is_symlink()):
        raise Refusal('Output must be a NEW direct child of the distinct batch directory')
    if not isinstance(manifest['files'], list) or not manifest['files']:
        raise Refusal('Missing frozen files')
    records, roles = {}, {}
    for item in manifest['files']:
        exact(item, ('path', 'sha256', 'role'))
        if item['role'] not in ROLES or item['path'] in records:
            raise Refusal('Duplicate frozen path or unknown role')
        record = checked_record({k: item[k] for k in ('path', 'sha256')})
        records[record['path']] = record
        roles.setdefault(item['role'], []).append(record)
    if set(roles) != ROLES:
        raise Refusal('Missing frozen file roles: ' + ','.join(sorted(ROLES - set(roles))))
    binary = absolute_file(manifest['binaryPath'])
    if (len(roles['binary']) != 1 or roles['binary'][0]['path'] != str(binary)
            or not os.access(binary, os.X_OK)):
        raise Refusal('Binary must be the single frozen executable')
    if str(Path(__file__).resolve()) not in {r['path'] for r in roles['code']}:
        raise Refusal('Runner source must be frozen as code')
    exact(manifest['inputs'], ('fixturePath', 'programPath', 'tasksPath'))
    for key, role in (('fixturePath', 'fixture'), ('programPath', 'program'), ('tasksPath', 'tasks')):
        input_path = absolute_file(manifest['inputs'][key])
        if str(input_path) not in {r['path'] for r in roles[role]} or not isinstance(read_json(input_path), dict):
            raise Refusal('Worker input must be frozen JSON in the correct role: ' + key)
    if len(roles['preparation']) != 1:
        raise Refusal('Exactly one frozen preparation receipt required')
    preparation = read_json(roles['preparation'][0]['path'])
    if (preparation.get('approvedNativeSubsteps') != STEPS or
            preparation.get('approvedNativeWallSeconds') != WALL_SECONDS or
            preparation.get('nativeExecutionStarted') is not False or
            len(preparation.get('oldRegisters', [])) != 6):
        raise Refusal('Preparation must name the distinct44/120 authorization and six old registers')
    protected = []
    for item in preparation['oldRegisters'] + [preparation['originalReleaseWorker']]:
        record = dict(item)
        record['path'] = str((REPO / item['path']).resolve())
        protected.append(checked_record(record))
    if len({r['path'] for r in protected}) != 7 or str(binary) in {r['path'] for r in protected}:
        raise Refusal('Old registers and binary must be distinct and preserved')
    environment_preflight()  # Read-only; fail before reserving/claiming the batch.
    return {'manifest': manifest, 'manifestPath': str(path), 'manifestSha256': sha(path),
            'records': list(records.values()), 'protected': protected,
            'binary': binary, 'output': output, 'batchRoot': batch_root}


def verify_files(prepared):
    for record in prepared['records'] + prepared['protected'] + [
            {'path': prepared['manifestPath'], 'sha256': prepared['manifestSha256']}]:
        checked_record(record)


class Journal:
    def __init__(self, prepared, clock):
        self.output, self.clock = prepared['output'], clock
        self.output.mkdir()  # Existing output is never reused.
        fsync_directory(self.output.parent)
        self.start_wall, self.start_mono = clock.wall(), clock.monotonic()
        self.deadline = self.start_mono + WALL_SECONDS
        self.path = self.output / 'physical-ledger.json'
        self.state = {'schema': 'laboratoirehumain.numeric-parity-ledger.v1',
            'authorizationId': AUTHORIZATION_ID, 'status': 'reserved',
            'reservedNativeSteps': STEPS, 'actualNativeStepsKnown': None,
            'refundNativeSteps': 0, 'maximumWallSeconds': WALL_SECONDS,
            'startedUtc': utc(self.start_wall), 'startedMonotonic': self.start_mono,
            'deadlineUnixS': self.start_wall + WALL_SECONDS, 'deadlineMonotonic': self.deadline,
            'manifestSha256': prepared['manifestSha256'], 'worker': None, 'events': []}
        # Exclusive immutable claim blocks a second invocation even if output changes.
        write_new(prepared['batchRoot'] / 'authorization-claim.json',
            {'authorizationId': AUTHORIZATION_ID, 'outputPath': str(self.output),
             'reservedNativeSteps': STEPS, 'maximumWallSeconds': WALL_SECONDS,
             'startedUtc': self.state['startedUtc'], 'deadlineUnixS': self.state['deadlineUnixS'],
             'manifestSha256': prepared['manifestSha256']})
        self.save('reserve-entire-batch-before-spawn')

    def save(self, event, **fields):
        self.state['events'].append({'event': event, 'utc': utc(self.clock.wall()), **fields})
        payload = dict(self.state)
        envelope = {'payload': payload, 'sha256': hashlib.sha256(canonical(payload)).hexdigest()}
        temporary = self.output / 'physical-ledger.next.json'
        write_new(temporary, envelope)
        os.replace(temporary, self.path)
        fsync_directory(self.output)

    def guard(self):
        if (self.clock.monotonic() >= self.deadline or
                self.clock.wall() >= self.state['deadlineUnixS'] or
                self.clock.wall() < self.start_wall):
            raise Refusal('Hard120s wall/monotonic deadline reached or clock reversed')

    def close(self, reason=None):
        self.state['status'] = 'closed-failure' if reason else 'closed-success'
        self.state['reason'] = reason
        self.state['elapsedWallSeconds'] = self.clock.monotonic() - self.start_mono
        self.save('close-without-refund')


def inspect_process(pid):
    status = (PROC / str(pid) / 'status').read_text().splitlines()
    threads = int(next(line.split(':', 1)[1] for line in status if line.startswith('Threads:')))
    children = ' '.join(map(str, sorted(child_pids(pid))))
    stat = process_stat(pid)
    executable = PROC / str(pid) / 'exe'
    binary = Path(os.readlink(executable))
    return {'pid': pid, 'threads': threads, 'children': children, 'startTicks': stat[19],
            'binaryPath': str(binary), 'binarySha256': sha(executable)}


def process_stat(pid):
    # comm may contain spaces and ')'; PPid is field 4, starttime field 22.
    raw = (PROC / str(pid) / 'stat').read_text()
    fields = raw.rsplit(')', 1)[1].split()
    if int(raw.split('(', 1)[0]) != pid or len(fields) < 20:
        raise Refusal('Invalid process identity/stat fields')
    return fields


def child_pids(parent_pid=None):
    # Standard Linux PPid metadata does not require CONFIG_CHECKPOINT_RESTORE.
    # Only disappearing PIDs are benign; denied access is never treated as empty.
    parent_pid = os.getpid() if parent_pid is None else parent_pid
    children, parent_seen = set(), False
    for entry in PROC.iterdir():
        if not entry.name.isdecimal():
            continue
        pid = int(entry.name)
        try:
            stat = process_stat(pid)
        except FileNotFoundError:
            continue
        parent_seen |= pid == parent_pid
        if int(stat[1]) == parent_pid:
            children.add(pid)
    if not parent_seen:
        raise Refusal('Process census cannot observe its required parent PID')
    return children


def environment_preflight():
    if threading.current_thread() is not threading.main_thread():
        raise Refusal('Signal supervision requires the main thread')
    if signal.getitimer(signal.ITIMER_REAL) != (0.0, 0.0):
        raise Refusal('Cannot replace an existing process alarm')
    inspect_process(os.getpid())
    os.getpgid(os.getpid())


def kill_pid_group(pid):
    # A constructor interrupted just after fork may not have completed setsid.
    # Kill both its new process group (when present) and its direct child PID.
    for kill in (lambda: os.killpg(pid, signal.SIGKILL), lambda: os.kill(pid, signal.SIGKILL)):
        try:
            kill()
        except ProcessLookupError:
            pass


class NativeWorker:
    def __init__(self, binary, output):
        self.stderr = (output / 'worker.stderr.log').open('xb')
        environment = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                           MKL_NUM_THREADS='1', RAYON_NUM_THREADS='1')
        try:
            self.process = subprocess.Popen([str(binary)], cwd=REPO, env=environment,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr,
                bufsize=0, start_new_session=True)
        except BaseException:
            self.stderr.close()
            raise
        self.pid = self.process.pid
        os.set_blocking(self.process.stdin.fileno(), False)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.buffer = bytearray()

    def send(self, value, deadline, guard):
        encoded = canonical(value) + b'\n'
        if len(encoded) > MAX_LINE:
            raise Refusal('Parent protocol line exceeds cap')
        offset = 0
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdin, selectors.EVENT_WRITE)
            while offset < len(encoded):
                guard()
                if time.monotonic() >= deadline:
                    raise Refusal('Permit send deadline reached')
                if not selector.select(min(0.05, max(0, deadline - time.monotonic()))):
                    continue
                try:
                    written = os.write(self.process.stdin.fileno(), encoded[offset:])
                except BlockingIOError:
                    continue
                if written <= 0:
                    raise Refusal('Worker stdin closed')
                offset += written

    def receive(self, deadline, guard):
        while True:
            guard()
            if time.monotonic() >= deadline:
                raise Refusal('Worker response deadline reached')
            if b'\n' in self.buffer:
                line, _, remainder = self.buffer.partition(b'\n')
                self.buffer = bytearray(remainder)
                if len(line) > MAX_LINE:
                    raise Refusal('Worker protocol line exceeds cap')
                return decode_json(line)
            if len(self.buffer) > MAX_LINE:
                raise Refusal('Worker protocol line exceeds cap')
            if self.selector.select(min(0.05, max(0, deadline - time.monotonic()))):
                data = os.read(self.process.stdout.fileno(), 65536)
                if not data:
                    raise Refusal('Worker exited without a complete result')
                self.buffer.extend(data)

    def kill(self):
        kill_pid_group(self.pid)

    def finish(self, deadline):
        self.process.stdin.close()
        try:
            return self.process.wait(timeout=max(0.001, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            self.kill()
            raise Refusal('Worker did not exit before the same hard deadline')

    def dispose(self):
        self.kill()
        # Reaping cannot authorize more integration: SIGKILL was already sent.
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass
        self.selector.close()
        for stream in (self.process.stdin, self.process.stdout, self.stderr):
            try:
                stream.close()
            except OSError:
                pass


@contextmanager
def hard_deadline(deadline, worker_getter):
    signals = (signal.SIGALRM, signal.SIGINT, signal.SIGTERM)
    previous = {sig: signal.getsignal(sig) for sig in signals}
    if signal.getitimer(signal.ITIMER_REAL) != (0.0, 0.0):
        raise Refusal('Cannot replace an existing process alarm')
    original_children = child_pids()
    def kill_new_children():
        for pid in child_pids() - original_children:
            kill_pid_group(pid)
    def interrupt(signum, _frame):
        reason = 'Hard120s deadline' if signum == signal.SIGALRM else f'Cancelled by {signal.Signals(signum).name}'
        cleanup_errors = []
        worker = worker_getter()
        if worker is not None:
            try:
                worker.kill()
            except BaseException as exc:
                cleanup_errors.append(str(exc))
        # Cover SIGALRM during Popen/NativeWorker construction, before the
        # returned worker is assigned. Only this context's new direct children
        # are eligible; pre-existing unrelated children are never signalled.
        try:
            kill_new_children()
        except BaseException as exc:
            cleanup_errors.append(str(exc))
        if cleanup_errors:
            raise Refusal(reason + '; process cleanup failed: ' + '; '.join(cleanup_errors))
        raise Refusal(reason + ': native process group killed')
    failure = None
    try:
        for sig in signals:
            signal.signal(sig, interrupt)
        signal.setitimer(signal.ITIMER_REAL, max(0.000001, deadline - time.monotonic()))
        yield
    except BaseException as exc:
        failure = exc
        raise
    finally:
        # Also covers any constructor exception after spawning, not only alarm.
        try:
            kill_new_children()
        except BaseException as exc:
            if failure is not None:
                raise Refusal(f'{type(failure).__name__}: {failure}; process cleanup failed: {exc}') from failure
            raise
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def run(manifest_path, *, execute=False, batch_root=BATCH_ROOT, clock=None,
        worker_factory=NativeWorker, inspector=inspect_process, alarm_factory=hard_deadline):
    prepared = preflight(manifest_path, batch_root=batch_root)
    if not execute:
        return {'status': 'static-preflight-passed', 'physicalIntegrations': 0,
                'manifestSha256': prepared['manifestSha256'], 'executeRequested': False}
    clock = clock or Clock()
    journal = Journal(prepared, clock)
    worker, process_identity, error = None, None, None
    transcript = (prepared['output'] / 'protocol.ndjson').open('xb')
    def record(direction, value):
        transcript.write(canonical({'direction': direction, 'utc': utc(clock.wall()), 'message': value}) + b'\n')
        transcript.flush()
        os.fsync(transcript.fileno())
    def guard():
        journal.guard()
        if worker is not None:
            current = inspector(worker.pid)
            if (current['pid'] != worker.pid or current['threads'] != 1 or current['children'] or
                    current['binaryPath'] != str(prepared['binary']) or
                    current['binarySha256'] != sha(prepared['binary'])):
                raise Refusal('Native process identity, thread, child or binary contract violated')
            if process_identity is not None and current != process_identity:
                raise Refusal('Native worker identity changed')
    try:
        with alarm_factory(journal.deadline, lambda: worker):
            verify_files(prepared)
            journal.guard()
            worker = worker_factory(prepared['binary'], prepared['output'])
            ready = worker.receive(journal.deadline, journal.guard)
            record('worker-to-parent', ready)
            exact(ready, ('event', 'pid'))
            if ready['event'] != 'ready' or type(ready['pid']) is not int or ready['pid'] != worker.pid:
                raise Refusal('Ready PID must equal the single spawned PID')
            guard()
            process_identity = inspector(worker.pid)
            journal.state['worker'] = process_identity
            journal.save('bind-single-native-worker')
            verify_files(prepared)
            guard()
            message = {'op': 'run', 'permit': {'reservedSteps': STEPS,
                'deadlineUnixS': journal.state['deadlineUnixS'], 'deadlineMonotonic': journal.deadline,
                'remainingWallSeconds': journal.deadline - clock.monotonic(), 'workerPid': worker.pid},
                'inputs': prepared['manifest']['inputs'],
                'outputPath': str(prepared['output'] / 'native-result.json'),
                'tracePath': str(prepared['output'] / 'native-trace.jsonl')}
            record('parent-to-worker', message)
            journal.state['actualNativeStepsKnown'] = None
            journal.save('permit-durably-recorded-before-send')
            worker.send(message, journal.deadline, guard)
            response = worker.receive(journal.deadline, guard)
            record('worker-to-parent', response)
            exact(response, ('event', 'op', 'actualNativeSteps', 'outcome', 'evidencePath'))
            if response['event'] != 'result' or response['op'] != 'numeric-parity':
                raise Refusal('Unexpected operation: no additional operation is authorized')
            actual, outcome = response['actualNativeSteps'], response['outcome']
            if type(actual) is not int or not 0 <= actual <= STEPS:
                raise Refusal('Invalid actual native step count')
            exact(outcome, GATES)
            if any(type(outcome[key]) is not bool for key in GATES):
                raise Refusal('Diagnostic gates must be explicit booleans')
            if response['evidencePath'] != message['outputPath']:
                raise Refusal('Evidence must use the exact assigned output path')
            evidence = absolute_file(message['outputPath'])
            report = read_json(evidence)
            if report.get('actualNativeSteps') != actual or report.get('outcome') != outcome:
                raise Refusal('Evidence contradicts native result receipt')
            journal.state.update(actualNativeStepsKnown=actual, outcome=outcome,
                                 evidence={'path': str(evidence), 'sha256': sha(evidence)})
            journal.save('archive-bounded-native-receipt-without-refund')
            trace = absolute_file(message['tracePath'])
            journal.state['trace'] = {'path': str(trace), 'sha256': sha(trace)}
            if not outcome['prefix_matches_archive'] and actual > 12:
                raise Refusal('Worker integrated after a mismatching prefix')
            if actual != STEPS or not all(outcome.values()):
                raise Refusal('Partial batch or native diagnostic refused; no retry')
            verify_files(prepared)
            guard()
            if worker.finish(journal.deadline) != 0:
                raise Refusal('Worker exit was not successful')
            journal.guard()
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        if worker is not None:
            worker.kill()
    finally:
        if worker is not None:
            try:
                worker.dispose()
            except BaseException as exc:
                error = (error + '; ' if error else '') + f'Worker cleanup failed: {exc}'
        # Inspect frozen and historical files after stop, even for a partial
        # result, malformed message, crash, or deadline. Do not hide the first
        # failure or modify/repair a protected artifact.
        try:
            verify_files(prepared)
            journal.state['terminalFrozenFilesVerified'] = True
        except BaseException as exc:
            journal.state['terminalFrozenFilesVerified'] = False
            error = (error + '; ' if error else '') + f'Terminal frozen-file audit: {exc}'
        if error is None:
            try:
                journal.guard()
            except Refusal as exc:
                error = str(exc)
        journal.close(error)
        if error is None:
            # A slow durable close cannot be reported as within the120s cap.
            try:
                journal.guard()
            except Refusal as exc:
                error = str(exc)
                journal.close(error)
        transcript.close()
    result = {'status': journal.state['status'], 'error': error,
              'fullRecoveryQualified': False, 'fullHistoryAuthorized': False,
              'budget': journal.state}
    write_new(prepared['output'] / 'run-result.json', result)
    if error:
        raise Refusal(error)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        result = run(args.manifest, execute=args.execute)
        print(json.dumps(result, allow_nan=False))
    except (Refusal, OSError, ValueError) as exc:
        parser.exit(1, f'Refused: {exc}\n')


if __name__ == '__main__':
    main()
