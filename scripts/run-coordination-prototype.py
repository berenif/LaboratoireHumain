#!/usr/bin/env python3
"""One persistent native worker; explicit --execute is required for any physics.

Default: verify the frozen inputs and preparation ledger, write a static receipt
to a NEW output directory, and exit without spawning a worker or changing the
budget. --execute freezes the existing preparation allocation; it cannot create,
renew, reopen, or resume a budget. Complete preparation leases first.

Manifest: coordination-prototype-budget.py schema 1, plus optional workerInputs
{programPath, tasksPath}. Alternatively use --program and --tasks. Both input
paths and --binary must occur in the manifest's frozen files; --binary must be
in its binary role. The continuation prerequisite is static, not prior physical
success. Code and executable must already be compiled and frozen.

NDJSON worker protocol (one JSON object per line; no other stdout):
  worker -> {event:"ready",pid:<its OS PID>}
  parent -> {op:"verify",permit,inputs,outputPath,tracePath}
  worker -> {event:"result",op:"verify",actualNativeSteps:16,outcome,evidencePath}
  parent -> {op:"history",id:"A"|"B"|"B-repeat"|"B60",enabled:false|true,
             headingDegrees:0|60,permit,inputs,outputPath,tracePath}
  worker -> {event:"planning-request",historyStep:0|24|...|936,
             longSteps:[480,480,480,480],shortProbes:0}
  parent -> {op:"planning-permit",permit,outputPath}
  worker -> {event:"result",op:"planning",actualNativeSteps,outcome,evidencePath}
  parent -> {op:"continue-history",permit:<original history reservation, guarded>}
  worker -> {event:"result",op:"history",actualNativeSteps,outcome,evidencePath}

permit = {reservationId,reservedSteps,deadlineUnixS,deadlineMonotonic,
          remainingWallSeconds,workerPid}. Every integration, including copies,
must be counted by the worker and stop at the first step/deadline cap. The worker
MUST remain blocked until a permit arrives, including nested planning requests.
The verify operation is exactly 16 integrations: A copy4/actual4, B copy4/actual4.
The frozen history controller is A local, B coordination, then B-repeat and B60
only after preceding B local success. No changes, retries or additional routes.

All outcomes require true booleans integrity_ok, support_feasible,
predictions_usable, diagnostic_present. History also requires local_success
(boolean; false is allowed). Verify also requires true native_accounting_verified
and prediction_state_and_motor_semantics_verified. These are checked claims,
not conclusions independently proved by the Python runner; their full evidence
must be written to the exact outputPath supplied in that operation. The runner
adds evidence={path,sha256} to the durable budget completion and admits verify
evidence before permitting histories. Planning diagnostics are technical, not
the local heel/support success criterion. Unknown protocol fields are refused.

After final result, stdin EOF requests a normal worker exit. On any error,
parent sends {op:"abort",reason}, closes stdin, and terminates/kills a worker
which does not exit promptly. Reservations remain charged. No shell is used,
no compilation is started, and stdout/stderr plus permits are archived.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import selectors
import subprocess
import time


REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('coordination_prototype_budget',
    Path(__file__).with_name('coordination-prototype-budget.py'))
budgetlib = importlib.util.module_from_spec(spec)
spec.loader.exec_module(budgetlib)
COMMON_OUTCOME = ('integrity_ok', 'support_feasible', 'predictions_usable', 'diagnostic_present')
VERIFY_STEPS = 16
MAX_MESSAGE_BYTES = 1024 * 1024


class RunnerError(RuntimeError):
    pass


def exact_fields(value, required, optional=()):
    if not isinstance(value, dict) or set(value) - set(required) - set(optional) or set(required) - set(value):
        raise RunnerError('Missing or unknown protocol fields; expected ' + ', '.join(required))


def write_new_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


class NativeWorker:
    """Single subprocess; selector timeout also handles a silent/hung worker."""

    def __init__(self, binary, output):
        self.stderr = (output / 'worker.stderr.log').open('xb')
        environment = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                           MKL_NUM_THREADS='1', RAYON_NUM_THREADS='1')
        try:
            self.process = subprocess.Popen([str(binary)], cwd=REPO, env=environment,
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.stderr, bufsize=0)
        except BaseException:
            self.stderr.close()
            raise
        self.pid = self.process.pid
        os.set_blocking(self.process.stdin.fileno(), False)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.buffer = bytearray()

    def send(self, value):
        encoded = budgetlib.canonical(value) + b'\n'
        if len(encoded) > MAX_MESSAGE_BYTES:
            raise RunnerError('Parent message exceeds protocol size cap')
        limit = 0.25 if value.get('op') == 'abort' else 2.0
        deadline = min(time.monotonic() + limit,
                       value.get('permit', {}).get('deadlineMonotonic', float('inf')))
        offset = 0
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdin, selectors.EVENT_WRITE)
            while offset < len(encoded):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RunnerError('Worker stdin timed out at its bounded send deadline')
                if not selector.select(min(0.1, remaining)):
                    continue
                try:
                    written = os.write(self.process.stdin.fileno(), encoded[offset:])
                except BlockingIOError:
                    continue
                if not written:
                    raise RunnerError('Worker stdin closed during its permit')
                offset += written

    def receive(self, deadline_monotonic):
        while True:
            if time.monotonic() >= deadline_monotonic:
                raise RunnerError('Worker response timed out at the existing deadline')
            if b'\n' in self.buffer:
                line, _, remainder = self.buffer.partition(b'\n')
                self.buffer = bytearray(remainder)
                if len(line) > MAX_MESSAGE_BYTES:
                    raise RunnerError('Worker protocol line too large')
                try:
                    value = json.loads(line)
                except (ValueError, UnicodeDecodeError) as exc:
                    raise RunnerError('Worker stdout is not NDJSON') from exc
                if not isinstance(value, dict):
                    raise RunnerError('Worker protocol message must be an object')
                return value
            if len(self.buffer) > MAX_MESSAGE_BYTES:
                raise RunnerError('Worker protocol line too large')
            if not self.selector.select(min(1.0, max(0, deadline_monotonic - time.monotonic()))):
                continue
            data = os.read(self.process.stdout.fileno(), 65536)
            if not data:
                raise RunnerError('Worker exited without its required result')
            self.buffer.extend(data)

    def stop(self, reason=None):
        if reason is not None and self.process.poll() is None:
            try:
                self.send({'op': 'abort', 'reason': reason[:2000]})
            except (RunnerError, BrokenPipeError, OSError, ValueError):
                pass
        try:
            self.process.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            code = self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                code = self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                code = self.process.wait(timeout=2)
        self.selector.close()
        self.process.stdout.close()
        self.stderr.close()
        return code


class Session:
    def __init__(self, ledger, worker, output):
        self.ledger = ledger
        self.worker = worker
        self.output = output
        self.transcript = (output / 'protocol.ndjson').open('x')
        self.events = 0

    def record(self, direction, message):
        self.transcript.write(json.dumps({'sequence': self.events, 'direction': direction,
                                          'utc': budgetlib.utc(time.time()), 'message': message},
                                         allow_nan=False) + '\n')
        self.transcript.flush()
        os.fsync(self.transcript.fileno())
        self.events += 1

    def send(self, message):
        self.record('parent-to-worker', message)
        self.worker.send(message)

    def receive(self, deadline):
        message = self.worker.receive(deadline)
        self.record('worker-to-parent', message)
        return message

    def permit(self, reservation):
        guard = self.ledger.guard(reservation['id'])
        return {'reservationId': reservation['id'], 'reservedSteps': reservation['nativeSteps'],
                'deadlineUnixS': budgetlib.parse_utc(reservation['deadlineUtc']),
                'deadlineMonotonic': guard['deadlineMonotonic'],
                'remainingWallSeconds': guard['remainingWallSeconds'], 'workerPid': self.worker.pid}

    def result(self, message, operation, reservation, expected_output):
        exact_fields(message, ('event', 'op', 'actualNativeSteps', 'outcome', 'evidencePath'))
        if message['event'] != 'result' or message['op'] != operation:
            raise RunnerError('Unexpected result operation')
        actual = budgetlib.integer(message['actualNativeSteps'], 'actualNativeSteps', 0, reservation['nativeSteps'])
        outcome = message['outcome']
        extras = ('local_success',) if operation == 'history' else budgetlib.NATIVE_GATES if operation == 'verify' else ()
        exact_fields(outcome, COMMON_OUTCOME + extras, () if operation == 'history' else ('local_success',))
        for key in COMMON_OUTCOME + (budgetlib.NATIVE_GATES if operation == 'verify' else ()):
            if type(outcome[key]) is not bool:
                raise RunnerError('Native diagnostic must be an explicit boolean: ' + key)
        if 'local_success' in outcome and type(outcome['local_success']) is not bool:
            raise RunnerError('History local_success must be an explicit boolean')
        reported = Path(message['evidencePath'])
        if not reported.is_absolute():
            reported = self.output / reported
        if (expected_output.is_symlink() or reported.resolve(strict=True) != expected_output.absolute() or
                expected_output.parent.resolve() != self.output):
            raise RunnerError('Worker evidence path differs from its assigned outputPath')
        if not expected_output.is_file():
            raise RunnerError('Worker did not archive its diagnostic evidence')
        outcome = dict(outcome, evidence={'path': str(expected_output), 'sha256': budgetlib.sha(expected_output)})

        def refuse_with_receipt(reason):
            try:
                self.ledger.record_native_failure(reservation['id'], actual_native_steps=actual,
                                                   outcome=outcome, reason=reason)
            except budgetlib.BudgetError as exc:
                raise RunnerError(reason + '; failure receipt could not be recorded: ' + str(exc)) from exc
            raise RunnerError(reason)

        if operation == 'verify' and actual != VERIFY_STEPS:
            refuse_with_receipt('Verification must report exactly the 16 reserved native integrations')
        for key in COMMON_OUTCOME + (budgetlib.NATIVE_GATES if operation == 'verify' else ()):
            if not outcome[key]:
                refuse_with_receipt('Native diagnostic failed: ' + key)
        try:
            self.ledger.complete(reservation['id'], actual_native_steps=actual, outcome=outcome)
        except budgetlib.BudgetError as exc:
            refuse_with_receipt('Budget completion refused: ' + str(exc))
        return outcome

    def verify(self, inputs):
        reservation = self.ledger.reserve('verification', native_steps=VERIFY_STEPS, worker_pid=self.worker.pid)
        output = self.output / '000-native-verification.json'
        self.send({'op': 'verify', 'permit': self.permit(reservation), 'inputs': inputs,
                   'outputPath': str(output), 'tracePath': str(output.with_suffix('.states.jsonl'))})
        response = self.receive(reservation['deadlineMonotonic'])
        outcome = self.result(response, 'verify', reservation, output)
        self.ledger.admit_native_evidence({'schema': 1, 'gates': {k: outcome[k] for k in budgetlib.NATIVE_GATES},
            'verifications': [{'reservation_id': reservation['id'], **outcome['evidence']}]})

    def history(self, name, index, inputs):
        reservation = self.ledger.reserve('history', history=name, worker_pid=self.worker.pid)
        output = self.output / f'{index:03d}-{name}.json'
        self.send({'op': 'history', 'id': name, 'enabled': name != 'A', 'headingDegrees': reservation['headingDegrees'],
                   'permit': self.permit(reservation), 'inputs': inputs, 'outputPath': str(output),
                   'tracePath': str(output.with_suffix('.states.jsonl'))})
        planning_index = 0
        while True:
            response = self.receive(reservation['deadlineMonotonic'])
            if response.get('event') != 'planning-request':
                return self.result(response, 'history', reservation, output)
            exact_fields(response, ('event', 'historyStep', 'longSteps', 'shortProbes'), ('requiredReservedSteps',))
            if (name == 'A' or response['longSteps'] != [480, 480, 480, 480] or
                    type(response['shortProbes']) is not int or response['shortProbes'] != 0):
                raise RunnerError('Planning request differs from the frozen four-preview/no-probe controller')
            if ('requiredReservedSteps' in response and
                    (type(response['requiredReservedSteps']) is not int or response['requiredReservedSteps'] != 1920)):
                raise RunnerError('Planning requested a different reservation size')
            planning = self.ledger.reserve('planning', history_id=reservation['id'],
                                           history_step=response['historyStep'], long_steps=response['longSteps'],
                                           short_probes=response['shortProbes'])
            planning_output = self.output / f'{index:03d}-{name}-planning-{planning_index:03d}.json'
            self.send({'op': 'planning-permit', 'permit': self.permit(planning), 'outputPath': str(planning_output)})
            planned = self.receive(planning['deadlineMonotonic'])
            self.result(planned, 'planning', planning, planning_output)
            # Native worker is still required to enforce the original history
            # permit's deadline and remaining actual-history steps on its own.
            self.send({'op': 'continue-history', 'permit': self.permit(reservation)})
            planning_index += 1


def prepare(binary, manifest_path, budget_root, program=None, tasks=None, ledger=None):
    manifest_path = Path(manifest_path).resolve(strict=True)
    manifest = json.loads(manifest_path.read_text())
    exact_fields(manifest, ('schema', 'files', 'prerequisites'), ('workerInputs',))
    frozen = budgetlib.Budget._manifest(manifest, manifest_path.parent)
    binary = Path(binary).resolve(strict=True)
    if not os.access(binary, os.X_OK):
        raise RunnerError('Frozen worker binary is not executable')
    if not any(Path(item['path']) == binary and item['sha256'] == budgetlib.sha(binary)
               for item in frozen['files']['binary']):
        raise RunnerError('--binary is not the exact frozen binary')
    inputs = manifest.get('workerInputs', {})
    exact_fields(inputs, (), ('programPath', 'tasksPath'))
    resolved = {}
    frozen_paths = {record['path'] for records in frozen['files'].values() for record in records}
    for key, explicit in (('programPath', program), ('tasksPath', tasks)):
        value = explicit if explicit is not None else inputs.get(key)
        if not value:
            raise RunnerError('Missing frozen worker input: ' + key)
        path = Path(value)
        path = (Path.cwd() / path).resolve(strict=True) if explicit is not None else (manifest_path.parent / path).resolve(strict=True)
        if str(path) not in frozen_paths:
            raise RunnerError('Worker input is not among frozen files: ' + key)
        if explicit is not None and key in inputs and path != (manifest_path.parent / inputs[key]).resolve(strict=True):
            raise RunnerError('CLI input differs from frozen workerInputs: ' + key)
        if not isinstance(json.loads(path.read_text()), dict):
            raise RunnerError('Worker input must be a JSON object: ' + key)
        resolved[key] = str(path)
    ledger = ledger or budgetlib.Budget(budget_root)
    status = ledger.status()
    if status['status'] != 'preparing':
        raise RunnerError('Runner requires the existing preparing allocation; no restart or renewal')
    return binary, manifest, resolved, ledger, status


def run(binary, manifest_path, output, budget_root, *, execute=False, program=None, tasks=None,
        ledger=None, worker_factory=NativeWorker):
    binary, manifest, inputs, ledger, initial = prepare(binary, manifest_path, budget_root, program, tasks, ledger)
    output = Path(output).absolute()
    output.mkdir(mode=0o700, parents=False, exist_ok=False)
    output = output.resolve(strict=True)
    receipt = {'schema': 1, 'scope': 'static runner preflight; no worker started',
               'executeRequested': execute, 'physicalIntegrations': 0,
               'binary': str(binary), 'binarySha256': budgetlib.sha(binary),
               'manifestPath': str(Path(manifest_path).resolve(strict=True)),
               'manifestSha256': budgetlib.sha(manifest_path), 'inputs': inputs,
               'budgetId': initial['id'], 'budgetStatus': initial['status'],
               'budgetCounters': initial['counters']}
    write_new_json(output / 'static-preflight.json', receipt)
    if not execute:
        return receipt
    worker = None
    session = None
    outcomes = []
    error = None
    try:
        ledger.freeze(manifest, Path(manifest_path).resolve().parent)
        worker = worker_factory(binary, output)
        session = Session(ledger, worker, output)
        ready = session.receive(time.monotonic() + 30)
        exact_fields(ready, ('event', 'pid'))
        if ready['event'] != 'ready' or type(ready['pid']) is not int or ready['pid'] != worker.pid:
            raise RunnerError('Worker ready PID does not match its single native process')
        session.verify(inputs)
        for index, name in enumerate(budgetlib.HISTORY_ORDER, start=1):
            outcome = session.history(name, index, inputs)
            outcomes.append({'history': name, 'localSuccess': outcome['local_success'], 'evidence': outcome['evidence']})
            if name != 'A' and not outcome['local_success']:
                break
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
    finally:
        if worker is not None:
            try:
                code = worker.stop(error)
                if code != 0 and error is None:
                    error = 'Worker did not exit cleanly after its final result: ' + str(code)
            except (Exception, KeyboardInterrupt) as exc:
                error = (error + '; ' if error else '') + 'Worker shutdown failed: ' + str(exc)
        if session is not None:
            session.transcript.close()
    if error is not None:
        try:
            if not ledger.status()['status'].startswith('closed'):
                ledger.close('Runner stopped: ' + error)
        except (budgetlib.BudgetError, OSError, ValueError):
            pass  # A deadline/integrity refusal may already have closed admission.
    try:
        final_budget = ledger.status()
    except (budgetlib.BudgetError, OSError, ValueError) as exc:
        error = (error + '; ' if error else '') + 'Final budget receipt unavailable: ' + str(exc)
        final_budget = {'status': 'unreadable-fail-closed', 'error': str(exc),
                        'authorizesIntegration': False}
    result = {'schema': 1, 'status': 'failed-closed' if error else 'protocol-completed',
              'error': error, 'histories': outcomes, 'budget': final_budget,
              'fullRecoveryQualified': False, 'fullHistoryAuthorized': False}
    write_new_json(output / 'run-result.json', result)
    if error:
        raise RunnerError(error)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--budget-root', type=Path, required=True)
    parser.add_argument('--program', type=Path)
    parser.add_argument('--tasks', type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        print(json.dumps(run(args.binary, args.manifest, args.output, args.budget_root,
                             execute=args.execute, program=args.program, tasks=args.tasks), indent=2))
    except (RunnerError, budgetlib.BudgetError, OSError, ValueError, TypeError) as exc:
        parser.exit(2, 'REFUSED: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
