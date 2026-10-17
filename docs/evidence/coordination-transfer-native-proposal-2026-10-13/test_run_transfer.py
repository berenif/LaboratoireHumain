#!/usr/bin/env python3
"""Pure protocol tests and two explicitly inert pipe/process tests; no physics.

Every execute=True call uses an in-memory fake worker and a temporary, synthetic
authorization. Real subprocess tests launch only the tiny scripts written here.
No project worker, model, FK, native kernel or existing run ledger is executed.
"""
from contextlib import contextmanager
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('transfer_runner', Path(__file__).with_name('run_transfer.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeClock:
    def __init__(self):
        self.elapsed = self.wall_shift = 0
    def wall(self):
        return 1791873840 + self.elapsed + self.wall_shift
    def monotonic(self):
        return 100 + self.elapsed


@contextmanager
def no_alarm(_deadline, _worker):
    yield


class FakeWorker:
    def __init__(self, test):
        self.test, self.pid, self.ready = test, 424242, True
        self.sent, self.killed, self.disposed = [], False, False
    def receive(self, deadline, guard):
        if self.ready:
            self.ready = False
            self.test.clock.elapsed += self.test.options.get('ready_elapsed', 0)
            self.test.clock.wall_shift += self.test.options.get('ready_wall_shift', 0)
            if self.test.options.get('mutate_input_at_ready'):
                self.test.entry.write_text('{"changedDuringStartup":true}\n')
            guard()
            return {'event': 'ready', 'pid': self.test.options.get('ready_pid', self.pid),
                    'protocol': self.test.options.get('ready_protocol', 'transfer-run-v1')}
        self.test.clock.elapsed += self.test.options.get('result_elapsed', 0)
        guard()
        if self.test.options.get('crash'):
            raise runner.Refusal('synthetic worker crash')
        return self.response
    def send(self, message, deadline, guard):
        guard()
        self.sent.append(copy.deepcopy(message))
        state = self.test.ledger()
        self.test.assertIsNone(state['actualNativeStepsKnown'])
        self.test.assertEqual(state['actualNativeStepsUpperBound'], runner.TOTAL)
        self.test.assertTrue(state['operationMayHaveStarted'])
        events = self.test.events()
        self.test.assertEqual(events[-1]['payload']['event'], 'record-single-transfer-operation-durably-before-send')
        for name, size in (('technical', 52), ('history', 131044)):
            permit = message[name + 'Permit']
            self.test.assertEqual(permit['workerPid'], self.pid)
            self.test.assertEqual(permit['reservedSteps'], size)
            self.test.assertEqual(state['reservations'][name]['workerPid'], self.pid)
            self.test.assertEqual(permit['deadlineMonotonic'], 7293)
        self.test.assertNotEqual(message['technicalPermit']['reservationId'], message['historyPermit']['reservationId'])
        report = self.test.report(message)
        edit = self.test.options.get('edit_report')
        if edit:
            edit(report)
        self.response = {k: copy.deepcopy(v) for k, v in report.items() if k != 'audit'}
        edit = self.test.options.get('edit_response')
        if edit:
            edit(self.response)
        if self.test.options.get('symlink_evidence'):
            Path(message['outputPath']).symlink_to(self.test.entry)
        else:
            runner.write_new(Path(message['outputPath']), report)
        if not self.test.options.get('missing_trace'):
            Path(message['tracePath']).write_text('{"syntheticOnly":true}\n')
        if self.test.options.get('checkpoint'):
            self.test.checkpoints(message, **self.test.options['checkpoint'])
        if self.test.options.get('mutate_old'):
            self.test.old[0].write_text('synthetic external mutation\n')
        if self.test.options.get('mutate_manifest'):
            self.test.manifest.write_text('{}\n')
        if self.test.options.get('send_failure'):
            raise OSError('synthetic partial request write')
    def finish(self, deadline):
        self.test.clock.elapsed += self.test.options.get('finish_elapsed', 0)
        return self.test.options.get('exit', 0)
    def kill(self):
        self.killed = True
    def dispose(self):
        self.kill()
        self.disposed = True


class RunnerTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='transfer-supervisor-pure-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.clock, self.options, self.workers, self.alarms = FakeClock(), {}, [], []
        self.binary = self.root / 'NOT_AN_EXECUTABLE_TEST_DATA_ONLY'
        self.binary.write_text('Never execute this fake binary\n')
        self.binary.chmod(0o700)
        self.old = [self.root / f'protected-{i}.json' for i in range(14)]
        for i, path in enumerate(self.old):
            runner.write_new(path, {'syntheticProtectedRecord': i})
        self.original_old = [p.read_bytes() for p in self.old]
        self.preparation = self.root / 'preparation.json'
        runner.write_new(self.preparation, {
            'protectedClosedLedgers': [self.record(p) for p in self.old[:5]],
            'protectedBinaries': [self.record(p) for p in self.old[5:9]],
            'protectedManifests': [self.record(p) for p in self.old[9:]]})
        self.value = {'schema': runner.MANIFEST_SCHEMA, 'protocolId': runner.PROTOCOL,
            'authorizationId': 'transfer-a36-distinct-SYNTHETIC-TEST-ONLY',
            'binaryPath': str(self.binary), 'outputPath': str(self.root / 'native-run'),
            'inputs': {}, 'files': [], 'prerequisites': dict.fromkeys(runner.PREREQUISITES, True)}
        code = str(Path(runner.__file__).resolve())
        for role in sorted(runner.ROLES):
            if role == 'binary': path = self.binary
            elif role == 'code': path = Path(code)
            elif role == 'preparation': path = self.preparation
            else:
                path = self.root / f'{role}.json'
                value = {'synthetic': role}
                if role == 'entry': value['nativeSubstep'] = 36
                if role == 'prefix': value.update(schema=1, prefixFrames=[{} for _ in range(36)])
                runner.write_new(path, value)
            self.value['files'].append(dict(self.record(path), role=role))
            for key, input_role in runner.INPUT_ROLES.items():
                if role == input_role: self.value['inputs'][key] = str(path)
        self.entry = Path(self.value['inputs']['entryPath'])
        self.manifest, self.authorization = self.root / 'manifest.json', self.root / 'authorization.json'
        self.approval = {'schema': runner.AUTHORIZATION_SCHEMA, 'protocolId': runner.PROTOCOL,
            'authorizationId': self.value['authorizationId'], 'approved': True,
            'approvalEvidence': 'SYNTHETIC TEST ONLY, not a project authorization',
            'approvedAtUtc': '2026-01-01T00:00:00+00:00', 'singleUse': True,
            'reopenHistoricalFamily': False, 'limits': runner.LIMITS.copy()}
        self.save_inputs()
        self.patch(runner, 'REPO', self.root)
        self.patch(runner, 'PREPARATION', self.preparation)
        self.patch(runner, 'PREPARATION_SHA256', runner.sha(self.preparation))
        self.patch(runner, 'required_code_paths', lambda: {code})
        self.patch(runner.common, 'environment_preflight', lambda: None)
        self.patch(runner.common.subprocess, 'Popen', mock.Mock(side_effect=AssertionError('NO PROCESS IN PURE TESTS')))
    def patch(self, target, name, value):
        patch = mock.patch.object(target, name, value)
        patch.start()
        self.addCleanup(patch.stop)
    def record(self, path):
        return {'path': str(path), 'sha256': runner.sha(path)}
    def save_inputs(self):
        self.manifest.write_text(json.dumps(self.value) + '\n')
        self.approval['manifestSha256'] = runner.sha(self.manifest)
        self.authorization.write_text(json.dumps(self.approval) + '\n')
    def factory(self, binary, output):
        self.assertEqual(binary, self.binary)
        self.assertTrue((self.root / 'authorization-claim.json').is_file())
        state = self.ledger()
        self.assertEqual(state['reservedNativeSteps'], runner.TOTAL)
        self.assertEqual(state['actualNativeStepsKnown'], 0)
        self.assertEqual(state['actualNativeStepsUpperBound'], 0)
        self.assertEqual(state['deadlineMonotonic'], 7300)
        if self.options.get('spawn_failure'): raise OSError('synthetic spawn failure')
        worker = FakeWorker(self)
        self.workers.append(worker)
        return worker
    def inspector(self, pid):
        return {'pid': pid, 'threads': self.options.get('threads', 1),
            'children': self.options.get('children', ''),
            'startTicks': 'changed' if self.options.get('change_identity') and self.workers[0].sent else '123',
            'binaryPath': str(self.binary), 'binarySha256': runner.sha(self.binary)}
    def run_batch(self, execute=True):
        return runner.run(self.manifest, self.authorization, execute=execute, batch_root=self.root,
            clock=self.clock, worker_factory=self.factory, inspector=self.inspector,
            alarm_factory=no_alarm, arm_deadline=self.alarms.append)
    def ledger(self):
        envelope = runner.read_json(self.root / 'native-run/physical-ledger.json')
        self.assertEqual(envelope['sha256'], runner.hashlib.sha256(runner.canonical(envelope['payload'])).hexdigest())
        return envelope['payload']
    def events(self):
        rows = [runner.common.decode_json(line) for line in (self.root / 'native-run/ledger-events.ndjson').read_bytes().splitlines()]
        previous = None
        for sequence, row in enumerate(rows):
            self.assertEqual(row['payload']['sequence'], sequence)
            self.assertEqual(row['payload']['previousSha256'], previous)
            self.assertEqual(row['sha256'], runner.hashlib.sha256(runner.canonical(row['payload'])).hexdigest())
            previous = row['sha256']
        return rows
    def failure(self, pattern):
        with self.assertRaisesRegex(runner.Refusal, pattern): self.run_batch()
        state = self.ledger()
        self.assertIn(state['status'], ('closed-refused', 'closed-unknown'))
        self.assertEqual(state['reservedNativeSteps'], runner.TOTAL)
        self.assertEqual(state['refundNativeSteps'], 0)
        for worker in self.workers:
            self.assertTrue(worker.killed)
            self.assertTrue(worker.disposed)
        self.events()
        return state
    def report(self, message):
        # Arithmetic-only fake success, explicitly not native evidence.
        technical, history = 52, 516
        audit = {'technicalPrefixNativeSteps': 36, 'technicalParityNativeSteps': 16,
            'realHistoryNativeSteps': 36, 'forecastCalls': 1,
            'forecastRequestedSteps': 480, 'forecastActualSteps': 480}
        accounting = lambda n: {'admittedNativeSteps': n, 'completedNativeSteps': n,
            'actualNativeStepsExact': True, 'abandonedNativeSlots': 0}
        return {'schema': 1, 'event': 'result', 'op': 'transfer-run', 'pid': self.workers[0].pid,
            'operationId': message['operationId'],
            'technicalReservationId': message['technicalPermit']['reservationId'],
            'historyReservationId': message['historyPermit']['reservationId'],
            'reservedNativeSteps': runner.TOTAL, 'technicalReservedNativeSteps': technical,
            'historyReservedNativeSteps': runner.HISTORY, 'technicalActualNativeSteps': technical,
            'historyActualNativeSteps': history, 'actualNativeSteps': technical + history,
            'outcome': dict.fromkeys(runner.GATES, True), 'evidencePath': message['outputPath'],
            'refusal': None, 'countsReconciled': True, 'technicalAccounting': accounting(technical),
            'historyAccounting': accounting(history), 'prefixExclusions': runner.PREFIX_EXCLUSIONS.copy(),
            'noRefund': True, 'automaticRenewalAllowed': False, 'audit': audit}
    def checkpoints(self, message, *, in_flight=False, partial=False, corrupt=False):
        for name in ('technical', 'history'):
            base = {'event': 'permit-activated', 'reservationId': message[name + 'Permit']['reservationId'],
                'admittedNativeSteps': 0, 'completedNativeSteps': 0, 'pendingNativeSlots': 0,
                'abandonedNativeSlots': 0, 'actualNativeStepsExact': True, 'nextNativeClock': 0}
            rows = [copy.deepcopy(base)]
            if name == 'technical':
                for i in range(2):
                    base.update(event='before-integration', admittedNativeSteps=i+1,
                        completedNativeSteps=i, pendingNativeSlots=3-i,
                        actualNativeStepsExact=False, nextNativeClock=i)
                    rows.append(copy.deepcopy(base))
                    if i == 1 and in_flight: break
                    base.update(event='after-integration', completedNativeSteps=i+1,
                                actualNativeStepsExact=True, nextNativeClock=i+1)
                    rows.append(copy.deepcopy(base))
            if corrupt: rows[-1]['reservationId'] = 'wrong-reservation'
            path = Path(message['outputPath'] + f'.{name}.accounting.jsonl')
            path.write_bytes(b''.join(runner.canonical(v)+b'\n' for v in rows) + (b'{"partial"' if partial else b''))

    def test_default_is_read_only_and_never_claims_or_spawns(self):
        result = self.run_batch(False)
        self.assertEqual(result['nativeSteps'], 0)
        self.assertFalse((self.root / 'authorization-claim.json').exists())
        self.assertFalse((self.root / 'native-run').exists())
        self.assertEqual(self.workers, [])

    def test_inspection_without_authorization_checks_frozen_manifest_only(self):
        self.authorization.unlink()
        result = runner.run(self.manifest, batch_root=self.root)
        self.assertEqual(result['status'], 'static-preflight-passed')
        self.assertEqual(result['nativeSteps'], 0)
        self.assertFalse(result['authorizationVerified'])
        self.assertFalse(result['inspectionGrantsNativePermission'])
        self.assertEqual(result['historicalProtectedFilesVerified'], 14)
        self.assertFalse((self.root / 'authorization-claim.json').exists())
        self.assertFalse((self.root / 'native-run').exists())

    def test_execution_without_authorization_refuses_before_claim(self):
        self.authorization.unlink()
        with self.assertRaisesRegex(runner.Refusal, 'requires a distinct explicit authorization'):
            runner.run(self.manifest, execute=True, batch_root=self.root,
                       worker_factory=self.factory)
        self.assertFalse((self.root / 'authorization-claim.json').exists())
        self.assertEqual(self.workers, [])

    def test_frozen_source_inventory_includes_both_supervisors_and_worker(self):
        with mock.patch.object(runner, 'REPO', Path(runner.__file__).resolve().parents[3]):
            paths = RunnerTests.inventory_function()
        self.assertIn(str(Path(runner.__file__).resolve()), paths)
        self.assertTrue(any(p.endswith('/transfer_worker.rs') for p in paths))
        self.assertTrue(any(p.endswith('/run-coordination-numeric-parity.py') for p in paths))

    def test_complete_fake_run_binds_both_reservations_and_preserves_old_files(self):
        result = self.run_batch()
        self.assertEqual(result['status'], 'closed-success')
        self.assertEqual(self.ledger()['actualNativeStepsKnown'], 568)
        self.assertEqual(len(self.workers), 1)
        self.assertEqual(len(self.workers[0].sent), 1)
        self.assertEqual(self.alarms, [130, 7293, 7300])
        self.assertEqual(self.original_old, [p.read_bytes() for p in self.old])
        self.events()

    def test_claim_prevents_retry_even_with_new_output(self):
        self.run_batch()
        self.value['outputPath'] = str(self.root / 'retry')
        self.save_inputs()
        with self.assertRaisesRegex(runner.Refusal, 'already claimed'): self.run_batch()
        self.assertEqual(len(self.workers), 1)

    def test_no_approval_no_claim(self):
        self.approval['approved'] = False
        self.save_inputs()
        with self.assertRaisesRegex(runner.Refusal, 'explicit approval'): self.run_batch()
        self.assertFalse((self.root / 'authorization-claim.json').exists())

    def test_frozen_hash_mismatch_no_claim(self):
        self.entry.write_text('{}')
        with self.assertRaisesRegex(runner.Refusal, 'hash mismatch'): self.run_batch()
        self.assertFalse((self.root / 'authorization-claim.json').exists())

    def test_input_mutation_during_startup_refuses_before_send(self):
        self.options['mutate_input_at_ready'] = True
        self.assertEqual(self.failure('hash mismatch')['actualNativeStepsKnown'], 0)
        self.assertEqual(self.workers[0].sent, [])

    def test_incomplete_code_inventory_refuses_before_claim(self):
        runner.required_code_paths = lambda: {'/missing-required-source.rs'}
        with self.assertRaisesRegex(runner.Refusal, 'source inventory'): self.run_batch()
        self.assertFalse((self.root / 'authorization-claim.json').exists())

    def test_changed_budget_is_not_authorization(self):
        self.approval['limits']['totalNativeSteps'] += 1
        self.save_inputs()
        with self.assertRaisesRegex(runner.Refusal, 'exact distinct'): self.run_batch()

    def test_extra_manifest_field_refused(self):
        self.value['allowRetry'] = True
        self.save_inputs()
        with self.assertRaisesRegex(runner.Refusal, 'Missing/unknown'): self.run_batch()

    def test_spawn_failure_is_known_zero_with_full_reserve_burned(self):
        self.options['spawn_failure'] = True
        self.assertEqual(self.failure('spawn failure')['actualNativeStepsKnown'], 0)

    def test_ready_pid_mismatch_is_known_zero(self):
        self.options['ready_pid'] = 7
        self.assertEqual(self.failure('Ready PID')['actualNativeStepsKnown'], 0)

    def test_ready_protocol_mismatch_refused_before_send(self):
        self.options['ready_protocol'] = 'old-run'
        self.failure('Ready PID')
        self.assertEqual(self.workers[0].sent, [])

    def test_startup_monotonic_cap_is_within_original_clock(self):
        self.options['ready_elapsed'] = 30
        self.assertEqual(self.failure('Startup exceeded')['actualNativeStepsKnown'], 0)

    def test_startup_utc_cap_also_applies(self):
        self.options['ready_wall_shift'] = 30
        self.assertEqual(self.failure('Startup exceeded')['actualNativeStepsKnown'], 0)

    def test_utc_reversal_refuses_without_native_steps(self):
        self.options['ready_wall_shift'] = -1
        self.assertEqual(self.failure('clock reversed')['actualNativeStepsKnown'], 0)

    def test_thread_contract_refuses_before_operation(self):
        self.options['threads'] = 2
        self.assertEqual(self.failure('Single worker')['actualNativeStepsKnown'], 0)

    def test_child_process_contract_refuses_before_operation(self):
        self.options['children'] = '666'
        self.assertEqual(self.failure('Single worker')['actualNativeStepsKnown'], 0)

    def test_identity_change_after_send_retains_uncertainty(self):
        self.options['change_identity'] = True
        state = self.failure('Single worker')
        self.assertIsNone(state['actualNativeStepsKnown'])
        self.assertEqual(state['actualNativeStepsUpperBound'], runner.TOTAL)

    def test_partial_send_cannot_report_zero(self):
        self.options['send_failure'] = True
        state = self.failure('partial request')
        self.assertIsNone(state['actualNativeStepsKnown'])
        self.assertEqual(state['actualNativeStepsUpperBound'], runner.TOTAL)

    def test_native_deadline_keeps_seven_seconds_for_shutdown(self):
        self.options['result_elapsed'] = 7193
        self.failure('Frozen deadline')
        self.assertEqual(self.ledger()['deadlineMonotonic'], 7300)

    def test_shutdown_cannot_renew_original_deadline(self):
        self.options['finish_elapsed'] = 7200
        self.assertEqual(self.failure('Frozen deadline')['actualNativeStepsKnown'], 568)

    def test_result_evidence_disagreement_refuses(self):
        self.options['edit_response'] = lambda r: r.update(pid=5)
        self.failure('Unexpected operation')

    def test_wrong_reservation_id_refuses(self):
        self.options['edit_report'] = lambda r: r.update(historyReservationId='wrong')
        self.failure('both durable reservations')

    def test_unknown_result_fields_refuse(self):
        self.options['edit_response'] = lambda r: r.update(retry=True)
        self.failure('Missing/unknown')

    def test_wrong_total_refuses(self):
        self.options['edit_report'] = lambda r: r.update(actualNativeSteps=569)
        self.failure('Total counter')

    def test_bool_counter_is_not_an_integer_native_count(self):
        self.options['edit_report'] = lambda r: r.update(historyActualNativeSteps=True)
        self.failure('Invalid bounded native counter')

    def test_unactivated_gates_report_known_zero_even_on_refusal(self):
        def fail(r):
            r.update(technicalActualNativeSteps=0, historyActualNativeSteps=0,
                actualNativeSteps=0, technicalAccounting=None, historyAccounting=None,
                refusal='synthetic activation failed')
            r['audit'] = dict.fromkeys(r['audit'], 0)
            r['outcome'] = dict.fromkeys(runner.GATES, False)
        self.options['edit_report'] = fail
        self.assertEqual(self.failure('Partial, unknown')['actualNativeStepsKnown'], 0)

    def test_known_partial_native_failure_is_preserved_before_gate(self):
        def fail(r):
            r['outcome']['local_success'] = False
            r['refusal'] = 'synthetic local failure'
        self.options['edit_report'] = fail
        state = self.failure('Partial, unknown')
        self.assertEqual(state['actualNativeStepsKnown'], 568)
        self.assertEqual(state['status'], 'closed-refused')

    def test_in_flight_receipt_preserves_completed_to_admitted_interval(self):
        def fail(r):
            r.update(historyActualNativeSteps=None, actualNativeSteps=None, refusal='synthetic in-flight call')
            r['historyAccounting'].update(admittedNativeSteps=517, actualNativeStepsExact=False)
            r['outcome']['integrity_ok'] = False
        self.options['edit_report'] = fail
        state = self.failure('Partial, unknown')
        self.assertEqual((state['actualNativeStepsLowerBound'], state['actualNativeStepsUpperBound']), (568, 569))

    def test_unknown_counter_may_not_claim_exact_gate(self):
        self.options['edit_report'] = lambda r: r.update(historyActualNativeSteps=None, actualNativeSteps=None)
        self.failure('accounting interval')

    def test_forecast_cadence_arithmetic_is_checked(self):
        self.options['edit_report'] = lambda r: r['audit'].update(forecastRequestedSteps=479)
        self.failure('exactly 480')

    def test_missing_trace_cannot_pass(self):
        self.options['missing_trace'] = True
        self.failure('Frozen file')

    def test_symlink_evidence_cannot_pass(self):
        self.options['symlink_evidence'] = True
        self.failure('not a symlink')

    def test_frozen_files_are_rechecked_after_result(self):
        self.options['mutate_old'] = True
        self.assertFalse(self.failure('hash mismatch')['terminalFrozenFilesVerified'])

    def test_manifest_cannot_change_during_run(self):
        self.options['mutate_manifest'] = True
        self.assertFalse(self.failure('hash mismatch')['terminalFrozenFilesVerified'])

    def test_crash_recovers_exact_completed_native_checkpoint_counts(self):
        self.options.update(crash=True, checkpoint={})
        # Empty dict intentionally enables no journal; explicit option below.
        self.options['checkpoint'] = {'in_flight': False}
        state = self.failure('worker crash')
        self.assertEqual(state['actualNativeStepsKnown'], 2)
        self.assertEqual(state['reservedNativeSteps'], runner.TOTAL)

    def test_crash_retains_one_uncertain_in_flight_native_call(self):
        self.options.update(crash=True, checkpoint={'in_flight': True})
        state = self.failure('worker crash')
        self.assertIsNone(state['actualNativeStepsKnown'])
        self.assertEqual((state['actualNativeStepsLowerBound'], state['actualNativeStepsUpperBound']), (1, 2))

    def test_partial_final_checkpoint_does_not_become_exact_zero(self):
        self.options.update(crash=True, checkpoint={'partial': True})
        state = self.failure('worker crash')
        self.assertIsNone(state['actualNativeStepsKnown'])
        self.assertEqual((state['actualNativeStepsLowerBound'], state['actualNativeStepsUpperBound']), (2, 4))

    def test_corrupt_checkpoint_never_erases_uncertainty(self):
        self.options.update(crash=True, checkpoint={'corrupt': True})
        state = self.failure('worker crash')
        self.assertIsNone(state['actualNativeStepsKnown'])
        self.assertEqual(state['actualNativeStepsUpperBound'], runner.TOTAL)


# Save original inventory function before per-test replacement.
RunnerTests.inventory_function = staticmethod(runner.required_code_paths)


class InertProcessTests(unittest.TestCase):
    """Only temporary scripts containing print/read/sleep are executable here."""
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='transfer-supervisor-INERT-process-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
    def script(self, body):
        path = self.root / 'inert_child.py'
        path.write_text(f'#!{sys.executable}\n' + body)
        path.chmod(0o700)
        return path
    def test_inert_pipe_ready_single_message_result_and_eof(self):
        path = self.script("import json, os, sys\nprint(json.dumps({'event':'ready','pid':os.getpid(),'protocol':'inert-only'}), flush=True)\nvalue=json.loads(sys.stdin.readline())\nprint(json.dumps({'echo':value,'inert':True}), flush=True)\nsys.stdin.read()\n")
        worker = runner.NativeWorker(path, self.root)
        try:
            deadline = time.monotonic() + 3
            self.assertEqual(worker.receive(deadline, lambda: None)['pid'], worker.pid)
            worker.send({'only': 'inert test'}, deadline, lambda: None)
            self.assertEqual(worker.receive(deadline, lambda: None), {'echo': {'only': 'inert test'}, 'inert': True})
            self.assertEqual(worker.finish(deadline), 0)
        finally:
            worker.dispose()
    def test_hard_alarm_kills_new_inert_process_group_not_existing_child(self):
        old = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(10)'], start_new_session=True)
        worker = None
        try:
            path = self.script('import time\ntime.sleep(10)\n')
            with self.assertRaisesRegex(runner.Refusal, 'Frozen transfer deadline'):
                with runner.hard_deadline(time.monotonic() + 0.2, lambda: worker):
                    worker = runner.NativeWorker(path, self.root)
                    worker.receive(time.monotonic() + 2, lambda: None)
            self.assertIsNone(old.poll())
            self.assertEqual(worker.process.wait(timeout=1), -signal.SIGKILL)
            self.assertEqual(signal.getitimer(signal.ITIMER_REAL), (0.0, 0.0))
        finally:
            if worker is not None: worker.dispose()
            old.kill()
            old.wait(timeout=1)

    def test_hard_alarm_during_constructor_cleans_unassigned_inert_child(self):
        spawned = []
        worker = None
        def constructor_interrupted_after_fork():
            child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(10)'], start_new_session=True)
            spawned.append(child)
            time.sleep(1)  # Interrupted by the 0.2-second alarm below.
            raise AssertionError('alarm failed')
        try:
            with self.assertRaisesRegex(runner.Refusal, 'Frozen transfer deadline'):
                with runner.hard_deadline(time.monotonic() + 0.2, lambda: worker):
                    worker = constructor_interrupted_after_fork()
            self.assertIsNone(worker)
            self.assertEqual(len(spawned), 1)
            self.assertEqual(spawned[0].wait(timeout=1), -signal.SIGKILL)
            self.assertEqual(signal.getitimer(signal.ITIMER_REAL), (0.0, 0.0))
        finally:
            for child in spawned:
                if child.poll() is None: child.kill()
                child.wait(timeout=1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
