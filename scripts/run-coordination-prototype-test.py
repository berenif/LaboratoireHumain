#!/usr/bin/env python3
"""Runner protocol tests with an in-memory fake worker. Never launches a process.

Only dummy JSON files, fake accounting receipts and inert OS pipes are used.
Reported native-step values below are synthetic protocol data, not integrations.
"""

from contextlib import contextmanager
import importlib.util
import signal
import json
import os
from pathlib import Path
import tempfile
import time
import types
import unittest
from unittest import mock


spec = importlib.util.spec_from_file_location('coordination_runner',
    Path(__file__).with_name('run-coordination-prototype.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
budget = runner.budgetlib


class FakeClock:
    def sample(self):
        return {'wall': 1791835200.0, 'monotonic': 1000.0, 'boot': 'synthetic-boot'}


class FakeWorker:
    """Sends only fabricated receipts; no subprocess, model, or physics exists."""
    def __init__(self, binary, output, settings):
        self.pid = 42
        self.output = output
        self.settings = settings
        self.incoming = [{'event': 'ready', 'pid': settings.get('ready_pid', self.pid)}]
        self.sent = []
        self.stop_reason = None
        self.current = None
        self.planning_index = 0

    def diagnostic(self, operation):
        result = {k: True for k in runner.COMMON_OUTCOME}
        if operation == 'verify':
            result.update({k: True for k in budget.NATIVE_GATES})
        if operation == 'history':
            result['local_success'] = self.settings.get('local_success', {}).get(self.current['id'], True)
        result.update(self.settings.get('outcome_overrides', {}).get(operation, {}))
        return result

    def result(self, message, operation, actual):
        path = Path(message['outputPath'])
        evidence = {'synthetic': True, 'neverIntegratedPhysics': True,
                    'operation': operation, 'actualNativeSteps': actual, 'outcome': self.diagnostic(operation)}
        evidence.update(self.settings.get('evidence_overrides', {}))
        path.write_text(json.dumps(evidence) + '\n')
        return {'event': 'result', 'op': operation, 'actualNativeSteps': actual,
                'outcome': self.diagnostic(operation), 'evidencePath': str(path)}

    def next_history_event(self):
        count = self.settings.get('planning_calls', 1) if self.current['enabled'] else 0
        if self.settings.get('A_requests_planning') and self.current['id'] == 'A':
            count = 1
        if self.planning_index < count:
            self.incoming.append({'event': 'planning-request', 'historyStep': self.planning_index * 24,
                                  'longSteps': self.settings.get('long_steps', [480] * 4),
                                  'shortProbes': 0, 'requiredReservedSteps': 1920})
        else:
            self.incoming.append(self.result(self.current, 'history', self.settings.get('history_steps', 960)))

    def send(self, message):
        self.sent.append(message)
        operation = message['op']
        if operation == 'verify':
            response = self.result(message, operation, self.settings.get('verify_steps', 16))
            if self.settings.get('extra_result_field'):
                response['unrequestedHistory'] = True
            if self.settings.get('outside_evidence'):
                response['evidencePath'] = str(self.settings['outside_evidence'])
            self.incoming.append(response)
            if self.settings.get('mutate_frozen'):
                Path(message['inputs']['tasksPath']).write_text('{"changed":true}\n')
            if self.settings.get('corrupt_budget'):
                Path(self.settings['corrupt_budget']).write_text('{"corrupted":true}\n')
        elif operation == 'history':
            self.current = message
            self.planning_index = 0
            self.next_history_event()
        elif operation == 'planning-permit':
            self.incoming.append(self.result(message, 'planning', self.settings.get('planning_steps', 1920)))
            self.planning_index += 1
            # Deliberately do not advance the story until its durable ACK.
        elif operation == 'continue-history':
            if message['permit']['reservationId'] != self.current['permit']['reservationId']:
                raise AssertionError('History ACK must preserve the original reservation')
            self.next_history_event()
        else:
            raise AssertionError('Unexpected parent operation: ' + operation)

    def receive(self, deadline_monotonic):
        if not self.incoming:
            raise runner.RunnerError('Synthetic worker is waiting for its required history ACK')
        return self.incoming.pop(0)

    def stop(self, reason=None):
        self.stop_reason = reason
        if self.settings.get('stop_raises'):
            raise RuntimeError('Synthetic worker stop failure')
        return 0


class RunnerTests(unittest.TestCase):
    def use_local_pair(self):
        self.manifest_value['protocolMode'] = budget.LOCAL_PAIR_MODE
        self.manifest_value['prerequisites'].pop('four_histories_frozen')
        self.manifest_value['prerequisites']['local_pair_frozen'] = True
        self.save_manifest()

    def test_local_pair_success_stops_after_b_at_exact_maximum(self):
        self.use_local_pair()
        self.settings.update(planning_calls=40, local_success={'A': False, 'B': True})
        result = self.run_protocol()
        self.assertEqual(result['protocolMode'], 'local-pair')
        self.assertEqual([h['history'] for h in result['histories']], ['A', 'B'])
        self.assertEqual(result['budget']['status'], 'closed-completed')
        self.assertEqual(result['budget']['counters']['reservedNativeSteps'], 78736)
        self.assertEqual(result['budget']['counters']['actualNativeStepsKnown'], 78736)
        self.assertEqual(len(self.workers), 1)
        messages = self.workers[0].sent
        self.assertEqual(sum(m['op'] == 'verify' for m in messages), 1)
        self.assertEqual([m['id'] for m in messages if m['op'] == 'history'], ['A', 'B'])
        self.assertEqual(sum(m['op'] == 'planning-permit' for m in messages), 40)
        self.assertEqual(sum(m['op'] == 'continue-history' for m in messages), 40)
        self.assertFalse(result['fullHistoryAuthorized'])
        self.assertFalse(result['fullRecoveryQualified'])

    def test_local_pair_failed_b_still_closes_after_b(self):
        self.use_local_pair()
        self.settings['local_success'] = {'A': False, 'B': False}
        result = self.run_protocol()
        self.assertEqual([h['history'] for h in result['histories']], ['A', 'B'])
        self.assertFalse(result['histories'][-1]['localSuccess'])
        self.assertEqual(result['budget']['status'], 'closed-completed')

    def test_local_pair_static_preflight_has_no_budget_mutation(self):
        self.use_local_pair()
        before = (self.budget_root / 'ledger.json').read_bytes()
        result = self.run_protocol(execute=False)
        self.assertEqual(result['maximumNativeSteps'], 78736)
        self.assertEqual(result['historyOrder'], ['A', 'B'])
        self.assertEqual(result['physicalIntegrations'], 0)
        self.assertEqual((self.budget_root / 'ledger.json').read_bytes(), before)
        self.assertEqual(self.workers, [])

    def test_unknown_protocol_mode_refused_before_worker(self):
        self.manifest_value['protocolMode'] = 'repeat-until-success'
        self.save_manifest()
        before = (self.budget_root / 'ledger.json').read_bytes()
        with self.assertRaisesRegex(budget.BudgetError, 'Unknown frozen protocolMode'):
            self.run_protocol()
        self.assertEqual((self.budget_root / 'ledger.json').read_bytes(), before)
        self.assertEqual(self.workers, [])

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='coordination-runner-test-')
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.binary = self.directory / 'never-executed-worker'
        self.binary.write_text('INERT SYNTHETIC FILE; NOT AN EXECUTABLE PROGRAM\n')
        self.binary.chmod(0o700)
        self.program = self.directory / 'program.json'
        self.tasks = self.directory / 'tasks.json'
        self.program.write_text('{"synthetic":"program"}\n')
        self.tasks.write_text('{"synthetic":"tasks"}\n')
        program_record = {'path': str(self.program), 'sha256': budget.sha(self.program)}
        self.manifest_value = {'schema': 1,
            'files': {role: [dict(program_record)] for role in budget.REQUIRED_ROLES},
            'prerequisites': {key: True for key in budget.PREREQUISITES},
            'workerInputs': {'programPath': str(self.program), 'tasksPath': str(self.tasks)}}
        self.manifest_value['files']['binary'] = [{'path': str(self.binary), 'sha256': budget.sha(self.binary)}]
        self.manifest_value['files']['profile'].append({'path': str(self.tasks), 'sha256': budget.sha(self.tasks)})
        self.manifest = self.directory / 'manifest.json'
        self.save_manifest()
        self.budget_root = self.directory / 'budget'
        self.ledger = budget.Budget(self.budget_root, clock=FakeClock(), process_inspector=lambda pid:
            {'pid': pid, 'threads': 1, 'boot': 'synthetic-boot', 'startTicks': '1',
             'binarySha256': budget.sha(self.binary)})
        self.ledger.init_preparation()
        self.output = self.directory / 'output'
        self.settings = {}
        self.workers = []

    def save_manifest(self):
        self.manifest.write_text(json.dumps(self.manifest_value) + '\n')

    def worker_factory(self, binary, output):
        worker = FakeWorker(binary, output, self.settings)
        self.workers.append(worker)
        return worker

    def run_protocol(self, execute=True):
        return runner.run(self.binary, self.manifest, self.output, self.budget_root,
                          execute=execute, ledger=self.ledger, worker_factory=self.worker_factory)

    def state(self):
        return self.ledger.status()

    def expect_failure(self, message):
        with self.assertRaisesRegex(runner.RunnerError, message):
            self.run_protocol()
        result = json.loads((self.output / 'run-result.json').read_text())
        self.assertEqual(result['status'], 'failed-closed')
        self.assertTrue(self.state()['status'].startswith('closed'))
        if self.workers:
            self.assertIsNotNone(self.workers[0].stop_reason)
        return result

    def test_local_pair_supervisor_external_wall_includes_start_and_stop(self):
        self.use_local_pair()
        deadlines = []
        @contextmanager
        def capture_alarm(deadline, worker_getter):
            deadlines.append(deadline)
            self.assertIsNone(worker_getter())
            yield
            self.assertIs(worker_getter(), self.workers[0])
        result = runner.run(self.binary, self.manifest, self.output, self.budget_root,
                            execute=True, ledger=self.ledger, worker_factory=self.worker_factory,
                            alarm_factory=capture_alarm)
        receipt = json.loads((self.output / 'process-supervision.json').read_text())
        self.assertEqual(receipt['maximumTotalWallSeconds'], 7137)
        self.assertEqual(receipt['nativeWallSeconds'], 7100)
        self.assertEqual(receipt['deadlineMonotonic'], deadlines[0])
        self.assertAlmostEqual(deadlines[0] - receipt['startedMonotonic'], 7137, places=6)
        self.assertEqual(result['status'], 'protocol-completed')

    def test_contradictory_evidence_step_count_stays_unknown(self):
        self.settings['evidence_overrides'] = {'actualNativeSteps': 15}
        self.expect_failure('Evidence contradicts')
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 0)
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16)

    def test_contradictory_evidence_outcome_stays_unknown(self):
        self.settings['evidence_overrides'] = {'outcome': {'integrity_ok': False}}
        self.expect_failure('Evidence contradicts')
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 0)

    def test_evidence_integer_cannot_impersonate_boolean_diagnostic(self):
        self.settings['evidence_overrides'] = {'outcome': {
            **{key: True for key in runner.COMMON_OUTCOME + budget.NATIVE_GATES}, 'integrity_ok': 1}}
        self.expect_failure('Evidence contradicts')
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 0)

    def test_evidence_float_cannot_impersonate_integer_count(self):
        self.settings['evidence_overrides'] = {'actualNativeSteps': 16.0}
        self.expect_failure('Evidence contradicts')
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 0)

    def test_default_performs_no_worker_launch_or_budget_mutation(self):
        before = (self.budget_root / 'ledger.json').read_bytes()
        result = self.run_protocol(execute=False)
        self.assertEqual(result['physicalIntegrations'], 0)
        self.assertFalse(result['executeRequested'])
        self.assertEqual(self.workers, [])
        self.assertEqual(before, (self.budget_root / 'ledger.json').read_bytes())

    def test_default_accepts_active_preparation_without_completing_it(self):
        self.ledger.reserve('preparation', active_seconds=3600, inspections=100)
        before = (self.budget_root / 'ledger.json').read_bytes()
        self.run_protocol(execute=False)
        self.assertEqual(before, (self.budget_root / 'ledger.json').read_bytes())

    def test_output_directory_never_overwritten(self):
        self.run_protocol(execute=False)
        with self.assertRaises(FileExistsError):
            self.run_protocol(execute=False)
        self.assertEqual(self.state()['status'], 'preparing')

    def test_missing_frozen_prerequisite_blocks_before_worker(self):
        self.manifest_value['prerequisites']['complete_numeric_reference'] = False
        self.save_manifest()
        with self.assertRaisesRegex(budget.BudgetError, 'Missing preparation prerequisites'):
            self.run_protocol()
        self.assertEqual(self.workers, [])
        self.assertEqual(self.state()['status'], 'preparing')

    def test_unknown_manifest_field_refused_without_execution(self):
        self.manifest_value['runExtraHistories'] = True
        self.save_manifest()
        with self.assertRaisesRegex(runner.RunnerError, 'unknown protocol fields'):
            self.run_protocol()
        self.assertEqual(self.workers, [])

    def test_changed_frozen_input_blocks_before_worker(self):
        self.program.write_text('{"changed":true}')
        with self.assertRaisesRegex(budget.BudgetError, 'hash mismatch'):
            self.run_protocol()
        self.assertEqual(self.workers, [])

    def test_full_synthetic_protocol_one_worker_and_separate_copy_accounting(self):
        result = self.run_protocol()
        self.assertEqual(len(self.workers), 1)
        self.assertEqual(result['status'], 'protocol-completed')
        self.assertEqual([e['history'] for e in result['histories']], budget.HISTORY_ORDER)
        self.assertEqual(result['budget']['counters']['reservedNativeSteps'], 16 + 4 * 960 + 3 * 1920)
        self.assertEqual(result['budget']['counters']['actualNativeStepsKnown'], 16 + 4 * 960 + 3 * 1920)
        self.assertFalse(result['fullRecoveryQualified'])
        self.assertFalse(result['fullHistoryAuthorized'])
        requests = [e for e in self.workers[0].sent if e['op'] == 'history']
        self.assertEqual([e['headingDegrees'] for e in requests], [0, 0, 0, 60])
        self.assertEqual([e['enabled'] for e in requests], [False, True, True, True])
        self.assertEqual(len([e for e in self.workers[0].sent if e['op'] == 'continue-history']), 3)
        self.assertIsNone(self.workers[0].stop_reason)

    def test_failed_A_local_diagnostic_still_compares_B(self):
        self.settings['local_success'] = {'A': False}
        result = self.run_protocol()
        self.assertEqual(len(result['histories']), 4)
        self.assertFalse(result['histories'][0]['localSuccess'])

    def test_failed_B_local_diagnostic_closes_without_repeats(self):
        self.settings['local_success'] = {'B': False}
        result = self.run_protocol()
        self.assertEqual([e['history'] for e in result['histories']], ['A', 'B'])
        self.assertEqual(result['budget']['status'], 'closed-completed')

    def test_wrong_worker_pid_blocks_before_native_reservation(self):
        self.settings['ready_pid'] = 999
        self.expect_failure('PID does not match')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 0)

    def test_wrong_verification_step_count_keeps_reservation(self):
        self.settings['verify_steps'] = 15
        self.expect_failure('exactly the 16')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16)
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 15)
        payload = json.loads((self.budget_root / 'ledger.json').read_text())['payload']
        entry = payload['reservations'][-1]
        self.assertEqual(entry['status'], 'failed-reported')
        self.assertEqual(entry['outcome']['evidence']['sha256'], budget.sha(self.output / '000-native-verification.json'))

    def test_missing_parity_gate_stops_before_histories(self):
        self.settings['outcome_overrides'] = {'verify': {'prediction_state_and_motor_semantics_verified': False}}
        self.expect_failure('Native diagnostic failed')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16)

    def test_unknown_result_fields_refused(self):
        self.settings['extra_result_field'] = True
        self.expect_failure('unknown protocol fields')

    def test_worker_evidence_cannot_escape_assigned_output(self):
        self.settings['outside_evidence'] = self.program
        self.expect_failure('evidence path differs')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16)

    def test_corrupted_budget_stops_and_still_archives_failure_receipt(self):
        self.settings['corrupt_budget'] = self.budget_root / 'ledger.json'
        with self.assertRaisesRegex(runner.RunnerError, 'Malformed ledger'):
            self.run_protocol()
        result = json.loads((self.output / 'run-result.json').read_text())
        self.assertEqual(result['status'], 'failed-closed')
        self.assertEqual(result['budget']['status'], 'unreadable-fail-closed')
        self.assertFalse(result['budget']['authorizesIntegration'])
        self.assertIsNotNone(self.workers[0].stop_reason)

    def test_native_input_mutation_retains_reserved_verification(self):
        self.settings['mutate_frozen'] = True
        self.expect_failure('Frozen input changed')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16)

    def test_A_cannot_request_planning(self):
        self.settings['A_requests_planning'] = True
        self.expect_failure('differs from the frozen')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16 + 960)

    def test_worker_cannot_change_candidate_count(self):
        self.settings['long_steps'] = [480] * 5
        self.expect_failure('differs from the frozen')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16 + 2 * 960)

    def test_worker_cannot_request_41st_planning_call(self):
        self.settings['planning_calls'] = 41
        self.expect_failure('Forty planning-call cap')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16 + 2 * 960 + 40 * 1920)

    def test_failed_planning_never_receives_history_continuation_ack(self):
        self.settings['outcome_overrides'] = {'planning': {'predictions_usable': False}}
        self.settings['planning_steps'] = 123
        self.expect_failure('predictions_usable')
        self.assertFalse(any(e['op'] == 'continue-history' for e in self.workers[0].sent))
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16 + 2 * 960 + 1920)
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 16 + 960 + 123)

    def test_partial_failed_history_archives_actual_and_refuses_next_history(self):
        self.settings['outcome_overrides'] = {'history': {'support_feasible': False}}
        self.settings['history_steps'] = 31
        self.expect_failure('support_feasible')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16 + 960)
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 16 + 31)
        self.assertEqual(len([m for m in self.workers[0].sent if m['op'] == 'history']), 1)

    def test_malformed_failed_result_is_not_accepted_as_known_actual(self):
        self.settings['outcome_overrides'] = {'verify': {'predictions_usable': 'false'}}
        self.expect_failure('explicit boolean')
        self.assertEqual(self.state()['counters']['reservedNativeSteps'], 16)
        self.assertEqual(self.state()['counters']['actualNativeStepsKnown'], 0)

    def test_shutdown_failure_is_archived_and_not_reported_success(self):
        self.settings['stop_raises'] = True
        with self.assertRaisesRegex(runner.RunnerError, 'shutdown failed'):
            self.run_protocol()
        result = json.loads((self.output / 'run-result.json').read_text())
        self.assertEqual(result['status'], 'failed-closed')
        self.assertTrue(self.state()['status'].startswith('closed'))


class PipeTransportTests(unittest.TestCase):
    """Inert pipes only: bounded write and partial-write handling, no subprocess."""
    def setUp(self):
        read_fd, write_fd = os.pipe()
        self.reader = os.fdopen(read_fd, 'rb', buffering=0)
        self.writer = os.fdopen(write_fd, 'wb', buffering=0)
        self.addCleanup(self.reader.close)
        self.addCleanup(self.writer.close)
        os.set_blocking(write_fd, False)
        self.worker = runner.NativeWorker.__new__(runner.NativeWorker)
        self.worker.process = types.SimpleNamespace(stdin=self.writer)
        self.worker.guard = lambda: None

    def test_send_handles_partial_os_writes(self):
        original = os.write
        with mock.patch.object(runner.os, 'write', side_effect=lambda fd, data: original(fd, data[:7])):
            self.worker.send({'op': 'synthetic-no-physics'})
        self.assertEqual(json.loads(self.reader.read(4096)), {'op': 'synthetic-no-physics'})

    def test_send_to_nonreading_full_pipe_times_out(self):
        while True:
            try:
                os.write(self.writer.fileno(), b'x' * 4096)
            except BlockingIOError:
                break
        started = time.monotonic()
        with self.assertRaisesRegex(runner.RunnerError, 'stdin timed out'):
            self.worker.send({'op': 'synthetic-no-physics',
                              'permit': {'deadlineMonotonic': started + 0.02}})
        self.assertLess(time.monotonic() - started, 0.5)


class ProcessSupervisionTests(unittest.TestCase):
    """Mock process metadata and signal delivery only; no subprocess or physics."""
    def setUp(self):
        self.worker = runner.NativeWorker.__new__(runner.NativeWorker)
        self.worker.pid = 42
        self.worker.binary = Path('/frozen/inert-worker')
        self.worker.binary_sha256 = 'synthetic-sha'
        self.worker.identity = None
        self.identity = {'pid': 42, 'threads': 1, 'children': '', 'startTicks': '10',
                         'binaryPath': '/frozen/inert-worker', 'binarySha256': 'synthetic-sha'}
        patch = mock.patch.object(runner.supervision, 'inspect_process', return_value=self.identity)
        self.inspector = patch.start()
        self.addCleanup(patch.stop)
        patch = mock.patch.object(runner.os, 'getpgid', return_value=42)
        self.pgid = patch.start()
        self.addCleanup(patch.stop)

    def test_one_thread_no_descendant_and_stable_identity_are_required(self):
        self.worker.guard()
        self.assertEqual(self.worker.identity, self.identity)
        self.inspector.return_value = dict(self.identity, startTicks='11')
        with self.assertRaisesRegex(runner.RunnerError, 'identity changed'):
            self.worker.guard()

    def test_child_process_refused(self):
        self.inspector.return_value = dict(self.identity, children='43')
        with self.assertRaisesRegex(runner.RunnerError, 'child'):
            self.worker.guard()

    def test_extra_thread_refused(self):
        self.inspector.return_value = dict(self.identity, threads=2)
        with self.assertRaisesRegex(runner.RunnerError, 'thread'):
            self.worker.guard()

    def test_changed_binary_refused(self):
        self.inspector.return_value = dict(self.identity, binarySha256='changed')
        with self.assertRaisesRegex(runner.RunnerError, 'binary'):
            self.worker.guard()

    def test_missing_private_process_group_refused(self):
        self.pgid.return_value = 2
        with self.assertRaisesRegex(runner.RunnerError, 'contract violated'):
            self.worker.guard()

    def test_kill_reuses_whole_process_group_cleanup(self):
        with mock.patch.object(runner.supervision, 'kill_pid_group') as kill:
            self.worker.kill()
        kill.assert_called_once_with(42)

    def test_error_stop_kills_before_wait_and_disposes_even_after_exit(self):
        self.worker.process = mock.Mock()
        self.worker.process.poll.return_value = 0
        self.worker.process.returncode = 0
        events = []
        self.worker.kill = lambda: events.append('kill-group')
        self.worker.dispose = lambda: events.append('dispose')
        self.worker.process.stdin.close.side_effect = lambda: events.append('close-stdin')
        self.assertEqual(self.worker.stop('synthetic receipt failure'), 0)
        self.assertEqual(events, ['kill-group', 'close-stdin', 'dispose'])

    def test_sigterm_during_constructor_kills_new_group_and_restores_handlers(self):
        handlers = {}
        def install(sig, handler):
            handlers[sig] = handler
        with mock.patch.object(runner.supervision.signal, 'signal', side_effect=install), \
             mock.patch.object(runner.supervision.signal, 'getsignal', return_value=signal.SIG_DFL), \
             mock.patch.object(runner.supervision.signal, 'getitimer', return_value=(0.0, 0.0)), \
             mock.patch.object(runner.supervision.signal, 'setitimer') as timer, \
             mock.patch.object(runner.supervision, 'child_pids', side_effect=[{7}, {7, 8}, {7, 8}]), \
             mock.patch.object(runner.supervision, 'kill_pid_group') as kill:
            with self.assertRaisesRegex(runner.RunnerError, 'SIGTERM'):
                with runner.hard_deadline(time.monotonic() + 7137, lambda: None):
                    handlers[signal.SIGTERM](signal.SIGTERM, None)
            self.assertEqual(kill.call_args_list, [mock.call(8), mock.call(8)])
            self.assertEqual(timer.call_args_list[-1], mock.call(signal.ITIMER_REAL, 0))
            self.assertTrue(all(handler == signal.SIG_DFL for handler in handlers.values()))

    def test_alarm_kills_assigned_worker_and_has_current_protocol_message(self):
        handlers = {}
        worker = mock.Mock()
        def install(sig, handler):
            handlers[sig] = handler
        with mock.patch.object(runner.supervision.signal, 'signal', side_effect=install), \
             mock.patch.object(runner.supervision.signal, 'getsignal', return_value=signal.SIG_DFL), \
             mock.patch.object(runner.supervision.signal, 'getitimer', return_value=(0.0, 0.0)), \
             mock.patch.object(runner.supervision.signal, 'setitimer'), \
             mock.patch.object(runner.supervision, 'child_pids', return_value=set()):
            with self.assertRaisesRegex(runner.RunnerError, 'Hard protocol deadline'):
                with runner.hard_deadline(time.monotonic() + 7137, lambda: worker):
                    handlers[signal.SIGALRM](signal.SIGALRM, None)
            worker.kill.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
