#!/usr/bin/env python3
"""Runner protocol tests with an in-memory fake worker. Never launches a process.

Only dummy JSON files, fake accounting receipts and inert OS pipes are used.
Reported native-step values below are synthetic protocol data, not integrations.
"""

import importlib.util
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
        path.write_text(json.dumps({'synthetic': True, 'neverIntegratedPhysics': True,
                                    'operation': operation, 'actualNativeSteps': actual}) + '\n')
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


if __name__ == '__main__':
    unittest.main()
