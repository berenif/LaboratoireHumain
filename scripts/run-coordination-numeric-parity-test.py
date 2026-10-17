#!/usr/bin/env python3
"""Synthetic protocol, fake-clock and inert-pipe tests. No native worker runs."""
from contextlib import contextmanager
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import time
import types
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('numeric_parity_runner',
    Path(__file__).with_name('run-coordination-numeric-parity.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeClock:
    elapsed = 0
    def wall(self):
        return 1791873840 + self.elapsed
    def monotonic(self):
        return 100 + self.elapsed


@contextmanager
def no_alarm(_deadline, _worker):
    yield


class FakeWorker:
    def __init__(self, test):
        self.test, self.pid = test, 424242
        self.ready = True
        self.sent = []
        self.killed = False
        self.disposed = False
        self.response = None

    def receive(self, deadline, guard):
        guard()
        if self.ready:
            self.ready = False
            return {'event': 'ready', 'pid': self.test.options.get('ready_pid', self.pid)}
        self.test.clock.elapsed += self.test.options.get('elapsed_on_receive', 0)
        guard()
        if self.test.options.get('crash'):
            raise runner.Refusal('synthetic crash before receipt')
        return self.response

    def send(self, message, deadline, guard):
        guard()
        self.sent.append(message)
        assert message['op'] == 'run'
        assert message['permit']['reservedSteps'] == 44
        actual = self.test.options.get('actual', 44)
        outcome = dict.fromkeys(runner.GATES, True)
        outcome.update(self.test.options.get('outcome', {}))
        report = {'actualNativeSteps': actual, 'outcome': outcome}
        if self.test.options.get('contradict_evidence'):
            report['actualNativeSteps'] = 1
        if self.test.options.get('symlink_evidence'):
            Path(message['outputPath']).symlink_to(self.test.fixture)
        else:
            runner.write_new(Path(message['outputPath']), report)
        if not self.test.options.get('missing_trace'):
            Path(message['tracePath']).write_text('{"synthetic":true}\n')
        self.response = {'event': 'result', 'op': self.test.options.get('op', 'numeric-parity'),
            'actualNativeSteps': actual, 'outcome': outcome,
            'evidencePath': self.test.options.get('evidence_path', message['outputPath'])}
        if self.test.options.get('extra_result'):
            self.response['retry'] = True
        if self.test.options.get('mutate_old'):
            self.test.old[0].write_text('mutated externally for synthetic test')
        if self.test.options.get('mutate_manifest'):
            self.test.manifest.write_text('{}')

    def finish(self, deadline):
        self.test.clock.elapsed += self.test.options.get('elapsed_on_finish', 0)
        return self.test.options.get('exit', 0)

    def kill(self):
        self.killed = True

    def dispose(self):
        self.kill()
        self.disposed = True


class RunnerTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='numeric-parity-pure-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.clock = FakeClock()
        self.options, self.workers = {}, []
        self.binary = self.root / 'inert-not-executable-code'
        self.binary.write_text('THIS FILE IS NEVER EXECUTED\n')
        self.binary.chmod(0o700)
        self.old = []
        for index in range(7):
            path = self.root / f'old-{index}.json'
            path.write_text(json.dumps({'old': index}) + '\n')
            self.old.append(path)
        self.original_old = [p.read_bytes() for p in self.old]
        self.preparation = self.root / 'preparation.json'
        runner.write_new(self.preparation, {'approvedNativeSubsteps': 44,
            'approvedNativeWallSeconds': 120, 'nativeExecutionStarted': False,
            'oldRegisters': [self.record(path) for path in self.old[:6]],
            'originalReleaseWorker': self.record(self.old[-1])})
        self.value = {'schema': runner.SCHEMA, 'authorizationId': runner.AUTHORIZATION_ID,
            'outputPath': str(self.root / 'native-run'), 'binaryPath': str(self.binary),
            'inputs': {}, 'files': [],
            'prerequisites': dict.fromkeys(runner.PREREQUISITES, True)}
        for role in sorted(runner.ROLES):
            if role == 'binary':
                path = self.binary
            elif role == 'code':
                path = Path(runner.__file__).resolve()
            elif role == 'preparation':
                path = self.preparation
            else:
                path = self.root / f'{role}.json'
                path.write_text(json.dumps({'synthetic': role}) + '\n')
            self.value['files'].append(dict(self.record(path), role=role))
            if role in ('fixture', 'program', 'tasks'):
                self.value['inputs'][role + 'Path'] = str(path)
        self.fixture = Path(self.value['inputs']['fixturePath'])
        self.manifest = self.root / 'manifest.json'
        self.save_manifest()
        patch = mock.patch.object(runner.subprocess, 'Popen', side_effect=AssertionError('NO SUBPROCESS IN PURE TESTS'))
        patch.start()
        self.addCleanup(patch.stop)

    def record(self, path):
        return {'path': str(path), 'sha256': runner.sha(path)}

    def save_manifest(self):
        self.manifest.write_text(json.dumps(self.value) + '\n')

    def factory(self, binary, output):
        self.assertTrue((self.root / 'authorization-claim.json').is_file())
        ledger = json.loads((output / 'physical-ledger.json').read_text())['payload']
        self.assertEqual(ledger['reservedNativeSteps'], 44)
        self.assertEqual(ledger['status'], 'reserved')
        self.assertEqual(ledger['deadlineMonotonic'], 220)
        if self.options.get('spawn_failure'):
            raise OSError('synthetic spawn failure')
        worker = FakeWorker(self)
        self.workers.append(worker)
        return worker

    def inspector(self, pid):
        return {'pid': pid, 'threads': self.options.get('threads', 1),
            'children': self.options.get('children', ''),
            'startTicks': '124' if self.options.get('change_identity') and self.workers[0].sent else '123',
            'binaryPath': str(self.binary), 'binarySha256': runner.sha(self.binary)}

    def run_batch(self, execute=True):
        return runner.run(self.manifest, execute=execute, batch_root=self.root,
            clock=self.clock, worker_factory=self.factory, inspector=self.inspector,
            alarm_factory=no_alarm)

    def ledger(self):
        envelope = json.loads((self.root / 'native-run/physical-ledger.json').read_text())
        self.assertEqual(envelope['sha256'], runner.hashlib.sha256(runner.canonical(envelope['payload'])).hexdigest())
        return envelope['payload']

    def failure(self, pattern):
        with self.assertRaisesRegex(runner.Refusal, pattern):
            self.run_batch()
        ledger = self.ledger()
        self.assertEqual(ledger['status'], 'closed-failure')
        self.assertEqual(ledger['reservedNativeSteps'], 44)
        self.assertEqual(ledger['refundNativeSteps'], 0)
        if self.workers:
            self.assertTrue(self.workers[0].killed)
            self.assertTrue(self.workers[0].disposed)
        return ledger

    def test_default_is_read_only_and_never_claims_or_spawns(self):
        result = self.run_batch(False)
        self.assertEqual(result['physicalIntegrations'], 0)
        self.assertFalse((self.root / 'authorization-claim.json').exists())
        self.assertFalse((self.root / 'native-run').exists())
        self.assertEqual(self.workers, [])

    def test_full44_reserved_before_one_worker_and_old_files_unchanged(self):
        result = self.run_batch()
        self.assertEqual(result['status'], 'closed-success')
        self.assertEqual(self.ledger()['actualNativeStepsKnown'], 44)
        self.assertEqual(len(self.workers), 1)
        self.assertEqual(len(self.workers[0].sent), 1)
        self.assertEqual(self.original_old, [p.read_bytes() for p in self.old])

    def test_claim_prevents_second_run_even_with_new_output(self):
        self.run_batch()
        self.value['outputPath'] = str(self.root / 'another-run')
        self.save_manifest()
        with self.assertRaises(FileExistsError):
            self.run_batch()
        self.assertEqual(len(self.workers), 1)

    def test_second_run_same_directory_is_refused(self):
        self.run_batch()
        with self.assertRaisesRegex(runner.Refusal, 'NEW direct child'):
            self.run_batch()
        self.assertEqual(len(self.workers), 1)

    def test_existing_output_refused_before_any_claim(self):
        (self.root / 'native-run').mkdir()
        with self.assertRaisesRegex(runner.Refusal, 'NEW direct child'):
            self.run_batch()
        self.assertFalse((self.root / 'authorization-claim.json').exists())

    def test_false_prerequisite_has_no_reservation(self):
        self.value['prerequisites']['static_tests_passed'] = False
        self.save_manifest()
        with self.assertRaisesRegex(runner.Refusal, 'prerequisite'):
            self.run_batch()
        self.assertEqual(self.workers, [])

    def test_unavailable_process_interface_refuses_before_claim(self):
        with mock.patch.object(runner, 'PROC', self.root / 'missing-proc'):
            with self.assertRaises(FileNotFoundError):
                self.run_batch()
        self.assertFalse((self.root / 'authorization-claim.json').exists())
        self.assertFalse((self.root / 'native-run').exists())
        self.assertEqual(self.workers, [])

    def test_denied_process_interface_refuses_before_claim(self):
        with mock.patch.object(runner, 'process_stat', side_effect=PermissionError('access denied')):
            with self.assertRaises(PermissionError):
                self.run_batch()
        self.assertFalse((self.root / 'authorization-claim.json').exists())
        self.assertFalse((self.root / 'native-run').exists())
        self.assertEqual(self.workers, [])

    def test_input_hash_mutation_blocks_before_spawn(self):
        self.fixture.write_text('{}')
        with self.assertRaisesRegex(runner.Refusal, 'hash mismatch'):
            self.run_batch()
        self.assertEqual(self.workers, [])

    def test_manifest_unknown_field_refused(self):
        self.value['extraHistory'] = True
        self.save_manifest()
        with self.assertRaisesRegex(runner.Refusal, 'unknown fields'):
            self.run_batch()

    def test_parent_does_not_launch_original_worker(self):
        self.value['binaryPath'] = str(self.old[-1])
        self.old[-1].chmod(0o700)
        for record in self.value['files']:
            if record['role'] == 'binary':
                record.update(self.record(self.old[-1]))
        self.save_manifest()
        with self.assertRaisesRegex(runner.Refusal, 'distinct and preserved'):
            self.run_batch()

    def test_wrong_ready_pid_closes_full_reservation(self):
        self.options['ready_pid'] = 17
        self.assertIsNone(self.failure('Ready PID')['actualNativeStepsKnown'])
        self.assertEqual(self.workers[0].sent, [])

    def test_multiple_threads_refused_before_permit(self):
        self.options['threads'] = 2
        self.failure('thread')
        self.assertEqual(self.workers[0].sent, [])

    def test_child_process_refused_before_permit(self):
        self.options['children'] = '555'
        self.failure('child')
        self.assertEqual(self.workers[0].sent, [])

    def test_pid_identity_change_after_permit_refuses_result(self):
        self.options['change_identity'] = True
        self.assertIsNone(self.failure('identity changed')['actualNativeStepsKnown'])

    def test_spawn_failure_never_refunds_or_restarts(self):
        self.options['spawn_failure'] = True
        self.failure('spawn failure')
        self.assertEqual(self.workers, [])

    def test_prefix_failure_records12_and_never_continues(self):
        self.options.update(actual=12, outcome={'prefix_matches_archive': False, 'parity_verified': False})
        self.assertEqual(self.failure('Partial batch')['actualNativeStepsKnown'], 12)
        self.assertEqual(len(self.workers[0].sent), 1)

    def test_step_after_mismatching_prefix_is_integrity_failure(self):
        self.options.update(actual=13, outcome={'prefix_matches_archive': False})
        self.assertEqual(self.failure('after a mismatching prefix')['actualNativeStepsKnown'], 13)

    def test_failed_parity_preserves_partial_receipt(self):
        self.options.update(actual=20, outcome={'parity_verified': False})
        ledger = self.failure('Partial batch')
        self.assertEqual(ledger['actualNativeStepsKnown'], 20)
        self.assertEqual(ledger['evidence']['sha256'], runner.sha(self.root / 'native-run/native-result.json'))

    def test_partial_success_claim_refused(self):
        self.options['actual'] = 43
        self.assertEqual(self.failure('Partial batch')['actualNativeStepsKnown'], 43)

    def test_over44_cannot_be_accepted_as_bounded_actual(self):
        self.options['actual'] = 45
        self.assertIsNone(self.failure('Invalid actual')['actualNativeStepsKnown'])

    def test_bool_actual_is_not_a_step_count(self):
        self.options['actual'] = True
        self.assertIsNone(self.failure('Invalid actual')['actualNativeStepsKnown'])

    def test_diagnostic_strings_are_not_booleans(self):
        self.options['outcome'] = {'integrity_ok': 'true'}
        self.assertIsNone(self.failure('explicit booleans')['actualNativeStepsKnown'])

    def test_result_unknown_field_refused(self):
        self.options['extra_result'] = True
        self.failure('unknown fields')

    def test_request_for_another_operation_refused(self):
        self.options['op'] = 'history'
        self.failure('no additional operation')

    def test_evidence_outside_output_refused(self):
        self.options['evidence_path'] = str(self.fixture)
        self.failure('exact assigned')

    def test_evidence_symlink_refused(self):
        self.options['symlink_evidence'] = True
        self.failure('not a symlink')

    def test_contradictory_receipt_cannot_credit_steps(self):
        self.options['contradict_evidence'] = True
        self.assertIsNone(self.failure('contradicts')['actualNativeStepsKnown'])

    def test_missing_trace_preserves_known_actual_but_fails(self):
        self.options['missing_trace'] = True
        self.assertEqual(self.failure('regular')['actualNativeStepsKnown'], 44)

    def test_old_register_changed_by_worker_is_detected(self):
        self.options['mutate_old'] = True
        self.assertEqual(self.failure('hash mismatch')['actualNativeStepsKnown'], 44)

    def test_old_register_final_audit_also_runs_after_partial_failure(self):
        self.options.update(mutate_old=True, actual=12, outcome={'parity_verified': False})
        ledger = self.failure('Partial batch')
        self.assertFalse(ledger['terminalFrozenFilesVerified'])
        self.assertIn('Terminal frozen-file audit', ledger['reason'])
        self.assertEqual(ledger['actualNativeStepsKnown'], 12)

    def test_manifest_changed_during_run_is_detected(self):
        self.options['mutate_manifest'] = True
        self.assertEqual(self.failure('hash mismatch')['actualNativeStepsKnown'], 44)

    def test_timeout_kills_and_preserves_unknown_actual_full_charge(self):
        self.options['elapsed_on_receive'] = 120
        self.assertIsNone(self.failure('deadline')['actualNativeStepsKnown'])

    def test_shutdown_has_no_extra_physics_grace(self):
        self.options['elapsed_on_finish'] = 120
        self.assertEqual(self.failure('deadline')['actualNativeStepsKnown'], 44)

    def test_terminal_file_audit_cannot_report_success_past_deadline(self):
        original, calls = runner.verify_files, []
        def audit(prepared):
            original(prepared)
            calls.append(True)
            if len(calls) == 4:
                self.clock.elapsed += 120
        with mock.patch.object(runner, 'verify_files', side_effect=audit):
            self.assertEqual(self.failure('deadline')['actualNativeStepsKnown'], 44)

    def test_crash_never_refunds_or_restarts(self):
        self.options['crash'] = True
        self.assertIsNone(self.failure('synthetic crash')['actualNativeStepsKnown'])
        self.assertEqual(len(self.workers), 1)

    def test_nonzero_exit_is_failure_after_full_receipt(self):
        self.options['exit'] = 2
        self.assertEqual(self.failure('exit')['actualNativeStepsKnown'], 44)


class ProcessMetadataTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='inert-proc-metadata-')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        patch = mock.patch.object(runner, 'PROC', self.root)
        patch.start()
        self.addCleanup(patch.stop)

    def stat(self, pid, parent, comm='has spaces ) and parentheses'):
        directory = self.root / str(pid)
        directory.mkdir()
        fields = ['S', str(parent)] + ['0'] * 17 + ['123456']
        (directory / 'stat').write_text(f'{pid} ({comm}) ' + ' '.join(fields))

    def test_ppid_census_needs_no_children_interface(self):
        self.stat(10, 1)
        self.stat(11, 10)
        self.stat(12, 10)
        self.stat(13, 11)
        self.assertEqual(runner.child_pids(10), {11, 12})
        self.assertEqual(runner.process_stat(10)[19], '123456')

    def test_disappearing_pid_is_skipped(self):
        self.stat(10, 1)
        (self.root / '11').mkdir()
        self.assertEqual(runner.child_pids(10), set())

    def test_missing_parent_is_not_an_empty_successful_census(self):
        self.stat(11, 1)
        with self.assertRaisesRegex(runner.Refusal, 'required parent PID'):
            runner.child_pids(10)

    def test_permission_error_is_never_ignored(self):
        self.stat(10, 1)
        with mock.patch.object(runner, 'process_stat', side_effect=PermissionError('denied')):
            with self.assertRaises(PermissionError):
                runner.child_pids(10)


class PipeTests(unittest.TestCase):
    def setUp(self):
        rfd, wfd = os.pipe()
        self.reader = os.fdopen(rfd, 'rb', buffering=0)
        self.writer = os.fdopen(wfd, 'wb', buffering=0)
        self.addCleanup(self.reader.close)
        self.addCleanup(self.writer.close)
        os.set_blocking(wfd, False)
        self.worker = runner.NativeWorker.__new__(runner.NativeWorker)
        self.worker.process = types.SimpleNamespace(stdin=self.writer)

    def test_partial_writes_preserve_entire_message(self):
        original = os.write
        with mock.patch.object(runner.os, 'write', side_effect=lambda fd, data: original(fd, data[:3])):
            self.worker.send({'op': 'inert-pure-test'}, time.monotonic() + 1, lambda: None)
        self.assertEqual(json.loads(self.reader.read(4096)), {'op': 'inert-pure-test'})

    def test_full_pipe_times_out_without_spawn_or_physics(self):
        while True:
            try:
                os.write(self.writer.fileno(), b'x' * 4096)
            except BlockingIOError:
                break
        with self.assertRaisesRegex(runner.Refusal, 'deadline'):
            self.worker.send({'op': 'inert'}, time.monotonic() + 0.01, lambda: None)

    def test_protocol_duplicate_keys_are_refused(self):
        with self.assertRaisesRegex(runner.Refusal, 'Duplicate'):
            runner.decode_json(b'{"actualNativeSteps":1,"actualNativeSteps":44}')

    def test_protocol_nonfinite_number_is_refused(self):
        with self.assertRaisesRegex(runner.Refusal, 'Nonfinite'):
            runner.decode_json(b'{"actualNativeSteps":NaN}')


class WatchdogTests(unittest.TestCase):
    """Mock signals and PIDs only; no process and no real alarm is created."""
    def test_timeout_during_constructor_kills_only_new_child(self):
        handlers = {}
        def set_handler(signum, handler):
            handlers[signum] = handler
        with mock.patch.object(runner.signal, 'getitimer', return_value=(0.0, 0.0)), \
             mock.patch.object(runner.signal, 'setitimer'), \
             mock.patch.object(runner.signal, 'signal', side_effect=set_handler), \
             mock.patch.object(runner, 'child_pids', side_effect=[{7}, {7, 8}, {7, 8}]), \
             mock.patch.object(runner, 'kill_pid_group') as killed:
            with self.assertRaisesRegex(runner.Refusal, 'process group killed'):
                with runner.hard_deadline(time.monotonic() + 120, lambda: None):
                    handlers[runner.signal.SIGALRM](runner.signal.SIGALRM, None)
            self.assertTrue(killed.called)
            self.assertEqual({call.args[0] for call in killed.call_args_list}, {8})

    def test_constructor_failure_also_kills_new_child_without_worker_reference(self):
        with mock.patch.object(runner.signal, 'getitimer', return_value=(0.0, 0.0)), \
             mock.patch.object(runner.signal, 'setitimer'), \
             mock.patch.object(runner.signal, 'signal'), \
             mock.patch.object(runner, 'child_pids', side_effect=[{7}, {7, 8}]), \
             mock.patch.object(runner, 'kill_pid_group') as killed:
            with self.assertRaisesRegex(RuntimeError, 'synthetic constructor'):
                with runner.hard_deadline(time.monotonic() + 120, lambda: None):
                    raise RuntimeError('synthetic constructor')
            killed.assert_called_once_with(8)


if __name__ == '__main__':
    unittest.main()
