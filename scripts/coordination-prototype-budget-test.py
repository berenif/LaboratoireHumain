#!/usr/bin/env python3
"""Synthetic budget tests only: fake clock/process, no native executable or world."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location('coordination_budget',
    Path(__file__).with_name('coordination-prototype-budget.py'))
budget = importlib.util.module_from_spec(spec)
spec.loader.exec_module(budget)


class FakeClock:
    def __init__(self):
        self.value = {'wall': 1791835200.0, 'monotonic': 1000.0, 'boot': 'synthetic-boot'}

    def sample(self):
        return dict(self.value)

    def advance(self, seconds):
        self.value['wall'] += seconds
        self.value['monotonic'] += seconds


class BudgetTests(unittest.TestCase):
    def test_unreported_native_reservation_is_never_an_exact_zero(self):
        self.manifest['protocolMode'] = budget.LOCAL_PAIR_MODE
        self.manifest['prerequisites'].pop('four_histories_frozen')
        self.manifest['prerequisites']['local_pair_frozen'] = True
        self.ledger.freeze(self.manifest)
        check = self.ledger.reserve('verification', native_steps=16, worker_pid=42)
        self.ledger.close('Synthetic disconnect; no native process was started')
        counts = self.ledger.status()['counters']
        self.assertEqual(counts['actualNativeStepsKnown'], 0)
        self.assertFalse(counts['actualNativeStepsExact'])
        self.assertEqual(counts['unreportedReservedNativeSteps'], 16)
        self.assertEqual(counts['actualNativeStepsLowerBound'], 0)
        self.assertEqual(counts['actualNativeStepsUpperBound'], 16)
        self.assertEqual(self.state()['reservations'][0]['id'], check['id'])

    def test_completed_receipt_restores_exact_native_count_without_refund(self):
        self.local_pair_ready()
        counts = self.ledger.status()['counters']
        self.assertTrue(counts['actualNativeStepsExact'])
        self.assertEqual(counts['unreportedReservedNativeSteps'], 0)
        self.assertEqual(counts['actualNativeStepsLowerBound'], 16)
        self.assertEqual(counts['actualNativeStepsUpperBound'], 16)
        self.assertEqual(counts['reservedNativeSteps'], 16)

    def local_pair_manifest(self):
        self.manifest['protocolMode'] = budget.LOCAL_PAIR_MODE
        self.manifest['prerequisites'].pop('four_histories_frozen')
        self.manifest['prerequisites']['local_pair_frozen'] = True

    def local_pair_ready(self):
        self.local_pair_manifest()
        self.ledger.freeze(self.manifest)
        check = self.ledger.reserve('verification', native_steps=16, worker_pid=42)
        self.finish(check)
        self.ledger.admit_native_evidence(self.native_evidence(check))

    def test_local_pair_full_allocation_closes_after_successful_b(self):
        self.local_pair_ready()
        initial_deadline = dict(self.state()['physics'])
        self.assertEqual(initial_deadline['deadlineMonotonic'] - self.clock.value['monotonic'], 7100)
        self.assertEqual(budget.LIMITS['physicsWallSeconds'], 7200)
        self.clock.advance(5)
        self.finish(self.history('A'), local=False)
        b = self.history('B')
        for i in range(40):
            plan = self.ledger.reserve('planning', history_id=b['id'], history_step=i * 24,
                                       long_steps=[480] * 4, short_probes=0)
            self.finish(plan)
        self.finish(b, local=True)
        state = self.state()
        self.assertEqual(state['status'], 'closed-completed')
        self.assertEqual(state['limits'], budget.LOCAL_PAIR_LIMITS)
        self.assertEqual(state['physics'], initial_deadline)
        self.assertEqual(budget.Budget.counters(state)['reservedNativeSteps'], 78736)
        self.assertEqual(budget.Budget.counters(state)['actualNativeStepsKnown'], 78736)
        self.assertEqual(budget.Budget.counters(state)['reservedPredictionAndVerificationSteps'], 76816)
        self.assertEqual(budget.LIMITS['totalNativeSteps'], 300000)
        before = (self.root / 'ledger.json').read_bytes()
        self.refused('Protocol is closed', lambda: self.history('B-repeat'), closed=False)
        self.assertEqual((self.root / 'ledger.json').read_bytes(), before)

    def test_local_pair_failed_b_closes_without_refund(self):
        self.local_pair_ready()
        self.finish(self.history('A'), local=False)
        self.finish(self.history('B'), local=False, actual=12)
        self.assertEqual(self.state()['status'], 'closed-completed')
        self.assertEqual(self.ledger.status()['counters']['reservedNativeSteps'], 1936)
        self.assertEqual(self.ledger.status()['counters']['actualNativeStepsKnown'], 988)

    def test_local_pair_requires_its_frozen_prerequisite(self):
        self.manifest['protocolMode'] = budget.LOCAL_PAIR_MODE
        self.refused('local_pair_frozen', lambda: self.ledger.freeze(self.manifest))
        self.assertIsNone(self.state()['physics'])

    def test_local_pair_mode_cannot_drift_from_frozen_manifest(self):
        self.local_pair_ready()
        state = self.state()
        state['protocolMode'] = budget.DEFAULT_PROTOCOL_MODE
        state['limits'] = dict(budget.LIMITS)
        budget.durable_save(self.root / 'ledger.json', state)
        self.refused('Frozen protocolMode changed', lambda: self.history('A'))

    def test_local_pair_refuses_imported_or_44_step_verification(self):
        self.local_pair_manifest()
        self.ledger.freeze(self.manifest)
        self.refused('exactly 16', lambda: self.ledger.reserve('verification', native_steps=44, worker_pid=42))
        self.assertEqual(self.ledger.status()['counters']['reservedNativeSteps'], 0)

    def test_local_pair_verification_must_complete_all_16(self):
        self.local_pair_manifest()
        self.ledger.freeze(self.manifest)
        check = self.ledger.reserve('verification', native_steps=16, worker_pid=42)
        self.refused('complete exactly 16', lambda: self.finish(check, actual=15))
        self.assertEqual(self.ledger.status()['counters']['reservedNativeSteps'], 16)

    def test_local_pair_requires_its_own_same_ledger_verification(self):
        self.local_pair_manifest()
        self.ledger.freeze(self.manifest)
        imported = {'id': 'foreign-44-step-receipt'}
        self.refused('unique completed charged', lambda: self.ledger.admit_native_evidence(self.native_evidence(imported)))
        self.assertIsNone(self.state()['physics'])

    def test_local_pair_refuses_short_probes(self):
        self.local_pair_ready()
        self.finish(self.history('A'))
        b = self.history('B')
        self.refused('short_probes', lambda: self.ledger.reserve('planning', history_id=b['id'],
                     history_step=0, long_steps=[480] * 4, short_probes=1))

    def test_local_pair_refuses_different_preview_count(self):
        self.local_pair_ready()
        self.finish(self.history('A'))
        b = self.history('B')
        self.refused('four 480-step', lambda: self.ledger.reserve('planning', history_id=b['id'],
                     history_step=0, long_steps=[480] * 3, short_probes=0))

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='coordination-budget-test-')
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.root = self.directory / 'new-run'
        self.clock = FakeClock()
        self.frozen = self.directory / 'synthetic-input.txt'
        self.frozen.write_text('Synthetic file; contains no executable or physical state.\n')
        self.digest = budget.sha(self.frozen)
        self.report = self.directory / 'synthetic-native-receipt.json'
        self.report.write_text('{"synthetic":true,"neverIntegratedPhysics":true}\n')
        self.report_digest = budget.sha(self.report)
        self.identity = {'pid': 42, 'startTicks': '123', 'boot': 'synthetic-boot',
                         'binarySha256': self.digest, 'threads': 1}
        self.manifest = {'schema': 1,
                         'files': {k: [{'path': str(self.frozen), 'sha256': self.digest}]
                                   for k in budget.REQUIRED_ROLES},
                         'prerequisites': {k: True for k in budget.PREREQUISITES}}
        self.ledger = budget.Budget(self.root, clock=self.clock,
                                    process_inspector=lambda pid: dict(self.identity, pid=pid))
        self.ledger.init_preparation()

    def state(self):
        return json.loads((self.root / 'ledger.json').read_text())['payload']

    def ready(self):
        self.ledger.freeze(self.manifest)
        self.preflight = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.finish(self.preflight)
        self.ledger.admit_native_evidence(self.native_evidence(self.preflight))

    def native_evidence(self, entry):
        return {'schema': 1, 'gates': {k: True for k in budget.NATIVE_GATES},
                'verifications': [{'reservation_id': entry['id'], 'path': str(self.report),
                                   'sha256': self.report_digest}]}

    def outcome(self, local=True, evidence=False):
        value = {k: True for k in ('integrity_ok', 'support_feasible', 'predictions_usable',
                                   'diagnostic_present')}
        if evidence:
            value.update(local_success=local, evidence={'path': str(self.report), 'sha256': self.report_digest})
        return value

    def finish(self, entry, local=True, actual=None):
        return self.ledger.complete(entry['id'], actual_native_steps=entry['nativeSteps'] if actual is None else actual,
                                    outcome=self.outcome(local, entry['kind'] in ('history', 'verification')))

    def history(self, name):
        return self.ledger.reserve('history', history=name, worker_pid=42)

    def b_history(self):
        self.ready()
        self.finish(self.history('A'), local=False)
        return self.history('B')

    def refused(self, expression, call, closed=True):
        with self.assertRaisesRegex(budget.BudgetError, expression):
            call()
        if closed:
            self.assertEqual(self.state()['status'], 'closed-refused')

    def test_new_directory_required_previous_ledgers_untouched(self):
        previous = self.directory / 'old-ledger.json'
        previous.write_text('{"status":"closed","reserved":9999}\n')
        old = previous.read_bytes()
        with self.assertRaises(FileExistsError):
            self.ledger.init_preparation()
        self.assertEqual(previous.read_bytes(), old)
        self.assertEqual(self.state()['status'], 'preparing')

    def test_prior_preparation_debit_is_explicit_and_retained(self):
        root = self.directory / 'second-synthetic-allocation'
        ledger = budget.Budget(root, clock=self.clock)
        state = ledger.init_preparation(started_at_utc=budget.utc(self.clock.value['wall'] - 60),
                                         initial_active_seconds=60, initial_inspections=2)
        self.assertEqual(budget.Budget.counters(state)['staticInspections'], 2)
        self.assertEqual(budget.Budget.counters(state)['preparationActiveSeconds'], 60)

    def test_preparation_reservation_never_refunds(self):
        entry = self.ledger.reserve('preparation', active_seconds=60, inspections=100)
        self.clock.advance(10)
        self.ledger.complete(entry['id'], actual_inspections=2)
        counts = budget.Budget.counters(self.state())
        self.assertEqual(counts['preparationActiveSeconds'], 60)
        self.assertEqual(counts['staticInspections'], 100)
        self.ready()
        self.assertEqual(self.state()['status'], 'frozen-ready')

    def test_extend_active_preparation_keeps_clock_and_initial_debits(self):
        entry = self.ledger.reserve('preparation', active_seconds=7200, inspections=5000)
        initial = self.state()['preparation']
        self.clock.advance(600)
        extended = self.ledger.extend_preparation(entry['id'], additional_inspections=1921)
        self.assertEqual(extended['id'], entry['id'])
        self.assertEqual(extended['startedUtc'], entry['startedUtc'])
        self.assertEqual(extended['startedMonotonic'], entry['startedMonotonic'])
        self.assertEqual(extended['activeSeconds'], 7200)
        self.assertEqual(extended['inspections'], 6921)
        self.assertEqual(self.state()['preparation'], initial)
        self.assertEqual(budget.Budget.counters(self.state())['staticInspections'], 6921)

    def test_preparation_extension_respects_inspection_cap(self):
        entry = self.ledger.reserve('preparation', active_seconds=7200, inspections=19000)
        self.refused('inspection cap', lambda: self.ledger.extend_preparation(entry['id'],
                     additional_inspections=1001))
        self.assertEqual(budget.Budget.counters(self.state())['staticInspections'], 19000)

    def test_preparation_extension_respects_active_time_cap(self):
        entry = self.ledger.reserve('preparation', active_seconds=140000)
        self.refused('active-time cap', lambda: self.ledger.extend_preparation(entry['id'],
                     additional_inspections=1, additional_active_seconds=4001))

    def test_preparation_extension_cannot_renew_expired_reservation(self):
        entry = self.ledger.reserve('preparation', active_seconds=60, inspections=10)
        self.clock.advance(60)
        self.refused('cannot be renewed', lambda: self.ledger.extend_preparation(entry['id'],
                     additional_inspections=1, additional_active_seconds=1))
        self.assertEqual(budget.Budget.counters(self.state())['preparationActiveSeconds'], 60)

    def test_preparation_40_hour_cap(self):
        entry = self.ledger.reserve('preparation', active_seconds=144000, inspections=0)
        self.ledger.complete(entry['id'], actual_inspections=0)
        self.refused('active-time cap', lambda: self.ledger.reserve('preparation', active_seconds=1))

    def test_inspection_cap(self):
        entry = self.ledger.reserve('preparation', active_seconds=60, inspections=20000)
        self.ledger.complete(entry['id'], actual_inspections=10)
        self.refused('inspection cap', lambda: self.ledger.reserve('preparation', active_seconds=1, inspections=1))

    def test_preparation_five_day_deadline(self):
        self.clock.advance(5 * 86400)
        self.refused('five-day deadline', lambda: self.ready())

    def test_preparation_monotonic_deadline(self):
        self.clock.value['monotonic'] += 5 * 86400
        self.refused('five-day deadline', lambda: self.ready())

    def test_preparation_timeout_keeps_debit(self):
        entry = self.ledger.reserve('preparation', active_seconds=10, inspections=2)
        self.clock.advance(11)
        self.refused('preparation time exceeded', lambda: self.ledger.complete(entry['id'], actual_inspections=2))
        self.assertEqual(budget.Budget.counters(self.state())['staticInspections'], 2)

    def test_freeze_refuses_pending_preparation(self):
        self.ledger.reserve('preparation', active_seconds=60)
        self.refused('Unfinished preparation', lambda: self.ready())

    def test_missing_prerequisite_closes_before_physics(self):
        self.manifest['prerequisites']['continuation_to_standing_defined_and_verified'] = False
        self.refused('Missing preparation prerequisites', lambda: self.ready())
        self.assertIsNone(self.state()['physics'])
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 0)

    def test_missing_binary_closes_before_physics(self):
        del self.manifest['files']['binary']
        self.refused('Missing frozen role: binary', lambda: self.ready())

    def test_mismatched_freeze_hash(self):
        self.manifest['files']['reference'][0]['sha256'] = '0' * 64
        self.refused('hash mismatch', lambda: self.ready())

    def test_static_freeze_only_opens_verification_even_if_native_gates_claimed(self):
        self.manifest['prerequisites'].update({k: True for k in budget.NATIVE_GATES})
        result = self.ledger.freeze(self.manifest)
        self.assertEqual(result['status'], 'frozen-verification')
        self.assertIsNone(self.state()['physics'])
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 0)

    def test_history_requires_native_admission_after_static_freeze(self):
        self.ledger.freeze(self.manifest)
        self.refused('requires frozen-ready', lambda: self.history('A'))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 0)

    def test_planning_requires_native_admission_after_static_freeze(self):
        self.ledger.freeze(self.manifest)
        self.refused('requires frozen-ready', lambda: self.ledger.reserve('planning',
                     history_id='invented', history_step=0, worker_pid=42))

    def test_verification_is_guarded_and_charged_before_native_admission(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=240, worker_pid=42)
        self.assertEqual(self.ledger.guard(entry['id'])['nativeSteps'], 240)
        self.finish(entry)
        self.assertEqual(self.state()['status'], 'frozen-verification')
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 240)

    def test_native_admission_never_resets_counter_worker_or_clock(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=240, worker_pid=42)
        self.clock.advance(100)
        self.finish(entry)
        before = self.state()
        self.clock.advance(100)
        self.ledger.admit_native_evidence(self.native_evidence(entry))
        after = self.state()
        self.assertEqual(after['status'], 'frozen-ready')
        for field in ('preparation', 'physics', 'worker', 'manifestSha256', 'reservations'):
            self.assertEqual(after[field], before[field])
        self.assertEqual(budget.Budget.counters(after), budget.Budget.counters(before))

    def test_native_admission_requires_actual_charged_steps(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.finish(entry, actual=0)
        self.refused('completed charged native verification', lambda:
                     self.ledger.admit_native_evidence(self.native_evidence(entry)))

    def test_native_admission_refuses_unfinished_verification(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.refused('Unfinished reservation', lambda:
                     self.ledger.admit_native_evidence(self.native_evidence(entry)))

    def test_native_admission_requires_both_accounting_and_parity_gates(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.finish(entry)
        evidence = self.native_evidence(entry)
        evidence['gates']['prediction_state_and_motor_semantics_verified'] = False
        self.refused('prediction parity gate', lambda: self.ledger.admit_native_evidence(evidence))

    def test_native_admission_refuses_unknown_receipt(self):
        self.ledger.freeze(self.manifest)
        evidence = self.native_evidence({'id': 'never-reserved'})
        self.refused('completed charged native verification', lambda:
                     self.ledger.admit_native_evidence(evidence))

    def test_native_admission_refuses_duplicate_receipt(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.finish(entry)
        evidence = self.native_evidence(entry)
        evidence['verifications'] *= 2
        self.refused('unique completed', lambda: self.ledger.admit_native_evidence(evidence))

    def test_native_admission_refuses_modified_verification_receipt(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.finish(entry)
        self.report.write_text('modified synthetic receipt')
        self.refused('does not match completed verification', lambda:
                     self.ledger.admit_native_evidence(self.native_evidence(entry)))

    def test_native_admission_keeps_one_worker(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=1, worker_pid=42)
        self.finish(entry)
        self.identity['startTicks'] = 'another-process'
        self.refused('Worker changed', lambda: self.ledger.admit_native_evidence(self.native_evidence(entry)))

    def test_admitted_native_evidence_is_rechecked_before_history(self):
        self.ready()
        self.report.write_text('modified synthetic receipt')
        self.refused('Native verification evidence changed', lambda: self.history('A'))

    def test_preflight_timeout_consumes_same_two_hour_deadline(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=240, worker_pid=42)
        self.clock.advance(7200)
        self.refused('two-hour wall deadline', lambda: self.finish(entry))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 240)

    def test_physics_refused_without_freeze(self):
        self.refused('requires frozen-ready', lambda: self.history('A'))

    def test_history_debit_is_durable_before_return_and_retained(self):
        self.ready()
        entry = self.history('A')
        state = self.state()
        self.assertEqual(state['reservations'][1]['id'], entry['id'])
        self.assertEqual(budget.Budget.counters(state)['reservedNativeSteps'], 961)
        self.finish(entry, actual=10)
        state = self.state()
        self.assertEqual(budget.Budget.counters(state)['reservedNativeSteps'], 961)
        self.assertEqual(budget.Budget.counters(state)['actualNativeStepsKnown'], 11)

    def test_uncertain_history_blocks_new_history(self):
        self.ready()
        self.history('A')
        self.refused('Uncertain previous native', lambda: self.history('B'))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 961)

    def test_failed_worker_never_refunds(self):
        self.ready()
        entry = self.history('A')
        self.refused('Worker failed or timed out', lambda: self.ledger.complete(entry['id'], success=False))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 961)

    def test_failed_verification_receipt_keeps_partial_actual_and_full_reservation(self):
        self.ledger.freeze(self.manifest)
        entry = self.ledger.reserve('verification', native_steps=16, worker_pid=42)
        outcome = self.outcome(evidence=True)
        outcome['predictions_usable'] = False
        recorded = self.ledger.record_native_failure(entry['id'], actual_native_steps=7,
                                                     outcome=outcome, reason='synthetic parity mismatch')
        self.assertEqual(recorded['status'], 'failed-reported')
        self.assertEqual(recorded['outcome']['evidence']['sha256'], self.report_digest)
        self.assertEqual(self.state()['status'], 'closed-native-failure')
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 16)
        self.assertEqual(budget.Budget.counters(self.state())['actualNativeStepsKnown'], 7)
        self.refused('Protocol is closed', lambda: self.history('A'), closed=False)

    def test_failed_planning_receipt_retains_pending_history_and_partial_copy_count(self):
        history = self.b_history()
        entry = self.ledger.reserve('planning', history_id=history['id'], history_step=0, long_steps=[480] * 4)
        outcome = self.outcome(evidence=True)
        outcome['integrity_ok'] = False
        self.ledger.record_native_failure(entry['id'], actual_native_steps=123,
                                           outcome=outcome, reason='synthetic preview failure')
        state = self.state()
        self.assertEqual(budget.Budget.counters(state)['reservedNativeSteps'], 3841)
        self.assertEqual(budget.Budget.counters(state)['actualNativeStepsKnown'], 1084)
        self.assertEqual(next(e for e in state['reservations'] if e['id'] == history['id'])['status'], 'reserved')
        self.assertEqual(state['status'], 'closed-native-failure')

    def test_failed_history_receipt_retains_partial_actual(self):
        self.ready()
        entry = self.history('A')
        outcome = self.outcome(local=False, evidence=True)
        outcome['support_feasible'] = False
        self.ledger.record_native_failure(entry['id'], actual_native_steps=31,
                                           outcome=outcome, reason='synthetic support refusal')
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 961)
        self.assertEqual(budget.Budget.counters(self.state())['actualNativeStepsKnown'], 32)

    def test_late_failure_receipt_never_renews_or_changes_admission_clock(self):
        self.ready()
        entry = self.history('A')
        before = self.state()
        self.clock.advance(7201)
        outcome = self.outcome(local=False, evidence=True)
        recorded = self.ledger.record_native_failure(entry['id'], actual_native_steps=31,
                                                     outcome=outcome, reason='synthetic late receipt')
        state = self.state()
        self.assertEqual(state['lastClock'], before['lastClock'])
        self.assertEqual(state['physics'], before['physics'])
        self.assertIn('two-hour wall deadline', recorded['admissionRefusalAtReceipt'])
        self.assertEqual(state['status'], 'closed-native-failure')

    def test_failure_receipt_can_append_after_refusal_without_reopening(self):
        self.ready()
        entry = self.history('A')
        self.clock.advance(7200)
        self.refused('two-hour wall deadline', lambda: self.ledger.guard(entry['id']))
        before = self.state()
        outcome = self.outcome(local=False, evidence=True)
        self.ledger.record_native_failure(entry['id'], actual_native_steps=31,
                                           outcome=outcome, reason='synthetic receipt after closure')
        after = self.state()
        self.assertEqual(after['status'], before['status'])
        self.assertEqual(after['closureReason'], before['closureReason'])
        self.assertEqual(after['physics'], before['physics'])
        self.assertEqual(after['lastClock'], before['lastClock'])
        self.assertEqual(budget.Budget.counters(after)['actualNativeStepsKnown'], 32)

    def test_failure_receipt_cannot_claim_steps_beyond_reservation(self):
        self.ready()
        entry = self.history('A')
        self.refused('actual_native_steps', lambda: self.ledger.record_native_failure(entry['id'],
                     actual_native_steps=961, outcome=self.outcome(evidence=True), reason='synthetic invalid count'))
        self.assertEqual(budget.Budget.counters(self.state())['actualNativeStepsKnown'], 1)

    def test_failure_receipt_requires_unchanged_evidence_hash(self):
        self.ready()
        entry = self.history('A')
        self.report.write_text('modified synthetic failure evidence')
        self.refused('evidence hash mismatch', lambda: self.ledger.record_native_failure(entry['id'],
                     actual_native_steps=12, outcome=self.outcome(evidence=True), reason='synthetic changed receipt'))
        self.assertEqual(budget.Budget.counters(self.state())['actualNativeStepsKnown'], 1)

    def test_restarted_worker_refused_after_synthetic_crash(self):
        self.ready()
        entry = self.history('A')
        self.identity['startTicks'] = '456'
        self.refused('Worker changed', lambda: self.finish(entry))

    def test_multithreaded_worker_refused(self):
        self.ready()
        self.identity['threads'] = 2
        self.refused('single-threaded', lambda: self.history('A'))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 1)

    def test_wrong_worker_binary_refused(self):
        self.ready()
        self.identity['binarySha256'] = 'f' * 64
        self.refused('not the frozen binary', lambda: self.history('A'))

    def test_changed_frozen_input_before_reservation(self):
        self.ready()
        self.frozen.write_text('Changed synthetic input.\n')
        self.refused('Frozen input changed', lambda: self.history('A'))

    def test_changed_frozen_input_during_history(self):
        self.ready()
        entry = self.history('A')
        self.frozen.write_text('Changed synthetic input.\n')
        self.refused('Frozen input changed', lambda: self.ledger.guard(entry['id']))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 961)

    def test_history_order_not_renewable(self):
        self.ready()
        self.refused('Frozen history order', lambda: self.history('B'))

    def test_A_diagnostic_failure_still_allows_causal_B(self):
        entry = self.b_history()
        self.assertEqual(entry['history'], 'B')

    def test_B_failure_closes_repetition_gate(self):
        entry = self.b_history()
        self.finish(entry, local=False)
        self.assertEqual(self.state()['status'], 'closed-completed')
        self.refused('Protocol is closed', lambda: self.history('B-repeat'), closed=False)

    def test_planning_nested_in_B_charges_both_long_and_short(self):
        history = self.b_history()
        call = self.ledger.reserve('planning', history_id=history['id'], history_step=0,
                                   long_steps=[480] * 4, short_probes=93)
        self.assertEqual(call['nativeSteps'], 2292)
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 4213)
        self.finish(call, actual=0)
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 4213)
        self.assertEqual(self.ledger.guard(history['id'])['nativeSteps'], 960)

    def test_A_has_no_planning(self):
        self.ready()
        history = self.history('A')
        self.refused('active B history', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=0, long_steps=[480]))

    def test_no_overlapping_prediction_calls(self):
        history = self.b_history()
        self.ledger.reserve('planning', history_id=history['id'], history_step=0, short_probes=1)
        self.refused('no uncertain call', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=24, short_probes=1))

    def test_history_cannot_advance_during_prediction(self):
        history = self.b_history()
        self.ledger.reserve('planning', history_id=history['id'], history_step=0, short_probes=1)
        self.refused('Complete nested prediction', lambda: self.ledger.guard(history['id']))

    def test_history_cannot_finish_with_uncertain_prediction(self):
        history = self.b_history()
        self.ledger.reserve('planning', history_id=history['id'], history_step=0, short_probes=1)
        self.refused('Uncertain nested prediction', lambda: self.finish(history))

    def test_planning_cadence(self):
        history = self.b_history()
        call = self.ledger.reserve('planning', history_id=history['id'], history_step=0, short_probes=1)
        self.finish(call)
        self.refused('cadence exceeds 10 Hz', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=4, short_probes=1))

    def test_rollout_count_cap(self):
        history = self.b_history()
        self.refused('four long rollouts', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=0, long_steps=[480] * 5))

    def test_rollout_length_cap(self):
        history = self.b_history()
        self.refused('long rollout steps', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=0, long_steps=[481]))

    def test_short_probe_count_cap(self):
        history = self.b_history()
        self.refused('short_probes', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=0, short_probes=94))

    def test_forty_calls_cannot_be_increased_by_using_fewer_steps(self):
        history = self.b_history()
        for i in range(40):
            self.finish(self.ledger.reserve('planning', history_id=history['id'], history_step=i * 24,
                                            short_probes=1))
        self.refused('Forty planning-call cap', lambda: self.ledger.reserve('planning', history_id=history['id'],
                     history_step=959, short_probes=1))

    def test_exact_full_allocation_is_300000_then_closed(self):
        self.ready()
        technical = self.ledger.reserve('verification', native_steps=21119, worker_pid=42)
        self.finish(technical)
        for name in budget.HISTORY_ORDER:
            history = self.history(name)
            if name != 'A':
                for i in range(40):
                    call = self.ledger.reserve('planning', history_id=history['id'], history_step=i * 24,
                                               long_steps=[480] * 4, short_probes=93)
                    self.finish(call)
            self.finish(history)
        state = self.state()
        self.assertEqual(budget.Budget.counters(state)['reservedNativeSteps'], 300000)
        self.assertEqual(budget.Budget.counters(state)['reservedPredictionAndVerificationSteps'], 296160)
        self.assertEqual(budget.Budget.counters(state)['actualNativeStepsKnown'], 300000)
        self.assertEqual(state['status'], 'closed-completed')
        self.assertEqual(state['reservations'][-41]['headingDegrees'], 60)
        self.refused('Protocol is closed', lambda: self.history('A'), closed=False)

    def test_verification_reservation_cap(self):
        self.ready()
        self.refused('Technical-verification step cap', lambda: self.ledger.reserve('verification',
                     native_steps=21121, worker_pid=42))

    def test_two_hour_deadline_at_guard(self):
        self.ready()
        history = self.history('A')
        self.clock.advance(7200)
        self.refused('two-hour wall deadline', lambda: self.ledger.guard(history['id']))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 961)

    def test_monotonic_deadline_prevents_slow_wall_clock(self):
        self.ready()
        history = self.history('A')
        self.clock.value['monotonic'] += 7200
        self.refused('two-hour wall deadline', lambda: self.ledger.guard(history['id']))

    def test_clock_rollback_fails_closed(self):
        self.ready()
        self.clock.value['wall'] -= 1
        self.refused('Clock regressed', lambda: self.history('A'))

    def test_reboot_does_not_renew_budget(self):
        self.clock.value['boot'] = 'another-boot'
        self.refused('machine rebooted', lambda: self.ready())

    def test_overspend_in_completion_refused_without_refund(self):
        self.ready()
        history = self.history('A')
        self.refused('actual_native_steps', lambda: self.finish(history, actual=961))
        self.assertEqual(budget.Budget.counters(self.state())['reservedNativeSteps'], 961)

    def test_missing_expected_diagnostic_stops(self):
        self.ready()
        history = self.history('A')
        outcome = self.outcome(evidence=True)
        outcome['diagnostic_present'] = False
        self.refused('expected diagnostic', lambda: self.ledger.complete(history['id'],
                     actual_native_steps=100, outcome=outcome))

    def test_integrity_failure_stops(self):
        self.ready()
        history = self.history('A')
        outcome = self.outcome(evidence=True)
        outcome['integrity_ok'] = False
        self.refused('integrity', lambda: self.ledger.complete(history['id'],
                     actual_native_steps=100, outcome=outcome))

    def test_support_infeasibility_stops(self):
        self.ready()
        history = self.history('A')
        outcome = self.outcome(evidence=True)
        outcome['support_feasible'] = False
        self.refused('support', lambda: self.ledger.complete(history['id'],
                     actual_native_steps=100, outcome=outcome))

    def test_unusable_predictions_stop(self):
        history = self.b_history()
        outcome = self.outcome(evidence=True)
        outcome['predictions_usable'] = False
        self.refused('prediction', lambda: self.ledger.complete(history['id'],
                     actual_native_steps=100, outcome=outcome))

    def test_invalid_boolean_is_not_a_step_count(self):
        self.ready()
        self.refused('native_steps', lambda: self.ledger.reserve('verification', native_steps=True, worker_pid=42))

    def test_unknown_request_key_refused(self):
        self.ready()
        self.refused('Unknown reservation parameters', lambda: self.ledger.reserve('history', history='A',
                     worker_pid=42, reset_budget=True))

    def test_checksum_corruption_is_not_repaired_or_credited(self):
        path = self.root / 'ledger.json'
        envelope = json.loads(path.read_text())
        envelope['payload']['limits']['totalNativeSteps'] += 1
        path.write_text(json.dumps(envelope))
        before = path.read_bytes()
        self.refused('integrity digest', lambda: self.ready(), closed=False)
        self.assertEqual(path.read_bytes(), before)

    def test_pending_atomic_write_blocks_resume(self):
        (self.root / '.ledger.pending').write_text('Incomplete synthetic crash write')
        self.refused('uncertain pending write', lambda: self.ready(), closed=False)

    def test_lock_blocks_competing_coordinator(self):
        with (self.root / 'coordinator.lock').open('r+') as lock:
            budget.fcntl.flock(lock, budget.fcntl.LOCK_EX | budget.fcntl.LOCK_NB)
            self.refused('Another coordinator', lambda: self.ready(), closed=False)

    def test_close_missing_preparation_keeps_zero_native_steps(self):
        result = self.ledger.close('Numeric continuation missing; this version closed without physics')
        self.assertEqual(result['counters']['reservedNativeSteps'], 0)
        self.assertEqual(result['status'], 'closed-explicit')
        self.refused('Protocol is closed', lambda: self.ready(), closed=False)

    def test_status_reads_closed_ledger_without_mutation_or_authorization(self):
        self.ledger.close('Synthetic preparation is closed')
        before = (self.root / 'ledger.json').read_bytes()
        report = self.ledger.status()
        self.assertEqual(report['status'], 'closed-explicit')
        self.assertEqual(report['counters']['reservedNativeSteps'], 0)
        self.assertFalse(report['authorizesIntegration'])
        self.assertEqual((self.root / 'ledger.json').read_bytes(), before)

    def test_status_does_not_refresh_expired_deadline(self):
        self.ready()
        self.history('A')
        before = (self.root / 'ledger.json').read_bytes()
        self.clock.advance(7300)
        report = self.ledger.status()
        self.assertFalse(report['authorizesIntegration'])
        self.assertEqual(report['physics']['deadlineWall'], self.clock.value['wall'] - 100)
        self.assertEqual((self.root / 'ledger.json').read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
