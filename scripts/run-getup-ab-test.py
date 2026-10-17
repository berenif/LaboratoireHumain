#!/usr/bin/env python3
"""Budget refusal tests, zero executables/world integrations."""
import copy
import datetime as dt
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('getup_budget', Path(__file__).with_name('run-getup-ab.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Admission(unittest.TestCase):
    def setUp(self):
        self.ledger = {'limits': copy.deepcopy(module.LIMITS), 'status': 'frozen-ready',
                       'historiesStarted': 0, 'reservedNativeSteps': 0, 'entries': [],
                       'manifestSha256': 'frozen', 'previousLedgers': {}}
        self.manifest = {'sha256': 'frozen', 'value': {'files': {}}}

    def refuse(self, phrase):
        before = copy.deepcopy(self.ledger)
        with self.assertRaisesRegex(AssertionError, phrase):
            module.admit(self.ledger, self.manifest)
        self.assertEqual(self.ledger, before)

    def test_closed(self):
        self.ledger['status'] = 'closed-no-complete-recovery'
        self.refuse('closed')

    def test_changed_limits(self):
        self.ledger['limits']['histories'] = 4
        self.refuse('contract changed')

    def test_changed_manifest(self):
        self.manifest['sha256'] = 'changed'
        self.refuse('Manifest changed')

    def test_uncertain(self):
        self.ledger.update(historiesStarted=1, reservedNativeSteps=6240,
                           entries=[{'status': 'timeout-reservation-retained'}])
        self.refuse('Uncertain')

    def test_expired(self):
        self.ledger['simulationStartedUtc'] = (dt.datetime.now(dt.timezone.utc)-dt.timedelta(seconds=601)).isoformat()
        self.refuse('Wall budget')

    def test_repeat_requires_complete_B(self):
        self.ledger.update(historiesStarted=2, reservedNativeSteps=12480,
                           entries=[{'status': 'process-completed', 'actualNativeSteps': 2,
                                     'fullRecoverySuccess': False} for _ in range(2)])
        self.refuse('Repeat requires complete B success')


if __name__ == '__main__':
    unittest.main()
