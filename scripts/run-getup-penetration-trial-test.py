#!/usr/bin/env python3
"""Admission checks only: no browser, reset, or native integration."""
import copy
import importlib.util
import unittest
from pathlib import Path
import tempfile
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('bounded_trial', Path(__file__).with_name('run-getup-penetration-trial.py'))
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'prior.json').write_text('{"status":"closed","nativeChargedAfter":299998,"wallChargedAfterSeconds":5926.328294294003}')
        (self.root/'input').write_text('frozen')
        self.ledger = dict(status='prepared', incrementalNativeAllowance=8200,
                           maximumMainNativeSteps=3200, maximumTeacherNativeSteps=5000,
                           incrementalWallAllowanceSeconds=180, maximumPhysicalHistories=1,
                           maximumForecastHistories=1, retry=False, refund=False,
                           priorClosedRecord='prior.json', priorClosedRecordSha256=trial.sha(self.root/'prior.json'),
                           historicalReservedNativeSteps=299998, historicalReservedWallSeconds=5926.328294294003,
                           inputs={'input':trial.sha(self.root/'input')}, sourceBase='base')

    def admit(self, ledger):
        with patch.object(trial, 'REPO', self.root), patch.object(trial.subprocess, 'check_output', return_value='base\n'):
            trial.admit(ledger)

    def test_valid_admission_is_read_only(self):
        before = {p.name:p.read_bytes() for p in self.root.iterdir()}
        self.admit(self.ledger)
        self.assertEqual(before, {p.name:p.read_bytes() for p in self.root.iterdir()})

    def test_closed_running_or_changed_limits_cannot_restart(self):
        for key, value in [('status','closed'), ('status','running-reservation-retained'),
                           ('incrementalNativeAllowance',8201), ('maximumMainNativeSteps',3201),
                           ('maximumTeacherNativeSteps',5001), ('incrementalWallAllowanceSeconds',181),
                           ('maximumPhysicalHistories',2), ('maximumForecastHistories',2),
                           ('retry',True), ('refund',True), ('sourceBase','changed')]:
            ledger = copy.deepcopy(self.ledger); ledger[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(AssertionError):
                self.admit(ledger)

    def test_frozen_source_and_prior_ledger_mutations_are_rejected(self):
        for name in ['input','prior.json']:
            p = self.root/name; before=p.read_bytes(); p.write_bytes(before+b' ')
            with self.subTest(name=name), self.assertRaises(AssertionError):
                self.admit(self.ledger)
            p.write_bytes(before)


if __name__ == '__main__':
    unittest.main()
