#!/usr/bin/env python3
"""Fail-closed budget admission. Uses a sentinel executable, no native worlds."""
import datetime as dt
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

RUNNER = Path(__file__).with_name('run-native-recovery-contract.py')
spec = importlib.util.spec_from_file_location('contract_budget', RUNNER)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BudgetAdmission(unittest.TestCase):
    def reject(self, edit, reason):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            kernel, program = root/'sentinel.py', root/'program.json'
            marker = root/'unexpected-launch'
            kernel.write_text('#!'+sys.executable+'\nfrom pathlib import Path\nPath('+repr(str(marker))+').touch()\n')
            kernel.chmod(0o700)
            program.write_text('{}')
            now = dt.datetime.now(dt.timezone.utc).isoformat()
            ledger = {'campaign': 'native-hybrid-v1', 'status': 'P1-observation-in-progress',
                      'contractStartedUtc': now, 'historiesStarted': 0, 'reservedNativeSteps': 0,
                      'entries': [], 'previousCampaignLedgerSha256': 'b61f17b180afece7db1e3a7d541edeaf24094f358edd2bb7e1362797d8b600e4',
                      'limits': {'contractHistories': 32, 'nativeStepsPerContractHistory': 6240,
                                 'contractWallS': 1800, 'globalWallS': 21600,
                                 'globalNativeSteps': 1509120}}
            edit(ledger)
            path = root/'ledger.json'
            path.write_text(json.dumps(ledger))
            original = path.read_bytes()
            result = subprocess.run([sys.executable, str(RUNNER), '--root', str(root),
                                     '--kernel', str(kernel), '--program', str(program), '--label', 'negative'],
                                    capture_output=True, text=True, check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(reason, result.stderr)
            self.assertEqual(path.read_bytes(), original)
            self.assertFalse(marker.exists(), 'Budget refusal launched a worker')

    def test_history_cap(self):
        self.reject(lambda l: l.update(historiesStarted=32), 'Contract histories exhausted')

    def test_wall_cap(self):
        self.reject(lambda l: l.update(contractStartedUtc=(dt.datetime.now(dt.timezone.utc)-dt.timedelta(seconds=1801)).isoformat()), 'Wall budget exhausted')

    def test_uncertain_launch(self):
        self.reject(lambda l: l.update(entries=[{'index': 0, 'status': 'reserved-before-launch'}]), 'Uncertain previous launch')

    def test_edited_limits(self):
        self.reject(lambda l: l['limits'].update(contractHistories=33), 'Budget contract changed')

    def test_old_ledger_changed(self):
        self.reject(lambda l: l.update(previousCampaignLedgerSha256='changed'), 'Closed campaign changed')


if __name__ == '__main__':
    unittest.main()
