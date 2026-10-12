#!/usr/bin/env python3
"""No-world regression checks for the proposed phase and numeric admission."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

SCRIPT = Path(__file__).with_name('recovery-intermediate-preflight.py')
SPEC = importlib.util.spec_from_file_location('intermediate_preflight', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Contract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = json.loads((MODULE.REPO/'docs/intermediate-transfer-plan-2026-10-12.json').read_text())
        cls.model = json.loads(MODULE.MODEL.read_text())
        cls.contract = json.loads(MODULE.CONTRACT.read_text())

    def test_plan_is_ordered_but_numerically_blocked(self):
        receipt = MODULE.inspect(self.plan)
        self.assertEqual(len(receipt['chosenPhases']), 7)
        self.assertEqual(receipt['physicalIntegrations'], 0)
        self.assertFalse(receipt['completeNumericTrajectoryReady'])
        self.assertFalse(receipt['newNativeHistoryAllowed'])
        self.assertGreater(receipt['naiveStitch']['pelvisPositionGapM'], .5)

    def test_whole_leg_must_be_excluded_before_swing(self):
        plan = copy.deepcopy(self.plan)
        plan['phases'][2]['release'].remove('leftForefoot')
        with self.assertRaises(AssertionError):
            MODULE.verify_plan(plan, self.model, self.contract)

    def test_original_clock_cannot_be_extended(self):
        plan = copy.deepcopy(self.plan)
        plan['maximumRecoveryTimeS'] = 30
        with self.assertRaises(AssertionError):
            MODULE.verify_plan(plan, self.model, self.contract)

    def test_unmodeled_root_command_is_rejected(self):
        blank = [[0.0]*3 for _ in range(25)]
        velocity = [[0.0]*3 for _ in range(25)]
        first = {'substep':0,'positions':copy.deepcopy(blank),'velocities':copy.deepcopy(velocity)}
        last = {'substep':8*240,'positions':copy.deepcopy(blank),'velocities':copy.deepcopy(velocity)}
        last['positions'][0][0] = .1
        self.assertIn('unmodeled/root command', '\n'.join(MODULE.check_keys(
            {'segmentIds':[s['id'] for s in self.model['segments']], 'keyframes':[first,last]},
            self.model)))

    def test_key_slew_over_native_rate_is_rejected(self):
        blank = [[0.0]*3 for _ in range(25)]
        velocity = [[0.0]*3 for _ in range(25)]
        frames = [{'substep': step, 'positions':copy.deepcopy(blank),
                   'velocities':copy.deepcopy(velocity)} for step in [0,1,8*240]]
        frames[1]['positions'][15][0] = .1
        self.assertIn('key target slew exceeds 1 rad/s', '\n'.join(MODULE.check_keys(
            {'segmentIds':[s['id'] for s in self.model['segments']], 'keyframes':frames},
            self.model)))


if __name__ == '__main__':
    unittest.main()
