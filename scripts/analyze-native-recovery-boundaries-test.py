#!/usr/bin/env python3
"""Negative regression of evidence admission. Zero native integration."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('boundary_analysis', Path(__file__).with_name('analyze-native-recovery-boundaries.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class EvidenceAdmission(unittest.TestCase):
    def make(self, root):
        enabled, disabled = root/'on', root/'off'
        enabled.mkdir(); disabled.mkdir()
        row = {'substep': 1, 'timeS': .004, 'loadsN': [1], 'bodiesHash': 'a', 'motorsHash': 'b', 'contactsHash': 'c'}
        report = {'initial': {}, 'final': {}, 'physicalSubsteps': 2, 'firstRefusal': {'check': 'self-contact'}, 'fullRecoverySuccess': False}
        for p in [enabled, disabled]:
            (p/'substeps.jsonl').write_text(json.dumps(row)+'\n')
            (p/'report.json').write_text(json.dumps(report))
        intervals = []
        for step, interval, before, after, impulse in [(1,0,0,3,3),(1,1,3,5,2),(2,0,5,6,1)]:
            intervals.append({'native_substep': step, 'ccd_interval': interval, 'dt_s': .002,
                              'momentum_before_ns': [0,before,0], 'momentum_after_ns': [0,after,0],
                              'gravity_and_user_impulse_ns': [0,0,0], 'floor_impulse_ns': [0,impulse,0],
                              'linear_residual_ns': [0,0,0], 'observation_error': None})
        (enabled/'solver-boundaries.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in intervals))
        return enabled, disabled, row, intervals

    def test_keeps_failed_step_and_sums_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            on, off, _, _ = self.make(Path(tmp))
            summary, aggregate = module.analyze(on, off)
            self.assertEqual(summary['solverIntervalsIncludingRefusal'], 3)
            self.assertEqual(summary['nativeStepsIncludingRefusal'], 2)
            self.assertEqual(aggregate[0]['allIntervalsFloorN'][1], 1250)
            self.assertFalse(summary['p1Passed'])

    def test_rejects_changed_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            on, off, row, _ = self.make(Path(tmp))
            row['contactsHash'] = 'changed'
            (off/'substeps.jsonl').write_text(json.dumps(row)+'\n')
            with self.assertRaisesRegex(AssertionError, 'Observation changed physics'):
                module.analyze(on, off)

    def test_rejects_changed_final_refused_world(self):
        with tempfile.TemporaryDirectory() as tmp:
            on, off, _, _ = self.make(Path(tmp))
            for directory, body_hash in [(on, 'a'), (off, 'changed')]:
                report_path = directory/'report.json'
                report = json.loads(report_path.read_text())
                report['finalWorldHashes'] = {'bodies': body_hash, 'motors': 'b', 'contacts': 'c', 'counters': 'd'}
                report_path.write_text(json.dumps(report))
            with self.assertRaisesRegex(AssertionError, 'Observation changed physics'):
                module.analyze(on, off)

    def test_rejects_missing_refused_step(self):
        with tempfile.TemporaryDirectory() as tmp:
            on, off, _, intervals = self.make(Path(tmp))
            (on/'solver-boundaries.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in intervals[:-1]))
            with self.assertRaisesRegex(AssertionError, 'Missing integrated boundary'):
                module.analyze(on, off)

    def test_rejects_duplicated_interval(self):
        with tempfile.TemporaryDirectory() as tmp:
            on, off, _, intervals = self.make(Path(tmp))
            intervals[1]['ccd_interval'] = 0
            (on/'solver-boundaries.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in intervals))
            with self.assertRaisesRegex(AssertionError, 'Duplicate or reordered'):
                module.analyze(on, off)


if __name__ == '__main__':
    unittest.main()
