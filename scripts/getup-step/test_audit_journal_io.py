"""Streaming audit ingestion preserves prior reducers without whole-file reads."""
import hashlib
import json
from pathlib import Path
import tempfile
import tracemalloc
import unittest
from unittest.mock import patch

import numpy as np

from audit_journal_io import CHUNK_SIZE, digest, json_array_items
import audit_source_arm_knee as arm
import audit_source_cadenced_steps as cadence
import audit_source_multistep as motion
import audit_source_upright_large_steps as upright


def notes_fixture():
    return [dict(activeJointGroundBarriers=2, activeSelfAvoidanceBarriers=0,
                 jointGroundAdjustments=[dict(adjacentColliderClearanceM=.17),
                                         dict(adjacentColliderClearanceM=.09)],
                 selfAvoidance=[dict(separationM=.13)],
                 postureExperiment=dict(armBalanceOffsetRad=[-.1, .2])),
            dict(activeJointGroundBarriers=0, activeSelfAvoidanceBarriers=3,
                 jointGroundAdjustments=[], selfAvoidance=[]),
            dict(activeJointGroundBarriers=1, activeSelfAvoidanceBarriers=1,
                 jointGroundAdjustments=[dict(adjacentColliderClearanceM=-.003)],
                 selfAvoidance=[dict(separationM=.04), dict(separationM=.08)],
                 postureExperiment=dict(armBalanceOffsetRad=[.3, -.25]))]


class GuardedReader:
    def __init__(self, stream, sizes):
        self.stream, self.sizes = stream, sizes

    def __enter__(self):
        self.stream.__enter__()
        return self

    def __exit__(self, *args):
        return self.stream.__exit__(*args)

    def read(self, size=-1):
        if size <= 0 or size > CHUNK_SIZE:
            raise AssertionError('unbounded or oversized audit read')
        self.sizes.append(size)
        return self.stream.read(size)


class AuditJournalIOTests(unittest.TestCase):
    def test_all_small_chunk_boundaries_match_json_loads(self):
        values = notes_fixture() + [None, True, -1.25e-12, 12345678901234567890,
            'a quoted "value" with \\ and café ☃', [1, dict(a=[2, 3])]]
        text = json.dumps(values, ensure_ascii=False) + '\n \t'
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'control.json'
            path.write_text(text, encoding='utf-8')
            for chunk_size in range(1, 33):
                with self.subTest(chunk_size=chunk_size):
                    self.assertEqual(list(json_array_items(path, chunk_size)), json.loads(text))

    def test_truncated_and_malformed_arrays_are_rejected(self):
        text = json.dumps([dict(nested=[1, 'a]b', dict(x='escaped "quote"')]), 1.234e-15])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'control.json'
            for end in range(len(text)):
                path.write_text(text[:end])
                with self.subTest(end=end), self.assertRaises(ValueError):
                    list(json_array_items(path, chunk_size=7))
            for invalid in ('{}', '[,1]', '[1,,2]', '[1,]', '[1 2]', '[1]{}', '[1e]', '[1.]'):
                path.write_text(invalid)
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    list(json_array_items(path, chunk_size=1))
            path.write_text(' [ ] \n')
            self.assertEqual(list(json_array_items(path, chunk_size=1)), [])

    def test_streamed_control_reducers_equal_prior_arithmetic(self):
        notes = notes_fixture()
        expected_motion = dict(controlSolves=len(notes),
            activeJointGroundBarrierSolves=sum(n['activeJointGroundBarriers'] > 0 for n in notes),
            activeSelfAvoidanceSolves=sum(n['activeSelfAvoidanceBarriers'] > 0 for n in notes),
            minimumAdjacentNonfootColliderClearanceM=min(a['adjacentColliderClearanceM']
                for n in notes for a in n['jointGroundAdjustments']))
        expected_gap = min((a['separationM'] for n in notes for a in n['selfAvoidance']), default=None)
        expected_offset = max((max(abs(np.array(n.get('postureExperiment', {}).get(
            'armBalanceOffsetRad', [0.])))) for n in notes), default=0.)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'control.json'
            path.write_text(json.dumps(notes))
            self.assertEqual(motion.audit_control_notes(json_array_items(path, 11)), expected_motion)
            self.assertEqual(arm.audit_control_notes(json_array_items(path, 13)),
                             (expected_gap, expected_offset))
            self.assertEqual(min((a['separationM'] for n in json_array_items(path, 17)
                                  for a in n['selfAvoidance']), default=None), expected_gap)
        self.assertEqual(arm.audit_control_notes(iter([])), (None, 0.))
        with self.assertRaises(ValueError):
            motion.audit_control_notes(iter([]))

    def test_many_notes_do_not_accumulate(self):
        # An 8 MB journal with a 4 KB read window. Whole-array loading would
        # retain all 8 MB of payload, independently of its JSON text buffer.
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'control.json'
            note = notes_fixture()[0]
            note['unusedPayload'] = 'x'*8192
            encoded = json.dumps(note)
            with path.open('w') as stream:
                stream.write('[')
                for index in range(1000):
                    stream.write((',' if index else '') + encoded)
                stream.write(']')
            tracemalloc.start()
            try:
                result = motion.audit_control_notes(json_array_items(path, 4096))
                peak = tracemalloc.get_traced_memory()[1]
            finally:
                tracemalloc.stop()
            self.assertEqual(result['controlSolves'], 1000)
            self.assertLess(peak, 1024*1024)

    def test_full_audit_paths_forbid_whole_journal_reads(self):
        # A stopped one-frame synthetic run is enough to exercise all inherited
        # frame/control/hash paths. It never opens a native world.
        q = [0., 0., .8, 1., 0., 0., 0.] + [0.]*21
        feet = [dict(position=[0., .1, .05], loadN=100., clearance=0.,
                     groundRays=[dict(hit=True, ownBodyExcluded=True)]*4),
                dict(position=[0., -.1, .05], loadN=100., clearance=0.,
                     groundRays=[dict(hit=True, ownBodyExcluded=True)]*4)]
        row = dict(sourceStep=1, elapsedS=.00125, phase='transfer', phaseElapsedS=.00125,
            qpos=q, qvel=[0.]*27, actualMotorTorquesNm=[0.]*21, warnings=0, nonfootLoadN=0.,
            feet=feet, maximumSelfPenetrationM=0., jointLimitMinimumMarginRad=.02,
            jointLimitMarginsRad=[.02]*21, minimumComCaptureMarginM=.02,
            jointGroundRays=[dict(hit=True, ownBodyExcluded=True)]*22,
            stepIndex=1, swingSide=0, swingForwardDisplacementM=0.,
            requestedForwardM=.18, loadedSlipDistanceM=[0., 0.])
        entry = dict(qpos=q, jointLimitMinimumMarginRad=.02)
        multi = dict(actualIntegrations=1, physicsSimSeconds=.00125, globalLoadedSlipM=[0., 0.],
            passed=False, stopped='synthetic-stop', finalQuietS=0., final=dict(row),
            steps=[dict(index=1, swingSide='left', requestedDistanceM=.18, achievedForwardM=0.)])
        reference = dict(heading=[1., 0., 0.], targets=[[[0., .1, .05]], [[0., -.1, .05]]])
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            for name, value in [('summary.json', dict(entry=entry, modelSHA256='model',
                    helperSHA256='helper', physicalStateWritesAfterInitialization=0, rootAssistance=False)),
                    ('multistep-summary.json', multi), ('controller-step-01.json', reference),
                    ('control.json', notes_fixture())]:
                (folder/name).write_text(json.dumps(value))
            (folder/'frames.ndjson').write_text(json.dumps(row)+'\n')
            (folder/'final.bin').write_bytes(b'unchanged binary\x00\xff')
            expected_hash = hashlib.sha256((folder/'control.json').read_bytes()).hexdigest()
            original_open, original_read_text = Path.open, Path.read_text
            sizes = []

            def guarded_open(path, *args, **kwargs):
                stream = original_open(path, *args, **kwargs)
                return GuardedReader(stream, sizes) if path.name == 'control.json' else stream

            def guarded_text(path, *args, **kwargs):
                if path.name in ('control.json', 'frames.ndjson'):
                    raise AssertionError('whole-journal read_text is forbidden')
                return original_read_text(path, *args, **kwargs)

            with patch.object(Path, 'open', guarded_open), patch.object(Path, 'read_text', guarded_text), \
                    patch.object(Path, 'read_bytes', side_effect=AssertionError('read_bytes is forbidden')):
                self.assertEqual(digest(folder/'control.json'), expected_hash)
                self.assertEqual(cadence.digest(folder/'control.json'), expected_hash)
                self.assertEqual(motion.audit(folder)['rayControlAudit']['controlSolves'], 3)
                self.assertEqual(arm.audit(folder)['minimumMeasuredSelfSeparationM'], .04)
                self.assertEqual(upright.audit(folder)['minimumCommandSampledSelfSeparationM'], .04)
            self.assertTrue(sizes)
            self.assertLessEqual(max(sizes), CHUNK_SIZE)


if __name__ == '__main__':
    unittest.main()
