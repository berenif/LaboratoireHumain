#!/usr/bin/env python3
"""Synthetic temporary files only; never reads/archives native-run evidence."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('archive_native_under_test', Path(__file__).with_name('archive_native.py'))
archive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(archive)


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='lh-inert-archive-test-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.run = self.root / 'native-run'
        self.run.mkdir()
        self.budget = self.root / 'budget'
        self.budget.mkdir()
        self.ledger = self.budget / 'ledger.json'
        self.index = self.root / 'archive-index.json'
        self.raw = self.run / '000-native-verification.json'
        self.data = b'{"synthetic":true,"physicalIntegrations":0}\n'
        self.raw.write_bytes(self.data)
        self.state = {'id': 'synthetic', 'status': 'closed-native-failure', 'events': [{'sequence': 1}],
                      'reservations': [{'outcome': {'evidence': {'path': str(self.raw),
                          'sha256': hashlib.sha256(self.data).hexdigest()}}}]}
        self.result = {'status': 'failed-closed', 'budget': {'id': 'synthetic',
                       'status': self.state['status'], 'authorizesIntegration': False,
                       'lastEvent': self.state['events'][-1]}, 'histories': []}
        self.save_receipts()

    def save_receipts(self):
        self.ledger.write_text(json.dumps({'payload': self.state,
            'sha256': hashlib.sha256(archive.canonical(self.state)).hexdigest()}))
        (self.run / 'run-result.json').write_text(json.dumps(self.result))

    def perform(self):
        return archive.archive(self.run, self.ledger, self.index)

    def test_small_and_empty_files_roundtrip_raw_files_remain(self):
        (self.run / '001-A.states.jsonl').write_bytes(b'')
        (self.run / '001-A.json.accounting.jsonl').write_bytes(b'{"inert":true}\n')
        (self.run / '002-B-planning-000.json').write_bytes(b'{"inert":true}\n')
        (self.run / '001-A.json').write_bytes(b'ordinary history report remains uncompressed')
        result = self.perform()
        self.assertEqual(len(result['files']), 4)
        self.assertEqual(result['nativeIntegrations'], 0)
        for row in result['files']:
            original = (self.root / row['rawPath']).read_bytes()
            restored = b''.join(gzip.decompress((self.root / part['gzipPath']).read_bytes())
                                for part in row['parts'])
            self.assertEqual(restored, original)
            self.assertEqual(hashlib.sha256(restored).hexdigest(), row['rawSha256'])
        self.assertEqual(self.raw.read_bytes(), self.data)
        self.assertFalse((self.run / '001-A.json.gz').exists())

    def test_chunk_boundaries_preserve_bytes_without_line_alignment(self):
        data = b'1234567890\n' * 9
        trace = self.run / '001-A.states.jsonl'
        trace.write_bytes(data)
        with mock.patch.object(archive, 'CHUNK_BYTES', 32):
            result = self.perform()
        row = next(r for r in result['files'] if r['rawPath'].endswith('states.jsonl'))
        self.assertEqual([p['rawOffset'] for p in row['parts']], [0, 32, 64, 96])
        self.assertEqual([p['rawBytes'] for p in row['parts']], [32, 32, 32, 3])
        restored = b''.join(gzip.decompress((self.root / p['gzipPath']).read_bytes()) for p in row['parts'])
        self.assertEqual(restored, data)
        self.assertEqual(trace.read_bytes(), data)

    def test_open_ledger_refused_without_any_output(self):
        self.state['status'] = 'running'
        self.save_receipts()
        with self.assertRaisesRegex(ValueError, 'Ledger is not closed'):
            self.perform()
        self.assertFalse(self.index.exists())
        self.assertEqual(list(self.run.glob('*.gz')), [])

    def test_existing_index_not_overwritten(self):
        self.index.write_bytes(b'PREEXISTING')
        with self.assertRaisesRegex(ValueError, 'Index already exists'):
            self.perform()
        self.assertEqual(self.index.read_bytes(), b'PREEXISTING')
        self.assertEqual(list(self.run.glob('*.gz')), [])

    def test_existing_later_archive_refused_before_any_output(self):
        later = self.run / '999-A.states.jsonl'
        later.write_bytes(b'INERT')
        packed = later.with_name(later.name + '.gz')
        packed.write_bytes(b'PREEXISTING')
        with self.assertRaisesRegex(ValueError, 'Archive output already exists'):
            self.perform()
        self.assertEqual(packed.read_bytes(), b'PREEXISTING')
        self.assertEqual(list(self.run.glob('*.gz')), [packed])
        self.assertFalse(self.index.exists())

    def test_closed_evidence_hash_mismatch_refused(self):
        self.raw.write_bytes(b'altered synthetic receipt')
        with self.assertRaisesRegex(ValueError, 'differs from closed receipt'):
            self.perform()
        self.assertFalse(self.index.exists())
        self.assertEqual(list(self.run.glob('*.gz')), [])

    def test_ledger_digest_mismatch_refused(self):
        envelope = json.loads(self.ledger.read_text())
        envelope['payload']['id'] = 'tampered'
        self.ledger.write_text(json.dumps(envelope))
        with self.assertRaisesRegex(ValueError, 'Ledger digest mismatch'):
            self.perform()

    def test_final_run_receipt_must_match_closed_ledger(self):
        self.result['budget']['lastEvent'] = {'sequence': 0}
        self.save_receipts()
        with self.assertRaisesRegex(ValueError, 'final ledger event'):
            self.perform()

    def test_pending_ledger_write_refused(self):
        (self.budget / '.ledger.pending').touch()
        with self.assertRaisesRegex(ValueError, 'Ledger write is incomplete'):
            self.perform()

    def test_symlink_raw_file_refused(self):
        (self.run / '001-A.states.jsonl').symlink_to(self.raw)
        with self.assertRaisesRegex(ValueError, 'non-symlink'):
            self.perform()
        self.assertEqual(list(self.run.glob('*.gz')), [])


if __name__ == '__main__':
    unittest.main()
