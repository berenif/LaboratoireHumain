#!/usr/bin/env python3
"""Synthetic file tests only. No native world, worker, FK or integration."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('builder', Path(__file__).with_name('prepare_local_pair_manifest.py'))
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class ManifestPreparationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.code = self.root / 'source.txt'
        self.code.write_text('source version one\n')
        self.proof = self.root / 'review.json'
        self.proof.write_text('{"synthetic":true}\n')
        self.files = {'code': [builder.record(self.code)]}
        self.prerequisites = ('checked_source', 'local_pair_frozen')
        self.receipt = builder.pending_receipt(self.files, self.prerequisites)

    def ready(self):
        for check in self.receipt['checks'].values():
            check.update(status='passed', evidence=[builder.record(self.proof)])

    def validate(self):
        return builder.validate_readiness(self.receipt, self.files, self.prerequisites)

    def test_pending_template_cannot_create_manifest(self):
        with self.assertRaisesRegex(ValueError, 'Missing passed evidence'):
            self.validate()

    def test_explicit_bound_receipt_passes_without_duplicating_evidence(self):
        self.ready()
        self.assertEqual(self.validate(), [builder.record(self.proof)])

    def test_missing_evidence_is_refused_even_with_passed_attestation(self):
        self.ready()
        self.receipt['checks']['local_pair_frozen']['evidence'] = []
        with self.assertRaisesRegex(ValueError, 'Missing passed evidence'):
            self.validate()

    def test_source_change_after_review_is_refused(self):
        self.ready()
        self.code.write_text('unreviewed source\n')
        with self.assertRaisesRegex(ValueError, 'Evidence is changed'):
            self.validate()

    def test_fresh_hash_without_reviewer_binding_is_refused(self):
        self.ready()
        self.code.write_text('unreviewed source\n')
        self.files['code'] = [builder.record(self.code)]
        with self.assertRaises(ValueError):
            self.validate()

    def test_missing_source_binding_is_refused(self):
        self.ready()
        self.receipt['checkedFiles'] = []
        with self.assertRaisesRegex(ValueError, 'incompletely bound'):
            self.validate()

    def test_changed_proof_is_refused(self):
        self.ready()
        self.proof.write_text('{"synthetic":"changed"}\n')
        with self.assertRaisesRegex(ValueError, 'Evidence is changed'):
            self.validate()

    def test_old_four_history_prerequisite_is_refused(self):
        self.ready()
        self.receipt['checks']['four_histories_frozen'] = self.receipt['checks'].pop('local_pair_frozen')
        with self.assertRaisesRegex(ValueError, 'local-pair prerequisites'):
            self.validate()

    def test_physical_or_authorizing_receipt_is_refused(self):
        self.ready()
        original = copy.deepcopy(self.receipt)
        for field, value in [('physicalIntegrations', 1), ('staticInspections', 1),
                             ('nativeExecutionAuthorized', True), ('physicalIntegrations', False)]:
            self.receipt = copy.deepcopy(original)
            self.receipt[field] = value
            with self.assertRaisesRegex(ValueError, 'pure and non-authorizing'):
                self.validate()

    def test_exclusive_output_preserves_existing_file(self):
        destination = self.root / 'manifest.json'
        builder.write_new(destination, {'original': True})
        with self.assertRaises(FileExistsError):
            builder.write_new(destination, {'replacement': True})
        self.assertEqual(json.loads(destination.read_text()), {'original': True})

    def test_archived_static_continuation_hashes_and_grid_are_complete(self):
        builder.validate_static_reference()

    def test_vendor_inventory_includes_build_config_and_data_but_not_compiled_cache(self):
        for name in ('src/lib.rs', 'build.rs', '.cargo/config.toml', 'data/shape.json', 'target/debug/generated.rs'):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('synthetic build input\n')
        listed = {path.relative_to(self.root).as_posix() for path in builder.source_tree_files(self.root)}
        self.assertTrue({'src/lib.rs', 'build.rs', '.cargo/config.toml', 'data/shape.json'} <= listed)
        self.assertNotIn('target/debug/generated.rs', listed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
