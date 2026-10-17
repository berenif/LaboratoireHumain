#!/usr/bin/env python3
"""Synthetic criteria tests. No native executable, world, or geometry."""
import copy
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('local_pair_builder', Path(__file__).with_name('prepare_manifest.py'))
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class LocalPairCriteriaTests(unittest.TestCase):
    def setUp(self):
        self.prior = builder.base.read(builder.base.HERE / 'local-pair-criteria.json')
        self.proposed = copy.deepcopy(self.prior)
        self.proposed['physics']['maximumWallSeconds'] = 7100

    def test_only_reduced_wall_limit_is_accepted(self):
        builder.validate_criteria(self.prior, self.proposed)

    def test_current_criteria_file_matches_exact_reduction(self):
        builder.validate_criteria(self.prior, builder.base.read(builder.HERE / 'local-pair-criteria.json'))

    def test_longer_real_history_is_refused(self):
        self.proposed['physics']['maximumSecondsPerHistory'] = 8
        with self.assertRaises(ValueError):
            builder.validate_criteria(self.prior, self.proposed)

    def test_relaxed_support_threshold_is_refused(self):
        self.proposed['local']['minimumFootSupportWeightFraction'] = 0.7
        with self.assertRaises(ValueError):
            builder.validate_criteria(self.prior, self.proposed)

    def test_extra_history_is_refused(self):
        self.proposed['histories'].append({'id': 'B-repeat'})
        with self.assertRaises(ValueError):
            builder.validate_criteria(self.prior, self.proposed)

    def test_changed_fixture_is_refused(self):
        self.proposed['fixture'] = 'another-fixture'
        with self.assertRaises(ValueError):
            builder.validate_criteria(self.prior, self.proposed)

    def test_previous_wall_limit_is_refused(self):
        with self.assertRaises(ValueError):
            builder.validate_criteria(self.prior, self.prior)


if __name__ == '__main__':
    unittest.main(verbosity=2)
