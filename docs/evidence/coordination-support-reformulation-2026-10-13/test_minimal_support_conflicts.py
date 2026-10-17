"""Pure certificate checks; no native libraries, model, simulation, FK or LP."""
import copy
import json
from pathlib import Path
import unittest
import minimal_support_conflicts as proof

HERE = Path(__file__).resolve().parent


class MinimalConflictTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures = json.loads((HERE/'archived-support-problems.json').read_text())
        cls.report = json.loads((HERE/'minimal-support-conflicts.json').read_text())

    def test_eight_positive_dual_certificates_and_64_exact_removal_witnesses(self):
        self.assertEqual(len(self.report['systems']), 8)
        self.assertEqual(self.report['exactRemovalWitnessCount'], 64)
        for system in self.report['systems']:
            self.assertTrue(proof.verify_iis(system))

    def test_report_reproduces_from_archived_coefficients(self):
        rebuilt = [proof.build_iis(case, side) for case in self.fixtures['cases'] for side in range(2)]
        self.assertEqual(rebuilt, self.report['systems'])
        self.assertEqual(self.fixtures['sourceRawSha256'], self.report['sourceRawSha256'])

    def test_corrupt_witness_is_refused(self):
        system = copy.deepcopy(self.report['systems'][0])
        point = system['removalWitnesses'][0]['coordinates'][0]
        point['numerator'] = str(int(point['numerator'])+int(point['denominator']))
        with self.assertRaises(AssertionError):
            proof.verify_iis(system)

    def test_nonpositive_dual_weight_is_refused(self):
        system = copy.deepcopy(self.report['systems'][0])
        system['positiveDualWeights'][0]['numerator'] = '-1'
        with self.assertRaises(AssertionError):
            proof.verify_iis(system)

    def test_original_tolerance_is_included_in_selected_bounds(self):
        for case, systems in zip(self.fixtures['cases'], [self.report['systems'][i:i+2] for i in range(0,8,2)]):
            for side, system in enumerate(systems):
                y = case['problem']['hard'][3*side+1]
                self.assertEqual(proof.decode(system['expandedUpperBounds'][0]), -proof.fraction(y['lower'])+proof.EPS)

    def test_two_components_already_conflict_for_three_feedback_scales(self):
        auxiliary = json.loads((HERE/'two-component-conflicts.json').read_text())
        self.assertEqual(len(auxiliary['checks']), 8)
        for case, pair in zip(self.fixtures['cases'], [auxiliary['checks'][i:i+2] for i in range(0,8,2)]):
            p = case['problem']
            lo, hi = [list(map(proof.fraction,p[k])) for k in ('lower','upper')]
            for side, item in enumerate(pair):
                ty, tz = p['hard'][3*side+1:3*side+3]
                y, z = [list(map(proof.fraction,t['row'])) for t in (ty,tz)]
                t = y[32+7*side]/z[32+7*side]
                row = [a-t*b for a,b in zip(y,z)]
                gap = proof.fraction(ty['lower'])-t*proof.fraction(tz['upper'])
                gap -= sum(a*(hi[k] if a>=0 else lo[k]) for k,a in enumerate(row))
                gap -= proof.EPS*(1+t+sum(map(abs,row)))
                self.assertEqual(gap,proof.decode(item['gapAfterOriginalTolerances']))
                self.assertEqual(gap>0,case['feedbackScale']>0)

    def test_witness_scope_does_not_claim_whole_qp_becomes_feasible(self):
        for case, systems in zip(self.fixtures['cases'], [self.report['systems'][i:i+2] for i in range(0,8,2)]):
            for side, system in enumerate(systems):
                other_y = case['problem']['hard'][3*(1-side)+1]
                # The documented extension sets inactive axes to zero. The
                # other foot then still violates its own lower material row.
                v = [proof.F(0)]*46
                for axis, encoded in zip(system['axisIndices'], system['removalWitnesses'][0]['coordinates']):
                    v[axis] = proof.decode(encoded)
                actual = proof.dot(list(map(proof.fraction,other_y['row'])),v)
                self.assertLess(actual, proof.fraction(other_y['lower'])-proof.EPS)
                self.assertFalse(system['minimumCardinalityClaimed'])
                self.assertTrue(system['minimalByInclusion'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
