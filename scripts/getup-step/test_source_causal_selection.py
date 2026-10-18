"""State-only support observation retains every measured admission gate."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import source_robust_support_steps as robust
import source_reviewed_walk as reviewed
from source_reviewed_walk import ReviewedController


class CausalSelection(unittest.TestCase):
    def controller(self, mode='continuous-com-causal'):
        state = SimpleNamespace(time=0., loaded=True, risk=0, present=np.ones(4, dtype=bool))
        c = ReviewedController.__new__(ReviewedController)
        c.install_reviewed(1, mode)
        c.phase_duration = lambda phase: 2.4
        c.stance_side = 0
        c.com0, c.com1 = np.zeros(3), np.array([.01, .01, .5])
        c.com0[2] = .5
        def arr(index, shape):
            if index == 20: return np.array([9.81 if state.loaded else 0.]+[0.]*16)
            if index == 6: return np.tile([.01, .01, .5], (17, 1))
            if index == 1: return np.zeros(27)
            raise AssertionError(index)
        c.w = SimpleNamespace(h=None, mass=1., arr=arr,
            read=lambda *args: np.zeros((3, 27)),
            feet=lambda: [dict(body=0), dict(body=1)],
            l=SimpleNamespace(lh_time=lambda h: state.time))
        c._forecast_unload_risk = lambda *args: (state.risk, dict(mock=True))
        c.admit_robust_reference = lambda *args: True
        corners = np.array([[-.05, -.05, 0.], [.05, -.05, 0.], [.05, .05, 0.], [-.05, .05, 0.]])
        return c, state, lambda *args: (corners, state.present.copy(), np.zeros(4))

    def observe(self, c, state, patch_value, times):
        with patch.object(robust, 'actual_stance_patch', side_effect=patch_value):
            for t in times:
                state.time = float(t)
                c._observe_support(float(t), 'transfer')

    def test_same_history_can_select_before_nominal_timer_only_in_new_mode(self):
        for mode, selected in [('continuous-com', False), ('continuous-com-causal', True)]:
            c, state, geometry = self.controller(mode)
            self.observe(c, state, geometry, np.arange(21)*.005)
            self.assertEqual(c.robust_support['selected'], selected)
            if selected:
                self.assertEqual(c.robust_support['historySamples'], 21)
                self.assertEqual(c.robust_support['selectedAtTransferS'], .1)
                self.assertEqual(c._robust_duration, 1.)
            else:
                self.assertEqual(len(c._robust_history), 0)

    def test_twenty_samples_are_insufficient(self):
        c, state, geometry = self.controller()
        self.observe(c, state, geometry, np.arange(20)*.005)
        self.assertFalse(c.robust_support['selected'])

    def test_unloaded_sample_clears_history(self):
        c, state, geometry = self.controller()
        self.observe(c, state, geometry, np.arange(20)*.005)
        state.loaded = False
        self.observe(c, state, geometry, [.1])
        self.assertEqual(len(c._robust_history), 0)
        self.assertFalse(c.robust_support['selected'])

    def test_observation_gap_restarts_full_history(self):
        c, state, geometry = self.controller()
        self.observe(c, state, geometry, np.arange(20)*.005)
        self.observe(c, state, geometry, [.11])
        self.assertEqual(len(c._robust_history), 1)
        self.assertFalse(c.robust_support['selected'])

    def test_missing_actual_survivor_or_disagreement_cannot_select(self):
        for missing, risk in [(True, 0), (False, -1)]:
            c, state, geometry = self.controller()
            if missing: state.present[1] = False
            state.risk = risk
            self.observe(c, state, geometry, np.arange(30)*.005)
            self.assertFalse(c.robust_support['selected'])

    def test_overlap_rejection_does_not_commit_correction(self):
        c, state, geometry = self.controller()
        calls = []
        c.admit_robust_reference = lambda *args: calls.append(args) or False
        self.observe(c, state, geometry, np.arange(21)*.005)
        self.assertEqual(len(calls), 1)
        self.assertFalse(c.robust_support['selected'])
        self.assertFalse(hasattr(c, '_robust_delta'))

    def test_bounded_correction_is_required_before_overlap_hook(self):
        c, state, geometry = self.controller()
        c.com1[:2] = [-.04, -.04]
        c.admit_robust_reference = lambda *args: self.fail('out-of-bound correction reached hook')
        self.observe(c, state, geometry, np.arange(21)*.005)
        self.assertFalse(c.robust_support['selected'])
        self.assertGreater(c.robust_support['lastRejectedCorrectionM'], .012)

    def test_real_overlap_hook_commits_only_certified_finite_plan(self):
        c, _, _ = self.controller()
        c.com1 = c.com0+np.array([.1, 0., 0.])
        self.assertTrue(ReviewedController.admit_robust_reference(c, .5, np.array([.004, 0., 0.]), 1.))
        self.assertTrue(c._causal_overlap_plan['passed'])
        c._causal_overlap_plan = None
        c.com1 = c.com0+np.array([1., 0., 0.])
        self.assertFalse(ReviewedController.admit_robust_reference(c, .5, np.zeros(3), 1.))
        self.assertIsNone(c._causal_overlap_plan)
        self.assertIn('totalSpeed', c.robust_support['lastRejectedCausalOverlap']['rejections'])

    def test_existing_mode_never_calls_new_certifier(self):
        c, _, _ = self.controller('continuous-com')
        with patch.object(reviewed, 'certify_overlap', side_effect=AssertionError('legacy certifier called')):
            self.assertTrue(ReviewedController.admit_robust_reference(c, .5, np.zeros(3), 1.))


if __name__ == '__main__': unittest.main()
