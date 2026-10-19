"""Pure numerical restoration tests; no native model or trajectory rollout."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import domain_aware_feasibility_sqp as s


class DomainError(ValueError):
    pass


class RestorationTests(unittest.TestCase):
    def solve(self, value, jacobian, initial=(0.,), anchor=(1.,), **kw):
        return s.restore_feasibility(value, jacobian, initial, anchor, [(-2., 2.)] * len(initial),
            domain_exceptions=(DomainError,), **kw)

    def test_undefined_full_trial_backtracks_to_exact_real_point(self):
        seen = []
        def value(x):
            seen.append(float(x[0]))
            if x[0] > .6:
                raise DomainError('real fixed branch undefined')
            return np.array([x[0] - .4])
        r = self.solve(value, lambda x: np.ones((1, 1)))
        self.assertTrue(r['numericallyFeasible'])
        self.assertEqual(r['coordinates'], [.5])
        self.assertEqual(seen, [0., 1., .5])
        self.assertEqual(r['domainRejectedTrials'][0]['requestedCoordinates'], [1.])
        self.assertEqual(r['progressRecords'][0]['alpha'], .5)
        self.assertFalse(r['projectionOrFabricatedConstraintValues'])
        self.assertFalse(r['objectiveOptimalityClaim'])

    def test_nondomain_errors_do_not_become_backtracking(self):
        def value(x):
            if x[0] > 0.:
                raise RuntimeError('source hash mismatch')
            return np.array([-.4])
        r = self.solve(value, lambda x: np.ones((1, 1)))
        self.assertEqual(r['terminalState'], 'evaluation-or-solver-error')
        self.assertEqual(r['domainRejectedTrials'], [])
        self.assertEqual(r['coordinates'], [0.])

    def test_feasible_initial_uses_no_local_optimizer(self):
        r = self.solve(lambda x: np.ones(2), lambda x: self.fail('no derivative needed'))
        self.assertTrue(r['numericallyFeasible'])
        self.assertEqual(r['localQPOptimizerCalls'], 0)
        self.assertEqual(r['linearFeasibilityLPCalls'], 0)
        self.assertFalse(r['physicalAdmission'])

    def test_local_subproblem_never_evaluates_trajectory(self):
        calls = []
        def value(x):
            calls.append(x.copy())
            return np.array([x[0] - .4])
        r = self.solve(value, lambda x: np.ones((1, 1)))
        self.assertTrue(r['numericallyFeasible'])
        self.assertEqual(len(calls), 2)
        self.assertEqual(r['localQPOptimizerCalls'], 1)
        self.assertEqual(r['totalScipyMinimizeCalls'], 1)

    def test_failed_convex_subproblem_does_not_apply_its_trial(self):
        with patch.object(s, 'minimize', return_value=SimpleNamespace(
                x=np.array([1.]), success=False, message='MOCK rejected', nit=1)):
            r = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)))
        self.assertEqual(r['terminalState'], 'local-quadratic-subproblem-rejected')
        self.assertEqual(r['coordinates'], [0.])
        self.assertFalse(r['numericallyFeasible'])

    def test_budget_is_installed_and_fails_closed(self):
        controls = []
        def install(control):
            controls.append(control)
            raise s.ComputeCap('MOCK cap')
        result = self.solve(lambda x: np.ones(1), lambda x: np.ones((1, 1)), set_control=install)
        self.assertEqual(result['terminalState'], 'compute-cap')
        self.assertEqual(result['localQPOptimizerCalls'], 0)
        self.assertEqual(len(controls), 1)

    def test_raised_local_qp_is_counted_before_execution(self):
        with patch.object(s, 'minimize', side_effect=s.ComputeCap('MOCK local cap')):
            result = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)))
        self.assertEqual(result['terminalState'], 'compute-cap')
        self.assertEqual(result['localQPOptimizerCalls'], 1)
        self.assertFalse(result['localQuadraticSubproblems'][0]['minimizeReturned'])

    def test_every_local_qp_callback_observes_budget(self):
        installed = []
        def install(check):
            installed.append(check)
        def optimizer(fun, x, **kw):
            # All five entry points must reach the same control closure.
            with patch.object(s.time, 'monotonic', return_value=1e30):
                for callback in (lambda: fun(x), lambda: kw['jac'](x),
                        lambda: kw['constraints'][0]['fun'](x),
                        lambda: kw['constraints'][0]['jac'](x),
                        lambda: kw['callback'](x)):
                    with self.assertRaises(s.ComputeCap):
                        callback()
            raise s.ComputeCap('MOCK all controlled')
        with patch.object(s, 'minimize', side_effect=optimizer):
            result = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)), set_control=install)
        self.assertEqual(result['terminalState'], 'compute-cap')
        self.assertEqual(result['localQPOptimizerCalls'], 1)

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):
            self.solve(lambda x: np.ones(1), lambda x: np.ones((1, 1)), initial=(3.,))
        with self.assertRaises(ValueError):
            self.solve(lambda x: np.ones(1), lambda x: np.ones((1, 1)), max_wall_seconds=301.)

    def test_anchor_and_bounds_are_detached_from_caller_aliases(self):
        anchor, bounds = np.array([1.]), [[-2., 2.]]
        def value(x):
            anchor[0] = -1.
            bounds[0][1] = .1
            return np.array([x[0] - .4])
        result = s.restore_feasibility(value, lambda x: np.ones((1, 1)), [0.],
            anchor, bounds, domain_exceptions=(DomainError,))
        self.assertEqual(result['objectiveAnchor'], [1.])
        self.assertEqual(result['coordinates'], [1.])

    def test_initial_domain_failure_retains_structured_receipt(self):
        error = DomainError('fixed branch')
        error.receipt = dict(maximumPoseResidual=.001)
        def value(x):
            raise error
        result = self.solve(value, lambda x: np.ones((1, 1)))
        self.assertEqual(result['solverException']['domainReceipt'], error.receipt)
        self.assertEqual(result['localQPOptimizerCalls'], 0)

    def test_minimum_step_restoration_avoids_anchor_tangent_excursion(self):
        seen = []
        def value(x):
            seen.append(float(x[0]))
            if x[0] > .6:
                raise DomainError('fixed branch')
            return np.array([x[0] - .4])
        result = self.solve(value, lambda x: np.ones((1, 1)), subproblem_reference='current')
        self.assertTrue(result['numericallyFeasible'])
        np.testing.assert_allclose(result['coordinates'], [.4], atol=1e-12)
        self.assertEqual(len(seen), 2)
        self.assertEqual(result['domainRejectedTrials'], [])
        self.assertFalse(result['anchorObjectiveOptimizedDuringRestoration'])
        self.assertEqual(result['objectiveAnchor'], [1.])
        self.assertFalse(result['objectiveOptimalityClaim'])

    def test_subproblem_reference_is_explicit_not_an_automatic_scan(self):
        with self.assertRaises(ValueError):
            self.solve(lambda x: np.ones(1), lambda x: np.ones((1, 1)),
                subproblem_reference='automatic')

    def test_direct_default_does_not_call_linear_feasibility_lp(self):
        with patch.object(s, 'linprog', side_effect=AssertionError('legacy must not call LP')):
            result = self.solve(lambda x: np.array([x[0]-.4]), lambda x: np.ones((1, 1)))
        self.assertTrue(result['numericallyFeasible'])
        self.assertEqual(result['linearSubproblemPolicy'], 'direct-slsqp')
        self.assertEqual(result['linearFeasibilityLPCalls'], 0)
        self.assertIsNone(result['localQPUnscaledFeasibilityTolerance'])

    def test_normalized_full_row_algebra_step_objective_and_separate_counts(self):
        initial = np.array([.2, -.1]); anchor = np.array([.7, .3])
        c = np.array([-1., -4., 3., 0.])
        J = np.array([[2., 0.], [0., 8.], [1., -1.], [0., 0.]])
        scale = np.array([2., 8., 3., 1.]); seen = []
        step = np.array([.5, .5])
        def value(x):
            seen.append(x.copy())
            return c + J @ (x-initial)
        def lp(objective, **kw):
            self.assertEqual(len(seen), 1)
            np.testing.assert_array_equal(objective, np.zeros(2))
            np.testing.assert_array_equal(kw['A_ub'], -J/scale[:, None])
            np.testing.assert_array_equal(kw['b_ub'], c/scale)
            np.testing.assert_array_equal(kw['bounds'], np.array([[-2., 2.]]*2)-initial[:, None])
            self.assertEqual(kw['method'], 'highs-ds')
            self.assertGreater(kw['options']['time_limit'], 0.)
            self.assertLessEqual(kw['options']['time_limit'], 180.)
            return SimpleNamespace(x=step.copy(), success=True, status=0, message='MOCK LP', nit=2)
        def qp(fun, start, **kw):
            self.assertEqual(len(seen), 1)  # LP witness has not become a trajectory seed.
            np.testing.assert_array_equal(start, step)
            probe = np.array([.12, -.21])
            np.testing.assert_array_equal(kw['constraints'][0]['fun'](probe), (c+J@probe)/scale)
            np.testing.assert_array_equal(kw['constraints'][0]['jac'](probe), J/scale[:, None])
            expected = probe-(anchor-initial)
            self.assertEqual(fun(probe), .5*float(expected@expected))
            np.testing.assert_array_equal(kw['jac'](probe), expected)
            np.testing.assert_array_equal(kw['bounds'], np.array([[-2., 2.]]*2)-initial[:, None])
            return SimpleNamespace(x=step.copy(), success=True, message='MOCK QP', nit=2)
        with patch.object(s, 'linprog', side_effect=lp), patch.object(s, 'minimize', side_effect=qp):
            result = self.solve(value, lambda x: J, initial=initial, anchor=anchor,
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertTrue(result['numericallyFeasible'])
        np.testing.assert_array_equal(seen, [initial, initial+step])
        self.assertEqual(result['linearFeasibilityLPCalls'], 1)
        self.assertEqual(result['totalScipyLinprogCalls'], 1)
        self.assertEqual(result['localQPOptimizerCalls'], 1)
        self.assertEqual(result['totalScipyMinimizeCalls'], 1)
        self.assertEqual(len(result['linearFeasibilitySubproblems']), 1)
        self.assertEqual(result['localQPUnscaledFeasibilityTolerance'], 1e-10)

    def test_lp_tolerance_witness_is_honest_not_a_nonlinear_seed(self):
        seen = []
        def value(x):
            seen.append(float(x[0])); return np.array([x[0]-.4])
        with patch.object(s, 'linprog', return_value=SimpleNamespace(x=np.array([.4-5e-10]),
                success=True, status=0, message='MOCK tolerance feasible', nit=1)), \
                patch.object(s, 'minimize', return_value=SimpleNamespace(x=np.array([.4]),
                success=True, message='MOCK QP', nit=1)):
            result = self.solve(value, lambda x: np.ones((1, 1)),
                subproblem_reference='current', linear_subproblem_policy='row-normalized-feasible-start')
        self.assertTrue(result['numericallyFeasible'])
        self.assertEqual(seen, [0., .4])
        record = result['linearFeasibilitySubproblems'][0]
        self.assertFalse(record['unscaledZeroToleranceSatisfied'])
        self.assertLess(record['minimumUnscaledLinearizedSlack'], 0.)
        self.assertFalse(record['nonlinearCandidateOrSeed'])

    def test_failed_lp_has_no_qp_or_applied_trial(self):
        seen = []
        def value(x):seen.append(x.copy());return np.array([-.4])
        with patch.object(s, 'linprog', return_value=SimpleNamespace(x=None,
                success=False, status=2, message='MOCK infeasible', nit=0)), \
                patch.object(s, 'minimize', side_effect=AssertionError('no QP after failed LP')):
            result = self.solve(value, lambda x: np.ones((1, 1)),
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(result['terminalState'], 'linear-feasibility-subproblem-rejected')
        self.assertEqual(result['coordinates'], [0.])
        self.assertEqual(len(seen), 1)
        self.assertEqual(result['linearFeasibilityLPCalls'], 1)
        self.assertEqual(result['localQPOptimizerCalls'], 0)

    def test_successful_qp_with_bad_unscaled_residual_is_retained_and_rejected(self):
        with patch.object(s, 'linprog', return_value=SimpleNamespace(x=np.array([.4]),
                success=True, status=0, message='MOCK LP', nit=1)), \
                patch.object(s, 'minimize', return_value=SimpleNamespace(x=np.array([.4-2e-10]),
                success=True, message='MOCK misleading success', nit=1)):
            result = self.solve(lambda x: np.array([x[0]-.4]), lambda x: np.ones((1, 1)),
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(result['terminalState'], 'local-quadratic-subproblem-rejected')
        self.assertEqual(result['coordinates'], [0.])
        record = result['localQuadraticSubproblems'][0]
        self.assertEqual(record['currentPointCoordinates'], [0.])
        self.assertEqual(record['returnedCoordinates'], [.4-2e-10])
        self.assertEqual(len(record['unscaledLinearizedSlacks']), 1)
        self.assertGreater(record['maximumUnscaledLinearizedViolation'], 1e-10)

    def test_raised_or_capped_lp_is_counted_before_execution(self):
        with patch.object(s, 'linprog', side_effect=s.ComputeCap('MOCK LP cap')), \
                patch.object(s, 'minimize', side_effect=AssertionError('no QP')):
            result = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)),
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(result['terminalState'], 'compute-cap')
        self.assertEqual(result['linearFeasibilityLPCalls'], 1)
        self.assertEqual(result['totalScipyLinprogCalls'], 1)
        self.assertFalse(result['linearFeasibilitySubproblems'][0]['linprogReturned'])
        self.assertEqual(result['localQPOptimizerCalls'], 0)

    def test_qp_cap_after_lp_keeps_both_attempt_counts(self):
        with patch.object(s, 'linprog', return_value=SimpleNamespace(x=np.array([.4]),
                success=True, status=0, message='MOCK LP', nit=1)), \
                patch.object(s, 'minimize', side_effect=s.ComputeCap('MOCK QP cap')):
            result = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)),
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(result['terminalState'], 'compute-cap')
        self.assertEqual(result['linearFeasibilityLPCalls'], 1)
        self.assertEqual(result['localQPOptimizerCalls'], 1)
        self.assertFalse(result['localQuadraticSubproblems'][0]['minimizeReturned'])

    def test_normalized_qp_all_callbacks_control_budget(self):
        def optimizer(fun, step, **kw):
            np.testing.assert_array_equal(kw['jac'](step), step)  # current objective
            with patch.object(s.time, 'monotonic', return_value=1e30):
                for callback in (lambda: fun(step), lambda: kw['jac'](step),
                        lambda: kw['constraints'][0]['fun'](step),
                        lambda: kw['constraints'][0]['jac'](step),
                        lambda: kw['callback'](step)):
                    with self.assertRaises(s.ComputeCap):callback()
            raise s.ComputeCap('MOCK controlled normalized QP')
        with patch.object(s, 'linprog', return_value=SimpleNamespace(x=np.array([.4]),
                success=True, status=0, message='MOCK LP', nit=1)), \
                patch.object(s, 'minimize', side_effect=optimizer):
            result = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)),
                subproblem_reference='current', linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(result['terminalState'], 'compute-cap')
        self.assertEqual((result['linearFeasibilityLPCalls'], result['localQPOptimizerCalls']), (1, 1))

    def test_lp_return_control_and_exact_bound_failure_precede_qp(self):
        def capped_lp(*a, **kw):
            s.time.monotonic.return_value = 1e30
            return SimpleNamespace(x=np.array([.4]), success=True, status=0, message='MOCK LP', nit=1)
        with patch.object(s.time, 'monotonic', return_value=0.), \
                patch.object(s, 'linprog', side_effect=capped_lp), \
                patch.object(s, 'minimize', side_effect=AssertionError('no QP after cap')):
            capped = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)),
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(capped['terminalState'], 'compute-cap')
        self.assertEqual((capped['linearFeasibilityLPCalls'], capped['localQPOptimizerCalls']), (1, 0))
        self.assertTrue(capped['linearFeasibilitySubproblems'][0]['linprogReturned'])
        with patch.object(s, 'linprog', return_value=SimpleNamespace(x=np.array([2.+1e-12]),
                success=True, status=0, message='MOCK outside bound', nit=1)), \
                patch.object(s, 'minimize', side_effect=AssertionError('no clipping into QP')):
            bounded = self.solve(lambda x: np.array([-.4]), lambda x: np.ones((1, 1)),
                linear_subproblem_policy='row-normalized-feasible-start')
        self.assertEqual(bounded['terminalState'], 'linear-feasibility-subproblem-rejected')
        self.assertEqual(bounded['coordinates'], [0.])
        self.assertEqual(bounded['localQPOptimizerCalls'], 0)
        self.assertGreater(bounded['linearFeasibilitySubproblems'][0]['maximumBoundViolation'], 0.)

    def test_linear_policy_is_explicit_no_automatic_method_fallback(self):
        with self.assertRaisesRegex(ValueError, 'local policy'):
            self.solve(lambda x: np.ones(1), lambda x: np.ones((1, 1)),
                linear_subproblem_policy='automatic')


if __name__ == '__main__':
    unittest.main(verbosity=2)
