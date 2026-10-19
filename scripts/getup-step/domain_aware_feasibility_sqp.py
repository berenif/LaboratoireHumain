"""Bounded genuine-domain SQP feasibility restoration.

Only the local quadratic subproblem sees linearized constraints. Every accepted
trajectory point is evaluated at its requested coordinates. An undefined IK
trial is rejected and the step is shortened, never projected or given a made-up
constraint/derivative. Feasibility termination is not a KKT/optimality claim.
"""
import hashlib
import math
import time

import numpy as np
from scipy.optimize import minimize, linprog


VERSION = 'genuine-domain-feasibility-sqp-v3-linear-feasible-start'
FEASIBILITY_TOLERANCE = 1e-10  # Numerical termination only; final gates are external.
LP_INITIAL_FEASIBILITY_TOLERANCE = 1e-9


class ComputeCap(RuntimeError):
    pass


def restore_feasibility(evaluate, jacobian, initial, anchor, bounds, *,
                        domain_exceptions, max_wall_seconds=180., max_iterations=80,
                        set_control=lambda check: None, progress_callback=None,
                        subproblem_reference='anchor',
                        linear_subproblem_policy='direct-slsqp'):
    if (not math.isfinite(max_wall_seconds) or not 0. < max_wall_seconds <= 300.
            or type(max_iterations) is not int or not 1 <= max_iterations <= 80):
        raise ValueError('one bounded restoration required')
    if subproblem_reference not in ('anchor', 'current'):
        raise ValueError('explicit anchor-projection or minimum-step restoration required')
    if linear_subproblem_policy not in ('direct-slsqp', 'row-normalized-feasible-start'):
        raise ValueError('explicit direct or row-normalized feasible-start local policy required')
    if (not isinstance(domain_exceptions, tuple) or not domain_exceptions
            or any(not isinstance(t, type) or not issubclass(t, Exception)
                   or issubclass(ComputeCap, t)
                   for t in domain_exceptions)):
        raise ValueError('explicit genuine-domain exception types required')
    x = np.array(initial, dtype=float, copy=True)
    target = np.array(anchor, dtype=float, copy=True)
    bounds = tuple((float(lo), float(hi)) for lo, hi in bounds)
    if (x.ndim != 1 or target.shape != x.shape or len(bounds) != len(x)
            or not np.isfinite(x).all() or not np.isfinite(target).all()
            or any(not math.isfinite(lo) or not math.isfinite(hi) or not lo <= q <= hi
                   for q, (lo, hi) in zip(x, bounds))):
        raise ValueError('finite initial/anchor inside declared bounds required')
    target.setflags(write=False)
    start = time.monotonic()
    trace, rejected, qp_records, lp_records = [], [], [], []
    last_requested = x.copy()
    terminal, failure, feasible = None, None, False

    def control():
        if time.monotonic() - start >= max_wall_seconds:
            raise ComputeCap('genuine-domain-restoration-compute-cap')

    def value(point):
        nonlocal last_requested
        control()
        last_requested = np.array(point, copy=True)
        c = np.array(evaluate(np.array(point, copy=True)), dtype=float, copy=True)
        control()
        if c.ndim != 1 or not len(c) or not np.isfinite(c).all():
            raise ValueError('finite genuine inequality values required')
        return c

    def violation(c):
        return float(max(0., -float(min(c))))

    def array_sha(a):
        return hashlib.sha256(np.asarray(a, dtype=np.float64).tobytes()).hexdigest()

    def witness(point, values, derivative, center, selected_bounds, scale):
        """Retain actual local answers, including failed ones; never repair them."""
        point = np.array(point, dtype=float, copy=True)
        if point.shape != center.shape or not np.isfinite(point).all():
            return dict(finiteWitness=False)
        raw = values + derivative @ (point - center)
        if not np.isfinite(raw).all():
            return dict(finiteWitness=False, returnedCoordinates=point.tolist())
        bound_violation = float(max(0., *(lo-q for q, (lo, hi) in zip(point, selected_bounds)),
                                    *(q-hi for q, (lo, hi) in zip(point, selected_bounds))))
        return dict(finiteWitness=True, returnedCoordinates=point.tolist(),
            unscaledLinearizedSlacks=raw.tolist(), minimumUnscaledLinearizedSlack=float(min(raw)),
            maximumUnscaledLinearizedViolation=violation(raw),
            minimumNormalizedLinearizedSlack=float(min(raw / scale)),
            maximumBoundViolation=bound_violation)

    try:
        set_control(control)
        c = value(x)
        for iteration in range(max_iterations + 1):
            control()
            current_violation = violation(c)
            if current_violation <= FEASIBILITY_TOLERANCE:
                feasible, terminal = True, 'numerically-feasible-pending-independent-final-gates'
                break
            if iteration == max_iterations:
                terminal = 'major-iteration-cap'
                break
            J = np.array(jacobian(x.copy()), dtype=float, copy=True)
            control()
            if J.shape != (len(c), len(x)) or not np.isfinite(J).all():
                raise ValueError('finite genuine current-point Jacobian required')
            # This is a convex LOCAL subproblem. It never calls trajectory FK.
            # Minimum-step mode is a declared feasibility-restoration phase,
            # not a claim to optimize the separately retained anchor objective.
            local_target = target if subproblem_reference == 'anchor' else x.copy()
            conditioned = linear_subproblem_policy == 'row-normalized-feasible-start'
            scale = np.maximum(np.max(abs(J), axis=1), abs(c)) if conditioned else np.ones(len(c))
            scale[scale == 0.] = 1.
            if not np.isfinite(scale).all() or np.min(scale) <= 0.:
                raise ValueError('finite strictly positive full-row normalization required')
            normalized_J = J / scale[:, None]
            local_initial = x.copy()
            local_initial_step = np.zeros(len(x))
            if conditioned:
                # One feasibility LP, in STEP coordinates, with the SAME full
                # linearization and bounds. It never invokes evaluate or FK.
                control()
                remaining = max_wall_seconds - (time.monotonic() - start)
                control()
                lp_record = dict(iteration=iteration+1, started=True,
                    linprogReturned=False, success=False, method='highs-ds',
                    currentPointCoordinates=x.tolist(), rowScaleSHA256=array_sha(scale),
                    cSHA256=array_sha(c), JSHA256=array_sha(J),
                    normalizedPrimalFeasibilityTolerance=LP_INITIAL_FEASIBILITY_TOLERANCE,
                    normalizedDualFeasibilityTolerance=LP_INITIAL_FEASIBILITY_TOLERANCE,
                    maximumWallSeconds=remaining, initializationOnly=True,
                    nonlinearCandidateOrSeed=False)
                lp_records.append(lp_record)
                lp = linprog(np.zeros(len(x)), A_ub=-normalized_J.copy(), b_ub=c/scale,
                    bounds=[(lo-q, hi-q) for q, (lo, hi) in zip(x, bounds)],
                    method='highs-ds', options=dict(presolve=True, time_limit=remaining,
                        primal_feasibility_tolerance=LP_INITIAL_FEASIBILITY_TOLERANCE,
                        dual_feasibility_tolerance=LP_INITIAL_FEASIBILITY_TOLERANCE))
                lp_record.update(linprogReturned=True, success=bool(lp.success),
                    status=int(lp.status), message=str(lp.message), localIterations=int(lp.nit))
                if lp.x is not None:
                    step = np.array(lp.x, dtype=float, copy=True)
                    if step.shape == x.shape and np.isfinite(step).all():
                        lp_record['returnedStepCoordinates'] = step.tolist()
                        local_initial_step = step.copy()
                        local_initial = x + step
                        lp_record.update(witness(local_initial, c, J, x, bounds, scale))
                control()
                if (not lp.success or not lp_record.get('finiteWitness', False)
                        or lp_record['minimumNormalizedLinearizedSlack'] < -LP_INITIAL_FEASIBILITY_TOLERANCE
                        or lp_record['maximumBoundViolation'] != 0.):
                    terminal = 'linear-feasibility-subproblem-rejected'
                    break
                lp_record['unscaledZeroToleranceSatisfied'] = lp_record['minimumUnscaledLinearizedSlack'] >= 0.
                # Keep the LP witness as a local optimizer initialization only.
                # It is NEVER sent to the genuine nonlinear evaluator as a seed.
            record = dict(iteration=iteration + 1, started=True,
                          minimizeReturned=False, success=False,
                          localObjectiveReference=subproblem_reference,
                          linearSubproblemPolicy=linear_subproblem_policy,
                          currentPointCoordinates=x.tolist(),
                          localInitialCoordinates=local_initial.tolist())
            qp_records.append(record)
            def objective(z):
                control()
                delta = z - (local_target-x) if conditioned else z-local_target
                return .5 * float(delta @ delta)
            def objective_jacobian(z):
                control()
                return z - (local_target-x) if conditioned else z-local_target
            def linear_constraints(z):
                control()
                step = z if conditioned else z - x
                return (c + J @ step) / scale
            def linear_jacobian(z):
                control()
                return normalized_J
            local = minimize(objective, local_initial_step if conditioned else local_initial,
                method='SLSQP',
                jac=objective_jacobian,
                bounds=([(lo-q, hi-q) for q, (lo, hi) in zip(x, bounds)] if conditioned else bounds),
                constraints=[dict(type='ineq', fun=linear_constraints,
                                  jac=linear_jacobian)],
                callback=lambda _: control(),
                options=dict(maxiter=100, ftol=1e-12, disp=False))
            record.update(minimizeReturned=True, success=bool(local.success),
                          message=str(local.message), localIterations=int(local.nit))
            returned = np.array(local.x, dtype=float, copy=True)
            y = (x + returned if conditioned and returned.shape == x.shape else returned)
            record.update(witness(y, c, J, x, bounds, scale))
            if conditioned:
                record['returnedStepCoordinates'] = returned.tolist()
            control()
            if (not local.success or y.shape != x.shape or not np.isfinite(y).all()
                    or any(not lo <= q <= hi for q, (lo, hi) in zip(y, bounds))
                    or (conditioned and (not record.get('finiteWitness', False)
                        or record['maximumUnscaledLinearizedViolation'] > FEASIBILITY_TOLERANCE))):
                terminal = 'local-quadratic-subproblem-rejected'
                break
            record['minimumLinearizedSlack'] = float(min(c + J @ (y - x)))
            direction = y - x
            accepted = False
            for backtrack in range(32):
                alpha = .5 ** backtrack
                trial = y.copy() if backtrack == 0 else (1. - alpha) * x + alpha * y
                try:
                    candidate = value(trial)
                except domain_exceptions as error:
                    rejected.append(dict(iteration=iteration + 1, backtrack=backtrack,
                        alpha=alpha, requestedCoordinates=trial.tolist(),
                        type=type(error).__name__, message=str(error),
                        domainReceipt=getattr(error, 'receipt', None)))
                    continue
                # Feasibility-first filter. The declared local objective owns
                # the QP direction; neither mode claims global minimization.
                trial_violation = violation(candidate)
                if (trial_violation <= FEASIBILITY_TOLERANCE
                        or trial_violation <= (1. - 1e-4 * alpha) * current_violation):
                    accepted = True
                    event = dict(iteration=iteration + 1, alpha=alpha,
                        backtracks=backtrack, previousViolation=current_violation,
                        maximumInequalityViolation=trial_violation,
                        maximumAcceptedCoordinateStep=float(max(abs(trial - x))),
                        objective=.5 * float((trial - target) @ (trial - target)),
                        coordinates=trial.tolist(), requestedPointEvaluatedExactly=True)
                    x, c = trial.copy(), candidate.copy()
                    trace.append(event)
                    if progress_callback is not None:
                        progress_callback(dict(event))
                    break
            if not accepted:
                terminal = 'genuine-domain-or-feasibility-line-search-exhausted'
                break
    except ComputeCap as error:
        terminal, failure = 'compute-cap', dict(type=type(error).__name__, message=str(error))
    except Exception as error:
        terminal, failure = 'evaluation-or-solver-error', dict(type=type(error).__name__,
            message=str(error), domainReceipt=getattr(error, 'receipt', None))
    return dict(version=VERSION, terminalState=terminal, numericallyFeasible=feasible,
        numericalFeasibilityTolerance=FEASIBILITY_TOLERANCE,
        coordinates=x.tolist(), lastRequestedCoordinates=last_requested.tolist(),
        completedMajorIterations=len(trace), progressRecords=trace,
        domainRejectedTrials=rejected, localQuadraticSubproblems=qp_records,
        linearSubproblemPolicy=linear_subproblem_policy,
        linearFeasibilitySubproblems=lp_records,
        linearFeasibilityLPCalls=len(lp_records), totalScipyLinprogCalls=len(lp_records),
        localQPUnscaledFeasibilityTolerance=FEASIBILITY_TOLERANCE if linear_subproblem_policy == 'row-normalized-feasible-start' else None,
        trajectorySolverInvocations=1, localQPOptimizerCalls=len(qp_records),
        totalScipyMinimizeCalls=len(qp_records), scalarSearchCalls=0,
        objectiveAnchor=target.tolist(), objectiveOptimalityClaim=False,
        localSubproblemObjective=('minimum-correction-from-fixed-anchor'
            if subproblem_reference == 'anchor' else 'minimum-step-from-current-iterate'),
        anchorObjectiveOptimizedDuringRestoration=subproblem_reference == 'anchor',
        solverException=failure, elapsedWallS=time.monotonic() - start,
        maxWallSeconds=max_wall_seconds, maxIterations=max_iterations,
        projectionOrFabricatedConstraintValues=False, physicalAdmission=False)
