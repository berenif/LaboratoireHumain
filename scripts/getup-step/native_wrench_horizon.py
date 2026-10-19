"""One bounded native-wrench flight restoration, with independent finite gates.

This library constructs no native world and owns no attempt files. The caller
supplies one current-input fixture, original counter-zero provider, exact sole
initial candidate and source-bound static checker. All returned trajectories are
still unadmitted to runtime, native800Hz guards or the independent audit.
"""
import copy, hashlib, math, time
from pathlib import Path
from collections import OrderedDict
import numpy as np

from analytic_com_geometry import AnalyticCOMGeometry
from analytic_native_planning_sample import AnalyticNativePlanningSampler
from coupled_posture_reference import digest
from coupled_posture_kinematics import P
from domain_aware_feasibility_sqp import restore_feasibility, ComputeCap
from native_endpoint_restoration import MOTOR_CAP, OPTIMIZER_MOTOR_CAP, PLANNING_NUMERICAL_INTERIOR
from native_wrench_constraints import NativeWrenchPathEvaluator, independent_native_primal_check
from strict_reference_cache import StrictReferenceCache
from wholebody_cycle_reference import PathRejected, PHASES
from wholebody_feasibility_polish import (AnalyticPolishEvaluator, CERTIFICATE_FRACTIONS,
    _sample_row, summarize_samples)

VERSION = 'native-wrench-whole-flight-physical-lumbar-feasibility-v2'
ANCHOR_SHA256 = '0959186db884653611f946799df1a515871df194d6993fb892c349be98232e77'
DESIGN_REVISION = dict(schema=1,kind='causal-physical-lumbar-only-v1',
    sourceDiagnosticReceiptFilename='native-held-002-linear-feasibility-diagnostic.json',
    sourceDiagnosticReceiptSHA256='c670daa6f173ea4f272c7325d1c3906bbda578f3768c02d6bd2df7a6dc9b259a',
    sourceLinearizationFilename='native-held-002-linearization.npz',
    sourceLinearizationSHA256='0e5a95a37dd52080929e05925304202fb73dad898f4fc3a8c1239dcd9d8e0db3',
    originalExperimentalWindowDeg=5.,sourcePhysicalLumbarLimitsUnchanged=True,
    allOtherHardLimitsUnchanged=True,nonlinearAdmissionInherited=False)
DECLARED_BRACKETS = (
    dict(phase='lift', bracketFractions=[.775, .8, .8250000000000001]),
    dict(phase='land', bracketFractions=[.7000000000000001, .7250000000000001, .75]))
SOURCES = ('native_wrench_horizon.py', 'native_wrench_constraints.py',
    'native_planned_jet_provider.py', 'domain_aware_feasibility_sqp.py',
    'analytic_native_planning_sample.py', 'analytic_com_geometry.py',
    'wholebody_cycle_reference.py', 'wholebody_feasibility_polish.py',
    'wholebody_activation_fixture.py', 'coupled_posture_reference.py',
    'coupled_posture_kinematics.py', 'source_reference_context.py',
    'phase_leg_reference.py', 'strict_reference_cache.py', 'native_endpoint_restoration.py',
    'source_posture_retarget.py','source_step_probe.py','source_dynamics.c')


class GenuineAnalyticPolishEvaluator(AnalyticPolishEvaluator):
    """Preserve genuine values; reconstruct ONLY retained Newton-domain errors."""
    def evaluate(self, x):
        self.check_control()
        values, rows = super().evaluate(x)
        # A budget exception caught by the older prescreen must remain a real
        # budget exception, never a synthetic domain or physical constraint.
        self.check_control()
        errors = [row for row in rows if 'error' in row or 'errorType' in row]
        if errors:
            if len(errors) == 1 and errors[0].get('errorType') == 'PathRejected':
                raise PathRejected(errors[0].get('error', 'original fixed IK branch rejected'),
                    originalPrescreenError=copy.deepcopy(errors[0]), originalErrorType='PathRejected')
            raise ValueError('non-domain analytic prescreen rejection: ' + repr(errors))
        expected = len(self.nodes) * len(self.labels) + len(self.envelope_specs)
        if (np.shape(values) != (expected,) or not np.isfinite(values).all()
                or len(rows) != len(self.nodes) + len(self.envelope_specs)
                or any('maximumPoseResidual' not in row for row in rows)):
            raise ValueError('complete genuine analytic constraint baseline required')
        return values, rows


def finite_command_nodes(durations):
    original = [(k, float(f * T), float(f)) for k, T in enumerate(durations)
                for f in CERTIFICATE_FRACTIONS]
    dense = [(k, float(t), float(t / T)) for k, T in enumerate(durations)
             for t in np.linspace(0., T, int(math.ceil(T / .005)) + 1)]
    return original, dense


def _lp_pass(lp, motor_key, equilibrium_key):
    if not isinstance(lp,dict):return False
    def below(key, cap):
        value = lp.get(key)
        return isinstance(value, (float, int, np.floating, np.integer)) and math.isfinite(value) and 0. <= value < cap
    return bool(lp.get('success') and below(motor_key, MOTOR_CAP)
        and below(equilibrium_key, 1e-8) and below('maximumFrictionViolation', 1e-8))


def certificate_binding(path, fixture):
    return dict(pathSHA256=path.key,candidateSHA256=digest(np.asarray(path.independent_knots).tolist()),
        inputBindingSHA256=digest(fixture.binding),contextSHA256=fixture.context.receipt()['contextSHA256'])


def actual_contact_binding(fixture, native_provider, cone):
    corners=tuple(fixture.support.surviving_corners)
    if len(corners)!=3 or len(set(corners))!=3:raise ValueError('exact retained three surviving stance corners required')
    points=np.asarray(fixture.support.stance_points_xyz,dtype=np.float64)[list(corners)]
    side=fixture.realizer.stance_side;stance=np.asarray(fixture.realizer.stance_pose.T,dtype=np.float64)
    origin=P@stance[:3,3]
    def same(a,b,shape):
        a=np.asarray(a,dtype=np.float64);b=np.asarray(b,dtype=np.float64)
        return a.shape==shape and b.shape==shape and a.tobytes()==b.tobytes()
    if not same(cone.points,points,(3,3)):raise ValueError('cone contacts differ from exact current surviving stance corners/order')
    if not same(cone.origin,origin,(3,)):raise ValueError('cone origin differs from fixed actual source stance-body origin')
    if hasattr(native_provider,'contacts'):
        contacts=tuple(native_provider.contacts)
        if(len(contacts)!=3 or any(s!=side for s,p in contacts)
                or not same([p for s,p in contacts],points,(3,3))):raise ValueError('native provider actual contact side/order differs')
    if hasattr(native_provider,'origin')and not same(native_provider.origin,origin,(3,)):raise ValueError('native provider fixed origin differs')
    if hasattr(native_provider,'stance_pose')and not same(native_provider.stance_pose,stance,(4,4)):raise ValueError('native provider fixed actual stance pose differs')
    if hasattr(native_provider,'stance_side')and native_provider.stance_side!=side:raise ValueError('native provider stance side differs')
    return dict(stanceSide=side,survivingCorners=list(corners),actualContactPointsSource=points.tolist(),
        actualContactPointBytesSHA256=hashlib.sha256(points.tobytes()).hexdigest(),fixedOriginSource=origin.tolist(),
        fixedStancePoseModel=stance.tolist(),providerContactFieldsChecked=hasattr(native_provider,'contacts'))


def certify_finite_horizon(path, fixture, native_provider, cone, static_checker, *,
                           max_wall_seconds=180., cancellation_check=None):
    """Original strict samples/native primal LPs, no world construction.

    static_checker(sample,data) must run the ORIGINAL static strict_contact_lp
    after explicitly installing that SAME qpos with zero velocity in scratch.
    It returns the original LP receipt. Every failure remains in the result.
    """
    if not callable(static_checker):raise ValueError('injected original same-pose static checker required')
    if not math.isfinite(max_wall_seconds) or not 0. < max_wall_seconds <= 180.:raise ValueError('certificate wall cap in(0,180] required')
    started = time.monotonic(); sampler = StrictReferenceCache(); cache = OrderedDict()
    original_nodes, dense_nodes = finite_command_nodes(path.durations)
    original, dense = [], []; stop = None; hits = misses = 0
    def control():
        if cancellation_check is not None and cancellation_check():raise ComputeCap('certificate-caller-cancelled')
        if time.monotonic() - started >= max_wall_seconds:raise ComputeCap('certificate-compute-cap')
    def sample_at(k, t, f):
        nonlocal hits, misses
        control(); key = (k, float(t))
        if key in cache:hits += 1;return dict(cache[key], fraction=f, phaseTimeS=t)
        misses += 1; row = dict(phase=PHASES[k], fraction=f, phaseTimeS=t, accepted=False,
            originalStrictMapEvaluationPassed=False)
        try:
            sample = sampler.sample(path, PHASES[k], t);control()
            row.update(_sample_row(path, sample, k, f, fixture.support, np.asarray(fixture.heading),
                np.asarray(fixture.context.com.position), np.asarray(fixture.context.joints.position)[:15]))
            geometry_ok = row['accepted'];row['geometryAccepted'] = geometry_ok;row['accepted'] = False
            data = native_provider(sample, final=False);control()
            primal = independent_native_primal_check(data, cone);control()
            native_ok = bool(primal.get('accepted') and _lp_pass(primal, 'maximumMotorFraction', 'maximumNativeEquilibriumResidual')
                and primal.get('normalForceUpperBound', 'missing') is None and primal.get('friction') == .6)
            gap = float(data['minimumCleanSelfGapM']);height = float(data['nativeCOMHeightM'])
            physical_ok = bool(math.isfinite(gap) and math.isfinite(height) and gap > .025
                and height >= .65 and float(sample.com.position[2]) >= .65)
            static = None; static_ok = True
            if (k == 0 and t == 0.) or t == path.durations[k]:
                static = static_checker(sample, data);control()
                static_ok = _lp_pass(static, 'requiredMotorFraction', 'maximumEqualityResidual')
            row.update(accepted=bool(geometry_ok and native_ok and physical_ok and static_ok),
                plannedNativeAccepted=native_ok, originalPhysicalGuardsAccepted=physical_ok,
                plannedNativePrimalLP=copy.deepcopy(primal), native=copy.deepcopy(data.get('native', {})),
                additionalOriginalStaticLP=copy.deepcopy(static), additionalStaticAccepted=static_ok,
                maximumMotorFraction=primal.get('maximumMotorFraction'), minimumCleanSelfGapM=gap,
                nativeCOMHeightM=height, commandCOMHeightM=float(sample.com.position[2]), nativeAdmission=False)
        except ComputeCap:raise
        except Exception as exc:row.update(accepted=False, errorType=type(exc).__name__, error=str(exc), domainReceipt=getattr(exc, 'receipt', None))
        cache[key] = copy.deepcopy(row)
        if len(cache) > 1024:cache.popitem(last=False)
        return row
    try:
        for node in original_nodes:original.append(sample_at(*node))
        for node in dense_nodes:dense.append(sample_at(*node))
        control()
    except ComputeCap as exc:stop = str(exc)
    complete = len(original) == 123 and len(dense) == 603
    passed = bool(stop is None and complete and all(row['accepted'] for row in original + dense))
    intervals = [T / math.ceil(T / .005) for T in path.durations]
    return dict(schema=1, version=VERSION, passed=passed, complete=complete, termination=stop,
        candidateCertificateBinding=certificate_binding(path,fixture),
        originalStrictMapUsed=True, original123Certificate=dict(passed=len(original)==123 and all(r['accepted']for r in original),
            sampleCount=len(original), declaredSampleCount=123),
        denseCommandGridCertificate=dict(passed=len(dense)==603 and all(r['accepted']for r in dense),sampleCount=len(dense),
            declaredSampleCount=603,maximumStepS=max(intervals),phaseDurationS=list(path.durations),originalStrictMapUsed=True,endpointInclusive=True),
        analyticSamples=original,denseCommandSamples=dense,failures=[r for r in original+dense if not r['accepted']],
        nominalPhysicalMetrics=summarize_samples(original),densePhysicalMetrics=summarize_samples(dense),
        cacheCounters=dict(hits=hits,misses=misses,capacity=1024),elapsedWallS=time.monotonic()-started,maxWallSeconds=max_wall_seconds,
        finiteSamplesOnly=True,wholePathProven=False,physicsIntegrations=0,nativeWorldConstructions=0,nativeAdmission=False,
        live800HzGuardsFresh21WindowAndIndependentAuditStillRequired=True)


class NativeWrenchHorizon:
    """Existing1102 geometric rows plus original13 native-wrench blocks."""
    def __init__(self, fixture, native_provider, cone, anchor_knots, initial_knots, *, initial_binding, static_checker):
        anchor=np.asarray(anchor_knots,dtype=np.float64);initial=np.asarray(initial_knots,dtype=np.float64)
        if anchor.shape!=(3,9)or initial.shape!=(3,9)or not np.isfinite(anchor).all()or not np.isfinite(initial).all():raise ValueError('finite exact3x9 anchor/sole initial required')
        if digest(anchor.tolist())!=ANCHOR_SHA256:raise ValueError('unchanged095918 minimum-correction anchor required')
        if not isinstance(initial_binding,dict)or initial_binding.get('candidateSHA256')!=digest(initial.tolist()):raise ValueError('explicit exact sole initial-candidate provenance required')
        if not callable(native_provider)or not callable(static_checker):raise ValueError('injected original native provider/static checker required')
        if (fixture.context.phase!='transfer'or fixture.binding.get('entryLiftElapsedS')!=0.
                or fixture.binding.get('contextSHA256')!=fixture.context.receipt()['contextSHA256']
                or fixture.binding.get('currentSupportSHA256')!=fixture.support.key or len(fixture.support.equations())!=9):raise ValueError('same-current stationary transfer/nine actual support faces required')
        contacts=actual_contact_binding(fixture,native_provider,cone)
        self.fixture=fixture;self.provider=native_provider;self.cone=cone;self.anchor=anchor.copy();self.initial=initial.reshape(27).copy();self.static_checker=static_checker
        self.anchor.setflags(write=False);self.initial.setflags(write=False)
        geometry=AnalyticCOMGeometry(fixture.realizer)
        self.kin=GenuineAnalyticPolishEvaluator(fixture.realizer,fixture.context,fixture.limits,fixture.heading,fixture.support,anchor,
            entry_elapsed=0.,envelope_brackets=copy.deepcopy(DECLARED_BRACKETS),global_support_phases=PHASES,node_geometry=geometry,lumbar_window_deg=None)
        if len(self.kin.nodes)*len(self.kin.labels)+len(self.kin.envelope_specs)!=1102:raise ValueError('unchanged1102-row all-phase geometry profile required')
        self.evaluator=NativeWrenchPathEvaluator(self.kin,native_provider,cone,sample_provider=AnalyticNativePlanningSampler(fixture.realizer))
        self.binding=dict(schema=1,version=VERSION,inputBinding=copy.deepcopy(fixture.binding),initialCandidate=copy.deepcopy(initial_binding),
            initialCandidateSHA256=digest(initial.tolist()),initialIndependentKnotsRad=initial.tolist(),objectiveAnchorSHA256=ANCHOR_SHA256,
            objectiveAnchorIndependentKnotsRad=anchor.tolist(),sourceJointLimitsRad=np.asarray(fixture.limits).tolist(),
            independentBoundsRad=[list(b)for b in self.evaluator.bounds],pathPhaseDurationS=list(self.kin.entry.durations),
            declaredAdditionalEnvelopeBrackets=copy.deepcopy(DECLARED_BRACKETS),globalPhaseSupportEnvelopeCoverage=self.kin.phase_coverage_receipt(),
            geometryConstraintRows=1102,nativeConstraintNodes=13,nativeConstraintRows=13*(42+len(cone.H)+3),
            contactWrenchCone=copy.deepcopy(cone.receipt),actualContactBinding=contacts,originalStrictMotorCap=MOTOR_CAP,optimizerMotorCap=OPTIMIZER_MOTOR_CAP,
            planningNumericalInterior=PLANNING_NUMERICAL_INTERIOR,experimentalLumbarWindowDeg=None,experimentalLumbarWindowRemoved=True,
            experimentalLumbarIsPhysicalLimit=False,designRevision=copy.deepcopy(DESIGN_REVISION),
            linearSubproblemPolicy='row-normalized-feasible-start',
            noHyperextension=True,sourceJointInteriorRad=.01,armAndMapReserveRad=.02,originalPhysicalCOMHeightFloorM=.65,
            minimumCleanSelfGapM=.025,conservativeFriction=.6,normalForceDomain='nonnegative-unbounded',
            frozenSourceHashes={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()for name in SOURCES},
            nativeWorldConstructions=0,physicsIntegrations=0,nativeAdmission=False)

    def solve(self, *, max_wall_seconds=300.,max_iterations=80,certificate_max_wall_seconds=180.,
              progress_callback=None,cancellation_check=None,certificate_function=certify_finite_horizon,subproblem_reference='current'):
        if not math.isfinite(certificate_max_wall_seconds)or not 0.<certificate_max_wall_seconds<=180.:raise ValueError('certificate wall cap in(0,180] required before solver')
        if not callable(certificate_function):raise ValueError('callable independent finite certificate required')
        if subproblem_reference not in('anchor','current'):raise ValueError('explicit anchor projection or minimum-step feasibility mode required')
        def set_control(control):
            def checked():
                if cancellation_check is not None and cancellation_check():raise ComputeCap('horizon-caller-cancelled')
                control()
            self.kin.check_control=checked
        def progress(event):
            event=dict(event,unadmittedIndependentKnotsRad=np.asarray(event['coordinates']).reshape(3,9).tolist(),
                candidateSHA256=digest(np.asarray(event['coordinates']).reshape(3,9).tolist()),nativeAdmission=False)
            if progress_callback is not None:progress_callback(copy.deepcopy(event))
        solver=restore_feasibility(lambda x:self.evaluator.evaluate(x)[0],self.evaluator.jacobian,self.initial,self.anchor.reshape(27),
            self.evaluator.bounds,domain_exceptions=(PathRejected,),max_wall_seconds=max_wall_seconds,max_iterations=max_iterations,
            set_control=set_control,progress_callback=progress,subproblem_reference=subproblem_reference,
            linear_subproblem_policy='row-normalized-feasible-start')
        final=np.asarray(solver['coordinates'],dtype=np.float64);path=self.kin.path(final,strict_map=True)
        certificate_attempted=solver['numericallyFeasible']is True
        if certificate_attempted:
            certificate=certificate_function(path,self.fixture,self.provider,self.cone,self.static_checker,
                max_wall_seconds=certificate_max_wall_seconds,cancellation_check=cancellation_check)
        else:
            certificate=dict(schema=1,version=VERSION,passed=False,complete=False,notAttempted=True,
                termination='not-attempted-solver-not-numerically-feasible',originalStrictMapUsed=False,
                candidateCertificateBinding=certificate_binding(path,self.fixture),analyticSamples=[],denseCommandSamples=[],failures=[],
                certificateRequiredBeforeAnyAdmission=True,finiteSamplesOnly=True,wholePathProven=False,
                nativeWorldConstructions=0,physicsIntegrations=0,nativeAdmission=False)
        expected_original,expected_dense=finite_command_nodes(path.durations)
        def complete_rows(key,expected):
            rows=certificate.get(key,[])
            if len(rows)!=len(expected):return False
            for row,(k,t,f)in zip(rows,expected):
                primal=row.get('plannedNativePrimalLP')or{}
                if not(row.get('phase')==PHASES[k]and row.get('fraction')==f and row.get('accepted')is True
                        and row.get('originalStrictMapEvaluationPassed')is True and row.get('geometryAccepted')is True
                        and row.get('plannedNativeAccepted')is True and row.get('originalPhysicalGuardsAccepted')is True
                        and primal.get('accepted')is True and primal.get('normalForceUpperBound','missing')is None
                        and primal.get('friction')==.6 and _lp_pass(primal,'maximumMotorFraction','maximumNativeEquilibriumResidual')):return False
                if (k==0 and t==0.)or t==path.durations[k]:
                    if not(row.get('additionalStaticAccepted')is True
                            and _lp_pass(row.get('additionalOriginalStaticLP',{}),'requiredMotorFraction','maximumEqualityResidual')):return False
            return True
        passed=bool(solver['numericallyFeasible']and certificate.get('passed')is True and certificate.get('complete')is True
            and certificate.get('originalStrictMapUsed')is True
            and certificate.get('candidateCertificateBinding')==certificate_binding(path,self.fixture)
            and complete_rows('analyticSamples',expected_original)and complete_rows('denseCommandSamples',expected_dense))
        binding=dict(copy.deepcopy(self.binding),subproblemReference=subproblem_reference,
            localSubproblemObjective='minimum-step-from-current-iterate'if subproblem_reference=='current'else'minimum-correction-from-fixed-anchor',
            fixed095918AnchorRetainedForProvenance=True,anchorObjectiveOptimizedDuringRestoration=subproblem_reference=='anchor')
        receipt=dict(schema=1,version=VERSION,binding=binding,domainAwareFeasibilitySolver=solver,
            trajectorySolverInvocations=solver['trajectorySolverInvocations'],localQPOptimizerCalls=solver['localQPOptimizerCalls'],
            totalScipyMinimizeCalls=solver['totalScipyMinimizeCalls'],optimizerCalls=solver['totalScipyMinimizeCalls'],
            linearSubproblemPolicy=solver['linearSubproblemPolicy'],linearFeasibilityLPCalls=solver['linearFeasibilityLPCalls'],
            totalScipyLinprogCalls=solver['totalScipyLinprogCalls'],linearFeasibilitySubproblems=solver['linearFeasibilitySubproblems'],
            numericalFeasibilityOnly=True,objectiveOptimalityClaim=False,finalCandidateSHA256=digest(final.reshape(3,9).tolist()),
            subproblemReference=subproblem_reference,anchorObjectiveOptimizedDuringRestoration=subproblem_reference=='anchor',
            unadmittedIndependentKnotsRad=final.reshape(3,9).tolist(),strictFiniteCertificate=certificate,
            certificateFunction=dict(module=getattr(certificate_function,'__module__',type(certificate_function).__module__),
                name=getattr(certificate_function,'__name__',type(certificate_function).__name__),attempted=certificate_attempted,
                originalDefaultCertificateUsed=bool(certificate_attempted and certificate_function is certify_finite_horizon)),
            finiteNativeFlightFeasible=passed,accepted=False,nativeAdmission=False,wholePathProven=False,
            lastRequestedCoordinates=solver['lastRequestedCoordinates'],domainRejectedTrials=solver['domainRejectedTrials'],
            genuineEvaluatorLastRejection=copy.deepcopy(self.evaluator.last_rejection),
            geometryCache=self.kin.cache_receipt(),nativeCache=copy.deepcopy(self.evaluator.stats),
            physicsIntegrations=0,nativeWorldConstructions=0,noSeedGoalWeightOrRadiusScan=True,
            live800HzGuardsFresh21WindowAndIndependentAuditStillRequired=True)
        return (path if passed else None),receipt
