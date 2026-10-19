"""One analytic, minimum-norm feasibility restoration of bound recovery iteration33.

This is a second, explicitly linked numerical producer stage. It cannot turn the
failed recovery into convergence, extrapolate an arm map, install a native pose,
or certify the continuous nonlinear path. The library owns no attempt files.
"""
import copy,json,math,time,traceback
from collections import OrderedDict
import numpy as np
from scipy.optimize import minimize,minimize_scalar
from scipy.spatial.transform import Rotation
from coupled_posture_reference import digest,C2Segment,Jet
from wholebody_cycle_reference import (WholeBodyCycle,PHASES,LEG_INDICES,VERSION,
    MAX_TOTAL_COM_SPEED,MAX_TOTAL_COM_ACCELERATION)

VERSION_POLISH='free-com-flight-feasibility-polish-v1'
VERSION_REFINEMENT='free-com-flight-collocation-refinement-v1'
VERSION_FRESH_RECOVERY='free-com-flight-fresh-recovery-v1'
FRACTIONS=(0.,.25,.5,.75,1.)
CERTIFICATE_FRACTIONS=tuple(float(x)for x in np.linspace(0.,1.,41))

class _ControlStop(RuntimeError):
    def __init__(self,kind):super().__init__(kind);self.kind=kind

class OptimizerRawDomainArmMap:
    """Optimizer ONLY: same original spline, inside its ORIGINAL RAW domain.

    The original reserved .02 inequality is independently unchanged in every
    constraint block. Strict final/path/runtime samples never use this proxy.
    No clipping or even the source evaluator's1e-12 raw extrapolation is allowed.
    """
    def __init__(self,original):self.original=original
    def __getattr__(self,name):return getattr(self.original,name)
    def evaluate(self,delta,reserve=.02):
        p=np.asarray(delta.position);ranges=np.asarray(self.original.ranges)
        if p.shape!=(2,)or not np.isfinite(p).all()or np.any(p<ranges[:,0])or np.any(p>ranges[:,1]):
            raise ValueError('optimizer sample outside ORIGINAL RAW verified map; no extrapolation')
        return self.original.evaluate(delta,reserve=0.)


def _bounds(realizer,context,limits):
    q0=np.array(context.joints.position)[:15]
    lo=limits[:15,0]+.01;hi=limits[:15,1]-.01
    lo[:3]=np.maximum(lo[:3],q0[:3]-math.radians(5.))
    hi[:3]=np.minimum(hi[:3],q0[:3]+math.radians(5.));hi[[6,12]]=np.minimum(hi[[6,12]],0.)
    return q0,lo,hi


def _stage_binding(binding,anchor):
    result=dict(binding)
    expected=dict(sourceCandidateMajorIteration=33,sourceCandidateSHA256=digest(anchor.reshape(3,9).tolist()),
        sourceRecoveryOptimizerCalls=1,sourceRecoveryOptimizerSuccess=False,sourceRecoveryTermination='compute-cap')
    for key,value in expected.items():
        if key not in result or type(result[key])is not type(value)or result[key]!=value:
            raise ValueError('bound failed recovery stage mismatch: '+key)
    for key in('sourceRecoverySHA256','sourceDiagnosticSHA256'):
        value=result.get(key)
        if not isinstance(value,str)or len(value)!=64 or any(x not in'0123456789abcdef'for x in value):
            raise ValueError('exact source SHA256 required: '+key)
    result.update(schema=1,sourceCandidateIndependentKnotsRad=anchor.reshape(3,9).tolist(),polishOptimizerCalls=1,
        numericalFeasibilityOnly=True,originalHardLimitsUnchanged=True,originalFootPathAndClocksUnchanged=True,
        analyticConstraintJets=True,goalOrWeightScan=False,nativeWorldConstructions=0,physicsIntegrations=0)
    return result


def fresh_recovery_binding(value,anchor,initial,input_binding,context):
    """New causal problem after missing raw proofs; never forge old stages."""
    result=copy.deepcopy(value);key=digest(anchor.reshape(3,9).tolist())
    expected=dict(schema=1,sourceCandidateSHA256=key,objectiveAnchorSHA256=key,initialContinuationCandidateSHA256=key,
        regeneratedActivationInputSHA256=input_binding.get('inputSHA256'),inputBindingSHA256=digest(input_binding),
        activationContextSHA256=context.receipt()['contextSHA256'],actualPayloadSHA256=input_binding.get('actualPayloadSHA256'),
        sourceNativeStep=context.source_step,sourceTimeS=context.source_time,entryLiftElapsedS=0.,
        original33AnchorAvailable=False,previousRawReceiptsRecovered=False,oldCertificatesReused=False,
        originalHardLimitsUnchanged=True,originalFootPathAndClocksUnchanged=True,goalOrWeightScan=False,producerOptimizerCalls=1)
    if any(k not in result or type(result[k])is not type(v)or result[k]!=v for k,v in expected.items()):
        raise ValueError('fresh recovered-coordinate problem identity/semantics differ')
    if digest(initial.reshape(3,9).tolist())!=key:raise ValueError('fresh anchor and sole initial candidate must be identical')
    for name in('sourceCoordinateReceiptFilename','regeneratedActivationInputFilename'):
        if not isinstance(result.get(name),str)or not result[name]or '/'in result[name]:raise ValueError('fresh recovery input basename required')
    for name in('sourceCoordinateReceiptSHA256','regeneratedActivationInputSHA256'):
        h=result.get(name)
        if not isinstance(h,str)or len(h)!=64 or any(c not in'0123456789abcdef'for c in h):raise ValueError('fresh recovery exact input SHA256 required')
    brackets=result.get('declaredAdditionalEnvelopeBrackets',[])
    expected_brackets=[dict(phase='lift',bracketFractions=[.775,.8,.8250000000000001]),
        dict(phase='land',bracketFractions=[.7000000000000001,.7250000000000001,.75])]
    if brackets!=expected_brackets:raise ValueError('exact fresh lift/land envelope declarations required')
    for bracket in brackets:
        b=bracket.get('bracketFractions',[])
        if len(b)!=3 or not all(isinstance(x,(int,float))and math.isfinite(x)for x in b)or not 0.<=b[0]<b[1]<b[2]<=1.:
            raise ValueError('fresh finite ordered additional envelope brackets required')
    return result


def normalized_constraint_nodes(phase_fractions=None):
    base=tuple((k,float(f))for k in range(3)for f in FRACTIONS if not(k and f==0.))
    if phase_fractions is None:return base
    if not isinstance(phase_fractions,dict)or set(phase_fractions)!=set(PHASES):
        raise ValueError('explicit fractions for exactly all three phases required')
    nodes=[]
    for k,phase in enumerate(PHASES):
        values=tuple(float(f)for f in phase_fractions[phase])
        if not values or not all(math.isfinite(f)and 0.<=f<=1. for f in values)or len(set(values))!=len(values):
            raise ValueError('unique finite phase constraint fractions in[0,1] required')
        nodes.extend((k,f)for f in sorted(values))
    if not set(base).issubset(set(nodes)):raise ValueError('all original13 constraint nodes must remain')
    return tuple(nodes)


def source_bracket_declarations(parent):
    """Only the retained worst FAILED original123 sample and its neighbors."""
    if parent.get('version')!=VERSION_POLISH or parent.get('optimizerSuccess')is not True or parent.get('accepted')is not False:
        raise ValueError('retained converged but failed sparse polish required')
    rows=parent.get('analyticSamples',[]);expected=[(p,f)for p in PHASES for f in CERTIFICATE_FRACTIONS]
    if len(rows)!=123 or any(r.get('phase')!=p or abs(r.get('fraction',-9.)-f)>1e-14 for r,(p,f)in zip(rows,expected)):
        raise ValueError('source original123 rows absent or reordered')
    output=[]
    for phase in PHASES:
        failed=[r for r in parent['analyticFailures']if r['phase']==phase]
        worst=max(failed,key=lambda r:r['commandAccelerationMps2'])if phase=='lift'else min(failed,key=lambda r:r['supportCaptureMarginM'])
        index=min(range(41),key=lambda i:abs(CERTIFICATE_FRACTIONS[i]-worst['fraction']))
        if not 0<index<40:raise ValueError('interior source bracket required')
        output.append(dict(phase=phase,worstSampleFraction=worst['fraction'],
            bracketFractions=list(CERTIFICATE_FRACTIONS[index-1:index+2])))
    return output


def select_source_bracket_extrema(realizer,context,limits,heading,support,parent,*,source_failed_polish_sha256,
        design_only=False,entry_elapsed=0.,max_wall_seconds=60.):
    """Exactly3 bounded TIME searches on the failed parent; no trajectory solve."""
    if not math.isfinite(max_wall_seconds)or not 0.<max_wall_seconds<=60.:raise ValueError('source selection cap in(0,60] required')
    brackets=source_bracket_declarations(parent);knots=np.asarray(parent['unadmittedFinalIndependentKnotsRad'])
    candidate=digest(knots.tolist())
    if candidate!=parent['finalCandidateSHA256']:raise ValueError('failed selection path candidate SHA differs')
    path=WholeBodyCycle(realizer,context,limits,heading,knots,design_only=design_only,entry_elapsed=entry_elapsed)
    start=time.monotonic();calls=0;results=[];evaluations=[];failure=None;eq=support.equations()
    try:
        for bracket in brackets:
            k=PHASES.index(bracket['phase']);left,center,right=bracket['bracketFractions']
            def scalar(f):
                if time.monotonic()-start>=max_wall_seconds:raise _ControlStop('source-selection-compute-cap')
                sample=path.sample(PHASES[k],float(f)*path.durations[k]);p,v,a=sample.com.arrays()
                cp=p[:2]+math.sqrt(p[2]/9.81)*v[:2]
                slack=(MAX_TOTAL_COM_ACCELERATION-float(np.linalg.norm(a))if k==0 else
                    float(min(np.min(-(eq[:,:2]@p[:2]+eq[:,2])),np.min(-(eq[:,:2]@cp+eq[:,2]))))-.003)
                evaluations.append(dict(phase=PHASES[k],fraction=float(f),minimumSlack=slack,
                    maximumPoseResidual=sample.diagnostics['maximumPoseResidual'],originalStrictMapUsed=True))
                return slack
            calls+=1
            answer=minimize_scalar(scalar,bounds=(left,right),method='bounded',options=dict(maxiter=64,xatol=1e-10))
            results.append(dict(phase=PHASES[k],success=bool(answer.success),fraction=float(answer.x),
                minimumSlack=float(answer.fun),timeS=float(answer.x*path.durations[k]),evaluations=int(answer.nfev),iterations=int(answer.nit)))
    except Exception as exc:failure=dict(type=type(exc).__name__,message=str(exc))
    passed=bool(calls==3 and len(results)==3 and all(r['success']and math.isfinite(r['minimumSlack'])and r['minimumSlack']<0. for r in results))
    receipt=dict(schema=1,sourceFailedPolishSHA256=source_failed_polish_sha256,selectionInputCandidateSHA256=candidate,
        realTrajectoryOptimizerCalls=0,scalarSearchCalls=calls,sourceBrackets=brackets,bracketExtremaResults=results,
        phaseDurationS=list(path.durations),strictSourceEvaluationRows=evaluations,originalStrictMapUsed=True,
        maxWallSeconds=max_wall_seconds,maximumIterationsPerBracket=64,fractionTolerance=1e-10,elapsedS=time.monotonic()-start,
        passed=passed,error=failure,nativeAdmission=False,wholePathProven=False)
    return([dict(phase=r['phase'],fraction=r['fraction'])for r in results]if passed else None),receipt


def refinement_binding(value,nodes,anchor):
    if value is None:
        if nodes!=normalized_constraint_nodes():raise ValueError('additional nodes require explicit causal refinement binding')
        return None
    r=copy.deepcopy(value)
    required=dict(schema=1,selectionPolicy='worst-failing-bracket-extremum-per-phase-v1',anchorUnchanged=True,
        onlyOneRefinement=True,intermediateFailedPolishOptimizerCalls=1,refinementOptimizerCalls=1)
    if any(type(r.get(k))is not type(v)or r.get(k)!=v for k,v in required.items()):raise ValueError('explicit one-refinement semantics required')
    for key in('sourceFailedPolishSHA256','selectionInputCandidateSHA256','selectionReceiptSHA256'):
        sha=r.get(key)
        if not isinstance(sha,str)or len(sha)!=64 or any(x not in'0123456789abcdef'for x in sha):raise ValueError('retained selection SHA256 required: '+key)
    added=r.get('addedConstraintNodes',[]);brackets=r.get('sourceBrackets',[]);results=r.get('bracketExtremaResults',[])
    if not(len(added)==len(brackets)==len(results)==3):raise ValueError('exactly three retained bracket extrema required')
    for phase,node,bracket,result in zip(PHASES,added,brackets,results):
        b=bracket.get('bracketFractions',[]);f=result.get('fraction')
        if(node.get('phase')!=phase or bracket.get('phase')!=phase or result.get('phase')!=phase or len(b)!=3
                or not all(isinstance(x,(int,float))and math.isfinite(x)for x in b)or not 0.<=b[0]<b[1]<b[2]<=1.
                or bracket.get('worstSampleFraction')!=b[1]or result.get('success')is not True
                or not isinstance(f,(int,float))or not math.isfinite(f)or not b[0]<=f<=b[2]or node.get('fraction')!=f
                or not math.isfinite(result.get('minimumSlack',math.inf))or result['minimumSlack']>=0.
                or result.get('evaluations',0)<=0 or result.get('iterations',0)<=0):
            raise ValueError('source bracket-bound successful finite negative extremum required')
    expected=tuple(sorted((*normalized_constraint_nodes(),*((PHASES.index(x['phase']),x['fraction'])for x in added))))
    if nodes!=expected:raise ValueError('refinement must retain original13 plus exactly three source extrema')
    r.update(sourceCandidateSHA256=digest(anchor.reshape(3,9).tolist()),sourceCandidateMajorIteration=33,
        originalConstraintNodeCount=13,refinedConstraintNodeCount=16,noFurtherAdaptiveLoop=True,
        initialAndObjectiveAnchor='unchanged exact recovery major iteration33',originalHardLimitsUnchanged=True)
    return r


class AnalyticPolishEvaluator:
    """Pure exact-key cached13-node analytic constraint function, no optimizer.

    Immutable entry data and independent C2 segments are the only path family.
    Every implicit foot solve retains the original fixed q0 branch seed/Newton
    cap/tolerance. Node keys include all9 position/velocity/acceleration values.
    """
    def __init__(self,realizer,context,limits,heading,support,anchor,*,design_only=False,entry_elapsed=0.,check_control=lambda:None,constraint_nodes=None,envelope_brackets=None,global_translate_support=False,node_geometry=None,global_support_phases=None):
        support.validate_identity(context)
        self.r=realizer;self.context=context;self.limits=np.array(limits,float,copy=True)
        if self.limits.shape!=(21,2)or not np.isfinite(self.limits).all():raise ValueError('finite original21x2 source limits required')
        self.anchor=np.array(anchor,dtype=np.float64,copy=True).reshape(27)
        if not np.isfinite(self.anchor).all():raise ValueError('finite bound27-coordinate candidate required')
        self.anchor.setflags(write=False);self.check_control=check_control;self.support=support
        self.q0,self.lo,self.hi=_bounds(realizer,context,self.limits);self.idx=realizer.independent_indices
        self.bounds=list(zip(np.tile(self.lo[self.idx],3),np.tile(self.hi[self.idx],3)))
        # Original entry constructor uses the original strict map. Only the
        # optimizer copy's future prediction uses the declared raw-domain proxy.
        self.entry=WholeBodyCycle(realizer,context,self.limits,heading,self.anchor.reshape(3,9),design_only=design_only,entry_elapsed=entry_elapsed)
        self.optimizer_realizer=copy.copy(realizer)
        self.optimizer_realizer.arm_map=OptimizerRawDomainArmMap(realizer.arm_map)
        self.declared_nodes=normalized_constraint_nodes(constraint_nodes)
        self.envelope_brackets=copy.deepcopy(envelope_brackets or[])
        self.nodes=normalized_constraint_nodes()if self.envelope_brackets else self.declared_nodes
        self.envelope_state=OrderedDict();self.jacobian_cache=OrderedDict();self.node_geometry=node_geometry
        phases=tuple(global_support_phases or(('translate',)if global_translate_support else()))
        if len(set(phases))!=len(phases)or any(p not in PHASES for p in phases):raise ValueError('unique declared global support phases required')
        self.global_support_phases=phases;self.global_translate_support=bool(phases)
        self.eq=support.equations();self.ranges=np.asarray(realizer.arm_map.ranges)
        n=len(self.eq)
        self.envelope_specs=[]
        if self.global_translate_support:
            for bracket in self.envelope_brackets:
                if bracket['phase']=='translate':continue
                self.envelope_specs.append(dict(phase=bracket['phase'],pointKind='acceleration'if bracket['phase']=='lift'else'combinedCOMCapture',
                    faceIndex=None,intervalFractions=[bracket['bracketFractions'][0],bracket['bracketFractions'][2]],
                    historicalSourceBracketFractions=bracket['bracketFractions']))
            for phase in self.global_support_phases:
                for point_kind in('COM','capture'):
                    for face in range(n):
                        for left,right in zip(FRACTIONS,FRACTIONS[1:]):
                            self.envelope_specs.append(dict(phase=phase,pointKind=point_kind,faceIndex=face,intervalFractions=[left,right]))
        else:
            self.envelope_specs=[dict(phase=b['phase'],pointKind='acceleration'if b['phase']=='lift'else'combinedCOMCapture',faceIndex=None,
                intervalFractions=[b['bracketFractions'][0],b['bracketFractions'][2]],historicalSourceBracketFractions=b['bracketFractions'])for b in self.envelope_brackets]
        self.labels=tuple([f'lower-joint-{i}-lower-bound'for i in range(15)]+[f'lower-joint-{i}-upper-bound'for i in range(15)]
            +[f'arm-{i}-lower-reserve'for i in range(6)]+[f'arm-{i}-upper-reserve'for i in range(6)]
            +[f'map-{i}-lower-reserve'for i in range(2)]+[f'map-{i}-upper-reserve'for i in range(2)]
            +[f'COM-support-face-{i}'for i in range(n)]+[f'capture-support-face-{i}'for i in range(n)]
            +['COM-speed-cap','COM-acceleration-cap','positive-COM-height','Newton-pose-residual'])
        self.node_cache=OrderedDict();self.candidate_cache=OrderedDict();self.node_capacity=4096;self.candidate_capacity=128
        self.stats=dict(entryTemplates=1,segmentOnlyRebuilds=0,nodeHits=0,nodeMisses=0,nodeEvictions=0,
            fullCandidateHits=0,fullCandidateMisses=0,fullCandidateEvictions=0,analyticCandidateEvaluations=0,analyticNodeEvaluations=0)
        self.stats.update(envelopeScalarSearchCalls=0,fixedPeakJacobianColumns=0,jacobianHits=0,jacobianMisses=0)
        self.exceptions=[];self.last_requested=None;self.last_fully_evaluated=None;self.last_result=None
        self.key_base=dict(version=VERSION,context=context.receipt()['contextSHA256'],stance=realizer.stance_pose.position,
            limits=self.entry.limits.tolist(),entryElapsed=entry_elapsed)

    def path(self,x,*,strict_map=False):
        p=copy.copy(self.entry);knots=np.array(x,dtype=np.float64,copy=True).reshape(3,9);knots.setflags(write=False)
        p.independent_knots=knots;p.realizer=self.r if strict_map else self.optimizer_realizer
        points=[np.array(p.lower0.position)[self.idx],*knots]
        p.independent_segments=tuple(C2Segment.between(0.,T,Jet.rest(a),Jet.rest(b))for a,b,T in zip(points,points[1:],p.durations))
        p.key=digest(dict(self.key_base,knots=knots.tolist()));self.stats['segmentOnlyRebuilds']+=1
        return p

    def constraint_block(self,sample):
        q=np.asarray(sample.lower_joints.position);full=np.asarray(sample.geometric.full_joints.position)
        delta=np.asarray(sample.geometric.future_arm_delta_right_left.position);p,v,a=sample.com.arrays()
        if p[2]<=0.:raise ValueError('nonpositive command COM height')
        cp=p[:2]+math.sqrt(p[2]/9.81)*v[:2];eq=self.eq
        return np.r_[q-self.lo,self.hi-q,full[15:]-self.limits[15:,0]-.02,self.limits[15:,1]-full[15:]-.02,
            delta-self.ranges[:,0]-.02,self.ranges[:,1]-delta-.02,-(eq[:,:2]@p[:2]+eq[:,2])-.003,
            -(eq[:,:2]@cp+eq[:,2])-.003,MAX_TOTAL_COM_SPEED-np.linalg.norm(v),
            MAX_TOTAL_COM_ACCELERATION-np.linalg.norm(a),p[2],2e-8-sample.diagnostics['maximumPoseResidual']]

    def node(self,path,k,f):
        self.check_control();t=float(f)*path.durations[k];jets=path.independent_segments[k].at(t)
        node_key=(k,float(t),*(np.asarray(a,dtype=np.float64).tobytes()for a in jets.arrays()))
        if node_key in self.node_cache:
            self.stats['nodeHits']+=1;self.node_cache.move_to_end(node_key);return self.node_cache[node_key]
        self.stats['nodeMisses']+=1;self.stats['analyticNodeEvaluations']+=1
        sample=(path.sample(PHASES[k],t)if self.node_geometry is None else self.node_geometry.sample(path,PHASES[k],t))
        self.check_control();block=self.constraint_block(sample);block.setflags(write=False)
        row=dict(phase=PHASES[k],fraction=float(f),maximumPoseResidual=float(sample.diagnostics['maximumPoseResidual']),
            commandCOM=sample.com.position,commandCOMVelocity=sample.com.velocity,commandCOMAcceleration=sample.com.acceleration,
            minimumSlack=float(min(block)),minimumMapSlackRad=float(min(block[42:46])))
        self.node_cache[node_key]=(block,row)
        if len(self.node_cache)>self.node_capacity:self.node_cache.popitem(last=False);self.stats['nodeEvictions']+=1
        return block,row

    def envelope_metric(self,block,k,spec=None):
        if spec is None:return float(block[-3])if k==0 else float(min(block[46:46+2*len(self.eq)]))
        kind=spec['pointKind']
        if kind=='acceleration':return float(block[-3])
        if kind=='combinedCOMCapture':return float(min(block[46:46+2*len(self.eq)]))
        return float(block[46+(len(self.eq)if kind=='capture'else 0)+spec['faceIndex']])

    def coverage_receipt(self):
        records=[]
        for j,spec in enumerate(self.envelope_specs):
            if spec['phase']=='translate'and spec['pointKind']in('COM','capture'):
                records.append(dict(pointKind=spec['pointKind'],faceIndex=spec['faceIndex'],intervalFractions=spec['intervalFractions'],
                    rowIndex=len(self.nodes)*len(self.labels)+j))
        return dict(schema=1,phase='translate',sourceSupportEquationsSHA256=digest(self.eq.tolist()),sourceSupportEquations=self.eq.tolist(),
            faceCount=len(self.eq),pointKinds=['COM','capture'],originalSupportReserveM=.003,
            intervalsFractions=[[a,b]for a,b in zip(FRACTIONS,FRACTIONS[1:])],constraints=records,
            closedIntervalEndpointsCompared=True,wholePathProven=False,
            boundedScalarMethod='bounded',maximumIterationsPerEnvelope=64,fractionTolerance=1e-10)


    def phase_coverage_receipt(self):
        records=[]
        for j,spec in enumerate(self.envelope_specs):
            if spec['pointKind']in('COM','capture'):
                records.append(dict(phase=spec['phase'],pointKind=spec['pointKind'],faceIndex=spec['faceIndex'],
                    intervalFractions=spec['intervalFractions'],rowIndex=len(self.nodes)*len(self.labels)+j))
        return dict(schema=1,phases=list(self.global_support_phases),sourceSupportEquationsSHA256=digest(self.eq.tolist()),sourceSupportEquations=self.eq.tolist(),
            faceCount=len(self.eq),pointKinds=['COM','capture'],originalSupportReserveM=.003,
            intervalsFractions=[[a,b]for a,b in zip(FRACTIONS,FRACTIONS[1:])],constraints=records,
            closedIntervalEndpointsCompared=True,wholePathProven=False,
            boundedScalarMethod='bounded',maximumIterationsPerEnvelope=64,fractionTolerance=1e-10)

    def fixed_constraints(self,x,peaks):
        path=self.path(x);blocks=[self.node(path,k,f)[0]for k,f in self.nodes]
        if self.envelope_specs:
            blocks.extend(np.array([self.envelope_metric(self.node(path,PHASES.index(spec['phase']),f)[0],PHASES.index(spec['phase']),spec)])for spec,f in zip(self.envelope_specs,peaks))
        return np.concatenate(blocks)

    def constraint_jacobian(self,x):
        # Envelope theorem: differentiate the selected scalar at its current
        # minimizer. The27 columns do NOT repeat any scalar time searches.
        self.check_control();x=np.asarray(x,dtype=np.float64);key=x.tobytes()
        if key in self.jacobian_cache:
            self.stats['jacobianHits']+=1;self.jacobian_cache.move_to_end(key);return self.jacobian_cache[key]
        baseline,rows=self.evaluate(x)
        expected=len(self.nodes)*len(self.labels)+len(self.envelope_specs)
        genuine=(baseline.shape==(expected,)and np.isfinite(baseline).all()
            and len(rows)==len(self.nodes)+len(self.envelope_specs)
            and all('maximumPoseResidual'in row and 'error'not in row and 'errorType'not in row for row in rows))
        # Negative REAL hard slacks remain differentiable. The synthetic all-1
        # rejection vector has no geometry and must never become a FD baseline.
        if not genuine:raise ValueError('cannot differentiate rejected synthetic analytic baseline')
        peaks=self.envelope_state[key]if self.envelope_specs else();columns=[]
        for j,(lo,hi)in enumerate(self.bounds):
            self.check_control();h=1e-6 if x[j]+1e-6<=hi else-1e-6
            if not lo<=x[j]+h<=hi:raise ValueError('original bound has no finite difference interior')
            shifted=x.copy();shifted[j]+=h;actual=shifted[j]-x[j]
            columns.append((self.fixed_constraints(shifted,peaks)-baseline)/actual);self.stats['fixedPeakJacobianColumns']+=1
        jac=np.column_stack(columns);jac.setflags(write=False);self.jacobian_cache[key]=jac;self.stats['jacobianMisses']+=1
        if len(self.jacobian_cache)>128:self.jacobian_cache.popitem(last=False)
        return jac

    def evaluate(self,x):
        self.check_control();x=np.asarray(x,dtype=np.float64)
        if x.shape!=(27,)or not np.isfinite(x).all():raise ValueError('finite27-coordinate optimizer input required')
        self.last_requested=np.array(x,copy=True);key=x.tobytes()
        if key in self.candidate_cache:
            self.stats['fullCandidateHits']+=1;self.candidate_cache.move_to_end(key)
            self.last_fully_evaluated=np.array(x,copy=True);self.last_result=self.candidate_cache[key]
            return self.last_result
        self.stats['fullCandidateMisses']+=1;self.stats['analyticCandidateEvaluations']+=1
        path=self.path(x);blocks=[];rows=[]
        # A rejected Newton/raw-map domain remains a full fixed-size rejection;
        # no alternative seed, clipped map or fabricated derivative is used.
        try:
            for k,f in self.nodes:
                self.check_control();t=f*path.durations[k]
                block,row=self.node(path,k,f)
                blocks.append(block);rows.append(row)
            peaks=[]
            for spec in self.envelope_specs:
                k=PHASES.index(spec['phase']);left,right=spec['intervalFractions']
                def scalar(f):return self.envelope_metric(self.node(path,k,float(f))[0],k,spec)
                self.stats['envelopeScalarSearchCalls']+=1
                answer=minimize_scalar(scalar,bounds=(left,right),method='bounded',options=dict(maxiter=64,xatol=1e-10))
                self.check_control()
                if not answer.success:raise ValueError('current-candidate bracket envelope search did not converge')
                choices=[(float(answer.fun),float(answer.x))]
                if self.global_translate_support:choices.extend((scalar(endpoint),endpoint)for endpoint in(left,right))
                minimum,f=min(choices,key=lambda value:value[0]);block,row=self.node(path,k,f);peaks.append(f)
                blocks.append(np.array([self.envelope_metric(block,k,spec)]));rows.append(dict(row,envelopeScalar=True,
                    minimumEnvelopeSlack=self.envelope_metric(block,k,spec),envelopeMetric=spec['pointKind'],faceIndex=spec['faceIndex'],
                    envelopeEvaluations=int(answer.nfev)+(2 if self.global_translate_support else 0),envelopeIterations=int(answer.nit),
                    sourceIntervalFractions=list(spec['intervalFractions']),closedIntervalEndpointsCompared=self.global_translate_support))
            values=np.concatenate(blocks)
            if self.envelope_specs:self.envelope_state[key]=tuple(peaks)

        except _ControlStop:raise
        except Exception as exc:
            if len(self.exceptions)<16:self.exceptions.append(dict(type=type(exc).__name__,message=str(exc)))
            rows.append(dict(phase=PHASES[k],fraction=f,errorType=type(exc).__name__,error=str(exc)))
            values=np.full(len(self.nodes)*len(self.labels)+len(self.envelope_specs),-1.)
            if self.envelope_specs:self.envelope_state[key]=tuple(sum(spec['intervalFractions'])/2. for spec in self.envelope_specs)
        if not np.isfinite(values).all():raise ValueError('nonfinite analytic constraint; never an accepted branch')
        values.setflags(write=False);result=(values,tuple(rows));self.candidate_cache[key]=result
        if len(self.candidate_cache)>self.candidate_capacity:
            old_key,_=self.candidate_cache.popitem(last=False);self.envelope_state.pop(old_key,None);self.stats['fullCandidateEvictions']+=1
        self.last_fully_evaluated=np.array(x,copy=True);self.last_result=result
        return result

    def cache_receipt(self):
        return dict(self.stats,nodeEntries=len(self.node_cache),nodeCapacity=self.node_capacity,
            fullCandidateEntries=len(self.candidate_cache),fullCandidateCapacity=self.candidate_capacity,
            exactFloat64ByteKeys=True,noMutableIKSeed=True)


def _sample_row(path,sample,k,f,support,heading,origin,q0):
    check=path.validate_sample(sample,support,require_clearance=k==1)
    p,v,a=sample.com.arrays();eq=support.equations();cp=p[:2]+math.sqrt(p[2]/9.81)*v[:2]
    row=dict(phase=PHASES[k],fraction=f,**check,originalStrictMapEvaluationPassed=True,
        lowerJoints=sample.lower_joints.position,commandCOM=sample.com.position,commandCOMVelocity=sample.com.velocity,
        commandCOMAcceleration=sample.com.acceleration,body1Rotation=sample.thorax.rotation,pelvisRotation=sample.pelvis.rotation,
        commandCOMForwardProgressM=float((p-origin)@heading),kneeReferenceGainDeg=math.degrees(q0[(12,6)[path.swing_side]]-sample.lower_joints.position[(12,6)[path.swing_side]]),
        kneeReferenceFlexionDeg=math.degrees(-sample.lower_joints.position[(12,6)[path.swing_side]]),
        supportCaptureMarginM=float(min(np.min(-(eq[:,:2]@p[:2]+eq[:,2])),np.min(-(eq[:,:2]@cp+eq[:,2])))),newton=sample.diagnostics)
    remove=Rotation.from_euler('Z',-math.atan2(heading[1],heading[0])).as_matrix()
    for label,orientation in(('body1',sample.thorax),('pelvis',sample.pelvis)):
        R=orientation.arrays()[0];yaw,pitch,roll=Rotation.from_matrix(remove@R).as_euler('ZYX',degrees=True)
        row[label+'Angles']=dict(headingResidualDeg=float(yaw),forwardPitchDeg=float(pitch),rollDeg=float(roll),
            totalTiltDeg=math.degrees(math.acos(float(np.clip(R[2,2],-1.,1.)))))
    return row


def summarize_samples(samples):
    metrics={}
    for phase in PHASES:
        allrows=[x for x in samples if x['phase']==phase];valid=[x for x in allrows if'commandCOM'in x]
        def bound(key):
            values=[x[key]for x in valid];return None if not values else dict(minimum=float(min(values)),maximum=float(max(values)))
        row=dict(samples=len(allrows),failedSamples=sum(not x['accepted']for x in allrows),metricsUnavailableSamples=len(allrows)-len(valid))
        for key in('commandSpeedMps','commandAccelerationMps2','supportCaptureMarginM','plannedWholeFootClearanceM',
                'minimumLowerReserveRad','minimumArmReserveRad','minimumMapSlackRad','minimumLumbarSlackRad','kneeReferenceGainDeg'):
            row[key]=bound(key)
        for body in('body1','pelvis'):
            row[body]={key:None if not valid else dict(minimum=min(x[body+'Angles'][key]for x in valid),maximum=max(x[body+'Angles'][key]for x in valid))
                for key in('forwardPitchDeg','rollDeg','totalTiltDeg')}
            row[body]['final']=None if not valid else valid[-1][body+'Angles']
        metrics[phase]=row
    valid=[x for x in samples if'commandCOM'in x];lift=[x for x in valid if x['phase']=='lift']
    peak=max(lift,key=lambda x:x['kneeReferenceGainDeg'])if lift else None
    return dict(phaseMetrics=metrics,earlyLiftKneePeak=peak,
        postLiftPeakExtensionDeg=None if not peak or not valid else peak['kneeReferenceGainDeg']-valid[-1]['kneeReferenceGainDeg'],
        sampledCOMForwardProgressM=None if not valid else valid[-1]['commandCOMForwardProgressM']-valid[0]['commandCOMForwardProgressM'],
        # Never bridge over a failed/unavailable sample and invent path length.
        sampledCOMPolylineLengthM=sum(float(np.linalg.norm(np.array(b['commandCOM'])-a['commandCOM']))for a,b in zip(samples,samples[1:])if'commandCOM'in a and'commandCOM'in b),
        COMPolylineOmitsUnavailableIntervals=any('commandCOM'not in x for x in samples))


def polish_horizon(realizer,current_context,source_limits,heading,current_support,source_candidate_knots,*,stage_binding=None,
        design_only=False,entry_elapsed=0.,input_binding=None,max_iterations=80,max_wall_seconds=180.,
        certificate_max_wall_seconds=120.,cancellation_check=None,progress_callback=None,
        constraint_nodes=None,collocation_refinement=None,exchange_history=None,initial_knots=None,
        global_translate_support_envelopes=False,node_geometry=None,node_geometry_binding=None,
        global_support_phases=None,activation_retargeting=None,certificate_sampler=None,certificate_sampler_binding=None,
        fresh_recovery_problem=None):
    """One bounded SLSQP; independently check EXACT final candidate at123 nodes.

    The return path exists only for successful polish plus full original strict
    finite admission and the original desired-early-correction check. Native,
    motor and continuous-path admission remain false. A cap retains the last
    completed major iteration (never a best iterate or FD-perturbed candidate).
    """
    nodes=normalized_constraint_nodes(constraint_nodes)
    fresh=fresh_recovery_problem is not None;refined=collocation_refinement is not None or fresh
    for value,cap,label in((max_wall_seconds,300. if refined else 180.,'optimizer'),(certificate_max_wall_seconds,180. if refined else 120.,'certificate')):
        if not math.isfinite(value)or not 0.<value<=cap:raise ValueError(label+' wall cap must be positive and at most'+str(cap))
    if type(max_iterations)is not int or not 1<=max_iterations<=80:raise ValueError('maximum iterations in1..80 required')
    for value,label in((cancellation_check,'cancellation check'),(progress_callback,'progress callback')):
        if value is not None and not callable(value):raise ValueError(label+' must be callable')
    anchor=np.asarray(source_candidate_knots,dtype=np.float64)
    if anchor.shape!=(3,9)or not np.isfinite(anchor).all():raise ValueError('finite bound3x9 source candidate required')
    anchor=anchor.reshape(27).copy();stage=None if fresh else _stage_binding(stage_binding,anchor)
    refinement=refinement_binding(collocation_refinement,nodes,anchor)
    geometry_binding=copy.deepcopy(node_geometry_binding or{})
    if node_geometry is not None:
        name=geometry_binding.get('filename');h=geometry_binding.get('SHA256')
        if not isinstance(name,str)or '/'in name or not isinstance(h,str)or len(h)!=64:
            raise ValueError('explicit fast analytic geometry source binding required')
    strict_sampler_binding=copy.deepcopy(certificate_sampler_binding or{})
    if certificate_sampler is not None:
        name=strict_sampler_binding.get('filename');h=strict_sampler_binding.get('SHA256')
        if(not callable(getattr(certificate_sampler,'sample',None))or not isinstance(name,str)or '/'in name or not name
                or not isinstance(h,str)or len(h)!=64):
            raise ValueError('explicit strict complete-reference sampler/source binding required')
    history=copy.deepcopy(exchange_history or[]);retarget=copy.deepcopy(activation_retargeting)
    initial=anchor.copy()if initial_knots is None else np.asarray(initial_knots,dtype=np.float64).reshape(27).copy()
    if not np.isfinite(initial).all():raise ValueError('finite deterministic continuation required')
    global_phases=tuple(global_support_phases or(('translate',)if global_translate_support_envelopes else()))
    fresh_problem=None
    if fresh:
        if(stage_binding is not None or collocation_refinement is not None or history or retarget is not None
                or nodes!=normalized_constraint_nodes()or global_phases!=PHASES or design_only or entry_elapsed!=0.
                or current_context.phase!='transfer'or(input_binding or{}).get('entryLiftElapsedS')!=0.
                or(input_binding or{}).get('contextSHA256')!=current_context.receipt()['contextSHA256']
                or len(current_support.equations())!=9
                or(input_binding or{}).get('currentSupportSHA256')!=current_support.key):
            raise ValueError('fresh recovery requires new stationary actual input/all-phase coverage and no historical stages')
        fresh_problem=fresh_recovery_binding(fresh_recovery_problem,anchor,initial,input_binding or{},current_context)
    elif retarget is not None:

        if refinement is None or global_phases!=PHASES:raise ValueError('actual activation requires all-phase support coverage')
        expected=dict(schema=1,activationSourceStep=current_context.source_step,activationSourceTimeS=current_context.source_time,
            activationContextSHA256=current_context.receipt()['contextSHA256'],retainedActivationInputSHA256=(input_binding or{}).get('inputSHA256'),
            actualPayloadSHA256=(input_binding or{}).get('actualPayloadSHA256'),compressedPayloadSHA256=(input_binding or{}).get('compressedPayloadSHA256'),
            objectiveAnchorSHA256=digest(anchor.reshape(3,9).tolist()),
            originalHardLimitsUnchanged=True,referenceFamilyUnchanged=True,physicalStateInstalled=False,retargetOptimizerCalls=1)
        if any(k not in retarget or type(retarget[k])is not type(v)or retarget[k]!=v for k,v in expected.items()):raise ValueError('explicit actual activation transition differs')
        if(current_context.phase!='transfer'or entry_elapsed!=0. or(input_binding or{}).get('entryLiftElapsedS')!=0.
                or(input_binding or{}).get('contextSHA256')!=current_context.receipt()['contextSHA256']
                or(input_binding or{}).get('currentSupportSHA256')!=current_support.key):raise ValueError('actual stationary transfer input/clock identity differs')
        if retarget.get('sourceProposalProducerOptimizerCalls')!=4:raise ValueError('explicit source proposal producer count4 required')
        if retarget.get('sourceProposalCandidateSHA256')!=retarget.get('initialContinuationCandidateSHA256'):
            raise ValueError('historical activation transition seed identity differs')
        if history:
            for entry in history:
                if(entry.get('optimizerCalls')!=1 or entry.get('hardLimitsUnchanged')is not True
                        or type(entry.get('optimizerSuccess'))is not bool
                        or entry.get('objectiveAnchorSHA256')!=digest(anchor.reshape(3,9).tolist())
                        or entry.get('inputBindingSHA256')!=digest(input_binding)
                        or entry.get('activationRetargetingSHA256')!=digest(retarget)
                        or entry.get('finalCandidateSHA256')!=entry.get('sourceCandidateSHA256')
                        or entry.get('sourceCandidateSHA256')!=entry.get('initialContinuationCandidateSHA256')):
                    raise ValueError('same-actual-input numerical exchange identity differs')
                name=entry.get('filename');h=entry.get('SHA256')
                if not isinstance(name,str)or '/'in name or not name or not isinstance(h,str)or len(h)!=64:
                    raise ValueError('retained same-input parent basename/SHA required')
                if(entry.get('optimizerSuccess')is False and(entry.get('terminalState')!='compute-cap'
                        or entry.get('finalCandidateOrigin')!='last-completed-major-iteration'
                        or type(entry.get('completedMajorIteration'))is not int or entry['completedMajorIteration']<1)):
                    raise ValueError('compute-cap continuation requires an exact completed major iteration')
            if digest(initial.reshape(3,9).tolist())!=history[-1]['sourceCandidateSHA256']:
                raise ValueError('continuation differs from exact last same-input candidate')
        elif retarget.get('initialContinuationCandidateSHA256')!=digest(initial.reshape(3,9).tolist()):
            raise ValueError('first retarget continuation differs from historical source seed')
        for name in('sourceProposalFilename','sourceProposalBindingFilename','retainedActivationInputFilename'):
            value=retarget.get(name)
            if not isinstance(value,str)or '/'in value or not value:raise ValueError('retained activation transition basename required')
        for name in('sourceProposalSHA256','sourceProposalBindingSHA256'):
            value=retarget.get(name)
            if not isinstance(value,str)or len(value)!=64:raise ValueError('retained activation transition SHA256 required')
    elif history:
        if refinement is None or not global_phases:raise ValueError('linked exchange requires complete global support coverage')
        for entry in history:
            if(entry.get('optimizerCalls')!=1 or entry.get('objectiveAnchorSHA256')!=digest(anchor.reshape(3,9).tolist())
                    or entry.get('hardLimitsUnchanged')is not True or entry.get('sourceCandidateSHA256')!=entry.get('initialContinuationCandidateSHA256')):
                raise ValueError('linked deterministic exchange semantics differ')
            name=entry.get('parentResultFilename');h=entry.get('parentResultSHA256')
            if not isinstance(name,str)or '/'in name or not name or not isinstance(h,str)or len(h)!=64:raise ValueError('retained exchange basename/SHA required')
        if digest(initial.reshape(3,9).tolist())!=history[-1]['sourceCandidateSHA256']:raise ValueError('continuation differs from last linked final candidate')
    elif initial_knots is not None or global_phases:
        raise ValueError('continuation/global coverage requires linked deterministic exchange or activation transition')
    control=None;start=None
    def check_optimizer():
        nonlocal control
        if cancellation_check is not None and cancellation_check():control='caller-cancelled';raise _ControlStop(control)
        if start is not None and time.monotonic()-start>=max_wall_seconds:control='compute-cap';raise _ControlStop(control)
    evaluator=AnalyticPolishEvaluator(realizer,current_context,source_limits,heading,current_support,anchor,
        design_only=design_only,entry_elapsed=entry_elapsed,check_control=check_optimizer,constraint_nodes=constraint_nodes,
        envelope_brackets=(fresh_problem['declaredAdditionalEnvelopeBrackets']if fresh else(None if refinement is None else refinement.get('sourceBrackets'))),
        global_translate_support=global_translate_support_envelopes,node_geometry=node_geometry,global_support_phases=global_phases)
    if fresh:
        for spec in evaluator.envelope_specs:
            if 'historicalSourceBracketFractions'in spec:spec['freshDeclaredBracketFractions']=spec.pop('historicalSourceBracketFractions')
    # Reject input outside original independent bounds; do not project a seed.
    if any(x<lo or x>hi for x,(lo,hi)in zip(initial,evaluator.bounds)):raise ValueError('bound source candidate outside original independent bounds')
    trace=[];progress=[];last_major=None
    def objective(x):
        check_optimizer();d=np.asarray(x)-anchor;return .5*float(d@d)
    def objective_jac(x):check_optimizer();return np.asarray(x)-anchor
    def callback(x):
        nonlocal last_major
        check_optimizer();c,rows=evaluator.evaluate(x);last_major=np.array(x,copy=True)
        event=dict(iteration=len(trace)+1,objective=objective(x),minimumAnalyticSlack=float(min(c)),
            minimumPrescreenSlack=float(min(c)),unadmittedIndependentKnotsRad=last_major.reshape(3,9).tolist(),
            candidateSHA256=digest(last_major.reshape(3,9).tolist()),analyticCandidateEvaluations=evaluator.stats['analyticCandidateEvaluations'],
            cacheCounters=evaluator.cache_receipt(),currentCandidateEnvelopeResults=[r for r in rows if r.get('envelopeScalar')],footBranchResidualSummary=dict(available=len(rows)==len(evaluator.nodes)+len(evaluator.envelope_specs) and all('maximumPoseResidual'in r for r in rows),
                rows=[dict(phase=r['phase'],fraction=r['fraction'],maximumPoseResidual=r.get('maximumPoseResidual'),error=r.get('error'))for r in rows]),
            nativeAdmission=False,physicalQualification=False)
        trace.append(dict(iteration=event['iteration'],objective=event['objective'],minimumAnalyticSlack=event['minimumAnalyticSlack']))
        progress.append(event)
        if progress_callback is not None:progress_callback(json.loads(json.dumps(event,allow_nan=False)))
    fit=None;failure=None;start=time.monotonic()
    try:
        fit=minimize(objective,initial.copy(),method='SLSQP',jac=objective_jac,bounds=evaluator.bounds,
            constraints=[dict(type='ineq',fun=lambda x:evaluator.evaluate(x)[0],
                **(dict(jac=evaluator.constraint_jacobian)if refined else{}))],callback=callback,
            options=dict(maxiter=max_iterations,ftol=1e-12,eps=1e-6,disp=False))
        check_optimizer()
    except _ControlStop as exc:failure=dict(type=type(exc).__name__,message=str(exc),kind=exc.kind)
    except Exception as exc:failure=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc())
    optimizer_elapsed=time.monotonic()-start
    # Explicit terminal policy, never a merit/best-candidate selection.
    final=np.asarray(fit.x,dtype=np.float64).copy()if fit is not None else(last_major.copy()if last_major is not None else initial.copy())
    final_origin='optimizer-returned-final'if fit is not None else('last-completed-major-iteration'if last_major is not None else('unchanged-deterministic-continuation'if history else'unchanged-bound-source-candidate'))
    certificate=dict(schema=1,declaredSampleCount=123,phases=list(PHASES),fractions=list(CERTIFICATE_FRACTIONS),
        maximumWallSeconds=certificate_max_wall_seconds,originalStrictMapUsed=True,optimizerProxyUsed=False,
        adaptiveGrid=False,wholeContinuousPathCertificate=False,liveCommandRecheckingRequired=True)
    receipt=dict(schema=1,version=VERSION_FRESH_RECOVERY if fresh else(VERSION_REFINEMENT if refined else VERSION_POLISH),inputBinding=input_binding or dict(contextSHA256=current_context.receipt()['contextSHA256']),
        twoStageFeasibilityRestoration=stage,optimizerCalls=1,totalProducerOptimizerCalls=1 if fresh else((retarget['sourceProposalProducerOptimizerCalls']+1+len(history))if retarget is not None else((3+len(history))if refined else 2)),
        optimizerCallsScope='this call1; recovery1 plus failed polish1 plus refinement1 when refined; opaque timeout separately recorded',
        family=dict(variables=27,independentIndices=evaluator.idx.tolist(),originalClocksS=[.8,1.2,1.],clocksS=list(evaluator.entry.durations),
            footLiftM=.04,ownFootAdvanceM=.182,sourceLandingContactPreferenceM=-.0005,sourceInteriorRad=.01,
            anchoredLumbarChangeDeg=5.,armAndMapReserveRad=.02,supportReserveM=.003,
            maximumCommandCOMSpeedMps=MAX_TOTAL_COM_SPEED,maximumCommandCOMAccelerationMps2=MAX_TOTAL_COM_ACCELERATION,
            objective='minimum .5 Euclidean squared27-coordinate correction from exact bound recovery major iteration33',
            originalBodyKneeGoalShapeRetainedAsAnchor=True,newPostureOrPelvisObjective=False,optimizerConstraintFractions=list(FRACTIONS),
            phaseSpecificConstraintNodes=[dict(phase=PHASES[k],fraction=f)for k,f in evaluator.declared_nodes],
            currentCandidateBracketEnvelopes=refined,envelopeTheoremJacobian=refined,globalTranslateSupportEnvelopes=global_translate_support_envelopes,globalSupportPhases=list(global_phases),
            refinementConstraintStructure='original13 full blocks plus all-declared-phase closed-quarter support faces and retained lift/land scalar envelopes'if retarget is not None else('original13 full blocks plus closed-quarter per-face translate and retained lift/land scalar envelopes'if history else('13 original full blocks plus3 scalar bracket envelopes'if refined else'13 original full blocks')),
            analyticCommandCOMJets=True,centralCOMDifferencing=False,optimizerOnlyRawVerifiedMapProxy=True,
            rawDomainExtrapolation=False,originalReservedMapInequalityUnchanged=True,strictOriginalMapInFinalPathAndRuntime=True,
            maxIterations=max_iterations,maxWallSeconds=max_wall_seconds,SLSQPEps=1e-6,SLSQPftol=1e-12,
            objectiveJacobian='exact x minus bound anchor',noRestart=True,noSeedGoalWeightScan=True),
        optimizerSuccess=bool(fit is not None and fit.success and control is None),optimizerStatus=None if fit is None else str(fit.message),
        optimizerIterations=None if fit is None else int(fit.nit),completedCallbackIterations=len(trace),optimizerElapsedS=optimizer_elapsed,
        optimizerException=failure,cooperativeControlTermination=control,trace=trace,progressRecords=progress,
        finalCandidateOrigin=final_origin,finalCandidateSHA256=digest(final.reshape(3,9).tolist()),
        unadmittedFinalIndependentKnotsRad=final.reshape(3,9).tolist(),
        unadmittedLastRequestedIndependentKnotsRad=None if evaluator.last_requested is None else evaluator.last_requested.reshape(3,9).tolist(),
        unadmittedLastFullyEvaluatedIndependentKnotsRad=None if evaluator.last_fully_evaluated is None else evaluator.last_fully_evaluated.reshape(3,9).tolist(),
        analyticPrescreenExceptions=evaluator.exceptions,exactInputCaches=evaluator.cache_receipt(),
        constraintRowsPerNode=len(evaluator.labels),originalConstraintRowCount=13*len(evaluator.labels),
        envelopeScalarConstraintCount=len(evaluator.envelope_specs),totalConstraintRowCount=13*len(evaluator.labels)+len(evaluator.envelope_specs),constraintLabels=list(evaluator.labels),analyticPrescreenNodes=[dict(phase=PHASES[k],fraction=f)for k,f in evaluator.declared_nodes],
        nativeAdmission=False,physicalQualification=False,nativeWorldConstructions=0,physicsIntegrations=0,stateInstallations=0,
        exactWholeNonlinearPathAdmission=False,globalInfeasibilityProven=False,repositoryRuntimeEdits=False,
        explicitPelvisPreferenceInFlightObjective=False,referenceLandingCenterDifferenceM=float(current_context.feet[realizer.swing_side].translation.position[2]-json.loads(current_context.config_json)['floorTargets'][realizer.swing_side]),
        plannerFastGeometryUsed=node_geometry is not None,plannerFastGeometrySourceName=geometry_binding.get('filename'),
        plannerFastGeometrySourceSHA256=geometry_binding.get('SHA256'),plannerFastGeometrySHA256=geometry_binding.get('SHA256'),
        strictCertificateCacheUsed=certificate_sampler is not None,strictCertificateCacheSourceName=strict_sampler_binding.get('filename'),
        strictCertificateCacheSHA256=strict_sampler_binding.get('SHA256'))
    if refinement is not None:receipt['collocationRefinement']=refinement
    if fresh:
        receipt.pop('twoStageFeasibilityRestoration')
        receipt.update(freshRecoveryProblem=fresh_problem,initialContinuationIndependentKnotsRad=initial.reshape(3,9).tolist(),
            initialContinuationCandidateSHA256=digest(initial.reshape(3,9).tolist()),objectiveAnchorSHA256=digest(anchor.reshape(3,9).tolist()),
            globalPhaseSupportEnvelopeCoverage=evaluator.phase_coverage_receipt(),activeEnvelopeConfiguration=evaluator.envelope_specs,
            activeConstraintRowCounts=dict(original=13*len(evaluator.labels),declaredAdditionalEnvelopes=2,
                globalPhaseSupportEnvelopes=8*len(evaluator.eq)*len(global_phases),total=13*len(evaluator.labels)+len(evaluator.envelope_specs)))
        receipt['optimizerCallsScope']='this explicitly new fresh-recovery problem only; lost historical producer receipts are not relabeled'
        receipt['family'].update(objective='minimum .5 Euclidean squared27-coordinate correction from exact recovered accepted coordinate vector',
            originalBodyKneeGoalShapeRetainedAsAnchor=True,refinementConstraintStructure='original13 full blocks plus all-phase closed-quarter support faces and2 freshly declared scalar envelopes')
    if history and retarget is None:
        receipt.update(exchangeHistory=history,initialContinuationIndependentKnotsRad=initial.reshape(3,9).tolist(),
            initialContinuationCandidateSHA256=digest(initial.reshape(3,9).tolist()),objectiveAnchorSHA256=digest(anchor.reshape(3,9).tolist()),
            globalTranslateSupportEnvelopeCoverage=evaluator.coverage_receipt(),
            translateFaceEnvelopeCoverage=[dict(quantity=r['pointKind'],faceIndex=r['faceIndex'],intervalFractions=r['intervalFractions'])for r in evaluator.coverage_receipt()['constraints']],
            activeConstraintRowCounts=dict(original=13*len(evaluator.labels),retainedLiftLandEnvelopes=2,
                globalTranslateEnvelopes=8*len(evaluator.eq),total=13*len(evaluator.labels)+len(evaluator.envelope_specs)),
            activeEnvelopeConfiguration=evaluator.envelope_specs,historicalCollocationBindingUnchanged=True)
    if retarget is not None:
        if history:receipt['exchangeHistory']=history
        receipt.update(activationRetargeting=retarget,initialContinuationIndependentKnotsRad=initial.reshape(3,9).tolist(),
            initialContinuationCandidateSHA256=digest(initial.reshape(3,9).tolist()),objectiveAnchorSHA256=digest(anchor.reshape(3,9).tolist()),
            globalPhaseSupportEnvelopeCoverage=evaluator.phase_coverage_receipt(),
            activeConstraintRowCounts=dict(original=13*len(evaluator.labels),retainedLiftLandEnvelopes=2,
                globalPhaseSupportEnvelopes=8*len(evaluator.eq)*len(global_phases),total=13*len(evaluator.labels)+len(evaluator.envelope_specs)),
            activeEnvelopeConfiguration=evaluator.envelope_specs,historicalCollocationBindingUnchanged=True)
    if evaluator.last_fully_evaluated is not None and np.array_equal(final,evaluator.last_fully_evaluated):
        c,rows=evaluator.last_result;receipt['lastEvaluatedFinalAnalyticPrescreen']=dict(minimumSlack=float(min(c)),constraints=c.tolist(),rows=rows)
    samples=[];dense=[];extrema_samples=[];searches=[];certificate_start=time.monotonic();certificate_stop=None
    path=evaluator.path(final,strict_map=True);sample_cache=OrderedDict();sample_cache_capacity=4096
    certificate_cache=dict(hits=0,misses=0,evictions=0)
    def check_certificate():
        if cancellation_check is not None and cancellation_check():raise _ControlStop('certificate-caller-cancelled')
        if time.monotonic()-certificate_start>=certificate_max_wall_seconds:raise _ControlStop('certificate-compute-cap')
    def sample_at(k,t,f):
        check_certificate();key=(int(k),float(t))
        if key in sample_cache:
            certificate_cache['hits']+=1;sample_cache.move_to_end(key);row=sample_cache[key]
        else:
            certificate_cache['misses']+=1
            try:
                sample=(path.sample(PHASES[k],t)if certificate_sampler is None else certificate_sampler.sample(path,PHASES[k],t));check_certificate()
                row=_sample_row(path,sample,k,f,current_support,np.asarray(heading),np.asarray(current_context.com.position),evaluator.q0)
                p,v,a=sample.com.arrays();eq=evaluator.eq;cp=p[:2]+math.sqrt(p[2]/9.81)*v[:2]
                row['timeExtremaScalarSlacks']=dict(COMSpeed=MAX_TOTAL_COM_SPEED-float(np.linalg.norm(v)),
                    COMAcceleration=MAX_TOTAL_COM_ACCELERATION-float(np.linalg.norm(a)),
                    **{f'COMFace{i}':float(z)for i,z in enumerate(-(eq[:,:2]@p[:2]+eq[:,2])-.003)},
                    **{f'captureFace{i}':float(z)for i,z in enumerate(-(eq[:,:2]@cp+eq[:,2])-.003)})
            except _ControlStop:raise
            except Exception as exc:row=dict(phase=PHASES[k],fraction=f,accepted=False,errorType=type(exc).__name__,error=str(exc),originalStrictMapEvaluationPassed=False)
            sample_cache[key]=row
            if len(sample_cache)>sample_cache_capacity:sample_cache.popitem(last=False);certificate_cache['evictions']+=1
        return dict(row,phase=PHASES[k],fraction=float(f),phaseTimeS=float(t))
    dense_grid=[np.linspace(0.,T,int(math.ceil(T/.005))+1)for T in path.durations]if refined else[]
    dense_receipt=dict(schema=1,maximumStepS=None if not refined else max(float(max(np.diff(times)))for times in dense_grid),
        passed=False,originalStrictMapUsed=True,sampleCount=0,phaseDurationS=list(path.durations),
        declaredSampleCount=sum(len(times)for times in dense_grid),endpointInclusive=True,nominalFrequencyHz=200.,wholePathProven=False)
    extrema_receipt=dict(schema=1,status='not-requested'if not refined else'pending',passed=not refined,
        originalStrictMapUsed=True,wholePathProven=False,method='bounded scalar minimization inside every strict local-minimum bracket on the dense grid; all endpoints separately sampled',
        metrics=['COMSpeed','COMAcceleration',*[f'COMFace{i}'for i in range(len(evaluator.eq))],*[f'captureFace{i}'for i in range(len(evaluator.eq))]],
        maximumIterationsPerBracket=40,timeToleranceS=1e-8,searchCount=0,phaseMetricCoverage=[])
    try:
        for k,phase in enumerate(PHASES):
            for f in CERTIFICATE_FRACTIONS:samples.append(sample_at(k,f*path.durations[k],f))
        if refined:
            for k,times in enumerate(dense_grid):
                for t in times:dense.append(sample_at(k,float(t),float(t/path.durations[k])))
            # All scalar faces are considered independently, including duplicate
            # actual hull/triangle faces. Only bracket searches vary time; there
            # is no additional trajectory optimizer or collocation refinement.
            for k,times in enumerate(dense_grid):
                phase_rows=[row for row in dense if row['phase']==PHASES[k]]
                for metric in extrema_receipt['metrics']:
                    check_certificate();valid=all('timeExtremaScalarSlacks'in row for row in phase_rows)
                    coverage=dict(phase=PHASES[k],metric=metric,allDenseValuesAvailable=valid,bracketCount=0)
                    extrema_receipt['phaseMetricCoverage'].append(coverage)
                    if not valid:continue
                    values=[row['timeExtremaScalarSlacks'][metric]for row in phase_rows]
                    coverage['minimumDenseSlack']=float(min(values))
                    for j in range(1,len(times)-1):
                        if not(values[j]<values[j-1]and values[j]<=values[j+1]):continue
                        coverage['bracketCount']+=1;left,right=float(times[j-1]),float(times[j+1])
                        record=dict(phase=PHASES[k],metric=metric,bracketS=[left,float(times[j]),right],success=False)
                        searches.append(record)
                        def scalar(t):
                            row=sample_at(k,float(t),float(t/path.durations[k]));extrema_samples.append(dict(row,extremaMetric=metric))
                            if 'timeExtremaScalarSlacks'not in row:raise ValueError('strict sample rejected during bounded time search')
                            return row['timeExtremaScalarSlacks'][metric]
                        try:
                            answer=minimize_scalar(scalar,bounds=(left,right),method='bounded',options=dict(maxiter=40,xatol=1e-8))
                            check_certificate();record.update(success=bool(answer.success),timeS=float(answer.x),fraction=float(answer.x/path.durations[k]),
                                minimumSlack=float(answer.fun),evaluations=int(answer.nfev),iterations=int(answer.nit))
                        except _ControlStop:raise
                        except Exception as exc:record.update(errorType=type(exc).__name__,error=str(exc))
    except _ControlStop as exc:certificate_stop=exc.kind
    certificate.update(completedSampleCount=len(samples),elapsedS=time.monotonic()-certificate_start,termination=certificate_stop,
        passed=len(samples)==123 and all(r['accepted']for r in samples))
    if refined:
        dense_receipt.update(sampleCount=len(dense),passed=len(dense)==dense_receipt['declaredSampleCount']and all(r['accepted']for r in dense))
        extrema_receipt.update(searchCount=len(searches),evaluatedSampleCount=len(extrema_samples),
            passed=bool(certificate_stop is None and len(extrema_receipt['phaseMetricCoverage'])==3*len(extrema_receipt['metrics'])
                and all(r['allDenseValuesAvailable']for r in extrema_receipt['phaseMetricCoverage'])
                and all(r['success']for r in searches)and all(r['accepted']for r in extrema_samples)))
        extrema_receipt['status']='passed-numeric-checks'if extrema_receipt['passed']else'rejected-or-incomplete'
    all_samples=[*samples,*dense,*extrema_samples]
    strict_map=bool(len(samples)==123 and(not refined or len(dense)==dense_receipt['declaredSampleCount'])
        and all(r['originalStrictMapEvaluationPassed']for r in all_samples))
    if fresh:
        receipt.update(maximumCorrectionFromFreshAnchorRad=float(max(abs(final-anchor))),freshRecovery123SampleCertificate=certificate)
    elif retarget is None:
        stage.update(maximumCorrectionRad=float(max(abs(final-anchor))),finalOriginalStrictMapRestored=strict_map,declared123SampleCertificate=True,certificateReceipt=certificate)
    else:
        # Historical stage is retained exactly; current strict admission is
        # explicit, not silently substituted for its old-input certificate.
        stage['maximumCorrectionRad']=float(max(abs(final-anchor)))
        receipt['activationRetargeting123SampleCertificate']=certificate
    endpoints=[r for r in samples if r['fraction']==1.];liftgain=endpoints[0].get('kneeReferenceGainDeg',0.)if endpoints else 0.
    finite_pass=bool(certificate['passed']and(not refined or dense_receipt['passed']and extrema_receipt['passed']))
    accepted=bool(receipt['optimizerSuccess']and finite_pass and strict_map and liftgain>1e-6)
    receipt.update(accepted=accepted,terminalState=control or certificate_stop or('finite-feasibility-restored'if accepted else'finite-feasibility-rejected'),
        analyticSamples=samples,analyticFailures=[r for r in all_samples if not r['accepted']],finiteSampledKinematicPass=finite_pass,
        endpointSummaries=endpoints,achievedEarlyKneeReferenceGainDeg=liftgain,desiredEarlyKneeCorrectionPresent=bool(liftgain>1e-6),
        finalOriginalStrictMapRestored=strict_map,path=path.receipt(),finiteProposalAdmittedForNative=False,
        certificateElapsedS=certificate['elapsedS'],optimizerElapsedPlusAdmissionS=optimizer_elapsed+certificate['elapsedS'],
        certificateExactInputCache=dict(certificate_cache,entries=len(sample_cache),capacity=sample_cache_capacity),
        independentPolynomialWholeBounds=[segment.component_bounds().tolist()for segment in path.independent_segments],**summarize_samples(samples))
    if refined:
        receipt.update(denseCommandGridCertificate=dense_receipt,denseCommandSamples=dense,
            extremaVerification=extrema_receipt,timeExtremaSearches=searches,extremaCommandSamples=extrema_samples,
            densePhaseMetrics=summarize_samples(dense),extremaScalarSearchCalls=len(searches),
            trajectoryOptimizerCalls=1,extremaSearchesAreNotTrajectoryOptimizers=True)
    if accepted:receipt['independentKnotsRad']=receipt.pop('unadmittedFinalIndependentKnotsRad')
    return(path if accepted else None),receipt
