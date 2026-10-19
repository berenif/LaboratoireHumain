"""Exact fixed-single-stance contact-wrench elimination for27-pose planning.

No World is constructed and no trajectory optimizer is called here. A native
provider supplies original M*qdd+bias, B, fixed-origin body Jacobian and actual
point Jacobians. This removes zero-objective force witnesses, not controller
normal/shear regularization or physical/runtime admission.
"""
import itertools,math
from collections import OrderedDict
import numpy as np
from scipy.optimize import linprog
from coupled_posture_reference import digest
from native_endpoint_restoration import MOTOR_CAP,OPTIMIZER_MOTOR_CAP,PLANNING_NUMERICAL_INTERIOR,stationary_sample_guard
from strict_reference_cache import StrictReferenceCache

VERSION='fixed-stance-native-wrench-elimination-v1'

def skew(p):
    x,y,z=np.asarray(p,float);return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

def force_inequalities():
    U=np.zeros((12,9))
    for i in range(3):U[4*i:4*i+4,3*i:3*i+3]=[[1.,0.,-.6],[-1.,0.,-.6],[0.,1.,-.6],[0.,-1.,-.6]]
    return U

class ContactWrenchCone:
    """All deterministic5-generator support facets of the original12 rays."""
    def __init__(self,points_source,origin_source,*,classification_tolerance=1e-11):
        points=np.asarray(points_source,dtype=np.float64);origin=np.asarray(origin_source,dtype=np.float64)
        if points.shape!=(3,3)or origin.shape!=(3,)or not np.isfinite(points).all()or not np.isfinite(origin).all():
            raise ValueError('exact actual3D three-contact points/fixed origin required')
        if classification_tolerance!=1e-11:raise ValueError('one declared classification tolerance; no tolerance scan')
        self.points=points.copy();self.origin=origin.copy();self.tolerance=classification_tolerance
        self.D=np.hstack([np.r_[np.eye(3),skew(p-origin)]for p in points])
        directions=np.array([[x,y,1.]for x in(-.6,.6)for y in(-.6,.6)]).T
        self.rays=np.column_stack([self.D[:,3*i:3*i+3]@directions[:,j]for i in range(3)for j in range(4)])
        length=float(max(np.linalg.norm(points-origin,axis=1)))
        if length<=1e-8:raise ValueError('nondegenerate actual contact length required')
        self.scale=np.r_[np.ones(3),np.ones(3)*length];C=self.rays/self.scale[:,None]
        singular=np.linalg.svd(C,compute_uv=False)
        if singular[-1]<=100*self.tolerance:raise ValueError('rank6 wrench cone or clear numerical rank gap absent')
        groups={};planes=0;minimum_gap=math.inf;minimum_rank=math.inf
        for subset in itertools.combinations(range(12),5):
            _,s,Vh=np.linalg.svd(C[:,subset].T,full_matrices=True)
            if self.tolerance<s[-1]<=100*self.tolerance:raise ValueError('ambiguous near-rank5 support subset')
            if s[-1]<=self.tolerance:continue
            h=Vh[-1];dots=h@C
            if max(dots)<=self.tolerance:pass
            elif min(dots)>=-self.tolerance:h=-h;dots=-dots
            else:continue
            nonzero=abs(dots)[abs(dots)>self.tolerance]
            if len(nonzero)and min(nonzero)<=100*self.tolerance:raise ValueError('ambiguous near-zero support-ray classification')
            support=tuple(int(i)for i in np.flatnonzero(abs(dots)<=self.tolerance))
            support_s=np.linalg.svd(C[:,support].T,compute_uv=False)
            rank=int(sum(support_s>self.tolerance))
            if rank!=5:raise ValueError('candidate support set is not a genuine rank5 cone facet')
            planes+=1;minimum_rank=min(minimum_rank,float(s[-1]))
            if len(nonzero):minimum_gap=min(minimum_gap,float(min(nonzero)))
            candidate=dict(support=support,normal=h.copy(),sourceSubset=subset,conditioning=float(s[-1]))
            if support not in groups or candidate['conditioning']>groups[support]['conditioning']:groups[support]=candidate
        if not groups:raise ValueError('no complete deterministic cone facets found')
        self.facets=tuple(groups[k]for k in sorted(groups));self.H=np.array([x['normal']/self.scale for x in self.facets])
        # Redundant explicit original total-normal sign, not an upper bound.
        self.H=np.vstack([self.H,[0.,0.,-1.,0.,0.,0.]])
        residual=float(np.max(self.H@self.rays))
        if residual>self.tolerance:raise ValueError('constructed facet excludes an original ray beyond classification roundoff')
        self.receipt=dict(schema=1,version=VERSION,pointsSource=points.tolist(),originSource=origin.tolist(),
            actualPointBytesSHA256=__import__('hashlib').sha256(points.tobytes()).hexdigest(),pointSemanticSHA256=digest(points.tolist()),
            momentScaleM=length,friction=.6,normalForceDomain='nonnegative-unbounded',rayWrenches=self.rays.T.tolist(),
            candidateSubsets=792,supportedRank5PlaneCount=planes,uniqueSupportingRaySetCount=len(groups),facetCount=len(self.facets),
            facetSupportingRayIndices=[list(x['support'])for x in self.facets],facetSourceSubsets=[list(x['sourceSubset'])for x in self.facets],
            physicalFacetNormals=self.H.tolist(),classificationTolerance=self.tolerance,minimumRank6SingularValue=float(singular[-1]),
            minimumAcceptedRank5SingularValue=minimum_rank,minimumNonzeroSupportDotGap=minimum_gap,maximumOriginalRayFacetResidual=residual,
            QJ=False,flattenedContacts=False,physicalInequalityToleranceRelaxation=0.,explicitTotalNormalSign=True)
        for a in(self.points,self.origin,self.D,self.rays,self.scale,self.H):a.setflags(write=False)
    def slacks(self,wrench,*,force_scale=1.):
        w=np.asarray(wrench,float)
        if w.shape!=(6,)or not np.isfinite(w).all()or not math.isfinite(force_scale)or force_scale<=0.:raise ValueError('finite wrench/positive scale required')
        return -(self.H@w)/force_scale
    def primal_check(self,wrench):
        """Independent ORIGINAL9 force variables, no facet constraints used."""
        w=np.asarray(wrench,float)
        fit=linprog(np.zeros(9),A_ub=force_inequalities(),b_ub=np.zeros(12),A_eq=self.D,b_eq=w,
            bounds=[(None,None),(None,None),(0.,None)]*3,method='highs')
        result=dict(success=bool(fit.success),status=str(fit.message),normalForceUpperBound=None,friction=.6)
        if fit.success:result.update(forcesSource=fit.x.reshape(3,3).tolist(),maximumWrenchResidual=float(max(abs(self.D@fit.x-w))),
            maximumFrictionViolation=float(max(0.,max(force_inequalities()@fit.x))))
        return result

def eliminate_native_wrench(data,cone):
    """Unique native required wrench/motor, with explicit fixed-frame checks."""
    B=np.asarray(data['B'],float);J=np.asarray(data.get('stanceJacobian',data.get('J')),float);G=np.asarray(data['G'],float);d=np.asarray(data.get('demand',data.get('d')),float)
    origin=np.asarray(data['referenceOriginSource'],dtype=np.float64)
    if(B.shape!=(27,21)or J.shape!=(6,27)or G.shape!=(27,9)or d.shape!=(27,)
            or not all(np.isfinite(a).all()for a in(B,J,G,d))or origin.tobytes()!=cone.origin.tobytes()):
        raise ValueError('finite native matrices in the EXACT immutable cone-origin frame required')
    if np.any(B[:6]!=0.):raise ValueError('root-zero original actuator structure required for wrench elimination')
    base=J[:,:6].T;Bj=B[6:];condition=float(np.linalg.cond(base));motor_condition=float(np.linalg.cond(Bj))
    if not math.isfinite(condition)or condition>1e10 or not math.isfinite(motor_condition)or motor_condition>1e10:
        raise ValueError('native root/motor elimination is singular or ill-conditioned')
    factorization=float(np.max(abs(G-J.T@cone.D)))
    if factorization>1e-8:raise ValueError('actual point Jacobians do not factor about the bound fixed origin')
    w=np.linalg.solve(base,d[:6]);u=np.linalg.solve(Bj,d[6:]-J[:,6:].T@w)
    return dict(wrench=w,motor=u,rootCondition=condition,motorCondition=motor_condition,pointJacobianFactorizationError=factorization,
        rootBalanceResidual=float(max(abs(base@w-d[:6]))),jointBalanceResidual=float(max(abs(Bj@u+J[:,6:].T@w-d[6:]))))

def independent_native_primal_check(data,cone):
    """Final full27-row native equilibrium with independent9-force witness."""
    reduced=eliminate_native_wrench(data,cone);B=np.asarray(data['B']);G=np.asarray(data['G']);d=np.asarray(data.get('demand',data.get('d')));u=reduced['motor']
    rhs=d-B@u;fit=linprog(np.zeros(9),A_ub=force_inequalities(),b_ub=np.zeros(12),A_eq=G,b_eq=rhs,
        bounds=[(None,None),(None,None),(0.,None)]*3,method='highs')
    result=dict(success=bool(fit.success),status=str(fit.message),motorCommands=u.tolist(),maximumMotorFraction=float(max(abs(u))),
        originalStrictMotorCap=MOTOR_CAP,normalForceUpperBound=None,friction=.6,nativeAdmission=False)
    if fit.success:
        residual=float(max(abs(B@u+G@fit.x-d)));friction=float(max(0.,max(force_inequalities()@fit.x)))
        result.update(forcesSource=fit.x.reshape(3,3).tolist(),maximumNativeEquilibriumResidual=residual,
            maximumFrictionViolation=friction,accepted=bool(max(abs(u))<MOTOR_CAP and residual<1e-8 and friction<1e-8))
    else:result['accepted']=False
    return result

def local_trust_bounds(original_bounds,exact_center,radius_rad):
    """One declared local search box, never clipping an optimizer evaluation."""
    center=np.asarray(exact_center,float).reshape(27)
    if not math.isfinite(radius_rad)or radius_rad<=0.:raise ValueError('positive predeclared numerical trust radius required')
    result=tuple((max(lo,float(q-radius_rad)),min(hi,float(q+radius_rad)))for q,(lo,hi)in zip(center,original_bounds))
    if len(result)!=27 or any(not lo<=q<=hi for q,(lo,hi)in zip(center,result)):raise ValueError('exact continuation lies outside original bounds')
    return result

class NativeWrenchPathEvaluator:
    """Thin27-coordinate extension of the existing geometric evaluator only.

    provider(complete_sample,final=False) returns original native matrices about cone.origin
    plus physical native/command COM height and clean gap. It may NOT project a
    rejected pose, change contacts, or substitute fabricated constraint values.
    """
    def __init__(self,kinematic_evaluator,native_provider,cone,*,endpoint_only=False,sample_provider=None):
        self.kin=kinematic_evaluator;self.provider=native_provider;self.cone=cone;self.endpoint_only=bool(endpoint_only)
        self.sampler=StrictReferenceCache()if sample_provider is None else sample_provider
        self.nodes=tuple((k,1.)for k in range(3))if endpoint_only else tuple(self.kin.nodes)
        self.bounds=tuple(self.kin.bounds);self.cache=OrderedDict();self.native_cache=OrderedDict();self.jac_cache=OrderedDict()
        self.stats=dict(nativeHits=0,nativeMisses=0,candidateHits=0,candidateMisses=0,nativeFDColumns=0)
        self.last_requested=None;self.last_rejection=None
    def native_node(self,path,k,f):
        self.kin.check_control();t=float(f)*path.durations[k];jet=path.independent_segments[k].at(t)
        key=(k,float(t),*(np.asarray(a,dtype=np.float64).tobytes()for a in jet.arrays()))
        if key in self.native_cache:self.stats['nativeHits']+=1;return self.native_cache[key]
        sample=self.sampler.sample(path,('lift','translate','land')[k],t)
        if self.endpoint_only:stationary_sample_guard(sample)
        data=self.provider(sample,final=False)
        reduced=eliminate_native_wrench(data,self.cone);u=reduced['motor'];W=float(data['forceScaleN'])
        if not math.isfinite(W)or W<=0.:raise ValueError('fixed original body-weight normalization required')
        guards=[float(sample.com.position[2])-.65,float(data['nativeCOMHeightM'])-.65,float(data['minimumCleanSelfGapM'])-.025]
        if not np.isfinite(guards).all():raise ValueError('finite original physical guards required')
        c=np.r_[OPTIMIZER_MOTOR_CAP-u,OPTIMIZER_MOTOR_CAP+u,self.cone.slacks(reduced['wrench'],force_scale=W),guards]
        row=dict(phase=('lift','translate','land')[k],fraction=float(f),minimumNativeSlack=float(min(c)),maximumMotorFraction=float(max(abs(u))),
            requiredWrench=reduced['wrench'].tolist(),pointJacobianFactorizationError=reduced['pointJacobianFactorizationError'],
            originalStrictMotorCap=MOTOR_CAP,optimizerMotorCap=OPTIMIZER_MOTOR_CAP,planningNumericalInterior=PLANNING_NUMERICAL_INTERIOR)
        result=(c,row);self.native_cache[key]=result;self.stats['nativeMisses']+=1
        if len(self.native_cache)>4096:self.native_cache.popitem(last=False)
        return result
    def geometry(self,x,path):
        if self.endpoint_only:
            values=[self.kin.node(path,k,f)for k,f in self.nodes]
            return np.concatenate([v[0]for v in values]),tuple(v[1]for v in values)
        values,rows=self.kin.evaluate(x)
        if any('error'in row or 'errorType'in row for row in rows):raise ValueError('genuine geometric baseline unavailable; no synthetic native derivative')
        return values,rows
    def evaluate(self,x):
        self.kin.check_control()
        x=np.asarray(x,dtype=np.float64);self.last_requested=x.copy()
        if x.shape!=(27,)or not np.isfinite(x).all():raise ValueError('finite27-pose decision required')
        key=x.tobytes()
        if key in self.cache:self.stats['candidateHits']+=1;return self.cache[key]
        try:
            path=self.kin.path(x);g,rows=self.geometry(x,path);native=[self.native_node(path,k,f)for k,f in self.nodes]
            result=(np.r_[g,*[v[0]for v in native]],dict(geometricRows=rows,nativeRows=[v[1]for v in native]))
        except Exception as exc:self.last_rejection=dict(candidate=x.tolist(),type=type(exc).__name__,message=str(exc));raise
        self.cache[key]=result;self.stats['candidateMisses']+=1
        if len(self.cache)>128:self.cache.popitem(last=False)
        return result
    def jacobian(self,x):
        self.kin.check_control()
        x=np.asarray(x,dtype=np.float64);key=x.tobytes()
        if key in self.jac_cache:return self.jac_cache[key]
        baseline,_=self.evaluate(x);path=self.kin.path(x)
        native_baseline=np.concatenate([self.native_node(path,k,f)[0]for k,f in self.nodes]);geometry_count=len(baseline)-len(native_baseline)
        if self.endpoint_only:J=np.zeros((len(baseline),27))
        else:J=np.vstack([self.kin.constraint_jacobian(x),np.zeros((len(native_baseline),27))])
        for j,(lo,hi)in enumerate(self.bounds):
            self.kin.check_control();h=1e-6 if x[j]+1e-6<=hi else-1e-6
            if not lo<=x[j]+h<=hi:raise ValueError('no bounded finite-difference interior')
            shifted=x.copy();shifted[j]+=h;actual=shifted[j]-x[j];p=self.kin.path(shifted)
            if self.endpoint_only:
                value,_=self.evaluate(shifted);J[:,j]=(value-baseline)/actual
            else:
                value=np.concatenate([self.native_node(p,k,f)[0]for k,f in self.nodes]);J[geometry_count:,j]=(value-native_baseline)/actual
            self.stats['nativeFDColumns']+=1
        self.jac_cache[key]=J
        if len(self.jac_cache)>32:self.jac_cache.popitem(last=False)
        return J
