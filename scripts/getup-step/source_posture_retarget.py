"""Standalone extraction of the ONE tested static retargeting family.

The supplied actual controller/world is read-only. All q/v installations are in
fresh disposable worlds at native counter zero. No integration, rollout,
forecast/history update, fallback family, or runtime arm-target replacement.
"""
from dataclasses import dataclass,asdict,is_dataclass
from collections import deque
from pathlib import Path
import ctypes as C
import inspect,json,hashlib,math
import numpy as np
import scipy
from scipy.optimize import minimize,linprog
from scipy.spatial.transform import Rotation
from coupled_posture_reference import (Jet,OrientationJet,CoupledSample,Candidate,
    C2Segment,CurrentSupport,chart,digest,frozen,MAX_TOTAL_COM_SPEED,MAX_TOTAL_COM_ACCELERATION)
from coupled_posture_kinematics import ModelFK,MatrixJet,P,pose_jet_from_source,pose_jet_to_source

VERSION='current-reference-single-family-v1'
MOTOR_CAP=.9249
SUPPORT_INSET=.004
MAX_ITERATIONS=100
COM_HANDOFF_DURATION=1.52
ORDER=(1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20)


def _jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if is_dataclass(value):return _jsonable(asdict(value))
    if isinstance(value,dict):return {str(k):_jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple,deque)):return [_jsonable(x) for x in value]
    if value is None or isinstance(value,(str,int,float,bool)):return value
    return repr(value)


def _sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()


@dataclass(frozen=True)
class RetargetResult:
    accepted: bool
    candidate: object
    receipt_json: str

    def receipt(self):
        return json.loads(self.receipt_json)


def initial_coupled_reference(result_or_receipt):
    """Rehydrate the bound previous-effective CoupledSample without aliases.

    Accepts RetargetResult, its immutable receipt_json, or a receipt() copy.
    The complete input digest is checked before any initial jet is returned.
    """
    if isinstance(result_or_receipt,RetargetResult):receipt=result_or_receipt.receipt()
    elif isinstance(result_or_receipt,str):receipt=json.loads(result_or_receipt)
    else:receipt=json.loads(json.dumps(_jsonable(result_or_receipt),allow_nan=False))
    inputs=receipt.get('input')
    if inputs is None or digest(inputs)!=receipt.get('inputSHA256'):
        raise ValueError('initial coupled reference input digest mismatch')
    raw=inputs['initialCoupledReference']
    return CoupledSample(Jet(**raw['com']),Jet(**raw['lower_joints']),
        OrientationJet(**raw['thorax']),OrientationJet(**raw['pelvis']))


def _actual_snapshot(c,context):
    w=c.w
    history={k:v for k,v in vars(c).items() if k not in ('w','_mapping') and not callable(v)}
    mapping=c._mapping
    mapstate={name:getattr(mapping,name,None) for name in ('ranges','neutral_pitch','goal')}
    mapstate['curves']=[dict(x=curve.x,c=curve.c) for curve in getattr(mapping,'curves',())]
    support_count=w.l.lh_support_count(w.h)
    return digest(_jsonable(dict(counter=w.l.lh_dim(w.h,6),sourceTime=w.l.lh_time(w.h),
        q=w.arr(0,(28,)).copy(),v=w.arr(1,(27,)).copy(),u=w.arr(2,(21,)).copy(),
        supportCount=support_count,support=w.arr(21,(128,))[:2*support_count].copy(),
        context=context.receipt(),controller=history,mapping=mapstate)))


def _current_support(c,context,provisional_survivors=None):
    w=c.w;foot=w.feet()[c.stance_side]
    corners=np.array(foot['points'],dtype=float);corners[:,2]-=foot['radius']
    count=w.l.lh_support_count(w.h)
    loaded=w.arr(21,(128,))[:2*count].reshape(-1,2).copy()
    if len(loaded)<3:raise ValueError('current actual loaded hull is absent')
    selected=getattr(c,'robust_support',{})
    selected_now=bool(selected.get('selected',False))
    if selected_now:
        raw_ids=tuple(getattr(c,'_robust_survivors',selected.get('survivingCorners',())))
        if provisional_survivors is not None and tuple(provisional_survivors)!=raw_ids:
            raise ValueError('explicit provisional IDs conflict with selected current geometry')
        status='selected causal history consulted for static geometry only'
    else:
        if provisional_survivors is None:raise ValueError('current selected causal support is absent; explicit provisional current survivor IDs required')
        raw_ids=tuple(provisional_survivors)
        status='PROVISIONAL current actual corner proposal; no admitted history'
    if (len(raw_ids)!=3 or len(set(raw_ids))!=3
            or any(not isinstance(i,(int,np.integer)) or isinstance(i,(bool,np.bool_)) or i not in range(4) for i in raw_ids)):
        raise ValueError('three unique integer current corner IDs required')
    ids=tuple(int(i) for i in raw_ids)
    matched=corners.copy();present=[];nearest=[];distances=[]
    for i,p in enumerate(corners):
        k=int(np.argmin(np.linalg.norm(loaded-p[:2],axis=1)))
        distance=float(np.linalg.norm(loaded[k]-p[:2]));distances.append(distance)
        present.append(distance<=.001);nearest.append(k)
        if present[-1]:matched[i,:2]=loaded[k]
    for i,k in enumerate(nearest):
        if nearest.count(k)>1:present[i]=False
    current=CurrentSupport(context.source_step,context.source_time,context.step_index,context.phase,
        tuple(map(tuple,loaded)),tuple(map(tuple,matched)),tuple(present),ids,True)
    current.validate_identity(context)
    return current,matched,np.array(distances),tuple(nearest),status


def _position_fk(model,q,root):
    """Position-only evaluation of the independently tested model frame chain."""
    poses={'torso':np.array(root,copy=True)}
    for part in model.parts[1:]:
        R=np.eye(4)
        if part.qindex is not None:
            c,s=math.cos(q[part.qindex]),math.sin(q[part.qindex])
            R[1:3,1:3]=[[c,-s],[s,c]]
        poses[part.name]=poses[part.parent]@part.parent_frame.T@R@part.child_inverse.T
    return poses


def _actuators(w):
    B=np.zeros((27,21))
    for i,j in enumerate(ORDER):B[6+j,i]=w.gears[i]
    return B


def _strict_lp(w,side,B,points):
    """Same original motor-bounded static LP; no QP/controller copy."""
    points=np.asarray(points);body=w.feet()[side]['body'];n=len(points);N=22+3*n
    A=np.zeros((27,N));A[:,:21]=B
    A[:,21:21+3*n]=np.hstack([w.read('lh_point_jac',(3,27),body,
        np.ascontiguousarray(p).ctypes.data_as(C.POINTER(C.c_double))).T for p in points])
    U=[]
    for j in range(21):
        for sign in (1.,-1.):
            row=np.zeros(N);row[j]=sign;row[-1]=-1.;U.append(row)
    for i in range(n):
        for x,y in ((1,0),(-1,0),(0,1),(0,-1)):
            row=np.zeros(N);row[21+3*i]=x;row[22+3*i]=y;row[23+3*i]=-.6;U.append(row)
    objective=np.zeros(N);objective[-1]=1.;bias=w.read('lh_bias',(27,))
    fit=linprog(objective,A_ub=np.array(U),b_ub=np.zeros(len(U)),A_eq=A,b_eq=bias,
        bounds=[(-.95,.95)]*21+[(None,None),(None,None),(0,w.mass*9.81)]*n+[(0,.95)],method='highs')
    receipt=dict(success=bool(fit.success),status=str(fit.message),requiredMotorFraction=None,
        sourceContactPointsM=points.tolist(),originalMotorBounds=[-.95,.95],friction=.6)
    if fit.success:
        forces=fit.x[21:21+3*n].reshape(n,3)
        receipt.update(requiredMotorFraction=float(fit.x[-1]),commands=fit.x[:21].tolist(),forcesN=forces.tolist(),
            maximumEqualityResidual=float(np.max(abs(A@fit.x-bias))),
            maximumFrictionViolation=float(max(0.,np.max(abs(forces[:,:2])-.6*forces[:,2,None]))),
            minimumNormalN=float(np.min(forces[:,2])))
    return receipt


def _native_limb_pitch(w,kind):
    R=w.arr(5,(17,3,3))[1];positions=w.arr(3,(17,3));out=[]
    for name in ('right','left'):
        a,b=(name+'_thigh',name+'_shin') if kind=='leg' else (name+'_upper_arm',name+'_hand')
        ia,ib=[w.l.lh_id(w.h,1,x.encode()) for x in (a,b)]
        d=R.T@(positions[ib]-positions[ia]);out.append(math.atan2(d[0],-d[2]))
    return np.array(out)


def _com_path_receipt(context,target,support):
    segment=C2Segment.between(context.source_time,COM_HANDOFF_DURATION,
        Jet(context.com.position,context.com.velocity,context.com.acceleration),Jet.rest(target))
    coeff=np.array(segment.coefficients);ps=[np.polynomial.Polynomial(coeff[:,i]) for i in range(3)]
    zlo,zhi=segment.component_bounds()[2]
    if zlo<=0.:raise ValueError('handoff has nonpositive COM height')
    taus=(math.sqrt(zlo/9.81),math.sqrt(zhi/9.81));records=[]
    for i,(nx,ny,offset) in enumerate(support.equations()):
        p=nx*ps[0]+ny*ps[1]+offset;v=p.deriv()/COM_HANDOFF_DURATION
        for name,poly in [('COM',p)]+[(f'capture-tau-bound-{j}',p+t*v) for j,t in enumerate(taus)]:
            xs=[0.,1.]+[float(x.real) for x in poly.deriv().roots() if abs(x.imag)<1e-10 and 0.<x.real<1.]
            values=[float(poly(u)) for u in xs]
            records.append(dict(face=i,kind=name,minimumMarginM=-max(values),extremalU=xs,extremalValues=values))
    margin=min(x['minimumMarginM'] for x in records)
    speed=segment.maximum_norm(1);acc=segment.maximum_norm(2)
    return dict(durationS=COM_HANDOFF_DURATION,completeCOMSegment=asdict(segment),
        maximumTotalSpeedMps=speed,maximumTotalAccelerationMps2=acc,
        frozenEnvelopePassed=speed<=MAX_TOTAL_COM_SPEED+1e-10 and acc<=MAX_TOTAL_COM_ACCELERATION+1e-10,
        minimumWholeCurrentCOMCaptureMarginM=margin,wholeCurrent3mmGatePassed=margin>=.003-1e-10,
        wholeCurrent4mmDesignInsetPassed=margin>=.004-1e-10,polynomialFaceExtrema=records,
        currentSupportOnlyNotFutureSupportProof=True,genericWholePath06GuardUnchanged=True)


def retarget_current_reference(controller,prepared_base_context,seed=None,*,
                               provisional_survivors=None,model_path=None,scratch_factory=None):
    """Return immutable candidate + bound receipt, or an explicit rejection.

    Default initializer is the active lower-joint reference projected ONLY into
    the fixed static optimizer bounds. It is not an applied angle clamp. A seed
    may supply 15/21 joint values or a previous immutable Candidate; its root/COM
    are ignored. There is exactly one SLSQP invocation, max100 iterations.

    model_path may be supplied explicitly. Otherwise it is resolved from the
    actual World module's existing ROOT. Missing dependencies reject clearly.
    scratch_factory(actual_world) is injectable for pure/mock tests; production
    defaults to a fresh World of the same class, base, helper and XML.
    provisional_survivors supplies three explicitly PROVISIONAL current actual
    corner IDs before selection. It never authorizes lift or inherits history.
    The caller freezes one returned candidate, starts fresh risk history under
    its effective tasks, and holds/rejects a different triangle. A later LP may
    certify only that unchanged candidate; automatic replanning is not provided.
    """
    c=controller;context=prepared_base_context;actual=c.w
    receipt=dict(schema=1,version=VERSION,accepted=False,oneBoundedFamily=True,
        maximumIterations=MAX_ITERATIONS,noFallback=True,noParameterScan=True,
        noNativeIntegration=True,noLiveStateInstall=True,physicsIntegrations=0,
        delayedArmTaskAndCausalHistoryUnchanged=True,fullPhysicalPathProven=False,
        requestedSourceStep=getattr(context,'source_step',None),requestedSourceTimeS=getattr(context,'source_time',None))
    scratch=None;audit=None;before=None;counts=dict(worlds=0,installs=0,lpCalls=0,geometryCalls=0)
    try:
        context.validate(c,context.elapsed,context.phase)
        if context.phase!='transfer':raise ValueError('retargeting is restricted to the prepared transfer reference')
        if context.elapsed<c.phase_duration('transfer')-1e-10:
            raise ValueError('retargeting requires the nominal transfer-reference endpoint')
        before=_actual_snapshot(c,context)
        cfg=json.loads(context.config_json)
        if int(cfg['stance_side'])!=int(c.stance_side) or not np.array_equal(np.array(cfg['leg_origin']),np.array(c.leg_origin)):
            raise ValueError('prepared stance side or persistent leg origins differ from actual controller')
        support,matched,match_distances,nearest,support_status=_current_support(c,context,provisional_survivors)
        if model_path is None:
            import sys
            module=sys.modules.get(type(actual).__module__);root=getattr(module,'ROOT',None)
            if root is None:raise ValueError('model_path required: actual World module has no ROOT')
            model_path=Path(root)/'rust/crates/model/data/getup-v1.json'
        model_path=Path(model_path)
        if not model_path.is_file():raise ValueError('required source model JSON is missing: '+str(model_path))
        model=ModelFK.from_model(json.loads(model_path.read_text()))
        limits=np.array(actual.limits,copy=True);origin=np.array(c.leg_origin,copy=True)
        ranges=np.array(c._mapping.ranges,copy=True);neutral=np.array(c._mapping.neutral_pitch,copy=True)
        if limits.shape!=(21,2) or ranges.shape!=(2,2) or origin.shape!=(2,):raise ValueError('invalid original model/map dimensions')
        active=np.array(context.joints.position);active_com=np.array(context.com.position);side=int(c.stance_side)
        if side not in (0,1):raise ValueError('feet order must be [left,right]')
        ids=tuple(support.surviving_corners);actual_triangle=matched[list(ids)]
        feet=[]
        for f in context.feet:
            p,_,_,R,_,_=f.arrays();feet.append(pose_jet_from_source(Jet.rest(p),R,np.zeros(3),np.zeros(3)).T)
        if len(feet)!=2:raise ValueError('both current active foot references required')
        fixed=feet[side];stance_name=('left_foot','right_foot')[side];free_side=1-side;free_name=('left_foot','right_foot')[free_side]
        bodyrows={o.body:o for o in context.orientations}
        if 1 not in bodyrows:raise ValueError('active body1 thorax reference required')
        base_R=np.array(bodyrows[1].rotation).reshape(3,3)
        base_root=pose_jet_from_source(Jet.rest(np.zeros(3)),base_R,np.zeros(3),np.zeros(3)).T
        baseline=_position_fk(model,active,base_root)
        Rsource=lambda poses,name:P@poses[name][:3,:3]@P.T
        baseline_R={'torso':base_R,'pelvis':(np.array(bodyrows[4].rotation).reshape(3,3) if 4 in bodyrows else Rsource(baseline,'pelvis'))}
        # Preserve initial reference jets, including a causal nonzero correction
        # or an orientation/lower-joint jet already active at this command.
        initial_root_jet=pose_jet_from_source(Jet.rest(np.zeros(3)),base_R,
            bodyrows[1].velocity,bodyrows[1].acceleration)
        initial_q_jet=Jet(context.joints.position,context.joints.velocity,context.joints.acceleration)
        initial_pelvis_pose=pose_jet_to_source(model.forward(initial_q_jet,initial_root_jet)['pelvis'])
        initial_pelvis=(OrientationJet(4,bodyrows[4].rotation,bodyrows[4].velocity,bodyrows[4].acceleration)
            if 4 in bodyrows else OrientationJet(4,initial_pelvis_pose.rotation,
                initial_pelvis_pose.angular_velocity_world,initial_pelvis_pose.angular_acceleration_world))
        initial_sample=CoupledSample(Jet(context.com.position,context.com.velocity,context.com.acceleration),
            Jet(*(x[:15] for x in initial_q_jet.arrays())),
            OrientationJet(1,bodyrows[1].rotation,bodyrows[1].velocity,bodyrows[1].acceleration),initial_pelvis)
        angles={name:chart(R) for name,R in baseline_R.items()}
        pitch_floor=angles['torso'][1]+math.radians(1.);roll_cap=abs(angles['torso'][2])
        tilt_cap=math.acos(float(np.clip(baseline_R['pelvis'][2,2],-1.,1.)))
        lo=np.maximum(limits[:15,0]+.02,active[:15]-.18);hi=np.minimum(limits[:15,1]-.02,active[:15]+.18)
        lo[:3]=np.maximum(lo[:3],active[:3]-math.radians(5.));hi[:3]=np.minimum(hi[:3],active[:3]+math.radians(5.))
        lo[[7,13]]=np.maximum(lo[[7,13]],limits[[7,13],0]+.06);hi[[7,13]]=np.minimum(hi[[7,13]],limits[[7,13],1]-.06)
        if np.any(lo>hi):raise ValueError('current static lower-joint bounds have empty intersection')
        if seed is None:start=np.minimum(np.maximum(active[:15],lo),hi);seed_kind='active-reference bounded static initializer'
        else:
            start=np.array(seed.target.lower_joints.position if isinstance(seed,Candidate) else seed,dtype=float).reshape(-1)
            if len(start)==21:start=start[:15]
            if start.shape!=(15,) or not np.isfinite(start).all() or np.any(start<lo-1e-10) or np.any(start>hi+1e-10):
                raise ValueError('supplied joint seed is outside the single fixed static family bounds')
            seed_kind='explicit joint-only seed'
        sources={str(Path(__file__)):_sha(__file__),str(model_path):_sha(model_path),str(actual.xml):_sha(actual.xml),str(actual.helper):_sha(actual.helper)}
        for obj in (type(actual),type(c),type(c._mapping),type(context)):
            try:p=Path(inspect.getfile(obj));sources[str(p)]=_sha(p)
            except (TypeError,OSError):pass
        bound_input=dict(sourceStep=context.source_step,sourceTimeS=context.source_time,stepIndex=context.step_index,
            phase=context.phase,referenceEpoch=context.epoch,contextSHA256=context.receipt()['contextSHA256'],
            actualStateSHA256=before,support=asdict(support),supportSHA256=support.key,
            supportStatus=support_status,provisionalSupportIsNotAdmittedHistory=not bool(getattr(c,'robust_support',{}).get('selected',False)),
            sourceSHA256=sources,legOriginRightLeftRad=origin.tolist(),originalMapRangesRightLeftRad=ranges.tolist(),
            seedKind=seed_kind,seedLowerJointRad=start.tolist(),activeBaselineJointRad=active.tolist(),
            activeCompleteCOM=context.com.__dict__,activeBodyReferences={str(k):asdict(v) for k,v in bodyrows.items()},
            initialCoupledReference=asdict(initial_sample),
            baselineReconstructedPelvisSource=baseline_R['pelvis'].tolist(),activeFootReferences=[asdict(f) for f in context.feet])
        inputs_sha=digest(_jsonable(bound_input));receipt.update(input=bound_input,inputSHA256=inputs_sha,
            policy=dict(objective='sum((all21q-active-referenceq)^2)+100*sum((horizontalCOM-active-totalCOM)^2)',
                forwardPitchIncreaseDeg=1.,rollNonworsening=True,pelvisTotalTiltNonworsening=True,
                lumbarChangeCapDeg=5.,localLowerJointChangeCapRad=.18,motorCap=MOTOR_CAP,
                strictMotorAdmissionExclusive=.925,supportInsetM=SUPPORT_INSET,cleanGapExclusiveM=.025,
                minimumJointInteriorRad=.02,minimumStaticAnkleReferenceRad=.06,minimumOriginalMapReserveRad=.02,
                matchDistanceM=.001,footPositionToleranceM=1e-6,footOrientationToleranceRad=1e-6),
            supportMatchDistancesM=match_distances.tolist(),supportNearestIndices=list(nearest),
            changedEffectiveTasksRequireFreshRiskHistory=True,riskHistoryAdmissionInherited=False,
            provisionalTriangleMustMatchFreshAdmittedTriangle=True,laterLPMayOnlyCertifyUnchangedCandidate=True,
            automaticReplanningImplemented=False,initialJetsComeFromPreviousEffectiveContext=True,
            measuredLiftRequiresPostureAndCorrectionComplete=True,
            numericalLibraries=dict(numpy=np.__version__,scipy=scipy.__version__))
        def new_scratch():
            w=(scratch_factory(actual) if scratch_factory is not None else type(actual)(actual.base,actual.helper,xml=actual.xml))
            if w is actual:raise ValueError('scratch factory returned the actual world')
            try:
                if w.l.lh_dim(w.h,6)!=0:raise ValueError('scratch world must remain at native counter zero')
                def forbidden(*args,**kwargs):raise AssertionError('native integration forbidden in static retargeting')
                w.l.lh_step_audited=forbidden
                w.l.lh_self_separation.argtypes=[C.c_void_p,C.POINTER(C.c_double)];w.l.lh_self_separation.restype=None
                if (_sha(w.xml)!=sources[str(actual.xml)] or _sha(w.helper)!=sources[str(actual.helper)]
                        or np.max(abs(np.array(w.limits)-limits))>1e-12):
                    raise ValueError('scratch model/helper/limits differ from actual selected model')
                counts['worlds']+=1;return w
            except Exception:
                w.close();raise
        def install(w,q,root):
            if w is actual or w.l.lh_dim(w.h,6)!=0:raise ValueError('state installation restricted to counter-zero scratch')
            quat=Rotation.from_matrix(P@root[:3,:3]@P.T).as_quat()
            w.install_cold(dict(qpos=np.r_[P@root[:3,3],quat[[3,0,1,2]],q].tolist(),qvel=[0.]*27))
            counts['installs']+=1
            if w.l.lh_dim(w.h,6)!=0:raise ValueError('scratch counter changed during static installation')
        def geometry(x):
            counts['geometryCalls']+=1
            relative=_position_fk(model,np.r_[x,active[15:]],np.eye(4));theta=[]
            for name in ('right','left'):
                d=P@(relative[name+'_shin'][:3,3]-relative[name+'_thigh'][:3,3]);theta.append(math.atan2(d[0],-d[2]))
            delta=-(np.array(theta)-origin)
            # The supplied original controller map is the authoritative mapping.
            arm=np.array(c._mapping.evaluate(delta)[0],copy=True);q=np.r_[x,arm]
            relative=_position_fk(model,q,np.eye(4));root=fixed@np.linalg.inv(relative[stance_name])
            poses=_position_fk(model,q,root);com=sum(p.mass*(poses[p.name]@np.r_[p.com,1.])[:3] for p in model.parts)/model.total_mass
            return q,root,poses,P@com,delta
        scratch=new_scratch();B=_actuators(scratch)
        if hasattr(c,'B') and np.max(abs(B-np.array(c.B)))>1e-12:raise ValueError('original actual actuator mapping mismatch')
        cached_x=None;cached=None;trace=[]
        def measured(x):
            nonlocal cached_x,cached
            if cached_x is None or not np.array_equal(cached_x,x):
                q,root,poses,com,delta=geometry(x);install(scratch,q,root)
                foot=scratch.feet()[side];points=np.array(foot['points']);points[:,2]-=foot['radius']
                lp4=_strict_lp(scratch,side,B,points);lp3=_strict_lp(scratch,side,B,actual_triangle);counts['lpCalls']+=2
                raw=scratch.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14)
                cached=(q,root,poses,com,delta,lp4,lp3,float(min(pairs[:,4])));cached_x=np.array(x,copy=True)
            return cached
        def equality(x):
            q,root,poses,com,delta=geometry(x)
            out=[com[2]-active_com[2],*(poses[free_name][:3,3]-feet[free_side][:3,3]),
                *Rotation.from_matrix(poses[free_name][:3,:3]@feet[free_side][:3,:3].T).as_rotvec()]
            for name in ('torso','pelvis'):
                d=chart(Rsource(poses,name))[0]-angles[name][0];out.append(math.atan2(math.sin(d),math.cos(d)))
            return np.array(out)
        def inequality(x):
            q,root,poses,com,delta,lp4,lp3,gap=measured(x)
            motors=np.array([p['requiredMotorFraction'] if p['success'] else 10. for p in (lp4,lp3)])
            mapmargin=np.minimum(delta-ranges[:,0],ranges[:,1]-delta)
            armmargin=np.minimum(q[15:]-limits[15:,0],limits[15:,1]-q[15:])
            tor=chart(Rsource(poses,'torso'));tilt=math.acos(float(np.clip(Rsource(poses,'pelvis')[2,2],-1.,1.)))
            eqs=support.equations()
            return np.r_[MOTOR_CAP-motors,gap-.025001,-(eqs[:,:2]@com[:2]+eqs[:,2])-SUPPORT_INSET,
                mapmargin-.02,armmargin-.02,tor[1]-pitch_floor,roll_cap-abs(tor[2]),tilt_cap-tilt]
        def objective(x):
            q,root,poses,com,delta=geometry(x)
            return float(np.sum((q-active)**2)+100.*np.sum((com[:2]-active_com[:2])**2))
        def callback(x):
            trace.append(dict(iteration=len(trace)+1,objective=objective(x),
                maximumEqualityResidual=float(max(abs(equality(x)))),minimumInequalitySlack=float(min(inequality(x)))))
        fit=minimize(objective,start,method='SLSQP',bounds=list(zip(lo,hi)),
            constraints=[dict(type='eq',fun=equality),dict(type='ineq',fun=inequality)],callback=callback,
            options=dict(maxiter=MAX_ITERATIONS,ftol=1e-11,eps=1e-6,disp=False))
        q,root,poses,com,delta,_,_,_=measured(fit.x)
        eq=equality(fit.x);ineq=inequality(fit.x)
        receipt.update(optimizer=dict(success=bool(fit.success),status=str(fit.message),iterations=int(fit.nit),
            objective=float(fit.fun),trace=trace,maximumEqualityResidual=float(max(abs(eq))),minimumInequalitySlack=float(min(ineq))))
        # A NEW static scratch world independently rechecks the terminal posture.
        scratch.close();scratch=None;audit=new_scratch();install(audit,q,root);B2=_actuators(audit)
        foot=audit.feet()[side];points=np.array(foot['points']);points[:,2]-=foot['radius']
        lps=dict(fourPoint=_strict_lp(audit,side,B2,points),selectedCandidateCorners=_strict_lp(audit,side,B2,points[list(ids)]),
            currentActualTriangle=_strict_lp(audit,side,B2,actual_triangle));counts['lpCalls']+=3
        native_com=audit.arr(6,(17,3))[1].copy();R1=audit.arr(5,(17,3,3))[1].copy();R4=audit.arr(5,(17,3,3))[4].copy()
        raw=audit.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14);gap=float(min(pairs[:,4]))
        legs=_native_limb_pitch(audit,'leg');arms=_native_limb_pitch(audit,'arm');native_delta=-(legs-origin)
        identity=float(max(abs(q[15:]-c._mapping.evaluate(native_delta)[0])))
        arm_fk=float(max(abs(arms-neutral-native_delta)))
        mapmargin=np.minimum(native_delta-ranges[:,0],ranges[:,1]-native_delta)
        margins=np.minimum(q-limits[:,0],limits[:,1]-q);pose_errors=[]
        for i,name in enumerate(('left_foot','right_foot')):
            body=audit.ids[name];p=audit.arr(3,(17,3))[body];R=audit.arr(5,(17,3,3))[body]
            pose_errors.append(dict(side=i,positionM=float(np.linalg.norm(p-P@feet[i][:3,3])),
                orientationRad=float(Rotation.from_matrix(R@(P@feet[i][:3,:3]@P.T).T).magnitude())))
        clearances=[z['distanceM']-.2-foot['radius'] for z in foot['groundRays']]
        # Independently reconstruct full joint FK/COM with the ORIGINAL map result.
        independent=model.forward(Jet.rest(q),MatrixJet.rest(root))
        fk_com=sum(p.mass*(independent[p.name].T@np.r_[p.com,1.])[:3] for p in model.parts)/model.total_mass
        com_error=float(max(abs(P@fk_com-native_com)))
        target_angles=[chart(R) for R in (R1,R4)];tilt=math.acos(float(np.clip(R4[2,2],-1.,1.)))
        headings=[math.atan2(math.sin(a[0]-angles[name][0]),math.cos(a[0]-angles[name][0])) for a,name in zip(target_angles,('torso','pelvis'))]
        equations=support.equations();margin=float(min(-(equations[:,:2]@native_com[:2]+equations[:,2])))
        checks=dict(optimizerConverged=bool(fit.success),equality=float(max(abs(eq)))<=1e-6,
            inequalities=float(min(ineq))>=-1e-8,feet=all(p['positionM']<=1e-6 and p['orientationRad']<=1e-6 for p in pose_errors),
            strictOriginalLPs=all(p['success'] and p['requiredMotorFraction']<.925 for p in lps.values()),
            preferredMotorCap=all(p['success'] and p['requiredMotorFraction']<=MOTOR_CAP+1e-9 for p in lps.values()),
            lpResiduals=all(p['success'] and p['maximumEqualityResidual']<=1e-8 and p['maximumFrictionViolation']<=1e-8 and p['minimumNormalN']>=-1e-8 for p in lps.values()),
            cleanGap=gap>.025,sourceInterior=bool(min(margins)>=.02-1e-9),referenceAnkles=bool(min(margins[[7,13]])>=.06-1e-9),
            originalMapReserve=bool(min(mapmargin)>=.02-1e-9),exactFutureArmIdentity=identity<1e-12,
            independentArmFK=arm_fk<1e-8,independentCOM=com_error<1e-10,currentSupport4mm=margin>=.004-1e-9,
            headings=max(abs(x) for x in headings)<=1e-8,lumbar=bool(max(abs(q[:3]-active[:3]))<=math.radians(5.)+1e-9),
            forward1degree=target_angles[0][1]>=pitch_floor-1e-8,rollNonworsened=abs(target_angles[0][2])<=roll_cap+1e-8,
            pelvisTiltNonworsened=tilt<=tilt_cap+1e-8,stanceContact=all(-.003<=x<=.003 for x in clearances),
            scratchCounterZero=audit.l.lh_dim(audit.h,6)==0)
        path=_com_path_receipt(context,native_com,support)
        unchanged=_actual_snapshot(c,context)==before;checks['actualInputAndHistoryUnchanged']=unchanged
        accepted=all(checks.values()) and path['frozenEnvelopePassed'] and path['wholeCurrent3mmGatePassed']
        target=CoupledSample(Jet.rest(native_com),Jet.rest(q[:15]),OrientationJet.rest(1,R1),OrientationJet.rest(4,R4))
        candidate=Candidate(f'current-feet-forward1deg-step{context.step_index}',inputs_sha,target)
        receipt.update(accepted=accepted,checks=checks,staticEndpointAdmitted=all(checks.values()),
            rejectionReasons=[name for name,passed in checks.items() if not passed]+(
                [] if path['frozenEnvelopePassed'] and path['wholeCurrent3mmGatePassed'] else ['declared COM handoff fails unchanged envelope/current3mm gate']),
            candidate=asdict(candidate),candidateKey=candidate.key,fullFutureMappedJointReferenceRad=q.tolist(),
            referenceAnkleMarginsRightLeftRad=margins[[7,13]].tolist(),mapMarginsRightLeftRad=mapmargin.tolist(),
            minimumCleanSelfGapM=gap,currentSupportEndpointMarginM=margin,
            futureArmIdentityErrorRad=identity,independentNativeArmFKErrorRad=arm_fk,independentCOMErrorM=com_error,
            feetErrors=pose_errors,strictLPs=lps,comHandoff=path,
            inheritedInitialReferenceAnkleMarginsRightLeftRad=np.minimum(active[[7,13]]-limits[[7,13],0],limits[[7,13],1]-active[[7,13]]).tolist(),
            genericWholePath06GuardUnchanged=True,actualInputAndHistoryUnchanged=unchanged,counts=counts)
        return RetargetResult(accepted,candidate if accepted else None,json.dumps(_jsonable(receipt),sort_keys=True,separators=(',',':'),allow_nan=False))
    except Exception as exc:
        receipt.update(rejectionReasons=[type(exc).__name__+': '+str(exc)],counts=counts)
        if before is not None:
            try:receipt['actualInputAndHistoryUnchanged']=_actual_snapshot(c,context)==before
            except Exception:receipt['actualInputAndHistoryUnchanged']=False
        return RetargetResult(False,None,json.dumps(_jsonable(receipt),sort_keys=True,separators=(',',':'),allow_nan=False))
    finally:
        for w in (scratch,audit):
            if w is not None and w is not actual:w.close()
