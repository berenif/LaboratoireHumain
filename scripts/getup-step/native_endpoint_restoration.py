"""One bounded stationary-endpoint restoration, never a runtime gait proposal.

The pose family is the existing27 stance-realized coordinates. Twenty-seven
normalized contact-force auxiliaries witness native gravity/passive equilibrium
at its three stationary endpoints. They have ZERO objective weight. Native
states are installed only into an explicitly supplied counter-zero scratch.
"""
import copy,math,time,traceback
from collections import OrderedDict
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from coupled_posture_reference import Jet,digest
from coupled_posture_kinematics import P
from wholebody_cycle_reference import PHASES
from wholebody_feasibility_polish import AnalyticPolishEvaluator
from strict_reference_cache import StrictReferenceCache

VERSION='native-stationary-endpoint-restoration-v1'
MOTOR_CAP=.925
PLANNING_NUMERICAL_INTERIOR=1e-7
OPTIMIZER_MOTOR_CAP=MOTOR_CAP-PLANNING_NUMERICAL_INTERIOR
FRICTION=.6

class ComputeCap(RuntimeError):pass

def stationary_sample_guard(sample,tolerance=1e-8):
    """No zeroing moving commands or certifying undeclared compatibility slack."""
    g=sample.geometric;objects=[('lower',sample.lower_joints),('full',g.full_joints),('commandCOM',sample.com),
        ('geometricCOM',g.com_source),('root',g.root_model),('body1Command',sample.thorax),('body4Command',sample.pelvis),
        ('body1Geometric',g.thorax),('body4Geometric',g.pelvis),('futureThigh',g.future_thigh_right_left),('futureArmDelta',g.future_arm_delta_right_left)]
    objects.extend(('commandFoot'+str(i),pose)for i,pose in enumerate(sample.feet))
    objects.extend(('geometricPose:'+name,pose)for name,pose in g.poses_model)
    maxima={}
    for name,obj in objects:
        values=[np.asarray(obj.velocity,float),np.asarray(obj.acceleration,float)]
        if not all(np.isfinite(a).all()for a in values):raise ValueError('nonfinite stationary sample jets')
        maxima[name]=max(float(np.max(abs(a)))for a in values)
    if max(maxima.values())>tolerance:raise ValueError('moving sample cannot be silently made stationary')
    residuals=dict(lowerFull=float(max(abs(np.asarray(sample.lower_joints.position)-np.asarray(g.full_joints.position)[:15]))),
        commandGeometricCOM=float(max(abs(np.asarray(sample.com.position)-np.asarray(g.com_source.position)))),
        body1=float(np.max(abs(sample.thorax.arrays()[0]-g.thorax.arrays()[0]))),
        body4=float(np.max(abs(sample.pelvis.arrays()[0]-g.pelvis.arrays()[0]))))
    for i,name in enumerate(('left_foot','right_foot')):residuals[name]=float(np.max(abs(sample.feet[i].T-g.pose(name).T)))
    if not all(math.isfinite(v)and v<=tolerance for v in residuals.values()):raise ValueError('stationary command/geometric compatibility parity failed')
    return dict(stationarityTolerance=tolerance,maximumStationaryJetRoundoff=max(maxima.values()),
        stationaryJetMaxima=maxima,commandGeometricParityTolerance=tolerance,commandGeometricParityResiduals=residuals)

def stationary_terms(B,G,bias,force_scale):
    """Exact Schur elimination; root actuator rows are retained, not assumed0."""
    B=np.asarray(B,float);G=np.asarray(G,float);bias=np.asarray(bias,float);W=float(force_scale)
    if(B.shape!=(27,21)or G.shape!=(27,9)or bias.shape!=(27,)or not all(np.isfinite(x).all()for x in(B,G,bias))
            or not math.isfinite(W)or W<=0.):raise ValueError('finite native27/21/9 equilibrium data required')
    u0=np.linalg.solve(B[6:],bias[6:]);K=np.linalg.solve(B[6:],G[6:]*W)
    r0=(B[:6]@u0-bias[:6])/W;R=G[:6]-B[:6]@K/W
    return dict(u0=u0,K=K,r0=r0,R=R,forceScaleN=W)

def endpoint_constraints(record,forces):
    f=np.asarray(forces,float).reshape(9);u=record['u0']-record['K']@f
    friction=[]
    for x,y,z in f.reshape(3,3):friction.extend((FRICTION*z-x,FRICTION*z+x,FRICTION*z-y,FRICTION*z+y))
    inequality=np.r_[record['kinematic'],OPTIMIZER_MOTOR_CAP-u,OPTIMIZER_MOTOR_CAP+u,friction,record['minimumCleanSelfGapM']-.025,record['commandCOMHeightM']-.65]
    return inequality,record['r0']+record['R']@f,u

def force_jacobians(record):
    n=len(record['kinematic']);J=np.zeros((n+56,9));J[n:n+21]=record['K'];J[n+21:n+42]=-record['K']
    for i in range(3):
        J[n+42+4*i:n+46+4*i,3*i:3*i+3]=[[-1,0,FRICTION],[1,0,FRICTION],[0,-1,FRICTION],[0,1,FRICTION]]
    return J,record['R']

class NativeStationaryScratch:
    """Original native data/strict LP on caller-owned counter-zero scratch only."""
    def __init__(self,world,contacts):
        from source_step_probe import ptr
        from source_posture_retarget import _actuators
        from wholebody_static_admission import strict_contact_lp
        self.w=world;self.ptr=ptr;self.actuators=_actuators;self.strict_lp=strict_contact_lp
        self.contacts=tuple((int(side),tuple(np.asarray(point,float)))for side,point in contacts)
        if len(self.contacts)!=3 or any(side not in(0,1)or len(p)!=3 or not np.isfinite(p).all()for side,p in self.contacts):
            raise ValueError('exactly3 actual retained stance contacts required')
        if len({s for s,_ in self.contacts})!=1:raise ValueError('same actual stance foot required')
        self.force_scale=float(world.mass*9.81);self.calls=0
        self._counter_zero()
    def _counter_zero(self):
        if self.w.l.lh_dim(self.w.h,6)!=0:raise ValueError('native endpoint provider requires counter-zero scratch')
    def evaluate(self,sample,*,final=False):
        import ctypes as C
        w=self.w;self._counter_zero();stationary=stationary_sample_guard(sample)
        g=sample.geometric;q=np.asarray(g.full_joints.position);root=g.root_model.T
        if q.shape!=(21,)or not np.isfinite(q).all():raise ValueError('finite source full21 geometry required')
        quaternion=Rotation.from_matrix(P@root[:3,:3]@P.T).as_quat()
        w.install_cold(dict(qpos=np.r_[P@root[:3,3],quaternion[[3,0,1,2]],q].tolist(),qvel=[0.]*27));self.calls+=1
        native_velocity=np.asarray(w.arr(1,(27,)))
        if not np.isfinite(native_velocity).all()or max(abs(native_velocity))>1e-8:
            raise ValueError('native endpoint scratch is not stationary after cold installation')
        B=self.actuators(w);feet=w.feet()
        G=np.hstack([w.read('lh_point_jac',(3,27),feet[side]['body'],self.ptr(np.ascontiguousarray(point))).T for side,point in self.contacts])
        bias=w.read('lh_bias',(27,)) # ORIGINAL qfrc_bias minus qfrc_passive.
        native_com=w.arr(6,(17,3))[1].copy();positions=w.arr(3,(17,3));rotations=w.arr(5,(17,3,3))
        if not all(np.isfinite(a).all()for a in(native_com,positions,rotations)):raise ValueError('nonfinite native stationary geometry')
        position_error=0.;rotation_error=0.;foot_error=0.;bodies=0
        for name,pose in g.poses_model:
            body=w.l.lh_id(w.h,1,name.encode())
            if body<1:
                if not name.startswith('@'):raise ValueError('unmapped physical body '+name)
                continue
            T=pose.T;pe=float(max(abs(positions[body]-P@T[:3,3])));re=float(np.max(abs(rotations[body]-P@T[:3,:3]@P.T)))
            position_error=max(position_error,pe);rotation_error=max(rotation_error,re);bodies+=1
            if name in('left_foot','right_foot'):foot_error=max(foot_error,pe,re)
        com_error=float(max(abs(native_com-np.asarray(g.com_source.position))))
        w.l.lh_self_separation.argtypes=[C.c_void_p,C.POINTER(C.c_double)];w.l.lh_self_separation.restype=None
        raw=w.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14)
        if len(pairs)==0:raise ValueError('original clean self-gap pairs absent')
        gap=float(min(pairs[:,4]));self._counter_zero()
        if max(position_error,rotation_error,com_error)>1e-8:raise ValueError('source/native stationary body/foot/COM parity failed')
        receipt=dict(nativeCounter=0,stationaryQvelZero=True,maximumNativeGeneralizedVelocity=float(max(abs(native_velocity))),nativeBiasIncludesPassive=True,**stationary,
            bodyCount=bodies,maximumBodyPositionErrorM=position_error,maximumBodyRotationMatrixError=rotation_error,
            maximumFootPoseError=foot_error,nativeCOMErrorM=com_error,nativeCOMHeightM=float(native_com[2]),minimumCleanSelfGapM=gap)
        if final:receipt['originalStrictLP']=self.strict_lp(w,self.contacts)
        return stationary_terms(B,G,bias,self.force_scale)|dict(minimumCleanSelfGapM=gap,native=receipt)

class StanceEndpointProvider:
    """Existing exact FK/IK only; endpoint k depends on ONLY its9 coordinates."""
    def __init__(self,realizer,context,limits,heading,support,initial_knots,native,*,shape_bounds='original-experimental'):
        if shape_bounds not in('original-experimental','source-physical'):raise ValueError('declared single shape-bound profile required')
        self.kin=AnalyticPolishEvaluator(realizer,context,limits,heading,support,initial_knots)
        self.initial=np.asarray(initial_knots,float).reshape(3,9).copy();self.native=native;self.sampler=StrictReferenceCache()
        self.shape_bounds=shape_bounds;self.limits=np.asarray(limits,float);self.q0=np.asarray(context.joints.position)[:15]
        if shape_bounds=='source-physical':
            self.kin.lo[:3]=self.limits[:3,0]+.01;self.kin.hi[:3]=self.limits[:3,1]-.01
            self.kin.bounds=list(zip(np.tile(self.kin.lo[self.kin.idx],3),np.tile(self.kin.hi[self.kin.idx],3)))
        self.bounds=tuple(self.kin.bounds)
    def __call__(self,k,independent9,*,strict=False):
        knots=self.initial.copy();knots[k]=independent9;path=self.kin.path(knots,strict_map=strict)
        sample=self.sampler.sample(path,PHASES[k],path.durations[k]);block=self.kin.constraint_block(sample)
        if not np.isfinite(block).all():raise ValueError('finite genuine endpoint geometry required')
        record=self.native.evaluate(sample,final=strict);q=np.asarray(sample.lower_joints.position)
        selected_slack=np.minimum(q-self.kin.lo,self.kin.hi-q)
        physical_slack=np.minimum(q-self.limits[:15,0]-.01,self.limits[:15,1]-.01-q)
        record.update(kinematic=block,phase=PHASES[k],maximumPoseResidual=sample.diagnostics['maximumPoseResidual'],
            lowerJointsRad=q.tolist(),commandCOM=sample.com.position,commandCOMHeightM=float(sample.com.position[2]),fullJointsRad=sample.geometric.full_joints.position,
            experimentalLumbarDeltaDeg=np.rad2deg(q[:3]-self.q0[:3]).tolist(),
            originalExperimentalLumbarSlackRad=(math.radians(5.)-abs(q[:3]-self.q0[:3])).tolist(),
            sourcePhysicalJointMinimumMarginRad=float(min(np.minimum(q-self.limits[:15,0],self.limits[:15,1]-q))),
            activeSelectedBoundJointIndices=np.flatnonzero(selected_slack<=1e-7).tolist(),
            activePhysicalInteriorBoundJointIndices=np.flatnonzero(physical_slack<=1e-7).tolist(),
            noHyperextensionSlackRad=float(-max(q[[6,12]])),strictOriginalMapUsed=bool(strict))
        if strict:
            original=path.validate_sample(sample,self.kin.support,require_clearance=k==1)
            selected=original['accepted']if self.shape_bounds=='original-experimental'else all(v for name,v in original['checks'].items()if name!='lumbar')
            record.update(originalGeometryCheck=original,selectedProfileGeometryAccepted=bool(selected))
        return record

class EndpointForceProblem:
    """54 variables; exact9-coordinate endpoint caches,27 FD +27 LINEAR columns."""
    def __init__(self,endpoint_provider,anchor_knots,initial_knots,initial_normalized_forces,*,binding=None):
        self.provider=endpoint_provider;self.anchor=np.asarray(anchor_knots,float).reshape(27).copy()
        pose=np.asarray(initial_knots,float).reshape(27);forces=np.asarray(initial_normalized_forces,float).reshape(27)
        self.initial=np.r_[pose,forces];self.binding=copy.deepcopy(binding or{});self.check_control=lambda:None
        self.binding.update(originalStrictMotorCap=MOTOR_CAP,optimizerMotorCap=OPTIMIZER_MOTOR_CAP,planningNumericalInterior=PLANNING_NUMERICAL_INTERIOR)
        mode=getattr(endpoint_provider,'shape_bounds',None)
        if mode is not None and self.binding.get('shapeBounds')!=mode:raise ValueError('declared/actual endpoint shape-bound profile differs')
        if not np.isfinite(self.initial).all()or not np.isfinite(self.anchor).all():raise ValueError('finite exact sole initial/anchor/force seeds required')
        force_bounds=[(None,None),(None,None),(0.,1.)]*9
        self.bounds=tuple(endpoint_provider.bounds)+tuple(force_bounds)
        if len(self.bounds)!=54:raise ValueError('27 pose plus27 normalized-force bounds required')
        for value,(lo,hi)in zip(self.initial,self.bounds):
            if(lo is not None and value<lo)or(hi is not None and value>hi):raise ValueError('sole initial candidate outside declared bounds; no clipping')
        self.node_cache=OrderedDict();self.candidate_cache=OrderedDict();self.jac_cache=OrderedDict()
        self.stats=dict(endpointHits=0,endpointMisses=0,endpointEvictions=0,candidateHits=0,candidateMisses=0,
            nativePathFDColumns=0,analyticForceColumns=0,jacobianHits=0)
    def endpoint(self,k,q9,*,strict=False):
        self.check_control();key=(k,np.asarray(q9,dtype=np.float64).tobytes())
        if not strict and key in self.node_cache:
            self.stats['endpointHits']+=1;self.node_cache.move_to_end(key);return self.node_cache[key]
        record=self.provider(k,np.asarray(q9,float),strict=strict);self.check_control()
        for name in('u0','K','r0','R','kinematic'):
            a=np.asarray(record[name]);
            if not np.isfinite(a).all():raise ValueError('nonfinite endpoint/native data')
        if not strict:
            self.stats['endpointMisses']+=1;self.node_cache[key]=record
            if len(self.node_cache)>256:self.node_cache.popitem(last=False);self.stats['endpointEvictions']+=1
        return record
    def objective(self,z):
        self.check_control();d=np.asarray(z)[:27]-self.anchor;return .5*float(d@d)
    def objective_jacobian(self,z):return np.r_[np.asarray(z)[:27]-self.anchor,np.zeros(27)]
    def evaluate(self,z):
        self.check_control();z=np.asarray(z,float)
        if z.shape!=(54,)or not np.isfinite(z).all():raise ValueError('finite54-coordinate decision required')
        key=z.tobytes()
        if key in self.candidate_cache:self.stats['candidateHits']+=1;return self.candidate_cache[key]
        self.stats['candidateMisses']+=1;ineq=[];eq=[];rows=[]
        for k in range(3):
            record=self.endpoint(k,z[9*k:9*k+9]);c,r,u=endpoint_constraints(record,z[27+9*k:36+9*k]);ineq.extend(c);eq.extend(r)
            rows.append(dict(phase=PHASES[k],minimumInequalitySlack=float(min(c)),maximumNormalizedBaseBalanceResidual=float(max(abs(r))),
                maximumMotorFraction=float(max(abs(u))),motorCommands=u.tolist(),normalizedForces=z[27+9*k:36+9*k].reshape(3,3).tolist(),
                experimentalLumbarDeltaDeg=record.get('experimentalLumbarDeltaDeg'),native=record.get('native',{})))
        result=(np.asarray(ineq),np.asarray(eq),rows);self.candidate_cache[key]=result
        if len(self.candidate_cache)>128:self.candidate_cache.popitem(last=False)
        return result
    def jacobians(self,z):
        self.check_control();z=np.asarray(z,float);key=z.tobytes()
        if key in self.jac_cache:self.stats['jacobianHits']+=1;return self.jac_cache[key]
        baseline,baseeq,_=self.evaluate(z);n=len(baseline)//3;J=np.zeros((len(baseline),54));E=np.zeros((18,54))
        for k in range(3):
            record=self.endpoint(k,z[9*k:9*k+9]);A,R=force_jacobians(record)
            J[k*n:(k+1)*n,27+9*k:36+9*k]=A;E[6*k:6*k+6,27+9*k:36+9*k]=R
        self.stats['analyticForceColumns']+=27
        for j,(lo,hi)in enumerate(self.bounds[:27]):
            self.check_control();h=1e-6 if hi is None or z[j]+1e-6<=hi else-1e-6
            if(lo is not None and z[j]+h<lo)or(hi is not None and z[j]+h>hi):raise ValueError('no finite-difference interior inside original bounds')
            k=j//9;q=z[9*k:9*k+9].copy();q[j%9]+=h;actual=q[j%9]-z[j]
            shifted=self.endpoint(k,q);c,r,_=endpoint_constraints(shifted,z[27+9*k:36+9*k])
            J[k*n:(k+1)*n,j]=(c-baseline[k*n:(k+1)*n])/actual;E[6*k:6*k+6,j]=(r-baseeq[6*k:6*k+6])/actual
            self.stats['nativePathFDColumns']+=1
        self.jac_cache[key]=(J,E)
        if len(self.jac_cache)>32:self.jac_cache.popitem(last=False)
        return J,E

def restore_endpoints(problem,*,max_wall_seconds=180.,max_iterations=80,progress_callback=None):
    """Exactly one bounded solve; ALL results are endpoint-only and UNADMITTED."""
    if not math.isfinite(max_wall_seconds)or not 0.<max_wall_seconds<=180.:raise ValueError('wall cap in(0,180] required')
    if type(max_iterations)is not int or not 1<=max_iterations<=80:raise ValueError('maximum iterations in1..80 required')
    if progress_callback is not None and not callable(progress_callback):raise ValueError('callable progress callback required')
    start=time.monotonic();trace=[];last_major=None;fit=None;failure=None;termination=None
    def control():
        if time.monotonic()-start>=max_wall_seconds:raise ComputeCap('endpoint-restoration-compute-cap')
    problem.check_control=control
    def callback(z):
        nonlocal last_major
        control();c,e,rows=problem.evaluate(z);last_major=np.array(z,copy=True)
        event=dict(iteration=len(trace)+1,objective=problem.objective(z),minimumInequalitySlack=float(min(c)),
            maximumNormalizedBaseBalanceResidual=float(max(abs(e))),unadmittedIndependentKnotsRad=z[:27].reshape(3,9).tolist(),
            unadmittedNormalizedForces=z[27:].reshape(3,3,3).tolist(),candidateSHA256=digest(z[:27].reshape(3,9).tolist()),
            cacheCounters=dict(problem.stats),endpointRows=rows,nativeAdmission=False,wholePathProven=False)
        trace.append(event)
        if progress_callback is not None:progress_callback(copy.deepcopy(event))
    try:
        fit=minimize(problem.objective,problem.initial.copy(),method='SLSQP',jac=problem.objective_jacobian,bounds=problem.bounds,
            constraints=[dict(type='ineq',fun=lambda z:problem.evaluate(z)[0],jac=lambda z:problem.jacobians(z)[0]),
                dict(type='eq',fun=lambda z:problem.evaluate(z)[1],jac=lambda z:problem.jacobians(z)[1])],callback=callback,
            options=dict(maxiter=max_iterations,ftol=1e-12,eps=1e-6,disp=False));control()
    except ComputeCap as exc:termination='compute-cap';failure=dict(type=type(exc).__name__,message=str(exc))
    except Exception as exc:termination='evaluation-or-optimizer-error';failure=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc())
    final=np.asarray(fit.x,float)if fit is not None else(last_major if last_major is not None else problem.initial.copy())
    final_rows=[]
    for k in range(3):
        try:
            record=problem.endpoint(k,final[9*k:9*k+9],strict=True);lp=record['native']['originalStrictLP']
            ok=bool(record.get('selectedProfileGeometryAccepted')is True and record['minimumCleanSelfGapM']>.025
                and record['commandCOMHeightM']>=.65 and record['native']['nativeCOMHeightM']>=.65 and lp['success']
                and lp['requiredMotorFraction']<MOTOR_CAP and lp['maximumEqualityResidual']<1e-8 and lp['maximumFrictionViolation']<1e-8)
            final_rows.append(dict(phase=PHASES[k],accepted=ok,originalStrictMapUsed=True,native=record['native'],
                minimumKinematicSlack=float(min(record['kinematic'])),lowerJointsRad=record.get('lowerJointsRad'),
                experimentalLumbarDeltaDeg=record.get('experimentalLumbarDeltaDeg'),originalExperimentalLumbarSlackRad=record.get('originalExperimentalLumbarSlackRad'),
                sourcePhysicalJointMinimumMarginRad=record.get('sourcePhysicalJointMinimumMarginRad'),noHyperextensionSlackRad=record.get('noHyperextensionSlackRad')))
            final_rows[-1].update(activeSelectedBoundJointIndices=record.get('activeSelectedBoundJointIndices'),
                activePhysicalInteriorBoundJointIndices=record.get('activePhysicalInteriorBoundJointIndices'),
                originalGeometryCheck=record.get('originalGeometryCheck'),selectedProfileGeometryAccepted=record.get('selectedProfileGeometryAccepted'))
        except ComputeCap as exc:termination=termination or'compute-cap';final_rows.append(dict(phase=PHASES[k],accepted=False,error=str(exc)))
        except Exception as exc:final_rows.append(dict(phase=PHASES[k],accepted=False,errorType=type(exc).__name__,error=str(exc)))
    success=bool(fit is not None and fit.success and termination is None)
    return dict(schema=1,version=VERSION,binding=problem.binding,optimizerCalls=1,optimizerSuccess=success,optimizerException=failure,
        optimizerMessage=None if fit is None else str(fit.message),completedMajorIterations=len(trace),progressRecords=trace,
        terminalState=termination or('endpoint-feasible'if success and all(r['accepted']for r in final_rows)else'endpoint-rejected'),
        unadmittedIndependentKnotsRad=final[:27].reshape(3,9).tolist(),finalCandidateSHA256=digest(final[:27].reshape(3,9).tolist()),
        unadmittedNormalizedForces=final[27:].reshape(3,3,3).tolist(),objectiveAnchorSHA256=digest(problem.anchor.reshape(3,9).tolist()),
        maximumCorrectionFromAnchorRad=float(max(abs(final[:27]-problem.anchor))),finalEndpointChecks=final_rows,
        endpointFeasible=bool(success and len(final_rows)==3 and all(r['accepted']for r in final_rows)),cacheCounters=dict(problem.stats),
        elapsedWallS=time.monotonic()-start,maxWallSeconds=max_wall_seconds,maxIterations=max_iterations,
        shapeBounds=problem.binding.get('shapeBounds'),variables=dict(path=27,normalizedContactForces=27,total=54),
        forceObjectiveWeight=0.,noSeedOrWeightScan=True,accepted=False,nativeAdmission=False,wholeCycleCertificate=False,
        originalStrictMotorCap=MOTOR_CAP,optimizerMotorCap=OPTIMIZER_MOTOR_CAP,planningNumericalInterior=PLANNING_NUMERICAL_INTERIOR,
        originalPhysicalCOMHeightFloorM=.65,
        physicsIntegrations=0,actualWorldStateWrites=0,onlyStationaryEndpointsChecked=True,dynamicPathFeasibilityProven=False)
