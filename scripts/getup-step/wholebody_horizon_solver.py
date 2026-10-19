"""Portable one-call free-COM horizon solver, with analytic finite admission.

The sole initializer repeats the qualified stationary independent coordinates.
One SLSQP horizon solve is permitted; no fallback or goal/weight scan exists. Its finite-difference COM prescreen is never an admission receipt.
Independent analytic command jets admit/reject the retained final proposal.
"""
import json,math,time,traceback,copy
from collections import OrderedDict
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from wholebody_cycle_reference import (WholeBodyCycle,Jet,MatrixJet,PHASES,FEET,LEG_INDICES,
    P,pose_jet_to_source,MAX_TOTAL_COM_SPEED,MAX_TOTAL_COM_ACCELERATION,VERSION)
from coupled_posture_reference import digest,rotation,C2Segment

MAX_ITERATIONS=80
FRACTIONS=(0.,.25,.5,.75,1.)
ANALYTIC_FRACTIONS=tuple(np.linspace(0.,1.,41))

class _ControlStop(RuntimeError):
    def __init__(self,kind,message):super().__init__(message);self.kind=kind


def solve_horizon(realizer,current_context,source_limits,heading,current_support,*,
        design_only=False,entry_elapsed=0.,input_binding=None,max_iterations=80,
        analytic_fractions=ANALYTIC_FRACTIONS,max_wall_seconds=1200.,cancellation_check=None,progress_callback=None):
    """Run exactly one bounded solve and retain its complete finite admission.

    Caller owns exclusive attempt bookkeeping and must freeze the returned path;
    a path is diagnostic unless finiteSampledKinematicPass and correction check
    both pass. Even that is not whole-path/motor/physical/native admission.
    """
    if not math.isfinite(max_wall_seconds)or not 0.<max_wall_seconds<=1200.:
        raise ValueError('cooperative wall ceiling must be positive and at most1200seconds')
    if cancellation_check is not None and not callable(cancellation_check):
        raise ValueError('optional cancellation check must be callable')
    if progress_callback is not None and not callable(progress_callback):
        raise ValueError('optional progress callback must be callable')
    if type(max_iterations)is not int or not 1<=max_iterations<=80:
        raise ValueError('fixed bounded maximum iterations in1..80 required')
    if not analytic_fractions or any(not 0.<=float(f)<=1. for f in analytic_fractions):
        raise ValueError('declared finite analytic fractions required')
    context=current_context;r=realizer;limits=np.asarray(source_limits,float);support=current_support
    support.validate_identity(context)
    binding=input_binding or dict(contextSHA256=context.receipt()['contextSHA256'])
    heading=np.asarray(heading,float)
    if not design_only and context.phase!='transfer':raise ValueError('current stationary transfer context required')
    q0=np.array(context.joints.position)[:15];idx=r.independent_indices;stanceidx=np.array(LEG_INDICES[r.stance_side])
    lo=limits[:15,0]+.01;hi=limits[:15,1]-.01
    lo[:3]=np.maximum(lo[:3],q0[:3]-math.radians(5.));hi[:3]=np.minimum(hi[:3],q0[:3]+math.radians(5.))
    hi[[6,12]]=np.minimum(hi[[6,12]],0.)
    initial=np.tile(q0[idx],(3,1))
    initializer=[dict(phase=phase,independentRad=q0[idx].tolist(),qualifiedStationaryReference=True)for phase in PHASES]
    x0=initial.reshape(-1)
    body=next(o for o in context.orientations if o.body==1)
    heading_angle=math.atan2(heading[1],heading[0]);target_body=rotation([heading_angle,math.radians(4.),0.])
    target_knee=(q0[(12,6)[r.swing_side]]-math.radians(10.),q0[(12,6)[r.swing_side]]-math.radians(5.),q0[(12,6)[r.swing_side]])
    nominal=np.tile(q0,(3,1));nominal[:,(12,6)[r.swing_side]]=target_knee
    # One immutable entry template: only independent C2 segments and the key
    # depend on optimizer coordinates. All entry FK/slacks/foot clocks are fixed.
    entry_template=WholeBodyCycle(r,context,limits,heading,initial,
        design_only=design_only,entry_elapsed=entry_elapsed)
    key_base=dict(version=VERSION,context=context.receipt()['contextSHA256'],
        stance=r.stance_pose.position,limits=entry_template.limits.tolist(),entryElapsed=entry_elapsed)
    node_cache=OrderedDict();node_cache_capacity=4096
    candidate_cache=OrderedDict();candidate_cache_capacity=128
    cache_stats=dict(entryTemplates=1,segmentOnlyRebuilds=0,nodeHits=0,nodeMisses=0,nodeEvictions=0,exactLastEvaluationHits=0,
        fullCandidateHits=0,fullCandidateMisses=0,fullCandidateEvictions=0)
    def path_from_knots(knots):
        path=copy.copy(entry_template);knots=np.array(knots,float,copy=True)
        knots.setflags(write=False);path.independent_knots=knots
        points=[np.array(path.lower0.position)[idx],*knots]
        path.independent_segments=tuple(C2Segment.between(0.,T,Jet.rest(a),Jet.rest(b))for a,b,T in zip(points,points[1:],path.durations))
        path.key=digest(dict(key_base,knots=knots.tolist()));cache_stats['segmentOnlyRebuilds']+=1
        return path
    cached_x=None;cached=None;evaluations=0;exceptions=[]
    last_candidate=None;control_stop=None
    def check_control():
        nonlocal control_stop
        if cancellation_check is not None and cancellation_check():
            control_stop=dict(kind='caller-cancelled',reason='caller cancellation check requested stop')
            raise _ControlStop(control_stop['kind'],control_stop['reason'])
        if time.monotonic()-start>=max_wall_seconds:
            control_stop=dict(kind='compute-cap',reason='cooperative wall compute ceiling reached')
            raise _ControlStop(control_stop['kind'],control_stop['reason'])

    def fast_geometry(path,k,t):
        check_control()
        # Cache key is the complete exact positional input for this pure node:
        # phase, time and independent9. Root/feet/slacks/map/seed are immutable
        # closure inputs. No previous IK answer ever becomes a future seed.
        independent=np.array(path.independent_segments[k].at(t).position)
        node_key=(int(k),float(t),tuple(float(x)for x in independent))
        if node_key in node_cache:
            cache_stats['nodeHits']+=1;node_cache.move_to_end(node_key)
            return node_cache[node_key]
        cache_stats['nodeMisses']+=1
        # Existing positional FK and original archived map, no surrogate model.
        T=path.command_foot(r.swing_side,k,t,sum(path.durations[:k])+t)
        tr=pose_jet_to_source(T);total=sum(path.durations[:k])+t
        st,sr=path.foot_slacks[r.swing_side]
        from wholebody_cycle_reference import compose_pose,subtract,pose_chart
        target=compose_pose(subtract(tr.translation,st.at(total)),subtract(pose_chart(T),sr.at(total)))
        q,newton=r.position(independent,target,q0,allow_unconverged=True)
        poses=r.kin.forward(np.r_[q,np.zeros(6)],np.eye(4));angles=[]
        for side in ('right','left'):
            d=P@(poses[side+'_shin'][:3,3]-poses[side+'_thigh'][:3,3]);angles.append(math.atan2(d[0],-d[2]))
        delta=-(np.array(angles)-r.leg_origin)
        arms=np.array(r.arm_map.evaluate(Jet.rest(delta),reserve=0.).position)+np.array(path.arm_slack.at(total).position)
        full=np.r_[q,arms];root=r.stance_pose.T@np.linalg.inv(poses[FEET[r.stance_side]])
        poses=r.kin.forward(full,root)
        com=sum(part['massKg']*(poses[part['id']]@np.r_[np.array([part['goldenMass']['centerOfMass'][z]for z in 'xyz']),1.])[:3]
            for part in r.kin.model['segments'])/r.kin.model['totalMassKg']
        commandcom=P@com+np.array(path.com_slack.at(total).position)
        output=(q,full,delta,commandcom,P@root[:3,:3]@P.T,poses,newton)
        # Returned geometry is read-only; cached values cannot acquire caller
        # mutations. This changes no scalar or numerical evaluation operation.
        for array in output[:5]:array.setflags(write=False)
        for array in poses.values():array.setflags(write=False)
        node_cache[node_key]=output
        if len(node_cache)>node_cache_capacity:
            node_cache.popitem(last=False);cache_stats['nodeEvictions']+=1
        return output

    nodes=[(k,float(f))for k in range(3)for f in FRACTIONS if not(k and f==0.)]
    count_per_node=15+15+6+6+2+2+len(support.equations())*2+4
    eq=support.equations();ranges=np.array(r.arm_map.ranges)
    def evaluate(x):
        nonlocal cached_x,cached,evaluations,last_candidate
        last_candidate=np.array(x,copy=True)
        check_control()
        candidate_key=np.asarray(x,dtype=np.float64).tobytes()
        if candidate_key in candidate_cache:
            if cached_x is not None and np.array_equal(x,cached_x):cache_stats['exactLastEvaluationHits']+=1
            cache_stats['fullCandidateHits']+=1;candidate_cache.move_to_end(candidate_key)
            cached_x=np.array(x,copy=True);cached=candidate_cache[candidate_key]
            return cached
        cache_stats['fullCandidateMisses']+=1
        evaluations+=1;path=path_from_knots(np.array(x).reshape(3,9))
        inequalities=[];objective=0.;rows=[]
        try:
            for k,f in nodes:
                t=path.durations[k]*f;q,full,delta,com,R,poses,newton=fast_geometry(path,k,t)
                dt=1e-4
                if f in (0.,1.):velocity=np.zeros(3);acceleration=np.zeros(3)
                else:
                    before=fast_geometry(path,k,t-dt)[3];after=fast_geometry(path,k,t+dt)[3]
                    velocity=(after-before)/(2*dt);acceleration=(after-2*com+before)/dt**2
                capture=com[:2]+math.sqrt(com[2]/9.81)*velocity[:2]
                margin=np.minimum(full-limits[:,0],limits[:,1]-full)
                inequalities.extend(np.r_[q-lo,hi-q,full[15:]-limits[15:,0]-.02,limits[15:,1]-full[15:]-.02,
                    delta-ranges[:,0]-.02,ranges[:,1]-delta-.02,
                    -(eq[:,:2]@com[:2]+eq[:,2])-.003,-(eq[:,:2]@capture+eq[:,2])-.003,
                    MAX_TOTAL_COM_SPEED-np.linalg.norm(velocity),MAX_TOTAL_COM_ACCELERATION-np.linalg.norm(acceleration),com[2],2e-8-newton['maximumPoseResidual']])
                angle=Rotation.from_matrix(target_body@R.T).as_rotvec()
                posture=q-nominal[k]
                # Only original4 body1 and.2 weak-posture row scales. Knee is a
                # soft target inside that existing weak row, never hard anatomy.
                objective+=.5*(sum((4.*angle)**2)+sum((.2*posture)**2))
                rows.append(dict(phase=PHASES[k],fraction=f,com=com.tolist(),minimumLowerReserveRad=float(min(margin[:15]))))
        except _ControlStop:raise
        except Exception as exc:
            # Failure remains a concrete prescreen rejection. No alternative
            # branch/seed, clipped target or optimizer restart is attempted.
            if len(exceptions)<8:exceptions.append(dict(type=type(exc).__name__,message=str(exc)))
            objective=1e6+sum((x-x0)**2);inequalities=[-1.]* (len(nodes)*count_per_node)
        cached_x=np.array(x,copy=True);cached=(float(objective),np.array(inequalities),rows)
        cached[1].setflags(write=False);candidate_cache[candidate_key]=cached
        if len(candidate_cache)>candidate_cache_capacity:
            candidate_cache.popitem(last=False);cache_stats['fullCandidateEvictions']+=1
        return cached

    declared=dict(schema=1,version='free-com-flight-horizon-once-v1',inputBinding=binding,
        family=dict(independentCoordinates=idx.tolist(),variables=27,phases=list(PHASES),
            clocksS=[.8-entry_elapsed,1.2,1.],originalClocksS=[.8,1.2,1.],originalLiftElapsedS=entry_elapsed,
            footLiftM=.04,ownFootAdvanceM=.182,sourceLandingContactPreferenceM=-.0005,
            implicitRigidFootExactAtEverySample=True,swingIKInterpolation=False,
            objective='existing4 body1 row toward4deg bust/zero roll plus existing.2 lower-posture row with soft knee gain10/5/0deg',
            kneeGoalHard=False,pelvisAnatomyHard=False,comTargetHard=False,actualStanceRealized=True,
            sourceInteriorRad=.01,armAndMapReserveRad=.02,anchoredLumbarChangeDeg=5.,
            maximumCommandCOMSpeedMps=MAX_TOTAL_COM_SPEED,maximumCommandCOMAccelerationMps2=MAX_TOTAL_COM_ACCELERATION,
            supportReserveM=.003,optimizerPrescreenFractions=list(FRACTIONS),
            optimizerDerivativeMethod='central finite difference positional prescreen only; final admission is independent analytic FK jets',
            analyticAdmissionFractions=list(analytic_fractions),wholeNonlinearPathCertificate=False,
            initializer='qualified stationary q0 independent coordinates replicated at all three endpoint knots; no projection',
            initializerTrace=initializer,body1Weight=4.,weakPostureWeight=.2,maxIterations=max_iterations,maxWallSeconds=max_wall_seconds,cooperativeCancellationEnabled=cancellation_check is not None,progressCallbackEnabled=progress_callback is not None,
            optimizerInvocationsAuthorized=1,noFallback=True,noGoalWeightScan=True),
        repositoryRuntimeEdits=False,nativeWorldConstructions=0,stateInstallations=0,physicsIntegrations=0)
    trace=[];progress_records=[]
    def callback(x):
        check_control()
        o,c,rows=evaluate(x);trace.append(dict(iteration=len(trace)+1,objective=o,minimumPrescreenSlack=float(min(c))))
        branch_rows=[]
        if len(rows)==len(nodes):
            for j,(k,f)in enumerate(nodes):
                residual=float(2e-8-c[j*count_per_node+count_per_node-1])
                branch_rows.append(dict(phase=PHASES[k],fraction=f,maximumPoseResidual=residual,
                    strictNewtonToleranceMet=bool(residual<2e-11),prescreenResidualAccepted=bool(residual<=2e-8)))
        event=dict(**trace[-1],unadmittedIndependentKnotsRad=np.array(x).reshape(3,9).tolist(),
            candidateSHA256=digest(np.array(x).reshape(3,9).tolist()),positionPrescreenEvaluations=evaluations,
            cacheCounters=dict(cache_stats,nodeEntries=len(node_cache),fullCandidateEntries=len(candidate_cache)),
            footBranchResidualSummary=dict(available=bool(branch_rows),rows=branch_rows,
                maximumPoseResidual=None if not branch_rows else max(row['maximumPoseResidual']for row in branch_rows)),
            nativeAdmission=False,physicalQualification=False)
        progress_records.append(event)
        if progress_callback is not None:
            # The caller receives a detached JSON-safe copy. Its mutation cannot
            # change retained trace, candidate, cache or future references.
            progress_callback(json.loads(json.dumps(event,allow_nan=False)))
    start=time.monotonic();fit=None;failure=None;path=None
    try:
        fit=minimize(lambda x:evaluate(x)[0],x0,method='SLSQP',bounds=list(zip(np.tile(lo[idx],3),np.tile(hi[idx],3))),
            constraints=[dict(type='ineq',fun=lambda x:evaluate(x)[1])],callback=callback,
            options=dict(maxiter=max_iterations,ftol=1e-10,eps=1e-6,disp=False))
        check_control()
    except _ControlStop as exc:
        failure=dict(type=type(exc).__name__,message=str(exc),kind=exc.kind);fit=None
    except Exception as exc:failure=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc())
    receipt=dict(declared,optimizerCalls=1,optimizerElapsedS=time.monotonic()-start,optimizerException=failure,
        optimizerSuccess=bool(fit is not None and fit.success),optimizerStatus=None if fit is None else str(fit.message),
        optimizerIterations=None if fit is None else int(fit.nit),positionPrescreenEvaluations=evaluations,prescreenExceptions=exceptions,trace=trace,
        nativeAdmission=False,globalInfeasibilityProven=False,finiteFailureNotGlobalProof=True)
    if fit is not None:
        knots=np.array(fit.x).reshape(3,9);path=path_from_knots(knots)
        receipt['independentKnotsRad']=knots.tolist();receipt['path']=path.receipt()
        samples=[];failures=[]
        for k,phase in enumerate(PHASES):
            for f in analytic_fractions:
                try:
                    check_control()
                    sample=path.sample(phase,float(f)*path.durations[k]);check=path.validate_sample(sample,support,require_clearance=phase=='translate')
                    row=dict(phase=phase,fraction=float(f),**check,lowerJoints=sample.lower_joints.position,
                        commandCOM=sample.com.position,commandCOMVelocity=sample.com.velocity,commandCOMAcceleration=sample.com.acceleration,
                        body1Rotation=sample.thorax.rotation,pelvisRotation=sample.pelvis.rotation,
                        kneeReferenceGainDeg=math.degrees(q0[(12,6)[r.swing_side]]-sample.lower_joints.position[(12,6)[r.swing_side]]),newton=sample.diagnostics)
                    if not check['accepted']:failures.append(dict(phase=phase,fraction=float(f),checks=check['checks']))
                except Exception as exc:
                    row=dict(phase=phase,fraction=float(f),accepted=False,errorType=type(exc).__name__,error=str(exc));failures.append(row)
                samples.append(row)
        endpoints=[row for row in samples if row['fraction']==1.]
        # Visual-correction diagnostic is independent of hard physical bounds.
        liftgain=endpoints[0].get('kneeReferenceGainDeg',0.)
        receipt.update(analyticSamples=samples,analyticFailures=failures,finiteSampledKinematicPass=bool(not failures),
            endpointSummaries=endpoints,achievedEarlyKneeReferenceGainDeg=liftgain,
            desiredEarlyKneeCorrectionPresent=bool(liftgain>1e-6),
            exactWholeNonlinearPathAdmission=False,liveSampleRecheckingRequired=True,
            finiteProposalAdmittedForNative=False,
            independentPolynomialWholeBounds=[segment.component_bounds().tolist()for segment in path.independent_segments])
    receipt['optimizerElapsedPlusAdmissionS']=time.monotonic()-start
    receipt['exactInputCaches']=dict(cache_stats,nodeCapacity=node_cache_capacity,nodeEntries=len(node_cache),
        fullCandidateCapacity=candidate_cache_capacity,fullCandidateEntries=len(candidate_cache),exactFloat64ByteKeys=True,noMutableIKSeed=True)
    receipt['progressRecords']=progress_records
    receipt['cooperativeControlTermination']=control_stop
    receipt['cooperativeControlGuardsPresentInThisLibrary']=True
    receipt['completedCallbackIterations']=len(trace)
    if control_stop is not None:
        receipt.update(terminalState=control_stop['kind'],accepted=False,finiteSampledKinematicPass=False,
            unadmittedLastCandidateIndependentKnotsRad=None if last_candidate is None else last_candidate.reshape(3,9).tolist(),
            unadmittedLastFullyEvaluatedIndependentKnotsRad=None if cached_x is None else cached_x.reshape(3,9).tolist(),
            lastCompletedPositionPrescreen=None if cached is None else dict(objective=cached[0],minimumSlack=float(min(cached[1])),rows=cached[2]),
            candidateDataAreNotAdmission=True,nativeAdmission=False,physicalQualification=False,globalInfeasibilityProven=False)
        if 'independentKnotsRad'in receipt:
            receipt['unadmittedOptimizerReturnedKnotsRad']=receipt.pop('independentKnotsRad')
        return None,receipt
    return (path if fit is not None else None),receipt
