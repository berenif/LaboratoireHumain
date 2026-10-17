#!/usr/bin/env python3
"""Sampled balance constraints and minimum tracking slack, no integration."""
import numpy as np
from scipy.spatial import ConvexHull,QhullError


def capture_rows(state,points,capture,margin=.003,gain=4.,allow_segment=False):
    """Frozen actual hull / variable-height LIP surrogate, not an invariant."""
    points=np.unique(np.asarray(points,dtype=float),axis=0);dimension=2;requested_margin=margin
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.all(np.isfinite(points)):raise ValueError('At least two finite actual support points required')
    try:
        hull=ConvexHull(points);normals=hull.equations[:,:2];offset=hull.equations[:,2]
    except QhullError as error:
        if not allow_segment or len(points)<2:raise ValueError('Noncollinear actual support hull required')from error
        distances=np.sum((points[:,None]-points[None,:])**2,axis=2);i,j=np.unravel_index(np.argmax(distances),distances.shape);a,b=points[[i,j]];length=np.linalg.norm(b-a)
        if length<1e-6:raise ValueError('Actual support segment has zero length')
        tangent=(b-a)/length;normal=np.array([-tangent[1],tangent[0]])
        normals=np.array([normal,-normal,tangent,-tangent]);offset=np.array([-normal@a,normal@a,-tangent@b,tangent@a]);dimension=1;margin=0.
    y=float(state['com'][1])
    if not np.isfinite(y) or y<=0.:raise ValueError('Positive finite COM height required')
    velocity=state['comVelocity'];omega=np.sqrt(9.81/y)
    h=-normals@np.asarray(capture)-offset-margin
    rows=normals@state['Jcom'][[0,2]]/omega
    upper=gain*h-normals@velocity[[0,2]]-.5*(normals@velocity[[0,2]])*velocity[1]/(omega*y)-normals@state['comBias'][[0,2]]/omega
    return rows,upper,dict(supportDimension=dimension,signedCaptureMarginM=float(min(h)+margin),requestedPolygonInsetM=requested_margin,insetM=margin,minimumInsetBarrierM=float(min(h)),initiallyInsideInset=bool(min(h)>=0.),interpretation='Sampled frozen-hull variable-height LIP capture guard; segment mode has ZERO inset and requests recovery only; angular momentum, contact-set motion and native tracking error are not bounded')


def solve_tracking_hierarchy(solver,H,g,E,b,C,upper,lo,hi,indices,lower_tracking,upper_tracking,tolerance=1e-6):
    """Minimize coupled squared tracking slack, then the original task cost.

    All input physics remains hard in both stages. No claim of native position
    invariance follows from a PD-acceleration tracking interval.
    """
    indices=np.asarray(indices,dtype=int);m=len(indices);n=len(g)
    lower_tracking=np.asarray(lower_tracking);upper_tracking=np.asarray(upper_tracking)
    fixed=lo[indices]==hi[indices];free=np.flatnonzero(~fixed);mf=len(free)
    fixed_slack=np.maximum.reduce([np.zeros(m),lower_tracking-lo[indices],hi[indices]-upper_tracking]);fixed_slack[~fixed]=0.
    constant_cost=float(.5*fixed_slack@fixed_slack)
    # Zero free slack is a global lower bound. A feasible optional-objective
    # solve at that bound is an exact certificate, avoiding a degenerate PSD
    # zero-minimum auxiliary QP without changing either priority or precision.
    zero_lo=lo.copy();zero_hi=hi.copy()
    zero_lo[indices]=np.maximum(zero_lo[indices],lower_tracking-fixed_slack)
    zero_hi[indices]=np.minimum(zero_hi[indices],upper_tracking+fixed_slack)
    if np.all(zero_lo<=zero_hi):
        zero=solver(H,g,E,b,C,upper,zero_lo,zero_hi)
        zero_residual=max(float(np.max(abs(E@zero.x-b),initial=0.)),float(np.max(C@zero.x-upper,initial=0.)),float(np.max(zero_lo-zero.x,initial=0.)),float(np.max(zero.x-zero_hi,initial=0.)))
        if zero.success and zero_residual<1e-6:
            actual=np.maximum.reduce([np.zeros(m),lower_tracking-zero.x[indices],zero.x[indices]-upper_tracking])
            return zero,dict(firstStageSuccess=True,firstStageMethod='Exact zero-free-slack feasibility certificate',firstStageMessage=zero.message,firstStageMaximumResidual=zero_residual,secondStageMaximumResidual=zero_residual,minimumSquaredSlackCost=constant_cost,constantFixedTrackingSlackCost=constant_cost,analyticallyEliminatedFixedTrackingCoordinates=indices[fixed].tolist(),minimumSlackRadps2=fixed_slack.tolist(),representedSlackRadps2=actual.tolist(),maximumRepresentedSlackRadps2=float(max(actual,default=0.)),qpos=(indices-6).tolist(),slackToleranceRadps2=tolerance,interpretation='Zero free tracking slack is feasible and globally minimal; fixed-coordinate unavoidable slack retained as constant primary cost. Original optional objective solved at that bound')
    first_H=np.zeros((n+mf,n+mf));first_H[n:,n:]=np.eye(mf)
    first_E=np.pad(E,((0,0),(0,mf)));first_C=np.pad(C,((0,0),(0,mf)))
    tracking_rows=np.zeros((2*mf,n+mf))
    for k,j in enumerate(free):
        i=indices[j];tracking_rows[2*k,i]=1.;tracking_rows[2*k,n+k]=-1.
        tracking_rows[2*k+1,i]=-1.;tracking_rows[2*k+1,n+k]=-1.
    tracking_upper=np.column_stack([upper_tracking[free],-lower_tracking[free]]).ravel()
    all_C=np.vstack([first_C,tracking_rows]);all_upper=np.r_[upper,tracking_upper]
    slack_cap=np.maximum(abs(lo[indices[free]]-upper_tracking[free]),abs(hi[indices[free]]-lower_tracking[free]))+1.
    first_lo=np.r_[lo,np.zeros(mf)];first_hi=np.r_[hi,slack_cap]
    first=solver(first_H,np.zeros(n+mf),first_E,b,all_C,all_upper,first_lo,first_hi)
    residual=max(float(np.max(abs(first_E@first.x-b),initial=0.)),float(np.max(all_C@first.x-all_upper,initial=0.)),float(np.max(first_lo-first.x,initial=0.)),float(np.max(first.x-first_hi,initial=0.)))
    info=dict(firstStageSuccess=bool(first.success and residual<1e-6),firstStageMessage=first.message,firstStageObjectiveGapTolerance=1e-8,analyticallyEliminatedFixedTrackingCoordinates=indices[fixed].tolist(),constantFixedTrackingSlackCost=constant_cost,firstStagePrimalObjective=first.get('primalObjective'),firstStageDualObjective=first.get('dualObjective'),firstStageDualResidual=first.get('dualResidual'),firstStageMaximumResidual=residual,slackToleranceRadps2=tolerance,interpretation='Coupled minimum squared PD-acceleration tracking slack under hard balance and physical constraints, then original optional task objective; contact-mode costs within round(cost,6) numerical bins tie on original optional cost')
    if not info['firstStageSuccess']:
        first.x=first.x[:n];first.success=False;return first,info
    slack=fixed_slack.copy();slack[free]=np.maximum(first.x[n:],0.);next_lo=lo.copy();next_hi=hi.copy()
    next_lo[indices]=np.maximum(next_lo[indices],lower_tracking-slack-tolerance)
    next_hi[indices]=np.minimum(next_hi[indices],upper_tracking+slack+tolerance)
    second=solver(H,g,E,b,C,upper,next_lo,next_hi)
    stage2_residual=max(float(np.max(abs(E@second.x-b),initial=0.)),float(np.max(C@second.x-upper,initial=0.)),float(np.max(next_lo-second.x,initial=0.)),float(np.max(second.x-next_hi,initial=0.)))
    second.success=bool(second.success and stage2_residual<1e-6)
    actual=np.maximum.reduce([np.zeros(m),lower_tracking-second.x[indices],second.x[indices]-upper_tracking])
    info.update(secondStageMaximumResidual=stage2_residual,minimumSquaredSlackCost=float(.5*slack@slack),minimumSlackRadps2=slack.tolist(),representedSlackRadps2=actual.tolist(),maximumRepresentedSlackRadps2=float(max(actual,default=0.)),qpos=(indices-6).tolist())
    return second,info
