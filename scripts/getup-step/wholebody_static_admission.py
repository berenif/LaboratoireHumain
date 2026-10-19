"""Finite current-support static path admission, never a physics rollout.

Only fresh counter-zero scratch worlds receive proposed states. The supplied
actual world is read-only and its complete native bytes/counter are rechecked.
Passing samples are not a whole-path or dynamic qualification. Every actual
command still requires unchanged native gates and fresh risk admission.
"""
import ctypes as C
import hashlib
import numpy as np
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation

from source_step_probe import ptr, sha
from source_arm_knee_mobility import native_bytes
from source_posture_retarget import _actuators
from coupled_posture_kinematics import P


def strict_contact_lp(w, contacts):
    """Original motor/static force LP for explicit (foot-side,point) contacts."""
    if not contacts:
        raise ValueError('current actual contact points required')
    B = _actuators(w); feet = w.feet(); n = len(contacts); N = 22+3*n
    A = np.zeros((27,N)); A[:,:21] = B
    for i,(side,point) in enumerate(contacts):
        if side not in (0,1): raise ValueError('left/right foot side required')
        p = np.asarray(point,dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all(): raise ValueError('finite actual point required')
        A[:,21+3*i:24+3*i] = w.read('lh_point_jac',(3,27),feet[side]['body'],
            ptr(np.ascontiguousarray(p))).T
    U = []
    for j in range(21):
        for sign in (1.,-1.):
            row = np.zeros(N);row[j]=sign;row[-1]=-1.;U.append(row)
    for i in range(n):
        for x,y in ((1,0),(-1,0),(0,1),(0,-1)):
            row=np.zeros(N);row[21+3*i]=x;row[22+3*i]=y;row[23+3*i]=-.6;U.append(row)
    objective=np.zeros(N);objective[-1]=1.;bias=w.read('lh_bias',(27,))
    fit=linprog(objective,A_ub=np.array(U),b_ub=np.zeros(len(U)),A_eq=A,b_eq=bias,
        bounds=[(-.95,.95)]*21+[(None,None),(None,None),(0,w.mass*9.81)]*n+[(0,.95)],method='highs')
    receipt=dict(success=bool(fit.success),status=str(fit.message),requiredMotorFraction=None,
        explicitContacts=[dict(footSide=s,pointSource=list(p)) for s,p in contacts],
        originalMotorBounds=[-.95,.95],conservativeStaticFriction=.6,contactModelChanged=False)
    if fit.success:
        forces=fit.x[21:21+3*n].reshape(n,3)
        receipt.update(requiredMotorFraction=float(fit.x[-1]),commands=fit.x[:21].tolist(),forcesN=forces.tolist(),
            maximumEqualityResidual=float(max(abs(A@fit.x-bias))),
            maximumFrictionViolation=float(max(0.,np.max(abs(forces[:,:2])-.6*forces[:,2,None]))),
            minimumNormalN=float(min(forces[:,2])))
    return receipt


def finite_static_admission(actual_world, samples, contacts, *, factory=None):
    """Two separately constructed scratch rechecks, no native integration.

    samples is a frozen iterable of (label, CycleSample), from a current-bound
    admitted geometric proposal. contacts remain actual matched support points,
    never a planned future contact. A candidate may not change after a rejection.
    """
    actual=actual_world;before=native_bytes(actual);counter=actual.l.lh_dim(actual.h,6)
    rows=list(samples)
    if not rows:raise ValueError('nonempty finite path required')
    expected=dict(modelSHA256=sha(actual.xml),helperSHA256=sha(actual.helper),
        bridgeSHA256=sha(actual.base/'source_bridge_v2.so'),
        engineSHA256=sha(actual.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    worlds=[];proof=[];accepted=True;failure=None
    try:
        for repetition in range(2):
            w=factory(actual) if factory else type(actual)(actual.base,actual.helper,xml=actual.xml)
            if w is actual:
                raise ValueError('fresh counter-zero scratch required')
            worlds.append(w)
            if w.l.lh_dim(w.h,6)!=0:
                raise ValueError('fresh counter-zero scratch required')
            if sha(w.xml)!=expected['modelSHA256'] or sha(w.helper)!=expected['helperSHA256']:
                raise ValueError('scratch source/helper differs')
            if np.max(abs(np.array(w.limits)-np.array(actual.limits)))>1e-12:
                raise ValueError('original source joint limits differ')
            def forbidden(*args,**kwargs):raise AssertionError('static admission must never integrate')
            w.l.lh_step_audited=forbidden
            w.l.lh_self_separation.argtypes=[C.c_void_p,C.POINTER(C.c_double)]
            w.l.lh_self_separation.restype=None
            for label,sample in rows:
                result=sample.geometric;q=np.array(result.full_joints.position);root=result.root_model.T
                if q.shape!=(21,) or not np.isfinite(q).all():raise ValueError('finite full21 proposed geometry required')
                quaternion=Rotation.from_matrix(P@root[:3,:3]@P.T).as_quat()
                w.install_cold(dict(qpos=np.r_[P@root[:3,3],quaternion[[3,0,1,2]],q].tolist(),qvel=[0.]*27))
                lp=strict_contact_lp(w,contacts)
                raw=w.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14)
                gap=float(min(pairs[:,4]))
                # Existing packet exposes source COM without stepping; its
                # support/contact readiness is not used as a future witness.
                native_com=np.array(w.packet()['com'])
                com_error=float(max(abs(native_com-np.array(result.com_source.position))))
                margins=np.minimum(q-np.array(w.limits)[:,0],np.array(w.limits)[:,1]-q)
                ok=bool(lp['success'] and lp['requiredMotorFraction']<.925 and
                    lp['maximumEqualityResidual']<1e-8 and lp['maximumFrictionViolation']<1e-8
                    and gap>.025 and com_error<1e-8 and min(margins[:15])>=.01-1e-9
                    and min(margins[15:])>=.02-1e-9 and w.l.lh_dim(w.h,6)==0)
                proof.append(dict(repetition=repetition,label=label,accepted=ok,staticLP=lp,
                    minimumCleanSelfGapM=gap,independentNativeCOMErrorM=com_error,
                    minimumLowerJointMarginRad=float(min(margins[:15])),nativeCounter=0))
                if not ok:
                    accepted=False;failure='finite current-support motor/contact/gap/FK admission';break
            if not accepted:break
    finally:
        for w in worlds:w.close()
        if native_bytes(actual)!=before or actual.l.lh_dim(actual.h,6)!=counter:
            raise ValueError('static path admission mutated the actual native world')
    return dict(schema=1,policy='two-counter-zero-finite-current-support-path-check-v1',
        accepted=accepted,rejection=failure,sourceStep=counter,
        actualNativePayloadSHA256=hashlib.sha256(before).hexdigest(),**expected,
        worldsConstructed=len(worlds),checks=proof,physicsIntegrations=0,actualStateWrites=0,
        counterZeroScratchInstallations=len(proof),actualBytesAndCounterUnchanged=True,
        finiteSamplesOnly=True,wholePathOrDynamicQualification=False,
        freshMeasuredRiskAndOriginalEveryNativeGatesStillRequired=True)
