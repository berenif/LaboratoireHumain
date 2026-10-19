"""Pure C2 coordinated swing-foot/leg IK reference. Never opens a native world.

One declared Cartesian family, not an additive knee target: 40 mm lift, 182 mm
translation, rigid orientation, 0.8/1.2/1.0 s quintics. A supplied analytic root
and stance path may move. Only swing joints are replaced. All other planned
joints and all measured delayed-arm state belong to their existing owners.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

P = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
FEET = ('left_foot', 'right_foot')
INDICES = (tuple(range(9, 15)), tuple(range(3, 9)))
KNEES = (12, 6)  # feet left/right; flexion is NEGATIVE q on BOTH sides.
DURATIONS = {'lift': .8, 'translate': 1.2, 'land': 1.}


def frozen(value):
    a = np.asarray(value, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('nonfinite pure reference input')
    return tuple(float(x) for x in a.reshape(-1))


@dataclass(frozen=True)
class Jet:
    position: tuple
    velocity: tuple
    acceleration: tuple

    def __post_init__(self):
        for field in ('position', 'velocity', 'acceleration'):
            object.__setattr__(self, field, frozen(getattr(self, field)))
        if len(self.position) != len(self.velocity) or len(self.position) != len(self.acceleration):
            raise ValueError('inconsistent jet dimensions')

    @classmethod
    def rest(cls, value):
        p = frozen(value)
        return cls(p, (0.,)*len(p), (0.,)*len(p))

    def arrays(self):
        return tuple(np.array(x) for x in (self.position, self.velocity, self.acceleration))


@dataclass(frozen=True)
class MatrixJet(Jet):
    """Homogeneous transform and its two derivatives, in model/FK Y-up axes."""
    def __post_init__(self):
        super().__post_init__()
        if len(self.position) != 16:
            raise ValueError('pose jet requires three 4x4 matrices')

    def arrays(self):
        return tuple(a.reshape(4, 4) for a in super().arrays())

    def __matmul__(self, other):
        x,v,a = self.arrays(); y,w,b = matrices(other)
        return MatrixJet(x@y, v@y+x@w, a@y+2*v@w+x@b)

    def inverse(self):
        x,v,a = self.arrays(); y=np.linalg.inv(x)
        return MatrixJet(y, -y@v@y, 2*y@v@y@v@y-y@a@y)


def matrices(pose):
    return tuple(np.asarray(x).reshape(4,4) for x in pose.arrays())


def skew(w):
    x,y,z = w
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def vee(m):
    return np.array([m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1]])/2.


def source_pose_jet(position, rotation, omega=(0.,0.,0.), alpha=(0.,0.,0.)):
    """Source Z-up position Jet and WORLD spatial angular jets -> model matrices."""
    p,v,a = position.arrays(); R=np.asarray(rotation).reshape(3,3)
    W=skew(omega); A=skew(alpha)
    T=np.eye(4); V=np.zeros((4,4)); H=np.zeros((4,4))
    T[:3,:3]=P.T@R@P; T[:3,3]=P.T@p
    V[:3,:3]=P.T@(W@R)@P; V[:3,3]=P.T@v
    H[:3,:3]=P.T@((A+W@W)@R)@P; H[:3,3]=P.T@a
    return MatrixJet(T,V,H)


def source_jet(pose):
    T,V,H=matrices(pose); R=P@T[:3,:3]@P.T
    dR=P@V[:3,:3]@P.T; ddR=P@H[:3,:3]@P.T
    W=dR@np.linalg.inv(R)
    return (Jet(P@T[:3,3], P@V[:3,3], P@H[:3,3]), R,
            vee(W), vee(ddR@np.linalg.inv(R)-W@W))


def assert_rigid_jet(pose, tolerance=2e-8):
    T,V,H=matrices(pose); R=T[:3,:3]; D=V[:3,:3]; A=H[:3,:3]
    errors=[np.max(abs(T[3]-[0.,0.,0.,1.])), np.max(abs(V[3])), np.max(abs(H[3])),
            np.max(abs(R.T@R-np.eye(3))), abs(np.linalg.det(R)-1.),
            np.max(abs(D.T@R+R.T@D)), np.max(abs(A.T@R+2*D.T@D+R.T@A))]
    if max(errors)>tolerance:
        raise ValueError('root/stance input is not a compatible rigid pose jet')


@dataclass(frozen=True)
class BaseFrameSample:
    root_model: MatrixJet
    joints: Jet
    stance_pose_model: MatrixJet

    @classmethod
    def from_coupled_frame(cls, sample):
        # Structural protocol; no import or mutation of another branch/module.
        return cls(sample.root_model, sample.joints, sample.stance_pose_model)


class ReferenceRejected(ValueError):
    def __init__(self, reason, **receipt):
        self.receipt = dict(accepted=False, reason=reason, physicsIntegrations=0,
            nativeStateInstallations=0, sourceAdmitted=False, **receipt)
        super().__init__(reason)


class AdmissionRequired(ValueError):
    pass


@dataclass(frozen=True)
class MeasuredAdvanceSample:
    native_step: int
    whole_foot_clearance_m: float
    swing_load_n: float
    stance_load_n: float


@dataclass(frozen=True)
class AdvanceWindow:
    """Caller-provided ACTUAL native observations, not planned FK clearances.

    This pure object cannot authenticate the observations. The native integration
    must reconstruct and audit them, and retain all other existing admission gates.
    """
    samples: tuple
    weight_n: float
    current_native_step: int

    def validate(self):
        if len(self.samples)<80 or not math.isfinite(self.weight_n) or self.weight_n<=0.:
            raise AdmissionRequired('80 measured native samples and valid weight required')
        samples=self.samples[-80:]
        if samples[-1].native_step!=self.current_native_step:
            raise AdmissionRequired('stale measured clearance window')
        for i,s in enumerate(samples):
            values=(s.whole_foot_clearance_m,s.swing_load_n,s.stance_load_n)
            if (not all(math.isfinite(v) for v in values) or
                (i and s.native_step!=samples[i-1].native_step+1) or
                s.whole_foot_clearance_m<.025 or s.swing_load_n>=3. or
                s.stance_load_n<.8*self.weight_n):
                raise AdmissionRequired('measured whole-foot/load/consecutive advance gate failed')
        return dict(samples=80,lastNativeStep=self.current_native_step,
            minimumWholeFootClearanceM=min(s.whole_foot_clearance_m for s in samples),
            independentlyAuthenticated=False)


class FKJets:
    """Analytic product-rule jets using the EXISTING pure Kinematics frame model."""
    def __init__(self, kinematics):
        self.kin=kinematics
        self.model=kinematics.model

    def forward(self, joints, root):
        q,v,a=joints.arrays(); out={'torso':MatrixJet(*matrices(root))}
        for part in self.model['segments'][1:]:
            name=part['id']; h=part['reference'].get('hinge')
            M=np.eye(4); D=np.zeros((4,4)); H=np.zeros((4,4))
            if h:
                j=h['qposIndex']; c,s=math.cos(q[j]),math.sin(q[j])
                M[1:3,1:3]=[[c,-s],[s,c]]
                F=np.array([[-s,-c],[c,-s]]); G=np.array([[-c,s],[-s,-c]])
                D[1:3,1:3]=F*v[j]; H[1:3,1:3]=G*v[j]**2+F*a[j]
            out[name]=(out[part['parent']]@MatrixJet.rest(self.kin.parent_frames[name])@
                MatrixJet(M,D,H)@MatrixJet.rest(self.kin.child_inverse[name]))
        return out

    def spatial_jacobian(self, q, root, foot, indices):
        poses=self.kin.forward(q,matrices(root)[0]); p=poses[foot][:3,3]
        # All source hinges rotate about frame-local X; model anchors include
        # the source axis rotations and the real q[6]/q[12] knee signs.
        axes={}
        for part in self.model['segments'][1:]:
            h=part['reference'].get('hinge')
            if h:
                T=poses[part['parent']]@self.kin.parent_frames[part['id']]
                axes[h['qposIndex']]=(T[:3,0],T[:3,3])
        return np.column_stack([np.r_[np.cross(axes[j][0],p-axes[j][1]),axes[j][0]] for j in indices])

    def inverse_jet(self, baseline, root, foot, target, indices, limits, seed=None):
        q,v,a=baseline.arrays(); idx=np.array(indices); desired=matrices(target)[0]
        lo,hi=np.asarray(limits)[idx].T
        initial=q[idx] if seed is None else np.asarray(seed)
        # Clipping is numerical initialization only; no infeasible target is
        # clipped into success, and the final pose/reserve is independently tested.
        initial=np.minimum(np.maximum(initial,lo+1e-11),hi-1e-11)
        def residual(x):
            b=q.copy(); b[idx]=x; T=self.kin.forward(b,matrices(root)[0])[foot]
            return np.r_[T[:3,3]-desired[:3,3], .15*Rotation.from_matrix(T[:3,:3]@desired[:3,:3].T).as_rotvec()]
        fit=least_squares(residual,initial,bounds=(lo,hi),method='trf',
            max_nfev=100,xtol=2e-13,ftol=2e-13,gtol=2e-13)
        q[idx]=fit.x; pose=self.kin.forward(q,matrices(root)[0])[foot]
        pe=float(np.linalg.norm(pose[:3,3]-desired[:3,3]))
        re=float(Rotation.from_matrix(pose[:3,:3]@desired[:3,:3].T).magnitude())
        if pe>2e-9 or re>2e-9:
            raise ReferenceRejected('fixed supplied-frame IK/foot pose is infeasible',
                foot=foot,positionResidualM=pe,rotationResidualRad=re,
                minimumInteriorSlackRad=float(np.min(np.minimum(fit.x-lo,hi-fit.x))),
                bestSwingJointsRad=fit.x.tolist())
        J=self.spatial_jacobian(q,root,foot,idx); condition=float(np.linalg.cond(J))
        if condition>1e7:
            raise ReferenceRejected('IK branch is singular',conditionNumber=condition)
        v[idx]=0.; a[idx]=0.
        p0,R0,w0,al0=source_jet(self.forward(Jet(q,v,a),root)[foot])
        pt,Rt,wt,alt=source_jet(target)
        # J is model axes; convert requested spatial differences back from source.
        v[idx]=np.linalg.solve(J,np.r_[P.T@(np.array(pt.velocity)-p0.velocity),P.T@(wt-w0)])
        pb,Rb,wb,alb=source_jet(self.forward(Jet(q,v,a),root)[foot])
        a[idx]=np.linalg.solve(J,np.r_[P.T@(np.array(pt.acceleration)-pb.acceleration),P.T@(alt-alb)])
        result=Jet(q,v,a); matched=self.forward(result,root)[foot]
        return result,matched,dict(positionResidualM=pe,rotationResidualRad=re,conditionNumber=condition)


def clock(t,duration):
    if not math.isfinite(t):
        raise ValueError('nonfinite phase elapsed time')
    if t<=0.: return 0.,0.,0.
    if t>=duration: return 1.,0.,0.
    u=t/duration
    return 10*u**3-15*u**4+6*u**5, (30*u**2-60*u**3+30*u**4)/duration, (60*u-180*u**2+120*u**3)/duration**2


@dataclass(frozen=True)
class LegFootSample:
    phase: str
    elapsed: float
    swing_side: int
    joint_indices: tuple
    joints: Jet  # ONLY the six replacement swing joints, in source qpos order.
    base_frame: BaseFrameSample
    foot_model: MatrixJet
    planned_whole_foot_clearance_m: float
    knee_flexion_from_lift_entry_rad: float
    diagnostics: tuple

    def foot_source(self):
        return source_jet(self.foot_model)


class PhaseLegTrajectory:
    def __init__(self, kinematics, swing_side, frame_path, limits, heading_source,
                 *, joint_reserve=.01, ankle_reserve=None, knee_preference_rad=math.radians(10.),
                 max_stance_flexion_rad, ground_height_m=0.):
        """frame_path(phase,elapsed)->BaseFrameSample with ANALYTIC input jets.

        No clocks are inferred from native state. Durations, height and stride are
        pinned. Reducing knee_preference_rad requires the caller's explicit input.
        The owning controller must provide C2 frame jets across actual waits and
        handoffs; this module validates supplied frame/stance compatibility.
        """
        if swing_side not in (0,1): raise ValueError('feet are [left,right]')
        self.fk=FKJets(kinematics); self.side=swing_side; self.stance=1-swing_side
        self.indices=INDICES[swing_side]; self.frame_path=frame_path
        self.limits=np.asarray(limits,float).copy(); self.reserves=np.full(21,joint_reserve)
        if joint_reserve<.01 or self.limits.shape!=(21,2): raise ValueError('source limits/reserves required')
        if ankle_reserve is not None:
            if ankle_reserve<joint_reserve: raise ValueError('ankle reserve cannot be relaxed')
            self.reserves[[7,13]]=ankle_reserve
        self.interior=self.limits+np.column_stack((self.reserves,-self.reserves))
        self.heading=np.asarray(heading_source,float).copy()
        if self.heading.shape!=(3,) or abs(self.heading[2])>1e-12 or abs(np.linalg.norm(self.heading)-1.)>1e-10:
            raise ValueError('unit horizontal source heading required')
        self.preference=float(knee_preference_rad); self.stance_cap=float(max_stance_flexion_rad)
        self.ground=float(ground_height_m)
        if not all(math.isfinite(x) for x in (self.preference,self.stance_cap,self.ground)) or self.preference<0. or self.stance_cap<=0.:
            raise ValueError('invalid knee/stance/ground inputs')
        initial=self._frame('lift',0.)
        self.q_entry=np.array(initial.joints.position)
        self.entry_pose=self.fk.kin.forward(self.q_entry,matrices(initial.root_model)[0])[FEET[self.side]]
        self.entry_source=source_jet(MatrixJet.rest(self.entry_pose))
        self.prepared_receipt=None
        self.q_entry.setflags(write=False)
        self.limits.setflags(write=False); self.interior.setflags(write=False)
        self.heading.setflags(write=False); self.reserves.setflags(write=False)

    def _frame(self,phase,t):
        if not math.isfinite(t) or t<0.: raise ValueError('finite nonnegative phase time required')
        f=BaseFrameSample.from_coupled_frame(self.frame_path(phase,t))
        assert_rigid_jet(f.root_model); assert_rigid_jet(f.stance_pose_model)
        if len(f.joints.position)!=21: raise ValueError('frame baseline requires 21 joint jets')
        # Future arms may be present in baseline for geometry, but this module
        # never returns or installs them as runtime targets.
        q,v,a=f.joints.arrays(); stance_idx=np.array(INDICES[self.stance])
        preserved_idx=np.array([i for i in range(15) if i not in self.indices])
        slack=np.minimum(q[preserved_idx]-self.interior[preserved_idx,0],self.interior[preserved_idx,1]-q[preserved_idx])
        if min(slack)<-1e-9 or q[KNEES[self.stance]]>0. or -q[KNEES[self.stance]]>self.stance_cap+1e-10:
            raise ReferenceRejected('supplied stance joint reserve/flexion rejected',phase=phase,elapsed=t)
        actual=self.fk.forward(f.joints,f.root_model)[FEET[self.stance]]
        errors=[float(np.max(abs(x-y))) for x,y in zip(matrices(actual),matrices(f.stance_pose_model))]
        if max(errors)>3e-8:
            raise ReferenceRejected('supplied root/stance/joint jets are incompatible',phase=phase,elapsed=t,poseJetErrors=errors)
        return f

    def target(self,phase,t,*,landing_seek=None):
        p0,R,_,_=self.entry_source; p=np.array(p0.position); v=np.zeros(3); a=np.zeros(3)
        if phase in DURATIONS: s,ds,dds=clock(t,DURATIONS[phase])
        elif phase in ('lift_wait','translate_wait','land_wait','settle','stand'): s,ds,dds=1.,0.,0.
        else: raise ValueError('unknown swing phase')
        if phase in ('lift','lift_wait'):
            p[2]+=.04*s; v[2]=.04*ds; a[2]=.04*dds
        elif phase in ('translate','translate_wait'):
            p+=.182*self.heading*s; p[2]+=.04
            v=.182*self.heading*ds; a=.182*self.heading*dds
        else:
            p+=.182*self.heading
            if phase=='land':
                p[2]+=.04-.0405*s; v[2]=-.0405*ds; a[2]=-.0405*dds
            else: p[2]-=.0005
        if landing_seek is not None:
            seek=np.asarray(landing_seek,dtype=float)
            if (seek.shape!=(3,) or not np.isfinite(seek).all() or seek[0]<0. or seek[0]>.003+1e-12
                    or abs(seek[1])>.003+1e-12 or abs(seek[2])>.03+1e-12):
                raise ReferenceRejected('existing contact-seek jet exceeds original depth/rate/acceleration bounds')
            if phase not in ('land','land_wait','settle','stand') and np.max(abs(seek))>0.:
                raise ReferenceRejected('nonzero contact-seek offset before landing')
            p[2]-=seek[0]; v[2]-=seek[1]; a[2]-=seek[2]
        return source_pose_jet(Jet(p,v,a),R)

    def geometry_sample(self,phase,t,*,landing_seek=None):
        """Diagnostic geometry only: MUST NOT be used to bypass evaluate() gates."""
        f=self._frame(phase,t); goal=self.target(phase,t,landing_seek=landing_seek)
        try:
            joint,matched,info=self.fk.inverse_jet(f.joints,f.root_model,FEET[self.side],goal,
                self.indices,self.interior,seed=self.q_entry[list(self.indices)])
        except ReferenceRejected as error:
            error.receipt.update(phase=phase,elapsedS=float(t),
                requestedFootSourcePosition=list(source_jet(goal)[0].position))
            raise
        q,v,a=joint.arrays(); knee=q[KNEES[self.side]]
        if knee>0.:
            raise ReferenceRejected('swing knee hyperextension',phase=phase,elapsed=t,kneeRad=float(knee))
        T=matrices(matched)[0]
        clearance=self.fk.kin.bottom(FEET[self.side],T)-self.ground
        # The nominal target reaching 40mm never proves ACTUAL native clearance.
        reserve=np.minimum(q-self.limits[:,0],self.limits[:,1]-q)
        info.update(dynamicAdmissionProven=False,armsReplaced=False,
            swingMinimumJointReserveRad=float(min(reserve[list(self.indices)])),
            swingPitchAnkleReserveRad=float(reserve[(13,7)[self.side]]),
            stanceMinimumJointReserveRad=float(min(reserve[list(INDICES[self.stance])])) )
        return LegFootSample(phase,float(t),self.side,self.indices,
            Jet(q[list(self.indices)],v[list(self.indices)],a[list(self.indices)]),f,matched,
            float(clearance),float(self.q_entry[KNEES[self.side]]-knee),tuple(info.items()))

    def prepare(self,samples_per_phase=17):
        """Finite sampling of ONE trajectory family; no coefficient/height search.

        This rejects sampled failures. There is no interval or dynamic proof;
        every later evaluation repeats IK/reserve/stance checks.
        """
        if type(samples_per_phase)is not int or samples_per_phase<5: raise ValueError('at least five trajectory samples per phase')
        values=[]
        try:
            incoming=self._frame('lift',0.)
            entry=self.geometry_sample('lift',0.)
            incoming_errors=[float(np.max(abs(np.array(x)[list(self.indices)]-y)))
                for x,y in zip(incoming.joints.arrays(),entry.joints.arrays())]
            if max(incoming_errors)>3e-8:
                raise ReferenceRejected('incoming lift-entry leg jets are incompatible; no C2 reset allowed',maximumJetError=max(incoming_errors))
            for phase,duration in DURATIONS.items():
                for t in np.linspace(0.,duration,samples_per_phase):
                    values.append(self.geometry_sample(phase,float(t)))
            early=[x for x in values if x.phase=='lift' or (x.phase=='translate' and x.elapsed<.6)]
            achieved=max(x.knee_flexion_from_lift_entry_rad for x in early)
            if achieved+1e-10<self.preference:
                raise ReferenceRejected('explicit early-knee preference missed; not silently reduced',
                    requestedEarlyFlexionRad=self.preference,achievedEarlyFlexionRad=achieved)
            peak=max(range(len(values)),key=lambda i:values[i].knee_flexion_from_lift_entry_rad)
            if values[peak] not in early:
                raise ReferenceRejected('global knee-flexion peak is later than the early window')
            after_peak=values[peak:]
            knee_positions=[sample.joints.position[3] for sample in after_peak]
            if (knee_positions[-1]<=knee_positions[0]+1e-8 or
                    any(b<a-1e-8 for a,b in zip(knee_positions,knee_positions[1:])) or
                    any(sample.joints.velocity[3]<-1e-7 for sample in after_peak)):
                raise ReferenceRejected('coordinated family does not provide progressive extension after early flexion')
            translation=[x for x in values if x.phase=='translate']
            if min(x.planned_whole_foot_clearance_m for x in translation)<.025-1e-9:
                raise ReferenceRejected('planned whole rigid-foot clearance is below 25mm')
            # Verify supplied phase/frame and derived leg/foot jets across every
            # stationary wait/handoff. No elapsed-time or velocity reset is used.
            boundaries=[(('lift',.8),('lift_wait',0.)),(('lift_wait',1.7),('translate',0.)),
                (('translate',1.2),('translate_wait',0.)),(('translate_wait',1.7),('land',0.)),
                (('land',1.),('land_wait',0.)),(('land_wait',1.7),('settle',0.)),(('settle',1.7),('stand',0.))]
            for (p,t),(r,u) in boundaries:
                left=self.geometry_sample(p,t); right=self.geometry_sample(r,u)
                errors=[max(abs(x-y)) for x,y in zip(left.joints.arrays(),right.joints.arrays())]
                errors.extend(np.max(abs(x-y)) for x,y in zip(matrices(left.foot_model),matrices(right.foot_model)))
                lf=self._frame(p,t); rf=self._frame(r,u)
                errors.extend(np.max(abs(x-y)) for x,y in zip(matrices(lf.root_model),matrices(rf.root_model)))
                errors.extend(np.max(abs(x-y)) for x,y in zip(matrices(lf.stance_pose_model),matrices(rf.stance_pose_model)))
                keep=[i for i in range(15) if i not in self.indices]
                errors.extend(np.max(abs(x[keep]-y[keep])) for x,y in zip(lf.joints.arrays(),rf.joints.arrays()))
                if max(errors)>3e-8:
                    raise ReferenceRejected('supplied frame or derived leg/foot handoff is not C2',leftPhase=p,rightPhase=r,maximumJetError=float(max(errors)))
            receipt=dict(accepted=True,sourceAdmitted=False,dynamicAdmissionProven=False,
                physicsIntegrations=0,nativeStateInstallations=0,trajectoryFamilies=1,parameterScan=False,
                swingSide=self.side,kneeQposIndex=KNEES[self.side],kneeFlexionSign=-1,
                requestedEarlyFlexionDeg=math.degrees(self.preference),achievedEarlyFlexionDeg=math.degrees(achieved),
                liftHeightM=.04,translationM=.182,phaseDurationsS=dict(DURATIONS),
                sampledPathPoints=len(values),intervalFeasibilityProven=False,
                sampledProgressiveExtensionAfterEarlyPeak=True,
                extensionFromPeakToLandingDeg=math.degrees(knee_positions[-1]-knee_positions[0]),
                minimumPlannedTranslateWholeFootClearanceM=min(x.planned_whole_foot_clearance_m for x in translation),
                measuredAdvanceWindowRequired=80,armsReplaced=False)
        except ReferenceRejected as error:
            receipt=dict(error.receipt,swingSide=self.side,liftHeightM=.04,translationM=.182,
                phaseDurationsS=dict(DURATIONS),trajectoryFamilies=1,parameterScan=False,
                requestedEarlyFlexionDeg=math.degrees(self.preference),armsReplaced=False)
        self.prepared_receipt=receipt
        return receipt

    def evaluate(self,phase,t,*,advance_window=None,landing_seek=None):
        if self.prepared_receipt is None or not self.prepared_receipt['accepted']:
            raise AdmissionRequired('accepted pure preparation required; source dynamics still unqualified')
        if phase not in ('lift','lift_wait'):
            if advance_window is None: raise AdmissionRequired('actual 80-sample whole-foot advance admission required')
            advance_window.validate()
        return self.geometry_sample(phase,t,landing_seek=landing_seek)


def overlay_joint_jet(baseline, sample, *, enabled):
    """Disabled returns the EXACT input object; enabled replaces only swing6.

    ReferenceContext.arms, its persistent origins/history, gain -1 and measured
    delayed targets remain wholly external. Do not replace those with future FK.
    """
    if not enabled: return baseline
    p,v,a=(np.array(x,copy=True) for x in baseline.arrays()); idx=list(sample.joint_indices)
    for dst,src in zip((p,v,a),sample.joints.arrays()): dst[idx]=src
    return Jet(p,v,a)


def overlay_references(baseline_joints, baseline_feet, sample, *, enabled):
    if not enabled: return baseline_joints,baseline_feet
    feet=list(baseline_feet); feet[sample.swing_side]=sample.foot_model
    return overlay_joint_jet(baseline_joints,sample,enabled=True),tuple(feet)


def overlay_reference_context(context, sample, *, enabled):
    """Uninstalled adapter for the existing frozen ReferenceContext dataclasses.

    Call only after evaluate() and the existing controller's native admissions.
    The owner must version/configure this opt-in hook in its causal receipt and
    use the SAME prepared context in actual/hypothetical QP solves. This function
    never observes arms or changes clocks, origins, epoch, gains, or COM.
    """
    if not enabled: return context
    from dataclasses import replace
    keep=[i for i in range(15) if i not in sample.joint_indices]
    if any(np.max(abs(np.asarray(a)[keep]-np.asarray(b)[keep]))>3e-8
            for a,b in zip(context.joints.arrays(),sample.base_frame.joints.arrays())):
        raise ReferenceRejected('context baseline differs from prepared root/stance frame joints')
    joints=overlay_joint_jet(context.joints,sample,enabled=True)
    joint_type=type(context.joints)
    new_joints=joint_type(joints.position,joints.velocity,joints.acceleration)
    position,R,omega,alpha=sample.foot_source(); feet=list(context.feet)
    old=feet[sample.swing_side]
    # The source has already prepared nominal foot AND contact-seek/braking
    # once. Never erase that causal jet by silently substituting a nominal foot.
    differences=[float(np.max(abs(np.asarray(a)-np.asarray(b)))) for a,b in
        zip(old.translation.arrays(),position.arrays())]
    differences.extend([float(np.max(abs(np.asarray(old.orientation.rotation).reshape(3,3)-R))),
        float(np.max(abs(np.asarray(old.orientation.velocity)-omega))),
        float(np.max(abs(np.asarray(old.orientation.acceleration)-alpha)))])
    if max(differences)>3e-8:
        raise ReferenceRejected('prepared context foot/seek jet differs from matched IK foot; forward the original seek jet',maximumJetError=max(differences))
    translation=type(old.translation)(position.position,position.velocity,position.acceleration)
    orientation=replace(old.orientation,rotation=frozen(R),velocity=frozen(omega),acceleration=frozen(alpha))
    feet[sample.swing_side]=replace(old,translation=translation,orientation=orientation)
    # replace() preserves the exact measured arms object and other untouched
    # immutable values. No generic future-FK arm field is ever copied in.
    return replace(context,joints=new_joints,feet=tuple(feet))
