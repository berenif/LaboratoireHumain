"""Pure stance-realized whole-cycle references; no native state or optimizer.

Nine independent coordinates (lumbar + stance leg) own a C2 path. The other six
coordinates are an implicit exact rigid-foot continuation, not interpolated IK
knots. Root is ALWAYS stance @ inverse(relative stance FK). Runtime arms remain
the caller's already-prepared measured target; mapped arms predict geometry only.
"""
from dataclasses import dataclass, replace
import json, math
import numpy as np
from scipy.spatial.transform import Rotation
from coupled_posture_reference import (Jet, C2Segment, OrientationJet, angular_chart,
    angular_world, CurrentSupport, digest, MAX_TOTAL_COM_SPEED, MAX_TOTAL_COM_ACCELERATION)
from coupled_posture_kinematics import (MatrixJet, ModelFK, P, frozen,
    pose_jet_from_source, pose_jet_to_source)
from phase_leg_reference import FKJets, AdvanceWindow

FEET=('left_foot','right_foot')
LEG_INDICES=(tuple(range(9,15)),tuple(range(3,9)))
KNEES=(12,6)
PHASES=('lift','translate','land')
DURATIONS=(.8,1.2,1.)
VERSION='stance-realized-wholebody-cycle-v1'

def add(a,b):return Jet(*(x+y for x,y in zip(a.arrays(),b.arrays())))
def subtract(a,b):return Jet(*(x-y for x,y in zip(a.arrays(),b.arrays())))
def as_jet(value):return Jet(value.position,value.velocity,value.acceleration)
def pose_chart(value):
    p=pose_jet_to_source(value)
    return angular_chart(np.array(p.rotation).reshape(3,3),p.angular_velocity_world,p.angular_acceleration_world)
def chart_difference(a,b):
    p,v,h=(x-y for x,y in zip(a.arrays(),b.arrays()))
    return Jet(np.arctan2(np.sin(p),np.cos(p)),v,h)
def compose_pose(translation,angles):return pose_jet_from_source(translation,*angular_world(angles))
def rest_orientation(body,R):return OrientationJet(body,frozen(R),(0.,)*3,(0.,)*3)

class PathRejected(ValueError):
    def __init__(self,reason,**receipt):
        super().__init__(reason);self.receipt=dict(accepted=False,reason=reason,**receipt)

class StanceRealizedIK:
    """Reusable independent9 -> exact other-foot15 analytic jet operation.

    Position Newton has fixed branch seed, tolerance, cap and iteration bound;
    there is no scipy optimization, history, last-call seed or state mutation.
    It never clips a position into an accepted source bound. Bounds are admitted
    separately on the returned answer. The root is not an independent variable.
    """
    def __init__(self,kinematics,stance_side,stance_pose,arm_map,leg_origin):
        if stance_side not in (0,1):raise ValueError('source foot order left/right')
        self.kin=kinematics;self.model=ModelFK.from_model(kinematics.model)
        self.fk=FKJets(kinematics);self.stance_side=stance_side;self.swing_side=1-stance_side
        self.stance_pose=stance_pose;self.arm_map=arm_map;self.leg_origin=tuple(leg_origin)
        self.swing_indices=np.array(LEG_INDICES[self.swing_side])
        self.independent_indices=np.array([i for i in range(15)if i not in self.swing_indices])

    def root_for(self,lower):
        full=Jet(*(np.r_[x,np.zeros(6)]for x in lower.arrays()))
        return self.stance_pose@self.model.forward(full)[FEET[self.stance_side]].inverse()

    def position(self,independent9,target_pose,seed_lower15,*,allow_unconverged=False):
        q=np.array(seed_lower15,float,copy=True)
        if q.shape!=(15,)or np.shape(independent9)!=(9,):raise ValueError('independent9 and seed15 required')
        q[self.independent_indices]=independent9
        # Positional root does not depend on the swing leg or any arm coordinate.
        relative=self.kin.forward(np.r_[q,np.zeros(6)],np.eye(4))
        root=self.stance_pose.T@np.linalg.inv(relative[FEET[self.stance_side]])
        desired=target_pose.T
        for iteration in range(32):
            poses=self.kin.forward(np.r_[q,np.zeros(6)],root);T=poses[FEET[self.swing_side]]
            residual=np.r_[desired[:3,3]-T[:3,3],
                Rotation.from_matrix(desired[:3,:3]@T[:3,:3].T).as_rotvec()]
            if np.max(abs(residual))<2e-11:break
            J=self.fk.spatial_jacobian(np.r_[q,np.zeros(6)],MatrixJet.rest(root),
                FEET[self.swing_side],self.swing_indices)
            if np.linalg.cond(J)>1e9:
                if allow_unconverged:break
                raise PathRejected('implicit swing-foot branch singular',iteration=iteration)
            delta=np.linalg.solve(J,residual)
            # Trust radius changes the iteration, never the requested foot target.
            delta*=min(1.,.3/max(np.max(abs(delta)),1e-30))
            q[self.swing_indices]+=delta
        else:
            if not allow_unconverged:raise PathRejected('fixed Newton branch did not converge',maximumPoseResidual=float(max(abs(residual))))
        # Re-evaluate after the last step, including an unaccepted prescreen only.
        T=self.kin.forward(np.r_[q,np.zeros(6)],root)[FEET[self.swing_side]]
        residual=np.r_[desired[:3,3]-T[:3,3],Rotation.from_matrix(desired[:3,:3]@T[:3,:3].T).as_rotvec()]
        if max(abs(residual))>=2e-11 and not allow_unconverged:raise PathRejected('fixed Newton branch did not converge',maximumPoseResidual=float(max(abs(residual))))
        return q,dict(iterations=iteration,maximumPoseResidual=float(max(abs(residual))),
            converged=bool(max(abs(residual))<2e-11),callOrderIndependent=True,clipping=False,optimizerCalls=0)

    def derive(self,independent9,target_pose,seed_lower15,*,runtime_arms=None):
        """Return lower Jet15, FKResult, Newton receipt, using existing FK jets."""
        ip,iv,ia=independent9.arrays()
        q,receipt=self.position(ip,target_pose,seed_lower15)
        v=np.zeros(15);a=np.zeros(15);v[self.independent_indices]=iv;a[self.independent_indices]=ia
        root=self.root_for(Jet(q,v,a))
        full=lambda p,r,s:Jet(np.r_[p,np.zeros(6)],np.r_[r,np.zeros(6)],np.r_[s,np.zeros(6)])
        bias=self.model.forward(full(q,v,a),root)[FEET[self.swing_side]]
        J=self.fk.spatial_jacobian(np.r_[q,np.zeros(6)],root,FEET[self.swing_side],self.swing_indices)
        target=pose_jet_to_source(target_pose);current=pose_jet_to_source(bias)
        # Existing Jacobian is model-axis spatial translation/angular velocity.
        rhs=np.r_[P.T@(np.array(target.translation.velocity)-current.translation.velocity),
            P.T@(np.array(target.angular_velocity_world)-current.angular_velocity_world)]
        v[self.swing_indices]=np.linalg.solve(J,rhs)
        root=self.root_for(Jet(q,v,a))
        bias2=pose_jet_to_source(self.model.forward(full(q,v,a),root)[FEET[self.swing_side]])
        rhs2=np.r_[P.T@(np.array(target.translation.acceleration)-bias2.translation.acceleration),
            P.T@(np.array(target.angular_acceleration_world)-bias2.angular_acceleration_world)]
        a[self.swing_indices]=np.linalg.solve(J,rhs2)
        lower=Jet(q,v,a);result=self.model.coupled(lower,self.stance_side,self.stance_pose,self.arm_map,self.leg_origin)
        if runtime_arms is not None:result=self.with_arms(result,as_jet(runtime_arms))
        actual=pose_jet_to_source(result.pose(FEET[self.swing_side]))
        receipt.update(maximumVelocityResidual=float(max(abs(np.r_[np.array(actual.translation.velocity)-target.translation.velocity,
            np.array(actual.angular_velocity_world)-target.angular_velocity_world]))),
            maximumAccelerationResidual=float(max(abs(np.r_[np.array(actual.translation.acceleration)-target.translation.acceleration,
            np.array(actual.angular_acceleration_world)-target.angular_acceleration_world]))))
        return lower,result,receipt

    def with_arms(self,result,arms):
        full=Jet(*(np.r_[a[:15],b]for a,b in zip(result.full_joints.arrays(),arms.arrays())))
        poses=self.model.forward(full,result.root_model)
        com=[np.zeros(3),np.zeros(3),np.zeros(3)]
        for part in self.model.parts:
            for order,M in enumerate(poses[part.name].arrays()):
                com[order]+=part.mass*(P@(M@np.r_[part.com,1.])[:3])/self.model.total_mass
        return replace(result,full_joints=full,com_source=Jet(*com),poses_model=tuple(poses.items()))


@dataclass(frozen=True)
class CycleSample:
    lower_joints: Jet
    com: Jet
    thorax: OrientationJet
    pelvis: OrientationJet
    feet: tuple
    geometric: object
    diagnostics: dict


class WholeBodyCycle:
    """Frozen flight proposal plus explicit causal settle boundary.

    Current prepared stationary TRANSFER is the runtime construction contract.
    The saved first-lift fixture is accepted only with design_only=True and its
    already elapsed .00125s is subtracted from the .8s original lift clock.
    """
    def __init__(self,realizer,context,source_limits,heading,independent_knots,
            *,design_only=False,entry_elapsed=0.,lumbar_window_deg=5.):
        if lumbar_window_deg is not None and (type(lumbar_window_deg) not in (int,float)
                or not math.isfinite(lumbar_window_deg) or lumbar_window_deg<=0.):
            raise ValueError('explicit positive experimental lumbar window or None required')
        self.lumbar_window_deg=None if lumbar_window_deg is None else float(lumbar_window_deg)
        if not design_only:
            config=json.loads(context.config_json);causal=json.loads(context.causal_json)
            if (context.phase!='transfer' or context.elapsed<config['phase_durations']['transfer']-1e-10
                    or any(context.com.velocity) or any(context.com.acceleration)):
                raise ValueError('runtime activation must follow completed stationary nominal/corrective transfer')
            correction=causal.get('robustCorrection',{})
            if correction.get('selected') and tuple(correction.get('clock',()))!=(1.,0.,0.):
                raise ValueError('original causal correction has not completed')
        self.realizer=realizer;self.entry=context;self.limits=np.array(source_limits,float,copy=True)
        self.limits.setflags(write=False)
        self.swing_side=realizer.swing_side;self.stance_side=realizer.stance_side
        self.q0=as_jet(context.joints);self.lower0=Jet(*(x[:15]for x in self.q0.arrays()))
        if any(self.lower0.velocity)or any(self.lower0.acceleration):raise ValueError('declared stationary activation family requires rest lower jets')
        self.entry_elapsed=float(entry_elapsed);self.durations=(.8-entry_elapsed,1.2,1.)
        if self.durations[0]<=0.:raise ValueError('lift clock already finished')
        self.heading=np.array(heading,float,copy=True);self.heading.setflags(write=False);self.design_only=bool(design_only)
        if self.heading.shape!=(3,)or abs(np.linalg.norm(self.heading)-1.)>1e-8 or abs(self.heading[2])>1e-12:raise ValueError('unit horizontal heading required')
        lower=np.array(self.lower0.position);idx=realizer.independent_indices
        knots=np.asarray(independent_knots,float)
        if knots.shape!=(3,9):raise ValueError('one frozen 3x9 flight horizon required')
        self.independent_knots=knots.copy();self.independent_knots.setflags(write=False)
        points=[lower[idx],*knots];self.independent_segments=tuple(C2Segment.between(0.,T,Jet.rest(a),Jet.rest(b))for a,b,T in zip(points,points[1:],self.durations))
        raw=realizer.model.coupled(self.lower0,self.stance_side,realizer.stance_pose,realizer.arm_map,realizer.leg_origin)
        # Geometric prediction gets a declared initial map/actual-arm bridge.
        arm_mismatch=subtract(as_jet(context.arms),Jet(*(x[15:]for x in raw.full_joints.arrays())))
        self.arm_slack=C2Segment.between(0.,self.durations[0],arm_mismatch,Jet.rest(np.zeros(6)))
        entry_real=realizer.with_arms(raw,as_jet(context.arms))
        self.com_slack=C2Segment.between(0.,self.durations[0],subtract(as_jet(context.com),entry_real.com_source),Jet.rest(np.zeros(3)))
        body=next(o for o in context.orientations if o.body==1)
        initial_body=angular_chart(*body.arrays());raw_body=angular_chart(*entry_real.thorax.arrays())
        self.body_slack=C2Segment.between(0.,self.durations[0],chart_difference(initial_body,raw_body),Jet.rest(np.zeros(3)))
        pelvis=next((o for o in context.orientations if o.body==4),None)
        initial_pelvis_slack=(Jet.rest(np.zeros(3))if pelvis is None else
            chart_difference(angular_chart(*pelvis.arrays()),angular_chart(*entry_real.pelvis.arrays())))
        self.pelvis_slack=C2Segment.between(0.,self.durations[0],initial_pelvis_slack,Jet.rest(np.zeros(3)))
        self.foot_slacks=[];self.entry_foot_charts=[]
        for side,foot in enumerate(context.feet):
            actual=pose_jet_to_source(raw.pose(FEET[side]));desired=angular_chart(*foot.orientation.arrays())
            translation=C2Segment.between(0.,self.durations[0],subtract(as_jet(foot.translation),actual.translation),Jet.rest(np.zeros(3)))
            rotation=C2Segment.between(0.,self.durations[0],chart_difference(desired,pose_chart(raw.pose(FEET[side]))),Jet.rest(np.zeros(3)))
            self.foot_slacks.append((translation,rotation));self.entry_foot_charts.append(desired)
        self.foot_slacks=tuple(self.foot_slacks);self.entry_foot_charts=tuple(self.entry_foot_charts)
        swing=context.feet[self.swing_side];p=np.array(swing.translation.position)
        high=p+np.array([0.,0.,.04]);end=high+.182*self.heading
        # Original source floor minus .5mm landing contact preference.
        landed=end.copy();landed[2]=p[2]-.0005
        self.foot_segments=(C2Segment.between(0.,self.durations[0],as_jet(swing.translation),Jet.rest(high)),
            C2Segment.between(0.,1.2,Jet.rest(high),Jet.rest(end)),
            C2Segment.between(0.,1.,Jet.rest(end),Jet.rest(landed)))
        key_data=dict(version=VERSION,context=context.receipt()['contextSHA256'],knots=knots.tolist(),
            stance=realizer.stance_pose.position,limits=self.limits.tolist(),entryElapsed=entry_elapsed)
        if self.lumbar_window_deg!=5.:key_data['lumbarWindowDeg']=self.lumbar_window_deg
        self.key=digest(key_data)

    def clock(self,phase,elapsed):
        aliases={'lift_wait':('lift',self.durations[0]),'translate_wait':('translate',1.2),
            'land_wait':('land',1.),'settle_handoff':('land',1.)}
        if phase=='transfer':return 0,0.,0.
        if phase in ('settle','stand','quiet'):raise PathRejected('causal measured same-foot settle plan required before quiet')
        if phase in aliases:phase,elapsed=aliases[phase]
        if phase not in PHASES:raise ValueError('unknown cycle phase')
        k=PHASES.index(phase);t=max(0.,min(float(elapsed),self.durations[k]))
        return k,t,sum(self.durations[:k])+t

    def command_foot(self,side,k,t,total,landing_adjustment=None):
        if side==self.swing_side:
            translation=self.foot_segments[k].at(t)
            if landing_adjustment is not None:
                values=tuple(float(x)for x in landing_adjustment)
                if len(values)!=3 or not np.isfinite(values).all():raise ValueError('owned contact-seek depth/rate/acceleration required')
                translation=subtract(translation,Jet(*([0.,0.,x]for x in values)))
            return compose_pose(translation,self.entry_foot_charts[side])
        # Stance geometry stays actual and fixed. Its original finite task mismatch
        # decays continuously; no actual stance transform is replaced.
        p=pose_jet_to_source(self.realizer.stance_pose)
        st,sr=self.foot_slacks[side]
        return compose_pose(add(p.translation,st.at(total)),add(pose_chart(self.realizer.stance_pose),sr.at(total)))

    def sample(self,phase,elapsed,*,runtime_arms=None,landing_adjustment=None):
        k,t,total=self.clock(phase,elapsed)
        command=self.command_foot(self.swing_side,k,t,total,landing_adjustment)
        tr=pose_jet_to_source(command);st,sr=self.foot_slacks[self.swing_side]
        geometric_foot=compose_pose(subtract(tr.translation,st.at(total)),subtract(pose_chart(command),sr.at(total)))
        independent=self.independent_segments[k].at(t)
        lower,result,newton=self.realizer.derive(independent,geometric_foot,self.lower0.position)
        mapped=Jet(*(x[15:]for x in result.full_joints.arrays()))
        predicted_arms=add(mapped,self.arm_slack.at(total))
        result=self.realizer.with_arms(result,predicted_arms if runtime_arms is None else as_jet(runtime_arms))
        R,w,a=angular_world(add(angular_chart(*result.thorax.arrays()),self.body_slack.at(total)))
        thorax=OrientationJet(1,frozen(R),frozen(w),frozen(a))
        R,w,a=angular_world(add(angular_chart(*result.pelvis.arrays()),self.pelvis_slack.at(total)))
        pelvis=OrientationJet(4,frozen(R),frozen(w),frozen(a))
        feet=tuple(self.command_foot(side,k,t,total,landing_adjustment)for side in (0,1))
        return CycleSample(lower,add(result.com_source,self.com_slack.at(total)),thorax,pelvis,feet,result,
            dict(**newton,armConditioning='predicted future map plus initial bridge'if runtime_arms is None else'exact prepared measured delayed arms',
                runtimeArmsReplaced=False,all21TasksExactlyCompatibleClaim=False,
                initialCompatibilitySlackActive=total<self.durations[0],phase=phase,phaseTimeS=t))

    def validate_sample(self,sample,current_support=None,*,require_clearance=False):
        full=np.array(sample.geometric.full_joints.position);q=full[:15]
        margins=np.minimum(full-self.limits[:,0],self.limits[:,1]-full)
        lumbar=(margins[:3]-.01 if self.lumbar_window_deg is None else
            math.radians(self.lumbar_window_deg)-abs(q[:3]-np.array(self.lower0.position)[:3]))
        delta=np.array(sample.geometric.future_arm_delta_right_left.position);ranges=np.array(self.realizer.arm_map.ranges)
        mapmargin=np.minimum(delta-ranges[:,0],ranges[:,1]-delta)-.02
        clearance=self.realizer.kin.bottom(FEET[self.swing_side],sample.geometric.pose(FEET[self.swing_side]).T)
        p,v,a=sample.com.arrays()
        checks=dict(lowerJointInterior=bool(min(margins[:15])>=.01-1e-9),armReserve=bool(min(margins[15:])>=.02-1e-9),
            mapReserve=bool(min(mapmargin)>=-1e-9),lumbar=bool(min(lumbar)>=-1e-9),noHyperextension=bool(max(q[[6,12]])<=1e-10),
            speed=bool(np.linalg.norm(v)<=MAX_TOTAL_COM_SPEED+1e-10),acceleration=bool(np.linalg.norm(a)<=MAX_TOTAL_COM_ACCELERATION+1e-10),
            clearance=bool(not require_clearance or clearance>=.025-1e-9))
        support_receipt=None
        if current_support is not None:
            try:support_receipt=current_support.validate_com(sample.com);checks['supportCapture']=True
            except ValueError:checks['supportCapture']=False
        return dict(accepted=all(checks.values()),checks=checks,minimumLowerReserveRad=float(min(margins[:15])),
            minimumArmReserveRad=float(min(margins[15:])),minimumMapSlackRad=float(min(mapmargin)),minimumLumbarSlackRad=float(min(lumbar)),
            plannedWholeFootClearanceM=float(clearance),commandSpeedMps=float(np.linalg.norm(v)),commandAccelerationMps2=float(np.linalg.norm(a)),support=support_receipt)

    def prepare_context(self,context,current_support,*,advance_window=None):
        """Only task owner; must be called once before actual/prospective QPs.

        Native causal history, unload, contact seek, land readiness, motors,
        collision/rays and measured clearance remain external original owners.
        """
        current_support.validate_identity(context)
        if context.step_index!=self.entry.step_index or context.epoch!=self.entry.epoch:
            raise PathRejected('wrong step or reference epoch for frozen whole-body path')
        if context.phase=='translate':
            if advance_window is None:raise PathRejected('original actual 80-sample advance window required')
            advance_window.validate()
        if context.source_step==self.entry.source_step and context.source_time==self.entry.source_time:
            current_support.validate_com(as_jet(context.com));return context
        seek=json.loads(context.causal_json).get('contactSeekAfter',{})
        adjustment=seek.get('jet',(0.,0.,0.))
        sample=self.sample(context.phase,context.elapsed,runtime_arms=context.arms,landing_adjustment=adjustment)
        check=self.validate_sample(sample,current_support,require_clearance=context.phase=='translate')
        if not check['accepted']:raise PathRejected('whole-body command admission failed',sample=check)
        # The lower owner does not rewrite the six existing weak arm-joint rows
        # either. Strong arm target/gain/history objects remain exactly intact.
        joints=type(context.joints).make(*(np.r_[a,b[15:]]for a,b in zip(sample.lower_joints.arrays(),context.joints.arrays())))
        factory=type(context.orientations[0]).make
        # Existing body1/body4 gains and weights remain exactly prepared. Adding
        # a new body4 row is an explicit outer-adapter opt-in, never silent here.
        targets={1:sample.thorax,4:sample.pelvis}
        orientations=tuple(factory(o.body,*targets[o.body].arrays(),kp=o.kp,kd=o.kd,weight=o.weight)
            if o.body in targets else o for o in context.orientations)
        feet=[]
        for old,pose in zip(context.feet,sample.feet):
            s=pose_jet_to_source(pose)
            # Contact-seek is already part of the original prepared foot command.
            # Preserve its owned difference from nominal at the adapter boundary.
            orient=type(old.orientation).make(old.orientation.body,np.array(s.rotation).reshape(3,3),s.angular_velocity_world,s.angular_acceleration_world,
                kp=old.orientation.kp,kd=old.orientation.kd,weight=old.orientation.weight)
            feet.append(type(old)(type(old.translation).make(*s.translation.arrays()),orient))
        diagnostic=json.loads(context.diagnostics_json);diagnostic['wholeBodyCycle']=dict(version=VERSION,pathSHA256=self.key,sample=check,**sample.diagnostics)
        return replace(context,joints=joints,com=type(context.com).make(*sample.com.arrays()),orientations=orientations,feet=tuple(feet),
            diagnostics_json=json.dumps(diagnostic,sort_keys=True,separators=(',',':')))

    def boundary_lower(self):return self.sample('settle_handoff',0.).lower_joints

    def receipt(self):
        return dict(version=VERSION,pathSHA256=self.key,independentIndices=self.realizer.independent_indices.tolist(),
            independentKnotsRad=self.independent_knots.tolist(),phaseDurationsS=self.durations,
            initialContextSHA256=self.entry.receipt()['contextSHA256'],runtimeActivationRequiresStationaryTransfer=True,
            savedFirstLiftIsOfflineDesignOnly=self.design_only,freeCOM=True,stanceRealizedRoot=True,arbitraryRoot=False,
            runtimeArmsRemainPreparedMeasured=True,compatibilitySlackOnCompleteCommand=True,
            experimentalLumbarWindowDeg=self.lumbar_window_deg,experimentalLumbarIsPhysicalLimit=False,
            lumbarOriginalPhysicalInteriorRad=.01,
            nativeAdmission=False,physicsIntegrations=0,stateInstallations=0,
            settle='causal measured same-current-foot endpoint/path must be separately admitted; no hybrid q0 return',
            wholeNonlinearPathProof=False,requiresEveryCommandCurrentSupportAndDerivativeChecks=True)
