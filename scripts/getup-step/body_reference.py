"""Offline whole-body candidate timeline. FK targets only, never physics writes.

Sparse force-optimized witnesses are NOT recorded teachers. This generator uses
only the repository's pure kinematics, manifold-corrects both-foot phases, and
adds an explicitly scoped actual-twist entry bridge. No QP controller is used.
"""
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from reference import native_pose_to_source_qpos
from source_math import P

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('pure_step_fk',ROOT/'scripts/getup-step-reference.py')
fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)


def proper_rotation(matrix):
    u,_,v=np.linalg.svd(matrix);r=u@v
    if np.linalg.det(r)<0:u[:,-1]*=-1;r=u@v
    return r


def smooth_clock(t,duration):
    if t<=0:return 0.,0.,0.
    if t>=duration:return 1.,0.,0.
    u=t/duration
    return 10*u**3-15*u**4+6*u**5,(30*u**2-60*u**3+30*u**4)/duration,(60*u-180*u**2+120*u**3)/duration**2


def skew(x):
    a,b,c=x;return np.array([[0,-c,b],[c,0,-a],[-b,a,0]])


def left_jacobian(phi):
    theta=np.linalg.norm(phi);k=skew(phi)
    if theta<1e-6:return np.eye(3)+.5*k+k@k/6
    return np.eye(3)+(1-np.cos(theta))/theta**2*k+(theta-np.sin(theta))/theta**3*k@k


def left_jacobian_dot(phi,velocity):
    e=1e-5
    return (left_jacobian(phi+e*velocity)-left_jacobian(phi-e*velocity))/(2*e)


class QuinticBridge:
    def __init__(self,p0,v0,a0,p1,v1,a1,duration):
        self.duration=duration
        if not np.isfinite(duration)or duration<=0:raise ValueError('invalid bridge duration')
        p0,v0,a0,p1,v1,a1=map(lambda x:np.asarray(x,dtype=float),(p0,v0,a0,p1,v1,a1))
        if not all(x.shape==p0.shape and np.isfinite(x).all()for x in [p0,v0,a0,p1,v1,a1]):raise ValueError('invalid bridge endpoints')
        t=duration;self.coeff=np.array([p0,v0,a0/2,*np.linalg.solve(np.array([[t**3,t**4,t**5],[3*t*t,4*t**3,5*t**4],[6*t,12*t*t,20*t**3]]),np.array([p1-p0-v0*t-a0*t*t/2,v1-v0-a0*t,a1-a0]))])
    def sample(self,t):
        if not 0<=t<=self.duration:raise ValueError('bridge time outside interval')
        c=self.coeff
        return sum(c[i]*t**i for i in range(6)),sum(i*c[i]*t**(i-1)for i in range(1,6)),sum(i*(i-1)*c[i]*t**(i-2)for i in range(2,6))


class BodyReference:
    DURATIONS={'prepare':3.,'lift':2.,'advance':2.,'lower':2.,'acquire':1.,'settle':3.}
    def __init__(self,plan,acquisition,model,*,dense_per_segment=33):
        self.plan=plan;self.kin=fk.Kinematics(model);self.parts=model['segments'];self.mass=sum(p['massKg']for p in self.parts)
        self.right=np.array(plan['actualRightFootFixedT']);self.initial_q=np.array(plan['pathKnots'][0]['q']);self.initial_root=np.array(plan['pathKnots'][0]['root'])
        self.initial_velocity=np.array(plan['initialActualFinalGeneralizedVelocity']);self.limits=self.kin.limits.astype(np.float32).astype(float)
        lower=self.limits[20,0];actual=self.initial_q[20]
        if actual!=float(np.nextafter(np.float32(lower),np.float32(-np.inf)))or self.initial_velocity[26]>=0:
            raise ValueError('entry elbow exception is restricted to the verified adjacent-f32 closing seed')
        self.exception=dict(joint=20,actualPosition=actual,actualVelocity=float(self.initial_velocity[26]),referencePosition=float(lower),referenceVelocity=0.,positionDifference=float(lower-actual),velocityDifference=float(-self.initial_velocity[26]),physicalStateChanged=False,plannedStopReactionCredited=False)
        rows={r['phase']:r for r in plan['pathKnots']};acq=acquisition['pathKnots']
        seq=[('prepare',[rows['actual-start'],rows['prepare']]),('lift',[rows['prepare'],rows['lift']]),('advance',[rows['lift'],rows['translate-mid'],rows['translate-full']]),('lower',[rows['translate-full'],rows['landing']]),('acquire',acq),('settle',[acq[-1],rows['placed-two-foot-stand']])]
        self.phases={};self.wall=0.;self.maximum_dense_ik_error=0.
        for name,anchors in seq:
            w=np.linspace(0,1,len(anchors));q=np.array([r['q']for r in anchors]);q[:,20]=lower
            seed=PchipInterpolator(w,q,axis=0);pos=PchipInterpolator(w,np.array([r['leftFoot']for r in anchors])[:,:3,3],axis=0)
            rot=Slerp(w,Rotation.from_matrix([proper_rotation(np.array(r['leftFoot'])[:3,:3])for r in anchors]));knots=np.linspace(0,1,dense_per_segment);dense=[]
            for u in knots:
                value=seed(u);root=self.root_from_q(value);goal_p=pos(u);goal_r=rot(u).as_matrix()
                def residual(leg):
                    z=value.copy();z[9:15]=leg;foot=self.kin.forward(z,root)['left_foot']
                    return np.r_[1000*(foot[:3,3]-goal_p),100*Rotation.from_matrix(goal_r.T@proper_rotation(foot[:3,:3])).as_rotvec()]
                fit=least_squares(residual,value[9:15],bounds=(self.limits[9:15,0]+1e-6,self.limits[9:15,1]-1e-6),xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=60)
                if not fit.success or np.max(abs(residual(fit.x)))>1e-4:raise ValueError('reference IK closure failed')
                value[9:15]=fit.x;self.maximum_dense_ik_error=max(self.maximum_dense_ik_error,float(np.max(abs(residual(fit.x)))));dense.append(value)
            self.phases[name]=dict(curve=PchipInterpolator(knots,np.array(dense),axis=0),position=pos,rotation=rot,start=self.wall,duration=self.DURATIONS[name]);self.wall+=self.DURATIONS[name]
        self.duration=self.wall
        q1,root1=self.regular_geometry('prepare',.5);dq1,ddq1=self.joint_motion('prepare',.5)
        q0=self.initial_q.copy();q0[20]=lower;v0=self.initial_velocity[6:].copy();v0[20]=0.
        self.joint_bridge=QuinticBridge(q0,v0,np.zeros(21),q1,dq1,ddq1,.5)
        p1,v1,a1,w1,alpha1=self.root_motion('prepare',.5)
        r0=proper_rotation(self.initial_root[:3,:3]);r1=proper_rotation(root1[:3,:3]);phi1=Rotation.from_matrix(r1@r0.T).as_rotvec()
        phiv1=np.linalg.solve(left_jacobian(phi1),w1);phia1=np.linalg.solve(left_jacobian(phi1),alpha1-left_jacobian_dot(phi1,phiv1)@phiv1)
        self.position_bridge=QuinticBridge(self.initial_root[:3,3],self.initial_velocity[:3],np.zeros(3),p1,v1,a1,.5)
        self.rotation_bridge=QuinticBridge(np.zeros(3),self.initial_velocity[3:6],np.zeros(3),phi1,phiv1,phia1,.5)
        self.r0=r0;self.root_scale=np.cbrt(np.linalg.det(self.initial_root[:3,:3]))
    def root_from_q(self,q):return self.right@np.linalg.inv(self.kin.forward(q,np.eye(4))['right_foot'])
    def regular_geometry(self,phase,t):
        item=self.phases[phase];u,_,_=smooth_clock(t,item['duration']);q=item['curve'](u);return q,self.root_from_q(q)
    def joint_motion(self,phase,t):
        item=self.phases[phase];u,r,a=smooth_clock(t,item['duration']);return item['curve'].derivative()(u)*r,item['curve'].derivative(2)(u)*r*r+item['curve'].derivative()(u)*a
    def root_motion(self,phase,t):
        e=1e-4;q,root=self.regular_geometry(phase,t);dq,ddq=self.joint_motion(phase,t)
        # Local joint Taylor jet matches the selected PCHIP span at C1 knots.
        poses={i:self.root_from_q(q+i*e*dq+.5*(i*e)**2*ddq)for i in [-2,-1,1,2]}
        r=proper_rotation(root[:3,:3]);first=(poses[1][:3,3]-poses[-1][:3,3])/(2*e);second=(poses[1][:3,3]-2*root[:3,3]+poses[-1][:3,3])/e**2
        w=Rotation.from_matrix(proper_rotation(poses[1][:3,:3])@proper_rotation(poses[-1][:3,:3]).T).as_rotvec()/(2*e)
        wm=Rotation.from_matrix(r@proper_rotation(poses[-2][:3,:3]).T).as_rotvec()/(2*e);wp=Rotation.from_matrix(proper_rotation(poses[2][:3,:3])@r.T).as_rotvec()/(2*e)
        return root[:3,3],first,second,w,(wp-wm)/(2*e)
    def geometry(self,phase,t):
        if phase=='prepare'and t<=.5:
            q,_,_=self.joint_bridge.sample(t);p,_,_=self.position_bridge.sample(t);phi,_,_=self.rotation_bridge.sample(t);root=np.eye(4);root[:3,3]=p;root[:3,:3]=Rotation.from_rotvec(phi).as_matrix()@self.r0*self.root_scale
            return q,root
        return self.regular_geometry(phase,t)
    def sample(self,phase,t):
        q,root=self.geometry(phase,t);poses=self.kin.forward(q,root);com=sum(p['massKg']*(poses[p['id']]@np.r_[fk.xyz(p['goldenMass']['centerOfMass']),1])[:3]for p in self.parts)/self.mass
        if phase=='prepare'and t<=.5:
            _,dq,_=self.joint_bridge.sample(t);_,v,_=self.position_bridge.sample(t);phi,phiv,_=self.rotation_bridge.sample(t);omega=left_jacobian(phi)@phiv
        else:
            dq,_=self.joint_motion(phase,t);_,v,_,omega,_=self.root_motion(phase,t)
        r=proper_rotation(root[:3,:3]);e=1e-5
        # COM velocity from a coherent local configuration/root twist jet.
        def com_at(sign):
            tr=root.copy();tr[:3,3]+=sign*e*v;tr[:3,:3]=Rotation.from_rotvec(sign*e*omega).as_matrix()@root[:3,:3]
            ps=self.kin.forward(q+sign*e*dq,tr)
            return sum(p['massKg']*(ps[p['id']]@np.r_[fk.xyz(p['goldenMass']['centerOfMass']),1])[:3]for p in self.parts)/self.mass
        cv=(com_at(1)-com_at(-1))/(2*e);foot=poses['left_foot']
        return dict(q=q,root=root,qpos=native_pose_to_source_qpos(root[:3,3],r,q),qvel=np.r_[P@v,P@r.T@omega,dq],com=P@com,com_ori=P@r.T@np.array([0.,1.,0.]),comVelocity=P@cv,leftFoot=foot,clearance=self.kin.bottom('left_foot',foot),rightFoot=poses['right_foot'])
