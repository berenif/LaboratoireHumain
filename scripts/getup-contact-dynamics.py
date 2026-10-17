#!/usr/bin/env python3
"""Measured-state floating-base dynamics for ordinary GetUp motor probes.
No dynamics are integrated here; contact/stop forces are optimization variables.
"""
import importlib.util
import itertools
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize, LinearConstraint, Bounds,OptimizeResult
from scipy import sparse
from scipy.spatial import ConvexHull,QhullError

ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('reference',ROOT/'scripts/getup-step-reference.py')
reference=importlib.util.module_from_spec(s);s.loader.exec_module(reference)


def normalized_rotation(r):return r/np.cbrt(np.linalg.det(r))
def skew(v):return np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
def support_projection(points,point):
    """Closest point in the actual solved-contact x/z hull, no added margin."""
    points=np.unique(np.asarray(points,dtype=float),axis=0);point=np.asarray(point,dtype=float)
    assert len(points)
    if len(points)==1:return points[0],float(np.linalg.norm(point-points[0]))
    try:
        hull=ConvexHull(points)
        if max(hull.equations[:,:2]@point+hull.equations[:,2])<=1e-10:return point.copy(),0.
        vertices=points[hull.vertices]
    except QhullError:
        distances=np.sum((points[:,None]-points[None,:])**2,axis=2);a,b=np.unravel_index(np.argmax(distances),distances.shape);vertices=points[[a,b]]
    best=None
    for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
        delta=b-a;t=np.clip((point-a)@delta/max(1e-20,delta@delta),0,1);candidate=a+t*delta;distance=np.linalg.norm(point-candidate)
        if best is None or distance<best[1]:best=(candidate,float(distance))
    return best


def solve_clarabel_qp(H,g,E,b,C,upper,lo,hi):
    """Same convex QP, official Clarabel convention: Ax+s=b, s>=0."""
    import clarabel
    assert clarabel.__version__=='0.11.1','Pinned research backend required'
    if np.any(lo>hi):raise ValueError('An upper bound is less than the corresponding lower bound.')
    n=len(g);I=sparse.eye(n,format='csc');A=sparse.vstack([sparse.csc_matrix(E),sparse.csc_matrix(C),I,-I],format='csc');rhs=np.r_[b,upper,hi,-lo]
    cones=[clarabel.ZeroConeT(len(b)),clarabel.NonnegativeConeT(len(upper)+2*n)]
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=200;settings.tol_gap_abs=1e-8;settings.tol_gap_rel=1e-8;settings.tol_feas=1e-9
    solver=clarabel.DefaultSolver(sparse.triu(sparse.csc_matrix(H)).tocsc(),g,A,rhs,cones,settings);solution=solver.solve();status=str(solution.status)
    return OptimizeResult(x=np.array(solution.x) if solution.x is not None else np.zeros(n),success=status=='Solved',message='Clarabel '+status,nit=solution.iterations)


def separating_float32_neighbor(q,v,lower,upper):
    """One representable neighbor, separating only; never a physical limit edit."""
    assert float(np.float32(lower))==lower and float(np.float32(upper))==upper
    before=float(np.nextafter(np.float32(lower),np.float32(-np.inf)))
    after=float(np.nextafter(np.float32(upper),np.float32(np.inf)))
    return bool((v>0 and before<=q<lower) or (v<0 and upper<q<=after))


def braking_acceleration_bounds(q,v,lower,upper,dt=.025,brake=50.,reserve=0.,native_float32_neighbors=False):
    """Model-relative stopping reserve; no change to native limits or motors."""
    assert dt>0 and brake>0 and 0<=reserve<(upper-lower)/2 and lower<upper
    def upper_stop(a):return q+dt*v+.5*dt*dt*a+max(v+dt*a,0.)**2/(2*brake)
    def lower_stop(a):return q+dt*v+.5*dt*dt*a-max(-v-dt*a,0.)**2/(2*brake)
    roundoff_release=native_float32_neighbors and separating_float32_neighbor(q,v,lower,upper)
    lower_ok=q-max(-v,0.)**2/(2*brake)>=lower-1e-9 or (roundoff_release and v>0)
    upper_ok=q+max(v,0.)**2/(2*brake)<=upper+1e-9 or (roundoff_release and v<0)
    if not (lower_ok and upper_ok):
        raise ValueError(f'Already outside model stopping reserve: q={q},v={v},limits={lower,upper}')
    # Buffer only future stopping positions, never the current physical-state test.
    future_lower=lower+reserve;future_upper=upper-reserve
    if upper_stop(-brake)>future_upper+1e-9 or lower_stop(brake)<future_lower-1e-9:
        raise ValueError('No model-relative braking acceleration interval')
    lo=-brake;hi=brake
    if upper_stop(hi)>future_upper:
        a,b=lo,hi
        for _ in range(60):
            mid=(a+b)/2
            if upper_stop(mid)<=future_upper:a=mid
            else:b=mid
        hi=a
    if lower_stop(lo)<future_lower:
        a,b=-brake,brake
        for _ in range(60):
            mid=(a+b)/2
            if lower_stop(mid)>=future_lower:b=mid
            else:a=mid
        lo=b
    if lo>hi+1e-9:raise ValueError('Empty model-relative braking acceleration interval')
    return lo,hi


class ContactDynamics:
    def __init__(self,model,contact_mode='normal-soft',flat_soles=False,joint_viability=False,qp_solver='slsqp',joint_reserve_margin=0.,force_jacobian='live-geometric'):
        self.kin=reference.Kinematics(model);self.kin.limits=self.kin.limits.astype(np.float32).astype(float);self.parts=model['segments'];self.by_id={p['id']:p for p in self.parts}
        self.mass=model['totalMassKg'];self.gears=self.kin.gears;self.armature=np.zeros(21);self.spring=np.zeros(21);self.damping=np.zeros(21)
        self.ancestors={}
        for p in self.parts:
            ancestors=[] if p['parent'] is None else self.ancestors[p['parent']].copy()
            h=p['reference'].get('hinge')
            if h:
                i=h['qposIndex'];ancestors.append(i);self.armature[i]=h['armature'];self.spring[i]=p['jointProfile']['axes'][0]['passiveStiffnessNmPerRad'];self.damping[i]=p['jointProfile']['axes'][0]['dampingNmsPerRad']
            self.ancestors[p['id']]=ancestors
        assert 0<=joint_reserve_margin<=.001 and (joint_viability or joint_reserve_margin==0.)
        assert force_jacobian in ['live-geometric','native-frozen']
        self.force_jacobian=force_jacobian;self.joint_reserve_margin=joint_reserve_margin;self.previous=None;self.contact_mode=contact_mode;self.flat_soles=flat_soles;self.flat_targets={};self.joint_viability=joint_viability;self.qp_solver=qp_solver
    def set_flat_targets(self,state):
        for foot in ['left_foot','right_foot']:
            R=normalized_rotation(state['geometry'][0][foot][:3,:3]);z=R[:,2].copy();z[1]=0.;z/=np.linalg.norm(z);y=np.array([0.,1.,0.]);self.flat_targets[foot]=np.column_stack([np.cross(y,z),y,z])
    def geometry(self,q,root):
        poses=self.kin.forward(q,root);axes=np.zeros((21,3));pivots=np.zeros((21,3))
        for p in self.parts[1:]:
            h=p['reference'].get('hinge')
            if h:
                parent=poses[p['parent']];rot=normalized_rotation(parent[:3,:3]);frame=self.kin.parent_frames[p['id']];i=h['qposIndex']
                axes[i]=rot@normalized_rotation(frame[:3,:3])[:,0];pivots[i]=parent[:3,3]+rot@frame[:3,3]
        return poses,axes,pivots,root[:3,3]
    def point_jacobian(self,geometry,body,point):
        poses,axes,pivots,rootpos=geometry;jv=np.zeros((3,27));jw=np.zeros((3,27));jv[:,:3]=np.eye(3);jv[:,3:6]=-skew(point-rootpos);jw[:,3:6]=np.eye(3)
        for i in self.ancestors[body]:jv[:,6+i]=np.cross(axes[i],point-pivots[i]);jw[:,6+i]=axes[i]
        return jv,jw
    def body_data(self,geometry):
        result=[]
        for p in self.parts:
            if p['massKg']==0:continue
            pose=geometry[0][p['id']];rot=normalized_rotation(pose[:3,:3]);com=pose[:3,3]+pose[:3,:3]@reference.xyz(p['goldenMass']['centerOfMass']);jv,jw=self.point_jacobian(geometry,p['id'],com)
            result.append((p['id'],p['massKg'],com,rot@np.array(p['goldenMass']['inertia'])@rot.T,jv,jw))
        return result
    def state(self,observation,snapshot,diagnostic):
        obs=np.array(observation);q=obs[:21];root=reference.transform(snapshot['segments'][0]['position'],snapshot['segments'][0]['rotation']);rot=normalized_rotation(root[:3,:3]);v=np.r_[obs[[41,42,40]],rot@obs[[44,45,43]],obs[46:67]]
        geom=self.geometry(q,root);data=self.body_data(geom);eps=1e-5;root2=root.copy();root2[:3,3]+=v[:3]*eps;root2[:3,:3]=Rotation.from_rotvec(v[3:6]*eps).as_matrix()@root[:3,:3]
        geom2=self.geometry(q+v[6:]*eps,root2);data2=self.body_data(geom2);M=np.zeros((27,27));bias=np.zeros(27);Jcom=np.zeros((3,27));com=np.zeros(3);com_bias=np.zeros(3)
        for (_,mass,point,I,jv,jw),(_,_,_,_,jv2,jw2) in zip(data,data2):
            av=(jv2-jv)@v/eps;aw=(jw2-jw)@v/eps;omega=jw@v;M+=mass*jv.T@jv+jw.T@I@jw;bias+=jv.T@(mass*(av+np.array([0.,9.81,0.])))+jw.T@(I@aw+np.cross(omega,I@omega));Jcom+=mass*jv;com+=mass*point;com_bias+=mass*av
        M[6:,6:]+=np.diag(self.armature);bias[6:]+=self.spring*q+self.damping*v[6:];bias[3:6]+=.1*v[3:6];Jcom/=self.mass;com/=self.mass;com_bias/=self.mass
        contacts=[]
        for foot in diagnostic['contactMotion']['feet']:
            if foot['loadN']<3.:continue
            for c in foot['contacts']:
                if c['normalImpulseNs']<=0:continue
                point=reference.xyz(c['point']);normal=reference.xyz(c['normal']);normal/=np.linalg.norm(normal);tangent=np.cross(normal,[0.,0.,1.]);tangent/=np.linalg.norm(tangent);basis=np.column_stack([tangent,np.cross(normal,tangent),normal]);jv,_=self.point_jacobian(geom,foot['body'],point)
                local=np.linalg.solve(geom[0][foot['body']],np.r_[point,1.]);point2=(geom2[0][foot['body']]@local)[:3];jv2,_=self.point_jacobian(geom2,foot['body'],point2)
                force_jacobian=jv;force_point=point
                if self.force_jacobian=='native-frozen':
                    assert 'solverLeverFromCom' in c,'Native frozen solver lever required'
                    pose=geom[0][foot['body']];native_com=pose[:3,3]+normalized_rotation(pose[:3,:3])@reference.xyz(self.by_id[foot['body']]['goldenMass']['centerOfMass']);force_point=native_com+reference.xyz(c['solverLeverFromCom']);force_jacobian=self.point_jacobian(geom,foot['body'],force_point)[0]
                contacts.append(dict(body=foot['body'],point=point,forcePoint=force_point,basis=basis,J=jv,Jforce=force_jacobian,bias=(jv2-jv)@v/eps,velocity=jv@v,observedVelocity=reference.xyz(c['velocityMps']),measuredNormalN=c['normalImpulseNs']*800.,gap=c.get('signedGapM')))
        return dict(q=q,root=root,v=v,M=M,bias=bias,Jcom=Jcom,com=com,comVelocity=Jcom@v,comBias=com_bias,contacts=contacts,geometry=geom,geometryNext=geom2,directionalEpsilon=eps,
            comParityErrorM=float(max(abs(com-reference.xyz(diagnostic['com'])))),comVelocityParityErrorMps=float(max(abs(Jcom@v-reference.xyz(diagnostic['comVelocity'])))),contactVelocityParityErrorMps=max((float(max(abs(c['velocity']-c['observedVelocity']))) for c in contacts),default=0.))
    def solve(self,state,target_com,target_rotation,posture,unload_left=False,swing=None,_inactive=None,left_force_ceiling_n=None):
        if self.contact_mode=='unilateral-soft' and _inactive is None:
            assert len(state['contacts'])<=8,'Bounded exhaustive two-foot contact-mode enumeration only'
            assert all(c['gap'] is not None for c in state['contacts']),'Fresh paired gaps required'
            best=None;attempts=[];warm=self.previous
            for bits in itertools.product([False,True],repeat=len(state['contacts'])):
                inactive=tuple(i for i,b in enumerate(bits) if b)
                if len(inactive)==len(bits):continue
                self.previous=warm
                try:action,info=self.solve(state,target_com,target_rotation,posture,unload_left,swing,inactive,left_force_ceiling_n)
                except (ValueError,AssertionError) as error:
                    attempts.append(dict(inactive=list(inactive),error=str(error)));continue
                attempts.append(dict(inactive=list(inactive),feasible=info['constraintFeasible'],optimizerSuccess=info['optimizerSuccess']))
                if info['success'] and (best is None or info['objectiveCost']<best[1]['objectiveCost']):best=(action,info,self.previous.copy())
            if best is None:
                self.previous=warm
                return np.zeros(21),dict(success=False,error='No admissible contact mode',contactModeAttempts=attempts)
            self.previous=best[2];best[1]['contactModeAttempts']=attempts
            return best[:2]
        q=state['q'];v=state['v'];contacts=state['contacts'];nc=len(contacts);assert nc>0
        inactive=set(() if _inactive is None else _inactive)
        active=[]
        for i,(low,high) in enumerate(self.kin.limits):
            if swing is not None and i==(13 if swing['body']=='left_foot' else 7) and not any(c['body']==swing['body'] for c in contacts):continue
            if abs(v[6+i])>.05:continue
            if abs(q[i]-low)<=1e-4 and v[6+i]<=.01:active.append((i,1.,low))
            elif abs(q[i]-high)<=1e-4 and v[6+i]>=-.01:active.append((i,-1.,high))
        n=48+3*nc+len(active);F=self.mass*9.81;A=np.zeros((27,n));A[:,:27]=state['M'];A[6:,27:48]=-np.diag(self.gears)
        for k,c in enumerate(contacts):A[:,48+3*k:51+3*k]=-c['Jforce'].T@c['basis']*F
        for k,(i,sign,_) in enumerate(active):A[6+i,48+3*nc+k]=-sign*self.gears[i]
        row_scale=np.r_[np.full(3,F),np.full(3,40.),self.gears];eq=[A/row_scale[:,None]];rhs=[-state['bias']/row_scale]
        for c in contacts if self.contact_mode!='unilateral-soft' else []:
            axes=np.eye(3) if self.contact_mode=='hard-sticking' else c['basis'][:,2:3].T
            row=np.zeros((len(axes),n));row[:,:27]=axes@c['J'];eq.append(row);rhs.append(axes@(-c['bias']-20*c['velocity']))
        for i,sign,bound in active:
            row=np.zeros((1,n));row[0,6+i]=1.;eq.append(row);rhs.append(np.array([-20*v[6+i]-100*(q[i]-bound)]))
        E=np.vstack(eq);b=np.concatenate(rhs)
        # Multiple material points on one rigid foot contain a redundant row.
        # Remove only numerical row dependence, retain the original residual.
        U,singular,_=np.linalg.svd(E,full_matrices=False);rank=sum(singular>1e-8)
        Er=U[:,:rank].T@E;br=U[:,:rank].T@b
        dependent_error=float(max(abs(b-U[:,:rank]@br)))
        assert dependent_error<1e-4,dependent_error
        tasks=[];targets=[]
        def task(j,desired,weight):
            row=np.zeros((len(desired),n));row[:,:27]=j;tasks.append(row*weight);targets.append(np.asarray(desired)*weight)
        if self.flat_soles:
            if not self.flat_targets:self.set_flat_targets(state)
            for foot in self.flat_targets:
                if not any(c['body']==foot for c in contacts):continue
                pose=state['geometry'][0][foot];pose2=state['geometryNext'][0][foot];R=normalized_rotation(pose[:3,:3]);target=self.flat_targets[foot];_,jw=self.point_jacobian(state['geometry'],foot,pose[:3,3]);_,jw2=self.point_jacobian(state['geometryNext'],foot,pose2[:3,3]);angular_bias=(jw2-jw)@v/state['directionalEpsilon'];error=.5*sum(np.cross(R[:,i],target[:,i]) for i in range(3));task(jw,50*error-12*(jw@v)-angular_bias,2.)
        if swing is not None:
            foot=swing['body'];pose=state['geometry'][0][foot];pose2=state['geometryNext'][0][foot];jv,jw=self.point_jacobian(state['geometry'],foot,pose[:3,3]);jv2,jw2=self.point_jacobian(state['geometryNext'],foot,pose2[:3,3]);eps=state['directionalEpsilon']
            linear_bias=(jv2-jv)@v/eps;angular_bias=(jw2-jw)@v/eps
            acceleration=100*(np.array(swing['position'])-pose[:3,3])+20*(np.array(swing.get('velocity',[0.,0.,0.]))-jv@v)+np.array(swing.get('acceleration',[0.,0.,0.]))
            task(jv,np.clip(acceleration,-2.,2.)-linear_bias,20.)
            rotation=normalized_rotation(pose[:3,:3]);error=.5*sum(np.cross(rotation[:,i],swing['rotation'][:,i]) for i in range(3))
            task(jw,50*error-12*(jw@v)-angular_bias,2.)
        if self.contact_mode=='normal-soft':
            for c in contacts:
                tangents=c['basis'][:,:2].T
                task(tangents@c['J'],tangents@(-c['bias']-20*c['velocity']),10.)
        if self.contact_mode=='unilateral-soft':
            for k,c in enumerate(contacts):
                if k in inactive:continue
                # Receding-horizon approximation, not the native biased-pass
                # CFM law on a final (bias-free-relaxed) reported velocity.
                task(c['J'],-c['bias']-20*c['velocity'],10.)
        desired_com=np.clip(9*(target_com-state['com'])-6*state['comVelocity'],-.3,.3)
        # COM balance outranks swing tracking: exact-state replay of batch08
        # shows weight10 sacrifices braking while all motor caps remain slack.
        task(state['Jcom'],desired_com-state['comBias'],100.)
        R=normalized_rotation(state['root'][:3,:3]);err=.5*sum(np.cross(R[:,i],target_rotation[:,i]) for i in range(3));jrot=np.zeros((3,27));jrot[:,3:6]=np.eye(3)
        task(jrot,25*err-10*v[3:6],2.)
        jpost=np.zeros((21,27));jpost[:,6:]=np.eye(21);task(jpost,np.clip(16*(posture-q)-8*v[6:],-5.,5.),.15)
        B=np.vstack(tasks);d=np.concatenate(targets);H=B.T@B+np.eye(n)*1e-6;g=-B.T@d
        if unload_left:
            row=np.zeros(n)
            for k,c in enumerate(contacts):
                if c['body']=='left_foot':row[50+3*k]=1.
            H+=25.*float(unload_left)**2*np.outer(row,row)
        H[27:48,27:48]+=np.eye(21)*.01
        for k,c in enumerate(contacts):
            H[48+3*k:51+3*k,48+3*k:51+3*k]+=np.eye(3)*.005;g[50+3*k]-=.005*c['measuredNormalN']/F
        lo=np.full(n,-50.);hi=np.full(n,50.);lo[:3]=-2.;hi[:3]=2.;lo[3:6]=-10.;hi[3:6]=10.
        offset=q-self.kin.nominal+.2*v[6:];lo[27:48]=np.maximum(-.9,-1.-offset);hi[27:48]=np.minimum(.9,1.-offset)
        for i,(lower,upper) in enumerate(self.kin.limits):
            if i in [row[0] for row in active]:continue
            # Constant-acceleration 25ms reachability guard; actual Native
            # unilateral limits remain authoritative, not replaced by this QP.
            lo[6+i]=max(lo[6+i],2*(lower-q[i]-.025*v[6+i])/.025**2)
            hi[6+i]=min(hi[6+i],2*(upper-q[i]-.025*v[6+i])/.025**2)
            if self.joint_viability:
                try:brake_lo,brake_hi=braking_acceleration_bounds(q[i],v[6+i],lower,upper,reserve=self.joint_reserve_margin,native_float32_neighbors=True)
                except ValueError as error:raise ValueError(f'Joint {i}: {error}') from error
                lo[6+i]=max(lo[6+i],brake_lo);hi[6+i]=min(hi[6+i],brake_hi)
        for k in range(nc):lo[48+3*k:51+3*k]=[-2.,-2.,0.];hi[48+3*k:51+3*k]=[2.,2.,2.]
        for k in inactive:lo[48+3*k:51+3*k]=0.;hi[48+3*k:51+3*k]=0.
        lo[48+3*nc:]=0.;hi[48+3*nc:]=5.
        inequalities=[]
        for k in range(nc):
            for a in [0,1]:
                for sign in [-1,1]:
                    row=np.zeros(n);row[48+3*k+a]=sign;row[50+3*k]=-.7;inequalities.append(row)
        upper=[0.]*len(inequalities)
        if self.contact_mode=='unilateral-soft':
            for k,c in enumerate(contacts):
                normal=c['basis'][:,2];row=np.zeros(n);row[:27]=.5*.025**2*(normal@c['J']);base=c['gap']+.025*(normal@c['velocity'])+.5*.025**2*(normal@c['bias'])
                if k in inactive:
                    inequalities.append(-row);upper.append(base)
                else:
                    inequalities.extend([row,-row]);upper.extend([-base,base-min(c['gap'],0.)])
        gap_end=len(inequalities)
        if left_force_ceiling_n is not None:
            assert np.isfinite(left_force_ceiling_n) and left_force_ceiling_n>=0.,'Invalid left normal force ceiling'
            row=np.zeros(n)
            for k,c in enumerate(contacts):
                if c['body']=='left_foot':row[50+3*k]=1.
            inequalities.append(row);upper.append(left_force_ceiling_n/F)
        C=np.array(inequalities);upper=np.array(upper);constraints=[LinearConstraint(Er,br,br),LinearConstraint(C,-np.inf,upper)]
        start=np.clip(np.zeros(n) if self.previous is None or len(self.previous)!=n else self.previous,lo,hi)
        if self.qp_solver=='clarabel':result=solve_clarabel_qp(H,g,Er,br,C,upper,lo,hi)
        else:result=minimize(lambda z:.5*z@H@z+g@z,start,jac=lambda z:H@z+g,bounds=Bounds(lo,hi),constraints=constraints,method='SLSQP',options={'maxiter':80,'ftol':1e-8,'disp':False})
        z=result.x;eq_error=float(max(abs(E@z-b)));inequality_error=float(max(C@z-upper));friction_error=float(max((C@z-upper)[:4*nc]));gap_error=float(max((C@z-upper)[4*nc:gap_end],default=0.));load_error=float(max((C@z-upper)[gap_end:],default=0.)*F);bounds_error=float(max(np.max(lo-z),np.max(z-hi)));self.previous=z
        braking_margins=[]
        if self.joint_viability:
            for i,(lower,upper) in enumerate(self.kin.limits):
                if i in [row[0] for row in active]:continue
                qnext=q[i]+.025*v[6+i]+.5*.025**2*z[6+i];vnext=v[6+i]+.025*z[6+i];braking_margins.append(min(upper-qnext-max(vnext,0.)**2/100.,qnext-min(vnext,0.)**2/100.-lower))
        motor=z[27:48]*self.gears;raw=offset+motor/self.gears;represented=np.clip(raw,-1.,1.);predicted=self.gears*np.clip(self.kin.nominal+represented-q-.2*v[6:],-1.,1.)
        return represented,dict(success=bool(result.success and eq_error<1e-4 and inequality_error<1e-6 and bounds_error<1e-6),qpSolver=self.qp_solver,forceJacobianMode=self.force_jacobian,maximumFrozenForcePointOffsetM=max(np.linalg.norm(c['forcePoint']-c['point']) for c in contacts),optimizerMessage=str(result.message),optimizerSuccess=bool(result.success),boundsError=bounds_error,constraintFeasible=bool(eq_error<1e-4 and inequality_error<1e-6 and bounds_error<1e-6),equalityError=eq_error,jointViabilityGuard=self.joint_viability,futureJointStoppingReserveBufferRad=self.joint_reserve_margin,separatingFloat32NeighborReleaseJoints=[i for i,(low,high) in enumerate(self.kin.limits) if self.joint_viability and i not in [row[0] for row in active] and separating_float32_neighbor(q[i],v[6+i],low,high)],jointLimitRepresentation='original model min/max deserialized to native f32, then promoted for analysis; installed physics unchanged',minimumPredictedJointStoppingMarginRad=min(braking_margins,default=None),dependentEqualityError=dependent_error,frictionError=friction_error,contactGapViolationM=gap_error,leftNormalForceCeilingN=left_force_ceiling_n,leftNormalForceCeilingViolationN=load_error,plannedLeftNormalForceN=float(sum(z[50+3*k]*F for k,c in enumerate(contacts) if c['body']=='left_foot')),inactiveContacts=sorted(inactive),predictedContactGapsM=[float(c['gap']+.025*(c['basis'][:,2]@c['velocity'])+.5*.025**2*(c['basis'][:,2]@(c['J']@z[:27]+c['bias']))) if c['gap'] is not None else None for c in contacts],objectiveCost=float(.5*z@H@z+g@z+.5*d@d),desiredComAcceleration=desired_com.tolist(),achievedComAcceleration=(state['Jcom']@z[:27]+state['comBias']).tolist(),qdd=z[:27].tolist(),motorTorquesNm=motor.tolist(),predictedRepresentedTorquesNm=predicted.tolist(),maximumTorqueRepresentationErrorNm=float(max(abs(predicted-motor))),plannedContactForcesWorldN=[(c['basis']@z[48+3*k:51+3*k]*F).tolist() for k,c in enumerate(contacts)],activeStops=[dict(qpos=i,sign=sign,reactionNm=float(z[48+3*nc+k]*self.gears[i])) for k,(i,sign,_) in enumerate(active)],maximumMotorFraction=float(max(abs(motor/self.gears))))
