#!/usr/bin/env python3
"""Offline attainable stance target; no engine, forces, or pose writes."""
import argparse,hashlib,importlib.util,json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize,LinearConstraint,least_squares,linprog
from scipy.interpolate import PchipInterpolator
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('dyn',ROOT/'scripts/getup-contact-dynamics.py');d=importlib.util.module_from_spec(s);s.loader.exec_module(d)

def static_capacity(c,q,root,contacts,left_share=None,generalized_load=None,velocity=None):
    geom=c.geometry(q,root);M=np.zeros((27,27));gravity=np.zeros(27);F=c.mass*9.81
    for _,mass,point,I,jv,jw in c.body_data(geom):gravity+=jv.T@[0.,mass*9.81,0.]
    gravity[6:]+=c.spring*q
    if generalized_load is not None:gravity=np.array(generalized_load)
    stops=[]
    for i,(lo,hi)in enumerate(c.kin.limits):
        if abs(q[i]-lo)<1e-5 and (velocity is None or (abs(velocity[6+i])<=.05 and velocity[6+i]<=.01)):stops.append((i,1))
        elif abs(q[i]-hi)<1e-5 and (velocity is None or (abs(velocity[6+i])<=.05 and velocity[6+i]>=-.01)):stops.append((i,-1))
    nc=len(contacts);n=21+3*nc+len(stops)+1;A=np.zeros((27,n));A[6:,:21]=-np.diag(c.gears);C=[]
    for k,con in enumerate(contacts):
        J,_=c.point_jacobian(geom,con['body'],con['point']);A[:,21+3*k:24+3*k]=-J.T@con['basis']*F
        for axis in [0,1]:
            for sign in [-1,1]:
                row=np.zeros(n);row[21+3*k+axis]=sign;row[23+3*k]=-.7;C.append(row)
    for k,(i,sign)in enumerate(stops):A[6+i,21+3*nc+k]=-sign*c.gears[i]
    for i in range(21):
        for sign in [-1,1]:
            row=np.zeros(n);row[i]=sign;row[-1]=-1.;C.append(row)
    # Keep the represented-residual feasibility inherited by the physical QP.
    offset=q-c.kin.nominal+(0. if velocity is None else .2*velocity[6:]);bounds=list(zip(-1.-offset,1.-offset))
    for con in contacts:bounds += [(0.,0.)]*3 if left_share==0 and con['body']=='left_foot' else [(-2.,2.),(-2.,2.),(0.,2.)]
    bounds += [(0.,5.)]*len(stops)+[(0.,None)];g=np.zeros(n);g[-1]=1.
    res=linprog(g,A_eq=A,b_eq=-gravity,A_ub=np.array(C),b_ub=np.zeros(len(C)),bounds=bounds,method='highs')
    return dict(feasible=bool(res.success),minimumRequiredMotorFraction=None if not res.success else float(res.x[-1]),motorFractions=None if not res.success else res.x[:21].tolist(),rootAndJointBalanceResidual=None if not res.success else float(max(abs(A@res.x+gravity))),limitReactions=[dict(qpos=i,sign=sign)for i,sign in stops])

def plan(seed,context,source_sha):
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());c=d.ContactDynamics(model);st=c.state(seed['state']['observation'],seed['snapshot'],seed['diagnostic']);ref=c.state(context['observation']['observation'],context['snapshot'],context['diagnostic']);q0=st['q'].copy();geom=st['geometry'];right=geom[0]['right_foot'];left=geom[0]['left_foot'];points=np.array([con['point'][[0,2]] for con in st['contacts']if con['body']=='right_foot']);hull=ConvexHull(points);centroid=np.mean(points[hull.vertices],axis=0)
    # Exact distal-foot moment: affine in single-support COP, before IK validation.
    rows=[];upper=[];moment=np.zeros(3)
    for i in [7,8]:
        axis=geom[1][i];pivot=geom[2][i];moment=np.zeros(3)
        for name,mass,point,*_ in c.body_data(geom):
            if i in c.ancestors[name]:moment+=np.cross(point-pivot,[0.,-mass*9.81,0.])
        def tau(p):return c.spring[i]*q0[i]-axis@(moment+np.cross([p[0]-pivot[0],-pivot[1],p[1]-pivot[2]],[0.,c.mass*9.81,0.]))
        b=tau([0.,0.]);gradient=np.array([tau([1.,0.])-b,tau([0.,1.])-b]);rows.extend([gradient,-gradient]);upper.extend([.8*c.gears[i]-b,.8*c.gears[i]+b])
    rows.extend(hull.equations[:,:2]);upper.extend(-hull.equations[:,2]-.005)
    selection=minimize(lambda p:.5*sum((p-centroid)**2),centroid,jac=lambda p:p-centroid,constraints=[LinearConstraint(np.array(rows),-np.inf,np.array(upper))],method='SLSQP',options=dict(ftol=1e-13,maxiter=100));assert selection.success
    target=ref['com'].copy();target[[0,2]]=selection.x;knots=[];previous=q0[:15].copy();massparts=[p for p in model['segments']if p['massKg']>0]
    def poses_for(z):
        q=q0.copy();q[:15]=z;local=c.kin.forward(q,np.eye(4))['right_foot'];root=right@np.linalg.inv(local);return q,root,c.kin.forward(q,root)
    for w in np.linspace(0.,1.,41):
        wanted=st['com']+w*(target-st['com']);wanted_R=d.normalized_rotation(st['root'][:3,:3])@Rotation.from_rotvec(w*Rotation.from_matrix(d.normalized_rotation(st['root'][:3,:3]).T@d.normalized_rotation(ref['root'][:3,:3])).as_rotvec()).as_matrix()
        def residual(z):
            q,root,poses=poses_for(z);com=sum(p['massKg']*(poses[p['id']]@np.r_[d.reference.xyz(p['goldenMass']['centerOfMass']),1.])[:3]for p in massparts)/c.mass
            return np.r_[(poses['left_foot'][:3,3]-left[:3,3])*1000.,Rotation.from_matrix(d.normalized_rotation(left[:3,:3]).T@d.normalized_rotation(poses['left_foot'][:3,:3])).as_rotvec()*100.,(com-wanted)*1000.,Rotation.from_matrix(wanted_R.T@d.normalized_rotation(root[:3,:3])).as_rotvec()*100.,(z-q0[:15])*.001]
        z=np.clip(previous,c.kin.limits[:15,0]+1e-10,c.kin.limits[:15,1]-1e-10);res=least_squares(residual,z,bounds=(c.kin.limits[:15,0],c.kin.limits[:15,1]),ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=300);previous=res.x;q,root,poses=poses_for(res.x);error=residual(res.x)[:12];com=sum(p['massKg']*(poses[p['id']]@np.r_[d.reference.xyz(p['goldenMass']['centerOfMass']),1.])[:3]for p in massparts)/c.mass
        capacity=static_capacity(c,q,root,st['contacts'],0 if w>=.95 else None);knots.append(dict(weight=float(w),q=q.tolist(),root=root.tolist(),com=com.tolist(),maximumWeightedIKError=float(max(abs(error))),leftFootPositionErrorM=float(max(abs(poses['left_foot'][:3,3]-left[:3,3]))),rootOrientationErrorRad=float(np.linalg.norm(error[9:12]/100.)),capacity=capacity));print('knot',w,'error',max(abs(error)),'motor',capacity['minimumRequiredMotorFraction'],flush=True)
    max_interp_foot=0.;min_nonfoot=float('inf')
    for k in knots:
        poses=c.kin.forward(k['q'],np.array(k['root']))
        for part in model['segments']:
            if part['id'] in ['left_foot','right_foot']:continue
            for cap in part['reference']['primitives']:
                min_nonfoot=min(min_nonfoot,min((poses[part['id']]@np.r_[d.reference.xyz(cap[end]),1.])[1]-cap['radius']for end in ['a','b']))
    for a,b in zip(knots,knots[1:]):
        q=(np.array(a['q'])+np.array(b['q']))/2;local=c.kin.forward(q,np.eye(4))['right_foot'];root=right@np.linalg.inv(local);pose=c.kin.forward(q,root)['left_foot'];max_interp_foot=max(max_interp_foot,float(max(abs(pose[:3,3]-left[:3,3]))))
    distance=float(np.linalg.norm((target-ref['com'])[[0,2]]));ramp=float(np.ceil((1.5*distance/.02)/.025)*.025);curve=PchipInterpolator([k['weight'] for k in knots],np.array([k['q'] for k in knots]),axis=0)
    def reference_at(t):
        u=np.clip(t/ramp,0.,1.);w=float(u*u*(3-2*u));q=curve(w);root=right@np.linalg.inv(c.kin.forward(q,np.eye(4))['right_foot']);return q,root
    dynamic=[];minimum_viability=float('inf');max_runtime_left_error=0.;max_hard_normal_residual=0.
    for ti in np.linspace(0.,ramp,round(ramp*40)+1):
        eps=1e-4;q,root=reference_at(ti);qm,rm=reference_at(ti-eps);qp,rp=reference_at(ti+eps);dq=(qp-qm)/(2*eps);vroot=(rp[:3,3]-rm[:3,3])/(2*eps);omega=Rotation.from_matrix(d.normalized_rotation(rp[:3,:3])@d.normalized_rotation(rm[:3,:3]).T).as_rotvec()/(2*eps);v=np.r_[vroot,omega,dq]
        qmm,rmm=reference_at(ti-2*eps);qpp,rpp=reference_at(ti+2*eps);wm=Rotation.from_matrix(d.normalized_rotation(root[:3,:3])@d.normalized_rotation(rmm[:3,:3]).T).as_rotvec()/(2*eps);wp=Rotation.from_matrix(d.normalized_rotation(rpp[:3,:3])@d.normalized_rotation(root[:3,:3]).T).as_rotvec()/(2*eps);dd=np.r_[(rp[:3,3]-2*root[:3,3]+rm[:3,3])/eps**2,(wp-wm)/(2*eps),(qp-2*q+qm)/eps**2]
        obs=np.array(seed['state']['observation']);obs[:21]=q;obs[[41,42,40]]=v[:3];obs[[44,45,43]]=d.normalized_rotation(root[:3,:3]).T@omega;obs[46:67]=dq;quat=Rotation.from_matrix(d.normalized_rotation(root[:3,:3])).as_quat();snap=dict(segments=[dict(position=dict(zip('xyz',root[:3,3])),rotation=dict(zip('xyzw',quat)))]);state=c.state(obs,snap,seed['diagnostic']);u=np.clip(ti/ramp,0.,1.);w=float(u*u*(3-2*u));cap=static_capacity(c,q,root,st['contacts'],0 if w>=.95 else None,generalized_load=state['M']@dd+state['bias'],velocity=v)
        for j,(low,high)in enumerate(c.kin.limits):
            if min(abs(q[j]-low),abs(q[j]-high))<1e-5:continue
            minimum_viability=min(minimum_viability,high-q[j]-max(dq[j],0.)**2/100.,q[j]-min(dq[j],0.)**2/100.-low)
        runtime_left=c.kin.forward(q,root)['left_foot'];max_runtime_left_error=max(max_runtime_left_error,float(max(abs(runtime_left[:3,3]-left[:3,3]))));normal_residual=max((abs(float(con['basis'][:,2]@(con['J']@dd+con['bias']+20*con['velocity'])))for con in state['contacts']),default=0.);max_hard_normal_residual=max(max_hard_normal_residual,normal_residual)
        contact_acc=max((float(np.linalg.norm(con['J']@dd+con['bias']))for con in state['contacts']),default=0.);contact_speed=max((float(np.linalg.norm(con['J']@v))for con in state['contacts']),default=0.)
        dynamic.append(dict(hardNormalAccelerationRowResidualMps2=normal_residual,maximumContactAccelerationMps2=contact_acc,maximumContactSpeedMps=contact_speed,timeS=float(ti),weight=w,capacity=cap,maxJointVelocityRadps=float(max(abs(dq))),maxJointAccelerationRadps2=float(max(abs(dd[6:])))))
    max_dynamic=max((r['capacity']['minimumRequiredMotorFraction'] or float('inf'))for r in dynamic)
    return dict(plannedContactPointsWorld=[dict(body=con['body'],point=con['point'].tolist(),basis=con['basis'].tolist())for con in st['contacts']],rampS=ramp,dynamicPath=dynamic,maximumRuntimePchipLeftFootErrorM=max_runtime_left_error,maximumPrescribedPathHardNormalResidualMps2=max_hard_normal_residual,prescribedPathInterpretation='approximate inverse-dynamics capacity diagnostic; not an exact hard-normal/stop controller trajectory certificate',prescribedPathMotorCapacityWithin90Percent=all(r['capacity']['feasible']for r in dynamic) and max_dynamic<.9,maximumDynamicMotorFraction=max_dynamic,minimumPathJointStoppingMarginRad=minimum_viability,maximumInterpolatedLeftFootErrorM=max_interp_foot,minimumNonFootCapsuleGroundClearanceM=min_nonfoot,maximumAdjacentJointChangeRad=float(max(abs(np.diff(np.array([k['q']for k in knots]),axis=0)).flat)),mode='static-capacity-interior-stance-target',sourceRuntimeSHA256=source_sha,sourceInitialCOM=ref['com'].tolist(),lockedTargetXZ=target[[0,2]].tolist(),oldHullCentroidXZ=centroid.tolist(),minimumHullMarginM=float(min(-hull.equations[:,:2]@selection.x-hull.equations[:,2])),selectionMotorFractionLimit=.8,pathKnots=knots,physicsIntegrations=0,allKnotIKPassed=all(k['maximumWeightedIKError']<.01 for k in knots),terminalStaticWithin80Percent=knots[-1]['capacity']['feasible']and knots[-1]['capacity']['minimumRequiredMotorFraction']<=.8001)

class FeasibleReferences:
    def __init__(self,plan,controller):
        self.controller=controller;rows=plan['pathKnots'];self.curve=PchipInterpolator([r['weight']for r in rows],np.array([r['q']for r in rows]),axis=0);self.right=controller.kin.forward(rows[0]['q'],np.array(rows[0]['root']))['right_foot']
    def reference(self,progress):
        q=self.curve(float(np.clip(progress,0.,1.)));root=self.right@np.linalg.inv(self.controller.kin.forward(q,np.eye(4))['right_foot']);geom=self.controller.geometry(q,root);com=sum(mass*point for _,mass,point,*_ in self.controller.body_data(geom))/self.controller.mass
        return q,d.normalized_rotation(root[:3,:3]),com


def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=Path,required=True);p.add_argument('--context',type=Path,required=True);p.add_argument('--runtime',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=plan(json.loads(a.seed.read_text()),json.loads(a.context.read_text()),hashlib.sha256(a.runtime.read_bytes()).hexdigest());result.update(plannerSHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),modelSHA256=hashlib.sha256((ROOT/'rust/crates/model/data/getup-v1.json').read_bytes()).hexdigest(),sourceSeedStateSHA256=hashlib.sha256(a.seed.read_bytes()).hexdigest(),sourceReferenceContextSHA256=hashlib.sha256(a.context.read_bytes()).hexdigest());a.output.write_text(json.dumps(result,indent=2)+'\n');print('target',result['lockedTargetXZ'],'IK',result['allKnotIKPassed'],'terminal80%',result['terminalStaticWithin80Percent'])
if __name__=='__main__':main()
