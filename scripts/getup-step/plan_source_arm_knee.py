#!/usr/bin/env python3
"""Opt-in arms/knees support-constrained reference. No physical integrations.

Based on plan_source_stance, retaining the same COM, contacts, limits and LP.
Only reference posture preferences and a collision-clearance penalty differ.
"""
import argparse,json
import numpy as np
from scipy.optimize import least_squares,linprog
from scipy.spatial.transform import Rotation
from source_step_probe import World,Controller,ROOT,ptr,sha
from source_step_preview import fk,P

def plan_posture(w,side,target_support=False,motor_target=.9,arm_goal=None,knee_goal=None,upright=False,ankle_reserve=None):
 assert side in [0,1]and .5<=motor_target<=.925
 assert sha(w.xml)==sha(w.base/'source-model/humanoid_high_freq.xml'),'planner requires the pinned source model'
 model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());kin=fk.Kinematics(model);parts=model['segments'];actual=w.packet();q=np.array(actual['qpos']);oldq=q[7:].copy();root=np.eye(4);root[:3,:3]=P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P;root[:3,3]=P.T@q[:3];poses=kin.forward(oldq,root);stance='left_foot'if side==0 else'right_foot';free='right_foot'if side==0 else'left_foot';fixed=poses[stance].copy();desired_com=P.T@(Controller(w,1-side,.062,True).com1 if target_support else np.array(actual['com']));desired_R=root[:3,:3].copy();desired_free=poses[free].copy();mass=model['totalMassKg']
 if target_support:
  initial_controller=Controller(w,1-side,.062,True);initial_controller.floorTargets[side]=w.ground_height(initial_controller.targets[side][0])+initial_controller.radii[side]-.00115;initial_controller.floorTargets[1-side]=w.ground_height(initial_controller.targets[1-side][0])+initial_controller.radii[1-side]+.0001;fixed[:3,:3]=P.T@initial_controller.flatRotations[side]@P;fixed[:3,3]=P.T@np.r_[initial_controller.targets[side][0][:2],initial_controller.floorTargets[side]];desired_free[:3,:3]=P.T@initial_controller.flatRotations[1-side]@P;desired_free[:3,3]=P.T@np.r_[initial_controller.targets[1-side][0][:2],initial_controller.floorTargets[1-side]]
 def geometry(angles):
  root=fixed@np.linalg.inv(kin.forward(angles,np.eye(4))[stance]);poses=kin.forward(angles,root);com=sum(p['massKg']*(poses[p['id']]@np.r_[fk.xyz(p['goldenMass']['centerOfMass']),1])[:3]for p in parts)/mass;return root,poses,com
 scratch=World(w.base,w.helper);scratch.install_cold(actual);static_c=Controller(scratch,1-side,.062,True)
 preferred=oldq.copy();weights=np.r_[np.full(15,.05),np.full(6,.4)]
 if arm_goal is not None:preferred[15:]=arm_goal;weights[15:]=1.2
 if knee_goal is not None:preferred[[6,12]]=knee_goal;weights[[6,12]]=.6
 def residual(angles):
  root,poses,com=geometry(angles);rerr=Rotation.from_matrix(root[:3,:3]@desired_R.T).as_rotvec();ferr=Rotation.from_matrix(poses[free][:3,:3]@desired_free[:3,:3].T).as_rotvec()
  Rs=P@root[:3,:3]@P.T;quat=Rotation.from_matrix(Rs).as_quat();scratch.install_cold(dict(qpos=np.r_[P@root[:3,3],quat[[3,0,1,2]],angles].tolist(),qvel=[0.]*27));F=np.array([0.,0.,mass*9.81]);position=scratch.arr(3,(17,3))[scratch.ids[stance]];wrench=np.r_[F,np.cross(P@com-position,F)];tau=scratch.read('lh_bias',(27,))-scratch.read('lh_body_jac',(6,27),scratch.ids[stance]).T@wrench;u=np.linalg.lstsq(static_c.B,tau,rcond=None)[0]
  raw=scratch.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14);collision=max(.025-min(pairs[:,4],default=1.),0.)
  upright_errors=[]
  if upright:
   tilt_weight=3. if upright is True else float(upright)
   for name in ('torso','pelvis'):
    body_R=P@poses[name][:3,:3]@P.T;up=body_R[:,2];upright_errors.extend(np.cross(up,[0.,0.,1.])[:2]*tilt_weight)
    # Preserve the measured axial heading as well. Tilt-only optimization
    # can otherwise trade a large spinal twist for lower leg motor demand.
    body=w.l.lh_id(w.h,1,name.encode());initial_R=w.arr(5,(17,3,3))[body]
    initial_yaw=np.arctan2(initial_R[1,0],initial_R[0,0]);yaw=np.arctan2(body_R[1,0],body_R[0,0])
    upright_errors.append(np.arctan2(np.sin(yaw-initial_yaw),np.cos(yaw-initial_yaw))*2.)
  return np.r_[(com-desired_com)*20,(poses[free][:3,3]-desired_free[:3,3])*50,ferr*3,rerr*.7,(angles-preferred)*weights,collision*20,np.maximum(abs(u)-motor_target,0.)*5,upright_errors]
 lower=w.limits[:,0]+.02;upper=w.limits[:,1]-.02
 if ankle_reserve is not None:
  assert upright and .02<=ankle_reserve<=.1
  lower[[7,13]]=w.limits[[7,13],0]+ankle_reserve;upper[[7,13]]=w.limits[[7,13],1]-ankle_reserve
 try:fit=least_squares(residual,np.clip(oldq,lower+1e-6,upper-1e-6),bounds=(lower,upper),max_nfev=200,ftol=1e-11,xtol=1e-11,gtol=1e-11)
 finally:scratch.close()
 newroot,newposes,com=geometry(fit.x);Rs=P@newroot[:3,:3]@P.T;quat=Rotation.from_matrix(Rs).as_quat();qpos=np.r_[P@newroot[:3,3],quat[[3,0,1,2]],fit.x];witness=dict(qpos=qpos.tolist(),qvel=np.zeros(27).tolist());clone=World(w.base,w.helper)
 try:
  clone.install_cold(witness);c=Controller(clone,1-side,.062,True);body=clone.ids[stance];foot=clone.feet()[side];J=np.hstack([clone.read('lh_point_jac',(3,27),body,ptr(point-np.array([0.,0.,foot['radius']]))).T for point in foot['points']]);B=c.B;bias=clone.read('lh_bias',(27,));Aeq=np.zeros((27,34));Aeq[:,:21]=B;Aeq[:,21:33]=J;inequality=[];bounds=[]
  for i in range(21):
   a=np.zeros(34);a[i]=1;a[-1]=-1;inequality.append(a);a=a.copy();a[i]=-1;inequality.append(a)
  for i in range(4):
   for x,y in [(1,0),(-1,0),(0,1),(0,-1)]:
    a=np.zeros(34);a[21+3*i]=x;a[22+3*i]=y;a[23+3*i]=-.6;inequality.append(a)
  # Only the strict LP admits a reference; the relaxed LP is diagnostic only.
  objective=np.zeros(34);objective[-1]=1;bounds=[(-.95,.95)]*21+[(None,None),(None,None),(0,mass*9.81)]*4+[(0,.95)];static=linprog(objective,A_ub=np.array(inequality),b_ub=np.zeros(len(inequality)),A_eq=Aeq,b_eq=bias,bounds=bounds,method='highs');relaxed=linprog(objective,A_ub=np.array(inequality),b_ub=np.zeros(len(inequality)),A_eq=Aeq,b_eq=bias,bounds=[(-3.,3.)]*21+[(None,None),(None,None),(0,mass*9.81)]*4+[(0,3.)],method='highs');raw=clone.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14);minimum_self=min(pairs[:,4],default=1.);contact_clearances=[r['distanceM']-.2-clone.feet()[side]['radius']for r in clone.feet()[side]['groundRays']];contact_pose=all(-.003<=x<=.003 for x in contact_clearances);packet=clone.packet();assert np.max(abs(np.array(packet['com'])-P@com))<1e-10
  result=dict(staticOnly=True,physicsIntegrations=0,optimizerSuccess=bool(fit.success),comReferenceSource=(P@com).tolist(),rootReferenceSource=Rs.tolist(),qReference=fit.x.tolist(),qposWitness=qpos.tolist(),comDisplacementM=float(np.linalg.norm(com-desired_com)),swingFootDisplacementM=float(np.linalg.norm(newposes[free][:3,3]-desired_free[:3,3])),rootChangeRad=float(Rotation.from_matrix(newroot[:3,:3]@desired_R.T).magnitude()),minimumJointReserveRad=float(min(np.minimum(fit.x-lower+.02,upper-fit.x+.02))),minimumSelfSeparationM=float(minimum_self),staticMotorContactFeasible=bool(static.success and contact_pose),stanceGroundRayClearancesM=contact_clearances,stanceContactPoseFeasible=contact_pose,minimumRequiredMotorFraction=None if not static.success else float(static.x[-1]),standingFoot=stance,staticStatus=static.message,relaxedStaticStatus=relaxed.message,relaxedMotorFraction=None if not relaxed.success else float(relaxed.x[-1]),relaxedMotorCommands=None if not relaxed.success else relaxed.x[:21].tolist(),relaxedStaticForces=None if not relaxed.success else relaxed.x[21:33].reshape(4,3).tolist(),original=actual,armGoalRad=None if arm_goal is None else np.array(arm_goal).tolist(),kneeGoalRad=None if knee_goal is None else np.array(knee_goal).tolist(),referenceOnlyPreferences=True)
  if upright:
   result['uprightBodyReferencesSource']={str(body):clone.arr(5,(17,3,3))[body].copy().tolist()for body in (1,4)}
   result['uprightPreferenceWeight']=3. if upright is True else float(upright)
   if ankle_reserve is not None:
    result['ankleReferenceReserveRad']=ankle_reserve
    result['minimumJointReserveRad']=float(np.min(np.minimum(fit.x-w.limits[:,0],w.limits[:,1]-fit.x)))
  return result
 finally:clone.close()
