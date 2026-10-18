#!/usr/bin/env python3
"""Bounded joint-torque-only source-engine transfer/liftoff diagnostic.

Original XML, original same-model cache; no physical root force or pose write.
The new C helper exposes read-only mass/Jacobian telemetry and scratch FK jets.
"""
import argparse,ctypes as C,hashlib,json,math,time
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.spatial.transform import Rotation
import clarabel
ROOT=Path(__file__).resolve().parents[2]
PTR=C.POINTER(C.c_double)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ptr(x):return x.ctypes.data_as(PTR)
def clock(t,T):
 u=np.clip(t/T,0,1)
 return 10*u**3-15*u**4+6*u**5,(30*u**2-60*u**3+30*u**4)/T,(60*u-180*u**2+120*u**3)/T**2
class World:
 def __init__(self,base,helper,xml=None):
  self.base=base;self.helper=helper;self.xml=xml or base/'source-model/humanoid_high_freq.xml';self.l=C.CDLL(str(helper));l=self.l
  for name,args,ret in [('lh_open',[C.c_char_p,C.c_char_p,C.c_int],C.c_void_p),('lh_close',[C.c_void_p],None),('lh_dim',[C.c_void_p,C.c_int],C.c_int),('lh_array',[C.c_void_p,C.c_int],PTR),('lh_id',[C.c_void_p,C.c_int,C.c_char_p],C.c_int),('lh_time',[C.c_void_p],C.c_double),('lh_checkpoint_size',[C.c_void_p],C.c_int),('lh_checkpoint_import',[C.c_void_p,C.c_void_p,C.c_int],C.c_int),('lh_checkpoint_export',[C.c_void_p,C.c_void_p,C.c_int],C.c_int),('lh_step_audited',[C.c_void_p,C.c_int],C.c_int),('lh_warnings',[C.c_void_p],C.c_int),('lh_foot_slip_speed',[C.c_void_p,C.c_int],C.c_double),('lh_mass_matrix',[C.c_void_p,PTR],None),('lh_bias',[C.c_void_p,PTR],None),('lh_com_jac',[C.c_void_p,PTR],None),('lh_body_jac',[C.c_void_p,C.c_int,PTR],None),('lh_point_jac',[C.c_void_p,C.c_int,PTR,PTR],None),('lh_geom',[C.c_void_p,C.c_int,PTR],None),('lh_joint_limits',[C.c_void_p,PTR],None),('lh_jdot',[C.c_void_p,C.c_int,PTR],None),('lh_motion_metrics',[C.c_void_p,PTR],None),('lh_support_count',[C.c_void_p],C.c_int),('lh_ground_ray',[C.c_void_p,PTR,PTR,PTR],None),('lh_joint_anchor',[C.c_void_p,C.c_int,PTR],None),('lh_joint_name',[C.c_void_p,C.c_int],C.c_char_p),('lh_joint_body',[C.c_void_p,C.c_int],C.c_int),('lh_set_state',[C.c_void_p,PTR,PTR],None),('lh_geom_friction',[C.c_void_p,C.c_int,PTR],None),('lh_contact_friction',[C.c_void_p,PTR],None)]:
   f=getattr(l,name);f.argtypes=args;f.restype=ret
  e=C.create_string_buffer(1024);self.h=l.lh_open(str(xml or base/'source-model/humanoid_high_freq.xml').encode(),e,1024)
  if not self.h:raise RuntimeError(e.value)
  self.ids={n:l.lh_id(self.h,1,n.encode())for n in ['torso','left_foot','right_foot']}
  self.mass=self.arr(10,(17,)).sum();self.limits=self.read('lh_joint_limits',(21,2));self.gears=self.arr(14,(21,6))[:,0].copy()
 def arr(self,k,shape):return np.ctypeslib.as_array(self.l.lh_array(self.h,k),shape=(math.prod(shape),)).reshape(shape)
 def read(self,name,shape,*args):
  out=np.zeros(shape);getattr(self.l,name)(self.h,*args,ptr(out));return out
 def restore(self,p):
  assert sha(self.xml)==sha(self.base/'source-model/humanoid_high_freq.xml'),'cross-model full-cache restore is forbidden'
  blob=p.read_bytes();binding=b''.join(bytes.fromhex(sha(p))for p in [self.xml,self.base/'source_bridge_v2.so',self.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'])
  assert blob[:8]==b'LHSC002!'and blob[8:104]==binding and hashlib.sha256(blob[136:]).digest()==blob[104:136]
  payload=blob[136:];buf=C.create_string_buffer(payload);assert self.l.lh_checkpoint_import(self.h,buf,len(payload))==len(payload)
  out=C.create_string_buffer(len(payload));assert self.l.lh_checkpoint_export(self.h,out,len(payload))==len(payload);assert out.raw==payload
 def install_cold(self,entry):
  q=np.array(entry['qpos'],dtype=float);v=np.array(entry['qvel'],dtype=float);assert q.shape==(28,)and v.shape==(27,)and self.l.lh_dim(self.h,6)==0
  self.l.lh_set_state(self.h,ptr(q),ptr(v))
 def friction_parameters(self):
  contacts=self.read('lh_contact_friction',(145,));return dict(compiledGeoms=[dict(geomId=i,friction=self.read('lh_geom_friction',(5,),i).tolist())for i in range(20)],effectiveFootGroundContacts=contacts[1:1+9*int(contacts[0])].reshape(-1,9).tolist())
 def checkpoint(self,path):
  size=self.l.lh_checkpoint_size(self.h);out=C.create_string_buffer(size);assert self.l.lh_checkpoint_export(self.h,out,size)==size
  payload=out.raw;binding=b''.join(bytes.fromhex(sha(p))for p in [self.xml,self.base/'source_bridge_v2.so',self.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'])
  path.write_bytes(b'LHSC002!'+binding+hashlib.sha256(payload).digest()+payload)
 def ground_ray(self,origin,direction=(0.,0.,-1.)):
  origin=np.array(origin,dtype=float);direction=np.array(direction,dtype=float);assert np.isfinite(origin).all()and np.isfinite(direction).all()and abs(np.linalg.norm(direction)-1)<1e-12
  hit=self.read('lh_ground_ray',(5,),ptr(origin),ptr(direction));good=bool(int(hit[0])>=0 and 0<=hit[1]<=5.)
  return dict(hit=good,geomId=int(hit[0])if good else None,distanceM=float(hit[1])if good else None,pointSource=hit[2:].tolist()if good else None,originSource=origin.tolist(),directionSource=direction.tolist(),ownBodyExcluded=True)
 def ground_height(self,point):
  origin=np.array(point).copy();origin[2]+=.5;ray=self.ground_ray(origin)
  if not ray['hit']:raise ValueError('no-ground-ray-hit: motion is not admitted')
  return ray['pointSource'][2]
 def joint_ground_rays(self):
  return [dict(joint=self.l.lh_joint_name(self.h,i).decode(),anchorGroundDistanceIsColliderClearance=False,**self.ground_ray(self.read('lh_joint_anchor',(3,),i)))for i in range(22)]
 def feet(self):
  result=[]
  for side in ['left_foot','right_foot']:
   body=self.ids[side];caps=[]
   for i in range(20):
    g=self.read('lh_geom',(17,),i)
    if int(g[0])!=body:continue
    radius,half=g[2:4];position=g[5:8];axis=g[8:].reshape(3,3)[:,2]
    caps.extend([position-axis*half,position+axis*half])
   rays=[dict(capsuleEndpoint=i,**self.ground_ray(point+np.array([0.,0.,.2])))for i,point in enumerate(caps)]
   if not all(ray['hit']for ray in rays):raise ValueError('no-ground-ray-hit-under-foot')
   result.append(dict(body=body,points=np.array(caps),radius=radius,clearance=min(ray['distanceM']-.2-radius for ray in rays),groundRays=rays))
  return result
 def packet(self):
  feet=self.feet();load=self.arr(20,(17,)).copy();footids=[f['body']for f in feet]
  motion=self.read('lh_motion_metrics',(3,));J=self.read('lh_com_jac',(3,27));comv=J@self.arr(1,(27,));q=self.arr(0,(28,))[7:];limits=np.minimum(q-self.limits[:,0],self.limits[:,1]-q)
  count=self.l.lh_support_count(self.h);points=self.arr(21,(128,))[:2*count].reshape(-1,2).copy()
  return dict(jointGroundRays=self.joint_ground_rays(),maximumBodyLinearMps=motion[0],maximumBodyAngularRadps=motion[1],maximumSelfPenetrationM=motion[2],comVelocity=comv.tolist(),torsoTiltDeg=float(np.rad2deg(np.arccos(np.clip(self.arr(5,(17,3,3))[1,2,2],-1,1)))),jointLimitMinimumMarginRad=float(min(limits)),jointLimitMarginsRad=limits.tolist(),actualMotorTorquesNm=self.arr(8,(27,))[6:].copy().tolist(),supportPointsXY=points.tolist(),sourceStep=self.l.lh_dim(self.h,6),sourceTimeS=self.l.lh_time(self.h),qpos=self.arr(0,(28,)).copy().tolist(),qvel=self.arr(1,(27,)).copy().tolist(),com=self.arr(6,(17,3))[1].copy().tolist(),feet=[dict(position=self.arr(3,(17,3))[i].copy().tolist(),groundRays=f['groundRays'],clearance=f['clearance'],loadN=load[i],loadedSlipSpeedMps=self.l.lh_foot_slip_speed(self.h,j))for j,(i,f)in enumerate(zip(footids,feet))],nonfootLoadN=max(load[i]for i in range(1,17)if i not in footids),warnings=self.l.lh_warnings(self.h))
 def close(self):self.l.lh_close(self.h)
class Controller:
 def __init__(self,w,swing_side=0,distance=.062,self_avoidance=False):
  assert swing_side in [0,1]and .02<=distance<=.12
  self.self_avoidance=self_avoidance
  if self_avoidance:w.l.lh_self_separation.argtypes=[C.c_void_p,PTR];w.l.lh_self_separation.restype=None
  self.swing_side=swing_side;self.stance_side=1-swing_side;self.step_distance=distance;self.swing_body=w.ids['left_foot'if swing_side==0 else'right_foot']
  self.w=w;self.q0=w.arr(0,(28,))[7:].copy();self.q0=np.clip(self.q0,w.limits[:,0]+.01,w.limits[:,1]-.01);self.arm_posture=self.q0[15:].copy();self.com0=w.arr(6,(17,3))[1].copy();self.targets=[(w.arr(3,(17,3))[w.ids[s]].copy(),w.arr(5,(17,3,3))[w.ids[s]].copy())for s in ['left_foot','right_foot']];self.rootR=w.arr(5,(17,3,3))[1].copy();self.flatRotations=[]
  for pos,R in self.targets:
   x=R[:,0].copy();x[2]=0;x/=np.linalg.norm(x);self.flatRotations.append(np.column_stack([x,np.cross([0.,0.,1.],x),[0.,0.,1.]]))
  self.radii=[f['radius']for f in w.feet()];self.floorTargets=[w.ground_height(pos)+radius-.0005 for (pos,R),radius in zip(self.targets,self.radii)]
  self.com1=self.com0.copy();self.com1[:2]=self.targets[self.stance_side][0][:2]+self.targets[self.stance_side][1][:2,0]*.035
  delta=self.com1[:2]-self.com0[:2];direction=delta/max(np.linalg.norm(delta),1e-9);self.lean_axis=np.r_[-direction[1]*.14,direction[0]*.14,0.]
  if self.self_avoidance:self.com1[2]-=.015
  self.heading=self.rootR[:,0].copy();self.heading[2]=0;self.heading/=np.linalg.norm(self.heading);self.landing=self.targets[self.swing_side][0]+self.step_distance*self.heading;self.landing[2]=w.ground_height(self.landing)+self.radii[self.swing_side]-.0005;self.finalCom=self.com0.copy();self.finalCom[:2]=(self.landing[:2]+self.targets[self.stance_side][0][:2])/2+.035*self.heading[:2]
  self.B=np.zeros((27,21));order=[1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20]
  for i,j in enumerate(order):self.B[6+j,i]=w.gears[i]
 def solve(self,t,phase='transfer'):
  w=self.w;v=w.arr(1,(27,)).copy();q=w.arr(0,(28,))[7:].copy();M=w.read('lh_mass_matrix',(27,27));bias=w.read('lh_bias',(27,));Jcom=w.read('lh_com_jac',(3,27));com=w.arr(6,(17,3))[1].copy();feet=w.feet();single=phase in ['lift','translate']or(phase=='land'and w.arr(20,(17,))[self.swing_body]<3.);ncontact=4 if single else 8;N=48+ncontact*3
  Aeq=[];beq=[];costA=[];costb=[];ineq=[];ub=[]
  def task(J,target,weight):
   row=np.zeros((len(target),N));row[:,:27]=J;costA.extend(row*weight);costb.extend(np.asarray(target)*weight)
  s,ds,dds=clock(t,2.)
  target=self.com0+(self.com1-self.com0)*s if phase=='transfer'else self.com1
  vel=(self.com1-self.com0)*ds if phase=='transfer'else np.zeros(3);acc=(self.com1-self.com0)*dds if phase=='transfer'else np.zeros(3)
  if phase in ['settle','stand']:
   s,ds,dds=clock(t,2.)if phase=='settle'else(1.,0.,0.);target=self.com1+(self.finalCom-self.com1)*s;vel=(self.finalCom-self.com1)*ds;acc=(self.finalCom-self.com1)*dds
  task(Jcom,acc+45*(target-com)+13*(vel-Jcom@v)-w.read('lh_jdot',(3,),-1),12.)
  Jr=w.read('lh_body_jac',(6,27),1)[3:];R=w.arr(5,(17,3,3))[1];root_target=self.rootR;root_velocity=np.zeros(3);root_acceleration=np.zeros(3)
  if self.self_avoidance:
   lean,lean_v,lean_a=clock(t,2.)if phase=='transfer'else(1.,0.,0.)
   if phase=='settle':lean,lean_v,lean_a=clock(t,2.);lean,lean_v,lean_a=1.-lean,-lean_v,-lean_a
   elif phase=='stand':lean,lean_v,lean_a=0.,0.,0.
   root_target=Rotation.from_rotvec(self.lean_axis*lean).as_matrix()@self.rootR;root_velocity=self.lean_axis*lean_v;root_acceleration=self.lean_axis*lean_a
  rerr=Rotation.from_matrix(root_target@R.T).as_rotvec();task(Jr,root_acceleration+40*rerr+12*(root_velocity-Jr@v)-w.read('lh_jdot',(6,),1)[3:],4.)
  task(np.eye(27)[6:],20*(self.q0-q)-8*v[6:],.2)
  if self.self_avoidance:task(np.eye(27)[21:],20*(self.arm_posture-q[15:])-10*v[21:],1.)
  contacts=[]
  for side,foot in enumerate(feet):
   body=foot['body'];J=w.read('lh_body_jac',(6,27),body);b=w.read('lh_jdot',(6,),body);pos=w.arr(3,(17,3))[body];R=w.arr(5,(17,3,3))[body];pos0,Rinitial=self.targets[side];Rflat=self.flatRotations[side];axis=Rotation.from_matrix(Rflat@Rinitial.T).as_rotvec();pt=pos0.copy();pvel=np.zeros(3);pacc=np.zeros(3);wvel=np.zeros(3);wacc=np.zeros(3);R0=Rflat
   if phase=='transfer':
    sf,dsf,ddsf=clock(t,2.);R0=Rotation.from_rotvec(axis*sf).as_matrix()@Rinitial;wvel=axis*dsf;wacc=axis*ddsf;pt[2]+=(self.floorTargets[side]-pos0[2])*sf;pvel[2]=(self.floorTargets[side]-pos0[2])*dsf;pacc[2]=(self.floorTargets[side]-pos0[2])*ddsf
   else:pt[2]=self.floorTargets[side]
   if side==self.swing_side and phase!='transfer':
    s,ds,dds=clock(t,1.)
    if phase=='lift':pt[2]+=.04*s;pvel[2]=.04*ds;pacc[2]=.04*dds
    elif phase=='translate':pt[2]+=.04;pt+=self.step_distance*self.heading*s;pvel=self.step_distance*self.heading*ds;pacc=self.step_distance*self.heading*dds
    elif phase=='land':pt=self.landing.copy();pt[2]+=.04*(1-s)-.0005*s;pvel[2]=-.0405*ds;pacc[2]=-.0405*dds
    else:pt=self.landing.copy();pt[2]-=.0005
   desired=np.r_[pacc+100*(pt-pos)+20*(pvel-J[:3]@v),wacc+100*Rotation.from_matrix(R0@R.T).as_rotvec()+20*(wvel-J[3:]@v)]-b
   task(J,desired,20. if side==self.swing_side and phase in ['lift','translate','land']else 25.)
   if not single or side==self.stance_side:
    for point in foot['points']:
     p=point.copy();p[2]-=foot['radius'];contacts.append((body,p))
  dynamics=np.zeros((27,N));dynamics[:,:27]=M;dynamics[:,27:48]=-self.B
  for i,(body,p)in enumerate(contacts):
   J=w.read('lh_point_jac',(3,27),body,ptr(p));dynamics[:,48+3*i:51+3*i]=-J.T
   for direction in [np.array([1,0,-.6]),np.array([-1,0,-.6]),np.array([0,1,-.6]),np.array([0,-1,-.6]),np.array([0,0,-1.])]:
    row=np.zeros(N);row[48+3*i:51+3*i]=direction;ineq.append(row);ub.append(0.)
   row=np.zeros(N);row[50+3*i]=1.;costA.append(row*.001);costb.append(w.mass*9.81/ncontact*.001)
   for horizontal in [0,1]:
    shear=np.zeros(N);shear[48+3*i+horizontal]=1.;costA.append(shear*getattr(self,'shear_weight',.1));costb.append(0.)
  if phase=='transfer'and getattr(self,'unload_admitted',False):
   unload=np.zeros(N)
   for i,(body,point)in enumerate(contacts):
    if body==self.swing_body:unload[50+3*i]=1.
   ineq.append(unload);ub.append(0.)
  Aeq.extend(dynamics);beq.extend(-bias)
  for i in range(21):
   row=np.zeros(N);row[27+i]=1;ineq.extend([row,-row]);ub.extend([.95,.95])
  # Predictive joint stopping bounds, with smooth recovery for initial soft-limit elbows.
  for i in range(21):
   row=np.zeros(N);row[6+i]=1;lo,hi=w.limits[i];reserve=getattr(self,'joint_reserve',0.);lo+=reserve;hi-=reserve;h=.12
   ineq.extend([row,-row]);ub.extend([2*(hi-q[i]-h*v[6+i])/h**2,2*(q[i]-lo+h*v[6+i])/h**2])
  ground_adjustments=[]
  for joint,ray in enumerate(w.joint_ground_rays()):
   if not ray['hit']:raise ValueError('no-ground-ray-hit-at-joint-'+ray['joint'])
   body=w.l.lh_joint_body(w.h,joint)
   if body in [w.ids['left_foot'],w.ids['right_foot']]:continue
   anchor=np.array(ray['originSource']);ground_z=anchor[2]-ray['distanceM']
   for geom_id in range(20):
    geom=w.read('lh_geom',(17,),geom_id)
    if int(geom[0])!=body:continue
    kind=int(geom[1]);radius=geom[2];center=geom[5:8];rotation=geom[8:].reshape(3,3)
    if kind==2:point=center-np.array([0.,0.,radius])
    elif kind==3:
     half=geom[3];axis=rotation[:,2];point=center-axis*half*np.sign(axis[2])-np.array([0.,0.,radius])
    else:raise ValueError('unqualified-joint-adjacent-collider')
    clearance=float(point[2]-ground_z);J=w.read('lh_point_jac',(3,27),body,ptr(point));approach=float(J[2]@v)
    active=clearance+.15*min(0.,approach)<.015
    ground_adjustments.append(dict(joint=ray['joint'],body=body,geomId=geom_id,anchorGroundDistanceM=ray['distanceM'],adjacentColliderClearanceM=clearance,verticalApproachMps=approach,active=active))
    if active:
     Jbody=w.read('lh_body_jac',(6,27),body);omega=Jbody[3:]@v;jet=w.read('lh_jdot',(6,),body);r=point-w.arr(3,(17,3))[body];bias_point=jet[:3]+np.cross(jet[3:],r)+np.cross(omega,np.cross(omega,r));minimum_acceleration=-20.*approach-100.*(clearance-.01)-bias_point[2]
     barrier=np.zeros(N);barrier[:27]=-J[2];ineq.append(barrier);ub.append(-minimum_acceleration)
  self_adjustments=[]
  if self.self_avoidance:
   separations=w.read('lh_self_separation',(897,));pairs=separations[1:1+14*int(separations[0])].reshape(-1,14)
   for pair in pairs:
    a,b,ga,gb=map(int,pair[:4]);gap=float(pair[4]);pa=pair[5:8].copy();pb=pair[8:11].copy();normal=pair[11:14];Ja=w.read('lh_point_jac',(3,27),a,ptr(pa));Jb=w.read('lh_point_jac',(3,27),b,ptr(pb));J=normal@(Ja-Jb);approach=float(J@v);active=gap+.15*min(0.,approach)<.025;self_adjustments.append(dict(bodyA=a,bodyB=b,geomA=ga,geomB=gb,separationM=gap,approachMps=approach,active=active))
    if active:
     def point_bias(body,point):
      Jbody=w.read('lh_body_jac',(6,27),body);omega=Jbody[3:]@v;jet=w.read('lh_jdot',(6,),body);r=point-w.arr(3,(17,3))[body];return jet[:3]+np.cross(jet[3:],r)+np.cross(omega,np.cross(omega,r))
     separation_bias=float(normal@(point_bias(a,pa)-point_bias(b,pb)));minimum_acceleration=-20.*approach-100.*(gap-.01)-separation_bias;barrier=np.zeros(N);barrier[:27]=-J;ineq.append(barrier);ub.append(-minimum_acceleration)
  ca=np.array(costA);cb=np.array(costb);P=ca.T@ca+np.diag(np.r_[np.full(27,1e-5),np.full(21,1e-3),np.full(ncontact*3,1e-8)]);linear=-ca.T@cb
  AA=np.vstack([Aeq,ineq]);bb=np.r_[beq,ub];settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=100
  sol=clarabel.DefaultSolver(sparse.csc_matrix(P),linear,sparse.csc_matrix(AA),bb,[clarabel.ZeroConeT(len(beq)),clarabel.NonnegativeConeT(len(ub))],settings).solve()
  if str(sol.status)not in ['Solved','AlmostSolved']:raise RuntimeError('QP '+str(sol.status))
  x=np.array(sol.x);return x[27:48],dict(comTarget=target.tolist(),comErrorM=float(np.linalg.norm(target-com)),predictedForcesN=x[48:].reshape(-1,3).tolist(),maximumMotorFraction=float(np.max(abs(x[27:48]))),dynamicsResidual=float(np.max(abs(dynamics@x+bias))),jointGroundAdjustments=ground_adjustments,activeJointGroundBarriers=sum(row['active']for row in ground_adjustments),selfAvoidance=self_adjustments,activeSelfAvoidanceBarriers=sum(row['active']for row in self_adjustments))
def main():
 from scipy.spatial import ConvexHull,QhullError
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=15.);p.add_argument('--helper',type=Path,required=True);p.add_argument('--xml',type=Path);p.add_argument('--cold-entry',type=Path);p.add_argument('--execute',action='store_true');args=p.parse_args();assert 0<args.seconds<=20.;args.output.mkdir(parents=True,exist_ok=True)
 w=World(args.base,args.helper,xml=args.xml)
 if args.cold_entry:w.install_cold(json.loads(args.cold_entry.read_text()))
 else:w.restore(args.base/'slow-continuation/final.bin')
 c=Controller(w);entry=w.packet();entry['frictionParameters']=w.friction_parameters();json.dumps(entry,allow_nan=False);u,info=c.solve(0);print('zero-step',info,flush=True)
 if not args.execute:w.close();return
 started=time.monotonic();startstep=entry['sourceStep'];slip=np.zeros(2);clearRun=0;maxClear=-1.;rows=[];stop='bounded-end';phase='transfer';phaseStart=0.;events=[];notes=[];maxu=0.;count=0;landRun=0;quietRun=0;quietAnchors=None;minimumPostLandingMargin=None;maxTorqueError=0.;minJointMargin=entry['jointLimitMinimumMarginRad'];maxPen=0.
 try:
  with(args.output/'frames.ndjson').open('x')as f:
   for tick in range(round(args.seconds/.00125)):
    t=tick*.00125;elapsed=t-phaseStart
    assert time.monotonic()-started<180.,'180s wall cap'
    if tick%4==0:
     u,info=c.solve(elapsed,phase);maxu=max(maxu,np.max(abs(u)));notes.append(dict(t=t,phase=phase,**info))
    w.arr(2,(21,))[:]=u;assert w.l.lh_step_audited(w.h,startstep+round(args.seconds/.00125))==startstep+tick+1;count+=1
    row=w.packet();row['elapsedS']=(tick+1)*.00125;row['phase']=phase;row['phaseElapsedS']=elapsed+.00125;slip+=np.array([a['loadedSlipSpeedMps']for a in row['feet']])*.00125;row['loadedSlipDistanceM']=slip.tolist();row['forwardFootDisplacementM']=float((np.array(row['feet'][0]['position'])-c.targets[0][0])@c.heading);maxTorqueError=max(maxTorqueError,float(np.max(abs(np.array(row['actualMotorTorquesNm'])-(c.B@u)[6:]))));row['torqueEquationErrorNm']=maxTorqueError;minJointMargin=min(minJointMargin,row['jointLimitMinimumMarginRad']);maxPen=max(maxPen,row['maximumSelfPenetrationM']);maxClear=max(maxClear,row['feet'][0]['clearance'])
    margin=None
    try:
     hull=ConvexHull(np.unique(np.array(row['supportPointsXY']),axis=0));eq=hull.equations;com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81);margin=float(min(np.min(-(eq[:,:2]@com[:2]+eq[:,2])),np.min(-(eq[:,:2]@capture+eq[:,2]))))
    except (QhullError,ValueError):pass
    row['minimumComCaptureMarginM']=margin
    if phase=='stand':
     positions=np.array([a['position']for a in row['feet']]);quietAnchors=positions.copy()if quietAnchors is None else quietAnchors;drift=float(np.max(np.linalg.norm(positions-quietAnchors,axis=1)));row['standingAnchorDriftM']=drift
     quiet=all(a['loadN']>=3 for a in row['feet'])and sum(a['loadN']for a in row['feet'])>=.8*w.mass*9.81 and row['torsoTiltDeg']<=5 and row['maximumBodyLinearMps']<=.22 and row['maximumBodyAngularRadps']<=.65 and margin is not None and margin>=.003 and .780625<=com[2]<=.829976 and drift<=.01 and row['maximumSelfPenetrationM']<=.005
     quietRun=quietRun+1 if quiet else 0;row['quietConsecutiveS']=quietRun*.00125
     minimumPostLandingMargin=margin if minimumPostLandingMargin is None else min(minimumPostLandingMargin,margin)if margin is not None else minimumPostLandingMargin
    f.write(json.dumps(row,allow_nan=False)+'\n');rows.append(row)
    if tick%200==0:print('progress',row['elapsedS'],phase,[round(a['loadN'],2)for a in row['feet']],[round(a['clearance'],5)for a in row['feet']],slip,'quiet',quietRun*.00125,flush=True)
    if row['warnings']or row['nonfootLoadN']>=3 or all(a['loadN']<3 for a in row['feet']):stop='physical-support-or-warning-guard';break
    if max(slip)>.01:stop='loaded-slip-10mm';break
    if row['maximumSelfPenetrationM']>.005:stop='self-penetration-5mm';break
    if row['com'][2]<.65:stop='height-guard';break
    if maxTorqueError>1e-8:stop='torque-equation-mismatch';break
    nextphase=None
    if phase=='transfer'and elapsed>=2.:
     current=np.array(row['com']);v=w.arr(1,(27,));J=w.read('lh_com_jac',(3,27));capture=current[:2]+(J@v)[:2]*math.sqrt(current[2]/9.81)
     if np.linalg.norm((current-c.com1)[:2])<.012 and np.linalg.norm(capture-c.com1[:2])<.015:nextphase='lift'
    elif phase=='lift':
     clearRun=clearRun+1 if row['feet'][0]['clearance']>=.025 and row['feet'][0]['loadN']<3 and row['feet'][1]['loadN']>=.8*w.mass*9.81 else 0
     if elapsed>=1. and clearRun>=80:nextphase='translate'
    elif phase=='translate'and elapsed>=1.:
     if row['forwardFootDisplacementM']>=.06 and row['feet'][0]['clearance']>=.025 and row['feet'][0]['loadN']<3:nextphase='land'
    elif phase=='land':
     landRun=landRun+1 if row['feet'][0]['loadN']>=3 and row['feet'][1]['loadN']>=3 and row['feet'][0]['clearance']<=.003 and row['forwardFootDisplacementM']>=.06 else 0
     if elapsed>=1. and landRun>=80:nextphase='settle'
    elif phase=='settle'and elapsed>=2.:nextphase='stand'
    elif phase=='stand'and quietRun>=4000 and row['forwardFootDisplacementM']>=.06:stop='measured-forward-step-plus5squiet';break
    if nextphase:
     events.append(dict(timeS=t+.00125,event=nextphase,measured=row));w.checkpoint(args.output/('phase-'+nextphase+'.bin'));phase=nextphase;phaseStart=t+.00125;c.q0=np.clip(w.arr(0,(28,))[7:].copy(),w.limits[:,0]+.002,w.limits[:,1]-.002);print('PHASE',phase,phaseStart,flush=True)
    elif phase!='stand'and elapsed>({'transfer':2.,'lift':1.,'translate':1.,'land':1.,'settle':2.}[phase]+2.):stop='phase-timeout-'+phase;break
 except BaseException as e:stop=repr(e);raise
 finally:
  w.checkpoint(args.output/'final.bin');summary=dict(stopped=stop,actualIntegrations=count,physicsSimSeconds=count*.00125,wallSeconds=time.monotonic()-started,events=events,maximumWholeFootClearanceM=maxClear,wholeFoot25mmWithStanceLoadConsecutiveS=clearRun*.00125,finalQuietConsecutiveS=quietRun*.00125,minimumStandingComCaptureMarginM=minimumPostLandingMargin,loadedSlipDistanceM=slip.tolist(),maximumMotorFraction=maxu,maximumTorqueEquationErrorNm=maxTorqueError,minimumJointLimitMarginRad=minJointMargin,maximumSelfPenetrationM=maxPen,entry=entry,final=None if not rows else rows[-1],torqueOnly=True,rootAssistance=False,physicalStateWritesAfterRestore=0,optimizerUpdates=0,helperSHA256=sha(args.helper),runnerSHA256=sha(__file__),modelSHA256=sha(w.xml),originalWarmStartCacheRestored=args.cold_entry is None,explicitColdInitialPoseAndVelocity=args.cold_entry is not None,modelPath=str(w.xml))
  (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(args.output/'control.json').write_text(json.dumps(notes,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items()if k not in ['events','entry','final']},indent=2),flush=True);w.close()
if __name__=='__main__':main()
