#!/usr/bin/env python3
"""Algebraic diagnostic from saved observations; no world restore or integration.
Float64 translation of the RNEA root balance, not an exact Rapier replay.
"""
import hashlib,json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
source=Path(sys.argv[1]); raw=source.read_bytes(); trial=json.loads(raw)
o=trial['result']['finalObservation']; step=o['ownership']['step']; ref=step['reference']
model=json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())['segments']
bodies=o['bodies']; ids={b['id']:i for i,b in enumerate(bodies)}
v=lambda d:np.array([d[k] for k in ('x','y','z')],dtype=float)
def rot(q):
 q=np.array(q,dtype=float);q=q/np.linalg.norm(q);x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
pos=np.array([b['position'] for b in bodies]); R=[rot(b['rotation']) for b in bodies]
w=np.array([b['angularVelocityRadps'] for b in bodies]);mass=np.array([b['massKg'] for b in bodies])
q=np.array(o['q']);dq=np.array(o['dq']);rq=np.array(ref['q']);rv=np.array(ref['dq']);target=np.array(ref['target']);dt=.00125;T=.8
bounds=np.zeros((21,2))
for p in model:
 h=p['reference']['hinge']
 if h:
  a=p['jointProfile']['axes'][0];bounds[h['qposIndex']]=[a['minRadians'],a['maxRadians']]
distance=np.where(rv>0,bounds[:,1]-rq,rq-bounds[:,0]);brake=bool(ref['braking'] or np.any((rv!=0)&(distance<=rv*rv/4+abs(rv)*dt+1e-6)))
if brake:
 t=np.minimum(dt,abs(rv)/2);acc=-np.sign(rv)*2;cq=rq+rv*t+.5*acc*t*t;cv=np.where(t<dt,0,rv+acc*t);ca=np.clip((cv-rv)/dt,-2,2)
else:
 t=min(ref['elapsed']+dt,T);x=t/T;delta=target-np.array(ref['start']);vel=np.array(ref['velocity'])
 cq=np.array(ref['start'])+delta*(10*x**3-15*x**4+6*x**5)+vel*T*(x-6*x**3+8*x**4-3*x**5)
 cv=delta*(30*x*x-60*x**3+30*x**4)/T+vel*(1-18*x*x+32*x**3-15*x**4)
 ca=delta*(60*x-180*x*x+120*x**3)/T**2+vel*(-36*x+96*x*x-60*x**3)/T
acc=np.clip(ca+16*(cq-q)+8*(cv-dq),-2,2)
parents=[None if p['parent'] is None else ids[p['parent']] for p in model]
def accelerations(root_linear,root_angular):
 lin=np.zeros((len(model),3));ang=lin.copy();lin[0]=root_linear;ang[0]=root_angular
 for i in range(1,len(model)):
  p=parents[i];h=model[i]['reference']['hinge']
  if h:
   frame=model[i]['jointProfile']['parentFrame'];axis=R[p]@rot([frame['rotation'][k] for k in ('x','y','z','w')])@np.array([1.,0,0]);axis/=np.linalg.norm(axis)
   anchor=pos[p]+R[p]@v(frame['anchor']);a=anchor-pos[p];b=pos[i]-anchor;j=h['qposIndex']
   ang[i]=ang[p]+axis*acc[j]+np.cross(w[p],axis*dq[j])
   lin[i]=lin[p]+np.cross(ang[p],a)+np.cross(w[p],np.cross(w[p],a))+np.cross(ang[i],b)+np.cross(w[i],np.cross(w[i],b))
  else:
   a=pos[i]-pos[p];ang[i]=ang[p];lin[i]=lin[p]+np.cross(ang[p],a)+np.cross(w[p],np.cross(w[p],a))
 return lin,ang
retained=ids['right_foot'];bias,alpha=accelerations(np.zeros(3),np.zeros(3));root_ang=-alpha[retained];root_lin=-bias[retained]-np.cross(root_ang,pos[retained]-pos[0]);lin,ang=accelerations(root_lin,root_ang)
force=np.zeros((len(model),3));moment=force.copy()
for i,p in enumerate(model):
 offset=R[i]@v(p['goldenMass']['centerOfMass']);I=R[i]@np.array(p['goldenMass']['inertia'])@R[i].T
 force[i]=(lin[i]+np.cross(ang[i],offset)+np.cross(w[i],np.cross(w[i],offset))+np.array([0,9.81,0]))*mass[i]
 moment[i]=I@ang[i]+np.cross(w[i],I@w[i])+np.cross(offset,force[i])
for i in reversed(range(1,len(model))):
 p=parents[i];moment[p]+=moment[i]+np.cross(pos[i]-pos[p],force[i]);force[p]+=force[i]
desired=np.r_[force[0],moment[0]];points=[];candidates=[]
# Protocol floor upper face is y=0; recorded extrema are independent of solver impulses.
for foot in o['floorFootGeometry']['feet']:
 i=foot['body']
 for point in foot['points']:
  point=np.array(point);offset=point-pos[i];normal_acc=(lin[i]+np.cross(ang[i],offset)+np.cross(w[i],np.cross(w[i],offset)))[1]
  used=bool(point[1]<=0 and -1e-4<=normal_acc<=1e-4)
  candidates.append({'body':bodies[i]['id'],'point':point.tolist(),'gap_m':point[1],'normal_acceleration_m_s2':normal_acc,'used':used})
  if used:points.append(point)
A=np.zeros((6,len(points)*3))
for k,p in enumerate(points):
 for j in range(3):
  e=np.eye(3)[j];A[j,k*3+j]=1;A[3:,k*3+j]=np.cross(p-pos[0],e)
u,s,vh=np.linalg.svd(A,full_matrices=True);rank=int(np.linalg.matrix_rank(A));projection=u[:,rank:].T@desired
allocated=A.T@np.linalg.solve(A@A.T+np.eye(6)*1e-6,desired)
residual=A@allocated-desired
out={'scope':__doc__,'source_sha256':hashlib.sha256(raw).hexdigest(),'floor_assumption_m':0,'braking_proposal':brake,
 'requested_wrench_world_about_torso_origin_N_Nm':desired.tolist(),'candidates':candidates,'rank':rank,'singular_values':s.tolist(),
 'condition_nonzero_spectrum':float(s[0]/s[rank-1]) if rank else None,'unrepresentable_wrench_projection_N_Nm':projection.tolist(),
 'allocated_point_forces_N_float64_regularized':allocated.reshape(-1,3).tolist(),'residual_N_Nm_float64_regularized':residual.tolist(),
 'residual_mixed_norm':float(np.linalg.norm(residual)),'unregularized_least_squares_residual':float(np.linalg.norm(A@np.linalg.lstsq(A,desired,rcond=None)[0]-desired)),
 'reference_acceleration_rad_s2':ca.tolist(),'tracking_acceleration_rad_s2':(16*(cq-q)+8*(cv-dq)).tolist(),'desired_joint_acceleration_rad_s2':acc.tolist(),'measured_dq_minus_previous_ref_dq_rad_s':(dq-rv).tolist(),'all_geometric_candidates_count':len(candidates),'admitted_controls':o['ownership']['last_application']['controls'],
 'exact_runtime_wrench_allocation_residual_saved':False,'exact_entry_world_saved':False}
print(json.dumps(out,indent=2))
