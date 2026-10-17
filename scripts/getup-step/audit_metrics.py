"""Zero-physics decoder for ACTUAL per-native audit frames and task errors.

No reference pose, cached older hull or potential contact is accepted as support.
Undefined point/segment hull margins stay None; they are not invented polygons.
COM uses the native snapshot quaternion action (including recorded norm drift),
whereas orientation errors use normalized SO3 rotations.
"""
import importlib.util
from pathlib import Path
import math,json
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from source_math import P,ORDER
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('task_audit_fk',ROOT/'scripts/getup-step-reference.py');fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)


def support_margin(points,position):
    points=np.asarray(points,dtype=float).reshape(-1,2);position=np.asarray(position,dtype=float)
    if position.shape!=(2,)or not np.isfinite(position).all()or not np.isfinite(points).all():raise ValueError('invalid measured hull geometry')
    points=np.unique(points,axis=0)
    dimension=-1 if len(points)==0 else int(np.linalg.matrix_rank(points-points[0],tol=1e-10))
    if dimension<2:return None,dimension
    hull=ConvexHull(points)
    # scipy equations have unit outward normals; positive distance is INSIDE.
    return float(np.min(-(hull.equations[:,:2]@position+hull.equations[:,2]))),dimension


def decode_native_frame(frame,sample,model):
    json.dumps([frame,sample],allow_nan=False)
    if type(sample.get('native_substep'))is not int or type(frame.get('nativeSubstep'))is not int or frame['nativeSubstep']!=sample.get('native_substep')or frame.get('stage')!='after':raise ValueError('audit/native sample stage or ID mismatch')
    physical=np.asarray(frame['physicalObservation'],dtype=float)
    if physical.shape!=(67,)or not np.isfinite(physical).all():raise ValueError('invalid actual physical observation')
    for key in ['com_height_m','torso_up']:
        if type(sample.get(key))not in(int,float)or not math.isfinite(sample[key]):raise ValueError('missing/nonfinite native '+key)
    parts={p['id']:p for p in model['segments']};snap={p['id']:p for p in frame['snapshot']['segments']}
    if len(frame['snapshot']['segments'])!=len(parts)or set(parts)!=set(snap):raise ValueError('incomplete measured body snapshot')
    if any(sum(p['rotation'][k]**2 for k in'xyzw')<1e-12 for p in snap.values()):raise ValueError('invalid measured quaternion')
    transforms={name:fk.transform(p['position'],p['rotation'])for name,p in snap.items()}
    mass=sum(p['massKg']for p in parts.values());com=sum(p['massKg']*(transforms[name]@np.r_[fk.xyz(p['goldenMass']['centerOfMass']),1])[:3]for name,p in parts.items())/mass
    if not np.isfinite(com).all()or not all(np.isfinite(t).all()for t in transforms.values()):raise ValueError('nonfinite reconstructed native geometry')
    velocity=P.T@physical[37:40];capture=com+velocity*math.sqrt(max(com[1],0.)/9.81)
    by_foot={}
    if len(frame['contactMotion']['feet'])!=2 or {f['body']for f in frame['contactMotion']['feet']}!={'left_foot','right_foot'}:raise ValueError('duplicate/missing actual feet')
    for foot in frame['contactMotion']['feet']:
        by_foot[foot['body']]=[fk.xyz(c['point'])[[0,2]]for c in foot['contacts']if c['normalImpulseNs']>0 and c['normal']['y']>=.65]
    if set(by_foot)!={'left_foot','right_foot'}:raise ValueError('incomplete actual foot contacts')
    right=by_foot['right_foot'];combined=by_foot['left_foot']+right
    enriched=dict(sample)
    for prefix,points in [('right',right),('combined',combined)]:
        for quantity,point in [('com',com),('capture',capture)]:
            margin,dimension=support_margin(points,point[[0,2]]);enriched[f'{prefix}_{quantity}_margin_m']=margin;enriched[f'{prefix}_support_dimension']=dimension
    # These authoritative native-rate fields must match reconstruction tolerance.
    if abs(sample['com_height_m']-com[1])>2e-5 or abs(sample['torso_up']-physical[36])>2e-6:raise ValueError('audit/native COM or torso mismatch')
    qgear=np.zeros(21)
    for part in parts.values():
        h=part['reference'].get('hinge')
        if h:qgear[h['qposIndex']]=h['gear']
    controls=np.array(frame['motorTorquesNmActuatorOrder'])/qgear[ORDER]
    if controls.shape!=(21,)or not np.isfinite(controls).all()or np.max(abs(controls))>1.000001:raise ValueError('invalid actual native actuator torque')
    leftpart=parts['left_foot'];leftpose=transforms['left_foot'];whole_clearance=min((leftpose@np.r_[fk.xyz(c[end]),1])[1]-c['radius']for c in leftpart['reference']['primitives']for end in ['a','b'])
    return dict(wholeFootClearanceM=float(whole_clearance),sample=enriched,physical=physical,comWorld=com,comVelocityWorld=velocity,captureWorld=capture,transforms=transforms,controls=np.clip(controls,-1.,1.))


def task_errors(measured,target,entry_heading,entry_left):
    """Post-interval actual errors against the held PRE-action phase target."""
    physical=measured['physical'];actual=measured['transforms']['left_foot'];desired=np.asarray(target['leftFoot'],dtype=float);heading=np.asarray(entry_heading,dtype=float);origin=np.asarray(entry_left,dtype=float)
    if heading.shape!=(3,)or origin.shape!=(3,)or not np.isfinite([heading,origin]).all()or abs(np.linalg.norm(heading)-1)>1e-8 or abs(heading[1])>1e-8:raise ValueError('invalid frozen entry heading')
    for name,shape in [('q',(21,)),('qvel',(27,)),('com',(3,)),('com_ori',(3,)),('comVelocity',(3,)),('leftFoot',(4,4))]:
        if np.shape(target[name])!=shape or not np.isfinite(target[name]).all():raise ValueError('invalid reference error target '+name)
    if not math.isfinite(target['clearance']):raise ValueError('invalid target clearance')
    actual_r=Rotation.from_matrix(actual[:3,:3]).as_matrix();target_r=Rotation.from_matrix(desired[:3,:3]).as_matrix()
    return dict(jointPositionErrorRad=physical[:21]-target['q'],hipSpeedErrorRadps=physical[46:67][[5,11]]-np.asarray(target['qvel'])[6:][[5,11]],comHeightErrorM=measured['comWorld'][1]-target['com'][2],torsoVerticalError=physical[34:37]-target['com_ori'],leftFootPositionErrorM=actual[:3,3]-desired[:3,3],leftFootRotationErrorRad=Rotation.from_matrix(actual_r@target_r.T).as_rotvec(),wholeFootClearanceErrorM=measured['wholeFootClearanceM']-target['clearance'],signedForwardErrorM=float(((actual[:3,3]-origin)-(desired[:3,3]-origin))@heading),comVelocityErrorMps=P@measured['comVelocityWorld']-target['comVelocity'])
