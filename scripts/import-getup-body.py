#!/usr/bin/env python3
"""Read pinned MJCF as data only. No MuJoCo, policy, or source Python execution.

Ordered hinges become a reduced-coordinate chain. Nine zero-mass coordinate
frames have no collision/render surface; the sixteen source bodies retain all
their mass. Primitive inertia is analytic, independent of render tessellation.
"""
import argparse, hashlib, json, math, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / 'rust/crates/model/data/getup-humanoid.xml'
COMMIT = '5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e'
EXPECTED_XML_SHA256 = '69a7e87fed168e1f4ad7dee43dfdd8df6bd74b3d4af06d99c991b46b941106db'
C = np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
def values(text, default): return np.array([float(x) for x in text.split()]) if text is not None else np.array(default,dtype=float)
def vec(v): return dict(zip('xyz', map(float,v)))
def quat(r): return dict(zip('xyzw', map(float,Rotation.from_matrix(r).as_quat())))
def pose(p,r): return {'position':vec(p),'rotation':quat(r)}
def frame(p,r): return {'anchor':vec(p),'rotation':quat(r)}
def basis(axis):
    axis=axis/np.linalg.norm(axis); helper=np.eye(3)[np.argmin(np.abs(axis))]
    y=np.cross(helper,axis);y/=np.linalg.norm(y)
    return np.column_stack([axis,y,np.cross(axis,y)])
def geometry_shape(g):
    radius=float(g.attrib['size'].split()[0])
    if g.get('type','capsule')=='sphere': a=b=C@values(g.get('pos'),[0,0,0]);kind='sphere'
    else:
        ends=values(g.get('fromto'),[]);assert len(ends)==6
        a=C@ends[:3];b=C@ends[3:];kind='capsule'
    return {'name':g.attrib['name'],'kind':kind,'a':vec(a),'b':vec(b),'radius':radius}
def primitive_mass(g):
    a=np.array(list(g['a'].values()));b=np.array(list(g['b'].values()));r=g['radius'];h=np.linalg.norm(b-a)/2
    mc=1000*math.pi*r*r*2*h;ms=1000*4*math.pi*r**3/3;m=mc+ms
    ia=mc*r*r/2+ms*2*r*r/5
    ip=mc*(3*r*r+4*h*h)/12+ms*(2*r*r/5+h*h+3*h*r/4)
    axis=(b-a)/(2*h) if h else np.array([0.,1.,0.])
    tensor=ip*np.eye(3)+(ia-ip)*np.outer(axis,axis)
    return m,(a+b)/2,tensor
def mesh(primitives):
    vertices=[];indices=[];sectors=20;cap_steps=6
    for g in primitives:
        a=np.array(list(g['a'].values()));b=np.array(list(g['b'].values()));r=g['radius'];length=np.linalg.norm(b-a)
        axis=(b-a)/length if length else np.array([0.,1.,0.]);B=basis(axis);center=(a+b)/2
        # Sphere/capsule surface of revolution, axis is B[:,0].
        rings=[]
        for j in range(1,2*cap_steps):
            angle=-math.pi/2+j*math.pi/(2*cap_steps)
            axial=r*math.sin(angle)+(length/2 if angle>=0 else -length/2)
            rings.append((axial,r*math.cos(angle)))
            if j==cap_steps and length: rings.insert(-1,(-length/2,r))
        start=len(vertices)//3;vertices.extend((center-axis*(length/2+r)).tolist())
        for axial,radial in rings:
            for k in range(sectors):
                angle=2*math.pi*k/sectors
                vertices.extend((center+B@np.array([axial,radial*math.cos(angle),radial*math.sin(angle)])).tolist())
        top=len(vertices)//3;vertices.extend((center+axis*(length/2+r)).tolist())
        for k in range(sectors):
            n=(k+1)%sectors;indices.extend([start,start+1+n,start+1+k])
        for j in range(len(rings)-1):
            for k in range(sectors):
                n=(k+1)%sectors;a0=start+1+j*sectors+k;a1=start+1+j*sectors+n;b0=a0+sectors;b1=a1+sectors
                indices.extend([a0,a1,b1,a0,b1,b0])
        last=start+1+(len(rings)-1)*sectors
        for k in range(sectors):indices.extend([last+k,last+(k+1)%sectors,top])
    return {'vertices':vertices,'indices':indices}
def import_model():
    xml=XML.read_bytes();assert hashlib.sha256(xml).hexdigest()==EXPECTED_XML_SHA256,'Pinned MJCF changed'
    root=ET.fromstring(xml)
    defaults={}
    def walk_defaults(node,inherited):
        merged=dict(inherited);joint=node.find('joint')
        if joint is not None:merged.update(joint.attrib)
        if node.get('class'):defaults[node.get('class')]=merged
        for child in node.findall('default'):walk_defaults(child,merged)
    walk_defaults(root.find('default'),{})
    motors=list(root.find('actuator'));motor_names=[x.attrib['joint'] for x in motors]
    joints=list(root.find('worldbody').iter('joint'));joint_names=[x.attrib['name'] for x in joints]
    assert len(joints)==21 and len(set(joint_names))==21 and set(motor_names)==set(joint_names)
    nodes=[];source_names=[];body_nodes={};welded={};physical_parent={}
    def body(node,parent=None,parent_p=np.zeros(3),parent_r=np.eye(3),weld=None):
        name=node.attrib['name'];source_names.append(name)
        p=C@values(node.get('pos'),[0,0,0]);q=values(node.get('quat'),[1,0,0,0]);R=C@Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()@C.T
        wp=parent_p+parent_r@p;wr=parent_r@R
        actual_parent=parent;hinges=node.findall('joint');primitives=[geometry_shape(g) for g in node.findall('geom')]
        if not hinges and parent is not None:weld=welded[parent]
        else:weld=name
        physical_parent[name]=None if parent is None else welded[parent]
        for k in range(max(1,len(hinges))):
            last=k==len(hinges)-1 or not hinges;identifier=name if last else '@'+hinges[k].attrib['name']
            local_p=p if k==0 else np.zeros(3);local_r=R if k==0 else np.eye(3)
            jp=None;hinge=None
            if hinges:
                j=hinges[k];attrs=defaults[j.get('class','body')]|j.attrib
                axis=C@values(attrs.get('axis'),[0,0,1]);anchor=C@values(attrs.get('pos'),[0,0,0]);B=basis(axis)
                lo,hi=np.radians(values(attrs['range'],[]));ai=motor_names.index(j.attrib['name']);gear=float(motors[ai].attrib['gear'])
                hinge={'name':j.attrib['name'],'qposIndex':joint_names.index(j.attrib['name']),'actuatorIndex':ai,'armature':float(attrs['armature']),'gear':gear,'axis':vec(axis/np.linalg.norm(axis)),'anchor':vec(anchor)}
                jp={'parentFrame':frame(local_p+local_r@anchor,local_r@B),'childFrame':frame(anchor,B),'axes':[{'coordinate':'x','minRadians':float(lo),'maxRadians':float(hi),'passiveStiffnessNmPerRad':float(attrs['stiffness']),'dampingNmsPerRad':float(attrs['damping']),'maxMotorTorqueNm':gear}],'limitSoftZoneFraction':0.0}
            elif parent is not None:jp={'parentFrame':frame(local_p,local_r),'childFrame':frame(np.zeros(3),np.eye(3)),'axes':[],'limitSoftZoneFraction':0.0}
            geoms=primitives if last else [];parts=[primitive_mass(g) for g in geoms];mass=sum(x[0] for x in parts)
            com=sum((m*c for m,c,_ in parts),np.zeros(3))/mass if mass else np.zeros(3)
            inertia=sum((I+m*(np.dot(c-com,c-com)*np.eye(3)-np.outer(c-com,c-com)) for m,c,I in parts),np.zeros((3,3)))
            role='coordinate' if not last else {'lower_waist':'lumbar','right_foot':'hindfoot','left_foot':'hindfoot'}.get(name,name.removeprefix('left_').removeprefix('right_'))
            region=None if not last else name if name in ['left_hand','right_hand','left_foot','right_foot'] else 'head' if name=='head' else 'legs' if any(x in name for x in ['thigh','shin','foot']) else 'arms' if any(x in name for x in ['arm','hand']) else 'pelvis' if name=='pelvis' else 'torso'
            nodes.append({'id':identifier,'region':region,'parent':actual_parent,'role':role,'side':'left' if name.startswith('left_') else 'right' if name.startswith('right_') else None,'massKg':mass,'geometry':mesh(geoms),'jointProfile':jp,'collisionExclusions':[],'initial':{'id':identifier,**pose(wp,wr),'linearVelocity':vec(np.zeros(3)),'angularVelocity':vec(np.zeros(3))},'goldenMass':{'volume':mass/1000,'centerOfMass':vec(com),'inertia':inertia.tolist()},'reference':{'sourceBody':name if last else None,'primitives':geoms,'hinge':hinge}})
            welded[identifier]=weld;actual_parent=identifier
        body_nodes[name]=len(nodes)-1
        for child in node.findall('body'):body(child,name,wp,wr,weld)
    body(root.find('worldbody/body'))
    # MuJoCo default filterparent operates on welded bodies, not on virtual hinges.
    for a in nodes:
        for b in nodes:
            if a is b:continue
            wa,wb=welded[a['id']],welded[b['id']]
            if not a['reference']['sourceBody'] or not b['reference']['sourceBody'] or wa==wb or physical_parent.get(wa)==wb or physical_parent.get(wb)==wa:a['collisionExclusions'].append(b['id'])
    assert len(source_names)==16 and sum(len(n['reference']['primitives']) for n in nodes)==19 and len(nodes)==25
    metadata={'repository':'https://github.com/tianxintao/get_up_control','commit':COMMIT,'asset':'data/humanoid_high_freq.xml','assetSha256':hashlib.sha256(xml).hexdigest(),'sourceBodyOrder':source_names,'jointOrder':joint_names,'actuatorOrder':motor_names,'actuatorToQpos':[joint_names.index(n) for n in motor_names],'coordinateFrames':9,'sourceDensityKgM3':1000,'physicsStepSeconds':.00125,'policyStepSeconds':.025,'groundFriction':1.0,'selfContactFriction':0.0,'standingPose':[0,-.11665,0,-.03601,-.03219,.05025,-.39803,-.30198,.13615,-.03601,-.03219,.05025,-.39803,-.30198,-.13615,.40763,-.41967,-1.5795,-.40763,.41967,-1.5795],'observationOrder':['joint_angles:21','head_height:1','extremities(left_hand,left_foot,right_hand,right_foot):12','torso_vertical:3','com_velocity:3','qvel(root_linear_world,root_angular_local,joints):27'],'weakInput':68,'slowInput':118,'slowTargets':'qpos[21], CoM height[1], torso vertical[3], at t and t+5; power last','observationNormalization':'none in Actor/env; raw physical units','action':'WEAK normalized motor commands; SLOW tanh residuals in qpos order, normalized PD then actuator gear','simulatorGaps':['Rapier reduced-coordinate dynamics and contact/limit solver are not MuJoCo 2.1.3','MuJoCo solref/solimp soft-contact and soft-limit laws are not reproduced','No learned actor or reference-generation pipeline is loaded']}
    # Independent source-body FK, without the serial-frame representation.
    standing_poses={}
    def source_fk(node,parent_p=np.zeros(3),parent_r=np.eye(3)):
        p=values(node.get('pos'),[0,0,0]);q=values(node.get('quat'),[1,0,0,0]);R=Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
        for j in node.findall('joint'):
            attrs=defaults[j.get('class','body')]|j.attrib
            axis=values(attrs.get('axis'),[0,0,1]);axis/=np.linalg.norm(axis);a=values(attrs.get('pos'),[0,0,0])
            lo,hi=np.radians(values(attrs['range'],[]));angle=np.clip(metadata['standingPose'][joint_names.index(j.attrib['name'])],lo,hi)
            rotate=Rotation.from_rotvec(axis*angle).as_matrix();p=p+R@(a-rotate@a);R=R@rotate
        wp=parent_p+parent_r@p;wr=parent_r@R
        standing_poses[node.attrib['name']]=pose(C@wp,C@wr@C.T)
        for child in node.findall('body'):source_fk(child,wp,wr)
    source_fk(root.find('worldbody/body'))
    metadata['standingBodyPoses']=standing_poses
    # Environment surfaces are independent of the superseded anatomical mesh.
    old=json.loads((ROOT/'rust/crates/model/data/environment-v1.json').read_text())
    vertical=[]
    for n in nodes:
        p=np.array(list(n['initial']['position'].values()));R=Rotation.from_quat(list(n['initial']['rotation'].values())).as_matrix()
        for g in n['reference']['primitives']:
            for endpoint in ['a','b']:
                y=(p+R@np.array(list(g[endpoint].values())))[1];vertical.extend([y-g['radius'],y+g['radius']])
    return {'schemaVersion':2,'totalMassKg':sum(n['massKg'] for n in nodes),'statureM':max(vertical)-min(vertical),'segments':nodes,'playground':old['playground'],'protocol':old['protocol'],'referenceMetadata':metadata}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');args=p.parse_args()
    model=import_model();target=ROOT/'rust/crates/model/data/getup-v1.json';data=json.dumps(model,separators=(',',':'))+'\n'
    if args.check:assert target.read_text()==data,'Generated reference model drift'
    else:target.write_text(data)
    print(json.dumps({'sourceBodies':16,'coordinateFrames':9,'hinges':21,'geoms':19,'massKg':model['totalMassKg'],'motorToJoint':model['referenceMetadata']['actuatorToQpos'],'xmlSha256':model['referenceMetadata']['assetSha256']}))
