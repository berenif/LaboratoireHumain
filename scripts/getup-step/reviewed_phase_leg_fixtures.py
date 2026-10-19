"""Explicit diagnostic fixture adapters. No native APIs or installations."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from phase_leg_reference import (P,Jet,MatrixJet,BaseFrameSample,FKJets,INDICES,
    FEET,clock,source_pose_jet)

ROOT=Path(__file__).resolve().parents[2]
MODEL=ROOT/'rust/crates/model/data/getup-v1.json'
PURE_FK=ROOT/'scripts/getup-step-reference.py'
SOURCE_XML=Path('/workspace/shared/cadence-robust-8-ten/model-session/source-model/humanoid_high_freq.xml')
SOURCE_RUN=Path('/workspace/shared/reviewed-walk/causal-selection-seven-01/run/generation-001')


def load_model():
    spec=importlib.util.spec_from_file_location('existing_pure_getup_fk',PURE_FK)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    model=json.loads(MODEL.read_text())
    return module.Kinematics(model)


def source_limits():
    # Source XML ranges differ from the historical JSON's narrowed metadata.
    # Consume the actually pinned source XML limits WITHOUT opening MuJoCo.
    joints=ET.parse(SOURCE_XML).getroot().find('worldbody').findall('.//joint')
    names=[j.attrib['name'] for j in joints]
    expected=json.loads(MODEL.read_text())['referenceMetadata']['jointOrder']
    if names!=expected: raise ValueError('pinned source joint order changed')
    return np.deg2rad(np.array([list(map(float,j.attrib['range'].split())) for j in joints]))


def source_root(qpos):
    qpos=np.array(qpos)
    R=Rotation.from_quat(qpos[[4,5,6,3]]).as_matrix()
    return source_pose_jet(Jet.rest(qpos[:3]),R)


def fixed_frame(kin,q,root,stance_side):
    q=Jet.rest(q); stance=MatrixJet.rest(kin.forward(np.array(q.position),root.arrays()[0])[FEET[stance_side]])
    sample=BaseFrameSample(root,q,stance)
    return lambda phase,t:sample


def source_fixed_fixture(step):
    kin=load_model(); limits=source_limits()
    controller=json.loads((SOURCE_RUN/f'controller-step-{step:02d}.json').read_text())
    reference=json.loads((SOURCE_RUN/f'stance-reference-{step:02d}.json').read_text())
    side=controller['swing_side']; q=np.array(reference['qReference'])
    root=source_root(reference['qposWitness'])
    return kin,side,fixed_frame(kin,q,root,1-side),limits,controller['heading']


class MovingMathFixture:
    """ONE explicit math fixture, deliberately not the source-admitted posture.

    Swing starts with 1rad knee flexion and 0.15rad plantar ankle reference.
    Root moves forward half the stride during translation; stance is supplied as
    an exact stationary rigid-foot path with compatible analytically derived IK
    joint jets. This only exercises moving-frame mathematics, not gait admission.
    No fixture is installed into a native state.
    """
    def __init__(self,kin,side,limits):
        self.kin=kin; self.fk=FKJets(kin); self.side=side; self.stance=1-side
        self.q=np.zeros(21); self.q[[5,11]]=-.85; self.q[[6,12]]=-1.; self.q[[7,13]]=-.15
        # Deliberately reserved, distinct stance math posture. It is NOT an
        # admitted source support configuration or a posture recommendation.
        self.q[INDICES[self.stance][2]]=-1.15
        self.q[INDICES[self.stance][4]]=.15
        root=np.eye(4); root[1,3]=1.4
        pose=kin.forward(self.q,root)[FEET[side]]
        root[1,3]-=kin.bottom(FEET[side],pose)
        self.root=MatrixJet.rest(root)
        self.stance_pose=MatrixJet.rest(kin.forward(self.q,root)[FEET[self.stance]])
        self.interior=limits+np.column_stack((np.full(21,.02),np.full(21,-.02)))

    def __call__(self,phase,t):
        if phase in ('lift','lift_wait'): s,ds,dds=0.,0.,0.
        elif phase=='translate': s,ds,dds=clock(t,1.2)
        else: s,ds,dds=1.,0.,0.
        T,V,A=self.root.arrays()
        delta=P.T@np.array([.091,0.,0.]); T[:3,3]+=delta*s
        V[:3,3]=delta*ds; A[:3,3]=delta*dds
        root=MatrixJet(T,V,A)
        joints,matched,info=self.fk.inverse_jet(Jet.rest(self.q),root,FEET[self.stance],
            self.stance_pose,INDICES[self.stance],self.interior)
        return BaseFrameSample(root,joints,self.stance_pose)
