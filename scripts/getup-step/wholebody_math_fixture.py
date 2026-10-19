"""Explicit mathematical API fixture. Never a source gait or admission seed.

Uses only the repository model. Limits, support and constant wide arm mapping
are SYNTHETIC to test pure protocol/jet math without private archived journals.
In a repository this file lives in scripts/getup-step; shared callers must pass
repo_root or set REVIEWED_WALK_REPO_ROOT. No absolute workspace path is baked in.
"""
import importlib.util,json,os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from coupled_posture_reference import Jet,CurrentSupport
from coupled_posture_kinematics import ModelFK,ArmMap,MatrixJet,P,pose_jet_to_source
from source_reference_context import ReferenceContext,Jet as ContextJet,Orientation,Foot
from wholebody_cycle_reference import StanceRealizedIK

def make_math_fixture(repo_root=None):
    root=Path(repo_root or os.environ.get('REVIEWED_WALK_REPO_ROOT') or Path(__file__).resolve().parents[2])
    model_path=root/'rust/crates/model/data/getup-v1.json'
    if not model_path.exists():raise ValueError('inject repo_root or REVIEWED_WALK_REPO_ROOT for the shared mathematical fixture')
    raw=json.loads(model_path.read_text())
    spec=importlib.util.spec_from_file_location('wholebody_math_existing_fk',root/'scripts/getup-step-reference.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    kin=module.Kinematics(raw);model=ModelFK.from_model(raw)
    lower=np.zeros(15);lower[[5,11]]=-.85;lower[[6,12]]=-1.;lower[[7,13]]=-.15
    limits=np.tile([-3.,3.],(21,1));limits[[6,12],1]=0.
    # Every spline coefficient is zero: both arm predictions are constant zero,
    # with explicitly wide synthetic map intervals and no source-table claims.
    arm_map=ArmMap(((-10.,10.),(-10.,10.)),((-10.,10.),(-10.,10.)),
        tuple(tuple(tuple((0.,0.,0.)for _ in range(1))for _ in range(4))for _ in range(2)),
        (0.,0.),(0.,)*6)
    leg_origin=model.thigh_jet(Jet.rest(lower)).position
    full=Jet.rest(np.r_[lower,np.zeros(6)])
    root_pose=np.eye(4);root_pose[1,3]=1.4
    pose=kin.forward(np.array(full.position),root_pose)['right_foot']
    root_pose[1,3]-=kin.bottom('right_foot',pose)
    poses=kin.forward(np.array(full.position),root_pose)
    stance=1;stance_pose=MatrixJet.rest(poses['right_foot'])
    realizer=StanceRealizedIK(kin,stance,stance_pose,arm_map,leg_origin)
    result=model.coupled(Jet.rest(lower),stance,stance_pose,arm_map,leg_origin)
    feet=[]
    for side,name in enumerate(('left_foot','right_foot')):
        p=pose_jet_to_source(result.pose(name))
        feet.append(Foot(ContextJet.make(*p.translation.arrays()),Orientation.make((10,7)[side],np.array(p.rotation).reshape(3,3),p.angular_velocity_world,p.angular_acceleration_world)))
    heading=np.array([1.,0.,0.]);landing=np.array(feet[0].translation.position)+.182*heading
    config=dict(phase_durations=dict(transfer=4.,lift=.8,translate=1.2,land=1.,settle=1.52),
        landing=landing.tolist(),floorTargets=[f.translation.position[2]for f in feet],
        explicitMathematicalFixture=True,syntheticLimitsAndSupport=True)
    body=Orientation.make(1,Rotation.from_euler('ZYX',[.01,.02,-.01]).as_matrix(),[0.,0.,0.],[0.,0.,0.])
    strong=ContextJet.make(np.zeros(6),np.linspace(.01,.02,6),np.zeros(6))
    context=ReferenceContext(2,'reviewed-reference-v1',100,.125,1,'lift',.00125,0,
        ContextJet.make(*result.com_source.arrays()),ContextJet.make(*full.arrays()),strong,(100.,20.,2.),
        (body,),tuple(feet),(),True,json.dumps(dict(robustCorrection=dict(selected=True,clock=[1.,0.,0.]),contactSeekAfter=dict(jet=[0.,0.,0.]))),
        json.dumps(config),json.dumps(dict(explicitMathematicalFixture=True)))
    center=np.array(context.com.position)[:2]
    points=np.array([[-2.,-1.],[2.,-1.],[0.,2.],[2.,2.]])+center
    support=CurrentSupport(100,.125,1,'lift',tuple(map(tuple,points)),tuple(map(tuple,np.column_stack((points,np.zeros(4))))),
        (True,)*4,(0,1,2),True)
    support.validate_com(context.com)
    return SimpleNamespace(kinematics=kin,kin=kin,model=model,raw_model=raw,arm_map=arm_map,mapping=arm_map,
        source_limits=limits,limits=limits,lower=Jet.rest(lower),full=full,leg_origin=tuple(leg_origin),
        stance_pose=stance_pose,realizer=realizer,context=context,support=support,heading=heading,
        feet_model=tuple(result.pose(name).T for name in ('left_foot','right_foot')),
        inputs=dict(controller=dict(heading=heading.tolist(),leg_origin=list(leg_origin),stance_side=1,swing_side=0)),
        binding=dict(explicitMathematicalFixture=True,sourceGaitAdmission=False,optimizerCalls=0,nativeWorldConstructions=0))
