"""Portable read-only adapter for an ACTUAL retained stationary TRANSFER input.

No journals, native library, state restore, physics, optimizer, or future actual
state are consumed. Native payload is hashed only as a same-counter witness.
Repo root, original ArmMap/table and source XML are explicitly injected.
"""
from dataclasses import asdict
import hashlib,importlib.util,json,os,zlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from source_reference_context import ReferenceContext
from coupled_posture_reference import CurrentSupport,digest
from coupled_posture_kinematics import ArmMap,MatrixJet
from wholebody_cycle_reference import StanceRealizedIK

INPUT_KIND='retained-native-stationary-transfer-activation-v1'
FROZEN_ANALYTIC_SOURCES=('coupled_posture_kinematics.py','coupled_posture_reference.py','phase_leg_reference.py','source_reference_context.py')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_retained_activation(input_path,*,repo_root=None,arm_map=None,arm_map_table_path=None,source_xml_path,
        kinematics=None,verify_payload=True,source_proposal_path=None,source_proposal_binding_path=None):
    path=Path(input_path);data=json.loads(path.read_text());context=ReferenceContext.from_receipt(data['referenceContext'])
    support=CurrentSupport(**data['currentSupport']);support.validate_identity(context);support.validate_com(context.com)
    config=json.loads(context.config_json);causal=json.loads(context.causal_json)
    if(context.phase!='transfer'or context.elapsed<config['phase_durations']['transfer']-1e-10
            or any(context.com.velocity)or any(context.com.acceleration)
            or any(context.joints.velocity)or any(context.joints.acceleration)):
        raise ValueError('retained completed stationary TRANSFER reference required')
    correction=causal.get('robustCorrection',{})
    if correction.get('selected')and tuple(correction.get('clock',()))!=(1.,0.,0.):raise ValueError('original causal correction not complete')
    if not support.require_triangle:raise ValueError('original current selected stance triangle required')
    proposal=data['proposal'];binding=proposal['binding'];stance=binding['stanceSide'];swing=binding['swingSide']
    if type(stance)is not int or stance not in(0,1)or swing!=1-stance:raise ValueError('original stance/swing sides required')
    if stance!=config['stance_side']or swing!=config['swing_side']:raise ValueError('retained controller sides differ')
    limits=np.asarray(binding['originalSourceLimitsRad'],float)
    origins=np.asarray(binding['legOriginRightLeftRad'],float);heading=np.asarray(config['heading'],float)
    feet=np.asarray(data['currentFeetModel'],float)
    if limits.shape!=(21,2)or not np.isfinite(limits).all()or np.any(limits[:,0]>=limits[:,1]):raise ValueError('original finite21 source limits required')
    if origins.shape!=(2,)or not np.array_equal(origins,np.asarray(config['leg_origin'])):raise ValueError('persistent side-ordered leg origins differ')
    if feet.shape!=(2,4,4)or not np.isfinite(feet).all():raise ValueError('both same-counter actual foot frames required')
    poses=tuple(MatrixJet.rest(T)for T in feet)
    if heading.shape!=(3,)or abs(np.linalg.norm(heading)-1.)>1e-8 or abs(heading[2])>1e-12:raise ValueError('original horizontal unit heading required')
    root=Path(repo_root or os.environ.get('REVIEWED_WALK_REPO_ROOT')or Path(__file__).resolve().parents[2])
    model_path=root/'rust/crates/model/data/getup-v1.json'
    if sha(model_path)!=binding['modelSHA256']:raise ValueError('retained original source model hash differs')
    if sha(source_xml_path)!=binding['sourceXMLSHA256']:raise ValueError('retained original source XML hash differs')
    if arm_map is None:
        if arm_map_table_path is None:raise ValueError('inject original ArmMap or archived original table')
        arm_map=ArmMap.from_archived_table(json.loads(Path(arm_map_table_path).read_text()))
    if digest(asdict(arm_map))!=binding['armMapSemanticSHA256']:raise ValueError('retained original ArmMap semantic hash differs')
    if kinematics is None:
        source=root/'scripts/getup-step-reference.py';spec=importlib.util.spec_from_file_location('retained_activation_existing_fk',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        kinematics=module.Kinematics(json.loads(model_path.read_text()))
    if digest(kinematics.model)!=digest(json.loads(model_path.read_text())):raise ValueError('injected kinematics model differs')
    payload_path=path.parent/data['retainedPayload']
    if Path(data['retainedPayload']).name!=data['retainedPayload']:raise ValueError('retained native payload basename required')
    payload_verified=False
    if verify_payload:
        compressed=payload_path.read_bytes()
        if hashlib.sha256(compressed).hexdigest()!=data['compressedPayloadSHA256']:raise ValueError('compressed native witness hash differs')
        count=data['actualPayloadBytes']
        if type(count)is not int or not 0<count<=32*1024*1024:raise ValueError('bounded retained payload byte count required')
        decoder=zlib.decompressobj();payload=decoder.decompress(compressed,count+1)
        if(len(payload)!=count or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail
                or hashlib.sha256(payload).hexdigest()!=data['actualPayloadSHA256']):raise ValueError('actual native witness bytes/hash differ')
        payload_verified=True
    source_binding_hash=None
    if source_proposal_path is not None:
        seed=json.loads(Path(source_proposal_path).read_text())
        if(sha(source_proposal_path)!=proposal['proposalSHA256']or digest(seed)!=proposal['offlineReceiptSHA256']
                or seed.get('independentKnotsRad')!=proposal['independentKnotsRad']):raise ValueError('captured frozen source proposal differs')
    if source_proposal_binding_path is not None:
        old_binding=json.loads(Path(source_proposal_binding_path).read_text());source_binding_hash=sha(source_proposal_binding_path)
        if old_binding!=binding:raise ValueError('captured frozen proposal annotation differs')
    realizer=StanceRealizedIK(kinematics,stance,poses[stance],arm_map,origins)
    bound=dict(schema=1,inputKind=INPUT_KIND,inputSHA256=sha(path),contextSHA256=context.receipt()['contextSHA256'],
        sourceNativeStep=context.source_step,sourceTimeS=context.source_time,referencePhase=context.phase,
        referenceElapsedS=context.elapsed,entryLiftElapsedS=0.,actualPayloadSHA256=data['actualPayloadSHA256'],
        actualPayloadBytes=data['actualPayloadBytes'],compressedPayloadSHA256=data['compressedPayloadSHA256'],
        payloadHashVerified=payload_verified,stateRestored=False,currentSupportSHA256=support.key,
        actualFeetModelSHA256=digest(feet.tolist()),currentFeetModelSHA256=digest(feet.tolist()),headingSource='referenceContext.config.heading',
        currentFeetModel=feet.tolist(),stancePoseModel=poses[stance].T.tolist(),heading=heading.tolist(),
        support=dict(sourceStep=context.source_step,sourceTimeS=context.source_time,phase=context.phase,
            loadedPointsXY=[list(p)for p in support.loaded_points_xy],stancePointsXYZ=[list(p)for p in support.stance_points_xyz],
            presentCorners=list(support.present_corners),survivingCorners=list(support.surviving_corners),equations=support.equations().tolist(),
            originalSupportReserveM=.003,triangleFromActualLoadedPoints=True),
        modelSHA256=binding['modelSHA256'],sourceXMLSHA256=binding['sourceXMLSHA256'],armMapSemanticSHA256=binding['armMapSemanticSHA256'],
        sourceProposalSHA256=proposal['proposalSHA256'],sourceProposalBindingSHA256=source_binding_hash,
        frozenSourceHashes={name:sha(root/'scripts/getup-step'/name)for name in FROZEN_ANALYTIC_SOURCES},
        actualRetainedActivationInput=True,oldFirstLiftInputReused=False,nativeWorldConstructions=0,physicsIntegrations=0,
        armConditioning='offline future-thigh map plus exact prepared-arm entry bridge; runtime retains measured delayed strong arms and weak tail')
    return SimpleNamespace(inputs=data,context=context,realizer=realizer,limits=limits,source_limits=limits,support=support,
        heading=heading,arm_map=arm_map,kinematics=kinematics,repo_root=root,binding=bound,payload_path=payload_path,
        source_xml_path=Path(source_xml_path),entry_lift_elapsed=0.,current_feet_model=feet)
