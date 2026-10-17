#!/usr/bin/env python3
"""Measured-contact inverse-dynamics transfer probe, untrained hybrid."""
import argparse,datetime,hashlib,importlib.util,json,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
stance_planner=module('stance_planner',ROOT/'scripts/plan-getup-stance-target.py');step_reference=module('contact_step',ROOT/'scripts/getup-contact-step.py');probe=module('probe',ROOT/'scripts/probe-getup-recovery-step.py');dynamics=module('contact_dynamics',ROOT/'scripts/getup-contact-dynamics.py')


def main():
 p=argparse.ArgumentParser(description=__doc__)
 for name in ['binary','checkpoint','output']:p.add_argument('--'+name,type=Path,required=True)
 p.add_argument('--force-jacobian',choices=['live-geometric','native-frozen'],default='live-geometric');p.add_argument('--joint-reserve-margin-rad',dest='joint_reserve_margin',type=float,default=0.);p.add_argument('--qp-solver',choices=['slsqp','clarabel'],default='slsqp');p.add_argument('--joint-viability',action='store_true');p.add_argument('--terminal-unload',action='store_true');p.add_argument('--stage-unload-after-readiness',action='store_true');p.add_argument('--motion-profile',action='store_true');p.add_argument('--flat-soles',action='store_true');p.add_argument('--balanced-entry',action='store_true');p.add_argument('--reference-context',type=Path);p.add_argument('--contact-mode',choices=['normal-soft','unilateral-soft'],default='normal-soft');p.add_argument('--recovery-step',action='store_true');p.add_argument('--target-calibration',type=Path);p.add_argument('--unload-left',action='store_true');p.add_argument('--hold-s',type=float,default=2.);p.add_argument('--return-ramp-s',type=float);p.add_argument('--shift-m',type=float,default=0.);p.add_argument('--ramp-s',type=float,default=1.5);p.add_argument('--seconds',type=float,default=12.);p.add_argument('--episode-limit-s',type=int,default=20);p.add_argument('--retain-balance',action='store_true');p.add_argument('--physics-budget',type=float,required=True);p.add_argument('--deadline',required=True);p.add_argument('--execute',action='store_true');a=p.parse_args();assert not a.stage_unload_after_readiness or a.terminal_unload;assert not a.terminal_unload or (a.recovery_step and a.unload_left and a.motion_profile)
 deadline=datetime.datetime.fromisoformat(a.deadline.replace('Z','+00:00')).timestamp();assert 0<deadline-time.time()<=3600.,'A fresh bounded deadline within one hour is required';assert 0<=a.physics_budget<=90 and 0<=a.shift_m<=.15;assert 20<=a.episode_limit_s<=40;assert 8<=a.seconds<=a.episode_limit_s-.25 and a.seconds*40==round(a.seconds*40)
 assert 1.5<=a.ramp_s<=12. and .5<=a.hold_s<=2.;return_ramp=a.ramp_s if a.return_ramp_s is None else a.return_ramp_s;assert 1.5<=return_ramp<=7.5;hold_start=1+a.ramp_s;hold_end=hold_start+a.hold_s;return_end=hold_end+return_ramp;return_score_start=return_end+1.;assert a.seconds>=return_score_start+5. or (a.flat_soles and a.shift_m==0 and a.retain_balance and a.seconds>=8.)
 a.output.mkdir(parents=True,exist_ok=True);(a.output/'started-once').open('x').close();started=time.monotonic();controller=dynamics.ContactDynamics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()),contact_mode=a.contact_mode,flat_soles=a.flat_soles,joint_viability=a.joint_viability,qp_solver=a.qp_solver,joint_reserve_margin=a.joint_reserve_margin,force_jacobian=a.force_jacobian);native=probe.Native(a.binary,a.checkpoint,a.physics_budget,deadline,a.output)
 if a.episode_limit_s!=20:native.command(dict(op='episodeLimit',seconds=a.episode_limit_s))
 config=dict(controller='measured-contact-whole-body-inverse-dynamics-untrained-hybrid',shiftM=a.shift_m,simulatedSeconds=a.seconds,physicsCapSeconds=a.physics_budget,deadlineUTC=a.deadline,softUnloadAfterActualReadiness=a.stage_unload_after_readiness,nearStopModeNoExplicitReleaseNoStep=True,externalForcesApplied=False,sourceWeightsChanged=False,model='continuous rigid-body mass+armature/Coriolis/gravity/passive springs+damping; original inner PD remains active800Hz; near-stop tolerance is predictive approximation, not strict instantaneous complementarity',contactConstraints='actual material-point normal acceleration hard, tangential slip stabilization soft, optimization only; never physical pins',acceptance='Zero: fullmodel1-6s>=5s anchoredstanding,noflight/nonfoot,<=1cmslip,<=2cmCOMdrift. Shift sign:actualCOM displacement alongrequested>=2.5mm for5mmtarget, no parasiticflight/slip, source-only after7s>=5sstanding. This is no stepping claim.',checkpointSha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),nativeBinarySha256=hashlib.sha256(a.binary.read_bytes()).hexdigest(),probeSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),dynamicsSha256=hashlib.sha256((ROOT/'scripts/getup-contact-dynamics.py').read_bytes()).hexdigest())
 for f in ['probe-getup-contact-feedback.py','getup-contact-dynamics.py','getup-contact-step.py','plan-getup-stance-target.py']: (a.output/f).write_bytes((ROOT/'scripts'/f).read_bytes())
 config['contactConstraints']='actual material-point predicted gap active/release constraints and soft velocity stabilization, optimization only; never physical pins' if a.contact_mode=='unilateral-soft' else config['contactConstraints']
 config['diagnosticEpisodeLimitS']=a.episode_limit_s
 config['jointViabilityGuard']=dict(enabled=a.joint_viability,horizonS=.025,futureStoppingReserveBufferRad=a.joint_reserve_margin,modelBrakingAccelerationRadps2=50.,nativeBrakingCapabilityProven=False,description='current and predicted position/velocity stopping reserve, free joints only; same native joint limits and motor caps, not guaranteed continuous-native viability')
 config['motionProfileActuation']=a.motion_profile
 config['motionProfileInterpretation']='qref=q0+t*dq0+.5t²*qdd,dqref=dq0+t*qdd, tau=tauFF+gear*((qref-q)+.2*(dqref-dq)), t=k/800 before integration, same original torque caps/passives. Experimental profile is refreshed each decision; checkpoint retains physical state, next explicit command must reestablish mode.' if a.motion_profile else 'held original residual with native inner PD'
 if a.motion_profile:config['model']=config['model'].replace('original inner PD remains active800Hz','acceleration-predicted q/dq inner impedance recomputed800Hz with original caps')
 config['qpSolver']=dict(backend=a.qp_solver,clarabelVersion='0.11.1' if a.qp_solver=='clarabel' else None,physicalGatesUnchanged=True,sameQPButNotIdenticalTrajectory=True)
 config['forceJacobian']=a.force_jacobian;config['forceJacobianInterpretation']='native frozen world lever for dynamics/contact wrenches; current material-point Jacobian retained for velocity/gap/tasks' if a.force_jacobian=='native-frozen' else 'original live geometric point for all maps'
 config['contactMode']=a.contact_mode
 config['contactModeInterpretation']='25ms constant-acceleration end-gap active/release surrogate; active gaps do not worsen existing penetration and end at/below zero, inactive forces zero and endgap nonnegative; soft existing velocity stabilization; not an exact biased-pass CFM equation' if a.contact_mode=='unilateral-soft' else 'original hard-normal acceleration and soft tangent stabilization'
 config['standingControllerAfterTargetReturn']='measured-contact inverse dynamics retained' if a.retain_balance else 'frozen GetUp SLOW'
 config['timelineS']=dict(rampStart=1.,fullShiftStart=hold_start,fullShiftEnd=hold_end,targetReturned=return_end,postReturnScoringStart=return_score_start)
 if a.retain_balance:config['acceptance']='New retained-balancer gate, not retroactive source-handoff success: actual matched COM/capture/load shift; target COM returned; final>=5s native anchored standing under explicit measured-state balance, no flight/nonfoot and loaded slip<=1cm. No stepping claim.'
 if a.flat_soles and a.shift_m==0 and not a.recovery_step:config['acceptance']='Prospective flat-hold: final5s of complete8s retained-model case native anchored standing; noflight/nonfoot, total loaded slip<=1cm; actual both capsule tracks plus noncollinear support hull>=1cm^2 and <=2deg tilt for final>=1s of40Hzdiagnostics. No stepping claim.'
 report=dict(status='preflight-only');samples=[];model_samples=[];source_samples=[];frames=[];notes=[];history=[];events=[];com_history=[];shift_history=[];solve_wall=state_wall=0.;failure=None;release_start=None;last_action=None;last_source=None;support_run=0.;unload_weight=0.;stepper=None;step_standing_samples=[];terminal_unload=step_reference.TerminalUnloadGoal();terminal_support_run=0.;feasible_refs=None;capacity_allow_stops=True
 actuator_of_qpos=np.zeros(21,dtype=int)
 for part in controller.parts:
  hinge=part['reference'].get('hinge')
  if hinge:actuator_of_qpos[hinge['qposIndex']]=hinge['actuatorIndex']
 try:
  state=native.command({'op':'observe'});initial_diag=native.command({'op':'diagnostic'});snapshot=native.command({'op':'snapshot'});assert state['metrics']['native_standing_without_foot_anchor'];model_state=controller.state(state['observation'],snapshot,initial_diag);reference_state=model_state
  if a.reference_context:
   context=json.loads(a.reference_context.read_text());reference_state=controller.state(context['observation']['observation'],context['snapshot'],context['diagnostic']);config['referenceContextSHA256']=hashlib.sha256(a.reference_context.read_bytes()).hexdigest();config['referenceChoice']='Retain previous measured-balancer initial COM/q/root-rotation targets, no physical state modification'
  if a.flat_soles:controller.set_flat_targets(reference_state)
  config['flatSolesLoadedAngularTask']=a.flat_soles
  config['flatOrientationTargetsWorld']={k:v.tolist() for k,v in controller.flat_targets.items()}
  config['balancedEntryNoSourceBlend']=a.balanced_entry
  initial_com=reference_state['com'].copy();initial_q=reference_state['q'].copy();initial_R=dynamics.normalized_rotation(reference_state['root'][:3,:3]);right=initial_diag['contactMotion']['feet'][1];target=np.average([probe.reference.xyz(c['point']) for c in right['contacts']],axis=0,weights=[c['normalImpulseNs'] for c in right['contacts']]);direction=target-initial_com;direction[1]=0;direction/=np.linalg.norm(direction);
  if a.target_calibration:
   calibration=json.loads(a.target_calibration.read_text())
   if calibration.get('mode') in ['actual-flat-sole-hull-centroid','static-capacity-interior-stance-target','no-stop-attainable-interior-posture-path']:
    assert a.flat_soles and calibration['sourceRuntimeSHA256']==config['checkpointSha256'];points=[[c['point'][k] for k in ['x','z']] for c in right['contacts']];assert dynamics.support_projection(points,calibration['lockedTargetXZ'])[1]<1e-6
    if calibration['mode'] in ['static-capacity-interior-stance-target','no-stop-attainable-interior-posture-path']:
     capacity_allow_stops=calibration['mode']!='no-stop-attainable-interior-posture-path';terminal_capacity=calibration['pathKnots'][-1]['capacity'];assert (calibration['terminalStaticWithin80Percent'] if capacity_allow_stops else (not calibration['limitReactionsCreditedInCertificate'] and not terminal_capacity['limitReactions'] and terminal_capacity['minimumRequiredMotorFraction']<=.8501 and calibration['minimumMovableJointInteriorMarginRad']>=.01));assert a.joint_viability and a.terminal_unload and calibration['allKnotIKPassed'] and calibration['prescribedPathMotorCapacityWithin90Percent'];assert calibration['minimumHullMarginM']>=.005 and calibration['maximumRuntimePchipLeftFootErrorM']<.0001 and calibration['minimumNonFootCapsuleGroundClearanceM']>.02
     assert calibration['sourceReferenceContextSHA256']==config['referenceContextSHA256'];assert calibration['modelSHA256']==hashlib.sha256((ROOT/'rust/crates/model/data/getup-v1.json').read_bytes()).hexdigest();assert abs(a.ramp_s-calibration['rampS'])<1e-8;feasible_refs=stance_planner.FeasibleReferences(calibration,controller);config['referenceChoice']='attainable planted-foot joint/root/COM path from actual warm seed; original context retained for return and source counterfactual';config['feasibleReferencesPlannerSHA256']=calibration['plannerSHA256'];assert calibration['plannerSHA256']==hashlib.sha256((ROOT/'scripts/plan-getup-stance-target.py').read_bytes()).hexdigest();config['actualReadinessCreditsLimitReactions']=capacity_allow_stops;config['actualCapacityReadiness']='Before initiating load ceiling: actual measured q/root/current-velocity bias and same near-stop eligibility, zero-left-force inverse dynamics requires<=90% motor fraction; planning indicator, not dynamic success proof';config['feasibleReferenceReturn']='After verified landing: original root orientation, joint posture interpolated to original during measured3s COM return, persistent measured-state balance afterward'
     if not capacity_allow_stops:assert calibration['forceJacobianMode']==a.force_jacobian=='native-frozen'
     if not capacity_allow_stops:config['actualCapacityReadiness']='Before initiating load ceiling: measured q/root/current-velocity bias and actual frozen contact force map, NO joint-stop reactions credited, zero-left-force capacity<=90% motor fraction for100ms; native stops/limits unchanged. Planning indicator, not dynamic success proof.'
   else:assert calibration['offsetNormM']<=.01
   assert max(abs(initial_com-np.array(calibration['sourceInitialCOM'])))<1e-5;target[[0,2]]=calibration['lockedTargetXZ'];direction=target-initial_com;direction[1]=0.;a.shift_m=float(np.linalg.norm(direction));assert a.shift_m<=.15;direction/=a.shift_m;config['shiftM']=a.shift_m;config['targetCalibrationSHA256']=hashlib.sha256(a.target_calibration.read_bytes()).hexdigest()

  if a.recovery_step:
   assert a.retain_balance and a.unload_left;config['nearStopModeNoExplicitReleaseNoStep']=False;config['stepMode']='measured-unload-gated swing, measured landing, then retained balance';stepper=step_reference.RecoveryStep(initial_com,initial_com+direction*a.shift_m,a.ramp_s,controller.kin,transfer_timeout_s=(1+a.ramp_s+2. if feasible_refs is not None else 9.5));config['transferReadinessTimeoutS']=stepper.transfer_timeout_s;config['stepAcceptance']='Actual>=5cm unload-land displacement/wholecapsuleclearance>=1.5cm, noflight/nonfoot, loadedslip<=1cm, actuallanding+>=5s retainedbalance anchoredstanding; not a learned step or original10sshuffling fix.';config['acceptance']=config['stepAcceptance']
  config['terminalUnloadForceGoal']=dict(enabled=a.terminal_unload,rampS=.5,gate='both COM and capture inside actual right-only hull (1um numeric tolerance), actual right load>=.8Mg across native samples for>=100ms; no lift until measuredleft<3N>=100ms',release='clean stop if eligibility lost after ramp starts; hold transfer until both actual unload and capacity/support readiness qualify; ceiling absent after measured swing transition');config['softUnloadLeft']=a.unload_left;config['initialCOM']=initial_com.tolist();config['shiftDirectionWorld']=direction.tolist()
  assert model_state['comParityErrorM']<1e-4 and model_state['comVelocityParityErrorMps']<2e-5 and model_state['contactVelocityParityErrorMps']<2e-5
  t=time.monotonic();action,info=controller.solve(model_state,initial_com,initial_R,initial_q);solve_wall+=time.monotonic()-t;assert info['success'],info;assert info['maximumTorqueRepresentationErrorNm']<1e-5
  report['preflight']=info;(a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
  if a.execute:
   report['status']='running-contact-feedback'
   for tick in range(round(a.seconds*40)):
    elapsed=tick*.025;right_distance=None;right_com_distance=None;left_force_ceiling=None;actual_capacity=None;posture_goal=initial_q;rotation_goal=initial_R
    source=np.array(native.command({'op':'initializedAction'})['action'])
    blend=1. if a.balanced_entry else (probe.smooth(elapsed) if elapsed<1 else (1. if a.retain_balance else (1-probe.smooth(elapsed-return_end) if elapsed>=return_end else 1.)))
    shift=probe.smooth((elapsed-1)/a.ramp_s) if elapsed<hold_start else (1-probe.smooth((elapsed-hold_end)/return_ramp) if elapsed>=hold_end else 1.)

    if stepper is not None:shift=probe.smooth((elapsed-1)/a.ramp_s)
    wanted_com=initial_com+direction*a.shift_m*shift
    if release_start is not None:
     blend=max(0.,1-probe.smooth((elapsed-release_start)/.05));action=source+blend*(last_action-last_source);info={'success':False,'fallback':True}
    elif blend>0:
     t=time.monotonic();model_state=controller.state(state['observation'],snapshot,diagnostic if tick else initial_diag);state_wall+=time.monotonic()-t
     t=time.monotonic()
     current_diag=initial_diag if tick==0 else diagnostic;right_points=[[c['point'][k] for k in ['x','z']] for c in current_diag['contactMotion']['feet'][1]['contacts']];right_distance=float('inf')
     if right_points:
      _,right_distance=dynamics.support_projection(right_points,probe.reference.xyz(current_diag['capture'])[[0,2]])
      _,right_com_distance=dynamics.support_projection(right_points,probe.reference.xyz(current_diag['com'])[[0,2]])
     strong=bool(tick>0 and all(row['loads_n'][1]>=.8*controller.mass*9.81 for row in last))
     eligible=bool(a.unload_left and shift>=.95 and right_distance<=.003 and strong)
     support_run=support_run+.025 if eligible else 0.;unload_weight=min(1.,unload_weight+.025/.2) if support_run>=.1 else 0.
     transfer_before=bool(stepper is not None and stepper.phase=='transfer')
     if a.terminal_unload and stepper.phase=='transfer':
      terminal_ready=bool(shift>=.95 and strong and right_distance<=1e-6 and right_com_distance is not None and right_com_distance<=1e-6)
      if terminal_ready and feasible_refs is not None and terminal_unload.anchor is None:
       actual_capacity=stance_planner.static_capacity(controller,model_state['q'],model_state['root'],[{**c,'point':c['forcePoint']} for c in model_state['contacts']],left_share=0,generalized_load=model_state['bias'],velocity=model_state['v'],allow_limit_reactions=capacity_allow_stops)
       terminal_ready=bool(actual_capacity['feasible'] and actual_capacity['minimumRequiredMotorFraction']<=.9)
      terminal_support_run=terminal_support_run+.025 if terminal_ready else 0.
      left_force_ceiling=terminal_unload.update(terminal_support_run>=.1,state['metrics']['foot_loads_n'][0])
      if terminal_unload.failed:
       failure='terminal-unload-support-eligibility-lost';events.append(dict(event='stop-no-retry-of-lost-unload-support',timeS=elapsed));break
     if a.stage_unload_after_readiness and transfer_before:unload_weight=terminal_unload.soft_weight
     swing=None
     if stepper is not None:
      wanted_com,swing,allow_unload=stepper.update(elapsed,state,model_state,current_diag,[] if tick==0 else last,right_distance,lift_ready=(not a.terminal_unload or terminal_support_run>=.1))
      if stepper.phase=='stop':
       failure='no-qualified-unload-step-trial-stopped';events.append(dict(event='stop-no-unloaded-COM-return',timeS=elapsed));break
      if not allow_unload:unload_weight=0.
      if a.terminal_unload and transfer_before and stepper.phase=='lift':
       if terminal_support_run<.1:
        failure='terminal-unload-support-not-qualified-at-lift';events.append(dict(event='stop-no-unqualified-lift',timeS=elapsed));break
       left_force_ceiling=None
     if feasible_refs is not None:
      progress=shift if stepper.phase=='transfer' else 1.;posture_goal,rotation_goal,planned_com=feasible_refs.reference(progress)
      if stepper.phase=='transfer':wanted_com=planned_com
      elif stepper.phase in ['return','stand']:
       r=1. if stepper.phase=='stand' else probe.smooth((elapsed-stepper.phase_start)/3.);posture_goal=posture_goal+r*(initial_q-posture_goal);rotation_goal=rotation_goal@dynamics.Rotation.from_rotvec(r*dynamics.Rotation.from_matrix(rotation_goal.T@initial_R).as_rotvec()).as_matrix()
     try:action,info=controller.solve(model_state,wanted_com,rotation_goal,posture_goal,unload_left=unload_weight,swing=swing,left_force_ceiling_n=left_force_ceiling)
     except (AssertionError,ValueError) as error:action=source.copy();info=dict(success=False,error=str(error))
     solve_wall+=time.monotonic()-t
     if not info['success']:
      if stepper is not None:
       failure='inverse-dynamics-step-trial-stopped';events.append(dict(event='stop-no-stale-control-or-source-switch',reason=info,timeS=elapsed));break
      failure='inverse-dynamics-infeasible';release_start=elapsed;last_action=source if last_action is None else last_action;last_source=source if last_source is None else last_source;events.append(dict(event='safe-source-release',reason=failure,timeS=elapsed));action=source+(last_action-last_source)
     else:action=source+blend*(action-source)
    else:action=source.copy();info={'success':True,'sourceOnly':True}
    action=np.clip(action,-1.,1.);decision=np.array(state['observation']);desired_torque=controller.gears*np.clip(controller.kin.nominal+action-decision[:21]-.2*decision[46:67],-1.,1.);before_velocity=probe.reference.xyz((initial_diag if tick==0 else diagnostic)['comVelocity'])
    command={'op':'step','action':action.tolist()}
    if a.motion_profile:
     qdd=blend*np.array(info.get('qdd',[0.]*27)[6:]);command=dict(op='stepMotionProfile',action=action.tolist(),torques_nm=desired_torque.tolist(),qdd=qdd.tolist())
    tr=native.command(command);state=tr['state'];last=tr['native_foot_samples'];assert last,'No native substeps admitted; no acceleration/torque claim';samples.extend(last);diagnostic=native.command({'op':'diagnostic'});snapshot=native.command({'op':'snapshot'})
    actual_native_torques=np.array([row['motor_torques_nm'] for row in last])[:,actuator_of_qpos]
    actual_acceleration=(probe.reference.xyz(diagnostic['comVelocity'])-before_velocity)/(.00125*len(last))
    if tick%2==0:frames.append(snapshot)
    if 1<=elapsed<6:model_samples.extend(last)
    if elapsed>=return_score_start:source_samples.extend(last)
    if stepper is not None and stepper.phase=='stand':step_standing_samples.extend(last)
    com=probe.reference.xyz(diagnostic['com']);cap=probe.reference.xyz(diagnostic['capture']);com_history.append(com)
    history.append(dict(tick=tick+1,elapsedS=state['elapsed_s'],com=diagnostic['com'],capture=diagnostic['capture'],comInSupport=diagnostic['predicates']['comInSupport'],captureInSupport=diagnostic['predicates']['captureInSupport'],rightLoadFraction=state['metrics']['foot_loads_n'][1]/max(1e-9,sum(state['metrics']['foot_loads_n'])),supportPoints=diagnostic['solverSupportPoints'],requestedCOM=wanted_com.tolist()))
    notes.append(dict(phase=stepper.phase if stepper is not None else ('model' if blend>0 else 'done'),modelBlend=blend,overrideMaxRad=float(max(abs(action-source))),solve=info,desiredTorquesAtDecisionQposOrderNm=desired_torque.tolist(),nativeMaximumDesiredTorqueErrorNm=np.max(abs(actual_native_torques-desired_torque),axis=1).tolist(),nativeMaximumActualMotorFraction=np.max(abs(actual_native_torques/controller.gears),axis=1).tolist(),actualComAccelerationWorldMps2=actual_acceleration.tolist(),desiredJointPostureRad=np.asarray(posture_goal).tolist(),desiredRootRotationWorld=np.asarray(rotation_goal).tolist(),softUnloadLeftWeight=unload_weight,leftForceCeilingN=left_force_ceiling,terminalUnloadRampElapsedS=terminal_unload.elapsed,actualPoseZeroAccelerationCapacity=actual_capacity,rightCaptureToSupportSegmentDistanceM=right_distance,rightCOMToSupportHullDistanceM=right_com_distance,terminalQualifiedSupportRunS=terminal_support_run,qualifiedRightSupportRunS=support_run))
    if hold_start<=elapsed<hold_end:shift_history.append(dict(displacementM=float((com-initial_com)@direction),captureDisplacementM=float((cap-initial_com)@direction),rightLoadFraction=history[-1]['rightLoadFraction']))
    unsafe=any(all(v<3 for v in row['loads_n']) or row['nonfoot_support'] for row in last) or np.linalg.norm((com-wanted_com)[[0,2]])>.08
    if stepper is not None and unsafe:
     failure='step-support-or-tracking-guard';events.append(dict(event='stop-rejected-step',timeS=elapsed+.025));break
    if release_start is None and unsafe and elapsed<return_score_start:
     failure='measured-support-or-tracking-guard';release_start=elapsed+.025;events.append(dict(event='safe-source-release',reason=failure,timeS=release_start))
    last_action=action.copy();last_source=source.copy()
    if tr['terminated'] or tr['truncated']:failure=tr['failure'] or 'episode-ended';break
   metrics=probe.analyse(samples);metrics.update(stopped=failure,final=state,initialDiagnostic=initial_diag,finalDiagnostic=diagnostic,controllerEvents=events,fullModelWindow=probe.analyse(model_samples),maximumCOMDisplacementAlongTargetM=float(max((c-initial_com)@direction for c in com_history)),minimumCOMDisplacementAlongTargetM=float(min((c-initial_com)@direction for c in com_history)),maximumHorizontalCOMDriftM=float(max(np.linalg.norm((c-initial_com)[[0,2]]) for c in com_history)),maximumActualNativeMotorTorqueNm=max(max(abs(v) for v in row['motor_torques_nm']) for row in samples))
   metrics['balanceReturnWindow' if a.retain_balance else 'sourceReturnWindow']=probe.analyse(source_samples)
   if a.flat_soles and a.shift_m==0:
    end=round(a.seconds*800);metrics['flatFinalFiveSeconds']=probe.analyse(samples[max(0,end-4000):end]);metrics['flatStandingGatePassed']=bool(failure is None and len(samples)==end and metrics['flatFinalFiveSeconds']['maximumContinuousNativeStandingWithFootAnchorS']>=5. and not metrics['flightNativeSamples'] and not metrics['nonfootSupportObserved'] and max(metrics['loadedContactSlipIntegralM'])<=.01)
   if stepper is not None:
    metrics['stepControllerEvents']=stepper.log;metrics['stepSchedulerFailure']=stepper.failed;metrics['postStepStanding']=probe.analyse(step_standing_samples);real=[e for e in metrics['unloadedRepositionEvents'] if e['side']==0 and e['horizontalDisplacementM']>=.05 and e['maximumClearanceM']>=.015 and e['minimumOtherLoadN']>=3.];metrics['stepMechanismPassed']=bool(failure is None and stepper.failed is None and real and metrics['flightNativeSamples']==0 and not metrics['nonfootSupportObserved'] and max(metrics['loadedContactSlipIntegralM'])<=.01 and metrics['postStepStanding']['maximumContinuousNativeStandingWithFootAnchorS']>=5.)
   metrics['fullModelHoldPassed']=bool(a.shift_m==0 and failure is None and len(model_samples)==4000 and metrics['fullModelWindow']['maximumContinuousNativeStandingWithFootAnchorS']>=5 and not metrics['fullModelWindow']['flightNativeSamples'] and not metrics['fullModelWindow']['nonfootSupportObserved'] and max(metrics['fullModelWindow']['loadedContactSlipIntegralM'])<=.01 and metrics['maximumHorizontalCOMDriftM']<=.02)
   if shift_history:
    metrics['fullShiftWindow']=dict(windowS=[hold_start,hold_end],medianCOMDisplacementAlongTargetM=float(np.median([v['displacementM'] for v in shift_history])),medianCaptureDisplacementAlongTargetM=float(np.median([v['captureDisplacementM'] for v in shift_history])),meanRightLoadFraction=float(np.mean([v['rightLoadFraction'] for v in shift_history])))
    if a.ramp_s==1.5:metrics['fullShiftWindow2_5To4_5S']={k:v for k,v in metrics['fullShiftWindow'].items() if k!='windowS'}
   (a.output/'hybrid.json').write_text(json.dumps(dict(metrics=metrics,frames=frames,controlNotes=notes,supportHistory=history),indent=2)+'\n');native.command({'op':'checkpoint','path':str((a.output/'hybrid-final.bin').resolve())});report.update(status='completed-untrained-contact-feedback-not-step-qualification',metrics=metrics)
   print(json.dumps({k:metrics[k] for k in ['stopped','fullModelHoldPassed','maximumCOMDisplacementAlongTargetM','flightNativeSamples','maximumContinuousNativeStandingWithFootAnchorS']}),flush=True)
 except BaseException as error:report.update(status='failed-or-incomplete',error=repr(error));raise
 finally:
  native.close();report.update(physicsWallSeconds=native.wall,nativeSubsteps=native.native,uncertainOutstandingRequests=native.uncertain,totalProcessWallSeconds=time.monotonic()-started,solverWallSeconds=solve_wall,stateComputationWallSeconds=state_wall);(a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
