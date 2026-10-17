#!/usr/bin/env python3
"""Measured-contact inverse-dynamics transfer probe, untrained hybrid."""
import argparse,datetime,hashlib,importlib.util,json,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
step_reference=module('contact_step',ROOT/'scripts/getup-contact-step.py');probe=module('probe',ROOT/'scripts/probe-getup-recovery-step.py');dynamics=module('contact_dynamics',ROOT/'scripts/getup-contact-dynamics.py')


def main():
 p=argparse.ArgumentParser(description=__doc__)
 for name in ['binary','checkpoint','output']:p.add_argument('--'+name,type=Path,required=True)
 p.add_argument('--recovery-step',action='store_true');p.add_argument('--target-calibration',type=Path);p.add_argument('--unload-left',action='store_true');p.add_argument('--hold-s',type=float,default=2.);p.add_argument('--return-ramp-s',type=float);p.add_argument('--shift-m',type=float,default=0.);p.add_argument('--ramp-s',type=float,default=1.5);p.add_argument('--seconds',type=float,default=12.);p.add_argument('--retain-balance',action='store_true');p.add_argument('--physics-budget',type=float,required=True);p.add_argument('--deadline',required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
 deadline=datetime.datetime.fromisoformat(a.deadline.replace('Z','+00:00')).timestamp();assert time.time()<deadline<=datetime.datetime(2026,10,17,14,25,tzinfo=datetime.timezone.utc).timestamp();assert 0<=a.physics_budget<=90 and 0<=a.shift_m<=.15;assert 12<=a.seconds<=19.75 and a.seconds*40==round(a.seconds*40)
 assert 1.5<=a.ramp_s<=7.5 and .5<=a.hold_s<=2.;return_ramp=a.ramp_s if a.return_ramp_s is None else a.return_ramp_s;assert 1.5<=return_ramp<=7.5;hold_start=1+a.ramp_s;hold_end=hold_start+a.hold_s;return_end=hold_end+return_ramp;return_score_start=return_end+1.;assert a.seconds>=return_score_start+5.
 a.output.mkdir(parents=True,exist_ok=True);(a.output/'started-once').open('x').close();started=time.monotonic();controller=dynamics.ContactDynamics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));native=probe.Native(a.binary,a.checkpoint,a.physics_budget,deadline,a.output)
 config=dict(controller='measured-contact-whole-body-inverse-dynamics-untrained-hybrid',shiftM=a.shift_m,simulatedSeconds=a.seconds,physicsCapSeconds=a.physics_budget,deadlineUTC=a.deadline,nearStopModeNoExplicitReleaseNoStep=True,externalForcesApplied=False,sourceWeightsChanged=False,model='continuous rigid-body mass+armature/Coriolis/gravity/passive springs+damping; original inner PD remains active800Hz; near-stop tolerance is predictive approximation, not strict instantaneous complementarity',contactConstraints='actual material-point normal acceleration hard, tangential slip stabilization soft, optimization only; never physical pins',acceptance='Zero: fullmodel1-6s>=5s anchoredstanding,noflight/nonfoot,<=1cmslip,<=2cmCOMdrift. Shift sign:actualCOM displacement alongrequested>=2.5mm for5mmtarget, no parasiticflight/slip, source-only after7s>=5sstanding. This is no stepping claim.',checkpointSha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),nativeBinarySha256=hashlib.sha256(a.binary.read_bytes()).hexdigest(),probeSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),dynamicsSha256=hashlib.sha256((ROOT/'scripts/getup-contact-dynamics.py').read_bytes()).hexdigest())
 for f in ['probe-getup-contact-feedback.py','getup-contact-dynamics.py','getup-contact-step.py']: (a.output/f).write_bytes((ROOT/'scripts'/f).read_bytes())
 config['standingControllerAfterTargetReturn']='measured-contact inverse dynamics retained' if a.retain_balance else 'frozen GetUp SLOW'
 config['timelineS']=dict(rampStart=1.,fullShiftStart=hold_start,fullShiftEnd=hold_end,targetReturned=return_end,postReturnScoringStart=return_score_start)
 if a.retain_balance:config['acceptance']='New retained-balancer gate, not retroactive source-handoff success: actual matched COM/capture/load shift; target COM returned; final>=5s native anchored standing under explicit measured-state balance, no flight/nonfoot and loaded slip<=1cm. No stepping claim.'
 report=dict(status='preflight-only');samples=[];model_samples=[];source_samples=[];frames=[];notes=[];history=[];events=[];com_history=[];shift_history=[];solve_wall=state_wall=0.;failure=None;release_start=None;last_action=None;last_source=None;support_run=0.;unload_weight=0.;stepper=None;step_standing_samples=[]
 actuator_of_qpos=np.zeros(21,dtype=int)
 for part in controller.parts:
  hinge=part['reference'].get('hinge')
  if hinge:actuator_of_qpos[hinge['qposIndex']]=hinge['actuatorIndex']
 try:
  state=native.command({'op':'observe'});initial_diag=native.command({'op':'diagnostic'});snapshot=native.command({'op':'snapshot'});assert state['metrics']['native_standing_without_foot_anchor'];model_state=controller.state(state['observation'],snapshot,initial_diag);initial_com=model_state['com'].copy();initial_q=model_state['q'].copy();initial_R=dynamics.normalized_rotation(model_state['root'][:3,:3]);right=initial_diag['contactMotion']['feet'][1];target=np.average([probe.reference.xyz(c['point']) for c in right['contacts']],axis=0,weights=[c['normalImpulseNs'] for c in right['contacts']]);direction=target-initial_com;direction[1]=0;direction/=np.linalg.norm(direction);
  if a.target_calibration:
   calibration=json.loads(a.target_calibration.read_text());assert calibration['offsetNormM']<=.01 and max(abs(initial_com-np.array(calibration['sourceInitialCOM'])))<1e-5;target[[0,2]]=calibration['lockedTargetXZ'];direction=target-initial_com;direction[1]=0.;a.shift_m=float(np.linalg.norm(direction));assert a.shift_m<=.15;direction/=a.shift_m;config['shiftM']=a.shift_m;config['targetCalibrationSHA256']=hashlib.sha256(a.target_calibration.read_bytes()).hexdigest()

  if a.recovery_step:
   assert a.retain_balance and a.unload_left;config['nearStopModeNoExplicitReleaseNoStep']=False;config['stepMode']='measured-unload-gated swing, measured landing, then retained balance';stepper=step_reference.RecoveryStep(initial_com,initial_com+direction*a.shift_m,a.ramp_s,controller.kin);config['stepAcceptance']='Actual>=5cm unload-land displacement/wholecapsuleclearance>=1.5cm, noflight/nonfoot, loadedslip<=1cm, actuallanding+>=5s retainedbalance anchoredstanding; not a learned step or original10sshuffling fix.'
  config['softUnloadLeft']=a.unload_left;config['initialCOM']=initial_com.tolist();config['shiftDirectionWorld']=direction.tolist()
  assert model_state['comParityErrorM']<1e-4 and model_state['comVelocityParityErrorMps']<2e-5 and model_state['contactVelocityParityErrorMps']<2e-5
  t=time.monotonic();action,info=controller.solve(model_state,initial_com,initial_R,initial_q);solve_wall+=time.monotonic()-t;assert info['success'],info;assert info['maximumTorqueRepresentationErrorNm']<1e-5
  report['preflight']=info;(a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
  if a.execute:
   report['status']='running-contact-feedback'
   for tick in range(round(a.seconds*40)):
    elapsed=tick*.025;right_distance=None
    source=np.array(native.command({'op':'initializedAction'})['action'])
    blend=probe.smooth(elapsed) if elapsed<1 else (1. if a.retain_balance else (1-probe.smooth(elapsed-return_end) if elapsed>=return_end else 1.))
    shift=probe.smooth((elapsed-1)/a.ramp_s) if elapsed<hold_start else (1-probe.smooth((elapsed-hold_end)/return_ramp) if elapsed>=hold_end else 1.)

    if stepper is not None:shift=probe.smooth((elapsed-1)/a.ramp_s)
    wanted_com=initial_com+direction*a.shift_m*shift
    if release_start is not None:
     blend=max(0.,1-probe.smooth((elapsed-release_start)/.05));action=source+blend*(last_action-last_source);info={'success':False,'fallback':True}
    elif blend>0:
     t=time.monotonic();model_state=controller.state(state['observation'],snapshot,diagnostic if tick else initial_diag);state_wall+=time.monotonic()-t
     t=time.monotonic()
     current_diag=initial_diag if tick==0 else diagnostic;right_points=[[c['point'][k] for k in ['x','z']] for c in current_diag['contactMotion']['feet'][1]['contacts']];right_distance=float('inf')
     if right_points:_,right_distance=dynamics.support_projection(right_points,probe.reference.xyz(current_diag['capture'])[[0,2]])
     strong=bool(tick>0 and all(row['loads_n'][1]>=.8*controller.mass*9.81 for row in last))
     eligible=bool(a.unload_left and shift>=.95 and right_distance<=.003 and strong)
     support_run=support_run+.025 if eligible else 0.;unload_weight=min(1.,unload_weight+.025/.2) if support_run>=.1 else 0.
     swing=None
     if stepper is not None:
      wanted_com,swing,allow_unload=stepper.update(elapsed,state,model_state,current_diag,[] if tick==0 else last,right_distance)
      if stepper.phase=='stop':
       failure='no-qualified-unload-step-trial-stopped';events.append(dict(event='stop-no-unloaded-COM-return',timeS=elapsed));break
      if not allow_unload:unload_weight=0.
     try:action,info=controller.solve(model_state,wanted_com,initial_R,initial_q,unload_left=unload_weight,swing=swing)
     except (AssertionError,ValueError) as error:action=source.copy();info=dict(success=False,error=str(error))
     solve_wall+=time.monotonic()-t
     if not info['success']:
      if stepper is not None:
       failure='inverse-dynamics-step-trial-stopped';events.append(dict(event='stop-no-stale-control-or-source-switch',reason=info,timeS=elapsed));break
      failure='inverse-dynamics-infeasible';release_start=elapsed;last_action=source if last_action is None else last_action;last_source=source if last_source is None else last_source;events.append(dict(event='safe-source-release',reason=failure,timeS=elapsed));action=source+(last_action-last_source)
     else:action=source+blend*(action-source)
    else:action=source.copy();info={'success':True,'sourceOnly':True}
    action=np.clip(action,-1.,1.);decision=np.array(state['observation']);desired_torque=controller.gears*np.clip(controller.kin.nominal+action-decision[:21]-.2*decision[46:67],-1.,1.);before_velocity=probe.reference.xyz((initial_diag if tick==0 else diagnostic)['comVelocity'])
    tr=native.command({'op':'step','action':action.tolist()});state=tr['state'];last=tr['native_foot_samples'];assert last,'No native substeps admitted; no acceleration/torque claim';samples.extend(last);diagnostic=native.command({'op':'diagnostic'});snapshot=native.command({'op':'snapshot'})
    actual_native_torques=np.array([row['motor_torques_nm'] for row in last])[:,actuator_of_qpos]
    actual_acceleration=(probe.reference.xyz(diagnostic['comVelocity'])-before_velocity)/(.00125*len(last))
    if tick%2==0:frames.append(snapshot)
    if 1<=elapsed<6:model_samples.extend(last)
    if elapsed>=return_score_start:source_samples.extend(last)
    if stepper is not None and stepper.phase=='stand':step_standing_samples.extend(last)
    com=probe.reference.xyz(diagnostic['com']);cap=probe.reference.xyz(diagnostic['capture']);com_history.append(com)
    history.append(dict(tick=tick+1,elapsedS=state['elapsed_s'],com=diagnostic['com'],capture=diagnostic['capture'],comInSupport=diagnostic['predicates']['comInSupport'],captureInSupport=diagnostic['predicates']['captureInSupport'],rightLoadFraction=state['metrics']['foot_loads_n'][1]/max(1e-9,sum(state['metrics']['foot_loads_n'])),supportPoints=diagnostic['solverSupportPoints'],requestedCOM=wanted_com.tolist()))
    notes.append(dict(phase=stepper.phase if stepper is not None else ('model' if blend>0 else 'done'),modelBlend=blend,overrideMaxRad=float(max(abs(action-source))),solve=info,desiredTorquesAtDecisionQposOrderNm=desired_torque.tolist(),nativeMaximumDesiredTorqueErrorNm=np.max(abs(actual_native_torques-desired_torque),axis=1).tolist(),nativeMaximumActualMotorFraction=np.max(abs(actual_native_torques/controller.gears),axis=1).tolist(),actualComAccelerationWorldMps2=actual_acceleration.tolist(),softUnloadLeftWeight=unload_weight,rightCaptureToSupportSegmentDistanceM=right_distance,qualifiedRightSupportRunS=support_run))
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
