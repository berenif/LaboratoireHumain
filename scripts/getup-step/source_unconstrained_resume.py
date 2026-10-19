#!/usr/bin/env python3
"""Continue a terminal wall-capped diagnostic from retained native/controller evidence."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/getup-step'))
import argparse,hashlib,json,time,shutil
from collections import deque
import numpy as np
from source_step_probe import World,sha
from source_arm_knee_mobility import native_bytes
from source_upright_large_steps import ExtendedArmMap
from source_robust_support_steps import RobustSupportController
from source_unconstrained_steps import UnconstrainedSteps

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
base=args.output.resolve()/'model-session'
folder=base.parent/'run/generation-001'
summary=json.loads((folder/'summary.json').read_text())
multi=json.loads((folder/'multistep-summary.json').read_text())
assert any(t in summary['stopped']for t in ('process cap','wall process cap')),summary['stopped']
assert summary['final']['stepIndex']==10 and summary['final']['phase']in('lift','translate','land','settle','stand')
previous=folder/'before-wall-cap-continuation';previous.mkdir(exist_ok=True)
for name in ('summary.json','multistep-summary.json','unconstrained-diagnostic.json','opposed-arm-run.json','final.bin','control.json'):
 if (previous/name).exists():assert sha(folder/name)==sha(previous/name),'continuation evidence differs: '+name
 else:shutil.copyfile(folder/name,previous/name)
notes=json.loads((folder/'control.json').read_text());last=notes[-1]
row=summary['final'];phase=row['phase'];index=row['stepIndex']
record=json.loads((folder/f'controller-step-{index:02d}.json').read_text())
arm=json.loads((folder/'arm-mapping.json').read_text())
diag=json.loads((folder/'unconstrained-diagnostic.json').read_text())
helper=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
w=World(base,helper);w.restore(folder/'final.bin')
payload=native_bytes(w)
assert w.l.lh_dim(w.h,6)==summary['actualIntegrations']
np.testing.assert_array_equal(w.arr(0,(28,)),row['qpos'])
np.testing.assert_array_equal(w.arr(1,(27,)),row['qvel'])
# Last measured native source time is used, not the rounded nominal clock.
p=folder/'frames.ndjson';tail=[]
with p.open('rb')as f:
 f.seek(max(0,p.stat().st_size-90_000_000))
 for line in f:
  try:r=json.loads(line)
  except:continue
  tail.append(r)
lastsolve=round(last['t']*800)
prior=next(r for r in tail if r['sourceStep']==lastsolve)
initialq=np.array(summary['entry']['qpos'][7:])
armmap=ExtendedArmMap(initialq,w.limits,np.array(arm['goalRad']))
assert armmap.receipt()==arm,'reconstructed arm-map receipt differs'
c=RobustSupportController.__new__(RobustSupportController)
for key,value in record.items():
 if isinstance(value,list):
  value=[tuple(np.array(x)for x in pair)for pair in value]if key=='targets'else np.array(value)
 setattr(c,key,value)
c.w=w;c._mapping=armmap
if phase in('settle','stand'):
 scratch=World(base,helper)
 try:
  scratch.restore(folder/f'step-{index:02d}-{phase}.bin')
  c.q0=np.clip(scratch.arr(0,(28,))[7:].copy(),w.limits[:,0]+.002,w.limits[:,1]-.002)
 finally:scratch.close()
c.previous_time=prior['sourceTimeS']
c.previous_leg_rate=np.array(last['opposedArmSwing']['measuredLegPitchRateRadps'])
seek=last['contactSeek'];c._seek_time=prior['sourceTimeS'];c._seek_depth=seek['depthM'];c._seek_rate=seek['rateMps']
c._seek_jet=(seek['depthM'],seek['rateMps'],seek['accelerationMps2']);c.contact_seek=seek
support=last['robustSupportAdmission'];c.robust_support=support.copy();c._robust_history=deque()
c._robust_delta=np.array(support['comCorrectionM']);c._robust_duration=support['correctionDurationS']
c._robust_start=support['selectedAtTransferS'];c._robust_weak=support['weakCorner']
c._robust_survivors=support['survivingCorners'];c._robust_clock=tuple(support['correctionClock'])
c._robust_initial_reserve=.003;c.robust_support_barrier=[]
assert native_bytes(w)==payload,'controller reconstruction changed actual native state'
live=UnconstrainedSteps.__new__(UnconstrainedSteps)
live.base=base;live.helper=helper;live.output=folder.parent;live.folder=folder
live.maximum=160000;live.sequence_integration_budget=160000
live.recorded_integrations=summary['actualIntegrations']
live.steps=summary['actualIntegrations'];live.total=summary['liveIntegrationsAcrossResets']
live.world=w;live.controller=c;live.journal=p.open('a');live.entry=summary['entry'];live.row=row
live.notes=notes;live.events=summary['events'];live.slip=np.array(summary['loadedSlipDistanceM'])
live.max_clear=summary['maximumWholeFootClearanceM'];live.max_motor=summary['maximumMotorFraction']
live.phase=phase;live.phase_start=row['elapsedS']-row['phaseElapsedS']
live.quiet_run=round(row.get('quietConsecutiveS',0)*800);live.anchors=None
if phase=='stand':
 start=next(r for r in tail if r['phase']=='stand')
 live.anchors=np.array([f['position']for f in start['feet']])
live.clear_run=live.land_run=live.unload_run=live.settle_ready_run=live.ready_run=0
live.loaded=np.zeros(2,dtype=int)
for r in tail:
 live.loaded=np.where(np.array([f['loadN']for f in r['feet']])>=3,live.loaded+1,0)
live.step_records=multi['steps'];live.step_index=index-1;live.distances=(.18,)*10
live.step_start=live.step_records[-1]['startNativeStep']
live.step_slip=np.sum([s['loadedSlipThisStepM']for s in live.step_records[:-1]],axis=0)
live.arm_map=armmap;live.leg_origin=np.array(record['leg_origin'])
live.arm_mapping_type=ExtendedArmMap;live.u=np.zeros(21)
live.paused=False;live.finished=False;live.failure=None;live.visible=True
live.experimental_distance_limit=.302;live.support_relative_reach=.30;live.upright_enabled=False
live.loaded_settle_handoff=True;live.continuous_loaded_handoff=False;live.implicit_arms=False
live.contact_impratio=8.;live.planned_support=True;live.time_scale=2.;live.scheduled=False
live.code_at_start=json.loads((folder/'opposed-arm-run.json').read_text())['codeSHA256AtStart']
live.quality_events=diag['firstQualityViolations'];live.quality_counts=diag['qualityViolationFrameCounts']
live.minimum_capture=diag['minimumCaptureMarginM'];live.minimum_joint=diag['minimumJointLimitMarginRad']
live.minimum_com_height=diag['minimumCOMHeightM'];live.maximum_penetration=diag['maximumSelfPenetrationM']
live.maximum_tilt=diag['maximumTorsoTiltDeg'];live.maximum_nonfoot_load=diag['maximumNonfootLoadN']
receipt=dict(schema=1,reason='outer wall cap; continuation from full native checkpoint',
 nativeCounterBefore=live.steps,nativeTimeBefore=w.l.lh_time(w.h),nativePayloadSHA256=hashlib.sha256(payload).hexdigest(),
 nativeQposQvelExact=True,nativeBytesUnchangedDuringControllerReconstruction=True,
 physicsStateReset=False,rootAssistance=False,controllerStateReconstructedFromRecordedEvidence=True,
 controllerObjectExactRoundtripProven=False,armMapReceiptExact=True,phaseBefore=phase,
 derivativeHistorySourceNativeStep=lastsolve,contactSeekJetRestored=True,robustCorrectionRestored=True,
 q0TargetRestoredFromPhaseCheckpoint=phase if phase in('settle','stand')else'initial step target',
 visualLoadedDurationCountersReconstructedFromTailOnly=True,runnerSHA256=sha(__file__))
(folder/'wall-cap-continuation.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('CONTINUITY',json.dumps(receipt),flush=True)
started=time.monotonic()
try:
 while not live.finished and not live.failure:
  assert time.monotonic()-started<600.,'600s continuation cap'
  live.advance(4)
  if live.steps%800==0:print('progress',live.steps/800,live.phase,'quiet',live.quiet_run/800,flush=True)
except Exception as e:
 live.failure=repr(e);print('CONTINUATION ERROR',repr(e),flush=True)
finally:live.close()
print('finished',live.finished,'failure',live.failure,flush=True)
raise SystemExit(0 if live.finished else 1)
