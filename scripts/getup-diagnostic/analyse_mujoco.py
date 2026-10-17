#!/usr/bin/env python3
"""Compare already archived trajectories only. Never imports a simulator."""
import json,pathlib,numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'outputs/getup-mujoco-comparison-2026-10-16'
C=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
vec=lambda x:np.array([x[k] for k in 'xyz'])
ref=json.loads((ROOT/'outputs/getup-policy-native-clock-2026-10-16/reference-report.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());cases=[]
for label,dt in [('matched800-closed',.00125),('source200-closed',.005),('matched800-replay',.00125)]:
 rows=[json.loads(x) for x in (OUT/label/'states.ndjson').read_text().splitlines()];commands=json.loads((OUT/label/'commands.json').read_text());comparison=[]
 for f in ref['teacher_frames']:
  t=round(f['time_s'],6);r=rows[round(t/dt)];native={p['id']:p for p in f['snapshot']['segments']}
  gap=max(float(np.linalg.norm(C@np.array(r['bodyPosesSource'][name]['position'])-vec(native[name]['position']))) for name in model['referenceMetadata']['sourceBodyOrder'])
  comparison.append({'timeS':t,'maximumJointAngleDifferenceRad':float(np.max(np.abs(np.array(r['qpos'][7:])-np.array(f['target'][:21])))),'maximumBodyPositionDifferenceM':gap,'headMujocoM':r['headHeightM'],'headRapierM':native['head']['position']['y'],'leftThighFootDepthMujocoM':r['leftThighFootPenetrationM']})
 first=next((r for r in rows if r['selfContactQualificationGateExceeded']),None)
 peak=max((c for r in rows for c in r['contacts'] if 'world' not in [c['body1'],c['body2']]),key=lambda c:-c['distanceM'])
 torque=0.
 for r in rows[1:]:
  expected=np.zeros(27)
  for p in model['segments']:
   h=p['reference']['hinge']
   if h:expected[6+h['qposIndex']]=h['gear']*r['control'][h['actuatorIndex']]
  torque=max(torque,float(np.max(np.abs(expected-np.array(r['qfrcActuator'])))))
 cases.append({'id':label,'first5mmExceedance':None if first is None else {'timeS':first['timeS'],'contacts':[c for c in first['contacts'] if 'world' not in [c['body1'],c['body2']] and c['distanceM']<-.005]},'peakSelfContact':peak,'maximumGeneralizedActuatorTorqueMappingErrorNm':torque,'initialCommandDifferenceFromRapier':commands[0]['maximumActionDifferenceFromRapier'],'commandDifferenceAt25msFromRapier':commands[1]['maximumActionDifferenceFromRapier'],'comparison':comparison})
r={'additionalPhysicsIntegrations':0,'cases':cases,'interpretation':'Original-engine closed-loop WEAK and exact Rapier command replay both violate the unchanged5mm qualification gate from this seed. Identical control replay diverges mechanically, establishing a dynamics/state-solver transfer gap. No evidence justifies a new actuator/observation wiring correction. Warm-start states are engine-specific and normalized native quaternion geometry differs by <=70micrometres; this is matched generalized state, not identical solver state.','nextStep':'Do not relax or exclude the pair. A candidate reference generator must enforce the existing collision/limit contract before SLOW admission; zero-shot WEAK does not satisfy it even in the original engine for this archived seed. No further physical experiment admitted.'}
(OUT/'comparison.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({'additionalPhysicsIntegrations':0,'cases':[{k:v for k,v in c.items() if k!='comparison'}|{'first25msComparison':c['comparison'][0],'last290msComparison':c['comparison'][-1]} for c in cases]},indent=2))
