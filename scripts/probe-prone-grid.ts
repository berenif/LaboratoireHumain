import { createEmbodiedCharacter } from '../src/character/index';
import { DynamicRecovery, type ProneExperimentParameters } from './experimental-recovery';
import { RECOVERY_POSE_FIXTURES, seedRecoveryFixture } from './recovery-fixtures';
import { rotate, length } from '../src/character/math';
import { writeFileSync } from 'node:fs';
const fixture=RECOVERY_POSE_FIXTURES.find(f=>f.id==='landed-prone-left')!;
const results=[];
let bestScore=-Infinity;
for(const kneeHip of [-.45,-.75,-1.05]) for(const rootTilt of [.8,1.2]) for(const armLoad of [.12,.24]) {
 const params:ProneExperimentParameters={rootTilt,spineTilt:-.3,rootHeight:.45,armLoad,kneeBend:2.1,kneeHip,backShift:.12};
 const c=await createEmbodiedCharacter('canvas2d');
 (c as unknown as {recovery:DynamicRecovery}).recovery=new DynamicRecovery(params);seedRecoveryFixture(c,fixture);
 let braceTime:number|null=null,maxY=0,maxUp=0,kneeTime=0,maxDrift=0,localBest=-Infinity,leadReleased=false;
 let bestPhysical:unknown=null;const rows=[];
 for(let tick=0;tick<480;tick++) {
  c.fixedUpdate(1/60,null);const s=c.getSnapshot('canvas2d'),r=s.diagnostics.recovery;rows.push(s);
  if(r.phase==='brace'&&braceTime===null)braceTime=tick/60;
  if(braceTime!==null) {
   const torso=s.segments.find(p=>p.id==='torso')!,pelvis=s.segments.find(p=>p.id==='pelvis')!;
   const up=rotate(torso.rotation,{x:0,y:1,z:0}).y;
   const loaded=r.contacts.filter(c=>c.loadBearing),arms=loaded.filter(c=>/Hand|Forearm/.test(c.segment));
   const knee=loaded.some(c=>c.segment==='leftShin');
   const drift=Math.max(0,...r.plantedTargets.filter(p=>p.segment.endsWith('Hand')).map(p=>p.driftM));
   maxY=Math.max(maxY,s.rootPosition.y);maxUp=Math.max(maxUp,up);maxDrift=Math.max(maxDrift,drift);if(knee)kneeTime+=1/60;
   if(r.releasedSupports.includes('rightFoot'))leadReleased=true;
   const score=s.rootPosition.y+up*.4+Number(knee)*.5+Number(arms.length>0)*.2-Math.max(0,drift-.03)*3-length(pelvis.linearVelocity)*.2;
   if(score>localBest){localBest=score;bestPhysical={time:tick/60,y:s.rootPosition.y,up,drift,knee,arms:arms.map(c=>c.segment),loads:loaded.map(c=>[c.segment,Math.round(c.forceN)]),phase:r.phase,stage:r.transferStage};}
  }
  if(s.state==='upright'||r.retries>0)break;
 }
 const result={params,braceTime,maxY,maxUp,kneeTime,maxDrift,leadReleased,score:localBest,bestPhysical};results.push(result);console.log(JSON.stringify(result));
 if(localBest>bestScore){bestScore=localBest;writeFileSync('evidence/prone-grid-best.json',JSON.stringify(rows));}
 c.dispose();
}
writeFileSync('evidence/prone-grid.json',JSON.stringify(results,null,2));
