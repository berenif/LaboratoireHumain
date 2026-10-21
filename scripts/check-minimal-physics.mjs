// Narrow behavioral smoke, NOT browser validation or broad fixture acceptance.
// One constructor (optional declared initial prone fixture), at most two real
// strikes, no mid-run reset or pose manipulation.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const assets=resolve(process.argv[2]),out=resolve(process.argv[3]??'outputs/minimal-demo/physics-smoke.json');
const initialHeadingDegrees=Number(process.argv[4]??0);assert(Number.isFinite(initialHeadingDegrees));
const entryOnly=process.argv[5]==='entry-only';
const initialGrounded=['belly','belly-entry','belly-standing'].includes(process.argv[5]);
const diagnoseStanding=process.argv[5]==='belly-standing';
const compareEntries=process.argv[5]==='belly-entry';
const strikeAction=process.argv[6]==='behind'?'StrikeFromBehind':'Strike';
const initialHeadingRadians=initialHeadingDegrees*Math.PI/180;
const sha=x=>createHash('sha256').update(x).digest('hex');
const wasm=readFileSync(assets+'/lh_worker_bg.wasm');assert.equal(sha(wasm),'f9b06192c1d9b265dd179e80d8aa6c4ce72ad5cc0fa2dd2bddb9ee8fdb87fa3a');
const profile=readFileSync(assets+'/getup-800hz-3plus1-repeat-cycle-v1.json','utf8');assert.equal(sha(profile),'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6');
const mod=await import(pathToFileURL(assets+'/lh_worker.js'));mod.initSync({module:wasm});
const w=initialGrounded?mod.WorkerRuntime.getup_fixture_with_profile('belly',profile):new mod.WorkerRuntime('protocol',initialHeadingRadians,'1',true,profile);w.set_root_translation_compensation_enabled(true);
const start=performance.now(),trace=[],commands=[];let reply=JSON.parse(w.advance(0)),sequence=0,issued=0,native=0,forecasts=0,transfers=[],previousForecast=0,previousKey='',reason='bounded-stop',entry=null,entries=[],fixtureTriggered=false,recoveredAt=null,forecastSource=null;
function observe(){const s=reply.snapshot,p=s.diagnostics.getup?.policy,f=JSON.parse(w.getup_fixture_report());assert.equal(s.stamp.generation,1);assert(f.nativeSubsteps>=native,'Native clock rewound');native=f.nativeSubsteps;
 const fc=p?.forecast_substeps??0;forecasts+=p?.started_at_s!==forecastSource?fc:(fc>=previousForecast?fc-previousForecast:fc);previousForecast=fc;forecastSource=p?.started_at_s;
 assert.equal(s.diagnostics.direct_pelvis_force_n,0);assert.equal(s.diagnostics.direct_pelvis_torque_nm,0);
 const row={native,time:s.simulation_time_s,tick:s.stamp.tick,generation:s.stamp.generation,motion:s.motion,counters:s.counters,policy:p,striker:s.striker?{phase:s.striker.phase,impactId:s.striker.impactId,impulse:s.striker.lastImpactImpulseNs}:null,head:s.segments.find(x=>x.id==='head')?.position,discontinuity:s.discontinuity,failure:reply.failure};
 if(diagnoseStanding&&native>=9500){const d=JSON.parse(w.getup_standing_diagnostic('null'));row.standing={predicates:d.predicates,failedMotion:d.motion.filter(x=>!x.linearPassed||!x.angularPassed),headM:d.headM,torsoUp:d.torsoUp,thresholds:d.thresholds,loadsN:d.loadsN,supportPersistence:d.supportPersistence,comVelocity:d.comVelocity};}
 if(p?.phase?.startsWith('experimental-joint-balance')){const c=JSON.parse(w.getup_report());if(c.terminal_balance&&!transfers.some(t=>t.balance.activated_native_substep===c.terminal_balance.activated_native_substep))transfers.push({native,ready:c.terminal_base_ready,balance:c.terminal_balance,standing:JSON.parse(w.getup_standing_diagnostic('null')).predicates});}
 const key=JSON.stringify([row.motion,row.counters,row.policy?.phase,row.policy?.prototype_standing_passed]);if(row.standing||key!==previousKey||s.stamp.tick%20===0){trace.push(row);previousKey=key;}return s;}
try{
 while(native<96000&&performance.now()-start<300000){
  const s=observe(),p=s.diagnostics.getup?.policy;
  if(p?.started_at_s!=null&&!entries.some(e=>e.status.started_at_s===p.started_at_s)){const c=JSON.parse(w.getup_report());const e={native,fixture:JSON.parse(w.getup_fixture_report()),teacherInitial:c.teacher_initial,sourceObservation:c.teacher_frames[0]?.observation,status:c.status};entries.push(e);entry??=e;if(entryOnly||(compareEntries&&entries.length===3)){reason='source-entry-diagnostic';break;}}
  if(diagnoseStanding&&native>=11040){reason='standing-gate-diagnostic';break;}
  if(reply.failure){reason='physical-failure';break;}
  if(s.counters.recoveries>=(initialGrounded?3:2)){reason='two-cycles-complete';break;}
  if(p?.since_impact_s>25&&!p.prototype_standing_passed){reason='25-second-recovery-deadline';break;}
  if(initialGrounded&&!fixtureTriggered&&native>=400){assert.equal(native,400);assert(JSON.parse(w.getup_fixture_report()).state.ready);w.trigger_getup_fixture();reply=JSON.parse(w.advance(0));fixtureTriggered=true;console.log('INITIAL GROUNDED TRIGGER',native);continue;}
  if(initialGrounded&&s.counters.recoveries>=issued+1&&recoveredAt===null)recoveredAt=native;
  if(s.striker?.available&&issued<2&&(!initialGrounded||(s.motion==='Upright'&&p?.prototype_standing_passed&&p.stable_standing_s>=1))&&(initialGrounded?recoveredAt!==null&&native-recoveredAt>=2400:(issued===0?s.stamp.tick>=40:s.counters.recoveries>=issued))){
   const command={stamp:{schema:mod.wire_schema(),generation:1,sequence:++sequence,tick:s.stamp.tick},action:strikeAction};
   commands.push({native,command});w.enqueue(JSON.stringify(command));reply=JSON.parse(w.advance(0));if(reply.acknowledgements.some(a=>a.rejected)){reason='strike-path-rejected';break;}issued++;recoveredAt=null;console.log('STRIKE',issued,native);continue;
  }
  reply=JSON.parse(w.advance(4));
 }
 const s=observe();const result={schema:'lh-minimal-two-cycle-smoke/v1',initialHeadingDegrees,initialHeadingRadians,initialGrounded,fixtureTriggered,strikeAction,entryOnly,entry,entries,transfers,terminalController:(()=>{const c=JSON.parse(w.getup_report());return{status:c.status,stable_substeps:c.stable_substeps,terminal_base_ready:c.terminal_base_ready,terminal_balance:c.terminal_balance};})(),reason,browserValidated:false,physicsRuntime:'actual Rust/WASM; exact sourceWorkerSha256 below',sourceWorkerSha256:sha(wasm),sourceGlueSha256:sha(readFileSync(assets+'/lh_worker.js')),profileSha256:sha(profile),constructors:1,resets:0,strikesRequested:issued,commands,nativeIntegrations:native,forecastIntegrations:forecasts,totalIntegrations:native+forecasts,elapsedWallSeconds:(performance.now()-start)/1000,passed:reason==='two-cycles-complete'&&s.counters.strikes===2&&s.counters.falls>=(initialGrounded?3:2)&&s.counters.recoveries>=(initialGrounded?3:2),final:s,trace};
 const data=JSON.stringify(result);assert(Buffer.byteLength(data)<=1024**2,'Compact evidence limit');mkdirSync(resolve(out,'..'),{recursive:true});writeFileSync(out,data+'\n');console.log(JSON.stringify({passed:result.passed,reason,native,forecasts,wall:result.elapsedWallSeconds,counters:s.counters,policy:s.diagnostics.getup?.policy,failure:reply.failure,bytes:Buffer.byteLength(data)}));
}finally{w.free();}
