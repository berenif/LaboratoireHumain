// Local, bounded continuation. Reuses SLOW inference/PD and the existing user
// push path. No Balance/Physics.step, no reference or physical-state reset in advance.
import {check,GetUpCycle,CycleObserver} from './cycle.js';
import {physicalObservation,observation,controls,ORDER} from './slow.js';
export const CONTINUATION=Object.freeze({maximumTicks:4000,quietTicks:800,pushTicks:120,dt:.00125,decisionTicks:20});
export const FINAL_STATE_SHA='a8f29036d655785d8450ebe657b1b613dc929f5178ef0cb3a7a457fb43b95049';
const same=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
export async function loadFinalCheckpoint(){
 check(['localhost','127.0.0.1','[::1]'].includes(location.hostname),'Checkpoint réservé au serveur local');
 const r=await fetch('/web/proofs/getup-browser-cycle/final-state.json');check(r.ok,'Checkpoint final indisponible');
 const bytes=await r.arrayBuffer(),sha=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),v=>v.toString(16).padStart(2,'0')).join('');
 check(sha===FINAL_STATE_SHA,'Identité du checkpoint final différente');
 const checkpoint=JSON.parse(new TextDecoder().decode(bytes));
 check(checkpoint.summary.done&&checkpoint.summary.ticks===10200&&checkpoint.summary.observedHold,'Checkpoint final non qualifié');
 return checkpoint;
}
// Pure stop predicate, also used at each native step (not just video samples).
export function supportStop(loads,clear,head,vertical){
 const nonfoot=Object.entries(loads).some(([b,f])=>![7,10].includes(+b)&&f>=3);
 if(head<.9&&(vertical<.5||nonfoot))return 'Chute mesurée';
 if(nonfoot)return 'Appui non plantaire mesuré';
 if(Object.values(loads).reduce((s,f)=>s+f,0)<.1&&clear.every(h=>h>.001))return 'Nouvel envol mesuré';
 if([7,10].some(b=>(loads[b]||0)<3))return 'Perte d’un appui plantaire chargé';
 return null;
}
export class SlowContinuation {
 constructor(p,assets,{checkpoint=null,previousSummary=null}={}){
  this.p=p;this.ticks=0;this.done=false;this.phase='SLOW-continuation';this.continuation=true;this.decisions=0;
  this.pushAppliedTicks=0;this.pushStartedAt=null;this.pushEndedAt=null;this.maxTorqueRatio=0;this.maxTorqueErrorNm=0;
  this.calmTicks=0;this.maxCalmTicks=0;this.events=[];this.lastExternal=Array(p.model.nbody*6).fill(0);
  const {mj,model:m,data:d}=p;
  check(mj.mj_versionString()==='3.17.0'&&m.nq===28&&m.nv===27&&m.nu===21&&p.dt===.00125,'Contrat continuation SLOW');
  p.joints.forEach((j,a)=>check(j.q-7===ORDER[a],'Mapping actionneurs'));
  const prior=checkpoint?.summary||previousSummary;
  check(prior?.done&&prior.ticks===10200&&prior.observedHold,'Continuation réservée à la fin du cycle connu');
  const spec=checkpoint?.spec||16383;this.buffer=new mj.DoubleBuffer(mj.mj_stateSize(m,spec));
  this.assets={...assets,entry:{spec,time:checkpoint?.state[0]??d.time}};
  if(checkpoint){
   // One explicitly requested initial restoration. No state setter after this.
   mj.mj_setState(m,d,checkpoint.state,spec);check(same(this.stateVector(),checkpoint.state),'Restauration non exacte');
   this.restoredExact=true;
  }
  this.origin=checkpoint?'checkpoint final exact du cycle 9dd33c0':'état final vivant du cycle';
  this.previousSummary=structuredClone(prior);this.initialState=this.stateVector();
  check(d.qfrc_applied.every(v=>v===0)&&d.xfrc_applied.every(v=>v===0)&&!p.grab&&!p.push,'Force initiale inattendue');
  this.observer=new CycleObserver(Array.from(m.body_mass).reduce((s,v)=>s+v,0)*9.81);
  p.forward();p.readReactions();physicalObservation(p);mj.mj_step1(m,d);
  this.last=this.snapshot();this.observer.observe({...this.last,phase:'SLOW-holding'});
  if(!this.observer.last.calm)this.finish('Calme initial non établi — aucune poussée');
 }
 stateVector(){return GetUpCycle.prototype.stateVector.call(this);}
 snapshot(){return {...GetUpCycle.prototype.snapshot.call(this),continuationTicks:this.ticks,pushAppliedTicks:this.pushAppliedTicks,externalLastStep:this.lastExternal,commands:this.p.joints.map(j=>({name:j.name,ctrl:this.p.data.ctrl[j.a],applied:this.p.data.qfrc_actuator[j.v],cap:Math.abs(j.gear)}))};}
 get nextTicks(){return this.done?0:Math.min(20,4000-this.ticks);}
 command(){
  const p=this.p,d=p.data,ref=this.assets.terminal;
  if(this.ticks%20===0){this.residual=this.assets.actor.infer(observation(physicalObservation(p),ref,ref)).action;this.decisions++;}
  d.ctrl.set(controls(ref.slice(0,21),this.residual,Array.from(d.qpos.slice(7)),Array.from(d.qvel.slice(6)),true));
 }
 advance(){
  check(!this.done&&this.ticks+this.nextTicks<=4000,'Budget continuation terminé');
  const p=this.p,{mj,model:m,data:d}=p,start=this.ticks,end=start+this.nextTicks;
  if(this.ticks===800){check(this.calmTicks===800,'Poussée interdite sans seconde calme');p.smallPush('forward');this.pushStartedAt=d.time;this.events.push({type:'push-start',t:d.time,body:1,force:[15,0,0],duration:.15});}
  while(this.ticks<end&&!this.done){
   d.xfrc_applied.fill(0);this.command();
   const pushing=this.ticks>=800&&this.ticks<920;
   if(pushing){check(p.push?.body===1,'Poussée interrompue/inattendue');p.applyPush();p.lastForce=15;}else{check(!p.push,'Poussée hors fenêtre');p.lastForce=0;}
   check(d.qfrc_applied.every(v=>v===0),'Force généralisée externe inattendue');
   check(d.xfrc_applied.every((v,i)=>v===(pushing&&i===6?15:0)),'Force utilisateur hors contrat');
   this.lastExternal=Array.from(d.xfrc_applied);
   mj.mj_step2(m,d);this.ticks++;p.steps++;if(pushing)this.pushAppliedTicks++;
   p.readReactions();
   for(const j of p.joints){const actual=d.qfrc_actuator[j.v],wanted=d.ctrl[j.a]*j.gear;
    this.maxTorqueRatio=Math.max(this.maxTorqueRatio,Math.abs(actual/j.gear));this.maxTorqueErrorNm=Math.max(this.maxTorqueErrorNm,Math.abs(actual-wanted));
    check(Number.isFinite(actual)&&Math.abs(actual)<=Math.abs(j.gear)+1e-8&&Math.abs(actual-wanted)<=2e-4,'Couple appliqué invalide');}
   check(d.qfrc_actuator.slice(0,6).every(v=>v===0),'Racine actionnée');
   // User force is regenerated only for the scheduled native steps, including
   // after a UI pause; no residual force survives the pulse or terminal stop.
   d.xfrc_applied.fill(0);
   if(this.pushAppliedTicks===120&&this.pushEndedAt===null){p.push=null;this.pushEndedAt=d.time;this.events.push({type:'push-end',t:d.time,ticks:120});}
   mj.mj_step1(m,d);
   check([...d.qpos,...d.qvel].every(Number.isFinite)&&Math.max(...d.qvel.map(Math.abs))<1e4,'État non fini/divergent');
   check(Math.abs(d.time-this.assets.entry.time-this.ticks*.00125)<1e-8,'Discontinuité temporelle');
   const head=p.names.indexOf('head'),torso=p.names.indexOf('torso');
   const cause=supportStop(p.reactions.normalLoads,[p.footClearance(7),p.footClearance(10)],d.xpos[head*3+2],d.xmat[torso*9+8]);
   if(cause)this.finish(cause);
  }
  this.last=this.snapshot();const o=this.observer.observe({...this.last,phase:'SLOW-holding'});
  this.calmTicks=o.calm?this.calmTicks+this.ticks-start:0;this.maxCalmTicks=Math.max(this.maxCalmTicks,this.calmTicks);
  if(!this.done&&this.ticks<=800&&!o.calm)this.finish('Calme non conservé avant poussée — aucune poussée');
  if(!this.done&&this.ticks===4000){this.completedWindow=true;this.finish('Fenêtre de 5 s terminée — en pause');}
  return this.last;
 }
 finish(reason){this.done=true;this.reason=reason;this.p.push=null;this.p.lastForce=0;this.p.data.xfrc_applied.fill(0);}
 summary(){return {...this.observer.summary(),continuation:true,ticks:this.ticks,maximumTicks:4000,phase:this.phase,done:this.done,reason:this.reason,origin:this.origin,restoredExact:!!this.restoredExact,startTime:this.assets.entry.time,
  calmSeconds:this.calmTicks*.00125,maxCalmSeconds:this.maxCalmTicks*.00125,events:[...this.events,...this.observer.events],pushAppliedTicks:this.pushAppliedTicks,pushStartedAt:this.pushStartedAt,pushEndedAt:this.pushEndedAt,
  maxTorqueRatio:this.maxTorqueRatio,maxTorqueErrorNm:this.maxTorqueErrorNm,
  completedWindow:!!this.completedWindow,
  observedHold:!!this.completedWindow&&this.done&&this.ticks===4000&&this.pushAppliedTicks===120&&this.calmTicks>=800&&this.observer.newFlightSamples===0&&this.observer.nonfootSamples===0,
  physicalStops:'Each native step: nonfoot >=3N, either foot <3N, flight, fall, nonfinite/caps; calm sampled 40Hz with existing thresholds.'};}
 dispose(){this.buffer.delete();}
}
