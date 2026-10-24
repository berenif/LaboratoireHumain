// Local-only extraction of run-getup-continuous-slow.mjs. Same commands/cadence;
// the observer below never writes controls or changes physical state.
import {DenseActor, physicalObservation, controls, observation, target, ORDER} from './slow.js';

export const PROTOCOL = Object.freeze({passiveDecisions:80, movingDecisions:340,
  holdingDecisions:150, passiveDt:.005, slowDt:.00125, maximumTicks:10200, sampleDt:.025});
export const check=(ok,message)=>{if(!ok)throw Error(message);};
const same=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
const hash=async b=>Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',b)),v=>v.toString(16).padStart(2,'0')).join('');
const files={
  entry:['/web/proofs/getup-weak-seed10/weak-final.json','8b9ef9f76bd26ec5047c5b1d13ecbf4715820ef3c9a8324568dd7cdbc9473281'],
  reference:['/web/proofs/getup-continuous-slow/reference.json','ef1aaae307153c55afd9f8da37d63999e65ee06f51e1e694e12e7badbc436dee'],
  actor:['/rust/crates/sim/data/getup-policy/student.bin','c489faed55bdab1669c51dab71a102db91ff2a55143789f682e45cd8169d6dbb'],
  model:['/web/dist/humanoid.xml','7a6e3dfc83a9e30bebe81e3e8c6e6b146f8447d1dbfaacd83de90f4937ea5765'],
};
export async function loadCycleAssets({assetBase=null}={}){
  // Hosted builds provide an explicit same-origin asset base. The legacy
  // repository-root loader remains local-only for retained diagnostic scripts.
  if(!assetBase)check(['127.0.0.1','localhost','[::1]'].includes(location.hostname),'Essai réservé au serveur local');
  const packaged={entry:'weak-final.json',reference:'reference.json',actor:'student.bin',model:'../humanoid.xml'};
  const bytes={};
  await Promise.all(Object.entries(files).map(async([key,[url,sha]])=>{
    const r=await fetch(assetBase?new URL(packaged[key],assetBase):url);check(r.ok,`Ressource GetUp indisponible : ${key}`);
    bytes[key]=await r.arrayBuffer();check(await hash(bytes[key])===sha,`Identité incorrecte : ${key}`);
  }));
  const json=b=>JSON.parse(new TextDecoder().decode(b));
  const metadataResponse=await fetch(assetBase?new URL('getup-v1.json',assetBase):'/rust/crates/model/data/getup-v1.json');check(metadataResponse.ok,'Métadonnées absentes');
  const metadata=await metadataResponse.json(),terminal=target(metadata.referenceMetadata.standingPose);
  const xml=new TextDecoder().decode(bytes.model).replace('<option timestep=".00125"/>','<option timestep=".005" integrator="Euler"/>');
  check(await hash(new TextEncoder().encode(xml))==='edf53e8509ea56be6f00d873e7a03d32d1e2f69b0c6a13ac95887ec8e8308655','Modèle WEAK différent');
  const reference=json(bytes.reference);check(reference.data.qpos.length===340,'Durée de référence différente');
  return {entry:json(bytes.entry),reference,actor:new DenseActor(bytes.actor),terminal,xml,
    identities:Object.fromEntries(Object.entries(files).map(([k,[,sha]])=>[k,sha]))};
}

export class CycleObserver {
  constructor(weight){this.weight=weight;this.events=[];this.holdingSamples=0;this.calmSamples=0;this.maxCalmSamples=0;this.bilateralSamples=0;this.maxBilateralSamples=0;this.flightSamples=0;this.newFlightSamples=0;this.nonfootSamples=0;this.fallSamples=0;this.fallenAtPassiveEnd=false;this.receptionTime=null;this.previousAir=false;}
  observe(r){
    const o=r.physical,feet=[7,10].map(b=>r.loads[b]||0);
    const clear=[7,10].map(b=>Math.min(...r.geoms.filter(g=>g.body===b).map(g=>Math.min(g.a[2],g.b[2])-g.radius)));
    const nonfoot=Object.entries(r.loads).some(([b,f])=>![7,10].includes(+b)&&f>=3);
    const air=Object.values(r.loads).reduce((s,f)=>s+f,0)<.1&&clear.every(h=>h>.001);
    const bilateral=feet.every(f=>f>=3),reception=bilateral&&feet[0]+feet[1]>=.8*this.weight&&!nonfoot;
    const calm=reception&&o[21]>1.4&&o[36]>Math.cos(Math.PI/9)&&Math.hypot(...o.slice(37,40))<.05&&Math.hypot(...r.qvel.slice(3,6))<.5;
    const fall=o[21]<.9&&(o[36]<.5||nonfoot);
    if(r.phase==='passive-fall'){
      this.fallSamples=fall?this.fallSamples+1:0;this.fallenAtPassiveEnd=this.fallSamples>=4;
      if(this.fallSamples===4)this.events.push({type:'fall-observed',t:r.t});
    }else{
      if(air&&!this.previousAir)this.events.push({type:this.receptionTime===null?'flight':'new-flight-after-reception',t:r.t});
      if(this.previousAir&&!air)this.events.push({type:Object.values(r.loads).some(f=>f>=.1)?'loaded-contact-after-flight':'flight-clearance-threshold-ended',t:r.t,feet});
      // Final observation starts when the reference ends. Foot loading during
      // the get-up itself (e.g. head .757 m at 18.350) is propulsion/support,
      // not the terminal reception. Keep all prior flights in the event log.
      if(r.phase==='SLOW-holding'&&reception&&this.receptionTime===null){this.receptionTime=r.t;this.events.push({type:'loaded-reception',t:r.t,feet});}
      if(air){this.flightSamples++;if(this.receptionTime!==null)this.newFlightSamples++;}
    }
    if(r.phase==='SLOW-holding'){
      this.holdingSamples++;this.calmSamples=calm?this.calmSamples+1:0;this.maxCalmSamples=Math.max(this.maxCalmSamples,this.calmSamples);
      this.bilateralSamples=bilateral?this.bilateralSamples+1:0;this.maxBilateralSamples=Math.max(this.maxBilateralSamples,this.bilateralSamples);
      if(nonfoot)this.nonfootSamples++;this.holdFallSamples=fall?(this.holdFallSamples||0)+1:0;
    }
    this.previousAir=air;this.last={t:r.t,feet,clear,air,bilateral,reception,calm,fall,head:o[21],tilt:Math.acos(Math.max(-1,Math.min(1,o[36]))),comSpeed:Math.hypot(...o.slice(37,40)),com:r.com};
    return this.last;
  }
  summary(){return {events:structuredClone(this.events),receptionTime:this.receptionTime,holdingSamples:this.holdingSamples,
    bilateralSamples:this.bilateralSamples,calmSamples:this.calmSamples,calmSeconds:this.calmSamples*.025,
    maxCalmSamples:this.maxCalmSamples,newFlightSamples:this.newFlightSamples,nonfootSamples:this.nonfootSamples,last:this.last,
    observedHold:this.holdingSamples===150&&this.receptionTime!==null&&this.calmSamples>=40&&this.newFlightSamples===0&&this.nonfootSamples===0,
    historicalVerdict:'af04325 FAIL unchanged; initial airborne holding samples were forbidden',
    stages:'Reactions: last solved substep. Geometry: refreshed state (1.25 ms stage offset during SLOW). Contacts sampled at 40 Hz.'};}
}

export class GetUpCycle {
  constructor(p,assets){
    this.p=p;this.assets=assets;this.ticks=0;this.passiveTicks=0;this.slowTicks=0;this.decisions=0;this.done=false;this.phase='passive-fall';this.maxTorqueRatio=0;this.maxTorqueErrorNm=0;this.transitions=[];
    const {mj,model:m,data:d}=p;check(mj.mj_versionString()==='3.17.0'&&m.nq===28&&m.nv===27&&m.nu===21&&p.dt===.005,'Contrat moteur');
    p.joints.forEach((j,a)=>check(j.q-7===ORDER[a],'Mapping actionneurs'));
    this.observer=new CycleObserver(Array.from(m.body_mass).reduce((s,v)=>s+v,0)*9.81);
    this.buffer=new mj.DoubleBuffer(assets.entry.state.length);
    // The sole restoration, before this cycle. No writes of q/qvel afterwards.
    mj.mj_setState(m,d,assets.entry.state,assets.entry.spec);
    check(same(this.stateVector(),assets.entry.state),'Restauration non exacte');
    p.forward();p.readReactions();physicalObservation(p);mj.mj_step1(m,d);
    this.last=this.snapshot();this.observer.observe(this.last);
  }
  stateVector(){this.p.mj.mj_getState(this.p.model,this.p.data,this.buffer,this.assets.entry.spec);return Array.from(this.buffer.GetView());}
  snapshot(){const p=this.p,d=p.data,physical=physicalObservation(p);return {phase:this.phase,t:d.time,cycleTime:d.time-this.assets.entry.time,physical,qpos:Array.from(d.qpos),qvel:Array.from(d.qvel),com:Array.from(d.subtree_com.slice(3,6)),loads:{...p.reactions.normalLoads},ctrl:Array.from(d.ctrl),bodypos:Array.from(d.xpos),bodyquat:Array.from(d.xquat),geoms:p.geometries()};}
  at(k){const ref=this.assets.reference.data;return k>=340?this.assets.terminal:[...ref.qpos[k].slice(7),ref.com[k][2],...ref.com_ori[k]];}
  get nextTicks(){return this.done?0:this.decisions<80?5:20;}
  enter(phase){this.phase=phase;this.transitions.push({phase,time:this.p.data.time,qpos:Array.from(this.p.data.qpos),qvel:Array.from(this.p.data.qvel)});}
  advance(){
    check(!this.done&&this.ticks+this.nextTicks<=10200,'Budget du cycle terminé');
    const p=this.p,{mj,model:m,data:d}=p,k=this.decisions-80;
    if(k===0){
      check(this.observer.fallenAtPassiveEnd,'Chute passive non établie');
      const before=this.stateVector();m.opt.timestep=.00125;check(same(before,this.stateVector()),'État modifié au changement de cadence');
      this.rateChangePreservedFullState=true;mj.mj_step1(m,d);this.enter('SLOW-moving');
    }
    if(k===340)this.enter('SLOW-holding');
    let now,residual;const holding=k>=340;
    if(k<0)d.ctrl.fill(0);
    else {now=this.at(k);residual=this.assets.actor.infer(observation(physicalObservation(p),now,this.at(k+5))).action;}
    for(let j=0;j<(k<0?5:20);j++){
      if(k>=0)d.ctrl.set(controls(now.slice(0,21),residual,Array.from(d.qpos.slice(7)),Array.from(d.qvel.slice(6)),holding));
      mj.mj_step2(m,d);this.ticks++;p.steps++;if(k<0)this.passiveTicks++;else this.slowTicks++;
      p.readReactions();
      for(const joint of p.joints){const actual=d.qfrc_actuator[joint.v],wanted=d.ctrl[joint.a]*joint.gear;
        this.maxTorqueRatio=Math.max(this.maxTorqueRatio,Math.abs(actual/joint.gear));this.maxTorqueErrorNm=Math.max(this.maxTorqueErrorNm,Math.abs(actual-wanted));
        check(Number.isFinite(actual)&&Math.abs(actual)<=Math.abs(joint.gear)+1e-8&&Math.abs(actual-wanted)<=2e-4,'Couple appliqué invalide');}
      check(d.qfrc_actuator.slice(0,6).every(x=>x===0)&&d.qfrc_applied.every(x=>x===0)&&d.xfrc_applied.every(x=>x===0),'Force externe inattendue');
      mj.mj_step1(m,d);check([...d.qpos,...d.qvel].every(Number.isFinite)&&Math.max(...d.qvel.map(Math.abs))<1e4,'État non fini/divergent');
      check(Math.abs(d.time-this.assets.entry.time-this.passiveTicks*.005-this.slowTicks*.00125)<1e-8,'Discontinuité temporelle');
    }
    this.decisions++;this.last=this.snapshot();this.observer.observe(this.last);
    if(this.decisions===80&&!this.observer.fallenAtPassiveEnd)this.finish('Chute passive non établie');
    else if((this.observer.holdFallSamples||0)>=4)this.finish('Rechute mesurée pendant observation');
    else if(this.ticks===10200)this.finish('Fenêtre terminée — simulation en pause');
    return this.last;
  }
  finish(reason){this.done=true;this.reason=reason;}
  summary(){return {ticks:this.ticks,passiveTicks:this.passiveTicks,slowTicks:this.slowTicks,phase:this.phase,done:this.done,reason:this.reason,transitions:this.transitions,rateChangePreservedFullState:!!this.rateChangePreservedFullState,maxTorqueRatio:this.maxTorqueRatio,maxTorqueErrorNm:this.maxTorqueErrorNm,...this.observer.summary()};}
  dispose(){this.buffer.delete();}
}
