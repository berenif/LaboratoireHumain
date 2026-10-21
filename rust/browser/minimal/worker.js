// One real WASM world, visibly prone at construction. No mid-run reset,
// pose edits, snapshot substitution or external support.
import init,{WorkerRuntime,getup_build_capabilities,wire_schema} from './lh_worker.js';
await init();
const profile=await (await fetch('./profile.json')).json();
const contract=await (await fetch('./capabilities.json')).json();
const capabilities=JSON.parse(getup_build_capabilities());
if(JSON.stringify(capabilities.effectiveSimFeatures)!==JSON.stringify(contract.effective_sim_features))throw Error('Wrong controller build');
if(!profile.getup_policy_enabled||profile.getup_repeated_recovery!=='completed-cycle-confirmed-fall-v1')throw Error('Wrong recovery profile');
const runtime=WorkerRuntime.getup_fixture_with_profile('belly',JSON.stringify(profile));
// Same numerical integrator carry as the verified worker fixtures; no force,
// pose, velocity or clock change (runtime.rs set_root_translation_compensation_enabled).
runtime.set_root_translation_compensation_enabled(true);
const identity=crypto.randomUUID();let sequence=0,reply=JSON.parse(runtime.advance(0)),halted=false,initialTriggered=false,readyAfterTick=null,lastRecoveries=0;
self.onmessage=({data:q})=>{
 try{
  if(halted)throw Error('World stopped; no automatic retry');
  let notice=null;
  if(q.op==='strike'||q.op==='strike-standard'){
   const p=reply.snapshot.diagnostics.getup?.policy;
   if(!initialTriggered||reply.snapshot.motion!=='Upright'||!p?.prototype_standing_passed||p.stable_standing_s<1||readyAfterTick===null||reply.snapshot.stamp.tick<readyAfterTick||!reply.snapshot.striker?.available)throw Error('Striker unavailable: wait for stable recovery');
   runtime.enqueue(JSON.stringify({stamp:{schema:wire_schema(),generation:1,sequence:++sequence,tick:reply.snapshot.stamp.tick},action:q.op==='strike'?'StrikeFromBehind':'Strike'}));
   reply=JSON.parse(runtime.advance(0));
   if(reply.acknowledgements.some(a=>a.rejected))notice='Trajet du percuteur bloqué : aucune frappe, aucun autre angle essayé.';
  }else if(q.op==='advance'){
   if(!Number.isInteger(q.ticks)||q.ticks<1||q.ticks>4)throw Error('Invalid advance');
   // The first 400 native samples settle the declared initial pose physically.
   const ticks=initialTriggered?q.ticks:Math.min(q.ticks,20-reply.snapshot.stamp.tick);
   reply=JSON.parse(runtime.advance(ticks));
   if(!initialTriggered&&!reply.failure&&reply.snapshot.stamp.tick===20){
    if(!JSON.parse(runtime.getup_fixture_report()).state.ready)throw Error('Initial prone body did not settle');
    runtime.trigger_getup_fixture(); // Lifecycle only; no pose/velocity/clock change.
    initialTriggered=true;reply=JSON.parse(runtime.advance(0));
   }
  }else throw Error('Unsupported operation');
  if(reply.snapshot.counters.recoveries>lastRecoveries){lastRecoveries=reply.snapshot.counters.recoveries;readyAfterTick=reply.snapshot.stamp.tick+120;}
  halted=Boolean(reply.failure);
  self.postMessage({id:q.id,identity,reply,notice,readyAfterTick});
 }catch(error){halted=true;self.postMessage({id:q.id,error:String(error),identity});}
};
self.postMessage({ready:true,identity,reply,profile:profile.id,capabilities});
