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
const identity=crypto.randomUUID();let sequence=0,reply=JSON.parse(runtime.advance(0)),halted=false,initialTriggered=false;
self.onmessage=({data:q})=>{
 try{
  if(halted)throw Error('World stopped; no automatic retry');
  if(q.op==='strike'){
   if(!initialTriggered||reply.snapshot.counters.recoveries<1||!reply.snapshot.striker?.available)throw Error('Striker unavailable');
   runtime.enqueue(JSON.stringify({stamp:{schema:wire_schema(),generation:1,sequence:++sequence,tick:reply.snapshot.stamp.tick},action:'Strike'}));
   reply=JSON.parse(runtime.advance(0));
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
  halted=Boolean(reply.failure);
  self.postMessage({id:q.id,identity,reply});
 }catch(error){halted=true;self.postMessage({id:q.id,error:String(error),identity});}
};
self.postMessage({ready:true,identity,reply,profile:profile.id,capabilities});
