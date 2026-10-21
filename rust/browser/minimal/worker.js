// One real WASM world. No fixture loading, reset, pose edits or external support.
import init,{WorkerRuntime,getup_build_capabilities,wire_schema} from './lh_worker.js';
await init();
const profile=await (await fetch('./profile.json')).json();
const contract=await (await fetch('./capabilities.json')).json();
const capabilities=JSON.parse(getup_build_capabilities());
if(JSON.stringify(capabilities.effectiveSimFeatures)!==JSON.stringify(contract.effective_sim_features))throw Error('Wrong controller build');
if(!profile.getup_policy_enabled||profile.getup_repeated_recovery!=='completed-cycle-confirmed-fall-v1')throw Error('Wrong recovery profile');
const runtime=new WorkerRuntime('protocol',0,'1',true,JSON.stringify(profile));
const identity=crypto.randomUUID();let sequence=0,reply=JSON.parse(runtime.advance(0)),halted=false;
self.onmessage=({data:q})=>{
 try{
  if(halted)throw Error('World stopped; no automatic retry');
  if(q.op==='strike'){
   if(!reply.snapshot.striker?.available)throw Error('Striker unavailable');
   runtime.enqueue(JSON.stringify({stamp:{schema:wire_schema(),generation:1,sequence:++sequence,tick:reply.snapshot.stamp.tick},action:'Strike'}));
   reply=JSON.parse(runtime.advance(0));
  }else if(q.op==='advance'){
   if(!Number.isInteger(q.ticks)||q.ticks<1||q.ticks>4)throw Error('Invalid advance');
   reply=JSON.parse(runtime.advance(q.ticks));
  }else throw Error('Unsupported operation');
  halted=Boolean(reply.failure);
  self.postMessage({id:q.id,identity,reply});
 }catch(error){halted=true;self.postMessage({id:q.id,error:String(error),identity});}
};
self.postMessage({ready:true,identity,reply,profile:profile.id,capabilities});
