import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { webcrypto } from 'node:crypto';
import test from 'node:test';
import vm from 'node:vm';

// Actual production worker transport, scripted Runtime. No native integration.
const profile=readFileSync('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json','utf8');
const features=JSON.parse(readFileSync('rust/browser/minimal/capabilities.json')).effective_sim_features;
async function harness({badProfile=false,badFeatures=false}={}) {
  const messages=[],calls=[];let instance,request=0;
  class Runtime {
    constructor(...args){calls.push(['constructor',...args]);instance=this;this.tick=0;this.recoveries=0;this.motion='Fallen';this.stable=0;this.passed=false;this.failure=null;}
    static getup_fixture_with_profile(kind,p){const r=new Runtime();calls.push(['fixture',kind,p]);return r;}
    set_root_translation_compensation_enabled(v){calls.push(['carry',v]);}
    getup_fixture_report(){return JSON.stringify({state:{ready:this.tick>=20}});}
    trigger_getup_fixture(){calls.push(['trigger',this.tick]);}
    enqueue_batch(x){calls.push(['commands',JSON.parse(x)]);}
    set_visible(v){calls.push(['visible',v]);}
    native_output_ms(){return 0;}
    advance(ticks){calls.push(['advance',ticks]);this.tick+=ticks;return JSON.stringify({completed_ticks:ticks,acknowledgements:[],failure:this.failure,snapshot:{stamp:{generation:1,tick:this.tick},motion:this.motion,counters:{recoveries:this.recoveries},diagnostics:{getup:{policy:{prototype_standing_passed:this.passed,stable_standing_s:this.stable}}},striker:{available:true},segments:Array.from({length:25},()=>({position:{x:0,y:0,z:0},rotation:{x:0,y:0,z:0,w:1},linearVelocity:{x:0,y:0,z:0},angularVelocity:{x:0,y:0,z:0}}))}});}
    advance_profiled(ticks){return JSON.stringify({reply:JSON.parse(this.advance(ticks)),timings:[],clockValid:true,runtimeMs:0,runtimeNonTickMs:0});}
  }
  const self={postMessage:m=>messages.push(m)};
  const context=vm.createContext({self,performance,crypto:webcrypto,TextEncoder,Uint8Array,Float32Array,ArrayBuffer,WorkerRuntime:Runtime,wire_schema:()=>4,getup_trial_profile:()=>profile,getup_build_capabilities:()=>JSON.stringify({effectiveSimFeatures:badFeatures?[]:features}),init:async()=>({}),fetch:async path=>({ok:true,text:async()=>badProfile?'{}':profile,json:async()=>({effective_sim_features:features})})});
  const source=readFileSync('rust/browser/sim-worker.js','utf8').replace(/^import .*;$/m,'');
  await new vm.Script(`(async()=>{${source}\n})()`).runInContext(context);
  async function send(value){await self.onmessage({data:{requestId:++request,...value}});const last=messages.at(-1);if(last.type==='snapshot')await self.onmessage({data:{type:'recycle',bufferId:last.bufferId,poseBuffer:last.poseBuffer}});return last;}
  return {send,calls,messages,get runtime(){return instance;}};
}
const initialize={type:'initialize',mode:'protocol',heading:0,generation:1,floorEnabled:true,recoveryDemo:true};
test('default transport keeps ordinary constructor, no recovery asset or lifecycle',async()=>{const h=await harness({badProfile:true,badFeatures:true});const r=await h.send({...initialize,recoveryDemo:undefined});assert.equal(r.type,'snapshot');assert(!h.calls.some(c=>['fixture','carry','trigger'].includes(c[0])));});
test('recovery settles exactly400 then triggers once without losing tick receipt',async()=>{const h=await harness();await h.send(initialize);assert.equal(h.calls.filter(c=>c[0]==='fixture').length,1);assert.deepEqual(h.calls.find(c=>c[0]==='carry'),['carry',true]);let r;for(let i=0;i<5;i++)r=await h.send({type:'advance',ticks:4});assert.equal(r.reply.snapshot.stamp.tick,20);assert.equal(r.reply.completed_ticks,4);assert.deepEqual(h.calls.filter(c=>c[0]==='trigger'),[['trigger',20]]);await h.send({type:'advance',ticks:1});assert.equal(h.calls.filter(c=>c[0]==='trigger').length,1);});
test('recovery rejects incompatible profile or feature closure before Runtime construction',async()=>{for(const flags of [{badProfile:true},{badFeatures:true}]){const h=await harness(flags);assert.equal((await h.send(initialize)).type,'error');assert.equal(h.calls.length,0);}});
test('recovery requires current stability and three-second post-recovery wait',async()=>{const h=await harness();await h.send(initialize);for(let i=0;i<5;i++)await h.send({type:'advance',ticks:4});Object.assign(h.runtime,{recoveries:1,motion:'Upright',passed:true,stable:1});await h.send({type:'advance',ticks:1});const command={type:'commands',commands:[{action:'StrikeFromBehind'}]};assert.equal((await h.send(command)).type,'error');h.runtime.tick+=120;await h.send({type:'advance',ticks:0});assert.equal((await h.send(command)).type,'snapshot');h.runtime.stable=0;await h.send({type:'advance',ticks:0});assert.equal((await h.send(command)).type,'error');});
test('recovery forbids stamped reset or variant-changing initialization',async()=>{const h=await harness();await h.send(initialize);for(const action of ['Reset',{Initialize:{mode:'protocol',heading:0}},{SelectStation:{station:'flat'}}])assert.equal((await h.send({type:'commands',commands:[{action}]})).type,'error');assert.equal(h.calls.filter(c=>c[0]==='constructor').length,1);});
test('failed preparation never synthesizes a trigger or refunds progress',async()=>{const h=await harness();await h.send(initialize);h.runtime.failure={check:'scripted physical stop'};const r=await h.send({type:'advance',ticks:4});assert.equal(r.reply.completed_ticks,4);assert(r.reply.failure);assert(!h.calls.some(c=>c[0]==='trigger'));});
